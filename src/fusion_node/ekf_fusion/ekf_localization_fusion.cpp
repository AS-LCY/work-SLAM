#include "ekf_localization_fusion.h"
#include <ros/ros.h>

namespace localization_module{


EkfLocalizationFusion::EkfLocalizationFusion(){
    // ROS_WARN("localization load params");
    if (set_params() == false) {
        ROS_INFO("load params in localization fusion failed!");
        exit(1);
    }else{
        ROS_INFO_STREAM(GREEN<<"load params in localization fusion success!"<<RESET);
    }
    ekf_ptr_ = std::make_shared<PoseEKF>(status_num_,input_num_, measure_num_,
                                            status_covariance_, input_covariance_,measure_covariance_,
                                            dt_, gating_params_);
    ROS_INFO("gating params: %f", gating_params_.dis_drift_lateral_thresh);
    
}

EkfLocalizationFusion::~EkfLocalizationFusion(){
    is_init_ = false;
    
}

void EkfLocalizationFusion::localization_fusion_core(const fairland_msgs::LocalizationPoseData& pose_msg, fairland_msgs::LocalizationPoseData *pose_msg_out){
    pose_msg_ = pose_msg;
    *pose_msg_out = pose_msg;
    set_localizationfusion_input(); // set measure_ & input_ (from pose_msg_)
    double dt = pose_msg.header.stamp.toSec() - ts_;
    ROS_INFO("input: v: %.3f, w: %.5f; dt: %.3f", input_(0,0),input_(1,0),dt);
    ekf_ptr_->predict(input_,dt);
    ts_ = pose_msg.header.stamp.toSec();// update for next loop

    ROS_INFO("measure: %.3f, %.3f, %.4f", measure_(0,0),measure_(1,0),measure_(2,0));
    bool trust_measure = false;//////////////////////// TODO 

    ekf_ptr_->update(measure_, trust_measure);
    status_estimated_ = ekf_ptr_->get_status_estimated();
    
    pose_msg_out->fusion_pose = pose_msg_out->slam_pose;
    pose_msg_out->fusion_pose.position.x = status_estimated_(0,0)+offset_x_;
    pose_msg_out->fusion_pose.position.y = status_estimated_(1,0)+offset_y_;
    geometry_msgs::Vector3 euler = localization_module::common::Quaternion::get_euler_zyx(pose_msg_out->fusion_pose.orientation);

    ROS_INFO("ekf dx: %.3f, dy: %.3f, dyaw: %.4f", \
                status_estimated_(0,0)-measure_(0,0), status_estimated_(1,0)-measure_(1,0), \
                localization_module::common::NumericalProcess::unify_angle(status_estimated_(2,0) - euler.z));
    if(use_ekf_yaw_){
        euler.z = status_estimated_(2,0);
    }
    pose_msg_out->fusion_pose.orientation =  localization_module::common::Quaternion::get_quaternion(euler);
    ROS_INFO_STREAM("offset_x_: "<<offset_x_ << " --- offset_y_: "<<offset_y_ );
}


void EkfLocalizationFusion::set_localizationfusion_input() {
    // 观测量 measure
    measure_(0,0) = pose_msg_.slam_pose.position.x - offset_x_; // ekf 观测量: x
    measure_(1,0) = pose_msg_.slam_pose.position.y - offset_y_; // ekf 观测量: y
    measure_(2,0) = localization_module::common::Quaternion::get_euler_zyx(pose_msg_.slam_pose.orientation).z; // ekf 观测量: theta
    measure_(2,0) = localization_module::common::NumericalProcess::unify_angle(measure_(2,0)); // ekf 观测量: theta-unified

    // 控制量 input
    input_(0,0) = pose_msg_.chassis_status.ac_linear_velocity;
    input_(1,0) = pose_msg_.angular_velocity.z;

}

void EkfLocalizationFusion::init(const fairland_msgs::LocalizationPoseData& status) {
    pose_msg_ = status;
    offset_x_ = pose_msg_.slam_pose.position.x;
    offset_y_ = pose_msg_.slam_pose.position.y;
    measure_(0,0) = 0.0;
    measure_(1,0) = 0.0;
    measure_(2,0) = localization_module::common::Quaternion::get_euler_zyx(pose_msg_.slam_pose.orientation).z;
    measure_(2,0) = localization_module::common::NumericalProcess::unify_angle(measure_(2,0));
    ekf_ptr_->set_status(measure_);
    ts_=status.header.stamp.toSec();
    status_estimated_ = measure_*1.0;
    is_init_ = true;
}

bool EkfLocalizationFusion::is_init() {
    return is_init_;
}

void EkfLocalizationFusion::reset(){
    is_init_=false;
    offset_x_=0.0;
    offset_y_=0.0;
    
    ekf_ptr_ -> reset(status_covariance_,input_covariance_,measure_covariance_);
    // ekf_ptr_->params_initialize();
    // ekf_ptr_->set_covariance(status_covariance_,input_covariance_,measure_covariance_);
}


bool EkfLocalizationFusion::matrix_init(const LocalizationFusionParams* lf_params) {
    if (lf_params->status_cov_mat_diag.size() != status_num_ ||
        lf_params->input_cov_mat_diag.size()!= input_num_ ||
        lf_params->measure_cov_mat_diag.size() != measure_num_) {
        return false;
    }

    status_covariance_ = Matrix::Zero(status_num_, status_num_);
    input_covariance_ =Matrix::Zero(input_num_,input_num_);
    measure_covariance_ = Matrix::Zero(measure_num_, measure_num_);

	for (int i = 0; i < status_num_; i++) {
		status_covariance_(i,i) = lf_params->status_cov_mat_diag[i] * lf_params->status_cov_mat_diag[i];
	}

    for (int i = 0; i < input_num_; i++) {
        input_covariance_(i,i) = lf_params->input_cov_mat_diag[i] * lf_params->input_cov_mat_diag[i];
    }

    for (int i = 0; i < measure_num_; i++) {
        measure_covariance_(i,i) = lf_params->measure_cov_mat_diag[i] * lf_params->measure_cov_mat_diag[i];
    }

    measure_ = Matrix::Zero(measure_num_,1);
    input_=Matrix::Zero(input_num_,1);
    status_estimated_ = Matrix::Zero(status_num_,1);

    return true;
}

bool EkfLocalizationFusion::set_params(){
    LocalizationFusionParamsManager* params_manager = LocalizationFusionParamsManager::Instance();
    const LocalizationFusionParams* lf_params = params_manager->get_localization_fusion_params();
    dt_ = (lf_params->localization_fusion_dt)*0.001;
    status_num_ = lf_params->status_num;
    input_num_ = lf_params->input_num;
    measure_num_ = lf_params->measure_num;
    gating_params_ = lf_params->gating_params;
    use_ekf_yaw_ = lf_params->use_ekf_yaw;

    if (matrix_init(lf_params) == false) {
        ROS_INFO("matrix init in localization fusion failed!");
        return false;
    }
    return true;
}


} // namespace localization_module
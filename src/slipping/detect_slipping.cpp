#include "detect_slipping.h"

namespace localization_module{
    


DetectSlipping::DetectSlipping(/* args */){
    if (!load_params()){
        ROS_ERROR_STREAM(BOLDRED << "Load DetectSlipping param failed!" << RESET);
    }else {
        ROS_INFO("Load detect-slipping param successfully!");
    }

    reset();

    log_info_manager_ = LocalizationModuleLogInfoManager::getInstance();
}

DetectSlipping::~DetectSlipping(){
}

bool DetectSlipping::detect_by_chassis_and_lidar(int &slip_flag){
    log_info_manager_->log_info.slip_flag = 0;
    log_info_manager_->log_info.slip_count = 0;
    if (!lidar_queue_init_ || !chassis_queue_init_){
        slipping_count_ = 0;
        // ROS_INFO_STREAM("detect waiting! lidar init = " << lidar_queue_init_  << ", chassis init = " << chassis_queue_init_);
        return false;
    }


    ///////////////////////////////////////////////////////////////////////////////
    // pop chassic data before lidar
    double lidar_que_time_range = pose_que_.back().header.stamp.toSec() - pose_que_.front().header.stamp.toSec();

    std::lock_guard<std::mutex> lock(chassis_que_mtx_);

    double que_time_fst = chassis_que_.front().header.stamp.toSec();
    double que_time_snd = 0.0;
    double que_time_last = chassis_que_.back().header.stamp.toSec();
    while(que_time_last - que_time_fst > lidar_que_time_range && chassis_que_.size() >=2){
        chassis_que_.pop();
        que_time_snd = chassis_que_.front().header.stamp.toSec();

        double pop_dt = que_time_snd - que_time_fst;
        double pop_vel = std::abs(chassis_que_.front().ac_linear_velocity);
        double pop_dist = pop_vel * pop_dt;

        // update
        chassis_sum_dist_ = chassis_sum_dist_ - pop_dist;
        que_time_fst = que_time_snd;
    }
    double chassis_que_time_range = chassis_que_.back().header.stamp.toSec() - chassis_que_.front().header.stamp.toSec();

    ///////////////////////////////////////////////////////////////////////////////
    // detect 
    slip_flag = 0;
    double slipping_dist = chassis_sum_dist_ - lidar_sum_dist_;

    if(slipping_dist > param_slipping_dist_thr_){
        slipping_count_ ++;
    }else{
        slipping_count_ = 0;
    }

    slipping_count_ = slipping_count_>200 ? 200 : slipping_count_;

    int slipping_count_thr = param_slipping_count_thr_;
    if(log_info_manager_->module_status == localization_module::ModuleStatus::MODULE_LOCALIZATION){
        slipping_count_thr = param_slipping_count_thr_ * 5;
    }

    if(slipping_count_ >= slipping_count_thr){
        slip_flag = 1;
        ROS_WARN_STREAM(RED << " --- Slipping ---" << RESET);
    }

    ///////////////////////////////////////////////////////////////////////////////
    log_info_manager_->log_info.lidar_queue_time_range = lidar_que_time_range;
    log_info_manager_->log_info.lidar_queue_sum_dist = lidar_sum_dist_;
    log_info_manager_->log_info.chassis_queue_time_range = chassis_que_time_range;
    log_info_manager_->log_info.chassis_queue_sum_dist = chassis_sum_dist_;
    log_info_manager_->log_info.slipping_dist = slipping_dist;
    log_info_manager_->log_info.slip_count = slipping_count_;
    log_info_manager_->log_info.slip_flag = slip_flag;
    // ROS_INFO_STREAM("[detect-slip]: lidar_que_time_range: " << lidar_que_time_range);
    // ROS_INFO_STREAM("[detect-slip]: lidar_que_dist_range: " << lidar_sum_dist_);
    // ROS_INFO_STREAM("[detect-slip]: chassis_que_time_range: " << chassis_que_time_range);
    // ROS_INFO_STREAM("[detect-slip]: chassis_que_dist_range: " << chassis_sum_dist_);

    return true;
}

bool DetectSlipping::detect_by_chassis_and_imu(){

    return true;
    
}

void DetectSlipping::update_chassis(fairland_msgs::chassic_data cur_chassis_msg){
    // ///////////////////////////////////////////////////////////////////////////
    // for detect slip
    static double last_time = cur_chassis_msg.header.stamp.toSec();
    // static bool first_chassis = true;

    if(!chassis_queue_init_){
        init_chassis_queue();
        // first_chassis = true;
        last_time = cur_chassis_msg.header.stamp.toSec();
        chassis_queue_init_ = true;
    }


    double cur_time = cur_chassis_msg.header.stamp.toSec();
    double cur_dt = cur_time - last_time;
    double cur_vel = std::abs(cur_chassis_msg.ac_linear_velocity);
    double cur_dist = cur_vel * cur_dt;

    // update
    last_time = cur_time;
    chassis_sum_dist_ = chassis_sum_dist_ + cur_dist;

    /////////////////////////////////////////////////////////////////////////////////////////////
    std::lock_guard<std::mutex> lock(chassis_que_mtx_);
    // push
    chassis_que_.push(cur_chassis_msg);
    double que_time_fst = chassis_que_.front().header.stamp.toSec();
    double que_time_snd = 0.0;

    while(cur_time - que_time_fst > param_detect_window_time_range_ && chassis_que_.size() >=2){
        chassis_que_.pop();
        que_time_snd = chassis_que_.front().header.stamp.toSec();

        double pop_dt = que_time_snd - que_time_fst;
        double pop_vel = std::abs(chassis_que_.front().ac_linear_velocity);
        double pop_dist = pop_vel * pop_dt;

        // update
        chassis_sum_dist_ = chassis_sum_dist_ - pop_dist;
        que_time_fst = que_time_snd;
    }

}

void DetectSlipping::update_lidar_by_distance(geometry_msgs::PoseStamped pose){
    // init
    if(!lidar_queue_init_){
        init_lidar_queue();
        lidar_queue_init_ = true;
    }

    // get curr_pose
    geometry_msgs::PoseStamped curr_pose = pose;

    // push
    pose_que_.push(curr_pose);

    geometry_msgs::PoseStamped back_pose = pose_que_.back();
    lidar_sum_dist_ = cal_dist(pose_que_.front(), pose_que_.back());

    ///////////////////////////////////
    // update: pose_que_, lidar_sum_dist_
    double curr_time = back_pose.header.stamp.toSec();
    while(curr_time - pose_que_.front().header.stamp.toSec() > param_detect_window_time_range_ && pose_que_.size() >=2){
        lidar_sum_dist_ = cal_dist(pose_que_.front(), back_pose); // curr_dust > 0 恒成立， std::abs();
        pose_que_.pop();
    }
}


void DetectSlipping::update_lidar_by_segment(geometry_msgs::PoseStamped pose){
    static geometry_msgs::PoseStamped last_pose = pose;         // 初始化 仅执行一遍
    static bool first_lidar = true;

    // init
    if(!lidar_queue_init_){
        init_lidar_queue();
        last_pose = pose;
        first_lidar = true;
        lidar_queue_init_ = true;
    }

    // get curr_pose
    geometry_msgs::PoseStamped curr_pose = pose;
    double cur_dist = cal_dist_along_heading(last_pose, curr_pose); // curr_dist > 0 恒成立， std::abs();
    // double cur_dist = cal_dist(last_pose, curr_pose); // curr_dust > 0 恒成立， std::abs();
    lidar_sum_dist_ = lidar_sum_dist_ + cur_dist;

    // push
    pose_que_.push(curr_pose);
    dist_que_.push(cur_dist);
    if(first_lidar){
        dist_que_.pop();    // dist_que_.size() == pose_que_.size() -1, 恒成立
        first_lidar = false;
    } 

    ///////////////////////////////////////////////////////////////////////////////
    // update: last_pose, pose_que_, dist_que_, lidar_sum_dist_
    double curr_time = pose_que_.back().header.stamp.toSec();
    while(curr_time - pose_que_.front().header.stamp.toSec() > param_detect_window_time_range_ &&  pose_que_.size() >=2){
        double pop_dist = dist_que_.front();
        lidar_sum_dist_ = lidar_sum_dist_ - pop_dist;
        pose_que_.pop();
        dist_que_.pop();
    }
    last_pose = curr_pose;

}

double DetectSlipping::cal_dist_along_heading(geometry_msgs::PoseStamped last_pose, geometry_msgs::PoseStamped curr_pose){

    double dx = curr_pose.pose.position.x - last_pose.pose.position.x;
    double dy = curr_pose.pose.position.y - last_pose.pose.position.y;

    double last_yaw = get_yaw_from_orientation(last_pose.pose.orientation);
    double curr_yaw = get_yaw_from_orientation(curr_pose.pose.orientation);
    double pose_yaw = angle_norm(last_yaw + angle_norm(curr_yaw - last_yaw)/2.0);  // 车身，两帧的角度均值
    double line_yaw = std::atan2(dy, dx);// 速度方向的角度
    double dyaw = angle_norm(line_yaw - pose_yaw);// 速度方向与车身方向的夹角

    double line_dist = std::sqrt(dx*dx + dy*dy);
    double curr_dist = std::abs(line_dist * std::cos(dyaw));// 车身方向的位移

    return curr_dist;
}


double DetectSlipping::cal_dist(geometry_msgs::PoseStamped last_pose, geometry_msgs::PoseStamped curr_pose){

    double dx = curr_pose.pose.position.x - last_pose.pose.position.x;
    double dy = curr_pose.pose.position.y - last_pose.pose.position.y;

    double line_dist = std::sqrt(dx*dx + dy*dy);

    return line_dist;
}

void DetectSlipping::update_imu(sensor_msgs::Imu imu_msg){

}

bool DetectSlipping::load_params(){
    LocalizationFusionParamsManager *param_manager = LocalizationFusionParamsManager::Instance();
    const LocalizationFusionParams* loaded_param = param_manager->get_localization_fusion_params();

    if (loaded_param == NULL) {
        ROS_ERROR_STREAM(RED << "loaded_param is NULL" <<RESET);
        return false;
    }else{
        param_detect_window_time_range_ = loaded_param->slip_params.detect_window_time_range;
        param_slipping_dist_thr_ = loaded_param->slip_params.slipping_dist_thr;
        param_slipping_count_thr_ = loaded_param->slip_params.slipping_count_thr;
        return true;
    }
}

void DetectSlipping::init_lidar_queue(){
    // std::queue container does not have a clear member function
    // clear queue: Swap with an empty queue; 
    std::queue<double>().swap(dist_que_);
    std::queue<geometry_msgs::PoseStamped>().swap(pose_que_);
    lidar_sum_dist_ = 0;
}
void DetectSlipping::init_chassis_queue(){
    // std::queue container does not have a clear member function
    // clear queue: Swap with an empty queue; 
    std::queue<fairland_msgs::chassic_data>().swap(chassis_que_);
    chassis_sum_dist_ = 0;
}

void DetectSlipping::reset(){
    lidar_queue_init_ = false;
    chassis_queue_init_ = false;
    slipping_count_ = 0;
}

} // namespace localization_module
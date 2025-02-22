#include "pose_filter.h"



namespace localization_module{


PoseFilter::PoseFilter(/* args */){
    filter_reset();
}

PoseFilter::~PoseFilter(){
}

void PoseFilter::update_chassis(const fairland_msgs::chassic_data &chassis_msg){
    /// param  /// TODO: 参数化    
    static const double chassis_period = 0.02; // unit: second
    static const double time_interval_reset_thr = 0.5; // unit: second 
    // static const double time_interval_reset_thr = slam_param_.localization.time_interval_reset_thr;

    if(!update_lidar_init_.load()){
        ROS_ERROR_STREAM("lidar pose not init yet");
        return;
    }

    /// member var /// TODO: add mutex
    std::lock_guard<std::mutex> lock(chassis_mtx_);

    double chassis_time = chassis_msg.header.stamp.toSec();
    if(update_lidar_init_.load() && !update_chassis_init_.load()){
        chassis_init();
        last_chassis_time_ = chassis_time;

        update_chassis_init_.store(true);
    }

    cur_chassis_msg_ = chassis_msg;// prepare for pose filter
    
    // get chassis_time_interval
    double time_interval = chassis_time - last_chassis_time_;
    if(time_interval > time_interval_reset_thr || time_interval < 0.0){
        time_interval = chassis_period;
    }

    // // 轮子记录的位置
    double chassis_linear_vel = chassis_msg.ac_linear_velocity;
    chassis_x_ += chassis_linear_vel * time_interval * cos(curr_lidar_yaw_.load());
    chassis_y_ += chassis_linear_vel * time_interval * sin(curr_lidar_yaw_.load());

    // update
    last_chassis_time_ = chassis_time;

}

void PoseFilter::update_lidar(/*double lidar_time,*/ Eigen::Isometry3d curr_lidar_orig){
    static const int lidar_smooth_method = slam_param_.localization.filter_method;

    if(!update_lidar_init_.load()){
        lidar_init(curr_lidar_orig);

        update_lidar_init_.store(true);
        return;
    }
    
    curr_lidar_yaw_.store(angle_norm(R2ypr(curr_lidar_orig.linear())[0]));// prepare for chassis update

    if(lidar_smooth_method == 0){
        lidar_position_filter_fst_order(last_lidar_smooth_, curr_lidar_orig, curr_lidar_smooth_);
    }else if(lidar_smooth_method == 1){
        lidar_position_filter_window(last_lidar_smooth_, curr_lidar_orig, curr_lidar_smooth_);
    }

    last_lidar_smooth_ = curr_lidar_smooth_;
}

bool PoseFilter::run_filter(double k_chassis){
    // params
    static const double chassis_linear_velocity_thr = slam_param_.localization.chassis_linear_velocity_thr;
    static const double motionless_chassis_ratio    = slam_param_.localization.motionless_chassis_ratio;
    static const bool   using_turning_proc          = slam_param_.localization.using_turning_proc;
    /// TODO: 
    static const double filter_reset_time_thr = 0.5; /// unit: second 
    
    if(!update_chassis_init_.load() || !update_chassis_init_.load()){
        return false;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    /// lidar  
    double curr_lidar_smooth_x = curr_lidar_smooth_.translation().x();
    double curr_lidar_smooth_y = curr_lidar_smooth_.translation().y();
    double last_lidar_filter_x = curr_lidar_filter_.translation().x();// curr_lidar_filter_ 更新之前，存储的是上一帧的结果
    double last_lidar_filter_y = curr_lidar_filter_.translation().y();

    ///////////////////////////////////////////////////////////////////////////////////////////
    // chassis: 轮子记录的位置增量, delta time 与 chassis period 不一样
    std::lock_guard<std::mutex> lock(chassis_mtx_); ////////////////////////// lock

    /// TODO: need_reset, 在此处还是在上一层的调用
    // if(lidar_time - )

    // get:  k_chassis
    if(using_turning_proc){
        bool turning_flag = cur_chassis_msg_.left_front_feedback * cur_chassis_msg_.right_front_feedback<0 ? 1 : 0;
        if(!turning_flag && abs(cur_chassis_msg_.ac_linear_velocity)<chassis_linear_velocity_thr){
            k_chassis = motionless_chassis_ratio;
        }
        if(turning_flag){
            k_chassis = motionless_chassis_ratio;
        }
    }    
    // get:  chassis_dx & chassis_dy
    double chassis_dx = chassis_x_ - last_chassis_x_; 
    double chassis_dy = chassis_y_ - last_chassis_y_;

    ///////////////////////////////////////////////////////////////////////////////////////////
    /// filter   
    // get:  x y
    curr_lidar_filter_.translation().x() = (last_lidar_filter_x + chassis_dx)*k_chassis + curr_lidar_smooth_x*(1.0-k_chassis);
    curr_lidar_filter_.translation().y() = (last_lidar_filter_y + chassis_dy)*k_chassis + curr_lidar_smooth_y*(1.0-k_chassis);
    // get: rotation & z
    curr_lidar_filter_ = curr_lidar_smooth_;

    // update
    last_chassis_x_ = chassis_x_; // 注意区分: last_chassis_x_ 与 update_chassis() 中上一帧数据，频率不一样, filter_freq = lidar_freq != chcass_freq
    last_chassis_y_ = chassis_y_; // last chassis 只在一个线程中，不需要 atomic

    return true;
}

void PoseFilter::lidar_position_filter_fst_order(Eigen::Isometry3d last_pose_smooth, Eigen::Isometry3d curr_pose_orig, Eigen::Isometry3d& curr_pose_smooth){

    // // int k = 0.5;
    static const int k = slam_param_.localization.fst_order_k;

    double curr_lidar_orig_x = curr_pose_orig.translation().x();
    double curr_lidar_orig_y = curr_pose_orig.translation().y();
    double curr_lidar_orig_z = curr_pose_orig.translation().z();

    double last_lidar_smooth_x = last_pose_smooth.translation().x();
    double last_lidar_smooth_y = last_pose_smooth.translation().y();
    double last_lidar_smooth_z = last_pose_smooth.translation().z();

    curr_pose_smooth.translation().x() = last_lidar_smooth_x * k + curr_lidar_orig_x * (1.0-k);
    curr_pose_smooth.translation().y() = last_lidar_smooth_y * k + curr_lidar_orig_y * (1.0-k);
    curr_pose_smooth.translation().z() = last_lidar_smooth_z * k + curr_lidar_orig_z * (1.0-k);

    // // fill log **************************************************************
    // double last_x, last_y, last_z, last_roll, last_pitch, last_yaw;
    // pcl::getTranslationAndEulerAngles(last_pose_filtered, last_x, last_y, last_z, last_roll, last_pitch, last_yaw); //  获取上一帧 的 位姿

    // double curr_x, curr_y, curr_z, curr_roll, curr_pitch, curr_yaw;
    // pcl::getTranslationAndEulerAngles(curr_pose_orig, curr_x, curr_y, curr_z, curr_roll, curr_pitch, curr_yaw); //  获取当前帧 的 位姿

    // double dx = curr_x - last_x; // map 坐标系下 x 方向位移
    // double dy = curr_y - last_y; // map 坐标系下 y 方向位移

    // double delta_xy = std::sqrt(dx*dx + dy*dy);
    // double delta_yaw = angle_norm(curr_yaw - last_yaw);

    // double theta = angle_norm(atan2(dy, dx) - curr_yaw);
    // double baselink_dy = delta_xy * sin(theta);// 相对前进方向的横向位移
    // double baselink_dx = delta_xy * cos(theta);// 
    // double baselink_dyaw = rad2deg(delta_yaw);// 

    // log_info_manager_->log_info.base_frame_dx = baselink_dx; 
    // log_info_manager_->log_info.base_frame_dy = baselink_dy; 
    // log_info_manager_->log_info.base_frame_dyaw = baselink_dyaw; 

    // log_info_manager_->log_info.map_frame_dx = dx; 
    // log_info_manager_->log_info.map_frame_dy = dy; 

}

void PoseFilter::lidar_position_filter_window(Eigen::Isometry3d last_pose_smooth, Eigen::Isometry3d curr_pose_orig, Eigen::Isometry3d& curr_pose_smooth){
    // params
    static const int window_size = slam_param_.localization.window_size;
    static const double baselink_dx_thr = slam_param_.localization.baselink_dx_thr;
    static const double baselink_dy_thr = slam_param_.localization.baselink_dy_thr;
    static const double baselink_dyaw_thr = slam_param_.localization.baselink_dyaw_thr;

    Eigen::Isometry3d curr_pose_push = curr_pose_orig;

    Eigen::Vector3d last_xyz, last_ypr;
    Eigen::Vector3d curr_xyz, curr_ypr;

    if(pose_orig_deque_.size() > 0 ){
        // last_pose = pose_orig_deque_.back();

        get_xyz_ypr(last_pose_smooth, last_xyz, last_ypr);  //  获取上一帧 的 位姿  // ypr order: yaw pitch roll
        get_xyz_ypr(curr_pose_orig, curr_xyz, curr_ypr);    //  获取当前帧 的 位姿  // ypr order: yaw pitch roll

        double dx = curr_xyz[0] - last_xyz[0]; // map 坐标系下 x 方向位移
        double dy = curr_xyz[1] - last_xyz[1]; // map 坐标系下 y 方向位移

        double delta_xy = std::sqrt(dx*dx + dy*dy);
        double delta_yaw = angle_norm(curr_ypr[0] - last_ypr[0]);

        double theta = angle_norm(atan2(dy, dx) - curr_ypr[0]);
        double baselink_dy = delta_xy * sin(theta);// 相对前进方向的横向位移
        double baselink_dx = delta_xy * cos(theta);// 

        log_info_manager_->log_info.base_frame_dy = baselink_dy; 
        log_info_manager_->log_info.base_frame_dx = baselink_dx; 

        if (abs(baselink_dy) > baselink_dy_thr && abs(delta_yaw * 180 / PI_M) < baselink_dyaw_thr){
            // delta_xy = delta_xy * cos(theta);
            // double final_dx = delta_xy * cos(curr_ypr[0]);
            // double final_dy = delta_xy * sin(curr_ypr[0]);
            // curr_pose_smooth.translation().x() = last_pose_smooth.translation().x() + final_dx;
            // curr_pose_smooth.translation().y() = last_pose_smooth.translation().y() + final_dy;

            double kk =0.8;
            curr_pose_push.translation().x() = last_pose_smooth.translation().x() * kk + curr_pose_orig.translation().x() * (1-kk);
            curr_pose_push.translation().y() = last_pose_smooth.translation().y() * kk + curr_pose_orig.translation().y() * (1-kk);
            curr_pose_push.translation().z() = last_pose_smooth.translation().z() * kk + curr_pose_orig.translation().z() * (1-kk);

            // curr_pose_smooth.translation().x() = last_pose_smooth.translation().x() + last_lidar_dx_;
            // curr_pose_smooth.translation().y() = last_pose_smooth.translation().y() + last_lidar_dy_;
            // curr_pose_smooth.translation().z() = last_pose_smooth.translation().z() + last_lidar_dz_;
        }
        if((abs(baselink_dx) > baselink_dx_thr)){ // 0.05
            double kk =0.8;
            // curr_pose_smooth = curr_pose;
            curr_pose_push.translation().x() = last_pose_smooth.translation().x() * kk + curr_pose_orig.translation().x() * (1-kk);
            curr_pose_push.translation().y() = last_pose_smooth.translation().y() * kk + curr_pose_orig.translation().y() * (1-kk);
            curr_pose_push.translation().z() = last_pose_smooth.translation().z() * kk + curr_pose_orig.translation().z() * (1-kk);
        }
    }

    curr_pose_smooth.linear() = curr_pose_orig.linear();
    pose_orig_deque_.push_back(curr_pose_push);
    if (pose_orig_deque_.size() > window_size){
        pose_orig_deque_.pop_front();
        float w_sum, sum_x, sum_y, sum_z;
        w_sum = sum_x = sum_y = sum_z = 0;
        int i = 0;
        for(std::deque<Eigen::Isometry3d>::iterator it = pose_orig_deque_.begin(); it!=pose_orig_deque_.end(); ++it){
            auto p = pose_orig_deque_[i];
            double w = i+1; // double w = 1;            
            sum_x += (it->translation().x() * w);
            sum_y += (it->translation().y() * w);
            sum_z += (it->translation().z() * w);
            w_sum += w;
            i++;
        }
        curr_pose_smooth.translation().x() = sum_x / w_sum;
        curr_pose_smooth.translation().y() = sum_y / w_sum;
        curr_pose_smooth.translation().z() = sum_z / w_sum;

        get_xyz_ypr(curr_pose_smooth, curr_xyz, curr_ypr); // ypr order: yaw pitch roll

        double dx = curr_xyz[0] - curr_xyz[0]; // map 坐标系下 x 方向位移
        double dy = curr_xyz[1] - last_xyz[1]; // map 坐标系下 y 方向位移
        double dist_xy = std::sqrt(dx*dx + dy*dy);
        double delta_yaw = angle_norm(curr_ypr[0] - last_ypr[0]); // ypr order: yaw pitch roll

        double theta = angle_norm(atan2(dy, dx) - curr_ypr[0]);
        double baselink_dy = dist_xy * sin(theta);// 相对前进方向的横向位移
        double baselink_dx = dist_xy * cos(theta);// 

        log_info_manager_->log_info.map_frame_dx = dx; 
        log_info_manager_->log_info.map_frame_dy = dy; 
        log_info_manager_->log_info.base_frame_dy = baselink_dy; 
        log_info_manager_->log_info.base_frame_dx = baselink_dx; 
        log_info_manager_->log_info.base_frame_dyaw = rad2deg(delta_yaw); 
    }
}

bool PoseFilter::lidar_init(Eigen::Isometry3d init_pose){

    last_lidar_smooth_ = init_pose;
    curr_lidar_smooth_ = init_pose;
    curr_lidar_filter_ = init_pose;

    last_chassis_x_ = init_pose.translation().x();
    last_chassis_y_ = init_pose.translation().y();

    return true;
}

bool PoseFilter::chassis_init(){
    chassis_x_ = curr_lidar_smooth_.translation().x();
    chassis_y_ = curr_lidar_smooth_.translation().y();

    return true;
}

bool PoseFilter::filter_reset(){
    update_lidar_init_.store(false);
    update_chassis_init_.store(false);

    pose_orig_deque_.clear();

    last_lidar_smooth_ = Eigen::Isometry3d::Identity();
    curr_lidar_smooth_ = Eigen::Isometry3d::Identity();
    curr_lidar_filter_ = Eigen::Isometry3d::Identity();
    curr_lidar_yaw_.store(0.0);

    last_chassis_x_ = 0.0;
    last_chassis_y_ = 0.0;

    // chassis_a_ = 0.0;

    return true;
}

bool PoseFilter::load_param(){
    // init 
    LocalizationModuleParamManager *param_manager = LocalizationModuleParamManager::Instance();
    const lidar_slam::LidarSlamParam* loaded_param = param_manager->get_loaded_param();

    if (loaded_param == NULL) {
        ROS_ERROR_STREAM(RED << "loaded_param is NULL" <<RESET);
        return false;
    }else{
        slam_param_ = *loaded_param;
        return true;
    }
}


    
} // namespace localization_module
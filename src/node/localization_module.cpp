#include <ros/ros.h>
#include "node/localization_module.h"

#define L_WHEEL     0.385f

namespace localization_module {
LocalizationModule::LocalizationModule(/*const std::string work_path,*/ ModuleStatus init_status){
    log_info_manager_ = LocalizationModuleLogInfoManager::getInstance();
    // curr_dir_ = work_path;

    // // load_params();
    if (!load_lidar_slam_param()){
        ROS_ERROR("Load lidar-slam param failed!");
    }else {
        ROS_INFO("\033[1;32mLoad lidar-slam param successfully!\033[0m");
    }

    //************************** CPU 绑定 *******************************
    CPU_ZERO(&mask); // 初始化 CPU 亲和性集合，将其设置为零
    for(int i=0; i<slam_param_.common.cpu_id.size();i++){
        CPU_SET(slam_param_.common.cpu_id[i], &mask); // 将线程绑定到 cpu_id 核心
        ROS_INFO("\033[1;32mset cpu: %d\033[0m", slam_param_.common.cpu_id[i]);
    }
    //************************** CPU 绑定 end *******************************

    if(!create_ROS_IO()){
        ROS_ERROR("Create ROS-IO failed!");
    }else {
        ROS_INFO("Create ROS-IO successfully!");
    }

    //************************** TODO: 待确认 *******************************
    if (show_rviz_){// this param load from lasunch file
        show_thread_ = std::thread(&LocalizationModule::show_thread, this);
        ROS_INFO("Show_thread started");
    }

    // TODO
    std::thread load_data_thread;
    if (offline_mode_){
        // 读取文件夹中的文件名
    }
    //***********************************************************************


    ROS_INFO("***************************************************");
    if(!init_module_by_set_status(init_status)){
        ROS_INFO("Try to init module with status: %s, but failed",print_ModuleStatus(init_status).c_str());
    }else{
        ROS_INFO("Localization Module Start with status:\033[1;32m %s\033[0m", print_ModuleStatus(running_module_status_).c_str());
    }

    position_filter_thread_.reset(new  std::thread(&LocalizationModule::position_filter_thread, this));
    
    // position_filter_thread_.join();

    ROS_INFO("***************************************************");
}

LocalizationModule::~LocalizationModule(){

}

bool LocalizationModule::position_init(Eigen::Isometry3d init_pose){
    if (running_module_status_ == MODULE_IDLE){
        ROS_INFO("pose init: wait for module start");
        position_initialized_ = false;
        return false;
    }

    if (!slam_ || releasing_slam_flag_){
        ROS_INFO("pose init: slam not ready !");
        position_initialized_ = false;
        return false;
    }


    // auto pose = slam_->getLidarInMap(); 
    auto pose = init_pose;

    if (abs(pose.translation().x())>0.01){
        lidar_x_ = pose.translation().x();
        lidar_y_ = pose.translation().y();
        lidar_a_ = angle_norm(R2ypr(pose.rotation()).x());

        filter_x_ = lidar_x_; // 当前位置
        filter_y_ = lidar_y_;
        filter_a_ = lidar_a_;

        last_lidar_x_ = lidar_x_;
        last_lidar_y_ = lidar_y_;
        last_lidar_a_ = lidar_a_;

        chassis_x_ = lidar_x_;
        chassis_y_ = lidar_y_;
        chassis_a_ = lidar_a_;
        last_chassis_x_ = chassis_x_;
        last_chassis_y_ = chassis_y_;
        last_chassis_a_ = chassis_a_;

        std::cout<< "positon: init x:" << filter_x_ << ", y:" << filter_y_ << ", a:" << filter_a_ << std::endl;
        return true;
    }else{
        std::cout << "position_filter: wait for lidar pose ..." << std::endl;
        position_initialized_ = false;
        sleep(1);
        return false;
    }
}

void LocalizationModule::lidar_position_filter_window(Eigen::Isometry3d pose_temp,Eigen::Isometry3d lidar_in_map, Eigen::Isometry3d & pose_filtered){
    fairland_msgs::LocalizationModuleLogInfo log_msg;
    window_size = slam_param_.localization.window_size;
    // Eigen::Vector3d pos_sum;
    Eigen::Isometry3d last_pose = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d cur_pose;
    // Eigen::Isometry3d pose_filtered;
    cur_pose = lidar_in_map;
    pose_filtered = cur_pose;

    if(pose_vec.size() > 0 ){
        // Eigen::Isometry3d last_pose = pose_vec[pose_vec.size()-1];
        last_pose = pose_vec[pose_vec.size()-1];

        double last_x, last_y, last_z, last_roll, last_pitch, last_yaw;
        pcl::getTranslationAndEulerAngles(last_pose, last_x, last_y, last_z, last_roll, last_pitch, last_yaw); //  获取上一帧 的 位姿

        double curr_x, curr_y, curr_z, curr_roll, curr_pitch, curr_yaw;
        pcl::getTranslationAndEulerAngles(cur_pose, curr_x, curr_y, curr_z, curr_roll, curr_pitch, curr_yaw); //  获取当前帧 的 位姿

        double dy = curr_y - last_y; // map 坐标系下 y 方向位移
        double dx = curr_x - last_x; // map 坐标系下 x 方向位移

        double delta_xy = std::sqrt(dx*dx + dy*dy);
        double delta_yaw = angle_norm(curr_yaw - last_yaw);

        double theta = angle_norm(atan2(dy, dx) - curr_yaw);
        double baselink_dy = delta_xy * sin(theta);// 相对前进方向的横向位移
        double baselink_dx = delta_xy * cos(theta);// 

        // log_msg.header.stamp = ros::Time().fromSec(slam_->get_lidar_time());
        // log_msg.delta_yaw = delta_yaw * 180 / PI_M;
        // log_msg.delta_xy = delta_xy;
        // log_msg.curr_yaw = curr_yaw * 180 / PI_M;

        // std::cout<<"temp_yaw: "<<angle_norm(theta - curr_yaw) * 180 / PI_M<<std::endl;
        // std::cout<<"curr_yaw: "<<curr_yaw * 180 / PI_M<<std::endl;
        // std::cout<<"delta_xy: "<<delta_xy<<std::endl;

        // std::cout<<"delta_yaw: \033[0m"<<delta_yaw * 180 / PI_M<<std::endl;
        // std::cout<<"theta: \033[0m"<<theta * 180 / PI_M<<std::endl;
        // std::cout<<"baselink_dy: "<<baselink_dy<<std::endl;

        log_info_manager_->log_info.base_frame_dy = baselink_dy; 
        log_info_manager_->log_info.base_frame_dx = baselink_dx; 

        if (abs(baselink_dy) > slam_param_.localization.baselink_dy_thr && abs(delta_yaw * 180 / PI_M)<slam_param_.localization.baselink_dyaw_thr){
            // std::cout<<"\033[1;32mdelta_y: \033[0m"<<baselink_dy<<std::endl;
            // std::cout<<"\033[1;32mdelta_yaw: \033[0m"<<delta_yaw * 180 / PI_M<<std::endl;
            // std::cout<<"\033[1;32mtheta: \033[0m"<<theta * 180 / PI_M<<std::endl;
            
            // delta_xy = delta_xy * cos(theta);
            // double final_dx = delta_xy * cos(curr_yaw);
            // double final_dy = delta_xy * sin(curr_yaw);
            // pose_filtered.translation().x() = last_pose.translation().x() + final_dx;
            // pose_filtered.translation().y() = last_pose.translation().y() + final_dy;

            double kk =0.8;
            pose_filtered.translation().x() = last_pose.translation().x() * kk + cur_pose.translation().x() * (1-kk);
            pose_filtered.translation().y() = last_pose.translation().y() * kk + cur_pose.translation().y() * (1-kk);
            pose_filtered.translation().z() = last_pose.translation().z() * kk + cur_pose.translation().z() * (1-kk);

            // pose_filtered.translation().x() = last_pose.translation().x() + last_lidar_dx_;
            // pose_filtered.translation().y() = last_pose.translation().y() + last_lidar_dy_;
            // pose_filtered.translation().z() = last_pose.translation().z() + last_lidar_dz_;
        }
        // if((abs(baselink_dx) > slam_param_.localization.baselink_dx_thr)){ // 0.05
        //     double kk =1;
        //     cur_pose = slam_->getLastOdomToMap() * slam_->getLidarInOdom();
        //     pose_filtered = cur_pose;
        //     // pose_filtered.translation().x() = last_pose.translation().x() * kk + pose_filtered.translation().x() * (1-kk);
        //     // pose_filtered.translation().y() = last_pose.translation().y() * kk + cur_pose.translation().y() * (1-kk);
        //     // pose_filtered.translation().z() = last_pose.translation().z() * kk + cur_pose.translation().z() * (1-kk);

        // }
    }

    pose_vec.push_back(pose_filtered);
    if (pose_vec.size() > window_size){
        // std::cout<<"pose_vec size: "<<pose_vec.size()<<endl;
        pose_vec.erase(pose_vec.begin());
        // pose_vec.pop_front();
        // std::cout<<"pose_vec size: "<<pose_vec.size()<<endl;

        float w_sum = 0;
        float sum_x = 0;
        float sum_y = 0;
        float sum_z = 0;

        for (int i=0; i<pose_vec.size(); i++){
            auto p = pose_vec[i];
            double w = i+1;
            // double w = 1;
            sum_x += (p.translation().x() * w);
            sum_y += (p.translation().y() * w);
            sum_z += (p.translation().z() * w);
            w_sum += w;
        }
        pose_filtered.translation().x() = sum_x / w_sum;
        pose_filtered.translation().y() = sum_y / w_sum;
        pose_filtered.translation().z() = sum_z / w_sum;

        pose_vec[pose_vec.size() -1] = pose_filtered;
    }

    last_lidar_dx_ = last_pose.translation().x() -  pose_filtered.translation().x();
    last_lidar_dy_ = last_pose.translation().y() -  pose_filtered.translation().y();
    last_lidar_dz_ = last_pose.translation().z() -  pose_filtered.translation().z();

    pub_log_.publish(log_msg);
}

void LocalizationModule::lidar_position_filter_fst_order(Eigen::Isometry3d last_pose, Eigen::Isometry3d curr_pose, Eigen::Isometry3d & pose_filtered){

    // set var for position filter
    lidar_time_ = slam_->get_lidar_time();
    double lidar_x_new = curr_pose.translation().x();
    double lidar_y_new = curr_pose.translation().y();

    // // int k = 0.5;
    int k = slam_param_.localization.fst_order_k;
    lidar_x_ = last_lidar_x_ * k + lidar_x_new * (1.0-k);
    lidar_y_ = last_lidar_y_ * k + lidar_y_new * (1.0-k);
    lidar_a_ = angle_norm(R2ypr(pose_filtered.rotation()).x());

    pose_filtered.translation().x() = lidar_x_;
    pose_filtered.translation().y() = lidar_y_;

    // fill log **************************************************************
    double last_x, last_y, last_z, last_roll, last_pitch, last_yaw;
    pcl::getTranslationAndEulerAngles(last_pose, last_x, last_y, last_z, last_roll, last_pitch, last_yaw); //  获取上一帧 的 位姿

    double curr_x, curr_y, curr_z, curr_roll, curr_pitch, curr_yaw;
    pcl::getTranslationAndEulerAngles(curr_pose, curr_x, curr_y, curr_z, curr_roll, curr_pitch, curr_yaw); //  获取当前帧 的 位姿

    double dy = curr_y - last_y; // map 坐标系下 y 方向位移
    double dx = curr_x - last_x; // map 坐标系下 x 方向位移

    double delta_xy = std::sqrt(dx*dx + dy*dy);
    // double delta_yaw = angle_norm(curr_yaw - last_yaw);

    double theta = angle_norm(atan2(dy, dx) - curr_yaw);
    double baselink_dy = delta_xy * sin(theta);// 相对前进方向的横向位移
    double baselink_dx = delta_xy * cos(theta);// 

    log_info_manager_->log_info.base_frame_dy = baselink_dy; 
    log_info_manager_->log_info.base_frame_dx = baselink_dx; 
    log_info_manager_->log_info.map_frame_dy = dy; 
    log_info_manager_->log_info.map_frame_dx = dx; 

}

void LocalizationModule::position_filter(){
    float ka = 0.5;  // 角度滤波系数 事实上不用
    detect_slipping();

    // 轮子记录的位置增量
    float chassis_dx = chassis_x_ - last_chassis_x_;
    float chassis_dy = chassis_y_ - last_chassis_y_;
    float chassis_da = angle_norm(chassis_a_ - last_chassis_a_);

    // float lidar_dx = lidar_x_ - last_lidar_x_;
    // float lidar_dy = lidar_y_ - last_lidar_y_;
    // float lidar_da = lidar_a_ - last_lidar_a_;

    filter_x_ = (filter_x_ + chassis_dx)*k_pos_ + lidar_x_*(1.0-k_pos_);
    filter_y_ = (filter_y_ + chassis_dy)*k_pos_ + lidar_y_*(1.0-k_pos_);
    filter_a_ = angle_norm((filter_a_ + chassis_da)*ka + lidar_a_*(1.0-ka)); // 这个就很鬼畜 // 这个值没有用上

    // update
    last_chassis_x_ = chassis_x_;
    last_chassis_y_ = chassis_y_;
    last_chassis_a_ = chassis_a_;

    last_lidar_x_ = lidar_x_;
    last_lidar_y_ = lidar_y_;
    last_lidar_a_ = lidar_a_;
}

float line_length(float dx, float dy){
  return std::sqrt(dx*dx + dy*dy);
}

void LocalizationModule::detect_slipping(){
    k_pos_ = 1 - slam_param_.localization.lidar_ratio;
    // 雷达定位值在车身对称轴方向上的增量
    float dx = lidar_x_ - last_lidar_x_;
    float dy = lidar_y_ - last_lidar_y_;

    float l_da = angle_norm(last_lidar_a_ + angle_norm(lidar_a_ - last_lidar_a_)/2.0);  // 两帧的角度均值
    float p_da = std::atan2(dy, dx);// 速度方向的角度
    float da = angle_norm(l_da - p_da);// 速度方向与车身方向的夹角
    float l_dr = line_length(dx, dy);// lidar 计算的 两帧之间的移动距离

    float ln_dr = l_dr * std::cos(da);// 车身方向的位移
    float o_dr = line_length(chassis_x_ - last_chassis_x_, chassis_y_ - last_chassis_y_);// 底盘计算的两帧之间的移动距离

    if (chassis_linear_velocity_ > 0.1 && chassis_angular_velocity_ < 0.2 && o_dr-ln_dr > o_dr*0.75f){ // 暂时写成定值
        if (slip_count_ > 3) {
            k_pos_ = 0.0;
            std::cout << " --- slipping ---" << std::endl; 
        }
        else slip_count_++;
    }
    else slip_count_ = 0;

    log_info_manager_->log_info.slip_count = slip_count_;
}

bool LocalizationModule::create_ROS_IO(){
    // subscriber ********************************************************************
	// ros::Subscriber sub_pcl = nh_.subscribe<livox_ros_driver2::CustomMsg>("/livox/lidar", 200000, &LocalizationModule::livox_pcl_cbk, this);
    sub_pointcloud2_ = nh_.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 10, &LocalizationModule::livox_pcl_cbk, this);
    sub_imu_ = nh_.subscribe<sensor_msgs::Imu>("/livox/imu", 200000, &LocalizationModule::imu_cbk, this);
    sub_chassis_ = nh_.subscribe<fros_hardware_node::chassic_data>("/flbot/hardware/chassic_data", 100, &LocalizationModule::chassis_cbk, this);

    sub_mapping_ctrl_ = nh_.subscribe(slam_param_.common.sub_topic_ctrl_cmd, 3 ,&LocalizationModule::localization_module_ctrl_cbk, this);
    
    // timer dealt ********************************************************************
    // 建图主要流程，timer 时间间隔需要调整，10hz? 100hz? 200hz?
    // timer_slam_ = nh_.createTimer(ros::Duration(0.01), &LocalizationModule::slam_dealt_timer, this);
    timer_slam_ = nh_.createTimer(ros::Duration(0.05), &LocalizationModule::slam_dealt_timer, this);
    timer_module_status_ = nh_.createTimer(ros::Duration(0.05), &LocalizationModule::pub_module_status_timer, this);

    
    // publish ************************************************************************
    pub_localization_module_status_ = nh_.advertise<fairland_msgs::LocalizationModuleStatus>(slam_param_.common.pub_topic_module_status, 100); 
    pub_filter_odometry_ = nh_.advertise<nav_msgs::Odometry>("/Odometry_lidar_in_map_filter", 100); 
    pub_log_ = nh_.advertise<fairland_msgs::LocalizationModuleLogInfo>(slam_param_.common.pub_topic_module_loginfo, 100); 

    // both 建图 & 定位
	pubLidarInMap = nh_.advertise<nav_msgs::Odometry>("/Odometry_lidar_in_map", 100);

    // only 建图 


    // only 定位

    // publish TODO: 还需要区分哪些是建图或定位发布的
    pubOdomCloud = nh_.advertise<sensor_msgs::PointCloud2>("/odom_cloud", 100000);  
	pubBodyCloud = nh_.advertise<sensor_msgs::PointCloud2>("/body_cloud", 100000);
    pubObstacleCloud = nh_.advertise<sensor_msgs::PointCloud2>("/obstacle_cloud", 100000);
    pubFilteredObstacleCloud = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_obstacle_cloud", 100000);
    pubTestCloud = nh_.advertise<sensor_msgs::PointCloud2>("/test_cloud", 100000);
	pubKdtreeCloud = nh_.advertise<sensor_msgs::PointCloud2>("/kdtree_cloud", 100000); 
	pubOptimizedPath= nh_.advertise<nav_msgs::Path>("/optimized_path", 1000);
    pubUnoptimizedPath= nh_.advertise<nav_msgs::Path>("/unoptimized_path", 1000);
	pubLoopConstraintEdge = nh_.advertise<visualization_msgs::MarkerArray>("/loop_closure_constraints", 1);
    pubKeyframePose = nh_.advertise<visualization_msgs::MarkerArray>("/key_frame_pose", 1);
	pubOdomAftMapped = nh_.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    pubLoadMap = nh_.advertise<sensor_msgs::PointCloud2>("/Load_map", 100000);
    pubRgbCloud= nh_.advertise<sensor_msgs::PointCloud2>("rgb_cloud", 1);
    // image_pub = nh_.advertise<sensor_msgs::Image>("fisheye_image", 1);

    return true;
}

void LocalizationModule::slam_dealt_timer(const ros::TimerEvent &event){
    // SLAM 主要流程， 对应于原来的 while (ros::ok()){...}
    std::thread::id thisId = std::this_thread::get_id();
    // std::cout << "debug: slam_dealt_timer    Thread ID: " << thisId << std::endl;

    if(slam_param_.common.cpu_id.size()>0){
        pthread_t this_thread = pthread_self(); // 获取当前线程的 ID
        if (pthread_setaffinity_np(this_thread, sizeof(mask), &mask) < 0) {
            perror("pthread_setaffinity_np");
            exit(EXIT_FAILURE);
        }
    }
    

    // if (!running_slam_){
    if (running_module_status_ == MODULE_IDLE || 
        running_module_status_ == MODULE_STARTING_SLAM || 
        running_module_status_ == MODULE_STOPPING_SLAM){
        ROS_INFO("running module status: %s ", print_ModuleStatus(running_module_status_).c_str());
        sleep(2);
        return;
    }

    // ROS_INFO("***********************************************");
    // cout<<"-----------------------------------------------"<<endl;
    // cout<<"***********************************************"<<endl;
    // ROS_INFO("running module status: %s", print_ModuleStatus(running_module_status_).c_str());

    // if (control_status_.reset){
    //     sleep(1);
    //     slam_.reset();
    //     sleep(1);
    //     localization_mode_ = control_status_.localizationMode;
    //     // slam_ = std::make_unique<lidar_slam::LidarSlam>(curr_dir_+std::string("/lib/"),localization_mode_,offline_mode_);
    //     slam_ = std::make_unique<lidar_slam::LidarSlam>(curr_dir_+std::string("/"),localization_mode_,offline_mode_, 0);
    //     show_load_map_ = 0;
    //     control_status_.reset = false;

    // }
    
    // ROS_INFO("trying to get loaded map...");
    // if (show_load_map_==0 && localization_mode_ && (slam_->getLoadMap())->points.size() > 0){
    // if (show_load_map_==0 && localization_mode_ && (slam_->getLoadMap()) && (slam_->getLoadMap())->points.size() > 0){
    // if (show_load_map_==0 && running_module_status_==MODULE_LOCALIZATION && (slam_->getLoadMap()) && (slam_->getLoadMap())->points.size() > 0){
    if (show_load_map_%200==0 && running_module_status_==MODULE_LOCALIZATION && (slam_->getLoadMap()) && (slam_->getLoadMap())->points.size() > 0){
        // ROS_INFO("load map");
        // sleep(1);
        sensor_msgs::PointCloud2 loadMap;
        pcl::toROSMsg(*(slam_->getLoadMap()), loadMap); 
        loadMap.header.stamp = ros::Time::now();
        loadMap.header.frame_id = "map";
        pubLoadMap.publish(loadMap);
        show_keyframe(slam_->getLoadKeyFrame(), pubKeyframePose);
        // ROS_INFO("load map success");
        // show_load_map_ ++;
    }
    show_load_map_ ++;



    if (just_show_mode_){
        return;
    } 
    /********************************- run slam -********************************/    

    thisId = std::this_thread::get_id();
    // std::cout << "debug: slam_->run()        Thread ID: " << thisId << std::endl;
    bool running_slam_flag = slam_->run();

    if (running_slam_flag && show_rviz_){
        // if (!localization_mode_ || slam_->isGloalLocalizationSuccess()){
        if ((running_module_status_ == MODULE_MAPPING || running_module_status_ == MODULE_SEC_MAPPING) 
            || slam_->isGloalLocalizationSuccess()){
            pub_odom_cloud(slam_->get_odom_cloud(), pubOdomCloud);
        }
        pub_test_cloud(slam_->getTestCloud(), localization_mode_, pubTestCloud);
        pub_lidar_cloud(slam_->get_lidar_cloud(), pubBodyCloud);
        pub_obstacle_cloud(slam_->getObstacleCloud(), pubObstacleCloud);
        pub_filtered_obstacle_cloud(slam_->getFilteredObstacleCloud(), pubFilteredObstacleCloud);
        publish_unoptimized_path(slam_->get_unoptimized_path(),pubUnoptimizedPath);
        publish_optimized_path(slam_->get_optimized_path(),string("odom"), pubOptimizedPath);
        visualizeLoopClosure(slam_->getloopIndex(),optimized_path_msg, pubLoopConstraintEdge);
        publish_transform(slam_->getOdomToMap(),string("map"),string("odom"));
        // pub_kdtree_cloud(slam_->get_kdtree_cloud());		//not used yet
    }
    // if(!localization_mode_){
    if(running_module_status_ == MODULE_MAPPING || running_module_status_ == MODULE_SEC_MAPPING){
    // pub_rgb_map(slam->getCurrentRGBMap());
        publish_odometry_lidar_in_map(slam_->getLidarInMap(), "map", "base_footprint", pubLidarInMap);
        pub_lidar_cloud(slam_->get_lidar_cloud(), pubBodyCloud);
    }else if(running_module_status_ == MODULE_LOCALIZATION){
        publish_odometry_lidar_in_map(slam_->getLidarInMap(), "map", "base_footprint", pubLidarInMap);
        publish_odometry(slam_->getLidarInOdom(), pubOdomAftMapped);
        pub_lidar_cloud(slam_->get_lidar_cloud(), pubBodyCloud);
    }
    if (show_rviz_){
        publish_static_transform(slam_->getWheelInLidar());
        publish_odometry(slam_->getLidarInOdom(), pubOdomAftMapped);
    }
}

// void LocalizationModule::position_filter_thread(){
//     position_init();
//     int pub_frequency = 20;
//     int rate = slam_param_.localization.filter_freq / pub_frequency;
//     const int filter_frequency = slam_param_.localization.filter_freq;
//     // const std::chrono::milliseconds period(1000 / filter_frequency);
//     // auto last_pub_time = std::chrono::steady_clock::now();// init
//     ros::Rate filter_rate = ros::Rate(filter_frequency);
//     while (ros::ok()) {
//         auto start = std::chrono::steady_clock::now();
//         // if(running_module_status_ == MODULE_LOCALIZATION && localization_status_ == L_NORMAL){
//             // double last_x, last_y, last_z, last_roll, last_pitch, last_yaw;
//             // pcl::getTranslationAndEulerAngles(correctionOdomToMap, last_x, last_y, last_z, last_roll, last_pitch, last_yaw); //  获取上一帧 相对 当前帧的 位姿
//             // double curr_x, curr_y, curr_z, curr_roll, curr_pitch, curr_yaw;
//             // pcl::getTranslationAndEulerAngles(temp_correct, curr_x, curr_y, curr_z, curr_roll, curr_pitch, curr_yaw); //  获取上一帧 相对 当前帧的 位姿
//             // log_info_manager_->log_info.lidar2odom_dtime  = curr_time_ - lastUpdateTime;
//             // log_info_manager_->log_info.lidar2odom_dx = curr_x - last_x;
//             // log_info_manager_->log_info.lidar2odom_dy = curr_y - last_y;
//             // log_info_manager_->log_info.lidar2odom_dz = curr_z - last_z;
//             // log_info_manager_->log_info.lidar2odom_droll  = 180 / PI_M * (curr_roll  - last_roll);
//             // log_info_manager_->log_info.lidar2odom_dpitch = 180 / PI_M * (curr_pitch - last_pitch);
//             // log_info_manager_->log_info.lidar2odom_dyaw   = 180 / PI_M * (curr_yaw   - last_yaw);
//             Eigen::Isometry3d lidar_in_map = slam_->getLidarInMap();
//             filter_odometry_.header.frame_id = "map";
//             filter_odometry_.child_frame_id = "base_footprint";
//             filter_odometry_.header.stamp = ros::Time().now(); // ros::Time().fromSec(lidar_end_time);
//             // filter_odometry_.header.stamp = ros::Time().fromSec(lidar_time_);
//             filter_odometry_.pose.pose.position.x = lidar_in_map.translation().x();
//             filter_odometry_.pose.pose.position.y = lidar_in_map.translation().y();
//             filter_odometry_.pose.pose.position.z = lidar_in_map.translation().z();
//             Eigen::Quaterniond quaternion = Eigen::Quaterniond(lidar_in_map.rotation());
//             filter_odometry_.pose.pose.orientation.x = quaternion.x();
//             filter_odometry_.pose.pose.orientation.y = quaternion.y();
//             filter_odometry_.pose.pose.orientation.z = quaternion.z();
//             filter_odometry_.pose.pose.orientation.w = quaternion.w();
//             Eigen::Isometry3d pose_filtered = Eigen::Isometry3d::Identity();
//             pose_filtered = lidar_in_map;
//             if(slam_param_.localization.filter_method == 0){
//                 lidar_position_filter_fst_order(lidar_in_map, pose_filtered);
//             }else if(slam_param_.localization.filter_method == 1){
//                 lidar_position_filter_window(lidar_in_map, pose_filtered);
//             }
//             lidar_x_ = pose_filtered.translation().x();
//             lidar_y_ = pose_filtered.translation().y();
//             lidar_a_ = R2ypr(pose_filtered.rotation()).x();
//             lidar_a_ = angle_norm(lidar_a_);
//             /// filter with chassis            
//             position_filter();
//             filter_odometry_.pose.pose.position.x = filter_x_;
//             filter_odometry_.pose.pose.position.y = filter_y_;
//             filter_cout_ ++;
//             if(filter_cout_ % rate == 0 ){
//                 filter_cout_ = 0;
//                 pub_filter_odometry_.publish(filter_odometry_);
//                 // auto odom_for_tf = odomAftMapped;
//                 auto odom_for_tf = filter_odometry_;
//                 static tf::TransformBroadcaster br;
//                 tf::Transform transform;
//                 tf::Quaternion q;
//                 transform.setOrigin(tf::Vector3(odom_for_tf.pose.pose.position.x,
//                                                 odom_for_tf.pose.pose.position.y,
//                                                 odom_for_tf.pose.pose.position.z));
//                 q.setW(odom_for_tf.pose.pose.orientation.w);
//                 q.setX(odom_for_tf.pose.pose.orientation.x);
//                 q.setY(odom_for_tf.pose.pose.orientation.y);
//                 q.setZ(odom_for_tf.pose.pose.orientation.z);
//                 transform.setRotation(q);
//                 br.sendTransform(tf::StampedTransform(transform, odom_for_tf.header.stamp, "map", "base_footprint"));
//             }
//             pub_log_.publish(log_info_manager_->log_info);
//             filter_rate.sleep();
//         }
//         // auto end = std::chrono::steady_clock::now();
// }

void LocalizationModule::position_filter_thread(){
    const int pub_frequency = 20;
    const int filter_frequency = slam_param_.localization.filter_freq;


    const std::chrono::milliseconds pub_period(1000 / pub_frequency);
    const std::chrono::milliseconds filter_period(1000 / filter_frequency);
    
    auto last_pub_time = std::chrono::steady_clock::now();// init

    static Eigen::Isometry3d last_pose = Eigen::Isometry3d::Identity();

    while (true) {
        auto start = std::chrono::steady_clock::now();

        if (running_module_status_ == MODULE_IDLE ){
            ROS_INFO("position_filter: wait for module start");
            position_initialized_ = false;
            sleep(2);
            continue;
        }

        if (!slam_ || releasing_slam_flag_){ // slam_ 对象为空, 或正在释放对象
            position_initialized_ = false;
            ROS_INFO("position_filter: slam not ready !");
            sleep(1);
            continue;
        }

        // init 
        if (!position_initialized_){
            bool localization_flag = (running_module_status_ == MODULE_LOCALIZATION ? 1 : 0);
            bool mapping_flag = ((running_module_status_ == MODULE_MAPPING || running_module_status_ == MODULE_MAPPING) ? 1 : 0);

            if ((localization_flag && localization_status_ == L_NORMAL) || 
                (mapping_flag && mapping_status_ == M_STANDBY)){
                auto curr_pose = slam_->getLidarInMap(); 
                if(position_init(curr_pose)){
                    position_initialized_ = true;
                }
                last_pose = curr_pose; // init
            }

            auto end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            if (elapsed < filter_period) {
                std::this_thread::sleep_for(filter_period - elapsed);
            }
            continue;
        }
        
        // 初始化后下一帧开始正常处理
        Eigen::Isometry3d lidar_in_map = slam_->getLidarInMap();
        // init filter_odometry

        nav_msgs::Odometry filter_odometry;
        filter_odometry.header.frame_id = "map";
        filter_odometry.child_frame_id = "base_footprint";
        filter_odometry.header.stamp = ros::Time().now(); // ros::Time().fromSec(lidar_end_time);
        filter_odometry.pose.pose.position.x = lidar_in_map.translation().x();
        filter_odometry.pose.pose.position.y = lidar_in_map.translation().y();
        filter_odometry.pose.pose.position.z = lidar_in_map.translation().z();
        Eigen::Quaterniond quaternion = Eigen::Quaterniond(lidar_in_map.rotation());
        filter_odometry.pose.pose.orientation.x = quaternion.x();
        filter_odometry.pose.pose.orientation.y = quaternion.y();
        filter_odometry.pose.pose.orientation.z = quaternion.z();
        filter_odometry.pose.pose.orientation.w = quaternion.w();

        // init pose_filtered
        Eigen::Isometry3d pose_filtered = Eigen::Isometry3d::Identity();
        pose_filtered = lidar_in_map;

        // // fill log ****************************************************
        // double last_x, last_y, last_z, last_roll, last_pitch, last_yaw;
        // pcl::getTranslationAndEulerAngles(correctionOdomToMap, last_x, last_y, last_z, last_roll, last_pitch, last_yaw); //  获取上一帧 相对 当前帧的 位姿
        // double curr_x, curr_y, curr_z, curr_roll, curr_pitch, curr_yaw;
        // pcl::getTranslationAndEulerAngles(temp_correct, curr_x, curr_y, curr_z, curr_roll, curr_pitch, curr_yaw); //  获取上一帧 相对 当前帧的 位姿

        // log_info_manager_->log_info.lidar2odom_dtime  = curr_time_ - lastUpdateTime;
        // log_info_manager_->log_info.lidar2odom_dx = curr_x - last_x;
        // log_info_manager_->log_info.lidar2odom_dy = curr_y - last_y;
        // log_info_manager_->log_info.lidar2odom_dz = curr_z - last_z;
        // log_info_manager_->log_info.lidar2odom_droll  = 180 / PI_M * (curr_roll  - last_roll);
        // log_info_manager_->log_info.lidar2odom_dpitch = 180 / PI_M * (curr_pitch - last_pitch);
        // log_info_manager_->log_info.lidar2odom_dyaw   = 180 / PI_M * (curr_yaw   - last_yaw);

        // // fill log end ****************************************************


        // // fill log ****************************************************

        // // fill log end ****************************************************

        if(slam_param_.localization.filter_method == 0){
            lidar_position_filter_fst_order(last_pose, lidar_in_map, pose_filtered);
        }else if(slam_param_.localization.filter_method == 1){
            lidar_position_filter_window(last_pose, lidar_in_map, pose_filtered);
        }
        

        lidar_x_ = pose_filtered.translation().x();
        lidar_y_ = pose_filtered.translation().y();
        lidar_a_ = angle_norm(R2ypr(pose_filtered.rotation()).x());
        /// filter with chassis           
        position_filter();
        
        // update pose_filtered with filter result
        pose_filtered.translation().x() = filter_x_;
        pose_filtered.translation().y() = filter_y_;

        // update filter_odometry with filter result
        filter_odometry.pose.pose.position.x = filter_x_;
        filter_odometry.pose.pose.position.y = filter_y_;
        
        // update last_pose 
        last_pose = pose_filtered;
        
        // pub log info
        pub_log_.publish(log_info_manager_->log_info);

        // publish odom and tf
        auto curr_time = std::chrono::steady_clock::now();                
        auto pub_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - last_pub_time);

        if (pub_elapsed > pub_period){
            pub_filter_odometry_.publish(filter_odometry);
            auto odom_for_tf = filter_odometry;

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            tf::Quaternion q;
            transform.setOrigin(tf::Vector3(odom_for_tf.pose.pose.position.x,
                                            odom_for_tf.pose.pose.position.y,
                                            odom_for_tf.pose.pose.position.z));
            q.setW(odom_for_tf.pose.pose.orientation.w);
            q.setX(odom_for_tf.pose.pose.orientation.x);
            q.setY(odom_for_tf.pose.pose.orientation.y);
            q.setZ(odom_for_tf.pose.pose.orientation.z);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, odom_for_tf.header.stamp, "map", "base_footprint"));
            last_pub_time = std::chrono::steady_clock::now();

        }// pub odom
       
        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (elapsed < filter_period) {
            std::this_thread::sleep_for(filter_period - elapsed);
        }

    } // while (true)
}

void LocalizationModule::pub_module_status_timer(const ros::TimerEvent &event){
    // make msg *************************************************************************
    // fill header
    fairland_msgs::LocalizationModuleStatus status_msg;
    status_msg.header.stamp = ros::Time().now();
    status_msg.header.frame_id = "base_link";

    // fill status_msg.module_status
    // ROS_INFO("set module_status");
    if(running_module_status_ == MODULE_IDLE){
        status_msg.module_status = fairland_msgs::LocalizationModuleStatus::IDLE;
    }else if(running_module_status_ == MODULE_MAPPING || running_module_status_ == MODULE_SEC_MAPPING){
        status_msg.module_status = fairland_msgs::LocalizationModuleStatus::MAPPING;
    }else if(running_module_status_ == MODULE_LOCALIZATION){
        status_msg.module_status = fairland_msgs::LocalizationModuleStatus::LOCALIZATION;
        localization_status_ = slam_->get_l_status();
    }else if(running_module_status_ == MODULE_STARTING_SLAM){
        status_msg.module_status = fairland_msgs::LocalizationModuleStatus::STARTING;
    }else if(running_module_status_ == MODULE_STOPPING_SLAM){
        status_msg.module_status = fairland_msgs::LocalizationModuleStatus::STOPPING;
    }else{
        ROS_ERROR("error running module status: %s", print_ModuleStatus(running_module_status_).c_str());
    }

    // fill status_msg.mapping_status
    // ROS_INFO("set mapping_status");
    if(mapping_status_ == M_INACTIVE){
        status_msg.mapping_status = fairland_msgs::LocalizationModuleStatus::M_INACTIVE;
    }else if(mapping_status_ == M_RELOCALIZING){
        status_msg.mapping_status = fairland_msgs::LocalizationModuleStatus::M_RELOCALIZING;
    }else if(mapping_status_ == M_RELOCALIZE_FAILED){
        status_msg.mapping_status = fairland_msgs::LocalizationModuleStatus::M_RELOCALIZE_FAILED;
    }else if(mapping_status_ == M_CREATING_ELE){
        status_msg.mapping_status = fairland_msgs::LocalizationModuleStatus::M_CREATING_ELE;
    }else if(mapping_status_ == M_STANDBY){
        status_msg.mapping_status = fairland_msgs::LocalizationModuleStatus::M_STANDBY;
    }else{
        ROS_ERROR("error mapping status: %s", print_MappingStatus(mapping_status_).c_str());
    }

    // fill status_msg.localization_status
    // ROS_INFO("set localization_status");
    if(localization_status_ == L_INACTIVE){
        status_msg.localization_status = fairland_msgs::LocalizationModuleStatus::L_INACTIVE;
    }else if(localization_status_ == L_RELOCALIZING){
        status_msg.localization_status = fairland_msgs::LocalizationModuleStatus::L_RELOCALIZING;
    }else if(localization_status_ == L_RELOCALIZE_FAILED){
        status_msg.localization_status = fairland_msgs::LocalizationModuleStatus::L_RELOCALIZE_FAILED;
    }else if(localization_status_ == L_NORMAL){
        status_msg.localization_status = fairland_msgs::LocalizationModuleStatus::L_NORMAL;
    }else if(localization_status_ == L_LOW_ACCURACY){
        status_msg.localization_status = fairland_msgs::LocalizationModuleStatus::L_LOW_ACCURACY;
    }else if(localization_status_ == L_FAILED){
        status_msg.localization_status = fairland_msgs::LocalizationModuleStatus::L_FAILED;
    }else{
        ROS_ERROR("error localization status: %s", print_LocalizationStatus(localization_status_).c_str());
    }

    pub_localization_module_status_.publish(status_msg);
    // ROS_INFO("pub: time: %lf ", status_msg.header.stamp.toSec());


}

// void LocalizationModule::livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg_in){
// void LocalizationModule::livox_pcl_cbk(const fairland_msgs::LivoxCustomMsg::ConstPtr &msg_in){
//     // if (!running_slam_){
//     if (set_module_status_ == MODULE_IDLE || running_module_status_ == MODULE_IDLE){
//         return;
//     }
//     // if(control_status_.reset||offline_mode_)
// 	msg->time_stamp = msg_in->header.stamp.toSec();
//     msg->point_num = msg_in->point_num;
//    // msg->lidar_id;
//   //  msg->rsvd[3];
//     msg->points.resize(msg_in->points.size());
//     for (int i =0; i<msg_in->points.size(); i++){
// 		msg->points[i].x = msg_in->points[i].x;
//         msg->points[i].y = msg_in->points[i].y;
//         msg->points[i].z = msg_in->points[i].z;
//         msg->points[i].reflectivity = msg_in->points[i].reflectivity;
//         msg->points[i].offset_time = msg_in->points[i].offset_time;
// 		msg->points[i].tag = msg_in->points[i].tag;
//         msg->points[i].line = msg_in->points[i].line;
// 	}
// 	slam_ -> livox_pcl_cbk(msg);
    
// }


void LocalizationModule::livox_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &ros_msg){
    // ROS_INFO("livox lidar callback~");

    if (running_module_status_ == MODULE_IDLE || 
        running_module_status_ == MODULE_STARTING_SLAM || 
        running_module_status_ == MODULE_STOPPING_SLAM){
        return;
    }

    // if(control_status_.reset||offline_mode_)
    //    return;

    double t0 = omp_get_wtime();
    std::cout<<"t0: "<<t0<<endl;
    printf("lidar time delay: %lf ms\n", (t0 - ros_msg->header.stamp.toSec())*1000);
    const double thr_x = slam_param_.lidar_preproc.point_filter_distance[0];
    const double thr_y = slam_param_.lidar_preproc.point_filter_distance[1];
    const double thr_z = slam_param_.lidar_preproc.point_filter_distance[2];
	
    int cloud_num = ros_msg->height * ros_msg->width;
    
    ///// MetaData --- header 
    pcl::PCLHeader pcl_header;
    pcl_header.seq = ros_msg->header.seq;
    pcl_header.stamp = ros_msg->header.stamp.toNSec() / 1000ull;
    pcl_header.frame_id = ros_msg->header.frame_id;
    ///// MetaData --- field
    std::vector<pcl::PCLPointField> pcl_fields;
    
    pcl_fields.resize(ros_msg->fields.size());
    std::vector<sensor_msgs::PointField>::const_iterator it = ros_msg->fields.begin();
    int i = 0;
    for(; it != ros_msg->fields.end(); ++it, ++i) {
      pcl_fields[i].name = it->name;
      pcl_fields[i].offset = it->offset;
      pcl_fields[i].datatype = it->datatype;
      pcl_fields[i].count = it->count;
    }
    //// create Mapping
    pcl::MsgFieldMap field_map;
    pcl::createMapping<LvxPointXYZITLO> (pcl_fields, field_map);

    std::shared_ptr<livox_ros::LidarMsg> msg(new livox_ros::LidarMsg);
    // msg->points.resize(cloud_num);

    msg->time_stamp = ros_msg->header.stamp.toSec();

    for (std::uint32_t row = 0; row < ros_msg->height; ++row){
        const std::uint8_t* row_data = &ros_msg->data[row * ros_msg->row_step];
        for (std::uint32_t col = 0; col < ros_msg->width; ++col){
            const std::uint8_t* msg_data = row_data + col * ros_msg->point_step;
            LvxPointXYZITLO temp_point;
            LvxPointXYZITLO* curpt = &temp_point;
            std::uint8_t* curpt_data = reinterpret_cast<std::uint8_t*>(curpt);

            for (const pcl::detail::FieldMapping& mapping : field_map){
                memcpy (curpt_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
            }

            livox_ros::LidarPoint livox_point;
            livox_point.x = curpt->x;
            livox_point.y = curpt->y;
            livox_point.z = curpt->z;
            livox_point.reflectivity = curpt->intensity;
            livox_point.tag = curpt->tag;
            livox_point.line = curpt->line;
            if(abs(livox_point.x) > thr_x || abs(livox_point.y) > thr_y || livox_point.z > thr_z){
               continue;
            }
            // livox_point.offset_time = curpt->offset_time;
            // 新版驱动的 pointcloud2 中， timestamp 为完整时间辍，但单位是纳秒，需要 * 1e-9，将单位统一为 秒
            // livox_point.offset_time = (curpt->timestamp / double(1000000000.0) - msg->time_stamp);
            livox_point.offset_time = (curpt->timestamp  * 1e-9 - msg->time_stamp);
            msg->points.push_back(livox_point);
        }
    }
    // msg->point_num = cloud_num;
    msg->point_num = msg->points.size();
    printf("orig lidar count: %d\n", cloud_num);
    printf("clip lidar count: %d\n", msg->point_num);

    slam_ -> livox_pcl_cbk(msg);
    // printf("lidar callback success\n");

    double t1 = omp_get_wtime();
    printf("lidar-callback, time cost: %f ms \033[0m\n", (t1 - t0)*1000);




    return;

}

void LocalizationModule::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in){
    // ROS_INFO("livox imu callback~");
    // if (!running_slam_){
    // if (set_module_status_ == MODULE_IDLE || running_module_status_ == MODULE_IDLE){
    //     return;
    // }
    // if (running_module_status_ == MODULE_IDLE || 
    //     running_module_status_ == MODULE_STARTING_SLAM || 
    //     running_module_status_ == MODULE_STOPPING_SLAM){
    //     return;
    // }

    // if(control_status_.reset||offline_mode_)
    //    return;

    // transfer IMU : IMU-frame to baselink-frame
    Eigen::Vector3d ang_before(msg_in->angular_velocity.x, msg_in->angular_velocity.y, msg_in->angular_velocity.z);
    Eigen::Vector3d acc_before(msg_in->linear_acceleration.x, msg_in->linear_acceleration.y, msg_in->linear_acceleration.z);
    Eigen::Vector3d ang_after = slam_param_.extrinsic.R_baselink_IMU * ang_before;
    Eigen::Vector3d acc_after = slam_param_.extrinsic.R_baselink_IMU * acc_before;

    std::shared_ptr<livox_ros::ImuMsg> msg(new livox_ros::ImuMsg);
	msg->time_stamp = msg_in->header.stamp.toSec();

	// msg->angular_velocity << msg_in->angular_velocity.x,msg_in->angular_velocity.y,msg_in->angular_velocity.z;
	// msg->linear_acceleration << msg_in->linear_acceleration.x,msg_in->linear_acceleration.y,msg_in->linear_acceleration.z;	
	msg->angular_velocity << ang_after[0],ang_after[1],ang_after[2];	
	msg->linear_acceleration << acc_after[0],acc_after[1],acc_after[2];	

    // if (running_slam_){
    //     slam_ -> imu_cbk(msg);
    // }

    if (running_module_status_ == MODULE_IDLE || 
        running_module_status_ == MODULE_STARTING_SLAM || 
        running_module_status_ == MODULE_STOPPING_SLAM){
        return;
    }else{
        slam_ -> imu_cbk(msg);
        return;
    }

}

void LocalizationModule::chassis_cbk(const fros_hardware_node::chassic_data::ConstPtr &msg_in){
    fros_hardware_node::chassic_data cur_chassis_msg = *msg_in;
    
    double chassis_linear_velocity_ = (cur_chassis_msg.left_front_feedback + cur_chassis_msg.right_front_feedback)/2.0;
    double chassis_angular_velocity_ = (cur_chassis_msg.right_front_feedback - cur_chassis_msg.left_front_feedback)/L_WHEEL;// 这个非常不准，理论上不应该用它，确认实际是否使用
    chassis_linear_velocity_ = cur_chassis_msg.ac_linear_velocity;
    // chassis_angular_velocity_ = cur_chassis_msg.ac_angular_velocity;
    // std::cout << "cal  linear  velocity: "<< chassis_linear_velocity_ <<endl;
    // std::cout << "read linear  velocity: "<< cur_chassis_msg.ac_linear_velocity <<endl;
    // std::cout << "cal  angular velocity: "<< chassis_angular_velocity_ <<endl;
    // std::cout << "read angular velocity: "<< cur_chassis_msg.ac_angular_velocity <<endl;

    static int receive_count=0;

    if(receive_count % 4 == 0){
        std::cout << "read chassis linear velocity: "<< cur_chassis_msg.ac_linear_velocity * 180/PI_M <<" deg" <<endl;
        receive_count = 0;
    }

    receive_count++;

    // 获取时间差
    ros::Time current_time = cur_chassis_msg.header.stamp;
    // current_time = ros::Time::now();
    double time_interval = (current_time - last_chassis_time_).toSec();
    last_chassis_time_ = current_time;

    // // 计算积分位置
    chassis_x_ += chassis_linear_velocity_ * time_interval * cos(lidar_a_);
    chassis_y_ += chassis_linear_velocity_ * time_interval * sin(lidar_a_);
    // chassis_a += angular_velocity * time_interval;
    // chassis_a = angle_norm(chassis_a);
    
}



void LocalizationModule::show_thread()
{
#if 0
    std::cout<<"start show thread "<<endl;
    const int frequency = 5.0; // 频率为1Hz
    const std::chrono::milliseconds period(1000 / frequency);
    lidar_slam::Viewer test_view(localization_mode_);
    while (ros::ok())
    {
        auto start = std::chrono::steady_clock::now();
        if (!control_status_.reset){
            test_view.Start();
            control_status_ = test_view.getControl();
            if (!localization_mode_){
                std::vector<Eigen::Isometry3d> optimized_poses = slam_->get_optimized_path();
                test_view.DrawTrajectory(optimized_poses,Eigen::Vector3f(0,1,0));
                test_view.DrawTrajectory(slam_->get_unoptimized_path(),Eigen::Vector3f(1,0,0));
                map<int, int> loopIndex = slam_->getloopIndex();
                for (auto it = loopIndex.begin(); it != loopIndex.end(); ++it) {
                     test_view.DrawLine(optimized_poses[it->first],optimized_poses[it->second],Eigen::Vector3f(0,0,0));
                }
                
                if (control_status_.showMap)
                    test_view.DrawCloud(slam_->getCurrentMap(),Eigen::Vector3f(0,0,1),1);
                if (control_status_.showLidar)
                   test_view.DrawCloud(slam_->get_odom_cloud(),slam_->getOdomToMap(),Eigen::Vector3f(1,0,0),2);
                if (control_status_.showObstacle)
                    test_view.DrawCloud(slam_->getFilteredObstacleCloud(),slam_->getWheelInMap(),Eigen::Vector3f(0,1,0),2.0);
                test_view.DrawPose(slam_->getWheelInMap());
            }
            else{
                if (control_status_.showMap)
                   test_view.DrawCloud(slam_->getLoadMapPoints(),Eigen::Vector3f(0,0,1),1.0);
                if (control_status_.showLidar)
                   test_view.DrawCloud(slam_->get_lidar_cloud(),slam_->getLidarInMap(),Eigen::Vector3f(1,0,0),2.0);
                if (control_status_.showObstacle)
                    test_view.DrawCloud(slam_->getFilteredObstacleCloud(),slam_->getWheelInMap(),Eigen::Vector3f(0,1,0),2.0);
                if (slam_->isGloalLocalizationSuccess())
                    test_view.DrawPose(slam_->getWheelInMap());
                test_view.DrawTrajectory(slam_->get_unoptimized_path(),Eigen::Vector3f(1,0,0));
            }

            if (control_status_.saveMap && !localization_mode_)
                slam_ -> save_map(curr_dir_+std::string("/map/"),0.1, 0, 0);
            test_view.Finish();  
        }
        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        if (elapsed < period)
        {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
#endif
}


// 这里其实还包含了 update path
void LocalizationModule::publish_unoptimized_path(const std::deque<Eigen::Isometry3d> path, ros::Publisher pubUnoptimizedPath)
{
	geometry_msgs::PoseStamped msg;

	int size = path.size();
	unoptimized_path_msg.poses.clear();
    unoptimized_path_msg.header.stamp = ros::Time().now();
    unoptimized_path_msg.header.frame_id = "odom";
	for (int i = 0 ; i <path.size();i++){
		msg.header.stamp = ros::Time().now();
		msg.header.frame_id = "odom";
		msg.pose.position.x = path[i].translation().x();
		msg.pose.position.y = path[i].translation().y();
		msg.pose.position.z = path[i].translation().z();
		/*Eigen::Quaterniond quaternion = path[i].rotation();
		msg.pose.orientation.x = quaternion.x();
		msg.pose.orientation.y = quaternion.y();
		msg.pose.orientation.z = quaternion.z();
		msg.pose.orientation.w = quaternion.w();*/
		unoptimized_path_msg.poses.push_back(msg);
	}
    pubUnoptimizedPath.publish(unoptimized_path_msg);
}

void LocalizationModule::publish_optimized_path(const std::vector<Eigen::Isometry3d> path,std::string frame, ros::Publisher pubOptimizedPath)
{
	geometry_msgs::PoseStamped msg;
    optimized_path_msg.poses.clear();
    optimized_path_msg.header.stamp = ros::Time().now();
    optimized_path_msg.header.frame_id = frame;
	int size = path.size();
	for (int i = 0 ; i <path.size();i++){
		msg.header.stamp = ros::Time().now();
		msg.header.frame_id = frame;
		msg.pose.position.x = path[i].translation().x();
		msg.pose.position.y = path[i].translation().y();
		msg.pose.position.z = path[i].translation().z();
		/*Eigen::Quaterniond quaternion = path[i].rotation();
		msg.pose.orientation.x = quaternion.x();
		msg.pose.orientation.y = quaternion.y();localization_mode_
		msg.pose.orientation.z = quaternion.z();
		msg.pose.orientation.w = quaternion.w();*/
		optimized_path_msg.poses.push_back(msg);
	}
    pubOptimizedPath.publish(optimized_path_msg);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////--------------------------- Init Module -----------------------------////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////


bool LocalizationModule::init_module_by_set_status(ModuleStatus set_status){
    set_module_status_ = set_status;

    if(set_module_status_ == MODULE_IDLE){
        // ROS_INFO("init module status: %s", print_ModuleStatus(set_module_status_).c_str());
    }else if (set_module_status_ == MODULE_MAPPING){
        if(start_mapping(set_module_status_)){
            running_module_status_ = set_module_status_;
            mapping_status_ = M_STANDBY;
        }else{
            set_module_status_ = running_module_status_;
        }
    }else if (set_module_status_ == MODULE_SEC_MAPPING){
        // TODO
        int map_id = 0;/////////////// TODO
        if(start_second_mapping(set_module_status_, map_id)){
            running_module_status_ = MODULE_SEC_MAPPING;
            mapping_status_ = M_STANDBY;
        }else{
            set_module_status_ = running_module_status_;
            release_slam_obj();
            ROS_WARN("slam obj destroyed!");
        }
    }else if (set_module_status_ == MODULE_LOCALIZATION){
        // TODO
        int map_id = 0;/////////////// TODO
        if(start_localization(set_module_status_, map_id)){
            running_module_status_ = MODULE_LOCALIZATION;
            // localization_status_ = L_RELOCALIZING;
        }else{
            set_module_status_ = running_module_status_;
        }
    }

    // ROS_INFO("init module status: %s", print_ModuleStatus(running_module_status_).c_str());

    return true;
}

}// namespace localization_module

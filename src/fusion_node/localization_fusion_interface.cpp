#include "localization_fusion_interface.h"
#include "fusion_node/common/type_conversion.h"


namespace localization_module{

LocalizationFusion::LocalizationFusion(){

    if (!load_params()){
        ROS_ERROR("Load localization-fusion param failed!");
    }else {
        ROS_INFO("\033[1;32mLoad localization-fusion param successfully!\033[0m");
    }

    ekf_fusion_ptr_= std::make_shared<EkfLocalizationFusion>();

    if(!create_ROS_IO()){
        ROS_ERROR("Create ROS-IO failed!");
    }else {
        ROS_INFO("Create ROS-IO successfully!");
    }

    init_chassis_imu_slam_odom_stamp();

    ros::spin();
}


LocalizationFusion::~LocalizationFusion(){
    ros::shutdown();
    
}


bool LocalizationFusion::create_ROS_IO(){

    // sub_imu_ = nh_.subscribe<sensor_msgs::Imu>(sub_imu_topic_, 2000, &LocalizationFusion::imu_msg_callback, this);
    // sub_chassis_ = nh_.subscribe<fairland_msgs::chassic_data>(sub_chassis_topic_, 100, &LocalizationFusion::chassis_msg_callback, this);
    // sub_slam_odom_ = nh_.subscribe<nav_msgs::Odometry>(sub_slam_odom_topic_, 100, &LocalizationFusion::slam_odometry_callback, this);

    sub_imu_ = nh_.subscribe(sub_imu_topic_, 2000, &LocalizationFusion::imu_msg_callback, this);
    sub_chassis_ = nh_.subscribe(sub_chassis_topic_, 100, &LocalizationFusion::chassis_msg_callback, this);
    sub_slam_odom_ = nh_.subscribe(sub_slam_odom_topic_, 100, &LocalizationFusion::slam_odometry_callback, this);

	pub_fusion_odom_ = nh_.advertise<nav_msgs::Odometry>(pub_localization_topic_, 100);

    
    return true;
}


void LocalizationFusion::chassis_msg_callback(const fairland_msgs::chassic_data::ConstPtr &chassis_msg_in){
    ROS_INFO_STREAM_ONCE(YELLOW<<"Received chassis msg"<<RESET);
    std::lock_guard<std::mutex> lock(mutex_);
    is_chassis_rcv_ = true;
    chassis_msg_ = *chassis_msg_in;
    // ROS_INFO_STREAM("chassis-vel: "<<chassis_msg_.ac_linear_velocity );
}

void LocalizationFusion::imu_msg_callback(const sensor_msgs::Imu::ConstPtr& imu_msg_in){
    ROS_INFO_STREAM_ONCE(YELLOW<<"Received imu msg"<<RESET);
    std::lock_guard<std::mutex> lock(mutex_);
    is_imu_rcv_ = true;
    imu_msg_ = *imu_msg_in;
}

void LocalizationFusion::slam_odometry_callback(const nav_msgs::Odometry::ConstPtr& slam_odometry_in){
    ROS_INFO_STREAM_ONCE(YELLOW<<"Received slam odometry"<<RESET);
    std::lock_guard<std::mutex> lock(mutex_);
    slam_odom_msg_ = *slam_odometry_in;

    if (!is_imu_rcv_){
        ROS_WARN_STREAM_ONCE(YELLOW<<"IMU data not received yet "<<RESET);
        return;
    }

    if(ekf_use_chassis_ && !is_chassis_rcv_){
        ROS_WARN_STREAM_ONCE(YELLOW<<"Chassis data not received yet "<<RESET);
        return;
    }

    ROS_INFO_STREAM("chassis_msg_.header.stamp: "<<chassis_msg_.header.stamp.toSec());
    ROS_INFO_STREAM("chassis_msg_.ac_linear_velocity: "<<chassis_msg_.ac_linear_velocity);

    check_slam_odometry(slam_odom_msg_);
    // TODO: check chassis time

    compose_status(slam_odom_msg_, imu_msg_, chassis_msg_, &status_origin_);
    compose_status(slam_odom_msg_, imu_msg_, chassis_msg_, &status_tmp_);

    // if (lf_need_init_ == true && is_chassis_rcv_ && is_imu_rcv_) { // 前面已经进行过 is_chassis_rcv_ && is_imu_rcv_ 的判断
    if (lf_need_init_ == true ) {
        ekf_fusion_ptr_->init(status_tmp_);
        ROS_INFO_STREAM(YELLOW<<"localization fusion init -----------------"<<RESET);
        lf_need_init_ = false;
    }

    if (ekf_fusion_ptr_->is_init()) {
        ROS_INFO_STREAM(GREEN<<"localization fusion start ----------------"<<RESET);
        ekf_fusion_ptr_->localization_fusion_core(status_tmp_, &status_lf_);
        ROS_INFO_STREAM(GREEN<<"localization fusion end ------------------"<<RESET);
        status_tmp_ = status_lf_;

        ROS_INFO("slam-  x: %.4f --- y:  %.4f --- yaw:  %.6f", status_lf_.slam_pose.position.x, status_lf_.slam_pose.position.y, status_lf_.slam_pose.orientation.z);
        ROS_INFO("fusion-x: %.4f --- y:  %.4f --- yaw:  %.6f", status_lf_.fusion_pose.position.x, status_lf_.fusion_pose.position.y, status_lf_.fusion_pose.orientation.z);
        ROS_INFO_STREAM(GREEN<<"------------------------------------------"<<RESET);
    }


    if(!ekf_use_chassis_){
        double dt = status_tmp_.header.stamp.toSec() - status_.header.stamp.toSec();
        double vx = (status_tmp_.fusion_pose.position.x - status_.fusion_pose.position.x)/dt;
        double vy = (status_tmp_.fusion_pose.position.y - status_.fusion_pose.position.y)/dt;

        double cal_yaw = (status_tmp_.fusion_pose.orientation.z + status_.fusion_pose.orientation.z) * 0.5;
        slam_speed_ = vx * std::cos(cal_yaw) + vy * std::sin(cal_yaw);
    }
    
    // update & publish
    status_ = status_tmp_;
    pub_localiztion();
}

void LocalizationFusion::pub_localiztion(){
    nav_msgs::Odometry fusion_odom;
    fusion_odom.header = status_.header;
    fusion_odom.header.seq = seq_count_ ++;

    Eigen::Isometry3d T_base2map = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond eigen_quat = localization_module::common::Quaternion::geo_quat_2_eigen_quat(status_.fusion_pose.orientation);
    T_base2map.pretranslate(Eigen::Vector3d(status_.fusion_pose.position.x, status_.fusion_pose.position.y, status_.fusion_pose.position.z));
    T_base2map.rotate(eigen_quat); // 应用四元数的旋转

    Eigen::Isometry3d T_lidar2map = Eigen::Isometry3d::Identity();
    T_lidar2map = T_base2map * T_lidar2baselink_;

    fusion_odom.pose.pose.position.x = T_lidar2map.translation().x();
    fusion_odom.pose.pose.position.y = T_lidar2map.translation().y();
    fusion_odom.pose.pose.position.z = T_lidar2map.translation().z();

    Eigen::Quaterniond e_quat2(T_lidar2map.rotation());
    geometry_msgs::Quaternion geo_quat = localization_module::common::Quaternion::eigen_quat_2_geo_quat(e_quat2);
    fusion_odom.pose.pose.orientation = geo_quat;

    pub_fusion_odom_.publish(fusion_odom);
    
    
    // send tf
    static tf::TransformBroadcaster br;
    tf::Transform transform_to_send = localization_module::common::conversions::odom_to_transform(fusion_odom);
    br.sendTransform(tf::StampedTransform(transform_to_send, fusion_odom.header.stamp, "map", "base_footprint"));

    

}

void LocalizationFusion::compose_status(nav_msgs::Odometry slam_odom, sensor_msgs::Imu imu_msg, fairland_msgs::chassic_data chassis_msg, 
                                        fairland_msgs::LocalizationPoseData* status_msg){
    if(!ekf_use_chassis_){
        chassis_msg.ac_linear_velocity = slam_speed_;
    }

    Eigen::Isometry3d T_baselink2map = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_lidar2map = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond eigen_quat = localization_module::common::Quaternion::geo_quat_2_eigen_quat(slam_odom.pose.pose.orientation);
    T_lidar2map.pretranslate(Eigen::Vector3d(slam_odom.pose.pose.position.x, slam_odom.pose.pose.position.y, slam_odom.pose.pose.position.z));
    T_lidar2map.rotate(eigen_quat); // 应用四元数的旋转

    T_baselink2map =  T_lidar2map * T_baselink2lidar_;
    Eigen::Quaterniond e_quat2(T_baselink2map.rotation());
    geometry_msgs::Quaternion geo_quat = localization_module::common::Quaternion::eigen_quat_2_geo_quat(e_quat2);

    status_msg->header = slam_odom.header;
    // status_msg->slam_pose = slam_odom.pose.pose;
    status_msg->slam_pose.orientation = geo_quat;
    status_msg->slam_pose.position.x = T_baselink2map.translation().x();
    status_msg->slam_pose.position.y = T_baselink2map.translation().y();
    status_msg->slam_pose.position.z = T_baselink2map.translation().z();

    status_msg->fusion_pose = status_msg->slam_pose;
    status_msg->chassis_status = chassis_msg;
    status_msg->linear_acceleration = imu_msg.linear_acceleration;
    status_msg->angular_velocity = imu_msg.angular_velocity;

}

void LocalizationFusion::check_slam_odometry(nav_msgs::Odometry slam_odom){
    double curr_slam_odom_time = slam_odom.header.stamp.toSec();

    double slam_dtime = curr_slam_odom_time - last_slam_odom_time_;
    ROS_INFO_STREAM(YELLOW<<"odom_dtime: "<<slam_dtime<<RESET);
    if(slam_dtime < 0 || std::abs(slam_dtime) > 3){
        lf_need_init_= true;
        ekf_fusion_ptr_->reset();
        // exit(1);
        ROS_INFO_STREAM(RED<<"localization fusion need reset!"<<RESET);
    }

    last_slam_odom_time_ = curr_slam_odom_time;
}

/////////////////////////////////////////////////////////////////////////////////////////////////

bool LocalizationFusion::load_params(){
    LocalizationFusionParamsManager* params_manager = LocalizationFusionParamsManager::Instance();
    const LocalizationFusionParams* lf_params = params_manager->get_localization_fusion_params();
    sub_imu_topic_ = lf_params->topic_params.sub_imu_topic;
    sub_chassis_topic_ = lf_params->topic_params.sub_chassis_topic;
    sub_slam_odom_topic_ = lf_params->topic_params.sub_slam_odom_topic;
    pub_localization_topic_ = lf_params->topic_params.pub_localization_topic;
    std::vector<double> baselink_in_lidar = lf_params->baselink_in_lidar;

    double roll  = localization_module::common::Quaternion::deg2rad(baselink_in_lidar[3]);
    double pitch = localization_module::common::Quaternion::deg2rad(baselink_in_lidar[4]);
    double yaw   = localization_module::common::Quaternion::deg2rad(baselink_in_lidar[5]);
    std::vector<double> rad_euler_zyx = {roll, pitch, yaw};
    Eigen::Quaterniond eigen_quat = localization_module::common::Quaternion::get_eigen_quaternion(rad_euler_zyx);

    // ROS_INFO_STREAM("baselink_in_lidar: " );
    // ROS_INFO_STREAM("x    : " << baselink_in_lidar[0]);
    // ROS_INFO_STREAM("y    : " << baselink_in_lidar[1]);
    // ROS_INFO_STREAM("z    : " << baselink_in_lidar[2]);
    // ROS_INFO_STREAM("roll : " << baselink_in_lidar[3]);
    // ROS_INFO_STREAM("pitch: " << baselink_in_lidar[4]);
    // ROS_INFO_STREAM("yaw  : " << baselink_in_lidar[5]);

    T_baselink2lidar_ = Eigen::Isometry3d::Identity();
    T_lidar2baselink_ = Eigen::Isometry3d::Identity();

    T_baselink2lidar_.pretranslate(Eigen::Vector3d(baselink_in_lidar[0], baselink_in_lidar[1], baselink_in_lidar[2]));
    T_baselink2lidar_.rotate(eigen_quat); // 应用四元数的旋转
    // std::cout << "T_baselink2lidar_: " <<std::endl;
    // std::cout << T_baselink2lidar_.translation()  <<std::endl;
    // std::cout << T_baselink2lidar_.rotation()  <<std::endl;

    T_lidar2baselink_ = T_baselink2lidar_.inverse();

    time_lost_thr_ = lf_params->time_lost_thr;

    ekf_use_chassis_ = lf_params->ekf_use_chassis;

    if(!ekf_use_chassis_){
        // is_chassis_rcv_ = true;
    }
    
    return true;
}

void LocalizationFusion::init_chassis_imu_slam_odom_stamp(){
    slam_odom_msg_.header.stamp = ros::Time::now();
    chassis_msg_.header.stamp = ros::Time::now();
    imu_msg_.header.stamp = ros::Time::now();

    ROS_INFO_STREAM("time-now: "<<imu_msg_.header.stamp.toSec());

    slam_speed_ = 0.0;
}


} // namespace localization_module
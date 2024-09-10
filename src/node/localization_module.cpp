#include <ros/ros.h>
#include "node/localization_module.h"

namespace localization_module {
LocalizationModule::LocalizationModule(/*const std::string work_path,*/ ModuleStatus init_status){
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


    ROS_INFO("***************************************************");
}

LocalizationModule::~LocalizationModule(){

}

bool LocalizationModule::create_ROS_IO(){
    // subscriber ********************************************************************
	// ros::Subscriber sub_pcl = nh_.subscribe<livox_ros_driver2::CustomMsg>("/livox/lidar", 200000, &LocalizationModule::livox_pcl_cbk, this);
    sub_pointcloud2_ = nh_.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 10, &LocalizationModule::livox_pcl_cbk, this);
    sub_imu_ = nh_.subscribe<sensor_msgs::Imu>("/livox/imu", 200000, &LocalizationModule::imu_cbk, this);

    sub_mapping_ctrl_ = nh_.subscribe(slam_param_.common.sub_topic_ctrl_cmd, 3 ,&LocalizationModule::localization_module_ctrl_cbk, this);
    
    // timer dealt ********************************************************************
    // 建图主要流程，timer 时间间隔需要调整，10hz? 100hz? 200hz?
    timer_slam_ = nh_.createTimer(ros::Duration(0.05), &LocalizationModule::slam_dealt_timer, this);
    timer_pub_module_status_ = nh_.createTimer(ros::Duration(0.05), &LocalizationModule::pub_module_status_timer, this);
    
    // publish ************************************************************************
    pub_localization_module_status_ = nh_.advertise<fairland_msgs::LocalizationModuleStatus>(slam_param_.common.pub_topic_module_status, 100); 

    // both 建图 & 定位
	pubLidarInMap = nh_.advertise<nav_msgs::Odometry>("/Odometry_lidar_in_map", 100000);

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
    if (show_load_map_==0 && running_module_status_==MODULE_LOCALIZATION && (slam_->getLoadMap()) && (slam_->getLoadMap())->points.size() > 0){
        // ROS_INFO("load map");
        sleep(1);
        sensor_msgs::PointCloud2 loadMap;
        pcl::toROSMsg(*(slam_->getLoadMap()), loadMap); 
        loadMap.header.stamp = ros::Time::now();
        loadMap.header.frame_id = "map";
        pubLoadMap.publish(loadMap);
        show_keyframe(slam_->getLoadKeyFrame(), pubKeyframePose);
        ROS_INFO("load map success");
        show_load_map_ ++;
    }

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
//     //    return;
// 	std::shared_ptr<livox_ros::LidarMsg> msg(new livox_ros::LidarMsg);
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

    // if (running_slam_){
    //     slam_ -> livox_pcl_cbk(msg);
    // }

    if (running_module_status_ == MODULE_IDLE || 
        running_module_status_ == MODULE_STARTING_SLAM || 
        running_module_status_ == MODULE_STOPPING_SLAM){
        return;
    }else{
        slam_ -> livox_pcl_cbk(msg);
        // ROS_INFO("lidar callback success");
        return;
    }

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

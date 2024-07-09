#include <ros/ros.h>
#include "./localization_module.h"
#include "localization_module.h"

namespace localization_module {
LocalizationModule::LocalizationModule(const std::string work_path){
    curr_dir_ = work_path;
    ROS_INFO("Current directory: %s", curr_dir_.c_str());

    load_params();

    sub_mapping_ctrl_ = nh_.subscribe("/mapping_manager_cmd", 3 ,&LocalizationModule::mapping_ctrl_cbk, this);
    
    // 建图主要流程，timer 时间间隔需要调整，10hz? 100hz? 200hz?
    timer_slam_ = nh_.createTimer(ros::Duration(0.1), &LocalizationModule::slam_dealt_timer, this);
    
    if (!show_rviz_)// this param load from launch file
        show_thread_ = std::thread(&LocalizationModule::show_thread, this);

    ROS_INFO("show_thread_");
    // TODO
    std::thread load_data_thread;
    if (offline_mode_){
        // 读取文件夹中的文件名
    }

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
  //  image_pub = nh_.advertise<sensor_msgs::Image>("fisheye_image", 1);


    // 目前没用，备用
	// ros::Subscriber sub_pcl = nh_.subscribe<livox_ros_driver2::CustomMsg>("/livox/lidar", 200000, &LocalizationModule::livox_pcl_cbk, this);
    sub_pointcloud2_ = nh_.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 200000, &LocalizationModule::livox_pcl_cbk, this);
    sub_imu_ = nh_.subscribe<sensor_msgs::Imu>("/livox/imu", 200000, &LocalizationModule::imu_cbk, this);
    ros::Subscriber sub_command  = nh_.subscribe("/command" ,200000,&LocalizationModule::command_cbk, this);

    ROS_INFO("module start ");
}

LocalizationModule::~LocalizationModule(){

}

void LocalizationModule::mapping_ctrl_cbk(const std_msgs::UInt32 &msg_in){
    /** msg_in
     *      1000: 开始建图； ////
     *      2000: 设置起点 ///
     *      3000: 创建地图元素过程中，清除当前元素（当前元素还未完成创建）
     *      4000: 设置终点 ///  1001: 开始建图 with ID=1；
     *      5000: 重定位，并开始继续建图
     *      9000: 退出建图 ////
     * 
     *      6000: 重定位，并开始定位
     *      7000: 退出定位
     *      
     * to be continued
     */  
    //// topic-name: "/mapping_manager_cmd"

    auto msg = msg_in;
    int ctrl_type = msg.data/1000 * 1000;
    if (ctrl_type == 1000){//开始建图, CREATE_FIRST_ZONE in mapmanager
        localization_mode_ = 0;
        start_mapping(localization_mode_);
        running_slam_ = true;
        slam_status_ = MAPPING_STARTED;
    }else if(ctrl_type == 2000){//设置起点
        // if(slam_status_ == MAPPING_STARTED){
        if(running_slam_ && slam_status_ >= MAPPING_STARTED){
            mark_start_point();
        }else{
            ROS_INFO("please make sure: slam_status_ == MAPPING_STARTED!");
        }
        slam_status_ = STARTPOINT_SET;
    }else if(ctrl_type == 3000){// 创建地图元素过程中，清除当前元素
        if(slam_status_ == STARTPOINT_SET){
            clear_curr_element();
        }
        slam_status_ = MAPPING_STARTED;
    }else if(ctrl_type == 4000){//设置终点
        if(slam_status_ == STARTPOINT_SET){
            int ele_id = msg.data % 1000;
            mark_end_point(ele_id);
        }else{
            ROS_INFO("please set start-point first!");
        }
        slam_status_ = ENDPOINT_SET;
    }else if(ctrl_type == 5000){// 重定位，并开始建图，/// TODO/////////////////////////////////////
        relocalize_and_mapping();
    }else if(ctrl_type == 9000){// 是否任何状态下均可退出建图 ？？？？
        stop_mapping();
        running_slam_ = false;
        slam_status_ = MODULE_INACTIVE;
    }else if(ctrl_type == 6000){// 开始定位，（先重定位，再定位）
        localization_mode_ = 1;
        int map_id = msg.data % 1000;
        relocalize_and_localization(localization_mode_, map_id);
        running_slam_ = true;
        slam_status_ = MAPPING_STARTED;
    }else if(ctrl_type == 7000){// 结束定位
        stop_localization();
    }else{
        ROS_INFO("mapping ctrl msg: %u invalid!", msg.data);
    }

    // delete map element , 不用操作

}

void LocalizationModule::start_mapping(bool module_mode){
    // TODO: 启动建图前，需要确定哪些参数？？
    // localization_mode_;
    // offline_mode_;
    // 初始位姿？
    ROS_INFO("create lidar_slam, slam_mode: start mapping");
	// slam_ = std::make_unique<lidar_slam::LidarSlam>(curr_dir_+std::string("/lib/"),module_mode,offline_mode_);	
	slam_ = std::make_unique<lidar_slam::LidarSlam>(curr_dir_+std::string("/"),module_mode,offline_mode_);	
    ROS_INFO("create lidar_slam success");
    start_index_ = -1;
    end_index_ = -1;
}

void LocalizationModule::mark_start_point(){
    // TODO：标记起点的 POSE
    start_index_ = slam_->get_curr_pose_index();
}

void LocalizationModule::mark_end_point(int save_id){
    // TODO：当前地图元素完成，判断是否保存点云地图
    end_index_ = slam_->get_curr_pose_index();
    // std::string pcd_path = curr_dir_  +std::string("/lib/map/") + std::to_string(save_id);
    std::string pcd_path = curr_dir_ + std::string("/map/") + std::to_string(save_id)+std::string("/");
    if (save_id > 0){
        slam_->save_map(pcd_path, 0.1, start_index_, end_index_);
        return;
    }
}

void LocalizationModule::clear_curr_element(){
    // TODO：清除当前正在创建的元素，清除 标记的Pose
    start_index_ = -1;
    end_index_ = -1;
}

void LocalizationModule::relocalize_and_mapping(){
    // TODO：重定位
}

void LocalizationModule::stop_mapping(){
    ROS_INFO("start stop mapping");
    control_status_.reset = true;
    running_slam_ = false;
    sleep(1);
    // TODO：退出

    // // used when test 
    // const std::string work_path = curr_dir_+std::string("/");
    // // slam_->reset(work_path,localization_mode_,offline_mode_);
    // bool flag = (slam_==nullptr);
    // cout<<"if slam_==nullptr: "<< flag <<endl;

    // lidar_slam::LidarSlam *temp_slam = slam_.release();
    // ROS_INFO("release successfully");
    // flag = (slam_==nullptr);
    // cout<<"if slam_==nullptr: "<< flag <<endl;

    // delete temp_slam;
    // temp_slam=nullptr;
    // ROS_INFO("delete successfully");
    // flag = (slam_==nullptr);
    // cout<<"if slam_==nullptr: "<< flag <<endl;

    lidar_slam::LidarSlam *temp_slam = slam_.release();
    delete temp_slam;
    temp_slam = nullptr;
    ROS_INFO("mapping stopped !");
}

void LocalizationModule::relocalize_and_localization(bool module_mode, int map_id){
    // 
    ROS_INFO("create lidar_slam, slam_mode: relocalize_and_localization");
	// slam_ = std::make_unique<lidar_slam::LidarSlam>(curr_dir_+std::string("/lib/"),module_mode,offline_mode_);	
	slam_ = std::make_unique<lidar_slam::LidarSlam>(curr_dir_+std::string("/"),module_mode,offline_mode_);	
    ROS_INFO("create lidar_slam success");
    start_index_ = -1;
    end_index_ = -1;

    std::string pcd_path = curr_dir_+std::string("/map/")+std::to_string(map_id)+std::string("/");
    slam_ -> load_map(pcd_path);

    running_slam_ = true;
}


void LocalizationModule::stop_localization(){
    // 
    ROS_INFO("start stop mapping");
    control_status_.reset = true;
    running_slam_ = false;
    sleep(1);

    lidar_slam::LidarSlam *temp_slam = slam_.release();
    delete temp_slam;
    temp_slam = nullptr;
    ROS_INFO("localization stopped !");
}


void LocalizationModule::slam_dealt_timer(const ros::TimerEvent &event){
    // SLAM 主要流程， 对应于原来的 while (ros::ok()){...}
    // ROS_INFO("slam_dealt_timer");

    if (!running_slam_){
        ROS_INFO("status: not running slam");
        return;
    }


    if (control_status_.reset){
        sleep(1);
        slam_.reset();
        sleep(1);
        localization_mode_ = control_status_.localizationMode;
        // slam_ = std::make_unique<lidar_slam::LidarSlam>(curr_dir_+std::string("/lib/"),localization_mode_,offline_mode_);
        slam_ = std::make_unique<lidar_slam::LidarSlam>(curr_dir_+std::string("/"),localization_mode_,offline_mode_);
        show_load_map_ = 0;
        control_status_.reset = false;

    }

    if (show_load_map_==0 && localization_mode_ && (slam_->getLoadMap())->points.size() > 0){
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
    bool running_slam_flag = slam_->run();

    if (running_slam_flag && show_rviz_){
        if (!localization_mode_ || slam_->isGloalLocalizationSuccess()){
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
    if(!localization_mode_){
    // pub_rgb_map(slam->getCurrentRGBMap());
    }
    if (show_rviz_){
        publish_static_transform(slam_->getWheelInLidar());
        publish_odometry(slam_->getLidarInOdom(), pubOdomAftMapped);
    }
}

void LocalizationModule::command_cbk(const std_msgs::Int32 &msg_in){
    switch(msg_in.data){
        case 0:
            printf("save_map\n");
            slam_ -> save_map(curr_dir_+std::string("/map/"),0.1,0,0);
            // slam_ -> save_map(curr_dir_+std::string("/lib/map/"),0.1,0,0);
            break;
        case 1:
            printf("load map\n");
            slam_ -> load_map(curr_dir_+std::string("/map/"));
            // slam_ -> load_map(curr_dir_+std::string("/lib/map/"));
            break;
        default:
           break;
    }

}

// void LocalizationModule::livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg_in){
void LocalizationModule::livox_pcl_cbk(const fairland_msgs::LivoxCustomMsg::ConstPtr &msg_in){
    if (!running_slam_){
        return;
    }
    if(control_status_.reset||offline_mode_)
       return;
	std::shared_ptr<livox_ros::LidarMsg> msg(new livox_ros::LidarMsg);
	msg->time_stamp = msg_in->header.stamp.toSec();
    msg->point_num = msg_in->point_num;
   // msg->lidar_id;
  //  msg->rsvd[3];
    msg->points.resize(msg_in->points.size());
    for (int i =0; i<msg_in->points.size(); i++){
		msg->points[i].x = msg_in->points[i].x;
        msg->points[i].y = msg_in->points[i].y;
        msg->points[i].z = msg_in->points[i].z;
        msg->points[i].reflectivity = msg_in->points[i].reflectivity;
        msg->points[i].offset_time = msg_in->points[i].offset_time;
		msg->points[i].tag = msg_in->points[i].tag;
        msg->points[i].line = msg_in->points[i].line;
	}
	slam_ -> livox_pcl_cbk(msg);
    
}


void LocalizationModule::livox_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &ros_msg){
    // ROS_INFO("livox lidar callback~");
    if (!running_slam_){
        return;
    }

    if(control_status_.reset||offline_mode_)
       return;
	
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
            livox_point.offset_time = curpt->offset_time;
            msg->points.push_back(livox_point);
        }
    }

    msg->time_stamp = ros_msg->header.stamp.toSec();
    msg->point_num = cloud_num;


    if (running_slam_){
        slam_ -> livox_pcl_cbk(msg);
    }
}

void LocalizationModule::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in){
    // ROS_INFO("livox imu callback~");
    if (!running_slam_){
        return;
    }
    if(control_status_.reset||offline_mode_)
       return;

    std::shared_ptr<livox_ros::ImuMsg> msg(new livox_ros::ImuMsg);
	msg->time_stamp = msg_in->header.stamp.toSec();
	msg->angular_velocity << msg_in->angular_velocity.x,msg_in->angular_velocity.y,msg_in->angular_velocity.z;
	msg->linear_acceleration << msg_in->linear_acceleration.x,msg_in->linear_acceleration.y,msg_in->linear_acceleration.z;	

    if (running_slam_){
        slam_ -> imu_cbk(msg);
    }
}


void LocalizationModule::show_thread()
{
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
		msg.pose.orientation.y = quaternion.y();
		msg.pose.orientation.z = quaternion.z();
		msg.pose.orientation.w = quaternion.w();*/
		optimized_path_msg.poses.push_back(msg);
	}
    pubOptimizedPath.publish(optimized_path_msg);
}


void LocalizationModule::load_params(){
    nh_.param<bool>("localization_mode", localization_mode_, false);
    nh_.param<bool>("just_show_mode", just_show_mode_, false);
    nh_.param<bool>("offline_mode_", offline_mode_, false);
    nh_.param<bool>("show_rviz", show_rviz_, false);
    nh_.param<bool>("fast", fast_mode_, false);
    nh_.param<string>("log_folder", log_folder_, " ");

}

}// namespace localization_module
#include "node/localization_module.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>

#define L_WHEEL 0.385f

namespace localization_module {

std::atomic<ModuleStatus> LocalizationModule::running_module_status_(ModuleStatus::MODULE_IDLE);
std::atomic<double>		  LocalizationModule::livox_cbk_update_time_(0.0);
LocalizationModule::LocalizationModule(rclcpp::Node::SharedPtr node, ModuleStatus init_status)
	: node_(node),
	  br_(node_),
	  tf_buffer_(node_->get_clock()),
	  tf_listener_(tf_buffer_)
// LocalizationModule::LocalizationModule(/*const std::string work_path,*/ ModuleStatus init_status)
{
	//**************************** 加载参数 ********************************
	if (!load_lidar_slam_param()) {
		// ROS_ERROR_STREAM(BOLDRED << "Load lidar-slam param failed!" << RESET);
		RCLCPP_INFO(node_->get_logger(), "Load lidar-slam param failed!");

	} else {
		// ROS_INFO("Load lidar-slam param successfully!");
		RCLCPP_INFO(node_->get_logger(), "Load lidar-slam param successfully!");
	}
	std::cout << "Load lidar-slam param successfully" << std::endl;
	//**************************** CPU 绑定 *********************************
	CPU_ZERO(&cpu_mask_); // 初始化 CPU 亲和性集合，将其设置为零
	for (int i = 0; i < slam_param_.common.cpu_id.size(); i++) {
		CPU_SET(slam_param_.common.cpu_id[i], &cpu_mask_); // 将线程绑定到 cpu_id 核心
		// ROS_INFO("\033[1;32mset cpu: %d\033[0m", slam_param_.common.cpu_id[i]);
		// RCLCPP_INFO(node_->get_logger(),"\033[1;32mset cpu: %d\033[0m", slam_param_.common.cpu_id[i]);
		RCLCPP_INFO(node_->get_logger(), "\033[1;32mset cpu: %d\033[0m",
					static_cast<int>(slam_param_.common.cpu_id[i]));
	}
	//--------------------------- CPU 绑定 end ------------------------------
	std::cout << "Load lidar-slam param successfully11" << std::endl;
	//************** 初始化一些成员变量, after param load  ********************
	module_member_init();
	std::cout << "Load lidar-slam param successfully22" << std::endl;
	//**************************** 创建 ROS IO ******************************
	if (!create_ROS_IO()) {
		// ROS_ERROR_STREAM(RED << "Create ROS-IO failed!" << RESET);
		RCLCPP_INFO(node_->get_logger(), "Create ROS-IO failed!");
	} else {
		// ROS_INFO("Create ROS-IO successfully!");
		RCLCPP_INFO(node_->get_logger(), "Create ROS-IO successfully!");
	}
	std::cout << "Load lidar-slam param successfully33" << std::endl;
	//**************************** 根据设置参数初始化 module status ******************************
	// ROS_INFO("***************************************************");
	RCLCPP_INFO(node_->get_logger(), "***************************************************");
	if (!init_module_by_set_status(init_status)) {
		RCLCPP_INFO(node_->get_logger(), "Try to init module with status: %s, but failed",
					print_ModuleStatus(init_status).c_str());
	} else {
		RCLCPP_INFO(node_->get_logger(), "Localization Module Start with status:\033[1;32m %s\033[0m",
					print_ModuleStatus(running_module_status_.load()).c_str());
	}
	RCLCPP_INFO(node_->get_logger(), "***************************************************");
	std::cout << "Load lidar-slam param successfully44" << std::endl;
	ros_spinner_start();
	std::cout << "Load lidar-slam param successfully55" << std::endl;
}

LocalizationModule::~LocalizationModule() {}

float line_length(float dx, float dy) { return std::sqrt(dx * dx + dy * dy); }

bool LocalizationModule::create_ROS_IO() {
	if (!node_) {
		throw std::runtime_error("ROS node not initialized");
	}
	std::cout << "[LocalizationModule::create_ROS_IO]:: 11" << std::endl;
	// ROS_INFO_STREAM("Thread["<< boost::this_thread::get_id() <<"] main thread");
	// subscriber ********************************************************************
	// ros::Subscriber sub_pcl = nh_.subscribe<livox_ros_driver2::CustomMsg>("/livox/lidar", 200000,
	// &LocalizationModule::livox_msg_cbk, this); sub_pointcloud2_ =
	// nh_.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 10, &LocalizationModule::livox_ros_cbk, this);
	// sub_pointcloud2_ = nh5_.subscribe<sensor_msgs::PointCloud2>(slam_param_.lidar_preproc.sub_lidar_topic, 10,
	// &LocalizationModule::lidar_ros_callback, this);
	sub_pointcloud2_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
		slam_param_.lidar_preproc.sub_lidar_topic, 10,
		std::bind(&LocalizationModule::lidar_ros_callback, this, std::placeholders::_1));

	std::cout << "=========================  slam_param_.lidar_preproc.sub_lidar_topic:  "
			  << slam_param_.lidar_preproc.sub_lidar_topic << std::endl;
	std::cout << "[LocalizationModule::create_ROS_IO]:: 22" << std::endl;
	// sub_imu_ = nh3_.subscribe<sensor_msgs::Imu>(slam_param_.lidar_preproc.sub_imu_topic, 2000,
	// &LocalizationModule::imu_callback, this);
	sub_imu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
		slam_param_.lidar_preproc.sub_imu_topic,
		rclcpp::QoS(2000), // ROS2中使用QoS替代简单的队列大小
		std::bind(&LocalizationModule::imu_callback, this, std::placeholders::_1));

	// sub_chassis_ = nh_.subscribe<fairland_msgs::chassic_data>("/flbot/hardware/chassic_data", 100,
	// &LocalizationModule::chassis_callback, this); publish
	// ************************************************************************ pub_localization_module_status_ =
	// nh5_.advertise<fairland_msgs::LocalizationModuleStatus>(slam_param_.common.pub_topic_module_status, 100);
	// pub_localization_module_health_ =
	// nh5_.advertise<fairland_msgs::LocalizationModuleHealth>(slam_param_.common.pub_topic_module_health, 100);
	pub_localization_module_status_ = node_->create_publisher<fairland_msgs::msg::LocalizationModuleStatus>(
		slam_param_.common.pub_topic_module_status, rclcpp::QoS(100));

	pub_localization_module_health_ = node_->create_publisher<fairland_msgs::msg::LocalizationModuleHealth>(
		slam_param_.common.pub_topic_module_health, rclcpp::QoS(100));

	// pub_filter_odometry_ = nh_.advertise<nav_msgs::Odometry>("/Odometry_lidar_in_map_filter", 100);
	// pub_log_ = nh_.advertise<fairland_msgs::LocalizationModuleLogInfo>(slam_param_.common.pub_topic_module_loginfo,
	// 100); pub_log_ = nh5_.advertise<std_msgs::Float64MultiArray>(slam_param_.common.pub_topic_module_loginfo, 100);
	pub_log_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(slam_param_.common.pub_topic_module_loginfo,
																		 rclcpp::QoS(100));

	// pub_slip_ = nh_.advertise<fairland_msgs::NameValues>(slam_param_.common.pub_topic_slipping, 100);

	// both 建图 & 定位
	// pubLidarInMap = nh_.advertise<nav_msgs::Odometry>("/Odometry_lidar_in_map", 100);
	pubOdomAftMapped = node_->create_publisher<nav_msgs::msg::Odometry>("/Odometry_lidar_in_map", rclcpp::QoS(100));

	// 在类声明中
	// rclcpp::CallbackGroup::SharedPtr slam_callback_group_;
	// rclcpp::CallbackGroup::SharedPtr ctrl_callback_group_;
	// rclcpp::TimerBase::SharedPtr timer_slam_;
	// rclcpp::Subscription<your_msg_type>::SharedPtr sub_mapping_ctrl_;
	/*
		// 在构造函数中初始化
		slam_callback_group_ = node_->create_callback_group(
			rclcpp::CallbackGroupType::MutuallyExclusive);

		// 创建定时器 (10Hz)
		// auto timer_options = rclcpp::TimerOptions();
		auto timer_options = rclcpp::NodeOptions();
		timer_options.callback_group = slam_callback_group_;
		this->timer_slam_ = node_->create_wall_timer(
			std::chrono::milliseconds(100), // 100ms = 10Hz
			std::bind(&LocalizationModule::slam_dealt_timer, this),
			timer_options
		);
	*/

	slam_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	this->timer_slam_ =
		node_->create_wall_timer(std::chrono::milliseconds(100), // 100ms = 10Hz
								 std::bind(&LocalizationModule::slam_dealt_timer, this), slam_callback_group_);

	ctrl_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	// 创建订阅器
	auto sub_options		   = rclcpp::SubscriptionOptions();
	sub_options.callback_group = ctrl_callback_group_;
	this->sub_mapping_ctrl_	   = node_->create_subscription<std_msgs::msg::UInt32>(
		   slam_param_.common.sub_topic_ctrl_cmd,
		   rclcpp::QoS(3), // 保持队列大小为3
		   std::bind(&LocalizationModule::localization_module_ctrl_callback, this, std::placeholders::_1), sub_options);

	// only 建图

	//需要单独声明一个ros::NodeHandle nh2_
	//为这个ros::Nodehandle指定单独的Callback队列 slam_queue_
	// ros::CallbackQueue slam_queue_;
	// nh2_.setCallbackQueue(&slam_queue_);
	// // 建图主要流程，timer 时间间隔需要调整，10hz? 100hz? 200hz?
	// timer_slam_ = nh2_.createTimer(ros::Duration(0.1), &LocalizationModule::slam_dealt_timer, this); //主函数

	// nh3_.setCallbackQueue(&slam_ctrl_queue_);
	// sub_mapping_ctrl_ = nh3_.subscribe(slam_param_.common.sub_topic_ctrl_cmd, 3
	// ,&LocalizationModule::localization_module_ctrl_callback, this);

	// timer dealt ********************************************************************
	// nh5_.setCallbackQueue(&health_queue_);    // TODO::
	// timer_module_status_ = nh5_.createTimer(ros::Duration(0.1), &LocalizationModule::pub_module_status_timer, this);
	this->timer_module_status_ =
		node_->create_wall_timer(std::chrono::milliseconds(100), // 100ms = 10Hz
								 std::bind(&LocalizationModule::pub_module_status_timer, this), ctrl_callback_group_);

	// only 定位

	// publish TODO: 还需要区分哪些是建图或定位发布的
	//------------------ // TODO::
	// pubOdomCloud = nh_.advertise<sensor_msgs::PointCloud2>("/odom_cloud", 10);
	// pubBodyCloud = nh_.advertise<sensor_msgs::PointCloud2>("/flbot/localization/body_cloud", 20);
	// pub_body_cloud_filter_ = nh_.advertise<sensor_msgs::PointCloud2>("/flbot/localization/body_cloud_filter", 20);
	// pub_key_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/flbot/localization/key_body_cloud", 20);
	// pubObstacleCloud = nh_.advertise<sensor_msgs::PointCloud2>("/obstacle_cloud", 10);
	// pubFilteredObstacleCloud = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_obstacle_cloud", 10);
	// pubTestCloud = nh_.advertise<sensor_msgs::PointCloud2>("/test_cloud", 10);
	// pubKdtreeCloud = nh_.advertise<sensor_msgs::PointCloud2>("/kdtree_cloud", 10);
	// pubOptimizedPath= nh_.advertise<nav_msgs::Path>("/optimized_path", 1000);
	// pubUnoptimizedPath= nh_.advertise<nav_msgs::Path>("/unoptimized_path", 1000);
	// pubLoopConstraintEdge = nh_.advertise<visualization_msgs::MarkerArray>("/flbot/mapping/loop_closure_constraints",
	// 1); pubKeyframePose = nh_.advertise<visualization_msgs::MarkerArray>("/key_frame_pose", 1); pubOdomAftMapped =
	// nh_.advertise<nav_msgs::Odometry>("/Odometry", 10); pubLoadMap =
	// nh_.advertise<sensor_msgs::PointCloud2>("/Load_map", 1); pubRgbCloud=
	// nh_.advertise<sensor_msgs::PointCloud2>("rgb_cloud", 1); pub_base_imu_ =
	// nh_.advertise<sensor_msgs::Imu>("/flbot/localization/imu", 100);

	// 在类的构造函数或初始化函数中创建发布器
	pubOdomCloud = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/odom_cloud", 10);
	pubBodyCloud = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/flbot/localization/body_cloud", 20);
	pub_body_cloud_filter_ =
		node_->create_publisher<sensor_msgs::msg::PointCloud2>("/flbot/localization/body_cloud_filter", 20);
	pub_key_cloud_	 = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/flbot/localization/key_body_cloud", 20);
	pubObstacleCloud = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacle_cloud", 10);
	pubFilteredObstacleCloud = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_obstacle_cloud", 10);
	pubTestCloud			 = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/test_cloud", 10);
	pubKdtreeCloud			 = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/kdtree_cloud", 10);
	pubOptimizedPath		 = node_->create_publisher<nav_msgs::msg::Path>("/optimized_path", 1000);
	pubUnoptimizedPath		 = node_->create_publisher<nav_msgs::msg::Path>("/unoptimized_path", 1000);
	pubLoopConstraintEdge =
		node_->create_publisher<visualization_msgs::msg::MarkerArray>("/flbot/mapping/loop_closure_constraints", 1);
	pubKeyframePose = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/key_frame_pose", 1);
	// pubOdomAftMapped = node_->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 10);
	pubLoadMap	  = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/Load_map", 1);
	pubRgbCloud	  = node_->create_publisher<sensor_msgs::msg::PointCloud2>("rgb_cloud", 1);
	pub_base_imu_ = node_->create_publisher<sensor_msgs::msg::Imu>("/flbot/localization/imu", 100);

	//-------------------
	// image_pub = nh_.advertise<sensor_msgs::Image>("fisheye_image", 1);

	// ---------------------------------------------------

	return true;
}

void LocalizationModule::ros_spinner_start() {
	// 创建多线程执行器
	// auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

	// 添加节点到执行器
	// executor->add_node(node_);

	// 为不同模块创建回调组（替代ROS1的队列）
	// auto slam_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	// auto ctrl_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	// auto filter_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	// auto health_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	// 创建各模块的订阅/定时器时指定对应的回调组
	// (通过前面提到的TimerOptions等方式)
	// executor->spin();  // 主线程会停在这里
	// 启动执行器（非阻塞）
	// std::thread(‌:ml-search[executor] { executor->spin(); }).detach();
}

/*
void LocalizationModule::ros_spinner_start(){
	//启动两个线程处理全局Callback队列
	ros::AsyncSpinner spinner(1);
	spinner.start();

	//启动一个线程处理 slam main 单独的队列
	ros::AsyncSpinner spinner_2(1, &slam_queue_);
	spinner_2.start();


	//启动一个线程处理 slam ctrl 单独的队列
	ros::AsyncSpinner spinner_3(1, &slam_ctrl_queue_);
	spinner_3.start();


	//启动一个线程处理 pose filter 单独的队列
	ros::AsyncSpinner spinner_4(1, &pose_filter_queue_);
	if(slam_param_.common.use_pose_filter){
		spinner_4.start();
	}

	//启动一个线程处理 slam ctrl 单独的队列
	ros::AsyncSpinner spinner_5(1, &health_queue_);
	spinner_5.start();


	ros::waitForShutdown();

}
*/

// void LocalizationModule::slam_dealt_timer(const ros::TimerEvent &event)

void LocalizationModule::slam_dealt_timer() { // TODO : 主函数{   // TODO : 主函数
	// SLAM 主要流程， 对应于原来的 while (ros::ok()){...}

	if (slam_param_.common.cpu_id.size() > 0) {
		pthread_t this_thread = pthread_self(); // 获取当前线程的 ID
		if (pthread_setaffinity_np(this_thread, sizeof(cpu_mask_), &cpu_mask_) < 0) {
			perror("pthread_setaffinity_np");
			exit(EXIT_FAILURE);
		}
	}

	static double last_slam_hb = hb_time_timer_slam_.load(); // 原子变量
	// hb_time_timer_slam_.store(ros::WallTime::now().toSec());
	hb_time_timer_slam_.store(node_->now().seconds());
	double slam_timer_interval = hb_time_timer_slam_.load() - last_slam_hb;
	// std::cout << " [LocalizationModule::slam_dealt_timer]: eeeeeee->" << slam_timer_interval<< std::endl;
	// ///////////////////////////////////////////////////////////
	// // DEBUG
	// static double hb_time_timer_slam_rostime = ros::Time::now().toSec();
	// static double last_slam_rostime = hb_time_timer_slam_rostime;

	// hb_time_timer_slam_rostime = ros::Time::now().toSec();
	// double slam_timer_interval_rostime = hb_time_timer_slam_rostime - last_slam_rostime;
	// last_slam_rostime = hb_time_timer_slam_rostime;
	// // ROS_INFO_STREAM("ROS time now     : " << setprecision(15) << hb_time_timer_slam_rostime);
	// // ROS_INFO_STREAM("ROS time interval: " << slam_timer_interval_rostime);
	// ///////////////////////////////////////////////////////////

	log_info_manager_->slam_info.data[29] = slam_timer_interval;
	last_slam_hb						  = hb_time_timer_slam_.load();

	if (slam_timer_interval < 0) {
		//  ROS_ERROR_STREAM(RED << "slam main thread time jump back, this_time - last_time = " << slam_timer_interval
		//  << " seconds" << RESET);
		RCLCPP_ERROR(node_->get_logger(), "slam main thread time jump back, this_time - last_time = %.3f seconds",
					 slam_timer_interval);
	}

	if (slam_timer_interval > 0.5) {
		//       ROS_ERROR_STREAM(RED << "slam main thread time jump to future, this_time - last_time = " <<
		//       slam_timer_interval << " seconds" << RESET);
		RCLCPP_ERROR(node_->get_logger(), "slam main thread time jump to future, this_time - last_time =  %.3f seconds",
					 slam_timer_interval);
	}

	// cout<<" *********************************************** "<<endl;
	// cout<<" ----------------------------------------------- "<<endl;
	ModuleStatus curr_running_module_status = running_module_status_.load();

	static unsigned int print_idle_cnt		   = 0;
	static unsigned int print_reset_slam_cnt   = 0;
	static unsigned int print_running_slam_cnt = 0;
	if (curr_running_module_status == ModuleStatus::MODULE_IDLE) {
		bool condition = (print_idle_cnt++ == 10);
		RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), condition,
									  "*************************************************");

		RCLCPP_INFO_STREAM_EXPRESSION(
			node_->get_logger(), condition,
			"slam dealt: running module status: " << print_ModuleStatus(curr_running_module_status));
		RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), condition,
									  "*************************************************");
		// ROS_INFO_COND(condition , "*************************************************");
		// ROS_INFO_COND(condition , "slam dealt: running module status: %s",
		// print_ModuleStatus(curr_running_module_status).c_str()); ROS_INFO_COND(condition ,
		// "-------------------------------------------------");
		print_reset_slam_cnt   = 0; // if not STARTING or STOPPING slam, reset to 0
		print_running_slam_cnt = 0; // if not RUNNING slam, reset to 0
		return;
	}

	if (curr_running_module_status == ModuleStatus::MODULE_STARTING_SLAM ||
		curr_running_module_status == ModuleStatus::MODULE_STOPPING_SLAM) {
		bool condition = (print_reset_slam_cnt++ == 0);
		// ROS_INFO_COND(condition , "*************************************************");
		// ROS_INFO_COND(condition , "slam dealt: running module status: %s",
		// print_ModuleStatus(curr_running_module_status).c_str()); ROS_INFO_COND(condition ,
		// "-------------------------------------------------");
		RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), condition,
									  "*************************************************");
		RCLCPP_INFO_STREAM_EXPRESSION(
			node_->get_logger(), condition,
			"slam dealt: running module status: " << print_ModuleStatus(curr_running_module_status));
		RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), condition,
									  "*************************************************");
		print_idle_cnt		   = 0; // if not IDLE, reset to 0
		print_running_slam_cnt = 0; // if not RUNNING slam, reset to 0
		return;
	}

	if (curr_running_module_status == ModuleStatus::MODULE_MAPPING ||
		curr_running_module_status == ModuleStatus::MODULE_SEC_MAPPING ||
		curr_running_module_status == ModuleStatus::MODULE_LOCALIZATION) {
		bool condition = (print_running_slam_cnt++ == 10);
		// ROS_INFO_STREAM("print_running_slam_cnt: " << print_running_slam_cnt);
		// ROS_INFO_COND(condition , "*************************************************");
		// ROS_INFO_COND(condition , "slam dealt: running module status: %s",
		// print_ModuleStatus(curr_running_module_status).c_str()); ROS_INFO_COND(condition ,
		// "-------------------------------------------------");
		RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), condition,
									  "*************************************************");
		RCLCPP_INFO_STREAM_EXPRESSION(
			node_->get_logger(), condition,
			"slam dealt: running module status: " << print_ModuleStatus(curr_running_module_status));
		RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), condition,
									  "*************************************************");
		print_idle_cnt		 = 0; // if not IDLE, reset to 0
		print_reset_slam_cnt = 0; // if not STARTING or STOPPING SLAM, reset to 0
	}

	// if (show_load_map_==0 && localization_mode_ && (slam_->getLoadMap())->points.size() > 0){
	// if (show_load_map_==0 && localization_mode_ && (slam_->getLoadMap()) && (slam_->getLoadMap())->points.size() >
	// 0){ if (show_load_map_==0 && curr_running_module_status == ModuleStatus::MODULE_LOCALIZATION &&
	// (slam_->getLoadMap()) && (slam_->getLoadMap())->points.size() > 0){
	if (show_load_map_ % 200 == 0 && curr_running_module_status == ModuleStatus::MODULE_LOCALIZATION &&
		(slam_->getLoadMap()) && (slam_->getLoadMap())->points.size() > 0) {
		// ROS_INFO("load map");
		// sleep(1);
		//   TODO:  暂时先不
		sensor_msgs::msg::PointCloud2 loadMap;
		pcl::toROSMsg(*(slam_->getLoadMap()), loadMap);
		loadMap.header.stamp	= node_->now();
		loadMap.header.frame_id = "map";
		pubLoadMap->publish(loadMap); // TODO：： 发布地图
		show_keyframe(slam_->getLoadKeyFrame());

		// ROS_INFO("load map success");
		// show_load_map_ ++;
	}
	show_load_map_++;

	if (just_show_mode_) { // TODO：： 忽略
		return;
	}
	/********************************- run slam -********************************/

	// thisId = std::this_thread::get_id();
	// running_slam_flag==false 的情况: 1第一帧; 2无点云； 3点云数量太少；
	bool running_slam_flag = slam_->run(); // TODO :: 按照timer 的周期执行
	// ros::Time ros_time_now = ros::Time().now();

	if (running_slam_flag) { // TODO: 发布一下调试点云信息
		if ((is_mapping_status(curr_running_module_status)) || slam_->isGloalLocalizationSuccess()) {
			publish_cloud(slam_->get_odom_cloud(), "base_link", pubOdomCloud);
		}
		// pub body cloud
		publish_cloud(slam_->get_lidar_cloud(), "lidar", pubBodyCloud);
		// publish_cloud(slam_->get_filter_lidar_cloud(), "lidar", ros_time_now,  pub_body_cloud_filter_);

		// process_loginfo();
	}

	auto localization_status_now = localization_status_.load();
	// TODO : 发布一下 建图点云信息
	if (is_mapping_status(curr_running_module_status) &&
		mapping_status_.load() == 3) { // mapping_status_ 是建图状态  slam 内部修改
		// publish_odometry_lidar_in_map(slam_->getLidarInMap() * T_lidar_baselink_,slam_->get_current_pose(),  "map",
		// "base_link", curr_running_module_status, pubLidarInMap); pub_lidar_cloud(slam_->get_lidar_cloud(),
		// pubBodyCloud);
		if (slam_->get_new_key_cloud_arrived()) {
			publish_cloud(slam_->get_lidar_cloud(), "lidar", pub_key_cloud_); // TODO:
			slam_->set_new_key_cloud_arrived(false);
		}
		// pub kdtree cloud
		publish_cloud(slam_->get_kdtree_cloud(), "mapping_odom", pubKdtreeCloud); // TODO::
		// pub odom cloud
		publish_cloud(slam_->get_odom_cloud(), "mapping_odom", pubOdomCloud); // TODO:

		visualizeLoopClosure(slam_->getloopIndex(), optimized_path_msg);		// TODO:
		publish_unoptimized_path(slam_->get_unoptimized_path(), string("map")); // TODO::
		// publish_optimized_path(slam_->get_optimized_path(),string("odom"), pubOptimizedPath);
		publish_optimized_path(slam_->get_optimized_path(), string("map"));
	} else if (curr_running_module_status == ModuleStatus::MODULE_LOCALIZATION &&
			   localization_status_is_ok(localization_status_now)) { // TODO: 忽略
		if (slam_->isGloalLocalizationSuccess()) {
			// publish_odometry_lidar_in_map(slam_->getLidarInMap() * T_lidar_baselink_, slam_->get_current_pose(),
			// "map", "base_link", curr_running_module_status, pubLidarInMap); publish_odometry(slam_->getLidarInOdom(),
			// pubOdomAftMapped);
		}
		// pub_lidar_cloud(slam_->get_lidar_cloud(), pubBodyCloud);
	}
	// if (show_rviz_){
	//     publish_static_transform(slam_->getWheelInLidar());
	//     publish_odometry(slam_->getLidarInOdom(), pubOdomAftMapped);
	// }
}

// TODO :调试信息
/*
void LocalizationModule::process_loginfo(){
	static Eigen::Isometry3d last_lidar_in_map = slam_->getLidarInMap();
	static Eigen::Isometry3d last_lidar_in_odom = slam_->getLidarInOdom();

	Eigen::Isometry3d curr_lidar_in_map = slam_->getLidarInMap();
	Eigen::Isometry3d curr_lidar_in_odom = slam_->getLidarInOdom();

	Eigen::Isometry3d lidar_in_map_inv = curr_lidar_in_map.inverse();
	Eigen::Isometry3d lidar_in_odom_inv = curr_lidar_in_odom.inverse();

	Eigen::Isometry3d last_lidar_in_map_baselink = lidar_in_map_inv * last_lidar_in_map;
	Eigen::Isometry3d curr_lidar_in_map_baselink = lidar_in_map_inv * curr_lidar_in_map;

	Eigen::Isometry3d last_lidar_in_odom_baselink = lidar_in_odom_inv * last_lidar_in_odom;
	Eigen::Isometry3d curr_lidar_in_odom_baselink = lidar_in_odom_inv * curr_lidar_in_odom;

	// log_info_manager_->slam_info.data[19] = curr_lidar_in_odom_baselink.translation().x() -
last_lidar_in_odom_baselink.translation().x() ;
	// log_info_manager_->slam_info.data[20] = curr_lidar_in_odom_baselink.translation().y() -
last_lidar_in_odom_baselink.translation().y() ;
	// log_info_manager_->slam_info.data[21] = curr_lidar_in_map_baselink.translation().x() -
last_lidar_in_map_baselink.translation().x() ;
	// log_info_manager_->slam_info.data[22] = curr_lidar_in_map_baselink.translation().y() -
last_lidar_in_map_baselink.translation().y() ;

	log_info_manager_->slam_info.data[19] = curr_lidar_in_odom.translation().x() - last_lidar_in_odom.translation().x()
; log_info_manager_->slam_info.data[20] = curr_lidar_in_odom.translation().y() - last_lidar_in_odom.translation().y() ;

	log_info_manager_->slam_info.data[21] = curr_lidar_in_map.translation().x() - last_lidar_in_map.translation().x() ;
	log_info_manager_->slam_info.data[22] = curr_lidar_in_map.translation().y() - last_lidar_in_map.translation().y() ;

	// # 19: baseframe_slam_dx   # 车身 坐标系下, dx
	// # 20: baseframe_slam_dy   # 车身 坐标系下, dy
	// # 21: baseframe_res_dx   # 车身 坐标系下, dx
	// # 22: baseframe_res_dy   # 车身 坐标系下, dy

	// // Eigen::Isometry3d curr_lidar_in_map = slam_->getLidarInMap();
	// // Eigen::Isometry3d lidar_in_map_inv = curr_lidar_in_map.inverse();
	// Eigen::Isometry3d curr_odom_to_map = slam_->getOdomToMap();
	// Eigen::Isometry3d curr_odom_to_map_baselink = lidar_in_map_inv * curr_odom_to_map;
	// log_info_manager_->slam_info.data[17] = curr_odom_to_map_baselink.translation().x();
	// log_info_manager_->slam_info.data[18] = curr_odom_to_map_baselink.translation().y();

	// update
	last_lidar_in_odom = curr_lidar_in_odom;
	last_lidar_in_map  = curr_lidar_in_map;
}
*/

// TODO :调试信息（健康状态）， 不影响程序运行
int LocalizationModule::check_fill_health_msg(ModuleStatus									curr_running_module_status,
											  fairland_msgs::msg::LocalizationModuleHealth& health_msg) {
	static const double imu_interval			   = 0.005;
	static const double lidar_interval			   = 0.1;
	static const double slam_interval			   = 0.1;
	static const double pose_interval			   = 0.05;
	static const double localize_interval		   = 1.0;
	static const double loop_closure_interval	   = 1.0;
	static const double secmap_relocalize_interval = 1.0;
	static const int	imu_ratio				   = 20; // 20
	static const int	lidar_ratio				   = 3;
	static const int	slam_ratio				   = 3;
	static const int	pose_ratio				   = 3;
	static const int	localize_ratio			   = 3;
	static const int	loop_closure_ratio		   = 3;
	static const int	secmap_relocalize_ratio	   = 3;
	static const int	point_cloud_size_thr	   = 600;

	// check ROS IO status **********************************************************************
	int health_status_now = 0;
	// health_status_.store(0); // reset to status ok

	auto   curr_ros_time			   = node_->now();
	double curr_time				   = rclcpp::Time(curr_ros_time).seconds(); //////////////////TODO::
	double delay_imu				   = curr_time - hb_time_cbk_imu_.load();
	double delay_lidar				   = curr_time - hb_time_cbk_lidar_.load();
	double delay_slam				   = curr_time - hb_time_timer_slam_.load();
	double delay_pose				   = curr_time - hb_time_timer_pose_.load();
	bool   hb_cbk_lidar				   = delay_lidar < lidar_interval * lidar_ratio ? true : false;
	bool   hb_cbk_imu				   = delay_imu < imu_interval * imu_ratio ? true : false;
	bool   hb_timer_slam			   = delay_slam < slam_interval * slam_ratio ? true : false;
	bool   hb_timer_pose			   = delay_pose < pose_interval * pose_ratio ? true : false;
	bool   hb_thread_localize		   = true;
	bool   hb_thread_loop_closure	   = true;
	bool   hb_thread_secmap_relocalize = true;
	bool   error_lidar_point_too_few   = false;
	bool   error_livox_driver_failed   = false;

	// if(curr_running_module_status == ModuleStatus::MODULE_IDLE){
	//     hb_cbk_lidar = 1;
	//     hb_cbk_imu = 1;
	// }

	if (!hb_cbk_lidar || !hb_cbk_imu || !hb_timer_slam || !hb_timer_pose) {
		health_status_now = 1;
	}

	// check thread in slam.cpp ******************************************************************
	double localize_delay		   = 0.0;
	double loop_closure_delay	   = 0.0;
	double secmap_relocalize_delay = 0.0;
	if (curr_running_module_status == ModuleStatus::MODULE_MAPPING) {
		loop_closure_delay	   = curr_time - slam_->get_hb_time_thread_loop_closure();
		hb_thread_loop_closure = loop_closure_delay < loop_closure_interval * loop_closure_ratio ? true : false;
		if (!hb_thread_loop_closure) {
			health_status_now = std::max(1, health_status_now);

			mapping_node_status_.store(4); // 0: inactive
		}
	} else if (curr_running_module_status == ModuleStatus::MODULE_SEC_MAPPING) {
		loop_closure_delay		= curr_time - slam_->get_hb_time_thread_loop_closure();
		secmap_relocalize_delay = curr_time - slam_->get_hb_time_thread_secmap_relocalize();
		hb_thread_loop_closure	= loop_closure_delay < loop_closure_interval * loop_closure_ratio ? true : false;
		hb_thread_secmap_relocalize =
			secmap_relocalize_delay < secmap_relocalize_interval * secmap_relocalize_ratio ? true : false;
		if (!hb_thread_loop_closure || !hb_thread_secmap_relocalize) {
			health_status_now = std::max(1, health_status_now);
			mapping_node_status_.store(4); // 0: inactive
		}
	} else if (curr_running_module_status == ModuleStatus::MODULE_LOCALIZATION) {
		localize_delay	   = curr_time - slam_->get_hb_time_thread_localize();
		hb_thread_localize = localize_delay < localize_interval * localize_ratio ? true : false;
		if (!hb_thread_localize) {
			health_status_now = std::max(1, health_status_now);
			local_node_status_.store(3); // 0: inactive
		}
	}
	// check lidar driver **************************************************************
	int orig_point_cloud_size	= 0;
	int sample_point_cloud_size = 0;
	if (hb_cbk_lidar) {
		orig_point_cloud_size	= cloud_size_orig_.load();
		sample_point_cloud_size = cloud_size_sample_.load();
		if (orig_point_cloud_size < point_cloud_size_thr) {
			error_lidar_point_too_few = true;
		}
	}
	if (slam_param_.lidar_preproc.lidar_type == 1 && orig_point_cloud_size == 96) {
		RCLCPP_ERROR(node_->get_logger(), "livox driver error, cloud-size: 96");
		error_livox_driver_failed = true;
		health_status_now		  = std::max(2, health_status_now);
	}

	// fill health msg *************************************************************************
	health_msg.cloud_size = orig_point_cloud_size;

	log_info_manager_->slam_info.data[12] = orig_point_cloud_size;	 //
	log_info_manager_->slam_info.data[14] = sample_point_cloud_size; //

	health_msg.delay_cbk_lidar				  = delay_lidar;			 // unit: s
	health_msg.delay_cbk_imu				  = delay_imu;				 // unit: s
	health_msg.delay_timer_slam				  = delay_slam;				 // unit: s
	health_msg.delay_timer_pose				  = delay_pose;				 // unit: s
	health_msg.delay_thread_localize		  = localize_delay;			 // unit: s
	health_msg.delay_thread_loop_closure	  = loop_closure_delay;		 // unit: s
	health_msg.delay_thread_secmap_relocalize = secmap_relocalize_delay; // unit: s

	health_msg.hb_cbk_lidar				   = hb_cbk_lidar;				  // value: [0] or [1]
	health_msg.hb_cbk_imu				   = hb_cbk_imu;				  // value: [0] or [1]
	health_msg.hb_timer_slam			   = hb_timer_slam;				  // value: [0] or [1]
	health_msg.hb_timer_pose			   = hb_timer_pose;				  // value: [0] or [1]
	health_msg.hb_thread_localize		   = hb_thread_localize;		  // value: [0] or [1]
	health_msg.hb_thread_loop_closure	   = hb_thread_loop_closure;	  // value: [0] or [1]
	health_msg.hb_thread_secmap_relocalize = hb_thread_secmap_relocalize; // value: [0] or [1]

	health_msg.error_lidar_point_too_few = error_lidar_point_too_few; // value: [0] or [1]
	health_msg.error_livox_driver_failed = error_livox_driver_failed; // value: [0] or [1]

	health_msg.health_status = health_status_now;

	return health_status_now;
}

// TODO: 发布健康状态，节点工作状态（重点）
void LocalizationModule::pub_module_status_timer() {
	// ******************************************************************************************
	auto curr_running_module_status = running_module_status_.load();
	auto curr_ros_time				= node_->now();

	fairland_msgs::msg::LocalizationModuleHealth health_msg;
	int health_status_now	   = check_fill_health_msg(curr_running_module_status, health_msg);
	health_msg.header.stamp	   = curr_ros_time;
	health_msg.header.frame_id = "base_link";

	health_status_.store(health_status_now);
	// make status msg *************************************************************************
	// fill header

	fairland_msgs::msg::LocalizationModuleStatus status_msg;

	check_fill_module_status_msg(curr_running_module_status, status_msg);
	status_msg.header.stamp	   = curr_ros_time;
	status_msg.header.frame_id = "base_link";

	// make health msg end *********************************************************************

	pub_localization_module_status_->publish(status_msg);
	pub_localization_module_health_->publish(health_msg);
	// ROS_INFO("pub: time: %lf ", status_msg.header.stamp.toSec());

	pub_log_->publish(log_info_manager_->slam_info);
}

//////////////////////////////////////局部函数
geometry_msgs::msg::TransformStamped initIdentityTransform() {
	geometry_msgs::msg::TransformStamped transform;

	// 初始化header
	transform.header.stamp	  = rclcpp::Clock().now();
	transform.header.frame_id = "odom";
	transform.child_frame_id  = "base_link";

	// 初始化平移部分 (0,0,0)
	transform.transform.translation.x = 0.0;
	transform.transform.translation.y = 0.0;
	transform.transform.translation.z = 0.0;

	// 初始化旋转部分 (单位四元数)
	tf2::Quaternion q;
	q.setRPY(0, 0, 0); // 无旋转
	transform.transform.rotation.x = q.x();
	transform.transform.rotation.y = q.y();
	transform.transform.rotation.z = q.z();
	transform.transform.rotation.w = q.w();

	return transform;
}

void LocalizationModule::check_fill_module_status_msg(ModuleStatus curr_running_module_status,
													  fairland_msgs::msg::LocalizationModuleStatus& status_msg) {
	// fill status_msg.module_status
	if (curr_running_module_status == ModuleStatus::MODULE_IDLE) {
		// status_msg.module_status = fairland_msgs::LocalizationModuleStatus::IDLE;
		status_msg.module_status = int(ModuleStatus_o::IDLE);
	} else if (is_mapping_status(curr_running_module_status)) {
		// status_msg.module_status = fairland_msgs::LocalizationModuleStatus::MAPPING;
		status_msg.module_status = int(ModuleStatus_o::MAPPING);
	} else if (curr_running_module_status == ModuleStatus::MODULE_LOCALIZATION) {
		// status_msg.module_status = fairland_msgs::LocalizationModuleStatus::LOCALIZATION;
		status_msg.module_status = int(ModuleStatus_o::LOCALIZATION);
	} else if (curr_running_module_status == ModuleStatus::MODULE_STARTING_SLAM) {
		// status_msg.module_status = fairland_msgs::LocalizationModuleStatus::STARTING;
		status_msg.module_status = int(ModuleStatus_o::STARTING);
	} else if (curr_running_module_status == ModuleStatus::MODULE_STOPPING_SLAM) {
		status_msg.module_status = int(ModuleStatus_o::STOPPING);
	} else {
		RCLCPP_ERROR(node_->get_logger(), "error running module status: %s",
					 print_ModuleStatus(curr_running_module_status).c_str());
	}

	// fill status_msg.localization_status
	fill_module_l_status(curr_running_module_status, status_msg);

	// fill status_msg.mapping_status
	fill_module_m_status(curr_running_module_status, status_msg);

	status_msg.map_saved = 0;
	if (curr_running_module_status == ModuleStatus::MODULE_IDLE && map_saved_.load() == 1) {
		status_msg.map_saved = 1;
		// ROS_WARN_STREAM(YELLOW << "[Status Timer]: status_msg.map_saved: " << int(status_msg.map_saved) << RESET);
		RCLCPP_WARN(node_->get_logger(), "[Status Timer]: status_msg.map_saved: %d", int(status_msg.map_saved));

		map_saved_.store(0);
	}

	if (status_msg.localization_status != 0 && status_msg.localization_status != 3) {
		// ROS_WARN_STREAM_THROTTLE(1.0, YELLOW << "[Status Timer]: localization_status: " <<
		// int(status_msg.localization_status) << RESET);
		RCLCPP_WARN(node_->get_logger(), "[Status Timer]: localization_status: %d",
					int(status_msg.localization_status));
	}
	if (status_msg.mapping_status != 0 && status_msg.mapping_status != 3) {
		// ROS_WARN_STREAM_THROTTLE(1.0, RED << "[Status Timer]: mapping_status: " << int(status_msg.mapping_status)  <<
		// RESET);
		RCLCPP_WARN(node_->get_logger(), "[Status Timer]: mapping_status: %d", int(status_msg.mapping_status));
	}
	/*
		geometry_msgs::msg::TransformStamped transform_o_b = initIdentityTransform();

		try {
			transform_o_b = tf_buffer_.lookupTransform(
			"odom", "base_link", tf2::TimePointZero);
		//   RCLCPP_INFO(this->get_logger(),
		//     "Transform: [%.2f, %.2f, %.2f] [%.2f, %.2f, %.2f, %.2f]",
		//     transform_o_b.transform.translation.x,
		//     transform_o_b.transform.translation.y,
		//     transform_o_b.transform.translation.z,
		//     transform_o_b.transform.rotation.x,
		//     transform_o_b.transform.rotation.y,
		//     transform_o_b.transform.rotation.z,
		//     transform_o_b.transform.rotation.w);
		} catch (tf2::TransformException &ex) {
		  RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
		}

		// Eigen::Isometry3d T_o_b_ = transformToEigen(transform_o_b);
		Eigen::Isometry3d T_o_b_ = tf2::transformToEigen(transform_o_b.transform);
	*/

	//////////////////////////////////////////////////////////////////////

	// TODO : 发布Odometry
	// if(curr_running_module_status == ModuleStatus::MODULE_LOCALIZATION && localization_status_.load()>2){
	if (curr_running_module_status == ModuleStatus::MODULE_LOCALIZATION &&
		localization_status_is_ok(localization_status_.load())) {
		publish_odometry_lidar_in_map(slam_->getLidarInMap() * T_lidar_baselink_, slam_->get_current_pose(), "map",
									  "base_link", curr_running_module_status);

		// 配合robot-localization 节点
		// Eigen::Isometry3d T_m_o_ = slam_->getLidarInMap() * T_lidar_baselink_ * T_o_b_.inverse();

		// publish_odometry_in_map(T_m_o_, "map", "odom");
		// publish_odometry_in_map(slam_->getOdomToMap(), "map", "mapping_odom");

		// publish_odometry(slam_->getLidarInOdom(), "odom", "lidar", pubOdomAftMapped);
	}

	if (is_mapping_status(curr_running_module_status) && mapping_status_is_ok(mapping_status_.load())) {
		publish_odometry_lidar_in_map(slam_->getLidarInMap() * T_lidar_baselink_, slam_->get_current_pose(), "map",
									  "base_link", curr_running_module_status);
		// publish_odometry(slam_->getLidarInOdom(), "odom", "lidar", pubOdomAftMapped);
		// 配合robot-localization 节点
		// Eigen::Isometry3d T_m_o_ = slam_->getLidarInMap() * T_lidar_baselink_ * T_o_b_.inverse();
		// publish_odometry_in_map(T_m_o_, "map", "odom");
		// publish_odometry_in_map(slam_->getOdomToMap(), "map", "mapping_odom");
	}
}

void LocalizationModule::fill_module_l_status(ModuleStatus									curr_running_module_status,
											  fairland_msgs::msg::LocalizationModuleStatus& status_msg) {
	if (curr_running_module_status != ModuleStatus::MODULE_LOCALIZATION) {
		status_msg.localization_status = int(LocalizationStatus::L_INACTIVE);
		localization_status_.store(0); // inactive
		return;
	}

	auto last_local_status = localization_status_.load();
	if (last_local_status == 2 || last_local_status == 5) {
		status_msg.localization_status		 = last_local_status;
		log_info_manager_->slam_info.data[2] = last_local_status; //
		return;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////
	int node_status		  = local_node_status_.load();
	int local_thrd_status = slam_->get_local_thrd_status();
	int slam_run_status	  = slam_->get_slam_run_status();

	//////////////////////////////////////////////////////////////////////////////////////////////
	static const bool check_delay = slam_param_.common.check_delay;
	if (!check_delay) {
		if (node_status == 2 || node_status == 3) {
			node_status = 1;
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////
	if (node_status == 0) {
		status_msg.localization_status =
			int(LocalizationStatus::L_INACTIVE); // fairland_msgs::LocalizationModuleStatus::L_INACTIVE;
		localization_status_.store(0);
	} else if (node_status == 1) {	// node_status = normal
		if (slam_run_status == 1) { // slam_run_status = normal
			status_msg.localization_status = local_thrd_status;
			localization_status_.store(local_thrd_status); // 与 定位线程的状态一致
		} else if (slam_run_status == 2) {				   //
			status_msg.localization_status =
				int(LocalizationStatus::L_FAILED); // fairland_msgs::LocalizationModuleStatus::L_FAILED;
			localization_status_.store(5);		   // 定位失败
		} else {
			// ROS_ERROR_STREAM("node_status: 1, slam_run_status: " << slam_run_status);
			// status_msg.localization_status = fairland_msgs::LocalizationModuleStatus::L_FAILED;
			// localization_status_.store(5); // 定位失败
		}
	} else if (node_status == 2) { // lidar cbk delay
		// ROS_ERROR_STREAM(RED << "lidar cbk delay !!!" << RESET);
		// RCLCPP_EROOR(node_->get_logger(),"lidar cbk delay !!!" );
		RCLCPP_ERROR(node_->get_logger(), "lidar cbk delay !!!");
		status_msg.localization_status =
			int(LocalizationStatus::L_FAILED); // fairland_msgs::LocalizationModuleStatus::L_FAILED;
		localization_status_.store(5);		   // 定位失败
	} else if (node_status == 3) {			   // localize thread delay
		// ROS_ERROR_STREAM(RED << "localize thread delay  !!!" << RESET);
		RCLCPP_ERROR(node_->get_logger(), "localize thread delay  !!!");
		status_msg.localization_status =
			int(LocalizationStatus::L_FAILED); // fairland_msgs::LocalizationModuleStatus::L_FAILED;
		localization_status_.store(5);		   // 定位失败
	} else {
		// ROS_ERROR_STREAM(RED << << RESET);
		// ROS_ERROR_STREAM("node_status: " << node_status);
		// ROS_ERROR_STREAM("slam_run_status: " << slam_run_status);
		// ROS_ERROR_STREAM("local_thrd_status: " << local_thrd_status);
		RCLCPP_ERROR(node_->get_logger(), "status error, set to L_FAILED");
		RCLCPP_ERROR(node_->get_logger(), "node_status: %d", node_status);
		RCLCPP_ERROR(node_->get_logger(), "slam_run_status:%d", slam_run_status);
		RCLCPP_ERROR(node_->get_logger(), "local_thrd_status:%d", local_thrd_status);
		status_msg.localization_status =
			int(LocalizationStatus::L_FAILED); // fairland_msgs::LocalizationModuleStatus::L_FAILED;
		localization_status_.store(5);		   // 定位失败
	}

	log_info_manager_->slam_info.data[2] = localization_status_.load(); //
}

void LocalizationModule::fill_module_m_status(ModuleStatus									curr_running_module_status,
											  fairland_msgs::msg::LocalizationModuleStatus& status_msg) {
	if (curr_running_module_status != ModuleStatus::MODULE_MAPPING &&
		curr_running_module_status != ModuleStatus::MODULE_SEC_MAPPING) {
		// status_msg.mapping_status = fairland_msgs::LocalizationModuleStatus::M_INACTIVE;
		status_msg.mapping_status = int(MappingStatus::M_INACTIVE);
		mapping_status_.store(0);
		return;
	}

	auto last_mapping_status = mapping_status_.load();
	if (last_mapping_status == 2 || last_mapping_status == 5) {
		status_msg.mapping_status = last_mapping_status;
		// log_info_manager_->slam_info.data[2]= last_mapping_status; //
		return;
	}
	//////////////////////////////////////////////////////////////////////////////////////////////
	int node_status				   = mapping_node_status_.load();
	int slam_run_status			   = slam_->get_slam_run_status();
	int secmap_relocal_thrd_status = slam_->get_secmap_relocal_thrd_status();

	//////////////////////////////////////////////////////////////////////////////////////////////
	static const bool check_delay = slam_param_.common.check_delay;
	if (!check_delay) {
		if (node_status == 2 || node_status == 3 || node_status == 4) {
			node_status = 1;
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////
	if (node_status == 0) {
		status_msg.mapping_status = int(MappingStatus::M_INACTIVE);
		mapping_status_.store(0);
	} else if (node_status == 1) {	// node_status = normal
		if (slam_run_status == 1) { // slam_run_status = normal
			if (curr_running_module_status == ModuleStatus::MODULE_MAPPING) {
				status_msg.mapping_status = int(MappingStatus::M_STANDBY);
				mapping_status_.store(3);
			} else if (curr_running_module_status == ModuleStatus::MODULE_SEC_MAPPING) {
				status_msg.mapping_status = secmap_relocal_thrd_status;
				mapping_status_.store(secmap_relocal_thrd_status);
			}
		} else if (slam_run_status == 2) { //
			status_msg.mapping_status = int(MappingStatus::M_FAILED);
			mapping_status_.store(5);
		} else {
			// ROS_ERROR_STREAM("node_status: 1, slam_run_status: " << slam_run_status);
			// status_msg.mapping_status = fairland_msgs::LocalizationModuleStatus::M_FAILED;
			// mapping_status_.store(5); // 建图失败
		}
	} else if (node_status == 2) { // lidar cbk delay
		RCLCPP_WARN(node_->get_logger(), "lidar cbk delay !!!");

		status_msg.mapping_status = int(MappingStatus::M_FAILED);
		mapping_status_.store(5);
	} else if (node_status == 3) { // localize thread delay
		RCLCPP_ERROR(node_->get_logger(), "secmap-relocal thread delay  !!!");
		status_msg.mapping_status = int(MappingStatus::M_FAILED);
		mapping_status_.store(5);
	} else if (node_status == 4) { // loop_closure_thread_delay
		RCLCPP_ERROR(node_->get_logger(), "loop_closure_thread_delay  !!!");
		status_msg.mapping_status = int(MappingStatus::M_FAILED);
		mapping_status_.store(5);
	} else {
		RCLCPP_ERROR(node_->get_logger(), "status error, set to M_FAILED");
		RCLCPP_ERROR(node_->get_logger(), "node_status:%d ", node_status);
		RCLCPP_ERROR(node_->get_logger(), "slam_run_status: :%d ", slam_run_status);
		RCLCPP_ERROR(node_->get_logger(), "secmap_relocal_thrd_status::%d ", secmap_relocal_thrd_status);
		status_msg.mapping_status = int(MappingStatus::M_FAILED);
		mapping_status_.store(5);
	}
}

// TODO : 点云统一的回调函数

// void LocalizationModule::lidar_ros_callback(const sensor_msgs::PointCloud2::ConstPtr &ros_msg)
void LocalizationModule::lidar_ros_callback(const PointCloud2::SharedPtr ros_msg) {
	static const int	cloud_size_to_keep	= slam_param_.lidar_preproc.cloud_size_to_keep;
	static const double time_cost_thr_print = slam_param_.lidar_preproc.time_cost_thr_print;

	static double last_lidar_hb = hb_time_cbk_lidar_;

	// hb_time_cbk_lidar_.store(ros::WallTime::now().toSec());
	hb_time_cbk_lidar_.store(node_->now().seconds());

	log_info_manager_->slam_info.data[23] = hb_time_cbk_lidar_ - last_lidar_hb;
	last_lidar_hb						  = hb_time_cbk_lidar_;

	// ///////////////////////////////////////////////////////////
	// // DEBUG
	// static double hb_time_timer_slam_rostime = ros_msg->header.stamp.toSec();
	// static double last_slam_rostime = hb_time_timer_slam_rostime;

	// hb_time_timer_slam_rostime = ros_msg->header.stamp.toSec();
	// double slam_timer_interval_rostime = hb_time_timer_slam_rostime - last_slam_rostime;
	// last_slam_rostime = hb_time_timer_slam_rostime;
	// // ROS_INFO_STREAM("ROS time now     : " << setprecision(15) << hb_time_timer_slam_rostime);
	// ROS_INFO_STREAM("lidar cbk time interval: " << slam_timer_interval_rostime);
	// ///////////////////////////////////////////////////////////

	if (slam_param_.common.cpu_id.size() > 0) {
		pthread_t this_thread = pthread_self(); // 获取当前线程的 ID
		if (pthread_setaffinity_np(this_thread, sizeof(cpu_mask_), &cpu_mask_) < 0) {
			perror("pthread_setaffinity_np");
			exit(EXIT_FAILURE);
		}
	}

	// cloud_size_orig_.store(ros_msg->width * ros_msg->height);

	// livox_cbk_update_time_.store(ros_msg->header.stamp.toSec());
	livox_cbk_update_time_.store(rclcpp::Time(ros_msg->header.stamp).seconds());
	// ROS_INFO_ONCE("received lidar -------------- lidar cbk");     //===============
	RCLCPP_INFO(node_->get_logger(), "received lidar -------------- lidar cbk");

	ModuleStatus curr_running_module_status = running_module_status_.load();
	if (curr_running_module_status == ModuleStatus::MODULE_IDLE ||
		curr_running_module_status == ModuleStatus::MODULE_STARTING_SLAM ||
		curr_running_module_status == ModuleStatus::MODULE_STOPPING_SLAM) {
		return;
	} // TODO: IDLE 直接返回

	double t0		 = omp_get_wtime();
	auto   start	 = std::chrono::system_clock::now();
	auto   now_as_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(start.time_since_epoch()).count();
	double now_sec	 = now_as_ns * 1e-9;
	// ROS_INFO("[lidar cbk]: lidar msg delay: %lf ms", (now_sec - ros_msg->header.stamp.toSec())*1000);

	/////////////////////////////////////////////////////////////////////////////

	static const auto	ring_count = slam_param_.lidar_preproc.cloud_ring_count;
	static const auto	col_count  = slam_param_.lidar_preproc.cloud_column_count;
	PointCloudType::Ptr cloud_preproc_ptr(new PointCloudType());
	cloud_preproc_ptr->points.reserve(ring_count * col_count);
	// cloud_preproc_ptr_->clear();
	lidar_ptr_->pre_process(ros_msg, cloud_preproc_ptr); // 降采样，去NAN
	int cloud_preproc_size = cloud_preproc_ptr->points.size();

	std::cout << "lidar cloud_preproc_size : " << cloud_preproc_size << std::endl;
	// PointCloudType::Ptr pcl_xyzin_cld(new PointCloudType());
	// lidar_ptr_ -> msg2pcl_clip(ros_msg, pcl_xyzin_cld);
	// // ROS_INFO_STREAM("clip lidar count: " << pcl_xyzin_cld->points.size());
	// PointCloudType::Ptr sample_cld_ptr(new PointCloudType());
	// lidar_ptr_->sampling_cloud(pcl_xyzin_cld, cloud_preproc_ptr);
	// int cloud_preproc_size = cloud_preproc_ptr->points.size();

	double t1 = omp_get_wtime();
	/////////////////////////////////////////////////////////////////////////////

	// if((sample_cld_size > cloud_size_to_keep + 500) || (sample_cld_size < cloud_size_to_keep - 500) ){
	//     ROS_INFO_STREAM("valid lidar num: " << pcl_xyzin_cld->points.size() << ", sample lidar num: " <<
	//     sample_cld_size);
	// }
	double t2 = omp_get_wtime();

	// cloud_size_orig_.store(pcl_xyzin_cld->points.size());
	cloud_size_orig_.store(lidar_ptr_->get_cloud_dense_size());
	cloud_size_sample_.store(cloud_preproc_size);

	// ROS_INFO_STREAM("cloud_size_orig: " << cloud_size_orig_.load());
	// ROS_INFO_STREAM("cloud_size_sample: " << cloud_preproc_size);

	// ModuleStatus curr_running_module_status = running_module_status_.load();
	// if (curr_running_module_status == ModuleStatus::MODULE_IDLE ||
	//     curr_running_module_status == ModuleStatus::MODULE_STARTING_SLAM ||
	//     curr_running_module_status == ModuleStatus::MODULE_STOPPING_SLAM){
	//     return;
	// }

	// slam_ -> lidar_pcl_cbk(sample_cld_ptr);
	slam_->lidar_pcl_cbk(cloud_preproc_ptr);
	// fill status_msg.localization_status
	double t3 = omp_get_wtime();

	double t100		= omp_get_wtime();
	auto   end		= std::chrono::system_clock::now();
	auto   duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	if ((t1 - t0) * 1000 > time_cost_thr_print) {
		// ROS_INFO_STREAM("lidar-callback, msg_2_pcl: "<< (t1 - t0)*1000 << " ms");
		// ROS_INFO_STREAM("lidar-callback, sampling : "<< (t2 - t1)*1000 << " ms");
		// ROS_INFO_STREAM("lidar-callback, push buff: "<< (t3 - t2)*1000 << " ms");
		// ROS_INFO_STREAM(YELLOW << "lidar-callback, time cost: "<< (t100 - t0)*1000 << " ms --------" <<RESET);
		RCLCPP_INFO(node_->get_logger(), "lidar-callback, time cost: %f ms --------", (t100 - t0) * 1000);
	}
}

// void LocalizationModule::imu_callback(const sensor_msgs::Imu::SharedPtr &msg_in)
void LocalizationModule::imu_callback(Imu::SharedPtr msg_in) {
	// hb_time_cbk_imu_.store(ros::WallTime::now().toSec());
	hb_time_cbk_imu_.store(node_->now().seconds());
	// ROS_INFO_ONCE("received imu -------------- imu cbk");
	// RCLCPP_INFO(node_->get_logger(),"received imu -------------- imu cbk" );

	// transfer IMU : IMU-frame to baselink-frame
	Eigen::Vector3d ang_before(msg_in->angular_velocity.x, msg_in->angular_velocity.y, msg_in->angular_velocity.z);
	Eigen::Vector3d acc_before(msg_in->linear_acceleration.x, msg_in->linear_acceleration.y,
							   msg_in->linear_acceleration.z);
	Eigen::Vector3d ang_after = slam_param_.extrinsic.R_baselink_IMU * ang_before;
	Eigen::Vector3d acc_after = slam_param_.extrinsic.R_baselink_IMU * acc_before;

	std::shared_ptr<livox_ros::ImuMsg> msg(new livox_ros::ImuMsg);
	//  msg->time_stamp = msg_in->header.stamp.toSec();
	msg->time_stamp = rclcpp::Time(msg_in->header.stamp).seconds();
	// msg->time_stamp = msg_in->header.stamp.toSec() + 28799.8614; temp, 测试万集雷达时用到

	if (slam_param_.lidar_preproc.lidar_type == 3) {
		// acc_after = acc_after / G_m_s2;
		acc_after = acc_after / 9.7;
	}
	// std::cout << RED << " acc_after : " << acc_after[0]<< " -- " << acc_after[1]<< " -- " << acc_after[2]
	// <<RESET<<std::endl;;

	// msg->angular_velocity << msg_in->angular_velocity.x,msg_in->angular_velocity.y,msg_in->angular_velocity.z;
	// msg->linear_acceleration <<
	// msg_in->linear_acceleration.x,msg_in->linear_acceleration.y,msg_in->linear_acceleration.z;
	msg->angular_velocity << ang_after[0], ang_after[1], ang_after[2];
	msg->linear_acceleration << acc_after[0], acc_after[1], acc_after[2];

	sensor_msgs::msg::Imu imu_in_base = *msg_in;
	imu_in_base.header.frame_id		  = "base_link";
	imu_in_base.angular_velocity.x	  = msg->angular_velocity.x();
	imu_in_base.angular_velocity.y	  = msg->angular_velocity.y();
	imu_in_base.angular_velocity.z	  = msg->angular_velocity.z();
	imu_in_base.linear_acceleration.x = msg->linear_acceleration.x();
	imu_in_base.linear_acceleration.y = msg->linear_acceleration.y();
	imu_in_base.linear_acceleration.z = msg->linear_acceleration.z();
	pub_base_imu_->publish(imu_in_base);

	////////////////////////////////////////////////////////////////////////////////
	// detect slip

	////////////////////////////////////////////////////////////////////////////////

	ModuleStatus curr_running_module_status = running_module_status_.load();

	if (curr_running_module_status == ModuleStatus::MODULE_IDLE ||
		curr_running_module_status == ModuleStatus::MODULE_STARTING_SLAM ||
		curr_running_module_status == ModuleStatus::MODULE_STOPPING_SLAM) {
		return;
	} else {
		slam_->imu_cbk(msg);
		return;
	}
}

void LocalizationModule::publish_unoptimized_path(const std::deque<Eigen::Isometry3d>& path, const std::string& frame) {
	geometry_msgs::msg::PoseStamped msg;

	unoptimized_path_msg.poses.clear();
	unoptimized_path_msg.header.stamp	 = node_->now();
	unoptimized_path_msg.header.frame_id = frame;

	for (int i = 0; i < path.size(); i++) {
		msg.header.stamp	= node_->now();
		msg.header.frame_id = frame;
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
	pubUnoptimizedPath->publish(unoptimized_path_msg);
}

void LocalizationModule::publish_optimized_path(const std::vector<Eigen::Isometry3d>& path, const std::string& frame) {
	geometry_msgs::msg::PoseStamped msg;
	// nav_msgs::msg::Path optimized_path_msg;

	optimized_path_msg.poses.clear();
	optimized_path_msg.header.stamp	   = node_->now();
	optimized_path_msg.header.frame_id = frame;

	for (size_t i = 0; i < path.size(); i++) {
		msg.header.stamp	= node_->now();
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
	pubOptimizedPath->publish(optimized_path_msg);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////--------------------------- Init Module -----------------------------////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

bool LocalizationModule::init_module_by_set_status(ModuleStatus set_status) {
	// set_module_status_ = set_status;
	int map_id = 1; /////////////// TODO
	if (set_status == ModuleStatus::MODULE_IDLE) {
		// ROS_INFO("init module status: %s", print_ModuleStatus(set_module_status_).c_str());
	} else if (set_status == ModuleStatus::MODULE_MAPPING) {
		if (start_mapping(map_id)) {
			// running_module_status_.store(ModuleStatus::MODULE_MAPPING);
		} else {
		}
	} else if (set_status == ModuleStatus::MODULE_SEC_MAPPING) {
		// TODO
		if (start_second_mapping(map_id)) {
			// running_module_status_.store(ModuleStatus::MODULE_SEC_MAPPING);
		} else {
			// set_module_status_ = running_module_status_;
			// release_slam_obj();
			// ROS_WARN_STREAM(YELLOW << "slam obj destroyed!"<< RESET);
			std::cout << YELLOW << "slam obj destroyed!" << RESET << std::endl;
		}
	} else if (set_status == ModuleStatus::MODULE_LOCALIZATION) {
		// TODO
		if (start_localization(map_id)) {
			// running_module_status_.store(ModuleStatus::MODULE_LOCALIZATION);
		} else {
		}
	}

	RCLCPP_INFO(node_->get_logger(), "init module status: %s",
				print_ModuleStatus(running_module_status_.load()).c_str());

	return true;
}

//---------------------------------------------------------------------------------------------------------

bool LocalizationModule::module_member_init() {
	// double curr_time = ros::WallTime::now().toSec();
	double curr_time = node_->now().seconds();
	hb_time_cbk_lidar_.store(curr_time);
	hb_time_cbk_imu_.store(curr_time);
	hb_time_cbk_module_ctrl_.store(curr_time);
	hb_time_timer_slam_.store(curr_time);
	hb_time_timer_pose_.store(curr_time);
	// hb_time_thread_localize_.store(curr_time);
	hb_time_thread_loop_closure_.store(curr_time);
	hb_time_thread_secmap_relocalize_.store(curr_time);

	health_status_.store(-1);

	mapping_node_status_.store(0);
	local_node_status_.store(0);

	mapping_status_.store(0);
	localization_status_.store(0);
	running_module_status_.store(ModuleStatus::MODULE_IDLE);

	// cloud_preproc_ptr_.reset(new PointCloudType());
	auto ring_count = slam_param_.lidar_preproc.cloud_ring_count;
	auto col_count	= slam_param_.lidar_preproc.cloud_column_count;
	// cloud_preproc_ptr_->points.reserve(ring_count * col_count);

	log_info_manager_ = LocalizationModuleLogInfoManager::getInstance();
	log_info_manager_->reset_log_info();

	// lidar reset , after param load
	lidar_ptr_ = LidarPreprocFactory::new_lidar_preproc(slam_param_.lidar_preproc.lidar_type, node_);
	// slipping_ptr_.reset(new DetectSlipping());

	return true;
}
//------------------------------------------- load params -------------------------------------------------

bool LocalizationModule::load_lidar_slam_param() {
	// init
	T_lidar_baselink_ = Eigen::Isometry3d::Identity();
	// LocalizationModuleParamManager *param_manager = LocalizationModuleParamManager::Instance();
	std::cout << "[ LocalizationModule::load_lidar_slam_param]: 11" << std::endl;
	LocalizationModuleParamManager* param_manager = LocalizationModuleParamManager::Instance(node_);
	std::cout << "[ LocalizationModule::load_lidar_slam_param]: 12" << std::endl;
	// const lidar_slam::LidarSlamParam* loaded_param = param_manager->get_loaded_param();
	// std::shared_ptr<const lidar_slam::LidarSlamParam> loaded_param = param_manager->get_loaded_param();
	const lidar_slam::LidarSlamParam& loaded_param = param_manager->get_loaded_param();
	std::cout << "[ LocalizationModule::load_lidar_slam_param]: 13" << std::endl;

	// if (loaded_param == NULL) {
	//     // ROS_ERROR_STREAM(RED << "loaded_param is NULL" <<RESET);
	//     RCLCPP_ERROR(node_-> get_logger(), "loaded_param is NULL");
	//     return false;
	// }else{
	// slam_param_ = *loaded_param;
	slam_param_		  = loaded_param;
	T_lidar_baselink_ = slam_param_.extrinsic.T_lidar_wheel;
	RCLCPP_INFO(node_->get_logger(), "loaded_param success!");
	return true;
	// }
}

bool LocalizationModule::is_mapping_status(ModuleStatus status) {
	if (status == ModuleStatus::MODULE_MAPPING || status == ModuleStatus::MODULE_SEC_MAPPING) {
		return true;
	} else {
		return false;
	}
}

} // namespace localization_module

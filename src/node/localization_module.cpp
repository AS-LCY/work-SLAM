#include "node/localization_module.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>

#define L_WHEEL 0.385f

namespace localization_module {

std::atomic<ModuleStatus> LocalizationModule::running_module_status_(ModuleStatus::MODULE_IDLE);
std::atomic<double> LocalizationModule::livox_cbk_update_time_(0.0);

LocalizationModule::LocalizationModule(rclcpp::Node::SharedPtr node, ModuleStatus init_status)
	: node_(node), br_(node_), tf_buffer_(node_->get_clock()), tf_listener_(tf_buffer_) {
	//**************************** 加载参数 ********************************
	if (!load_lidar_slam_param()) {
		TRACE_ERR_CLASS("Load lidar-slam param failed!");
	} else {
		TRACE_INFO_CLASS("Load lidar-slam param successfully!");
	}

	//**************************** CPU 绑定 *********************************
	CPU_ZERO(&cpu_mask_); // 初始化 CPU 亲和性集合，将其设置为零
	for (int i = 0; i < slam_param_.common.cpu_id.size(); i++) {
		CPU_SET(slam_param_.common.cpu_id[i], &cpu_mask_); // 将线程绑定到 cpu_id 核心
		TRACE_INFO_CLASS("set cpu: %d", static_cast<int>(slam_param_.common.cpu_id[i]));
	}

	//************** 初始化一些成员变量, after param load  ********************
	module_member_init();

	//**************************** 创建 ROS IO ******************************
	if (!create_ROS_IO()) {
		TRACE_ERR_CLASS("Create ROS-IO failed!");
	} else {
		TRACE_INFO_CLASS("Create ROS-IO successfully!");
	}

	//**************************** 根据设置参数初始化 module status ******************************
	if (!init_module_by_set_status(init_status)) {
		TRACE_ERR_CLASS("Try to init module with status: %s, but failed", print_ModuleStatus(init_status).c_str());
	} else {
		TRACE_INFO_CLASS("Localization Module Start with status: %s",
						 print_ModuleStatus(running_module_status_.load()).c_str());
	}

	ros_spinner_start();
}

LocalizationModule::~LocalizationModule() {}

float line_length(float dx, float dy) { return std::sqrt(dx * dx + dy * dy); }

bool LocalizationModule::create_ROS_IO() {
	if (!node_) {
		throw std::runtime_error("ROS node not initialized");
	}
	sub_pointcloud2_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
		slam_param_.lidar_preproc.sub_lidar_topic, 10,
		std::bind(&LocalizationModule::lidar_ros_callback, this, std::placeholders::_1));

	sub_imu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
		slam_param_.lidar_preproc.sub_imu_topic,
		rclcpp::QoS(2000), // ROS2中使用QoS替代简单的队列大小
		std::bind(&LocalizationModule::imu_callback, this, std::placeholders::_1));

	pub_localization_module_status_ = node_->create_publisher<fairland_msgs::msg::LocalizationModuleStatus>(
		slam_param_.common.pub_topic_module_status, rclcpp::QoS(100));

	pub_localization_module_health_ = node_->create_publisher<fairland_msgs::msg::LocalizationModuleHealth>(
		slam_param_.common.pub_topic_module_health, rclcpp::QoS(100));

	pub_log_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(slam_param_.common.pub_topic_module_loginfo,
																		 rclcpp::QoS(100));
	// pub_slip_ = nh_.advertise<fairland_msgs::NameValues>(slam_param_.common.pub_topic_slipping, 100);

	// both 建图 & 定位
	pubOdomAftMapped = node_->create_publisher<nav_msgs::msg::Odometry>("/Odometry_lidar_in_map", rclcpp::QoS(20));
	// T_map_baselink

	slam_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	this->timer_slam_ =
		node_->create_wall_timer(std::chrono::milliseconds(100), // 100ms = 10Hz  //TODO(jxl): 主线程应该多少ms周期运行
								 std::bind(&LocalizationModule::slam_dealt_timer, this), slam_callback_group_);

	ctrl_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	// 创建订阅器
	auto sub_options = rclcpp::SubscriptionOptions();
	sub_options.callback_group = ctrl_callback_group_;
	this->sub_mapping_ctrl_ = node_->create_subscription<std_msgs::msg::UInt32>(
		slam_param_.common.sub_topic_ctrl_cmd,
		rclcpp::QoS(3), // 保持队列大小为3
		std::bind(&LocalizationModule::localization_module_ctrl_callback, this, std::placeholders::_1), sub_options);

	// only 建图
	this->timer_module_status_ =
		node_->create_wall_timer(std::chrono::milliseconds(100), // 100ms = 10Hz
								 std::bind(&LocalizationModule::pub_module_status_timer, this), ctrl_callback_group_);

	// TODO: 还需要区分哪些是建图或定位发布的
	pubOdomCloud = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/odom_cloud", 10); // lio odom系下的点云

	pubBodyCloud = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/flbot/localization/baselink_cloud", 10);
	//转到和base_link系朝向一致的点云, 位置还在雷达位置处
	// TODO(jxl): 可以把点云转到base_link位置处

	// pub_body_cloud_filter_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
	// 	"/flbot/localization/body_cloud_filter", 20); //没有实际发布
	// pub_key_cloud_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/flbot/localization/key_body_cloud",
	// 20);
	// //当前帧如果是关键帧也发布， 为了实时性暂时屏蔽掉发布

	// pubObstacleCloud = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacle_cloud", 10); //没有实际发布
	// pubFilteredObstacleCloud =
	// 	node_->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_obstacle_cloud", 10);	  //没有实际发布
	// pubTestCloud = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/test_cloud", 10);	  //没有实际发布
	// pubKdtreeCloud = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/kdtree_cloud", 10); //没有实际发布

	pubOptimizedPath = node_->create_publisher<nav_msgs::msg::Path>(
		"/keyframe_baselink_in_map_path",
		10); // mapping或sec_mapping模式下，根据后端keyframe位姿(lidar位姿)在map系下，计算出的T_map_baselink

	// pubUnoptimizedPath = node_->create_publisher<nav_msgs::msg::Path>(
	// 	"/unoptimized_path", 10); //定位模式下：每一帧雷达pose在map系下； 建图模式下还是关键帧pose

	pubBaseLinkMapPath = node_->create_publisher<nav_msgs::msg::Path>("/baselink_in_map_path", 10);
	//(二次)建图，定位模式下，10hz的T_map_baselink

	pub_pose_graph_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/pose_graph/nodes_and_edges", 10);

	pubLoopConstraintEdge = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
		"/flbot/mapping/loop_closure_constraints", 1); //建图模式下：只发布闭环nodes和edges，没有整体pose graph结构

	pubKeyframePose = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/key_frame_pose", 1);
	//在定位模式下，发布之前建图结束后加载的关键帧位姿。
	// TODO(jxl): 定位模式下不关心关键帧，只有mapping或sec_mapping模式下才关心关键帧

	pubLoadMap = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/Load_map", 1); //每隔20s发布一次加载的地图

	// pubRgbCloud = node_->create_publisher<sensor_msgs::msg::PointCloud2>("rgb_cloud", 1); //没有发布
	// pub_base_imu_ =
	// 	node_->create_publisher<sensor_msgs::msg::Imu>("/flbot/localization/imu", 100); // base_link下的acc，gyro，

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

void LocalizationModule::slam_dealt_timer() { //主线程
	if (slam_param_.common.cpu_id.size() > 0) {
		pthread_t this_thread = pthread_self(); // 获取当前线程的 ID
		if (pthread_setaffinity_np(this_thread, sizeof(cpu_mask_), &cpu_mask_) < 0) {
			perror("pthread_setaffinity_np");
			exit(EXIT_FAILURE);
		}
	}

	static double last_slam_hb = hb_time_timer_slam_.load();
	hb_time_timer_slam_.store(node_->now().seconds());
	double slam_timer_interval = hb_time_timer_slam_.load() - last_slam_hb;

	log_info_manager_->slam_info.data[29] = slam_timer_interval;
	last_slam_hb = hb_time_timer_slam_.load();

	if (slam_timer_interval < 0) {
		TRACE_ERR_CLASS("slam main thread time jump back, this_time - last_time = %.3f seconds", slam_timer_interval);
	}
	if (slam_timer_interval > 0.5) { //两次loop之间时间超过0.5s
		TRACE_ERR_CLASS("slam main thread time jump to future, this_time - last_time = %.3f seconds",
						slam_timer_interval);
	}

	ModuleStatus curr_running_module_status = running_module_status_.load();
	// TRACE_INFO_CLASS("slam dealt: running module status: %s",
	// print_ModuleStatus(curr_running_module_status).c_str());

	if (curr_running_module_status == ModuleStatus::MODULE_IDLE ||
		curr_running_module_status == ModuleStatus::MODULE_STARTING_SLAM ||
		curr_running_module_status == ModuleStatus::MODULE_STOPPING_SLAM) {
		// TRACE_INFO_CLASS("do nothing");
		return;
	}

	if (curr_running_module_status == ModuleStatus::MODULE_MAPPING ||
		curr_running_module_status == ModuleStatus::MODULE_SEC_MAPPING ||
		curr_running_module_status == ModuleStatus::MODULE_LOCALIZATION) {
	}

	if (show_load_map_ % 200 == 0 && curr_running_module_status == ModuleStatus::MODULE_LOCALIZATION &&
		(slam_->getLoadMap()) && (slam_->getLoadMap())->points.size() > 0) {
		sensor_msgs::msg::PointCloud2 loadMap;
		pcl::toROSMsg(*(slam_->getLoadMap()), loadMap);
		loadMap.header.stamp = node_->now();
		loadMap.header.frame_id = "map";
		pubLoadMap->publish(loadMap);
		// TODO(jxl): 定位模式下，每隔20s发布一次加载的地图，没必要。可以只发布一次来可视化，在debug模式下。

		show_keyframe(slam_->getLoadKeyFrame());
		show_load_map_ = 0;
	}
	show_load_map_++;

	if (just_show_mode_) {
		return;
	}

	/********************************- run slam -********************************/

	// running_slam_flag==false 的情况: 1第一帧; 2无点云； 3点云数量太少；
	bool running_slam_flag = slam_->run();

	if (running_slam_flag) {
		if ((is_mapping_status(curr_running_module_status)) || slam_->isGloalLocalizationSuccess()) {
			// publish_cloud(slam_->get_odom_cloud(), "base_link", pubOdomCloud);
			// TODO(jxl): odom系下的点云，怎么frame_id是base_link？先注释掉
		}
		publish_cloud(slam_->get_baselink_cloud(), "base_link", pubBodyCloud);
		// process_loginfo();
	}

	auto localization_status_now = localization_status_.load();
	if (is_mapping_status(curr_running_module_status) &&
		mapping_status_.load() == MappingStatus::Standby) { // m_standby
		// if (slam_->get_new_key_cloud_arrived()) {
		// 	publish_cloud(slam_->get_lidar_cloud(), "lidar", pub_key_cloud_);
		// 	slam_->set_new_key_cloud_arrived(false);
		// }
		// publish_cloud(slam_->get_kdtree_cloud(), "mapping_odom", pubKdtreeCloud);

		publish_cloud(slam_->get_odom_cloud(), "odom", pubOdomCloud);

		visualizeLoopClosure(slam_->getloopIndex(),
							 optimized_path_msg); // TODO(jxl): 只发布了闭环nodes和edges， 整个pose graph结构看不到

		// publish_unoptimized_path(slam_->get_unoptimized_path(), string("map"));

		if (pub_pose_graph_->get_subscription_count() > 0) {
			const auto& keyframe_poses = slam_->getAllKeyframeBaselinkNodes(); // T_map_baselink
			const auto& all_loop_edges = slam_->getAllLoopEdges();
			visualizePoseGraph(keyframe_poses, all_loop_edges);
		}

		publish_optimized_path(slam_->get_optimized_path(), string("map"));
	} else if (curr_running_module_status == ModuleStatus::MODULE_LOCALIZATION &&
			   localization_status_is_ok(localization_status_now)) {
		if (slam_->isGloalLocalizationSuccess()) {
			// publish_odometry(slam_->getLidarInOdom(), pubOdomAftMapped);
			// TODO(jxl): 用新的接口发布T_map_lidar?
		}
	}

	// if (show_rviz_){
	//     publish_static_transform(slam_->getWheelInLidar());
	//     publish_odometry(slam_->getLidarInOdom(), pubOdomAftMapped);
	// }
}

//调试信息
/*
void LocalizationModule::process_loginfo() {
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
	last_lidar_in_odom_baselink.translation().x();
	// log_info_manager_->slam_info.data[20] = curr_lidar_in_odom_baselink.translation().y() -
	last_lidar_in_odom_baselink.translation().y();
	// log_info_manager_->slam_info.data[21] = curr_lidar_in_map_baselink.translation().x() -
	last_lidar_in_map_baselink.translation().x();
	// log_info_manager_->slam_info.data[22] = curr_lidar_in_map_baselink.translation().y() -
	last_lidar_in_map_baselink.translation().y();

	log_info_manager_->slam_info.data[19] = curr_lidar_in_odom.translation().x() - last_lidar_in_odom.translation().x();
	log_info_manager_->slam_info.data[20] = curr_lidar_in_odom.translation().y() - last_lidar_in_odom.translation().y();

	log_info_manager_->slam_info.data[21] = curr_lidar_in_map.translation().x() - last_lidar_in_map.translation().x();
	log_info_manager_->slam_info.data[22] = curr_lidar_in_map.translation().y() - last_lidar_in_map.translation().y();

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
	last_lidar_in_map = curr_lidar_in_map;
}*/

// 调试信息（健康状态）， 不影响程序运行
common_status::HealthStatus LocalizationModule::check_fill_health_msg(
	ModuleStatus curr_running_module_status, fairland_msgs::msg::LocalizationModuleHealth& health_msg) {
	static const double imu_interval = 0.005;
	static const double lidar_interval = 0.1;
	static const double slam_interval = 0.1;
	static const double pose_interval = 0.05;
	static const double localize_interval = 1.0;
	static const double loop_closure_interval = 1.0;
	static const double secmap_relocalize_interval = 1.0;
	static const int imu_ratio = 20; // 20
	static const int lidar_ratio = 3;
	static const int slam_ratio = 3;
	static const int pose_ratio = 3;
	static const int localize_ratio = 3;
	static const int loop_closure_ratio = 3;
	static const int secmap_relocalize_ratio = 3;
	static const int point_cloud_size_thr = 600;

	// check ROS IO status **********************************************************************
	HealthStatus health_status_now = HealthStatus::AllOk;

	auto curr_ros_time = node_->now();
	double curr_time = rclcpp::Time(curr_ros_time).seconds(); //////////////////TODO::
	double delay_imu = curr_time - hb_time_cbk_imu_.load();
	double delay_lidar = curr_time - hb_time_cbk_lidar_.load();
	double delay_slam = curr_time - hb_time_timer_slam_.load();
	double delay_pose = curr_time - hb_time_timer_pose_.load();
	bool hb_cbk_lidar = delay_lidar < lidar_interval * lidar_ratio ? true : false;
	bool hb_cbk_imu = delay_imu < imu_interval * imu_ratio ? true : false;
	bool hb_timer_slam = delay_slam < slam_interval * slam_ratio ? true : false;
	bool hb_timer_pose = delay_pose < pose_interval * pose_ratio ? true : false;
	bool hb_thread_localize = true;
	bool hb_thread_loop_closure = true;
	bool hb_thread_secmap_relocalize = true;
	bool error_lidar_point_too_few = false;
	bool error_livox_driver_failed = false;

	// if(curr_running_module_status == ModuleStatus::MODULE_IDLE){
	//     hb_cbk_lidar = 1;
	//     hb_cbk_imu = 1;
	// }

	if (!hb_cbk_lidar || !hb_cbk_imu || !hb_timer_slam || !hb_timer_pose) {
		health_status_now = HealthStatus::ErrorStop;
	}

	// check thread in slam.cpp ******************************************************************
	double localize_delay = 0.0;
	double loop_closure_delay = 0.0;
	double secmap_relocalize_delay = 0.0;
	if (curr_running_module_status == ModuleStatus::MODULE_MAPPING) {
		loop_closure_delay = curr_time - slam_->get_hb_time_thread_loop_closure();
		hb_thread_loop_closure = loop_closure_delay < loop_closure_interval * loop_closure_ratio ? true : false;
		if (!hb_thread_loop_closure) {
			health_status_now = static_cast<HealthStatus>(std::max(1, static_cast<int>(health_status_now)));
			mapping_node_status_.store(MappingNodeStatus::LoopClosureThreadDelay);
		}
	} else if (curr_running_module_status == ModuleStatus::MODULE_SEC_MAPPING) {
		loop_closure_delay = curr_time - slam_->get_hb_time_thread_loop_closure();
		secmap_relocalize_delay = curr_time - slam_->get_hb_time_thread_secmap_relocalize();
		hb_thread_loop_closure = loop_closure_delay < loop_closure_interval * loop_closure_ratio ? true : false;
		hb_thread_secmap_relocalize =
			secmap_relocalize_delay < secmap_relocalize_interval * secmap_relocalize_ratio ? true : false;
		if (!hb_thread_loop_closure || !hb_thread_secmap_relocalize) {
			health_status_now = static_cast<HealthStatus>(std::max(1, static_cast<int>(health_status_now)));
			mapping_node_status_.store(MappingNodeStatus::LoopClosureThreadDelay);
		}
	} else if (curr_running_module_status == ModuleStatus::MODULE_LOCALIZATION) {
		localize_delay = curr_time - slam_->get_hb_time_thread_localize();
		hb_thread_localize = localize_delay < localize_interval * localize_ratio ? true : false;
		if (!hb_thread_localize) {
			health_status_now = static_cast<HealthStatus>(std::max(1, static_cast<int>(health_status_now)));
			local_node_status_.store(LocalNodeStatus::LocalizeThreadDelay);
		}
	}
	// check lidar driver **************************************************************
	int orig_point_cloud_size = 0;
	int sample_point_cloud_size = 0;
	if (hb_cbk_lidar) {
		orig_point_cloud_size = cloud_size_orig_.load();
		sample_point_cloud_size = cloud_size_sample_.load();
		if (orig_point_cloud_size < point_cloud_size_thr) {
			error_lidar_point_too_few = true;
		}
	}
	if (slam_param_.lidar_preproc.lidar_type == 1 && orig_point_cloud_size == 96) {
		TRACE_ERR_CLASS("livox driver error, cloud-size: 96");
		error_livox_driver_failed = true;
		health_status_now = static_cast<HealthStatus>(std::max(2, static_cast<int>(health_status_now)));
	}

	// fill health msg *************************************************************************
	health_msg.cloud_size = orig_point_cloud_size;

	log_info_manager_->slam_info.data[12] = orig_point_cloud_size;	 //
	log_info_manager_->slam_info.data[14] = sample_point_cloud_size; //

	health_msg.delay_cbk_lidar = delay_lidar;							 // unit: s
	health_msg.delay_cbk_imu = delay_imu;								 // unit: s
	health_msg.delay_timer_slam = delay_slam;							 // unit: s
	health_msg.delay_timer_pose = delay_pose;							 // unit: s
	health_msg.delay_thread_localize = localize_delay;					 // unit: s
	health_msg.delay_thread_loop_closure = loop_closure_delay;			 // unit: s
	health_msg.delay_thread_secmap_relocalize = secmap_relocalize_delay; // unit: s

	health_msg.hb_cbk_lidar = hb_cbk_lidar;								  // value: [0] or [1]
	health_msg.hb_cbk_imu = hb_cbk_imu;									  // value: [0] or [1]
	health_msg.hb_timer_slam = hb_timer_slam;							  // value: [0] or [1]
	health_msg.hb_timer_pose = hb_timer_pose;							  // value: [0] or [1]
	health_msg.hb_thread_localize = hb_thread_localize;					  // value: [0] or [1]
	health_msg.hb_thread_loop_closure = hb_thread_loop_closure;			  // value: [0] or [1]
	health_msg.hb_thread_secmap_relocalize = hb_thread_secmap_relocalize; // value: [0] or [1]

	health_msg.error_lidar_point_too_few = error_lidar_point_too_few; // value: [0] or [1]
	health_msg.error_livox_driver_failed = error_livox_driver_failed; // value: [0] or [1]

	health_msg.health_status = static_cast<int>(health_status_now);

	return health_status_now;
}

// 发布健康状态，节点工作状态（重点）
void LocalizationModule::pub_module_status_timer() {
	// ******************************************************************************************
	auto curr_running_module_status = running_module_status_.load();
	auto curr_ros_time = node_->now();

	fairland_msgs::msg::LocalizationModuleHealth health_msg;
	HealthStatus health_status_now = check_fill_health_msg(curr_running_module_status, health_msg);
	health_msg.header.stamp = curr_ros_time;
	health_msg.header.frame_id = "base_link";
	health_status_.store(health_status_now);

	fairland_msgs::msg::LocalizationModuleStatus status_msg;
	check_fill_module_status_msg(curr_running_module_status, status_msg);
	status_msg.header.stamp = curr_ros_time;
	status_msg.header.frame_id = "base_link";

	pub_localization_module_status_->publish(status_msg);
	pub_localization_module_health_->publish(health_msg);
	pub_log_->publish(log_info_manager_->slam_info);
}

//局部函数
geometry_msgs::msg::TransformStamped initIdentityTransform() {
	geometry_msgs::msg::TransformStamped transform;

	// 初始化header
	transform.header.stamp = rclcpp::Clock().now();
	transform.header.frame_id = "odom";
	transform.child_frame_id = "base_link";

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
	if (curr_running_module_status == ModuleStatus::MODULE_IDLE) {
		status_msg.module_status = int(ModuleStatus_o::IDLE);
	} else if (is_mapping_status(curr_running_module_status)) {
		status_msg.module_status = int(ModuleStatus_o::MAPPING);
	} else if (curr_running_module_status == ModuleStatus::MODULE_LOCALIZATION) {
		status_msg.module_status = int(ModuleStatus_o::LOCALIZATION);
	} else if (curr_running_module_status == ModuleStatus::MODULE_STARTING_SLAM) {
		status_msg.module_status = int(ModuleStatus_o::STARTING);
	} else if (curr_running_module_status == ModuleStatus::MODULE_STOPPING_SLAM) {
		status_msg.module_status = int(ModuleStatus_o::STOPPING);
	} else {
		TRACE_ERR_CLASS("error running module status: %s", print_ModuleStatus(curr_running_module_status).c_str());
	}

	// status_msg.localization_status
	fill_module_l_status(curr_running_module_status, status_msg);

	// status_msg.mapping_status
	fill_module_m_status(curr_running_module_status, status_msg);

	status_msg.map_saved = 0;
	if (curr_running_module_status == ModuleStatus::MODULE_IDLE && map_saved_.load() == 1) {
		status_msg.map_saved = 1;
		TRACE_WARN_CLASS("[Status Timer]: status_msg.map_saved: %d", int(status_msg.map_saved));
		map_saved_.store(0);
	}

	if (status_msg.localization_status != 0 && status_msg.localization_status != 3) {
		TRACE_WARN_CLASS("[Status Timer]: localization_status: %d", int(status_msg.localization_status));
	}
	if (status_msg.mapping_status != 0 && status_msg.mapping_status != 3) {
		TRACE_WARN_CLASS("[Status Timer]: mapping_status: %d", int(status_msg.mapping_status));
	}

	auto localization_ok = curr_running_module_status == ModuleStatus::MODULE_LOCALIZATION &&
						   localization_status_is_ok(localization_status_.load());
	auto mapping_ok = is_mapping_status(curr_running_module_status) && mapping_status_is_ok(mapping_status_.load());
	if (localization_ok || mapping_ok) {
		auto T_map_baselink = slam_->getLidarInMap() * T_lidar_baselink_;
		auto T_odom_imu_updated = slam_->get_localization_base();
		publish_odometry_lidar_in_map(T_map_baselink, T_odom_imu_updated, "map", "base_link",
									  curr_running_module_status);
		publish_OdomToMap_tf(slam_->getOdomToMap());
	}
}

void LocalizationModule::fill_module_l_status(ModuleStatus curr_running_module_status,
											  fairland_msgs::msg::LocalizationModuleStatus& status_msg) {
	if (curr_running_module_status != ModuleStatus::MODULE_LOCALIZATION) {
		status_msg.localization_status = static_cast<int>(LocalizationStatus::Inactive);
		localization_status_.store(LocalizationStatus::Inactive);
		return;
	}

	auto last_local_status = localization_status_.load();
	if (last_local_status == LocalizationStatus::RelocalizeFailed || last_local_status == LocalizationStatus::Failed) {
		status_msg.localization_status = static_cast<int>(last_local_status);
		log_info_manager_->slam_info.data[2] = static_cast<int>(last_local_status);
		return;
	}

	auto node_status = local_node_status_.load();
	auto local_thrd_status = slam_->get_local_thrd_status();
	auto slam_run_status = slam_->get_slam_run_status();
	static const bool check_delay = slam_param_.common.check_delay;
	if (!check_delay) {
		if (node_status == LocalNodeStatus::LidarCallbackDelay || node_status == LocalNodeStatus::LocalizeThreadDelay) {
			node_status = LocalNodeStatus::Normal;
		}
	}

	if (node_status == LocalNodeStatus::Inactive) {
		status_msg.localization_status = static_cast<int>(LocalizationStatus::Inactive);
		localization_status_.store(LocalizationStatus::Inactive);
	} else if (node_status == LocalNodeStatus::Normal) {
		if (slam_run_status == SlamRunStatus::Normal) {
			status_msg.localization_status = static_cast<int>(local_thrd_status);
			localization_status_.store(local_thrd_status); // 与 定位线程的状态一致
		} else if (slam_run_status == SlamRunStatus::SlamFail) {
			status_msg.localization_status = static_cast<int>(LocalizationStatus::Failed);
			localization_status_.store(LocalizationStatus::Failed);
		}
	} else if (node_status == LocalNodeStatus::LidarCallbackDelay) {
		TRACE_ERR_CLASS("lidar cbk delay !!!");
		status_msg.localization_status = static_cast<int>(LocalizationStatus::Failed);
		localization_status_.store(LocalizationStatus::Failed);
	} else if (node_status == LocalNodeStatus::LocalizeThreadDelay) {
		TRACE_ERR_CLASS("localize thread delay  !!!");
		status_msg.localization_status = static_cast<int>(LocalizationStatus::Failed);
		localization_status_.store(LocalizationStatus::Failed);
		TRACE_ERR_CLASS("localization status error, set to Failed");
		TRACE_ERR_CLASS("node_status: %s", magic_enum::enum_name(node_status));
		TRACE_ERR_CLASS("slam_run_status:%s", magic_enum::enum_name(slam_run_status));
		TRACE_ERR_CLASS("local_thrd_status:%s", magic_enum::enum_name(local_thrd_status));

		status_msg.localization_status = static_cast<int>(LocalizationStatus::Failed);
		localization_status_.store(LocalizationStatus::Failed);
	}

	log_info_manager_->slam_info.data[2] = static_cast<int>(localization_status_.load());
}

void LocalizationModule::fill_module_m_status(ModuleStatus curr_running_module_status,
											  fairland_msgs::msg::LocalizationModuleStatus& status_msg) {
	if (curr_running_module_status != ModuleStatus::MODULE_MAPPING &&
		curr_running_module_status != ModuleStatus::MODULE_SEC_MAPPING) {
		status_msg.mapping_status = static_cast<int>(MappingStatus::Inactive);
		mapping_status_.store(MappingStatus::Inactive);
		return;
	}

	auto last_mapping_status = mapping_status_.load();
	if (last_mapping_status == MappingStatus::RelocalizeFailed || last_mapping_status == MappingStatus::Failed) {
		status_msg.mapping_status = static_cast<int>(last_mapping_status);
		// log_info_manager_->slam_info.data[2]= static_cast<int>(last_mapping_status);
		return;
	}

	auto node_status = mapping_node_status_.load();
	auto slam_run_status = slam_->get_slam_run_status();
	auto secmap_relocal_thrd_status = slam_->get_secmap_relocal_thrd_status();
	static const bool check_delay = slam_param_.common.check_delay;
	if (!check_delay) {
		if (node_status == MappingNodeStatus::LidarCallbackDelay ||
			node_status == MappingNodeStatus::SecMapRelocalThreadDelay ||
			node_status == MappingNodeStatus::LoopClosureThreadDelay) {
			node_status = MappingNodeStatus::Normal;
		}
	}

	if (node_status == MappingNodeStatus::Inactive) {
		status_msg.mapping_status = static_cast<int>(MappingStatus::Inactive);
		mapping_status_.store(MappingStatus::Inactive);
	} else if (node_status == MappingNodeStatus::Normal) {
		if (slam_run_status == SlamRunStatus::Normal) {
			if (curr_running_module_status == ModuleStatus::MODULE_MAPPING) {
				status_msg.mapping_status = static_cast<int>(MappingStatus::Standby);
				mapping_status_.store(MappingStatus::Standby);
			} else if (curr_running_module_status == ModuleStatus::MODULE_SEC_MAPPING) {
				status_msg.mapping_status = static_cast<int>(secmap_relocal_thrd_status);
				MappingStatus mapping_status =
					magic_enum::enum_cast<MappingStatus>(static_cast<int>(secmap_relocal_thrd_status)).value();
				mapping_status_.store(mapping_status);
			}
		} else if (slam_run_status == SlamRunStatus::SlamFail) {
			status_msg.mapping_status = static_cast<int>(MappingStatus::Failed);
			mapping_status_.store(MappingStatus::Failed);
		}
	} else if (node_status == MappingNodeStatus::LidarCallbackDelay) {
		TRACE_WARN_CLASS("lidar cbk delay !!!");
		status_msg.mapping_status = static_cast<int>(MappingStatus::Failed);
		mapping_status_.store(MappingStatus::Failed);
	} else if (node_status == MappingNodeStatus::SecMapRelocalThreadDelay) {
		TRACE_ERR_CLASS("secmap-relocal thread delay  !!!");
		status_msg.mapping_status = static_cast<int>(MappingStatus::Failed);
		mapping_status_.store(MappingStatus::Failed);
	} else if (node_status == MappingNodeStatus::LoopClosureThreadDelay) {
		TRACE_ERR_CLASS("loop_closure_thread_delay  !!!");
		status_msg.mapping_status = static_cast<int>(MappingStatus::Failed);
		mapping_status_.store(MappingStatus::Failed);
	} else {
		TRACE_ERR_CLASS("mapping status error, set to Failed");
		TRACE_ERR_CLASS("node_status:%s ", magic_enum::enum_name(node_status));
		TRACE_ERR_CLASS("slam_run_status: :%s ", magic_enum::enum_name(slam_run_status));
		TRACE_ERR_CLASS("secmap_relocal_thrd_status::%s ", magic_enum::enum_name(secmap_relocal_thrd_status));
		status_msg.mapping_status = static_cast<int>(MappingStatus::Failed);
		mapping_status_.store(MappingStatus::Failed);
	}
}

void LocalizationModule::lidar_ros_callback(const PointCloud2::SharedPtr ros_msg) {
	static const int cloud_size_to_keep = slam_param_.lidar_preproc.cloud_size_to_keep;
	static const double time_cost_thr_print = slam_param_.lidar_preproc.time_cost_thr_print;

	static double last_lidar_hb = hb_time_cbk_lidar_;
	hb_time_cbk_lidar_.store(node_->now().seconds());
	log_info_manager_->slam_info.data[23] = hb_time_cbk_lidar_ - last_lidar_hb;
	last_lidar_hb = hb_time_cbk_lidar_;

	if (slam_param_.common.cpu_id.size() > 0) {
		pthread_t this_thread = pthread_self(); // 获取当前线程的 ID
		if (pthread_setaffinity_np(this_thread, sizeof(cpu_mask_), &cpu_mask_) < 0) {
			perror("pthread_setaffinity_np");
			exit(EXIT_FAILURE);
		}
	}

	livox_cbk_update_time_.store(rclcpp::Time(ros_msg->header.stamp).seconds());

	ModuleStatus curr_running_module_status = running_module_status_.load();
	if (curr_running_module_status == ModuleStatus::MODULE_IDLE ||
		curr_running_module_status == ModuleStatus::MODULE_STARTING_SLAM ||
		curr_running_module_status == ModuleStatus::MODULE_STOPPING_SLAM) {
		return;
	}

	double t0 = omp_get_wtime();
	auto start = std::chrono::system_clock::now();

	static const auto ring_count = slam_param_.lidar_preproc.cloud_ring_count;
	static const auto col_count = slam_param_.lidar_preproc.cloud_column_count;
	PointCloudType::Ptr cloud_preproc_ptr(new PointCloudType());
	cloud_preproc_ptr->points.reserve(ring_count * col_count);
	lidar_ptr_->pre_process(ros_msg, cloud_preproc_ptr); // 降采样，去NAN
	int cloud_preproc_size = cloud_preproc_ptr->points.size();

	double t1 = omp_get_wtime();
	slam_->lidar_pcl_cbk(cloud_preproc_ptr); // 传入降采样后的点云给算法
	double t100 = omp_get_wtime();

	auto end = std::chrono::system_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

	if ((t1 - t0) * 1000 > time_cost_thr_print) { // 10ms
		TRACE_DBG_CLASS("lidar pre_process, time cost: %f ms --------", (t100 - t0) * 1000);
	}
}

void LocalizationModule::imu_callback(Imu::SharedPtr msg_in) {
	hb_time_cbk_imu_.store(node_->now().seconds());

	// transfer IMU : IMU-frame to baselink-frame
	Eigen::Vector3d ang_before(msg_in->angular_velocity.x, msg_in->angular_velocity.y, msg_in->angular_velocity.z);
	Eigen::Vector3d acc_before(msg_in->linear_acceleration.x, msg_in->linear_acceleration.y,
							   msg_in->linear_acceleration.z);
	Eigen::Vector3d ang_after = slam_param_.extrinsic.R_baselink_IMU * ang_before;
	Eigen::Vector3d acc_after = slam_param_.extrinsic.R_baselink_IMU * acc_before;

	std::shared_ptr<livox_ros::ImuMsg> msg(new livox_ros::ImuMsg);
	msg->time_stamp = rclcpp::Time(msg_in->header.stamp).seconds();
	// msg->time_stamp = msg_in->header.stamp.toSec() + 28799.8614; temp, 测试万集雷达时用到

	if (slam_param_.lidar_preproc.lidar_type == 3) {
		acc_after = acc_after / G_m_s2;
	}

	msg->angular_velocity << ang_after[0], ang_after[1], ang_after[2];
	msg->linear_acceleration << acc_after[0], acc_after[1], acc_after[2];

	// sensor_msgs::msg::Imu imu_in_base = *msg_in;
	// imu_in_base.header.frame_id = "base_link";
	// imu_in_base.angular_velocity.x = msg->angular_velocity.x();
	// imu_in_base.angular_velocity.y = msg->angular_velocity.y();
	// imu_in_base.angular_velocity.z = msg->angular_velocity.z();
	// imu_in_base.linear_acceleration.x = msg->linear_acceleration.x();
	// imu_in_base.linear_acceleration.y = msg->linear_acceleration.y();
	// imu_in_base.linear_acceleration.z = msg->linear_acceleration.z();
	// pub_base_imu_->publish(imu_in_base);

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

// void LocalizationModule::publish_unoptimized_path(
// 	const std::deque<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& path, const std::string& frame) {
// 	geometry_msgs::msg::PoseStamped msg;

// 	unoptimized_path_msg.poses.clear();
// 	unoptimized_path_msg.header.stamp = node_->now();
// 	unoptimized_path_msg.header.frame_id = frame;

// 	for (int i = 0; i < path.size(); i++) {
// 		msg.header.stamp = node_->now();
// 		msg.header.frame_id = frame;
// 		msg.pose.position.x = path[i].translation().x();
// 		msg.pose.position.y = path[i].translation().y();
// 		msg.pose.position.z = path[i].translation().z();
// 		/*Eigen::Quaterniond quaternion = path[i].rotation();
// 		msg.pose.orientation.x = quaternion.x();
// 		msg.pose.orientation.y = quaternion.y();
// 		msg.pose.orientation.z = quaternion.z();
// 		msg.pose.orientation.w = quaternion.w();*/
// 		unoptimized_path_msg.poses.push_back(msg);
// 	}
// 	pubUnoptimizedPath->publish(unoptimized_path_msg);
// }

void LocalizationModule::publish_optimized_path(
	const std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& path, const std::string& frame) {
	geometry_msgs::msg::PoseStamped msg;

	optimized_path_msg.poses.clear();
	optimized_path_msg.header.stamp = node_->now();
	optimized_path_msg.header.frame_id = frame;

	for (size_t i = 0; i < path.size(); i++) {
		msg.header.stamp = node_->now();
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

bool LocalizationModule::init_module_by_set_status(ModuleStatus set_status) {
	// set_module_status_ = set_status;
	int map_id = 1;
	if (set_status == ModuleStatus::MODULE_IDLE) {
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
		}
	} else if (set_status == ModuleStatus::MODULE_LOCALIZATION) {
		// TODO
		if (start_localization(map_id)) {
			// running_module_status_.store(ModuleStatus::MODULE_LOCALIZATION);
		} else {
		}
	}

	TRACE_INFO_CLASS("init module status: %s", print_ModuleStatus(running_module_status_.load()).c_str());

	return true;
}

bool LocalizationModule::module_member_init() {
	double curr_time = node_->now().seconds();
	hb_time_cbk_lidar_.store(curr_time);
	hb_time_cbk_imu_.store(curr_time);
	hb_time_cbk_module_ctrl_.store(curr_time);
	hb_time_timer_slam_.store(curr_time);
	hb_time_timer_pose_.store(curr_time);
	// hb_time_thread_localize_.store(curr_time);
	hb_time_thread_loop_closure_.store(curr_time);
	hb_time_thread_secmap_relocalize_.store(curr_time);

	health_status_.store(HealthStatus::AllOk);

	mapping_node_status_.store(MappingNodeStatus::Inactive);
	local_node_status_.store(LocalNodeStatus::Inactive);

	mapping_status_.store(MappingStatus::Inactive);
	localization_status_.store(LocalizationStatus::Inactive);
	running_module_status_.store(ModuleStatus::MODULE_IDLE);

	// cloud_preproc_ptr_.reset(new PointCloudType());
	auto ring_count = slam_param_.lidar_preproc.cloud_ring_count;
	auto col_count = slam_param_.lidar_preproc.cloud_column_count;
	// cloud_preproc_ptr_->points.reserve(ring_count * col_count);

	log_info_manager_ = LocalizationModuleLogInfoManager::getInstance();
	log_info_manager_->reset_log_info();

	// lidar reset , after param load
	lidar_ptr_ = LidarPreprocFactory::new_lidar_preproc(slam_param_.lidar_preproc.lidar_type, node_);
	// slipping_ptr_.reset(new DetectSlipping());

	return true;
}

bool LocalizationModule::load_lidar_slam_param() {
	T_lidar_baselink_ = Eigen::Isometry3d::Identity();
	LocalizationModuleParamManager* param_manager = LocalizationModuleParamManager::Instance(node_);
	const lidar_slam::LidarSlamParam& loaded_param = param_manager->get_loaded_param();
	slam_param_ = loaded_param;
	T_lidar_baselink_ = slam_param_.extrinsic.T_lidar_wheel;
	TRACE_INFO_CLASS("loaded_param success!");
	return true;
}

bool LocalizationModule::is_mapping_status(ModuleStatus status) {
	if (status == ModuleStatus::MODULE_MAPPING || status == ModuleStatus::MODULE_SEC_MAPPING) {
		return true;
	} else {
		return false;
	}
}

} // namespace localization_module

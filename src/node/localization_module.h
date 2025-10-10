#ifndef LOCALIZATION_MODULE_H
#define LOCALIZATION_MODULE_H

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "boost/thread.hpp"

// ROS2 headers
#include <pcl_conversions/pcl_conversions.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

// ROS2 message headers
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Eigen
#include <Eigen/Core>

// Custom messages
#include "flbot_msgs/msg/chassis_data.hpp"
#include "flbot_msgs/msg/livox_custom_msg.hpp"
#include "flbot_msgs/msg/localization_module_health.hpp"
#include "flbot_msgs/msg/localization_module_log_info.hpp"
#include "flbot_msgs/msg/localization_module_status.hpp"
#include "flbot_msgs/msg/name_values.hpp"

// Project headers
#include "lidar/livox/ros_livox_datatype_def.h"
#include "lidar_slam/common_lib.h"
#include "lidar_slam/lidar_slam.hpp"
#include "magic_enum/magic_enum.hpp"
#include "node/log_info_manager.hpp"
#include "node/module_param_def.h"
#include "node/module_status_def.h"
#include "node/param_manager.hpp"

// Lidar headers
#include "lidar/hesai/lidar_preproc_JT16.h"
#include "lidar/hesai/pcl_point_type_def_hs.h"
#include "lidar/lanhai/lidar_preproc_M300.h"
#include "lidar/lanhai/pcl_point_type_def_bs.h"
#include "lidar/lidar_preproc_factory.hpp"
#include "lidar/lidar_preproc_parent.h"
#include "lidar/livox/lidar_preproc_Mid360.h"
#include "lidar/livox/pcl_point_type_def_lvx.h"
#include "lidar/robosense/lidar_preproc_Airy.h"
#include "lidar/robosense/pcl_point_type_def_rs.h"

namespace localization_module {

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace std_msgs::msg;
using namespace sensor_msgs::msg;
using namespace lidar_slam;
using namespace flbot_msgs::msg;

// ROS2 消息类型别名
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using Odometry = nav_msgs::msg::Odometry;
using Path = nav_msgs::msg::Path;
using Imu = sensor_msgs::msg::Imu;
using UInt32 = std_msgs::msg::UInt32;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;

// 自定义消息类型别名
using LivoxCustomMsg = flbot_msgs::msg::LivoxCustomMsg;
using LocalizationModuleStatus = flbot_msgs::msg::LocalizationModuleStatus;
using LocalizationModuleHealth = flbot_msgs::msg::LocalizationModuleHealth;
using NameValues = flbot_msgs::msg::NameValues;
using ChassisData = flbot_msgs::msg::ChassisData;

enum SlamCtrlCmd {
	START_MAPPING = 1000,
	START_SEC_MAPPING = 2000,
	CANCLE_MAPPING = 5000,
	SAVE_AND_END_MAPPING = 6000,
	START_LOCALIZATION = 7000,
	EXIT_LOCALIZATION = 8000,
	START_RELOCALIZATION = 9000,
	RESTART_SEC_MAPPING = 9100,
	// MAPPING_POINT_BEGIN     = 3000,  // 设置起点
	// MAPPING_ELE_DELETE      = 4000,  // 创建地图元素过程中，清除当前元素（当前元素还未完成创建）
	// MAPPING_POINT_END       = 5000,  // 设置终点
	CMD_MAX = 9999
};

class LocalizationModule {
	DECL_CLASSNAME(LocalizationModule)
   public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using HealthStatus = common_status::HealthStatus;
	using LocalizationStatus = common_status::LocalizationStatus;
	using LocalNodeStatus = common_status::LocalNodeStatus;
	using MappingNodeStatus = common_status::MappingNodeStatus;
	using MappingStatus = common_status::MappingStatus;
	using SecmapRelocalThrdStatus = common_status::SecmapRelocalThrdStatus;
	using SlamRunStatus = common_status::SlamRunStatus;

	LocalizationModule(rclcpp::Node::SharedPtr node, ModuleStatus init_status);
	~LocalizationModule();

   private:
	bool is_mapping_status(ModuleStatus status);
	bool need_start_localization(ModuleStatus running_module_status_now, LocalizationStatus localiztion_status_now);
	bool localization_status_is_ok(LocalizationStatus localiztion_status_now);
	bool localization_status_is_failed(LocalizationStatus localiztion_status_now);
	bool mapping_status_is_ok(MappingStatus mapping_status_now);
	bool mapping_status_is_failed(MappingStatus mapping_status_now);

	bool module_member_init();
	bool load_lidar_slam_param();
	bool create_ROS_IO();
	void ros_spinner_start();

	// 建图
	// bool mark_start_point();
	// bool mark_end_point(int save_id);
	// bool clear_curr_element();

	bool make_map_directory_name(int map_id);

	bool start_mapping(int map_id);
	bool start_second_mapping(int map_id);
	bool stop_mapping();
	bool save_extrinsic_to_file();
	bool start_localization(int map_id);

	bool stop_localization();

	bool start_relocalization(int map_id); // = restart_localization;;
	bool restart_second_mapping(int map_id);
	bool stop_mapping_without_saving_map();

	bool init_module_by_set_status(ModuleStatus set_status);

	bool make_slam_obj(lidar_slam::LidarSlamParam yaml_param, ModuleStatus set_status);

	void release_slam_obj();

	// Updated callback signatures
	void localization_module_ctrl_callback(const std_msgs::msg::UInt32::SharedPtr msg_in);

	void slam_dealt_timer();

	void pub_module_status_timer();

	void imu_callback(Imu::SharedPtr msg_in);
	void lidar_ros_callback(const PointCloud2::SharedPtr ros_msg);

	void publish_optimized_path(const std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& path,
								const std::string& frame);

	// 发布点云
	void publish_cloud(const double& cloud_time, PointCloudType::Ptr pcl_cloud_in, const std::string& frame_id,
					   rclcpp::Publisher<PointCloud2>::SharedPtr pub);

	// 发布地图中的激光雷达位姿
	void publish_odometry_lidar_in_map(const double& lidar_in_map_time, const Eigen::Isometry3d& lidar_in_map,
									   const lidar_slam::Localization_base& T_odom_imu, const std::string& frameid,
									   const std::string& child_frameid, ModuleStatus curr_running_module_status);
	void publish_OdomToMap_tf(const double& lidar_in_map_time, const Eigen::Isometry3d& T_map_odom);

	void process_loginfo();

	void visualizePoseGraph(const std::vector<KeyPose>& poses, const std::vector<std::pair<int, int>>& loop_edges);

	// void show_keyframe(
	// 	const std::vector<lidar_slam::ScInfo, Eigen::aligned_allocator<lidar_slam::ScInfo>>& loadKeyframe);

	void fill_log(const Eigen::Isometry3d& last_lidar_in_odom, const Eigen::Isometry3d& curr_lidar_in_odom);

	// 检查并填充健康消息
	common_status::HealthStatus check_fill_health_msg(ModuleStatus curr_running_module_status,
													  LocalizationModuleHealth& health_msg);

	// 检查并填充模块状态消息
	void check_fill_module_status_msg(ModuleStatus curr_running_module_status, LocalizationModuleStatus& status_msg);

	// 填充定位状态
	void fill_module_l_status(ModuleStatus curr_running_module_status, LocalizationModuleStatus& status_msg);

	// 填充建图状态
	void fill_module_m_status(ModuleStatus curr_running_module_status, LocalizationModuleStatus& status_msg);

	string print_SlamCtrlCmd(SlamCtrlCmd e) {
		switch (e) {
			CASE_STR(START_MAPPING);
			CASE_STR(START_SEC_MAPPING);
			// CASE_STR(MAPPING_POINT_BEGIN);
			// CASE_STR(MAPPING_ELE_DELETE);
			// CASE_STR(MAPPING_POINT_END);
			CASE_STR(CANCLE_MAPPING);
			CASE_STR(SAVE_AND_END_MAPPING);
			CASE_STR(START_LOCALIZATION);
			CASE_STR(EXIT_LOCALIZATION);
			CASE_STR(START_RELOCALIZATION);
			CASE_STR(RESTART_SEC_MAPPING);
			CASE_STR(CMD_MAX);
			default:
				break;
		}
		return "UNKNOW_SlamCtrlCmd!";
	}

   private:
	rclcpp::Node::SharedPtr node_;

	std::atomic<LocalNodeStatus> local_node_status_{ LocalNodeStatus::Inactive };
	std::atomic<MappingNodeStatus> mapping_node_status_{ MappingNodeStatus::Inactive };

	// 各线程、callback、timer heartbeat
	// std::atomic<double> hb_time_cbk_imu_;
	// std::atomic<double> hb_time_cbk_lidar_;
	std::atomic<double> hb_time_cbk_module_ctrl_;
	std::atomic<double> hb_time_timer_slam_;

	double imu_msg_interval_ = 0.005;
	double lidar_msg_interval_ = 0.1;

	double delay_imu_ = 0.f;
	double delay_lidar_ = 0.f;

	std::atomic<double> hb_time_thread_loop_closure_;
	std::atomic<double> hb_time_thread_secmap_relocalize_;
	std::atomic<HealthStatus> health_status_{ HealthStatus::AllOk };
	std::atomic<int> cloud_size_after_preprocess_{ 0 };

	Eigen::Isometry3d T_lidar_baselink_;

	// ROS2 接口
	rclcpp::Publisher<LocalizationModuleStatus>::SharedPtr pub_localization_module_status_;
	rclcpp::Publisher<LocalizationModuleHealth>::SharedPtr pub_localization_module_health_;
	rclcpp::Publisher<Float64MultiArray>::SharedPtr pub_log_;

	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud2_;
	rclcpp::Subscription<Imu>::SharedPtr sub_imu_;
	rclcpp::Subscription<UInt32>::SharedPtr sub_mapping_ctrl_;

	rclcpp::TimerBase::SharedPtr timer_slam_;
	rclcpp::TimerBase::SharedPtr timer_module_status_;

	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

	rclcpp::CallbackGroup::SharedPtr slam_callback_group_;
	rclcpp::CallbackGroup::SharedPtr ctrl_callback_group_;

	std::unique_ptr<lidar_slam::LidarSlam> slam_;
	bool releasing_slam_flag_ = false;

	// 通用
	string curr_dir_; // localization_module CMake dir
	bool localization_mode_ = false;
	bool offline_mode_ = false; // unused

	bool show_rviz_ = false;
	bool fast_mode_ = false; // unused, 只有参数读入
	string log_folder_;

	// show thread
	// lidar_slam::Control_status control_status_;

	// 运行模式
	static std::atomic<ModuleStatus> running_module_status_;

	std::atomic<MappingStatus> mapping_status_{ MappingStatus::Inactive };

	/** @map_saved_:
	 * 0: map not saved yet
	 * 1: map already saved (only set to 1 when stop mapping with saving map)
	 */
	std::atomic<int> map_saved_{ 0 };

	// int start_index_ = -1; // not used now, 目前不涉及创建元素的操作
	// int end_index_ = -1; // not used now, 目前不涉及创建元素的操作

	std::atomic<LocalizationStatus> localization_status_{ LocalizationStatus::Inactive };

	bool global_map_pubed_ = false;

	cpu_set_t cpu_mask_;

	// 点云和里程计发布者
	rclcpp::Publisher<Odometry>::SharedPtr pubLidarInMap = nullptr;
	rclcpp::Publisher<PointCloud2>::SharedPtr pubOdomCloud = nullptr;
	rclcpp::Publisher<PointCloud2>::SharedPtr pubBodyCloud = nullptr;
	rclcpp::Publisher<PointCloud2>::SharedPtr pubObstacleCloud = nullptr;
	rclcpp::Publisher<PointCloud2>::SharedPtr pubFilteredObstacleCloud = nullptr;
	rclcpp::Publisher<PointCloud2>::SharedPtr pubTestCloud = nullptr;
	rclcpp::Publisher<PointCloud2>::SharedPtr pubKdtreeCloud = nullptr;
	rclcpp::Publisher<Path>::SharedPtr pubOptimizedPath = nullptr;
	rclcpp::Publisher<Path>::SharedPtr pubUnoptimizedPath = nullptr;
	rclcpp::Publisher<Path>::SharedPtr pubBaseLinkMapPath = nullptr;
	rclcpp::Publisher<Path>::SharedPtr pubBaseLinkOdompPath = nullptr;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge = nullptr;
	rclcpp::Publisher<Odometry>::SharedPtr pubOdomAftMapped = nullptr;
	rclcpp::Publisher<Odometry>::SharedPtr pubLioOdom = nullptr;
	rclcpp::Publisher<Odometry>::SharedPtr pubLioOdomImu = nullptr;

	rclcpp::Publisher<PointCloud2>::SharedPtr pubLoadMap = nullptr;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubKeyframePose = nullptr;
	rclcpp::Publisher<PointCloud2>::SharedPtr pubRgbCloud = nullptr;

	rclcpp::Publisher<Imu>::SharedPtr pub_base_imu_ = nullptr;
	rclcpp::Publisher<PointCloud2>::SharedPtr pub_key_cloud_ = nullptr;
	rclcpp::Publisher<PointCloud2>::SharedPtr pub_body_cloud_filter_ = nullptr;

	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_pose_graph_ = nullptr;

	// 路径消息
	Path optimized_path_msg;
	Path baselink_in_map_path_msg;
	Path baselink_in_odom_path_msg;

	/// params load from yaml
	lidar_slam::LidarSlamParam slam_param_;

	inline static localization_module::LocalizationModuleLogInfoManager& log_info_manager_ =
		localization_module::LocalizationModuleLogInfoManager::getInstance();

	// lidar ptr
	std::shared_ptr<LidarPreprocParent> lidar_ptr_;

	tf2_ros::TransformBroadcaster br_;

	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;
};

} // namespace localization_module

#endif // LOCALIZATION_MODULE_H

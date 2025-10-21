#ifndef LOCALIZATION_MODULE_PARAM_MANAGER_H
#define LOCALIZATION_MODULE_PARAM_MANAGER_H

#include <logTracer/tracer.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "node/module_param_def.h"

using namespace std;

namespace localization_module {

class LocalizationModuleParamManager {
	DECL_CLASSNAME(LocalizationModuleParamManager)
   public:
	static LocalizationModuleParamManager* Instance(rclcpp::Node::SharedPtr node) {
		static LocalizationModuleParamManager* instance = nullptr;
		if (instance == nullptr) {
			instance = new LocalizationModuleParamManager(node);
		}
		return instance;
	}

	bool load_config_params() {
		std::string ns = "";
		bool success = true;

		/// common *******************************************
		node_->declare_parameter<bool>("common.run_on_mower", true);
		node_->get_parameter("common.run_on_mower", loaded_param_.common.run_on_mower);
		node_->declare_parameter<bool>("common.time_sync_en", false);
		node_->get_parameter("common.time_sync_en", loaded_param_.common.time_sync_en);

		node_->declare_parameter<int>("common.map_relative_to", 2);
		node_->get_parameter("common.map_relative_to", loaded_param_.common.map_relative_to);
		node_->declare_parameter<std::string>("common.map_directory", "/home/pmm/data/5-lanhai/");
		node_->get_parameter("common.map_directory", loaded_param_.common.map_directory);
		node_->declare_parameter<std::string>("common.sub_topic_ctrl_cmd", "/flbot/localization_module/ctrl_cmd");
		node_->get_parameter("common.sub_topic_ctrl_cmd", loaded_param_.common.sub_topic_ctrl_cmd);
		node_->declare_parameter<std::string>("common.pub_topic_module_status",
											  "/flbot/localization_module/module_status");
		node_->get_parameter("common.pub_topic_module_status", loaded_param_.common.pub_topic_module_status);
		node_->declare_parameter<std::string>("common.pub_topic_module_health",
											  "/flbot/localization_module/module_health");
		node_->get_parameter("common.pub_topic_module_health", loaded_param_.common.pub_topic_module_health);
		node_->declare_parameter<std::string>("common.pub_topic_module_loginfo", "/flbot/localization_module/log_info");
		node_->get_parameter("common.pub_topic_module_loginfo", loaded_param_.common.pub_topic_module_loginfo);
		node_->declare_parameter<std::string>("common.pub_topic_slipping", "/flbot/localization_module/slipping_old");
		node_->get_parameter("common.pub_topic_slipping", loaded_param_.common.pub_topic_slipping);
		node_->declare_parameter<int>("common.receive_lidar_freq", 10);
		node_->get_parameter("common.receive_lidar_freq", loaded_param_.common.receive_lidar_freq);
		node_->declare_parameter<double>("common.slam_lose_rate_time_thr", 0.1);
		node_->get_parameter("common.slam_lose_rate_time_thr", loaded_param_.common.slam_lose_rate_time_thr);

		node_->declare_parameter<int>("common.before_downsample_size_thr", 3000);
		node_->get_parameter("common.before_downsample_size_thr", loaded_param_.common.before_downsample_size_thr);
		node_->declare_parameter<int>("common.feats_down_size_thr", 200);
		node_->get_parameter("common.feats_down_size_thr", loaded_param_.common.feats_down_size_thr);

		node_->declare_parameter<std::vector<long int>>("common.cpu_id", std::vector<long int>());
		node_->get_parameter("common.cpu_id", loaded_param_.common.cpu_id);
		node_->declare_parameter<bool>("common.use_wheel_odom", false);
		node_->get_parameter("common.use_wheel_odom", loaded_param_.common.use_wheel_odom);
		node_->declare_parameter<std::string>("common.sub_wheel_odom_topic", "/flbot/hardware/chassic_data");
		node_->get_parameter("common.sub_wheel_odom_topic", loaded_param_.common.sub_wheel_odom_topic);
		node_->declare_parameter<double>("common.wheel_cov", 0.001);
		node_->get_parameter("common.wheel_cov", loaded_param_.common.wheel_cov);
		node_->declare_parameter<double>("common.nhc_y_cov", 0.001);
		node_->get_parameter("common.nhc_y_cov", loaded_param_.common.nhc_y_cov);
		node_->declare_parameter<double>("common.nhc_z_cov", 0.1);
		node_->get_parameter("common.nhc_z_cov", loaded_param_.common.nhc_z_cov);

		/// 处理地图目录路径
		std::string parent_dir;
		if (loaded_param_.common.map_relative_to == 0) { // 相对于pkg
			parent_dir = ament_index_cpp::get_package_share_directory("lidar_slam");
		} else if (loaded_param_.common.map_relative_to == 1) { // 相对于catkin_ws
			std::string package_path = ament_index_cpp::get_package_share_directory("lidar_slam");
			parent_dir = package_path + "/../../";
		} else if (loaded_param_.common.map_relative_to == 2) { // 绝对路径
			parent_dir = "";
		}

		loaded_param_.common.map_directory = parent_dir + loaded_param_.common.map_directory;
		std::string map_directory_on_mower_temp = "";
		std::vector<long int> cpu_id_on_mower_temp;

		node_->declare_parameter<std::vector<long int>>("common.cpu_id_on_mower", std::vector<long int>());
		node_->get_parameter("common.cpu_id_on_mower", cpu_id_on_mower_temp);
		node_->declare_parameter<std::string>("common.map_directory_on_mower", "");
		node_->get_parameter("common.map_directory_on_mower", map_directory_on_mower_temp);

		if (loaded_param_.common.run_on_mower) {
			loaded_param_.common.map_directory = map_directory_on_mower_temp;
			loaded_param_.common.cpu_id = cpu_id_on_mower_temp;
		}

		/// extrinsic *******************************************
		node_->declare_parameter<bool>("extrinsic.extrinsic_est_en", false);
		node_->get_parameter("extrinsic.extrinsic_est_en", loaded_param_.extrinsic.extrinsic_est_en);
		std::vector<double> baselink_lidar_trans;
		node_->declare_parameter<std::vector<double>>("extrinsic.baselink_lidar_trans", std::vector<double>());
		node_->get_parameter("extrinsic.baselink_lidar_trans", baselink_lidar_trans);
		std::vector<double> baselink_lidar_rot;
		node_->declare_parameter<std::vector<double>>("extrinsic.baselink_lidar_rot", std::vector<double>());
		node_->get_parameter("extrinsic.baselink_lidar_rot", baselink_lidar_rot);

		// T_baselink_alignedlidar
		Eigen::Isometry3d T_baselink_alignedlidar = makeTransform(baselink_lidar_trans, std::vector<double>{ 0, 0, 0 });

		// T_alignedlidar_lidar
		Eigen::Isometry3d T_alignedlidar_lidar = makeTransform(std::vector<double>{ 0, 0, 0 }, baselink_lidar_rot);

		// T_lidar_imu
		std::vector<double> lidar_imu_trans;
		node_->declare_parameter<std::vector<double>>("extrinsic.lidar_imu_trans", std::vector<double>());
		node_->get_parameter("extrinsic.lidar_imu_trans", lidar_imu_trans);
		std::vector<double> lidar_imu_rot;
		node_->declare_parameter<std::vector<double>>("extrinsic.lidar_imu_rot", std::vector<double>());
		node_->get_parameter("extrinsic.lidar_imu_rot", lidar_imu_rot);
		Eigen::Isometry3d T_lidar_imu = makeTransform(lidar_imu_trans, lidar_imu_rot);

		// T_baselink_imu
		Eigen::Isometry3d T_baselink_imu = T_baselink_alignedlidar * T_alignedlidar_lidar * T_lidar_imu;

		// T_alignedimu_imu
		Eigen::Isometry3d T_alignedimu_imu = Eigen::Isometry3d::Identity();
		T_alignedimu_imu.linear() = T_baselink_imu.linear();
		T_alignedimu_imu.translation() = Eigen::Vector3d::Zero();
		//用来把原始imu数据转到和baselink对齐的imu坐标系下

		// T_baselink_alignedimu
		Eigen::Isometry3d T_baselink_alignedimu = Eigen::Isometry3d::Identity();
		T_baselink_alignedimu = T_baselink_imu * T_alignedimu_imu.inverse();

		// T_alignedlidar_alignedimu
		Eigen::Isometry3d T_alignedlidar_alignedimu = Eigen::Isometry3d::Identity();
		T_alignedlidar_alignedimu = T_baselink_alignedlidar.inverse() * T_baselink_alignedimu;
		Eigen::Isometry3d T_alignedimu_alignedlidar = T_alignedlidar_alignedimu.inverse();

		//前端lio中需要的外参是：
		loaded_param_.extrinsic.extrinT = T_alignedimu_alignedlidar.translation();
		loaded_param_.extrinsic.extrinR = T_alignedimu_alignedlidar.linear().matrix();

		loaded_param_.extrinsic.R_baselink_IMU = T_baselink_imu.linear().matrix();
		//仅用来转换IMU数据到baselink坐标系下

		// T_alignedlidar_baselink
		loaded_param_.extrinsic.T_lidar_baselink = T_baselink_alignedlidar.inverse();

		// T_alignedimu_baselink
		loaded_param_.extrinsic.T_imu_baselink = T_baselink_alignedimu.inverse();

		Eigen::Vector3d baselink_alignedlidar_ypr = R2ypr(T_baselink_alignedlidar.linear().matrix());
		Eigen::Vector3d baselink_alignedimu_ypr = R2ypr(T_baselink_alignedimu.linear().matrix());
		TRACE_INFO_CLASS("baselink_alignedlidar_ypr: %f, %f, %f", baselink_alignedlidar_ypr.x() * RAD2DEGREE,
						 baselink_alignedlidar_ypr.y() * RAD2DEGREE, baselink_alignedlidar_ypr.z() * RAD2DEGREE);
		TRACE_INFO_CLASS("baselink_alignedimu_ypr: %f, %f, %f", baselink_alignedimu_ypr.x() * RAD2DEGREE,
						 baselink_alignedimu_ypr.y() * RAD2DEGREE, baselink_alignedimu_ypr.z() * RAD2DEGREE);

		/// lidar_preproc params *******************************************
		node_->declare_parameter<int>("lidar_preproc.lidar_type", 5);
		node_->get_parameter("lidar_preproc.lidar_type", loaded_param_.lidar_preproc.lidar_type);

		if (loaded_param_.lidar_preproc.lidar_type == 2 || loaded_param_.lidar_preproc.lidar_type == 3) {
			node_->declare_parameter<int>("lidar_preproc.cloud_column_count", 5);
			node_->get_parameter("lidar_preproc.cloud_column_count", loaded_param_.lidar_preproc.cloud_column_count);
			node_->declare_parameter<int>("lidar_preproc.cloud_ring_count", 5);
			node_->get_parameter("lidar_preproc.cloud_ring_count", loaded_param_.lidar_preproc.cloud_ring_count);
			node_->declare_parameter<float>("lidar_preproc.edge_curvature_thr", 5.0);
			node_->get_parameter("lidar_preproc.edge_curvature_thr", loaded_param_.lidar_preproc.edge_curvature_thr);
			node_->declare_parameter<float>("lidar_preproc.surf_curvature_thr", 5.0);
			node_->get_parameter("lidar_preproc.surf_curvature_thr", loaded_param_.lidar_preproc.surf_curvature_thr);
		}

		node_->declare_parameter<std::string>("lidar_preproc.sub_lidar_topic", "/M300/lidar");
		node_->get_parameter("lidar_preproc.sub_lidar_topic", loaded_param_.lidar_preproc.sub_lidar_topic);
		node_->declare_parameter<std::string>("lidar_preproc.sub_imu_topic", "/M300/imu");
		node_->get_parameter("lidar_preproc.sub_imu_topic", loaded_param_.lidar_preproc.sub_imu_topic);
		node_->declare_parameter<float>("lidar_preproc.blind_distance", 0.2);
		node_->get_parameter("lidar_preproc.blind_distance", loaded_param_.lidar_preproc.blind_distance);
		node_->declare_parameter<float>("lidar_preproc.max_distance", 69.0);
		node_->get_parameter("lidar_preproc.max_distance", loaded_param_.lidar_preproc.max_distance);
		node_->declare_parameter<std::vector<double>>("lidar_preproc.z_range", std::vector<double>());
		node_->get_parameter("lidar_preproc.z_range", loaded_param_.lidar_preproc.z_range);

		node_->declare_parameter<int>("lidar_preproc.keep_lidar_num_before_curr", 1);
		node_->get_parameter("lidar_preproc.keep_lidar_num_before_curr",
							 loaded_param_.lidar_preproc.keep_lidar_num_before_curr);
		node_->declare_parameter<int>("lidar_preproc.point_filter_num", 6);
		node_->get_parameter("lidar_preproc.point_filter_num", loaded_param_.lidar_preproc.point_filter_num);
		node_->declare_parameter<int>("lidar_preproc.ring_filter_num", 1);
		node_->get_parameter("lidar_preproc.ring_filter_num", loaded_param_.lidar_preproc.ring_filter_num);
		node_->declare_parameter<int>("lidar_preproc.cloud_size_to_keep", 2500);
		node_->get_parameter("lidar_preproc.cloud_size_to_keep", loaded_param_.lidar_preproc.cloud_size_to_keep);

		// mapping params *******************************************
		node_->declare_parameter<int>("lidar_preproc.extract_cloud_method", 0);
		node_->get_parameter("lidar_preproc.extract_cloud_method", loaded_param_.lidar_preproc.extract_cloud_method);
		node_->declare_parameter<double>("lidar_preproc.leafsize", 0.5);
		node_->get_parameter("lidar_preproc.leafsize", loaded_param_.lidar_preproc.leafsize);
		node_->declare_parameter<std::vector<double>>("lidar_preproc.leafsize_vec", std::vector<double>());
		node_->get_parameter("lidar_preproc.leafsize_vec", loaded_param_.lidar_preproc.leafsize_vec);
		node_->declare_parameter<std::vector<double>>("lidar_preproc.voxel_region_xyz", std::vector<double>());
		node_->get_parameter("lidar_preproc.voxel_region_xyz", loaded_param_.lidar_preproc.voxel_region_xyz);
		node_->declare_parameter<double>("lidar_preproc.boundary_z", 4.0);
		node_->get_parameter("lidar_preproc.boundary_z", loaded_param_.lidar_preproc.boundary_z);
		node_->declare_parameter<double>("mapping.acc_cov", 0.1);
		node_->get_parameter("mapping.acc_cov", loaded_param_.mapping.acc_cov);
		node_->declare_parameter<double>("mapping.gyr_cov", 0.1);
		node_->get_parameter("mapping.gyr_cov", loaded_param_.mapping.gyr_cov);
		node_->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
		node_->get_parameter("mapping.b_acc_cov", loaded_param_.mapping.b_acc_cov);
		node_->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
		node_->get_parameter("mapping.b_gyr_cov", loaded_param_.mapping.b_gyr_cov);
		node_->declare_parameter<double>("mapping.cloud_leaf_size", 0.5);
		node_->get_parameter("mapping.cloud_leaf_size", loaded_param_.mapping.cloud_leaf_size);
		node_->declare_parameter<double>("mapping.key_frame_distance", 1.0);
		node_->get_parameter("mapping.key_frame_distance", loaded_param_.mapping.key_frame_distance);
		node_->declare_parameter<double>("mapping.key_frame_angle", 0.2);
		node_->get_parameter("mapping.key_frame_angle", loaded_param_.mapping.key_frame_angle);

		node_->declare_parameter<double>("mapping.loopSearchDistance", 1.5);
		node_->get_parameter("mapping.loopSearchDistance", loaded_param_.mapping.loopSearchDistance);
		node_->declare_parameter<double>("mapping.loopSearchTimeDiff", 30.0);
		node_->get_parameter("mapping.loopSearchTimeDiff", loaded_param_.mapping.loopSearchTimeDiff);
		node_->declare_parameter<int>("mapping.loopSearchSkipKey", 5);
		node_->get_parameter("mapping.loopSearchSkipKey", loaded_param_.mapping.loopSearchSkipKey);
		node_->declare_parameter<double>("mapping.loopIcpScore", 0.3);
		node_->get_parameter("mapping.loopIcpScore", loaded_param_.mapping.loopIcpScore);

		node_->declare_parameter<double>("mapping.save_map_resolution", 0.1);
		node_->get_parameter("mapping.save_map_resolution", loaded_param_.mapping.save_map_resolution);

		/// localization params *******************************************
		node_->declare_parameter<float>("localization.fgicp_peroid_sec", 5.0);
		node_->get_parameter("localization.fgicp_peroid_sec", loaded_param_.localization.fgicp_peroid_sec);
		node_->declare_parameter<double>("localization.cloud_leaf_size_localize", 0.3);
		node_->get_parameter("localization.cloud_leaf_size_localize",
							 loaded_param_.localization.cloud_leaf_size_localize);
		node_->declare_parameter<int>("localization.fgicp_thread_num", 2);
		node_->get_parameter("localization.fgicp_thread_num", loaded_param_.localization.fgicp_thread_num);
		node_->declare_parameter<float>("localization.fgicp_trans_eps", 0.01);
		node_->get_parameter("localization.fgicp_trans_eps", loaded_param_.localization.fgicp_trans_eps);
		node_->declare_parameter<int>("localization.fgicp_max_iter", 64);
		node_->get_parameter("localization.fgicp_max_iter", loaded_param_.localization.fgicp_max_iter);
		node_->declare_parameter<float>("localization.fgicp_max_corres_dist", 2.0);
		node_->get_parameter("localization.fgicp_max_corres_dist", loaded_param_.localization.fgicp_max_corres_dist);
		node_->declare_parameter<int>("localization.fgicp_max_corres_num", 20);
		node_->get_parameter("localization.fgicp_max_corres_num", loaded_param_.localization.fgicp_max_corres_num);
		node_->declare_parameter<float>("localization.fgicp_inlier_max_valid_point_dist", 40);
		node_->get_parameter("localization.fgicp_inlier_max_valid_point_dist",
							 loaded_param_.localization.fgicp_inlier_max_valid_point_dist);
		node_->declare_parameter<float>("localization.fgicp_inlier_max_corres_dist", 0.5);
		node_->get_parameter("localization.fgicp_inlier_max_corres_dist",
							 loaded_param_.localization.fgicp_inlier_max_corres_dist);
		node_->declare_parameter<float>("localization.fgicp_inlier_rate_thr", 0.8);
		node_->get_parameter("localization.fgicp_inlier_rate_thr", loaded_param_.localization.fgicp_inlier_rate_thr);
		node_->declare_parameter<float>("localization.fgicp_inlier_avg_error_thr", 0.25);
		node_->get_parameter("localization.fgicp_inlier_avg_error_thr",
							 loaded_param_.localization.fgicp_inlier_avg_error_thr);

		/// re-localization params *******************************************s
		node_->declare_parameter<double>("re_localization.score_thr", 0.05);
		node_->get_parameter("re_localization.score_thr", loaded_param_.re_localization.score_thr);

		node_->declare_parameter<int>("re_localization.time_out_thr", 30);
		node_->get_parameter("re_localization.time_out_thr", loaded_param_.re_localization.time_out_thr);

		/// ikdtree params *******************************************
		node_->declare_parameter<double>("ikdtree.cube_len", 400.0);
		node_->get_parameter("ikdtree.cube_len", loaded_param_.ikdtree.cube_len);
		node_->declare_parameter<double>("ikdtree.det_range", 40.0);
		node_->get_parameter("ikdtree.det_range", loaded_param_.ikdtree.det_range);
		node_->declare_parameter<double>("ikdtree.kdTreeReconstructRadius", 400.0);
		node_->get_parameter("ikdtree.kdTreeReconstructRadius", loaded_param_.ikdtree.kdTreeReconstructRadius);
		node_->declare_parameter<double>("ikdtree.kdTreeReconstructKeyFrameLeafSize", 10.0);
		node_->get_parameter("ikdtree.kdTreeReconstructKeyFrameLeafSize",
							 loaded_param_.ikdtree.kdTreeReconstructKeyFrameLeafSize);
		node_->declare_parameter<double>("ikdtree.kdTreeReconstructPointLeafSize", 1.0);
		node_->get_parameter("ikdtree.kdTreeReconstructPointLeafSize",
							 loaded_param_.ikdtree.kdTreeReconstructPointLeafSize);
		node_->declare_parameter<double>("ikdtree.map_leaf_size", 0.5);
		node_->get_parameter("ikdtree.map_leaf_size", loaded_param_.ikdtree.map_leaf_size);

		///  detect slip params *******************************************
		node_->declare_parameter<double>("detect_slip.detect_window_time_range", 2.0);
		node_->get_parameter("detect_slip.detect_window_time_range",
							 loaded_param_.detect_slip.detect_window_time_range);
		node_->declare_parameter<int>("detect_slip.slipping_count_thr", 5);
		node_->get_parameter("detect_slip.slipping_count_thr", loaded_param_.detect_slip.slipping_count_thr);
		node_->declare_parameter<double>("detect_slip.slipping_dist_thr", 0.15);
		node_->get_parameter("detect_slip.slipping_dist_thr", loaded_param_.detect_slip.slipping_dist_thr);

		TRACE_INFO_CLASS("run_on_mower: %d", loaded_param_.common.run_on_mower);
		TRACE_INFO_CLASS("set cpu_id size: %d", (int)loaded_param_.common.cpu_id.size());
		TRACE_INFO_CLASS("map directory: %s", loaded_param_.common.map_directory.c_str());

		return success;
	}

	// const lidar_slam::LidarSlamParam* get_loaded_param() const {
	//     return &loaded_param_;
	// }

	// 方案1：返回const引用（推荐）
	const lidar_slam::LidarSlamParam& get_loaded_param() const { return loaded_param_; }

   private:
	// 私有构造函数
	LocalizationModuleParamManager(rclcpp::Node::SharedPtr node) : node_(node) {
		if (!load_config_params()) {
			TRACE_ERR_CLASS("Failed to load configuration parameters");
			throw std::runtime_error("Parameter loading failed");
		}
	}

	~LocalizationModuleParamManager() = default;

	rclcpp::Node::SharedPtr node_;
	lidar_slam::LidarSlamParam loaded_param_;
};

} // namespace localization_module

#endif
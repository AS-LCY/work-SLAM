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

		// ... 其他参数处理逻辑保持不变

		/// extrinsic *******************************************
		vector<double> extrinsic_T;
		vector<double> extrinsic_R;
		std::vector<double> Lidar_In_Wheel;					   // 4* 4
		std::vector<double> extrinsic_euler_IMU_in_lidar;	   // 1 * 3
		std::vector<double> extrinsic_euler_lidar_in_baselink; // 1 * 3

		node_->declare_parameter<std::vector<double>>("extrinsic.extrinsic_T", std::vector<double>());
		node_->get_parameter("extrinsic.extrinsic_T", extrinsic_T);

		node_->declare_parameter<std::vector<double>>("extrinsic.extrinsic_R", std::vector<double>());
		node_->get_parameter("extrinsic.extrinsic_R", extrinsic_R);

		node_->declare_parameter<bool>("extrinsic.extrinsic_est_en", false);
		node_->get_parameter("extrinsic.extrinsic_est_en", loaded_param_.extrinsic.extrinsic_est_en);

		node_->declare_parameter<std::vector<double>>("extrinsic.Lidar_In_Wheel", std::vector<double>());
		node_->get_parameter("extrinsic.Lidar_In_Wheel", Lidar_In_Wheel);

		node_->declare_parameter<std::vector<double>>("extrinsic.extrinsic_euler_IMU_in_lidar", std::vector<double>());
		node_->get_parameter("extrinsic.extrinsic_euler_IMU_in_lidar", extrinsic_euler_IMU_in_lidar);

		node_->declare_parameter<std::vector<double>>("extrinsic.extrinsic_euler_lidar_in_baselink",
													  std::vector<double>());
		node_->get_parameter("extrinsic.extrinsic_euler_lidar_in_baselink", extrinsic_euler_lidar_in_baselink);
		///注意ROS2中使用点号(.)代替了斜杠(/)作为参数命名空间分隔符，且需要先声明参数再获取。

		// 矩阵赋值逻辑保持不变
		loaded_param_.extrinsic.extrinT << extrinsic_T[0], extrinsic_T[1], extrinsic_T[2];
		double yaw = extrinsic_R[0] / 180 * M_PI;
		double pitch = extrinsic_R[1] / 180 * M_PI;
		double roll = extrinsic_R[2] / 180 * M_PI;
		loaded_param_.extrinsic.extrinR = ypr2R(Eigen::Vector3d{ yaw, pitch, roll }); // T_imu_lidar

		// IMU in base_link
		Eigen::Matrix3d R_imu_in_lidar = Eigen::Matrix3d::Identity();
		if (extrinsic_euler_IMU_in_lidar.size() == 3) {
			double yaw2 = extrinsic_euler_IMU_in_lidar[0] / 180 * M_PI;
			double pitch2 = extrinsic_euler_IMU_in_lidar[1] / 180 * M_PI;
			double roll2 = extrinsic_euler_IMU_in_lidar[2] / 180 * M_PI;
			R_imu_in_lidar = rpy2R(Eigen::Vector3d{ roll2, pitch2, yaw2 }); // TODO(jxl): 内部实现和ypr2R等价
		} else if (extrinsic_euler_IMU_in_lidar.size() == 4) {
			double qx = extrinsic_euler_IMU_in_lidar[0];
			double qy = extrinsic_euler_IMU_in_lidar[1];
			double qz = extrinsic_euler_IMU_in_lidar[2];
			double qw = extrinsic_euler_IMU_in_lidar[3];
			Eigen::Quaterniond eigen_quat = Eigen::Quaterniond(qw, qx, qy, qz);
			R_imu_in_lidar = eigen_quat.toRotationMatrix();
		}

		double yaw3 = extrinsic_euler_lidar_in_baselink[0] / 180 * M_PI;
		double pitch3 = extrinsic_euler_lidar_in_baselink[1] / 180 * M_PI;
		double roll3 = extrinsic_euler_lidar_in_baselink[2] / 180 * M_PI;
		auto R_lidar_in_base = rpy2R(Eigen::Vector3d{ roll3, pitch3, yaw3 });
		loaded_param_.extrinsic.R_baselink_IMU = R_lidar_in_base * R_imu_in_lidar;
		//仅用来转换IMU数据到baselink坐标系下

		loaded_param_.extrinsic.yaw_pitch_roll_deg = extrinsic_euler_lidar_in_baselink;

		// T_wheel_lidar & T_lidar_wheel
		Eigen::Matrix4d T_wheel_lidar;
		T_wheel_lidar << Lidar_In_Wheel[0], Lidar_In_Wheel[1], Lidar_In_Wheel[2], Lidar_In_Wheel[3], Lidar_In_Wheel[4],
			Lidar_In_Wheel[5], Lidar_In_Wheel[6], Lidar_In_Wheel[7], Lidar_In_Wheel[8], Lidar_In_Wheel[9],
			Lidar_In_Wheel[10], Lidar_In_Wheel[11], Lidar_In_Wheel[12], Lidar_In_Wheel[13], Lidar_In_Wheel[14],
			Lidar_In_Wheel[15];
		loaded_param_.extrinsic.T_wheel_lidar.matrix() = T_wheel_lidar;
		loaded_param_.extrinsic.T_lidar_wheel = loaded_param_.extrinsic.T_wheel_lidar.inverse();

		//计算T_imu_baselink
		auto T_imu_lidar = Eigen::Isometry3d::Identity();
		T_imu_lidar.linear() = loaded_param_.extrinsic.extrinR;
		T_imu_lidar.translation() = loaded_param_.extrinsic.extrinT;
		loaded_param_.extrinsic.T_imu_baselink = T_imu_lidar * loaded_param_.extrinsic.T_lidar_wheel;

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
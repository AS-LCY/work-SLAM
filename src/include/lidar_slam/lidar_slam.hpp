#ifndef LIDAR_SLAM_H
#define LIDAR_SLAM_H
#include <math.h>
#include <omp.h>
#include <pthread.h>
#include <unistd.h>

#include <Eigen/Core>
#include <csignal>
#include <fstream>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "boost/thread.hpp"
// #include <filesystem> // c++17

#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>

#include <sophus/se3.hpp>

// #include <fast_gicp/gicp/fast_gicp.hpp>

#include "flbot_msgs/msg/chassis_data.hpp"
#include "lidar_slam/IMU_Processing.hpp"
#include "lidar_slam/backend.hpp"
#include "lidar_slam/cloud_map.hpp"
#include "lidar_slam/common_lib.h"
#include "lidar_slam/global_localization.hpp"
#include "lidar_slam/localization.hpp"

// #include "lidar_slam/Viewer.hpp"
// #include "include/livox_ros_driver2.h"
// #include "driver_node.h"
// #include "lddc.h"
#include "lidar/livox/ros_livox_datatype_def.h"
#include "node/module_param_def.h"
// #include "node/module_status_def.h"
#include "node/log_info_manager.hpp"

// lidar
#include "lidar/hesai/lidar_preproc_JT16.h"
#include "lidar/hesai/pcl_point_type_def_hs.h"
#include "lidar/lanhai/lidar_preproc_M300.h"
#include "lidar/lanhai/pcl_point_type_def_bs.h"
#include "lidar/lidar_preproc_factory.hpp"
#include "lidar/livox/lidar_preproc_Mid360.h"
#include "lidar/livox/pcl_point_type_def_lvx.h"
#include "lidar/robosense/lidar_preproc_Airy.h"
#include "lidar/robosense/pcl_point_type_def_rs.h"

namespace lidar_slam {

struct Localization_base {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	state_ikfom imu_state;
	double update_time = 0;

	Localization_base() {
		imu_state = state_ikfom();
		update_time = 0;
	}
};

enum SlamWorkMode { MAPPING = 1, SEC_MAPPING = 2, LOCALIZATION = 3, UNKNOWN };

string print_SlamWorkMode(SlamWorkMode e);

class LidarSlam {
	DECL_CLASSNAME(LidarSlam)
   public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using HealthStatus = common_status::HealthStatus;
	using LocalizationStatus = common_status::LocalizationStatus;
	using LocalNodeStatus = common_status::LocalNodeStatus;
	using MappingNodeStatus = common_status::MappingNodeStatus;
	using MappingStatus = common_status::MappingStatus;
	using SecmapRelocalThrdStatus = common_status::SecmapRelocalThrdStatus;
	using SlamRunStatus = common_status::SlamRunStatus;
	using ChassisData = flbot_msgs::msg::ChassisData;

	LidarSlam(const LidarSlamParam yaml_param, SlamWorkMode init_mode, rclcpp::Node::SharedPtr node); // new added
	LidarSlam() = delete;
	LidarSlam(const LidarSlam&) = delete;
	void reset(SlamWorkMode work_mode, rclcpp::Node::SharedPtr node); // new added

	~LidarSlam() {
		thread_run_ = false;
		thread_->join();
		thread_.reset(nullptr);

		if (working_mode_ == SEC_MAPPING) {
			global_localization_thread_->join();
			global_localization_thread_.reset(nullptr);
		}
	};

	bool run();

	void wheel_odom_cbk(const WheelOdomData& msg);
	void lidar_pcl_cbk(const PointCloudType::Ptr cloud);
	void imu_cbk(const std::shared_ptr<livox_ros::ImuMsg>& msg_in);
	bool save_map(string saveMapDirectory, double resolution, int start_index, int end_index) {
		if (working_mode_ == LOCALIZATION) {
			return true;
		} else if (working_mode_ == MAPPING || working_mode_ == SEC_MAPPING) {
			return back_end_->saveMap(saveMapDirectory, resolution, start_index, end_index);
		} else {
			return true;
		}
	}

	bool load_map(string directory) {
		globalLocalizationSuccess_ = false;

		// sleep(1); 只有 IDLE -> LOCALIZATION / SEC_MAPPING 时会加载地图， globalLocalization 相关为空，无需睡眠
		if (working_mode_ == LOCALIZATION) {
			localization_->loadMap(directory);
		} else if (working_mode_ == SEC_MAPPING) {
			if (!cloud_map_manager_->load_map_data(directory)) {
				return false;
			}
		} else {
			return false;
		}
		return true;
	}

	inline int get_curr_pose_index() const { return back_end_->getCurrentPoseIndex(); }

	inline PointCloudType::Ptr get_lidar_cloud() {
		std::unique_lock<std::mutex> lk(mtx_lidar_cloud_);
		return undistortCloud_;
	}

	inline PointCloudType::Ptr get_baselink_cloud(double& cloud_time) {
		std::unique_lock<std::mutex> lk(mtx_lidar_cloud_);
		PointCloudType::Ptr baselink_cloud;
		baselink_cloud.reset(new PointCloudType());
		baselink_cloud->resize(undistortCloud_->points.size());
		baselink_cloud = transformPointCloud(undistortCloud_, T_lidar_wheel_.inverse());
		cloud_time = T_odom_lidar_time_;
		return baselink_cloud;
	}

	inline PointCloudType::Ptr get_odom_cloud(double& cloud_time) {
		PointCloudType::Ptr UndistortCloudInOdom;
		UndistortCloudInOdom.reset(new PointCloudType());
		{
			std::unique_lock<std::mutex> cloud_lock(mtx_lidar_cloud_);
			UndistortCloudInOdom->resize(undistortCloud_->points.size());
			UndistortCloudInOdom = transformPointCloud(undistortCloud_, T_odom_lidar_);
		}
		cloud_time = T_odom_lidar_time_;
		return UndistortCloudInOdom;
	}

	inline std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> get_optimized_path() {
		std::unique_lock<std::mutex> lk(mtx_path_);
		return optimized_path_;
	}
	inline std::map<int, int> getloopIndex() const { return back_end_->getloopIndex(); }

	inline std::vector<std::pair<int, int>> getAllLoopEdges() { return back_end_->getAllLoopEdges(); }
	inline std::vector<KeyPose> getAllKeyframeNodes() { return back_end_->getKeyframePoses(); }
	inline std::vector<KeyPose> getAllKeyframeBaselinkNodes() {
		auto poses = back_end_->getKeyframePoses();
		for (auto& p : poses) {
			p.pose = p.pose * T_lidar_wheel_;
		}
		return poses;
	}

	inline Eigen::Isometry3d getOdomToMap() {
		if (working_mode_ == LOCALIZATION) {
			T_map_odom_ = localization_->getOdomToMap();
		} else {
			T_map_odom_ = back_end_->getOdomToMap();
		}
		return T_map_odom_;
	}

	// jxl: 不管是建图还是定位，都是10hz的T_odom_lidar
	inline Eigen::Isometry3d getLidarInOdom(double& T_odom_lidar_time) {
		if (working_mode_ == MAPPING || working_mode_ == SEC_MAPPING || working_mode_ == LOCALIZATION) {
			std::unique_lock<std::mutex> lk(mtx_pose_);
			T_odom_lidar_time = T_odom_lidar_time_;
			return T_odom_lidar_; // 10hz

			// 200hz预测
			// std::unique_lock<std::mutex> current_pose_lock(mtx_current_pose_);
			// Eigen::Isometry3d T_odom_imu(
			// 	Sophus::SE3d(current_pose_.imu_state.rot, current_pose_.imu_state.pos).matrix());
			// T_odom_lidar_time = current_pose_.update_time;
			// current_pose_lock.unlock();
			// Eigen::Isometry3d T_odom_lidar = T_odom_imu * T_imu_lidar_;
			// return T_odom_lidar;
		} else {
			TRACE_ERR_CLASS("In other mode, T_odom_lidar = I");
			return Eigen::Isometry3d::Identity();
		}
	}

	inline Localization_base get_current_pose() { // T_odom_imu, 200hz
		std::unique_lock<std::mutex> current_pose_lock(mtx_current_pose_);
		return current_pose_;
	}

	inline Localization_base get_localization_base() { // T_odom_imu, 10hz
		std::unique_lock<std::mutex> localization_base_lock(mtx_localization_base_);
		return localization_base_;
	}

	inline double get_slam_time() const {
		if (working_mode_ == MAPPING || working_mode_ == SEC_MAPPING || working_mode_ == LOCALIZATION) {
			return localization_base_.update_time;
		} else {
			double temp = rclcpp::Clock().now().seconds();
			// TODO(jxl):
			// 不要使用这种接口，这种接口获取now时间永远是系统当前时间，在跑bag时，我们希望的是bag中当时/clock时间，其它所有地方一并修改
			// rclcpp::Time node_time = rclcpp_node_->now();
			return temp;
		}
	}

	inline Eigen::Isometry3d getWheelInOdom(double& T_odom_lidar_time) {
		return getLidarInOdom(T_odom_lidar_time) * T_lidar_wheel_;
	}

	inline pcl::PointCloud<pcl::PointXYZI>::Ptr getLoadMap() const { return localization_->getLoadMap(); }

	inline bool isGloalLocalizationSuccess() const { return globalLocalizationSuccess_; }

	inline Eigen::Isometry3d getLidarInMap(double& T_odom_lidar_time) {
		Eigen::Isometry3d T_map_lidar = getOdomToMap() * getLidarInOdom(T_odom_lidar_time);
		return T_map_lidar;
	}

	inline Eigen::Isometry3d getWheelInMap(double& T_odom_lidar_time) {
		return getLidarInMap(T_odom_lidar_time) * T_lidar_wheel_;
	}
	inline Eigen::Isometry3d getWheelInLidar() const { return T_lidar_wheel_; }
	inline std::vector<ScInfo, Eigen::aligned_allocator<ScInfo>> getLoadKeyFrame() const {
		return localization_->getLoadKeyFrame();
	}
	inline PointCloudType::Ptr getTestCloud() const {
		PointCloudType::Ptr temp(new PointCloudType());
		if (working_mode_ == LOCALIZATION)
			return localization_->getTestCloud();
		else if (working_mode_ == MAPPING || working_mode_ == SEC_MAPPING)
			return back_end_->getTestCloud();
		else
			return temp;
	}
	inline PointCloudType::Ptr getCurrentMap() { return back_end_->getCurrentMap(); }
	inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCurrentRGBMap() const { return back_end_->getCurrentRGBMap(); }

	inline double get_lidar_time() const { return lidar_end_time_; }
	double get_lio_cost_time() { return lio_cost_time_.load(); }
	double get_hb_time_thread_localize() { return hb_time_thread_localize_.load(); }
	double get_hb_time_thread_loop_closure() { return hb_time_thread_loop_closure_.load(); }
	double get_hb_time_thread_secmap_relocalize() { return hb_time_thread_secmap_relocalize_.load(); }
	inline int get_feats_down_size() const { return feats_down_size_; }
	inline Eigen::Matrix<double, 6, 1> get_lio_state_diag_cov() { return lio_state_diag_cov_; }

	LocalizationStatus get_local_thrd_status() { return local_thrd_status_.load(); }
	SlamRunStatus get_slam_run_status() { return slam_run_status_.load(); }
	SecmapRelocalThrdStatus get_secmap_relocal_thrd_status() { return secmap_relocal_thrd_status_.load(); }
	inline LocalizeStatus get_localize_status() const { return localize_status_; }

   private:
	bool sync_packages(MeasureGroup& meas);
	void loopClosureThread();
	void sec_mapping_loopClosureThread();
	void localizationThread();

	void global_localization_for_sec_mapping_thread();

	void print_imu_state(const state_ikfom& state, const Eigen::Matrix<double, 24, 24>& cov);
	bool check_occlusion(const double& ratio_threshold);
	bool check_lio_vel_abnormal(const state_ikfom& imu_state);
	bool check_pointcloud_state_abnormal();
	std::deque<WheelOdomData> getDataInRangeAndClean(double lidar_beg_time, double lidar_end_time);
	void upsampling_current_pose(const double& init_acc_norm);

   private:
	std::atomic<LocalizationStatus> local_thrd_status_{ LocalizationStatus::Inactive };
	std::atomic<SlamRunStatus> slam_run_status_{ SlamRunStatus::Inactive };
	std::atomic<SecmapRelocalThrdStatus> secmap_relocal_thrd_status_{ SecmapRelocalThrdStatus::Inactive };

	// 各 线程、callback、timer heartbeat
	std::atomic<double> hb_time_thread_localize_;		   // status = LOCALIZATION
	std::atomic<double> hb_time_thread_loop_closure_;	   // status = MAPPING or SEC_MAPPING
	std::atomic<double> hb_time_thread_secmap_relocalize_; // status = SECMAPPING

	LidarSlamParam config_param_;
	int feats_down_size_thr_ = 100;
	bool flag_keep_only_last_lidar_ = true;

	bool need_localize_ = true;

	deque<double> time_buffer_;				  // 记录lidar时间, lidar header time
	deque<PointCloudType::Ptr> lidar_buffer_; // 记录特征提取或间隔采样后的lidar（特征）数据
	deque<std::shared_ptr<livox_ros::ImuMsg>> imu_buffer_;
	deque<WheelOdomData> wheel_odom_buffer_;

	// bool lidar_pushed_ = false;
	atomic<double> lidar_end_time_;	   // < 当前帧雷达，帧结束的时间，update: sync_packages()
	double lidar_mean_scantime_ = 0.0; // < 单帧点云的 time period
	double first_lidar_time_ = 0.0;	   // 第一帧点云的时间

	int scan_num_ = 0;
	bool flg_first_scan_ = true;
	double last_timestamp_lidar_ = 0;
	double last_timestamp_imu_ = -1.0;
	double timediff_lidar_wrt_imu_ = 0.0;
	bool time_sync_en_ = false;
	bool timediff_set_flg_ = false; // 标记是否已经进行了时间补偿
	bool reseting_ = false;

	std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> optimized_path_;

	MeasureGroup Measures_;
	Eigen::Matrix<double, 6, 6> T_odom_lidar_cov_ = Eigen::Matrix<double, 6, 6>::Zero();
	Eigen::Isometry3d T_odom_lidar_ = Eigen::Isometry3d::Identity();
	double T_odom_lidar_time_ = 0.f;
	Eigen::Isometry3d T_lidar_wheel_ = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d T_imu_baselink_ = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d T_imu_lidar_ = Eigen::Isometry3d::Identity();

	bool thread_run_ = true;
	bool globalLocalizationSuccess_ = false;

	Localization_base localization_base_; // 10hz, T_odom_imu, 每次点云处理后更新，并作为current_pose 积分结果的base
	Localization_base current_pose_; // 200hz, T_odom_imu
	std::atomic_bool laser_updated_ = { false };

	std::unique_ptr<KD_TREE<pcl::PointXYZINormal>> ikdtree_ = nullptr;

	std::unique_ptr<std::thread> thread_ = nullptr;
	std::unique_ptr<std::thread> global_localization_thread_ = nullptr;

	std::mutex mtx_imu_buffer_;
	std::mutex mtx_lidar_buffer_;
	std::mutex mtx_wheel_odom_buffer_;

	std::mutex mtx_odom_cloud_;
	std::mutex mtx_lidar_cloud_;
	std::mutex mtx_obstacle_cloud_;

	std::mutex mtx_localization_base_;
	std::mutex mtx_current_pose_;

	std::mutex mtx_path_;
	std::mutex mtx_pose_;

	esekfom::esekf kf_;

	std::unique_ptr<ImuProcess> p_imu_ = nullptr;
	std::unique_ptr<BackEnd> back_end_ = nullptr;
	std::unique_ptr<Localization> localization_ = nullptr;
	std::unique_ptr<GlobalLocalization> global_localization_ = nullptr;
	std::unique_ptr<CloudMap> cloud_map_manager_ = nullptr;

	PointCloudType::Ptr UndistortCloudInOdom_;
	PointCloudType::Ptr undistortCloud_; // lidar 系
	PointCloudType::Ptr FilteredUndistortCloud_;

	pcl::VoxelGrid<PointType> downSizeFilterCloud_;			 // lio
	pcl::VoxelGrid<PointType> downSizeFilterCloud_test_;	 // global localize
	pcl::VoxelGrid<PointType> downSizeFilterCloud_localize_; // localize

	SlamWorkMode working_mode_ = UNKNOWN;

	int global_localize_count_ = 0;
	int lidar_no_point_count_ = 0;
	inline static localization_module::LocalizationModuleLogInfoManager& log_info_manager_ =
		localization_module::LocalizationModuleLogInfoManager::getInstance();

	// cpu_set_t mask;

	Eigen::Isometry3d init_T_map_odom_ = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d T_map_odom_ = Eigen::Isometry3d::Identity();

	std::atomic<double> lio_cost_time_{ 0.f };
	int feats_down_size_ = 0;
	Eigen::Matrix<double, 6, 1> lio_state_diag_cov_;

	LocalizeStatus localize_status_;

	//定位线程使用
	Eigen::Isometry3d T_odom_lidar_curr_ = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d T_odom_lidar_last_ = Eigen::Isometry3d::Identity();
	Eigen::Matrix<double, 6, 6> T_odom_lidar_cov_curr_ = Eigen::Matrix<double, 6, 6>::Zero();
	Eigen::Matrix<double, 6, 6> T_odom_lidar_cov_last_ = Eigen::Matrix<double, 6, 6>::Zero();
	double lidar_time_curr_ = 0.0;
	double lidar_time_last_ = 0.0;
};

} // namespace lidar_slam
#endif

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
struct LidarParam {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	V3D extrinT;
	M3D extrinR;
	Eigen::Isometry3d T_wheel_lidar = Eigen::Isometry3d::Identity();
	bool localization_mode;
	bool offline_mode;
	std::string load_map_path;
	std::string save_log_path;
	double log_keep_time;
	double cloud_leaf_size;
	double map_leaf_size;
	double cube_len;
	double det_range;
	double blind_distance;
	double obstacle_max_range;
	double obstacle_min_height;
	double obstacle_max_height;
	double obstacle_filter_size;
	int point_filter_num;
	double key_frame_distance;
	double key_frame_angle;
	double loopSearchDistance;
	double kdTreeReconstructRadius;
	double kdTreeReconstructKeyFrameLeafSize;
	double kdTreeReconstructPointLeafSize;
};

struct Localization_base {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	state_ikfom imu_state;
	double base_time = 0;
	double update_time = 0;

	Localization_base() {
		imu_state = state_ikfom();
		base_time = 0;
		update_time = 0;
	}
};

enum SlamWorkMode { MAPPING = 1, SEC_MAPPING = 2, LOCALIZATION = 3, UNKNOWN };

string print_SlamWorkMode(SlamWorkMode e);

class LidarSlam {
	DECL_CLASSNAME(LidarSlam)
   public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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

	void lidar_pcl_cbk(const PointCloudType::Ptr cloud);
	void imu_cbk(const std::shared_ptr<livox_ros::ImuMsg>& msg_in);
	bool save_map(string saveMapDirectory, double resolution, int start_index, int end_index) {
		if (working_mode_ == LOCALIZATION) {
			return true;
		} else if (working_mode_ == MAPPING || working_mode_ == SEC_MAPPING) {
			return back_end_->saveMap(saveMapDirectory, resolution, getOdomToMap(), start_index, end_index);
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

	inline PointCloudType::Ptr get_filter_lidar_cloud() {
		std::unique_lock<std::mutex> lk(mtx_lidar_cloud_); // TODO(jxl): 用自己的mtx
		return FilteredUndistortCloud_;
	}

	inline PointCloudType::Ptr get_odom_cloud() {
		std::unique_lock<std::mutex> lk(mtx_odom_cloud_);
		return UndistortCloudInOdom_;
	}

	inline PointCloudType::Ptr get_kdtree_cloud() const { return kdtreeCloud_; }
	inline std::deque<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> get_unoptimized_path() {
		std::unique_lock<std::mutex> lk(mtx_path_);
		return unoptimized_path_;
	}
	inline std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> get_optimized_path() {
		std::unique_lock<std::mutex> lk(mtx_path_);
		return optimized_path_;
	}
	inline std::map<int, int> getloopIndex() const { return back_end_->getloopIndex(); }

	inline Eigen::Isometry3d getOdomToMap() const {
		if (working_mode_ == LOCALIZATION) {
			return localization_->getOdomToMap();
		} else if (working_mode_ == MAPPING) {
			Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
			transform.matrix().block<3, 3>(0, 0) = p_imu_->initial_rotate_;
			return transform;
		} else if (working_mode_ == SEC_MAPPING) {
			return global_localization_->get_global_odom_to_map();
		} else {
			return Eigen::Isometry3d::Identity();
		}
	}

	inline Eigen::Isometry3d getLastOdomToMap() const {
		if (working_mode_ == LOCALIZATION) {
			return localization_->getLastOdomToMap();
		} else {
			return Eigen::Isometry3d::Identity();
		}
	}

	inline Eigen::Isometry3d getLidarInOdom() {
		std::unique_lock<std::mutex> lk(mtx_pose_);
		if (working_mode_ == MAPPING || working_mode_ == SEC_MAPPING) {
			return T_odom_lidar_;					// TODO(jxl): 什么含义
		} else if (working_mode_ == LOCALIZATION) { // TODO(jxl): curr_pose是lidar位置处的base_link位姿，
													// 如果是其它雷达(lidar和imu旋转不是单位阵，会有问题)
			Eigen::Isometry3d T_odom_b(Sophus::SE3d(current_pose_.imu_state.rot, current_pose_.imu_state.pos).matrix());
			Eigen::Isometry3d T_b_lidar(
				Sophus::SE3d(current_pose_.imu_state.offset_R_L_I, current_pose_.imu_state.offset_T_L_I).matrix());
			Eigen::Isometry3d temp = T_odom_b * T_b_lidar;
			return temp;
		} else {
			Eigen::Isometry3d temp = Eigen::Isometry3d::Identity();
			return temp;
		}
	}

	inline Localization_base get_current_pose() {
		std::unique_lock<std::mutex> lk(mtx_pose_); // TODO(jxl): 这个锁是用来锁T_odom_lidar_的
		return current_pose_;
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

	inline Eigen::Isometry3d getWheelInOdom() { return getLidarInOdom() * T_lidar_wheel_; }

	inline pcl::PointCloud<pcl::PointXYZI>::Ptr getLoadMap() const { return localization_->getLoadMap(); }

	inline std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& getLoadMapPoints() const {
		return localization_->getLoadMapPoints();
	}

	inline bool isGloalLocalizationSuccess() const { return globalLocalizationSuccess_; }

	inline Eigen::Isometry3d getLidarInMap() { //插值
		// Eigen::Isometry3d T_odom_b(Sophus::SE3d(current_pose_.imu_state.rot, current_pose_.imu_state.pos).matrix());
		// Eigen::Isometry3d T_b_lidar(Sophus::SE3d(current_pose_.imu_state.offset_R_L_I,
		// current_pose_.imu_state.offset_T_L_I).matrix());
		// Eigen::Isometry3d T_map_lidar  =  getOdomToMap() * T_odom_b * T_b_lidar;
		Eigen::Isometry3d T_map_lidar = getOdomToMap() * getLidarInOdom();
		return T_map_lidar;
	}

	inline Eigen::Isometry3d getWheelInMap() {
		return getLidarInMap() * T_lidar_wheel_; //插值
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
	inline PointCloudType::Ptr getCurrentMap() const { return back_end_->getCurrentMap(getOdomToMap()); }
	inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCurrentRGBMap() const { return back_end_->getCurrentRGBMap(); }

	// PointCloudType::Ptr getObstacleCloud()
	// {
	//     return ObstacleCloud_;
	// }
	// PointCloudType::Ptr getFilteredObstacleCloud()
	// {
	//     std::unique_lock<std::mutex> lk(mtx_obstacle_cloud_);
	//     return FilteredObstacleCloud_;
	// }

	inline double get_lidar_time() const { return lidar_end_time_; }

	// string print_SlamWorkMode(SlamWorkMode e){
	//     switch (e){
	//     CASE_STR(MAPPING);
	//     CASE_STR(SEC_MAPPING);
	//     CASE_STR(LOCALIZATION);
	//     default:
	//         break;
	//     }
	//     return "UNKNOW_SlamWorkMode!";
	// }

	void set_new_key_cloud_arrived(bool flag) { new_key_cloud_arrived_ = flag; }
	bool get_new_key_cloud_arrived() { return new_key_cloud_arrived_; }

	double get_hb_time_thread_localize() { return hb_time_thread_localize_.load(); }
	double get_hb_time_thread_loop_closure() { return hb_time_thread_loop_closure_.load(); }
	double get_hb_time_thread_secmap_relocalize() { return hb_time_thread_secmap_relocalize_.load(); }

	int get_local_thrd_status() { return local_thrd_status_.load(); }
	int get_slam_run_status() { return slam_run_status_.load(); }
	int get_secmap_relocal_thrd_status() { return secmap_relocal_thrd_status_.load(); }

   private:
	bool sync_packages(MeasureGroup& meas);
	void loopClosureThread();
	void sec_mapping_loopClosureThread();
	void localizationThread();
	// void relocalizationForMappingThread();

	void global_localization_for_sec_mapping_thread();

   private:
	////////////////////////////////////////////////////////////////////////////////////////////////////
	/// slam status    //
	/*************************************************** */
	/** @local_thrd_status_:
	 * 0: inactive
	 * 1: relocalize ing
	 * 2: relocalize failed
	 * 3: normal
	 * 4: local low accuracy
	 * 5: local failed
	 */
	std::atomic<int> local_thrd_status_{ 0 };

	/*************************************************** */
	/** @slam_run_status_:
	 * 0: inactive
	 * 1: normal
	 * 2: slam fail: cloud no enough point
	 */
	std::atomic<int> slam_run_status_{ 0 };

	/*************************************************** */
	/** @secmap_relocal_thrd_status_:
	 * 0: inactive
	 * 1: relocalize ing
	 * 2: relocalize failed
	 * 3: normal
	 */
	std::atomic<int> secmap_relocal_thrd_status_{ 0 };

	////////////////////////////////////////////////////////////////////////////////////////////////////
	// 各 线程、callback、timer heartbeat
	std::atomic<double> hb_time_thread_localize_;		   // status = LOCALIZATION
	std::atomic<double> hb_time_thread_loop_closure_;	   // status = MAPPING or SEC_MAPPING
	std::atomic<double> hb_time_thread_secmap_relocalize_; // status = SECMAPPING

	// LidarParam param;
	LidarSlamParam config_param_;
	int feats_down_size_thr_ = 100;
	bool flag_keep_only_last_lidar_ = true;
	bool new_key_cloud_arrived_ = false;

	bool need_localize_ = true;

	deque<double> time_buffer_;				  // 记录lidar时间, lidar header time
	deque<PointCloudType::Ptr> lidar_buffer_; // 记录特征提取或间隔采样后的lidar（特征）数据
	deque<std::shared_ptr<livox_ros::ImuMsg>> imu_buffer_;

	bool lidar_pushed_ = false;
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

	bool imu_file_shift_ = false; // TODO(jxl): 这个变量没有使用
	std::deque<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> unoptimized_path_;
	std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> optimized_path_;
	std::vector<livox_ros::ImuMsg> temp_imu_msg_; // TODO(jxl): 这个变量没有使用

	std::deque<double> pcd_file_; // TODO(jxl): 这个变量没有使用
	std::ofstream localization_file_;

	MeasureGroup Measures_;
	Eigen::Isometry3d T_odom_lidar_ = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d T_lidar_wheel_ = Eigen::Isometry3d::Identity();

	bool thread_run_ = true;
	bool globalLocalizationSuccess_ = false;
	bool loop_closure_wait_ = false; // TODO(jxl): 没有使用

	Localization_base localization_base_; // 每次点云处理后更新，并作为current_pose 积分结果的base
	Localization_base current_pose_;

	std::unique_ptr<KD_TREE<pcl::PointXYZINormal>> ikdtree_ = nullptr;

	std::unique_ptr<std::thread> thread_ = nullptr;
	std::unique_ptr<std::thread> global_localization_thread_ = nullptr;

	mutex mtx_imu_buffer_;
	mutex mtx_lidar_buffer_;
	mutex mtx_odom_cloud_;
	mutex mtx_lidar_cloud_;
	mutex mtx_obstacle_cloud_;
	mutex mtx_localization_base_;
	mutex mtx_current_pose_; // TODO(jxl): 没有使用
	mutex mtx_path_;
	mutex mtx_pose_;

	esekfom::esekf kf_;

	std::unique_ptr<ImuProcess> p_imu_ = nullptr;
	std::unique_ptr<BackEnd> back_end_ = nullptr;
	std::unique_ptr<Localization> localization_ = nullptr;
	std::unique_ptr<GlobalLocalization> global_localization_ = nullptr;
	std::unique_ptr<CloudMap> cloud_map_manager_ = nullptr;

	// stop local thread 专用
	// std::condition_variable cv_stop_local_;
	// std::mutex mtx_stop_thread_;
	// bool flag_stop_thread_ = false;

	PointCloudType::Ptr UndistortCloudInOdom_;
	PointCloudType::Ptr undistortCloud_; // lidar 系
	PointCloudType::Ptr FilteredUndistortCloud_;

	pcl::VoxelGrid<PointType> downSizeFilterCloud_;
	pcl::VoxelGrid<PointType> downSizeFilterCloud_test_;

	PointCloudType::Ptr kdtreeCloud_;
	// PointCloudType::Ptr ObstacleCloud_; // disable ObstacleCloud_ by pmm
	PointCloudType::Ptr FilteredObstacleCloud_;

	SlamWorkMode working_mode_ = UNKNOWN;
	// LocalizationStatus l_status_ = L_INACTIVE;
	// MappingStatus m_status_ = M_INACTIVE;
	// bool second_mapping_need_global_localization_ = false;

	// LocalizationStatus l_local_thread_status_ = L_INACTIVE;
	// LocalizationStatus l_slam_thread_status_ = L_INACTIVE;

	int global_localize_count_ = 0;
	int lidar_no_point_count_ = 0;
	localization_module::LocalizationModuleLogInfoManager* log_info_manager_;

	// cpu_set_t mask;
};

} // namespace lidar_slam
#endif

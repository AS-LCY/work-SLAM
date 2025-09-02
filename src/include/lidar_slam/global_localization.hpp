#ifndef FLBOT_LIDAR_SLAM_GLOBAL_LOCALIZATION_HPP
#define FLBOT_LIDAR_SLAM_GLOBAL_LOCALIZATION_HPP

// std
#include <sstream>
#include <string>
#include <vector>
// #include <filesystem> // c++17

// pcl
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

// lidar_slam
#include "lidar_slam/common_lib.h"
#include "lidar_slam/data_struct_define.h"
#include "lidar_slam/scan_context/Scancontext.h"

namespace lidar_slam {

class GlobalLocalization {
   public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	GlobalLocalization();
	~GlobalLocalization();

	bool global_localize(PointCloudType::Ptr cloudIn, Eigen::Isometry3d pose, Matrix3d initial_rotate,
						 double score_thr);

	bool fill_sc_manager(std::vector<ScInfo> input_sc_info);
	bool get_sc_manager_ready() { return sc_manager_ready_; }
	bool set_global_map(PointCloudType::Ptr loaded_global_map);
	bool get_global_map_ready() { return global_map_ready_; }

	Eigen::Isometry3d get_global_odom_to_map() { return global_odom_to_map_; }

   private:
	bool scancontex_search(PointCloudType::Ptr cloud_in, Matrix3d initial_rotate, std::pair<int, float>& best_match,
						   std::pair<double, double>& best_trans);

	Eigen::Matrix4d cal_init_transform(Matrix3d initial_rotate, std::pair<int, float> best_match,
									   std::pair<double, double> best_trans);

	bool registration_icp(PointCloudType::Ptr cloud_in, Eigen::Isometry3d pose, Eigen::Matrix4d init_guess,
						  double score_thr, Eigen::Isometry3d& res_global_odom_to_map);

   public:
   private:
	// load map
	std::shared_ptr<SCManager> sc_manager_;

	bool map_ready_ = false;
	PointCloudType::Ptr loaded_global_map_;
	std::vector<ScInfo> loaded_sc_info_; // 对应本地文件：data

	std::vector<KeyPose> loaded_keyframe_poses_;
	pcl::PointCloud<PointType>::Ptr loaded_key_point_;
	std::vector<PointCloudType::Ptr> loaded_keyframe_clouds_;

	// KeyMat polarcontext_invkeys_mat_;
	// std::vector<Eigen::MatrixXd> polarcontexts_;

	std::vector<Eigen::Isometry3d> accumulate_key_pose_;
	PointCloudType::Ptr accumulate_map_;

	// global-localize
	bool global_map_ready_ = false;
	bool sc_manager_ready_ = false;
	PointCloudType::Ptr test_match_cloud_; // debug

	// result of global localization
	Eigen::Isometry3d global_odom_to_map_ = Eigen::Isometry3d::Identity();
};

} // namespace lidar_slam

#endif // define FLBOT_LIDAR_SLAM_GLOBAL_LOCALIZATION_H
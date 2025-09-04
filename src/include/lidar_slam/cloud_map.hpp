#ifndef FLBOT_LIDAR_SLAM_CLOUD_MAP_HPP
#define FLBOT_LIDAR_SLAM_CLOUD_MAP_HPP

#include <sstream>
#include <string>
#include <vector>
// #include <filesystem> // c++17

// pcl
#define PCL_NO_PRECOMPILE
// #include <pcl/common/common.h>
// #include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

// lidar_slam
#include <logTracer/tracer.h>

#include "lidar_slam/common_lib.h"
#include "lidar_slam/data_struct_define.h"
#include "lidar_slam/scan_context/Scancontext.h"

namespace lidar_slam {

class CloudMap {
	DECL_CLASSNAME(CloudMap)
   public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	CloudMap();
	~CloudMap();

	/// @brief  从本地加载地图，用于二次建图
	/// @return 总的点云地图，关键帧点云，关键帧pose
	bool load_map_data(std::string map_dir);

	PointCloudType::Ptr get_loaded_cloud_map() { return loaded_global_map_; }
	std::vector<ScInfo, Eigen::aligned_allocator<ScInfo>> get_load_sc_info_() { return loaded_sc_info_; }
	std::vector<PointCloudType::Ptr> get_loaded_keyframe_clouds() { return loaded_keyframe_clouds_; }
	std::vector<KeyPose, Eigen::aligned_allocator<KeyPose>> get_loaded_keyframe_poses() {
		return loaded_keyframe_poses_;
	}
	bool get_map_data_status() { return map_data_ready_; }

   private:
	//从本地加载总的点云地图
	bool load_cloud_map(std::string map_dir);

	// 从本地加载关键帧信息（关键帧点云、关键帧pose）
	bool load_key_frames(std::string keyframe_dir);

	bool map_data_ready_ = false;
	PointCloudType::Ptr loaded_global_map_;
	std::vector<KeyPose, Eigen::aligned_allocator<KeyPose>> loaded_keyframe_poses_;
	std::vector<PointCloudType::Ptr> loaded_keyframe_clouds_;
	std::vector<ScInfo, Eigen::aligned_allocator<ScInfo>> loaded_sc_info_;

	/// 加载data数据所用
	std::shared_ptr<SCManager> sc_manager_;

	KeyMat polarcontext_invkeys_mat_;
	std::vector<Eigen::MatrixXd> polarcontexts_;
	pcl::PointCloud<PointType>::Ptr loaded_key_point_;
	std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> accumulate_key_pose_;
	PointCloudType::Ptr accumulate_map_;
};

} // namespace lidar_slam

#endif // end define: FLBOT_SLAM_CLOUD_MAP_H
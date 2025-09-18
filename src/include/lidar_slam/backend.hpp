#ifndef BACKEND_H
#define BACKEND_H
#pragma once
#include <omp.h>

#include <mutex>
#include <shared_mutex>
// #include <math.h> // ikd_Tree.h 中已包含
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <thread>
// #include <filesystem> // c++17

#include <Eigen/Core>
#include <csignal>

// #include <opencv2/opencv.hpp>
// #include <opencv2/core.hpp>

// pcl
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>

#include <pcl/search/impl/search.hpp>
// #include <pcl/filters/impl/voxel_grid.hpp>

// #include <pcl/registration/ndt.h> // 没用上
// #include <pcl/filters/crop_box.h> //没用上
// #include <pcl/filters/passthrough.h> // getObstacleMap 中使用，此函数未使用上

// gstam
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <logTracer/tracer.h>

// lidar_slam
#include "lidar_slam/common_lib.h"
#include "lidar_slam/data_struct_define.h"
#include "lidar_slam/ikd_Tree.h"
#include "lidar_slam/scan_context/Scancontext.h"

namespace lidar_slam {

class BackEnd {
	DECL_CLASSNAME(BackEnd)
   public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	BackEnd(const Eigen::Isometry3d& T_lidar_imu, float dist, float angle, float loop_dist, float loop_time,
			int loop_skip_key, float loop_icp_score);
	~BackEnd();

	void saveCurrentCloud(PointCloudType::Ptr points, Eigen::Isometry3d pose);
	PointCloudType::Ptr getCurrentMap();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCurrentRGBMap();
	bool saveKeyFramesAndFactor(const Eigen::Isometry3d& init_T_map_odom, Eigen::Isometry3d transformTobeMapped,
								double time);
	void performLoopClosure(double time);

	// if start_index == end_index == 0; save all;
	bool saveMap(std::string saveMapDirectory, double resolution, int start_index, int end_index);
	bool correctPoses();
	void recontructIKdTree(KD_TREE<PointType>& ikdtree, double kdTreeReconstructRadius,
						   float kdTreeReconstructKeyFrameLeafSize, double kdTreeReconstructPointLeafSize);

	KeyPose getCurrentPose() {
		std::unique_lock<std::shared_mutex> keyframe_poses_lock(mtxPose_);
		return KeyPoses_.back();
	}

	inline std::vector<KeyPose> getKeyframePoses() {
		std::unique_lock<std::shared_mutex> keyframe_poses_lock(mtxPose_);
		return KeyPoses_;
	}

	inline Eigen::Isometry3d getOdomToMap() {
		std::unique_lock<std::mutex> T_map_odom_lock(mtxTmapOdom_);
		return T_map_odom_;
	}

	inline std::vector<std::pair<int, int>> getAllLoopEdges() {
		std::unique_lock<std::mutex> lk(mtxLoopInfo_);
		return all_loop_edges_;
	}

	int getCurrentPoseIndex() {
		int temp_index = int(KeyPoses_.size()) - 1;
		int curr_index = temp_index < 0 ? 0 : temp_index;
		return curr_index;
	}
	std::map<int, int> getloopIndex() { return loopIndexContainer_; }
	PointCloudType::Ptr getTestCloud() { return gravityAlignedCLoud_; }

	bool get_loaded_key_cloud_status() { return loaded_key_clouds_ready_; }

	bool set_loaded_key_clouds(std::vector<PointCloudType::Ptr> input_vec_key_clouds,
							   std::vector<ScInfo, Eigen::aligned_allocator<ScInfo>> input_vec_sc_info,
							   std::vector<KeyPose, Eigen::aligned_allocator<KeyPose>> input_vec_key_poses);

   private:
	bool saveFrame(Eigen::Isometry3d transformTobeMapped);

	void addOdomFactor(Eigen::Isometry3d transformTobeMapped);
	void addLoopFactor();

	bool detectLoopClosureDistance(int* latestID, int* closestID, double time);
	void loopFindNearKeyframes(PointCloudType::Ptr& nearKeyframes, const int& key, const int& searchNum);
	void loopFindNearKeyframesWithRespectTo(PointCloudType::Ptr& nearKeyframes, const int& key, const int& searchNum,
											const int _wrt_key);

	inline bool checkPreviousLoopFailure(const int& key, const int& value) const {
		if (failed_loop_indexs_.empty()) {
			return false;
		}
		auto range = failed_loop_indexs_.equal_range(key);
		for (auto it = range.first; it != range.second; ++it) {
			if (it->second == value) {
				return true;
			}
		}
		return false;
	}

   private:
	bool loaded_key_clouds_ready_ = false;

	std::vector<std::pair<int, int>> all_loop_edges_;

	pcl::PointCloud<PointType>::Ptr KeyPoint_;
	std::vector<KeyPose> KeyPoses_;
	pcl::PointCloud<PointType>::Ptr CopyKeyPoint_;
	std::vector<KeyPose> CopyKeyPoses_;
	std::vector<PointCloudType::Ptr> KeyFrameCloud_;
	std::vector<KeyPose> OdomKeyPoses_;
	PointCloudType::Ptr show_map_ = nullptr;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_rgb_map_;

	float keyframeDistThreshold_;  //  判断是否为关键帧的距离阈值
	float keyframeAngleThreshold_; //  判断是否为关键帧的角度阈值
	float loopKeyframeSearchRadius_;
	float loopKeyframeSearchTimeDiff_;
	int loopKeyframeSearchSkipKey_;
	float loopIcpScore_ = 0.5;

	std::multimap<int, int> failed_loop_indexs_;
	map<int, int> loopIndexContainer_;
	vector<pair<int, int>> loopIndexQueue_;
	vector<gtsam::Pose3> loopPoseQueue_;
	vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue_;

	gtsam::NonlinearFactorGraph gtSAMgraph_;
	gtsam::ISAM2* isam_;
	gtsam::Values initialEstimate_;
	gtsam::Values isamCurrentEstimate_;
	gtsam::ISAM2Params parameters_;

	bool aLoopIsClosed_ = false;
	int show_index_ = 0;

	SCManager scManager_;

	pcl::VoxelGrid<PointType> downSizeFilterICP_;
	PointCloudType::Ptr gravityAlignedCLoud_;

	std::shared_mutex mtxPose_;
	std::shared_mutex mtxPose_copy_;
	std::shared_mutex mtxCloud_;
	std::mutex mtxLoopInfo_;
	std::mutex mtxCurrentMap_;
	std::mutex mtxCurrentRGBMap_;

	Eigen::Isometry3d T_map_odom_ = Eigen::Isometry3d::Identity();
	std::mutex mtxTmapOdom_;

	Eigen::Isometry3d T_lidar_imu_ = Eigen::Isometry3d::Identity();
};
} // namespace lidar_slam

#endif

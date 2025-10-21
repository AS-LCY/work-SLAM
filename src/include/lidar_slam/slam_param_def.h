
#ifndef FLBOT_LIDAR_SLAM_PARAM_DEF_H
#define FLBOT_LIDAR_SLAM_PARAM_DEF_H

#include <string>
#include <vector>

#include "lidar_slam/common_lib.h"

namespace lidar_slam {
struct ExtrinsicParam {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	bool extrinsic_est_en;
	V3D extrinT;
	M3D extrinR;
	Eigen::Isometry3d T_lidar_baselink = Eigen::Isometry3d::Identity();
	Eigen::Matrix3d R_baselink_IMU = Eigen::Matrix3d::Identity();
	Eigen::Isometry3d T_imu_baselink = Eigen::Isometry3d::Identity();
};

struct LidarPreprocParam {
	int lidar_type = 1;
	std::string sub_lidar_topic = "";
	std::string sub_imu_topic = "";

	int extract_cloud_method;

	///// extract by ring_feature: vanjee & rs
	int cloud_column_count = 1200; // rs & vanjee
	int cloud_ring_count = 48;	   // rs & vanjee

	float edge_curvature_thr = 1.0;
	float surf_curvature_thr = 0.1;
	float surf_leafsize = 0.4;

	float blind_distance;
	float max_distance;
	std::vector<double> z_range = { -5.0, 20.0 };
	int keep_lidar_num_before_curr = 1;
	int point_filter_num = 2;
	int ring_filter_num = 1;
	int cloud_size_to_keep = 2000;
	double leafsize;
	std::vector<double> leafsize_vec = { 0.2, 0.5 };
	std::vector<double> voxel_region_xyz;
	double boundary_z = 2;
	double time_cost_thr_print = 10;
};

struct ReLocalizationParam {
	double score_thr;
	int time_out_thr; // 以秒为单位
};

struct MappingParam {
	double acc_cov;
	double gyr_cov;
	double b_acc_cov;
	double b_gyr_cov;
	double cloud_leaf_size;
	double key_frame_distance;
	double key_frame_angle;
	double loopSearchDistance;
	double loopSearchTimeDiff;
	int loopSearchSkipKey;
	double loopIcpScore;
	double save_map_resolution;
};

struct LocalizationParam {
	float fgicp_peroid_sec = 1;
	double cloud_leaf_size_localize = 0.3;
	int fgicp_thread_num = 2;
	float fgicp_trans_eps = 1e-2;
	int fgicp_max_iter = 64;
	float fgicp_max_corres_dist = 2.0;
	int fgicp_max_corres_num = 20;
	float fgicp_inlier_max_valid_point_dist = 40.0;
	float fgicp_inlier_max_corres_dist = 0.5;
	float fgicp_inlier_rate_thr = 0.8;
	float fgicp_inlier_avg_error_thr = 0.25;
};

struct IkdTreeParam {
	double cube_len;
	double det_range;
	double kdTreeReconstructRadius;
	double kdTreeReconstructKeyFrameLeafSize;
	double kdTreeReconstructPointLeafSize;
	double map_leaf_size;
};

} // namespace lidar_slam

#endif
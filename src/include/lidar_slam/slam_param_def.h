
#ifndef FLBOT_LIDAR_SLAM_PARAM_DEF_H
#define FLBOT_LIDAR_SLAM_PARAM_DEF_H

#include <vector>
#include <string>
#include "lidar_slam/common_lib.h"

namespace lidar_slam{
struct ExtrinsicParam{
    bool extrinsic_est_en;
    V3D extrinT; 
    M3D extrinR;
    Eigen::Isometry3d  T_wheel_lidar = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d  T_lidar_wheel = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d R_baselink_IMU = Eigen::Matrix3d::Identity();
};

struct LidarPreprocParam{
    int lidar_type=1;
    std::string sub_lidar_topic="";
    std::string sub_imu_topic="";
    int line_count;
    double blind_distance;
    bool flag_keep_only_last_lidar=false;
    int point_filter_num = 2;
    int ring_filter_num = 1;
    std::vector<double> point_filter_distance;
    bool feature_enabled;
    // bool simple_voxel_enabled;
    int extract_cloud_method;
    double leafsize;
    std::vector<double> leafsize_vec={0.2, 0.5};
    std::vector<double> voxel_region_xyz;
    double boundary_z = 2;
    double obstacle_max_range;
    double obstacle_max_height;
    double obstacle_min_height; // above wheel center
    double obstacle_filter_size;
    double grid_size;

};


struct ReLocalizationParam{
    double score_thr;
    int time_out_thr;// 以秒为单位
};

struct MappingParam{
    double acc_cov;
    double gyr_cov;
    double b_acc_cov;
    double b_gyr_cov;
    double cloud_leaf_size;
    double key_frame_distance;
    double key_frame_angle;
    double loopSearchDistance;
    double loopSearchTimeDiff;
    int  loopSearchSkipKey;
    double loopIcpScore;
    // bool use_ele_pcd_flag;
    // bool save_ele_pcd_flag;
    // std::string save_map_dir;
    double save_map_resolution;
};

struct LocalizationParam{
    // std::string load_map_dir;
    double fgicp_score_thr = 0.1;
    int fgicp_freq = 1;
    int filter_method = 0;
    float fst_order_k = 0.7;
    double odom2map_delta_thr = 0.025;
    double odom2map_delta_set = 0.01;
    double lidar_ratio = 0.5;
    double baselink_dy_thr = 0.1;
    double baselink_dx_thr = 0.1;
    double baselink_dyaw_thr = 1;
    int window_size = 5;
    int filter_freq = 200;
    bool using_turning_proc = false;
    double chassis_linear_velocity_thr = 0.02;
    double motionless_chassis_ratio = 1.0;
    double lidar_cbk_delay_thr = 1.0;

};

struct SecondMappingParam{
    // std::string load_map_dir;
};

struct IkdTreeParam{
    double cube_len;
    double det_range;
    double kdTreeReconstructRadius;
    double kdTreeReconstructKeyFrameLeafSize;
    double kdTreeReconstructPointLeafSize;
    double map_leaf_size;
};


} // namespace lidar_slam


#endif
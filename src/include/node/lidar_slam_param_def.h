#ifndef FLBOT_LIDAR_SLAM_PARAM_DEF_H
#define FLBOT_LIDAR_SLAM_PARAM_DEF_H

#include <string>
#include "lidar_slam/common_lib.h"

namespace lidar_slam{

struct CommonParam{
    bool time_sync_en = false;
    bool localization_mode = false;
    bool offline_mode = false;
    bool fast_mode = false;
    bool just_show_mode = false;
    bool show_rviz = true;
    double log_keep_time = 0;
    std::string save_log_dir="/home/";
    int map_relative_to=0;
    std::string map_directory="/map/";
    std::string sub_topic_ctrl_cmd;
    std::string pub_topic_module_status;
};

struct ExtrinsicParam{
    bool extrinsic_est_en;
    V3D extrinT; 
    M3D extrinR;
    Eigen::Isometry3d  T_wheel_lidar = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d  T_lidar_wheel = Eigen::Isometry3d::Identity();
};

struct LidarPreprocParam{
    int line_count;
    double blind_distance;
    int point_filter_num;
    bool feature_enabled;
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
    bool use_ele_pcd_flag;
    bool save_ele_pcd_flag;
    // std::string save_map_dir;
    double save_map_resolution;
};

struct LocalizationParam{
    // std::string load_map_dir;
    double fgicp_score_thr = 0.1;
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


struct LidarSlamParam{
    CommonParam common;
    ExtrinsicParam extrinsic;
    LidarPreprocParam lidar_preproc;
    ReLocalizationParam re_localization;
    MappingParam mapping;
    LocalizationParam localization;
    SecondMappingParam sec_mapping;
    IkdTreeParam ikdtree;
};

} // namespace lidar_slam


#endif
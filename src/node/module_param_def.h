#ifndef FLBOT_LOCALIZATION_MODULE_PARAM_DEF_H
#define FLBOT_LOCALIZATION_MODULE_PARAM_DEF_H

#include <string>
#include "lidar_slam/common_lib.h"
#include "lidar_slam/slam_param_def.h"

namespace lidar_slam{

struct CommonParam{
    bool run_on_mower = true;
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
    std::string pub_topic_module_health;
    
    std::string pub_topic_module_loginfo;
    std::string pub_topic_slipping;
    int receive_lidar_freq = 10;
    double slam_lose_rate_time_thr = 0.1;
    int lidar_no_point_count_thr = 10;
    int feats_down_size_thr = 100;
    bool use_pose_filter = false;
    std::vector<int> cpu_id;
};

struct DetectSlipParam{
    double detect_window_time_range = 2;
    int slipping_count_thr = 5;
    double slipping_dist_thr = 0.2;

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
    DetectSlipParam detect_slip;
};

} // namespace lidar_slam


#endif
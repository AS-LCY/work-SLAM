#ifndef FLBOT_LOCALIZATION_MODULE_PARAM_DEF_H
#define FLBOT_LOCALIZATION_MODULE_PARAM_DEF_H

#include <string>
#include "lidar_slam/common_lib.h"
#include "lidar_slam/slam_param_def.h"

namespace lidar_slam{

struct CommonParam{
    bool time_sync_en = false;
    bool localization_mode = false;
    bool offline_mode = false;
    bool temp_test_offline = false;
    bool fast_mode = false;
    bool just_show_mode = false;
    bool show_rviz = true;
    double log_keep_time = 0;
    std::string save_log_dir="/home/";
    int map_relative_to=0;
    std::string map_directory="/map/";
    std::string sub_topic_ctrl_cmd;
    std::string pub_topic_module_status;
    std::string pub_topic_module_loginfo;
    int receive_lidar_freq = 10;
    double slam_lose_rate_time_thr = 0.1;
    int lidar_no_point_count_thr = 10;
    int feats_down_size_thr = 100;
    std::vector<int> cpu_id;
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
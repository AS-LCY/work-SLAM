#ifndef FLBOT_LOCALIZTION_MODULE_DETECT_SLIPPING
#define FLBOT_LOCALIZTION_MODULE_DETECT_SLIPPING

#include <mutex>
#include <queue>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

#include "fairland_msgs/chassic_data.h"
#include "fairland_msgs/LocalizationModuleLogInfo.h"

#include "lidar_slam/common_lib.h"

#include "node/log_info_manager.hpp"
#include "node/param_manager.hpp"

namespace localization_module{
    

class DetectSlipping{
private:
    /* data */
public:
    DetectSlipping(/* args */);
    ~DetectSlipping();

    bool detect_by_chassis_and_lidar(int &slip_flag);
    bool detect_by_chassis_and_imu();

    void update_chassis(fairland_msgs::chassic_data cur_chassis_msg);
    void update_lidar_by_segment(geometry_msgs::PoseStamped pose);
    void update_lidar_by_distance(geometry_msgs::PoseStamped pose);
    void update_imu(sensor_msgs::Imu);
    void reset();

    bool get_lidar_queue_init(){
        return lidar_queue_init_;
    }


private:
    double cal_dist_along_heading(geometry_msgs::PoseStamped last_pose, geometry_msgs::PoseStamped curr_pose);
    double cal_dist(geometry_msgs::PoseStamped last_pose, geometry_msgs::PoseStamped curr_pose);
    void init_lidar_queue();
    void init_chassis_queue();


    bool load_params();


public:
    LocalizationModuleLogInfoManager * log_info_manager_;

private:
    // param
    double param_detect_window_time_range_ = 2.0;
    double param_slipping_dist_thr_ = 2.0;
    int param_slipping_count_thr_ = 5;

    std::queue<double> dist_que_;
    std::queue<geometry_msgs::PoseStamped> pose_que_;
    std::queue<sensor_msgs::Imu> imu_que_;
    std::queue<fairland_msgs::chassic_data> chassis_que_;

    std::mutex chassis_que_mtx_;

    int slipping_count_ = 0;
    
    bool lidar_queue_init_ = false;
    bool chassis_queue_init_ = false;

    double lidar_sum_dist_ = 0;
    double chassis_sum_dist_ = 0;


};


} // namespace localization_module




#endif

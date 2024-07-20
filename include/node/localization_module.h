#ifndef LOCALIZATION_MODULE_H
#define LOCALIZATION_MODULE_H

#include <string>
#include <vector>
#include <cstdlib>
// ros
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>

// ros-msg
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Eigen
#include <Eigen/Core>

// pcl
#define PCL_NO_PRECOMPILE
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>

// cv
#include <opencv2/opencv.hpp>

// livox
// #include <livox_ros_driver2/CustomMsg.h>
#include "fairland_msgs/LivoxCustomMsg.h"

#include "lidar_slam.hpp"
#include <Viewer.hpp>
#include "v4l2cam.h"
#include "publish_common.h"
#include "point_type_livox_def.h"
#include "lidar_slam_param_def.h"
#include "common_lib.h"

namespace localization_module{

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace sensor_msgs;

enum SlamCtrlCmd{
    START_MAPPING           = 1000,  // 开始建图
    MAPPING_POINT_BEGIN     = 2000,  // 设置起点
    MAPPING_ELE_DELETE      = 3000,  // 创建地图元素过程中，清除当前元素（当前元素还未完成创建）
    MAPPING_POINT_END       = 4000,  // 设置终点
    START_SEC_MAPPING       = 5000,  // 重定位->建图
    START_LOCALIZATION      = 6000,  // 重定位->定位
    EXIT_LOCALIZATION       = 7000,  // 退出定位
    EXIT_MAPPING            = 9000,  // 退出建图
    CMD_MAX
};

enum ModuleStatus{
    MODULE_IDLE = 0,
    MODULE_MAPPING =1,
    MODULE_SEC_MAPPING =2,
    MODULE_LOCALIZATION = 3
};

enum MappingStatus{
    MAPPING_INACTIVE = 0,
    MAPPING_RE_LOCALIZING =1,
    MAPPING_CREATING_ELE = 2,
    MAPPING_STANDBY =3
};

enum LocalizationStatus{
    LOCALIZATION_INACTIVE = 0,
    LOCALIZATION_RE_LOCALIZING =1,
    LOCALIZATION_LOCALIZING = 2
};

// enum MappingStatus{
//     MAPPING_INACTIVE = 0,    // 初始状态
//     MAPPING_STARTED = 1,    // 在建图中，等待设置起点
//     STARTPOINT_SET = 2,     // 已设置起点，等待设置终点(或闭合)
//     ENDPOINT_SET  = 3       // 已设置终点（或已闭合），当前元素创建结束
// };

// enum SlamMode{
//     INACTIVE = 0,       // 未激活状态
//     MAPPING = 1,        // 建图模式
//     LOCALIZATION = 2    // 定位模式
// };

class LocalizationModule{
public:
    LocalizationModule(){};
    LocalizationModule(const std::string work_path, ModuleStatus init_status);
    ~LocalizationModule();


private:
    void show_thread();

    void load_params();

    bool load_lidar_slam_param();
    bool create_ROS_IO();

    // 建图
    // void start_mapping(bool module_mode);
    void start_mapping(ModuleStatus set_status);
    void mark_start_point();
    void mark_end_point(int save_id);
    void clear_curr_element();
    // void start_second_mapping(bool localization_mode, int map_id);
    void start_second_mapping(ModuleStatus set_status, int map_id);
    void stop_mapping();

    void start_localization(ModuleStatus set_status, int map_id);

    void start_localization(bool module_mode, int map_id);
    void stop_localization();


    bool run_module_by_set_status(ModuleStatus set_status);
    void make_slam_obj(string work_path, bool localization_mode, bool offline_mode, bool sec_mapping);
    bool make_slam_obj(lidar_slam::LidarSlamParam yaml_param, ModuleStatus set_status);

    // void make_slam_obj(string work_path, bool slam_mode, bool offline_mode);
    void release_slam_obj();

    // callback 
    void mapping_ctrl_cbk(const std_msgs::UInt32 &msg_in);
    void slam_dealt_timer(const ros::TimerEvent &event);

    // void command_cbk(const std_msgs::Int32 &msg_in);
    // void livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg_in);
    void livox_pcl_cbk(const fairland_msgs::LivoxCustomMsg::ConstPtr &msg_in);

    void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);

    void livox_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &ros_msg);

    void publish_unoptimized_path(const std::deque<Eigen::Isometry3d> path, ros::Publisher pubUnoptimizedPath);
    void publish_optimized_path(const std::vector<Eigen::Isometry3d> path, std::string frame, ros::Publisher pubOptimizedPath);

    string print_ModuleStatus(ModuleStatus e){
        switch (e){
        CASE_STR(MODULE_IDLE);
        CASE_STR(MODULE_MAPPING);
        CASE_STR(MODULE_SEC_MAPPING);
        CASE_STR(MODULE_LOCALIZATION);
        default:
            break;
        }
        return "UNKNOW_MappingStatus!";
    }

    string print_MappingStatus(MappingStatus e){
        switch (e){
        CASE_STR(MAPPING_INACTIVE);
        CASE_STR(MAPPING_RE_LOCALIZING);
        CASE_STR(MAPPING_CREATING_ELE);
        CASE_STR(MAPPING_STANDBY);
        default:
            break;
        }
        return "UNKNOW_MappingStatus!";
    }

    string print_LocalizationStatus(LocalizationStatus e){
        switch (e){
        CASE_STR(LOCALIZATION_INACTIVE);
        CASE_STR(LOCALIZATION_RE_LOCALIZING);
        CASE_STR(LOCALIZATION_LOCALIZING);
        default:
            break;
        }
        return "UNKNOW_LocalizationStatus!";
    }

    // string print_SlamMode(SlamMode e){
    //     switch (e){
    //     CASE_STR(INACTIVE);
    //     CASE_STR(MAPPING);
    //     CASE_STR(LOCALIZATION);
    //     default:
    //         break;
    //     }
    //     return "UNKNOW_SlamMode!";
    // }


    template <class T>
    void get_param(const std::string& param_str, T& param, bool* is_success){
        if(!nh_.getParamCached(param_str,param)){
            ROS_WARN("load param %s failed", param_str.c_str());
            *is_success = false;
        }
    };

public:

private:
    ros::NodeHandle nh_;
    ros::Timer timer_slam_;

    ros::Subscriber sub_mapping_ctrl_;
    ros::Subscriber sub_pointcloud2_;
    ros::Subscriber sub_imu_;


    // slam node
    std::unique_ptr<lidar_slam::LidarSlam> slam_;

    // 通用
    string curr_dir_; // localization_module CMake dir
    bool localization_mode_ = false;
    bool offline_mode_ = false; // unused
    bool just_show_mode_ = false;
    bool show_rviz_ = false;
    bool fast_mode_ = false; // unused, 只有参数读入
    string log_folder_;

    // show thread
    lidar_slam::Control_status control_status_;

    // 模块 localization module
    ModuleStatus set_module_status_ = MODULE_IDLE;
    ModuleStatus running_module_status_ = MODULE_IDLE;

    // 建图 *******************************************
    // bool running_slam_ = false;
    // SlamMode slam_mode_ = INACTIVE;
    MappingStatus mapping_status_ = MAPPING_INACTIVE; // if change to module_status_??
    LocalizationStatus localization_status_ = LOCALIZATION_INACTIVE;

    // 二次建图
    // bool second_mapping_ = false;
    int start_index_ = -1;
    int end_index_ = -1;

    // 定位 *******************************************


    // other thread
    std::thread show_thread_;
    int show_load_map_ = 0;

    // 应该是目前没在用
    // ros::ServiceServer srvSaveMap; // 应该是目前没在用
    // std::vector<Eigen::Isometry3d> keyPoses;

    nav_msgs::Path unoptimized_path_msg;
    nav_msgs::Path optimized_path_msg;
    ros::Publisher pubOdomCloud;
    ros::Publisher pubBodyCloud;
    ros::Publisher pubObstacleCloud;
    ros::Publisher pubFilteredObstacleCloud;
    ros::Publisher pubTestCloud;
    ros::Publisher pubKdtreeCloud;
    ros::Publisher pubOptimizedPath;
    ros::Publisher pubUnoptimizedPath; 
    ros::Publisher pubLoopConstraintEdge;
    ros::Publisher pubOdomAftMapped;
    ros::Publisher pubLidarInMap;
    ros::Publisher pubLoadMap;
    ros::Publisher pubKeyframePose;
    ros::Publisher pubRgbCloud;
    //ros::Publisher image_pub;    


    /// params load from yaml
    lidar_slam::LidarSlamParam slam_param_;

    
};






}// namespace localization_module

#endif // LOCALIZATION_MODULE_H

#ifndef LOCALIZATION_MODULE_H
#define LOCALIZATION_MODULE_H

#include <string>
#include <vector>
#include <cstdlib>
#include <filesystem> // c++17 标准
// ros
#include <ros/ros.h>
#include <ros/package.h>
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
// #define PCL_NO_PRECOMPILE
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
// #include <opencv2/opencv.hpp>

// msg
// #include <livox_ros_driver2/CustomMsg.h>
#include "fairland_msgs/LivoxCustomMsg.h"
#include "fairland_msgs/LocalizationModuleStatus.h"

#include "lidar_slam/common_lib.h"
#include "lidar_slam/lidar_slam.hpp"
#include "lidar_slam/Viewer.hpp"
#include "livox_datatype/livox_ros_datatype_def.h"

#include "node/publish_common.h"
#include "node/lidar_slam_param_def.h"
#include "node/point_type_livox_def.h"

#include "v4l2cam.h"

namespace localization_module{

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace sensor_msgs;

enum SlamCtrlCmd{
    START_MAPPING           = 1000,  // 开始建图
    START_SEC_MAPPING       = 2000,  // 重定位->建图，二次建图
    MAPPING_POINT_BEGIN     = 3000,  // 设置起点
    MAPPING_ELE_DELETE      = 4000,  // 创建地图元素过程中，清除当前元素（当前元素还未完成创建）
    MAPPING_POINT_END       = 5000,  // 设置终点
    EXIT_MAPPING            = 6000,  // 退出建图
    START_LOCALIZATION      = 7000,  // 重定位->定位
    EXIT_LOCALIZATION       = 8000,  // 退出定位
    CMD_MAX
};

enum ModuleStatus{
    MODULE_IDLE = 0,
    MODULE_MAPPING =1,
    MODULE_SEC_MAPPING =2,
    MODULE_LOCALIZATION = 3,
    MODULE_STARTING_SLAM = 4,
    MODULE_STOPPING_SLAM = 5
};

enum MappingStatus{
    M_INACTIVE = 0,
    M_RELOCALIZING =1,
    M_RELOCALIZE_FAILED =2,
    M_CREATING_ELE = 3,
    M_STANDBY =4
};

enum LocalizationStatus{
    L_INACTIVE = 0,
    L_RELOCALIZING =1,
    L_RELOCALIZE_FAILED = 2,
    L_NORMAL = 3,
    L_LOW_ACCURACY = 4,
    L_FAILED = 5
};

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
    bool start_mapping(ModuleStatus set_status);
    bool mark_start_point();
    bool mark_end_point(int save_id);
    bool clear_curr_element();
    // void start_second_mapping(bool localization_mode, int map_id);
    bool start_second_mapping(ModuleStatus set_status, int map_id);
    bool stop_mapping();

    bool start_localization(ModuleStatus set_status, int map_id);

    // void start_localization(bool module_mode, int map_id);
    bool stop_localization();


    bool init_module_by_set_status(ModuleStatus set_status);
    // void make_slam_obj(string work_path, bool localization_mode, bool offline_mode, bool sec_mapping);
    bool make_slam_obj(lidar_slam::LidarSlamParam yaml_param, ModuleStatus set_status);

    // void make_slam_obj(string work_path, bool slam_mode, bool offline_mode);
    void release_slam_obj();

    // callback 
    void localization_module_ctrl_cbk(const std_msgs::UInt32 &msg_in);
    void slam_dealt_timer(const ros::TimerEvent &event);
    void pub_module_status_timer(const ros::TimerEvent &event);

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
        CASE_STR(MODULE_STARTING_SLAM);
        CASE_STR(MODULE_STOPPING_SLAM);
        default:
            break;
        }
        return "UNKNOW_MappingStatus!";
    }

    string print_MappingStatus(MappingStatus e){
        switch (e){
        CASE_STR(M_INACTIVE);
        CASE_STR(M_RELOCALIZING);
        CASE_STR(M_RELOCALIZE_FAILED);
        CASE_STR(M_CREATING_ELE);
        CASE_STR(M_STANDBY);
        default:
            break;
        }
        return "UNKNOW_MappingStatus!";
    }

    string print_LocalizationStatus(LocalizationStatus e){
        switch (e){
        CASE_STR(L_INACTIVE);
        CASE_STR(L_RELOCALIZING);
        CASE_STR(L_RELOCALIZE_FAILED);
        CASE_STR(L_NORMAL);
        CASE_STR(L_LOW_ACCURACY);
        default:
            break;
        }
        return "UNKNOW_LocalizationStatus!";
    }

    string print_SlamCtrlCmd(SlamCtrlCmd e){
        switch (e){
        CASE_STR(START_MAPPING);
        CASE_STR(START_SEC_MAPPING);
        CASE_STR(MAPPING_POINT_BEGIN);
        CASE_STR(MAPPING_ELE_DELETE);
        CASE_STR(MAPPING_POINT_END);
        CASE_STR(EXIT_MAPPING);
        CASE_STR(START_LOCALIZATION);
        CASE_STR(EXIT_LOCALIZATION);
        CASE_STR(CMD_MAX);
        default:
            break;
        }
        return "UNKNOW_SlamCtrlCmd!";
    }

    template <class T>
    void get_param(const std::string& param_str, T& param, bool* is_success){
        if(!nh_.getParamCached(param_str,param)){
            ROS_WARN("load param %s failed", param_str.c_str());
            *is_success = false;
        }
    };

    ros::NodeHandle nh_;
    ros::Timer timer_slam_;
    ros::Timer timer_pub_module_status_;

    ros::Subscriber sub_mapping_ctrl_;
    ros::Subscriber sub_pointcloud2_;
    ros::Subscriber sub_imu_;

    ros::Publisher pub_localization_module_status_;


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
    // bool running_slam_ = false;
    ModuleStatus last_running_module_status_ = MODULE_IDLE;
    ModuleStatus set_module_status_ = MODULE_IDLE;
    ModuleStatus running_module_status_ = MODULE_IDLE;

    // 建图 *******************************************
    MappingStatus mapping_status_ = M_INACTIVE;
    int start_index_ = -1;
    int end_index_ = -1;

    // 定位 *******************************************
    LocalizationStatus localization_status_ = L_INACTIVE;


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

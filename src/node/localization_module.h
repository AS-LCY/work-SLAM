#ifndef LOCALIZATION_MODULE_H
#define LOCALIZATION_MODULE_H

#include <string>
#include <vector>
#include <cstdlib>
#include <chrono>
#include <atomic>

#include "boost/thread.hpp"

// #include <filesystem> // c++17 标准
// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <image_transport/image_transport.h>

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

// // pcl
// // #define PCL_NO_PRECOMPILE
// #include <pcl/search/impl/search.hpp>
// #include <pcl/range_image/range_image.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/common/common.h>
// #include <pcl/common/transforms.h>
// #include <pcl/registration/icp.h>
// #include <pcl/registration/ndt.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/filters/filter.h>
// #include <pcl/filters/crop_box.h>

// cv
// #include <opencv2/opencv.hpp>

// msg
// #include <livox_ros_driver2/CustomMsg.h>
#include "fairland_msgs/LivoxCustomMsg.h"
#include "fairland_msgs/LocalizationModuleStatus.h"
#include "fairland_msgs/LocalizationModuleLogInfo.h"



#include "lidar_slam/common_lib.h"
#include "lidar_slam/lidar_slam.hpp"
// #include "lidar_slam/Viewer.hpp"
#include "lidar/livox/ros_livox_datatype_def.h"

#include "node/module_param_def.h"
#include "node/module_status_def.h"
#include "node/log_info_manager.hpp"
#include "node/param_manager.hpp"

// 另一个节点中定义
#include "fairland_msgs/chassic_data.h"

// lidar
#include "lidar/lidar_preproc_factory.hpp"
#include "lidar/lidar_preproc_parent.h"
#include "lidar/livox/pcl_point_type_def_lvx.h"
#include "lidar/livox/lidar_preproc_Mid360.h"
#include "lidar/robosense/pcl_point_type_def_rbs.h"
#include "lidar/robosense/lidar_preproc_Airy.h"

// #include "v4l2cam.h"

namespace localization_module{

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace sensor_msgs;
using namespace lidar_slam;
// using namespace fairland_msgs::LocalizationModuleStatus;

enum SlamCtrlCmd{
    START_MAPPING           = 1000,  // 开始建图
    START_SEC_MAPPING       = 2000,  // 重定位->建图，二次建图
    MAPPING_POINT_BEGIN     = 3000,  // 设置起点
    MAPPING_ELE_DELETE      = 4000,  // 创建地图元素过程中，清除当前元素（当前元素还未完成创建）
    MAPPING_POINT_END       = 5000,  // 设置终点
    EXIT_MAPPING            = 6000,  // 退出建图
    START_LOCALIZATION      = 7000,  // 重定位->定位
    EXIT_LOCALIZATION       = 8000,  // 退出定位
    START_RELOCALIZATION    = 9000,  // 重定位，定位过程中，重新进行重定位
    CMD_MAX
};


class LocalizationModule{
public:
    LocalizationModule(){};
    // LocalizationModule(const std::string work_path, ModuleStatus init_status);
    LocalizationModule(ModuleStatus init_status);
    ~LocalizationModule();


private:
    // void show_thread();
    bool is_mapping_status(ModuleStatus status);

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

    bool start_relocalization(int map_id);


    bool init_module_by_set_status(ModuleStatus set_status);
    // void make_slam_obj(string work_path, bool localization_mode, bool offline_mode, bool sec_mapping);
    bool make_slam_obj(lidar_slam::LidarSlamParam yaml_param, ModuleStatus set_status);

    // void make_slam_obj(string work_path, bool slam_mode, bool offline_mode);
    void release_slam_obj();

    // callback 
    void localization_module_ctrl_cbk(const std_msgs::UInt32 &msg_in);
    void slam_dealt_timer(const ros::TimerEvent &event);
    void pose_filter_timer(const ros::TimerEvent &event);
    void pub_module_status_timer(const ros::TimerEvent &event);

    // void command_cbk(const std_msgs::Int32 &msg_in);
    // void livox_msg_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg_in);
    // void livox_msg_cbk(const fairland_msgs::LivoxCustomMsg::ConstPtr &msg_in);

    void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
    
    void lidar_ros_cbk(const sensor_msgs::PointCloud2::ConstPtr &ros_msg);
    void livox_ros_cbk(const sensor_msgs::PointCloud2::ConstPtr &ros_msg);
    void chassis_cbk(const fairland_msgs::chassic_data::ConstPtr &msg_in);

    void publish_unoptimized_path(const std::deque<Eigen::Isometry3d> path, std::string frame, ros::Publisher pubUnoptimizedPath);
    void publish_optimized_path(const std::vector<Eigen::Isometry3d> path, std::string frame, ros::Publisher pubOptimizedPath);
    
    // publish common
    void pub_odom_cloud(PointCloudXYZI::Ptr msg_in, ros::Publisher pubOdomCloud);
    void pub_lidar_cloud(PointCloudXYZI::Ptr msg_in, ros::Publisher pubBodyCloud);
    // void pub_obstacle_cloud(PointCloudXYZI::Ptr msg_in, ros::Publisher pubObstacleCloud);
    void pub_filtered_obstacle_cloud(PointCloudXYZI::Ptr msg_in, ros::Publisher pubFilteredObstacleCloud);
    void pub_test_cloud(PointCloudXYZI::Ptr msg_in, bool localization_mode,ros::Publisher pubTestCloud);
    void pub_kdtree_cloud(PointCloudXYZI::Ptr msg_in, ros::Publisher pubKdtreeCloud);
    void publish_odometry(const Eigen::Isometry3d lidar_in_odom, ros::Publisher pubOdomAftMapped);
    void publish_odometry_lidar_in_map(const Eigen::Isometry3d lidar_in_map, string frameid, string child_frameid, ros::Publisher publisher);
    void publish_static_transform(const Eigen::Isometry3d wheel_in_lidar);
    void publish_transform(const Eigen::Isometry3d& correction,string parent, string child);
    void publish_lidar_to_map(const Eigen::Isometry3d& lidar_in_map, ros::Publisher pubOdomCloud);
    void visualizeLoopClosure(map<int, int> loopIndexContainer, nav_msgs::Path optimized_path_msg, ros::Publisher pubLoopConstraintEdge);
    void show_keyframe(std::vector<lidar_slam::ScInfo> loadKeyframe, ros::Publisher pubKeyframePose);
    void pub_rgb_map(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud, ros::Publisher pubRgbCloud);

    // position filter
    void lidar_position_filter_fst_order(Eigen::Isometry3d last_pose, const Eigen::Isometry3d lidar_in_map, Eigen::Isometry3d & pose_filtered);
    void lidar_position_filter_window(Eigen::Isometry3d last_pose, Eigen::Isometry3d lidar_in_map, Eigen::Isometry3d & pose_filtered);
    void position_filter_thread();
    bool position_init(Eigen::Isometry3d init_pose);
    void position_filter();
    void detect_slipping();
    void reset_pose_filter();

    void position_filter_chassis_lidar(double & filtered_x, double & filtered_y, double & filtered_a);

    void fill_log(Eigen::Isometry3d last_lidar_in_odom, Eigen::Isometry3d curr_lidar_in_odom);

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
        CASE_STR(START_RELOCALIZATION);
        CASE_STR(CMD_MAX);
        default:
            break;
        }
        return "UNKNOW_SlamCtrlCmd!";
    }
    
    // template <class T>
    // void get_param(const std::string& param_str, T& param, bool* is_success){
    //     if(!nh_.getParamCached(param_str,param)){
    //         ROS_WARN("load param failed : %s ", param_str.c_str());
    //         *is_success = false;
    //     }else{
    //         ROS_INFO("load param success: %s", param_str.c_str());
    //     }
    // };

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    ros::NodeHandle nh_;
    ros::Timer timer_slam_;
    ros::Timer timer_module_status_;

    ros::NodeHandle nh2_;
    ros::CallbackQueue slam_queue_;
    ros::Timer timer_pose_filter_;
    
    ros::NodeHandle nh3_;
    ros::CallbackQueue slam_ctrl_queue_;

    ros::NodeHandle nh4_;
    ros::CallbackQueue pose_filter_queue_;

    ros::Subscriber sub_mapping_ctrl_;
    ros::Subscriber sub_pointcloud2_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_chassis_;

    ros::Publisher pub_localization_module_status_;
    ros::Publisher pub_filter_odometry_;
    ros::Publisher pub_log_;
    ros::Publisher pub_heartbeat_;


    // slam node
    std::unique_ptr<lidar_slam::LidarSlam> slam_;
    bool releasing_slam_flag_ = false;

    // 通用
    string curr_dir_; // localization_module CMake dir
    bool localization_mode_ = false;
    bool offline_mode_ = false; // unused
    bool just_show_mode_ = false;
    bool show_rviz_ = false;
    bool fast_mode_ = false; // unused, 只有参数读入
    string log_folder_;

    // show thread
    // lidar_slam::Control_status control_status_;

    // 模块 localization module
    // bool running_slam_ = false;
    ModuleStatus last_running_module_status_ = ModuleStatus::MODULE_IDLE;
    ModuleStatus set_module_status_ = ModuleStatus::MODULE_IDLE;
    // ModuleStatus running_module_status_ = ModuleStatus::MODULE_IDLE;
    static std::atomic<ModuleStatus> running_module_status_;

    // 建图 *******************************************
    // MappingStatus mapping_status_ = M_INACTIVE;
    int start_index_ = -1;
    int end_index_ = -1;

    // 定位 *******************************************
    // enum LocalizationStatus
    // localization_status_: 在localization_module.cpp(&.h)中只作初始化为 L_INACTIVE 的操作; 实际的全部状态来源:lidar_slam.cpp(&.h).
    // lidar_slam::LocalizationStatus localization_status_ = L_INACTIVE;


    // other thread
    std::thread show_thread_;
    int show_load_map_ = 0;


    cpu_set_t mask;

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

    ros::Publisher pub_base_imu_;


    /// params load from yaml
    lidar_slam::LidarSlamParam slam_param_;

    /// odometry filter ****************************
    // std::unique_ptr<std::thread> position_filter_thread_ = nullptr;
    // std::thread position_filter_thread_;
    // std::vector<Eigen::Vector3d> pose_vec_;
    std::vector<Eigen::Isometry3d> pose_vec_;
    // std::deque<Eigen::Vector3d> pose_vec_;
    int window_size = 5;

    fairland_msgs::chassic_data cur_chassis_msg_;
    
    bool position_initialized_ = false;
    ros::Time last_chassis_time_;
    double last_chassis_x_ = 0.0;
    double last_chassis_y_ = 0.0;
    double last_chassis_a_ = 0.0;
    double chassis_x_ = 0.0;
    double chassis_y_ = 0.0;
    double chassis_a_ = 0.0;
    double last_lidar_dx_ = 0.0f;
    double last_lidar_dy_ = 0.0f;
    double last_lidar_dz_ = 0.0f;
    double last_lidar_x_ = 0.0f;
    double last_lidar_y_ = 0.0f; 
    double last_lidar_z_ = 0.0f; 
    double last_lidar_a_ = 0.0f;
    double lidar_x_ = 0.0f;      // 雷达给出位置（一阶滤波）
    double lidar_y_ = 0.0f; 
    double lidar_z_ = 0.0f; 
    double lidar_a_ = 0.0f;       // 雷达给出角度 
    double lidar_time_;

    double k_pos_;
    double chassis_linear_velocity_;
    double chassis_angular_velocity_;

    int slip_count_ = 0;
    double filter_x_;
    double filter_y_;
    double filter_a_;
    int filter_cout_ = 0;
    static std::atomic<double> livox_cbk_update_time_;
    
    // nav_msgs::Odometry filter_odometry_;
    LocalizationModuleLogInfoManager * log_info_manager_;

    // lidar 
    std::shared_ptr<LidarPreprocParent> lidar_ptr_;
    
};






}// namespace localization_module

#endif // LOCALIZATION_MODULE_H

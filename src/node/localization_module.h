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
#include "fairland_msgs/LocalizationModuleHealth.h"
#include "fairland_msgs/LocalizationModuleLogInfo.h"
#include "fairland_msgs/NameValues.h"



#include "lidar_slam/common_lib.h"
#include "lidar_slam/lidar_slam.hpp"
// #include "lidar_slam/Viewer.hpp"
#include "lidar/livox/ros_livox_datatype_def.h"

#include "node/module_param_def.h"
#include "node/module_status_def.h"
#include "node/log_info_manager.hpp"
#include "node/param_manager.hpp"
#include "node/detect_slipping.h"
#include "node/pose_filter.h"

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
    EXIT_MAPPING            = 3000,  // 退出建图
    START_LOCALIZATION      = 7000,  // 重定位->定位
    EXIT_LOCALIZATION       = 8000,  // 退出定位
    START_RELOCALIZATION    = 9000,  // 重定位，定位过程中，重新进行重定位
    RESTART_SEC_MAPPING     = 9100,  // 重启二次建图（一般是二次建图重定位失败的情况）
    // MAPPING_POINT_BEGIN     = 3000,  // 设置起点
    // MAPPING_ELE_DELETE      = 4000,  // 创建地图元素过程中，清除当前元素（当前元素还未完成创建）
    // MAPPING_POINT_END       = 5000,  // 设置终点
    CMD_MAX = 9999
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

    bool module_member_init();
    bool load_lidar_slam_param();
    bool create_ROS_IO();
    void ros_spinner_start();

    // 建图
    // bool mark_start_point();
    // bool mark_end_point(int save_id);
    // bool clear_curr_element();

    bool make_map_directory_name(int map_id);
    
    bool start_mapping(int map_id);
    bool start_second_mapping(int map_id);
    bool stop_mapping();

    // bool start_localization(ModuleStatus set_status, int map_id);
    bool start_localization(int map_id);

    // void start_localization(bool module_mode, int map_id);
    bool stop_localization();

    bool start_relocalization(int map_id); // = restart_localization;;
    bool restart_second_mapping( int map_id);
    bool stop_mapping_without_saving_map();

    bool init_module_by_set_status(ModuleStatus set_status);
    // void make_slam_obj(string work_path, bool localization_mode, bool offline_mode, bool sec_mapping);
    bool make_slam_obj(lidar_slam::LidarSlamParam yaml_param, ModuleStatus set_status);

    // void make_slam_obj(string work_path, bool slam_mode, bool offline_mode);
    void release_slam_obj();

    // callback 
    void localization_module_ctrl_callback(const std_msgs::UInt32 &msg_in);
    void slam_dealt_timer(const ros::TimerEvent &event);
    void pose_filter_timer(const ros::TimerEvent &event);
    void pub_module_status_timer(const ros::TimerEvent &event);

    // void command_cbk(const std_msgs::Int32 &msg_in);
    // void livox_msg_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg_in);
    // void livox_msg_cbk(const fairland_msgs::LivoxCustomMsg::ConstPtr &msg_in);

    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg_in);
    
    void lidar_ros_callback(const sensor_msgs::PointCloud2::ConstPtr &ros_msg);
    void chassis_callback(const fairland_msgs::chassic_data::ConstPtr &msg_in);

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
    bool position_init(Eigen::Isometry3d init_pose);
    void position_filter();
    void detect_slipping();
    int detect_slipping(Eigen::Isometry3d curr_pose);
    void reset_pose_filter();

    void position_filter_chassis_lidar(double & filtered_x, double & filtered_y, double & filtered_a, double k_chassis);

    void fill_log(Eigen::Isometry3d last_lidar_in_odom, Eigen::Isometry3d curr_lidar_in_odom);

    void fill_slipping_msg(fairland_msgs::NameValues& slipping_msg);
    
    int  check_fill_health_msg(ModuleStatus curr_running_module_status, fairland_msgs::LocalizationModuleHealth &health_msg);
    void check_fill_module_status_msg(ModuleStatus curr_running_module_status, fairland_msgs::LocalizationModuleStatus &status_msg);
    void fill_module_l_status(ModuleStatus curr_running_module_status, fairland_msgs::LocalizationModuleStatus &status_msg);
    void fill_module_m_status(ModuleStatus curr_running_module_status, fairland_msgs::LocalizationModuleStatus &status_msg);

    string print_SlamCtrlCmd(SlamCtrlCmd e){
        switch (e){
        CASE_STR(START_MAPPING);
        CASE_STR(START_SEC_MAPPING);
        // CASE_STR(MAPPING_POINT_BEGIN);
        // CASE_STR(MAPPING_ELE_DELETE);
        // CASE_STR(MAPPING_POINT_END);
        CASE_STR(EXIT_MAPPING);
        CASE_STR(START_LOCALIZATION);
        CASE_STR(EXIT_LOCALIZATION);
        CASE_STR(START_RELOCALIZATION);
        CASE_STR(RESTART_SEC_MAPPING);
        CASE_STR(CMD_MAX);
        default:
            break;
        }
        return "UNKNOW_SlamCtrlCmd!";
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    /// module status    //
    /*************************************************** */
    /** @local_node_status_: 
     * 0: inactive
     * 1: normal
     * 2: lidar cbk delay
     * 3: localize thread delay
     */
    std::atomic<int> local_node_status_{0};
    /*************************************************** */
    /** @local_node_status_: 
     * 0: inactive
     * 1: normal
     * 2: lidar cbk delay
     * 3: secmap-relocal thread delay
     * 4: loop_closure_thread_delay 
     */
    std::atomic<int> mapping_node_status_{0};
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // 各 线程、callback、timer heartbeat
    std::atomic<double> hb_time_cbk_imu_;
    std::atomic<double> hb_time_cbk_lidar_;
    std::atomic<double> hb_time_cbk_module_ctrl_;
    std::atomic<double> hb_time_timer_slam_;
    std::atomic<double> hb_time_timer_pose_;
    // std::atomic<double> hb_time_thread_localize_;
    std::atomic<double> hb_time_thread_loop_closure_;
    std::atomic<double> hb_time_thread_secmap_relocalize_;
    // health_status_: -------------------------------------
    // 0: all ok
    // 1: error, stop pub tf & odom
    // 2: error, reset slam to IDLE
    std::atomic<int>  health_status_;
    
    std::atomic<int>  cloud_size_;
    // -----------------------------------------------------
    Eigen::Isometry3d T_lidar_baselink_;

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

    ros::NodeHandle nh5_;
    ros::CallbackQueue health_queue_;

    ros::Subscriber sub_mapping_ctrl_;
    ros::Subscriber sub_pointcloud2_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_chassis_;

    ros::Publisher pub_localization_module_status_;
    ros::Publisher pub_localization_module_health_;    
    ros::Publisher pub_filter_odometry_;
    ros::Publisher pub_log_;
    ros::Publisher pub_slip_;    
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
    // ModuleStatus last_running_module_status_ = ModuleStatus::MODULE_IDLE;
    // ModuleStatus set_module_status_ = ModuleStatus::MODULE_IDLE;
    // ModuleStatus running_module_status_ = ModuleStatus::MODULE_IDLE;
    static std::atomic<ModuleStatus> running_module_status_;

    // 建图 *******************************************
    // MappingStatus mapping_status_ = M_INACTIVE;
    std::atomic<int> mapping_status_{0};
    std::atomic<int> localization_status_{0};
    int start_index_ = -1;
    int end_index_ = -1;

    // 定位 *******************************************
    // enum LocalizationStatus
    // localization_status_: 在localization_module.cpp(&.h)中只作初始化为 L_INACTIVE 的操作; 实际的全部状态来源:lidar_slam.cpp(&.h).
    // lidar_slam::LocalizationStatus localization_status_ = L_INACTIVE;


    // other thread
    std::thread show_thread_;
    int show_load_map_ = 0;


    cpu_set_t cpu_mask_;

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
    ros::Publisher pub_key_cloud_;


    /// params load from yaml
    lidar_slam::LidarSlamParam slam_param_;

    /// odometry filter ****************************
    // std::unique_ptr<std::thread> position_filter_thread_ = nullptr;
    // std::thread position_filter_thread_;
    // std::vector<Eigen::Vector3d> pose_vec_;
    std::vector<Eigen::Isometry3d> pose_vec_;
    // std::deque<Eigen::Vector3d> pose_vec_;
    // int window_size = 5;

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
    int filter_count_ = 0;
    static std::atomic<double> livox_cbk_update_time_;
    
    // nav_msgs::Odometry filter_odometry_;
    LocalizationModuleLogInfoManager * log_info_manager_;

    // lidar 
    std::shared_ptr<LidarPreprocParent> lidar_ptr_;
    std::shared_ptr<DetectSlipping> slipping_ptr_;
    std::shared_ptr<PoseFilter> pose_filter_ptr_;
    
};






}// namespace localization_module

#endif // LOCALIZATION_MODULE_H

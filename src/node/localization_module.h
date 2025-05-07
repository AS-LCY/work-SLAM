#ifndef LOCALIZATION_MODULE_H
#define LOCALIZATION_MODULE_H

#include <string>
#include <vector>
#include <cstdlib>
#include <chrono>
#include <atomic>
#include <fstream>

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
#include "fairland_msgs/chassic_data.h"



#include "lidar_slam/common_lib.h"
#include "lidar_slam/lidar_slam.hpp"
// #include "lidar_slam/Viewer.hpp"
#include "lidar/livox/ros_livox_datatype_def.h"

#include "node/module_param_def.h"
#include "node/module_status_def.h"
#include "node/log_info_manager.hpp"
#include "node/param_manager.hpp"
// #include "node/pose_filter.h"


// lidar
#include "lidar/lidar_preproc_factory.hpp"
#include "lidar/lidar_preproc_parent.h"
#include "lidar/livox/pcl_point_type_def_lvx.h"
#include "lidar/livox/lidar_preproc_Mid360.h"
#include "lidar/robosense/pcl_point_type_def_rs.h"
#include "lidar/robosense/lidar_preproc_Airy.h"
#include "lidar/lanhai/pcl_point_type_def_bs.h"
#include "lidar/lanhai/lidar_preproc_M300.h"
#include "lidar/hesai/pcl_point_type_def_hs.h"
#include "lidar/hesai/lidar_preproc_JT16.h"

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
    CANCLE_MAPPING          = 5000,  // 不保存地图， 直接取消建图
    SAVE_AND_END_MAPPING    = 6000,  // 保存地图， 并结束建图
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
    bool need_start_localization(ModuleStatus running_module_status_now, int localiztion_status_now);    
    bool localization_status_is_ok(int localiztion_status_now);
    bool localization_status_is_failed(int localiztion_status_now);
    bool mapping_status_is_ok(int mapping_status_now);
    bool mapping_status_is_failed(int mapping_status_now);

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
    bool save_extrinsic_to_file();
    bool start_localization(int map_id);

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
    void publish_cloud(PointCloudType::Ptr pcl_cloud_in, std::string frame_id, ros::Time ros_time, ros::Publisher pub_cloud);
    void publish_odometry(const Eigen::Isometry3d isometry_3d, std::string frameid, std::string child_frameid, ros::Publisher pub);
    void publish_odometry_lidar_in_map(const Eigen::Isometry3d lidar_in_map, lidar_slam::Localization_base curr_pose, 
                                        string frameid, string child_frameid, ModuleStatus curr_running_module_status, 
                                        ros::Publisher pubOdomAftMapped);
    void process_loginfo();



    // void pub_odom_cloud(PointCloudType::Ptr msg_in, ros::Publisher pubOdomCloud);
    // void pub_lidar_cloud(PointCloudType::Ptr msg_in, ros::Publisher pubBodyCloud);
    // void pub_obstacle_cloud(PointCloudType::Ptr msg_in, ros::Publisher pubObstacleCloud);
    // void pub_filtered_obstacle_cloud(PointCloudType::Ptr msg_in, ros::Publisher pubFilteredObstacleCloud);
    // void pub_kdtree_cloud(PointCloudType::Ptr msg_in, ros::Publisher pubKdtreeCloud);
    // void pub_rgb_map(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud, ros::Publisher pubRgbCloud);
    void pub_test_cloud(PointCloudType::Ptr msg_in, bool localization_mode,ros::Publisher pubTestCloud);
    void publish_static_transform(const Eigen::Isometry3d wheel_in_lidar);
    void publish_transform(const Eigen::Isometry3d& correction,string parent, string child);
    void publish_lidar_to_map(const Eigen::Isometry3d& lidar_in_map, ros::Publisher pubOdomCloud);
    void visualizeLoopClosure(map<int, int> loopIndexContainer, nav_msgs::Path optimized_path_msg, ros::Publisher pubLoopConstraintEdge);
    void show_keyframe(std::vector<lidar_slam::ScInfo> loadKeyframe, ros::Publisher pubKeyframePose);


    void fill_log(Eigen::Isometry3d last_lidar_in_odom, Eigen::Isometry3d curr_lidar_in_odom);

    // void fill_slipping_msg(fairland_msgs::NameValues& slipping_msg);
    
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
        CASE_STR(CANCLE_MAPPING);
        CASE_STR(SAVE_AND_END_MAPPING);
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
    /** @mapping_node_status_: 
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
    // -------------------------------------
    /** @health_status_: 
     * 0: all ok
     * 1: error, stop pub tf & odom
     * 2: error, reset slam to IDLE
     */
    std::atomic<int>  health_status_;
    
    std::atomic<int>  cloud_size_orig_;
    std::atomic<int>  cloud_size_sample_;
    std::atomic<int>  cloud_size_feat_;
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
    // ros::Publisher pub_filter_odometry_;
    ros::Publisher pub_log_;
    ros::Publisher pub_slip_;    


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
    static std::atomic<ModuleStatus> running_module_status_;

    // 建图 *******************************************
    /** @mapping_status_: 
     * 0: m_inactive
     * 1: m_relocalize ing
     * 2: m_relocalize failed
     * 3: m_standby
     * 4: m_creating_ele (not used)
     * 5: m_failed
     */
    std::atomic<int> mapping_status_{0};

    /** @map_saved_: 
     * 0: map not saved yet
     * 1: map already saved (only set to 1 when stop mapping with saving map)
     */
    std::atomic<int> map_saved_{0};

    // int start_index_ = -1; // not used now, 目前不涉及创建元素的操作
    // int end_index_ = -1; // not used now, 目前不涉及创建元素的操作
    

    // 定位 *******************************************
    /** @localization_status_: 
     * 0: l_inactive
     * 1: l_relocalize ing
     * 2: l_relocalize failed
     * 3: l_normal
     * 4: l_low_accuracy 
     * 5: l_failed
     */
    std::atomic<int> localization_status_{0};

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
    ros::Publisher pub_body_cloud_filter_;


    /// params load from yaml
    lidar_slam::LidarSlamParam slam_param_;
    
    static std::atomic<double> livox_cbk_update_time_;
    
    LocalizationModuleLogInfoManager * log_info_manager_;

    // lidar ptr
    std::shared_ptr<LidarPreprocParent> lidar_ptr_;
    
};






}// namespace localization_module

#endif // LOCALIZATION_MODULE_H

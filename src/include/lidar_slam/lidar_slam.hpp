#ifndef LIDAR_SLAM_H
#define LIDAR_SLAM_H
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Eigen/Core>
#include <filesystem>

#include <pcl/filters/voxel_grid.h>
#include <sophus/se3.hpp>
#include <yaml-cpp/yaml.h>

#include <fast_gicp/gicp/fast_gicp.hpp>

#include "lidar_slam/common_lib.h"
#include "lidar_slam/global_localization.hpp"
#include "lidar_slam/localization.hpp"
#include "lidar_slam/backend.hpp"
#include "lidar_slam/IMU_Processing.hpp"
#include "lidar_slam/preprocess.h"
#include "lidar_slam/Viewer.hpp"
// #include "include/livox_ros_driver2.h"
// #include "driver_node.h"
// #include "lddc.h"
#include "livox_datatype/livox_ros_datatype_def.h"
#include "node/lidar_slam_param_def.h"
// #include "lds_lidar.h"

namespace lidar_slam {
struct LidarParam{
 V3D extrinT; 
 M3D extrinR;
 Eigen::Isometry3d  T_wheel_lidar = Eigen::Isometry3d::Identity();
 bool localization_mode;
 bool offline_mode;
 std::string load_map_path;
 std::string save_log_path;
 double log_keep_time;
 double cloud_leaf_size;
 double map_leaf_size;
 double cube_len;
 double det_range;
 double blind_distance;
 double obstacle_max_range;
 double obstacle_min_height;
 double obstacle_max_height;
 double obstacle_filter_size;
 int point_filter_num;
 double key_frame_distance;
 double key_frame_angle;
 double loopSearchDistance;
 double kdTreeReconstructRadius;
 double kdTreeReconstructKeyFrameLeafSize;
 double kdTreeReconstructPointLeafSize;
};

struct Localization_base{
    state_ikfom  imu_state;
    double base_time = 0;
    double update_time = 0;
    Localization_base(){
        imu_state = state_ikfom();
        double base_time = 0;
        double update_time = 0;      
    }
};

enum SlamWorkMode{
    MAPPING = 1,
    SEC_MAPPING = 2,
    LOCALIZATION = 3,
    UNKNOWN
};

class LidarSlam
{
    public:
        LidarSlam(const std::string work_path,bool localization_mode,bool offline, bool sec_mapping);
        LidarSlam(const LidarSlamParam yaml_param, SlamWorkMode init_mode);// new added
        LidarSlam() = delete;
        LidarSlam(const LidarSlam&) = delete; 
        void reset(SlamWorkMode work_mode);// new added
        void reset(const std::string work_path,bool localization_mode,bool offline, bool sec_mapping);
        // void start_driver(const std::string work_path);// disable start_driver of lidar
        ~LidarSlam(){ 
            // cout<<"destruct"<<endl;
            // cout <<"work mode: "<<print_SlamWorkMode(working_mode_)<<endl;
            thread_run = false;
            thread->join();
            thread.reset(nullptr);
            show_thread->join();
            show_thread.reset(nullptr);
            // if(sec_mapping_){
            if(working_mode_ == SEC_MAPPING){
                second_mapping_thread->join();
                second_mapping_thread.reset(nullptr);
            }

            //  LivoxLidarSdkUninit();// disable start_driver of lidar

            // cout<<"destruct end"<<endl;
         };
        bool run();
        void livox_pcl_cbk(const std::shared_ptr<livox_ros::LidarMsg> &msg_in);
        void livox_pcl_offline_cbk(const PointCloudXYZI::Ptr msg_in,double time_stamp);
       // void cmd_cbk(WorkState& msg);
        void imu_cbk(const std::shared_ptr<livox_ros::ImuMsg> &msg_in);
        void image_cbk(const cv::Mat& img,double time);
        void filter_obstacle_cloud(const PointCloudXYZI::Ptr cloud);
        bool save_map(string saveMapDirectory,double resolution, int start_index, int end_index){ 
            // if (param.localization_mode){
            if (working_mode_ == LOCALIZATION){
                return true;
            }else if(working_mode_ == MAPPING || working_mode_ == SEC_MAPPING){
                return back_end->saveMap(saveMapDirectory,resolution,getOdomToMap(), start_index, end_index);
            }else{
                return true;
            }
        };
       
        bool load_map(string directory){
            globalLocalizationSuccess = false;
            sleep(1);
            localization->loadMap(directory);
            return true;
        }

        int get_curr_pose_index(){
            return back_end->getCurrentPoseIndex();
        }
        
        PointCloudXYZI::Ptr get_lidar_cloud()
        {
          std::lock_guard<std::mutex> lk(mtx_lidar_cloud);
          return undistortCloud;
        }

        PointCloudXYZI::Ptr get_odom_cloud()
        {
           std::lock_guard<std::mutex> lk(mtx_odom_cloud);
           return UndistortCloudInOdom;
        }

        PointCloudXYZI::Ptr get_kdtree_cloud()
        {
           return kdtreeCloud;
        }
        std::deque<Eigen::Isometry3d> get_unoptimized_path()
        {
           std::lock_guard<std::mutex> lk(mtx_path);
           return unoptimized_path;
        }
        std::vector<Eigen::Isometry3d> get_optimized_path()
        {
           std::lock_guard<std::mutex> lk(mtx_path);
           return optimized_path;
        }
        map<int, int> getloopIndex()
        {
           return back_end->getloopIndex();
        }
        Eigen::Isometry3d getOdomToMap(){
            if (working_mode_ == LOCALIZATION){
               return localization->getOdomToMap();
            }else if(working_mode_ == MAPPING){
                Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
                transform.matrix().block<3, 3>(0, 0) = p_imu->initial_rotate;
                return transform;
            }else if(working_mode_ == SEC_MAPPING){
                return localization->getOdomToMap();
            }else{
                cout << "working_mode: "<<print_SlamWorkMode(working_mode_)<<", error mode"<<endl;
                return Eigen::Isometry3d::Identity();
            }
            // if (param.localization_mode)
            //    return localization->getOdomToMap();
            // else{
            //     if (sec_mapping_){
            //         return localization->getOdomToMap();
            //     }
            //     Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
            //     transform.matrix().block<3, 3>(0, 0) = p_imu->initial_rotate;
            //     return transform;
            // }
        }
        Eigen::Isometry3d getLidarInOdom(){
            std::lock_guard<std::mutex> lk(mtx_pose);
            // if(!param.localization_mode){
            if(working_mode_ == MAPPING || working_mode_==SEC_MAPPING){
                return T_odom_lidar;
            } 
            else if(working_mode_==LOCALIZATION){
                Eigen::Isometry3d T_odom_b(Sophus::SE3d(current_pose.imu_state.rot, current_pose.imu_state.pos).matrix());
                Eigen::Isometry3d T_b_lidar(Sophus::SE3d(current_pose.imu_state.offset_R_L_I, current_pose.imu_state.offset_T_L_I).matrix());
                Eigen::Isometry3d temp  =   T_odom_b * T_b_lidar;
                return  temp;
            }else{
                Eigen::Isometry3d temp = Eigen::Isometry3d::Identity();
                return temp;
            }
        } 
        Eigen::Isometry3d getWheelInOdom(){
            return getLidarInOdom()* T_lidar_wheel;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr getLoadMap(){
            return localization->getLoadMap();
        }
        std::vector<Eigen::Vector3f>& getLoadMapPoints(){
            return localization->getLoadMapPoints();
        }
        bool isGloalLocalizationSuccess(){
            return globalLocalizationSuccess;
        }
        Eigen::Isometry3d getLidarInMap(){  //插值

            // Eigen::Isometry3d T_odom_b(Sophus::SE3d(current_pose.imu_state.rot, current_pose.imu_state.pos).matrix());
            // Eigen::Isometry3d T_b_lidar(Sophus::SE3d(current_pose.imu_state.offset_R_L_I, current_pose.imu_state.offset_T_L_I).matrix());
            // Eigen::Isometry3d T_map_lidar  =  getOdomToMap() * T_odom_b * T_b_lidar;
            Eigen::Isometry3d T_map_lidar  =  getOdomToMap() * getLidarInOdom();
            return  T_map_lidar;
        }
        Eigen::Isometry3d getWheelInMap(){  //插值

            return  getLidarInMap() * T_lidar_wheel;
        }
        Eigen::Isometry3d getWheelInLidar(){ 

            return T_lidar_wheel;
        }
        std::vector<ScInfo> getLoadKeyFrame(){
            return localization -> getLoadKeyFrame();
        }
        PointCloudXYZI::Ptr getTestCloud(){
            PointCloudXYZI::Ptr temp(new PointCloudXYZI());
            // if (param.localization_mode)
            if (working_mode_==LOCALIZATION)
                return localization -> getTestCloud();
            else if(working_mode_==MAPPING || working_mode_ ==SEC_MAPPING)
                return back_end-> getTestCloud();
            else
                return temp;
        }
        PointCloudXYZI::Ptr getCurrentMap()
        {
            return back_end->getCurrentMap(getOdomToMap());
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCurrentRGBMap()
        {
            return back_end->getCurrentRGBMap();
        }
        PointCloudXYZI::Ptr getObstacleCloud()
        {
            return ObstacleCloud;
        }
        PointCloudXYZI::Ptr getFilteredObstacleCloud()
        {
            std::lock_guard<std::mutex> lk(mtx_obstacle_cloud);
            return FilteredObstacleCloud;
        }

        string print_SlamWorkMode(SlamWorkMode e){
            switch (e){
            CASE_STR(MAPPING);
            CASE_STR(SEC_MAPPING);
            CASE_STR(LOCALIZATION);
            default:
                break;
            }
            return "UNKNOW_SlamWorkMode!";
        }

    private:
        // LidarParam param;
        LidarSlamParam config_param_;
        deque<double> time_buffer;               // 记录lidar时间
        deque<PointCloudXYZI::Ptr> lidar_buffer; //记录特征提取或间隔采样后的lidar（特征）数据
        deque<std::shared_ptr<livox_ros::ImuMsg>> imu_buffer;
        bool lidar_pushed = false;
        // atomic<double> lidar_end_time = 0;
        atomic<double> lidar_end_time;
        double lidar_mean_scantime = 0.0;
        double first_lidar_time = 0.0;
        int scan_num = 0;
        bool flg_first_scan = true;
        double last_timestamp_lidar = 0;
        double last_timestamp_imu = -1.0;
        double timediff_lidar_wrt_imu = 0.0;
        bool time_sync_en = false;
        bool timediff_set_flg = false; // 标记是否已经进行了时间补偿
        bool reseting = false;
        bool imu_file_shift = false;
        std::deque<Eigen::Isometry3d> unoptimized_path;
        std::vector<Eigen::Isometry3d> optimized_path;
        std::vector<livox_ros::ImuMsg> temp_imu_msg;
        std::deque<double> pcd_file;
        std::ofstream imu_file;
        std::ofstream localization_file;
        MeasureGroup Measures;
        Eigen::Isometry3d  T_odom_lidar;
        Eigen::Isometry3d  T_lidar_wheel;

        // livox_ros::DriverNode livox_node;
        std::deque<std::pair<double,Eigen::Isometry3d>> poses_buffer;
        
        bool thread_run = true;
        bool globalLocalizationSuccess = false;
        bool localization_wait = false;
        bool loop_closure_wait = false;
        Localization_base localization_base;
        Localization_base current_pose;

        std::unique_ptr<KD_TREE<pcl::PointXYZINormal>> ikdtree= nullptr;
        std::unique_ptr<std::thread> thread = nullptr;
        std::unique_ptr<std::thread> show_thread = nullptr; 
        std::unique_ptr<std::thread> second_mapping_thread = nullptr; 
        mutex mtx_buffer;
        mutex mtx_odom_cloud;
        mutex mtx_lidar_cloud;
        mutex mtx_obstacle_cloud;
        mutex mtx_localization_base;
        mutex mtx_current_pose;
        mutex mtx_path;
        mutex mtx_pose;
        esekfom::esekf kf;
        std::unique_ptr<Preprocess> p_lidar_pre= nullptr;
        std::unique_ptr<ImuProcess> p_imu= nullptr;
        std::unique_ptr<BackEnd> back_end= nullptr;
        std::unique_ptr<Localization> localization= nullptr;
        std::unique_ptr<GlobalLocalization> global_localization= nullptr;

        PointCloudXYZI::Ptr UndistortCloudInOdom;
        PointCloudXYZI::Ptr undistortCloud;  // lidar 系
        PointCloudXYZI::Ptr FilteredUndistortCloud;
        pcl::VoxelGrid<PointType> downSizeFilterCloud;
        PointCloudXYZI::Ptr kdtreeCloud;
        PointCloudXYZI::Ptr ObstacleCloud;
        PointCloudXYZI::Ptr FilteredObstacleCloud;
        
        SlamWorkMode working_mode_ = UNKNOWN;
        bool second_mapping_need_global_localization_ = false;


        bool sync_packages(MeasureGroup &meas);
        void loopClosureThread();
        void localizationThread();
        void second_mapping_thread_func();
        void relocalizationForMappingThread();
        void showThread();
        void delete_log_file(double keep_time);


};

}
#endif


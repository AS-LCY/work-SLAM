#ifndef FLBOT_LIDAR_SLAM_GLOBAL_LOCALIZATION_HPP
#define FLBOT_LIDAR_SLAM_GLOBAL_LOCALIZATION_HPP

// std
#include <string>
#include <vector>
#include <sstream>
// #include <filesystem> // c++17

// pcl
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

// lidar_slam
#include "lidar_slam/common_lib.h"
#include "lidar_slam/data_struct_define.h"
#include "lidar_slam/scan_context/Scancontext.h" 


namespace lidar_slam{

class GlobalLocalization{
public:
    GlobalLocalization();
    ~GlobalLocalization();

    // ////////////////////////////////// - load map - ////////////////////////////////////
    // /// @brief  从本地加载地图
    // /// @return 总的点云地图，关键帧点云，关键帧pose
    // bool load_map_data(std::string map_dir);

    // PointCloudXYZI::Ptr get_loaded_global_map(){
    //     return loaded_global_map_;
    // }
    // std::vector<ScInfo> get_load_sc_info_(){
    //     return loaded_sc_info_;
    // }
    // std::vector<PointCloudXYZI::Ptr> get_loaded_keyframe_clouds(){
    //     return loaded_keyframe_clouds_;
    // }
    // std::vector<KeyPose> get_loaded_keyframe_poses(){
    //     return loaded_keyframe_poses_;
    // }
    // bool get_map_status(){
    //     return map_ready_;
    // }

    //////////////////////////////////// - global-localize - ////////////////////////////////////

    bool global_localize(PointCloudXYZI::Ptr cloudIn, Eigen::Isometry3d pose, Matrix3d initial_rotate, double score_thr);

    bool fill_sc_manager(std::vector<ScInfo> input_sc_info);
    bool get_sc_manager_ready(){
        return sc_manager_ready_;
    }
    bool set_global_map(PointCloudXYZI::Ptr loaded_global_map);
    bool get_global_map_ready(){
        return global_map_ready_;
    }

    Eigen::Isometry3d get_global_odom_to_map(){
        return global_odom_to_map_;
    }



private:

    // //////////////////////////////////// - load map - ////////////////////////////////////
    // /// @brief 从本地加载总的点云地图
    // bool load_cloud_map(std::string map_dir);
    // /// @brief 从本地加载关键帧信息（关键帧点云、关键帧pose）
    // bool load_key_frames(std::string keyframe_dir);


    //////////////////////////////////// - global-localize - /////////////////////////////    
    
    /// @brief  
    /// @param cloud_in         : param in, 用于生辰当前帧点云的 sc 
    /// @param initial_rotate   : param in, 对点云 cloud_in 作重力校准
    /// @param best_match       : result out, refer to sc
    /// @param best_trans       : result out, refer to translation
    /// @return 
    bool scancontex_search(PointCloudXYZI::Ptr cloud_in, Matrix3d initial_rotate, std::pair<int, float> &best_match, std::pair<double, double> &best_trans);
    
    /// @brief 
    /// @param initial_rotate   : param in 
    /// @param best_match       : param in 
    /// @param best_trans       : param in 
    /// @return Eigen::Matrix4d : init_transform (used in icp)
    Eigen::Matrix4d cal_init_transform(Matrix3d initial_rotate, std::pair<int, float> best_match, std::pair<double, double> best_trans);


    /// @brief  
    /// @param cloud_in     : param in 
    /// @param pose         : param in 
    /// @param init_guess   : param in 
    /// @param score_thr    : param in 
    /// @param res_global_odom_to_map    : param out  Eigen::Isometry3d result
    bool registration_icp(PointCloudXYZI::Ptr cloud_in, Eigen::Isometry3d pose, Eigen::Matrix4d init_guess, double score_thr, Eigen::Isometry3d& res_global_odom_to_map);


public:

private:
    //////////////////////////////////// - load map - ////////////////////////////////////
    std::shared_ptr<SCManager> sc_manager_;

    bool map_ready_ = false;
    PointCloudXYZI::Ptr loaded_global_map_;
    std::vector<ScInfo> loaded_sc_info_;// 对应本地文件：data

    std::vector<KeyPose> loaded_keyframe_poses_;
    pcl::PointCloud<PointType>::Ptr loaded_key_point_;
    std::vector<PointCloudXYZI::Ptr> loaded_keyframe_clouds_;

    // KeyMat polarcontext_invkeys_mat_;
    // std::vector<Eigen::MatrixXd> polarcontexts_;

    std::vector<Eigen::Isometry3d> accumulate_key_pose_;
    PointCloudXYZI::Ptr accumulate_map_;

    //////////////////////////////////// - global-localize - ////////////////////////////////////
    bool global_map_ready_ = false;
    bool sc_manager_ready_ =false;
    PointCloudXYZI::Ptr test_match_cloud_;//debug

    // result of global localization
    Eigen::Isometry3d global_odom_to_map_ = Eigen::Isometry3d::Identity();


};


} // namespace lidar_slam

#endif //define FLBOT_LIDAR_SLAM_GLOBAL_LOCALIZATION_H
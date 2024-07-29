#ifndef FLBOT_SLAM_CLOUD_MAP_H
#define FLBOT_SLAM_CLOUD_MAP_H

#include <string>
#include <vector>
#include <sstream>
#include <filesystem>

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include "common_lib.h"
#include "data_struct_define.h"
#include "common_lib.h"
#include "scan_context/Scancontext.h" // 只用了数据结构 ScInfo


namespace lidar_slam{

class CloudMap{
public:
    CloudMap();
    ~CloudMap();

    /// @brief  从本地加载地图，用于二次建图
    /// @return 总的点云地图，关键帧点云，关键帧pose
    bool load_map_data(std::string map_dir);


    PointCloudXYZI::Ptr get_loaded_cloud_map(){
        return loaded_cloud_map_;
    }
    std::vector<ScInfo> get_load_sc_info_(){
        return load_sc_info_;
    }
    std::vector<PointCloudXYZI::Ptr> get_loaded_keyframe_clouds(){
        return loaded_keyframe_clouds_;
    }
    std::vector<KeyPose> get_loaded_keyframe_poses(){
        return loaded_keyframe_poses_;
    }
    bool get_map_status(){
        return map_ready_;
    }




private:
    /// @brief 从本地加载总的点云地图
    bool load_cloud_map(std::string map_dir);

    /// @brief 从本地加载关键帧信息（关键帧点云、关键帧pose）
    bool load_key_frames(std::string keyframe_dir);

    //// variable
    bool map_ready_ = false;
    PointCloudXYZI::Ptr loaded_cloud_map_;
    std::vector<KeyPose> loaded_keyframe_poses_;
    std::vector<PointCloudXYZI::Ptr> loaded_keyframe_clouds_;

    /// 加载data数据所用
    std::shared_ptr<SCManager> sc_manager_;
    std::vector<ScInfo> load_sc_info_;

    KeyMat polarcontext_invkeys_mat_;
    std::vector<Eigen::MatrixXd> polarcontexts_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr key_point_;
    std::vector<Eigen::Isometry3d> accumulate_key_pose_;
    PointCloudXYZI::Ptr accumulate_map_;
};

} // namespace lidar_slam

#endif // end define: FLBOT_SLAM_CLOUD_MAP_H
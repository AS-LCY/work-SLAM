
#ifndef FLBOT_LIDAR_PREPROC_AIRY_H
#define FLBOT_LIDAR_PREPROC_AIRY_H

// #include <string>
// #include <ros/ros.h>

#include "lidar/lidar_preproc_parent.h"
// #include "node/param_manager.hpp"

#include <pcl/filters/voxel_grid.h>


namespace localization_module{
class LidarPreprocAiry: public LidarPreprocParent {

public:
    LidarPreprocAiry();
    ~LidarPreprocAiry();
    
    bool pre_process(const sensor_msgs::PointCloud2::ConstPtr ros_msg_in, PointCloudType::Ptr& pcl_xyzin_out) override;
    
    bool msg2pcl_clip(const sensor_msgs::PointCloud2::ConstPtr ros_msg_in, PointCloudType::Ptr pcl_xyzin_out) override;

    bool msg2pcl_feat_pre(const sensor_msgs::PointCloud2::ConstPtr ros_msg_in);
    bool extract_by_ring_feature(PointCloudType::Ptr pcl_xyzin_out);
    bool extract_by_ring_feature();

private:
    bool set_param();
    void allocate_memory_init_variable();
    void extract_cloud_by_interval_sampling(const pcl::PointCloud<RsPointXYZIRT>::Ptr pcl_rs_in, PointCloudType::Ptr pcl_xyzin_out);

    void cal_smoothness();
    void mark_occluded_points(); ///< 标记遮挡和平行点
    void extract_feature_points(); ///< 提取surface和corner特征

private:
    // loaded param from yaml

    double thr_region_x_ = 0.0;
    double thr_region_y_ = 0.0;
    double thr_region_z_ = 0.0;

    // int point_filter_num_ = 2;
    // int ring_filter_num_ = 1;
    // loaded param end /////

    // variable
    pcl::VoxelGrid<PointType> downsize_filter_;
    float surf_leafsize_ = 0.4;

    std::vector<int> ring_index_start_;
    std::vector<int> ring_index_end_;
    std::vector<int> pnt_col_idx_;
    std::vector<float> pnt_range_;

    // PointCloudType::Ptr cloud_dense_;
    std::vector<smoothness_t> cloud_smoothness_;      // 存储每个点的曲率与索引
    float *cloud_curvature_;
    int *cloud_neighbor_picked_;
    int *cloud_label_; // 标记面点的索引的值为-1 ， 角点的索引的值为1， 初始化为0 

    PointCloudType::Ptr cloud_corner_;    // 保存有效点
    PointCloudType::Ptr cloud_surface_;   // 保存面点

    // lidar_slam::LidarPreprocParam param_;




};

} // namespace localization_module
#endif
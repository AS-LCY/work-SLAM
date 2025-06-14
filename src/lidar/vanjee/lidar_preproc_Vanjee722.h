
#ifndef FLBOT_LIDAR_PREPROC_VANJEE_H
#define FLBOT_LIDAR_PREPROC_VANJEE_H

// #include <string>
// #include <ros/ros.h>

#include <pcl/filters/voxel_grid.h>

#include "lidar/lidar_preproc_parent.h"
// #include "node/param_manager.hpp"




namespace localization_module{
class LidarPreprocVanjee722: public LidarPreprocParent {

public:
    LidarPreprocVanjee722();
    ~LidarPreprocVanjee722();

    bool pre_process(const sensor_msgs::PointCloud2::ConstPtr ros_msg_in, PointCloudType::Ptr& pcl_xyzin_out) override;

    // bool pre_process(const pcl::PointCloud<RsPointXYZIRT>::Ptr pcl_rs_in, PointCloudType::Ptr pcl_cld_out)  override ;
    bool msg2pcl_clip(const sensor_msgs::PointCloud2::ConstPtr ros_msg_in, PointCloudType::Ptr pcl_xyzin_out) override;


private:
    bool set_param();
    void allocate_memory_init_variable();
    // void extract_cloud_by_interval_sampling(const pcl::PointCloud<RsPointXYZIRT>::Ptr pcl_rs_in, PointCloudType::Ptr pcl_xyzin_out);


private:
    // loaded param from yaml
    double thr_region_x_ = 0.0;
    double thr_region_y_ = 0.0;
    double thr_region_z_ = 0.0;

    // int point_filter_num_ = 2;
    // int ring_filter_num_ = 1;
    // loaded param end /////


    // lidar_slam::LidarPreprocParam param_;



    // variable
    ////////////////////////////////////////////////////////////////////////////////
    // extract cloud by ring_feature
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

    pcl::PointCloud<PointType>::Ptr cloud_corner_;    // 保存有效点
    pcl::PointCloud<PointType>::Ptr cloud_surface_;   // 保存面点
    ////////////////////////////////////////////////////////////////////////////////

};

} // namespace localization_module
#endif
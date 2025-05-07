#ifndef FLBOT_LIDAR_PREPROC_MID360_H
#define FLBOT_LIDAR_PREPROC_MID360_H


#include "lidar/lidar_preproc_parent.h"

#include "lidar_slam/slam_param_def.h"



namespace localization_module{
class LidarPreprocMid360: public LidarPreprocParent{

public:
    LidarPreprocMid360();
    ~LidarPreprocMid360();

    bool msg2pcl_clip(const sensor_msgs::PointCloud2::ConstPtr ros_msg_in, PointCloudType::Ptr pcl_xyzin_out) override ;
    // bool msg2pcl_clip(const sensor_msgs::PointCloud2::ConstPtr &ros_msg_in, std::shared_ptr<livox_ros::LidarMsg> &lvx_msg_out) override ;
    // bool pre_process(const std::shared_ptr<livox_ros::LidarMsg> msg, PointCloudType::Ptr pcl_cld_out) override ;


private:
    bool set_param();

    void extract_cloud_by_feature(const std::shared_ptr<livox_ros::LidarMsg> msg, PointCloudType::Ptr pcl_cld_out );
    void extract_cloud_by_simple_voxel(const std::shared_ptr<livox_ros::LidarMsg> msg, PointCloudType::Ptr pcl_cld_out );
    void extract_cloud_by_interval_sampling(const std::shared_ptr<livox_ros::LidarMsg> msg, PointCloudType::Ptr pcl_cld_out );
    void extract_cloud_by_interval_and_voxel(const std::shared_ptr<livox_ros::LidarMsg> msg, PointCloudType::Ptr pcl_cld_out );


private:
    // loaded param from yaml
    double thr_region_x_ = 0.0;
    double thr_region_y_ = 0.0;
    double thr_region_z_ = 0.0;

    // double obstacle_square_ = 0.0;
    int point_filter_num_ = 2;
    // loaded param end /////


    lidar_slam::LidarPreprocParam param_;




};

} // namespace localization_module

#endif
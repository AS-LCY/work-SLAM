
#ifndef FLBOT_LIDAR_PREPROC_M300_H
#define FLBOT_LIDAR_PREPROC_M300_H

// #include <string>
// #include <ros/ros.h>

#include "lidar/lidar_preproc_parent.h"
// #include "node/param_manager.hpp"




namespace localization_module{
class LidarPreprocM300: public LidarPreprocParent {

public:
    LidarPreprocM300();
    ~LidarPreprocM300();
    
    bool pre_process(const sensor_msgs::PointCloud2::ConstPtr ros_msg_in, PointCloudType::Ptr& pcl_xyzin_out) override;
    
    // bool pre_process(const pcl::PointCloud<BsPointXYZI>::Ptr pcl_rs_in, PointCloudType::Ptr pcl_cld_out)  override ;
    bool msg2pcl_clip(const sensor_msgs::PointCloud2::ConstPtr ros_msg_in, PointCloudType::Ptr pcl_xyzin_out) override;


private:
    bool set_param();
    // void extract_cloud_by_interval_sampling(const pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_rs_in, PointCloudType::Ptr pcl_xyzin_out);


private:
    // loaded param from yaml
    double thr_region_x_ = 0.0;
    double thr_region_y_ = 0.0;
    double thr_region_z_ = 0.0;

    // int point_filter_num_ = 2;
    // int ring_filter_num_ = 1;
    // loaded param end /////


    // lidar_slam::LidarPreprocParam param_;




};

} // namespace localization_module
#endif

#ifndef FLBOT_LIDAR_PREPROC_AIRY_H
#define FLBOT_LIDAR_PREPROC_AIRY_H

// #include <string>
// #include <ros/ros.h>

#include "lidar/lidar_preproc_parent.h"
// #include "node/param_manager.hpp"




namespace localization_module{
class LidarPreprocAiry: public LidarPreprocParent {

public:
    LidarPreprocAiry();
    ~LidarPreprocAiry();

    bool pre_process(const pcl::PointCloud<RsPointXYZIRT>::Ptr pcl_rs_in, PointCloudXYZI::Ptr pcl_cld_out)  override ;
    bool msg2pcl_clip(const sensor_msgs::PointCloud2::ConstPtr ros_msg_in, pcl::PointCloud<RsPointXYZIRT>::Ptr pcl_cld_out) override ;


private:
    bool set_param();





};

} // namespace localization_module
#endif
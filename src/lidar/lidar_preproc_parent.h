
#ifndef FLBOT_LIDAR_PREPROC_PARENT_H
#define FLBOT_LIDAR_PREPROC_PARENT_H

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "node/param_manager.hpp"

#include "lidar/livox/pcl_point_type_def_lvx.h"
#include "lidar/livox/ros_livox_datatype_def.h"
#include "lidar/robosense/pcl_point_type_def_rbs.h"

namespace localization_module{
class LidarPreprocParent{

public:
    LidarPreprocParent();
    virtual ~LidarPreprocParent();

    // for livox
    virtual bool msg2pcl_clip(const sensor_msgs::PointCloud2::ConstPtr &ros_msg_in, std::shared_ptr<livox_ros::LidarMsg> &lvx_msg_out){return true;}
    virtual bool pre_process(const std::shared_ptr<livox_ros::LidarMsg> msg, PointCloudXYZI::Ptr pcl_cld_out){return true;}
    
    // for robosense
    virtual bool pre_process(const pcl::PointCloud<RsPointXYZIRT>::Ptr pcl_cld_in, PointCloudXYZI::Ptr pcl_cld_out){return true;}
    virtual bool msg2pcl_clip(const sensor_msgs::PointCloud2::ConstPtr ros_msg_in, pcl::PointCloud<RsPointXYZIRT>::Ptr pcl_cld_out){return true;}
    


protected:
    // virtual bool set_param()=0;
    // virtual void msg2pcl_clip()=0;

private:
    




};

} // namespace localization_module
#endif 
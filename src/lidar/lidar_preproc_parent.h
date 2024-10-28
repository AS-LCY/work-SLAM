
#ifndef FLBOT_LIDAR_PREPROC_PARENT_H
#define FLBOT_LIDAR_PREPROC_PARENT_H

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "lidar_slam/common_lib.h"
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
    virtual bool msg2pcl_clip(const sensor_msgs::PointCloud2::ConstPtr ros_msg_in, PointCloudXYZI::Ptr pcl_xyzin_out){return true;}
    


protected:
    // virtual bool set_param()=0;
    // virtual void msg2pcl_clip()=0;

private:
    




};

} // namespace localization_module


namespace lidar_common {
    
template<typename T>
bool is_nan_pt(T pt) {
    if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
        return true;
    } else {
        return false;
    }
}
} // namespace lidar_common

#endif 
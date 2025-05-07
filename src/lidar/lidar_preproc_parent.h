
#ifndef FLBOT_LIDAR_PREPROC_PARENT_H
#define FLBOT_LIDAR_PREPROC_PARENT_H

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "lidar_slam/common_lib.h"
#include "node/param_manager.hpp"

#include "lidar/livox/pcl_point_type_def_lvx.h"
#include "lidar/livox/ros_livox_datatype_def.h"
#include "lidar/robosense/pcl_point_type_def_rs.h"
#include "lidar/vanjee/pcl_point_type_def_vj.h"
#include "lidar/lanhai/pcl_point_type_def_bs.h"
#include "lidar/hesai/pcl_point_type_def_hs.h"


namespace localization_module{
class LidarPreprocParent{

public:
    LidarPreprocParent();
    virtual ~LidarPreprocParent();


    // for robosense & vanjee
    virtual bool msg2pcl_clip(const sensor_msgs::PointCloud2::ConstPtr ros_msg_in, PointCloudType::Ptr pcl_xyzin_out){return true;}
    
    // common
    void sampling_cloud(PointCloudType::Ptr in_cloud_ptr, PointCloudType::Ptr out_cloud_ptr);


protected:
    // virtual bool set_param()=0;
    // virtual void msg2pcl_clip()=0;

private:

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// member variable

protected:
    int cloud_size_to_keep_ = 2000;
    double blind_range_square_ = 0.0;
    double max_range_square_ = 0.0;

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
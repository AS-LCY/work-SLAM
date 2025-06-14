
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

struct smoothness_t{ 
    float value;
    size_t idx;
};

class LidarPreprocParent{

public:
    LidarPreprocParent();
    virtual ~LidarPreprocParent();

    virtual bool pre_process(const sensor_msgs::PointCloud2::ConstPtr ros_msg_in, PointCloudType::Ptr& pcl_xyzin_out){return true;};

    // for robosense & vanjee
    virtual bool msg2pcl_clip(const sensor_msgs::PointCloud2::ConstPtr ros_msg_in, PointCloudType::Ptr pcl_xyzin_out){return true;}
    
    // common
    void sampling_cloud(PointCloudType::Ptr in_cloud_ptr, PointCloudType::Ptr out_cloud_ptr);

    int get_cloud_dense_size(){
        return cloud_dense_->points.size();
    }

    bool set_common_params();


protected:
    // virtual bool set_param()=0;
    // virtual void msg2pcl_clip()=0;

private:

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// member variable

protected:
    PointCloudType::Ptr cloud_dense_;

    int extract_cloud_method_ = 0;
    int col_cnt_ = 1200;
    int ring_cnt_ = 48;
    double edge_curv_thr_ = 1.0;
    double surf_curv_thr_ = 0.1;

    int cloud_size_to_keep_ = 2000;
    double blind_range_square_ = 0.0;
    double max_range_square_ = 0.0;


    int point_filter_num_ = 2;
    int ring_filter_num_ = 1;
    std::vector<float> z_range_={-5.0, 20.0};

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
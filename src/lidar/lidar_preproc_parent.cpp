# include "lidar/lidar_preproc_parent.h"

namespace localization_module {

LidarPreprocParent::LidarPreprocParent(){

}


LidarPreprocParent::~LidarPreprocParent(){
    

}


// bool LidarPreprocParent::msg2pcl_clip(const sensor_msgs::PointCloud2::ConstPtr &ros_msg_in, std::shared_ptr<livox_ros::LidarMsg> &lvx_msg_out){return true;}
// bool LidarPreprocParent::msg2pcl_clip(const sensor_msgs::PointCloud2::ConstPtr &ros_msg_in, pcl::PointCloud<RsPointXYZIRT>::Ptr &pcl_cld_out){return true;}

// bool LidarPreprocParent::pre_process(const std::shared_ptr<livox_ros::LidarMsg> msg, PointCloudXYZI& pcl_cld_out){return true;}
// bool LidarPreprocParent::pre_process(const pcl::PointCloud<RsPointXYZIRT>::Ptr pcl_rs_in, PointCloudXYZI& pcl_cld_out){return true;}


} // namespace localization_module




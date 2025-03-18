# include "lidar/lidar_preproc_parent.h"

namespace localization_module {

LidarPreprocParent::LidarPreprocParent(){

}


LidarPreprocParent::~LidarPreprocParent(){
    

}

void LidarPreprocParent::sampling_cloud(PointCloudXYZI::Ptr in_cloud_ptr, PointCloudXYZI::Ptr out_cloud_ptr){
    int in_size = in_cloud_ptr->points.size();
    int cal_ratio = std::round(in_size * 1.0 / cloud_size_to_keep_);
    int point_filter_ratio = cal_ratio > 1 ? cal_ratio : 1;

    int out_size = in_size / point_filter_ratio;
    out_cloud_ptr->points.resize(out_size);

#pragma omp parallel for num_threads(MP_PROC_NUM)
    for (int i = 0; i < out_size; ++i){
        const auto pointFrom = in_cloud_ptr->points[i];
        out_cloud_ptr->points[i] = in_cloud_ptr->points[i * point_filter_ratio];
        out_cloud_ptr->points[i].curvature = in_cloud_ptr->points[i * point_filter_ratio].curvature * 1000; // unit=ms, 单位从秒转换为毫秒
    }

    out_cloud_ptr->header = in_cloud_ptr->header;
    out_cloud_ptr->width    = out_cloud_ptr->points.size();
    out_cloud_ptr->height   = 1;
    out_cloud_ptr->is_dense = 1;

    return ;
}                                                       


// bool LidarPreprocParent::msg2pcl_clip(const sensor_msgs::PointCloud2::ConstPtr &ros_msg_in, std::shared_ptr<livox_ros::LidarMsg> &lvx_msg_out){return true;}
// bool LidarPreprocParent::msg2pcl_clip(const sensor_msgs::PointCloud2::ConstPtr &ros_msg_in, pcl::PointCloud<RsPointXYZIRT>::Ptr &pcl_cld_out){return true;}

// bool LidarPreprocParent::pre_process(const std::shared_ptr<livox_ros::LidarMsg> msg, PointCloudXYZI& pcl_cld_out){return true;}
// bool LidarPreprocParent::pre_process(const pcl::PointCloud<RsPointXYZIRT>::Ptr pcl_rs_in, PointCloudXYZI& pcl_cld_out){return true;}


} // namespace localization_module




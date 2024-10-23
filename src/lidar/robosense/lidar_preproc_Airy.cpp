#include "lidar/robosense/lidar_preproc_Airy.h"


namespace localization_module {

LidarPreprocAiry::LidarPreprocAiry(){
    if(!set_param()){
        ROS_ERROR("Set lidar param failed!");
    }else {
        ROS_INFO("\033[1;32mSet lidar-Airy param successfully!\033[0m");
    }
}


LidarPreprocAiry::~LidarPreprocAiry(){

}


bool LidarPreprocAiry::set_param(){

    return true;
}

bool LidarPreprocAiry::msg2pcl_clip(const sensor_msgs::PointCloud2::ConstPtr ros_msg_in, pcl::PointCloud<RsPointXYZIRT>::Ptr pcl_cld_out){
    int test = 0;

    return true;
}

///////////////// 入口函数 /////////////////
                  
bool LidarPreprocAiry::pre_process(const pcl::PointCloud<RsPointXYZIRT>::Ptr pcl_rs_in, PointCloudXYZI::Ptr pcl_cld_out){
    int temp = 0;


    return true;

}

} // namespace localization_module
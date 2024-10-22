#include "lidar/robosense/lidar_preproc_Airy.h"


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

}

bool LidarPreprocAiry::msg2pcl_clip(){

}

///////////////// 入口函数 /////////////////
bool LidarPreprocAiry::process(){

}
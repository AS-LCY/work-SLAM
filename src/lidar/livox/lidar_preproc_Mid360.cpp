#include "lidar/livox/lidar_preproc_Mid360.h"


LidarPreprocMid360::LidarPreprocMid360(){
    if(!set_param()){
        ROS_ERROR("Set lidar param failed!");
    }else {
        ROS_INFO("\033[1;32mSet lidar-Mid360 param successfully!\033[0m");
    }


}


LidarPreprocMid360::~LidarPreprocMid360(){
    
}

bool LidarPreprocMid360::set_param(){

}


bool LidarPreprocMid360::msg2pcl_clip(){

}


///////////////// 入口函数 /////////////////
bool LidarPreprocAiry::process(){
    
}
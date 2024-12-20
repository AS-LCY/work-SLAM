#include <ros/ros.h>
#include "node/localization_module.h"


int main(int argc,char **argv){

    ros::init(argc, argv, "localization_module");
    ros::NodeHandle nh;
    std::string curr_path;
    // #ifdef CURRENT_DIR
    //     // std::cout<<"CURRENT_DIR is defined"<<std::endl;
    //     ROS_INFO("CURRENT_DIR is defined");
    //     curr_path = CURRENT_DIR;
    // #else
    //     // std::cout<<"CURRENT_DIR is not defined"<<std::endl;
    //     ROS_WARN("CURRENT_DIR is not defined");
    // #endif
    
    // 设置locale为默认值，以支持当前系统的默认编码
    setlocale(LC_ALL, "");

    int init_module_status = 0;
    nh.param<int>("/flbot/lidar_slam/common/init_module_status", init_module_status, 0);

    localization_module::ModuleStatus init_status = static_cast<localization_module::ModuleStatus>(init_module_status);
    

    localization_module::LocalizationModule localization_module(/*curr_path,*/ init_status);
    
    ROS_INFO("\033[1;32m----> localization_module start! \033[0m");

    // ros::MultiThreadedSpinner spinner(10);
    // spinner.spin();
    // ros::spin();
    return 0;
}


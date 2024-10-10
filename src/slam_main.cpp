#include <ros/ros.h>
#include "node/localization_module.h"


int main(int argc,char **argv){

    ros::init(argc, argv, "localization_module");
    ros::NodeHandle nh;
    std::string curr_path;
    #ifdef CURRENT_DIR
        std::cout<<"CURRENT_DIR is defined"<<std::endl;
        curr_path = CURRENT_DIR;
    #else
        std::cout<<"CURRENT_DIR is not defined"<<std::endl;
    #endif

    int init_module_status = 0;
    nh.param<int>("/flbot/lidar_slam/common/init_module_status", init_module_status, 0);

    localization_module::ModuleStatus init_status = static_cast<localization_module::ModuleStatus>(init_module_status);
    

    localization_module::LocalizationModule localization_module(/*curr_path,*/ init_status);
    
    ROS_INFO("\033[1;32m----> localization_module start! \033[0m");

    // ros::MultiThreadedSpinner spinner(10);
    // spinner.spin();
    ros::spin();
    return 0;
}


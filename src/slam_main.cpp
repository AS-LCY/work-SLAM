#include <ros/ros.h>
#include "localization_module/localization_module.h"


int main(int argc,char **argv){

    ros::init(argc, argv, "localization_module");
    std::string curr_path;
    #ifdef CURRENT_DIR
        std::cout<<"CURRENT_DIR is defined"<<std::endl;
        curr_path = CURRENT_DIR;
    #else
        std::cout<<"CURRENT_DIR is not defined"<<std::endl;
        exit(0);
    #endif

    localization_module::LocalizationModule localization_mod(curr_path);
    

    ros::MultiThreadedSpinner spinner(5);
    spinner.spin();
    return 0;
}


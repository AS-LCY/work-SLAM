#include <ros/ros.h>
#include "node/localization_module.h"

#include <signal.h>
#include <sys/resource.h>
#define SAVE_CORE_DUMP

#define CORE_SIZE 1024 * 1024 * 500 * 1.2


int main(int argc,char **argv){
#ifdef SAVE_CORE_DUMP
    ROS_INFO("save core dump is enable");
    // 程序崩溃核心转储
    struct rlimit rlmt;
    if (getrlimit(RLIMIT_CORE, &rlmt) == -1) {
        return -1;
    }
    ROS_INFO(
        "Before set rlimit CORE dump current is:%d, max is:%d", (int)rlmt.rlim_cur,
        (int)rlmt.rlim_max);

    rlmt.rlim_cur = (rlim_t)CORE_SIZE;
    rlmt.rlim_max = (rlim_t)CORE_SIZE;
    if (setrlimit(RLIMIT_CORE, &rlmt) == -1) {
        return -1;
    }
    if (getrlimit(RLIMIT_CORE, &rlmt) == -1) {
        return -1;
    }
    ROS_INFO(
        "After set rlimit CORE dump current is:%d, max is:%d", (int)rlmt.rlim_cur,
        (int)rlmt.rlim_max);
    // //! core缓存数设为三个 运行roslaunch一般保存在 $HOME/.ros/下
    // std::string HOME(getenv("HOME"));
    // // if (boost::filesystem::exists(HOME + "/.ros/core-decision_planni")) {
    // if (boost::filesystem::exists(HOME + "/.ros/core-lidar_slam_node")) {
    //     system("rm core-lidar_slam_node");
    // }
#else
    ROS_INFO("save core dump is disable");
#endif

    ROS_INFO("\033[1;32m----> ros init \033[0m");

    ros::init(argc, argv, "localization_module");
    ros::NodeHandle nh;
    std::string curr_path;
    
    // 设置locale为默认值，以支持当前系统的默认编码
    setlocale(LC_ALL, "");

    int init_module_status = 0;
    nh.param<int>("/flbot/lidar_slam/common/init_module_status", init_module_status, 0);

    localization_module::ModuleStatus init_status = static_cast<localization_module::ModuleStatus>(init_module_status);
    

    ROS_INFO("\033[1;32m----> localization_module starting! \033[0m");
    localization_module::LocalizationModule localization_module(init_status);
    

    return 0;
}


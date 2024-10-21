#ifndef LOCALIZATION_MODULE_STATUS_DEF_H
#define LOCALIZATION_MODULE_STATUS_DEF_H
#include <string>
#include <vector>

#include "lidar_slam/common_lib.h"


namespace localization_module{

using namespace std;

enum class ModuleStatus{
    MODULE_IDLE = 0,
    MODULE_MAPPING =1,
    MODULE_SEC_MAPPING =2,
    MODULE_LOCALIZATION = 3,
    MODULE_STARTING_SLAM = 4,
    MODULE_STOPPING_SLAM = 5
};


static string print_ModuleStatus(ModuleStatus e){
    switch (e){
        case ModuleStatus::MODULE_IDLE: return "IDLE";
        case ModuleStatus::MODULE_MAPPING: return "MAPPING";
        case ModuleStatus::MODULE_SEC_MAPPING: return "SEC_MAPPING";
        case ModuleStatus::MODULE_LOCALIZATION: return "LOCALIZATION";
        case ModuleStatus::MODULE_STARTING_SLAM: return "STARTING_SLAM";
        case ModuleStatus::MODULE_STOPPING_SLAM: return "STOPPING_SLAM";
        default:
            break;
    }
    return "UNKNOW_ModuleStatus!";
}


} //namespace localization_module

namespace lidar_slam{
using namespace std;

enum LocalizationStatus{
    L_INACTIVE = 0,
    L_RELOCALIZING =1,
    L_RELOCALIZE_FAILED = 2,
    L_NORMAL = 3,
    L_LOW_ACCURACY = 4,
    L_FAILED = 5
};

enum MappingStatus{
    M_INACTIVE = 0,
    M_RELOCALIZING =1,
    M_RELOCALIZE_FAILED =2,
    M_CREATING_ELE = 3,
    M_STANDBY =4,
    M_FAILED = 5
};

static string print_LocalizationStatus(LocalizationStatus e){
    switch (e){
    CASE_STR(L_INACTIVE);
    CASE_STR(L_RELOCALIZING);
    CASE_STR(L_RELOCALIZE_FAILED);
    CASE_STR(L_NORMAL);
    CASE_STR(L_LOW_ACCURACY);
    default:
        break;
    }
    return "UNKNOW_LocalizationStatus!";
}

static string print_MappingStatus(MappingStatus e){
    switch (e){
    CASE_STR(M_INACTIVE);
    CASE_STR(M_RELOCALIZING);
    CASE_STR(M_RELOCALIZE_FAILED);
    CASE_STR(M_CREATING_ELE);
    CASE_STR(M_STANDBY);
    CASE_STR(M_FAILED);
    default:
        break;
    }
    return "UNKNOW_MappingStatus!";
}


}// namespace lidar_slam

#endif
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



enum ModuleStatus_o{          // 发布出去的状态
    IDLE = 0,             // # 0: 空闲，等待（建图/定位）指令, default
    STARTING = 1,        // # 1: 正在启动建图或定位
    MAPPING = 2,         // # 2: 建图模式
    LOCALIZATION = 3,     //# 3: 定位模式
    STOPPING = 4         // # 4: 正在退出建图或定位
};

enum LocalizationStatus {
    //仅当 module_status == LOCALIZATION 时, localization_status 有效
    L_INACTIVE = 0,             // # 0: 定位模式未激活,  default
    L_RELOCALIZING = 1,         // # 1: 重定位
    L_RELOCALIZE_FAILED = 2,    // # 2: 重定位失败
    L_NORMAL = 3,               // # 3: 定位中，定位精度正常
    L_LOW_ACCURACY = 4,          //# 4: 定位中，定位精度低
    L_FAILED = 5               // # 5: 定位失败
};

enum MappingStatus{
//仅当 module_status == MAPPING 时, mapping_status 有效
M_INACTIVE = 0,//              # 0: 建图模式未激活, default
M_RELOCALIZING = 1,//          # 1: 重定位
M_RELOCALIZE_FAILED = 2,//     # 2: 重定位失败
M_STANDBY = 3,//               # 3: 建图中，等待创建地图元素
M_CREATING_ELE = 4,//          # 4: 建图中，正在创建元素 # 弃用
M_FAILED = 5                //# 5: 建图失败
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

// namespace lidar_slam{
namespace localization_module{
using namespace std;

// enum LocalizationStatus{
//     L_INACTIVE = 0,
//     L_RELOCALIZING =1,
//     L_RELOCALIZE_FAILED = 2,
//     L_NORMAL = 3,
//     L_LOW_ACCURACY = 4,
//     L_FAILED = 5
// };

// enum MappingStatus{
//     M_INACTIVE = 0,
//     M_RELOCALIZING =1,
//     M_RELOCALIZE_FAILED =2,
//     M_STANDBY =3,
//     M_CREATING_ELE = 4,
//     M_FAILED = 5
// };

// static string print_LocalizationStatus(LocalizationStatus e){
//     switch (e){
//     CASE_STR(L_INACTIVE);
//     CASE_STR(L_RELOCALIZING);
//     CASE_STR(L_RELOCALIZE_FAILED);
//     CASE_STR(L_NORMAL);
//     CASE_STR(L_LOW_ACCURACY);
//     default:
//         break;
//     }
//     return "UNKNOW_LocalizationStatus!";
// }

// static string print_MappingStatus(MappingStatus e){
//     switch (e){
//     CASE_STR(M_INACTIVE);
//     CASE_STR(M_RELOCALIZING);
//     CASE_STR(M_RELOCALIZE_FAILED);
//     CASE_STR(M_CREATING_ELE);
//     CASE_STR(M_STANDBY);
//     CASE_STR(M_FAILED);
//     default:
//         break;
//     }
//     return "UNKNOW_MappingStatus!";
// }


}// namespace localization_module

#endif
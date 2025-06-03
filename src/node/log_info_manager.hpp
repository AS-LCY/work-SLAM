#ifndef LOCALIZATION_MODULE_LOG_INFO_MANAGER_H
#define LOCALIZATION_MODULE_LOG_INFO_MANAGER_H

#include <std_msgs/Float64MultiArray.h>
// #include "fairland_msgs/LocalizationModuleLogInfo.h"
// #include "fairland_msgs/LocalizationModuleStatus.h"
#include "node/module_status_def.h"

namespace localization_module{

class LocalizationModuleLogInfoManager{
private:
    LocalizationModuleLogInfoManager() {};
    ~LocalizationModuleLogInfoManager() {};
    LocalizationModuleLogInfoManager(const LocalizationModuleLogInfoManager &);
    LocalizationModuleLogInfoManager & operator=(const LocalizationModuleLogInfoManager &) = delete;

public:
    
    static LocalizationModuleLogInfoManager * getInstance() { 
        static LocalizationModuleLogInfoManager * instance;
        if (instance == nullptr) {
            instance = new LocalizationModuleLogInfoManager();
        }
        return instance; 
    };

    std_msgs::Float64MultiArray slam_info;
    std_msgs::Float64MultiArray fusion_info;


    void reset_log_info(){
        slam_info.data.clear();
        slam_info.data.resize(40);
        fusion_info.data.clear();
        fusion_info.data.resize(30);
    }
}; // class LocalizationModuleLogInfoManager


} // namespace localization_module


#endif
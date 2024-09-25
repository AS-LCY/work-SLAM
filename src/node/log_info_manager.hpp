#ifndef LOCALIZATION_MODULE_LOG_INFO_MANAGER_H
#define LOCALIZATION_MODULE_LOG_INFO_MANAGER_H

#include "fairland_msgs/LocalizationModuleLogInfo.h"

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

    fairland_msgs::LocalizationModuleLogInfo log_info;

}; // class LocalizationModuleLogInfoManager


} // namespace localization_module


#endif
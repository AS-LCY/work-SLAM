#ifndef LOCALIZATION_MODULE_LOG_INFO_MANAGER_H
#define LOCALIZATION_MODULE_LOG_INFO_MANAGER_H

#include "fairland_msgs/LocalizationModuleLogInfo.h"
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

    fairland_msgs::LocalizationModuleLogInfo log_info;
    // fairland_msgs::LocalizationModuleStatus status_info;

    localization_module::ModuleStatus module_status;
    lidar_slam::LocalizationStatus l_status;
    lidar_slam::MappingStatus m_status;

    void reset_log_info(){
        fairland_msgs::LocalizationModuleLogInfo temp_log_info;
        log_info = temp_log_info;
    }
    void reset_module_status(){
        // status_info.module_status = fairland_msgs::LocalizationModuleStatus::IDLE;
        // status_info.localization_status = fairland_msgs::LocalizationModuleStatus::L_INACTIVE;
        // status_info.mapping_status = fairland_msgs::LocalizationModuleStatus::M_INACTIVE;
        module_status = localization_module::MODULE_IDLE;
        l_status = lidar_slam::L_INACTIVE;
        m_status = lidar_slam::M_INACTIVE;
    }

}; // class LocalizationModuleLogInfoManager


} // namespace localization_module


#endif
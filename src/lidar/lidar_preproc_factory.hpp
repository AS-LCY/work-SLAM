
#ifndef FLBOT_LIDAR_PREPROC_FACTORY_HPP
#define FLBOT_LIDAR_PREPROC_FACTORY_HPP

#include <memory>
#include <string>
#include <ros/ros.h>

#include "lidar/lidar_preproc_parent.h"
#include "lidar/livox/lidar_preproc_Mid360.h"
#include "lidar/robosense/lidar_preproc_Airy.h"


namespace localization_module {

class LidarPreprocFactory{
public:

    static std::shared_ptr<LidarPreprocParent> new_lidar_preproc(const int lidar_type){
        std::shared_ptr<LidarPreprocParent> lidar_preproc_tmp;
        ROS_INFO("---");
        ROS_INFO("valid lidar_type in factory: 1-lvx-Mid360 | 2-RS-Airy | 3-Vanjee722 ");
        ROS_INFO_STREAM(BOLDGREEN<<"curr lidar_type: "<<lidar_type<<RESET);
        if (lidar_type == 1){
            lidar_preproc_tmp.reset(new LidarPreprocMid360());
        }else if(lidar_type == 2){ 
            lidar_preproc_tmp.reset(new LidarPreprocAiry());
        }else if(lidar_type == 3){ // vanjee 数据类型与 rslidar 一样, 共用
            lidar_preproc_tmp.reset(new LidarPreprocAiry());
        }else {
            lidar_preproc_tmp.reset();
            ROS_ERROR_STREAM(RED << "Unknown lidar type(==" << lidar_type <<") in lidar factory!" <<RESET);
            exit(0);
        }

        return std::move(lidar_preproc_tmp);
    }
    

    // static std::shared_ptr<LidarPreprocParent> new_lidar_preproc(const int lidar_type, std::string prefix){
    //     std::shared_ptr<LidarPreprocParent> lidar_preproc_tmp;
    //     ROS_INFO("---");
    //     ROS_INFO("valid lidar_type in factory: 1-Mid360 | 2-Airy ");
    //     ROS_INFO("curr  lidar_type: %s", lidar_type.c_str());
    //     if (lidar_type == 1){
    //         lidar_preproc_tmp.reset(new LidarPreprocMid360(prefix));
    //     }else if(lidar_type == 2){ 
    //         lidar_preproc_tmp.reset(new LidarPreprocAiry(prefix));
    //     } else {
    //         lidar_preproc_tmp.reset();
    //         ROS_ERROR_STREAM(RED << "Unknown lidar type in lidar factory!" << RESET);
    //         exit(0);
    //     }

    //     return std::move(lidar_preproc_tmp);
    // }


};

} // namespace localization_module
#endif

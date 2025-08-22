
#ifndef FLBOT_LIDAR_PREPROC_FACTORY_HPP
#define FLBOT_LIDAR_PREPROC_FACTORY_HPP

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "lidar/lidar_preproc_parent.h"
#include "lidar/livox/lidar_preproc_Mid360.h"
#include "lidar/robosense/lidar_preproc_Airy.h"
#include "lidar/vanjee/lidar_preproc_Vanjee722.h"
#include "lidar/lanhai/lidar_preproc_M300.h"
#include "lidar/hesai/lidar_preproc_JT16.h"

namespace localization_module {

class LidarPreprocFactory{
public:
    static std::shared_ptr<LidarPreprocParent> new_lidar_preproc(const int lidar_type, rclcpp::Node::SharedPtr node){
        std::shared_ptr<LidarPreprocParent> lidar_preproc_tmp;
        auto logger = rclcpp::get_logger("lidar_preproc_factory");
        
        RCLCPP_INFO(logger, "---");
        RCLCPP_INFO(logger, "valid lidar_type in factory: 1-lvx-Mid360 | 2-RS-Airy | 3-Vanjee722 | 4-HS-JT16 | 5-BS-M300");
        RCLCPP_INFO(logger, "curr lidar_type: %d", lidar_type);
        
        if (lidar_type == 1){
            lidar_preproc_tmp.reset(new LidarPreprocMid360(node));
        }else if(lidar_type == 2){ 
            lidar_preproc_tmp.reset(new LidarPreprocAiry(node));
        }else if(lidar_type == 3){
            lidar_preproc_tmp.reset(new LidarPreprocVanjee722(node));
        }else if(lidar_type == 4){
            lidar_preproc_tmp.reset(new LidarPreprocJT16(node));
        }else if(lidar_type == 5){
            lidar_preproc_tmp.reset(new LidarPreprocM300(node));
        }else {
            lidar_preproc_tmp.reset();
            RCLCPP_ERROR(logger, "Unknown lidar type(==%d) in lidar factory!", lidar_type);
            exit(0);
        }

        return std::move(lidar_preproc_tmp);
    }
};

} // namespace localization_module
#endif

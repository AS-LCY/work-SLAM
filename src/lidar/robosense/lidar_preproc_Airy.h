
#ifndef FLBOT_LIDAR_PREPROC_AIRY_H
#define FLBOT_LIDAR_PREPROC_AIRY_H

#include <string>
#include <ros/ros.h>

#include "lidar/lidar_preproc_parent.h"




class LidarPreprocAiry: public LidarPreprocParent{

public:
    LidarPreprocAiry();
    ~LidarPreprocAiry();



private:
    bool set_param();

    bool msg2pcl_clip();




};

#endif
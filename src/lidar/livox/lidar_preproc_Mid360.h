#ifndef FLBOT_LIDAR_PREPROC_MID360_H
#define FLBOT_LIDAR_PREPROC_MID360_H

#include <ros/ros.h>
#include <string>

#include "lidar/livox/point_type_livox_def.h"
#include "lidar/lidar_preproc_parent.h"

class LidarPreprocMid360: public LidarPreprocParent{

public:
    LidarPreprocMid360();
    ~LidarPreprocMid360();


private:
    bool set_param();
    bool msg2pcl_clip();




};

#endif
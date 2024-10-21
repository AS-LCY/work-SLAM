
#ifndef FLBOT_LIDAR_PREPROC_PARENT_H
#define FLBOT_LIDAR_PREPROC_PARENT_H

#include <string>

#include "node/param_manager.hpp"

class LidarPreprocParent{

public:
    LidarPreprocParent();
    ~LidarPreprocParent();

    virtual bool process()=0;

protected:
    virtual bool set_param()=0;
    virtual void msg2pcl_clip()=0;

private:
    




};

#endif 
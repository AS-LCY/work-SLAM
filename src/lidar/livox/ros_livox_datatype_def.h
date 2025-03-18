#ifndef LIVOX_ROS_DATATYPE_DEF_H
#define LIVOX_ROS_DATATYPE_DEF_H

#include "fairland_msgs/LivoxCustomMsg.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>

namespace livox_ros {

/** Send pointcloud message Data to ros subscriber or save them in rosbag file */
typedef enum {
  kOutputToRos = 0,
  kOutputToRosBagFile = 1,
} DestinationOfMessageOutput;

/** The message type of transfer */
typedef enum {
  kPointCloud2Msg = 0,
  kLivoxCustomMsg = 1,
  kPclPxyziMsg = 2,
  kLivoxImuMsg = 3,
} TransferType;

// using CustomMsg = fairland_msgs::LivoxCustomMsg;
// using CustomPoint = fairland_msgs::LivoxCustomPoint;

/** Type-Definitions based on ROS versions */
/*using Publisher = ros::Publisher;
using PublisherPtr = ros::Publisher*;
using PointCloud2 = sensor_msgs::PointCloud2;
using PointField = sensor_msgs::PointField;
using CustomMsg = livox_ros_driver2::CustomMsg;
using CustomPoint = livox_ros_driver2::CustomPoint;
using ImuMsg = sensor_msgs::Imu;


using PointCloud = pcl::PointCloud<pcl::PointXYZI>*/
struct LidarPoint{
    // u_int32_t offset_time;//      # offset time relative to the base time
    double offset_time;//      # offset time relative to the base time
    float x;//               # X axis, unit:m
    float y;//               # Y axis, unit:m
    float z;//               # Z axis, unit:m
    u_int8_t reflectivity;//      # reflectivity, 0~255
    u_int8_t tag;//               # livox tag
    u_int8_t line;//              # laser number in lidar
};
struct LidarMsg{
    double time_stamp;
    u_int16_t point_num;
    u_int8_t lidar_id;
    u_int8_t rsvd[3];
    std::vector<LidarPoint> points; 

};

struct ImuMsg{
  double time_stamp;
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d linear_acceleration;
};

} // namespace livox_ros

#endif
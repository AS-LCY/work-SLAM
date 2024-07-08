#ifndef PUBLISH_COMMON_H
#define PUBLISH_COMMON_H

#include <string>
#include <vector>
#include <cstdlib>
// ros
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>

// ros-msg
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Eigen
#include <Eigen/Core>

// pcl
#define PCL_NO_PRECOMPILE
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>

// cv
#include <opencv2/opencv.hpp>

// livox
// #include <livox_ros_driver2/CustomMsg.h>
#include <flbot_msgs/LivoxCustomMsg.h>

#include <Viewer.hpp>
#include "v4l2cam.h"
#include "lidar_slam.hpp"



namespace localization_module{

void pub_odom_cloud(PointCloudXYZI::Ptr msg_in, ros::Publisher pubOdomCloud);
void pub_lidar_cloud(PointCloudXYZI::Ptr msg_in, ros::Publisher pubBodyCloud);

void pub_obstacle_cloud(PointCloudXYZI::Ptr msg_in, ros::Publisher pubObstacleCloud);


void pub_filtered_obstacle_cloud(PointCloudXYZI::Ptr msg_in, ros::Publisher pubFilteredObstacleCloud);

void pub_test_cloud(PointCloudXYZI::Ptr msg_in, bool localization_mode,ros::Publisher pubTestCloud);

void pub_kdtree_cloud(PointCloudXYZI::Ptr msg_in, ros::Publisher pubKdtreeCloud);


void publish_odometry(const Eigen::Isometry3d lidar_in_odom, ros::Publisher pubOdomAftMapped);

void publish_static_transform(const Eigen::Isometry3d wheel_in_lidar);


void publish_transform(const Eigen::Isometry3d& correction,string parent, string child);


void publish_lidar_to_map(const Eigen::Isometry3d& lidar_in_map, ros::Publisher pubOdomCloud);

void visualizeLoopClosure(map<int, int> loopIndexContainer, nav_msgs::Path optimized_path_msg, ros::Publisher pubLoopConstraintEdge);


void show_keyframe(std::vector<ScInfo> loadKeyframe, ros::Publisher pubKeyframePose);

void pub_rgb_map(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud, ros::Publisher pubRgbCloud);

} // namespace localization_module




#endif
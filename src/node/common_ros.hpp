#pragma once

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "lidar_slam/common_lib.h"

namespace localization_module{

static double get_yaw_from_orientation(const geometry_msgs::msg::Quaternion& orientation) {
    Eigen::Quaterniond quat(
        orientation.w,
        orientation.x,
        orientation.y,
        orientation.z
    );

    Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();
    Eigen::Vector3d angles = R2ypr(rotation_matrix);
    return angles[0]; // yaw
}

static geometry_msgs::msg::Pose eigen_isometry_to_geo_pose(const Eigen::Isometry3d& eigen_transform) {
    geometry_msgs::msg::Pose geo_pose;
    geo_pose.position.x = eigen_transform.translation().x();
    geo_pose.position.y = eigen_transform.translation().y();
    geo_pose.position.z = eigen_transform.translation().z();

    Eigen::Quaterniond quaternion(eigen_transform.linear());
    geo_pose.orientation.x = quaternion.x();
    geo_pose.orientation.y = quaternion.y();
    geo_pose.orientation.z = quaternion.z();
    geo_pose.orientation.w = quaternion.w();

    return geo_pose;
}

static nav_msgs::msg::Odometry isometry3d_to_odom(
    const Eigen::Isometry3d& isometry_in, 
    const std::string& frame_in, 
    const std::string& child_frame_in,
    const rclcpp::Time& stamp = rclcpp::Time()
) {
    nav_msgs::msg::Odometry res_odometry;
    res_odometry.header.stamp = stamp;
    res_odometry.header.frame_id = frame_in;
    res_odometry.child_frame_id = child_frame_in;
    
    res_odometry.pose.pose.position.x = isometry_in.translation().x();
    res_odometry.pose.pose.position.y = isometry_in.translation().y();
    res_odometry.pose.pose.position.z = isometry_in.translation().z();
    
    Eigen::Quaterniond quaternion = Eigen::Quaterniond(isometry_in.rotation());
    res_odometry.pose.pose.orientation.x = quaternion.x();
    res_odometry.pose.pose.orientation.y = quaternion.y();
    res_odometry.pose.pose.orientation.z = quaternion.z();
    res_odometry.pose.pose.orientation.w = quaternion.w();

    return res_odometry;
}





//////////////////////////////////////////////||||||/////

// ROS2 中不再需要 odom_to_transform 函数，使用 tf2 库替代
// 替代方案：直接使用 tf2::fromMsg() 和 Eigen::Isometry3d 转换
// 示例：
//   Eigen::Isometry3d transform = tf2::transformToEigen(transform_stamped.transform);

} // namespace localization_module
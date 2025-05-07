#pragma once

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "lidar_slam/common_lib.h"

namespace localization_module{

static double get_yaw_from_orientation(geometry_msgs::Quaternion orientation){
    Eigen::Quaterniond quat;
    quat.x() = orientation.x;
    quat.y() = orientation.y;
    quat.z() = orientation.z;
    quat.w() = orientation.w;

    Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();
    Eigen::Vector3d angles = R2ypr(rotation_matrix);
    double yaw   = angles[0];
    // double pitch = angles[1];
    // double roll  = angles[2];

    return yaw;
}

static geometry_msgs::Pose eigen_isometry_to_geo_pose(Eigen::Isometry3d eigen_transform){
    geometry_msgs::Pose geo_pose;
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

static nav_msgs::Odometry isometry3d_to_odom(const Eigen::Isometry3d isometry_in, std::string frame_in, std::string child_frame_in){
    nav_msgs::Odometry res_odometry;
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

static tf::Transform odom_to_transform(const nav_msgs::Odometry odom_in){
    tf::Transform res_transform;
    
    tf::Quaternion q;
    res_transform.setOrigin(tf::Vector3(odom_in.pose.pose.position.x,
                                        odom_in.pose.pose.position.y,
                                        odom_in.pose.pose.position.z));
    q.setW(odom_in.pose.pose.orientation.w);
    q.setX(odom_in.pose.pose.orientation.x);
    q.setY(odom_in.pose.pose.orientation.y);
    q.setZ(odom_in.pose.pose.orientation.z);
    res_transform.setRotation(q);    

    return res_transform;
}

} // namespace localization_module
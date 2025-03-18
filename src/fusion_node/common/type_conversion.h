#ifndef FLBOT_LOCALIZATION_COMMON_TYPE_CONVERSION_H
#define FLBOT_LOCALIZATION_COMMON_TYPE_CONVERSION_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h>

namespace localization_module{
namespace common{
namespace conversions{

inline 
tf::Transform odom_to_transform(const nav_msgs::Odometry odom_in){
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


inline 
nav_msgs::Odometry isometry3d_to_odom(const Eigen::Isometry3d isometry_in, std::string frame_in, std::string child_frame_in){
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


    
} // namespace type_conversion    
} // namespace common    
} // namespace localization_module




#endif // FLBOT_LOCALIZATION_COMMON_TYPE_CONVERSION_H
#ifndef FLBOT_LIDAR_SLAM_DATASTRUCT_DEFINE_H
#define FLBOT_LIDAR_SLAM_DATASTRUCT_DEFINE_H

#include <Eigen/Core>
#include <Eigen/Dense>

namespace lidar_slam {

struct KeyPose {
	Eigen::Isometry3d pose;
	int index;
	double time;
	double roll;
	double pitch;
	double yaw;
}; // defined in backend before

struct ScInfo {
	int id;
	Eigen::Isometry3d pose;
	Eigen::MatrixXd polarcontext;
}; // defined in scanContext before

} // namespace lidar_slam

#endif
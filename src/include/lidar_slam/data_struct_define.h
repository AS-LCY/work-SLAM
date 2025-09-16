#ifndef FLBOT_LIDAR_SLAM_DATASTRUCT_DEFINE_H
#define FLBOT_LIDAR_SLAM_DATASTRUCT_DEFINE_H

#include <Eigen/Core>
#include <Eigen/Dense>

namespace lidar_slam {

struct KeyPose {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
	int index = -1;
	double time = -1.f;
	double roll = 0.f;
	double pitch = 0.f;
	double yaw = 0.f;
	KeyPose() {}
	KeyPose(const Eigen::Isometry3d& _pose, const int& idx, const double& t, const double& r, const double& p,
			const double& y)
		: pose(_pose), index(idx), time(t), roll(r), pitch(p), yaw(y) {}
};

struct ScInfo {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	int id;
	Eigen::Isometry3d pose;
	Eigen::MatrixXd polarcontext;
};

} // namespace lidar_slam

#endif
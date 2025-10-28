
#pragma once

#include <logTracer/tracer.h>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "lidar_slam/common_lib.h"

namespace lidar_slam {

class EkfSmoother {
	DECL_CLASSNAME(EkfSmoother)

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   public:
	EkfSmoother();
	~EkfSmoother();

	void init(const Sophus::SE3d& T_init, const Eigen::Matrix<double, 6, 6>& P_init);

	void processModel(const Sophus::SE3d& T_prev_curr_input, const Eigen::Matrix<double, 6, 6> Q);

	void update(const Sophus::SE3d& T_curr_meas, const Eigen::Matrix<double, 6, 6>& R);

	Sophus::SE3d getState() const { return T_; }

   private:
	Sophus::SE3d T_;				//状态变量， T_map_lidar
	Eigen::Matrix<double, 6, 6> P_; //协方差矩阵, 前三维平移，后三维旋转
};

} // namespace lidar_slam
#include "lidar_slam/ekf_smoother.h"

#include <Eigen/Dense>
#include <sophus/se3.hpp>

namespace lidar_slam {

EkfSmoother::EkfSmoother() {
	T_ = Sophus::SE3d();
	P_ = Eigen::Matrix<double, 6, 6>::Identity() * 1e-6;
}

EkfSmoother::~EkfSmoother() = default;

void EkfSmoother::init(const Sophus::SE3d& T_init, const Eigen::Matrix<double, 6, 6>& P_init) {
	T_ = T_init;
	P_ = P_init;
}

// ekf估计的状态变量：T_map_lidar
// 过程模型： T_k+1 = T_k * U  (U = T_prev_curr_input)
void EkfSmoother::processModel(const Sophus::SE3d& T_prev_curr_input, const Eigen::Matrix<double, 6, 6> Q_global) {
	// TRACE_INFO_CLASS("EkfSmoother processModel...");
	// TRACE_INFO_CLASS("process noise = %f, %f, %f, %f, %f, %f", Q_global(0, 0), Q_global(1, 1), Q_global(2, 2),
	// 				 Q_global(3, 3), Q_global(4, 4), Q_global(5, 5));

	T_ = T_ * T_prev_curr_input;

	// 线性化雅可比
	Eigen::Matrix<double, 6, 6> J = T_prev_curr_input.inverse().Adj();

	//方差P是在全局坐标系下，所以过程噪声Q也是在全局系下
	P_ = J * P_ * J.transpose() + Q_global;
}

// 观测模型为单位阵
void EkfSmoother::update(const Sophus::SE3d& T_curr_meas, const Eigen::Matrix<double, 6, 6>& R) {
	// TRACE_INFO_CLASS("EkfSmoother update...");
	// TRACE_INFO_CLASS("measurement noise = %f, %f, %f, %f, %f, %f", R(0, 0), R(1, 1), R(2, 2), R(3, 3), R(4, 4),
	// 				 R(5, 5));

	// 计算测量残差：在李代数中 r = log( T_^{-1} * T_meas )
	Sophus::SE3d err = T_.inverse() * T_curr_meas;
	Eigen::Matrix<double, 6, 1> r = err.log();

	// 测量矩阵 H = I (在李代数空间)
	Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Identity();
	Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + R;
	Eigen::Matrix<double, 6, 6> K = P_ * H.transpose() * S.inverse();
	Eigen::Matrix<double, 6, 1> delta = K * r;
	Sophus::SE3d dT = Sophus::SE3d::exp(delta);
	T_ = T_ * dT;

	Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
	P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * R * K.transpose();

	// 对称化以抑制数值误差
	P_ = 0.5 * (P_ + P_.transpose());
}

} // namespace lidar_slam
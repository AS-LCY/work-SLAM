#include "pose_ekf.h"

#include <ros/ros.h>

#include <iostream>

#include "fusion_node/common/numerical_process.h"

namespace localization_module {

PoseEKF::PoseEKF() {}

PoseEKF::PoseEKF(const int& status_num, const int& input_num, const int& measure_num, const Matrix& status_cov,
				 const Matrix& input_cov, const Matrix& measure_cov, const double& dt,
				 const EkfGatingParams& gating_params) {
	set_dimension(status_num, input_num, measure_num);
	ekf_matrix_init();
	set_covariance(status_cov, input_cov, measure_cov);
	dt_ = dt;
}

PoseEKF::~PoseEKF() {}

void PoseEKF::reset(const Matrix& status_cov, const Matrix& input_cov, const Matrix& measure_cov) {
	ROS_WARN_STREAM("Reseting Pose-EKF ...");
	// set_dimension(status_num,input_num,measure_num); // dimension should not be changed
	ekf_matrix_init();
	set_covariance(status_cov, input_cov, measure_cov); // P_ need reset

	ROS_INFO_STREAM("P_ : ");
	ROS_INFO_STREAM(P_);
}

// void PoseEKF::params_initialize() {
//     ekf_matrix_init();
// }

void PoseEKF::move(const Matrix& input, const double& dt) {
	auto theta = status_(2, 0); // theta(k-1)
	auto vel = input(0, 0);		// vel(k-1)
	auto w = input(1, 0);		// omega(k-1)
	auto dist = vel * dt;
	// ROS_INFO("ekf move: dist: %.4f, dx: %.4f, dy: %.4f", dist, dist*std::cos(theta), dist*std::sin(theta));

	Eigen::Matrix<double, 3, 1> dx;
	dx << dist * std::cos(theta), dist * std::sin(theta), w * dt;
	// dx << dist*std::cos(theta),
	//     dist*std::sin(theta),
	//     0;

	status_ += dx;

	ROS_INFO("ekf move: dist: %7.4f, theta: %7.4f ", dist, theta);
	ROS_INFO("ekf move:   dx: %7.4f,    dy: %7.4f, dyaw: %7.4f", dx(0, 0), dx(1, 0), dx(2, 0));
}

void PoseEKF::predict(const Matrix& input, const double& dt) {
	status_pre_ = status_ * 1.0; // 更新上一帧后验
	move(input, dt);
	status_(2, 0) = localization_module::common::NumericalProcess::unify_angle(status_(2, 0));

	// compute f,v jacobian
	auto s = dt * std::sin(status_(2, 0));
	auto c = dt * std::cos(status_(2, 0));
	fj_(0, 2) = -s * input(0, 0);
	fj_(1, 2) = c * input(0, 0);

	vj_(0, 0) = c;
	vj_(1, 0) = s;
	vj_(2, 1) = dt;

	// compute p, 先验
	// P_=fj_*P_*fj_.transpose()+vj_*M_*vj_.transpose();
	P_ = fj_ * P_ * fj_.transpose() + M_;

	// copy
	status_prior_ = status_ * 1.0;
	P_prior_ = P_ * 1.0;
}

void PoseEKF::update(const Matrix& measure, bool trust_measure) {
	// trust_measure 暂时不用

	measure_ = measure * 1.0;
	// status_(2, 0) = measure_(2, 0);

	// h jacobian
	H_ = ekf_eye_n_; // demension: [status_num_ * status_num_]

	// temp mat for cal
	Matrix PHt = P_ * H_.transpose(); // temp mat, 多处用到
	S_ = H_ * PHt + R_;
	// hx
	Matrix hx = status_;
	Matrix y = residual(measure, hx); //残差

	// 卡尔曼增益
	K_ = PHt * S_.inverse();
	status_ = status_ + K_ * y;
	// ROS_INFO_STREAM("K_: " );
	// ROS_INFO_STREAM(K_);

	// numerically stable version
	Matrix IKH = ekf_eye_n_ - K_ * H_;
	P_ = IKH * P_ * IKH.transpose() + K_ * R_ * K_.transpose();
	// P_ = IKH*P_;

	status_post_ = status_ * 1.0;
	// printf("Matrix copy is %s\n", (status_post_(1,0)==status_prior_(1,0)?"shallow copy":"deep copy"));
	P_post_ = P_ * 1.0;
	// ROS_INFO_STREAM("P_post_: ");
	// ROS_INFO_STREAM(P_post_);
}

Matrix PoseEKF::get_mah_vec(const Matrix& res, const Matrix& measure) {
	// auto yaw = measure(2,0);
	// auto yc = std::cos(yaw);
	// auto ys = std::sin(yaw);
	// auto rx = res(0,0);
	// auto ry = res(1,0);
	// auto r_yaw = res(2,0);
	// auto r_s = rx * yc + ry * ys;
	// auto r_l = -rx * ys + ry * yc;

	// auto sx = std::sqrt(S_(0,0));
	// auto sy = std::sqrt(S_(1,1));
	// auto s_yaw = std::sqrt(S_(2,2));

	// auto s_s = sx * yc + sy * ys;
	// auto s_l = -sx * ys + sy * yc;
	// auto mah_station = std::fabs(r_s / s_s);
	// auto mah_lateral = std::fabs(r_l / s_l);
	// auto mah_yaw = std::fabs(r_yaw / s_yaw);

	Matrix mah_vec = Matrix::Zero(3, 3);
	// mah_vec<< mah_station, mah_lateral, mah_yaw,
	//           r_s, r_l, r_yaw,
	//           s_s, s_l, s_yaw;
	return mah_vec;
}

// 残差
Matrix PoseEKF::residual(const Matrix& a, const Matrix& b) {
	Matrix y = a - b;
	y(2, 0) = localization_module::common::NumericalProcess::unify_angle(y(2, 0));
	return y;
}

bool PoseEKF::input_check() {
	if (P_.rows() != P_.cols() || P_.rows() != status_num_ || R_.rows() != R_.cols() || R_.rows() != measure_num_ ||
		M_.rows() != M_.cols() || M_.rows() != input_num_ || dt_ <= 0) {
		return false;
	}
	return true;
}

void PoseEKF::ekf_matrix_init() {
	measure_ = Matrix::Zero(measure_num_, 1);
	status_ = Matrix::Zero(status_num_, 1);
	status_prior_ = Matrix::Zero(status_num_, 1);
	status_post_ = Matrix::Zero(status_num_, 1);
	status_pre_ = Matrix::Zero(status_num_, 1);
	P_ = Matrix::Zero(status_num_, status_num_);
	P_prior_ = Matrix::Zero(status_num_, status_num_);
	P_post_ = Matrix::Zero(status_num_, status_num_);
	fj_ = Matrix::Identity(status_num_, status_num_);
	vj_ = Matrix::Zero(status_num_, input_num_);
	M_ = Matrix::Zero(input_num_, input_num_);
	H_ = Matrix::Identity(status_num_, measure_num_);
	K_ = Matrix::Zero(status_num_, measure_num_);
	ekf_eye_n_.setIdentity(status_num_, status_num_);
}

void PoseEKF::set_dimension(const int& status_num, const int& input_num, const int& measure_num) {
	status_num_ = status_num;
	input_num_ = input_num;
	measure_num_ = measure_num;
}

void PoseEKF::set_dt(const double& dt) { dt_ = dt; }

void PoseEKF::set_status(const Matrix& status) { status_ = status * 1.0; }

void PoseEKF::set_covariance(const Matrix& status_cov, const Matrix& input_cov, const Matrix& measure_cov) {
	P_ = status_cov * 1.0;
	M_ = input_cov * 1.0;
	R_ = measure_cov * 1.0;
}

Matrix PoseEKF::get_status_estimated() { return status_post_; }

} // namespace localization_module

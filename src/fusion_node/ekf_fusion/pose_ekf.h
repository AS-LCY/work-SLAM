
#ifndef FLBOT_LOCALIZTION_POSE_EKF_H
#define FLBOT_LOCALIZTION_POSE_EKF_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include "../fusion_param.hpp"

namespace localization_module {

/// @brief The alias for Eigen::Matrix Type, the same as Eigen::MatrixXd
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Matrix;

/// @brief vehicle ekf based on bicycle motion model
class PoseEKF {
   public:
	PoseEKF();

	PoseEKF(const int& status_num, const int& input_num, const int& measure_num, const Matrix& status_cov,
			const Matrix& input_cov, const Matrix& measure_cov, const double& dt, const EkfGatingParams& gating_params);

	~PoseEKF();

	/// @brief get estimated status for model every loop
	Matrix get_status_estimated();

	void reset(const Matrix& status_cov, const Matrix& input_cov, const Matrix& measure_cov);

	void set_dimension(const int& status_num, const int& input_num, const int& measure_num);

	void set_dt(const double& dt);

	void set_covariance(const Matrix& status_cov, const Matrix& input_cov, const Matrix& measure_cov);

	// void params_initialize();

	///@brief Set the status object, for initialize status
	///@param status
	void set_status(const Matrix& status);

	///@brief motion model
	///@param u command input [speed, w]
	///@param dt dt
	void move(const Matrix& u, const double& dt);

	void predict(const Matrix& u, const double& dt);
	void update(const Matrix& measure, bool trust_measure);

   private:
	/// @brief matrixs of extended kalman filter initialization
	void ekf_matrix_init();

	bool input_check();

	bool ekf_matrix_check();

	Matrix residual(const Matrix& a, const Matrix& b);
	Matrix get_mah_vec(const Matrix& res, const Matrix& measure);

   private:
	int	   status_num_;	 // status num
	int	   measure_num_; // measure num
	int	   input_num_;	 // input num
	double dt_;			 // period of extended kalman filter

	// params for motion model
	double wheelbase_;
	double lr_; // distance from mass center to rear bump

	Eigen::MatrixXd ekf_eye_n_; // unit matrix with dimension [status_num * status_num]

	Matrix P_;		 // status cov
	Matrix M_;		 // input cov, Q, 手动输入一次, 滤波过程中不变, [v, w]
	Matrix R_;		 // measure cov, 手动输入一次, 滤波过程中不变, []
	Matrix status_;	 // status  [x, y, theta]
	Matrix measure_; // measure [x, y, theta]
	Matrix P_prior_;
	Matrix P_post_;
	Matrix status_prior_;
	Matrix status_post_;
	Matrix status_pre_;
	Matrix fj_; // f jacobian, f 对 status [x, y, theta]  的协方差矩阵
	Matrix vj_; // v jacobian, f 对 input [v, w] 的协方差矩阵
	Matrix H_;	// h jacobian; 在这里是单位阵, 也就是观测量直接就是状态量,
	Matrix S_;
	// Matrix SI_; // not in use
	Matrix K_;

	double			lateral_update_max_ = 0.10;
	double			lateral_update_min_ = 0.01;
	EkfGatingParams gating_params_;
	bool			is_turn_ = false;
};

} // namespace localization_module

#endif // FLBOT_LOCALIZTION_POSE_EKF_H

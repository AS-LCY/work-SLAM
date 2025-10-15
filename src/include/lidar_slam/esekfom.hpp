#ifndef ESEKFOM_EKF_HPP
#define ESEKFOM_EKF_HPP

// 以下 use-ikfom.hpp 中均已包含
// #include <vector>
// #include <cstdlib>
// #include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <Eigen/Dense>
// #include <Eigen/Sparse>
#include <logTracer/tracer.h>
#include <omp.h>

#include "lidar_slam/ikd_Tree.h"
#include "lidar_slam/use-ikfom.hpp"

//该hpp主要包含：广义加减法，前向传播主函数，计算特征点残差及其雅可比，ESKF主函数

const double epsi = 0.001; // ESKF迭代时，如果dx<epsi 认为收敛
// const double epsi = 0.0005; // ESKF迭代时，如果dx<epsi 认为收敛

namespace esekfom {
using namespace Eigen;

static int guess_points_num = 4000; // raw value: 100000
static PointCloudType::Ptr normvec(
	new PointCloudType(guess_points_num, 1)); //特征点在地图中对应的平面参数(平面的单位法向量,以及当前点到平面距离)
static PointCloudType::Ptr laserCloudOri(new PointCloudType(guess_points_num, 1)); //有效特征点
static PointCloudType::Ptr corr_normvect(new PointCloudType(guess_points_num, 1)); //有效特征点对应点法相量
static std::vector<bool> point_selected_surf{ guess_points_num, false };		   //判断是否是有效特征点

struct dyn_share_datastruct {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	bool valid;												   //有效特征点数量是否满足要求
	bool converge;											   //迭代时，是否已经收敛
	Eigen::Matrix<double, Eigen::Dynamic, 1> h;				   //残差	(公式(14)中的z)
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> h_x; //雅可比矩阵H (公式(14)中的H)
};

class esekf {
	DECL_CLASSNAME(esekf)
   public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Matrix<double, 24, 24> cov;				// 24X24的协方差矩阵
	typedef Matrix<double, 24, 1> vectorized_state; // 24X1的向量

	esekf() {}
	esekf(const Eigen::Isometry3d& T_imu_baselink, const double& gyr_cov, const double& wheel_cov,
		  const double& nhc_y_cov, const double& nhc_z_cov) {
		R_imu_baselink_ = T_imu_baselink.linear();
		t_imu_baselink_ = T_imu_baselink.translation();
		gyr_cov_ = gyr_cov;
		wheel_cov_ = wheel_cov;
		nhc_y_cov_ = nhc_y_cov;
		nhc_z_cov_ = nhc_z_cov;
	}
	~esekf() {}

	state_ikfom get_x() { return x_; }

	cov get_P() { return P_; }

	const input_ikfom& get_input() const { return in_; }

	void change_x(state_ikfom& input_state) { x_ = input_state; }

	void change_P(cov& input_cov) { P_ = input_cov; }

	//广义加法  公式(4)
	state_ikfom boxplus(state_ikfom x, Eigen::Matrix<double, 24, 1> f_) {
		state_ikfom x_r;
		x_r.pos = x.pos + f_.block<3, 1>(0, 0);

		x_r.rot = x.rot * Sophus::SO3d::exp(f_.block<3, 1>(3, 0));
		x_r.offset_R_L_I = x.offset_R_L_I * Sophus::SO3d::exp(f_.block<3, 1>(6, 0));
		// x_r.rot = x.rot * Sophus::SO3::exp(f_.block<3, 1>(3, 0));
		// x_r.offset_R_L_I = x.offset_R_L_I * Sophus::SO3::exp(f_.block<3, 1>(6, 0));

		x_r.offset_T_L_I = x.offset_T_L_I + f_.block<3, 1>(9, 0);
		x_r.vel = x.vel + f_.block<3, 1>(12, 0);
		x_r.bg = x.bg + f_.block<3, 1>(15, 0);
		x_r.ba = x.ba + f_.block<3, 1>(18, 0);
		x_r.grav = x.grav + f_.block<3, 1>(21, 0);

		return x_r;
	}

	//前向传播  公式(4-8)
	void predict(double& dt, Eigen::Matrix<double, 12, 12>& Q, const input_ikfom& i_in) {
		in_ = i_in;
		Eigen::Matrix<double, 24, 1> f_ = get_f(x_, i_in);	  //公式(3)的f
		Eigen::Matrix<double, 24, 24> f_x_ = df_dx(x_, i_in); //公式(7)的df/dx
		Eigen::Matrix<double, 24, 12> f_w_ = df_dw(x_, i_in); //公式(7)的df/dw

		x_ = boxplus(x_, f_ * dt); //前向传播 公式(4)

		f_x_ = Matrix<double, 24, 24>::Identity() + f_x_ * dt; //之前Fx矩阵里的项没加单位阵，没乘dt   这里补上

		P_ = (f_x_)*P_ * (f_x_).transpose() + (dt * f_w_) * Q * (dt * f_w_).transpose(); //传播协方差矩阵，即公式(8)
	}

	void h_share_model_wheel_odom(dyn_share_datastruct& ekfom_data, const double& baselink_linear_vel,
								  const Eigen::Vector3d& meas_imu_gyro, double wheel_cov_noise) {
		ekfom_data.h_x = MatrixXd::Zero(3, 24);
		ekfom_data.h.resize(3);

		M3D angv_crossmat;
		V3D gyr = meas_imu_gyro;
		V3D gyr_vec(gyr[0] - x_.bg(0), gyr[1] - x_.bg(1), gyr[2] - x_.bg(2)); // TODO(jxl): 要减去bias吗？
		angv_crossmat << SKEW_SYM_MATRX(gyr_vec);

		V3D wheel_v_vec(baselink_linear_vel, 0.0, 0.0);
		// V3D baselink_v_est =
		// 	R_imu_baselink_.transpose() * (x_.rot.matrix().transpose() * x_.vel + angv_crossmat * t_imu_baselink_);

		TRACE_INFO("imu world vel = %f, %f, %f", x_.vel.x(), x_.vel.y(), x_.vel.z());
		TRACE_INFO("mea imu gyro(deg) = %f, %f, %f", meas_imu_gyro.x() * RAD2DEGREE, meas_imu_gyro.y() * RAD2DEGREE,
				   meas_imu_gyro.z() * RAD2DEGREE);
		TRACE_INFO("bg bias(deg) = %f, %f, %f", x_.bg.x() * RAD2DEGREE, x_.bg.y() * RAD2DEGREE, x_.bg.z() * RAD2DEGREE);
		TRACE_INFO("imu gyro unbiased(deg) = %f, %f, %f", gyr_vec.x() * RAD2DEGREE, gyr_vec.y() * RAD2DEGREE,
				   gyr_vec.z() * RAD2DEGREE);
		TRACE_INFO("t_imu_baselink = %f, %f, %f", t_imu_baselink_.x(), t_imu_baselink_.y(), t_imu_baselink_.z());
		TRACE_INFO("imu_world_R is SO(3) = %d", isSO3(x_.rot.matrix()));
		TRACE_INFO("R_imu_baselink is SO(3) = %d", isSO3(R_imu_baselink_));
		const Eigen::Vector3d without_lever_arm_vel =
			R_imu_baselink_.transpose() * x_.rot.matrix().transpose() * x_.vel;
		const Eigen::Vector3d lever_arm_vel = R_imu_baselink_.transpose() * angv_crossmat * t_imu_baselink_;
		const Eigen::Vector3d baselink_v_est = without_lever_arm_vel + lever_arm_vel;
		TRACE_INFO("without_lever_arm_vel= %f, %f, %f,  norm : %f", without_lever_arm_vel.x(),
				   without_lever_arm_vel.y(), without_lever_arm_vel.z(), without_lever_arm_vel.norm());
		TRACE_INFO("lever arm vel= %f, %f, %f,  norm : %f", lever_arm_vel.x(), lever_arm_vel.y(), lever_arm_vel.z(),
				   lever_arm_vel.norm());
		TRACE_INFO_CLASS("estimate baselink vel = %f, %f, %f", baselink_v_est.x(), baselink_v_est.y(),
						 baselink_v_est.z());
		V3D res = wheel_v_vec - baselink_v_est; // 残差 = 测量值 - 估计值
		TRACE_INFO_CLASS("meas vel = %f, wheel odom residual: %f, %f, %f", baselink_linear_vel, res.x(), res.y(),
						 res.z());

		// covariance
		M3D bg_crossmat;
		bg_crossmat << SKEW_SYM_MATRX(t_imu_baselink_);
		Eigen::Matrix3d tmp_mat = R_imu_baselink_.transpose() * bg_crossmat;
		Eigen::Matrix3d cov_mat = Eigen::Matrix3d::Identity();
		cov_mat(0, 0) = wheel_cov_;
		if (gyr_vec.norm() > 0.3) {
			cov_mat(1, 1) = baselink_linear_vel * gyr_vec.norm();
		} else {
			cov_mat(1, 1) = nhc_y_cov_;
		}
		cov_mat(2, 2) = nhc_z_cov_;
		cov_mat = (cov_mat + tmp_mat * tmp_mat.transpose() * gyr_cov_).eval();
		wheel_cov_noise = cov_mat.diagonal().maxCoeff();
		// TRACE_INFO_CLASS("wheel odom cov noise: %f", wheel_cov_noise);

		double use_weight = false;
		Eigen::Matrix3d weight_matrix = Eigen::Matrix3d::Identity();
		Eigen::LLT<Eigen::Matrix3d> llt(cov_mat);
		if (llt.info() == Eigen::Success && use_weight) {
			weight_matrix = llt.matrixU(); // L^T
		}
		res = (weight_matrix * res).eval();

		// residual
		ekfom_data.h(0) = res.x();
		ekfom_data.h(1) = res.y();
		ekfom_data.h(2) = res.z();

		// jacobian
		M3D rot_crossmat;
		V3D tmp_vel = x_.rot.matrix().transpose() * x_.vel;
		rot_crossmat << SKEW_SYM_MATRX(tmp_vel); // 当前状态imu系下 点坐标反对称矩阵
		ekfom_data.h_x.block<3, 3>(0, 3) = -weight_matrix * R_imu_baselink_.transpose() * rot_crossmat; // J_rot

		ekfom_data.h_x.block<3, 3>(0, 12) =
			-weight_matrix * R_imu_baselink_.transpose() * x_.rot.matrix().transpose(); // J_vel

		// TODO(jxl): 是否使能计算对bg的雅可比
		ekfom_data.h_x.block<3, 3>(0, 15) = -weight_matrix * R_imu_baselink_.transpose() * bg_crossmat; // J_bg

		return;
	}

	//计算每个特征点的残差及H矩阵
	void h_share_model(dyn_share_datastruct& ekfom_data, PointCloudType::Ptr& feats_down_body,
					   KD_TREE<PointType>& ikdtree, vector<PointVector>& Nearest_Points, bool extrinsic_est) {
		int feats_down_size = feats_down_body->points.size();
		laserCloudOri->clear();
		corr_normvect->clear();
		laserCloudOri->resize(feats_down_size);
		corr_normvect->resize(feats_down_size);
		point_selected_surf.resize(feats_down_size, false);

		double t1 = omp_get_wtime();
		omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
		for (int i = 0; i < feats_down_size; i++) //遍历所有的特征点
		{
			PointType& point_body = feats_down_body->points[i];
			PointType point_world;

			V3D p_body(point_body.x, point_body.y, point_body.z);

			//把Lidar坐标系的点先转到IMU坐标系，再根据前向传播估计的位姿x，转到世界坐标系
			V3D p_global(x_.rot * (x_.offset_R_L_I * p_body + x_.offset_T_L_I) + x_.pos);
			point_world.x = p_global(0);
			point_world.y = p_global(1);
			point_world.z = p_global(2);
			point_world.intensity = point_body.intensity;

			vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
			auto& points_near =
				Nearest_Points[i]; // Nearest_Points[i]打印出来发现是按照离point_world距离，从小到大的顺序的vector

			double ta = omp_get_wtime();
			if (ekfom_data.converge) {
				//寻找point_world的最近邻的平面点
				ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
				//判断是否是有效匹配点，与loam系列类似，要求特征点最近邻的地图点数量>阈值，距离<阈值
				//满足条件的才置为true   zx 5m ?
				point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS		  ? false
										 : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
																					  : true;
			}
			if (!point_selected_surf[i]) continue; //如果该点不满足条件  不进行下面步骤

			Matrix<float, 4, 1> pabcd;		//平面点信息
			point_selected_surf[i] = false; //将该点设置为无效点，用来判断是否满足条件
			//拟合平面方程ax+by+cz+d=0并求解点到平面距离
			if (esti_plane(pabcd, points_near, 0.1f)) {
				float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z +
							pabcd(3); //当前点到平面的距离
				float s =
					1 - 0.9 * fabs(pd2) / sqrt(p_body.norm()); //如果残差大于经验阈值，则认为该点是有效点
															   //简言之，距离原点越近的lidar点  要求点到平面的距离越苛刻

				if (s > 0.9) //如果残差大于阈值，则认为该点是有效点
				{
					point_selected_surf[i] = true;
					normvec->points[i].x = pabcd(0); //存储平面的单位法向量  以及当前点到平面距离
					normvec->points[i].y = pabcd(1);
					normvec->points[i].z = pabcd(2);
					normvec->points[i].intensity = pd2;
				}
			}
		}
		double t2 = omp_get_wtime();
		int effct_feat_num = 0; //有效特征点的数量
		for (int i = 0; i < feats_down_size; i++) {
			if (point_selected_surf[i]) //对于满足要求的点
			{
				laserCloudOri->points[effct_feat_num] = feats_down_body->points[i]; //把这些点重新存到laserCloudOri中
				corr_normvect->points[effct_feat_num] = normvec->points[i]; //存储这些点对应的法向量和到平面的距离
				effct_feat_num++;
			}
		}

		if (effct_feat_num < 1) {
			ekfom_data.valid = false;
			// ROS_WARN_STREAM(YELLOW<<"No Effective Points!"<<RESET);
			return;
		}

		// 雅可比矩阵H和残差向量的计算
		ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);
		ekfom_data.h.resize(effct_feat_num);

		for (int i = 0; i < effct_feat_num; i++) {
			V3D point_(laserCloudOri->points[i].x, laserCloudOri->points[i].y, laserCloudOri->points[i].z);
			M3D point_crossmat;
			point_crossmat << SKEW_SYM_MATRX(point_);
			V3D point_I_ = x_.offset_R_L_I * point_ + x_.offset_T_L_I;
			M3D point_I_crossmat;
			point_I_crossmat << SKEW_SYM_MATRX(point_I_);

			// 得到对应的平面的法向量
			const PointType& norm_p = corr_normvect->points[i];
			V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

			// 计算雅可比矩阵H
			V3D C(x_.rot.matrix().transpose() * norm_vec);
			V3D A(point_I_crossmat * C);
			if (extrinsic_est) {
				V3D B(point_crossmat * x_.offset_R_L_I.matrix().transpose() * C);
				ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B),
					VEC_FROM_ARRAY(C);
			} else {
				ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0,
					0.0, 0.0, 0.0; // zx 公式有点不一样
			}

			//残差：点面距离
			ekfom_data.h(i) = -norm_p.intensity;
		}
	}

	//广义减法
	vectorized_state boxminus(state_ikfom x1, state_ikfom x2) {
		vectorized_state x_r = vectorized_state::Zero();

		x_r.block<3, 1>(0, 0) = x1.pos - x2.pos;

		x_r.block<3, 1>(3, 0) = Sophus::SO3d(x2.rot.matrix().transpose() * x1.rot.matrix()).log();
		x_r.block<3, 1>(6, 0) = Sophus::SO3d(x2.offset_R_L_I.matrix().transpose() * x1.offset_R_L_I.matrix()).log();

		// x_r.block<3, 1>(3, 0) = Sophus::SO3(x2.rot.matrix().transpose() * x1.rot.matrix()).log();
		// x_r.block<3, 1>(6, 0) = Sophus::SO3(x2.offset_R_L_I.matrix().transpose() * x1.offset_R_L_I.matrix()).log();

		x_r.block<3, 1>(9, 0) = x1.offset_T_L_I - x2.offset_T_L_I;
		x_r.block<3, 1>(12, 0) = x1.vel - x2.vel;
		x_r.block<3, 1>(15, 0) = x1.bg - x2.bg;
		x_r.block<3, 1>(18, 0) = x1.ba - x2.ba;
		x_r.block<3, 1>(21, 0) = x1.grav - x2.grav;

		return x_r;
	}

	// ESKF
	void update_iterated_dyn_share_modified(double R, PointCloudType::Ptr& feats_down_body, KD_TREE<PointType>& ikdtree,
											vector<PointVector>& Nearest_Points, int maximum_iter, bool extrinsic_est) {
		normvec->resize(int(feats_down_body->points.size()));

		dyn_share_datastruct dyn_share;
		dyn_share.valid = true;
		dyn_share.converge = true;
		int t = 0;
		state_ikfom x_propagated = x_;
		//这里的x_和P_分别是经过正向传播后的状态量和协方差矩阵，因为会先调用predict函数再调用这个函数
		cov P_propagated = P_;

		vectorized_state dx_new = vectorized_state::Zero(); // 24X1的向量

		for (int i = -1; i < maximum_iter; i++) // maximum_iter是卡尔曼滤波的最大迭代次数
		{
			dyn_share.valid = true;
			// 计算雅克比，也就是点面残差的导数 H(代码里是h_x)

			double t_update_0 = omp_get_wtime();
			h_share_model(dyn_share, feats_down_body, ikdtree, Nearest_Points, extrinsic_est);

			if (!dyn_share.valid) {
				continue;
			}
			vectorized_state dx;
			double t_update_1 = omp_get_wtime(); // jacob cal

			dx_new = boxminus(x_, x_propagated); //公式(18)中的 x^k - x^
			double t_update_2 = omp_get_wtime(); // x^k - x^

			//由于H矩阵是稀疏的，只有前12列有非零元素，后12列是零 因此这里采用分块矩阵的形式计算 减少计算量
			auto H = dyn_share.h_x.eval();										// m X 12 的矩阵
			Eigen::Matrix<double, 24, 24> HTH = Matrix<double, 24, 24>::Zero(); //矩阵 H^T * H
			double t_update_3 = omp_get_wtime();								// H^T * H

			HTH.block<12, 12>(0, 0) = H.transpose() * H;

			Eigen::Matrix<double, 24, 24> K_front = (HTH / R + P_.inverse()).inverse();
			Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K;
			K = K_front.block<24, 12>(0, 0) * H.transpose() / R; //卡尔曼增益  这里R视为常数
			double t_update_4 = omp_get_wtime();				 //计算卡尔曼增益

			Eigen::Matrix<double, 24, 24> KH = Matrix<double, 24, 24>::Zero(); //矩阵 K * H
			KH.block<24, 12>(0, 0) = K * H;
			double t_update_5 = omp_get_wtime(); //计算矩阵 K * H

			Matrix<double, 24, 1> dx_ =
				K * dyn_share.h + (KH - Matrix<double, 24, 24>::Identity()) * dx_new; //公式(18)  J 是 I

			x_ = boxplus(x_, dx_); //公式(18)

			dyn_share.converge = true;
			for (int j = 0; j < 24; j++) {
				if (std::fabs(dx_[j]) > epsi) //如果dx>epsi 认为没有收敛
				{
					dyn_share.converge = false;
					break;
				}
			}

			double t_update_6 = omp_get_wtime(); //计算矩阵 K * H
			if (dyn_share.converge) t++;

			if (!t && i == maximum_iter - 2) //如果迭代了3次还没收敛 强制令成true， h_share_model 函数中会重新寻找近邻点
			{
				dyn_share.converge = true;
			}
			double t_update_7 = omp_get_wtime(); //计算矩阵 K * H
			// ROS_INFO_STREAM("iter: "<< i << " -----------------------------------------" << RESET);
			// ROS_INFO_STREAM("------ 计算雅克比矩阵, time cost    : " << (t_update_1-t_update_0)*1000 << " ms" );
			// ROS_INFO_STREAM("------ 计算矩阵 x^k - x^, time cost: " << (t_update_2-t_update_1)*1000 << " ms" );
			// ROS_INFO_STREAM("------ 计算矩阵 H^T * H, time cost : " << (t_update_3-t_update_2)*1000 << " ms" );
			// ROS_INFO_STREAM("------ 计算卡尔曼增益, time cost    : " << (t_update_4-t_update_3)*1000 << " ms" );
			// ROS_INFO_STREAM("------ 计算 K * H, time cost       : " << (t_update_5-t_update_4)*1000 << " ms" );
			// ROS_INFO_STREAM("------ boxplus, time cost         : " << (t_update_6-t_update_5)*1000 << " ms" );
			// ROS_INFO_STREAM("------ end  , time cost           : " << (t_update_7-t_update_0)*1000 << " ms" );
			// ROS_INFO_STREAM("iter: "<< i << " -----------------------------------------" << RESET);

			if (t > 1 || i == maximum_iter - 1) {
				P_ = (Matrix<double, 24, 24>::Identity() - KH) * P_; //公式(19)
				return;
			}
		}
	}

	void update_iterated_dyn_share_wheel_odom(const double& baselink_linear_vel, const Eigen::Vector3d& meas_imu_gyro,
											  int maximum_iter = 0) {
		dyn_share_datastruct dyn_share;
		dyn_share.valid = true;
		dyn_share.converge = true;

		state_ikfom x_propagated = x_;
		//这里的x_和P_分别是经过正向传播后的状态量和协方差矩阵，因为会先调用predict函数再调用这个函数
		cov P_propagated = P_;

		vectorized_state dx_new = vectorized_state::Zero(); // 24X1的向量

		for (int i = -1; i < maximum_iter; i++) // maximum_iter是卡尔曼滤波的最大迭代次数
		{
			dyn_share.valid = true;
			double wheel_cov_noise = 0.1;
			h_share_model_wheel_odom(dyn_share, baselink_linear_vel, meas_imu_gyro, wheel_cov_noise);
			if (!dyn_share.valid) {
				continue;
			}
			vectorized_state dx;
			dx_new = boxminus(x_, x_propagated); //公式(18)中的 x^k - x^

			// //由于H矩阵是稀疏的，只有前12列有非零元素，后12列是零 因此这里采用分块矩阵的形式计算 减少计算量
			// auto H = dyn_share.h_x.eval();										// m X 12 的矩阵
			// Eigen::Matrix<double, 24, 24> HTH = Matrix<double, 24, 24>::Zero(); //矩阵 H^T * H

			// HTH.block<12, 12>(0, 0) = H.transpose() * H;

			// Eigen::Matrix<double, 24, 24> K_front = (HTH / R + P_.inverse()).inverse();
			// Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K;
			// K = K_front.block<24, 12>(0, 0) * H.transpose() / R; //卡尔曼增益  这里R视为常数

			// Eigen::Matrix<double, 24, 24> KH = Matrix<double, 24, 24>::Zero(); //矩阵 K * H
			// KH.block<24, 12>(0, 0) = K * H;

			// Matrix<double, 24, 1> dx_ =
			// 	K * dyn_share.h + (KH - Matrix<double, 24, 24>::Identity()) * dx_new; //公式(18)  J 是 I

			Eigen::Matrix<double, 3, 24> H = dyn_share.h_x.eval();
			Eigen::Matrix<double, 24, 24> HTH = Eigen::Matrix<double, 24, 24>::Zero();
			HTH = H.transpose() * H;
			Eigen::Matrix<double, 24, 24> K_front = (HTH / wheel_cov_noise + P_.inverse()).inverse(); //前半部分
			Eigen::Matrix<double, 24, 3> K = K_front * H.transpose() / wheel_cov_noise;
			Eigen::Matrix<double, 24, 24> KH = Matrix<double, 24, 24>::Zero();
			KH = K * H;
			Eigen::Matrix<double, 24, 1> dx_ =
				K * dyn_share.h + (KH - Matrix<double, 24, 24>::Identity()) * dx_new; //公式(18)  J 是 I

			TRACE_INFO_CLASS("dx_p = %f, %f, %f", dx_[0], dx_[1], dx_[2]);
			TRACE_INFO_CLASS("dx_Q(deg) = %f, %f, %f", dx_[3] * RAD2DEGREE, dx_[4] * RAD2DEGREE, dx_[5] * RAD2DEGREE);
			TRACE_INFO_CLASS("dx_V = %f, %f, %f", dx_[12], dx_[13], dx_[14]);
			TRACE_INFO_CLASS("dx_bg(deg) = %f, %f, %f", dx_[15] * RAD2DEGREE, dx_[16] * RAD2DEGREE,
							 dx_[17] * RAD2DEGREE);
			TRACE_INFO_CLASS("dx_ba = %f, %f, %f", dx_[18], dx_[19], dx_[20]);
			TRACE_INFO_CLASS("dx_gw = %f, %f, %f\n\n", dx_[21], dx_[22], dx_[23]);

			x_ = boxplus(x_, dx_); //公式(18)

			dyn_share.converge = true;
			if (i == maximum_iter - 1) {
				P_ = (Matrix<double, 24, 24>::Identity() - KH) * P_; //公式(19)
				return;
			}
		}
	}

   private:
	input_ikfom in_;
	state_ikfom x_;
	cov P_ = cov::Identity();
	Eigen::Matrix3d R_imu_baselink_ = Eigen::Matrix3d::Identity();
	Eigen::Vector3d t_imu_baselink_ = Eigen::Vector3d::Zero();
	double gyr_cov_ = 0.0005;										   // std cov 1.28°
	double wheel_cov_ = 0.001, nhc_y_cov_ = 0.001, nhc_z_cov_ = 0.001; // std cov = 0.03m
};

} // namespace esekfom

#endif //  ESEKFOM_EKF_HPP

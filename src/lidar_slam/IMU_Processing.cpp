
#include "lidar_slam/IMU_Processing.hpp"

//这个hpp主要包含：IMU数据预处理：IMU初始化，IMU正向传播，反向传播补偿运动失真

ImuProcess::ImuProcess() : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1) {
	init_iter_num_ = 1;
	Q = process_noise_cov(); //调用use-ikfom.hpp里面的process_noise_cov初始化噪声协方差

	cov_acc_ = V3D(0.1, 0.1, 0.1); //加速度协方差初始化
	cov_gyr_ = V3D(0.1, 0.1, 0.1); //角速度协方差初始化
	// TODO(jxl): 初始值给的有点大

	cov_bias_gyr_ = V3D(0.0001, 0.0001, 0.0001); //角速度bias协方差初始化
	cov_bias_acc_ = V3D(0.0001, 0.0001, 0.0001); //加速度bias协方差初始化

	mean_acc_ = V3D(0, 0, -1.0); // TODO(jxl): reset应该为（0，0，0）
	mean_gyr_ = V3D(0, 0, 0);
	angvel_last_ = Vector3d(0, 0, 0);		 //上一帧角速度初始化
	Lidar_T_wrt_IMU_ = Vector3d(0, 0, 0);	 // lidar到IMU的位置外参初始化
	Lidar_R_wrt_IMU_ = Matrix3d::Identity(); // lidar到IMU的旋转外参初始化
	last_imu_.reset(new livox_ros::ImuMsg);	 //上一帧imu初始化
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() {
	mean_acc_ = V3D(0, 0, -1.0); // TODO(jxl): reset应该为（0，0，0）
	mean_gyr_ = V3D(0, 0, 0);
	angvel_last_ = Vector3d(0, 0, 0);
	imu_need_init_ = true;
	start_timestamp_ = -1; //开始时间戳
	init_iter_num_ = 1;
	IMUpose_.clear();						 // imu位姿清空
	last_imu_.reset(new livox_ros::ImuMsg);	 //上一帧imu初始化
	cur_pcl_un_.reset(new PointCloudType()); //当前帧点云未去畸变初始化
}

//传入外部参数
void ImuProcess::set_param(const V3D& transl, const M3D& rot, const V3D& gyr, const V3D& acc, const V3D& gyr_bias,
						   const V3D& acc_bias) {
	Lidar_T_wrt_IMU_ = transl;
	Lidar_R_wrt_IMU_ = rot;
	cov_gyr_scale_ = gyr;
	cov_acc_scale_ = acc;
	cov_bias_gyr_ = gyr_bias;
	cov_bias_acc_ = acc_bias;
}

void ImuProcess::IMU_init(const MeasureGroup& meas, esekfom::esekf& kf_state, int& N) {
	V3D cur_acc, cur_gyr;

	if (b_first_frame_) //如果为第一帧IMU
	{
		Reset(); //重置IMU参数
		N = 1;
		b_first_frame_ = false;
		const auto& imu_acc = meas.imu.front()->linear_acceleration; // IMU初始时刻的加速度
		const auto& gyr_acc = meas.imu.front()->angular_velocity;	 // IMU初始时刻的角速度
		mean_acc_ << imu_acc[0], imu_acc[1], imu_acc[2];			 //第一帧加速度值作为初始化均值
		mean_gyr_ << gyr_acc[0], gyr_acc[1], gyr_acc[2];			 //第一帧角速度值作为初始化均值
		first_lidar_time_ = meas.lidar_beg_time; //将当前IMU帧对应的lidar起始时间 作为初始时间
	}

	// TODO(jxl): 添加静止判断
	for (const auto& imu : meas.imu) //根据所有IMU数据，计算平均值和方差
	{
		const auto& imu_acc = imu->linear_acceleration;
		const auto& gyr_acc = imu->angular_velocity;
		cur_acc << imu_acc[0], imu_acc[1], imu_acc[2];
		cur_gyr << gyr_acc[0], gyr_acc[1], gyr_acc[2];

		mean_acc_ += (cur_acc - mean_acc_) / N; //根据当前帧和均值差作为均值的更新
		mean_gyr_ += (cur_gyr - mean_gyr_) / N;

		cov_acc_ = cov_acc_ * (N - 1.0) / N + (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) / N;
		cov_gyr_ = cov_gyr_ * (N - 1.0) / N + (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) / N / N * (N - 1);

		N++;
	}
	initial_rotate_ = g2R(mean_acc_);

	state_ikfom init_state = kf_state.get_x();				  //在esekfom.hpp获得x_的状态
	init_state.grav = -mean_acc_ / mean_acc_.norm() * G_m_s2; //得平均测量的单位方向向量 * 重力加速度预设值

	// init_state.rot = Sophus::SO3d(R0);
	// TODO(jxl): 初始角度应该要设置，比如在斜坡上启动建图, init_state.rot = Sophus::SO3d(initial_rotate_);

	init_state.bg = mean_gyr_; //静止角速度测量作为陀螺仪偏差

	init_state.offset_T_L_I = Lidar_T_wrt_IMU_;				  //将lidar和imu外参传入
	init_state.offset_R_L_I = Sophus::SO3d(Lidar_R_wrt_IMU_); // imu frame to laser frame

	kf_state.change_x(init_state); //将初始化后的状态传入esekfom.hpp中的x_

	Matrix<double, 24, 24> init_P = MatrixXd::Identity(24, 24); //在esekfom.hpp获得P_的协方差矩阵
	init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;		// delta_R
	init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;	// delta_t
	init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;	// bg
	init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;	// ba
	init_P(21, 21) = init_P(22, 22) = init_P(23, 23) = 0.00001; // g^w

	// TODO(jxl):
	//  init_P.block<3, 3>(0, 0) = 1e-5 * Eigen::Matrix3d::Identity();	 // P
	//  init_P.block<3, 3>(3, 3) = 1e-5 * Eigen::Matrix3d::Identity();	 // Q
	//  init_P.block<3, 3>(12, 12) = 1e-5 * Eigen::Matrix3d::Identity(); // V

	kf_state.change_P(init_P);
	last_imu_ = meas.imu.back();
}

//反向传播
void ImuProcess::UndistortPcl(const MeasureGroup& meas, esekfom::esekf& kf_state, PointCloudType& pcl_out) {
	/***将上一帧最后尾部的imu添加到当前帧头部的imu ***/
	auto v_imu = meas.imu;		 //取出当前帧的IMU队列
	v_imu.push_front(last_imu_); //将上一帧最后尾部的imu添加到当前帧头部的imu
	const double& imu_end_time = v_imu.back()->time_stamp; // 拿到当前帧尾部的imu的时间
	const double& pcl_beg_time = meas.lidar_beg_time;	   // 点云开始和结束的时间戳
	const double& pcl_end_time = meas.lidar_end_time;

	pcl_out = *(meas.lidar);
	sort(pcl_out.points.begin(), pcl_out.points.end(), time_list); //时间戳越大越往后

	state_ikfom imu_state = kf_state.get_x(); // 获取上一次KF估计的后验状态作为本次IMU预测的初始状态
	IMUpose_.clear();
	IMUpose_.push_back(set_pose6d(0.0, acc_last_, angvel_last_, imu_state.vel, imu_state.pos, imu_state.rot.matrix()));
	//将初始状态加入IMUpose中,包含有时间间隔，上一帧加速度，上一帧角速度，上一帧速度，上一帧位置，上一帧旋转矩阵

	V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
	// angvel_avr为平均角速度，acc_avr为平均加速度，acc_imu为imu加速度，vel_imu为imu速度，pos_imu为imu位置

	M3D R_imu; // IMU旋转矩阵，消除运动失真的时候用

	double dt = 0;
	input_ikfom in;

	// 遍历本次估计的所有IMU测量并且进行积分，离散中值法 前向传播
	for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
		auto&& head = *(it_imu);					   //拿到当前帧的imu数据
		auto&& tail = *(it_imu + 1);				   //拿到下一帧的imu数据
		if (tail->time_stamp < last_lidar_end_time_) { //判断时间先后顺序：下一帧时间戳是否小于上一帧结束时间戳
			continue;
		}

		angvel_avr << 0.5 * (head->angular_velocity[0] + tail->angular_velocity[0]), // 中值积分
			0.5 * (head->angular_velocity[1] + tail->angular_velocity[1]),
			0.5 * (head->angular_velocity[2] + tail->angular_velocity[2]);
		acc_avr << 0.5 * (head->linear_acceleration[0] + tail->linear_acceleration[0]),
			0.5 * (head->linear_acceleration[1] + tail->linear_acceleration[1]),
			0.5 * (head->linear_acceleration[2] + tail->linear_acceleration[2]);

		acc_avr = acc_avr * G_m_s2 / mean_acc_.norm(); //通过重力数值对加速度进行调整(除上初始化的IMU大小*9.8)

		//如果IMU开始时刻早于上次雷达最晚时刻(因为将上次最后一个IMU插入到此次开头了，所以会出现一次这种情况)
		if (head->time_stamp < last_lidar_end_time_) {
			dt = tail->time_stamp - last_lidar_end_time_; //从上次雷达时刻末尾开始传播 计算与此次IMU结尾之间的时间差
		} else {
			dt = tail->time_stamp - head->time_stamp; //两个IMU时刻之间的时间间隔
		}

		in.acc = acc_avr; // 两帧IMU的中值作为输入in  用于前向传播
		in.gyro = angvel_avr;
		Q.block<3, 3>(0, 0).diagonal() = cov_gyr_; // TODO(jxl): 可以直接用cov_gyr_scale_
		Q.block<3, 3>(3, 3).diagonal() = cov_acc_; // TODO(jxl): 可以直接用cov_acc_scale_
		Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
		Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc_;

		kf_state.predict(dt, Q, in); // IMU前向传播，每次传播的时间间隔为dt

		imu_state = kf_state.get_x();
		angvel_last_ =
			V3D(tail->angular_velocity[0], tail->angular_velocity[1], tail->angular_velocity[2]) - imu_state.bg;
		acc_last_ = V3D(tail->linear_acceleration[0], tail->linear_acceleration[1], tail->linear_acceleration[2]) *
					G_m_s2 / mean_acc_.norm();
		acc_last_ = imu_state.rot * (acc_last_ - imu_state.ba) + imu_state.grav; // acc_global

		double&& offs_t = tail->time_stamp - pcl_beg_time; //后一个IMU时刻距离此次雷达开始的时间间隔
		IMUpose_.push_back(
			set_pose6d(offs_t, acc_last_, angvel_last_, imu_state.vel, imu_state.pos, imu_state.rot.matrix()));
	}

	// 把最后一帧IMU测量也补上
	dt = abs(pcl_end_time - imu_end_time);
	kf_state.predict(dt, Q, in);
	imu_state = kf_state.get_x();

	last_imu_ = meas.imu.back();		 //保存最后一个IMU测量，以便于下一帧使用
	last_lidar_end_time_ = pcl_end_time; //保存这一帧最后一个雷达测量的结束时间，以便于下一帧使用
	if (pcl_out.points.begin() == pcl_out.points.end()) {
		return;
	}
	auto it_pcl = pcl_out.points.end() - 1;

	//遍历每个IMU帧
	for (auto it_kp = IMUpose_.end() - 1; it_kp != IMUpose_.begin(); it_kp--) {
		auto head = it_kp - 1;
		auto tail = it_kp;
		R_imu << MAT_FROM_ARRAY(head->rot);		 //拿到前一帧的IMU旋转矩阵
		vel_imu << VEC_FROM_ARRAY(head->vel);	 //拿到前一帧的IMU速度
		pos_imu << VEC_FROM_ARRAY(head->pos);	 //拿到前一帧的IMU位置
		acc_imu << VEC_FROM_ARRAY(tail->acc);	 //拿到后一帧的IMU加速度, // acc_global
		angvel_avr << VEC_FROM_ARRAY(tail->gyr); //拿到后一帧的IMU角速度

		// 之前点云按照时间从小到大排序过，IMUpose也同样是按照时间从小到大push进入的，此时从IMUpose的末尾开始循环，也就是从时间最大处开始，因此只需要判断
		// 点云时间需>IMU head时刻即可，不需要判断点云时间<IMU tail
		for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) {
			dt = it_pcl->curvature / double(1000) - head->offset_time; // t_point - t_imu_head

			// P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei)

			M3D R_i(R_imu * Sophus::SO3d::exp(angvel_avr * dt)
								.matrix()); //点it_pcl所在时刻的旋转：前一帧的IMU旋转矩阵 * exp(后一帧角速度*dt)
			V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z); //点所在时刻的位置(雷达坐标系下)
			V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
			//从点所在的世界位置 - 雷达end时刻，imu frame的世界位置

			V3D P_compensate = imu_state.offset_R_L_I.matrix().transpose() *
							   (imu_state.rot.matrix().transpose() *
									(R_i * (imu_state.offset_R_L_I.matrix() * P_i + imu_state.offset_T_L_I) + T_ei) -
								imu_state.offset_T_L_I);
			// 把每个lidar点转到lidar_end_time时刻时，laser_frame下
			// 滤波器predict的是状态是，每一imu时刻，imu frame在imu_0_frame(odom)下的状态

			it_pcl->x = P_compensate(0);
			it_pcl->y = P_compensate(1);
			it_pcl->z = P_compensate(2);

			if (it_pcl == pcl_out.points.begin()) {
				break;
			}
		}
	}
}

void ImuProcess::Process(const MeasureGroup& meas, esekfom::esekf& kf_state, PointCloudType::Ptr& cur_pcl_un_) {
	if (meas.imu.empty()) {
		return;
	}
	assert(meas.lidar != nullptr);

	if (imu_need_init_) {
		IMU_init(meas, kf_state, init_iter_num_); //如果开头几帧  需要初始化IMU参数

		imu_need_init_ = true;

		last_imu_ = meas.imu.back();

		state_ikfom imu_state = kf_state.get_x();

		if (init_iter_num_ > MAX_INI_COUNT) {
			cov_acc_ *= pow(G_m_s2 / mean_acc_.norm(), 2);
			imu_need_init_ = false;

			cov_acc_ = cov_acc_scale_; // 初始化用的是cov_acc，即V3D(0.1, 0.1, 0.1)，然后初始化完成后切换到cov_acc_scale
			cov_gyr_ = cov_gyr_scale_;
			printf("IMU Initial Done\n");
		}

		return;
	}

	UndistortPcl(meas, kf_state, *cur_pcl_un_);
}

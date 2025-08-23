#include "lidar_slam/lidar_slam.hpp"
namespace lidar_slam {

string print_SlamWorkMode(SlamWorkMode e) {
	switch (e) {
		CASE_STR(MAPPING);
		CASE_STR(SEC_MAPPING);
		CASE_STR(LOCALIZATION);
		default:
			break;
	}
	return "UNKNOW_SlamWorkMode!";
}

LidarSlam::LidarSlam(const std::string work_path, bool localization_mode, bool offline, bool second_mapping,
					 rclcpp::Node::SharedPtr node) {
	LidarSlam::reset(work_path, localization_mode, offline, second_mapping, node);
}

LidarSlam::LidarSlam(const LidarSlamParam yaml_param, SlamWorkMode start_mode, rclcpp::Node::SharedPtr node) {
	config_param_ = yaml_param;
	feats_down_size_thr_ = config_param_.common.feats_down_size_thr;
	LidarSlam::reset(start_mode, node);
}

void LidarSlam::reset(SlamWorkMode work_mode, rclcpp::Node::SharedPtr node) {
	reseting = true;
	log_info_manager_ = localization_module::LocalizationModuleLogInfoManager::getInstance();
	log_info_manager_->reset_log_info();
	slam_run_status_.store(0);

	sleep(1);

	// CPU_ZERO(&mask); // 初始化 CPU 亲和性集合，将其设置为零
	// CPU_SET(0, &mask); // 将线程绑定到 cpu_id 核心

	/// 激光和IMU预处理相关 *******************************************
	time_buffer.clear();  // 记录lidar时间
	lidar_buffer.clear(); //记录特征提取或间隔采样后的lidar（特征）数据
	imu_buffer.clear();
	lidar_pushed = false;
	lidar_end_time = 0;
	lidar_mean_scantime = 0.0;
	first_lidar_time = 0.0;
	scan_num = 0;
	flg_first_scan = true;
	last_timestamp_lidar = 0;
	last_timestamp_imu = -1.0;
	timediff_lidar_wrt_imu = 0.0;
	time_sync_en = false;
	timediff_set_flg = false; // 标记是否已经进行了时间补偿
	Measures = MeasureGroup();
	temp_imu_msg.clear();

	/// 点云 reset *******************************************
	UndistortCloudInOdom.reset(new PointCloudType());
	undistortCloud.reset(new PointCloudType()); // lidar 系
	FilteredUndistortCloud.reset(new PointCloudType());
	kdtreeCloud.reset(new PointCloudType());
	// ObstacleCloud.reset(new PointCloudType());
	// FilteredObstacleCloud.reset(new PointCloudType());

	/// mapping 相关 *******************************************
	unoptimized_path.clear();
	optimized_path.clear();
	T_odom_lidar = Eigen::Isometry3d::Identity();
	localization_base = Localization_base();
	current_pose = Localization_base();
	imu_file_shift = false; // TODO

	auto cloud_leaf_size = config_param_.mapping.cloud_leaf_size;
	downSizeFilterCloud.setLeafSize(cloud_leaf_size, cloud_leaf_size, cloud_leaf_size);

	auto cloud_leaf_size_test = config_param_.lidar_preproc.leafsize;
	downSizeFilterCloud_test.setLeafSize(cloud_leaf_size_test, cloud_leaf_size_test, cloud_leaf_size_test);

	auto key_frame_distance = config_param_.mapping.key_frame_distance;
	auto key_frame_angle = config_param_.mapping.key_frame_angle;
	auto loopSearchDistance = config_param_.mapping.loopSearchDistance;
	auto loopSearchTimeDiff = config_param_.mapping.loopSearchTimeDiff;
	auto loopSearchSkipKey = config_param_.mapping.loopSearchSkipKey;
	auto loopIcpScore = config_param_.mapping.loopIcpScore;

	// back_end.reset(new BackEnd(key_frame_distance, key_frame_angle, loopSearchDistance));
	back_end.reset(new BackEnd(key_frame_distance, key_frame_angle, loopSearchDistance, loopSearchTimeDiff,
							   loopSearchSkipKey, loopIcpScore));

	/// sec_mapping & localizaiton ********************************
	globalLocalizationSuccess = false;
	global_localize_count_ = 0;

	/// important objs *******************************************
	// ikdtree
	ikdtree.reset(new KD_TREE<pcl::PointXYZINormal>());
	kf = esekfom::esekf();
	// lidar & imu 预处理
	const auto blind_distance = config_param_.lidar_preproc.blind_distance;
	const auto point_filter_num = config_param_.lidar_preproc.point_filter_num;
	const auto line_count = config_param_.lidar_preproc.line_count;
	// const auto obstacle_max_range = config_param_.lidar_preproc.obstacle_max_range;
	// const auto feature_enabled = config_param_.lidar_preproc.feature_enabled;
	// p_lidar_pre.reset(new Preprocess());
	// p_lidar_pre->set(config_param_.lidar_preproc);

	// lidar reset
	lidar_pre_ptr_ = localization_module::LidarPreprocFactory::new_lidar_preproc(
		config_param_.lidar_preproc.lidar_type, node); //////////////////////////////////////

	const auto gyr_cov = config_param_.mapping.gyr_cov;
	const auto acc_cov = config_param_.mapping.acc_cov;
	const auto b_gyr_cov = config_param_.mapping.b_gyr_cov;
	const auto b_acc_cov = config_param_.mapping.b_acc_cov;
	const auto extrinT = config_param_.extrinsic.extrinT;
	const auto extrinR = config_param_.extrinsic.extrinR;
	p_imu.reset(new ImuProcess());
	p_imu->set_param(extrinT, extrinR, V3D(gyr_cov, gyr_cov, gyr_cov), V3D(acc_cov, acc_cov, acc_cov),
					 V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov), V3D(b_acc_cov, b_acc_cov, b_acc_cov));
	// 定位
	localization.reset(new Localization());
	global_localization_.reset(new GlobalLocalization());
	cloud_map_manager_.reset(new CloudMap());

	// cout << "slam reset 5"<<endl;
	// 线程相关 ************************************************
	if (thread != nullptr) {
		thread_run = false;
		thread->join();
		// thread->detach();
		// show_thread->join();
		thread_run = true;
		if (work_mode == SEC_MAPPING) {
			global_localization_thread_->join();
		}
	}

	reseting = false;
	double curr_time = rclcpp::Clock().now().seconds();
	if (work_mode == MAPPING) {
		thread.reset(new std::thread(&LidarSlam::loopClosureThread, this));
		hb_time_thread_loop_closure_.store(curr_time);
	} else if (work_mode == SEC_MAPPING) {
		// cloud_map_manager_->load_map_data(config_param_.common.map_directory);
		global_localization_thread_.reset(
			new std::thread(&LidarSlam::global_localization_for_sec_mapping_thread, this));
		thread.reset(new std::thread(&LidarSlam::sec_mapping_loopClosureThread, this));
		hb_time_thread_loop_closure_.store(curr_time);
		hb_time_thread_secmap_relocalize_.store(curr_time);
		// second_mapping_thread.reset(new std::thread(&LidarSlam::relocalizationForMappingThread, this));
	} else if (work_mode == LOCALIZATION) {
		thread.reset(new std::thread(&LidarSlam::localizationThread, this));
		hb_time_thread_localize_.store(curr_time);
	}
	// show_thread.reset(new std::thread(&LidarSlam::showThread, this));
	working_mode_ = work_mode;

	// ROS_INFO_STREAM(GREEN << "slam reset successfully" <<RESET);
}

void LidarSlam::reset(const std::string work_path, bool localization_mode, bool offline, bool second_mapping,
					  rclcpp::Node::SharedPtr node) {
	cout << "this reset func has already been disabled, please use the new one" << endl;
	// ROS_WARN_STREAM(YELLOW << "this reset func has already been disabled, please use new version" << RESET);
	return;
}

bool LidarSlam::sync_packages(MeasureGroup& meas) {
	if (lidar_buffer.empty() || imu_buffer.empty()) {
		// printf("wait lidar & imu data\n");
		// ROS_INFO_STREAM("buffer empty, sync_packages failed !, lidar size: " << lidar_buffer.size() << ", imu size: "
		// << imu_buffer.size());
		return false;
	}
	if (reseting == true) {
		cout << "reseting, sync_packages return !" << endl;
		// ROS_INFO_STREAM("reseting, sync_packages return !");
		return false;
	}
	if (lidar_pushed && omp_get_wtime() - time_buffer.front() > 1.5 * 0.1) {
		cout << "lidar lose rate: " << omp_get_wtime() - time_buffer.front() << endl;
		// ROS_WARN_STREAM(RED << "lidar lose rate: " << omp_get_wtime()-time_buffer.front() << RESET);
	}
	/*** push a lidar scan ***/
	// static PointCloudType::Ptr pcl_temp(new PointCloudType());
	if (!lidar_pushed) {
		meas.lidar = lidar_buffer.front();		   // lidar指针指向最旧的lidar数据
		meas.lidar_beg_time = time_buffer.front(); //记录最早时间

		// //更新结束时刻的时间
		// if (meas.lidar->points.size() <= 1) // time too little 时间太短，点数不足
		// {
		//     lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime; // 记录lidar结束时间为 起始时间 +
		//     单帧扫描时间 ROS_WARN_STREAM(YELLOW << "Too few input point cloud!" << RESET);
		// }
		// else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime) //最后一个点的时间
		// 小于 单帧扫描时间的一半
		// {
		//     lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime; // 记录lidar结束时间为 起始时间 +
		//     单帧扫描时间
		// }
		// else
		// {
		//     scan_num++;
		//     lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
		//     //结束时间设置为 起始时间 + 最后一个点的时间（相对） zx 排序了么？
		//     // 动态更新每帧lidar数据平均扫描时间
		//     lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) /
		//     scan_num;
		// }
		// pcl_temp = meas.lidar;// 指向同一个地址
		// sort(pcl_temp->points.begin(), pcl_temp->points.end(), time_list);  //
		// 按由小到大排列，这里curvature中存放了时间戳（在preprocess.cpp中）

		// lidar_mean_scantime = 0.1;
		// TODO: 当时间发生跳变，一帧内的时间，可能是跳变前后的，这一帧点云需要删掉
		lidar_mean_scantime = meas.lidar->points.back().curvature * 0.001;
		lidar_mean_scantime = lidar_mean_scantime < 0.1 ? 0.1 : lidar_mean_scantime;
		lidar_mean_scantime = lidar_mean_scantime > 0.15 ? 0.1 : lidar_mean_scantime;
		lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;

		if (lidar_mean_scantime > 0.11) {
			cout << "lidar scan time: " << lidar_mean_scantime << endl;
			// ROS_INFO_STREAM(RED << "lidar scan time: " << lidar_mean_scantime << RESET);
		}

		meas.lidar_end_time = lidar_end_time;

		lidar_pushed = true;
	}

	if (last_timestamp_imu < lidar_end_time) {
		cout << RED << "latest imu time < lidar time " << RESET << endl;
		cout << YELLOW << "last_timestamp_imu: " << setprecision(15) << last_timestamp_imu << RESET << endl;
		cout << YELLOW << "lidar_end_time: " << setprecision(15) << lidar_end_time << RESET << endl;

		// ROS_WARN_STREAM(RED << "latest imu time < lidar time "<< RESET);
		// ROS_WARN_STREAM(YELLOW << "last_timestamp_imu: " << setprecision(15) << last_timestamp_imu << RESET);
		// ROS_WARN_STREAM(YELLOW << "lidar_end_time: " << setprecision(15)  << lidar_end_time << RESET);
		return false;
	}
	/*** push imu data, and pop from imu buffer ***/
	double imu_time = imu_buffer.front()->time_stamp; // 最旧IMU时间
	meas.imu.clear();

	std::lock_guard<std::mutex> lk(mtx_buffer);
	while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) { //记录imu数据，imu时间小于当前帧lidar结束时间
		imu_time = imu_buffer.front()->time_stamp;
		if (imu_time > lidar_end_time) break;
		meas.imu.push_back(imu_buffer.front()); //记录当前lidar帧内的imu数据到meas.imu
		imu_buffer.pop_front();
	}
	// ROS_INFO_STREAM(BOLDYELLOW << "Measures.imu.size(): "<< meas.imu.size() << RESET);

	if (meas.imu.empty()) {
		cout << RED << "Measure.imu is empty. " << RESET << endl;
		lidar_pushed = false;
		cout << "imu_buffer.front.time: " << setprecision(15) << imu_time << endl;
		cout << "imu_buffer.back.time: " << setprecision(15) << imu_buffer.back()->time_stamp << endl;
		cout << "lidar count: " << meas.lidar->points.size() << endl;
		cout << "lidar first point: " << setprecision(5) << meas.lidar->points.front().curvature * 0.001 << endl;
		cout << "lidar end point: " << setprecision(5) << meas.lidar->points.back().curvature * 0.001 << endl;
		cout << "lidar_beg_time.time: " << setprecision(15) << meas.lidar_beg_time << endl;
		cout << "lidar_end_time.time: " << setprecision(15) << lidar_end_time << endl;
		lidar_buffer.pop_front();
		time_buffer.pop_front();
		return false;
	}
	log_info_manager_->slam_info.data[30] = meas.lidar->points.size();
	log_info_manager_->slam_info.data[31] = lidar_buffer.front()->points.size();
	lidar_buffer.pop_front();
	time_buffer.pop_front();
	// cout<<"********************* lidar pop ************"<<endl;
	lidar_pushed = false;
	return true;
}

void LidarSlam::sec_mapping_loopClosureThread() {
	cout << "\033[1;32msec_mapping_loopClosureThread \033[0m" << endl;
	// ROS_INFO_STREAM(BOLDGREEN<<"sec_mapping_loopClosureThread "<<RESET);
	const int frequency = 1; // 频率为1Hz
	const std::chrono::milliseconds period(1000 / frequency);
	while (thread_run && reseting == false) {
		hb_time_thread_loop_closure_.store(rclcpp::Clock().now().seconds());

		// std::thread::id thisId = std::this_thread::get_id();
		// std::cout << "debug: loopClosureThread   Thread ID: " << thisId << std::endl;
		auto start = std::chrono::steady_clock::now();
		// 对于二次建图，重定位成功之前，不进行回环检测
		if (!globalLocalizationSuccess) {
			// empty
		} else if (!back_end->get_loaded_key_cloud_status()) {
			cout << "\033[1;32mSetting gtsam ... \033[0m" << endl;
			// ROS_INFO_STREAM(BOLDGREEN<< "Setting gtsam ... "<<RESET);
			auto loaded_keyframe_clouds =
				cloud_map_manager_->get_loaded_keyframe_clouds(); // auto : std::vector<PointCloudType::Ptr>
			auto loaded_keyframe_poses = cloud_map_manager_->get_loaded_keyframe_poses(); // auto : std::vector<KeyPose>
			auto loaded_sc_info = cloud_map_manager_->get_load_sc_info_();				  // auto : std::vector<KeyPose>
			auto global_odom_to_map = global_localization_->get_global_odom_to_map();	  // auto : Eigen::Isometry3d
			//////// TODO bug here
			if (cloud_map_manager_->get_map_data_status()) {
				back_end->set_loaded_key_clouds(loaded_keyframe_clouds, loaded_sc_info, loaded_keyframe_poses,
												global_odom_to_map);
			}
		} else {
			// if (loop_closure_wait){
			back_end->performLoopClosure(lidar_end_time); //  回环检测
		}

		auto end = std::chrono::steady_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		if (elapsed < period) {
			std::this_thread::sleep_for(period - elapsed);
		}
	}
}

void LidarSlam::loopClosureThread() {
	const int frequency = 1; // 频率为1Hz
	const std::chrono::milliseconds period(1000 / frequency);
	while (thread_run && reseting == false) {
		hb_time_thread_loop_closure_.store(rclcpp::Clock().now().seconds());

		auto start = std::chrono::steady_clock::now();
		// if (loop_closure_wait){
		back_end->performLoopClosure(lidar_end_time); //  回环检测
		// }
		// performSCLoopClosure();
		auto end = std::chrono::steady_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

		if (elapsed < period) {
			std::this_thread::sleep_for(period - elapsed);
		}
	}
}

void LidarSlam::localizationThread() {
	const int frequency = 1.0; // 频率为1Hz
	auto period_relocal = std::chrono::milliseconds(1000);
	// const int frequency = 2.0; // 频率为2Hz
	const float period_local_sec = config_param_.localization.fgicp_peroid_sec; // 频率为2Hz
	// const int period_fgicp_ms = period_local_sec * 1000;
	// std::chrono::milliseconds period_local_ms(period_fgicp_ms);
	// const std::chrono::milliseconds period(1000 / frequency);
	const auto score_thr = config_param_.re_localization.score_thr;
	const auto global_localize_time_out_thr = config_param_.re_localization.time_out_thr;
	const int global_localize_times = global_localize_time_out_thr * frequency; // 重定位次数
	// int global_localize_count_ = 0;
	const auto odom2map_delta_thr = config_param_.localization.odom2map_delta_thr;
	const auto odom2map_delta_set = config_param_.localization.odom2map_delta_set;
	const auto use_pose_filter = config_param_.common.use_pose_filter;

	// const auto fgicp_score_thr = config_param_.localization.fgicp_score_thr;
	const auto fgicp_score_fail_thr = config_param_.localization.fgicp_score_fail_thr;
	const auto fgicp_score_low_accuracy_thr = config_param_.localization.fgicp_score_low_accuracy_thr;
	const auto fgicp_fail_count_thr = config_param_.localization.fgicp_fail_count_thr;
	const auto fgicp_low_accuracy_count_thr = config_param_.localization.fgicp_low_accuracy_count_thr;

	int gicp_fail_count = 0;
	int gicp_low_acc_count = 0;
	pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>());
	// PointCloudType::Ptr temp(new PointCloudType());
	PointCloudType::Ptr UndistortCloudInOdom_test(new PointCloudType());

	static int wait_time = 0;

	while (thread_run && reseting == false) {
		hb_time_thread_localize_.store(rclcpp::Clock().now().seconds());
		auto start = std::chrono::steady_clock::now();
		temp.reset(new pcl::PointCloud<pcl::PointXYZI>());
		// temp.reset(new PointCloudType());
		// UndistortCloudInOdom_test.reset(new PointCloudType());
		{
			std::lock_guard<std::mutex> lk(mtx_odom_cloud);
			// downSizeFilterCloud_test.setInputCloud(UndistortCloudInOdom);
			// downSizeFilterCloud_test.filter(*UndistortCloudInOdom_test);
			pcl::copyPointCloud(*(UndistortCloudInOdom), *temp);
			// pcl::copyPointCloud(*(UndistortCloudInOdom_test), *temp);
		}

		if (local_thrd_status_.load() == 2) { // 重定位失败
			cout << "global Localization failed: time out " << endl;
			// ROS_INFO("global Localization failed: time out ");
			// auto end = std::chrono::steady_clock::now();
			// auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
			// if (elapsed < period_relocal) {
			//     std::this_thread::sleep_for(period_relocal - elapsed);
			// }
			// continue;

		} else {
			if (!globalLocalizationSuccess) {
				local_thrd_status_.store(1);

				// check
				if (!getLoadMap()) {
					cout << YELLOW << "globalLocalization failed: map not ready ... " << RESET << endl;
				} else if (!temp || temp->points.size() == 0) {
					cout << YELLOW << "globalLocalization failed: cloud empty ... " << RESET << endl;
					// check end
				} else {
					cout << "point(in use) count: " << temp->points.size() << endl;
					cout << "start globalLocalization ... " << endl;

					// state.state("lost");
					PointCloudType::Ptr FilteredUndistortCloud_test(new PointCloudType());
					{
						std::lock_guard<std::mutex> lk(mtx_lidar_cloud);
						downSizeFilterCloud_test.setInputCloud(undistortCloud);
						downSizeFilterCloud_test.filter(*FilteredUndistortCloud_test);
					}

					double t0 = omp_get_wtime();
					// globalLocalizationSuccess =
					// localization->globalLocalization(undistortCloud,T_odom_lidar,p_imu->initial_rotate,score_thr);
					globalLocalizationSuccess = localization->globalLocalization(
						FilteredUndistortCloud_test, T_odom_lidar, p_imu->initial_rotate, score_thr);
					cout << "globalLocalizationSuccess: " << globalLocalizationSuccess << endl;
					double t1 = omp_get_wtime();
					cout << "global Localization cost time: " << (t1 - t0) * 1000 << " ms" << endl;

					global_localize_count_++;
				}
				if (!globalLocalizationSuccess && global_localize_count_ > global_localize_times) {
					cout << "global Localization failed: time out" << endl;
					local_thrd_status_.store(2);
				}
				if (globalLocalizationSuccess) {
					cout << BOLDGREEN << " ======= global Localization Success ======= " << RESET << endl;
					global_localize_count_ = 0;
					local_thrd_status_.store(3);

					need_localize_ = false;
					wait_time++;
				}
				auto end = std::chrono::steady_clock::now();
				auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
				if (elapsed < period_relocal) {
					std::this_thread::sleep_for(period_relocal - elapsed);
				}
				continue;
			} else {
				if (wait_time >= period_local_sec) {
					need_localize_ = true;
					wait_time = 0;
				}

				if (need_localize_) {
					cout << "localizing ... " << endl;
					double fit_score = 0.0; // gicp_fit_score
					std::cout << "temp :: " << temp->size() << "   x:" << temp->points[100].x << std::endl;
					if (localization->localize(temp, fit_score, fgicp_score_fail_thr, fgicp_score_low_accuracy_thr,
											   odom2map_delta_thr, odom2map_delta_set, use_pose_filter)) {
						std::cout << "fit_score :" << fit_score << std::endl;

						// ROS_INFO_STREAM("fit_score: " << fit_score);
						log_info_manager_->slam_info.data[3] = 1; // if converge
						if (fit_score < fgicp_score_low_accuracy_thr) {
							local_thrd_status_.store(3);
							gicp_fail_count = 0;
							gicp_low_acc_count = 0;

							need_localize_ = false;
							wait_time++;
							cout << "need_localize_ set to false, wait for next" << endl;

						} else if (fit_score < fgicp_score_fail_thr) {
							gicp_low_acc_count++;
							cout << YELLOW << "fast gicp low accuracy count: " << gicp_low_acc_count << RESET << endl;
						} else {
							gicp_fail_count++;
							gicp_low_acc_count++;
							cout << RED << "fast gicp fail count: " << gicp_fail_count << RESET << endl;
							cout << YELLOW << "fast gicp low accuracy count: " << gicp_low_acc_count << RESET << endl;
						}
					} else {									  // 未收敛
						log_info_manager_->slam_info.data[3] = 0; // if converge
						// ROS_INFO_STREAM("fit_score: " << fit_score);
						gicp_fail_count++;
						gicp_low_acc_count++;
						cout << RED << "fast gicp fail count: " << gicp_fail_count << RESET << endl;
						cout << YELLOW << "fast gicp low accuracy count: " << gicp_low_acc_count << RESET << endl;
					}

					if (gicp_fail_count >= fgicp_fail_count_thr ||
						gicp_low_acc_count >= fgicp_low_accuracy_count_thr) { // 连续多帧 fast-gicp 失败，则认为定位失败
						local_thrd_status_.store(5);
					} else if (gicp_fail_count >= 1 || gicp_low_acc_count >= 2) { //
						local_thrd_status_.store(4);
					}

					Eigen::Isometry3d curr_lidar_in_map = getLidarInMap();
					Eigen::Isometry3d curr_odom_to_map = getOdomToMap();
					// Eigen::Isometry3d lidar_in_map_inv = curr_lidar_in_map.inverse();
					// Eigen::Isometry3d curr_odom_to_map_baselink = lidar_in_map_inv * curr_odom_to_map;
					// log_info_manager_->slam_info.data[17] = curr_odom_to_map_baselink.translation().x();
					// log_info_manager_->slam_info.data[18] = curr_odom_to_map_baselink.translation().y();
					log_info_manager_->slam_info.data[17] = curr_odom_to_map.translation().x();
					log_info_manager_->slam_info.data[18] = curr_odom_to_map.translation().y();

					log_info_manager_->slam_info.data[4] = fit_score;
					log_info_manager_->slam_info.data[5] = gicp_fail_count;
					log_info_manager_->slam_info.data[6] = gicp_low_acc_count;

					// state.state("normal");

				} else {
					wait_time++;
				}
			}
			// state_pub_->publish(&state);
		}

		auto end = std::chrono::steady_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

		// ROS_INFO_STREAM("elapsed: " << elapsed.count() << " ms");
		if (elapsed < period_relocal) {
			std::this_thread::sleep_for(period_relocal - elapsed);
		}
	}
}

void LidarSlam::global_localization_for_sec_mapping_thread() {
	// const int frequency = 1.0; // 频率为1Hz
	const int frequency = 1.0; // 频率为2Hz
	const std::chrono::milliseconds period(1000 / frequency);
	const auto score_thr = config_param_.re_localization.score_thr;
	const auto global_localize_time_out_thr = config_param_.re_localization.time_out_thr;
	const int global_localize_times = global_localize_time_out_thr * frequency; // 重定位次数
	int global_localize_count = 0;

	while (thread_run && reseting == false) {
		hb_time_thread_secmap_relocalize_.store(rclcpp::Clock().now().seconds());

		// std::thread::id thisId = std::this_thread::get_id();
		// std::cout << "debug: global_localization Thread ID: " << thisId << std::endl;
		auto start = std::chrono::steady_clock::now();
		// if(second_mapping_need_global_localization_){
		// }

		// if(m_status_ == M_RELOCALIZE_FAILED){
		if (secmap_relocal_thrd_status_.load() == 2) { // 重定位失败
			std::cout << RED << "secmap relocalization failed: time out " << RESET << endl;

		} else {
			if (!globalLocalizationSuccess) {
				secmap_relocal_thrd_status_.store(1); //重定位中
				if (!cloud_map_manager_->get_map_data_status()) {
					cout << YELLOW << "secmap relocalizing: map not ready ... " << RESET << endl;
					auto end = std::chrono::steady_clock::now();
					auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
					if (elapsed < period) {
						std::this_thread::sleep_for(period - elapsed);
					}
					continue;
				}

				// check
				if (!global_localization_->get_global_map_ready()) {
					if (!global_localization_->set_global_map(cloud_map_manager_->get_loaded_cloud_map())) {
						cout << YELLOW << "secmap relocalizing: map not ready (global-map) ... " << RESET << endl;
					} else {
						cout << GREEN << "set_global_map ready " << RESET << endl;
					}
				}
				if (!global_localization_->get_sc_manager_ready()) {
					if (!global_localization_->fill_sc_manager(cloud_map_manager_->get_load_sc_info_())) {
						cout << "secmap relocalizing: map not ready (sc-manager) ... " << endl;
					} else {
						cout << "secmap relocalizing: fill_sc_manager ready " << endl;
					}
				}

				if (!UndistortCloudInOdom || UndistortCloudInOdom->points.size() == 0) {
					cout << YELLOW << "secmap relocalizing: cloud empty ... " << RESET << endl;
					// check end
				} else {
					cout << "point(in use) count: " << UndistortCloudInOdom->points.size() << endl;
					// pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>());
					PointCloudType::Ptr temp(new PointCloudType());
					{
						std::lock_guard<std::mutex> lk(mtx_odom_cloud);
						pcl::copyPointCloud(*(UndistortCloudInOdom), *temp);
					}

					// state.state("lost");
					mutex mtx_lidar_cloud;
					globalLocalizationSuccess = global_localization_->global_localize(undistortCloud, T_odom_lidar,
																					  p_imu->initial_rotate, score_thr);
					// globalLocalizationSuccess =
					// localization->globalLocalization(undistortCloud,T_odom_lidar,p_imu->initial_rotate, score_thr);

					cout << "globalLocalizationSuccess: " << globalLocalizationSuccess << endl;
					cout << "global_localize_times_count: " << global_localize_count << endl;
					if (globalLocalizationSuccess) {
						// m_status_ = M_STANDBY;
						cout << BOLDGREEN << " ======= global Localization Success ======= " << RESET << endl;
						global_localize_count_ = 0;
						secmap_relocal_thrd_status_.store(3); //重定位成功 =============================================
					} else {
						global_localize_count++;
					}
				}
				if (global_localize_count > global_localize_times) {
					// cout << "global Localization failed: time out"<<endl;
					cout << RED << "global Localization failed: time out" << RESET << endl;
					// m_status_ = M_RELOCALIZE_FAILED;
					secmap_relocal_thrd_status_.store(2); //重定位失败 =============================================
				}
			}
		}
		auto end = std::chrono::steady_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		if (elapsed < period) {
			std::this_thread::sleep_for(period - elapsed);
		}
	}
}

void LidarSlam::showThread() {
	// return;
	const int frequency = 1.0; // 频率为1Hz
	const std::chrono::milliseconds period(1000 / frequency);
	while (thread_run && reseting == false) {
		auto start = std::chrono::steady_clock::now();
		auto end = std::chrono::steady_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

		if (elapsed < period) {
			std::this_thread::sleep_for(period - elapsed);
		}
	}
}

// void LidarSlam::robosense_pcl_cbk(const pcl::PointCloud<RsPointXYZIRT>::Ptr &cloud){
//     double t0 = omp_get_wtime();
//     if (reseting)
//         return;
// }

void LidarSlam::lidar_pcl_cbk(const PointCloudType::Ptr cloud) {
	static const int keep_lidar_num_before_curr = config_param_.lidar_preproc.keep_lidar_num_before_curr;
	static const double jump_back_time_thr = -1.0;
	if (reseting) {
		return;
	}

	double t0 = omp_get_wtime();

	// 出现的bug: 系统时间跳变，往回跳零点几秒，程序出错，slam 显示定位正常，但是定位已经不对
	// 处理： 当时间跳回过去，跳变间隔在1秒以内，则跳过这几帧数据
	double curr_time = cloud->header.stamp * 1.0 * 1e-6; // 转换为单位： second
	// double delta_time = curr_time - last_timestamp_lidar;
	// if(delta_time < 0 && delta_time > jump_back_time_thr){
	//     ROS_ERROR_STREAM("lidar cbk: time jump back, curr_time - last_time = " << delta_time);
	//     // last_timestamp_lidar 不更新，等待最新时间 到达 跳变前的时间
	//     return;
	// }
	// if(delta_time < 0 && delta_time < jump_back_time_thr){
	//     ROS_ERROR_STREAM("lidar cbk: time jump back(>1s), curr_time - last_time = " << delta_time);
	// }
	if (curr_time < last_timestamp_lidar) {
		cout << "lidar loop back, clear buffer" << endl;
		lidar_buffer.clear();
		cout << "************************* lidar_buffer clear *********" << endl;
	}

	/*  else if (msg->time_stamp - curr_time > 1.5 * 0.1){
		// printf("lidar lose rate");
		ROS_WARN_STREAM(YELLOW << "lidar lose rate" << RESET);
	}*/

	if (!time_sync_en && abs(last_timestamp_imu - curr_time) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty()) {
		cout << YELLOW << setprecision(15) << "IMU and LiDAR not Synced, IMU time: " << last_timestamp_imu
			 << ", lidar header time: " << curr_time << RESET << endl;
		cout << YELLOW << setprecision(15)
			 << "IMU and LiDAR not Synced, imu - lidar time diff: " << last_timestamp_imu - curr_time << RESET << endl;
	}

	if (time_sync_en && !timediff_set_flg && abs(curr_time - last_timestamp_imu) > 1 && !imu_buffer.empty()) {
		timediff_set_flg = true;
		timediff_lidar_wrt_imu = curr_time + 0.1 - last_timestamp_imu; //????
		// cout<<"Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu<< endl;
	}

	std::lock_guard<std::mutex> lk(mtx_buffer);
	while (lidar_buffer.size() > keep_lidar_num_before_curr) {
		lidar_buffer.pop_front();
		time_buffer.pop_front();
		cout << "pop one scan! " << endl;
	}
	lidar_buffer.push_back(cloud); //储存处理后的lidar特征
	time_buffer.push_back(curr_time);
	// ROS_INFO_STREAM("lidar pushed ");

	last_timestamp_lidar = curr_time;
	double t1 = omp_get_wtime();

	return;
}

/*****************************************************************************************************
// void LidarSlam::robosense_pcl_cbk(const PointCloudType::Ptr &cloud){
//     const bool flag_keep_only_last_lidar = config_param_.lidar_preproc.flag_keep_only_last_lidar;
//     double t0 = omp_get_wtime();
//     if (reseting)
//         return;

//     double curr_time = cloud->header.stamp * 1.0 * 1e-6;

//     if ( curr_time < last_timestamp_lidar){
//         // printf("lidar loop back, clear buffer");
//         ROS_INFO("lidar loop back, clear buffer");
//         lidar_buffer.clear();
//         // cout<<"************************* lidar_buffer clear *********"<<endl;
//         ROS_INFO("************************* lidar_buffer clear *********");
//     }

//     //  else if (msg->time_stamp - curr_time > 1.5 * 0.05){
//     //     // printf("lidar lose rate");
//     //     ROS_WARN_STREAM(YELLOW << "lidar lose rate" << RESET);
//     // }

//     if (!time_sync_en && abs(last_timestamp_imu - curr_time) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty()){
//         // printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu,
curr_time);
//         ROS_WARN_STREAM(YELLOW << "IMU and LiDAR not Synced, IMU time: "<< last_timestamp_imu << ", lidar header
time: " << curr_time << RESET);
//     }

//     if (time_sync_en && !timediff_set_flg && abs(curr_time - last_timestamp_imu) > 1 && !imu_buffer.empty()){
//         timediff_set_flg = true;
//         timediff_lidar_wrt_imu = curr_time + 0.1 - last_timestamp_imu; //????
//         // printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
//         ROS_INFO("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
//     }

//     std::lock_guard<std::mutex> lk(mtx_buffer);
//     if (flag_keep_only_last_lidar){
//         lidar_buffer.clear();
//         time_buffer.clear();
//     }
//     lidar_buffer.push_back(cloud); //储存处理后的lidar特征
//     time_buffer.push_back(curr_time);
//    // s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;

//     last_timestamp_lidar = curr_time;
//     double t1 = omp_get_wtime();
//     // printf("lidar-preproc , time cost: %f ms \033[0m \n", (t1 - t0)*1000);

//     return;
// }
*****************************************************************************************************/

/*****************************************************************************************************
// void LidarSlam::livox_pcl_cbk(const std::shared_ptr<livox_ros::LidarMsg> &msg_in){
//     // const bool flag_keep_only_last_lidar = config_param_.lidar_preproc.flag_keep_only_last_lidar;
//     double t0 = omp_get_wtime();
//     if (reseting)
//         return;

//     // double preprocess_start_time = omp_get_wtime();
//     // scan_count++;
//     std::shared_ptr<livox_ros::LidarMsg> msg(new livox_ros::LidarMsg(*msg_in));
//     if (msg->time_stamp < last_timestamp_lidar)
//     {
//         // printf("lidar loop back, clear buffer");
//         ROS_WARN_STREAM(YELLOW << "lidar loop back, clear buffer"<< RESET);
//         lidar_buffer.clear();
//         // cout<<"************************* lidar_buffer clear *********"<<endl;
//         ROS_WARN_STREAM(YELLOW<<"************************* lidar_buffer clear *********"<<RESET);
//     }
//     //   else if (msg->time_stamp - last_timestamp_lidar > 1.5 * 0.05){
//     //     // printf("lidar lose rate");
//     //     ROS_ERROR_STREAM(RED << "lidar lose rate" << RESET);
//     // }
//     last_timestamp_lidar = msg->time_stamp;

//     if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() &&
!lidar_buffer.empty())
//     {
//         // printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu,
last_timestamp_lidar);
//         ROS_WARN_STREAM(YELLOW << "IMU and LiDAR not Synced, IMU time: "<<last_timestamp_imu <<", lidar header time:
"<< last_timestamp_lidar<<RESET);
//     }

//     if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 &&
!imu_buffer.empty())
//     {
//         timediff_set_flg = true;
//         timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu; //????
//         // printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
//         ROS_INFO("Self sync IMU and LiDAR, time diff is %.10lf ", timediff_lidar_wrt_imu);
//     }

//     PointCloudType::Ptr ptr(new PointCloudType());

//     // 特征提取或间隔采样
//     // p_lidar_pre->process(msg, ptr);
//     lidar_pre_ptr_->pre_process(msg, ptr);

//     // {
//     //     std::lock_guard<std::mutex> lk(mtx_obstacle_cloud);
//     //     ObstacleCloud->points.clear();
//     //     FilteredObstacleCloud->points.clear();
//     //     // ObstacleCloud = transformPointCloud(p_lidar_pre->pl_obstacle, param.T_wheel_lidar);
//     //     int size = p_lidar_pre->pl_obstacle->points.size();
//     // }

//     std::lock_guard<std::mutex> lk(mtx_buffer);
//     if (flag_keep_only_last_lidar_){
//         lidar_buffer.clear();
//         time_buffer.clear();
//     }
//     lidar_buffer.push_back(ptr); //储存处理后的lidar特征
//     // cout<<"********************* lidar_buffer push back *********"<<endl;
//     time_buffer.push_back(last_timestamp_lidar);
//    // s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;

//     double t1 = omp_get_wtime();
//     // printf("lidar-preproc , time cost: %f ms \033[0m \n", (t1 - t0)*1000);
// }
*****************************************************************************************************/

void LidarSlam::imu_cbk(const std::shared_ptr<livox_ros::ImuMsg>& msg_in) { // base_link下的acc，gyro
	static const double jump_back_time_thr = -1.0;
	if (reseting) return;
	std::shared_ptr<livox_ros::ImuMsg> msg(new livox_ros::ImuMsg(*msg_in));
	if (!config_param_.common.offline_mode) {
		if (!imu_file_shift) {
			if (temp_imu_msg.size() > 0) {
				for (auto msg : temp_imu_msg) {
					imu_file << std::fixed << std::setprecision(9) << msg.time_stamp << " " << msg.angular_velocity.x()
							 << " " << msg.angular_velocity.y() << " " << msg.angular_velocity.z() << " "
							 << msg.linear_acceleration.x() << " " << msg.linear_acceleration.y() << " "
							 << msg.linear_acceleration.z() << std::endl;
				}
				temp_imu_msg.clear();
			}
			imu_file << std::fixed << std::setprecision(9) << msg->time_stamp << " " << msg->angular_velocity.x() << " "
					 << msg->angular_velocity.y() << " " << msg->angular_velocity.z() << " "
					 << msg->linear_acceleration.x() << " " << msg->linear_acceleration.y() << " "
					 << msg->linear_acceleration.z() << std::endl;
		} else {
			temp_imu_msg.push_back(*msg);
		}
	}
	// lidar 和 imu时间差过大，且开启 时间同步, 纠正当前输入imu的时间
	if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en) {
		// 对输入imu时间，纠正为 时间差 + 原始时间
		msg->time_stamp = (timediff_lidar_wrt_imu + msg_in->time_stamp);
	}

	double curr_timestamp_imu = msg->time_stamp;

	std::lock_guard<std::mutex> lk(mtx_buffer);
	// double delta_time = curr_timestamp_imu - last_timestamp_imu;
	// if(delta_time < 0 && delta_time > jump_back_time_thr){
	//     ROS_ERROR_STREAM("imu cbk: time jump back, curr_time - last_time = " << delta_time<<endl;
	//     // last_timestamp_imu 不更新，等待最新时间 到达 跳变前的时间
	//     return;
	// }
	if (curr_timestamp_imu < last_timestamp_imu) {
		cout << YELLOW << "imu loop back, clear buffer" << RESET << endl;
		imu_buffer.clear();
	} else if (curr_timestamp_imu - last_timestamp_imu > 1.5 * 0.1) {
		cout << YELLOW << "imu lose rate" << RESET << endl;
	}
	imu_buffer.push_back(msg);
	// cout<<"************************* imu_buffer push back *********"<<endl;
	last_timestamp_imu = curr_timestamp_imu; // update imu time
	localization_wait = true;
	// if (globalLocalizationSuccess||!param.localization_mode){// TODO add lock

	// std::lock_guard<std::mutex> lk2(mtx_pose);
	if (globalLocalizationSuccess || working_mode_ == MAPPING || working_mode_ == SEC_MAPPING) { // TODO add lock
		if (current_pose.base_time < localization_base.base_time - 0.005) {
			// std::cout << "predicate pose "<<current_pose.imu_state.pos.transpose()<<std::endl;
			// std::cout << "update pose "<<localization_base.imu_state.pos.transpose()<<std::endl;
			current_pose = localization_base;
			for (auto it = imu_buffer.begin(); it != imu_buffer.end(); it++) {
				const std::shared_ptr<livox_ros::ImuMsg>& msg = *it;
				if (msg->time_stamp > localization_base.update_time) {
					double dt = msg->time_stamp - localization_base.update_time;
					// if (dt < 0 || dt > 0.01)
					//     printf("error predicate 1 %f\n",dt);
					V3D angvel = V3D(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]) -
								 current_pose.imu_state.bg;
					V3D acc =
						V3D(msg->linear_acceleration[0], msg->linear_acceleration[1], msg->linear_acceleration[2]) *
						G_m_s2 / (p_imu->mean_acc.norm());
					acc = current_pose.imu_state.rot * (acc - current_pose.imu_state.ba) + current_pose.imu_state.grav;
					current_pose.imu_state.pos += current_pose.imu_state.vel * dt + 0.5 * acc * dt * dt;
					current_pose.imu_state.vel += acc * dt;
					current_pose.imu_state.rot = current_pose.imu_state.rot * Sophus::SO3d::exp(angvel * dt);
					// current_pose.imu_state.rot = current_pose.imu_state.rot * Sophus::SO3::exp(angvel * dt);
					localization_base.update_time = msg->time_stamp;
				}
			}
		} else {
			if (msg->time_stamp > localization_base.update_time) {
				double dt = msg->time_stamp - localization_base.update_time;
				// if (dt < 0 || dt > 0.01)
				//     printf("error predicate 2 %f\n",dt);
				V3D angvel = V3D(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]) -
							 current_pose.imu_state.bg;
				V3D acc = V3D(msg->linear_acceleration[0], msg->linear_acceleration[1], msg->linear_acceleration[2]) *
						  G_m_s2 / (p_imu->mean_acc.norm());
				acc = current_pose.imu_state.rot * (acc - current_pose.imu_state.ba) + current_pose.imu_state.grav;
				current_pose.imu_state.pos += current_pose.imu_state.vel * dt + 0.5 * acc * dt * dt;
				current_pose.imu_state.vel += acc * dt;
				current_pose.imu_state.rot = current_pose.imu_state.rot * Sophus::SO3d::exp(angvel * dt);
				// current_pose.imu_state.rot = current_pose.imu_state.rot * Sophus::SO3::exp(angvel * dt);
				localization_base.update_time = msg->time_stamp;
			}
		}
	}
	// poses_buffer.push_back(std::pair(curr_timestamp_imu,getLidarInOdom())); // 此为c++17用法
	poses_buffer.push_back(std::make_pair(curr_timestamp_imu, getLidarInOdom())); // 此为c++11用法
	if (poses_buffer.size() > 200) poses_buffer.pop_front();
	localization_wait = false;
	// if (param.localization_mode){

	// }
}

void LidarSlam::delete_log_file(double keep_time) { // about 100MB pr 60s
	if (pcd_file.size() < 10) return;
	if (pcd_file.back() - pcd_file.front() > keep_time) {
		// std::filesystem::remove(config_param_.common.save_log_dir + std::to_string(pcd_file.front()) + ".pcd");
		std::string file = config_param_.common.save_log_dir + std::to_string(pcd_file.front()) + ".pcd";
		std::remove(file.c_str());
		pcd_file.pop_front();
	}
	imu_file_shift = true;
	// trim_log_file(keep_time / 60.0 * 2 * 1024 * 1024,param.save_log_path +
	// std::string("imu_data.txt"),imu_file,pcd_file.front());
	imu_file_shift = false;
}

bool LidarSlam::run() {
	static const int prm_lidar_no_point_count_thr = config_param_.common.lidar_no_point_count_thr;

	// std::thread::id thisId = std::this_thread::get_id();
	// std::cout << "debug: lidar slam main     Thread ID: " << thisId << std::endl;
	/// 在Measure内，储存当前lidar数据及lidar扫描时间内对应的imu数据序列
	static int frame_num = 0;
	static double aver_time_consu = 0, aver_time_icp = 0, aver_time_incre = 0, aver_time_solve = 0;
	double t0, t1, t2, t3, t4, t5, match_start, solve_start, run_start, run_end, t0_backend, t1_backend, t0_transform,
		t1_transform;
	run_start = omp_get_wtime();
	// cout<<"lidar buffer size: "<<lidar_buffer.size()<<endl;
	// cout<<"imu   buffer size: "<<imu_buffer.size()<<endl;

	if (sync_packages(Measures)) {
		// ROS_INFO_STREAM(setprecision(15) << ros::Time::now().toSec() << ": ---------sync_packages " << GREEN <<
		// "success" << RESET <<" --------------------------"); 第一帧lidar数据
		static double last_lidar_time = Measures.lidar_beg_time;

		if (flg_first_scan) {
			first_lidar_time = Measures.lidar_beg_time; //记录第一帧绝对时间
			p_imu->first_lidar_time = first_lidar_time; //记录第一帧绝对时间
			flg_first_scan = false;
			cout << "***************** flg_first_scan ********" << endl;
			return false;
		}

		// log_info_manager_->slam_info.data[13]= -100; //
		// log_info_manager_->slam_info.data[15]= -100; //
		log_info_manager_->slam_info.data[24] = Measures.lidar_beg_time - last_lidar_time;
		log_info_manager_->slam_info.data[25] = Measures.lidar_beg_time;
		log_info_manager_->slam_info.data[26] = Measures.lidar_end_time - Measures.lidar_beg_time;
		log_info_manager_->slam_info.data[27] = Measures.imu.front()->time_stamp - Measures.lidar_beg_time;
		log_info_manager_->slam_info.data[28] = Measures.imu.back()->time_stamp - Measures.lidar_beg_time;
		last_lidar_time = Measures.lidar_beg_time;
		t0 = omp_get_wtime();

		// 根据imu数据序列和lidar数据，向前传播纠正点云的畸变, 此前已经完成间隔采样或特征提取
		{
			std::lock_guard<std::mutex> lk(mtx_lidar_cloud);
			undistortCloud->clear();
			p_imu->Process(Measures, kf, undistortCloud);
			log_info_manager_->slam_info.data[11] = undistortCloud->size(); //
		}
		state_ikfom state_point;
		state_point = kf.get_x();
		Eigen::Isometry3d T_b_lidar(
			Sophus::SE3d(state_point.offset_R_L_I, state_point.offset_T_L_I).matrix()); // TODO 不优化外参数就提出去
		Eigen::Isometry3d T_odom_b(Sophus::SE3d(state_point.rot, state_point.pos).matrix());
		// Eigen::Isometry3d T_b_lidar(Sophus::SE3(state_point.offset_R_L_I, state_point.offset_T_L_I).matrix());// TODO
		// 不优化外参数就提出去 Eigen::Isometry3d T_odom_b(Sophus::SE3(state_point.rot, state_point.pos).matrix());
		{
			std::lock_guard<std::mutex> lk(mtx_pose);
			T_odom_lidar = T_odom_b * T_b_lidar; // TODO check this
		}
		//  pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I; // global系 lidar位置
		t1 = omp_get_wtime();
		if (undistortCloud->empty() || (undistortCloud == NULL)) {
			lidar_no_point_count_++;
			if (lidar_no_point_count_ > prm_lidar_no_point_count_thr) {
				slam_run_status_.store(2);
			}
			cout << YELLOW << "No point, skip this scan!" << RESET << endl;
			log_info_manager_->slam_info.data[15] = lidar_no_point_count_; //
			log_info_manager_->slam_info.data[13] = 0;					   //
			return false;
		}

		// 检查当前lidar数据时间，与最早lidar数据时间是否足够
		bool flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < 0.1 ? false : true;

		/*** Segment the map in lidar FOV ***/
		// 动态调整局部地图,在拿到eskf前馈结果后
		ikdtree->lasermap_fov_segment(
			T_odom_lidar.translation()); // 根据lidar在W系下的位置，重新确定局部地图的包围盒角点，移除远端的点
		t2 = omp_get_wtime();
		/*** downsample the feature points in a scan ***/
		// ROS_INFO("undistortCloudcount: %d", undistortCloud->points.size());
		downSizeFilterCloud.setInputCloud(undistortCloud); // zx 0.5?
		downSizeFilterCloud.filter(*FilteredUndistortCloud);

		int feats_down_size = FilteredUndistortCloud->points.size(); //当前帧降采样后点数
		log_info_manager_->slam_info.data[13] = feats_down_size;	 //
		PointCloudType::Ptr FilteredUndistortCloudInOdom(new PointCloudType());
		double filter_time = omp_get_wtime();
		/*** initialize the map kdtree ***/
		if (ikdtree->Root_Node == nullptr) {
			// if (feats_down_size > 5)
			if (feats_down_size > feats_down_size_thr_) {
				ikdtree->set_downsample_param(config_param_.ikdtree.map_leaf_size); // 0.5 默认0.2
				ikdtree->set_cube_len(config_param_.ikdtree.cube_len);
				ikdtree->set_det_range(config_param_.ikdtree.det_range);

				FilteredUndistortCloudInOdom->resize(feats_down_size);
				//   for (int i = 0; i < feats_down_size; i++)
				//   {
				FilteredUndistortCloudInOdom =
					transformPointCloud(FilteredUndistortCloud, T_odom_lidar); // point转到odom系下
				//   }
				// world系下对当前帧降采样后的点云，初始化lkd-tree
				ikdtree->Build(FilteredUndistortCloudInOdom->points);
			}
			// std::cout << "build ikdtree! "<<feats_down_size<< std::endl;
			cout << GREEN << "build ikdtree! " << feats_down_size << RESET << endl;
			return false;
		}

		int featsFromMapNum = ikdtree->validnum();
		int kdtree_size_st = ikdtree->size();

		// cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num:
		// "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

		/*** ICP and iterated Kalman filter update ***/
		// if (feats_down_size < 5)
		if (feats_down_size < feats_down_size_thr_) {
			lidar_no_point_count_++;
			if (lidar_no_point_count_ > prm_lidar_no_point_count_thr) {
				slam_run_status_.store(2);
			}
			log_info_manager_->slam_info.data[15] = lidar_no_point_count_; //
			cout << YELLOW << "No point after filter, skip this scan!" << RESET << endl;
			return false;
		} else {
			lidar_no_point_count_ = 0;
			log_info_manager_->slam_info.data[15] = lidar_no_point_count_; //
			slam_run_status_.store(1);
		}

		FilteredUndistortCloudInOdom->resize(feats_down_size);

		/* if (true) // If you need to see map point, change to "if(1)" //zx delete this publish
		{
			PointVector().swap(ikdtree.PCL_Storage);
			ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
			kdtreeCloud->clear();
			kdtreeCloud->points = ikdtree.PCL_Storage;
			// publish_map(pubLaserCloudMap);
		}*/
		vector<PointVector> Nearest_Points;
		Nearest_Points.resize(feats_down_size);

		/*** iterated state estimation ***/
		double t_update_start = omp_get_wtime();
		kf.update_iterated_dyn_share_modified(0.001, FilteredUndistortCloud, *ikdtree, Nearest_Points, 4, false);
		double t_update_end = omp_get_wtime();
		state_point = kf.get_x();
		// auto p = kf.get_P();
		// for (int ii=0; ii<24; ii++){
		//     std::cout<<"i= "<<ii<<"--cov(i,i): "<<p(ii, ii)<<endl;
		// }
		T_b_lidar = Sophus::SE3d(state_point.offset_R_L_I, state_point.offset_T_L_I).matrix();
		T_odom_b = Sophus::SE3d(state_point.rot, state_point.pos).matrix();
		// T_b_lidar = Sophus::SE3(state_point.offset_R_L_I, state_point.offset_T_L_I).matrix();
		// T_odom_b = Sophus::SE3(state_point.rot, state_point.pos).matrix();
		T_odom_lidar = T_odom_b * T_b_lidar; // TODO check this
		//  double t_update_end = omp_get_wtime();
		//   while(localization_wait){ // TODO check

		//   }
		localization_base.imu_state = state_point;
		localization_base.base_time = lidar_end_time;
		localization_base.update_time = lidar_end_time;

		t0_backend = omp_get_wtime();
		// if (!param.localization_mode){
		if (working_mode_ == MAPPING || working_mode_ == SEC_MAPPING) {
			if (working_mode_ == SEC_MAPPING && !globalLocalizationSuccess) {
				cout << "Waiting for global localization ..." << endl;
			} else if (working_mode_ == SEC_MAPPING && !back_end->get_loaded_key_cloud_status()) {
				cout << "Waiting for loading key cloud ..." << endl;
			} else {
				loop_closure_wait = true;

				// cout<<"************* backend: checking keyPosesCount: "<<back_end->getKeyframePoses().size()<<endl;
				bool insert = back_end->saveKeyFramesAndFactor(T_odom_lidar, undistortCloud,
															   lidar_end_time); // TODO add transform
				if (insert) {
					cout << YELLOW << "************************* backend: keyPosesCount: "
						 << back_end->getKeyframePoses().size() - 1 << RESET << endl;
					// cout<<"debug: loaded_key_clouds_ready_: "<<back_end->get_loaded_key_cloud_status()<<endl;
					back_end->saveCurrentCloud(undistortCloud,
											   getLidarInMap()); //注意这里只是为了取水平面，后端还是在odom坐标系
					{
						std::lock_guard<std::mutex> lk(mtx_path);
						unoptimized_path.emplace_back(getWheelInMap()); // TODO max size
						if (unoptimized_path.size() > 200) unoptimized_path.pop_front();
					}
					T_odom_lidar = back_end->getCurrentPose().pose;
					T_odom_b = T_odom_lidar * T_b_lidar.inverse();
					state_ikfom state_updated = kf.get_x();
					state_updated.pos = T_odom_b.translation();
					state_updated.rot = Sophus::SO3d(T_odom_b.rotation());
					// state_updated.rot =  Sophus::SO3(T_odom_b.rotation());
					kf.change_x(state_updated);
					new_key_cloud_arrived_ = true;
				}

				// 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿，更新里程计轨迹， 重构ikdtree

				// ROS_INFO_STREAM(BLUE<<"check LoopIsClosed "<<RESET);
				bool LoopIsClosed = back_end->correctPoses();
				{
					std::lock_guard<std::mutex> lk(mtx_path);
					optimized_path.clear();
					std::vector<KeyPose> lidar_in_odom;
					lidar_in_odom = back_end->getKeyframePoses();
					for (int i = 0; i < lidar_in_odom.size(); i++) { // TODO max size
						optimized_path.emplace_back(getOdomToMap() * lidar_in_odom[i].pose * T_lidar_wheel);
					}
				}
				if (LoopIsClosed) {
					back_end->recontructIKdTree(*ikdtree, config_param_.ikdtree.kdTreeReconstructRadius,
												config_param_.ikdtree.kdTreeReconstructKeyFrameLeafSize,
												config_param_.ikdtree.kdTreeReconstructPointLeafSize);
				}
				loop_closure_wait = false;
			}

		} else if (working_mode_ == LOCALIZATION) {
			{
				std::lock_guard<std::mutex> lk(mtx_path);
				unoptimized_path.emplace_back(getWheelInMap()); // TODO max size
				if (unoptimized_path.size() > 200) unoptimized_path.pop_front();
			}
		}

		t1_backend = omp_get_wtime();
		// std::cout<<"test "<< R2ypr(T_odom_lidar.matrix().block<3, 3>(0, 0)).transpose() << std::endl;
		// if (!param.localization_mode){
		// if (working_mode_==MAPPING || working_mode_ == SEC_MAPPING){
		// }

		t0_transform = omp_get_wtime();
		{
			std::lock_guard<std::mutex> lk(mtx_odom_cloud);
			UndistortCloudInOdom->resize(undistortCloud->points.size());
			UndistortCloudInOdom = transformPointCloud(undistortCloud, T_odom_lidar);
		}
		t1_transform = omp_get_wtime();
		t3 = omp_get_wtime();
		// std::cout << "debug: UndistortCloud      Thread ID: " << thisId << std::endl;

		/*** add the feature points to map kdtree ***/
		FilteredUndistortCloudInOdom = transformPointCloud(FilteredUndistortCloud, T_odom_lidar);

		t4 = omp_get_wtime();
		ikdtree->map_incremental(FilteredUndistortCloudInOdom, Nearest_Points, flg_EKF_inited);
		t5 = omp_get_wtime();
		{
			frame_num++;
			int kdtree_size_end = ikdtree->size();
			// std::cout <<"??? " <<aver_time_consu * (frame_num - 1) / frame_num << " "<<(t5 - t0) / frame_num<< "
			// "<<frame_num<< std::endl;
			aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
			aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t_update_end - t_update_start) / frame_num;

			//   printf("[ mapping ]: time: IMU process: %0.6f,kdtree size %d test %0.6f,test1 %0.6f, ave ICP: %0.6f,
			//   map incre: %0.6f ave total: %0.6f \n" , t1 - t0, kdtree_size_end, filter_time - t2,t3 - t_update_end,
			//   aver_time_icp, t5 - t4, aver_time_consu);
		}
		run_end = omp_get_wtime();

		if (run_end - run_start > config_param_.common.slam_lose_rate_time_thr) {
			cout << RED << "lidar-slam    , time cost: " << (run_end - run_start) * 1000 << " ms, lose rate !!!!!!!"
				 << RESET << endl;
			// ROS_INFO_STREAM("p_imu->Process, cloud deskew    , time cost: " << (t1-t0)*1000 << " ms;");
			// ROS_INFO_STREAM("ikdtree->lasermap_fov_segment   , time cost: " << (t2-t1)*1000 << " ms;");
			// ROS_INFO_STREAM("lidar slam main process step1   , time cost: " << (t0_backend-t2)*1000 << " ms;");
			// ROS_INFO_STREAM("lidar slam main update  time    , time cost: " << (t_update_end-t_update_start)*1000 <<
			// " ms;"); ROS_INFO_STREAM("lidar slam main process         , time cost: " << (t3-t2)*1000 << " ms;");
			// ROS_INFO_STREAM("main: lidar slam backend        , time cost: " << (t1_backend-t0_backend)*1000 << "
			// ms;"); ROS_INFO_STREAM("main: transform undistortCloud  , time cost: " <<
			// (t1_transform-t0_transform)*1000 << " ms;"); ROS_INFO_STREAM("transform FilteredUndistortCloud, time
			// cost: " << (t4-t3)*1000 << " ms;"); ROS_INFO_STREAM("ikdtree->map_incremental        , time cost: " <<
			// (t5-t4)*1000 << " ms;"); ROS_INFO_STREAM("feats_down_size: " << feats_down_size);
			cout << "-------------------------------------------------- " << endl;
		} else if (run_end - run_start > 0.07) {
			cout << YELLOW << "lidar-slam    , time cost: " << (run_end - run_start) * 1000 << " ms -------------------"
				 << RESET << endl;
			// ROS_INFO_STREAM("p_imu->Process, cloud deskew    , time cost: " << (t1-t0)*1000 << " ms;");
			// ROS_INFO_STREAM("ikdtree->lasermap_fov_segment   , time cost: " << (t2-t1)*1000 << " ms;");
			// ROS_INFO_STREAM("lidar slam main process step1   , time cost: " << (t0_backend-t2)*1000 << " ms;");
			// ROS_INFO_STREAM("lidar slam main update  time    , time cost: " << (t_update_end-t_update_start)*1000 <<
			// " ms;"); ROS_INFO_STREAM("lidar slam main process         , time cost: " << (t3-t2)*1000 << " ms;");
			// ROS_INFO_STREAM("main: lidar slam backend        , time cost: " << (t1_backend-t0_backend)*1000 << "
			// ms;"); ROS_INFO_STREAM("main: transform undistortCloud  , time cost: " <<
			// (t1_transform-t0_transform)*1000 << " ms;"); ROS_INFO_STREAM("transform FilteredUndistortCloud, time
			// cost: " << (t4-t3)*1000 << " ms;"); ROS_INFO_STREAM("ikdtree->map_incremental        , time cost: " <<
			// (t5-t4)*1000 << " ms;"); ROS_INFO_STREAM("feats_down_size: " << feats_down_size);
			// ROS_INFO_STREAM("-------------------------------------------------- " );
		} else {
			// ROS_INFO_STREAM(GREEN  <<"lidar-slam    , time cost: "<< (run_end - run_start)*1000<<" ms
			// -------------------"<<RESET);
		}

		// ROS_INFO_STREAM("---------end---------------------------------------------");
		lidar_no_point_count_ = 0;

		return true;
	} else {
		// cout << "sync measure failed !"<<endl;
		// ROS_WARN_STREAM("---------sync_packages " << RED << "failed " << RESET <<" --------------------------");
		delete_log_file(config_param_.common.log_keep_time);
	}

	return false;
}
} // namespace lidar_slam

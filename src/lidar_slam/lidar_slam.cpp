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

LidarSlam::LidarSlam(const LidarSlamParam yaml_param, SlamWorkMode start_mode, rclcpp::Node::SharedPtr node) {
	config_param_ = yaml_param;
	feats_down_size_thr_ = config_param_.common.feats_down_size_thr;
	LidarSlam::reset(start_mode, node);
}

void LidarSlam::reset(SlamWorkMode work_mode, rclcpp::Node::SharedPtr node) {
	log_info_manager_ = localization_module::LocalizationModuleLogInfoManager::getInstance();
	log_info_manager_->reset_log_info();
	slam_run_status_.store(0);

	sleep(1); // TODO(jxl): 要休眠1s吗

	// CPU_ZERO(&mask); // 初始化 CPU 亲和性集合，将其设置为零
	// CPU_SET(0, &mask); // 将线程绑定到 cpu_id 核心

	/// 激光和IMU预处理相关
	time_buffer_.clear();  // 记录lidar时间
	lidar_buffer_.clear(); //记录特征提取或间隔采样后的lidar（特征）数据
	imu_buffer_.clear();

	lidar_pushed_ = false;
	lidar_end_time_ = 0;
	lidar_mean_scantime_ = 0.0;
	first_lidar_time_ = 0.0;

	scan_num_ = 0;
	flg_first_scan_ = true;
	last_timestamp_lidar_ = 0;
	last_timestamp_imu_ = -1.0;
	timediff_lidar_wrt_imu_ = 0.0;

	time_sync_en_ = false;
	timediff_set_flg_ = false; // 标记是否已经进行了时间补偿

	Measures_ = MeasureGroup();
	temp_imu_msg_.clear();

	/// 点云 reset
	UndistortCloudInOdom_.reset(new PointCloudType());
	undistortCloud_.reset(new PointCloudType()); // lidar 系
	FilteredUndistortCloud_.reset(new PointCloudType());
	kdtreeCloud_.reset(new PointCloudType());

	// ObstacleCloud_.reset(new PointCloudType());
	// FilteredObstacleCloud_.reset(new PointCloudType());

	/// mapping 相关
	unoptimized_path_.clear();
	optimized_path_.clear();
	T_odom_lidar_ = Eigen::Isometry3d::Identity();
	localization_base_ = Localization_base();
	current_pose_ = Localization_base();
	imu_file_shift_ = false;

	auto cloud_leaf_size = config_param_.mapping.cloud_leaf_size;
	downSizeFilterCloud_.setLeafSize(cloud_leaf_size, cloud_leaf_size, cloud_leaf_size);

	auto cloud_leaf_size_test = config_param_.lidar_preproc.leafsize;
	downSizeFilterCloud_test_.setLeafSize(cloud_leaf_size_test, cloud_leaf_size_test, cloud_leaf_size_test);

	auto key_frame_distance = config_param_.mapping.key_frame_distance;
	auto key_frame_angle = config_param_.mapping.key_frame_angle;
	auto loopSearchDistance = config_param_.mapping.loopSearchDistance;
	auto loopSearchTimeDiff = config_param_.mapping.loopSearchTimeDiff;
	auto loopSearchSkipKey = config_param_.mapping.loopSearchSkipKey;
	auto loopIcpScore = config_param_.mapping.loopIcpScore;
	back_end_.reset(new BackEnd(key_frame_distance, key_frame_angle, loopSearchDistance, loopSearchTimeDiff,
								loopSearchSkipKey, loopIcpScore));

	/// sec_mapping & localizaiton
	globalLocalizationSuccess_ = false;
	global_localize_count_ = 0;

	// ikdtree_
	ikdtree_.reset(new KD_TREE<pcl::PointXYZINormal>());
	kf_ = esekfom::esekf();

	const auto gyr_cov = config_param_.mapping.gyr_cov;
	const auto acc_cov = config_param_.mapping.acc_cov;
	const auto b_gyr_cov = config_param_.mapping.b_gyr_cov;
	const auto b_acc_cov = config_param_.mapping.b_acc_cov;

	const auto extrinT = config_param_.extrinsic.extrinT;
	const auto extrinR = config_param_.extrinsic.extrinR;

	p_imu_.reset(new ImuProcess());
	p_imu_->set_param(extrinT, extrinR, V3D(gyr_cov, gyr_cov, gyr_cov), V3D(acc_cov, acc_cov, acc_cov),
					  V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov), V3D(b_acc_cov, b_acc_cov, b_acc_cov));
	// 定位
	localization_.reset(new Localization());

	global_localization_.reset(new GlobalLocalization());
	cloud_map_manager_.reset(new CloudMap());

	// 线程相关 ************************************************
	if (thread_ != nullptr) {
		thread_run_ = false;
		thread_->join();
		// show_thread_->join();
		thread_run_ = true;
		if (work_mode == SEC_MAPPING) {
			global_localization_thread_->join();
		}
	}

	reseting_ = false;
	double curr_time = rclcpp::Clock().now().seconds();
	if (work_mode == MAPPING) {
		thread_.reset(new std::thread(&LidarSlam::loopClosureThread, this));
		hb_time_thread_loop_closure_.store(curr_time);
	} else if (work_mode == SEC_MAPPING) {
		global_localization_thread_.reset(
			new std::thread(&LidarSlam::global_localization_for_sec_mapping_thread, this)); //全局定位初始化

		thread_.reset(new std::thread(&LidarSlam::sec_mapping_loopClosureThread, this));

		hb_time_thread_loop_closure_.store(curr_time);
		hb_time_thread_secmap_relocalize_.store(curr_time);
	} else if (work_mode == LOCALIZATION) {
		thread_.reset(new std::thread(&LidarSlam::localizationThread, this));
		hb_time_thread_localize_.store(curr_time);
	}
	working_mode_ = work_mode; // mapping, sec_mapping, localization
}

bool LidarSlam::sync_packages(MeasureGroup& meas) {
	if (lidar_buffer_.empty() || imu_buffer_.empty()) {
		return false;
	}
	if (reseting_ == true) {
		cout << "reseting true, sync packages return false!" << endl;
		return false;
	}
	if (lidar_pushed_ &&
		omp_get_wtime() - time_buffer_.front() > 0.15) { // TODO(jxl): 应该是用系统或者bag中当前时刻去作差
		cout << "lidar lose rate: " << omp_get_wtime() - time_buffer_.front() << endl;
	}

	if (!lidar_pushed_) {
		meas.lidar = lidar_buffer_.front();									// lidar指针指向最旧的lidar数据
		meas.lidar_beg_time = time_buffer_.front();							// 记录最早时间
		lidar_mean_scantime_ = meas.lidar->points.back().curvature * 0.001; //在sampling_cloud()中curvature单位转成了ms
		lidar_mean_scantime_ = (lidar_mean_scantime_ < 0.1 || lidar_mean_scantime_ > 0.15) ? 0.1 : lidar_mean_scantime_;
		lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
		meas.lidar_end_time = lidar_end_time_;
		lidar_pushed_ = true;
	}

	if (last_timestamp_imu_ < lidar_end_time_) {
		cout << RED << "latest imu time < lidar time " << RESET << endl;
		cout << YELLOW << "last_timestamp_imu_: " << setprecision(15) << last_timestamp_imu_ << RESET << endl;
		cout << YELLOW << "lidar_end_time_: " << setprecision(15) << lidar_end_time_ << RESET << endl;
		return false;
	}

	/*** push imu data, and pop from imu buffer ***/
	double imu_time = imu_buffer_.front()->time_stamp; // 最旧IMU时间
	meas.imu.clear();

	std::unique_lock<std::mutex> lk(mtx_buffer_);
	while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) { //记录imu数据，imu时间小于当前帧lidar结束时间
		imu_time = imu_buffer_.front()->time_stamp;
		if (imu_time > lidar_end_time_) {
			break;
		}
		meas.imu.push_back(imu_buffer_.front()); //记录当前lidar帧内的imu数据到meas.imu
		imu_buffer_.pop_front();				 // TODO(jxl): 加锁
	}

	if (meas.imu.empty()) {
		cout << RED << "Measure.imu is empty. " << RESET << endl;
		lidar_pushed_ = false;
		cout << "imu_buffer_.front.time: " << setprecision(15) << imu_time << endl;
		cout << "imu_buffer_.back.time: " << setprecision(15) << imu_buffer_.back()->time_stamp << endl;
		cout << "lidar count: " << meas.lidar->points.size() << endl;
		cout << "lidar first point: " << setprecision(5) << meas.lidar->points.front().curvature * 0.001 << endl;
		cout << "lidar end point: " << setprecision(5) << meas.lidar->points.back().curvature * 0.001 << endl;
		cout << "lidar_beg_time.time: " << setprecision(15) << meas.lidar_beg_time << endl;
		cout << "lidar_end_time_.time: " << setprecision(15) << lidar_end_time_ << endl;
		lidar_buffer_.pop_front(); // TODO(jxl): 加锁
		time_buffer_.pop_front();
		return false;
	}
	log_info_manager_->slam_info.data[30] = meas.lidar->points.size();
	log_info_manager_->slam_info.data[31] = lidar_buffer_.front()->points.size();

	lidar_buffer_.pop_front();
	time_buffer_.pop_front();

	lidar_pushed_ = false;
	return true;
}

void LidarSlam::sec_mapping_loopClosureThread() {
	const int frequency = 1; // 频率为1Hz
	const std::chrono::milliseconds period(1000 / frequency);
	while (thread_run_ && reseting_ == false) {
		hb_time_thread_loop_closure_.store(rclcpp::Clock().now().seconds());
		auto start = std::chrono::steady_clock::now();

		// 对于二次建图，重定位成功之前，不进行回环检测
		if (!globalLocalizationSuccess_) {
			//.... do nothing
		} else if (!back_end_->get_loaded_key_cloud_status()) {
			// TODO(jxl): add warnning log

			const std::vector<PointCloudType::Ptr>& loaded_keyframe_clouds =
				cloud_map_manager_->get_loaded_keyframe_clouds();
			const auto& loaded_keyframe_poses = cloud_map_manager_->get_loaded_keyframe_poses();
			const auto& loaded_sc_info = cloud_map_manager_->get_load_sc_info_();
			const Eigen::Isometry3d& global_odom_to_map = global_localization_->get_global_odom_to_map();
			// sec_mapping模式下，最开始全局重定位确定的T_map_odom

			// TODO bug here  //TODO(jxl): 什么意思，这里有bug？
			if (cloud_map_manager_->get_map_data_status()) {
				back_end_->set_loaded_key_clouds(loaded_keyframe_clouds, loaded_sc_info, loaded_keyframe_poses,
												 global_odom_to_map);
			}
		} else {
			back_end_->performLoopClosure(lidar_end_time_); //回环检测
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
	while (thread_run_ && reseting_ == false) {
		hb_time_thread_loop_closure_.store(rclcpp::Clock().now().seconds());

		auto start = std::chrono::steady_clock::now();
		back_end_->performLoopClosure(lidar_end_time_); //  回环检测
		// back_end_->performSCLoopClosure();

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

	const auto use_pose_filter = config_param_.common.use_pose_filter;

	const auto score_thr = config_param_.re_localization.score_thr;
	const auto global_localize_time_out_thr = config_param_.re_localization.time_out_thr;
	const int global_localize_times = global_localize_time_out_thr * frequency; // 重定位次数

	const float period_local_sec = config_param_.localization.fgicp_peroid_sec;
	const auto odom2map_delta_thr = config_param_.localization.odom2map_delta_thr;
	const auto odom2map_delta_set = config_param_.localization.odom2map_delta_set;

	const auto fgicp_score_fail_thr = config_param_.localization.fgicp_score_fail_thr;
	const auto fgicp_score_low_accuracy_thr = config_param_.localization.fgicp_score_low_accuracy_thr;
	const auto fgicp_fail_count_thr = config_param_.localization.fgicp_fail_count_thr;
	const auto fgicp_low_accuracy_count_thr = config_param_.localization.fgicp_low_accuracy_count_thr;

	int gicp_fail_count = 0;
	int gicp_low_acc_count = 0;
	pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>());
	static int wait_time = 0;

	while (thread_run_ && reseting_ == false) {
		hb_time_thread_localize_.store(rclcpp::Clock().now().seconds());
		auto start = std::chrono::steady_clock::now();
		temp.reset(new pcl::PointCloud<pcl::PointXYZI>());
		{
			std::unique_lock<std::mutex> lk(mtx_odom_cloud_);
			pcl::copyPointCloud(*(UndistortCloudInOdom_), *temp); // odom下的点云
		}

		if (localize_thrd_status_.load() == 2) { // 重定位失败
			cout << "global Localization failed: time out " << endl;
		} else {
			if (!globalLocalizationSuccess_) {
				localize_thrd_status_.store(1);

				if (!getLoadMap()) {
					cout << YELLOW << "globalLocalization failed: map not ready ... " << RESET << endl;
				} else if (!temp || temp->points.size() == 0) {
					cout << YELLOW << "globalLocalization failed: cloud empty ... " << RESET << endl;
				} else {
					cout << "point(in use) count: " << temp->points.size() << endl;
					cout << "start globalLocalization ... " << endl;
					PointCloudType::Ptr FilteredUndistortCloud_test(new PointCloudType());
					{
						std::unique_lock<std::mutex> lk(mtx_lidar_cloud_);
						downSizeFilterCloud_test_.setInputCloud(undistortCloud_); // lidar系下的点云
						downSizeFilterCloud_test_.filter(*FilteredUndistortCloud_test);
					}

					double t0 = omp_get_wtime();
					globalLocalizationSuccess_ = localization_->globalLocalization(
						FilteredUndistortCloud_test, T_odom_lidar_, p_imu_->initial_rotate_, score_thr);
					// TODO(jxl): T_odom_lidar加锁，拷贝后解锁
					// initial_rotate_可以拷贝出来再使用

					// TODO(jxl): 定位模式下的定位模块全局定位初始化，和sec_mapping模式下全局定位初始化原理一样吗？

					double t1 = omp_get_wtime();
					// cout << "global Localization cost time: " << (t1 - t0) * 1000 << " ms" << endl;

					global_localize_count_++;
				}
				if (!globalLocalizationSuccess_ && global_localize_count_ > global_localize_times) {
					cout << "global Localization failed: time out" << endl;
					localize_thrd_status_.store(2);
				}
				if (globalLocalizationSuccess_) {
					cout << BOLDGREEN << " ======= global Localization Success ======= " << RESET << endl;
					global_localize_count_ = 0;
					localize_thrd_status_.store(3);
					need_localize_ = false; //全局重定位成功后要等60s才会进行第一次定位
					wait_time++;
				}

				// TODO(jxl): 这段代码可以不要，最外层有休眠
				auto end = std::chrono::steady_clock::now();
				auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
				if (elapsed < period_relocal) {
					std::this_thread::sleep_for(period_relocal - elapsed);
				}
				continue;
			} else { //全局重定位成功
				if (wait_time >= period_local_sec) {
					need_localize_ = true;
					wait_time = 0;
				}

				if (need_localize_) {
					cout << "localizing ... " << endl;
					double fit_score = 0.0; // gicp_fit_score
					if (localization_->localize(temp, fit_score, fgicp_score_fail_thr, fgicp_score_low_accuracy_thr,
												odom2map_delta_thr, odom2map_delta_set, use_pose_filter)) {
						std::cout << "fit_score :" << fit_score << std::endl;
						log_info_manager_->slam_info.data[3] = 1; // if converge
						if (fit_score < fgicp_score_low_accuracy_thr) {
							localize_thrd_status_.store(3);
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
					} else { // 未收敛
						log_info_manager_->slam_info.data[3] = 0;
						gicp_fail_count++;
						gicp_low_acc_count++;
						cout << RED << "fast gicp fail count: " << gicp_fail_count << RESET << endl;
						cout << YELLOW << "fast gicp low accuracy count: " << gicp_low_acc_count << RESET << endl;
					}

					if (gicp_fail_count >= fgicp_fail_count_thr ||
						gicp_low_acc_count >= fgicp_low_accuracy_count_thr) { // 连续多帧 fast-gicp 失败，则认为定位失败
						localize_thrd_status_.store(5);
					} else if (gicp_fail_count >= 1 || gicp_low_acc_count >= 2) {
						localize_thrd_status_.store(4);
					}

					Eigen::Isometry3d curr_lidar_in_map = getLidarInMap();
					Eigen::Isometry3d curr_odom_to_map = getOdomToMap();
					log_info_manager_->slam_info.data[17] = curr_odom_to_map.translation().x();
					log_info_manager_->slam_info.data[18] = curr_odom_to_map.translation().y();
					log_info_manager_->slam_info.data[4] = fit_score;
					log_info_manager_->slam_info.data[5] = gicp_fail_count;
					log_info_manager_->slam_info.data[6] = gicp_low_acc_count;
				} else {
					wait_time++;
				}
			}
		}

		auto end = std::chrono::steady_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		if (elapsed < period_relocal) {
			std::this_thread::sleep_for(period_relocal - elapsed);
		}
	}
}

void LidarSlam::global_localization_for_sec_mapping_thread() {
	const int frequency = 1.0; // 频率为2Hz
	const std::chrono::milliseconds period(1000 / frequency);
	const auto score_thr = config_param_.re_localization.score_thr;
	const auto global_localize_time_out_thr = config_param_.re_localization.time_out_thr;
	const int global_localize_times = global_localize_time_out_thr * frequency; // 重定位次数
	int global_localize_count = 0; // TODO(jxl): 统计全局重定位失败的次数，重命名

	while (thread_run_ && reseting_ == false) {
		hb_time_thread_secmap_relocalize_.store(rclcpp::Clock().now().seconds());
		auto start = std::chrono::steady_clock::now();
		if (secmap_relocal_thrd_status_.load() == 2) { // 重定位失败
			std::cout << RED << "secmap relocalization failed: time out " << RESET << endl;
		} else {
			if (!globalLocalizationSuccess_) {
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

				if (!UndistortCloudInOdom_ || UndistortCloudInOdom_->points.size() == 0) {
					cout << YELLOW << "secmap relocalizing: cloud empty ... " << RESET << endl;
				} else {
					cout << "point(in use) count: " << UndistortCloudInOdom_->points.size() << endl;
					// PointCloudType::Ptr temp(new PointCloudType());
					// {
					// 	std::unique_lock<std::mutex> lk(mtx_odom_cloud_);
					// 	pcl::copyPointCloud(*(UndistortCloudInOdom_), *temp); // TODO(jxl)：拷贝完后的temp点云没有使用?
					// }

					globalLocalizationSuccess_ = global_localization_->global_localize(
						undistortCloud_, T_odom_lidar_, p_imu_->initial_rotate_, score_thr);
					// TODO(jxl): undistort_cloud， T_odom_lidar在run()线程中，需要加锁，拷贝后再释放
					// initial_rotate_可以拷贝出来

					if (globalLocalizationSuccess_) {
						// m_status_ = M_STANDBY;
						cout << BOLDGREEN << " ======= global Localization Success ======= " << RESET << endl;
						global_localize_count_ = 0;
						secmap_relocal_thrd_status_.store(3); //重定位成功 =============================================
					} else {
						global_localize_count++;
					}
				}
				if (global_localize_count > global_localize_times) {
					cout << RED << "global Localization failed: time out" << RESET << endl;
					// m_status_ = M_RELOCALIZE_FAILED;
					secmap_relocal_thrd_status_.store(2); //重定位失败 =============================================
				}
			}
			// TODO(jxl): sec_mapping模式下，重定位成功后，可以结束线程
			// ...
			// ...
		}
		auto end = std::chrono::steady_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		if (elapsed < period) {
			std::this_thread::sleep_for(period - elapsed);
		}
	}
}

void LidarSlam::lidar_pcl_cbk(const PointCloudType::Ptr cloud) {
	static const int keep_lidar_num_before_curr = config_param_.lidar_preproc.keep_lidar_num_before_curr;
	static const double jump_back_time_thr = -1.0;
	if (reseting_) {
		return;
	}

	// 出现的bug: 系统时间跳变，往回跳零点几秒，程序出错，slam 显示定位正常，但是定位已经不对
	// 处理： 当时间跳回过去，跳变间隔在1秒以内，则跳过这几帧数据

	double curr_time = cloud->header.stamp * 1.0 * 1e-6; // 转换为单位： second
	//在msg2pcl_clip()中，pcl_header.stamp的单位是us

	if (curr_time < last_timestamp_lidar_) {
		cout << "lidar loop back, clear buffer" << endl;
		lidar_buffer_.clear(); // TODO(jxl): 应该只需要pop当前帧就行，而且lidar_buffer应该和imu_buffer使用不同的mtx。
	}
	if (!time_sync_en_ && abs(last_timestamp_imu_ - curr_time) > 10.0 && !imu_buffer_.empty() &&
		!lidar_buffer_.empty()) { // TODO(jxl): 10s的阈值是否有点大， 而且也没做处理
		cout << YELLOW << setprecision(15) << "IMU and LiDAR not Synced, IMU time: " << last_timestamp_imu_
			 << ", lidar header time: " << curr_time << RESET << endl;
		cout << YELLOW << setprecision(15)
			 << "IMU and LiDAR not Synced, imu - lidar time diff: " << last_timestamp_imu_ - curr_time << RESET << endl;
	}
	if (time_sync_en_ && !timediff_set_flg_ && abs(last_timestamp_imu_ - curr_time) > 1 && !imu_buffer_.empty()) {
		timediff_set_flg_ = true;
		timediff_lidar_wrt_imu_ = curr_time + 0.1 - last_timestamp_imu_;
		// cout<<"Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu_<< endl;
	}

	std::unique_lock<std::mutex> lk(mtx_buffer_);
	while (lidar_buffer_.size() > keep_lidar_num_before_curr) {
		lidar_buffer_.pop_front();
		time_buffer_.pop_front();
		// TODO(jxl): 添加警告信息，提示同步那边，或者处理雷达点云有问题，耗时太长
	}
	lidar_buffer_.push_back(cloud); //点云已经在驱动中转换成和base_link朝向一致，原点位置还是雷达安装位置
	time_buffer_.push_back(curr_time);

	last_timestamp_lidar_ = curr_time;
	return;
}

void LidarSlam::imu_cbk(const std::shared_ptr<livox_ros::ImuMsg>& msg_in) {
	static const double jump_back_time_thr = -1.0;
	if (reseting_) {
		return;
	}
	std::shared_ptr<livox_ros::ImuMsg> msg(new livox_ros::ImuMsg(*msg_in));

	// TODO(jxl): 这段代码没有多大作用，确认后删除
	//  if (!config_param_.common.offline_mode) {
	//  	if (!imu_file_shift_) {
	//  		if (temp_imu_msg_.size() > 0) {
	//  			for (auto msg : temp_imu_msg_) {
	//  				imu_file_ << std::fixed << std::setprecision(9) << msg.time_stamp << " " <<
	//  msg.angular_velocity.x()
	//  						  << " " << msg.angular_velocity.y() << " " << msg.angular_velocity.z() << " "
	//  						  << msg.linear_acceleration.x() << " " << msg.linear_acceleration.y() << " "
	//  						  << msg.linear_acceleration.z() << std::endl;
	//  			}
	//  			temp_imu_msg_.clear();
	//  		}
	//  		imu_file_ << std::fixed << std::setprecision(9) << msg->time_stamp << " " << msg->angular_velocity.x()
	//  				  << " " << msg->angular_velocity.y() << " " << msg->angular_velocity.z() << " "
	//  				  << msg->linear_acceleration.x() << " " << msg->linear_acceleration.y() << " "
	//  				  << msg->linear_acceleration.z() << std::endl;
	//  	} else {
	//  		temp_imu_msg_.push_back(*msg);
	//  	}
	//  }

	// lidar 和 imu时间差过大，且开启时间同步, 纠正当前输入imu的时间
	if (abs(timediff_lidar_wrt_imu_) > 0.1 && time_sync_en_) {
		msg->time_stamp = (timediff_lidar_wrt_imu_ + msg_in->time_stamp);
	}
	double curr_timestamp_imu = msg->time_stamp;

	std::unique_lock<std::mutex> lk(mtx_buffer_); // TODO(jxl): 使用imu_buffer自己的mtx, 而且锁范围太大
	if (curr_timestamp_imu < last_timestamp_imu_) {
		cout << YELLOW << "imu loop back, clear buffer" << RESET << endl;
		imu_buffer_.clear();
	} else if (curr_timestamp_imu - last_timestamp_imu_ > 0.15) { // TODO(jxl): imu 200hz，该阈值有点大
		cout << YELLOW << "imu lose rate" << RESET << endl;
	}
	imu_buffer_.push_back(msg); // base_link下的acc，gyro

	last_timestamp_imu_ = curr_timestamp_imu;
	localization_wait_ = true; // TODO(jxl): 这个变量可以不要了

	// TODO(jxl): globalLocalizationSuccess_要加锁，拷贝后释放掉锁
	if (globalLocalizationSuccess_ || working_mode_ == MAPPING || working_mode_ == SEC_MAPPING) {
		if (current_pose_.base_time < localization_base_.base_time) { //(curr_t, localize_t)

			current_pose_ = localization_base_;
			// TODO(jxl): curr_pose只在imu回调中更新。
			// TODO(jxl): 一般来说，不应该会出现curr < localize情况，出现时刻，位姿会跳变，时间戳相差多少？

			for (auto it = imu_buffer_.begin(); it != imu_buffer_.end(); it++) {
				const std::shared_ptr<livox_ros::ImuMsg>& msg = *it;
				if (msg->time_stamp > localization_base_.update_time) {
					double dt = msg->time_stamp - localization_base_.update_time;
					V3D angvel = V3D(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]) -
								 current_pose_.imu_state.bg;
					V3D acc =
						V3D(msg->linear_acceleration[0], msg->linear_acceleration[1], msg->linear_acceleration[2]) *
						G_m_s2 / (p_imu_->mean_acc_.norm()); // msg中acc单位是g
					acc =
						current_pose_.imu_state.rot * (acc - current_pose_.imu_state.ba) + current_pose_.imu_state.grav;
					current_pose_.imu_state.pos += current_pose_.imu_state.vel * dt + 0.5 * acc * dt * dt;
					current_pose_.imu_state.vel += acc * dt;
					current_pose_.imu_state.rot = current_pose_.imu_state.rot * Sophus::SO3d::exp(angvel * dt);
					// TODO(jxl): current_pose的base_time，update_time未更新

					localization_base_.update_time = msg->time_stamp;
					// TODO(jxl): dt是应该更新，但是 localization_base_的时间戳不应该更新。
				}
			}
		} else {													//(localize_t, curr_t)
			if (msg->time_stamp > localization_base_.update_time) { // TODO(jxl): bug，应该是和curr_t作差
				double dt = msg->time_stamp - localization_base_.update_time;
				V3D angvel = V3D(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]) -
							 current_pose_.imu_state.bg;
				V3D acc = V3D(msg->linear_acceleration[0], msg->linear_acceleration[1], msg->linear_acceleration[2]) *
						  G_m_s2 / (p_imu_->mean_acc_.norm()); // msg中acc单位是g
				acc = current_pose_.imu_state.rot * (acc - current_pose_.imu_state.ba) + current_pose_.imu_state.grav;
				current_pose_.imu_state.pos += current_pose_.imu_state.vel * dt + 0.5 * acc * dt * dt;
				current_pose_.imu_state.vel += acc * dt;
				current_pose_.imu_state.rot = current_pose_.imu_state.rot * Sophus::SO3d::exp(angvel * dt);
				// TODO(jxl): current_pose的base_time，update_time未更新

				localization_base_.update_time = msg->time_stamp;
				// TODO(jxl): dt是应该更新，但是 localization_base_的时间戳不应该更新。
			}
		}
	}

	// TODO(jxl): 这段代码没什么用
	poses_buffer_.push_back(std::make_pair(curr_timestamp_imu, getLidarInOdom()));
	if (poses_buffer_.size() > 200) {
		poses_buffer_.pop_front();
	}
	localization_wait_ = false;
}

void LidarSlam::delete_log_file(double keep_time) { // about 100MB pr 60s
	if (pcd_file_.size() < 10) {
		return;
	}
	if (pcd_file_.back() - pcd_file_.front() > keep_time) {
		std::string file = config_param_.common.save_log_dir + std::to_string(pcd_file_.front()) + ".pcd";
		std::remove(file.c_str());
		pcd_file_.pop_front();
	}
	imu_file_shift_ = true;
	// trim_log_file(keep_time / 60.0 * 2 * 1024 * 1024,param.save_log_path +
	// std::string("imu_data.txt"),imu_file_,pcd_file_.front());
	imu_file_shift_ = false;
}

bool LidarSlam::run() { // lio线程
	static const int prm_lidar_no_point_count_thr = config_param_.common.lidar_no_point_count_thr;

	/// 在Measure内，储存当前lidar数据及lidar扫描时间内对应的imu数据序列
	static int frame_num = 0;
	static double aver_time_consu = 0, aver_time_icp = 0, aver_time_incre = 0, aver_time_solve = 0;
	double t0, t1, t2, t3, t4, t5, match_start, solve_start, run_start, run_end, t0_backend, t1_backend, t0_transform,
		t1_transform;

	run_start = omp_get_wtime();

	if (sync_packages(Measures_)) {
		// TODO(jxl): add log sync success

		static double last_lidar_time = Measures_.lidar_beg_time;
		if (flg_first_scan_) {
			first_lidar_time_ = Measures_.lidar_beg_time;  //记录第一帧绝对时间
			p_imu_->first_lidar_time_ = first_lidar_time_; //记录第一帧绝对时间
			flg_first_scan_ = false;
			cout << "***************** flg first scan ********" << endl;
			return false;
		}

		// log_info_manager_->slam_info.data[13]= -100;
		// log_info_manager_->slam_info.data[15]= -100;
		log_info_manager_->slam_info.data[24] = Measures_.lidar_beg_time - last_lidar_time;
		log_info_manager_->slam_info.data[25] = Measures_.lidar_beg_time;
		log_info_manager_->slam_info.data[26] = Measures_.lidar_end_time - Measures_.lidar_beg_time;
		log_info_manager_->slam_info.data[27] = Measures_.imu.front()->time_stamp - Measures_.lidar_beg_time;
		log_info_manager_->slam_info.data[28] = Measures_.imu.back()->time_stamp - Measures_.lidar_beg_time;
		last_lidar_time = Measures_.lidar_beg_time;

		t0 = omp_get_wtime();
		// 根据imu数据序列和lidar数据，向前传播纠正点云的畸变, 此前已经完成间隔采样或特征提取
		{
			std::unique_lock<std::mutex> lk(mtx_lidar_cloud_);
			undistortCloud_->clear();
			p_imu_->Process(Measures_, kf_, undistortCloud_); //雷达points, 在最后一个点时刻的laser_frame下
			log_info_manager_->slam_info.data[11] = undistortCloud_->size();
		}
		state_ikfom state_point;
		state_point = kf_.get_x(); // 滤波器predict的是状态是，每一imu时刻，imu frame在imu_0_frame(odom)下的状态
		Eigen::Isometry3d T_b_lidar(Sophus::SE3d(state_point.offset_R_L_I, state_point.offset_T_L_I)
										.matrix()); // T_imu_laser, laser frame wrt imu
		Eigen::Isometry3d T_odom_b(Sophus::SE3d(state_point.rot, state_point.pos).matrix());
		{
			std::unique_lock<std::mutex> lk(mtx_pose_);
			T_odom_lidar_ = T_odom_b * T_b_lidar;
		}

		t1 = omp_get_wtime();
		if (undistortCloud_->empty() || (undistortCloud_ == nullptr)) {
			lidar_no_point_count_++;
			if (lidar_no_point_count_ > prm_lidar_no_point_count_thr) {
				slam_run_status_.store(2);
			}
			cout << YELLOW << "No point, skip this scan!" << RESET << endl;
			log_info_manager_->slam_info.data[15] = lidar_no_point_count_;
			log_info_manager_->slam_info.data[13] = 0;
			return false;
		}

		// 检查当前lidar数据时间，与最早lidar数据时间是否足够
		bool flg_EKF_inited = (Measures_.lidar_beg_time - first_lidar_time_) < 0.1 ? false : true;

		/*** Segment the map in lidar FOV ***/
		ikdtree_->lasermap_fov_segment(T_odom_lidar_.translation());
		// 动态调整局部地图,在拿到eskf前馈结果后, 根据lidar在W系下的位置，重新确定局部地图的包围盒角点，移除远端的点

		t2 = omp_get_wtime();
		downSizeFilterCloud_.setInputCloud(undistortCloud_);
		downSizeFilterCloud_.filter(*FilteredUndistortCloud_);

		int feats_down_size = FilteredUndistortCloud_->points.size(); //当前帧降采样后点数
		log_info_manager_->slam_info.data[13] = feats_down_size;
		PointCloudType::Ptr FilteredUndistortCloudInOdom(new PointCloudType());
		double filter_time = omp_get_wtime();

		/*** initialize the map kdtree ***/
		if (ikdtree_->Root_Node == nullptr) {
			if (feats_down_size > feats_down_size_thr_) {
				ikdtree_->set_downsample_param(config_param_.ikdtree.map_leaf_size);
				ikdtree_->set_cube_len(config_param_.ikdtree.cube_len);
				ikdtree_->set_det_range(config_param_.ikdtree.det_range);

				FilteredUndistortCloudInOdom->resize(feats_down_size);
				FilteredUndistortCloudInOdom =
					transformPointCloud(FilteredUndistortCloud_, T_odom_lidar_); // point转到odom系下
				ikdtree_->Build(
					FilteredUndistortCloudInOdom->points); // world系下对当前帧降采样后的点云，初始化lkd-tree
			}
			cout << GREEN << "initiate  ikdtree_! " << feats_down_size << RESET << endl;
			return false;
		}

		int featsFromMapNum = ikdtree_->validnum();
		int kdtree_size_st = ikdtree_->size();
		if (feats_down_size < feats_down_size_thr_) {
			lidar_no_point_count_++;
			if (lidar_no_point_count_ > prm_lidar_no_point_count_thr) {
				slam_run_status_.store(2);
			}
			log_info_manager_->slam_info.data[15] = lidar_no_point_count_;
			cout << YELLOW << "No point after filter, skip this scan!" << RESET << endl;
			return false;
		} else {
			lidar_no_point_count_ = 0;
			log_info_manager_->slam_info.data[15] = lidar_no_point_count_;
			slam_run_status_.store(1);
		}
		FilteredUndistortCloudInOdom->resize(feats_down_size);

		/* if (true) // If you need to see map point, change to "if(true)" //zx delete this publish
		{
			PointVector().swap(ikdtree_.PCL_Storage);
			ikdtree_.flatten(ikdtree_.Root_Node, ikdtree_.PCL_Storage, NOT_RECORD);
			kdtreeCloud_->clear();
			kdtreeCloud_->points = ikdtree_.PCL_Storage;
			// publish_map(pubLaserCloudMap);
		}*/

		vector<PointVector> Nearest_Points;
		Nearest_Points.resize(feats_down_size);

		/*** iterated state estimation ***/
		double t_update_start = omp_get_wtime();
		kf_.update_iterated_dyn_share_modified(0.001, FilteredUndistortCloud_, *ikdtree_, Nearest_Points, 4,
											   false); //迭代4次
		double t_update_end = omp_get_wtime();
		state_point = kf_.get_x();

		T_b_lidar = Sophus::SE3d(state_point.offset_R_L_I, state_point.offset_T_L_I).matrix();
		T_odom_b = Sophus::SE3d(state_point.rot, state_point.pos).matrix(); // b: 指的论文中的body，imu系
		T_odom_lidar_ = T_odom_b * T_b_lidar;

		localization_base_.imu_state = state_point;		//滤波器估计的imu的状态，更新localization_base
		localization_base_.base_time = lidar_end_time_; // TODO(jxl): base_time多余
		localization_base_.update_time = lidar_end_time_;

		t0_backend = omp_get_wtime();
		if (working_mode_ == MAPPING || working_mode_ == SEC_MAPPING) {
			if (working_mode_ == SEC_MAPPING && !globalLocalizationSuccess_) {
				cout << "Waiting for global localization success ..." << endl;
			} else if (working_mode_ == SEC_MAPPING && !back_end_->get_loaded_key_cloud_status()) {
				cout << "Waiting for loading key cloud done ..." << endl;
			} else {
				loop_closure_wait_ = true; // TODO(jxl): 该变量没有什么用处

				bool insert = back_end_->saveKeyFramesAndFactor(T_odom_lidar_, undistortCloud_, lidar_end_time_);
				if (insert) { //是关键帧
					cout << YELLOW << "************************* backend: keyPosesCount: "
						 << back_end_->getKeyframePoses().size() - 1 << RESET << endl;

					back_end_->saveCurrentCloud(undistortCloud_, getLidarInMap()); //保存当前帧点云的信息到scManager_中
					//注意这里只是为了取水平面，后端还是在odom坐标系

					{
						std::unique_lock<std::mutex> lk(mtx_path_);
						unoptimized_path_.emplace_back(getWheelInMap());
						if (unoptimized_path_.size() > 200) {
							unoptimized_path_.pop_front();
						}
					}

					// TODO(jxl): 苗苗让后端不维护T_map_odom, 还是保持I，但是不可能啊
					T_odom_lidar_ = back_end_->getCurrentPose().pose; // curr keyframe in map，是带了回环优化后的pose
					T_odom_b = T_odom_lidar_ * T_b_lidar.inverse();
					state_ikfom state_updated = kf_.get_x();
					state_updated.pos = T_odom_b.translation();
					state_updated.rot = Sophus::SO3d(T_odom_b.rotation());
					kf_.change_x(state_updated); // TODO(jxl): 强行把滤波器的状态改变，逻辑上说不通

					new_key_cloud_arrived_ = true;
				}

				// TODO(jxl): 不是关键帧也会运行下面的逻辑
				//...

				// 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿，更新里程计轨迹， 重构ikdtree
				bool LoopIsClosed = back_end_->correctPoses();
				// TODO(jxl): correct_pose函数应该放到saveKeyFramesAndFactor()中去

				{
					std::unique_lock<std::mutex> lk(mtx_path_);
					optimized_path_.clear();
					auto lidar_in_odom = back_end_->getKeyframePoses();
					for (int i = 0; i < lidar_in_odom.size(); i++) {
						optimized_path_.emplace_back(getOdomToMap() * lidar_in_odom[i].pose * T_lidar_wheel_);
					}
				}
				if (LoopIsClosed) { // TODO(jxl):
									// 更新前端的local_map，这么做讲不通啊，前端应该只提供odom_pose，后端来通过闭环检测和优化来维护T_map_odom
					back_end_->recontructIKdTree(*ikdtree_, config_param_.ikdtree.kdTreeReconstructRadius,
												 config_param_.ikdtree.kdTreeReconstructKeyFrameLeafSize,
												 config_param_.ikdtree.kdTreeReconstructPointLeafSize);
				}
				loop_closure_wait_ = false;
			}

		} else if (working_mode_ == LOCALIZATION) {
			{
				std::unique_lock<std::mutex> lk(mtx_path_);
				unoptimized_path_.emplace_back(getWheelInMap()); // TODO(jxl): 定位模式下，这个T_map_baselink
				if (unoptimized_path_.size() > 200) {
					unoptimized_path_.pop_front();
				}
			}
		}
		t1_backend = omp_get_wtime();

		t0_transform = omp_get_wtime();
		{
			std::unique_lock<std::mutex> lk(mtx_odom_cloud_); // TODO(jxl): undistortCloud的锁
			UndistortCloudInOdom_->resize(undistortCloud_->points.size());
			UndistortCloudInOdom_ = transformPointCloud(undistortCloud_, T_odom_lidar_);
		}
		t1_transform = omp_get_wtime();

		t3 = omp_get_wtime();
		FilteredUndistortCloudInOdom = transformPointCloud(FilteredUndistortCloud_, T_odom_lidar_);

		t4 = omp_get_wtime();
		ikdtree_->map_incremental(FilteredUndistortCloudInOdom, Nearest_Points,
								  flg_EKF_inited); // add the feature points to map kdtree
		// TODO(jxl): 前端的local_map更新应该在这, 对照原来fast-lio2的代码
		//...
		//..
		t5 = omp_get_wtime();

		{
			frame_num++;
			int kdtree_size_end = ikdtree_->size();
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
			// ROS_INFO_STREAM("p_imu_->Process, cloud deskew    , time cost: " << (t1-t0)*1000 << " ms;");
			// ROS_INFO_STREAM("ikdtree_->lasermap_fov_segment   , time cost: " << (t2-t1)*1000 << " ms;");
			// ROS_INFO_STREAM("lidar slam main process step1   , time cost: " << (t0_backend-t2)*1000 << " ms;");
			// ROS_INFO_STREAM("lidar slam main update  time    , time cost: " << (t_update_end-t_update_start)*1000 <<
			// " ms;");
			// ROS_INFO_STREAM("lidar slam main process         , time cost: " << (t3-t2)*1000 << " ms;");
			// ROS_INFO_STREAM("main: lidar slam backend        , time cost: " << (t1_backend-t0_backend)*1000 << "
			// ms;");
			// ROS_INFO_STREAM("main: transform undistortCloud_  , time cost: " <<
			// (t1_transform-t0_transform)*1000 << " ms;");
			// ROS_INFO_STREAM("transform FilteredUndistortCloud_, time
			// cost: " << (t4-t3)*1000 << " ms;");
			// ROS_INFO_STREAM("ikdtree_->map_incremental        , time cost: " <<
			// (t5-t4)*1000 << " ms;");
			// ROS_INFO_STREAM("feats_down_size: " << feats_down_size);
			// ROS_INFO_STREAM("-------------------------------------------------- " );
		} else if (run_end - run_start > 0.07) {
			cout << YELLOW << "lidar-slam    , time cost: " << (run_end - run_start) * 1000 << " ms -------------------"
				 << RESET << endl;
			// ROS_INFO_STREAM("p_imu_->Process, cloud deskew    , time cost: " << (t1-t0)*1000 << " ms;");
			// ROS_INFO_STREAM("ikdtree_->lasermap_fov_segment   , time cost: " << (t2-t1)*1000 << " ms;");
			// ROS_INFO_STREAM("lidar slam main process step1   , time cost: " << (t0_backend-t2)*1000 << " ms;");
			// ROS_INFO_STREAM("lidar slam main update  time    , time cost: " << (t_update_end-t_update_start)*1000 <<
			// " ms;");
			// ROS_INFO_STREAM("lidar slam main process         , time cost: " << (t3-t2)*1000 << " ms;");
			// ROS_INFO_STREAM("main: lidar slam backend        , time cost: " << (t1_backend-t0_backend)*1000 << "
			// ms;");
			// ROS_INFO_STREAM("main: transform undistortCloud_  , time cost: " <<
			// (t1_transform-t0_transform)*1000 << " ms;");
			// ROS_INFO_STREAM("transform FilteredUndistortCloud_, time
			// cost: " << (t4-t3)*1000 << " ms;");
			// ROS_INFO_STREAM("ikdtree_->map_incremental        , time cost: " <<
			// (t5-t4)*1000 << " ms;");
			// ROS_INFO_STREAM("feats_down_size: " << feats_down_size);
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
		// delete_log_file(config_param_.common.log_keep_time);

		// TODO(jxl): 同步失败，应该reset Measures_
	}

	return false;
}
} // namespace lidar_slam

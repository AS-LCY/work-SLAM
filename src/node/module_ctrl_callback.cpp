
#include "localization_module.h"
#include "node/localization_module.h"

namespace localization_module {
void LocalizationModule::localization_module_ctrl_callback(const std_msgs::msg::UInt32::SharedPtr msg_in) {
	if (slam_param_.common.cpu_id.size() > 0) {
		pthread_t this_thread = pthread_self(); // 获取当前线程的 ID
		if (pthread_setaffinity_np(this_thread, sizeof(cpu_mask_), &cpu_mask_) < 0) {
			perror("pthread_setaffinity_np");
			exit(EXIT_FAILURE);
		}
	}
	hb_time_cbk_module_ctrl_.store(node_->now().seconds());

	/** msg_in *************************************************************************************
	 * enum SlamCtrlCmd{
	 *     START_MAPPING           = 1000,  // 开始建图
	 *     START_SEC_MAPPING       = 2000,  // 重定位->建图，二次建图
	 *     CANCLE_MAPPING          = 5000,  // 不保存地图， 直接取消建图
	 *     SAVE_AND_END_MAPPING    = 6000,  // 保存地图，退出建图
	 *     START_LOCALIZATION      = 7000,  // localization, 重定位->定位
	 *     EXIT_LOCALIZATION       = 8000,  // exit localization, 退出定位
	 *     START_RELOCALIZATION    = 9000,  // relocalization, 重定位，定位过程中，重新进行重定位
	 *     RESTART_SEC_MAPPING     = 9100,  // restart sec-mapping, 重启二次建图（一般是二次建图重定位失败的情况）
	 *     CMD_MAX                 = 9999
	 *     [MAPPING_POINT_BEGIN]     = 3000,  // invalid, 设置起点
	 *     [MAPPING_ELE_DELETE ]     = 4000,  // invalid, 创建地图元素过程中，清除当前元素（当前元素还未完成创建）
	 *     [MAPPING_POINT_END  ]     = 5000,  // invalid, 设置终点，带子地图ID，5001，ID=1
	 * };
	 *
	 * to be continued
	 */
	//// topic-name: "/flbot/localization_module/ctrl_cmd" *****************************************

	auto msg = msg_in;
	int ctrl_type = msg->data / 100 * 100;
	auto curr_cmd = static_cast<SlamCtrlCmd>(ctrl_type);
	std::cout << BOLDGREEN << "Received Ctrl Msg: " << msg->data << RESET << std::endl;
	std::cout << BOLDGREEN << "Received Ctrl Cmd: " << print_SlamCtrlCmd(curr_cmd) << RESET << std::endl;

	int map_id = msg->data % 100;
	if (map_id == 0) {
		map_id = 1;
	}

	switch (curr_cmd) {
		case START_MAPPING: {			 // 初次建图，或重置后建图
			if (start_mapping(map_id)) { //启动建图成功
				std::cout << GREEN << "start_mapping success!" << RESET << std::endl;
			} else { //启动建图失败
				std::cout << RED << "start_mapping failed!" << RESET << std::endl;
			}
			break;
		}
		case START_SEC_MAPPING: { // 重定位，并开始建图
			if (start_second_mapping(map_id)) {
				std::cout << GREEN << "start_second_mapping success!" << RESET << std::endl;
			} else {
				std::cout << RED << "Start Sec-mapping failed!" << RESET << std::endl;
				release_slam_obj();
				std::cout << YELLOW << "slam obj destroyed!" << RESET << std::endl;
			}
			break;
		}
		case CANCLE_MAPPING: { // 不保存地图， 直接退出建图
			if (stop_mapping_without_saving_map()) {
				std::cout << GREEN << "stop_mapping(not saving map) success!" << RESET << std::endl;
			} else {
				std::cout << RED << "stop_mapping failed!" << RESET << std::endl;
			}
			break;
		}
		case SAVE_AND_END_MAPPING: { // 保存地图，并结束建图
			if (stop_mapping()) {
				std::cout << GREEN << "stop_mapping success!" << RESET << std::endl;
			} else {
				std::cout << RED << "stop_mapping failed!" << RESET << std::endl;
			}
			break;
		}
		case START_LOCALIZATION: { // 开始定位，（先重定位，再定位）
			if (start_localization(map_id)) {
				std::cout << GREEN << "start_localization success!" << RESET << std::endl;
			} else {
				std::cout << RED << "start_localization failed!" << RESET << std::endl;
			}
			break;
		}
		case EXIT_LOCALIZATION: {	   // 退出定位
			if (stop_localization()) { // 停止定位成功
				std::cout << GREEN << "stop_localization success!" << RESET << std::endl;
			} else {
				std::cout << RED << "stop_localization failed!" << RESET << std::endl;
			}
			break;
		}
		case START_RELOCALIZATION: { // 重新进行重定位
			if (start_relocalization(map_id)) {
				RCLCPP_INFO(node_->get_logger(), "restart localization successfully!");
			} else {
				std::cout << RED << "start_relocalization failed!" << RESET << std::endl;
			}
			break;
		}
		case RESTART_SEC_MAPPING: { // 重新进行重定位
			if (restart_second_mapping(map_id)) {
				RCLCPP_INFO(node_->get_logger(), "restart sec_mapping successfully!");
			} else {
				std::cout << RED << "restart sec_mapping failed!" << RESET << std::endl;
			}
			break;
		}
		default: {
			RCLCPP_INFO(node_->get_logger(), "mapping ctrl msg: %u invalid!", msg->data);
			break;
		}
	}
}

bool LocalizationModule::start_mapping(int map_id) {
	auto set_status = ModuleStatus::MODULE_MAPPING;
	auto running_module_status_now = running_module_status_.load();
	RCLCPP_INFO(node_->get_logger(), "Module Status for now: %s",
				print_ModuleStatus(running_module_status_now).c_str());
	RCLCPP_INFO(node_->get_logger(), "Trying to set module_status: %s", print_ModuleStatus(set_status).c_str());

	if (!make_map_directory_name(map_id)) { // 基于 map_id, 保存在 [slam_param_.common.map_directory]
		std::cout << "Map Directory Error!" << std::endl;
		exit(EXIT_FAILURE);
	}
	if (running_module_status_now == ModuleStatus::MODULE_IDLE) {
		running_module_status_.store(ModuleStatus::MODULE_STARTING_SLAM);
		make_slam_obj(slam_param_, set_status);
		running_module_status_.store(ModuleStatus::MODULE_MAPPING);
		mapping_node_status_.store(1); // 1: normal
		return true;
	} else if (is_mapping_status(running_module_status_now)) {
		RCLCPP_INFO(node_->get_logger(), "skip, already running mapping now");
		return false;
	} else if (running_module_status_now == ModuleStatus::MODULE_LOCALIZATION) {
		RCLCPP_INFO(node_->get_logger(), "skip, running localizing now, please stop localizing first");
		return false;
	} else {
		RCLCPP_INFO(node_->get_logger(), "skip, status error!");
		RCLCPP_INFO(node_->get_logger(), "running_module_status_now: %s",
					print_ModuleStatus(running_module_status_now).c_str());
		RCLCPP_INFO(node_->get_logger(), "set_module_status: %s", print_ModuleStatus(set_status).c_str());
		RCLCPP_INFO(node_->get_logger(), "mapping_status: %d", mapping_status_.load());
		RCLCPP_INFO(node_->get_logger(), "****************************");
		return false;
	}
}

bool LocalizationModule::start_second_mapping(int map_id) {
	auto running_module_status_now = running_module_status_.load();
	ModuleStatus set_status = ModuleStatus::MODULE_SEC_MAPPING;

	RCLCPP_INFO(node_->get_logger(), "Module Status for now: %s",
				print_ModuleStatus(running_module_status_now).c_str());
	RCLCPP_INFO(node_->get_logger(), "Trying to set module_status: %s", print_ModuleStatus(set_status).c_str());

	if (!make_map_directory_name(map_id)) { // 基于 map_id, 保存在 [slam_param_.common.map_directory]
		std::cout << "Map Directory Error!" << std::endl;
		exit(EXIT_FAILURE);
	}
	std::string load_map_dir = slam_param_.common.cloud_map_directory;

	if (running_module_status_now == ModuleStatus::MODULE_IDLE) {
		running_module_status_.store(ModuleStatus::MODULE_STARTING_SLAM);
		make_slam_obj(slam_param_, set_status);
		if (!slam_->load_map(load_map_dir)) {
			std::cout << RED << "load map failed!" << RESET << std::endl;

			usleep(500000); // 单位: 微秒, 500,000us = 500ms = 0.5s
			release_slam_obj();
			RCLCPP_INFO(node_->get_logger(), "slam destoried!");

			// update module-status
			running_module_status_.store(ModuleStatus::MODULE_IDLE);

			return false;
		} else { // start_second_mapping success
			running_module_status_.store(ModuleStatus::MODULE_SEC_MAPPING);
			mapping_node_status_.store(1); // 1: normal
			return true;
		}
	} else if (is_mapping_status(running_module_status_now)) {
		RCLCPP_INFO(node_->get_logger(), "skip, already running mapping now");
		return false;
	} else if (running_module_status_now == ModuleStatus::MODULE_LOCALIZATION) {
		RCLCPP_INFO(node_->get_logger(), "skip, running localizing now, please stop localizing first");
		return false;
	} else {
		RCLCPP_INFO(node_->get_logger(), "skip, status error!");
		RCLCPP_INFO(node_->get_logger(), "running_module_status_now: %s",
					print_ModuleStatus(running_module_status_now).c_str());
		RCLCPP_INFO(node_->get_logger(), "set_module_status: %s", print_ModuleStatus(set_status).c_str());
		RCLCPP_INFO(node_->get_logger(), "mapping_status: %d", mapping_status_.load());
		RCLCPP_INFO(node_->get_logger(), "****************************");
		return false;
	}
}

bool LocalizationModule::stop_mapping_without_saving_map() {
	auto running_module_status_now = running_module_status_.load();
	auto set_status = ModuleStatus::MODULE_IDLE;
	if (is_mapping_status(running_module_status_now)) {
		running_module_status_.store(ModuleStatus::MODULE_STOPPING_SLAM);
		usleep(500000); // 单位: 微秒, 500,000us = 500ms = 0.5s
		release_slam_obj();
		RCLCPP_INFO(node_->get_logger(), "mapping stopped without saving map!");

		running_module_status_.store(ModuleStatus::MODULE_IDLE);
		mapping_node_status_.store(0); // 0: inactive

		return true;
	} else if (running_module_status_now == ModuleStatus::MODULE_IDLE ||
			   running_module_status_now == ModuleStatus::MODULE_LOCALIZATION) {
		RCLCPP_INFO(node_->get_logger(), "skip, can not stop mapping, running_module_status_: %s",
					print_ModuleStatus(running_module_status_now).c_str());
		return false;
	} else {
		RCLCPP_INFO(node_->get_logger(), "skip, status error!");
		RCLCPP_INFO(node_->get_logger(), "running_module_status_now: %s",
					print_ModuleStatus(running_module_status_now).c_str());
		RCLCPP_INFO(node_->get_logger(), "set_module_status: %s", print_ModuleStatus(set_status).c_str());
		RCLCPP_INFO(node_->get_logger(), "mapping_status: %d", mapping_status_.load());
		RCLCPP_INFO(node_->get_logger(), "localization_status: %d", localization_status_.load());
		RCLCPP_INFO(node_->get_logger(), "****************************");
		return false;
	}
}

bool LocalizationModule::stop_mapping() {
	// TODO: 问题：当点云量为0时，程序挂掉
	auto running_module_status_now = running_module_status_.load();
	auto mapping_status_now = mapping_status_.load();
	ModuleStatus set_status = ModuleStatus::MODULE_IDLE;
	if (is_mapping_status(running_module_status_now)) {
		if (mapping_status_now == 3) { // m_standby
			running_module_status_.store(ModuleStatus::MODULE_STOPPING_SLAM);
			RCLCPP_INFO(node_->get_logger(), "mapping_status: %d", mapping_status_.load());
			RCLCPP_INFO(node_->get_logger(), "\033[1;32mstart saving map data\033[0m");
			std::string pcd_dir = slam_param_.common.cloud_map_directory;
			const auto resolution = slam_param_.mapping.save_map_resolution;
			if (!slam_->save_map(pcd_dir, resolution, 0, 0)) {
				std::cout << RED << "save map data failed!" << RESET << std::endl;
			} else {
				RCLCPP_INFO(node_->get_logger(), "\033[1;32msave map data success!\033[0m");
			}

			save_extrinsic_to_file();

			map_saved_.store(1);
			std::cout << YELLOW << "[Slam ctrl]: map_saved_: " << map_saved_.load() << RESET << std::endl;

			RCLCPP_INFO(node_->get_logger(), "start stop mapping");
			sleep(1);
			// set_module_status_ = ModuleStatus::MODULE_IDLE;
			release_slam_obj();
			RCLCPP_INFO(node_->get_logger(), "mapping stopped !");
			running_module_status_.store(ModuleStatus::MODULE_IDLE);
			mapping_node_status_.store(0); // 0: inactive

			return true;
		} else if (mapping_status_now == 4) { // m_creating_ele (not used)
			RCLCPP_INFO(node_->get_logger(), "mapping_status: %d", mapping_status_.load());
			RCLCPP_INFO(node_->get_logger(), "skip, please finish current map-element, or delete it first !");
			return false;
		} else {
			RCLCPP_INFO(node_->get_logger(), "skip, status error!");
			RCLCPP_INFO(node_->get_logger(), "set_module_status: %s", print_ModuleStatus(set_status).c_str());
			RCLCPP_INFO(node_->get_logger(), "running_module_status_now: %s",
						print_ModuleStatus(running_module_status_now).c_str());
			RCLCPP_INFO(node_->get_logger(), "mapping_status: %d", mapping_status_.load());
			RCLCPP_INFO(node_->get_logger(), "localization_status: %d", localization_status_.load());
			RCLCPP_INFO(node_->get_logger(), "****************************");
			return false;
		}
	} else if (running_module_status_now == ModuleStatus::MODULE_IDLE ||
			   running_module_status_now == ModuleStatus::MODULE_LOCALIZATION) {
		RCLCPP_INFO(node_->get_logger(), "skip, can not stop mapping, running_module_status_: %s",
					print_ModuleStatus(running_module_status_.load()).c_str());
		return false;
	} else {
		RCLCPP_INFO(node_->get_logger(), "skip, status error!");
		RCLCPP_INFO(node_->get_logger(), "running_module_status_now: %s",
					print_ModuleStatus(running_module_status_now).c_str());
		RCLCPP_INFO(node_->get_logger(), "set_module_status: %s", print_ModuleStatus(set_status).c_str());
		RCLCPP_INFO(node_->get_logger(), "mapping_status: %d", mapping_status_.load());
		RCLCPP_INFO(node_->get_logger(), "localization_status: %d", localization_status_.load());
		RCLCPP_INFO(node_->get_logger(), "****************************");
		return false;
	}
}

bool LocalizationModule::save_extrinsic_to_file() {
	std::string extrinsic_file_name = slam_param_.common.cloud_map_directory + "/extrinsic.txt";
	std::ofstream extrinsic_file(extrinsic_file_name);
	if (!extrinsic_file.is_open()) {
		std::cout << "open extrinsic file failed!" << std::endl;
		return false;
	}

	extrinsic_file << "extrinsic_euler_lidar_in_baselink: \n";
	extrinsic_file << "  yaw:   " << slam_param_.extrinsic.yaw_pitch_roll_deg[0] << " degree, \n";
	extrinsic_file << "  pitch: " << slam_param_.extrinsic.yaw_pitch_roll_deg[1] << " degree, \n";
	extrinsic_file << "  roll:  " << slam_param_.extrinsic.yaw_pitch_roll_deg[2] << " degree, \n";
	extrinsic_file.close();
	return true;
}

bool LocalizationModule::start_localization(int map_id) {
	ModuleStatus running_module_status_now = running_module_status_.load();
	ModuleStatus set_status = ModuleStatus::MODULE_LOCALIZATION;
	auto localization_status_now = localization_status_.load();

	RCLCPP_INFO(node_->get_logger(), "Module Status for now: %s",
				print_ModuleStatus(running_module_status_now).c_str());
	RCLCPP_INFO(node_->get_logger(), "Trying to set module_status: %s", print_ModuleStatus(set_status).c_str());

	if (!make_map_directory_name(map_id)) { // 基于 map_id, 保存在 [slam_param_.common.map_directory]
		std::cout << "Map Directory Error!" << std::endl;
		exit(EXIT_FAILURE);
	}
	std::string load_map_dir = slam_param_.common.cloud_map_directory;
	std::cout << "load_map_dir: " << slam_param_.common.cloud_map_directory << std::endl;

	// ModuleStatus curr_running_module_status = running_module_status_.load();

	if (need_start_localization(running_module_status_now, localization_status_now)) {
		// update status
		running_module_status_.store(ModuleStatus::MODULE_STARTING_SLAM);

		make_slam_obj(slam_param_, set_status);

		if (!slam_->load_map(load_map_dir)) {
			std::cout << RED << "load map failed!" << RESET << std::endl;
			release_slam_obj();

			running_module_status_.store(ModuleStatus::MODULE_IDLE);
			local_node_status_.store(0); // 0 = INACTIVE
			return false;
		} else {
			show_load_map_ = 0;
			// update status
			running_module_status_.store(ModuleStatus::MODULE_LOCALIZATION);
			local_node_status_.store(1); // 1 = NORMAL
			return true;
		}
	} else if (running_module_status_now == ModuleStatus::MODULE_LOCALIZATION &&
			   (localization_status_is_ok(localization_status_now))) {
		RCLCPP_INFO(node_->get_logger(), "skip, already running localization normally now");
		return false;
	} else if (is_mapping_status(running_module_status_now)) {
		RCLCPP_INFO(node_->get_logger(), "skip, running mapping now, please stop mapping first");
		return false;
	} else {
		RCLCPP_INFO(node_->get_logger(), "skip, status error! start localization failed !");
		RCLCPP_INFO(node_->get_logger(), "running_module_status_now: %s",
					print_ModuleStatus(running_module_status_now).c_str());
		RCLCPP_INFO(node_->get_logger(), "set_module_status: %s", print_ModuleStatus(set_status).c_str());
		RCLCPP_INFO(node_->get_logger(), "running_module_status_now: %s",
					print_ModuleStatus(running_module_status_now).c_str());
		RCLCPP_INFO(node_->get_logger(), "mapping_status: %d", mapping_status_.load());
		RCLCPP_INFO(node_->get_logger(), "****************************");
		return false;
	}
}

bool LocalizationModule::stop_localization() {
	auto running_module_status_now = running_module_status_.load();
	auto set_status = ModuleStatus::MODULE_LOCALIZATION;

	if (running_module_status_now == ModuleStatus::MODULE_LOCALIZATION) {
		// update module-status
		running_module_status_.store(ModuleStatus::MODULE_STOPPING_SLAM);

		usleep(500000); // 单位: 微秒, 500,000us = 500ms = 0.5s
		release_slam_obj();
		RCLCPP_INFO(node_->get_logger(), "localization stopped !");

		// update module-status
		running_module_status_.store(ModuleStatus::MODULE_IDLE);
		local_node_status_.store(0);

		return true;
	} else if (running_module_status_now == ModuleStatus::MODULE_IDLE ||
			   running_module_status_now == ModuleStatus::MODULE_MAPPING ||
			   running_module_status_now == ModuleStatus::MODULE_SEC_MAPPING) {
		RCLCPP_INFO(node_->get_logger(), "skip, can not stop localization, running_module_status_: %s",
					print_ModuleStatus(running_module_status_now).c_str());
		return false;
	} else {
		RCLCPP_INFO(node_->get_logger(), "skip, status error!");
		RCLCPP_INFO(node_->get_logger(), "last_running_module_status_: %s",
					print_ModuleStatus(running_module_status_now).c_str());
		RCLCPP_INFO(node_->get_logger(), "set_module_status: %s", print_ModuleStatus(set_status).c_str());
		RCLCPP_INFO(node_->get_logger(), "running_module_status_: %s",
					print_ModuleStatus(running_module_status_now).c_str());
		RCLCPP_INFO(node_->get_logger(), "mapping_status: %d", mapping_status_.load());
		RCLCPP_INFO(node_->get_logger(), "localization_status: %d", localization_status_.load());
		RCLCPP_INFO(node_->get_logger(), "****************************");
		return false;
	}
}

bool LocalizationModule::start_relocalization(int map_id) {
	ModuleStatus running_module_status_now = running_module_status_.load();
	if (running_module_status_now == ModuleStatus::MODULE_LOCALIZATION) {
		// last_running_module_status_ = running_module_status_;
		// set_module_status_ = ModuleStatus::MODULE_IDLE;
		// running_module_status_.store(ModuleStatus::MODULE_STOPPING_SLAM);
		if (!stop_localization()) {
			return false;
		} else {
			if (!start_localization(map_id)) {
				return false;
			} else {
				return true;
			}
		}
	} else if (running_module_status_now == ModuleStatus::MODULE_IDLE ||
			   running_module_status_now == ModuleStatus::MODULE_MAPPING) {
		RCLCPP_INFO(node_->get_logger(), "skip, can not stop localization, running_module_status_: %s",
					print_ModuleStatus(running_module_status_now).c_str());
		return false;
	} else {
		RCLCPP_INFO(node_->get_logger(), "skip, status error!");
		RCLCPP_INFO(node_->get_logger(), "running_module_status_: %s",
					print_ModuleStatus(running_module_status_now).c_str());
		RCLCPP_INFO(node_->get_logger(), "mapping_status: %d", mapping_status_.load());
		RCLCPP_INFO(node_->get_logger(), "localization_status: %d", localization_status_.load());
		RCLCPP_INFO(node_->get_logger(), "****************************");
		return false;
	}
}

bool LocalizationModule::restart_second_mapping(int map_id) {
	ModuleStatus curr_running_module_status = running_module_status_.load();

	if (curr_running_module_status == ModuleStatus::MODULE_SEC_MAPPING) {
		if (!stop_mapping_without_saving_map()) {
			return false;
		} else {
			if (!start_second_mapping(map_id)) {
				return false;
			} else {
				return true;
			}
		}

	} else if (curr_running_module_status == ModuleStatus::MODULE_IDLE ||
			   curr_running_module_status == ModuleStatus::MODULE_MAPPING ||
			   curr_running_module_status == ModuleStatus::MODULE_LOCALIZATION) {
		RCLCPP_INFO(node_->get_logger(), "skip, can not restart sec_mapping, running_module_status_: %s",
					print_ModuleStatus(curr_running_module_status).c_str());
		return false;
	} else {
		RCLCPP_INFO(node_->get_logger(), "skip, status error!");
		RCLCPP_INFO(node_->get_logger(), "running_module_status_: %s",
					print_ModuleStatus(curr_running_module_status).c_str());
		RCLCPP_INFO(node_->get_logger(), "mapping_status: %d", mapping_status_.load());
		RCLCPP_INFO(node_->get_logger(), "localization_status: %d", localization_status_.load());
		RCLCPP_INFO(node_->get_logger(), "****************************");
		return false;
	}
}

bool LocalizationModule::make_slam_obj(lidar_slam::LidarSlamParam yaml_param, ModuleStatus set_status) {
	lidar_slam::SlamWorkMode set_slam_mode = lidar_slam::SlamWorkMode::UNKNOWN;
	if (set_status == ModuleStatus::MODULE_MAPPING) {
		set_slam_mode = lidar_slam::SlamWorkMode::MAPPING;
	} else if (set_status == ModuleStatus::MODULE_SEC_MAPPING) {
		set_slam_mode = lidar_slam::SlamWorkMode::SEC_MAPPING;
	} else if (set_status == ModuleStatus::MODULE_LOCALIZATION) {
		set_slam_mode = lidar_slam::SlamWorkMode::LOCALIZATION;
	} else {
		return false;
	}
	RCLCPP_INFO(node_->get_logger(), "Making obj(lidar_slam) --- with: set_slam_mode = %s",
				lidar_slam::print_SlamWorkMode(set_slam_mode).c_str());
	slam_ = std::make_unique<lidar_slam::LidarSlam>(yaml_param, set_slam_mode, node_);
	RCLCPP_INFO(node_->get_logger(), "\033[1;32mMake obj(lidar_slam) successfully !\033[0m");
	return true;
}

void LocalizationModule::release_slam_obj() {
	releasing_slam_flag_ = true;

	usleep(500000); // 单位: 微秒, 500,000us = 500ms = 0.5s
	RCLCPP_INFO(node_->get_logger(), "start stopping lidar_slam");

	lidar_slam::LidarSlam* temp_slam = slam_.release();
	delete temp_slam;
	temp_slam = nullptr;

	running_module_status_.store(ModuleStatus::MODULE_IDLE);
	mapping_node_status_.store(0); // 0: inactive
	local_node_status_.store(0);   // 0: inactive
	RCLCPP_INFO(node_->get_logger(), "\033[1;32mlidar_slam stopped !\033[0m");
	releasing_slam_flag_ = false;
}

bool LocalizationModule::make_map_directory_name(int map_id) {
	std::string map_folder = " ";
	if (slam_param_.common.run_on_mower) {
		node_->get_parameter_or("map_manager.map_folder", map_folder, slam_param_.common.map_directory);
	} else {
		map_folder = slam_param_.common.map_directory;
	}

	slam_param_.common.cloud_map_directory =
		map_folder + std::string("/") + std::to_string(map_id) + std::string("/3dmap/");

	// 检查并创建地图路径
	if (create_directory_if_not_exists(slam_param_.common.map_directory)) {
		std::cout << "Directory created or already exists: " << slam_param_.common.map_directory << std::endl;
	} else {
		std::cout << RED << "Failed to create directory: " << slam_param_.common.map_directory << std::endl;
		return false;
	}
	return true;
}

bool LocalizationModule::need_start_localization(ModuleStatus running_module_status_now, int localiztion_status_now) {
	if (running_module_status_now == ModuleStatus::MODULE_IDLE) {
		return true;
	} else if (running_module_status_now == ModuleStatus::MODULE_LOCALIZATION &&
			   localization_status_is_failed(localiztion_status_now)) {
		return true;
	} else {
		return false;
	}
}

bool LocalizationModule::localization_status_is_ok(int localiztion_status_now) {
	if (localiztion_status_now == 3 || localiztion_status_now == 4) {
		return true;
	} else {
		return false;
	}
}
bool LocalizationModule::localization_status_is_failed(int localiztion_status_now) {
	if (localiztion_status_now == 2 || localiztion_status_now == 5) {
		return true;
	} else {
		return false;
	}
}
bool LocalizationModule::mapping_status_is_ok(int mapping_status_now) {
	if (mapping_status_now == 3) {
		return true;
	} else {
		return false;
	}
}
bool LocalizationModule::mapping_status_is_failed(int mapping_status_now) {
	if (mapping_status_now == 2 || mapping_status_now == 5) {
		return true;
	} else {
		return false;
	}
}

} // namespace localization_module
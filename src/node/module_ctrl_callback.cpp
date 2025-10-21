
#include <logTracer/tracer.h>

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
	TRACE_INFO_CLASS("Received Ctrl Msg: %d", msg->data);
	// TRACE_INFO_CLASS("Received Ctrl Cmd: %d", print_SlamCtrlCmd(curr_cmd));

	int map_id = msg->data % 100;
	if (map_id == 0) {
		map_id = 1;
	}

	switch (curr_cmd) {
		case START_MAPPING: {			 // 初次建图，或重置后建图
			if (start_mapping(map_id)) { //启动建图成功
				TRACE_INFO_CLASS("start_mapping success!");
			} else { //启动建图失败
				TRACE_ERR_CLASS("start_mapping failed!");
			}
			break;
		}
		case START_SEC_MAPPING: { // 重定位，并开始建图
			if (start_second_mapping(map_id)) {
				TRACE_INFO_CLASS("start_second_mapping success!");
			} else {
				TRACE_ERR_CLASS("start_second_mapping failed!");
				release_slam_obj();
				TRACE_INFO_CLASS("slam obj destroyed!");
			}
			break;
		}
		case CANCLE_MAPPING: { // 不保存地图， 直接退出建图
			if (stop_mapping_without_saving_map()) {
				TRACE_INFO_CLASS("stop_mapping(not saving map) success!");
			} else {
				TRACE_ERR_CLASS("stop_mapping(not saving map) failed!");
			}
			break;
		}
		case SAVE_AND_END_MAPPING: { // 保存地图，并结束建图
			if (stop_mapping()) {
				TRACE_INFO_CLASS("stop_mapping success!");
			} else {
				TRACE_ERR_CLASS("stop_mapping failed!");
			}
			break;
		}
		case START_LOCALIZATION: { // 开始定位，（先重定位，再定位）
			if (start_localization(map_id)) {
				TRACE_INFO_CLASS("start_localization success!");
			} else {
				TRACE_ERR_CLASS("start_localization failed!");
			}
			break;
		}
		case EXIT_LOCALIZATION: {	   // 退出定位
			if (stop_localization()) { // 停止定位成功
				TRACE_INFO_CLASS("stop_localization success!");
			} else {
				TRACE_ERR_CLASS("stop_localization failed!");
			}
			break;
		}
		case START_RELOCALIZATION: { // 重新进行重定位
			if (start_relocalization(map_id)) {
				TRACE_INFO_CLASS("restart localization successfully!");
			} else {
				TRACE_ERR_CLASS("start_relocalization failed!");
			}
			break;
		}
		case RESTART_SEC_MAPPING: { // 重新进行重定位
			if (restart_second_mapping(map_id)) {
				TRACE_INFO_CLASS("restart sec_mapping successfully!");
			} else {
				TRACE_ERR_CLASS("restart sec_mapping failed!");
			}
			break;
		}
		default: {
			TRACE_INFO_CLASS("mapping ctrl msg: %u invalid!", msg->data);
			break;
		}
	}
}

bool LocalizationModule::start_mapping(int map_id) {
	auto set_status = ModuleStatus::MODULE_MAPPING;
	auto running_module_status_now = running_module_status_.load();
	TRACE_INFO_CLASS("Module Status for now: %s", print_ModuleStatus(running_module_status_now).c_str());
	TRACE_INFO_CLASS("Trying to set module_status: %s", print_ModuleStatus(set_status).c_str());

	if (!make_map_directory_name(map_id)) { // 基于 map_id, 保存在 [slam_param_.common.map_directory]
		TRACE_ERR_CLASS("Map Directory Error!");
		exit(EXIT_FAILURE);
	}
	if (running_module_status_now == ModuleStatus::MODULE_IDLE) {
		running_module_status_.store(ModuleStatus::MODULE_STARTING_SLAM);
		make_slam_obj(slam_param_, set_status);
		running_module_status_.store(ModuleStatus::MODULE_MAPPING);
		mapping_node_status_.store(MappingNodeStatus::Normal);
		return true;
	} else if (is_mapping_status(running_module_status_now)) {
		TRACE_INFO_CLASS("skip, already running mapping now");
		return false;
	} else if (running_module_status_now == ModuleStatus::MODULE_LOCALIZATION) {
		TRACE_INFO_CLASS("skip, running localizing now, please stop localizing first");
		return false;
	} else {
		TRACE_INFO_CLASS("skip, status error!");
		TRACE_INFO_CLASS("running_module_status_now: %s", print_ModuleStatus(running_module_status_now).c_str());
		TRACE_INFO_CLASS("set_module_status: %s", print_ModuleStatus(set_status).c_str());
		TRACE_INFO_CLASS("mapping_status: %d", mapping_status_.load());
		TRACE_INFO_CLASS("****************************");
		return false;
	}
}

bool LocalizationModule::start_second_mapping(int map_id) {
	auto running_module_status_now = running_module_status_.load();
	ModuleStatus set_status = ModuleStatus::MODULE_SEC_MAPPING;

	TRACE_INFO_CLASS("Module Status for now: %s", print_ModuleStatus(running_module_status_now).c_str());
	TRACE_INFO_CLASS("Trying to set module_status: %s", print_ModuleStatus(set_status).c_str());

	if (!make_map_directory_name(map_id)) { // 基于 map_id, 保存在 [slam_param_.common.map_directory]
		TRACE_ERR_CLASS("Map Directory Error!");
		exit(EXIT_FAILURE);
	}
	std::string load_map_dir = slam_param_.common.cloud_map_directory;

	if (running_module_status_now == ModuleStatus::MODULE_IDLE) {
		running_module_status_.store(ModuleStatus::MODULE_STARTING_SLAM);
		make_slam_obj(slam_param_, set_status);
		if (!slam_->load_map(load_map_dir)) {
			TRACE_ERR_CLASS("load map failed!");

			usleep(500000); // 单位: 微秒, 500,000us = 500ms = 0.5s
			release_slam_obj();
			TRACE_INFO_CLASS("slam destoried!");

			// update module-status
			running_module_status_.store(ModuleStatus::MODULE_IDLE);

			return false;
		} else { // start_second_mapping success
			running_module_status_.store(ModuleStatus::MODULE_SEC_MAPPING);
			mapping_node_status_.store(MappingNodeStatus::Normal);
			return true;
		}
	} else if (is_mapping_status(running_module_status_now)) {
		TRACE_INFO_CLASS("skip, already running mapping now");
		return false;
	} else if (running_module_status_now == ModuleStatus::MODULE_LOCALIZATION) {
		TRACE_INFO_CLASS("skip, running localizing now, please stop localizing first");
		return false;
	} else {
		TRACE_INFO_CLASS("skip, status error!");
		TRACE_INFO_CLASS("running_module_status_now: %s", print_ModuleStatus(running_module_status_now).c_str());
		TRACE_INFO_CLASS("set_module_status: %s", print_ModuleStatus(set_status).c_str());
		TRACE_INFO_CLASS("mapping_status: %d", mapping_status_.load());
		TRACE_INFO_CLASS("****************************");
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
		TRACE_INFO_CLASS("mapping stopped without saving map!");

		running_module_status_.store(ModuleStatus::MODULE_IDLE);
		mapping_node_status_.store(MappingNodeStatus::Inactive);
		return true;
	} else if (running_module_status_now == ModuleStatus::MODULE_IDLE ||
			   running_module_status_now == ModuleStatus::MODULE_LOCALIZATION) {
		TRACE_INFO_CLASS("skip, can not stop mapping, running_module_status_: %s",
						 print_ModuleStatus(running_module_status_now).c_str());
		return false;
	} else {
		TRACE_INFO_CLASS("skip, status error!");
		TRACE_INFO_CLASS("running_module_status_now: %s", print_ModuleStatus(running_module_status_now).c_str());
		TRACE_INFO_CLASS("set_module_status: %s", print_ModuleStatus(set_status).c_str());
		TRACE_INFO_CLASS("mapping_status: %d", mapping_status_.load());
		TRACE_INFO_CLASS("localization_status: %d", localization_status_.load());
		TRACE_INFO_CLASS("****************************");
		return false;
	}
}

bool LocalizationModule::stop_mapping() {
	// TODO: 问题：当点云量为0时，程序挂掉
	auto running_module_status_now = running_module_status_.load();
	auto mapping_status_now = mapping_status_.load();
	ModuleStatus set_status = ModuleStatus::MODULE_IDLE;
	if (is_mapping_status(running_module_status_now)) {
		if (mapping_status_now == MappingStatus::Standby) {
			running_module_status_.store(ModuleStatus::MODULE_STOPPING_SLAM);
			TRACE_INFO_CLASS("mapping_status: %d", mapping_status_.load());
			TRACE_INFO_CLASS("start saving map data");
			std::string pcd_dir = slam_param_.common.cloud_map_directory;
			const auto resolution = slam_param_.mapping.save_map_resolution;
			if (!slam_->save_map(pcd_dir, resolution, 0, 0)) {
				TRACE_ERR_CLASS("save map data failed!");
			} else {
				TRACE_INFO_CLASS("save map data success!");
			}

			// save_extrinsic_to_file();

			map_saved_.store(1);
			TRACE_INFO_CLASS("map_saved: %d", map_saved_.load());

			TRACE_INFO_CLASS("start stop mapping");
			sleep(1);
			// set_module_status_ = ModuleStatus::MODULE_IDLE;
			release_slam_obj();
			TRACE_INFO_CLASS("mapping stopped !");
			running_module_status_.store(ModuleStatus::MODULE_IDLE);
			mapping_node_status_.store(MappingNodeStatus::Inactive);

			return true;
		} else if (mapping_status_now == MappingStatus::CreatingEle) { // (not used)
			TRACE_INFO_CLASS("mapping_status: %d", mapping_status_.load());
			TRACE_INFO_CLASS("skip, please finish current map-element, or delete it first !");
			return false;
		} else {
			TRACE_INFO_CLASS("skip, status error!");
			TRACE_INFO_CLASS("set_module_status: %s", print_ModuleStatus(set_status).c_str());
			TRACE_INFO_CLASS("running_module_status_now: %s", print_ModuleStatus(running_module_status_now).c_str());
			TRACE_INFO_CLASS("mapping_status: %d", mapping_status_.load());
			TRACE_INFO_CLASS("localization_status: %d", localization_status_.load());
			TRACE_INFO_CLASS("****************************");
			return false;
		}
	} else if (running_module_status_now == ModuleStatus::MODULE_IDLE ||
			   running_module_status_now == ModuleStatus::MODULE_LOCALIZATION) {
		TRACE_INFO_CLASS("skip, can not stop mapping, running_module_status_: %s",
						 print_ModuleStatus(running_module_status_.load()).c_str());
		return false;
	} else {
		TRACE_INFO_CLASS("skip, status error!");
		TRACE_INFO_CLASS("running_module_status_now: %s", print_ModuleStatus(running_module_status_now).c_str());
		TRACE_INFO_CLASS("set_module_status: %s", print_ModuleStatus(set_status).c_str());
		TRACE_INFO_CLASS("mapping_status: %d", mapping_status_.load());
		TRACE_INFO_CLASS("localization_status: %d", localization_status_.load());
		TRACE_INFO_CLASS("****************************");
		return false;
	}
}

// bool LocalizationModule::save_extrinsic_to_file() {
// 	std::string extrinsic_file_name = slam_param_.common.cloud_map_directory + "/extrinsic.txt";
// 	std::ofstream extrinsic_file(extrinsic_file_name);
// 	if (!extrinsic_file.is_open()) {
// 		TRACE_ERR_CLASS("open extrinsic file failed!");
// 		return false;
// 	}

// 	extrinsic_file << "extrinsic_euler_lidar_in_baselink: \n";
// 	extrinsic_file << "  yaw:   " << slam_param_.extrinsic.yaw_pitch_roll_deg[0] << " degree, \n";
// 	extrinsic_file << "  pitch: " << slam_param_.extrinsic.yaw_pitch_roll_deg[1] << " degree, \n";
// 	extrinsic_file << "  roll:  " << slam_param_.extrinsic.yaw_pitch_roll_deg[2] << " degree, \n";
// 	extrinsic_file.close();
// 	return true;
// }

bool LocalizationModule::start_localization(int map_id) {
	ModuleStatus running_module_status_now = running_module_status_.load();
	ModuleStatus set_status = ModuleStatus::MODULE_LOCALIZATION;
	auto localization_status_now = localization_status_.load();

	TRACE_INFO_CLASS("Module Status for now: %s", print_ModuleStatus(running_module_status_now).c_str());
	TRACE_INFO_CLASS("Trying to set module_status: %s", print_ModuleStatus(set_status).c_str());

	if (!make_map_directory_name(map_id)) { // 基于 map_id, 保存在 [slam_param_.common.map_directory]
		TRACE_ERR_CLASS("Map Directory Error!");
		exit(EXIT_FAILURE);
	}
	std::string load_map_dir = slam_param_.common.cloud_map_directory;
	TRACE_INFO_CLASS("load_map_dir: %s", load_map_dir.c_str());

	// ModuleStatus curr_running_module_status = running_module_status_.load();

	if (need_start_localization(running_module_status_now, localization_status_now)) {
		running_module_status_.store(ModuleStatus::MODULE_STARTING_SLAM);

		make_slam_obj(slam_param_, set_status);

		if (!slam_->load_map(load_map_dir)) {
			TRACE_ERR_CLASS("load map failed!");
			release_slam_obj();

			running_module_status_.store(ModuleStatus::MODULE_IDLE);
			local_node_status_.store(LocalNodeStatus::Inactive);
			return false;
		} else {
			running_module_status_.store(ModuleStatus::MODULE_LOCALIZATION);
			local_node_status_.store(LocalNodeStatus::Normal);
			return true;
		}
	} else if (running_module_status_now == ModuleStatus::MODULE_LOCALIZATION &&
			   (localization_status_is_ok(localization_status_now))) {
		TRACE_INFO_CLASS("skip, already running localization normally now");
		return false;
	} else if (is_mapping_status(running_module_status_now)) {
		TRACE_INFO_CLASS("skip, running mapping now, please stop mapping first");
		return false;
	} else {
		TRACE_INFO_CLASS("skip, status error! start localization failed !");
		TRACE_INFO_CLASS("running_module_status_now: %s", print_ModuleStatus(running_module_status_now).c_str());
		TRACE_INFO_CLASS("set_module_status: %s", print_ModuleStatus(set_status).c_str());
		TRACE_INFO_CLASS("running_module_status_now: %s", print_ModuleStatus(running_module_status_now).c_str());
		TRACE_INFO_CLASS("mapping_status: %d", mapping_status_.load());
		TRACE_INFO_CLASS("****************************");
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
		TRACE_INFO_CLASS("localization stopped !");

		// update module-status
		running_module_status_.store(ModuleStatus::MODULE_IDLE);
		local_node_status_.store(LocalNodeStatus::Inactive);

		return true;
	} else if (running_module_status_now == ModuleStatus::MODULE_IDLE ||
			   running_module_status_now == ModuleStatus::MODULE_MAPPING ||
			   running_module_status_now == ModuleStatus::MODULE_SEC_MAPPING) {
		TRACE_INFO_CLASS("skip, can not stop localization, running_module_status_: %s",
						 print_ModuleStatus(running_module_status_now).c_str());
		return false;
	} else {
		TRACE_INFO_CLASS("skip, status error!");
		TRACE_INFO_CLASS("last_running_module_status_: %s", print_ModuleStatus(running_module_status_now).c_str());
		TRACE_INFO_CLASS("set_module_status: %s", print_ModuleStatus(set_status).c_str());
		TRACE_INFO_CLASS("running_module_status_: %s", print_ModuleStatus(running_module_status_now).c_str());
		TRACE_INFO_CLASS("mapping_status: %d", mapping_status_.load());
		TRACE_INFO_CLASS("localization_status: %d", localization_status_.load());
		TRACE_INFO_CLASS("****************************");
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
		TRACE_INFO_CLASS("skip, can not stop localization, running_module_status_: %s",
						 print_ModuleStatus(running_module_status_now).c_str());
		return false;
	} else {
		TRACE_INFO_CLASS("skip, status error!");
		TRACE_INFO_CLASS("running_module_status_: %s", print_ModuleStatus(running_module_status_now).c_str());
		TRACE_INFO_CLASS("mapping_status: %d", mapping_status_.load());
		TRACE_INFO_CLASS("localization_status: %d", localization_status_.load());
		TRACE_INFO_CLASS("****************************");
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
		TRACE_INFO_CLASS("skip, can not restart sec_mapping, running_module_status_: %s",
						 print_ModuleStatus(curr_running_module_status).c_str());
		return false;
	} else {
		TRACE_INFO_CLASS("skip, status error!");
		TRACE_INFO_CLASS("running_module_status_: %s", print_ModuleStatus(curr_running_module_status).c_str());
		TRACE_INFO_CLASS("mapping_status: %d", mapping_status_.load());
		TRACE_INFO_CLASS("localization_status: %d", localization_status_.load());
		TRACE_INFO_CLASS("****************************");
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
	TRACE_INFO_CLASS("Making obj(lidar_slam) --- with: set_slam_mode = %s",
					 lidar_slam::print_SlamWorkMode(set_slam_mode).c_str());
	slam_ = std::make_unique<lidar_slam::LidarSlam>(yaml_param, set_slam_mode, node_);
	TRACE_INFO_CLASS("Make obj(lidar_slam) successfully !");
	return true;
}

void LocalizationModule::release_slam_obj() {
	releasing_slam_flag_ = true;

	usleep(500000); // 单位: 微秒, 500,000us = 500ms = 0.5s
	TRACE_INFO_CLASS("start stopping lidar_slam");

	lidar_slam::LidarSlam* temp_slam = slam_.release();
	delete temp_slam;
	temp_slam = nullptr;

	running_module_status_.store(ModuleStatus::MODULE_IDLE);
	mapping_node_status_.store(MappingNodeStatus::Inactive);
	local_node_status_.store(LocalNodeStatus::Inactive);
	TRACE_INFO_CLASS("lidar_slam stopped !");
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
		TRACE_INFO_CLASS("Directory created or already exists:  %s", slam_param_.common.map_directory.c_str());
	} else {
		TRACE_ERR_CLASS("Failed to create directory: %s", slam_param_.common.map_directory.c_str());
		return false;
	}
	return true;
}

bool LocalizationModule::need_start_localization(ModuleStatus running_module_status_now,
												 LocalizationStatus localiztion_status_now) {
	if (running_module_status_now == ModuleStatus::MODULE_IDLE) {
		return true;
	} else if (running_module_status_now == ModuleStatus::MODULE_LOCALIZATION &&
			   localization_status_is_failed(localiztion_status_now)) {
		return true;
	} else {
		return false;
	}
}

bool LocalizationModule::localization_status_is_ok(LocalizationStatus localiztion_status_now) {
	if (localiztion_status_now == LocalizationStatus::Normal ||
		localiztion_status_now == LocalizationStatus::LowAccuracy) {
		return true;
	} else {
		return false;
	}
}
bool LocalizationModule::localization_status_is_failed(LocalizationStatus localiztion_status_now) {
	if (localiztion_status_now == LocalizationStatus::RelocalizeFailed ||
		localiztion_status_now == LocalizationStatus::Failed) {
		return true;
	} else {
		return false;
	}
}
bool LocalizationModule::mapping_status_is_ok(MappingStatus mapping_status_now) {
	if (mapping_status_now == MappingStatus::Standby) {
		return true;
	} else {
		return false;
	}
}
bool LocalizationModule::mapping_status_is_failed(MappingStatus mapping_status_now) {
	if (mapping_status_now == MappingStatus::RelocalizeFailed || mapping_status_now == MappingStatus::Failed) {
		return true;
	} else {
		return false;
	}
}

} // namespace localization_module
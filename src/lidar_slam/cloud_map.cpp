#include "lidar_slam/cloud_map.hpp"

namespace lidar_slam {

CloudMap::CloudMap() {
	loaded_sc_info_.clear();
	loaded_global_map_.reset(new PointCloudType());

	loaded_keyframe_poses_.clear();
	loaded_keyframe_clouds_.clear();
}

CloudMap::~CloudMap() {
	loaded_sc_info_.clear();
	loaded_global_map_.reset(new PointCloudType());

	loaded_keyframe_poses_.clear();
	loaded_keyframe_clouds_.clear();
}

//////////////////////////////////// - load map - /////////////////////////////////////////////////////

bool CloudMap::load_map_data(std::string map_dir) {
	map_data_ready_ = false;

	if (!load_cloud_map(map_dir)) {
		cout << "load cloud map failed!" << endl;
		// ROS_ERROR_STREAM(RED << "load cloud map failed!" << RESET);
		return false;
	}
	std::string keyframe_dir = map_dir + "/key_frame_cloud/";
	if (!load_key_frames(keyframe_dir)) {
		cout << "load key frame clouds failed!" << endl;
		// ROS_ERROR_STREAM(RED << "load key frame clouds failed!" << RESET);
		return false;
	}

	map_data_ready_ = true;
	cout << "\033[1;32m************************* load all map_data success\033[0m, map_data_ready_ = true" << endl;
	// ROS_INFO_STREAM(BOLDGREEN <<"************************* load all map_data success," <<RESET<<" map_data_ready_ =
	// true");
	return true;
}

bool CloudMap::load_key_frames(std::string keyframe_dir) {
	std::string keyframe_pose_path = keyframe_dir + "/key_frame_pose.txt";
	TRACE_INFO_CLASS("loading key_frame_pose from : %s", keyframe_pose_path.c_str());
	std::ifstream pose_file(keyframe_pose_path);
	try {
		if (!pose_file) {
			throw std::runtime_error("Failed to open pose_file");
		}
	} catch (const std::exception& e) {
		TRACE_ERR_CLASS("key frame pose file %s does not exist.", keyframe_pose_path.c_str());
	}

	/// read data line by line, split one line by ","
	loaded_key_point_.reset(new pcl::PointCloud<PointType>());
	loaded_keyframe_poses_.clear();
	std::string line;
	while (std::getline(pose_file, line)) {
		std::stringstream ss(line);
		std::string token;
		std::vector<double> values; // 单行数据已经全部临时存于 values
		while (std::getline(ss, token, ',')) {
			double value = std::stod(token);
			values.push_back(value);
		}

		// 开始读入 read_one_line
		KeyPose read_one_line;
		int idx = 0;

		/// 第 0 项： index
		read_one_line.index = static_cast<int>(values[idx++]); // 数据保存
		/// 第 1 项： time (lidar_end_time)
		read_one_line.time = static_cast<double>(values[idx++]); // 数据保存

		/// 第 2-17 项： 转换矩阵（4 x 4），旋转 + 平移
		for (int row = 0; row < 4; ++row) {
			for (int col = 0; col < 4; ++col) {
				read_one_line.pose(row, col) = values[idx++];
			}
		}
		Eigen::Vector3d translation = read_one_line.pose.translation();
		PointType point;
		point.x = translation.x();			 // 将x坐标设置为平移向量的x分量
		point.y = translation.y();			 // 将y坐标设置为平移向量的y分量
		point.z = translation.z();			 // 将z坐标设置为平移向量的z分量
		loaded_key_point_->push_back(point); // 数据保存

		Eigen::Matrix3d matrix = read_one_line.pose.matrix().block<3, 3>(0, 0);
		Eigen::Vector3d euler_angles = R2ypr(matrix); // defined in common_lib.h
		read_one_line.yaw = euler_angles[0];
		read_one_line.pitch = euler_angles[1];
		read_one_line.roll = euler_angles[2];

		loaded_keyframe_poses_.push_back(read_one_line); // 数据保存
	}
	pose_file.close();
	TRACE_INFO_CLASS("loaded_keyframe_poses size: %d", (int)loaded_keyframe_poses_.size());

	loaded_keyframe_clouds_.clear();
	int key_poses_size = loaded_keyframe_poses_.size();
	TRACE_INFO_CLASS("loading key_frame_cloud from dir: %s", keyframe_dir.c_str());
	for (int i = 0; i < key_poses_size; i++) {
		int pose_index = loaded_keyframe_poses_[i].index;
		std::string key_cloud_path = keyframe_dir + "/" + std::to_string(pose_index) + ".pcd";
		TRACE_INFO_CLASS("loading key_frame_cloud : %s", key_cloud_path.c_str());

		PointCloudType::Ptr temp_cloud(new PointCloudType());
		if (0 == access(key_cloud_path.c_str(), 0)) {
			pcl::io::loadPCDFile(key_cloud_path, *temp_cloud);
			loaded_keyframe_clouds_.push_back((temp_cloud));
		} else {
			TRACE_ERR_CLASS("key cloud file %s does not exist.", key_cloud_path.c_str());
			return false;
		}
	}

	return true;
}

bool CloudMap::load_cloud_map(std::string map_dir) {
	loaded_global_map_.reset(new PointCloudType());
	std::string cloud_map_file_path = map_dir + "cloud_map.pcd";
	TRACE_INFO_CLASS("loading cloud map from : %s", cloud_map_file_path.c_str());

	if (0 == access(cloud_map_file_path.c_str(), 0)) {
		pcl::io::loadPCDFile(cloud_map_file_path, *loaded_global_map_);
	} else {
		TRACE_ERR_CLASS("map file %s does not exist.", cloud_map_file_path.c_str());
		return false;
	}
	// TODO: show map point
	// ...

	// load data(pose & ScanContex)
	std::string sc_data_file_path = map_dir + "data";
	loaded_sc_info_.clear();
	TRACE_INFO_CLASS("loading sc-data from : %s", sc_data_file_path.c_str());
	std::ifstream file(sc_data_file_path);
	try {
		if (!file) {
			throw std::runtime_error("Failed to open file");
		}
	} catch (const std::exception& e) {
		TRACE_ERR_CLASS("sc data file %s does not exist.", sc_data_file_path.c_str());
	}

	// KeyMat polarcontext_invkeys_mat;
	// std::vector<Eigen::MatrixXd> polarcontexts;

	// read data line by line, split one line by ","
	std::string line;
	while (std::getline(file, line)) {
		ScInfo read_one_line;
		std::stringstream ss(line);
		std::string token;
		std::vector<double> values; // 单行数据临时存于 values
		while (std::getline(ss, token, ',')) {
			double value = std::stod(token);
			values.push_back(value);
		}
		int index = 0;
		/// 第 1 项： id
		read_one_line.id = static_cast<int>(values[index++]); // 数据保存

		/// 第 2-17 项： 转换矩阵（4 x 4），旋转 + 平移
		for (int row = 0; row < 4; ++row) {
			for (int col = 0; col < 4; ++col) {
				read_one_line.pose(row, col) = values[index++]; // 数据保存
			}
		}

		/// 第 18-19 项： sc 数据size
		int maxrow = values[index++];
		int maxcol = values[index++];
		/// check： sc 数据 size 是否对应
		if (values.size() - index != (maxrow * maxcol)) {
			TRACE_ERR_CLASS("sc data size not match, load sc data failed.");
			return false;
		}

		/// 第 20-end 项： sc 数据
		read_one_line.polarcontext.resize(maxrow, maxcol);
		for (int row = 0; row < maxrow; ++row) {
			for (int col = 0; col < maxcol; ++col) {
				read_one_line.polarcontext(row, col) = values[index++]; // 数据保存
			}
		}
		loaded_sc_info_.push_back(read_one_line); // 数据保存
	}
	file.close();

	if (loaded_sc_info_.size() == 0) {
		return false;
	}

	// sc_manager_->buildRingKeyKDTree(polarcontext_invkeys_mat, polarcontexts);
	TRACE_INFO_CLASS("loaded_sc_info size: %d", (int)loaded_sc_info_.size());
	return true;
}
//////////////////////////////////// - load map end - /////////////////////////////////////////////////

//////////////////////////////////// - save map - /////////////////////////////////////////////////////
/// TODO:
// bool CloudMap::save_map_data(){
// }

//////////////////////////////////// - save map end - /////////////////////////////////////////////////

} // namespace lidar_slam
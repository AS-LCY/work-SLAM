
#include "lidar_slam/localization.hpp"
namespace lidar_slam {
Localization::Localization() {
	log_info_manager_ = localization_module::LocalizationModuleLogInfoManager::getInstance();
	log_info_manager_->reset_log_info();

	gicp_.reset(new fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI>());
	gicp_->setNumThreads(1); // TODO(jxl)
	gicp_->setTransformationEpsilon(0.01);
	gicp_->setMaximumIterations(64);
	gicp_->setMaxCorrespondenceDistance(2.0); // TODO(jxl)
	gicp_->setCorrespondenceRandomness(20);

	ndt_.reset(new pcl::NormalDistributionsTransform<PointType, PointType>());
	icp_.reset(new pcl::IterativeClosestPoint<PointType, PointType>());

	// 根据输入数据的尺度设置NDT相关参数
	ndt_->setTransformationEpsilon(0.01); //为终止条件设置最小转换差异
	ndt_->setStepSize(0.1);				  //为more-thuente线搜索设置最大步长
	ndt_->setResolution(0.5);			  //设置NDT网格网格结构的分辨率（voxelgridcovariance）
	ndt_->setMaximumIterations(35);

	// 3. 设置参数
	icp_->setMaximumIterations(50);			  // 最大迭代次数
	icp_->setTransformationEpsilon(1e-8);	  // 变换收敛阈值
	icp_->setEuclideanFitnessEpsilon(1);	  // 误差收敛阈值
	icp_->setMaxCorrespondenceDistance(0.05); // 最大对应点距离 //TODO(jxl)

	KeyPoint_.reset(new pcl::PointCloud<pcl::PointXYZ>());
	CloudGlobalMap_.reset(new PointCloudType());
	accumulateMap_.reset(new PointCloudType());
	testMatchcloud_.reset(new PointCloudType());
	CloudGlobalMapIn_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	CloudGlobalMapIn_PointType_.reset(new PointCloudType());
	map_ready_ = false;
	filter_init_ = false;
}

Localization::~Localization() {}

void copyPointCloudManual(const PointCloudType::Ptr& src, pcl::PointCloud<pcl::PointXYZI>::Ptr& dst) {
	if (!src || !dst) return;

	dst->clear();
	dst->header = src->header;
	dst->is_dense = true; // 确保目标点云标记为 dense
	dst->sensor_orientation_ = src->sensor_orientation_;
	dst->sensor_origin_ = src->sensor_origin_;

	// 保留空间优化性能
	dst->reserve(src->size());
	for (const auto& src_pt : src->points) {
		if (!std::isfinite(src_pt.x) || !std::isfinite(src_pt.y) || !std::isfinite(src_pt.z)) {
			continue;
		}

		pcl::PointXYZI dst_pt;
		dst_pt.x = src_pt.x;
		dst_pt.y = src_pt.y;
		dst_pt.z = src_pt.z;
		dst_pt.intensity = src_pt.intensity;
		dst->push_back(dst_pt);
	}
	dst->width = dst->size();
	dst->height = 1; // 转换为无序点云
}

bool Localization::loadMap(std::string path) {
	map_ready_ = false;
	CloudGlobalMap_.reset(new PointCloudType());
	CloudGlobalMapIn_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	CloudGlobalMapIn_PointType_.reset(new PointCloudType());
	show_map_points_.clear();
	PointCloudType::Ptr TempMap(new PointCloudType());

	std::string cloud_map_file_path = path + std::string("cloud_map.pcd");
	std::ifstream cloud_file(cloud_map_file_path);
	if (cloud_file && cloud_file.good()) {
		if (pcl::io::loadPCDFile(cloud_map_file_path, *TempMap) == -1) {
			std::cerr << "Failed to load PCD file" << std::endl;
			return false;
		}
		*CloudGlobalMap_ = *TempMap;
		std::cout << "load map from : " << cloud_map_file_path << "--- point size: " << TempMap->points.size()
				  << std::endl;
		std::cout << "Cloud validity: " << CloudGlobalMap_->is_dense << " " << CloudGlobalMap_->points.size()
				  << std::endl;
	}

	// no ComplementMap.pcd
	// std::string ComplementMap_file_path = path + std::string("ComplementMap.pcd");
	// std::ifstream map_file(ComplementMap_file_path);
	// if (map_file && map_file.good()) {
	// 	TempMap->points.clear();
	// 	pcl::io::loadPCDFile(ComplementMap_file_path, *TempMap);
	// 	*CloudGlobalMap_ += *TempMap;
	// 	std::cout << "load map from : " << ComplementMap_file_path << "size " << TempMap->points.size() << std::endl;
	// }

	pcl::copyPointCloud(*CloudGlobalMap_, *CloudGlobalMapIn_PointType_); // TODO(jxl): 没必要拷贝一次
	pcl::VoxelGrid<PointType> downSizeFilter;
	PointCloudType::Ptr GlobalMapShow(new PointCloudType());
	double min_voxel_size = 0.1;
	if (CloudGlobalMap_->points.size() < 100000.0)
		downSizeFilter.setLeafSize(0.5, 0.5, 0.5); // for global map visualization
	else {
		min_voxel_size = min(0.3 * CloudGlobalMap_->points.size() / 100000.0, 1.0);
		downSizeFilter.setLeafSize(min_voxel_size, min_voxel_size, min_voxel_size); // for global map visualization
	}
	downSizeFilter.setInputCloud(CloudGlobalMapIn_PointType_);
	downSizeFilter.filter(*GlobalMapShow);
	std::cout << "load map from : " << path + std::string("=GlobalMap.pcd") << "size " << CloudGlobalMap_->points.size()
			  << std::endl;
	std::cout << "show map points: " << GlobalMapShow->points.size() << std::endl;
	for (int i = 0; i < GlobalMapShow->points.size(); i++) {
		Eigen::Vector3f point;
		point.x() = GlobalMapShow->points[i].x;
		point.y() = GlobalMapShow->points[i].y;
		point.z() = GlobalMapShow->points[i].z;
		show_map_points_.push_back(point);
	}
	if (CloudGlobalMap_->points.size() == 0) {
		std::cerr << "Failed to load map." << std::endl;
		return false;
	}

	std::vector<std::string> files;
	files.emplace_back(path + std::string("data"));
	files.emplace_back(path + std::string("Complementdata"));
	std::string line;

	LoadData_.clear();
	polarcontext_invkeys_mat_.clear();
	polarcontexts_.clear();
	KeyPoint_.reset(new pcl::PointCloud<pcl::PointXYZ>());
	accumulateMap_->points.clear();
	accumulateKeypose_.clear();
	scManager_.reset(new SCManager()); // TODO：是否每次加载地图都需要 重置ScanContex，即重定位

	for (auto filename : files) {
		std::ifstream file(filename);
		if (!file) {
			std::cerr << "Failed to open file " << filename << std::endl;
			continue;
		} else {
			std::cout << "load file " << filename << std::endl;
		}

		while (std::getline(file, line)) {
			ScInfo readData;
			std::stringstream ss(line);
			std::string token;
			std::vector<double> values;
			while (std::getline(ss, token, ',')) {
				double value = std::stod(token);
				values.push_back(value);
			}
			int index = 0;
			readData.id = static_cast<int>(values[index++]);

			for (int row = 0; row < 4; ++row) {
				for (int col = 0; col < 4; ++col) {
					readData.pose(row, col) = values[index++];
				}
			}
			Eigen::Vector3d translation = readData.pose.translation();
			pcl::PointXYZ point;
			point.x = translation.x(); // 将x坐标设置为平移向量的x分量
			point.y = translation.y(); // 将y坐标设置为平移向量的y分量
			point.z = translation.z(); // 将z坐标设置为平移向量的z分量
			KeyPoint_->push_back(point);
			accumulateKeypose_.push_back(readData.pose);

			int maxrow = values[index++];
			int maxcol = values[index++];
			if (values.size() - index != (maxrow * maxcol)) {
				std::cout << " error :" << values.size() << " " << index << " " << maxrow * maxcol << std::endl;
				return false;
			}
			readData.polarcontext.resize(maxrow, maxcol);
			for (int row = 0; row < maxrow; ++row) {
				for (int col = 0; col < maxcol; ++col) {
					readData.polarcontext(row, col) = values[index++];
				}
			}
			Eigen::MatrixXd sc = readData.polarcontext; // v1
			Eigen::MatrixXd ringkey = scManager_->makeRingkeyFromScancontext(sc);
			polarcontext_invkeys_mat_.push_back(eig2stdvec(ringkey));
			polarcontexts_.push_back(sc);
			LoadData_.push_back(readData);
		}
		file.close();
	}
	if (LoadData_.size() == 0) {
		return false;
	}
	scManager_->buildRingKeyKDTree(polarcontext_invkeys_mat_, polarcontexts_); //用来全局重定位

	std::cout << "get_load_data : " << LoadData_.size() << std::endl;
	map_ready_ = true;
	cout << "\033[1;32mLoad map success!\033[0m" << endl;
	copyPointCloudManual(CloudGlobalMapIn_PointType_, CloudGlobalMapIn_); //对加载进来的全局点云降采样后，又赋值回去
	// TODO(jxl): 没必要拷贝一次

	// ndt_->setInputTarget(CloudGlobalMapIn_PointType_);
	// icp_->setInputTarget(CloudGlobalMapIn_PointType_);
	gicp_->setInputTarget(CloudGlobalMapIn_);

	std::cout << "--------------------" << std::endl;
	return true;
}

bool Localization::localize(pcl::PointCloud<pcl::PointXYZI>::Ptr odomCloud, double& fit_score, double score_fail_thr,
							double score_low_accuracy_thr, double odom2map_delta_thr, double odom2map_delta_set,
							bool use_pose_filter) {
	static double odom2map_x_filter = 0.0;
	static double odom2map_y_filter = 0.0;

	static const double ratio = 1.0;
	std::cout << "odomCloud:" << odomCloud->points.size() << std::endl;
	std::cout << "CloudGlobalMapIn_:" << CloudGlobalMapIn_->points.size() << std::endl;
	if (!map_ready_) {
		return false;
	}

	gicp_->setInputSource(odomCloud);
	pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>());
	gicp_->align(*unused_result, correctionOdomToMap_.matrix().cast<float>());
	PointCloudType::Ptr output_cloud(new PointCloudType());
	std::cout << "..........----......" << std::endl;

	if (!gicp_->hasConverged()) {
		cout << RED << "gicp not converged " << RESET << endl;
		return false;
	} else {
		fit_score = gicp_->getFitnessScore();
		if (fit_score < score_low_accuracy_thr) {
			lastCorrectionOdomToMap_ = correctionOdomToMap_;
			lastUpdateTime_ = curr_time_;

			cout << GREEN << "gicp success with score " << gicp_->getFitnessScore() << RESET << endl;
			correctionOdomToMap_.matrix() = gicp_->getFinalTransformation().matrix().cast<double>();
			curr_time_ = omp_get_wtime();

			double last_x, last_y, last_z, last_roll, last_pitch, last_yaw;
			Eigen::Affine3d affine_transform(lastCorrectionOdomToMap_);
			pcl::getTranslationAndEulerAngles(affine_transform, last_x, last_y, last_z, last_roll, last_pitch,
											  last_yaw);
			double curr_x, curr_y, curr_z, curr_roll, curr_pitch, curr_yaw;
			Eigen::Affine3d affine_transform_co(correctionOdomToMap_);
			pcl::getTranslationAndEulerAngles(affine_transform_co, curr_x, curr_y, curr_z, curr_roll, curr_pitch,
											  curr_yaw);
			double abs_dx = std::abs(curr_x - last_x);
			double abs_dy = std::abs(curr_y - last_y);

			if (!filter_init_) {
				odom2map_x_filter = correctionOdomToMap_.translation().x();
				odom2map_y_filter = correctionOdomToMap_.translation().y();
				filter_init_ = true;
			} else { // ratio = 1.0
				odom2map_x_filter = ratio * correctionOdomToMap_.translation().x() + (1 - ratio) * odom2map_x_filter;
				odom2map_y_filter = ratio * correctionOdomToMap_.translation().y() + (1 - ratio) * odom2map_y_filter;
				correctionOdomToMap_.translation().x() = odom2map_x_filter;
				correctionOdomToMap_.translation().y() = odom2map_y_filter;
			}

			// // update log_info (log_info_manager_) 暂时注释掉，后期再补充
			// log_info_manager_->log_info.odom2map_dtime  = curr_time_ - lastUpdateTime_;
			// log_info_manager_->log_info.pose_odom2map.position.x = curr_x;
			// log_info_manager_->log_info.pose_odom2map.position.y = curr_y;
			// log_info_manager_->log_info.pose_odom2map.position.z = curr_z;
			// log_info_manager_->log_info.odom2map_dxyz.x = curr_x - last_x;
			// log_info_manager_->log_info.odom2map_dxyz.y = curr_y - last_y;
			// log_info_manager_->log_info.odom2map_dxyz.z = curr_z - last_z;
			// log_info_manager_->log_info.odom2map_drpy.x  = 180 / PI_M * (curr_roll  - last_roll);
			// log_info_manager_->log_info.odom2map_drpy.y  = 180 / PI_M * (curr_pitch - last_pitch);
			// log_info_manager_->log_info.odom2map_drpy.z  = 180 / PI_M * (curr_yaw   - last_yaw);

		} else {
			cout << YELLOW << "gicp converged, score: " << fit_score << RESET << endl;
		}

		return true;
	}
}

bool Localization::globalLocalization(PointCloudType::Ptr cloudIn, Eigen::Isometry3d pose, Matrix3d initial_rotate,
									  double score) {
	if (!map_ready_) {
		cout << YELLOW << "map not ready" << RESET << endl;
		return false;
	}
	Eigen::Vector3d current_euler = R2ypr(initial_rotate);
	// double current_yaw = current_euler[0];
	double current_pitch = current_euler[1];
	double current_roll = current_euler[2];

	PointCloudType::Ptr gravityAlignedCLoud(new PointCloudType());
	Eigen::Isometry3d Transform = Eigen::Isometry3d::Identity();
	Transform.matrix().block<3, 3>(0, 0) = initial_rotate;
	*gravityAlignedCLoud = *transformPointCloud(cloudIn, Transform); //用来生成当前点云的 sc-info

	std::vector<std::pair<double, double>> search_trans = { { 0, 0 },	{ -4, 0 }, { 4, 0 },  { 0, -4 },  { 0, 4 },
															{ -4, -4 }, { -4, 4 }, { 4, -4 }, { 4, 4 },	  { -2, 0 },
															{ 2, 0 },	{ 0, -2 }, { 0, 2 },  { -2, -2 }, { -2, 2 },
															{ 2, -2 },	{ 2, 2 } };
	double min_dist = std::numeric_limits<double>::max();
	std::pair<int, float> best_match{ -1, 0.0 };
	std::pair<double, double> best_trans;
	double t0 = omp_get_wtime();
	for (auto& t : search_trans) {
		Eigen::MatrixXd sc = scManager_->makeScancontext(*(gravityAlignedCLoud), t.first, t.second);
		std::vector<float> ringkey = eig2stdvec(scManager_->makeRingkeyFromScancontext(sc));
		Eigen::MatrixXd sectorkey = scManager_->makeSectorkeyFromScancontext(sc);
		double sc_dist = 1.0;
		auto match = scManager_->detectClosestMatch(sc, ringkey, sectorkey, sc_dist);
		if (match.first != -1) {
			std::cout << "trans: " << t.first << " " << t.second;
			std::cout << " score: " << sc_dist << std::endl;
		}
		if (sc_dist < min_dist) {
			min_dist = sc_dist;
			best_match = match;
			best_trans = t;
		}
	}

	double t1 = omp_get_wtime();
	cout << GREEN << "search_trans cost time: " << (t1 - t0) * 1000 << " ms" << RESET << endl;
	int match_idx = best_match.first;

	// ICP param-set
	pcl::IterativeClosestPoint<PointType, PointType> icp;
	icp.setMaxCorrespondenceDistance(100); // TODO(jxl)
	icp.setMaximumIterations(100);
	icp.setTransformationEpsilon(1e-6);
	icp.setEuclideanFitnessEpsilon(1e-6);
	icp.setRANSACIterations(0);

	if (match_idx != -1) {
		std::cout << "use index " << match_idx << std::endl;
		Eigen::Matrix4d init_guess =
			LoadData_[match_idx].pose.matrix(); // TODO(jxl): LoadData_和scManager_共同决定初值，原理？
		Eigen::Vector3d euler = R2ypr(init_guess.block<3, 3>(0, 0));
		// Eigen::Vector3d euler = init_guess.block<3, 3>(0, 0).eulerAngles(2, 1, 0);

		euler[0] += -best_match.second;
		std::cout << "rotate yaw: " << -best_match.second << std::endl;
		Eigen::Matrix3d rotate = ypr2R(Eigen::Vector3d(euler[0], current_pitch, current_roll));
		init_guess.block<3, 3>(0, 0) = rotate;
		euler = R2ypr(init_guess.block<3, 3>(0, 0));

		Eigen::Vector2d offset_in_lidar{ -best_trans.first, -best_trans.second };
		Eigen::Rotation2D<double> rotation(euler[0]);
		Eigen::Vector2d offset_in_map = rotation * offset_in_lidar;

		init_guess.coeffRef(0, 3) = init_guess.coeffRef(0, 3) + offset_in_map[0];
		init_guess.coeffRef(1, 3) = init_guess.coeffRef(1, 3) + offset_in_map[1];
		// init_guess.coeffRef(2, 3) = 0;

		// std::cout << "initial yaw "<<euler[0]*180/M_PI<<" pitch "<<euler[1]*180/M_PI<< " roll
		// "<<euler[2]*180/M_PI<<std::endl;
		// std::cout << " trans "<<init_guess.block<3, 1>(0, 3).transpose()<<std::endl;

		Eigen::Isometry3d testtransform(init_guess);
		testMatchcloud_ = transformPointCloud(cloudIn, testtransform);

		icp.setInputSource(cloudIn);
		icp.setInputTarget(CloudGlobalMap_);
		PointCloudType::Ptr unused_result(new PointCloudType());
		icp.align(*unused_result, init_guess.cast<float>());

		// 未收敛，或者匹配不够好
		if (icp.hasConverged() == false || icp.getFitnessScore() > score) { // TODO add number in getFitnessScore
			std::cout << "globalLocalization icp fail with score: " << icp.getFitnessScore() << std::endl;
			return false;
		} else {
			cout << GREEN << "globalLocalization success with score: " << icp.getFitnessScore() << RESET << endl;
		}
		Eigen::Isometry3d lidar_in_map;
		lidar_in_map.matrix() = icp.getFinalTransformation().matrix().cast<double>();

		correctionOdomToMap_ = lidar_in_map * pose.inverse();
		lastUpdateTime_ = omp_get_wtime();
		// euler = lidar_in_map.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0);
		// std::cout << "final yaw"<<euler[0]<<" pitch "<<euler[1]<< " roll "<<euler[2];
		// std::cout << "x "<<lidar_in_map.translation().x()<<" y "<<lidar_in_map.translation().y()<< " z
		// "<<lidar_in_map.translation().z()<<std::endl;

		double t2 = omp_get_wtime();
		cout << GREEN << "icp cost time: " << (t2 - t1) * 1000 << " ms" << RESET << endl;
		return true;
	} else {
		cout << "scancontext search fail, score {} " << match_idx << " " << min_dist << endl;
		return false;
	}
}

} // namespace lidar_slam

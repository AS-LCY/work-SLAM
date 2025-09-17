
#include "lidar_slam/localization.hpp"
namespace lidar_slam {
Localization::Localization() {
	log_info_manager_.reset_log_info();

	gicp_.reset(new fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI>());
	gicp_->setNumThreads(2);
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
	icp_->setMaxCorrespondenceDistance(0.05); // 最大对应点距离 // TODO(jxl)

	KeyPoint_.reset(new pcl::PointCloud<pcl::PointXYZ>());
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
	CloudGlobalMapIn_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	CloudGlobalMapIn_PointType_.reset(new PointCloudType());

	std::string cloud_map_file_path = path + std::string("cloud_map.pcd");
	std::ifstream cloud_file(cloud_map_file_path);
	if (cloud_file && cloud_file.good()) {
		if (pcl::io::loadPCDFile(cloud_map_file_path, *CloudGlobalMapIn_PointType_) == -1) {
			TRACE_ERR_CLASS("Failed to load PCD file %s", cloud_map_file_path.c_str());
			return false;
		}
		TRACE_INFO_CLASS("load map from : %s", cloud_map_file_path.c_str());
	}

	if (CloudGlobalMapIn_PointType_->points.size() == 0) {
		TRACE_ERR_CLASS("Failed to load map.");
		return false;
	}

	std::vector<std::string> files;
	files.emplace_back(path + std::string("data"));
	std::string line;

	LoadData_.clear();
	polarcontext_invkeys_mat_.clear();
	polarcontexts_.clear();
	KeyPoint_.reset(new pcl::PointCloud<pcl::PointXYZ>());

	accumulateKeypose_.clear();
	scManager_.reset(new SCManager()); // TODO：是否每次加载地图都需要 重置ScanContex，即重定位

	for (auto filename : files) {
		std::ifstream file(filename);
		if (!file) {
			TRACE_ERR_CLASS("Failed to open file %s", filename.c_str());
			continue;
		} else {
			TRACE_INFO_CLASS("load file %s", filename.c_str());
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
				TRACE_ERR_CLASS("sc data size not match, load sc data failed.");
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

	map_ready_ = true;
	TRACE_INFO_CLASS("loadData size: %d", LoadData_.size());
	TRACE_INFO_CLASS("Load map success!");

	copyPointCloudManual(CloudGlobalMapIn_PointType_, CloudGlobalMapIn_);

	// ndt_->setInputTarget(CloudGlobalMapIn_PointType_);
	// icp_->setInputTarget(CloudGlobalMapIn_PointType_);
	gicp_->setInputTarget(CloudGlobalMapIn_);

	return true;
}

bool Localization::localize(pcl::PointCloud<pcl::PointXYZI>::Ptr odomCloud, double& fit_score, double score_fail_thr,
							double score_low_accuracy_thr, double odom2map_delta_thr, double odom2map_delta_set,
							bool use_pose_filter) {
	double localize_start = omp_get_wtime();
	static double odom2map_x_filter = 0.0;
	static double odom2map_y_filter = 0.0;
	static const double ratio = 1.0;

	TRACE_INFO_CLASS("odomCloud size: %d", (int)odomCloud->points.size());
	TRACE_INFO_CLASS("CloudGlobalMapIn size: %d", (int)CloudGlobalMapIn_->points.size());

	if (!map_ready_) {
		TRACE_WARN_CLASS("map not ready...");
		return false;
	}

	gicp_->setInputSource(odomCloud);
	pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>());
	gicp_->align(*unused_result, correctionOdomToMap_.matrix().cast<float>());
	TRACE_INFO_CLASS("match with offline map done");

	PointCloudType::Ptr output_cloud(new PointCloudType());

	if (!gicp_->hasConverged()) {
		TRACE_ERR_CLASS("gicp not converged.");
		return false;
	} else {
		fit_score = gicp_->getFitnessScore(); // TODO(jxl): 统计内点，还是全部点
		if (fit_score < score_low_accuracy_thr) {
			lastCorrectionOdomToMap_ = correctionOdomToMap_;
			lastUpdateTime_ = curr_time_;

			TRACE_INFO_CLASS("gicp success with score %f < %f", fit_score, score_low_accuracy_thr);
			correctionOdomToMap_.matrix() = gicp_->getFinalTransformation().matrix().cast<double>();
			curr_time_ = omp_get_wtime();

			// double last_x, last_y, last_z, last_roll, last_pitch, last_yaw;
			// Eigen::Affine3d affine_transform(lastCorrectionOdomToMap_);
			// pcl::getTranslationAndEulerAngles(affine_transform, last_x, last_y, last_z, last_roll, last_pitch,
			// 								  last_yaw);
			// double curr_x, curr_y, curr_z, curr_roll, curr_pitch, curr_yaw;
			// Eigen::Affine3d affine_transform_co(correctionOdomToMap_);
			// pcl::getTranslationAndEulerAngles(affine_transform_co, curr_x, curr_y, curr_z, curr_roll, curr_pitch,
			// 								  curr_yaw);
			// double abs_dx = std::abs(curr_x - last_x);
			// double abs_dy = std::abs(curr_y - last_y);

			// if (!filter_init_) {
			// 	odom2map_x_filter = correctionOdomToMap_.translation().x();
			// 	odom2map_y_filter = correctionOdomToMap_.translation().y();
			// 	filter_init_ = true;
			// } else { // ratio = 1.0
			// 	odom2map_x_filter = ratio * correctionOdomToMap_.translation().x() + (1 - ratio) * odom2map_x_filter;
			// 	odom2map_y_filter = ratio * correctionOdomToMap_.translation().y() + (1 - ratio) * odom2map_y_filter;
			// 	correctionOdomToMap_.translation().x() = odom2map_x_filter;
			// 	correctionOdomToMap_.translation().y() = odom2map_y_filter;
			// }

			// // update log_info (log_info_manager_) 暂时注释掉，后期再补充
			// log_info_manager_.log_info.odom2map_dtime  = curr_time_ - lastUpdateTime_;
			// log_info_manager_.log_info.pose_odom2map.position.x = curr_x;
			// log_info_manager_.log_info.pose_odom2map.position.y = curr_y;
			// log_info_manager_.log_info.pose_odom2map.position.z = curr_z;
			// log_info_manager_.log_info.odom2map_dxyz.x = curr_x - last_x;
			// log_info_manager_.log_info.odom2map_dxyz.y = curr_y - last_y;
			// log_info_manager_.log_info.odom2map_dxyz.z = curr_z - last_z;
			// log_info_manager_.log_info.odom2map_drpy.x  = 180 / PI_M * (curr_roll  - last_roll);
			// log_info_manager_.log_info.odom2map_drpy.y  = 180 / PI_M * (curr_pitch - last_pitch);
			// log_info_manager_.log_info.odom2map_drpy.z  = 180 / PI_M * (curr_yaw   - last_yaw);

		} else {
			TRACE_INFO_CLASS("gicp success with score %f > %f", fit_score, score_low_accuracy_thr);
		}

		double localize_end = omp_get_wtime();
		TRACE_INFO_CLASS("localization cost time: %f ms", (localize_end - localize_start) * 1000);

		return true;
	}
}

bool Localization::globalLocalization(PointCloudType::Ptr cloudIn, Eigen::Isometry3d pose, Matrix3d initial_rotate,
									  double score) {
	if (!map_ready_) {
		TRACE_ERR_CLASS("map not ready");
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
			TRACE_INFO_CLASS("trans: %f, %f,  score: %f", t.first, t.second, sc_dist);
		}
		if (sc_dist < min_dist) {
			min_dist = sc_dist;
			best_match = match;
			best_trans = t;
		}
	}

	double t1 = omp_get_wtime();
	TRACE_INFO_CLASS("search_trans cost time: %f ms", (t1 - t0) * 1000);
	int match_idx = best_match.first;

	// ICP param-set
	pcl::IterativeClosestPoint<PointType, PointType> icp;
	icp.setMaxCorrespondenceDistance(100); // TODO(jxl)
	icp.setMaximumIterations(100);
	icp.setTransformationEpsilon(1e-6);
	icp.setEuclideanFitnessEpsilon(1e-6);
	icp.setRANSACIterations(0);

	if (match_idx != -1) {
		TRACE_INFO_CLASS("use index: %d", match_idx);
		Eigen::Matrix4d init_guess =
			LoadData_[match_idx].pose.matrix(); // TODO(jxl): LoadData_和scManager_共同决定初值，原理？
		Eigen::Vector3d euler = R2ypr(init_guess.block<3, 3>(0, 0));
		// Eigen::Vector3d euler = init_guess.block<3, 3>(0, 0).eulerAngles(2, 1, 0);

		euler[0] += -best_match.second;
		TRACE_INFO_CLASS("rotate yaw:  %f degree", euler[0] * RAD2DEGREE);

		Eigen::Matrix3d rotate = ypr2R(Eigen::Vector3d(euler[0], current_pitch, current_roll));
		init_guess.block<3, 3>(0, 0) = rotate;
		euler = R2ypr(init_guess.block<3, 3>(0, 0));

		Eigen::Vector2d offset_in_lidar{ -best_trans.first, -best_trans.second };
		Eigen::Rotation2D<double> rotation(euler[0]);
		Eigen::Vector2d offset_in_map = rotation * offset_in_lidar;

		init_guess.coeffRef(0, 3) = init_guess.coeffRef(0, 3) + offset_in_map[0];
		init_guess.coeffRef(1, 3) = init_guess.coeffRef(1, 3) + offset_in_map[1];
		// init_guess.coeffRef(2, 3) = 0;

		TRACE_INFO_CLASS("scManager given initial yaw: %f, pitch: %f, roll: %f", euler[0] * RAD2DEGREE,
						 euler[1] * RAD2DEGREE, euler[2] * RAD2DEGREE);
		TRACE_INFO_CLASS("scManager given intial trans x: %f, y: %f, z: %f", init_guess.coeffRef(0, 3),
						 init_guess.coeffRef(1, 3), init_guess.coeffRef(2, 3));

		Eigen::Isometry3d testtransform(init_guess);
		testMatchcloud_ = transformPointCloud(cloudIn, testtransform);

		icp.setInputSource(cloudIn);
		icp.setInputTarget(CloudGlobalMapIn_PointType_);
		PointCloudType::Ptr unused_result(new PointCloudType());
		icp.align(*unused_result, init_guess.cast<float>());

		// 未收敛，或者匹配不够好
		if (icp.hasConverged() == false || icp.getFitnessScore() > score) { // TODO(jxl): 统计内点，还是全部点
			TRACE_WARN_CLASS("globalLocalization icp fail with score: %f > %f", icp.getFitnessScore(), score);
			return false;
		} else {
			TRACE_INFO_CLASS("\n");
			TRACE_INFO_CLASS("globalLocalization icp success with score: %f < %f", icp.getFitnessScore(), score);
		}
		Eigen::Isometry3d lidar_in_map;
		lidar_in_map.matrix() = icp.getFinalTransformation().matrix().cast<double>();
		correctionOdomToMap_ = lidar_in_map * pose.inverse();
		lastUpdateTime_ = omp_get_wtime();

		Eigen::Vector3d T_map_odom_t = correctionOdomToMap_.translation();
		Eigen::Vector3d T_map_odom_euler = correctionOdomToMap_.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0);
		auto updated_euler = lidar_in_map.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0);

		TRACE_INFO_CLASS("icp given init T_map_lidar yaw: %f, pitch: %f, roll: %f", updated_euler[0] * RAD2DEGREE,
						 updated_euler[1] * RAD2DEGREE, updated_euler[2] * RAD2DEGREE);
		TRACE_INFO_CLASS("icp given init T_map_lidar trans x: %f, y: %f, z: %f", lidar_in_map.translation().x(),
						 lidar_in_map.translation().y(), lidar_in_map.translation().z());

		TRACE_INFO_CLASS("icp given T_map_odom yaw: %f, pitch: %f, roll: %f", T_map_odom_euler[0] * RAD2DEGREE,
						 T_map_odom_euler[1] * RAD2DEGREE, T_map_odom_euler[2] * RAD2DEGREE);
		TRACE_INFO_CLASS("icp given T_map_odom trans x: %f, y: %f, z: %f", T_map_odom_t.x(), T_map_odom_t.y(),
						 T_map_odom_t.z());
		double t2 = omp_get_wtime();
		TRACE_INFO_CLASS("localization global init icp cost time: %f ms", (t2 - t1) * 1000);
		return true;
	} else {
		TRACE_ERR_CLASS("scancontext search fail, score: %f", min_dist);
		return false;
	}
}

} // namespace lidar_slam

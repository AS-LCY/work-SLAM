
#include "lidar_slam/localization.hpp"

using namespace kiss_matcher;

namespace lidar_slam {
Localization::Localization(LocalizationParam param, const RelocalizationConfig& relocalize_params) {
	param_ = param;
	relocalize_config_ = relocalize_params;
	log_info_manager_.reset_log_info();

	gicp_.reset(new fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI>());
	gicp_->setNumThreads(param_.fgicp_thread_num);
	gicp_->setTransformationEpsilon(param_.fgicp_trans_eps);
	gicp_->setMaximumIterations(param_.fgicp_max_iter);
	gicp_->setMaxCorrespondenceDistance(param_.fgicp_max_corres_dist);
	gicp_->setCorrespondenceRandomness(param_.fgicp_max_corres_num);
	max_correspondence_dist_square_ = std::pow(param_.fgicp_inlier_max_corres_dist, 2);

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

	initGlobalLocalize();
}

void Localization::initGlobalLocalize() {
	relocalize_config_.matcher_config_ = kiss_matcher::KISSMatcherConfig(relocalize_config_.voxel_res_, false);
	//作者原本在类型转换(存在深拷贝)之后，才用自己的tbb版本降采样。我们在后面会提前用pcl来降采样，所以设置为false。

	relocalize_config_.matcher_config_.use_quatro_ = true;

	auto& gc = relocalize_config_.gicp_config_;
	gc.max_corr_dist_ = relocalize_config_.voxel_res_ * gc.scale_factor_for_corr_dist_;

	src_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	tgt_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	coarse_aligned_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	aligned_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	debug_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

	global_reg_handler_ = std::make_shared<kiss_matcher::KISSMatcher>(relocalize_config_.matcher_config_);
	local_reg_handler_ = std::make_shared<small_gicp::RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI>>();

	local_reg_handler_->setNumThreads(gc.num_threads_);
	local_reg_handler_->setCorrespondenceRandomness(gc.correspondence_randomness_);
	local_reg_handler_->setMaxCorrespondenceDistance(gc.max_corr_dist_);
	local_reg_handler_->setVoxelResolution(relocalize_config_.voxel_res_); //影响其内部source和target点云voxelmap_的计算
	local_reg_handler_->setRegistrationType("VGICP");					   // "VGICP" or "GICP"

	ds_source_cloud_filter_.setLeafSize(gc.ds_source_leaf_size_, gc.ds_source_leaf_size_, gc.ds_source_leaf_size_);
	ds_target_cloud_filter_.setLeafSize(gc.ds_target_leaf_size_, gc.ds_target_leaf_size_, gc.ds_target_leaf_size_);
	source_voxel_grid_filter_.setLeafSize(gc.ds_source_leaf_size_, gc.ds_source_leaf_size_, gc.ds_source_leaf_size_);
	target_voxel_grid_filter_.setLeafSize(gc.ds_target_leaf_size_, gc.ds_target_leaf_size_, gc.ds_target_leaf_size_);
	source_ds_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	target_ds_.reset(new pcl::PointCloud<pcl::PointXYZI>());
	cropped_target_.reset(new pcl::PointCloud<pcl::PointXYZI>());
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

	auto t_loadmap_start = std::chrono::high_resolution_clock::now();
	std::string cloud_map_file_path = path + std::string("cloud_map.pcd");
	std::ifstream cloud_file(cloud_map_file_path);
	if (cloud_file && cloud_file.good()) {
		if (pcl::io::loadPCDFile(cloud_map_file_path, *CloudGlobalMapIn_PointType_) == -1) {
			TRACE_ERR_CLASS("Failed to load PCD file %s", cloud_map_file_path.c_str());
			return false;
		}
		TRACE_INFO_CLASS("load map from : %s", cloud_map_file_path.c_str());
	}
	auto t_loadmap_end = std::chrono::high_resolution_clock::now();
	auto loadmap_cost_time =
		std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_loadmap_end - t_loadmap_start).count();
	TRACE_INFO_CLASS("load map cost time: %f ms", loadmap_cost_time);

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

void Localization::localize(pcl::PointCloud<pcl::PointXYZI>::Ptr odomCloud, LocalizeResultStatus& localize_status,
							LocalizationStatus& localize_state_status, const Sophus::SE3d& T_odom_lidar,
							const Sophus::SE3d& T_lidar_delta,
							const Eigen::Matrix<double, 6, 6>& T_lidar_delta_cov_local) {
	double localize_start = omp_get_wtime();
	if (!map_ready_) {
		TRACE_WARN_CLASS("map not ready...");
		return;
	}

	gicp_->setInputSource(odomCloud);
	pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_ptr(new pcl::PointCloud<pcl::PointXYZI>());
	gicp_->align(*aligned_ptr, correctionOdomToMap_.matrix().cast<float>());

	if (!gicp_->hasConverged()) {
		TRACE_ERR_CLASS("gicp not converged.");
		localize_status.converged = false;
		localize_state_status = LocalizationStatus::Failed;
		return;
	} else {
		localize_status.converged = true;
		double matching_error = 0.f;
		int num_inliers = 0;
		int num_valid_points = 0;
		std::vector<int> k_indices;
		std::vector<float> k_sq_dists;
		int too_far_points_num = 0;
		for (int i = 0; i < aligned_ptr->size(); i++) {
			const auto& pt = aligned_ptr->at(i);
			if (pt.getVector3fMap().norm() > param_.fgicp_inlier_max_valid_point_dist) {
				too_far_points_num++;
				continue;
			}
			num_valid_points++;

			gicp_->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
			if (k_sq_dists[0] < max_correspondence_dist_square_) {
				matching_error += std::sqrt(k_sq_dists[0]);
				num_inliers++;
			}
		}

		if (num_inliers != 0) {
			matching_error /= num_inliers;
		}
		double inlier_fraction = static_cast<float>(num_inliers) / std::max(1, num_valid_points);
		if (num_valid_points == 0) { //在特别空旷的场景下，nearby点个数为0，num_inliers为0，会触发误判逻辑
			TRACE_INFO_CLASS("points sum num = %d, too far points num = %d, nearby point num = %d, num_inliers = %d",
							 odomCloud->points.size(), too_far_points_num, num_valid_points, num_inliers);
			TRACE_INFO_CLASS("reset inlier_fraction = 100%");
			inlier_fraction = 1.0;
		}

		double localize_end = omp_get_wtime();
		double cost_time = (localize_end - localize_start) * 1000;
		localize_status.fit_score = matching_error;
		localize_status.num_inliers = num_inliers;
		localize_status.inlier_fraction = inlier_fraction;
		localize_status.cost_time = cost_time;
		TRACE_INFO_CLASS("gicp converged with inlier avg score: %f, inlier num = %d, inlier rate = %f, cost time = %f",
						 matching_error, num_inliers, inlier_fraction, cost_time);

		const double& inlier_avg_error = param_.fgicp_inlier_avg_error_thr;
		const double& inlier_rate = param_.fgicp_inlier_rate_thr;
		if (inlier_fraction < inlier_rate) {
			localize_state_status = LocalizationStatus::Failed;
			TRACE_ERR_CLASS("localization failed, for low inlier rate: %f% < %f%", inlier_fraction * 100.f,
							inlier_rate * 100.f);
		} else if (matching_error < inlier_avg_error) {
			localize_state_status = LocalizationStatus::Normal;
			assignMapToOdom(matching_error, T_odom_lidar, T_lidar_delta, T_lidar_delta_cov_local);
			TRACE_INFO_CLASS("localization Normal, for good inlier rate: %f%,  small avg score: %f < %f",
							 inlier_fraction * 100.f, matching_error, inlier_avg_error);
		} else {
			localize_state_status = LocalizationStatus::LowAccuracy;
			assignMapToOdom(matching_error, T_odom_lidar, T_lidar_delta, T_lidar_delta_cov_local);
			TRACE_ERR_CLASS("localization LowAccuracy, for good inlier rate: %f%, but high avg score: %f > %f",
							inlier_fraction * 100.f, matching_error, inlier_avg_error);
		}
	}
}

void Localization::assignMapToOdom(double matching_error, const Sophus::SE3d& T_odom_lidar,
								   const Sophus::SE3d& T_lidar_delta,
								   const Eigen::Matrix<double, 6, 6>& T_lidar_delta_cov_local) {
	Eigen::Isometry3d matched_result = Eigen::Isometry3d::Identity();
	matched_result.matrix() = gicp_->getFinalTransformation().matrix().cast<double>();

	// 直接赋值
	// correctionOdomToMap_ = matched_result;

	// 使用固定比率的指数平滑
	// correctionOdomToMap_ = smoothUpdateTransform(correctionOdomToMap_last_, matched_result);
	// correctionOdomToMap_last_ = correctionOdomToMap_;

	// 使用EKF平滑滤波器
	// double predict_noise = T_lidar_delta_cov_local.diagonal().maxCoeff();
	// double meas_noise = matching_error;
	// double scale_factor = meas_noise / predict_noise;
	// TRACE_INFO_CLASS("scale_factor = %f", scale_factor);

	matching_error *= 1e-1; //测量噪声比预测噪声大很多，缩小测量噪声
	Eigen::Matrix<double, 6, 1> noise_vec(matching_error, matching_error, matching_error, matching_error,
										  matching_error, matching_error); //前三维平移，后三维旋转
	Eigen::Matrix<double, 6, 6> meas_cov_global = noise_vec.asDiagonal();
	Sophus::SE3d T_map_odom = convertIsometry3dToSE3d(matched_result);
	Eigen::Isometry3d smoothed_T_map_odom =
		smootherMatchResult(T_map_odom, T_odom_lidar, T_lidar_delta, T_lidar_delta_cov_local, meas_cov_global);
	correctionOdomToMap_ = smoothed_T_map_odom;

	// Eigen::Vector3d trans = correctionOdomToMap_.translation();
	// Eigen::Vector3d euler = R2ypr(correctionOdomToMap_.rotation()) * RAD2DEGREE;
	// TRACE_INFO_CLASS("T_map_odom translation: x= %f, y= %f, z= %f", trans.x(), trans.y(), trans.z());
	// TRACE_INFO_CLASS("T_map_odom rotation: yaw= %f, pitch= %f, roll= %f", euler.x(), euler.y(), euler.z());
}

Eigen::Isometry3d Localization::smootherMatchResult(const Sophus::SE3d& T_map_odom, const Sophus::SE3d& T_odom_lidar,
													const Sophus::SE3d& T_lidar_delta,
													const Eigen::Matrix<double, 6, 6>& T_lidar_delta_cov_local,
													const Eigen::Matrix<double, 6, 6>& meas_cov_global) {
	Sophus::SE3d T_map_lidar_last = ekf_smoother_.getState();
	Eigen::Matrix<double, 6, 6> T_lidar_delta_cov_global =
		compute_global_cov(T_map_lidar_last, T_lidar_delta_cov_local);
	ekf_smoother_.processModel(T_lidar_delta, T_lidar_delta_cov_global);

	Sophus::SE3d T_map_lidar_meas = T_map_odom * T_odom_lidar;
	ekf_smoother_.update(T_map_lidar_meas, meas_cov_global);

	Sophus::SE3d T_map_lidar_curr = ekf_smoother_.getState();
	Sophus::SE3d smoothed_T_map_odom_sophus = T_map_lidar_curr * T_odom_lidar.inverse();
	Eigen::Isometry3d smoothed_T_map_odom = convertSE3dToIsometry3d(smoothed_T_map_odom_sophus);
	return smoothed_T_map_odom;
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
		correctionOdomToMap_ = lidar_in_map * pose.inverse(); //全局重定位不能做平滑
		correctionOdomToMap_last_ = correctionOdomToMap_;

		Eigen::Vector3d T_map_odom_t = correctionOdomToMap_.translation();
		Eigen::Vector3d T_map_odom_euler = correctionOdomToMap_.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0);
		auto updated_euler = lidar_in_map.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0);

		//初始化 ekf smoother
		Sophus::SE3d T_map_lidar_init = convertIsometry3dToSE3d(lidar_in_map);
		Eigen::Matrix<double, 6, 1> noise_vec(0.1, 0.1, 0.1, 0.1, 0.1, 0.1); //前三维平移，后三维旋转
		Eigen::Matrix<double, 6, 6> T_map_lidar_init_cov = noise_vec.asDiagonal();
		ekf_smoother_.init(T_map_lidar_init, T_map_lidar_init_cov);

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

pcl::PointCloud<pcl::PointXYZI>::Ptr Localization::cropTargetCloud(const Eigen::Isometry3d& pose,
																   const double& radius) {
	pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
	kdtree.setInputCloud(CloudGlobalMapIn_);

	pcl::PointXYZI searchPoint;
	searchPoint.x = pose.translation().x();
	searchPoint.y = pose.translation().y();
	searchPoint.z = pose.translation().z();

	std::vector<int> indices;
	std::vector<float> sqr_dists;
	kdtree.radiusSearch(searchPoint, radius, indices, sqr_dists);

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZI>());
	cloud_roi->resize(indices.size());

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(indices.size()); ++i) {
		cloud_roi->points[i] = CloudGlobalMapIn_->points[indices[i]];
	}

	cloud_roi->width = static_cast<uint32_t>(cloud_roi->points.size());
	cloud_roi->height = 1;
	cloud_roi->is_dense = true;

	return cloud_roi;
}

RegOutput Localization::icpAlignment() {
	RegOutput reg_output;
	aligned_->clear();

	local_reg_handler_->setInputSource(coarse_aligned_);
	local_reg_handler_->setInputTarget(tgt_cloud_);

	local_reg_handler_->align(*aligned_);

	const auto& local_reg_result = local_reg_handler_->getRegistrationResult();
	double overlapness = static_cast<double>(local_reg_result.num_inliers) / coarse_aligned_->size() * 100.0;
	reg_output.overlapness_ = overlapness;

	// NOTE(hlim): fine_T_coarse
	reg_output.pose_ = local_reg_handler_->getFinalTransformation().cast<double>();
	// if matchness overlapness is over than threshold,
	// that means the registration result is likely to be sufficiently overlapped
	if (overlapness > relocalize_config_.gicp_config_.overlap_threshold_) {
		reg_output.is_valid_ = true;
		reg_output.is_converged_ = true;
	}
	if (relocalize_config_.verbose_) {
		if (overlapness >= relocalize_config_.gicp_config_.overlap_threshold_) {
			TRACE_INFO_CLASS("global localization: local refine overlapness: %f% >= thresh: %f%", overlapness,
							 relocalize_config_.gicp_config_.overlap_threshold_);
		} else {
			TRACE_ERR_CLASS("global localization: local refine overlapness: %f% < thresh: %f%", overlapness,
							relocalize_config_.gicp_config_.overlap_threshold_);
		}
	}
	return reg_output;
}

RegOutput Localization::coarseToFineAlignment() {
	RegOutput reg_output;
	coarse_aligned_->clear();

	const auto& src_vec = convertCloudToVec(*src_cloud_);
	std::vector<Eigen::Vector3f> tgt_vec;
	if (!global_match_target_cloud_assigned_.load()) {
		tgt_vec = convertCloudToVec(*tgt_cloud_);
		global_match_target_cloud_assigned_.store(true);
	}
	TRACE_INFO_CLASS("Global registration: source cloud size: %d, target cloud size: %d", src_cloud_->points.size(),
					 tgt_cloud_->points.size());
	const auto& solution = global_reg_handler_->estimate(src_vec, tgt_vec);

	Eigen::Matrix4d coarse_alignment = Eigen::Matrix4d::Identity();
	coarse_alignment.block<3, 3>(0, 0) = solution.rotation.cast<double>();
	coarse_alignment.topRightCorner(3, 1) = solution.translation.cast<double>();
	auto coarse_map_odom_euler = coarse_alignment.block<3, 3>(0, 0).eulerAngles(2, 1, 0);
	TRACE_INFO_CLASS("kiss matcher given init T_map_odom yaw: %f, pitch: %f, roll: %f",
					 coarse_map_odom_euler[0] * RAD2DEGREE, coarse_map_odom_euler[1] * RAD2DEGREE,
					 coarse_map_odom_euler[2] * RAD2DEGREE);
	TRACE_INFO_CLASS("kiss matcher given init T_map_odom trans x: %f, y: %f, z: %f", coarse_alignment(0, 3),
					 coarse_alignment(1, 3), coarse_alignment(2, 3));
	*coarse_aligned_ = transformPcd(*src_cloud_, coarse_alignment);

	const size_t num_inliers = global_reg_handler_->getNumFinalInliers();
	reg_output.num_final_inliers_ = num_inliers; // TODO(jxl): 内点个数怎么计算的？
	if (relocalize_config_.verbose_) {
		if (num_inliers >= relocalize_config_.num_inliers_threshold_) {
			TRACE_INFO_CLASS("final inliers = %d >= thresh = %d", num_inliers,
							 relocalize_config_.num_inliers_threshold_);
		} else {
			TRACE_ERR_CLASS("ERROR: final inliers = %d < thresh = %d", num_inliers,
							relocalize_config_.num_inliers_threshold_);
		}
	}

	// NOTE(hlim): A small number of inliers suggests that the initial alignment may have failed,
	// so fine alignment is meaningless.
	if (!solution.valid || num_inliers < relocalize_config_.num_inliers_threshold_) {
		return reg_output;
	} else {
		lidar_slam::TicToc timer;
		const auto& fine_output = icpAlignment(); // local refine
		const auto t = timer.toc();
		TRACE_INFO_CLASS("local refine cost time = %f ms", t);
		reg_output = fine_output;
		reg_output.pose_ = fine_output.pose_ * coarse_alignment;

		// Use this cloud to debug whether the transformation is correct.
		// *debug_cloud_        = transformPcd(src, reg_output.pose_);
	}
	return reg_output;
}

bool Localization::globalLocalization(const pcl::PointCloud<pcl::PointXYZI>::Ptr odom_cloud,
									  const Eigen::Isometry3d& T_odom_lidar_curr, const int try_num) {
	if (!odom_cloud || !CloudGlobalMapIn_) {
		TRACE_ERR_CLASS("point cloud ptr is nullptr.");
		return false;
	}
	if (odom_cloud->points.size() < 200) {
		TRACE_WARN_CLASS("relocalization input cloud size too small: %d < 200", odom_cloud->points.size());
		return false;
	}

	lidar_slam::TicToc timer_voxel;
	source_ds_->clear();
	if (!global_match_target_cloud_assigned_.load()) target_ds_->clear();

	lidar_slam::TicToc source_voxel_timer;
	ds_source_cloud_filter_.setInputCloud(odom_cloud);
	ds_source_cloud_filter_.filter(*source_ds_);
	src_cloud_ = source_ds_; //指向同一个地址
	const auto source_voxel_cost_time = source_voxel_timer.toc();
	TRACE_INFO_CLASS("raw source cloud: %d, downsampled source cloud: %d, cost time: %f ms", odom_cloud->points.size(),
					 source_ds_->points.size(), source_voxel_cost_time);

	// lidar_slam::TicToc target_crop_timer;
	// const Eigen::Isometry3d& T_map_lidar_guess = correctionOdomToMap_ * T_odom_lidar_curr;
	// cropped_target_ = cropTargetCloud(T_map_lidar_guess);

	// { //使用crop box裁剪虽然比cropTargetCloud更快，但是kiss matcher的trans inliers却更少
	// 	const Eigen::Vector3d& center = T_map_lidar_guess.translation();
	// 	Eigen::Vector3d euler = T_map_lidar_guess.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0);
	// 	double length = 60.0;
	// 	Eigen::Vector3f size(length, length, length);
	// 	Eigen::Vector4f min_pt(center.x() - size.x() / 2, center.y() - size.y() / 2, center.z() - size.z() / 2, 1.0f);
	// 	Eigen::Vector4f max_pt(center.x() + size.x() / 2, center.y() + size.y() / 2, center.z() + size.z() / 2, 1.0f);
	// 	pcl::CropBox<pcl::PointXYZI> crop_box;
	// 	crop_box.setInputCloud(CloudGlobalMapIn_);
	// 	crop_box.setMin(min_pt);
	// 	crop_box.setMax(max_pt);
	// 	Eigen::Vector3f rotation(0, 0, euler[0]);
	// 	crop_box.setRotation(rotation);
	// 	crop_box.filter(*cropped_target_);
	// }
	// const auto target_crop_cost_time = target_crop_timer.toc();
	// TRACE_INFO_CLASS("raw target cloud: %d, cropped target cloud: %d, cost time: %f ms",
	// 				 CloudGlobalMapIn_->points.size(), cropped_target_->points.size(), target_crop_cost_time);

	lidar_slam::TicToc target_voxel_timer;
	if (!global_match_target_cloud_assigned_.load()) {
		ds_target_cloud_filter_.setInputCloud(CloudGlobalMapIn_); // CloudGlobalMapIn_, cropped_target_
		ds_target_cloud_filter_.filter(*target_ds_);
		tgt_cloud_ = target_ds_; //指向同一个地址
		TRACE_INFO_CLASS("set target cloud for global localization.");
	} else {
		TRACE_INFO_CLASS("target cloud for global localization already set.");
	}
	const auto target_voxel_cost_time = target_voxel_timer.toc();
	TRACE_INFO_CLASS("downsampled target cloud: %d, cost time: %f ms", tgt_cloud_->points.size(),
					 target_voxel_cost_time);
	const auto voxel_cost_time = timer_voxel.toc();
	TRACE_INFO_CLASS("voxel cost time = %f ms", voxel_cost_time);

	lidar_slam::TicToc timer_alignment;
	const auto& reg_output = coarseToFineAlignment();
	global_reg_handler_->print();
	const auto t = timer_alignment.toc();
	TRACE_INFO_CLASS("global localization cost time = %f ms", t);

	if (!reg_output.is_valid_) {
		TRACE_ERR_CLASS("global localization alignment rejected. # of inliers: %d", reg_output.num_final_inliers_);
		return false;
	}

	correctionOdomToMap_.matrix() = reg_output.pose_; //全局重定位不能做平滑
	correctionOdomToMap_last_ = correctionOdomToMap_; // T_map_odom

	Eigen::Vector3d T_map_odom_t = correctionOdomToMap_.translation();
	Eigen::Vector3d T_map_odom_euler = correctionOdomToMap_.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0);
	Eigen::Isometry3d lidar_in_map = correctionOdomToMap_ * T_odom_lidar_curr;
	auto updated_euler = lidar_in_map.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0);

	//初始化 ekf smoother
	Sophus::SE3d T_map_lidar_init = convertIsometry3dToSE3d(lidar_in_map);
	Eigen::Matrix<double, 6, 1> noise_vec(0.1, 0.1, 0.1, 0.1, 0.1, 0.1); //前三维平移，后三维旋转
	Eigen::Matrix<double, 6, 6> T_map_lidar_init_cov = noise_vec.asDiagonal();
	ekf_smoother_.init(T_map_lidar_init, T_map_lidar_init_cov);

	TRACE_INFO_CLASS("icp given init T_map_lidar yaw: %f, pitch: %f, roll: %f", updated_euler[0] * RAD2DEGREE,
					 updated_euler[1] * RAD2DEGREE, updated_euler[2] * RAD2DEGREE);
	TRACE_INFO_CLASS("icp given init T_map_lidar trans x: %f, y: %f, z: %f", lidar_in_map.translation().x(),
					 lidar_in_map.translation().y(), lidar_in_map.translation().z());

	TRACE_INFO_CLASS("icp given T_map_odom yaw: %f, pitch: %f, roll: %f", T_map_odom_euler[0] * RAD2DEGREE,
					 T_map_odom_euler[1] * RAD2DEGREE, T_map_odom_euler[2] * RAD2DEGREE);
	TRACE_INFO_CLASS("icp given T_map_odom trans x: %f, y: %f, z: %f", T_map_odom_t.x(), T_map_odom_t.y(),
					 T_map_odom_t.z());
	return true;
}

} // namespace lidar_slam

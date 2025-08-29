
#include "lidar_slam/localization.hpp"
namespace lidar_slam {
Localization::Localization() {
	log_info_manager_ = localization_module::LocalizationModuleLogInfoManager::getInstance();
	log_info_manager_->reset_log_info();

	gicp_.reset(new fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI>());
	gicp_->setNumThreads(1);
	gicp_->setTransformationEpsilon(0.01);
	gicp_->setMaximumIterations(64);
	gicp_->setMaxCorrespondenceDistance(2.0);
	gicp_->setCorrespondenceRandomness(20);

	// pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt;
	// ndt.reset(new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
	ndt_.reset(new pcl::NormalDistributionsTransform<PointType, PointType>());
	icp_.reset(new pcl::IterativeClosestPoint<PointType, PointType>());
	// 根据输入数据的尺度设置NDT相关参数
	ndt_->setTransformationEpsilon(0.01); //为终止条件设置最小转换差异
	ndt_->setStepSize(0.1);				  //为more-thuente线搜索设置最大步长
	ndt_->setResolution(0.5);			  //设置NDT网格网格结构的分辨率（voxelgridcovariance）
	//以上参数在使用房间尺寸比例下运算比较好，但如果需要处理例如一个杯子的扫描之类更小的物体，需要对参数进行缩小

	//设置匹配迭代的最大次数，这个参数控制程序运行的最大迭代次数，一般来说这个限制值之前优化程序会在epsilon变换阀值下终止
	//添加最大迭代次数限制能够增加程序的鲁棒性阻止了它在错误的方向上运行时间过长
	ndt_->setMaximumIterations(35);

	// 3. 设置参数

	icp_->setMaximumIterations(50);			  // 最大迭代次数
	icp_->setTransformationEpsilon(1e-8);	  // 变换收敛阈值
	icp_->setEuclideanFitnessEpsilon(1);	  // 误差收敛阈值
	icp_->setMaxCorrespondenceDistance(0.05); // 最大对应点距离

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

	// 逐点复制并过滤 NaN
	for (const auto& src_pt : src->points) {
		// 检查坐标是否有效
		if (!std::isfinite(src_pt.x) || !std::isfinite(src_pt.y) || !std::isfinite(src_pt.z)) {
			continue; // 跳过无效点
		}

		pcl::PointXYZI dst_pt;
		dst_pt.x = src_pt.x;
		dst_pt.y = src_pt.y;
		dst_pt.z = src_pt.z;
		dst_pt.intensity = src_pt.intensity;
		dst->push_back(dst_pt);
	}

	// 更新点云尺寸信息
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
	// if (std::filesystem::exists(cloud_map_file_path)){
	std::ifstream cloud_file(cloud_map_file_path);
	if (cloud_file.good()) {
		// pcl::io::loadPCDFile(cloud_map_file_path, *TempMap);

		// ROS_INFO_STREAM("load map from : " << cloud_map_file_path<<"--- point size: "<<TempMap->points.size() );
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
	// // no ComplementMap.pcd
	std::string ComplementMap_file_path = path + std::string("ComplementMap.pcd");
	// if (std::filesystem::exists(ComplementMap_file_path)){
	std::ifstream map_file(ComplementMap_file_path);
	if (map_file.good()) {
		TempMap->points.clear();
		pcl::io::loadPCDFile(ComplementMap_file_path, *TempMap);
		*CloudGlobalMap_ += *TempMap;
		std::cout << "load map from : " << ComplementMap_file_path << "size " << TempMap->points.size() << std::endl;
		// ROS_INFO_STREAM("load map from : " << ComplementMap_file_path<<"size "<<TempMap->points.size() );
	}

	pcl::copyPointCloud(*(CloudGlobalMap_), *CloudGlobalMapIn_PointType_);
	// copyPointCloudManual(CloudGlobalMap_, CloudGlobalMapIn_);
	// pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
	// pcl::PointCloud<pcl::PointXYZI>::Ptr GlobalMapShow(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::VoxelGrid<PointType> downSizeFilter;
	PointCloudType::Ptr GlobalMapShow(new PointCloudType());

	double min_voxel_size = 0.1;
	if (CloudGlobalMap_->points.size() < 100000.0)
		downSizeFilter.setLeafSize(0.5, 0.5, 0.5); // for global map visualization
	else {
		min_voxel_size = min(0.3 * CloudGlobalMap_->points.size() / 100000.0, 2.0);
		downSizeFilter.setLeafSize(min_voxel_size, min_voxel_size, min_voxel_size); // for global map visualization
	}

	downSizeFilter.setInputCloud(CloudGlobalMapIn_PointType_);

	downSizeFilter.filter(*GlobalMapShow);

	std::cout << "load map from : " << path + std::string("=GlobalMap.pcd") << "size " << CloudGlobalMap_->points.size()
			  << std::endl;
	std::cout << "show map points: " << GlobalMapShow->points.size() << std::endl;

	std::cout << "Eigen default alignment: " << EIGEN_DEFAULT_ALIGN_BYTES << std::endl;

	for (int i = 0; i < GlobalMapShow->points.size(); i++) {
		Eigen::Vector3f point;
		point.x() = GlobalMapShow->points[i].x;
		point.y() = GlobalMapShow->points[i].y;
		point.z() = GlobalMapShow->points[i].z;

		show_map_points_.push_back(point);
	}
	if (CloudGlobalMap_->points.size() == 0) {
		std::cerr << "Failed to load map." << std::endl;
		// ROS_ERROR_STREAM(RED << "Failed to load map."  << RESET);
		return false;
	}

	std::vector<std::string> files;
	files.emplace_back(path + std::string("data"));
	files.emplace_back(path + std::string("Complementdata"));
	// std::ifstream file(path+std::string("/data"));

	// if (!file) {
	//     std::cerr << "Failed to open file for reading." << std::endl;
	//     return false;
	// }

	std::string line;
	LoadData_.clear();
	polarcontext_invkeys_mat_.clear();
	polarcontexts_.clear();
	KeyPoint_.reset(new pcl::PointCloud<pcl::PointXYZ>());
	accumulateMap_->points.clear();
	accumulateKeypose_.clear();

	////////////////////////////////////////////////////

	scManager_.reset(new SCManager()); /////////////// TODO，是否每次加载地图都需要 重置ScanContex，即重定位
	for (auto filename : files) {
		std::ifstream file(filename);
		if (!file) {
			std::cerr << "Failed to open file " << filename << std::endl;
			// ROS_ERROR_STREAM(RED << "Failed to open file "<< filename  <<RESET);
			continue;
		} else {
			std::cout << "load file " << filename << std::endl;
			// ROS_INFO_STREAM("load file "<< filename );
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
			//  std::cout << "  id: " <<readData.id << std::endl;

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
			// std::cout << " \n transform:"<<readData.pose.matrix() << std::endl;
			int maxrow = values[index++];
			int maxcol = values[index++];
			if (values.size() - index != (maxrow * maxcol)) {
				std::cout << " error :" << values.size() << " " << index << " " << maxrow * maxcol << std::endl;
				// ROS_ERROR_STREAM(RED << " error :"<<values.size()<<" "<<index<<" "<<maxrow*maxcol <<RESET);
				return false;
			}
			readData.polarcontext.resize(maxrow, maxcol);
			for (int row = 0; row < maxrow; ++row) {
				for (int col = 0; col < maxcol; ++col) {
					readData.polarcontext(row, col) = values[index++];
				}
			}
			// loadScManager.loadScancontextAndKeys(readData.polarcontext);
			Eigen::MatrixXd sc = readData.polarcontext; // v1
			Eigen::MatrixXd ringkey = scManager_->makeRingkeyFromScancontext(sc);
			// std::vector<float> polarcontext_invkey_vec = eig2stdvec( ringkey );
			polarcontext_invkeys_mat_.push_back(eig2stdvec(ringkey));
			polarcontexts_.push_back(sc);
			//   if (i == 17)
			//    std::cout << " \n matrix:"  << std::endl;
			//    std::cout << value <<",";
			LoadData_.push_back(readData);
		}
		file.close();
	}
	if (LoadData_.size() == 0) return false;
	scManager_->buildRingKeyKDTree(polarcontext_invkeys_mat_, polarcontexts_);

	std::cout << "get_load_data : " << LoadData_.size() << std::endl;
	// ROS_INFO_STREAM("get_load_data : " << LoadData_.size() );
	map_ready_ = true;
	cout << "\033[1;32mLoad map success!\033[0m" << endl;
	// ROS_INFO_STREAM(BOLDGREEN << "Load map success!" << RESET );
	copyPointCloudManual(CloudGlobalMapIn_PointType_,
						 CloudGlobalMapIn_); // TODO：： 这里解决过一个 align free 的问题 ，避免栈区内存拷贝给堆区内存上

	// ndt_->setInputTarget(CloudGlobalMapIn_PointType_);  //目标点云
	gicp_->setInputTarget(CloudGlobalMapIn_); //目标点云
	// icp_->setInputTarget(CloudGlobalMapIn_PointType_);  //目标点云
	std::cout << "--------------------" << std::endl;
	return true;
}

// bool Localization::localize(pcl::PointCloud<pcl::PointXYZI>::Ptr odomCloud, double score_thr, double
// odom2map_delta_thr, double odom2map_delta_set, bool use_pose_filter) bool
// Localization::localize(pcl::PointCloud<pcl::PointXYZI>::Ptr odomCloud, double score_thr, double odom2map_delta_thr,
// double odom2map_delta_set, bool use_pose_filter)
bool Localization::localize(pcl::PointCloud<pcl::PointXYZI>::Ptr odomCloud, double& fit_score, double score_fail_thr,
							double score_low_accuracy_thr, double odom2map_delta_thr, double odom2map_delta_set,
							bool use_pose_filter)
// bool Localization::localize( PointCloudType::Ptr odomCloud, double &fit_score,  double score_fail_thr, double
// score_low_accuracy_thr,
//     double odom2map_delta_thr, double odom2map_delta_set, bool use_pose_filter)
{
	// pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZI>());
	// pcl::copyPointCloud(*(odomCloud), *cloudIn);
	static double odom2map_x_filter = 0.0;
	static double odom2map_y_filter = 0.0;
	// static const double ratio = 0.1;
	static const double ratio = 1.0;
	std::cout << "odomCloud:" << odomCloud->points.size() << std::endl;
	std::cout << "CloudGlobalMapIn_:" << CloudGlobalMapIn_->points.size() << std::endl;
	if (!map_ready_) return false;
	// TODO::
	gicp_->setInputSource(odomCloud);
	// TODO:: gicp_->setInputTarget(CloudGlobalMapIn_);
	pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>()); //
	// PointCloudType::Ptr unused_result(new PointCloudType());
	// TODO::
	gicp_->align(*unused_result, correctionOdomToMap_.matrix().cast<float>());

	// 设置使用机器人测距法得到的粗略初始变换矩阵结果
	// 计算需要的刚体变换以便将输入的源点云匹配到目标点云
	// pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	PointCloudType::Ptr output_cloud(new PointCloudType());
	// ndt_->setInputSource(odomCloud);  //源点云
	//  icp_->setInputSource(odomCloud);  //源点云
	// Setting point cloud to be aligned to.
	// ndt.setInputTarget(CloudGlobalMapIn_);  //目标点云
	// ndt_->align(*output_cloud, correctionOdomToMap_.matrix().cast<float>());
	//  icp_->align(*output_cloud, correctionOdomToMap_.matrix().cast<float>());
	std::cout << "..........----......" << std::endl;

	/*
		 // 创建GICP对象
		 pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

		 // 设置KD树加速源点云与目标点云的匹配搜索
		 pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
		 tree1->setInputCloud(source_cloud);
		 pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
		 tree2->setInputCloud(target_cloud);
		 gicp.setSearchMethodSource(tree1);
		 gicp.setSearchMethodTarget(tree2);

		 // 设置GICP的输入源和目标点云
		 gicp.setInputSource(source_cloud);
		 gicp.setInputTarget(target_cloud);

		 // 设置GICP的配准参数
		 gicp.setMaxCorrespondenceDistance(100);      // 设置最大匹配距离
		 gicp.setTransformationEpsilon(1e-10);        // 设置最小变换差异作为终止条件
		 gicp.setEuclideanFitnessEpsilon(0.001);      // 设置收敛条件：均方误差和
		 gicp.setMaximumIterations(iterations);       // 设置最大迭代次数

		 // 执行GICP配准
		 gicp.align(*icp_cloud);
	 */

	// double last_temp_x, last_temp_y, last_temp_z, last_temp_roll, last_temp_pitch, last_temp_yaw;
	// pcl::getTranslationAndEulerAngles(correctionOdomToMap_, last_temp_x, last_temp_y, last_temp_z, last_temp_roll,
	// last_temp_pitch, last_temp_yaw); //  获取上一帧 相对 当前帧的 位姿

	// Eigen::Isometry3d temp_curr_correctionOdomToMap = Eigen::Isometry3d::Identity();
	// temp_curr_correctionOdomToMap.matrix() = gicp_->getFinalTransformation().matrix().cast<double>();
	// double curr_temp_x, curr_temp_y, curr_temp_z, curr_temp_roll, curr_temp_pitch, curr_temp_yaw;
	// pcl::getTranslationAndEulerAngles(temp_curr_correctionOdomToMap, curr_temp_x, curr_temp_y, curr_temp_z,
	// curr_temp_roll, curr_temp_pitch, curr_temp_yaw); //  获取上一帧 相对 当前帧的 位姿

	// cout << YELLOW << "x: "<< last_temp_x << " y: "<< last_temp_y << " z: "<< last_temp_z << " roll: "<<
	// last_temp_roll << " pitch: "<< last_temp_pitch << " yaw: "<< last_temp_yaw << RESET << endl; cout << YELLOW <<
	// "x: "<< curr_temp_x << " y: "<< curr_temp_y << " z: "<< curr_temp_z << " roll: "<< curr_temp_roll << " pitch: "<<
	// curr_temp_pitch << " yaw: "<< curr_temp_yaw << RESET << endl; fit_score = gicp_->getFitnessScore();
	if (gicp_->hasConverged() == false) {
		// if(icp_->hasConverged() ==false){
		// ROS_ERROR_STREAM(RED << "gicp not converged "<<RESET);
		cout << RED << "gicp not converged " << RESET << endl;
		return false;
	} else {
		fit_score = gicp_->getFitnessScore();
		// fit_score = icp_->getFitnessScore();
		if (fit_score < score_low_accuracy_thr) {
			lastCorrectionOdomToMap_ = correctionOdomToMap_;
			lastUpdateTime_ = curr_time_;
			// ROS_INFO_STREAM(GREEN << "gicp success with score "<< gicp_->getFitnessScore() << RESET);
			cout << GREEN << "gicp success with score " << gicp_->getFitnessScore() << RESET << endl;
			// cout <<    GREEN << "gicp success with score "<< icp_->getFitnessScore() << RESET << endl;
			correctionOdomToMap_.matrix() = gicp_->getFinalTransformation().matrix().cast<double>();
			// correctionOdomToMap_.matrix() =  icp_->getFinalTransformation().matrix().cast<double>();

			curr_time_ = omp_get_wtime();

			double last_x, last_y, last_z, last_roll, last_pitch, last_yaw;
			// pcl::getTranslationAndEulerAngles(lastCorrectionOdomToMap_, last_x, last_y, last_z, last_roll,
			// last_pitch, last_yaw); //  获取上一帧 相对 当前帧的 位姿
			Eigen::Affine3d affine_transform(lastCorrectionOdomToMap_);
			pcl::getTranslationAndEulerAngles(affine_transform, last_x, last_y, last_z, last_roll, last_pitch,
											  last_yaw);
			double curr_x, curr_y, curr_z, curr_roll, curr_pitch, curr_yaw;
			Eigen::Affine3d affine_transform_co(correctionOdomToMap_);
			pcl::getTranslationAndEulerAngles(affine_transform_co, curr_x, curr_y, curr_z, curr_roll, curr_pitch,
											  curr_yaw); //  获取上一帧 相对 当前帧的 位姿

			double abs_dx = std::abs(curr_x - last_x);
			double abs_dy = std::abs(curr_y - last_y);

			if (!filter_init_) {
				odom2map_x_filter = correctionOdomToMap_.translation().x();
				odom2map_y_filter = correctionOdomToMap_.translation().y();
				filter_init_ = true;
			} else {
				odom2map_x_filter = ratio * correctionOdomToMap_.translation().x() + (1 - ratio) * odom2map_x_filter;
				odom2map_y_filter = ratio * correctionOdomToMap_.translation().y() + (1 - ratio) * odom2map_y_filter;
				correctionOdomToMap_.translation().x() = odom2map_x_filter;
				correctionOdomToMap_.translation().y() = odom2map_y_filter;
			}

			// if(use_pose_filter){
			//     if(abs_dx > odom2map_delta_thr){
			//         // correctionOdomToMap_.translation().x() = lastCorrectionOdomToMap_.translation().x() + 0.025 *
			//         (curr_x - last_x)/abs_dx; correctionOdomToMap_.translation().x() =
			//         lastCorrectionOdomToMap_.translation().x() + odom2map_delta_set * (curr_x - last_x)/abs_dx;
			//     }
			//     if(abs_dy > odom2map_delta_thr){
			//         // correctionOdomToMap_.translation().y() = lastCorrectionOdomToMap_.translation().y() + 0.025 *
			//         (curr_y - last_y)/abs_dy; correctionOdomToMap_.translation().y() =
			//         lastCorrectionOdomToMap_.translation().y() + odom2map_delta_set * (curr_y - last_y)/abs_dy;
			//     }
			// }

			// pcl::getTranslationAndEulerAngles(correctionOdomToMap_, curr_x, curr_y, curr_z, curr_roll, curr_pitch,
			// curr_yaw); //  获取上一帧 相对 当前帧的 位姿
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
			// ROS_INFO_STREAM(YELLOW << "gicp converged, score: "<< fit_score <<RESET);
			cout << YELLOW << "gicp converged, score: " << fit_score << RESET << endl;
		}

		return true;
	}
}

bool Localization::globalLocalization(PointCloudType::Ptr cloudIn, Eigen::Isometry3d pose, Matrix3d initial_rotate,
									  double score) {
	if (!map_ready_) {
		// ROS_WARN_STREAM(YELLOW << "map not ready" << RESET);
		cout << YELLOW << "map not ready" << RESET << endl;
		return false;
	}
	// std::cout << "-------------------------------------------"<<std::endl;
	Eigen::Vector3d current_euler = R2ypr(initial_rotate); // R2ypr(pose.matrix().block<3, 3>(0, 0));
	// Eigen::Vector3d current_euler = pose.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0);
	// double current_yaw = current_euler[0];
	double current_pitch = current_euler[1];
	double current_roll = current_euler[2];
	// std::cout << "current yaw"<<current_euler[0]*180/M_PI<<" pitch "<<current_euler[1]*180/M_PI<< " roll
	// "<<current_euler[2]*180/M_PI<<std::endl; Eigen::Isometry3d newTransform = Eigen::Isometry3d::Identity();
	// newTransform.rotate(Eigen::AngleAxisd(current_pitch, Eigen::Vector3d::UnitY()));
	// newTransform.rotate(Eigen::AngleAxisd(current_roll, Eigen::Vector3d::UnitX()));
	// std::cout <<newTransform.matrix()<<std::endl;
	// newTransform.matrix().block<3, 3>(0, 0) = ypr2R(Eigen::Vector3d(0,current_pitch,current_roll));
	PointCloudType::Ptr gravityAlignedCLoud(new PointCloudType());
	Eigen::Isometry3d Transform = Eigen::Isometry3d::Identity();
	Transform.matrix().block<3, 3>(0, 0) = initial_rotate;
	*gravityAlignedCLoud = *transformPointCloud(cloudIn, Transform);
	//***************** gravityAlignedCLoud 用来生成当前点云的 sc-info

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

		// /*  for (const auto& number : ringkey) {
		//       std::cout << number << " ";
		//   }
		//   std::cout << std::endl;*/
		Eigen::MatrixXd sectorkey = scManager_->makeSectorkeyFromScancontext(sc);
		double sc_dist = 1.0;
		auto match = scManager_->detectClosestMatch(sc, ringkey, sectorkey, sc_dist);
		if (match.first != -1) {
			std::cout << "trans: " << t.first << " " << t.second;
			std::cout << " score: " << sc_dist << std::endl;
			// ROS_INFO_STREAM("trans: "<< t.first << " " <<t.second <<" score: "<<sc_dist);
		}
		if (sc_dist < min_dist) {
			min_dist = sc_dist;
			best_match = match;
			best_trans = t;
		}
	}
	double t1 = omp_get_wtime();
	// ROS_INFO_STREAM(GREEN << "search_trans cost time: " << (t1-t0) * 1000 << " ms" << RESET);
	cout << GREEN << "search_trans cost time: " << (t1 - t0) * 1000 << " ms" << RESET << endl;

	int match_idx = best_match.first;

	// ICP param-set
	pcl::IterativeClosestPoint<PointType, PointType> icp;
	icp.setMaxCorrespondenceDistance(100);
	icp.setMaximumIterations(100);
	icp.setTransformationEpsilon(1e-6);
	icp.setEuclideanFitnessEpsilon(1e-6);
	icp.setRANSACIterations(0);

	if (match_idx != -1) {
		std::cout << "use index " << match_idx << std::endl;
		// ROS_INFO_STREAM("use index "<< match_idx);
		Eigen::Matrix4d init_guess = LoadData_[match_idx].pose.matrix();
		Eigen::Vector3d euler = R2ypr(init_guess.block<3, 3>(0, 0));
		// Eigen::Vector3d euler = init_guess.block<3, 3>(0, 0).eulerAngles(2, 1, 0);

		euler[0] += -best_match.second;
		std::cout << "rotate yaw: " << -best_match.second << std::endl;
		// ROS_INFO_STREAM("rotate yaw: "<<-best_match.second);
		// Eigen::Matrix3d rotate = (Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitZ()) *
		//                         Eigen::AngleAxisd(current_pitch, Eigen::Vector3d::UnitY()) *
		//                         Eigen::AngleAxisd(current_roll, Eigen::Vector3d::UnitX())).toRotationMatrix();// TODO
		//                         use current pr?
		Eigen::Matrix3d rotate = ypr2R(Eigen::Vector3d(euler[0], current_pitch, current_roll));
		init_guess.block<3, 3>(0, 0) = rotate;
		// std::cout << "original trans"<<init_guess.block<3, 1>(0, 3).transpose()<<std::endl;
		euler = R2ypr(init_guess.block<3, 3>(0, 0));
		// std::cout << "original yaw"<<euler[0]*180/M_PI<<" pitch "<<euler[1]*180/M_PI<< " roll
		// "<<euler[2]*180/M_PI<<std::endl; euler = init_guess.block<3, 3>(0, 0).eulerAngles(2, 1, 0);

		// ICP Settings zx gicp ?
		Eigen::Vector2d offset_in_lidar{ -best_trans.first, -best_trans.second };
		Eigen::Rotation2D<double> rotation(euler[0]);
		Eigen::Vector2d offset_in_map = rotation * offset_in_lidar;
		// std::cout << "lidar offset " << offset_in_lidar.transpose() <<std::endl;
		// std::cout << "map offset " << offset_in_map.transpose() <<std::endl;
		init_guess.coeffRef(0, 3) = init_guess.coeffRef(0, 3) + offset_in_map[0];
		init_guess.coeffRef(1, 3) = init_guess.coeffRef(1, 3) + offset_in_map[1];
		// init_guess.coeffRef(2, 3) = 0;
		// std::cout << "initial yaw "<<euler[0]*180/M_PI<<" pitch "<<euler[1]*180/M_PI<< " roll
		// "<<euler[2]*180/M_PI<<std::endl; std::cout << " trans "<<init_guess.block<3, 1>(0, 3).transpose()<<std::endl;
		// //use the outcome of ndt as the initial guess for ICP
		Eigen::Isometry3d testtransform(init_guess);
		testMatchcloud_ = transformPointCloud(cloudIn, testtransform);
		icp.setInputSource(cloudIn);
		icp.setInputTarget(CloudGlobalMap_);
		// std::cout << "globalLocalization icp fail "<<cloudIn->points.size()<<"
		// "<<CloudGlobalMap_->points.size()<<std::endl;
		PointCloudType::Ptr unused_result(new PointCloudType());
		icp.align(*unused_result, init_guess.cast<float>());
		// 未收敛，或者匹配不够好
		// if (icp.hasConverged() == false || icp.getFitnessScore() > 0.2){//TODO add number in getFitnessScore
		if (icp.hasConverged() == false || icp.getFitnessScore() > score) { // TODO add number in getFitnessScore
			std::cout << "globalLocalization icp fail with score: " << icp.getFitnessScore() << std::endl;
			// ROS_WARN_STREAM(YELLOW << "globalLocalization icp fail with score: "<< icp.getFitnessScore() << RESET);
			return false;
		} else {
			// ROS_INFO_STREAM(GREEN << "globalLocalization success with score: " << icp.getFitnessScore() << RESET);
			cout << GREEN << "globalLocalization success with score: " << icp.getFitnessScore() << RESET << endl;
		}
		Eigen::Isometry3d lidar_in_map;
		lidar_in_map.matrix() = icp.getFinalTransformation().matrix().cast<double>();

		// 初始值的确定和当前的位置无关，但是获取最后的odom-2-map与当前的lidar-in-odom 有关
		// ？？？？ 可以在定位过程中（例：定位失败时）直接启动重定位，而不需要整个重启定位模块，？？？并不能
		// 要确保 lidar odom 没有问题才可以，但是怎么能确定呢？？？
		correctionOdomToMap_ = lidar_in_map * pose.inverse();
		lastUpdateTime_ = omp_get_wtime();
		// euler = lidar_in_map.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0);
		// std::cout << "final yaw"<<euler[0]<<" pitch "<<euler[1]<< " roll "<<euler[2];
		// std::cout << "x "<<lidar_in_map.translation().x()<<" y "<<lidar_in_map.translation().y()<< " z
		// "<<lidar_in_map.translation().z()<<std::endl;

		// float x, y, z, roll, pitch, yaw;
		// pcl::getTranslationAndEulerAngles(correctionOdomToMap_, x, y, z, roll, pitch, yaw); //  获取上一帧 相对
		// 当前帧的 位姿 std::cout << "icp results"<<" "<< x <<" "<< y <<" "<< z <<" "<< yaw <<" "<< pitch <<" "<<
		// roll<<std::endl; std::cout << "-------------------------------------------"<<std::endl;
		double t2 = omp_get_wtime();
		// ROS_INFO_STREAM(GREEN << "icp cost time: " << (t2-t1) * 1000 << " ms" << RESET);
		cout << GREEN << "icp cost time: " << (t2 - t1) * 1000 << " ms" << RESET << endl;
		return true;
		// std::cout << "scancontext search success, score {} " <<match_idx<<" "<< min_dist<<std::endl;
	} else {
		// std::cout << "-------------------------------------------"<<std::endl;
		// ROS_WARN_STREAM("scancontext search fail, score {} "<<match_idx<<" "<< min_dist);
		cout << "scancontext search fail, score {} " << match_idx << " " << min_dist << endl;
		return false;
	}
}

} // namespace lidar_slam

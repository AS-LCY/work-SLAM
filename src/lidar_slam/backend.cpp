#include "lidar_slam/backend.hpp"
namespace lidar_slam {
BackEnd::BackEnd(float dist, float angle, float loop_dist, float loop_time, int loop_skip_key, float loop_icp_score) {
	KeyPoint_.reset(new pcl::PointCloud<PointType>());
	CopyKeyPoint_.reset(new pcl::PointCloud<PointType>());
	show_map_.reset(new pcl::PointCloud<PointType>());
	show_rgb_map_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

	loopIndexQueue_.clear();
	loopPoseQueue_.clear();
	loopNoiseQueue_.clear();

	KeyPoses_.clear();
	CopyKeyPoses_.clear();
	KeyFrameCloud_.clear();

	keyframeDistThreshold_ = dist;
	keyframeAngleThreshold_ = angle;
	loopKeyframeSearchRadius_ = loop_dist;
	loopKeyframeSearchTimeDiff_ = loop_time;
	loopKeyframeSearchSkipKey_ = loop_skip_key;

	loopIcpScore_ = loop_icp_score;

	parameters_.relinearizeThreshold = 0.01; // TODO(jxl): 0.01 ~ 0.05; 阈值越小，精度越高，但计算量越大。
	parameters_.relinearizeSkip = 1;

	isam_ = new gtsam::ISAM2(parameters_);
	downSizeFilterICP_.setLeafSize(0.4, 0.4, 0.4); // TODO(jxl): 配置文件来设置
	aLoopIsClosed_ = false;

	gravityAlignedCLoud_.reset(new PointCloudType());
}

BackEnd::~BackEnd() {}
bool BackEnd::saveFrame(Eigen::Isometry3d transformTobeMapped) {
	if (KeyPoint_->points.empty()) return true;
	Eigen::Affine3f transBetween;
	Eigen::Isometry3d temp = KeyPoses_.back().pose.inverse() * transformTobeMapped;
	transBetween = temp.cast<float>();
	float x, y, z, roll, pitch, yaw;
	pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);
	if (abs(roll) < keyframeAngleThreshold_ && abs(pitch) < keyframeAngleThreshold_ &&
		abs(yaw) < keyframeAngleThreshold_ && sqrt(x * x + y * y + z * z) < keyframeDistThreshold_)
		return false;
	return true;
}

void BackEnd::addOdomFactor(Eigen::Isometry3d transformTobeMapped) {
	if (KeyPoint_->points.empty()) {
		// 第一帧初始化先验因子
		gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances(
			(gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished());
		gtSAMgraph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(transformTobeMapped.matrix()), priorNoise));

		initialEstimate_.insert(0, gtsam::Pose3(transformTobeMapped.matrix()));
	} else {
		gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =
			gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
		// TODO(jxl): noise有点小，也要添加loss func

		gtsam::Pose3 poseFrom(KeyPoses_.back().pose.matrix()); /// pre
		gtsam::Pose3 poseTo(transformTobeMapped.matrix());	   // cur
		// 参数：前一帧id，当前帧id，前一帧与当前帧的位姿变换（作为观测值），噪声协方差
		gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(KeyPoint_->size() - 1, KeyPoint_->size(),
														   poseFrom.between(poseTo), odometryNoise));
		// 变量节点设置初始值
		initialEstimate_.insert(KeyPoint_->size(), poseTo);
	}
}

void BackEnd::addLoopFactor() {
	if (loopIndexQueue_.empty()) return;
	// 闭环队列
	for (int i = 0; i < (int)loopIndexQueue_.size(); ++i) {
		// 闭环边对应两帧的索引
		int indexFrom = loopIndexQueue_[i].first; //   cur
		int indexTo = loopIndexQueue_[i].second;  //    pre
		// 闭环边的位姿变换
		gtsam::Pose3 poseBetween = loopPoseQueue_[i];
		gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue_[i];
		gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
	}

	// ROS_INFO_STREAM(BOLDRED<<"addLoopFactor, loopIndexQueue_ size = " << loopIndexQueue_.size() <<"
	// *************************** "<<RESET);
	//  mtxLoopInfo_.lock(); // TODO this cause CPU high
	std::unique_lock<std::mutex> lk(mtxLoopInfo_); // TODO(jxl): 在函数一进来就应该就上锁
	loopIndexQueue_.clear();
	loopPoseQueue_.clear();
	loopNoiseQueue_.clear();
	aLoopIsClosed_ = true;
	//   mtxLoopInfo_.unlock();
}

//在lio的线程中运行
bool BackEnd::saveKeyFramesAndFactor(Eigen::Isometry3d transformTobeMapped, PointCloudType::Ptr lidar_cloud,
									 double time) {
	if (!saveFrame(transformTobeMapped)) { //是否关键帧
		return false;
	}

	addOdomFactor(transformTobeMapped); // T_odom_lidar
	// addGPSFactor();

	addLoopFactor();

	isam_->update(gtSAMgraph_, initialEstimate_);
	isam_->update();

	if (aLoopIsClosed_) // 有回环因子，多update几次
	{
		isam_->update();
		isam_->update();
		isam_->update();
		isam_->update();
		isam_->update();
	}
	// update之后要清空一下保存的因子图，注：历史数据不会清掉，ISAM保存起来了
	gtSAMgraph_.resize(0);
	initialEstimate_.clear();

	PointType thisPose3D;
	KeyPose thisPose6D;
	gtsam::Pose3 latestEstimate;

	// 优化结果
	isamCurrentEstimate_ = isam_->calculateBestEstimate();
	latestEstimate = isamCurrentEstimate_.at<gtsam::Pose3>(isamCurrentEstimate_.size() - 1);
	thisPose3D.x = latestEstimate.translation().x();
	thisPose3D.y = latestEstimate.translation().y();
	thisPose3D.z = latestEstimate.translation().z();

	thisPose3D.intensity = KeyPoint_->size(); // 索引
	mtxPose_.lock();
	KeyPoint_->push_back(thisPose3D); //  新关键帧帧放入队列中

	// cloudKeyPoses6D加入当前帧位姿
	thisPose6D.pose = Eigen::Isometry3d(latestEstimate.matrix());
	thisPose6D.index = thisPose3D.intensity;
	thisPose6D.time = time;
	thisPose6D.roll = latestEstimate.rotation().roll();
	thisPose6D.pitch = latestEstimate.rotation().pitch();
	thisPose6D.yaw = latestEstimate.rotation().yaw();
	KeyPoses_.push_back(thisPose6D);
	mtxPose_.unlock();

	return true;
}

void BackEnd::saveCurrentCloud(PointCloudType::Ptr points, Eigen::Isometry3d pose) {
	auto start = std::chrono::high_resolution_clock::now();
	PointCloudType::Ptr currentCLoud(new PointCloudType());
	pcl::copyPointCloud(*points, *currentCLoud); // TODO(jxl): 没必要拷贝来拷贝去
	{
		std::unique_lock<std::mutex> lk(mtxCloud_);
		KeyFrameCloud_.emplace_back(currentCLoud);
	}
	//   Eigen::Vector3d euler =  pose.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0);
	Eigen::Vector3d euler = R2ypr(pose.matrix().block<3, 3>(0, 0)); // TODO(jxl): 和eigen的接口计算原理一样，为何要重写
	euler[0] = 0;
	Eigen::Isometry3d Transform = Eigen::Isometry3d::Identity();
	Transform.matrix().block<3, 3>(0, 0) = ypr2R(euler);
	gravityAlignedCLoud_.reset(new PointCloudType());
	*gravityAlignedCLoud_ = *transformPointCloud(currentCLoud, Transform); //关键帧点云转成和重力对齐
	scManager_.makeAndSaveScancontextAndKeys(*gravityAlignedCLoud_);
}

bool BackEnd::correctPoses() {
	auto start = std::chrono::high_resolution_clock::now();
	if (KeyPoint_->points.empty()) return false;
	if (aLoopIsClosed_) {
		int numPoses = isamCurrentEstimate_.size();
		mtxPose_.lock();
		for (int i = 0; i < numPoses; ++i) {
			KeyPoint_->points[i].x = isamCurrentEstimate_.at<gtsam::Pose3>(i).translation().x();
			KeyPoint_->points[i].y = isamCurrentEstimate_.at<gtsam::Pose3>(i).translation().y();
			KeyPoint_->points[i].z = isamCurrentEstimate_.at<gtsam::Pose3>(i).translation().z();

			KeyPoses_[i].pose = Eigen::Isometry3d(isamCurrentEstimate_.at<gtsam::Pose3>(i).matrix());
			KeyPoses_[i].roll = isamCurrentEstimate_.at<gtsam::Pose3>(i).rotation().roll();
			KeyPoses_[i].pitch = isamCurrentEstimate_.at<gtsam::Pose3>(i).rotation().pitch();
			KeyPoses_[i].yaw = isamCurrentEstimate_.at<gtsam::Pose3>(i).rotation().yaw();
		}
		mtxPose_.unlock();
		aLoopIsClosed_ = false;
		show_index_ = 0;
		std::unique_lock<std::mutex> lk(mtxCurrentMap_);
		show_map_->clear();

		auto end = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		TRACE_INFO_CLASS("backend correct pose cost time: %f ms", double(duration.count()));

		return true;
	}
	return false;
}

void BackEnd::recontructIKdTree(KD_TREE<PointType>& ikdtree, double kdTreeReconstructRadius,
								float kdTreeReconstructKeyFrameLeafSize, double kdTreeReconstructPointLeafSize) {
	auto start = std::chrono::high_resolution_clock::now();
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMapPoses(new pcl::KdTreeFLANN<PointType>());
	PointCloudType::Ptr subMapKeyPoses(new PointCloudType());
	PointCloudType::Ptr subMapKeyPosesDS(new PointCloudType());
	PointCloudType::Ptr subMapKeyFrames(new PointCloudType());
	PointCloudType::Ptr subMapKeyFramesDS(new PointCloudType());

	// kdtree查找最近一帧关键帧相邻的关键帧集合
	std::vector<int> pointSearchIndGlobalMap;
	std::vector<float> pointSearchSqDisGlobalMap;
	mtxPose_.lock();
	kdtreeGlobalMapPoses->setInputCloud(KeyPoint_);
	kdtreeGlobalMapPoses->radiusSearch(KeyPoint_->back(), kdTreeReconstructRadius, pointSearchIndGlobalMap,
									   pointSearchSqDisGlobalMap, 0);

	for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
		subMapKeyPoses->push_back(KeyPoint_->points[pointSearchIndGlobalMap[i]]); //  subMap的pose集合
	mtxPose_.unlock();

	pcl::VoxelGrid<PointType> downSizeFilterSubMapKeyPoses;
	downSizeFilterSubMapKeyPoses.setLeafSize(kdTreeReconstructKeyFrameLeafSize, kdTreeReconstructKeyFrameLeafSize,
											 kdTreeReconstructKeyFrameLeafSize); // for global map visualization
	downSizeFilterSubMapKeyPoses.setInputCloud(subMapKeyPoses);
	downSizeFilterSubMapKeyPoses.filter(*subMapKeyPosesDS); //  subMap poses  downsample

	// 提取局部相邻关键帧对应的特征点云
	for (int i = 0; i < (int)subMapKeyPosesDS->size(); ++i) {
		int thisKeyInd = (int)subMapKeyPosesDS->points[i].intensity;

		// TODO(jxl): KeyFrameCloud的锁，KeyPoses的锁
		*subMapKeyFrames += *transformPointCloud(KeyFrameCloud_[thisKeyInd],
												 KeyPoses_[thisKeyInd].pose); //  fast_lio only use surfCloud
	}
	// 降采样，发布
	pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
	downSizeFilterGlobalMapKeyFrames.setLeafSize(kdTreeReconstructPointLeafSize, kdTreeReconstructPointLeafSize,
												 kdTreeReconstructPointLeafSize); // for global map visualization
	downSizeFilterGlobalMapKeyFrames.setInputCloud(subMapKeyFrames);
	downSizeFilterGlobalMapKeyFrames.filter(*subMapKeyFramesDS);

	ikdtree.reconstruct(subMapKeyFramesDS->points);
	TRACE_INFO_CLASS("Reconstructed  ikdtree ");

	int featsFromMapNum = ikdtree.validnum();
	int kdtree_size_st = ikdtree.size();
}

bool BackEnd::detectLoopClosureDistance(int* latestID, int* closestID, double time) {
	// 当前关键帧帧
	int loopKeyCur = CopyKeyPoint_->size() - 1; //  当前关键帧索引
	int loopKeyPre = -1;

	// 当前帧已经添加过闭环对应关系，不再继续添加
	auto it = loopIndexContainer_.find(loopKeyCur);
	if (it != loopIndexContainer_.end()) {
		return false;
	}
	// 在历史关键帧中查找与当前关键帧距离最近的关键帧集合
	std::vector<int> pointSearchIndLoop;	 //  候选关键帧索引
	std::vector<float> pointSearchSqDisLoop; //  候选关键帧距离
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses(new pcl::KdTreeFLANN<PointType>());
	kdtreeHistoryKeyPoses->setInputCloud(CopyKeyPoint_); //  历史帧构建kdtree
	kdtreeHistoryKeyPoses->radiusSearch(CopyKeyPoint_->back(), loopKeyframeSearchRadius_, pointSearchIndLoop,
										pointSearchSqDisLoop, 0);
	// 在候选关键帧集合中，找到与当前帧时间相隔较远的帧，设为候选匹配帧
	for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i) {
		int id = pointSearchIndLoop[i];
		if (abs(KeyPoses_[id].time - time) > loopKeyframeSearchTimeDiff_ &&
			abs(loopKeyCur - id) > loopKeyframeSearchSkipKey_) {
			loopKeyPre = id;
			break;
		}
	}
	if (loopKeyPre == -1 || loopKeyCur == loopKeyPre) return false;
	*latestID = loopKeyCur;
	*closestID = loopKeyPre;

	return true;
}

//提取key索引的关键帧前后相邻若干帧的关键帧特征点集合，降采样
void BackEnd::loopFindNearKeyframes(PointCloudType::Ptr& nearKeyframes, const int& key, const int& searchNum) {
	// 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合
	nearKeyframes->clear();
	int cloudSize = CopyKeyPoses_.size();
	auto keyframes_size = KeyFrameCloud_.size();

	for (int i = -searchNum; i <= searchNum; ++i) {
		int keyNear = key + i;
		if (keyNear < 0 || keyNear >= cloudSize) continue;
		if (keyNear < 0 || keyNear >= keyframes_size) continue;
		*nearKeyframes += *transformPointCloud(KeyFrameCloud_[keyNear], CopyKeyPoses_[keyNear].pose);
	}

	if (nearKeyframes->empty()) return;

	// 降采样
	PointCloudType::Ptr cloud_temp(new PointCloudType());
	downSizeFilterICP_.setInputCloud(nearKeyframes);
	downSizeFilterICP_.filter(*cloud_temp);
	*nearKeyframes = *cloud_temp;
}
void BackEnd::loopFindNearKeyframesWithRespectTo(PointCloudType::Ptr& nearKeyframes, const int& key,
												 const int& searchNum, const int _wrt_key) {
	// extract near keyframes
	nearKeyframes->clear();
	int cloudSize = KeyPoses_.size();
	for (int i = -searchNum; i <= searchNum; ++i) {
		int keyNear = key + i;
		if (keyNear < 0 || keyNear >= cloudSize) continue;
		*nearKeyframes += *transformPointCloud(KeyFrameCloud_[keyNear], KeyPoses_[_wrt_key].pose);
	}

	if (nearKeyframes->empty()) return;

	// downsample near keyframes
	PointCloudType::Ptr cloud_temp(new PointCloudType());
	downSizeFilterICP_.setInputCloud(nearKeyframes);
	downSizeFilterICP_.filter(*cloud_temp);
	*nearKeyframes = *cloud_temp;
}

// sec_mapping模式下，在启动后端线程之前，加载之前建图的meta信息以及全局定位初始化信息
bool BackEnd::set_loaded_key_clouds(std::vector<PointCloudType::Ptr> input_vec_key_clouds,
									std::vector<ScInfo, Eigen::aligned_allocator<ScInfo>> input_vec_sc_info,
									std::vector<KeyPose, Eigen::aligned_allocator<KeyPose>> input_vec_key_poses,
									Eigen::Isometry3d T_map_odom) {
	KeyPoses_.clear();
	KeyPoint_.reset(new pcl::PointCloud<PointType>());

	KeyFrameCloud_.assign(input_vec_key_clouds.begin(), input_vec_key_clouds.end());

	// 加载的 pose 是当前 map 坐标系下的，T_map_lidar = input_key_pose
	// 需要将其转换到 当前的 odom 坐标系下，T_odom_lidar（未知量）
	// T_map_odom： 传入的这个值是重定位结果
	TRACE_INFO_CLASS("loaded_key_poses size: %d", input_vec_key_poses.size());

	int i = 0;
	for (auto& kp : input_vec_key_poses) {
		Eigen::Isometry3d T_map_lidar = kp.pose;
		Eigen::Isometry3d T_odom_lidar = T_map_odom.inverse() * T_map_lidar;
		Eigen::Vector3d euler = R2ypr(T_odom_lidar.matrix().block<3, 3>(0, 0));

		addOdomFactor(T_odom_lidar);

		KeyPose temp_pose;
		temp_pose.pose = T_odom_lidar;
		temp_pose.index = kp.index;
		temp_pose.time = kp.time;
		temp_pose.yaw = euler[0];
		temp_pose.pitch = euler[1];
		temp_pose.roll = euler[2];
		KeyPoses_.push_back(temp_pose);

		PointType temp_pnt;
		temp_pnt.x = T_odom_lidar.translation().x();
		temp_pnt.y = T_odom_lidar.translation().y();
		temp_pnt.z = T_odom_lidar.translation().z();
		KeyPoint_->points.push_back(temp_pnt);

		////// saveCurrentCloud(KeyFrameCloud_[i], T_map_lidar);
		scManager_.loadScancontextAndKeys(input_vec_sc_info[i].polarcontext);
	}

	isam_->update(gtSAMgraph_, initialEstimate_);
	gtSAMgraph_.resize(0);
	initialEstimate_.clear();

	loaded_key_clouds_ready_ = true;
	return true;
}

void BackEnd::performLoopClosure(double time) {
	if (KeyPoint_->points.empty() == true) {
		return;
	}

	mtxPose_.lock();
	CopyKeyPoint_->clear();
	*CopyKeyPoint_ = *KeyPoint_;
	CopyKeyPoses_.clear();
	CopyKeyPoses_ = KeyPoses_;
	mtxPose_.unlock();

	auto loop_detected_start = std::chrono::high_resolution_clock::now();

	int loopKeyCur;
	int loopKeyPre;
	// 在历史关键帧中查找与当前关键帧距离最近的关键帧集合，选择时间相隔较远的一帧作为候选闭环帧
	if (!detectLoopClosureDistance(&loopKeyCur, &loopKeyPre, time)) {
		return;
	}
	TRACE_INFO_CLASS("potential detected loop between %d and %d.", loopKeyCur, loopKeyPre);
	auto loop_detected_end = std::chrono::high_resolution_clock::now();
	auto loop_detect_duration =
		std::chrono::duration_cast<std::chrono::milliseconds>(loop_detected_end - loop_detected_start);
	TRACE_INFO_CLASS("backend loop detect cost time: %f ms", double(loop_detect_duration.count()));

	// 提取
	PointCloudType::Ptr cureKeyframeCloud(new PointCloudType()); //  cue keyframe
	PointCloudType::Ptr prevKeyframeCloud(new PointCloudType()); //   history keyframe submap
	{
		// 提取当前关键帧特征点集合，降采样
		loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0); //  将cur keyframe 转换到world系下
		// 提取闭环匹配关键帧前后相邻若干帧的关键帧特征点集合，降采样
		loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, 20); //  选取historyKeyframeSearchNum个keyframe拼成submap
	}

	// ICP Settings zx gicp ?
	pcl::IterativeClosestPoint<PointType, PointType> icp;
	icp.setMaxCorrespondenceDistance(loopKeyframeSearchRadius_ * 2);
	icp.setMaximumIterations(100);
	icp.setTransformationEpsilon(1e-6);
	icp.setEuclideanFitnessEpsilon(1e-6);
	icp.setRANSACIterations(0);

	// scan-to-map，调用icp匹配
	icp.setInputSource(cureKeyframeCloud); // source和target都是在world系下
	icp.setInputTarget(prevKeyframeCloud);
	PointCloudType::Ptr unused_result(new PointCloudType());
	icp.align(*unused_result);
	auto loop_edge_match_end = std::chrono::high_resolution_clock::now();
	auto loop_edge_match_duration =
		std::chrono::duration_cast<std::chrono::milliseconds>(loop_edge_match_end - loop_detected_end);
	TRACE_INFO_CLASS("backend loop_edge match cost time: %f ms", double(loop_edge_match_duration.count()));

	// 未收敛，或者匹配不够好
	if (icp.hasConverged() == false || icp.getFitnessScore() > loopIcpScore_) {
		TRACE_WARN_CLASS("loop closure between %d and %d, icp hasConverged: %d, fitness score: %f > thresh: %f",
						 loopKeyCur, loopKeyPre, icp.hasConverged(), icp.getFitnessScore(), loopIcpScore_);
		return; // TODO(jxl): 统计分数时，应该设置inlier阈值
	}

	TRACE_INFO_CLASS("true loop found! between %d and %d.", loopKeyCur, loopKeyPre);

	// 闭环优化得到的当前关键帧与闭环关键帧之间的位姿变换
	float x, y, z, roll, pitch, yaw;
	Eigen::Affine3f correctionLidarFrame;
	correctionLidarFrame = icp.getFinalTransformation();
	Eigen::Affine3f tWrong = CopyKeyPoses_[loopKeyCur].pose.cast<float>(); // 闭环优化前当前帧位姿
	Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;			   // 闭环优化后当前帧位姿
	gtsam::Pose3 poseFrom = gtsam::Pose3(tCorrect.matrix().cast<double>());
	gtsam::Pose3 poseTo = gtsam::Pose3(CopyKeyPoses_[loopKeyPre].pose.matrix().cast<double>());
	gtsam::Vector Vector6(6);
	float noiseScore = icp.getFitnessScore();
	Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
	gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
	TRACE_INFO_CLASS("loop closure noise score: %f", noiseScore);

	// 添加闭环因子需要的数据
	std::unique_lock<std::mutex> lk(mtxLoopInfo_);
	loopIndexQueue_.push_back(make_pair(loopKeyCur, loopKeyPre));
	loopPoseQueue_.push_back(poseFrom.between(poseTo));
	loopNoiseQueue_.push_back(constraintNoise);
	loopIndexContainer_[loopKeyCur] = loopKeyPre; //   使用hash map 存储回环对
}

// void BackEnd::UpdateImage(const cv::Mat &image,Eigen::Isometry3d lidar_pose)
// {
//     PointCloudType lidar_cloud_in_map;
//     if (KeyPoses_.size() == 0)
//        return;
//     {
//         std::unique_lock<std::mutex> lk(mtxCloud_);
//         std::unique_lock<std::mutex> lk2(mtxPose_);
//         int size = min((int)KeyPoses_.size(),(int)KeyFrameCloud_.size());
//         lidar_cloud_in_map = *transformPointCloud(KeyFrameCloud_[size-1],KeyPoses_[size-1].pose);

//     }
//     Eigen::Isometry3d map_to_lidar = lidar_pose.inverse();
//     Eigen::Matrix3d lidar_to_camera_rotate;
//      lidar_to_camera_rotate  << 1,0,0,
//                              0,0.422618,-0.906308,
//                              0,0.906308,0.422618;
//     Eigen::Vector3d lidar_to_camera_trans;
//     lidar_to_camera_trans << 0, -0.07625, -0.0476;
//     Eigen::Isometry3d lidar_to_camera = Eigen::Isometry3d::Identity();
//     lidar_to_camera.matrix().block<3, 3>(0, 0) = lidar_to_camera_rotate;
//     lidar_to_camera.matrix().block<3, 1>(3, 0) = lidar_to_camera_trans;
//     Eigen::Isometry3d map_to_camera = lidar_to_camera * map_to_lidar;
//     // pcl::PointCloud<pcl::PointXYZRGBNormal> rgb_cloud;

//     std::vector<cv::Point3f> pts_3d;
//     for (size_t i = 0; i < lidar_cloud_in_map.size(); i += 1) {
//         pcl::PointXYZINormal point_3d = lidar_cloud_in_map.points[i];
//         Eigen::Vector3d point_in_map = Eigen::Vector3d(point_3d.x, point_3d.y, point_3d.z);
//         Eigen::Vector3d point_in_camera = map_to_camera.matrix().block<3, 3>(0, 0) * point_in_map +
//         map_to_camera.matrix().block<3, 1>(3, 0);

//         if (point_in_camera[2] > 0) {
//         pts_3d.emplace_back(cv::Point3f(point_in_map.x(),point_in_map.y(), point_in_map.z()));
//         }
//     }
//     Eigen::Vector3d euler = map_to_camera.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0);
//     Eigen::Matrix<double, 6, 1> extrinsic_params;
//     extrinsic_params[0] = euler[0];
//     extrinsic_params[1] = euler[1];
//     extrinsic_params[2] = euler[2];
//     extrinsic_params[3] = map_to_camera.matrix().coeffRef(0, 3);
//     extrinsic_params[4] = map_to_camera.matrix().coeffRef(1, 3);
//     extrinsic_params[5] = map_to_camera.matrix().coeffRef(2, 3);
//     Eigen::AngleAxisd rotation_vector3;
//     rotation_vector3 =
//         Eigen::AngleAxisd(extrinsic_params[0], Eigen::Vector3d::UnitZ()) *
//         Eigen::AngleAxisd(extrinsic_params[1], Eigen::Vector3d::UnitY()) *
//         Eigen::AngleAxisd(extrinsic_params[2], Eigen::Vector3d::UnitX());
//     cv::Mat camera_matrix =
//         (cv::Mat_<double>(3, 3) << 500, 0.0, 960, 0.0, 500, 600, 0.0, 0.0, 1.0);
//     cv::Mat distortion_coeff =
//         (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);
//     cv::Mat r_vec =
//         (cv::Mat_<double>(3, 1)
//             << rotation_vector3.angle() * rotation_vector3.axis().transpose()[0],
//         rotation_vector3.angle() * rotation_vector3.axis().transpose()[1],
//         rotation_vector3.angle() * rotation_vector3.axis().transpose()[2]);

//     cv::Mat t_vec = (cv::Mat_<double>(3, 1) << extrinsic_params[3],
//                     extrinsic_params[4], extrinsic_params[5]);
//     std::vector<cv::Point2f> pts_2d;
//     cv::projectPoints(pts_3d, r_vec, t_vec, camera_matrix, distortion_coeff,
//                         pts_2d);
//     int image_rows = 1920;
//     int image_cols = 1200;
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud;
//     color_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
//         new pcl::PointCloud<pcl::PointXYZRGB>);
//     for (size_t i = 0; i < pts_2d.size(); i++) {
//         if (pts_2d[i].x >= 0 && pts_2d[i].x < image_cols && pts_2d[i].y >= 0 &&
//             pts_2d[i].y < image_rows) {
//         cv::Scalar color =
//             image.at<cv::Vec3b>((int)pts_2d[i].y, (int)pts_2d[i].x);
//         /* if (color[0] == 0 && color[1] == 0 && color[2] == 0) {
//             continue;
//         }*/
//         /* if (pts_3d[i].x > 100) {
//             continue;
//         }*/
//         pcl::PointXYZRGB p;
//         p.x = pts_3d[i].x;
//         p.y = pts_3d[i].y;
//         p.z = pts_3d[i].z;
//         // p.a = 255;
//         p.b = color[0];
//         p.g = color[1];
//         p.r = color[2];
//         color_cloud->points.push_back(p);
//         }
//     }
//         std::unique_lock<std::mutex> lk(mtxCurrentRGBMap_);
//         *show_rgb_map_   += *color_cloud;
//         double resolution = 0.1;
//         pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilter;
//         downSizeFilter.setInputCloud(show_rgb_map_);
//         downSizeFilter.setLeafSize(resolution, resolution, resolution);
//         downSizeFilter.filter(*show_rgb_map_);
// }

/*  void performSCLoopClosure()
  {
	  ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time); //  时间戳
	  string odometryFrame = "odom";
	  if (cloudKeyPoses3D->points.empty() == true)
		  return;
	  mtx.lock();
	  *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
	  *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
	  mtx.unlock();

	  // find keys
	  auto detectResult = scManager_.detectLoopClosureID(); // first: nn index, second: yaw diff
	  int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;;
	  int loopKeyPre = detectResult.first;
	  float yawDiffRad = detectResult.second; // not use for v1 (because pcl icp withi initial somthing wrong...)
	  if( loopKeyPre == -1 )
		  return;

   //   std::cout << "SC loop found! between " << loopKeyCur << " and " << loopKeyPre << "." << std::endl; // giseop

	  // extract cloud
	  PointCloudType::Ptr cureKeyframeCloud(new PointCloudType());
	  PointCloudType::Ptr prevKeyframeCloud(new PointCloudType());
	  {
		  // loopFindNearKeyframesWithRespectTo(cureKeyframeCloud, loopKeyCur, 0, loopKeyPre); // giseop
		  // loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);

		  int base_key = 0;
		 // loopFindNearKeyframesWithRespectTo(cureKeyframeCloud, loopKeyCur, 0, base_key); // giseop
		 // loopFindNearKeyframesWithRespectTo(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, base_key); //
  giseop
			 // 提取当前关键帧特征点集合，降采样
		  loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
  // 提取闭环匹配关键帧前后相邻若干帧的关键帧特征点集合，降采样
		  loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
		  if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
			  return;
		  if (pubHistoryKeyFrames.getNumSubscribers() != 0)
			  publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
	  }

	  // ICP Settings
	  static pcl::IterativeClosestPoint<PointType, PointType> icp;
	  icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
	  icp.setMaximumIterations(100);
	  icp.setTransformationEpsilon(1e-6);
	  icp.setEuclideanFitnessEpsilon(1e-6);
	  icp.setRANSACIterations(0);

	  // Align clouds
	  icp.setInputSource(cureKeyframeCloud);
	  icp.setInputTarget(prevKeyframeCloud);
	  PointCloudType::Ptr unused_result(new PointCloudType());
	  icp.align(*unused_result);
	  // giseop
	  // TODO icp align with initial

	  if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore) {
		  std::cout << "ICP fitness test failed (" << icp.getFitnessScore() << " > " << historyKeyframeFitnessScore <<
  "). Reject this SC loop." << std::endl; return; } else { std::cout << "ICP fitness test passed (" <<
  icp.getFitnessScore() << " < " << historyKeyframeFitnessScore << "). Add this SC loop." << std::endl;
	  }
	  std::cout << "SC loop found! between " << loopKeyCur << " and " << loopKeyPre << "." << std::endl; // giseop
	  // publish corrected cloud
	  if (pubIcpKeyFrames.getNumSubscribers() != 0)
	  {
		  PointCloudType::Ptr closed_cloud(new PointCloudType());
		  pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
		  publishCloud(&pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
	  }

	  // Get pose transformation
	  float x, y, z, roll, pitch, yaw;
	  Eigen::Isometry3d correctionLidarFrame;
	  correctionLidarFrame = icp.getFinalTransformation();

	  // // transform from world origin to wrong pose
	   Eigen::Isometry3d tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
	  // // transform from world origin to corrected pose
	   Eigen::Isometry3d tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a
  fixed frame pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw); gtsam::Pose3 poseFrom =
  gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z)); gtsam::Pose3 poseTo =
  pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);

	   gtsam::Vector Vector6(6);
	   float noiseScore = icp.getFitnessScore();
	   Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
	   gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);

	  // giseop
   //   pcl::getTranslationAndEulerAngles (correctionLidarFrame, x, y, z, roll, pitch, yaw);
	//  gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
	//  gtsam::Pose3 poseTo = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));

	  // giseop, robust kernel for a SC loop
	 // float robustNoiseScore = 0.5; // constant is ok...
	 // gtsam::Vector robustNoiseVector6(6);
	  //robustNoiseVector6 << robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore,
  robustNoiseScore;
	 // gtsam::noiseModel::Diagonal::shared_ptr robustConstraintNoise =
  gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6);
	//  gtsam::noiseModel::Base::shared_ptr robustConstraintNoise;
	 // robustConstraintNoise = gtsam::noiseModel::Robust::Create(
	   //   gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure, but
  with a good front-end loop detector, Cauchy is empirically enough.
		//  gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6)
	//  ); // - checked it works. but with robust kernel, map modification may be delayed (i.e,. requires more
  true-positive loop factors)

	  // Add pose constraint
	  mtx.lock();
	  loopIndexQueue_.push_back(make_pair(loopKeyCur, loopKeyPre));
	  loopPoseQueue_.push_back(poseFrom.between(poseTo));
	  loopNoiseQueue_.push_back(constraintNoise);
	  mtx.unlock();

	  // add loop constriant
	//   loopIndexContainer_[loopKeyCur] = loopKeyPre;
	  loopIndexContainer_.insert(std::pair<int, int>(loopKeyCur, loopKeyPre)); // giseop for multimap
  } */

pcl::PointCloud<pcl::PointXYZRGB>::Ptr BackEnd::getCurrentRGBMap() {
	std::unique_lock<std::mutex> lk(mtxCurrentRGBMap_);
	return show_rgb_map_;
}

PointCloudType::Ptr BackEnd::getCurrentMap(Eigen::Isometry3d T_map_odom) {
	std::unique_lock<std::mutex> lk(mtxCurrentMap_); // TODO(jxl): 该锁是锁show_map，show_index,
	// PointCloudType::Ptr globalSurfCloudDS(new PointCloudType());

	// TODO(jxl): mtxCloud_是锁KeyFrameCloud，KeyPoses也有自己的锁
	if (KeyPoses_.size() == 0) return show_map_;
	{
		std::unique_lock<std::mutex> lk(mtxCloud_);
		std::unique_lock<std::mutex> lk2(mtxPose_);
		int size = min((int)KeyPoses_.size(), (int)KeyFrameCloud_.size());
		for (int i = show_index_; i < size; i++) {
			*show_map_ += *transformPointCloud(KeyFrameCloud_[i], T_map_odom * KeyPoses_[i].pose);
		}
	}
	show_index_ = (int)KeyPoses_.size() - 1;
	double resolution = 0.1;
	pcl::VoxelGrid<PointType> downSizeFilter;
	downSizeFilter.setInputCloud(show_map_);
	downSizeFilter.setLeafSize(resolution, resolution, resolution);
	downSizeFilter.filter(*show_map_);
	return show_map_;
}

bool BackEnd::saveMap(string saveMapDirectory, double resolution, Eigen::Isometry3d T_map_odom, int start_index,
					  int end_index) {
	if (KeyPoses_.empty() || KeyPoses_.size() == 0) {
		TRACE_ERR_CLASS("key frame empty");
		return false;
	}

	// 检查并创建 yaml 中的地图路径
	if (create_directory_if_not_exists(saveMapDirectory)) {
		TRACE_INFO_CLASS("Directory created or already exists:  %s", saveMapDirectory.c_str());
	} else {
		TRACE_INFO_CLASS("Failed to create directory:  %s", saveMapDirectory.c_str());
		return false;
	}

	// 创建关键帧点云保存路径
	std::string save_key_frame_cloud_dir = saveMapDirectory + "/key_frame_cloud/";
	if (create_directory_if_not_exists(save_key_frame_cloud_dir)) {
		TRACE_INFO_CLASS("Directory created or already exists:  %s", save_key_frame_cloud_dir.c_str());
	} else {
		TRACE_ERR_CLASS("Failed to create directory:  %s", save_key_frame_cloud_dir.c_str());
		return false;
	}

	std::string pcd_file_path = "";

	PointCloudType::Ptr globalMapCloud(new PointCloudType());
	PointCloudType::Ptr globalSurfCloudDS(new PointCloudType());
	ScInfo infos[(int)KeyPoses_.size()];

	// 注意：拼接地图时，keyframe是lidar系，而fastlio更新后的存到的cloudKeyPoses6D 关键帧位姿是body系下的，需要把
	// cloudKeyPoses6D  转换为T_world_lidar 。 T_world_lidar = T_world_body * T_body_lidar , T_body_lidar 是外参
	int start = 0, end = 0;
	int KeyPosesSize = (int)KeyPoses_.size();
	pcd_file_path = saveMapDirectory + "/cloud_map.pcd";
	if (start_index == 0 && end_index == 0) {
		start = 0;
		end = KeyPosesSize - 1;
	} else if (start_index == -1 || end_index == -1) {
		TRACE_INFO_CLASS("start-point or end-point not set, save all to cloud_map.pcd ");
		start = 0;
		end = KeyPosesSize - 1;
	} else {
		start = start_index;
		end = end_index;
		if (end > KeyPosesSize - 1) {
			end = KeyPosesSize - 1;
		}
	}
	std::string key_frame_cloud_path = "";
	for (int i = start; i <= end; i++) {
		// 生成地图
		*globalMapCloud += *transformPointCloud(KeyFrameCloud_[i], T_map_odom * KeyPoses_[i].pose);

		// ScanContex 信息组合获取
		ScInfo info;
		info.id = i;
		info.pose = T_map_odom * KeyPoses_[i].pose;
		info.polarcontext = scManager_.getSc(i);
		infos[i] = info;

		// 保存关键帧点云
		key_frame_cloud_path = save_key_frame_cloud_dir + std::to_string(i) + ".pcd";
		int success = pcl::io::savePCDFileBinary(key_frame_cloud_path, *KeyFrameCloud_[i]);
	}
	TRACE_INFO_CLASS("Save resolution:  %f", resolution);

	pcl::VoxelGrid<PointType> downSizeFilter;
	downSizeFilter.setInputCloud(globalMapCloud);
	downSizeFilter.setLeafSize(resolution, resolution, resolution);
	downSizeFilter.filter(*globalSurfCloudDS);

	TRACE_INFO_CLASS("cloud_map size:  %d", (int)globalSurfCloudDS->points.size());
	TRACE_INFO_CLASS("Saving map to pcd file:   %s", pcd_file_path.c_str());
	int ret = pcl::io::savePCDFileBinary(pcd_file_path, *globalSurfCloudDS); //  稠密地图
	if (ret == -1) {														 //失败
		TRACE_ERR_CLASS("save cloud-map failed");
		return false;
	} else if (ret == 0) { //成功
		TRACE_INFO_CLASS("Saving map to pcd files completed");
	}

	TRACE_INFO_CLASS("Saving loop data to :  %s", (saveMapDirectory + "/data").c_str());
	std::ofstream file(saveMapDirectory + "/data");
	std::ofstream file_pose(save_key_frame_cloud_dir + "/key_frame_pose.txt");
	if (!file.is_open()) {
		TRACE_ERR_CLASS("sc data file open failed");
		return false;
	}
	if (!file_pose.is_open()) {
		TRACE_ERR_CLASS("key_frame_pose file open failed");
		return false;
	}
	for (int i = start; i <= end; i++) {
		file << infos[i].id << ',';
		Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", ",", "", "", "", "");
		file << (infos[i].pose).matrix().format(fmt) << ',';
		file << infos[i].polarcontext.rows() << ',' << infos[i].polarcontext.cols() << ',';
		file << infos[i].polarcontext.format(fmt) << '\n';

		// key_frame Pose
		file_pose << infos[i].id << ',';
		file_pose << std::setprecision(15) << KeyPoses_[i].time << ',';
		file_pose << std::setprecision(6) << (infos[i].pose).matrix().format(fmt) << '\n';
	}
	file.close();
	file_pose.close();
	TRACE_INFO_CLASS("Saving loop data completed");

	return true;
}

} // namespace lidar_slam

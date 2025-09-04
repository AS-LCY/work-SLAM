#include "lidar_slam/global_localization.hpp"

namespace lidar_slam {

GlobalLocalization::GlobalLocalization() {
	loaded_sc_info_.clear();
	loaded_global_map_.reset(new PointCloudType());

	loaded_keyframe_poses_.clear();
	loaded_keyframe_clouds_.clear();
}

GlobalLocalization::~GlobalLocalization() {
	loaded_sc_info_.clear();
	loaded_global_map_.reset(new PointCloudType());

	loaded_keyframe_poses_.clear();
	loaded_keyframe_clouds_.clear();
}

bool GlobalLocalization::global_localize(PointCloudType::Ptr cloud_in, Eigen::Isometry3d pose, Matrix3d initial_rotate,
										 double score_thr) {
	/// check map data status
	/// make ScanContext using loaded_sc_info
	if (!global_map_ready_) {
		TRACE_ERR_CLASS("global map not ready!");
		return false;
	}
	if (!sc_manager_ready_) {
		TRACE_ERR_CLASS("sc manager not ready!");
		return false;
	}

	/// search best match of scancontex
	std::pair<int, float> best_match{ -1, 0.0 };
	std::pair<double, double> best_trans;
	if (!scancontex_search(cloud_in, initial_rotate, best_match, best_trans)) {
		TRACE_ERR_CLASS("scancontex search failed!");
		return false;
	}
	int best_match_idx = best_match.first;
	TRACE_INFO_CLASS("scancontext search success, use index %d", best_match_idx);

	Eigen::Matrix4d init_guess = cal_init_transform(initial_rotate, best_match, best_trans);
	Eigen::Isometry3d test_transform(init_guess);					   /// debug
	test_match_cloud_ = transformPointCloud(cloud_in, test_transform); /// debug

	// result of global localization: global_odom_to_map_
	if (!registration_icp(cloud_in, pose, init_guess, score_thr, global_odom_to_map_)) {
		return false;
	} else {
		return true;
	}
}

bool GlobalLocalization::scancontex_search(PointCloudType::Ptr cloud_in, Matrix3d initial_rotate,
										   std::pair<int, float>& best_match, std::pair<double, double>& best_trans) {
	// transform (gravity_align) curr cloud
	PointCloudType::Ptr gravity_aligned_cLoud(new PointCloudType());
	Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
	transform.matrix().block<3, 3>(0, 0) = initial_rotate;
	*gravity_aligned_cLoud = *transformPointCloud(cloud_in, transform);

	// set search_trans
	std::vector<std::pair<double, double>> search_trans = { { 0, 0 },	{ -4, 0 }, { 4, 0 },  { 0, -4 },  { 0, 4 },
															{ -4, -4 }, { -4, 4 }, { 4, -4 }, { 4, 4 },	  { -2, 0 },
															{ 2, 0 },	{ 0, -2 }, { 0, 2 },  { -2, -2 }, { -2, 2 },
															{ 2, -2 },	{ 2, 2 } };

	double min_dist = std::numeric_limits<double>::max();
	for (auto& t : search_trans) {
		Eigen::MatrixXd sc =
			sc_manager_->makeScancontext(*(gravity_aligned_cLoud), t.first, t.second); // get sc of curr cloud
		std::vector<float> ringkey = eig2stdvec(sc_manager_->makeRingkeyFromScancontext(sc));
		Eigen::MatrixXd sectorkey = sc_manager_->makeSectorkeyFromScancontext(sc);

		double sc_dist = 1.0; // 当前匹配的距离(这个仅仅是初始化)，不是阈值
		auto match = sc_manager_->detectClosestMatch(sc, ringkey, sectorkey, sc_dist);
		if (match.first != -1) {
			TRACE_INFO_CLASS("trans: %f, %f, score: %f", t.first, t.second, sc_dist);
		}
		if (sc_dist < min_dist) {
			min_dist = sc_dist;
			best_match = match;
			best_trans = t;
		}
	}

	// check scancontext search
	int match_idx = best_match.first;
	if (match_idx == -1) {
		TRACE_INFO_CLASS("scancontext search fail, score: %f", min_dist);
		return false;
	} else {
		return true;
	}
}

Eigen::Matrix4d GlobalLocalization::cal_init_transform(Matrix3d initial_rotate, std::pair<int, float> best_match,
													   std::pair<double, double> best_trans) {
	int match_idx = best_match.first;

	Eigen::Matrix4d init_guess = loaded_sc_info_[match_idx].pose.matrix(); // use loaded data
	Eigen::Vector3d euler = R2ypr(init_guess.block<3, 3>(0, 0));

	// 初始值: 确定 yaw 角, 用搜索到的 sc-info
	euler[0] += -best_match.second;
	TRACE_INFO_CLASS("rotate yaw:  %f degree", euler[0] * 180 / M_PI);

	// 初始值: 确定 pitch, roll, 用重力校正时的 initial_rotate,
	Eigen::Vector3d current_euler = R2ypr(initial_rotate);
	double current_pitch = current_euler[1];
	double current_roll = current_euler[2];
	Eigen::Matrix3d rotate = ypr2R(Eigen::Vector3d(euler[0], current_pitch, current_roll));
	init_guess.block<3, 3>(0, 0) = rotate;
	euler = R2ypr(init_guess.block<3, 3>(0, 0));

	Eigen::Vector2d offset_in_lidar{ -best_trans.first, -best_trans.second };
	Eigen::Rotation2D<double> rotation(euler[0]);
	Eigen::Vector2d offset_in_map = rotation * offset_in_lidar;
	init_guess.coeffRef(0, 3) = init_guess.coeffRef(0, 3) + offset_in_map[0];
	init_guess.coeffRef(1, 3) = init_guess.coeffRef(1, 3) + offset_in_map[1];
	// init_guess.coeffRef(2, 3) = 0;

	TRACE_INFO_CLASS("initial yaw: %f, pitch: %f, roll: %f", euler[0] * 180 / M_PI, euler[1] * 180 / M_PI,
					 euler[2] * 180 / M_PI);
	TRACE_INFO_CLASS("intial trans x: %f, y: %f, z: %f", init_guess.coeffRef(0, 3), init_guess.coeffRef(1, 3),
					 init_guess.coeffRef(2, 3));
	return init_guess;
}

bool GlobalLocalization::registration_icp(PointCloudType::Ptr cloud_in,
										  Eigen::Isometry3d pose, // T_odom_lidar
										  Eigen::Matrix4d init_guess, double score_thr,
										  Eigen::Isometry3d& res_global_odom_to_map) {
	pcl::IterativeClosestPoint<PointType, PointType> icp;
	icp.setMaxCorrespondenceDistance(100);
	icp.setMaximumIterations(100);
	icp.setTransformationEpsilon(1e-6);
	icp.setEuclideanFitnessEpsilon(1e-6);
	icp.setRANSACIterations(0);
	icp.setInputSource(cloud_in);
	icp.setInputTarget(loaded_global_map_);

	PointCloudType::Ptr unused_result(new PointCloudType());
	icp.align(*unused_result, init_guess.cast<float>());
	if (icp.hasConverged() == false || icp.getFitnessScore() > score_thr) { // TODO(jxl): 统计内点，还是全部点
		TRACE_ERR_CLASS("globalLocalization icp failed with score: %f, thres: %f", icp.getFitnessScore(), score_thr);
		return false;
	} else {
		TRACE_INFO_CLASS("globalLocalization success with score: %f", icp.getFitnessScore())
	}
	Eigen::Isometry3d first_lidar_in_map;
	first_lidar_in_map.matrix() = icp.getFinalTransformation().matrix().cast<double>();
	res_global_odom_to_map = first_lidar_in_map * pose.inverse();

	auto euler = first_lidar_in_map.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0);
	TRACE_INFO_CLASS("init lidar in map yaw: %f, pitch: %f, roll: %f", euler[0] * 180 / M_PI, euler[1] * 180 / M_PI,
					 euler[2] * 180 / M_PI);
	TRACE_INFO_CLASS("init lidar in map trans x: %f, y: %f, z: %f", first_lidar_in_map.translation().x(),
					 first_lidar_in_map.translation().y(), first_lidar_in_map.translation().z());

	return true;
}

bool GlobalLocalization::set_global_map(PointCloudType::Ptr input_global_map) {
	if (input_global_map->empty() || input_global_map->points.empty() || input_global_map->points.size() == 0) {
		TRACE_INFO_CLASS(" loaded global map empty!");
		return false;
	}

	*loaded_global_map_ = *input_global_map;
	global_map_ready_ = true;
	return true;
}

bool GlobalLocalization::fill_sc_manager(const std::vector<ScInfo, Eigen::aligned_allocator<ScInfo>>& input_sc_info) {
	sc_manager_.reset(new SCManager()); //// important
	if (input_sc_info.empty() || input_sc_info.size() == 0) {
		TRACE_INFO_CLASS(" loaded sc info empty!");
		return false;
	}

	loaded_sc_info_ = input_sc_info;
	TRACE_INFO_CLASS("global-localization: loaded_sc_info_ size = %d", loaded_sc_info_.size());

	KeyMat polarcontext_invkeys_mat;
	std::vector<Eigen::MatrixXd> polarcontexts;
	int i = 0;
	for (const auto& scinfo : input_sc_info) {
		Eigen::MatrixXd sc = scinfo.polarcontext;
		Eigen::MatrixXd ringkey = sc_manager_->makeRingkeyFromScancontext(sc);
		polarcontext_invkeys_mat.push_back(eig2stdvec(ringkey));
		polarcontexts.push_back(sc);
	}
	sc_manager_->buildRingKeyKDTree(polarcontext_invkeys_mat, polarcontexts); // save in sc_manager_
	sc_manager_ready_ = true;
	TRACE_INFO_CLASS("fill_sc_manager success, sc_manager_ready = true");
	return true;
}

} // namespace lidar_slam
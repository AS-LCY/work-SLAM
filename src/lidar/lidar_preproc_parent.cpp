
#include "lidar/lidar_preproc_parent.h"

#include <rclcpp/rclcpp.hpp>

namespace localization_module {

LidarPreprocParent::LidarPreprocParent(rclcpp::Node::SharedPtr node) {
	cloud_dense_.reset(new PointCloudType());
	set_common_params(node);
}

LidarPreprocParent::~LidarPreprocParent() {}

void LidarPreprocParent::sampling_cloud(PointCloudType::Ptr in_cloud_ptr, PointCloudType::Ptr out_cloud_ptr) {
	int in_size = in_cloud_ptr->points.size();
	int cal_ratio = std::round(in_size * 1.0 / cloud_size_to_keep_);
	int point_filter_ratio = cal_ratio > 1 ? cal_ratio : 1;
	int out_size = in_size / point_filter_ratio;
	out_cloud_ptr->points.resize(out_size);

#pragma omp parallel for num_threads(MP_PROC_NUM)
	for (int i = 0; i < out_size; ++i) {
		out_cloud_ptr->points[i] = in_cloud_ptr->points[i * point_filter_ratio];
		out_cloud_ptr->points[i].curvature =
			in_cloud_ptr->points[i * point_filter_ratio].curvature * 1000; //单位转成了ms
	}

	out_cloud_ptr->header = in_cloud_ptr->header;
	out_cloud_ptr->width = out_cloud_ptr->points.size();
	out_cloud_ptr->height = 1;
	out_cloud_ptr->is_dense = 1;
}

bool LidarPreprocParent::set_common_params(rclcpp::Node::SharedPtr node) {
	LocalizationModuleParamManager* param_manager = LocalizationModuleParamManager::Instance(node);
	const lidar_slam::LidarSlamParam& loaded_param = param_manager->get_loaded_param();

	blind_range_square_ = loaded_param.lidar_preproc.blind_distance * loaded_param.lidar_preproc.blind_distance;
	max_range_square_ = loaded_param.lidar_preproc.max_distance * loaded_param.lidar_preproc.max_distance;
	point_filter_num_ = loaded_param.lidar_preproc.point_filter_num;
	ring_filter_num_ = loaded_param.lidar_preproc.ring_filter_num;
	z_range_ = loaded_param.lidar_preproc.z_range;
	cloud_size_to_keep_ = loaded_param.lidar_preproc.cloud_size_to_keep;
	return true;
}

} // namespace localization_module

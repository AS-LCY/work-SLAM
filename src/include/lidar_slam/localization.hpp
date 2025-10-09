#ifndef LOCALIZATION_H
#define LOCALIZATION_H
#include <omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

#include <Eigen/Core>
#include <csignal>
#include <fstream>
#include <mutex>
#include <thread>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/common/common.h>
// #include <pcl/common/transforms.h>
// #include <pcl/registration/ndt.h>
// #include <pcl/registration/gicp.h>
// #include <pcl/filters/filter.h>
// #include <pcl/filters/crop_box.h>
// #include <pcl/search/impl/search.hpp>
// #include <pcl/range_image/range_image.h> // 深度图像相关（将从图像采集器到场景中各点的距离值作为像素值的图像）

#include <pcl/registration/ndt.h> //NDT(正态分布)配准类头文件

#include <fast_gicp/gicp/fast_gicp.hpp>

#include "lidar_slam/ikd_Tree.h"
#include "lidar_slam/scan_context/Scancontext.h"
#include "lidar_slam/slam_param_def.h"
#include "node/log_info_manager.hpp"

namespace lidar_slam {
class Localization {
	DECL_CLASSNAME(Localization)
   public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using LocalizationStatus = common_status::LocalizationStatus;

	Localization(LocalizationParam param);
	~Localization();
	bool loadMap(std::string path);

	void localize(pcl::PointCloud<pcl::PointXYZI>::Ptr odomCloud, LocalizeStatus& localize_status,
				  LocalizationStatus& localize_state_status);

	bool globalLocalization(PointCloudType::Ptr lidarCloud, Eigen::Isometry3d pose, Matrix3d initial_rotate,
							double score);

	Eigen::Isometry3d getOdomToMap() { return correctionOdomToMap_; }

	std::vector<ScInfo, Eigen::aligned_allocator<ScInfo>> getLoadKeyFrame() { return LoadData_; }
	pcl::PointCloud<pcl::PointXYZI>::Ptr getLoadMap() {
		if (!map_ready_) {
			return nullptr;
		}
		return CloudGlobalMapIn_;
	}

	PointCloudType::Ptr getTestCloud() { return testMatchcloud_; }

   private:
	pcl::NormalDistributionsTransform<PointType, PointType>::Ptr ndt_;
	pcl::IterativeClosestPoint<PointType, PointType>::Ptr icp_;
	fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp_;

	KeyMat polarcontext_invkeys_mat_;
	std::vector<Eigen::MatrixXd> polarcontexts_;

	std::vector<Eigen::Isometry3d> accumulateKeypose_;
	PointCloudType::Ptr testMatchcloud_;
	pcl::PointCloud<pcl::PointXYZI>::Ptr CloudGlobalMapIn_;
	PointCloudType::Ptr CloudGlobalMapIn_PointType_;
	std::vector<ScInfo, Eigen::aligned_allocator<ScInfo>> LoadData_;
	std::shared_ptr<SCManager> scManager_;

	pcl::PointCloud<pcl::PointXYZ>::Ptr KeyPoint_;
	bool map_ready_;
	bool filter_init_ = false;
	Eigen::Isometry3d correctionOdomToMap_ = Eigen::Isometry3d::Identity(); // T_map_odom

	inline static localization_module::LocalizationModuleLogInfoManager& log_info_manager_ =
		localization_module::LocalizationModuleLogInfoManager::getInstance();

	LocalizationParam param_;
	double max_correspondence_dist_square_;
};

} // namespace lidar_slam
#endif

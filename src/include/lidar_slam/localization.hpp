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

#include <pcl/filters/crop_box.h>
#include <pcl/registration/ndt.h> //NDT(正态分布)配准类头文件

#include <fast_gicp/gicp/fast_gicp.hpp>

#include "cpu_bbs3d/bbs3d.hpp"
#include "kiss_matcher/KISSMatcher.hpp"
#include "lidar_slam/ekf_smoother.h"
#include "lidar_slam/ikd_Tree.h"
#include "lidar_slam/scan_context/Scancontext.h"
#include "lidar_slam/slam_param_def.h"
#include "lidar_slam/thread_safe_voxelgrid.hpp"
#include "lidar_slam/tictoc.hpp"
#include "node/log_info_manager.hpp"
#include "node/module_param_def.h"
#include "pointcloud_iof/pcd_loader.hpp"
#include "pointcloud_iof/pcl_eigen_converter.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/pcl/pcl_point_traits.hpp"
#include "small_gicp/pcl/pcl_registration.hpp"

namespace lidar_slam {

class Localization {
	DECL_CLASSNAME(Localization)
   public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using LocalizationStatus = common_status::LocalizationStatus;

	Localization(CommonParam common_param, LocalizationParam param, const RelocalizationConfig& relocalize_params);
	~Localization();
	bool loadMap(std::string path);

	void localize(pcl::PointCloud<pcl::PointXYZI>::Ptr odomCloud, LocalizeResultStatus& localize_status,
				  LocalizationStatus& localize_state_status, const Sophus::SE3d& T_odom_lidar,
				  const Sophus::SE3d& T_lidar_delta, const Eigen::Matrix<double, 6, 6>& T_lidar_delta_cov_local);

	bool globalLocalization(PointCloudType::Ptr lidarCloud, Eigen::Isometry3d pose, Matrix3d initial_rotate,
							double score);
	bool globalLocalization(const std::string& global_reg_method, const pcl::PointCloud<pcl::PointXYZI>::Ptr odom_cloud,
							const Eigen::Isometry3d& T_odom_lidar_curr, const int try_num);

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
	void initGlobalLocalize();
	Eigen::Isometry3d smootherMatchResult(const Sophus::SE3d& T_map_odom, const Sophus::SE3d& T_odom_lidar,
										  const Sophus::SE3d& T_lidar_delta,
										  const Eigen::Matrix<double, 6, 6>& T_lidar_delta_cov_local,
										  const Eigen::Matrix<double, 6, 6>& meas_cov_global);
	void assignMapToOdom(double matching_error, const Sophus::SE3d& T_odom_lidar, const Sophus::SE3d& T_lidar_delta,
						 const Eigen::Matrix<double, 6, 6>& T_lidar_delta_cov_local);

	pcl::PointCloud<pcl::PointXYZI>::Ptr cropCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
												   const Eigen::Isometry3d& pose, const double& radius = 40.f);
	pcl::PointCloud<pcl::PointXYZI>::Ptr boxCropCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
													  const Eigen::Isometry3d& pose);
	RegOutput icpAlignment();
	void localRefine(const Eigen::Matrix4d& coarse_alignment, RegOutput& reg_output);
	RegOutput coarseToFineAlignment();

	bool bbsGlobaLocalize(const Eigen::Isometry3d& T_odom_lidar_curr, Eigen::Matrix4d& bbs_pose);
	RegOutput bbsCoarseToFineAlignment(const Eigen::Isometry3d& T_odom_lidar_curr);
	void processSourceAndTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud_processed,
								pcl::PointCloud<pcl::PointXYZI>::Ptr tgt_cloud_processed,
								const Eigen::Isometry3d& T_odom_lidar_curr);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorizePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_xyzI,
															  uint8_t r, uint8_t g, uint8_t b);
	void saveTwoCloudsToOnePCD(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud1,
							   const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud2, const std::string& filename);

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
	Eigen::Isometry3d correctionOdomToMap_ = Eigen::Isometry3d::Identity();		 // T_map_odom
	Eigen::Isometry3d correctionOdomToMap_last_ = Eigen::Isometry3d::Identity(); //上一次T_map_odom

	EkfSmoother ekf_smoother_;

	inline static localization_module::LocalizationModuleLogInfoManager& log_info_manager_ =
		localization_module::LocalizationModuleLogInfoManager::getInstance();

	LocalizationParam param_;
	double max_correspondence_dist_square_;

	// global localize
	bool debug_relocalize_ = false;
	RelocalizationConfig relocalize_config_;
	std::unique_ptr<kiss_matcher::KISSMatcher> global_reg_handler_ = nullptr;
	std::unique_ptr<small_gicp::RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI>> local_reg_handler_ = nullptr;
	pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud_ = nullptr;
	pcl::PointCloud<pcl::PointXYZI>::Ptr tgt_cloud_ = nullptr;
	pcl::PointCloud<pcl::PointXYZI>::Ptr coarse_aligned_ = nullptr;
	pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_ = nullptr;
	pcl::PointCloud<pcl::PointXYZI>::Ptr debug_cloud_ = nullptr;
	std::atomic_bool global_match_target_cloud_assigned_{ false };

	pcl::VoxelGrid<pcl::PointXYZI> ds_source_cloud_filter_;
	pcl::VoxelGrid<pcl::PointXYZI> ds_target_cloud_filter_;
	// lidar_slam::ThreadSafeVoxelGrid<pcl::PointXYZI> source_voxel_grid_filter_;
	// lidar_slam::ThreadSafeVoxelGrid<pcl::PointXYZI> target_voxel_grid_filter_;

	pcl::PointCloud<pcl::PointXYZI>::Ptr source_ds_ = nullptr;
	pcl::PointCloud<pcl::PointXYZI>::Ptr target_ds_ = nullptr;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_target_ = nullptr;
	// bool kiss_matcher_target_cloud_change_ = false; // false: 使用全局地图， true使用裁剪的全局地图

	std::unique_ptr<cpu_bbs3d::BBS3D> bbs3d_ptr = nullptr;
	std::vector<Eigen::Vector3d> tar_points;
	std::vector<Eigen::Vector3d> src_points;
	CommonParam common_param_;
};

} // namespace lidar_slam
#endif

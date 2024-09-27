#ifndef LOCALIZATION_H
#define LOCALIZATION_H
#include <omp.h>
#include <mutex>
// #include <math.h> // ikd_Tree.h 中已包含
#include <thread>
#include <fstream>
#include <csignal>
// #include <unistd.h> // ikd_Tree.h 中已包含
#include <Eigen/Core>

#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/common/common.h>
// #include <pcl/common/transforms.h>
// #include <pcl/registration/ndt.h>
// #include <pcl/registration/gicp.h>
// #include <pcl/filters/filter.h>
// #include <pcl/filters/crop_box.h>
// #include <pcl/search/impl/search.hpp>
// #include <pcl/range_image/range_image.h> // 深度图像相关（将从图像采集器到场景中各点的距离值作为像素值的图像）

#include <fast_gicp/gicp/fast_gicp.hpp>

#include "lidar_slam/ikd_Tree.h"
#include "lidar_slam/scan_context/Scancontext.h"

#include "node/log_info_manager.hpp"

namespace lidar_slam {
class Localization
{
public:
   Localization();
   ~Localization();
   bool loadMap(std::string path);
   bool localize(pcl::PointCloud<pcl::PointXYZI>::Ptr odomCloud, double score_thr, double odom2map_delta_thr, double odom2map_delta_set);
   bool globalLocalization(PointCloudXYZI::Ptr lidarCloud,Eigen::Isometry3d pose,Matrix3d initial_rotate, double score);
   Eigen::Isometry3d getOdomToMap(){
      //  Eigen::Isometry3d isometry3d; 
      //  isometry3d.matrix().block<3, 3>(0, 0) = correctionOdomToMap.matrix().block<3, 3>(0, 0).cast<double>();
       // isometry3d.matrix().block<3, 1>(0, 3) = correctionOdomToMap.matrix().block<3, 1>(0, 3).cast<double>();
        return correctionOdomToMap;
   }
   Eigen::Isometry3d getLastOdomToMap(){
        return lastCorrectionOdomToMap;
   }
   std::vector<ScInfo> getLoadKeyFrame(){
      return LoadData;
   }
   pcl::PointCloud<pcl::PointXYZI>::Ptr getLoadMap(){
      if (!map_ready_) return nullptr;
      return CloudGlobalMapIn;
   }
   PointCloudXYZI::Ptr getTestCloud(){
      return testMatchcloud;
   }
   std::vector<Eigen::Vector3f>& getLoadMapPoints(){
      //if (!map_ready_) return std::vector<Eigen::Vector3f>{};
      return show_map_points;
   }
private:
   KeyMat polarcontext_invkeys_mat_;
   std::vector<Eigen::MatrixXd> polarcontexts_;
   PointCloudXYZI::Ptr CloudGlobalMap;
   PointCloudXYZI::Ptr accumulateMap_;
   std::vector<Eigen::Isometry3d> accumulateKeypose_;
   PointCloudXYZI::Ptr testMatchcloud;
   pcl::PointCloud<pcl::PointXYZI>::Ptr CloudGlobalMapIn;
   std::vector<ScInfo> LoadData;
   std::shared_ptr<SCManager> scManager;
   std::vector<Eigen::Vector3f> show_map_points;
  // pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr fast_gicp;
   fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp; // TODO test gicp with normal
   pcl::PointCloud<pcl::PointXYZ>::Ptr KeyPoint_;
   bool map_ready_;
   Eigen::Isometry3d correctionOdomToMap = Eigen::Isometry3d::Identity();
   Eigen::Isometry3d lastCorrectionOdomToMap = Eigen::Isometry3d::Identity();
   
   double lastUpdateTime = 0.0f;
   double curr_time_ = 0.0f;
   localization_module::LocalizationModuleLogInfoManager * log_info_manager_;


};
}
#endif

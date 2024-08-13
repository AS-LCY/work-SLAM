#ifndef BACKEND_H
#define BACKEND_H
#pragma once
#include <omp.h>
#include <mutex>
#include <math.h>
#include <cmath>
#include <thread>
#include <fstream>
#include <filesystem>
#include <csignal>
#include <unistd.h>
#include <Eigen/Core>
// #include <opencv2/opencv.hpp>
// #include <opencv2/core.hpp>
// pcl
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
// gstam
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
// lidar_slam
#include "lidar_slam/ikd_Tree.h"
#include "lidar_slam/common_lib.h"
#include "lidar_slam/scan_context/Scancontext.h"
#include "lidar_slam/data_struct_define.h"
namespace lidar_slam {
// struct KeyPose
// {
//     Eigen::Isometry3d pose;
//     int  index;
//     double time;
//     double roll;
//     double pitch;
//     double yaw;
// }; // moved to data_struct_define.h

class BackEnd
{
public:
    BackEnd(float dist, float angle,float loop_dist);
    ~BackEnd();

    void saveCurrentCloud(PointCloudXYZI::Ptr points,Eigen::Isometry3d pose);
    PointCloudXYZI::Ptr getCurrentMap(Eigen::Isometry3d T_map_odom);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCurrentRGBMap();
    bool saveKeyFramesAndFactor(Eigen::Isometry3d transformTobeMapped,PointCloudXYZI::Ptr lidar_cloud,double time);
    void performLoopClosure(double time);
    // if start_index == end_index == 0; save all;
    bool saveMap(std::string saveMapDirectory,double resolution,Eigen::Isometry3d T_map_odom, int start_index, int end_index);
    bool correctPoses();
    void recontructIKdTree(KD_TREE<PointType> &ikdtree,double kdTreeReconstructRadius,float kdTreeReconstructKeyFrameLeafSize,double kdTreeReconstructPointLeafSize);
    PointCloudXYZI::Ptr getObstacleMap(Eigen::Isometry3d T_map_odom,double min_height,double max_height);/// 没用上
    KeyPose getCurrentPose()
    {
        return KeyPoses.back();
    }
    // void UpdateImage(const cv::Mat &image,Eigen::Isometry3d pose);
    std::vector<KeyPose> getKeyframePoses()
    {
        return KeyPoses;
    }
    int getCurrentPoseIndex(){
        int temp_index = int(KeyPoses.size())-1;
        int curr_index = temp_index < 0 ? 0 : temp_index;
        return curr_index;
    }
    std::map<int, int> getloopIndex()
    {
        return loopIndexContainer;
    }
    PointCloudXYZI::Ptr getTestCloud(){
      return gravityAlignedCLoud;
    }
    
    bool get_loaded_key_cloud_status(){
        return loaded_key_clouds_ready_;
    }

    bool set_loaded_key_clouds(std::vector<PointCloudXYZI::Ptr> input_vec_key_clouds, 
                                std::vector<KeyPose> input_vec_key_poses, Eigen::Isometry3d trans_map_odom);

private:
    bool loaded_key_clouds_ready_ = false;

    pcl::PointCloud<PointType>::Ptr KeyPoint;
    std::vector<KeyPose> KeyPoses;
    pcl::PointCloud<PointType>::Ptr CopyKeyPoint;
    std::vector<KeyPose> CopyKeyPoses;
    std::vector<PointCloudXYZI::Ptr> KeyFrameCloud;
    PointCloudXYZI::Ptr show_map;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_rgb_map;

    float keyframeDistThreshold;  //  判断是否为关键帧的距离阈值
    float keyframeAngleThreshold; //  判断是否为关键帧的角度阈值
    float loopKeyframeSearchRadius;
    map<int, int> loopIndexContainer;
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    gtsam::NonlinearFactorGraph gtSAMgraph;
    gtsam::ISAM2 *isam;
    gtsam::Values initialEstimate;
    gtsam::Values isamCurrentEstimate;
    gtsam::ISAM2Params parameters;
    bool aLoopIsClosed;
    int show_index = 0;
    SCManager scManager;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    PointCloudXYZI::Ptr gravityAlignedCLoud;
    // cv::Mat image;
   

    bool saveFrame(Eigen::Isometry3d transformTobeMapped);
    bool create_directory_if_not_exists(const std::string &directoryPath);

    void addOdomFactor(Eigen::Isometry3d transformTobeMapped);
    void addLoopFactor();
    
    
    bool detectLoopClosureDistance(int *latestID, int *closestID, double time);
    void loopFindNearKeyframes(PointCloudXYZI::Ptr &nearKeyframes, const int &key, const int &searchNum);
    void loopFindNearKeyframesWithRespectTo(PointCloudXYZI::Ptr& nearKeyframes, const int& key, const int& searchNum, const int _wrt_key);
    std::mutex mtxPose;
    std::mutex mtxCloud;
    std::mutex mtxLoopInfo;
    std::mutex mtxCurrentMap;
    std::mutex mtxCurrentRGBMap;
 

}; 
}


#endif


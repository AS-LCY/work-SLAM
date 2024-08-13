#include "lidar_slam/backend.hpp"
namespace lidar_slam {
BackEnd::BackEnd(float dist, float angle,float loop_dist){
   KeyPoint.reset(new pcl::PointCloud<PointType>());
   CopyKeyPoint.reset(new pcl::PointCloud<PointType>());
   show_map.reset(new pcl::PointCloud<PointType>());
   show_rgb_map.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
   loopIndexQueue.clear();
   loopPoseQueue.clear();
   loopNoiseQueue.clear();
   KeyPoses.clear();
   CopyKeyPoses.clear();
   KeyFrameCloud.clear();
   keyframeDistThreshold = dist;
   keyframeAngleThreshold = angle; 
   loopKeyframeSearchRadius = loop_dist;
   parameters.relinearizeThreshold = 0.01;
   parameters.relinearizeSkip = 1;
   isam = new gtsam::ISAM2(parameters);
   downSizeFilterICP.setLeafSize(0.4, 0.4, 0.4);//TODO param?
   aLoopIsClosed = false;

   gravityAlignedCLoud.reset(new PointCloudXYZI());

}

BackEnd::~BackEnd(){

}
bool BackEnd::saveFrame(Eigen::Isometry3d transformTobeMapped)
{
    if (KeyPoint->points.empty())
        return true;
    Eigen::Affine3f transBetween;
    Eigen::Isometry3d temp = KeyPoses.back().pose.inverse() * transformTobeMapped;
    transBetween = temp.cast<float>();
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);
    if (abs(roll) < keyframeAngleThreshold &&
        abs(pitch) < keyframeAngleThreshold &&
        abs(yaw) < keyframeAngleThreshold &&
        sqrt(x * x + y * y + z * z) < keyframeDistThreshold)
        return false;
    return true;
}


void BackEnd::addOdomFactor(Eigen::Isometry3d transformTobeMapped)
{
    if (KeyPoint->points.empty())
    {
        // 第一帧初始化先验因子
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) <<1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished()); // rad*rad, meter*meter   // indoor 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12    //  1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(transformTobeMapped.matrix()), priorNoise));
        // 变量节点设置初始值
        initialEstimate.insert(0, gtsam::Pose3(transformTobeMapped.matrix()));
    }
    else
    {
        // 添加激光里程计因子
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom(KeyPoses.back().pose.matrix()); /// pre
        gtsam::Pose3 poseTo(transformTobeMapped.matrix());                   // cur
        // 参数：前一帧id，当前帧id，前一帧与当前帧的位姿变换（作为观测值），噪声协方差
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(KeyPoint->size() - 1, KeyPoint->size(), poseFrom.between(poseTo), odometryNoise));
        // 变量节点设置初始值
        initialEstimate.insert(KeyPoint->size(), poseTo);
    }
}

void BackEnd::addLoopFactor()
{
    
    if (loopIndexQueue.empty())
        return;
    // 闭环队列
    for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
    {
        // 闭环边对应两帧的索引
        int indexFrom = loopIndexQueue[i].first; //   cur
        int indexTo = loopIndexQueue[i].second;  //    pre
        // 闭环边的位姿变换
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }
  //  mtxLoopInfo.lock(); // TODO this cause CPU high
    std::lock_guard<std::mutex> lk(mtxLoopInfo);
    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    aLoopIsClosed = true;
 //   mtxLoopInfo.unlock();
}

bool BackEnd::saveKeyFramesAndFactor(Eigen::Isometry3d transformTobeMapped ,PointCloudXYZI::Ptr lidar_cloud,double time)
{

    // 计算当前帧与前一帧位姿变换，如果变化太小，不设为关键帧，反之设为关键帧
    if (saveFrame(transformTobeMapped) == false)
        return false;
    // 激光里程计因子(from fast-lio),  输入的是frame_relative pose  帧间位姿(body 系下)
    addOdomFactor(transformTobeMapped);
    //// GPS因子 (UTM -> WGS84)
    // addGPSFactor();
    //// 闭环因子 (rs-loop-detect)  基于欧氏距离的检测
    addLoopFactor();
    // 执行优化
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    if (aLoopIsClosed) // 有回环因子，多update几次
    {
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
    }
    // update之后要清空一下保存的因子图，注：历史数据不会清掉，ISAM保存起来了
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    PointType thisPose3D;
    KeyPose thisPose6D;
    gtsam::Pose3 latestEstimate;
    // 优化结果
    isamCurrentEstimate = isam->calculateBestEstimate();// TODO 没有优化的话，取到的是什么值
    // 当前帧位姿结果
    latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);
    // cloudKeyPoses3D加入当前帧位置
    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    // 索引
    thisPose3D.intensity = KeyPoint->size(); //  使用intensity作为该帧点云的index
    mtxPose.lock();
    KeyPoint->push_back(thisPose3D);         //  新关键帧帧放入队列中
    // cloudKeyPoses6D加入当前帧位姿
    thisPose6D.pose = Eigen::Isometry3d(latestEstimate.matrix());
    thisPose6D.index = thisPose3D.intensity;
    thisPose6D.time = time;
    thisPose6D.roll = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw = latestEstimate.rotation().yaw();
    KeyPoses.push_back(thisPose6D);
    mtxPose.unlock();
    // saveCurrentCloud(lidar_cloud,thisPose6D.pose);
    return true;
    // ROS_INFO("key pose %d ",cloudKeyPoses6D->size());
    // 位姿协方差
    // poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);  // TODO gps used

    //// ESKF状态和方差  更新
    // state_ikfom state_updated = kf.get_x(); //  获取cur_pose (还没修正)  // TODO update state getCurrentPose
    // Eigen::Vector3d pos(latestEstimate.translation().x(), latestEstimate.translation().y(), latestEstimate.translation().z());
    // Eigen::Quaterniond q = EulerToQuat(latestEstimate.rotation().roll(), latestEstimate.rotation().pitch(), latestEstimate.rotation().yaw());

    // //  更新状态量
    // state_updated.pos = pos;
    // state_updated.rot =  Sophus::SO3d(q);
    // state_point = state_updated; // 对state_point进行更新，state_point可视化用到

    // kf.change_x(state_updated);  //  对cur_pose 进行isam2优化后的修正

    // TODO:  P的修正有待考察，按照yanliangwang的做法，修改了p，会跑飞
    // esekfom::esekf<state_ikfom, 12, input_ikfom>::cov P_updated = kf.get_P(); // 获取当前的状态估计的协方差矩阵
    // P_updated.setIdentity();
    // P_updated(6, 6) = P_updated(7, 7) = P_updated(8, 8) = 0.00001;
    // P_updated(9, 9) = P_updated(10, 10) = P_updated(11, 11) = 0.00001;
    // P_updated(15, 15) = P_updated(16, 16) = P_updated(17, 17) = 0.0001;
    // P_updated(18, 18) = P_updated(19, 19) = P_updated(20, 20) = 0.001;
    // P_updated(21, 21) = P_updated(22, 22) = 0.00001;
    // kf.change_P(P_updated);

    // 当前帧激光角点、平面点，降采样集合

    // PointCloudXYZI::Ptr thisSurfKeyFrame(new PointCloudXYZI()); // TODO see saveCurrentCloud

    // pcl::copyPointCloud(*feats_undistort, *thisSurfKeyFrame); // 存储关键帧,没有降采样的点云

    // 保存特征点降采样集合

    // surfCloudKeyFrames.push_back(thisSurfKeyFrame);

    // scManager.makeAndSaveScancontextAndKeys(*thisSurfKeyFrame);


    // updatePath(thisPose6D); //  可视化update后的path  // TODO update path
}



void BackEnd::saveCurrentCloud(PointCloudXYZI::Ptr points,Eigen::Isometry3d pose)
{
    PointCloudXYZI::Ptr currentCLoud(new PointCloudXYZI());
    pcl::copyPointCloud(*points, *currentCLoud);
    {
        std::lock_guard<std::mutex> lk(mtxCloud);
            KeyFrameCloud.emplace_back(currentCLoud);
    }
//   Eigen::Vector3d euler =  pose.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0);
    Eigen::Vector3d euler = R2ypr(pose.matrix().block<3, 3>(0, 0));
    euler[0] = 0;
    Eigen::Isometry3d Transform = Eigen::Isometry3d::Identity();
    Transform.matrix().block<3, 3>(0, 0) = ypr2R(euler); 
    // std::cout << "test martrix "<< R2ypr(Transform.matrix().block<3, 3>(0, 0)).transpose()<<std::endl;
    // Eigen::Isometry3d newTransform = Eigen::Isometry3d::Identity();
    /*  Eigen::Vector3d test = pose.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0);
    std::cout << "test "<<test.transpose()<<std::endl;
    Eigen::Isometry3d testTransform = Eigen::Isometry3d::Identity();
    testTransform.rotate(Eigen::AngleAxisd(test[1], Eigen::Vector3d::UnitY()));
    testTransform.rotate(Eigen::AngleAxisd(test[2], Eigen::Vector3d::UnitX()));
    std::cout << "test martrix "<< testTransform.matrix()<<std::endl;
    std::cout << "euler "<< euler.transpose()<<std::endl;*/
    /* newTransform.rotate(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
    newTransform.rotate(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));*/
    // newTransform.matrix().block<3, 3>(0, 0) = ypr2R(Eigen::Vector3d(0,pitch,roll)); 
    // std::cout << "euler martrix "<< newTransform.matrix()<<std::endl;
    gravityAlignedCLoud.reset(new PointCloudXYZI());
    *gravityAlignedCLoud = *transformPointCloud(currentCLoud, Transform);
    scManager.makeAndSaveScancontextAndKeys(*gravityAlignedCLoud);
}

bool BackEnd::correctPoses()
{
    if (KeyPoint->points.empty())
        return false;
    if (aLoopIsClosed)
    {
        // 清空里程计轨迹
        // globalPath.poses.clear();
        // 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿
        int numPoses = isamCurrentEstimate.size();
        mtxPose.lock();
        for (int i = 0; i < numPoses; ++i)
        {
            KeyPoint->points[i].x = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().x();
            KeyPoint->points[i].y = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().y();
            KeyPoint->points[i].z = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().z();

            KeyPoses[i].pose = Eigen::Isometry3d(isamCurrentEstimate.at<gtsam::Pose3>(i).matrix());
            KeyPoses[i].roll = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().roll();
            KeyPoses[i].pitch = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().pitch();
            KeyPoses[i].yaw = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().yaw();

            // 更新里程计轨迹
          //  updatePath(cloudKeyPoses6D->points[i]); // TODO check path
        }
        mtxPose.unlock();
        // 清空局部map， reconstruct  ikdtree submap
        // recontructIKdTree(ikdtree); 
        std::cout <<"ISMA2 Update"<< std::endl;
        aLoopIsClosed = false;
        show_index = 0;
        std::lock_guard<std::mutex> lk(mtxCurrentMap);
        show_map->clear();
        return true;
    }
    return false;

}
/*void BackEnd::updateMapRGB(PointCloudXYZI::Ptr last_cloud,Eigen::Isometry3d cupose)
{

}*/
void BackEnd::recontructIKdTree(KD_TREE<PointType> &ikdtree,double kdTreeReconstructRadius,float kdTreeReconstructKeyFrameLeafSize,double kdTreeReconstructPointLeafSize){


    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMapPoses(new pcl::KdTreeFLANN<PointType>());
    PointCloudXYZI::Ptr subMapKeyPoses(new PointCloudXYZI());
    PointCloudXYZI::Ptr subMapKeyPosesDS(new PointCloudXYZI());
    PointCloudXYZI::Ptr subMapKeyFrames(new PointCloudXYZI());
    PointCloudXYZI::Ptr subMapKeyFramesDS(new PointCloudXYZI());

    // kdtree查找最近一帧关键帧相邻的关键帧集合
    std::vector<int> pointSearchIndGlobalMap;
    std::vector<float> pointSearchSqDisGlobalMap;
    mtxPose.lock();
    kdtreeGlobalMapPoses->setInputCloud(KeyPoint);
    kdtreeGlobalMapPoses->radiusSearch(KeyPoint->back(), kdTreeReconstructRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);// TODO check time
    
    for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
        subMapKeyPoses->push_back(KeyPoint->points[pointSearchIndGlobalMap[i]]);     //  subMap的pose集合
    mtxPose.unlock();    
    // 降采样
    pcl::VoxelGrid<PointType> downSizeFilterSubMapKeyPoses;
    downSizeFilterSubMapKeyPoses.setLeafSize(kdTreeReconstructKeyFrameLeafSize, kdTreeReconstructKeyFrameLeafSize, kdTreeReconstructKeyFrameLeafSize); // for global map visualization
    downSizeFilterSubMapKeyPoses.setInputCloud(subMapKeyPoses);
    downSizeFilterSubMapKeyPoses.filter(*subMapKeyPosesDS);         //  subMap poses  downsample
    // 提取局部相邻关键帧对应的特征点云
    for (int i = 0; i < (int)subMapKeyPosesDS->size(); ++i)
    {
        // 距离过大
        // if (pointDistance(subMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > 1000.0) // TODO 
            //    continue;
        int thisKeyInd = (int)subMapKeyPosesDS->points[i].intensity;
        // *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
        *subMapKeyFrames += *transformPointCloud(KeyFrameCloud[thisKeyInd], KeyPoses[thisKeyInd].pose); //  fast_lio only use  surfCloud
    }
    // 降采样，发布
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;                                                                                   // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setLeafSize(kdTreeReconstructPointLeafSize, kdTreeReconstructPointLeafSize, kdTreeReconstructPointLeafSize); // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setInputCloud(subMapKeyFrames);
    downSizeFilterGlobalMapKeyFrames.filter(*subMapKeyFramesDS);

    std::cout << "subMapKeyFramesDS sizes  =  "   << subMapKeyFramesDS->points.size()  << std::endl;
    
    ikdtree.reconstruct(subMapKeyFramesDS->points);
    std::cout << "Reconstructed  ikdtree " << std::endl;
    int featsFromMapNum = ikdtree.validnum();
    int kdtree_size_st = ikdtree.size();
    std::cout << "featsFromMapNum  =  "   << featsFromMapNum   <<  "\t" << " kdtree_size_st   =  "  <<  kdtree_size_st  << std::endl;

}

bool BackEnd::detectLoopClosureDistance(int *latestID, int *closestID, double time)
{
    // 当前关键帧帧
    int loopKeyCur = CopyKeyPoint->size() - 1; //  当前关键帧索引
    int loopKeyPre = -1;

    // 当前帧已经添加过闭环对应关系，不再继续添加
    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;
    // 在历史关键帧中查找与当前关键帧距离最近的关键帧集合
    std::vector<int> pointSearchIndLoop;                        //  候选关键帧索引
    std::vector<float> pointSearchSqDisLoop;                    //  候选关键帧距离
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses(new pcl::KdTreeFLANN<PointType>());
    kdtreeHistoryKeyPoses->setInputCloud(CopyKeyPoint); //  历史帧构建kdtree
    kdtreeHistoryKeyPoses->radiusSearch(CopyKeyPoint->back(), loopKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
    // 在候选关键帧集合中，找到与当前帧时间相隔较远的帧，设为候选匹配帧
    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
    {
        int id = pointSearchIndLoop[i];
        if (abs(KeyPoses[id].time - time) > 30.0)
        {
            loopKeyPre = id;
            break;
        }
    }
    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
        return false;
    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    std::cout <<"Find loop clousre frame " << std::endl;
    return true;
}
/**
 * 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合，降采样
 */
void BackEnd::loopFindNearKeyframes(PointCloudXYZI::Ptr &nearKeyframes, const int &key, const int &searchNum)
{
    // 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合
    nearKeyframes->clear();
    int cloudSize = CopyKeyPoses.size();
    auto keyframes_size = KeyFrameCloud.size() ;
    
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize)
                continue;

            if (keyNear < 0 || keyNear >= keyframes_size)
                continue;

            // *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
            // 注意：cloudKeyPoses6D 存储的是 T_w_b , 而点云是lidar系下的，构建icp的submap时，需要通过外参数T_b_lidar 转换 , 参考pointBodyToWorld 的转换
            *nearKeyframes += *transformPointCloud(KeyFrameCloud[keyNear], CopyKeyPoses[keyNear].pose); 
        }
    


    if (nearKeyframes->empty())
        return;

    // 降采样
    PointCloudXYZI::Ptr cloud_temp(new PointCloudXYZI());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}
void BackEnd::loopFindNearKeyframesWithRespectTo(PointCloudXYZI::Ptr& nearKeyframes, const int& key, const int& searchNum, const int _wrt_key)
{
    // extract near keyframes
    nearKeyframes->clear();
    int cloudSize = KeyPoses.size();
    for (int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize )
            continue;
        *nearKeyframes += *transformPointCloud(KeyFrameCloud[keyNear], KeyPoses[_wrt_key].pose);
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    PointCloudXYZI::Ptr cloud_temp(new PointCloudXYZI());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

void BackEnd::performLoopClosure(double time)
{
    if (KeyPoint->points.empty() == true)
    {
        return;
    }

    mtxPose.lock();
    CopyKeyPoint->clear();
    *CopyKeyPoint = *KeyPoint;
    CopyKeyPoses.clear();
    CopyKeyPoses = KeyPoses;
    mtxPose.unlock();

    // 当前关键帧索引，候选闭环匹配帧索引
    int loopKeyCur;
    int loopKeyPre;
    // 在历史关键帧中查找与当前关键帧距离最近的关键帧集合，选择时间相隔较远的一帧作为候选闭环帧
    if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre,time) == false)
    {
        return;
    }

    // 提取
    PointCloudXYZI::Ptr cureKeyframeCloud(new PointCloudXYZI()); //  cue keyframe
    PointCloudXYZI::Ptr prevKeyframeCloud(new PointCloudXYZI()); //   history keyframe submap
    {
        // 提取当前关键帧特征点集合，降采样
        loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0); //  将cur keyframe 转换到world系下
        // 提取闭环匹配关键帧前后相邻若干帧的关键帧特征点集合，降采样
        loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, 20); //  选取historyKeyframeSearchNum个keyframe拼成submap
    }

    // ICP Settings zx gicp ?
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // scan-to-map，调用icp匹配
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(prevKeyframeCloud);
    PointCloudXYZI::Ptr unused_result(new PointCloudXYZI());
    icp.align(*unused_result);

    // 未收敛，或者匹配不够好
    if (icp.hasConverged() == false || icp.getFitnessScore() > 0.3)
        return;

    std::cout << "RS loop found! between " << loopKeyCur << " and " << loopKeyPre << "." << std::endl; // giseop
   // std::cout << "icp  success  " << std::endl;



    // 闭环优化得到的当前关键帧与闭环关键帧之间的位姿变换
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();

    // 闭环优化前当前帧位姿
    Eigen::Affine3f tWrong = CopyKeyPoses[loopKeyCur].pose.cast<float>();
    // 闭环优化后当前帧位姿
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
    gtsam::Pose3 poseFrom = gtsam::Pose3(tCorrect.matrix().cast<double>());
    // 闭环匹配帧的位姿
    gtsam::Pose3 poseTo = gtsam::Pose3(CopyKeyPoses[loopKeyPre].pose.matrix().cast<double>());
    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore() ; //  loop_clousre  noise from icp
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
    // std::cout << "loopNoiseQueue   =   " << noiseScore << std::endl;

    // 添加闭环因子需要的数据
    // mtxLoopInfo.lock(); // TODO 
    std::lock_guard<std::mutex> lk(mtxLoopInfo);
    loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));
    loopNoiseQueue.push_back(constraintNoise);
    loopIndexContainer[loopKeyCur] = loopKeyPre; //   使用hash map 存储回环对
    // mtxLoopInfo.unlock();

    
}

// void BackEnd::UpdateImage(const cv::Mat &image,Eigen::Isometry3d lidar_pose)
// {
//     // std::cout << "111111111"<<std::endl;
//     PointCloudXYZI lidar_cloud_in_map;
//     if (KeyPoses.size() == 0)
//        return;
//     {
//         std::lock_guard<std::mutex> lk(mtxCloud);
//         std::lock_guard<std::mutex> lk2(mtxPose);
//         int size = min((int)KeyPoses.size(),(int)KeyFrameCloud.size());
//         lidar_cloud_in_map = *transformPointCloud(KeyFrameCloud[size-1],KeyPoses[size-1].pose);

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
//         Eigen::Vector3d point_in_camera = map_to_camera.matrix().block<3, 3>(0, 0) * point_in_map + map_to_camera.matrix().block<3, 1>(3, 0);
       
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
//         std::lock_guard<std::mutex> lk(mtxCurrentRGBMap);
//         *show_rgb_map   += *color_cloud;
//         double resolution = 0.1;
//         pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilter;
//         downSizeFilter.setInputCloud(show_rgb_map);
//         downSizeFilter.setLeafSize(resolution, resolution, resolution);
//         downSizeFilter.filter(*show_rgb_map);
//         // std::cout << "22222222222"<<std::endl;
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
        auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff 
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;;
        int loopKeyPre = detectResult.first;
        float yawDiffRad = detectResult.second; // not use for v1 (because pcl icp withi initial somthing wrong...)
        if( loopKeyPre == -1 )
            return;

     //   std::cout << "SC loop found! between " << loopKeyCur << " and " << loopKeyPre << "." << std::endl; // giseop

        // extract cloud
        PointCloudXYZI::Ptr cureKeyframeCloud(new PointCloudXYZI());
        PointCloudXYZI::Ptr prevKeyframeCloud(new PointCloudXYZI());
        {
            // loopFindNearKeyframesWithRespectTo(cureKeyframeCloud, loopKeyCur, 0, loopKeyPre); // giseop 
            // loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);

            int base_key = 0;
           // loopFindNearKeyframesWithRespectTo(cureKeyframeCloud, loopKeyCur, 0, base_key); // giseop 
           // loopFindNearKeyframesWithRespectTo(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, base_key); // giseop 
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
        PointCloudXYZI::Ptr unused_result(new PointCloudXYZI());
        icp.align(*unused_result);
        // giseop 
        // TODO icp align with initial 

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore) {
            std::cout << "ICP fitness test failed (" << icp.getFitnessScore() << " > " << historyKeyframeFitnessScore << "). Reject this SC loop." << std::endl;
            return;
        } else {
            std::cout << "ICP fitness test passed (" << icp.getFitnessScore() << " < " << historyKeyframeFitnessScore << "). Add this SC loop." << std::endl;
        }
        std::cout << "SC loop found! between " << loopKeyCur << " and " << loopKeyPre << "." << std::endl; // giseop
        // publish corrected cloud
        if (pubIcpKeyFrames.getNumSubscribers() != 0)
        {
            PointCloudXYZI::Ptr closed_cloud(new PointCloudXYZI());
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
         Eigen::Isometry3d tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
         pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
         gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
         gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);

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
        //robustNoiseVector6 << robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore;
       // gtsam::noiseModel::Diagonal::shared_ptr robustConstraintNoise = gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6);
      //  gtsam::noiseModel::Base::shared_ptr robustConstraintNoise; 
       // robustConstraintNoise = gtsam::noiseModel::Robust::Create(
         //   gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure, but with a good front-end loop detector, Cauchy is empirically enough.
          //  gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6)
      //  ); // - checked it works. but with robust kernel, map modification may be delayed (i.e,. requires more true-positive loop factors)

        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(constraintNoise);
        mtx.unlock();

        // add loop constriant
      //   loopIndexContainer[loopKeyCur] = loopKeyPre;
        loopIndexContainer.insert(std::pair<int, int>(loopKeyCur, loopKeyPre)); // giseop for multimap
    } */// performSCLoopClosure
pcl::PointCloud<pcl::PointXYZRGB>::Ptr BackEnd::getCurrentRGBMap()
{
   std::lock_guard<std::mutex> lk(mtxCurrentRGBMap);
   return show_rgb_map;
}
PointCloudXYZI::Ptr BackEnd::getCurrentMap(Eigen::Isometry3d T_map_odom)
{
    std::lock_guard<std::mutex> lk(mtxCurrentMap);
   // PointCloudXYZI::Ptr globalSurfCloudDS(new PointCloudXYZI());
    if (KeyPoses.size() == 0)
       return show_map;
    {
        std::lock_guard<std::mutex> lk(mtxCloud);
        std::lock_guard<std::mutex> lk2(mtxPose);
        int size = min((int)KeyPoses.size(),(int)KeyFrameCloud.size());
        for (int i = show_index; i < size; i++) {
            *show_map   += *transformPointCloud(KeyFrameCloud[i],T_map_odom *KeyPoses[i].pose);
        }
    }
    show_index = (int)KeyPoses.size() - 1;
    double resolution = 0.1;
    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setInputCloud(show_map);
    downSizeFilter.setLeafSize(resolution, resolution, resolution);
    downSizeFilter.filter(*show_map);
    return show_map;
}

/// 没用上
PointCloudXYZI::Ptr BackEnd::getObstacleMap(Eigen::Isometry3d T_map_odom,double min_height,double max_height)
{
    std::lock_guard<std::mutex> lk(mtxCurrentMap);
   // PointCloudXYZI::Ptr globalSurfCloudDS(new PointCloudXYZI());
    if (KeyPoses.size() == 0)
       return show_map;
    pcl::PassThrough<PointType> pass;
	//pass.setInputCloud(cloud);              //设置输入点云
	pass.setFilterFieldName("z");           //设置过滤时所需要点云类型的Z字段
	pass.setFilterLimits(min_height, max_height);         //设置在过滤字段的范围
	pass.setFilterLimitsNegative(false);    //设置保留(false)范围内还是过滤掉(true)范围内（对范围取反）
	//pass.filter(*cloud_filtered);           //执行滤波，保存过滤结果在cloud_filtered

    {
        std::lock_guard<std::mutex> lk(mtxCloud);
        std::lock_guard<std::mutex> lk2(mtxPose);
        int size = min((int)KeyPoses.size(),(int)KeyFrameCloud.size());
        
        for (int i = show_index; i < size; i++) {
            PointCloudXYZI::Ptr cloud_filtered;
            pass.setInputCloud(KeyFrameCloud[i]); 
            pass.filter(*cloud_filtered);
            *show_map   += *transformPointCloud(cloud_filtered,T_map_odom *KeyPoses[i].pose);
        }
    }
    show_index = (int)KeyPoses.size() - 1;
    double resolution = 0.1;
    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setInputCloud(show_map);
    downSizeFilter.setLeafSize(resolution, resolution, resolution);
    downSizeFilter.filter(*show_map);
    return show_map;
}

bool BackEnd::saveMap(string saveMapDirectory,double resolution,Eigen::Isometry3d T_map_odom, int start_index, int end_index)
{
    cout << "****************************************************" << endl;
    // 检查并创建 yaml 中的地图路径
    if (create_directory_if_not_exists(saveMapDirectory)) {
        std::cout << "Directory created or already exists: " << saveMapDirectory << std::endl;
    } else {
        std::cerr << "Failed to create directory: " << saveMapDirectory << std::endl;
    }
    // 创建关键帧点云保存路径
    std::string save_key_frame_cloud_dir = saveMapDirectory + "/key_frame_cloud/";
    if (create_directory_if_not_exists(save_key_frame_cloud_dir)) {
        std::cout << "Directory created or already exists: " << save_key_frame_cloud_dir << std::endl;
    } else {
        std::cerr << "Failed to create directory: " << save_key_frame_cloud_dir << std::endl;
    }

    std::string pcd_file_path = "";

    PointCloudXYZI::Ptr globalMapCloud(new PointCloudXYZI());
    PointCloudXYZI::Ptr globalSurfCloudDS(new PointCloudXYZI());
    ScInfo infos[(int)KeyPoses.size()];
    // 注意：拼接地图时，keyframe是lidar系，而fastlio更新后的存到的cloudKeyPoses6D 关键帧位姿是body系下的，需要把
    //cloudKeyPoses6D  转换为T_world_lidar 。 T_world_lidar = T_world_body * T_body_lidar , T_body_lidar 是外参
    int start = 0, end=0;
    int KeyPosesSize = (int)KeyPoses.size();
    pcd_file_path = saveMapDirectory + "/cloud_map.pcd";
    if (start_index == 0 && end_index == 0){
        start = 0;
        end = KeyPosesSize -1;
        // pcd_file_path = saveMapDirectory + "/GlobalMap.pcd";
    }else if(start_index == -1 || end_index == -1){
        cout << "start-point or end-point not set, save all to cloud_map.pcd "<<endl;
        start = 0;
        end = KeyPosesSize -1;
    }else{
        start = start_index;
        end = end_index;
        if (end > KeyPosesSize-1){
            end = KeyPosesSize -1;
        }
    }

    std::string key_frame_cloud_path = "";
    //   for (int i = 0; i < (int)KeyPoses.size(); i++) {
    for (int i = start; i <= end; i++) {
        // 生成地图
        *globalMapCloud   += *transformPointCloud(KeyFrameCloud[i],T_map_odom * KeyPoses[i].pose);
        // ScanContex 信息组合获取
        ScInfo info;
        info.id = i;
        info.pose = T_map_odom * KeyPoses[i].pose;
        info.polarcontext = scManager.getSc(i);
        infos[i] = info;
        // 保存关键帧点云
        key_frame_cloud_path = save_key_frame_cloud_dir  + std::to_string(i) + ".pcd";
        int success = pcl::io::savePCDFileBinary(key_frame_cloud_path, *KeyFrameCloud[i]);
    }
    cout << "\n\nSave resolution: " << resolution << endl;
    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setInputCloud(globalMapCloud);
    downSizeFilter.setLeafSize(resolution, resolution, resolution);
    downSizeFilter.filter(*globalSurfCloudDS);

    cout << "Saving map to pcd file: "<<pcd_file_path << endl;
    int ret = pcl::io::savePCDFileBinary(pcd_file_path, *globalSurfCloudDS);       //  稠密地图  
    cout << "Saving map to pcd files completed" << endl;    
    cout << "Saving loop data" << endl; 
    std::ofstream file(saveMapDirectory + "/data");
    std::ofstream file_pose(save_key_frame_cloud_dir + "/key_frame_pose.txt");
    if (file.is_open()){

    }
    if(file_pose.is_open()){

    }
    // for (int i = 0; i < (int)KeyPoses.size(); i++) {
    for (int i = start; i <= end; i++) {
        file << infos[i].id << ',';
        Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", ",", "", "", "", "");
        file << (infos[i].pose).matrix().format(fmt) << ',';
        file << infos[i].polarcontext.rows() << ',' << infos[i].polarcontext.cols() << ',';
        file << infos[i].polarcontext.format(fmt) << '\n';
        // key_frame Pose
        file_pose << infos[i].id << ',';
        file_pose << KeyPoses[i].time << ',';
        file_pose << (infos[i].pose).matrix().format(fmt) << '\n';
    }
    file.close();
    file_pose.close();
    cout << "Saving loop data completed" << endl;
    cout << "****************************************************" << endl;
    return ret;
}

bool BackEnd::create_directory_if_not_exists(const std::string& directoryPath){
    std::filesystem::path path(directoryPath);

    if (!std::filesystem::exists(path)){
        try {
            std::filesystem::create_directories(path);
            return true; // 创建目录成功
        }catch (const std::filesystem::filesystem_error& ex){
            std::cerr << "Error creating directory: " << ex.what() << std::endl;
            return false; // 创建目录失败
        }
    } else {
        return true; // 目录已存在
    }
}


}

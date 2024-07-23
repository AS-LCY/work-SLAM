
#include "localization.hpp"
#include <filesystem>
namespace lidar_slam {
Localization::Localization(){
    
    gicp.reset(new fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setNumThreads(1);
    gicp->setTransformationEpsilon(0.01);
    gicp->setMaximumIterations(64);
    gicp->setMaxCorrespondenceDistance(2.0);
    gicp->setCorrespondenceRandomness(20);
    KeyPoint_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    CloudGlobalMap.reset(new PointCloudXYZI());
    accumulateMap_.reset(new PointCloudXYZI());
    testMatchcloud.reset(new PointCloudXYZI());
    CloudGlobalMapIn.reset(new pcl::PointCloud<pcl::PointXYZI>());
    map_ready_ = false;

}
Localization::~Localization(){

}
bool Localization::loadMap(std::string path){
    map_ready_ = false;
    CloudGlobalMap.reset(new PointCloudXYZI());
    CloudGlobalMapIn.reset(new pcl::PointCloud<pcl::PointXYZI>());
    show_map_points.clear();
    PointCloudXYZI::Ptr TempMap(new PointCloudXYZI());

    std::string cloud_map_file_path = path+std::string("cloud_map.pcd");
    if (std::filesystem::exists(cloud_map_file_path)){
        pcl::io::loadPCDFile(cloud_map_file_path, *TempMap); 
        *CloudGlobalMap = *TempMap;
        std::cout << "load map from : " << cloud_map_file_path<<"--- point size: "<<TempMap->points.size() << std::endl;
    }
    // // no ComplementMap.pcd
    std::string ComplementMap_file_path = path+std::string("ComplementMap.pcd");
    if (std::filesystem::exists(ComplementMap_file_path)){
        TempMap->points.clear();
        pcl::io::loadPCDFile(ComplementMap_file_path, *TempMap); 
        *CloudGlobalMap += *TempMap;
        std::cout << "load map from : " << ComplementMap_file_path<<"size "<<TempMap->points.size() << std::endl;
    }  
    pcl::copyPointCloud(*(CloudGlobalMap), *CloudGlobalMapIn);
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    pcl::PointCloud<pcl::PointXYZI>::Ptr GlobalMapShow(new pcl::PointCloud<pcl::PointXYZI>());
    double min_voxel_size = 0.1;
    if (CloudGlobalMap->points.size() < 100000.0)
        downSizeFilter.setLeafSize(0.5, 0.5, 0.5); // for global map visualization
    else{
            min_voxel_size = min(0.3 * CloudGlobalMap->points.size()/100000.0,2.0);
            downSizeFilter.setLeafSize(min_voxel_size,min_voxel_size,min_voxel_size); // for global map visualization
    }
    downSizeFilter.setInputCloud(CloudGlobalMapIn);
    downSizeFilter.filter(*GlobalMapShow); 
    // std::cout << "load map from : " << path+std::string("=GlobalMap.pcd")<<"size "<<CloudGlobalMap->points.size() << std::endl;
    // std::cout << "show map points: " << GlobalMapShow->points.size() << std::endl;
    for(int i = 0;i<GlobalMapShow->points.size();i++){
        Eigen::Vector3f point;
        point.x() =  GlobalMapShow->points[i].x;
        point.y() =  GlobalMapShow->points[i].y;
        point.z() =  GlobalMapShow->points[i].z;

        show_map_points.push_back(point);
    }
    if (CloudGlobalMap->points.size() == 0){
        std::cerr << "Failed to load map." << std::endl;
        return false;            
    }
    std::vector<std::string> files;
    files.emplace_back(path+std::string("data"));
    files.emplace_back(path+std::string("Complementdata"));
    // std::ifstream file(path+std::string("/data"));
    // if (!file) {
    //     std::cerr << "Failed to open file for reading." << std::endl;
    //     return false;
    // }
    std::string line;
    LoadData.clear();
    polarcontext_invkeys_mat_.clear();
    polarcontexts_.clear();
    KeyPoint_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    accumulateMap_->points.clear();
    accumulateKeypose_.clear();
    scManager.reset(new SCManager());/////////////// TODO，是否每次加载地图都需要 重置ScanContex，即重定位
    for (auto filename:files){
        std::ifstream file(filename);
        if (!file) {
            std::cerr << "Failed to open file "<< filename << std::endl;
            continue;
        }
        else{
            std::cout << "load file "<< filename << std::endl;
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
            if (values.size() - index != (maxrow*maxcol)){
                std::cout << " error :"<<values.size()<<" "<<index<<" "<<maxrow*maxcol<< std::endl;
                return false;
            }
            readData.polarcontext.resize(maxrow,maxcol);
            for (int row = 0; row < maxrow; ++row) {
                for (int col = 0; col < maxcol; ++col) {
                    readData.polarcontext(row, col) = values[index++];
                }
            } 
            // loadScManager.loadScancontextAndKeys(readData.polarcontext); 
            Eigen::MatrixXd sc = readData.polarcontext; // v1
            Eigen::MatrixXd ringkey = scManager->makeRingkeyFromScancontext( sc );
            // std::vector<float> polarcontext_invkey_vec = eig2stdvec( ringkey );   
            polarcontext_invkeys_mat_.push_back(eig2stdvec(ringkey));  
            polarcontexts_.push_back(sc); 
            //   if (i == 17)
            //    std::cout << " \n matrix:"  << std::endl;
            //    std::cout << value <<",";
            LoadData.push_back(readData);
        }
        file.close();
    }
    if (LoadData.size() == 0)
        return false;
    scManager->buildRingKeyKDTree(polarcontext_invkeys_mat_, polarcontexts_);
    std::cout << "get_load_data : " << LoadData.size() << std::endl;
    map_ready_ = true;
    return true;
}



void Localization::localize(pcl::PointCloud<pcl::PointXYZI>::Ptr odomCloud)
{
  //  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZI>());
  //  pcl::copyPointCloud(*(odomCloud), *cloudIn);

    if (!map_ready_) return;
    gicp->setInputSource(odomCloud);
    gicp->setInputTarget(CloudGlobalMapIn);
    pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>());
    gicp->align(*unused_result, correctionOdomToMap.matrix().cast<float>());                    
    if (gicp->hasConverged() == false || gicp->getFitnessScore() > 0.1){// TODO check param
        std::cout << "gicp fail "<<std::endl;
    }
    else{
        std::cout << "gicp success with score "<< gicp->getFitnessScore() << std::endl;       
        correctionOdomToMap.matrix() = gicp->getFinalTransformation().matrix().cast<double>();
    //    float x, y, z, roll, pitch, yaw;
     //   pcl::getTranslationAndEulerAngles(correctionOdomToMap, x, y, z, roll, pitch, yaw); //  获取上一帧 相对 当前帧的 位姿
      //  std::cout << "gicp results"<<" "<< x <<" "<< y <<" "<< z <<" "<< yaw <<" "<< pitch <<" "<< roll<<std::endl;
    }
}

bool Localization::globalLocalization(PointCloudXYZI::Ptr cloudIn,Eigen::Isometry3d pose,Matrix3d initial_rotate, double score)
{ 
    if (!map_ready_) {
        cout<<"map not ready"<<endl;
        return false;
    }
    // std::cout << "-------------------------------------------"<<std::endl;
    Eigen::Vector3d current_euler = R2ypr(initial_rotate);//R2ypr(pose.matrix().block<3, 3>(0, 0));
    // Eigen::Vector3d current_euler = pose.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0);
    // double current_yaw = current_euler[0];
    double current_pitch = current_euler[1];
    double current_roll = current_euler[2];
    // std::cout << "current yaw"<<current_euler[0]*180/M_PI<<" pitch "<<current_euler[1]*180/M_PI<< " roll "<<current_euler[2]*180/M_PI<<std::endl;
    // Eigen::Isometry3d newTransform = Eigen::Isometry3d::Identity();
    // newTransform.rotate(Eigen::AngleAxisd(current_pitch, Eigen::Vector3d::UnitY()));
    // newTransform.rotate(Eigen::AngleAxisd(current_roll, Eigen::Vector3d::UnitX()));
    // std::cout <<newTransform.matrix()<<std::endl;
    // newTransform.matrix().block<3, 3>(0, 0) = ypr2R(Eigen::Vector3d(0,current_pitch,current_roll)); 
    PointCloudXYZI::Ptr gravityAlignedCLoud(new PointCloudXYZI());
    Eigen::Isometry3d Transform = Eigen::Isometry3d::Identity();
    Transform.matrix().block<3, 3>(0, 0) = initial_rotate;
    *gravityAlignedCLoud = *transformPointCloud(cloudIn, Transform);

    // ICP param-set
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);  
    std::vector<std::pair<double, double>> search_trans = {
        {0, 0}, {-4, 0}, {4, 0}, {0, -4}, {0, 4}, {-4, -4}, {-4, 4}, {4, -4}, {4, 4},
                {-2, 0}, {2, 0}, {0, -2}, {0, 2}, {-2, -2}, {-2, 2}, {2, -2}, {2, 2}
    };
    double min_dist = std::numeric_limits<double>::max();
    std::pair<int, float> best_match{-1, 0.0};
    std::pair<double, double> best_trans;
    for (auto &t : search_trans) {
        Eigen::MatrixXd sc = scManager->makeScancontext(*(gravityAlignedCLoud), t.first, t.second);
        std::vector<float> ringkey = eig2stdvec(scManager->makeRingkeyFromScancontext(sc));
        
        // /*  for (const auto& number : ringkey) {
        //       std::cout << number << " ";
        //   }
        //   std::cout << std::endl;*/
        Eigen::MatrixXd sectorkey = scManager->makeSectorkeyFromScancontext(sc);
        double sc_dist = 1.0;
        auto match = scManager->detectClosestMatch(sc, ringkey, sectorkey, sc_dist);
        if (match.first != -1){
          std::cout <<"trans: "<< t.first << " " <<t.second;
          std::cout <<" score: "<<sc_dist<<std::endl;
        }
        if (sc_dist < min_dist) {
            min_dist = sc_dist;
            best_match = match;
            best_trans = t;
        }
    }
    int match_idx = best_match.first;
    
    if (match_idx != -1) {
        std::cout << "use index "<< match_idx<<std::endl;
        Eigen::Matrix4d init_guess = LoadData[match_idx].pose.matrix();
        Eigen::Vector3d euler = R2ypr(init_guess.block<3, 3>(0, 0));
        // Eigen::Vector3d euler = init_guess.block<3, 3>(0, 0).eulerAngles(2, 1, 0);

        euler[0] += -best_match.second;
        std::cout << "rotate yaw"<<-best_match.second<<std::endl;
        // Eigen::Matrix3d rotate = (Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitZ()) *
        //                         Eigen::AngleAxisd(current_pitch, Eigen::Vector3d::UnitY()) *
        //                         Eigen::AngleAxisd(current_roll, Eigen::Vector3d::UnitX())).toRotationMatrix();// TODO use current pr?
        Eigen::Matrix3d rotate = ypr2R(Eigen::Vector3d(euler[0],current_pitch,current_roll));                        
        init_guess.block<3, 3>(0, 0) = rotate;
        // std::cout << "original trans"<<init_guess.block<3, 1>(0, 3).transpose()<<std::endl;        
        euler = R2ypr(init_guess.block<3, 3>(0, 0));
        // std::cout << "original yaw"<<euler[0]*180/M_PI<<" pitch "<<euler[1]*180/M_PI<< " roll "<<euler[2]*180/M_PI<<std::endl;
        // euler = init_guess.block<3, 3>(0, 0).eulerAngles(2, 1, 0);

        // ICP Settings zx gicp ?
        Eigen::Vector2d offset_in_lidar{-best_trans.first,-best_trans.second};
        Eigen::Rotation2D<double> rotation(euler[0]);
        Eigen::Vector2d offset_in_map = rotation * offset_in_lidar; 
        // std::cout << "lidar offset " << offset_in_lidar.transpose() <<std::endl; 
        // std::cout << "map offset " << offset_in_map.transpose() <<std::endl; 
        init_guess.coeffRef(0, 3)= init_guess.coeffRef(0, 3)+ offset_in_map[0];
        init_guess.coeffRef(1, 3)= init_guess.coeffRef(1, 3)+ offset_in_map[1];
        // init_guess.coeffRef(2, 3) = 0;
        // std::cout << "initial yaw "<<euler[0]*180/M_PI<<" pitch "<<euler[1]*180/M_PI<< " roll "<<euler[2]*180/M_PI<<std::endl;
        // std::cout << " trans "<<init_guess.block<3, 1>(0, 3).transpose()<<std::endl;        //use the outcome of ndt as the initial guess for ICP
        Eigen::Isometry3d testtransform(init_guess);
        testMatchcloud = transformPointCloud(cloudIn, testtransform);
        icp.setInputSource(cloudIn);
        icp.setInputTarget(CloudGlobalMap);
        // std::cout << "globalLocalization icp fail "<<cloudIn->points.size()<<" "<<CloudGlobalMap->points.size()<<std::endl;
        PointCloudXYZI::Ptr unused_result(new PointCloudXYZI());
        icp.align(*unused_result, init_guess.cast<float>());
        // 未收敛，或者匹配不够好
        // if (icp.hasConverged() == false || icp.getFitnessScore() > 0.2){//TODO add number in getFitnessScore
        if (icp.hasConverged() == false || icp.getFitnessScore() > score){//TODO add number in getFitnessScore
            std::cout << "globalLocalization icp fail with score: "<< icp.getFitnessScore()<<std::endl;
            return false;
        }
        else{
            std::cout << "globalLocalization success with score: " << icp.getFitnessScore() << std::endl;
        }   
        Eigen::Isometry3d lidar_in_map;
        lidar_in_map.matrix() = icp.getFinalTransformation().matrix().cast<double>();
        correctionOdomToMap = lidar_in_map*pose.inverse();
        // euler = lidar_in_map.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0);
        // std::cout << "final yaw"<<euler[0]<<" pitch "<<euler[1]<< " roll "<<euler[2];
        // std::cout << "x "<<lidar_in_map.translation().x()<<" y "<<lidar_in_map.translation().y()<< " z "<<lidar_in_map.translation().z()<<std::endl;

        // float x, y, z, roll, pitch, yaw;
        // pcl::getTranslationAndEulerAngles(correctionOdomToMap, x, y, z, roll, pitch, yaw); //  获取上一帧 相对 当前帧的 位姿
        // std::cout << "icp results"<<" "<< x <<" "<< y <<" "<< z <<" "<< yaw <<" "<< pitch <<" "<< roll<<std::endl;
        // std::cout << "-------------------------------------------"<<std::endl;
        return true;
        // std::cout << "scancontext search success, score {} " <<match_idx<<" "<< min_dist<<std::endl;
    } else {
        // std::cout << "-------------------------------------------"<<std::endl;
        return false;
        std::cout << "scancontext search fail, score {} "<<match_idx<<" "<< min_dist<<std::endl;
    }


}

}

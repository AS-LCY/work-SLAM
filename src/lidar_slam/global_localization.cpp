#include "lidar_slam/global_localization.hpp"


namespace lidar_slam{


GlobalLocalization::GlobalLocalization(){
    loaded_sc_info_.clear();
    loaded_global_map_.reset(new PointCloudXYZI());

    loaded_keyframe_poses_.clear();
    loaded_keyframe_clouds_.clear();
    
}

GlobalLocalization::~GlobalLocalization(){
    
    loaded_sc_info_.clear();
    loaded_global_map_.reset(new PointCloudXYZI());

    loaded_keyframe_poses_.clear();
    loaded_keyframe_clouds_.clear();
    
}

// //////////////////////////////////// - load map - /////////////////////////////////////////////////////

// bool GlobalLocalization::load_map_data(std::string map_dir){
//     map_ready_ = false;
//     if(!load_cloud_map(map_dir)){
//         cout<<"load cloud map failed!"<<endl;
//         return false;
//     }
//     std::string keyframe_dir = map_dir + "/key_frame_cloud/";
//     if(!load_key_frames(keyframe_dir)){
//         cout<<"load key frame clouds failed!"<<endl;
//         return false;
//     }
//     map_ready_ = true;
//     return true;
// }

// bool GlobalLocalization::load_key_frames(std::string keyframe_dir){
//     /// load key frame poses **************************************************************************
//     std::string keyframe_pose_path = keyframe_dir + "/key_frame_pose.txt";
//     std::cout << "tring to load key_frame_pose from : " << keyframe_pose_path<<std::endl;
//     /// open pose_file
//     std::ifstream pose_file(keyframe_pose_path);
//     try {
//         if (!pose_file) {
//             throw std::runtime_error("Failed to open pose_file");
//         }
//     } catch (const std::exception& e) {
//         std::cerr << "Exception occurred: " << e.what() << std::endl;
//     }
//     /// read data line by line, split one line by ","
//     loaded_key_point_.reset(new pcl::PointCloud<PointType>());
//     loaded_keyframe_poses_.clear();
//     std::string line;
//     while (std::getline(pose_file, line)) { 
//         std::stringstream ss(line);
//         std::string token;
//         std::vector<double> values;// 单行数据已经全部临时存于 values
//         while (std::getline(ss, token, ',')) {
//             double value = std::stod(token);
//             values.push_back(value);
//         }
//         // 开始读入 read_one_line
//         KeyPose read_one_line;
//         int idx = 0;
//         /// 第 0 项： index
//         read_one_line.index = static_cast<int>(values[idx++]);// 数据保存 ---------------------------------
//         /// 第 1 项： time (lidar_end_time)
//         read_one_line.time = static_cast<double>(values[idx++]);// 数据保存 -------------------------------
//         // std::cout << "  idx: " <<read_one_line.index << std::endl;
//         /// 第 2-17 项： 转换矩阵（4 x 4），旋转 + 平移
//         for (int row = 0; row < 4; ++row) {
//             for (int col = 0; col < 4; ++col) {
//                 read_one_line.pose(row, col) = values[idx++];
//             }
//         }
//         Eigen::Vector3d translation = read_one_line.pose.translation(); 
//         PointType point;
//         point.x = translation.x(); // 将x坐标设置为平移向量的x分量
//         point.y = translation.y(); // 将y坐标设置为平移向量的y分量
//         point.z = translation.z(); // 将z坐标设置为平移向量的z分量
//         loaded_key_point_->push_back(point);// 数据保存 ---------------------------------------------------------
//         // Eigen::Matrix3d matrix = read_one_line.pose.linear();
//         // Eigen::Vector3d euler_angles = matrix.eulerAngles(2, 1, 0); // 获取 Z-Y-X 欧拉角
//         Eigen::Matrix3d matrix = read_one_line.pose.matrix().block<3, 3>(0, 0);
//         Eigen::Vector3d euler_angles = R2ypr(matrix);// defined in common_lib.h
//         read_one_line.yaw   = euler_angles[0];// 数据保存 -------------------------------------------------
//         read_one_line.pitch = euler_angles[1];// 数据保存 -------------------------------------------------
//         read_one_line.roll  = euler_angles[2];// 数据保存 -------------------------------------------------
//         loaded_keyframe_poses_.push_back(read_one_line);// 数据保存 ---------------------------------------
//     }
//     pose_file.close();
//     /// load key frame poses end *************************************************************************
//     /// load key frame cloud start ***********************************************************************
//     loaded_keyframe_clouds_.clear();
//     int key_poses_size = loaded_keyframe_poses_.size();
//     for(int i=0; i< key_poses_size; i++){
//         int pose_index = loaded_keyframe_poses_[i].index;
//         std::string key_cloud_path = keyframe_dir + "/" + std::to_string(pose_index) +  ".pcd";
//         std::cout << "tring to load key_frame_cloud from : " << key_cloud_path<<std::endl;
//         PointCloudXYZI::Ptr temp_cloud(new PointCloudXYZI());
//         if (std::filesystem::exists(key_cloud_path)){
//             pcl::io::loadPCDFile(key_cloud_path, *temp_cloud);
//             loaded_keyframe_clouds_.push_back((temp_cloud));
//             std::cout <<"key cloud loaded --- points count: "<<temp_cloud->points.size() << std::endl;
//         }else {
//             std::cerr << "key cloud file does not exist." << std::endl;
//             return false;
//         }
//     }
//     return true;
// }

// bool GlobalLocalization::load_cloud_map(std::string map_dir){
//     /// load cloud_map.pcd ***************************************************************************
//     loaded_global_map_.reset(new PointCloudXYZI());
//     std::string cloud_map_file_path = map_dir + "cloud_map.pcd";
//     std::cout << "tring to load map from : " << cloud_map_file_path<<std::endl;
//     if (std::filesystem::exists(cloud_map_file_path)){
//         pcl::io::loadPCDFile(cloud_map_file_path, *loaded_global_map_);
//         std::cout <<"map loaded --- points count: "<<loaded_global_map_->points.size() << std::endl;
//     }else {
//         std::cerr << "map file does not exist." << std::endl;
//         return false;
//     }
//     /// TODO: show map point ---------------------------------
//     /// load data(pose & ScanContex) *****************************************************************
//     std::string sc_data_file_path = map_dir + "data";
//     std::cout << "tring to load sc-data from : " << sc_data_file_path<<std::endl;
//     /// open file
//     std::ifstream file(sc_data_file_path);
//     try {
//         if (!file) {
//             throw std::runtime_error("Failed to open file");
//         }
//     } catch (const std::exception& e) {
//         std::cerr << "Exception occurred: " << e.what() << std::endl;
//     }
//     KeyMat polarcontext_invkeys_mat;
//     std::vector<Eigen::MatrixXd> polarcontexts;
//     sc_manager_.reset(new SCManager());
//     loaded_sc_info_.clear();
//     // polarcontext_invkeys_mat.clear();
//     // polarcontexts.clear();
//     // loaded_key_point_.reset(new pcl::PointCloud<pcl::PointXYZ>());
//     // accumulate_map_->points.clear(); // 没用上
//     /// read data line by line, split one line by ","
//     std::string line;
//     while (std::getline(file, line)) {        
//         ScInfo read_one_line;
//         std::stringstream ss(line);
//         std::string token;
//         std::vector<double> values;// 单行数据临时存于 values
//         while (std::getline(ss, token, ',')) {
//             double value = std::stod(token);
//             values.push_back(value);
//         }
//         int index = 0;
//         /// 第 1 项： id
//         read_one_line.id = static_cast<int>(values[index++]);// 数据保存 ---------------------------------
//         // std::cout << "  id: " <<read_one_line.id << std::endl;
//         /// 第 2-17 项： 转换矩阵（4 x 4），旋转 + 平移
//         for (int row = 0; row < 4; ++row) {
//             for (int col = 0; col < 4; ++col) {
//                 read_one_line.pose(row, col) = values[index++];
//             }
//         }
//         accumulate_key_pose_.push_back(read_one_line.pose);// 数据保存 -----------------------------------
//         // std::cout << " \n transform:"<<read_one_line.pose.matrix() << std::endl;
//         /// 第 18-19 项： sc 数据size
//         int maxrow = values[index++];
//         int maxcol = values[index++];
//         /// check： sc 数据 size 是否对应
//         if (values.size() - index != (maxrow*maxcol)){
//             std::cout << " error :"<<values.size()<<" "<<index<<" "<<maxrow*maxcol<< std::endl;
//             return false;
//         }
//         /// 第 20-end 项： sc 数据
//         read_one_line.polarcontext.resize(maxrow,maxcol);
//         for (int row = 0; row < maxrow; ++row) {
//             for (int col = 0; col < maxcol; ++col) {
//                 read_one_line.polarcontext(row, col) = values[index++];// 数据保存 ------------------------
//             }
//         } 
//         // loadScManager.loadScancontextAndKeys(read_one_line.polarcontext); 
//         Eigen::MatrixXd sc = read_one_line.polarcontext; // v1
//         Eigen::MatrixXd ringkey = sc_manager_->makeRingkeyFromScancontext( sc );
//         polarcontext_invkeys_mat.push_back(eig2stdvec(ringkey));  // 数据转换后保存 -----------------------
//         polarcontexts.push_back(sc); // 数据保存 ---------------------------------------------------------
//         //   if (i == 17)
//         //    std::cout << " \n matrix:"  << std::endl;
//         //    std::cout << value <<",";
//         loaded_sc_info_.push_back(read_one_line);// 数据保存 ------------------------------------------------
//     }
//     file.close();
//     if (loaded_sc_info_.size() == 0)
//         return false;
//     sc_manager_->buildRingKeyKDTree(polarcontext_invkeys_mat, polarcontexts);
//     std::cout << "get_load_data : " << loaded_sc_info_.size() << std::endl;
//     /// load data(pose & ScanContex) end *****************************************************************
//     return true;
// }

// //////////////////////////////////// - load map end - /////////////////////////////////////////////////


//////////////////////////////////// - global_localize - /////////////////////////////////////////
bool GlobalLocalization::global_localize(PointCloudXYZI::Ptr cloud_in, 
                                            Eigen::Isometry3d pose, 
                                            Matrix3d initial_rotate, 
                                            double score_thr){
    /// check map data status *********************************************************************
    /// make ScanContext using loaded_sc_info
    if(!global_map_ready_){
        std::cout<<"global map not ready!" <<std::endl;
        return false;
    }
    if(!sc_manager_ready_){
        std::cout<<"sc manager not ready!" <<std::endl;
        return false;
    }
    
    // /// check map data status
    // if (!map_ready_) {
    //     std::cout<<"map not ready, global localizzation stopped!"<<std::endl;
    //     return false;
    // }

    /// search best match of scancontex ***********************************************************
    std::pair<int, float> best_match{-1, 0.0};
    std::pair<double, double> best_trans;
    if(!scancontex_search(cloud_in, initial_rotate, best_match, best_trans)){
        std::cout<<"scancontex search failed!" <<std::endl;
        return false;
    }
    int best_match_idx = best_match.first;
    std::cout << "scancontext search success, use index "<< best_match_idx <<std::endl;

    /// get init transform ************************************************************************
    Eigen::Matrix4d init_guess = cal_init_transform(initial_rotate, best_match, best_trans);     
    Eigen::Isometry3d test_transform(init_guess);/// debug
    test_match_cloud_ = transformPointCloud(cloud_in, test_transform);/// debug

    /// exec icp（result: global_odom_to_map）******************************************************
    if(!registration_icp(cloud_in, pose, init_guess, score_thr)){
        return false;
    }else{
        return true;
    }
}


bool GlobalLocalization::set_global_map(PointCloudXYZI::Ptr input_global_map){
    if(input_global_map->empty() || input_global_map->points.empty() || input_global_map->points.size()==0){
        std::cout <<" loaded global map empty!"<<std::endl;
        return false;
    }

    *loaded_global_map_ = *input_global_map;
    global_map_ready_ = true;
    return true;
}

/// @brief  
/// @param input_sc_info   : param in, loaded data
/// @result : sc_manager_
/// @return : if fill success
bool GlobalLocalization::fill_sc_manager(std::vector<ScInfo> input_sc_info){
    if(input_sc_info.empty() || input_sc_info.size()==0){
        std::cout <<" loaded sc info empty!"<<std::endl;
        return false;
    }

    KeyMat polarcontext_invkeys_mat;
    std::vector<Eigen::MatrixXd> polarcontexts;
    for(ScInfo& scinfo : input_sc_info ){
        Eigen::MatrixXd sc = scinfo.polarcontext;
        Eigen::MatrixXd ringkey = sc_manager_->makeRingkeyFromScancontext( sc );
        polarcontext_invkeys_mat.push_back(eig2stdvec(ringkey));
        polarcontexts.push_back(sc);
    }
    sc_manager_->buildRingKeyKDTree(polarcontext_invkeys_mat, polarcontexts);// save in sc_manager_
    std::cout << "get_load_data : " << loaded_sc_info_.size() << std::endl;

    sc_manager_ready_ = true;
    return true;
}

/// @brief  
/// @param cloud_in         : param in 
/// @param initial_rotate   : param in 
/// @param best_match       : result out 
/// @param best_trans       : result out 
/// @return : if search success
bool GlobalLocalization::scancontex_search(PointCloudXYZI::Ptr cloud_in, Matrix3d initial_rotate, 
                                            std::pair<int, float>& best_match, std::pair<double, double>& best_trans){

    // transform (gravity_align) curr cloud
    PointCloudXYZI::Ptr gravity_aligned_cLoud(new PointCloudXYZI());
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.matrix().block<3, 3>(0, 0) = initial_rotate;
    *gravity_aligned_cLoud = *transformPointCloud(cloud_in, transform);

    // set search_trans 
    /// TODO: parameterize search_trans ?
    std::vector<std::pair<double, double>> search_trans = {
        {0, 0}, {-4, 0}, {4, 0}, {0, -4}, {0, 4}, {-4, -4}, {-4, 4}, {4, -4}, {4, 4},
                {-2, 0}, {2, 0}, {0, -2}, {0, 2}, {-2, -2}, {-2, 2}, {2, -2}, {2, 2}
    };

    double min_dist = std::numeric_limits<double>::max();
    // std::pair<int, float> best_match{-1, 0.0};
    // std::pair<double, double> best_trans;
    for (auto &t : search_trans) {
        Eigen::MatrixXd sc = sc_manager_->makeScancontext(*(gravity_aligned_cLoud), t.first, t.second);// get sc of curr cloud
        std::vector<float> ringkey = eig2stdvec(sc_manager_->makeRingkeyFromScancontext(sc));
        Eigen::MatrixXd sectorkey = sc_manager_->makeSectorkeyFromScancontext(sc);
        /// TODO: parameterize sc_dist
        double sc_dist = 1.0;
        auto match = sc_manager_->detectClosestMatch(sc, ringkey, sectorkey, sc_dist);
        if (match.first != -1){
          std::cout <<"trans: "<< t.first << " " <<t.second;
          std::cout <<"; score: "<<sc_dist<<std::endl;
        }
        if (sc_dist < min_dist) {
            min_dist = sc_dist;
            best_match = match;
            best_trans = t;
        }
    }

    // check scancontext search
    int match_idx = best_match.first;

    if(match_idx == -1){
        std::cout << "scancontext search fail, score {}: "<<match_idx<<" "<< min_dist<<std::endl;
        return false;
    }else{
        return true;// match_idx != -1, (scancontext search success)
    }

}

/// @brief 
/// @param initial_rotate   : param in 
/// @param best_match       : param in 
/// @param best_trans       : param in 
/// @return Eigen::Matrix4d : init_transform (used in icp)
Eigen::Matrix4d GlobalLocalization::cal_init_transform(Matrix3d initial_rotate, std::pair<int, float> best_match, std::pair<double, double> best_trans){
    int match_idx = best_match.first;

    Eigen::Matrix4d init_guess = loaded_sc_info_[match_idx].pose.matrix();// use loaded data
    Eigen::Vector3d euler = R2ypr(init_guess.block<3, 3>(0, 0));
    // Eigen::Vector3d euler = init_guess.block<3, 3>(0, 0).eulerAngles(2, 1, 0);

    euler[0] += -best_match.second;
    std::cout << "rotate yaw"<<-best_match.second<<std::endl;

    Eigen::Vector3d current_euler = R2ypr(initial_rotate);//R2ypr(pose.matrix().block<3, 3>(0, 0));
    //  Eigen::Vector3d current_euler = pose.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0);
    double current_pitch = current_euler[1];
    double current_roll = current_euler[2];

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


    return init_guess;
}

/// @brief  
/// @param cloud_in     : param in 
/// @param pose         : param in 
/// @param init_guess   : param in 
/// @return : (stored in private:) Eigen::Isometry3d global_odom_to_map_
bool GlobalLocalization::registration_icp(PointCloudXYZI::Ptr cloud_in, Eigen::Isometry3d pose, Eigen::Matrix4d init_guess, double score_thr){
    // set: icp-common
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    // set: icp-cloud
    icp.setInputSource(cloud_in);
    icp.setInputTarget(loaded_global_map_);

    // exec icp
    PointCloudXYZI::Ptr unused_result(new PointCloudXYZI());
    icp.align(*unused_result, init_guess.cast<float>());
    // 未收敛，或者匹配不够好
    if (icp.hasConverged() == false || icp.getFitnessScore() > score_thr){//TODO add number in getFitnessScore
        std::cout << "globalLocalization icp fail with score: "<< icp.getFitnessScore()<<std::endl;
        return false;
    }else{
        std::cout << "globalLocalization success with score: " << icp.getFitnessScore() << std::endl;
    }   
    Eigen::Isometry3d first_lidar_in_map;// first_lidar in_map: == odom
    first_lidar_in_map.matrix() = icp.getFinalTransformation().matrix().cast<double>();
    global_odom_to_map_ = first_lidar_in_map * pose.inverse();
    // euler = first_lidar_in_map.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0);
    // std::cout << "final yaw"<<euler[0]<<" pitch "<<euler[1]<< " roll "<<euler[2];
    // std::cout << "x "<<first_lidar_in_map.translation().x()<<" y "<<first_lidar_in_map.translation().y()<< " z "<<first_lidar_in_map.translation().z()<<std::endl;

    // float x, y, z, roll, pitch, yaw;
    // pcl::getTranslationAndEulerAngles(correctionOdomToMap, x, y, z, roll, pitch, yaw); //  获取上一帧 相对 当前帧的 位姿
    // std::cout << "icp results"<<" "<< x <<" "<< y <<" "<< z <<" "<< yaw <<" "<< pitch <<" "<< roll<<std::endl;
    // std::cout << "-------------------------------------------"<<std::endl;

    return true;

}


}// namespace lidar_slam
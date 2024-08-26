#include "lidar_slam/cloud_map.hpp"

namespace lidar_slam{

CloudMap::CloudMap(){
    loaded_sc_info_.clear();
    loaded_global_map_.reset(new PointCloudXYZI());

    loaded_keyframe_poses_.clear();
    loaded_keyframe_clouds_.clear();

}

CloudMap::~CloudMap(){
    loaded_sc_info_.clear();
    loaded_global_map_.reset(new PointCloudXYZI());

    loaded_keyframe_poses_.clear();
    loaded_keyframe_clouds_.clear();

}

//////////////////////////////////// - load map - /////////////////////////////////////////////////////

bool CloudMap::load_map_data(std::string map_dir){
    map_data_ready_ = false;
    
    if(!load_cloud_map(map_dir)){
        cout<<"load cloud map failed!"<<endl;
        return false;
    }
    std::string keyframe_dir = map_dir + "/key_frame_cloud/";
    if(!load_key_frames(keyframe_dir)){
        cout<<"load key frame clouds failed!"<<endl;
        return false;
    }

    map_data_ready_ = true;
    cout<<"load map_data success, map_data_ready_ = true"<<endl;
    return true;
}


bool CloudMap::load_key_frames(std::string keyframe_dir){
    /// load key frame poses **************************************************************************
    std::string keyframe_pose_path = keyframe_dir + "/key_frame_pose.txt";
    // std::cout << "loading key_frame_pose from : " << keyframe_pose_path<<std::endl;
    std::cout << "loading key_frame_pose from : " << keyframe_pose_path<<" -- ";

    /// open pose_file
    std::ifstream pose_file(keyframe_pose_path);
    try {
        if (!pose_file) {
            throw std::runtime_error("Failed to open pose_file");
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
    }

    /// read data line by line, split one line by ","
    loaded_key_point_.reset(new pcl::PointCloud<PointType>());
    loaded_keyframe_poses_.clear();
    std::string line;
    while (std::getline(pose_file, line)) { 
        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;// 单行数据已经全部临时存于 values
        while (std::getline(ss, token, ',')) {
            double value = std::stod(token);
            values.push_back(value);
        }

        // 开始读入 read_one_line
        KeyPose read_one_line;
        int idx = 0;

        /// 第 0 项： index
        read_one_line.index = static_cast<int>(values[idx++]);// 数据保存 ---------------------------------
        /// 第 1 项： time (lidar_end_time)
        read_one_line.time = static_cast<double>(values[idx++]);// 数据保存 -------------------------------
        // std::cout << "  idx: " <<read_one_line.index << std::endl;

        /// 第 2-17 项： 转换矩阵（4 x 4），旋转 + 平移
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                read_one_line.pose(row, col) = values[idx++];
            }
        }
        Eigen::Vector3d translation = read_one_line.pose.translation();
        PointType point;
        point.x = translation.x(); // 将x坐标设置为平移向量的x分量
        point.y = translation.y(); // 将y坐标设置为平移向量的y分量
        point.z = translation.z(); // 将z坐标设置为平移向量的z分量
        loaded_key_point_->push_back(point);// 数据保存 ---------------------------------------------------------
         
        // Eigen::Matrix3d matrix = read_one_line.pose.linear();
        // Eigen::Vector3d euler_angles = matrix.eulerAngles(2, 1, 0); // 获取 Z-Y-X 欧拉角
        Eigen::Matrix3d matrix = read_one_line.pose.matrix().block<3, 3>(0, 0);
        Eigen::Vector3d euler_angles = R2ypr(matrix);// defined in common_lib.h
        read_one_line.yaw   = euler_angles[0];// 数据保存 -------------------------------------------------
        read_one_line.pitch = euler_angles[1];// 数据保存 -------------------------------------------------
        read_one_line.roll  = euler_angles[2];// 数据保存 -------------------------------------------------

        loaded_keyframe_poses_.push_back(read_one_line);// 数据保存 ---------------------------------------
    }
    pose_file.close();
    std::cout << "success -- loaded_keyframe_poses size: "<<loaded_keyframe_poses_.size() <<std::endl;
    /// load key frame poses end *************************************************************************

    /// load key frame cloud start ***********************************************************************
    loaded_keyframe_clouds_.clear();
    int key_poses_size = loaded_keyframe_poses_.size();
    for(int i=0; i< key_poses_size; i++){
        int pose_index = loaded_keyframe_poses_[i].index;
        std::string key_cloud_path = keyframe_dir + "/" + std::to_string(pose_index) +  ".pcd";
        std::cout << "loading key_frame_cloud from : " << key_cloud_path<<" -- ";

        PointCloudXYZI::Ptr temp_cloud(new PointCloudXYZI());
        // if (std::filesystem::exists(key_cloud_path)){
        if (0 == access(key_cloud_path.c_str(), 0)){
            pcl::io::loadPCDFile(key_cloud_path, *temp_cloud);
            loaded_keyframe_clouds_.push_back((temp_cloud));
            std::cout <<"success -- points count: "<<temp_cloud->points.size() << std::endl;
        }else {
            std::cerr << "failed -- key cloud file "<< std::to_string(pose_index) <<".pcd does not exist." << std::endl;
            return false;
        }
    }
    
    return true;
}

bool CloudMap::load_cloud_map(std::string map_dir){
    /// load cloud_map.pcd ***************************************************************************
    loaded_global_map_.reset(new PointCloudXYZI());
    std::string cloud_map_file_path = map_dir + "cloud_map.pcd";
    // std::cout << "loading map from : " << cloud_map_file_path<<std::endl;
    std::cout << "loading map from : " << cloud_map_file_path<<" -- ";
    // if (std::filesystem::exists(cloud_map_file_path)){
    if (0 == access(cloud_map_file_path.c_str(), 0)){
        pcl::io::loadPCDFile(cloud_map_file_path, *loaded_global_map_);
        std::cout <<"success -- points count: "<<loaded_global_map_->points.size() << std::endl;
    }else {
        std::cerr << "failed -- map file does not exist." << std::endl;
        return false;
    }
    /// TODO: show map point ---------------------------------

    /// load data(pose & ScanContex) *****************************************************************
    std::string sc_data_file_path = map_dir + "data";
    loaded_sc_info_.clear();

    /// open file
    // std::cout << "tring to load sc-data from : " << sc_data_file_path<<std::endl;
    std::cout << "loading sc-data from : " << sc_data_file_path<<" -- ";
    std::ifstream file(sc_data_file_path);
    try {
        if (!file) {
            throw std::runtime_error("Failed to open file");
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
    }
    
    // KeyMat polarcontext_invkeys_mat;
    // std::vector<Eigen::MatrixXd> polarcontexts;
    /// read data line by line, split one line by ","
    std::string line;
    while (std::getline(file, line)) {        
        ScInfo read_one_line;
        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;// 单行数据临时存于 values
        while (std::getline(ss, token, ',')) {
            double value = std::stod(token);
            values.push_back(value);
        }
        int index = 0;
        /// 第 1 项： id
        read_one_line.id = static_cast<int>(values[index++]);// 数据保存 ---------------------------------
        // std::cout << "  id: " <<read_one_line.id << std::endl;

        /// 第 2-17 项： 转换矩阵（4 x 4），旋转 + 平移
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                read_one_line.pose(row, col) = values[index++];// 数据保存 -------------------------------
            }
        }

        /// 第 18-19 项： sc 数据size
        int maxrow = values[index++];
        int maxcol = values[index++];
        /// check： sc 数据 size 是否对应
        if (values.size() - index != (maxrow*maxcol)){
            std::cout << " error :"<<values.size()<<" "<<index<<" "<<maxrow*maxcol<< std::endl;
            return false;
        }

        /// 第 20-end 项： sc 数据
        read_one_line.polarcontext.resize(maxrow,maxcol);
        for (int row = 0; row < maxrow; ++row) {
            for (int col = 0; col < maxcol; ++col) {
                read_one_line.polarcontext(row, col) = values[index++];// 数据保存 ------------------------
            }
        } 
        loaded_sc_info_.push_back(read_one_line);// 数据保存 ----------------------------------------------
    }
    file.close();

    if (loaded_sc_info_.size() == 0)
        return false;
    // sc_manager_->buildRingKeyKDTree(polarcontext_invkeys_mat, polarcontexts);
    std::cout << "success -- loaded_sc_info size : " << loaded_sc_info_.size() << std::endl;


    /// load data(pose & ScanContex) end *****************************************************************


    return true;
}
//////////////////////////////////// - load map end - /////////////////////////////////////////////////



//////////////////////////////////// - save map - /////////////////////////////////////////////////////
/// TODO: 
// bool CloudMap::save_map_data(){
// }



//////////////////////////////////// - save map end - /////////////////////////////////////////////////

} // namespace lidar_slam
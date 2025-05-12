#include "lidar/robosense/lidar_preproc_Airy.h"


namespace localization_module {

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

LidarPreprocAiry::LidarPreprocAiry(){
    if(!set_param()){
        ROS_ERROR_STREAM(RED << "Set lidar param failed!" << RESET);
    }else {
        ROS_INFO("Set lidar-Airy param successfully!");
    }
    
    allocate_memory_init_variable();

    ROS_INFO("Reset to lidar_preproc_Airy successfully!");

}


LidarPreprocAiry::~LidarPreprocAiry(){

}


// ///////////////// 入口函数 /////////////////  
bool LidarPreprocAiry::pre_process(const sensor_msgs::PointCloud2::ConstPtr ros_msg_in, PointCloudType::Ptr& pcl_xyzin_out){
    if(extract_cloud_method_ == 0){
        // ROS_INFO_STREAM("use sampling ");
        cloud_dense_->clear();
        pcl_xyzin_out->clear();
        msg2pcl_clip(ros_msg_in, cloud_dense_);    
        sampling_cloud(cloud_dense_, pcl_xyzin_out);
    }else if (extract_cloud_method_ == 3){
        cloud_dense_->clear();
        pcl_xyzin_out->clear();
        msg2pcl_feat_pre(ros_msg_in);
        // extract_by_ring_feature(pcl_xyzin_out);
        extract_by_ring_feature();

        *pcl_xyzin_out += *cloud_corner_;
        *pcl_xyzin_out += *cloud_surface_;

        pcl_xyzin_out->header = cloud_dense_->header;
        pcl_xyzin_out->width    = pcl_xyzin_out->points.size();
        pcl_xyzin_out->height   = 1;
        pcl_xyzin_out->is_dense = 1;

    }else{
        return false;
    }
    
    

    return true;
}




// current used
bool LidarPreprocAiry::msg2pcl_clip(const sensor_msgs::PointCloud2::ConstPtr ros_msg_in, PointCloudType::Ptr pcl_xyzin_out){

    ROS_INFO_ONCE("Airy: ros_msg_in --> pcl_xyzin_out");

    int cloud_num = ros_msg_in->height * ros_msg_in->width;
    double header_time = ros_msg_in->header.stamp.toSec();

    ///// MetaData --- header 
    pcl::PCLHeader pcl_header;
    pcl_header.seq = ros_msg_in->header.seq;
    pcl_header.stamp = ros_msg_in->header.stamp.toNSec() / 1000ull;
    pcl_header.frame_id = ros_msg_in->header.frame_id;
    ///// MetaData --- field
    std::vector<pcl::PCLPointField> pcl_fields;
    
    pcl_fields.resize(ros_msg_in->fields.size());
    std::vector<sensor_msgs::PointField>::const_iterator it = ros_msg_in->fields.begin();
    int i = 0;
    for(; it != ros_msg_in->fields.end(); ++it, ++i) {
      pcl_fields[i].name = it->name;
      pcl_fields[i].offset = it->offset;
      pcl_fields[i].datatype = it->datatype;
      pcl_fields[i].count = it->count;
    //   ROS_INFO_STREAM(RED<<"field-name: "<<pcl_fields[i].name<<RESET); // check filed name
    }
    //// create Mapping
    pcl::MsgFieldMap field_map;
    pcl::createMapping<RsPointXYZIRT> (pcl_fields, field_map);

    ///////////////////////////////////////////////////////////////////////////////////////////
    /// fill pcl_xyzin_out

    // 说明： 每一条 ring 的第一个点的数据，存于 height = 0；
    uint valid_num = 0;
    pcl_xyzin_out->points.reserve(cloud_num);
    for (std::uint32_t h = 0; h < ros_msg_in->height; ++h){
        const std::uint8_t* h_data = &ros_msg_in->data[h * ros_msg_in->row_step];
        for (std::uint32_t w = 0; w < ros_msg_in->width; ++w){//width = ring =48
            const std::uint8_t* msg_data = h_data + w * ros_msg_in->point_step;

    // for (std::uint32_t w = 0; w < ros_msg_in->width; ++w){//width = ring =48
    //     const std::uint8_t w_step = w * ros_msg_in->point_step;
    //     for (std::uint32_t h = 0; h < ros_msg_in->height; ++h){
    //         const std::uint8_t* h_data = &ros_msg_in->data[h * ros_msg_in->row_step];
    //         const std::uint8_t* msg_data = h_data + w_step;    
            auto col = h;
            auto row = w;
            RsPointXYZIRT temp_point;
            RsPointXYZIRT* curpt = &temp_point;
            std::uint8_t* curpt_data = reinterpret_cast<std::uint8_t*>(curpt);

            for (const pcl::detail::FieldMapping& mapping : field_map){
                memcpy (curpt_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
            }

            ///// make it dense
            if (lidar_common::is_nan_pt(*curpt)) { continue; } 
            
            // if(abs(curpt->x) > thr_region_x_ || abs(curpt->y) > thr_region_y_ || abs(curpt->z) > thr_region_z_){
            //    continue;
            // }
            double range_square = curpt->x * curpt->x + curpt->y * curpt->y + curpt->z * curpt->z;
            if(range_square < blind_range_square_ || range_square > max_range_square_){
                continue;
            }

            // // if(row % point_filter_num_ == 0){
            // if(col % point_filter_num_ == 0 && row % ring_filter_num_ == 0){
            //     PointType xyzin_point;
            //     xyzin_point.x = curpt->x;
            //     xyzin_point.y = curpt->y;
            //     xyzin_point.z = curpt->z;
            //     xyzin_point.intensity = curpt->intensity;
                
            //     xyzin_point.curvature = (curpt->timestamp - header_time) * 1000; // offset, unit = ms
            //     pcl_xyzin_out->points.push_back(xyzin_point);
            // }

            PointType xyzin_point;
            xyzin_point.x = curpt->x;
            xyzin_point.y = curpt->y;
            xyzin_point.z = curpt->z;
            xyzin_point.intensity = curpt->intensity;            
            // xyzin_point.curvature = (curpt->timestamp - header_time) * 1000; // offset, unit = ms
            xyzin_point.curvature = curpt->timestamp - header_time; // offset, unit = second

            pcl_xyzin_out->points.push_back(xyzin_point);

        }
    }
    pcl_xyzin_out->points.shrink_to_fit(); // 改变 capacity 大小，可能会导致内存的重新分配，增加性能开销
    ///// Copy info fields
    pcl_xyzin_out->header   = pcl_header;
    pcl_xyzin_out->width    = pcl_xyzin_out->points.size();
    pcl_xyzin_out->height   = 1;
    pcl_xyzin_out->is_dense = 1;

    return true;
}


// bool LidarPreprocAiry::pre_process(const pcl::PointCloud<RsPointXYZIRT>::Ptr pcl_rs_in, PointCloudType::Ptr pcl_xyzin_out){

//     ROS_INFO("Airy: pcl_rs_in --> pcl_xyzin_out");
//     const int extract_cloud_method = param_.extract_cloud_method;
//     int plsize = pcl_rs_in->width;

//     pcl_xyzin_out->clear();
//     pcl_xyzin_out->reserve(plsize);


//     if (extract_cloud_method == 0){
//         extract_cloud_by_interval_sampling(pcl_rs_in, pcl_xyzin_out);
//     } else{
//         // printf("extract_cloud_method set error!\n");
//         ROS_ERROR_STREAM(RED << "extract_cloud_method set error!" << RESET);
//         exit(1);
//     }
    
//     // printf("extract lidar count: %ld\n", pcl_xyzin_out->points.size());
//     ROS_INFO("extract lidar count: %ld", pcl_xyzin_out->points.size());

//     return true;
// }

// //////////////////////////////////////////////////////////////////////////////////////////////////
// // 间隔采样
// void LidarPreprocAiry::extract_cloud_by_interval_sampling(const pcl::PointCloud<RsPointXYZIRT>::Ptr pcl_rs_in, PointCloudType::Ptr pcl_xyzin_out){

// }

//// used to prepare extracting feature points
bool LidarPreprocAiry::msg2pcl_feat_pre(const sensor_msgs::PointCloud2::ConstPtr ros_msg_in){

    ROS_INFO_ONCE("Airy: ros_msg_in --> pcl_xyzin_out");

    int cloud_num = ros_msg_in->height * ros_msg_in->width;
    double header_time = ros_msg_in->header.stamp.toSec();

    ///// MetaData --- header 
    pcl::PCLHeader pcl_header;
    pcl_header.seq = ros_msg_in->header.seq;
    pcl_header.stamp = ros_msg_in->header.stamp.toNSec() / 1000ull;
    pcl_header.frame_id = ros_msg_in->header.frame_id;
    ///// MetaData --- field
    std::vector<pcl::PCLPointField> pcl_fields;
    
    pcl_fields.resize(ros_msg_in->fields.size());
    std::vector<sensor_msgs::PointField>::const_iterator it = ros_msg_in->fields.begin();
    int i = 0;
    for(; it != ros_msg_in->fields.end(); ++it, ++i) {
      pcl_fields[i].name = it->name;
      pcl_fields[i].offset = it->offset;
      pcl_fields[i].datatype = it->datatype;
      pcl_fields[i].count = it->count;
    //   ROS_INFO_STREAM(RED<<"field-name: "<<pcl_fields[i].name<<RESET); // check filed name
    }
    //// create Mapping
    pcl::MsgFieldMap field_map;
    pcl::createMapping<RsPointXYZIRT> (pcl_fields, field_map);

    ///////////////////////////////////////////////////////////////////////////////////////////
    /// fill cloud_dense_

    // 说明： 每一条 ring 的第一个点的数据，存于 height = 0；
    uint valid_num = 0;
    cloud_dense_->points.reserve(cloud_num);

    // std::vector<bool> start_ring_idx_fill;
    // start_ring_idx_fill.assign(ring_cnt_, false);

    for (std::uint32_t w = 0; w < ros_msg_in->width; ++w){//width = ring =48
        ring_index_start_[w] = valid_num -1 + 5;
        const std::uint8_t w_step = w * ros_msg_in->point_step;
        for (std::uint32_t h = 0; h < ros_msg_in->height; ++h){
            const std::uint8_t* h_data = &ros_msg_in->data[h * ros_msg_in->row_step];
            const std::uint8_t* msg_data = h_data + w_step;
            auto col = h;
            auto row = w;
            RsPointXYZIRT temp_point;
            RsPointXYZIRT* curpt = &temp_point;
            std::uint8_t* curpt_data = reinterpret_cast<std::uint8_t*>(curpt);

            for (const pcl::detail::FieldMapping& mapping : field_map){
                memcpy (curpt_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
            }

            ///// make it dense
            if (lidar_common::is_nan_pt(*curpt)) { continue; } 
            
            // if(abs(curpt->x) > thr_region_x_ || abs(curpt->y) > thr_region_y_ || abs(curpt->z) > thr_region_z_){
            //    continue;
            // }
            double range_square = curpt->x * curpt->x + curpt->y * curpt->y + curpt->z * curpt->z;
            if(range_square < blind_range_square_ || range_square > max_range_square_){
                continue;
            }

            // // if(row % point_filter_num_ == 0){
            // if(col % point_filter_num_ == 0 && row % ring_filter_num_ == 0){
            //     PointType xyzin_point;
            //     xyzin_point.x = curpt->x;
            //     xyzin_point.y = curpt->y;
            //     xyzin_point.z = curpt->z;
            //     xyzin_point.intensity = curpt->intensity;
                
            //     xyzin_point.curvature = (curpt->timestamp - header_time) * 1000; // offset, unit = ms
            //     cloud_dense_->points.push_back(xyzin_point);
            // }

            PointType xyzin_point;
            xyzin_point.x = curpt->x;
            xyzin_point.y = curpt->y;
            xyzin_point.z = curpt->z;
            xyzin_point.intensity = curpt->intensity;            
            xyzin_point.curvature = (curpt->timestamp - header_time) * 1000; // offset, unit = ms
            // xyzin_point.curvature = (curpt->timestamp - header_time); // offset, unit = second

            // // ring_index_start_: the first pnt of one ring
            // if(!start_ring_idx_fill[w]){
            //     ring_index_start_[w] = valid_num -1 + 5; /// TODO: check if need -1, 按理应该不需要-1, 但是liosam是-1,需确认
            //     start_ring_idx_fill[w] = true;
            // }
            // // ring_index_end_: the last pnt of one ring
            // ring_index_end_[w] = valid_num -1 - 5; /// TODO: check if need -1

            cloud_dense_->points.push_back(xyzin_point);
            pnt_col_idx_[valid_num] = col;
            pnt_range_[valid_num] = std::sqrt(range_square);
            valid_num++;

        }
        ring_index_end_[w] = valid_num -1 - 5;
        // ROS_INFO_STREAM("ring: " << w << ", ring_index_start_:" << ring_index_start_[w]);
        // ROS_INFO_STREAM("ring: " << w << ", ring_index_end_  :" << ring_index_end_[w]);
        // ROS_INFO_STREAM("ring: " << w << ", point count      :" << ring_index_end_[w]+6 - ring_index_start_[w]+4);
        // ROS_INFO_STREAM("-----------------------------------------");
    }
    // cloud_dense_->points.shrink_to_fit();
    ///// Copy info fields
    cloud_dense_->header   = pcl_header;
    cloud_dense_->width    = cloud_dense_->points.size();
    cloud_dense_->height   = 1;
    cloud_dense_->is_dense = 1;

    // pcl_xyzin_out = cloud_dense_;

    return true;
}


// liosam 中的feature extract
// bool LidarPreprocAiry::extract_by_ring_feature(PointCloudType::Ptr pcl_xyzin_out){
bool LidarPreprocAiry::extract_by_ring_feature(){
    // cloud_dense_, pnt_range_
    // ring_index_start_, ring_index_end_
    // pnt_col_idx_

    // 计算每个点的曲率
    cal_smoothness();

    // 标记遮挡和平行点
    mark_occluded_points();

    // 提取surface和corner特征
    extract_feature_points();


    return true;

}


// 计算每个点的曲率
void LidarPreprocAiry::cal_smoothness(){
    int cloud_size = cloud_dense_->points.size();

#pragma omp parallel for num_threads(MP_PROC_NUM)
    for (int i = 5; i < cloud_size - 5; i++)    {   
        // 如果是球面，则当前点周围的10个点的距离之和　减去　当前点距离的10倍　应该等于0
        float diff_range = pnt_range_[i-5] + pnt_range_[i-4]
                         + pnt_range_[i-3] + pnt_range_[i-2]
                         + pnt_range_[i-1] - pnt_range_[i] * 10
                         + pnt_range_[i+1] + pnt_range_[i+2]
                         + pnt_range_[i+3] + pnt_range_[i+4]
                         + pnt_range_[i+5];            

        cloud_curvature_[i] = diff_range*diff_range;//diffX * diffX + diffY * diffY + diffZ * diffZ;

        cloud_neighbor_picked_[i] = 0;
        cloud_label_[i] = 0; // 初始化为0， 角点为1, 面点为-1
        // cloud_smoothness_ for sorting
        cloud_smoothness_[i].value = cloud_curvature_[i];
        cloud_smoothness_[i].idx = i;
    }

}

// 标记遮挡和平行点
void LidarPreprocAiry::mark_occluded_points(){
    int cloud_size = cloud_dense_->points.size();
    // mark occluded points and parallel beam points
#pragma omp parallel for num_threads(MP_PROC_NUM)
    for (int i = 5; i < cloud_size - 6; ++i){
        // occluded points 遮蔽点
        float depth1 = pnt_range_[i];
        float depth2 = pnt_range_[i+1];
        // 列索引间的距离
        int column_diff = std::abs(int(pnt_col_idx_[i+1] - pnt_col_idx_[i]));
        
        // 相邻两点如果列索引太小，则这个点周围的点不进行特征提取
        // 平行线和遮挡的判断参考LOAM
        // cloud_neighbor_picked_： 如果标记为1，表示后续不做特征点提取
        if (column_diff < 10){
            // 10 pixel diff in range image
            if (depth1 - depth2 > 0.3){
                cloud_neighbor_picked_[i - 5] = 1;
                cloud_neighbor_picked_[i - 4] = 1;
                cloud_neighbor_picked_[i - 3] = 1;
                cloud_neighbor_picked_[i - 2] = 1;
                cloud_neighbor_picked_[i - 1] = 1;
                cloud_neighbor_picked_[i] = 1;
            }else if (depth2 - depth1 > 0.3){
                cloud_neighbor_picked_[i + 1] = 1;
                cloud_neighbor_picked_[i + 2] = 1;
                cloud_neighbor_picked_[i + 3] = 1;
                cloud_neighbor_picked_[i + 4] = 1;
                cloud_neighbor_picked_[i + 5] = 1;
                cloud_neighbor_picked_[i + 6] = 1;
            }
        }
        // parallel beam
        // 平行线的情况,根据左右两点与该点的深度差,确定该点是否会被选择为特征点
        float diff1 = std::abs(float(pnt_range_[i-1] - pnt_range_[i]));
        float diff2 = std::abs(float(pnt_range_[i+1] - pnt_range_[i]));

        if (diff1 > 0.02 * pnt_range_[i] && diff2 > 0.02 * pnt_range_[i]){ /////////////////////////////////////////////// 这里有参数
            cloud_neighbor_picked_[i] = 1;
        }
    }
}

// 提取surface和corner特征
void LidarPreprocAiry::extract_feature_points(){
    cloud_corner_->clear();
    cloud_surface_->clear();

    pcl::PointCloud<PointType>::Ptr temp_surface_cloud(new pcl::PointCloud<PointType>());
    // 当新的激光雷达扫描到达时，我们首先执行特征提取。 通过评估局部区域上点的粗糙度来提取边缘和平面特征。 粗糙度值大的点被分类为边缘特征。
    pcl::PointCloud<PointType>::Ptr temp_surface_cloud_DS(new pcl::PointCloud<PointType>());
    

    for (int i = 0; i < ring_cnt_; i++) { // 每根线 ring   
        temp_surface_cloud->clear();

        // 每根线分成6部分   
        for (int j = 0; j < 6; j++){
            // 从startRingIndex到endRingIndex,分成6分
            // 第一份的索引就是 startRingIndex* (6-j)/6 + endRingInde * j/6
            int sp = (ring_index_start_[i] * (6 - j) + ring_index_end_[i] * j) / 6;
            // ep 就是sp的下一个循环的值的前一个索引，ep[j] = sp[j+1] - 1
            int ep = (ring_index_start_[i] * (5 - j) + ring_index_end_[i] * (j + 1)) / 6 - 1;

            if (sp >= ep)
                continue;

            // 将这段点云按照曲率从小到大进行排序
            std::sort(cloud_smoothness_.begin()+sp, cloud_smoothness_.begin()+ep, by_value());

            int largest_picked_num = 0;
            // 从后往前进行遍历, 进行【角点】的提取与保存
            for (int k = ep; k >= sp; k--){   
                // 最后的点 的曲率最大，如果满足条件，就是角点
                // edgeThreshold为0.1(? 1.0吧)，正圆的曲率为0
                int ind = cloud_smoothness_[k].idx;
                if (cloud_neighbor_picked_[ind] == 0 && cloud_curvature_[ind] > edge_curv_thr_){   ////////////////////// 这里有参数， 参数值有待确认
                    // 每一段最多只取20个角点
                    largest_picked_num++;
                    if (largest_picked_num <= 20){ ///////////////////////////////////////////////////////////////////// 这里有参数
                        cloud_label_[ind] = 1;
                        cloud_corner_->points.push_back(cloud_dense_->points[ind]);
                        cloud_corner_->points.back().normal_x = 1;
                    } else {
                        break;
                    }

                    // 防止特征点聚集，将ind及其前后各5个点标记，不做特征点提取
                    cloud_neighbor_picked_[ind] = 1;
                    for (int l = 1; l <= 5; l++){
                        int column_diff = std::abs(int(pnt_col_idx_[ind + l] - pnt_col_idx_[ind + l - 1]));
                        if (column_diff > 10)
                            break;
                        cloud_neighbor_picked_[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--){   
                        // 每个点index之间的差值。附近点都是有效点的情况下，相邻点间的索引只差１
                        int column_diff = std::abs(int(pnt_col_idx_[ind + l] - pnt_col_idx_[ind + l + 1]));
                        // 附近有无效点，或者是每条线的起点和终点的部分
                        if (column_diff > 10)
                            break;
                        cloud_neighbor_picked_[ind + l] = 1;
                    }
                }
            }
            
            // 进行面点的提取
            // for (int k = sp; k <= ep; k++) {
            //     int ind = cloud_smoothness_[k].idx;
            //     if (cloud_neighbor_picked_[ind] == 0 && cloud_curvature_[ind] < surf_curv_thr_) {
            //         cloud_label_[ind] = -1; // 标记为-1,表示面点
            //         temp_surface_cloud->points.push_back(cloud_dense_->points[ind]);
            //         // 这个点及前后各5个点不再进行提取特征，防止平面点聚集
            //         cloud_neighbor_picked_[ind] = 1;

            //         for (int l = 1; l <= 5; l++) {
            //             int column_diff = std::abs(int(pnt_col_idx_[ind + l] - pnt_col_idx_[ind + l - 1]));
            //             if (column_diff > 10)
            //                 break;
            //             cloud_neighbor_picked_[ind + l] = 1;
            //         }
            //         for (int l = -1; l >= -5; l--) {
            //             int column_diff = std::abs(int(pnt_col_idx_[ind + l] - pnt_col_idx_[ind + l + 1]));
            //             if (column_diff > 10)
            //                 break;
            //             cloud_neighbor_picked_[ind + l] = 1;
            //         }
            //     }
            // }
            // temp_面点临时保存在surface_cloud
            for (int k = sp; k <= ep; k++) {
                if (cloud_label_[k] <= 0){
                    temp_surface_cloud->points.push_back(cloud_dense_->points[k]);
                }
            }
        }
        // // 对面点进行降采样，结果临时保存在 temp_surface_cloud_DS
        temp_surface_cloud_DS->clear();
        downsize_filter_.setInputCloud(temp_surface_cloud);
        downsize_filter_.filter(*temp_surface_cloud_DS);
        // // 将临时变量中的值放入surfaceCloud
        *cloud_surface_ += *temp_surface_cloud_DS;
        // *cloud_surface_ += *temp_surface_cloud;
    }
}


void LidarPreprocAiry::allocate_memory_init_variable(){

    cloud_dense_.reset(new PointCloudType());
    cloud_dense_->points.reserve(ring_cnt_*col_cnt_);

    cloud_corner_.reset(new PointCloudType());
    cloud_surface_.reset(new PointCloudType());
    cloud_corner_->points.reserve(ring_cnt_*col_cnt_);
    cloud_surface_->points.reserve(ring_cnt_*col_cnt_);

    downsize_filter_.setLeafSize(surf_leafsize_, surf_leafsize_, surf_leafsize_);

    ring_index_start_.assign(ring_cnt_, 0);
    ring_index_end_.assign(ring_cnt_, 0);
    pnt_col_idx_.assign(ring_cnt_*col_cnt_, 0);
    pnt_range_.assign(ring_cnt_*col_cnt_, 0);

    cloud_smoothness_.resize(ring_cnt_*col_cnt_);
    cloud_curvature_ = new float[ring_cnt_*col_cnt_];

    cloud_neighbor_picked_ = new int[ring_cnt_*col_cnt_];
    cloud_label_ = new int[ring_cnt_*col_cnt_];

}


bool LidarPreprocAiry::set_param(){

    LocalizationModuleParamManager *param_manager = LocalizationModuleParamManager::Instance();
    const lidar_slam::LidarSlamParam* loaded_param = param_manager->get_loaded_param();

    if (loaded_param == NULL) {
        ROS_ERROR_STREAM(RED << "loaded_param is NULL" << RESET);
        return false;
    }else{
        param_ = loaded_param->lidar_preproc;

        // thr_region_x_ = loaded_param->lidar_preproc.point_filter_distance[0];
        // thr_region_y_ = loaded_param->lidar_preproc.point_filter_distance[1];
        // thr_region_z_ = loaded_param->lidar_preproc.point_filter_distance[2];

        blind_range_square_ = loaded_param->lidar_preproc.blind_distance * loaded_param->lidar_preproc.blind_distance;
        max_range_square_ = loaded_param->lidar_preproc.max_distance * loaded_param->lidar_preproc.max_distance;
        point_filter_num_ = loaded_param->lidar_preproc.point_filter_num;
        ring_filter_num_ = loaded_param->lidar_preproc.ring_filter_num;
        cloud_size_to_keep_ = loaded_param->lidar_preproc.cloud_size_to_keep;

        ////////////////////////////////////////////////////////////////////////////////
        // extract cloud by ring_feature
        extract_cloud_method_ = loaded_param->lidar_preproc.extract_cloud_method;
        col_cnt_ = loaded_param->lidar_preproc.cloud_column_count;
        ring_cnt_ = loaded_param->lidar_preproc.cloud_ring_count;
        edge_curv_thr_ = loaded_param->lidar_preproc.edge_curvature_thr;
        surf_curv_thr_ = loaded_param->lidar_preproc.surf_curvature_thr;
        surf_leafsize_ = loaded_param->lidar_preproc.surf_leafsize;


        // ROS_ERROR("thr_region_x_ : %lf", thr_region_x_);
        // ROS_ERROR("thr_region_y_ : %lf", thr_region_y_);
        // ROS_ERROR("blind_range_square_ : %lf", blind_range_square_);
        // ROS_ERROR("point_filter_num_ : %d", point_filter_num_);

        return true;
    }
}


} // namespace localization_module
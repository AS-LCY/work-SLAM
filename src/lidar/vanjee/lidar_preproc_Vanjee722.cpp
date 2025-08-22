#include "lidar/vanjee/lidar_preproc_Vanjee722.h"


namespace localization_module {

LidarPreprocVanjee722::LidarPreprocVanjee722(rclcpp::Node::SharedPtr node): LidarPreprocParent(node){
    if(!set_param(node)){
        // ROS_ERROR_STREAM(RED << "Set lidar param failed!" << RESET);
    }else {
        // ROS_INFO("Set lidar-Vanjee722 param successfully!");
    }

    allocate_memory_init_variable();
    // ROS_INFO("Reset to lidar_preproc_Vanjee722 successfully!");
}


LidarPreprocVanjee722::~LidarPreprocVanjee722(){

}


bool LidarPreprocVanjee722::pre_process(const sensor_msgs::msg::PointCloud2::SharedPtr ros_msg_in, PointCloudType::Ptr& pcl_xyzin_out){
    if(extract_cloud_method_ == 0){
        cloud_dense_->clear();
        msg2pcl_clip(ros_msg_in, cloud_dense_);
    
        pcl_xyzin_out->clear();
        sampling_cloud(cloud_dense_, pcl_xyzin_out);
    }else if (extract_cloud_method_ == 3){

    }else{
        return false;
    }
    
    
    return true;
}


// current used
bool LidarPreprocVanjee722::msg2pcl_clip(const sensor_msgs::msg::PointCloud2::SharedPtr ros_msg_in, PointCloudType::Ptr pcl_xyzin_out){

    // ROS_INFO_ONCE("Vanjee: ros_msg_in --> pcl_xyzin_out");

    int cloud_num = ros_msg_in->height * ros_msg_in->width;
    // double header_time = ros_msg_in->header.stamp.toSec();
    double header_time = rclcpp::Time(ros_msg_in->header.stamp).seconds();

    ///// MetaData --- header 
    pcl::PCLHeader pcl_header;
    // pcl_header.seq = ros_msg_in->header.seq;
    // pcl_header.stamp = ros_msg_in->header.stamp.toNSec() / 1000ull;
    pcl_header.stamp = rclcpp::Time(ros_msg_in->header.stamp).nanoseconds() / 1000ull;
    pcl_header.frame_id = ros_msg_in->header.frame_id;
    ///// MetaData --- field
    std::vector<pcl::PCLPointField> pcl_fields;
    
    pcl_fields.resize(ros_msg_in->fields.size());
    std::vector<sensor_msgs::msg::PointField>::const_iterator it = ros_msg_in->fields.begin();
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
        for (std::uint32_t w = 0; w < ros_msg_in->width; ++w){
            const std::uint8_t* msg_data = h_data + w * ros_msg_in->point_step;
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
            if(range_square < blind_range_square_ || range_square > max_range_square_ || curpt->z < z_range_[0] || curpt->z > z_range_[1]){
                continue;
            }

            // // if(row % point_filter_num_ == 0){
            // if(col % point_filter_num_ == 0 && row % ring_filter_num_ == 0){
            //     PointType xyzin_point;
            //     xyzin_point.x = curpt->x;
            //     xyzin_point.y = curpt->y;
            //     xyzin_point.z = curpt->z;
            //     xyzin_point.intensity = curpt->intensity;
                
            //     // xyzin_point.curvature = (curpt->timestamp - header_time) * 1000; // offset, unit = ms
            //     // xyzin_point.curvature = (curpt->timestamp ) * 1000; // offset, unit = ms
            //     xyzin_point.curvature = curpt->timestamp ; // offset, unit = second
            //     pcl_xyzin_out->points.push_back(xyzin_point);
            // }
            PointType xyzin_point;
            xyzin_point.x = curpt->x;
            xyzin_point.y = curpt->y;
            xyzin_point.z = curpt->z;
            xyzin_point.intensity = curpt->intensity;
            
            // xyzin_point.curvature = (curpt->timestamp - header_time) * 1000; // offset, unit = ms
            // xyzin_point.curvature = (curpt->timestamp ) * 1000; // offset, unit = ms
            xyzin_point.curvature = curpt->timestamp ; // offset, unit = second
            pcl_xyzin_out->points.push_back(xyzin_point);

        }
    }
    pcl_xyzin_out->points.shrink_to_fit();
    ///// Copy info fields
    pcl_xyzin_out->header   = pcl_header;
    pcl_xyzin_out->width    = pcl_xyzin_out->points.size();
    pcl_xyzin_out->height   = 1;
    pcl_xyzin_out->is_dense = 1;

    return true;
}

// ///////////////// 入口函数 /////////////////                  
// bool LidarPreprocVanjee722::pre_process(const pcl::PointCloud<RsPointXYZIRT>::Ptr pcl_rs_in, PointCloudType::Ptr pcl_xyzin_out){

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
// void LidarPreprocVanjee722::extract_cloud_by_interval_sampling(const pcl::PointCloud<RsPointXYZIRT>::Ptr pcl_rs_in, PointCloudType::Ptr pcl_xyzin_out){

// }

void LidarPreprocVanjee722::allocate_memory_init_variable(){
    cloud_dense_.reset(new PointCloudType());
    cloud_dense_->points.reserve(ring_cnt_*col_cnt_);

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



bool LidarPreprocVanjee722::set_param(rclcpp::Node::SharedPtr node){

    LocalizationModuleParamManager *param_manager = LocalizationModuleParamManager::Instance(node);
    // const lidar_slam::LidarSlamParam* loaded_param = param_manager->get_loaded_param();
    // std::shared_ptr<const lidar_slam::LidarSlamParam> loaded_param = param_manager->get_loaded_param();
    const lidar_slam::LidarSlamParam& loaded_param = param_manager->get_loaded_param();
    // if (loaded_param == NULL) {
    //     // ROS_ERROR_STREAM(RED << "loaded_param is NULL" << RESET);
    //     return false;
    // }else{
        // param_ = loaded_param->lidar_preproc;

        // thr_region_x_ = loaded_param->lidar_preproc.point_filter_distance[0];
        // thr_region_y_ = loaded_param->lidar_preproc.point_filter_distance[1];
        // thr_region_z_ = loaded_param->lidar_preproc.point_filter_distance[2];

        // blind_range_square_ = loaded_param->lidar_preproc.blind_distance * loaded_param->lidar_preproc.blind_distance;
        // max_range_square_ = loaded_param->lidar_preproc.max_distance * loaded_param->lidar_preproc.max_distance;
        // point_filter_num_ = loaded_param->lidar_preproc.point_filter_num;
        // ring_filter_num_ = loaded_param->lidar_preproc.ring_filter_num;
        // cloud_size_to_keep_ = loaded_param->lidar_preproc.cloud_size_to_keep;

        ////////////////////////////////////////////////////////////////////////////////
        // extract cloud by ring_feature
        extract_cloud_method_ = loaded_param.lidar_preproc.extract_cloud_method;
        col_cnt_ = loaded_param.lidar_preproc.cloud_column_count;
        ring_cnt_ = loaded_param.lidar_preproc.cloud_ring_count;
        edge_curv_thr_ = loaded_param.lidar_preproc.edge_curvature_thr;
        surf_curv_thr_ = loaded_param.lidar_preproc.surf_curvature_thr;
        surf_leafsize_ = loaded_param.lidar_preproc.surf_leafsize;
        ////////////////////////////////////////////////////////////////////////////////


        // ROS_ERROR("thr_region_x_ : %lf", thr_region_x_);
        // ROS_ERROR("thr_region_y_ : %lf", thr_region_y_);
        // ROS_ERROR("blind_range_square_ : %lf", blind_range_square_);
        // ROS_ERROR("point_filter_num_ : %d", point_filter_num_);

        return true;
    // }
}

} // namespace localization_module
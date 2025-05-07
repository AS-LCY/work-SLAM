#include "lidar/livox/lidar_preproc_Mid360.h"


namespace localization_module {

LidarPreprocMid360::LidarPreprocMid360(){
    if(!set_param()){
        ROS_ERROR_STREAM(RED << "Set lidar param failed!" << RESET);
    }else {
        ROS_INFO("Set lidar-Mid360 param successfully!");
    }

    ROS_INFO("Reset to LidarPreproc-Mid360 successfully!");

}


LidarPreprocMid360::~LidarPreprocMid360(){
    
}

bool LidarPreprocMid360::set_param(){
    LocalizationModuleParamManager *param_manager = LocalizationModuleParamManager::Instance();
    const lidar_slam::LidarSlamParam* loaded_param = param_manager->get_loaded_param();

    if (loaded_param == NULL) {
        ROS_ERROR_STREAM(RED << "loaded_param is NULL" <<RESET);
        return false;
    }else{
        param_ = loaded_param->lidar_preproc;

        // thr_region_x_ = loaded_param->lidar_preproc.point_filter_distance[0];
        // thr_region_y_ = loaded_param->lidar_preproc.point_filter_distance[1];
        // thr_region_z_ = loaded_param->lidar_preproc.point_filter_distance[2];

        blind_range_square_ = loaded_param->lidar_preproc.blind_distance * loaded_param->lidar_preproc.blind_distance;
        max_range_square_ = loaded_param->lidar_preproc.max_distance * loaded_param->lidar_preproc.max_distance;
        // obstacle_square_ = loaded_param->lidar_preproc.obstacle_max_range * loaded_param->lidar_preproc.obstacle_max_range;
        point_filter_num_ = loaded_param->lidar_preproc.point_filter_num;

        cloud_size_to_keep_ = loaded_param->lidar_preproc.cloud_size_to_keep;
        return true;
    }
}

bool LidarPreprocMid360::msg2pcl_clip(const sensor_msgs::PointCloud2::ConstPtr ros_msg_in, PointCloudType::Ptr pcl_xyzin_out){

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
    }
    //// create Mapping
    pcl::MsgFieldMap field_map;
    pcl::createMapping<LvxPointXYZITLO> (pcl_fields, field_map);

    ///////////////////////////////////////////////////////////////////////////////////////////
    /// fill lvx_msg_out
    // lvx_msg_out->time_stamp = ros_msg_in->header.stamp.toSec();

    pcl_xyzin_out->points.reserve(cloud_num);
    for (std::uint32_t row = 0; row < ros_msg_in->height; ++row){
        const std::uint8_t* row_data = &ros_msg_in->data[row * ros_msg_in->row_step];
        for (std::uint32_t col = 0; col < ros_msg_in->width; ++col){
            const std::uint8_t* msg_data = row_data + col * ros_msg_in->point_step;
            LvxPointXYZITLO temp_point;
            LvxPointXYZITLO* curpt = &temp_point;
            std::uint8_t* curpt_data = reinterpret_cast<std::uint8_t*>(curpt);

            for (const pcl::detail::FieldMapping& mapping : field_map){
                memcpy (curpt_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
            }

            // if(abs(curpt->x) > thr_region_x_ || abs(curpt->y) > thr_region_y_ || curpt->z > thr_region_z_){
            //    continue;
            // }

            if ((curpt->tag & 0x30) == 0x20 || (curpt->tag & 0x30) == 0x30
                || (curpt->tag & 0x03) == 0x02 || (curpt->tag & 0x03) == 0x03){
                continue;
            }
            double range_square = curpt->x * curpt->x + curpt->y * curpt->y + curpt->z * curpt->z;
            if(range_square < blind_range_square_ || range_square > max_range_square_){
                continue;
            }

            PointType xyzin_point;
            xyzin_point.x = curpt->x;
            xyzin_point.y = curpt->y;
            xyzin_point.z = curpt->z;
            xyzin_point.intensity = curpt->intensity;
            // 新版驱动的 pointcloud2 中， timestamp 为完整时间辍，但单位是纳秒，需要 * 1e-9，将单位统一为 秒       
            // xyzin_point.curvature = (curpt->timestamp * 1e-9 - header_time) * 1000; // offset, unit = ms
            xyzin_point.curvature = curpt->timestamp * 1e-9 - header_time; // offset, unit = second
            // ROS_INFO_STREAM("single point time: " << xyzin_point.curvature << " ms");
            
            pcl_xyzin_out->points.push_back(xyzin_point);

            // // livox_point.offset_time = curpt->offset_time;
            // // 新版驱动的 pointcloud2 中， timestamp 为完整时间辍，但单位是纳秒，需要 * 1e-9，将单位统一为 秒
            // // livox_point.offset_time = (curpt->timestamp / double(1000000000.0) - msg->time_stamp);
            // livox_point.offset_time = (curpt->timestamp  * 1e-9 - lvx_msg_out->time_stamp);
            // lvx_msg_out->points.push_back(livox_point);
        }
    }
    pcl_xyzin_out->points.shrink_to_fit();

    pcl_xyzin_out->header   = pcl_header;
    pcl_xyzin_out->width    = pcl_xyzin_out->points.size();
    pcl_xyzin_out->height   = 1;
    pcl_xyzin_out->is_dense = 1;

    return true;

}

// //// 弃用
// bool LidarPreprocMid360::msg2pcl_clip(const sensor_msgs::PointCloud2::ConstPtr &ros_msg_in, std::shared_ptr<livox_ros::LidarMsg> &lvx_msg_out){

//     int cloud_num = ros_msg_in->height * ros_msg_in->width;

//     ///// MetaData --- header 
//     pcl::PCLHeader pcl_header;
//     pcl_header.seq = ros_msg_in->header.seq;
//     pcl_header.stamp = ros_msg_in->header.stamp.toNSec() / 1000ull;
//     pcl_header.frame_id = ros_msg_in->header.frame_id;
//     ///// MetaData --- field
//     std::vector<pcl::PCLPointField> pcl_fields;
    
//     pcl_fields.resize(ros_msg_in->fields.size());
//     std::vector<sensor_msgs::PointField>::const_iterator it = ros_msg_in->fields.begin();
//     int i = 0;
//     for(; it != ros_msg_in->fields.end(); ++it, ++i) {
//       pcl_fields[i].name = it->name;
//       pcl_fields[i].offset = it->offset;
//       pcl_fields[i].datatype = it->datatype;
//       pcl_fields[i].count = it->count;
//     }
//     //// create Mapping
//     pcl::MsgFieldMap field_map;
//     pcl::createMapping<LvxPointXYZITLO> (pcl_fields, field_map);

//     ///////////////////////////////////////////////////////////////////////////////////////////
//     /// fill lvx_msg_out
//     lvx_msg_out->time_stamp = ros_msg_in->header.stamp.toSec();

//     for (std::uint32_t row = 0; row < ros_msg_in->height; ++row){
//         const std::uint8_t* row_data = &ros_msg_in->data[row * ros_msg_in->row_step];
//         for (std::uint32_t col = 0; col < ros_msg_in->width; ++col){
//             const std::uint8_t* msg_data = row_data + col * ros_msg_in->point_step;
//             LvxPointXYZITLO temp_point;
//             LvxPointXYZITLO* curpt = &temp_point;
//             std::uint8_t* curpt_data = reinterpret_cast<std::uint8_t*>(curpt);

//             for (const pcl::detail::FieldMapping& mapping : field_map){
//                 memcpy (curpt_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
//             }

//             livox_ros::LidarPoint livox_point;
//             livox_point.x = curpt->x;
//             livox_point.y = curpt->y;
//             livox_point.z = curpt->z;
//             livox_point.reflectivity = curpt->intensity;
//             livox_point.tag = curpt->tag;
//             livox_point.line = curpt->line;
//             if(abs(livox_point.x) > thr_region_x_ || abs(livox_point.y) > thr_region_y_ || livox_point.z > thr_region_z_){
//                continue;
//             }

//             if ((livox_point.tag & 0x30) == 0x20 || (livox_point.tag & 0x30) == 0x30
//                 || (livox_point.tag & 0x03) == 0x02 || (livox_point.tag & 0x03) == 0x03){
//                 continue;
//             }
//             double range = livox_point.x * livox_point.x + livox_point.y * livox_point.y + livox_point.z * livox_point.z;
//             if (range < blind_range_square_){
//                 continue;
//             }

//             // livox_point.offset_time = curpt->offset_time;
//             // 新版驱动的 pointcloud2 中， timestamp 为完整时间辍，但单位是纳秒，需要 * 1e-9，将单位统一为 秒
//             // livox_point.offset_time = (curpt->timestamp / double(1000000000.0) - msg->time_stamp);
//             livox_point.offset_time = (curpt->timestamp  * 1e-9 - lvx_msg_out->time_stamp);
//             lvx_msg_out->points.push_back(livox_point);
//         }
//     }
//     lvx_msg_out->point_num = lvx_msg_out->points.size();

//     return true;

// }


// ///////////////// 入口函数 /////////////////
// bool LidarPreprocMid360::pre_process(const std::shared_ptr<livox_ros::LidarMsg> msg, PointCloudType::Ptr pcl_cld_out){
//     const int extract_cloud_method = param_.extract_cloud_method;
//     int plsize = msg->point_num;

//     pcl_cld_out->clear();
//     pcl_cld_out->reserve(plsize);


//     if (extract_cloud_method == 0){
//         extract_cloud_by_interval_sampling(msg, pcl_cld_out);
//     }else if(extract_cloud_method == 1){
//         extract_cloud_by_simple_voxel(msg, pcl_cld_out);
//     }else if(extract_cloud_method == 2){
//         extract_cloud_by_interval_and_voxel(msg, pcl_cld_out);
//     }else if(extract_cloud_method == 3){
//         extract_cloud_by_feature(msg, pcl_cld_out);
//     } else{
//         // printf("extract_cloud_method set error!\n");
//         ROS_ERROR_STREAM(RED << "extract_cloud_method set error!" << RESET);
//         exit(1);
//     }
    
//     // printf("extract lidar count: %ld\n", pcl_cld_out->points.size());
//     ROS_INFO("extract lidar count: %ld", pcl_cld_out->points.size());
//     return true;
    
// }

//////////////////////////////////////////////////////////////////////////////////////////////////
// 间隔采样
void LidarPreprocMid360::extract_cloud_by_interval_sampling(const std::shared_ptr<livox_ros::LidarMsg> msg, PointCloudType::Ptr pcl_cld_out){
    // std::cout<<"extract cloud by method:  interval sampling"<<std::endl;
    int plsize = msg->point_num;
    uint valid_num = 0;
    for (uint i = 1; i < plsize; i++){//zd delete (msg->points[i].line < N_SCANS)
        if (((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00) 
            && ((msg->points[i].tag & 0x03) == 0x01 || (msg->points[i].tag & 0x03) == 0x00)){
            valid_num++;
            double range_square = msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y + msg->points[i].z * msg->points[i].z;
            // if (range < obstacle_square_ && range>blind_range_square_){
            //     PointType point;
            //     point.x = msg->points[i].x;
            //     point.y = msg->points[i].y;
            //     point.z = msg->points[i].z;            
            //     pl_obstacle->points.push_back(point);
            // }
                
            if (valid_num % point_filter_num_ == 0)
            {
                PointType temp_pt;
                temp_pt.x = 0;
                temp_pt.y = 0;
                temp_pt.z = 0;
                PointType curr_pt;
                curr_pt.x = msg->points[i].x;
                curr_pt.y = msg->points[i].y;
                curr_pt.z = msg->points[i].z;
                curr_pt.intensity = msg->points[i].reflectivity;
                curr_pt.curvature = msg->points[i].offset_time * float(1000); // use curvature as time of each laser points, curvature unit: ms

                // pl_full[i].x = msg->points[i].x;
                // pl_full[i].y = msg->points[i].y;
                // pl_full[i].z = msg->points[i].z;
                // pl_full[i].intensity = msg->points[i].reflectivity;
                // // pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms
                // pl_full[i].curvature = msg->points[i].offset_time * float(1000); // use curvature as time of each laser points, curvature unit: ms
                // if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7))
                // if ((abs(pl_full[i].x - pl_full[i - 1].x) > 0.15) || (abs(pl_full[i].y - pl_full[i - 1].y) > 0.15) || (abs(pl_full[i].z - pl_full[i - 1].z) > 0.15))
                
                static PointType last_pt = temp_pt;
                if ((abs(curr_pt.x - last_pt.x) > 0.15) || (abs(curr_pt.y - last_pt.y) > 0.15) || (abs(curr_pt.z - last_pt.z) > 0.15))
                {
                    if (range_square > blind_range_square_ && range_square < max_range_square_){
                        // pcl_cld_out->push_back(pl_full[i]);
                        pcl_cld_out->push_back(curr_pt);
                    }
                    last_pt = curr_pt;
                }//if
            }//if
        }//if
    }//for
}



//////////////////////////////////////////////////////////////////////////////////////////////////

void LidarPreprocMid360::extract_cloud_by_interval_and_voxel(const std::shared_ptr<livox_ros::LidarMsg> msg, PointCloudType::Ptr pcl_cld_out){
    // std::cout<<"extract cloud by method: interval and voxel"<<std::endl;
    const double leafsize = param_.leafsize;
    const double extent_xmin = param_.voxel_region_xyz[0];
    const double extent_xmax = param_.voxel_region_xyz[1];
    const double extent_ymin = param_.voxel_region_xyz[2];
    const double extent_ymax = param_.voxel_region_xyz[3];
    const double extent_zmin = param_.voxel_region_xyz[4];
    const double extent_zmax = param_.voxel_region_xyz[5];
    // std::cout<<"leafsize: "<<leafsize<<endl;
    ROS_INFO_STREAM("leafsize: "<<leafsize);

    int plsize = msg->point_num;
    uint valid_num = 0;

    double extent_leafsize_inv = 1.0/leafsize;

    int Xcnt_region = (extent_xmax - extent_xmin)*extent_leafsize_inv  +1; 
    int Ycnt_region = (extent_ymax - extent_ymin)*extent_leafsize_inv  +1; 
    int Zcnt_region = (extent_zmax - extent_zmin)*extent_leafsize_inv  +1; 
    int vect_size = Xcnt_region * Ycnt_region * Zcnt_region;

    unsigned char *flag_if_fill=(unsigned char*)calloc(vect_size,sizeof(unsigned char));
    int Xindex=0, Yindex=0, Zindex=0;


    // std::cout<<"interval and voxel - 1"<<std::endl;
    for (uint i = 1; i < plsize; i++)
    {//zd delete (msg->points[i].line < N_SCANS)
        if (((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00) 
            && ((msg->points[i].tag & 0x03) == 0x01 || (msg->points[i].tag & 0x03) == 0x00))
        {
            valid_num++;
            double range_square = msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y + msg->points[i].z * msg->points[i].z;
            // if (range_square < obstacle_square_ && range_square>blind_range_square_){
            //     PointType point;
            //     point.x = msg->points[i].x;
            //     point.y = msg->points[i].y;
            //     point.z = msg->points[i].z;            
            //     pl_obstacle->points.push_back(point);
            // }
                
            if (valid_num % point_filter_num_ == 0)
            {
                PointType curr_pt;
                curr_pt.x = msg->points[i].x;
                curr_pt.y = msg->points[i].y;
                curr_pt.z = msg->points[i].z;
                curr_pt.intensity = msg->points[i].reflectivity;
                curr_pt.curvature = msg->points[i].offset_time * float(1000); // use curvature as time of each laser points, curvature unit: ms

                // pl_full[i].x = msg->points[i].x;
                // pl_full[i].y = msg->points[i].y;
                // pl_full[i].z = msg->points[i].z;
                // pl_full[i].intensity = msg->points[i].reflectivity;
                // // pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms
                // pl_full[i].curvature = msg->points[i].offset_time * float(1000); // use curvature as time of each laser points, curvature unit: ms
                // // std::cout << "pl_full[i].curvature: " << pl_full[i].curvature << std::endl;


                Xindex = int((curr_pt.x - extent_xmin) *extent_leafsize_inv);
                Yindex = int((curr_pt.y - extent_ymin) *extent_leafsize_inv);
                Zindex = int((curr_pt.z - extent_zmin) *extent_leafsize_inv);
                size_t index = Ycnt_region * Zcnt_region * Xindex + Zcnt_region * Yindex + Zindex;

                if (range_square > blind_range_square_ && range_square < max_range_square_){
                    if(index>=0 && index<vect_size){
                        if(!flag_if_fill[index]){
                            pcl_cld_out->push_back(curr_pt);
                            flag_if_fill[index] = 1;
                        }
                    }else{
                        pcl_cld_out->push_back(curr_pt);
                    }
                }

                // // if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7))
                // if ((abs(pl_full[i].x - pl_full[i - 1].x) > 0.15) || (abs(pl_full[i].y - pl_full[i - 1].y) > 0.15) || (abs(pl_full[i].z - pl_full[i - 1].z) > 0.15))
                // {
                //     if (range > blind_range_square_ && range < max_range_square_){
                //         pcl_cld_out->push_back(pl_full[i]);
                //     }
                // }//if
            }//if
        }//if
    }//for
    
    // std::cout<<"interval and voxel - 2"<<std::endl;
    free(flag_if_fill);

}


void LidarPreprocMid360::extract_cloud_by_simple_voxel(const std::shared_ptr<livox_ros::LidarMsg> msg, PointCloudType::Ptr pcl_cld_out){
    // std::cout<<"extract cloud by method: simple voxel"<<std::endl;
    const std::vector<double> leafsize = param_.leafsize_vec;
    const double extent_xmin = param_.voxel_region_xyz[0];
    const double extent_xmax = param_.voxel_region_xyz[1];
    const double extent_ymin = param_.voxel_region_xyz[2];
    const double extent_ymax = param_.voxel_region_xyz[3];
    const double extent_zmin = param_.voxel_region_xyz[4];
    const double extent_zmax = param_.voxel_region_xyz[5];
    const double boundary_z = param_.boundary_z;

    std::vector<double> region_zmin_vec, region_zmax_vec;
    region_zmin_vec.push_back(extent_zmin);
    region_zmin_vec.push_back(boundary_z);
    region_zmax_vec.push_back(boundary_z);
    region_zmax_vec.push_back(extent_zmax);

    // double leafsize_inv = 1.0/leafsize;

    int plsize = msg->point_num;

    int devision_cnt = 2 ;
    std::vector<vector<size_t>> divided_clouds_index_vector(devision_cnt, std::vector<size_t>());

    for (size_t i=0; i<plsize; i++){
        auto* cur_pt = &msg->points[i];
        if (!((cur_pt->tag & 0x30) == 0x10 || (cur_pt->tag & 0x30) == 0x00)){
            continue;
        }

        if (cur_pt->x > extent_xmin || cur_pt->x < extent_xmax || 
            cur_pt->y > extent_ymin || cur_pt->y < extent_ymax ||
            cur_pt->z > extent_zmin || cur_pt->z < extent_zmax ){
            double range = cur_pt->x * cur_pt->x + cur_pt->y * cur_pt->y + cur_pt->z * cur_pt->z;

            for (int j=0; j<devision_cnt; j++){
                float region_zmin = region_zmin_vec[j];
                float region_zmax = region_zmax_vec[j];
                if (cur_pt->z >= region_zmin && cur_pt->z < region_zmax){
                    divided_clouds_index_vector.at(j).push_back(i);
                    break;
                }
            }/// in the downsample-region, divide cloud and downsample
        }
    }


    const double region_xmin = extent_xmin;
    const double region_xmax = extent_xmax;
    const double region_ymin = extent_ymin;
    const double region_ymax = extent_ymax;
    double region_zmin = 0.0; // initialize 
    double region_zmax = 0.0;


    for (int i=0; i<devision_cnt; i++){
        std::vector<size_t> cur_region_idx = divided_clouds_index_vector.at(i);
        size_t cur_cloud_sz = cur_region_idx.size();

        region_zmin = region_zmin_vec[i];
        region_zmax = region_zmax_vec[i];

        // double region_leafsize = leafsize/(i+1);
        double region_leafsize_inv = 1.0/leafsize[i];

        int Xcnt_region = (region_xmax - region_xmin)*region_leafsize_inv  +1; 
        int Ycnt_region = (region_ymax - region_ymin)*region_leafsize_inv  +1; 
        int Zcnt_region = (region_zmax - region_zmin)*region_leafsize_inv  +1; 
        int vect_size = Xcnt_region * Ycnt_region * Zcnt_region;

        unsigned char *flag_if_fill=(unsigned char*)calloc(vect_size,sizeof(unsigned char));


        int Xindex=0, Yindex=0, Zindex=0;

        for (size_t j = 0; j < cur_cloud_sz; j++) 
        {
            auto* cur_pt = &msg->points[cur_region_idx[j]];
            Xindex = int((cur_pt->x - region_xmin) *region_leafsize_inv);
            Yindex = int((cur_pt->y - region_ymin) *region_leafsize_inv);
            Zindex = int((cur_pt->z - region_zmin) *region_leafsize_inv);
            size_t index = Ycnt_region * Zcnt_region * Xindex + Zcnt_region * Yindex + Zindex;

            if (!flag_if_fill[index])
            {
                PointType curr_point;
                curr_point.x = cur_pt->x;
                curr_point.y = cur_pt->y;
                curr_point.z = cur_pt->z;
                curr_point.intensity = cur_pt->reflectivity;
                curr_point.curvature = cur_pt->offset_time * float(1000); // use curvature as time of each laser points, curvature unit: ms

                // pl_full[cur_region_idx[j]].x = cur_pt->x;
                // pl_full[cur_region_idx[j]].y = cur_pt->y;
                // pl_full[cur_region_idx[j]].z = cur_pt->z;
                // pl_full[cur_region_idx[j]].intensity = cur_pt->reflectivity;
                // pl_full[cur_region_idx[j]].curvature = cur_pt->offset_time * float(1000); // use curvature as time of each laser points, curvature unit: ms

                double range_square = cur_pt->x * cur_pt->x + cur_pt->y * cur_pt->y + cur_pt->z * cur_pt->z;
                if (range_square > blind_range_square_ && range_square < max_range_square_){
                    pcl_cld_out->push_back(curr_point); 
                    flag_if_fill[index] = 1;   
                }
            }
        }        
        // delete[]flag_if_fill;//释放
	    // flag_if_fill=NULL;
        free(flag_if_fill);
    }

}



//////////////////////////////////////////////////////////////////////////////////////////////////
// 特征点计算
// extract_cloud_by_feature()
// give_feature()
// plane_judge()
// edge_jump_judge()
void LidarPreprocMid360::extract_cloud_by_feature(const std::shared_ptr<livox_ros::LidarMsg> msg, PointCloudType::Ptr pcl_cld_out){

    return;
}


} // namespace localization_module
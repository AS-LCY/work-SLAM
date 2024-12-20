#include "lidar_slam/preprocess.h"


Preprocess::Preprocess()
    : feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1) 
{
  inf_bound = 10;
  N_SCANS = 6;

  group_size = 8;
  disA = 0.01;
  disA = 0.1; // B?
  p2l_ratio = 225;
  limit_maxmid = 6.25;
  limit_midmin = 6.25;
  limit_maxmin = 3.24;
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;
  edgeb = 0.1;
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;
  given_offset_time = false;

  jump_up_limit = cos(jump_up_limit / 180 * M_PI);
  jump_down_limit = cos(jump_down_limit / 180 * M_PI);
  cos160 = cos(cos160 / 180 * M_PI);
  smallp_intersect = cos(smallp_intersect / 180 * M_PI);
  pl_obstacle.reset(new PointCloudXYZI());
}

Preprocess::~Preprocess() {}

// void Preprocess::set(bool feat_en,bool voxel_en, int lid_type, double bld, int pfilt_num,int line,double obstacle)
void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num,int line,double obstacle)
{
  feature_enabled = feat_en;
  // simple_voxel_enabled_ = voxel_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
  N_SCANS = line;
  obstacle_range = obstacle;
}

void Preprocess::set(lidar_slam::LidarPreprocParam param_in){
    feature_enabled = param_in.feature_enabled;
    // simple_voxel_enabled_ = param_in.simple_voxel_enabled;
    lidar_type = param_in.lidar_type;
    blind = param_in.blind_distance;
    point_filter_num = param_in.point_filter_num;
    N_SCANS = param_in.line_count;
    obstacle_range = param_in.obstacle_max_range;

    param_ = param_in;
}


void Preprocess::process(const std::shared_ptr<livox_ros::LidarMsg> msg, PointCloudXYZI::Ptr &pcl_out)
{
    avia_handler(msg);
    *pcl_out = pl_surf;
    // printf("extract lidar count: %ld\n", pcl_out->points.size());
    ROS_INFO("extract lidar count: %ld", pcl_out->points.size());
}


void Preprocess::avia_handler(const std::shared_ptr<livox_ros::LidarMsg> msg)
{
    const int extract_cloud_method = param_.extract_cloud_method;
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();
    pl_obstacle->clear();
    double t1 = omp_get_wtime();
    int plsize = msg->point_num;
    //  cout<<"plsie: "<<plsize<<endl;

    pl_corn.reserve(plsize);
    pl_surf.reserve(plsize);
    pl_full.resize(plsize);

    for (int i = 0; i < N_SCANS; i++)
    {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize);
    }
    // uint valid_num = 0;

    if (extract_cloud_method == 0){
        extract_cloud_by_interval_sampling(msg);
    }else if(extract_cloud_method == 1){
        extract_cloud_by_simple_voxel(msg);
    }else if(extract_cloud_method == 2){
        extract_cloud_by_interval_and_voxel(msg);
    }else if(extract_cloud_method == 3){
        extract_cloud_by_feature(msg);
    } else{
        // printf("extract_cloud_method set error!\n");
        ROS_ERROR("extract_cloud_method set error!");
        exit(1);
    }
    // printf("test %d %d \n",pl_full.size(),pl_obstacle.size());
}



//////////////////////////////////////////////////////////////////////////////////////////////////

void Preprocess::extract_cloud_by_interval_and_voxel(const std::shared_ptr<livox_ros::LidarMsg> msg){
    // std::cout<<"extract cloud by method: interval and voxel"<<std::endl;
    const double leafsize = param_.leafsize;
    const double extent_xmin = param_.voxel_region_xyz[0];
    const double extent_xmax = param_.voxel_region_xyz[1];
    const double extent_ymin = param_.voxel_region_xyz[2];
    const double extent_ymax = param_.voxel_region_xyz[3];
    const double extent_zmin = param_.voxel_region_xyz[4];
    const double extent_zmax = param_.voxel_region_xyz[5];
    const double blind_square = param_.blind_distance * param_.blind_distance;
    const double obstacle_square = obstacle_range * obstacle_range;
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
            if (range_square < obstacle_square && range_square>blind_square){
                PointType point;
                point.x = msg->points[i].x;
                point.y = msg->points[i].y;
                point.z = msg->points[i].z;            
                pl_obstacle->points.push_back(point);
            }
                
            if (valid_num % point_filter_num == 0)
            {
                pl_full[i].x = msg->points[i].x;
                pl_full[i].y = msg->points[i].y;
                pl_full[i].z = msg->points[i].z;
                pl_full[i].intensity = msg->points[i].reflectivity;
                // pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms
                pl_full[i].curvature = msg->points[i].offset_time * float(1000); // use curvature as time of each laser points, curvature unit: ms
                // std::cout << "pl_full[i].curvature: " << pl_full[i].curvature << std::endl;


                Xindex = int((pl_full[i].x - extent_xmin) *extent_leafsize_inv);
                Yindex = int((pl_full[i].y - extent_ymin) *extent_leafsize_inv);
                Zindex = int((pl_full[i].z - extent_zmin) *extent_leafsize_inv);
                size_t index = Ycnt_region * Zcnt_region * Xindex + Zcnt_region * Yindex + Zindex;

                if(range_square > blind_square){
                    if(index>=0 && index<vect_size){
                        if(!flag_if_fill[index]){
                            pl_surf.push_back(pl_full[i]);
                            flag_if_fill[index] = 1;
                        }
                    }else{
                        pl_surf.push_back(pl_full[i]);
                    }
                }

                // // if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7))
                // if ((abs(pl_full[i].x - pl_full[i - 1].x) > 0.15) || (abs(pl_full[i].y - pl_full[i - 1].y) > 0.15) || (abs(pl_full[i].z - pl_full[i - 1].z) > 0.15))
                // {
                //     if (range_square > (blind_square)){
                //         pl_surf.push_back(pl_full[i]);
                //     }
                // }//if
            }//if
        }//if
    }//for
    
    // std::cout<<"interval and voxel - 2"<<std::endl;
    free(flag_if_fill);

}


void Preprocess::extract_cloud_by_simple_voxel(const std::shared_ptr<livox_ros::LidarMsg> msg){
    // std::cout<<"extract cloud by method: simple voxel"<<std::endl;
    const std::vector<double> leafsize = param_.leafsize_vec;
    const double extent_xmin = param_.voxel_region_xyz[0];
    const double extent_xmax = param_.voxel_region_xyz[1];
    const double extent_ymin = param_.voxel_region_xyz[2];
    const double extent_ymax = param_.voxel_region_xyz[3];
    const double extent_zmin = param_.voxel_region_xyz[4];
    const double extent_zmax = param_.voxel_region_xyz[5];
    const double blind_square = param_.blind_distance * param_.blind_distance;
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
                pl_full[cur_region_idx[j]].x = cur_pt->x;
                pl_full[cur_region_idx[j]].y = cur_pt->y;
                pl_full[cur_region_idx[j]].z = cur_pt->z;
                pl_full[cur_region_idx[j]].intensity = cur_pt->reflectivity;
                pl_full[cur_region_idx[j]].curvature = cur_pt->offset_time * float(1000); // use curvature as time of each laser points, curvature unit: ms

                double range = cur_pt->x * cur_pt->x + cur_pt->y * cur_pt->y + cur_pt->z * cur_pt->z;
                if (range > (blind_square)){
                    pl_surf.push_back(pl_full[cur_region_idx[j]]); 
                    flag_if_fill[index] = 1;   
                }
            }
        }        
        // delete[]flag_if_fill;//释放
	    // flag_if_fill=NULL;
        free(flag_if_fill);
    }


}






// void Preprocess::extract_cloud_by_simple_voxel(const std::shared_ptr<livox_ros::LidarMsg> msg){
//     const double leafsize1 = param_.leafsize[0];
//     const double leafsize2 = param_.leafsize[1];
//     const double extent_xmin = param_.voxel_region_xyz[0];
//     const double extent_xmax = param_.voxel_region_xyz[1];
//     const double extent_ymin = param_.voxel_region_xyz[2];
//     const double extent_ymax = param_.voxel_region_xyz[3];
//     const double extent_zmin = param_.voxel_region_xyz[4];
//     const double extent_zmax = param_.voxel_region_xyz[5];
//     const double blind_square = param_.blind_distance * param_.blind_distance;

//     double leafsize_inv = 1.0/leafsize;

//     int Xcnt_region = (extent_xmax - extent_xmin)*leafsize_inv  +1; 
//     int Ycnt_region = (extent_ymax - extent_ymin)*leafsize_inv  +1; 
//     int Zcnt_region = (extent_zmax - extent_zmin)*leafsize_inv  +1; 
//     int vect_size = Xcnt_region * Ycnt_region * Zcnt_region;
//     unsigned char *flag_if_fill=(unsigned char*)calloc(vect_size,sizeof(unsigned char));

//     int plsize = msg->point_num;
//     int Xindex=0, Yindex=0, Zindex=0;

//     for (size_t i = 0; i < plsize; i++) 
//     {
//         auto* cur_pt = &msg->points[i];
//         if (((cur_pt->tag & 0x30) == 0x10 || (cur_pt->tag & 0x30) == 0x00)){
//             double range = cur_pt->x * cur_pt->x + cur_pt->y * cur_pt->y + cur_pt->z * cur_pt->z;
//             Xindex = int((cur_pt->x - extent_xmin) *leafsize_inv);
//             Yindex = int((cur_pt->y - extent_ymin) *leafsize_inv);
//             Zindex = int((cur_pt->z - extent_zmin) *leafsize_inv);
//             size_t index = Ycnt_region * Zcnt_region * Xindex + Zcnt_region * Yindex + Zindex;

//             if (!flag_if_fill[index])
//             {
//                 pl_full[i].x = msg->points[i].x;
//                 pl_full[i].y = msg->points[i].y;
//                 pl_full[i].z = msg->points[i].z;
//                 pl_full[i].intensity = msg->points[i].reflectivity;
//                 pl_full[i].curvature = msg->points[i].offset_time * float(1000); // use curvature as time of each laser points, curvature unit: ms

//                 if (range > (blind_square)){
//                     pl_surf.push_back(pl_full[i]); 
//                     flag_if_fill[index] = 1;   
//                 }
//             }
//         }
//     }        
//     // delete[]flag_if_fill;//释放
//     // flag_if_fill=NULL;
//     free(flag_if_fill);

// }

//////////////////////////////////////////////////////////////////////////////////////////////////
// 间隔采样
void Preprocess::extract_cloud_by_interval_sampling(const std::shared_ptr<livox_ros::LidarMsg> msg){
    // std::cout<<"extract cloud by method:  interval sampling"<<std::endl;
    int plsize = msg->point_num;
    uint valid_num = 0;
    double blind_square = blind * blind;
    double obstacle_square = obstacle_range * obstacle_range;
    for (uint i = 1; i < plsize; i++)
    {//zd delete (msg->points[i].line < N_SCANS)
        if (((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00) 
            && ((msg->points[i].tag & 0x03) == 0x01 || (msg->points[i].tag & 0x03) == 0x00))
        {
            valid_num++;
            double range = msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y + msg->points[i].z * msg->points[i].z;
            if (range < obstacle_square && range>blind_square){
                PointType point;
                point.x = msg->points[i].x;
                point.y = msg->points[i].y;
                point.z = msg->points[i].z;            
                pl_obstacle->points.push_back(point);
            }
                
            if (valid_num % point_filter_num == 0)
            {
                pl_full[i].x = msg->points[i].x;
                pl_full[i].y = msg->points[i].y;
                pl_full[i].z = msg->points[i].z;
                pl_full[i].intensity = msg->points[i].reflectivity;
                // pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms
                pl_full[i].curvature = msg->points[i].offset_time * float(1000); // use curvature as time of each laser points, curvature unit: ms
                // std::cout << "pl_full[i].curvature: " << pl_full[i].curvature << std::endl;
                // if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7))
                if ((abs(pl_full[i].x - pl_full[i - 1].x) > 0.15) || (abs(pl_full[i].y - pl_full[i - 1].y) > 0.15) || (abs(pl_full[i].z - pl_full[i - 1].z) > 0.15))
                {
                    if (range > (blind_square)){
                        pl_surf.push_back(pl_full[i]);    
                    }
                }//if
            }//if
        }//if
    }//for
}



//////////////////////////////////////////////////////////////////////////////////////////////////
// 特征点计算
// extract_cloud_by_feature()
// give_feature()
// plane_judge()
// edge_jump_judge()
void Preprocess::extract_cloud_by_feature(const std::shared_ptr<livox_ros::LidarMsg> msg){
    // std::cout<<"extract cloud by method: feature"<<std::endl;
    int plsize = msg->point_num;
    for (uint i = 1; i < plsize; i++){
        if ((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)){
            pl_full[i].x = msg->points[i].x;
            pl_full[i].y = msg->points[i].y;
            pl_full[i].z = msg->points[i].z;
            pl_full[i].intensity = msg->points[i].reflectivity;
            // pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points
            pl_full[i].curvature = msg->points[i].offset_time * float(1000); // use curvature as time of each laser points, curvature unit: ms

            bool is_new = false;
            if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7))
            {
                pl_buff[msg->points[i].line].push_back(pl_full[i]);
            }
        }
    }
    static int count = 0;
    static double time = 0.0;
    count++;
    double t0 = omp_get_wtime();
    for (int j = 0; j < N_SCANS; j++)
    {
        if (pl_buff[j].size() <= 5)
            continue;
        pcl::PointCloud<PointType> &pl = pl_buff[j];
        plsize = pl.size();
        vector<orgtype> &types = typess[j];
        types.clear();
        types.resize(plsize);
        plsize--;
        for (uint i = 0; i < plsize; i++)
        {
            types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
            vx = pl[i].x - pl[i + 1].x;
            vy = pl[i].y - pl[i + 1].y;
            vz = pl[i].z - pl[i + 1].z;
            types[i].dista = sqrt(vx * vx + vy * vy + vz * vz);
        }
        types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
        give_feature(pl, types);
        // pl_surf += pl;
    }
    time += omp_get_wtime() - t0;
    // printf("Feature extraction time: %lf \n", time / count);
    ROS_INFO("Feature extraction time: %lf ", time / count);
}

void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  int plsize = pl.size();
  int plsize2;
  if (plsize == 0)
  {
    // printf("something wrong\n");
    ROS_ERROR("something wrong: cloud_ssize == 0");
    return;
  }
  uint head = 0;

  while (types[head].range < blind)
  {
    head++;
  }

  // Surf
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  uint i_nex = 0, i2;
  uint last_i = 0;
  uint last_i_nex = 0;
  int last_state = 0;
  int plane_type;

  for (uint i = head; i < plsize2; i++)
  {
    if (types[i].range < blind)
    {
      continue;
    }

    i2 = i;

    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);

    if (plane_type == 1)
    {
      for (uint j = i; j <= i_nex; j++)
      {
        if (j != i && j != i_nex)
        {
          types[j].ftype = Real_Plane;
        }
        else
        {
          types[j].ftype = Poss_Plane;
        }
      }

      // if(last_state==1 && fabs(last_direct.sum())>0.5)
      if (last_state == 1 && last_direct.norm() > 0.1)
      {
        double mod = last_direct.transpose() * curr_direct;
        if (mod > -0.707 && mod < 0.707)
        {
          types[i].ftype = Edge_Plane;
        }
        else
        {
          types[i].ftype = Real_Plane;
        }
      }

      i = i_nex - 1;
      last_state = 1;
    }
    else // if(plane_type == 2)
    {
      i = i_nex;
      last_state = 0;
    }
    // else if(plane_type == 0)
    // {
    //   if(last_state == 1)
    //   {
    //     uint i_nex_tem;
    //     uint j;
    //     for(j=last_i+1; j<=last_i_nex; j++)
    //     {
    //       uint i_nex_tem2 = i_nex_tem;
    //       Eigen::Vector3d curr_direct2;

    //       uint ttem = plane_judge(pl, types, j, i_nex_tem, curr_direct2);

    //       if(ttem != 1)
    //       {
    //         i_nex_tem = i_nex_tem2;
    //         break;
    //       }
    //       curr_direct = curr_direct2;
    //     }

    //     if(j == last_i+1)
    //     {
    //       last_state = 0;
    //     }
    //     else
    //     {
    //       for(uint k=last_i_nex; k<=i_nex_tem; k++)
    //       {
    //         if(k != i_nex_tem)
    //         {
    //           types[k].ftype = Real_Plane;
    //         }
    //         else
    //         {
    //           types[k].ftype = Poss_Plane;
    //         }
    //       }
    //       i = i_nex_tem-1;
    //       i_nex = i_nex_tem;
    //       i2 = j-1;
    //       last_state = 1;
    //     }

    //   }
    // }

    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;
  }

  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for (uint i = head + 3; i < plsize2; i++)
  {
    if (types[i].range < blind || types[i].ftype >= Real_Plane)
    {
      continue;
    }

    if (types[i - 1].dista < 1e-16 || types[i].dista < 1e-16)
    {
      continue;
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
    Eigen::Vector3d vecs[2];

    for (int j = 0; j < 2; j++)
    {
      int m = -1;
      if (j == 1)
      {
        m = 1;
      }

      if (types[i + m].range < blind)
      {
        if (types[i].range > inf_bound)
        {
          types[i].edj[j] = Nr_inf;
        }
        else
        {
          types[i].edj[j] = Nr_blind;
        }
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z);
      vecs[j] = vecs[j] - vec_a;

      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
      if (types[i].angle[j] < jump_up_limit)
      {
        types[i].edj[j] = Nr_180;
      }
      else if (types[i].angle[j] > jump_down_limit)
      {
        types[i].edj[j] = Nr_zero;
      }
    }

    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
    if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero && types[i].dista > 0.0225 && types[i].dista > 4 * types[i - 1].dista)
    {
      if (types[i].intersect > cos160)
      {
        if (edge_jump_judge(pl, types, i, Prev))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor && types[i - 1].dista > 0.0225 && types[i - 1].dista > 4 * types[i].dista)
    {
      if (types[i].intersect > cos160)
      {
        if (edge_jump_judge(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf)
    {
      if (edge_jump_judge(pl, types, i, Prev))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor)
    {
      if (edge_jump_judge(pl, types, i, Next))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor)
    {
      if (types[i].ftype == Nor)
      {
        types[i].ftype = Wire;
      }
    }
  }

  plsize2 = plsize - 1;
  double ratio;
  for (uint i = head + 1; i < plsize2; i++)
  {
    if (types[i].range < blind || types[i - 1].range < blind || types[i + 1].range < blind)
    {
      continue;
    }

    if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8)
    {
      continue;
    }

    if (types[i].ftype == Nor)
    {
      if (types[i - 1].dista > types[i].dista)
      {
        ratio = types[i - 1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i - 1].dista;
      }

      if (types[i].intersect < smallp_intersect && ratio < smallp_ratio)
      {
        if (types[i - 1].ftype == Nor)
        {
          types[i - 1].ftype = Real_Plane;
        }
        if (types[i + 1].ftype == Nor)
        {
          types[i + 1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }

  int last_surface = -1;
  for (uint j = head; j < plsize; j++)
  {
    if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane)
    {
      if (last_surface == -1)
      {
        last_surface = j;
      }

      if (j == uint(last_surface + point_filter_num - 1))
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);

        last_surface = -1;
      }
    }
    else
    {
      if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane)
      {
        pl_corn.push_back(pl[j]);
      }
      if (last_surface != -1)
      {
        PointType ap;
        for (uint k = last_surface; k < j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j - last_surface);
        ap.y /= (j - last_surface);
        ap.z /= (j - last_surface);
        ap.intensity /= (j - last_surface);
        ap.curvature /= (j - last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

/*void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  pl.height = 1;
  pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}*/

int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  double group_dis = disA * types[i_cur].range + disB;
  group_dis = group_dis * group_dis;
  // i_nex = i_cur;

  double two_dis;
  vector<double> disarr;
  disarr.reserve(20);

  for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++)
  {
    if (types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    disarr.push_back(types[i_nex].dista);
  }

  for (;;)
  {
    if ((i_cur >= pl.size()) || (i_nex >= pl.size()))
      break;

    if (types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx * vx + vy * vy + vz * vz;
    if (two_dis >= group_dis)
    {
      break;
    }
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  double leng_wid = 0;
  double v1[3], v2[3];
  for (uint j = i_cur + 1; j < i_nex; j++)
  {
    if ((j >= pl.size()) || (i_cur >= pl.size()))
      break;
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    v2[0] = v1[1] * vz - vy * v1[2];
    v2[1] = v1[2] * vx - v1[0] * vz;
    v2[2] = v1[0] * vy - vx * v1[1];

    double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
    if (lw > leng_wid)
    {
      leng_wid = lw;
    }
  }

  if ((two_dis * two_dis / leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();
    return 0;
  }

  uint disarrsize = disarr.size();
  for (uint j = 0; j < disarrsize - 1; j++)
  {
    for (uint k = j + 1; k < disarrsize; k++)
    {
      if (disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  if (disarr[disarr.size() - 2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

  if (lidar_type == AVIA)
  {
    double dismax_mid = disarr[0] / disarr[disarrsize / 2];
    double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];

    if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  else
  {
    double dismax_min = disarr[0] / disarr[disarrsize - 2];
    if (dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }

  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;
}

bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  if (nor_dir == 0)
  {
    if (types[i - 1].range < blind || types[i - 2].range < blind)
    {
      return false;
    }
  }
  else if (nor_dir == 1)
  {
    if (types[i + 1].range < blind || types[i + 2].range < blind)
    {
      return false;
    }
  }
  double d1 = types[i + nor_dir - 1].dista;
  double d2 = types[i + 3 * nor_dir - 2].dista;
  double d;

  if (d1 < d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

  if (d1 > edgea * d2 || (d1 - d2) > edgeb)
  {
    return false;
  }

  return true;
}

// 特征点计算 end
//////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////
// 


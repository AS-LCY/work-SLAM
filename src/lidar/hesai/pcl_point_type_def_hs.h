#ifndef FAIRLAND_LIDAR_POINT_TYPE_HESAI_LIDAR_H
#define FAIRLAND_LIDAR_POINT_TYPE_HESAI_LIDAR_H


#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp> 

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>


// /// hesai Lidar pointcloud data struct
// struct HsPointXYZI
// {
//   PCL_ADD_POINT4D;      // 16
//   float intensities;      // 4
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;
// POINT_CLOUD_REGISTER_POINT_STRUCT(HsPointXYZI, 
//         (float, x, x)(float, y, y)(float, z, z)
//         (float, intensity, intensity)
//         )

#endif
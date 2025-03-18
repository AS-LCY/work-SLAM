#ifndef FAIRLAN_LIDAR_POINT_TYPE_VANJEE_LIDAR_H
#define FAIRLAN_LIDAR_POINT_TYPE_VANJEE_LIDAR_H


#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp> 

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>


/// Vanjee Lidar pointcloud data struct
struct VjPointXYZIRT
{
  PCL_ADD_POINT4D;      // 16
  float intensity;      // 4
  uint16_t ring = 0;    // 2
  double timestamp = 0; // 8 // = offset_time (unit: s), absolute_time = header_time + timestamp, different from robosense
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VjPointXYZIRT, 
        (float, x, x)(float, y, y)(float, z, z)
        (float, intensity, intensity)
        (std::uint16_t, ring, ring)
        (double, timestamp, timestamp)
        )

#endif
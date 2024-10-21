#ifndef FAIRLAN_LIDAR_POINT_TYPE_ROBOSENSE_LIDAR_H
#define FAIRLAN_LIDAR_POINT_TYPE_ROBOSENSE_LIDAR_H


#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp> 

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>


/// Livox Lidar pointcloud data struct 
struct RsPointXYZIRT
{
  PCL_ADD_POINT4D; // 16
  float intensity;  // 4
  std::uint8_t ring = 0;// 1
  double  timestamp;// 8  
  // std::uint32_t  offset_time;// 4  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(LvxPointXYZITLO, 
  (float, x, x)(float, y, y)(float, z, z)
  (float, intensity, intensity)  
  (std::uint8_t, ring, ring)  
  // (std::uint32_t, offset_time, offset_time)
  (double, timestamp, timestamp)
  )

/// about offset_time
/// offset_time = point_time - header_time, unit: ns,  
/// point_time  = header_time + offset_time / 1000000000.0, unit: s

#endif
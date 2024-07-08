#ifndef FAIRLAN_LIDAR_POINT_TYPE_LIVOX_LIDAR_H
#define FAIRLAN_LIDAR_POINT_TYPE_LIVOX_LIDAR_H


#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp> 

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>


/// Livox Lidar pointcloud data struct 
struct LvxPointXYZITLO
{
  PCL_ADD_POINT4D; // 16
  float intensity;  // 4
  std::uint8_t tag = 0;// 1
  std::uint8_t line = 0;// 1
  std::uint32_t  offset_time;// 4  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(LvxPointXYZITLO, 
  (float, x, x)(float, y, y)(float, z, z)
  (float, intensity, intensity)  
  (std::uint8_t, tag, tag) 
  (std::uint8_t, line, line)  
  (std::uint32_t, offset_time, offset_time)
  )

/// about offset_time
/// offset_time = point_time - header_time, unit: ns,  
/// point_time  = header_time + offset_time / 1000000000.0, unit: s

#endif
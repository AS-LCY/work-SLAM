#ifndef FAIRLAND_LIDAR_POINT_TYPE_ROBOSENSE_LIDAR_H
#define FAIRLAND_LIDAR_POINT_TYPE_ROBOSENSE_LIDAR_H

#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/impl/point_types.hpp>

/// Robosense Lidar pointcloud data struct
struct RsPointXYZIRT {
	PCL_ADD_POINT4D;	  // 16
	float intensity;	  // 4
	uint16_t ring = 0;	  // 2
	double timestamp = 0; // 8
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
													 std::uint16_t, ring, ring)(double, timestamp, timestamp))

#endif
#ifndef PREPROCESS_H
#define PREPROCESS_H

#include "lidar/livox/ros_livox_datatype_def.h"
#include "lidar_slam/common_lib.h"
#include "lidar_slam/slam_param_def.h"
// #include "lddc.h"
using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

/*struct LidarPoint{
	u_int32_t offset_time;//      # offset time relative to the base time
	float x;//               # X axis, unit:m
	float y;//               # Y axis, unit:m
	float z;//               # Z axis, unit:m
	u_int8_t reflectivity;//      # reflectivity, 0~255
	u_int8_t tag;//               # livox tag
	u_int8_t line;//              # laser number in lidar
};
struct LidarMsg{
	double time_stamp;
	u_int16_t point_num;
	u_int8_t lidar_id;
	u_int8_t rsvd[3];
	std::vector<LidarPoint> points;

};*/
enum LID_TYPE { AVIA = 1, VELO16, OUST64, RS32 }; //{1, 2, 3, 4}
enum TIME_UNIT { SEC = 0, MS = 1, US = 2, NS = 3 };
enum Feature { Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint };
enum Surround { Prev, Next };
enum E_jump { Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind };

struct orgtype {
	double range;
	double dista;
	double angle[2];
	double intersect;
	E_jump edj[2];
	Feature ftype;
	orgtype() {
		range = 0;
		edj[Prev] = Nr_nor;
		edj[Next] = Nr_nor;
		ftype = Nor;
		intersect = 2;
	}
};

namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
	PCL_ADD_POINT4D;
	float intensity;
	float time;
	uint16_t ring;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
								  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
																		  intensity)(float, time, time)(std::uint16_t,
																										ring, ring))

namespace ouster_ros {
struct EIGEN_ALIGN16 Point {
	PCL_ADD_POINT4D;
	float intensity;
	uint32_t t;
	uint16_t reflectivity;
	uint8_t ring;
	uint16_t ambient;
	uint32_t range;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)
/**
 * 6D位姿点云结构定义
*/
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D     
    PCL_ADD_INTENSITY;  
    float roll;         
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
} EIGEN_ALIGN16;                    

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;
class Preprocess
{
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();
  
  void process(const std::shared_ptr<livox_ros::LidarMsg> msg, PointCloudType::Ptr &pcl_out);
 // void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudType::Ptr &pcl_out);
  // void set(bool feat_en, bool voxel_en, int lid_type, double bld, int pfilt_num,int line,double obstacle);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num,int line,double obstacle);
  void set(lidar_slam::LidarPreprocParam param_in);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudType pl_full, pl_corn, pl_surf;
  PointCloudType::Ptr pl_obstacle;
  PointCloudType pl_buff[128]; //maximum 128 line lidar
  vector<orgtype> typess[128]; //maximum 128 line lidar
  float time_unit_scale;
  int lidar_type, point_filter_num, N_SCANS, time_unit;
  double blind,obstacle_range;
  bool feature_enabled, given_offset_time;
//  ros::Publisher pub_full, pub_surf, pub_corn;
    

  private:
  //void rs_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void avia_handler(const std::shared_ptr<livox_ros::LidarMsg> msg);
 // void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
 // void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);


  void extract_cloud_by_feature(const std::shared_ptr<livox_ros::LidarMsg> msg);
  void extract_cloud_by_simple_voxel(const std::shared_ptr<livox_ros::LidarMsg> msg);
  void extract_cloud_by_interval_sampling(const std::shared_ptr<livox_ros::LidarMsg> msg);
  void extract_cloud_by_interval_and_voxel(const std::shared_ptr<livox_ros::LidarMsg> msg);

  void give_feature(PointCloudType &pl, vector<orgtype> &types);
  //void pub_func(PointCloudType &pl, const ros::Time &ct);
  int  plane_judge(const PointCloudType &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudType &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudType &pl, vector<orgtype> &types, uint i, Surround nor_dir);
  
  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;

  /////
  double leafsize = 0.2;
  std::vector<double> downsample_region_xyz_;
  lidar_slam::LidarPreprocParam param_;
};
#endif

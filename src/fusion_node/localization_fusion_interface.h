
#ifndef FLBOT_LOCALIZTION_FUSION_INTERFACE_H
#define FLBOT_LOCALIZTION_FUSION_INTERFACE_H

#include <ros/ros.h>

#include <mutex>
// ros-msg
// #include <std_msgs/UInt32.h>
// #include <geometry_msgs/Twist.h>
// #include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
// #include <sensor_msgs/NavSatFix.h>
// #include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>

#include <logTracer/tracer.h>

#include "ekf_fusion/ekf_localization_fusion.h"
#include "flbot_msgs/LocalizationPoseData.h"
#include "flbot_msgs/NameValues.h"
#include "flbot_msgs/chassic_data.h"
#include "fusion_param.hpp"
#include "node/log_info_manager.hpp"
#include "slipping/detect_slipping.h"

namespace localization_module {

class LocalizationFusion {
	DECL_CLASSNAME(LocalizationFusion)
   public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	LocalizationFusion();
	~LocalizationFusion();

   private:
	bool load_params();
	bool create_ROS_IO();

	// callbacks
	void chassis_msg_callback(const flbot_msgs::chassic_data::ConstPtr& chassis_msg_in);
	void slam_odometry_callback(const nav_msgs::Odometry::ConstPtr& slam_odometry_in);
	void imu_msg_callback(const sensor_msgs::Imu::ConstPtr& imu_msg_in);

	void init_chassis_imu_slam_odom_stamp();

	void check_slam_odometry(nav_msgs::Odometry slam_odom);
	void compose_status(int slip_flag, nav_msgs::Odometry slam_odom, sensor_msgs::Imu imu_msg,
						flbot_msgs::chassic_data chassis_msg, flbot_msgs::LocalizationPoseData* status_msg);
	void pub_localiztion(flbot_msgs::LocalizationPoseData cur_status);
	void pub_fusion_info(double time_last);

	// slipping detect
	int detect_slipping(nav_msgs::Odometry curr_odom);
	// void fill_slipping_msg(flbot_msgs::NameValues& slipping_msg, ros::Time slam_odom_stamp, int slip_flag);
	void fill_slipping_msg(std_msgs::Float64MultiArray& slipping_msg, ros::Time slam_odom_stamp, int slip_flag);

   public:
	static LocalizationModuleLogInfoManager& log_info_manager_;

   private:
	std::mutex mutex_; ///< the only mutex
	std::shared_ptr<EkfLocalizationFusion> ekf_fusion_ptr_;
	std::shared_ptr<DetectSlipping> slipping_ptr_;

	bool is_chassis_rcv_ = false;
	bool is_imu_rcv_ = false;
	bool lf_need_init_ = true;

	bool use_fusion_ = true;

	long seq_count_ = 0;
	Eigen::Isometry3d T_baselink2lidar_;
	Eigen::Isometry3d T_lidar2baselink_;

	double last_slam_odom_time_ = 0.0;
	double slam_speed_ = 0;

	// param read
	double time_lost_thr_ = 3.0; // unit: second
	bool ekf_use_chassis_ = true;

	flbot_msgs::LocalizationPoseData last_status_;
	flbot_msgs::LocalizationPoseData status_tmp_;
	flbot_msgs::LocalizationPoseData status_origin_; ///< the origin status message
	flbot_msgs::LocalizationPoseData status_lf_;	 ///< the lfed status message

	ros::NodeHandle nh_;
	ros::Subscriber sub_imu_;
	ros::Subscriber sub_chassis_;
	ros::Subscriber sub_slam_odom_;
	ros::Publisher pub_fusion_odom_;
	ros::Publisher pub_slip_;
	ros::Publisher pub_info_;
	std::string sub_imu_topic_;
	std::string sub_chassis_topic_;
	std::string sub_slam_odom_topic_;
	std::string pub_localization_topic_;
	std::string pub_slipping_topic_;

	flbot_msgs::chassic_data chassis_msg_; ///< the chassis message
	nav_msgs::Odometry slam_odom_msg_;	   ///< the gnss message
	sensor_msgs::Imu imu_msg_;			   ///< the imu message
};

LocalizationModuleLogInfoManager& LocalizationFusion::log_info_manager_ =
	LocalizationModuleLogInfoManager::getInstance();

} // namespace localization_module

#endif // FLBOT_LOCALIZTION_FUSION_H

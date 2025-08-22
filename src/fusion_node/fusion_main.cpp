#include <ros/ros.h>
// #include "lidar_slam/common_lib.h"
#include "fusion_node/localization_fusion_interface.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "pose_fusion");
	// ros::NodeHandle nh;

	localization_module::LocalizationFusion localization_fusion;

	ROS_INFO("\033[1;32m----> Localization Fusion Start! \033[0m");

	// ros::spin();
	return 0;
}

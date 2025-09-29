#include "node/localization_module.h"

namespace localization_module {

void LocalizationModule::publish_cloud(const double& cloud_time, PointCloudType::Ptr pcl_cloud_in,
									   const std::string& frame_id, rclcpp::Publisher<PointCloud2>::SharedPtr pub) {
	sensor_msgs::msg::PointCloud2 ros_cloud_msg;
	pcl::toROSMsg(*pcl_cloud_in, ros_cloud_msg);
	ros_cloud_msg.header.stamp = rclcpp::Time(cloud_time * 1e9);
	ros_cloud_msg.header.frame_id = frame_id;
	pub->publish(ros_cloud_msg);
}

void LocalizationModule::publish_odometry_lidar_in_map(
	const double& lidar_in_map_time,				 // unit: second
	const Eigen::Isometry3d& lidar_in_map,			 // T_map_baselink
	const lidar_slam::Localization_base& T_odom_imu, // 10hz，T_odom_imu, imu和lidar和base_link朝向一致
	const std::string& frameid,						 // map
	const std::string& child_frameid,				 // base_link
	ModuleStatus curr_running_module_status) {
	const auto msg_stamp = rclcpp::Time(lidar_in_map_time * 1e9);
	nav_msgs::msg::Odometry odomAftMapped;
	odomAftMapped.header.frame_id = frameid;
	odomAftMapped.child_frame_id = child_frameid;
	odomAftMapped.header.stamp = msg_stamp;

	static Eigen::Quaterniond q_last = Eigen::Quaterniond::Identity();
	static double q_time_last = lidar_in_map_time;
	static bool first_pub = true;
	double q_time_curr = lidar_in_map_time;
	odomAftMapped.pose.pose.position.x = lidar_in_map.translation().x();
	odomAftMapped.pose.pose.position.y = lidar_in_map.translation().y();
	odomAftMapped.pose.pose.position.z = lidar_in_map.translation().z();
	Eigen::Quaterniond quaternion = Eigen::Quaterniond(lidar_in_map.rotation());
	quaternion.normalize();

	if (first_pub) {
		first_pub = false;
	} else {
		double dt = q_time_curr - q_time_last;
		assert(dt != 0.f);
		Eigen::Quaterniond dq = q_last.inverse() * quaternion;
		dq.normalize();
		Eigen::AngleAxisd dq_angle_axis(dq);
		double dq_angle = dq_angle_axis.angle();
		Eigen::Vector3d dq_axis = dq_angle_axis.axis();
		if (dq_angle > M_PI) { // 防止数值不稳定 (当角度接近 0 时)
			dq_angle -= 2 * M_PI;
		}
		Eigen::Vector3d gyro = (dq_angle / dt) * dq_axis;
		odomAftMapped.twist.twist.angular.x = gyro.x() - T_odom_imu.imu_state.bg.x();
		odomAftMapped.twist.twist.angular.y = gyro.y() - T_odom_imu.imu_state.bg.y();
		odomAftMapped.twist.twist.angular.z = gyro.z() - T_odom_imu.imu_state.bg.z(); //和imu的gyro可以做校验
	}
	q_last = quaternion;
	q_time_last = q_time_curr;

	odomAftMapped.pose.pose.orientation.x = quaternion.x();
	odomAftMapped.pose.pose.orientation.y = quaternion.y();
	odomAftMapped.pose.pose.orientation.z = quaternion.z();
	odomAftMapped.pose.pose.orientation.w = quaternion.w();

	auto vel = T_odom_imu.imu_state.rot.inverse() * T_odom_imu.imu_state.vel;
	// jxl: 直接取逆然后相乘，计算的结果就是对的；rot.matrix().inverse()是错的
	TRACE_DBG_CLASS("before: %f, %f, %f\n", T_odom_imu.imu_state.vel.x(), T_odom_imu.imu_state.vel.y(),
					T_odom_imu.imu_state.vel.z());
	TRACE_DBG_CLASS("after: %f, %f, %f\n\n\n", vel.x(), vel.y(), vel.z());

	odomAftMapped.twist.twist.linear.x = vel[0]; // baselink下的线速度
	odomAftMapped.twist.twist.linear.y = vel[1];
	odomAftMapped.twist.twist.linear.z = vel[2];

	//输出ba, bg到twist的diag cov来可视化
	odomAftMapped.twist.covariance[0] = T_odom_imu.imu_state.ba.x();  // (0,0)
	odomAftMapped.twist.covariance[7] = T_odom_imu.imu_state.ba.y();  // (1,1)
	odomAftMapped.twist.covariance[14] = T_odom_imu.imu_state.ba.z(); // (2,2)
	odomAftMapped.twist.covariance[21] = T_odom_imu.imu_state.bg.x(); // (3,3)
	odomAftMapped.twist.covariance[28] = T_odom_imu.imu_state.bg.y(); // (4,4)
	odomAftMapped.twist.covariance[35] = T_odom_imu.imu_state.bg.z(); // (5,5)

	// log_info_manager_->log_info.slam_vel_x = odomAftMapped.twist.twist.linear.x;
	log_info_manager_.slam_info.data[9] = odomAftMapped.twist.twist.linear.x; // slam_vel_x

	pubOdomAftMapped->publish(odomAftMapped);

	//发布T_odom_baselink的里程计
	Eigen::Isometry3d T_odom_imu_eigen = Eigen::Isometry3d::Identity();
	T_odom_imu_eigen.translation() = T_odom_imu.imu_state.pos;
	T_odom_imu_eigen.linear() = T_odom_imu.imu_state.rot.unit_quaternion().toRotationMatrix();
	auto T_odom_baselink = T_odom_imu_eigen * slam_param_.extrinsic.T_imu_baselink;
	nav_msgs::msg::Odometry lio_odom;
	lio_odom.header.stamp = msg_stamp;
	lio_odom.header.frame_id = "odom";
	lio_odom.child_frame_id = child_frameid;
	lio_odom.pose.pose.position.x = T_odom_baselink.translation().x();
	lio_odom.pose.pose.position.y = T_odom_baselink.translation().y();
	lio_odom.pose.pose.position.z = T_odom_baselink.translation().z();
	lio_odom.pose.pose.orientation.x = Eigen::Quaterniond(T_odom_baselink.linear()).x();
	lio_odom.pose.pose.orientation.y = Eigen::Quaterniond(T_odom_baselink.linear()).y();
	lio_odom.pose.pose.orientation.z = Eigen::Quaterniond(T_odom_baselink.linear()).z();
	lio_odom.pose.pose.orientation.w = Eigen::Quaterniond(T_odom_baselink.linear()).w();
	Eigen::Matrix<double, 6, 1> lio_state_diag_cov = slam_->get_lio_state_diag_cov();
	lio_odom.pose.covariance[0] = lio_state_diag_cov(0);  // position
	lio_odom.pose.covariance[7] = lio_state_diag_cov(1);  // rotation
	lio_odom.pose.covariance[14] = lio_state_diag_cov(2); // velocity
	lio_odom.pose.covariance[21] = lio_state_diag_cov(3); // ba
	lio_odom.pose.covariance[28] = lio_state_diag_cov(4); // bg
	pubLioOdom->publish(lio_odom);

	geometry_msgs::msg::TransformStamped transform;
	transform.header.stamp = msg_stamp;
	transform.header.frame_id = odomAftMapped.header.frame_id;
	transform.child_frame_id = odomAftMapped.child_frame_id;
	transform.transform.translation.x = odomAftMapped.pose.pose.position.x;
	transform.transform.translation.y = odomAftMapped.pose.pose.position.y;
	transform.transform.translation.z = odomAftMapped.pose.pose.position.z;
	transform.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
	transform.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
	transform.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
	transform.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;

	br_.sendTransform(transform);

	// pub T_map_baselink path
	baselink_in_map_path_msg.header.stamp = msg_stamp;
	baselink_in_map_path_msg.header.frame_id = frameid;
	geometry_msgs::msg::PoseStamped msg;
	msg.header.stamp = msg_stamp;
	msg.header.frame_id = child_frameid;
	msg.pose = odomAftMapped.pose.pose;
	baselink_in_map_path_msg.poses.push_back(msg);
	auto path_size = baselink_in_map_path_msg.poses.size();
	auto keep_path_length = 3 * 1000; // 10hz, 300s path
	if (path_size > keep_path_length) {
		baselink_in_map_path_msg.poses.clear();
	}
	pubBaseLinkMapPath->publish(baselink_in_map_path_msg);

	// pub T_odom_baselink path
	baselink_in_odom_path_msg.header.stamp = msg_stamp;
	baselink_in_odom_path_msg.header.frame_id = "odom";
	geometry_msgs::msg::PoseStamped odom_msg;
	odom_msg.header.stamp = msg_stamp;
	odom_msg.header.frame_id = child_frameid;
	odom_msg.pose = lio_odom.pose.pose;
	baselink_in_odom_path_msg.poses.push_back(odom_msg);
	auto odom_path_size = baselink_in_odom_path_msg.poses.size();
	auto odom_keep_path_length = 3 * 1000; // 10hz, 300s path
	if (odom_path_size > odom_keep_path_length) {
		baselink_in_odom_path_msg.poses.clear();
	}
	pubBaseLinkOdompPath->publish(baselink_in_odom_path_msg);
}

void LocalizationModule::publish_OdomToMap_tf(const double& lidar_in_map_time, const Eigen::Isometry3d& T_map_odom) {
	geometry_msgs::msg::TransformStamped transform;
	transform.header.stamp = rclcpp::Time(lidar_in_map_time * 1e9);
	transform.header.frame_id = "map";
	transform.child_frame_id = "odom";

	transform.transform.translation.x = T_map_odom.translation().x();
	transform.transform.translation.y = T_map_odom.translation().y();
	transform.transform.translation.z = T_map_odom.translation().z();
	Eigen::Quaterniond q(T_map_odom.rotation());
	transform.transform.rotation.x = q.x();
	transform.transform.rotation.y = q.y();
	transform.transform.rotation.z = q.z();
	transform.transform.rotation.w = q.w();

	br_.sendTransform(transform);
}

void LocalizationModule::visualizePoseGraph(const std::vector<KeyPose>& poses,
											const std::vector<std::pair<int, int>>& loop_edges) {
	visualization_msgs::msg::MarkerArray markerArray;
	static size_t nodes_id = 0;
	static size_t edges_id = 0;

	// 1. 节点 Marker 初始化
	visualization_msgs::msg::Marker nodes;
	nodes.lifetime = rclcpp::Duration(0, 0);
	nodes.header.frame_id = "map";
	nodes.header.stamp = node_->now();
	nodes.ns = "pose_graph_nodes";
	nodes.id = nodes_id++;
	nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
	nodes.action = visualization_msgs::msg::Marker::ADD;
	nodes.pose.orientation.w = 1.0;
	nodes.scale.x = 0.1;
	nodes.scale.y = 0.1;
	nodes.scale.z = 0.1;
	nodes.color.r = 0.0f;
	nodes.color.g = 1.f;
	nodes.color.b = 1.f;
	nodes.color.a = 1.f; //透明度

	// 2. 边 Marker 初始化（关键帧之间的边 + 闭环边）
	visualization_msgs::msg::Marker edges;
	edges.lifetime = rclcpp::Duration(0, 0);
	edges.header.frame_id = "map";
	edges.header.stamp = node_->now();
	edges.ns = "pose_graph_edges";
	edges.id = edges_id++;
	edges.type = visualization_msgs::msg::Marker::LINE_LIST;
	edges.action = visualization_msgs::msg::Marker::ADD;
	edges.pose.orientation.w = 1.0;
	edges.scale.x = 0.01; // 线宽

	static int view_start_node_id = 0;
	static int view_start_loop_id = 0;

	for (int i = view_start_node_id; i < (int)poses.size(); i++) {
		geometry_msgs::msg::Point p;
		p.x = poses[i].pose.translation().x();
		p.y = poses[i].pose.translation().y();
		p.z = poses[i].pose.translation().z();
		nodes.points.push_back(p);

		// 如果不是第一个节点，发布与前一帧的边
		if (i > 0) {
			geometry_msgs::msg::Point p_prev;
			p_prev.x = poses[i - 1].pose.translation().x();
			p_prev.y = poses[i - 1].pose.translation().y();
			p_prev.z = poses[i - 1].pose.translation().z();

			edges.points.push_back(p_prev);
			edges.points.push_back(p);

			std_msgs::msg::ColorRGBA color;
			color.r = 1.f;
			color.g = 1.f;
			color.b = 1.f;
			color.a = 1.f;
			edges.colors.push_back(color);
			edges.colors.push_back(color);
		}
	}

	// 4. 添加新增的闭环边
	for (int i = view_start_loop_id; i < (int)loop_edges.size(); i++) {
		int id1 = loop_edges[i].first;
		int id2 = loop_edges[i].second;
		if (id1 >= poses.size() || id2 >= poses.size()) continue;

		geometry_msgs::msg::Point p1, p2;
		p1.x = poses[id1].pose.translation().x();
		p1.y = poses[id1].pose.translation().y();
		p1.z = poses[id1].pose.translation().z();

		p2.x = poses[id2].pose.translation().x();
		p2.y = poses[id2].pose.translation().y();
		p2.z = poses[id2].pose.translation().z();

		edges.points.push_back(p1);
		edges.points.push_back(p2);

		std_msgs::msg::ColorRGBA color;
		color.r = 0.0f;
		color.g = 1.0f;
		color.b = 0.0f;
		color.a = 1.0f;
		edges.colors.push_back(color);
		edges.colors.push_back(color);
	}

	// 5. 打包到 MarkerArray 并发布
	markerArray.markers.push_back(nodes);
	markerArray.markers.push_back(edges);
	pub_pose_graph_->publish(markerArray);

	// 6. 更新索引，下次调用时只绘制新增的
	view_start_node_id = poses.size();
	view_start_loop_id = loop_edges.size();
}

// void LocalizationModule::show_keyframe(
// 	const std::vector<lidar_slam::ScInfo, Eigen::aligned_allocator<lidar_slam::ScInfo>>& loadKeyframe) {
// 	visualization_msgs::msg::MarkerArray MarkerArray; //定义MarkerArray对象
// 	int number = loadKeyframe.size();				  // object_in为输入的目标个数
// 	for (int i = 0; i < number; i++) {
// 		if (i % 10 != 0) {
// 			continue;
// 		}
// 		visualization_msgs::msg::Marker Marker; //定义Marker对象
// 		Marker.header.frame_id = "map";
// 		Marker.header.stamp = node_->now();
// 		Marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING; //选用文本类型
// 		Marker.ns = "basic_shapes";										 //必写，否则rviz无法显示
// 		Marker.pose.orientation.w = 1.0;								 //文字的方向
// 		Marker.id =
// 			i;
// //用来标记同一帧不同的对象，如果后面的帧的对象少于前面帧的对象，那么少的id将在rviz中残留，所以需要后续的实时更新程序
// 		Marker.scale.x = 1.5;
// 		Marker.scale.y = 1.5;
// 		Marker.scale.z = 1.5; //文字的大小
// 		Marker.color.b = 25;
// 		Marker.color.g = 0;
// 		Marker.color.r = 25; //文字的颜色
// 		Marker.color.a = 1;	 //必写，否则rviz无法显示
// 		geometry_msgs::msg::Pose pose;
// 		pose.position.x = loadKeyframe[i].pose.translation().x();
// 		pose.position.y = loadKeyframe[i].pose.translation().y();
// 		pose.position.z = loadKeyframe[i].pose.translation().z();
// 		ostringstream str;
// 		//     str<< loadKeyframe[i].id << " " << loadKeyframe[i].pose.translation().x() << " " <<
// 		//     loadKeyframe[i].pose.translation().y() << " " << loadKeyframe[i].pose.translation().z();
// 		str << loadKeyframe[i].id;
// 		Marker.text = str.str(); //文字内容
// 		Marker.pose = pose;		 //文字的位置
// 		MarkerArray.markers.push_back(Marker);
// 	}
// 	pubKeyframePose->publish(MarkerArray);
// }

} // namespace localization_module

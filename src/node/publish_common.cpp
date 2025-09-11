#include "node/localization_module.h"

namespace localization_module {

void LocalizationModule::publish_cloud(PointCloudType::Ptr pcl_cloud_in, const std::string& frame_id,
									   rclcpp::Publisher<PointCloud2>::SharedPtr pub) {
	sensor_msgs::msg::PointCloud2 ros_cloud_msg;
	pcl::toROSMsg(*pcl_cloud_in, ros_cloud_msg);
	ros_cloud_msg.header.stamp = node_->now();
	ros_cloud_msg.header.frame_id = frame_id;
	pub->publish(ros_cloud_msg);
}

void LocalizationModule::pub_test_cloud(PointCloudType::Ptr msg_in, bool localization_mode) {
	sensor_msgs::msg::PointCloud2 laserCloudmsg;
	pcl::toROSMsg(*msg_in, laserCloudmsg);
	laserCloudmsg.header.stamp = node_->now();
	if (localization_mode)
		laserCloudmsg.header.frame_id = "map";
	else
		laserCloudmsg.header.frame_id = "wheel";
	pubTestCloud->publish(laserCloudmsg);
}

// void LocalizationModule::updatePath(const nav_msgs::Odometry odomAftMapped)
// {
//     geometry_msgs::PoseStamped pose_stamped;
//     pose_stamped.header = odomAftMapped.header;
//     pose_stamped.pose.position.x = odomAftMapped.pose.pose.position.x;
//     pose_stamped.pose.position.y = odomAftMapped.pose.pose.position.y;
//     pose_stamped.pose.position.z = odomAftMapped.pose.pose.position.z;
//     pose_stamped.pose.orientation = odomAftMapped.pose.pose.orientation;

//     globalPath.poses.push_back(pose_stamped);
// }

void LocalizationModule::publish_odometry_in_map(const Eigen::Isometry3d& odom_in_map, const std::string& frameid,
												 const std::string& child_frameid) {
	geometry_msgs::msg::TransformStamped transform;

	transform.header.stamp = node_->now();
	transform.header.frame_id = frameid;	  // 替换为实际的父坐标系
	transform.child_frame_id = child_frameid; // 替换为实际的子坐标系

	transform.transform.translation.x = odom_in_map.translation().x();
	transform.transform.translation.y = odom_in_map.translation().y();
	transform.transform.translation.z = odom_in_map.translation().z();
	Eigen::Quaterniond quaternion = Eigen::Quaterniond(odom_in_map.rotation());
	transform.transform.rotation.w = quaternion.w();
	transform.transform.rotation.x = quaternion.x();
	transform.transform.rotation.y = quaternion.y();
	transform.transform.rotation.z = quaternion.z();

	br_.sendTransform(transform);
}

void LocalizationModule::publish_odometry_lidar_in_map(
	const Eigen::Isometry3d& lidar_in_map,			 // T_map_baselink
	const lidar_slam::Localization_base& T_odom_imu, // 10hz，T_odom_imu, imu和lidar和base_link朝向一致
	const std::string& frameid,						 // map
	const std::string& child_frameid,				 // base_link
	ModuleStatus curr_running_module_status) {
	nav_msgs::msg::Odometry odomAftMapped;
	odomAftMapped.header.frame_id = frameid;
	odomAftMapped.child_frame_id = child_frameid;
	odomAftMapped.header.stamp = node_->now(); // TODO(jxl): 不应该是now时间，应该是用的哪个msg计算的pose，就是哪个时间
	odomAftMapped.pose.pose.position.x = lidar_in_map.translation().x();
	odomAftMapped.pose.pose.position.y = lidar_in_map.translation().y();
	odomAftMapped.pose.pose.position.z = lidar_in_map.translation().z();
	Eigen::Quaterniond quaternion = Eigen::Quaterniond(lidar_in_map.rotation());
	odomAftMapped.pose.pose.orientation.x = quaternion.x();
	odomAftMapped.pose.pose.orientation.y = quaternion.y();
	odomAftMapped.pose.pose.orientation.z = quaternion.z();
	odomAftMapped.pose.pose.orientation.w = quaternion.w();

	if (curr_running_module_status == ModuleStatus::MODULE_LOCALIZATION) {
		odomAftMapped.pose.covariance[1] = 3;
	} else if (is_mapping_status(curr_running_module_status)) {
		odomAftMapped.pose.covariance[1] = 2;
	}

	// Eigen::Isometry3d iso_transform = Eigen::Isometry3d::Identity();
	// Eigen::Matrix3d mat = curr_pose.imu_state.rot.matrix();
	// iso_transform.linear() = mat;
	// Eigen::Isometry3d iso_transform_inv = iso_transform.inverse();
	// Eigen::Matrix3d rot = iso_transform_inv.linear();
	// auto vel = rot * curr_pose.imu_state.vel;
	auto vel = T_odom_imu.imu_state.rot.inverse() * T_odom_imu.imu_state.vel;
	// jxl: 直接取逆然后相乘，计算的结果就是对的；rot.matrix().inverse()是错的
	TRACE_DBG_CLASS("before: %f, %f, %f\n", T_odom_imu.imu_state.vel.x(), T_odom_imu.imu_state.vel.y(),
					T_odom_imu.imu_state.vel.z());
	TRACE_DBG_CLASS("after: %f, %f, %f\n\n\n", vel.x(), vel.y(), vel.z());

	odomAftMapped.twist.twist.linear.x = vel[0]; // baselink下的线速度
	odomAftMapped.twist.twist.linear.y = vel[1];
	odomAftMapped.twist.twist.linear.z = vel[2];

	// log_info_manager_->log_info.slam_vel_x = odomAftMapped.twist.twist.linear.x;
	log_info_manager_->slam_info.data[9] = odomAftMapped.twist.twist.linear.x; // slam_vel_x

	pubOdomAftMapped->publish(odomAftMapped);

	geometry_msgs::msg::TransformStamped transform;
	transform.header.stamp = node_->now(); // TODO(jxl): 不应该是now时间，应该是用的哪个msg计算的pose，就是哪个时间
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
	baselink_in_map_path_msg.header.stamp = node_->now();
	baselink_in_map_path_msg.header.frame_id = frameid;
	geometry_msgs::msg::PoseStamped msg;
	msg.header.stamp = node_->now();
	msg.header.frame_id = child_frameid;
	msg.pose = odomAftMapped.pose.pose;
	baselink_in_map_path_msg.poses.push_back(msg);
	auto path_size = baselink_in_map_path_msg.poses.size();
	auto keep_path_length = 3 * 1000; // 10hz, 300s path
	if (path_size > keep_path_length) {
		baselink_in_map_path_msg.poses.clear();
	}
	pubBaseLinkMapPath->publish(baselink_in_map_path_msg);
}

void LocalizationModule::publish_OdomToMap_tf(const Eigen::Isometry3d& T_map_odom) {
	geometry_msgs::msg::TransformStamped transform;
	transform.header.stamp = node_->now();
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

/*
void LocalizationModule::publish_odometry(const Eigen::Isometry3d isometry_3d, std::string frameid, std::string
child_frameid, ros::Publisher pub_odom)
{
	nav_msgs::Odometry odom;
	odom.header.frame_id = frameid;
	odom.child_frame_id = child_frameid;
	odom.header.stamp = ros::Time().now(); // ros::Time().fromSec(lidar_end_time);
	odom.pose.pose.position.x = isometry_3d.translation().x();
	odom.pose.pose.position.y = isometry_3d.translation().y();
	odom.pose.pose.position.z = isometry_3d.translation().z();
	Eigen::Quaterniond quaternion = Eigen::Quaterniond(isometry_3d.rotation());
	odom.pose.pose.orientation.x = quaternion.x();
	odom.pose.pose.orientation.y = quaternion.y();
	odom.pose.pose.orientation.z = quaternion.z();
	odom.pose.pose.orientation.w = quaternion.w();
	pub_odom.publish(odom);
	// static tf::TransformBroadcaster br;
	// tf::Transform transform;
	// tf::Quaternion q;
	// transform.setOrigin(tf::Vector3(odom.pose.pose.position.x,
	//                                 odom.pose.pose.position.y,
	//                                 odom.pose.pose.position.z));
	// q.setW(odom.pose.pose.orientation.w);
	// q.setX(odom.pose.pose.orientation.x);
	// q.setY(odom.pose.pose.orientation.y);
	// q.setZ(odom.pose.pose.orientation.z);
	// transform.setRotation(q);
	// br.sendTransform(tf::StampedTransform(transform, odom.header.stamp, frameid, child_frameid));
}
*/

void LocalizationModule::publish_static_transform(const Eigen::Isometry3d& wheel_in_lidar) {
	nav_msgs::msg::Odometry odomAftMapped;
	odomAftMapped.header.frame_id = "lidar";
	odomAftMapped.child_frame_id = "wheel";
	odomAftMapped.header.stamp = node_->now(); // ros::Time().now(); // ros::Time().fromSec(lidar_end_time);
	odomAftMapped.pose.pose.position.x = wheel_in_lidar.translation().x();
	odomAftMapped.pose.pose.position.y = wheel_in_lidar.translation().y();
	odomAftMapped.pose.pose.position.z = wheel_in_lidar.translation().z();
	Eigen::Quaterniond quaternion = Eigen::Quaterniond(wheel_in_lidar.rotation());
	odomAftMapped.pose.pose.orientation.x = quaternion.x();
	odomAftMapped.pose.pose.orientation.y = quaternion.y();
	odomAftMapped.pose.pose.orientation.z = quaternion.z();
	odomAftMapped.pose.pose.orientation.w = quaternion.w();
	// static tf::TransformBroadcaster br;
	// tf::Transform transform;
	// tf::Quaternion q;
	// transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
	//                                 odomAftMapped.pose.pose.position.y,
	//                                 odomAftMapped.pose.pose.position.z));
	// q.setW(odomAftMapped.pose.pose.orientation.w);
	// q.setX(odomAftMapped.pose.pose.orientation.x);
	// q.setY(odomAftMapped.pose.pose.orientation.y);
	// q.setZ(odomAftMapped.pose.pose.orientation.z);
	// transform.setRotation(q);

	geometry_msgs::msg::TransformStamped transform;

	transform.header.stamp = node_->now();
	transform.header.frame_id = odomAftMapped.header.frame_id; // 替换为实际的父坐标系
	transform.child_frame_id = odomAftMapped.child_frame_id;   // 替换为实际的子坐标系

	transform.transform.translation.x = odomAftMapped.pose.pose.position.x;
	transform.transform.translation.y = odomAftMapped.pose.pose.position.y;
	transform.transform.translation.z = odomAftMapped.pose.pose.position.z;

	transform.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
	transform.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
	transform.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
	transform.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;

	// br_.sendTransform(transform);
	// br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "lidar", "wheel"));
}

void LocalizationModule::publish_transform(const Eigen::Isometry3d& correction, const std::string& parent,
										   const std::string& child) {
	Eigen::Vector3d pos = correction.translation();
	Eigen::Quaterniond quaternion = Eigen::Quaterniond(correction.matrix().block<3, 3>(0, 0));
	nav_msgs::msg::Odometry transformToPub;
	transformToPub.pose.pose.position.x = pos(0);
	transformToPub.pose.pose.position.y = pos(1);
	transformToPub.pose.pose.position.z = pos(2);
	transformToPub.pose.pose.orientation.x = quaternion.x();
	transformToPub.pose.pose.orientation.y = quaternion.y();
	transformToPub.pose.pose.orientation.z = quaternion.z();
	transformToPub.pose.pose.orientation.w = quaternion.w();
	// static tf::TransformBroadcaster br;
	// tf::Transform transform;
	// tf::Quaternion q;
	// transform.setOrigin(tf::Vector3(transformToPub.pose.pose.position.x,
	//                                 transformToPub.pose.pose.position.y,
	//                                 transformToPub.pose.pose.position.z));
	// q.setW(transformToPub.pose.pose.orientation.w);
	// q.setX(transformToPub.pose.pose.orientation.x);
	// q.setY(transformToPub.pose.pose.orientation.y);
	// q.setZ(transformToPub.pose.pose.orientation.z);
	// transform.setRotation(q);

	geometry_msgs::msg::TransformStamped transform;

	transform.header.stamp = node_->now();
	transform.header.frame_id = parent; // 替换为实际的父坐标系
	transform.child_frame_id = child;	// 替换为实际的子坐标系

	transform.transform.translation.x = transformToPub.pose.pose.position.x;
	transform.transform.translation.y = transformToPub.pose.pose.position.y;
	transform.transform.translation.z = transformToPub.pose.pose.position.z;

	transform.transform.rotation.w = transformToPub.pose.pose.orientation.w;
	transform.transform.rotation.x = transformToPub.pose.pose.orientation.x;
	transform.transform.rotation.y = transformToPub.pose.pose.orientation.y;
	transform.transform.rotation.z = transformToPub.pose.pose.orientation.z;

	// br_.sendTransform(transform);
	// br.sendTransform(tf::StampedTransform(transform, ros::Time().now(), parent, child));
}

/*
void LocalizationModule::publish_lidar_to_map(const Eigen::Isometry3d& lidar_in_map)
{
	Eigen::Vector3d pos = lidar_in_map.translation();
	Eigen::Quaterniond quaternion = Eigen::Quaterniond(lidar_in_map.matrix().block<3, 3>(0, 0));
	nav_msgs::msg::Odometry transformToPub;
	transformToPub.pose.pose.position.x = pos(0);
	transformToPub.pose.pose.position.y = pos(1);
	transformToPub.pose.pose.position.z = pos(2);
	transformToPub.pose.pose.orientation.x = quaternion.x();
	transformToPub.pose.pose.orientation.y = quaternion.y();
	transformToPub.pose.pose.orientation.z = quaternion.z();
	transformToPub.pose.pose.orientation.w = quaternion.w();
	// static tf::TransformBroadcaster br;
	// tf::Transform transform;
	// tf::Quaternion q;
	// transform.setOrigin(tf::Vector3(transformToPub.pose.pose.position.x,
	//                                 transformToPub.pose.pose.position.y,
	//                                 transformToPub.pose.pose.position.z));
	// q.setW(transformToPub.pose.pose.orientation.w);
	// q.setX(transformToPub.pose.pose.orientation.x);
	// q.setY(transformToPub.pose.pose.orientation.y);
	// q.setZ(transformToPub.pose.pose.orientation.z);
	// transform.setRotation(q);

	// br.sendTransform(tf::StampedTransform(transform, ros::Time().now(), "map", "lidar"));
}
*/

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

void LocalizationModule::visualizeLoopClosure(const std::map<int, int>& loopIndexContainer, Path& optimized_path_msg) {
	string odometryFrame = "odom";

	if (loopIndexContainer.empty()) return;

	visualization_msgs::msg::MarkerArray markerArray;
	// 闭环顶点
	visualization_msgs::msg::Marker markerNode;
	markerNode.header.frame_id = odometryFrame;
	markerNode.header.stamp = node_->now(); // timeLaserInfoStamp;
	markerNode.action = visualization_msgs::msg::Marker::ADD;
	markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
	markerNode.ns = "loop_nodes";
	markerNode.id = 0;
	markerNode.pose.orientation.w = 1;
	markerNode.scale.x = 0.3;
	markerNode.scale.y = 0.3;
	markerNode.scale.z = 0.3;
	markerNode.color.r = 0;
	markerNode.color.g = 0.8;
	markerNode.color.b = 1;
	markerNode.color.a = 1;
	// 闭环边
	visualization_msgs::msg::Marker markerEdge;
	markerEdge.header.frame_id = odometryFrame;
	markerEdge.header.stamp = node_->now(); // timeLaserInfoStamp;;
	markerEdge.action = visualization_msgs::msg::Marker::ADD;
	markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
	markerEdge.ns = "loop_edges";
	markerEdge.id = 1;
	markerEdge.pose.orientation.w = 1;
	markerEdge.scale.x = 0.1;
	markerEdge.color.r = 0.9;
	markerEdge.color.g = 0.9;
	markerEdge.color.b = 0;
	markerEdge.color.a = 1;

	int loop_i = 0;
	// 遍历闭环
	for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it) {
		int key_cur = it->first;
		int key_pre = it->second;
		geometry_msgs::msg::Point p;
		p.x = optimized_path_msg.poses[key_cur].pose.position.x;
		p.y = optimized_path_msg.poses[key_cur].pose.position.y;
		p.z = optimized_path_msg.poses[key_cur].pose.position.z;

		markerNode.points.push_back(p);
		markerEdge.points.push_back(p);
		p.x = optimized_path_msg.poses[key_pre].pose.position.x;
		p.y = optimized_path_msg.poses[key_pre].pose.position.y;
		p.z = optimized_path_msg.poses[key_pre].pose.position.z;
		markerNode.points.push_back(p);
		markerEdge.points.push_back(p);
	}

	markerArray.markers.push_back(markerNode);
	markerArray.markers.push_back(markerEdge);
	pubLoopConstraintEdge->publish(markerArray);
}

void LocalizationModule::show_keyframe(
	const std::vector<lidar_slam::ScInfo, Eigen::aligned_allocator<lidar_slam::ScInfo>>& loadKeyframe) {
	visualization_msgs::msg::MarkerArray MarkerArray; //定义MarkerArray对象
	int number = loadKeyframe.size();				  // object_in为输入的目标个数
	for (int i = 0; i < number; i++) {
		if (i % 10 != 0) {
			continue;
		}
		visualization_msgs::msg::Marker Marker; //定义Marker对象
		Marker.header.frame_id = "map";
		Marker.header.stamp = node_->now();
		Marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING; //选用文本类型
		Marker.ns = "basic_shapes";										 //必写，否则rviz无法显示
		Marker.pose.orientation.w = 1.0;								 //文字的方向
		Marker.id =
			i; //用来标记同一帧不同的对象，如果后面的帧的对象少于前面帧的对象，那么少的id将在rviz中残留，所以需要后续的实时更新程序
		Marker.scale.x = 1.5;
		Marker.scale.y = 1.5;
		Marker.scale.z = 1.5; //文字的大小
		Marker.color.b = 25;
		Marker.color.g = 0;
		Marker.color.r = 25; //文字的颜色
		Marker.color.a = 1;	 //必写，否则rviz无法显示
		geometry_msgs::msg::Pose pose;
		pose.position.x = loadKeyframe[i].pose.translation().x();
		pose.position.y = loadKeyframe[i].pose.translation().y();
		pose.position.z = loadKeyframe[i].pose.translation().z();
		ostringstream str;
		//     str<< loadKeyframe[i].id << " " << loadKeyframe[i].pose.translation().x() << " " <<
		//     loadKeyframe[i].pose.translation().y() << " " << loadKeyframe[i].pose.translation().z();
		str << loadKeyframe[i].id;
		Marker.text = str.str(); //文字内容
		Marker.pose = pose;		 //文字的位置
		MarkerArray.markers.push_back(Marker);
	}
	pubKeyframePose->publish(MarkerArray);
}

// void LocalizationModule::pub_odom_cloud(PointCloudType::Ptr msg_in, ros::Publisher pubOdomCloud)
// {
// 	sensor_msgs::PointCloud2 laserCloudmsg;
// 	pcl::toROSMsg(*msg_in, laserCloudmsg);
// 	laserCloudmsg.header.stamp = ros::Time().now();
// 	laserCloudmsg.header.frame_id = "odom";
// 	pubOdomCloud.publish(laserCloudmsg);
// }

// void LocalizationModule::pub_lidar_cloud(PointCloudType::Ptr msg_in, ros::Publisher pubBodyCloud)
// {
// 	sensor_msgs::PointCloud2 laserCloudmsg;
// 	pcl::toROSMsg(*msg_in, laserCloudmsg);
// 	laserCloudmsg.header.stamp = ros::Time().now();
// 	laserCloudmsg.header.frame_id = "lidar";
// 	// laserCloudmsg.header.frame_id = "base_footprint";
// 	pubBodyCloud.publish(laserCloudmsg);
// }

// void LocalizationModule::pub_kdtree_cloud(PointCloudType::Ptr msg_in, ros::Publisher pubKdtreeCloud)
// {
// 	sensor_msgs::PointCloud2 laserCloudmsg;
// 	pcl::toROSMsg(*msg_in, laserCloudmsg);
// 	laserCloudmsg.header.stamp = ros::Time().now();
// 	laserCloudmsg.header.frame_id = "odom";
// 	pubKdtreeCloud.publish(laserCloudmsg);
// }

// void LocalizationModule::pub_obstacle_cloud(PointCloudType::Ptr msg_in, ros::Publisher pubObstacleCloud)
// {
// 	sensor_msgs::PointCloud2 laserCloudmsg;
// 	pcl::toROSMsg(*msg_in, laserCloudmsg);
// 	laserCloudmsg.header.stamp = ros::Time().now();
// 	laserCloudmsg.header.frame_id = "wheel";
// 	pubObstacleCloud.publish(laserCloudmsg);
// }

// void LocalizationModule::pub_rgb_map(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud, ros::Publisher pubRgbCloud){
//   sensor_msgs::PointCloud2 pub_cloud;
//   pcl::toROSMsg(*rgb_cloud, pub_cloud);
//   pub_cloud.header.frame_id = "odom";
//   pub_cloud.header.stamp = ros::Time().now();
//   pubRgbCloud.publish(pub_cloud);
// }

// void LocalizationModule::pub_filtered_obstacle_cloud(PointCloudType::Ptr msg_in, ros::Publisher
// pubFilteredObstacleCloud)
// {
// 	sensor_msgs::PointCloud2 laserCloudmsg;
// 	pcl::toROSMsg(*msg_in, laserCloudmsg);
// 	laserCloudmsg.header.stamp = ros::Time().now();
// 	laserCloudmsg.header.frame_id = "wheel";
// 	pubFilteredObstacleCloud.publish(laserCloudmsg);
// }

// void LocalizationModule::publish_odometry(const Eigen::Isometry3d lidar_in_odom, ros::Publisher pubOdomAftMapped)
// {
// 	nav_msgs::Odometry odomAftMapped;
//     odomAftMapped.header.frame_id = "odom";
//     odomAftMapped.child_frame_id = "lidar";
//     odomAftMapped.header.stamp = ros::Time().now(); // ros::Time().fromSec(lidar_end_time);
// 	odomAftMapped.pose.pose.position.x = lidar_in_odom.translation().x();
//     odomAftMapped.pose.pose.position.y = lidar_in_odom.translation().y();
//     odomAftMapped.pose.pose.position.z = lidar_in_odom.translation().z();
// 	Eigen::Quaterniond quaternion = Eigen::Quaterniond(lidar_in_odom.rotation());
//     odomAftMapped.pose.pose.orientation.x = quaternion.x();
//     odomAftMapped.pose.pose.orientation.y = quaternion.y();
//     odomAftMapped.pose.pose.orientation.z = quaternion.z();
//     odomAftMapped.pose.pose.orientation.w = quaternion.w();
//     pubOdomAftMapped.publish(odomAftMapped);
//     static tf::TransformBroadcaster br;
//     tf::Transform transform;
//     tf::Quaternion q;
//     transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
//                                     odomAftMapped.pose.pose.position.y,
//                                     odomAftMapped.pose.pose.position.z));
//     q.setW(odomAftMapped.pose.pose.orientation.w);
//     q.setX(odomAftMapped.pose.pose.orientation.x);
//     q.setY(odomAftMapped.pose.pose.orientation.y);
//     q.setZ(odomAftMapped.pose.pose.orientation.z);
//     transform.setRotation(q);
//     // br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "odom", "lidar"));
// }

} // namespace localization_module

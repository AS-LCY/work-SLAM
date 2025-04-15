#include "node/localization_module.h"


namespace localization_module{

void LocalizationModule::publish_cloud(PointCloudType::Ptr pcl_cloud_in, std::string frame_id, ros::Time ros_time, ros::Publisher pub_cloud){

	sensor_msgs::PointCloud2 ros_cloud_msg;
	pcl::toROSMsg(*pcl_cloud_in, ros_cloud_msg);
	ros_cloud_msg.header.stamp = ros_time;
	ros_cloud_msg.header.frame_id = frame_id;
	pub_cloud.publish(ros_cloud_msg);
}


void LocalizationModule::pub_test_cloud(PointCloudType::Ptr msg_in, bool localization_mode,ros::Publisher pubTestCloud)
{
	sensor_msgs::PointCloud2 laserCloudmsg;
	pcl::toROSMsg(*msg_in, laserCloudmsg);
	laserCloudmsg.header.stamp = ros::Time().now();
    if (localization_mode)
	   laserCloudmsg.header.frame_id = "map";
    else
       laserCloudmsg.header.frame_id = "wheel";
	pubTestCloud.publish(laserCloudmsg);
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

void LocalizationModule::publish_odometry_lidar_in_map(const Eigen::Isometry3d lidar_in_map, lidar_slam::Localization_base curr_pose, string frameid, string child_frameid, ModuleStatus curr_running_module_status, ros::Publisher pubOdomAftMapped)
{
	nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = frameid;
    odomAftMapped.child_frame_id = child_frameid;
    odomAftMapped.header.stamp = ros::Time().now(); // ros::Time().fromSec(lidar_end_time);
    // odomAftMapped.header.stamp = ros::Time().fromSec(lidar_time_);
	odomAftMapped.pose.pose.position.x = lidar_in_map.translation().x();
    odomAftMapped.pose.pose.position.y = lidar_in_map.translation().y();
    odomAftMapped.pose.pose.position.z = lidar_in_map.translation().z();
	Eigen::Quaterniond quaternion = Eigen::Quaterniond(lidar_in_map.rotation());
    odomAftMapped.pose.pose.orientation.x = quaternion.x();
    odomAftMapped.pose.pose.orientation.y = quaternion.y();
    odomAftMapped.pose.pose.orientation.z = quaternion.z();
    odomAftMapped.pose.pose.orientation.w = quaternion.w();

    if(curr_running_module_status == ModuleStatus::MODULE_LOCALIZATION){
        odomAftMapped.pose.covariance[1] = 3;
    }else if(is_mapping_status(curr_running_module_status)){
        odomAftMapped.pose.covariance[1] = 2;
    }

    Eigen::Isometry3d iso_transform = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d mat = curr_pose.imu_state.rot.matrix();
    iso_transform.linear() = mat;
    Eigen::Isometry3d iso_transform_inv = iso_transform.inverse();
    Eigen::Matrix3d rot = iso_transform_inv.linear();

    auto vel = rot * curr_pose.imu_state.vel;

    odomAftMapped.twist.twist.linear.x = vel[0];
    odomAftMapped.twist.twist.linear.y = vel[1];
    odomAftMapped.twist.twist.linear.z = vel[2];

    // log_info_manager_->log_info.slam_vel_x = odomAftMapped.twist.twist.linear.x;
    log_info_manager_->slam_info.data[9] = odomAftMapped.twist.twist.linear.x; // slam_vel_x

    pubOdomAftMapped.publish(odomAftMapped);


    // auto odom_for_tf = odomAftMapped;
    // // auto odom_for_tf = filter_odometry_;

    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // tf::Quaternion q;
    // transform.setOrigin(tf::Vector3(odom_for_tf.pose.pose.position.x,
    //                                 odom_for_tf.pose.pose.position.y,
    //                                 odom_for_tf.pose.pose.position.z));
    // q.setW(odom_for_tf.pose.pose.orientation.w);
    // q.setX(odom_for_tf.pose.pose.orientation.x);
    // q.setY(odom_for_tf.pose.pose.orientation.y);
    // q.setZ(odom_for_tf.pose.pose.orientation.z);
    // transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, odom_for_tf.header.stamp, frameid, child_frameid));
}

void LocalizationModule::publish_odometry(const Eigen::Isometry3d isometry_3d, std::string frameid, std::string child_frameid, ros::Publisher pub_odom)
{
    // cout<<"********************* pub odometry "<<endl;
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

void LocalizationModule::publish_static_transform(const Eigen::Isometry3d wheel_in_lidar)
{
	nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "lidar";
    odomAftMapped.child_frame_id = "wheel";
    odomAftMapped.header.stamp = ros::Time().now(); // ros::Time().fromSec(lidar_end_time);
	odomAftMapped.pose.pose.position.x = wheel_in_lidar.translation().x();
    odomAftMapped.pose.pose.position.y = wheel_in_lidar.translation().y();
    odomAftMapped.pose.pose.position.z = wheel_in_lidar.translation().z();
	Eigen::Quaterniond quaternion = Eigen::Quaterniond(wheel_in_lidar.rotation());
    odomAftMapped.pose.pose.orientation.x = quaternion.x();
    odomAftMapped.pose.pose.orientation.y = quaternion.y();
    odomAftMapped.pose.pose.orientation.z = quaternion.z();
    odomAftMapped.pose.pose.orientation.w = quaternion.w();
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                    odomAftMapped.pose.pose.position.y,
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "lidar", "wheel"));
}

void LocalizationModule::publish_transform(const Eigen::Isometry3d& correction,string parent, string child)
{
    Eigen::Vector3d pos = correction.translation();
    Eigen::Quaterniond quaternion = Eigen::Quaterniond(correction.matrix().block<3, 3>(0, 0));
    nav_msgs::Odometry transformToPub;
    transformToPub.pose.pose.position.x = pos(0);
    transformToPub.pose.pose.position.y = pos(1);
    transformToPub.pose.pose.position.z = pos(2);
    transformToPub.pose.pose.orientation.x = quaternion.x();
    transformToPub.pose.pose.orientation.y = quaternion.y();
    transformToPub.pose.pose.orientation.z = quaternion.z();
    transformToPub.pose.pose.orientation.w = quaternion.w();
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(transformToPub.pose.pose.position.x,
                                    transformToPub.pose.pose.position.y,
                                    transformToPub.pose.pose.position.z));
    q.setW(transformToPub.pose.pose.orientation.w);
    q.setX(transformToPub.pose.pose.orientation.x);
    q.setY(transformToPub.pose.pose.orientation.y);
    q.setZ(transformToPub.pose.pose.orientation.z);
    transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time().now(), parent, child));
}

void LocalizationModule::publish_lidar_to_map(const Eigen::Isometry3d& lidar_in_map, ros::Publisher pubOdomCloud)
{
  //  std::cout << lidar_in_map.translation().transpose()<<std::endl;
    Eigen::Vector3d pos = lidar_in_map.translation();
    Eigen::Quaterniond quaternion = Eigen::Quaterniond(lidar_in_map.matrix().block<3, 3>(0, 0));
    nav_msgs::Odometry transformToPub;
    transformToPub.pose.pose.position.x = pos(0);
    transformToPub.pose.pose.position.y = pos(1);
    transformToPub.pose.pose.position.z = pos(2);
    transformToPub.pose.pose.orientation.x = quaternion.x();
    transformToPub.pose.pose.orientation.y = quaternion.y();
    transformToPub.pose.pose.orientation.z = quaternion.z();
    transformToPub.pose.pose.orientation.w = quaternion.w();
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(transformToPub.pose.pose.position.x,
                                    transformToPub.pose.pose.position.y,
                                    transformToPub.pose.pose.position.z));
    q.setW(transformToPub.pose.pose.orientation.w);
    q.setX(transformToPub.pose.pose.orientation.x);
    q.setY(transformToPub.pose.pose.orientation.y);
    q.setZ(transformToPub.pose.pose.orientation.z);
    transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time().now(), "map", "lidar"));
}

void LocalizationModule::visualizeLoopClosure(map<int, int> loopIndexContainer, nav_msgs::Path optimized_path_msg, ros::Publisher pubLoopConstraintEdge)
{
    // ROS_ERROR_STREAM(RED << "visualizeLoopClosure" << RESET);
    ros::Time timeLaserInfoStamp = ros::Time().now(); //  时间戳
    string odometryFrame = "odom";

    if (loopIndexContainer.empty())
        return;

    // ROS_ERROR_STREAM(RED << "visualizeLoopClosure" << RESET);
    visualization_msgs::MarkerArray markerArray;
    // 闭环顶点
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = odometryFrame;
    markerNode.header.stamp = timeLaserInfoStamp;
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
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
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = odometryFrame;
    markerEdge.header.stamp = timeLaserInfoStamp;
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9;
    markerEdge.color.g = 0.9;
    markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    // ROS_ERROR_STREAM(RED << "visualizeLoopClosure before for loop " << RESET);

    int loop_i=0;
    // 遍历闭环
    for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
    {
        // ROS_ERROR_STREAM(RED << "loop_i == %d --- 1", loop_i << RESET);
        int key_cur = it->first;
        int key_pre = it->second;
        // ROS_ERROR_STREAM(RED << "loop_i == %d --- 2", loop_i << RESET);
        geometry_msgs::Point p;
        p.x = optimized_path_msg.poses[key_cur].pose.position.x;
        p.y = optimized_path_msg.poses[key_cur].pose.position.y;
        p.z = optimized_path_msg.poses[key_cur].pose.position.z;
        // ROS_ERROR_STREAM(RED << "loop_i == %d --- 3", loop_i << RESET);
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        p.x = optimized_path_msg.poses[key_pre].pose.position.x;
        p.y = optimized_path_msg.poses[key_pre].pose.position.y;
        p.z = optimized_path_msg.poses[key_pre].pose.position.z;
        // ROS_ERROR_STREAM(RED << "loop_i == %d --- 4", loop_i << RESET);
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        // ROS_ERROR_STREAM(RED << "loop_i == %d --- 5", loop_i ++ << RESET);
    }
    // ROS_ERROR_STREAM(RED << "visualizeLoopClosure before pub " << RESET);

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubLoopConstraintEdge.publish(markerArray);

    // ROS_ERROR_STREAM(RED << "visualizeLoopClosure success "<< RESET);
}

void LocalizationModule::show_keyframe(std::vector<lidar_slam::ScInfo> loadKeyframe, ros::Publisher pubKeyframePose){
    visualization_msgs::MarkerArray MarkerArray;//定义MarkerArray对象
    int number = loadKeyframe.size();//object_in为输入的目标个数
	for(int i = 0; i < number; i++)
	{
        if (i%10 != 0)
            continue;
		visualization_msgs::Marker Marker;//定义Marker对象
        Marker.header.frame_id = "map";
        Marker.header.stamp = ros::Time().now();
		Marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;//选用文本类型
		Marker.ns = "basic_shapes";//必写，否则rviz无法显示
		Marker.pose.orientation.w = 1.0;//文字的方向
		Marker.id =i;//用来标记同一帧不同的对象，如果后面的帧的对象少于前面帧的对象，那么少的id将在rviz中残留，所以需要后续的实时更新程序
		Marker.scale.x = 1.5;
		Marker.scale.y = 1.5;
		Marker.scale.z = 1.5;//文字的大小
		Marker.color.b = 25;
		Marker.color.g = 0;
		Marker.color.r = 25;//文字的颜色
		Marker.color.a = 1;//必写，否则rviz无法显示
        geometry_msgs::Pose pose;
        pose.position.x = loadKeyframe[i].pose.translation().x();
        pose.position.y = loadKeyframe[i].pose.translation().y();
        pose.position.z = loadKeyframe[i].pose.translation().z();
        ostringstream str;
   //     str<< loadKeyframe[i].id << " " << loadKeyframe[i].pose.translation().x() << " " << loadKeyframe[i].pose.translation().y() << " " << loadKeyframe[i].pose.translation().z();
        str<< loadKeyframe[i].id;
        Marker.text=str.str();//文字内容
        Marker.pose=pose;//文字的位置
        MarkerArray.markers.push_back(Marker);
	}
    pubKeyframePose.publish(MarkerArray);
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



// void LocalizationModule::pub_filtered_obstacle_cloud(PointCloudType::Ptr msg_in, ros::Publisher pubFilteredObstacleCloud)
// {
// 	sensor_msgs::PointCloud2 laserCloudmsg;
// 	pcl::toROSMsg(*msg_in, laserCloudmsg);
// 	laserCloudmsg.header.stamp = ros::Time().now();
// 	laserCloudmsg.header.frame_id = "wheel";
// 	pubFilteredObstacleCloud.publish(laserCloudmsg);
// }


// void LocalizationModule::publish_odometry(const Eigen::Isometry3d lidar_in_odom, ros::Publisher pubOdomAftMapped)
// {
//     // cout<<"********************* pub odometry "<<endl;
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



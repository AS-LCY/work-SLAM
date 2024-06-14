#include <string>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include "lidar_slam.hpp"
#include <cstdlib>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <Viewer.hpp>
#include "v4l2cam.h"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
std::unique_ptr<lidar_slam::LidarSlam> slam;
ros::Publisher pubOdomCloud;
ros::Publisher pubBodyCloud;
ros::Publisher pubObstacleCloud;
ros::Publisher pubFilteredObstacleCloud;
ros::Publisher pubTestCloud;
ros::Publisher pubKdtreeCloud;
ros::Publisher pubOptimizedPath;
ros::Publisher pubUnoptimizedPath; 
ros::Publisher pubLoopConstraintEdge;
ros::Publisher pubOdomAftMapped;
ros::Publisher pubLoadMap;
ros::Publisher pubKeyframePose;
ros::Publisher pubRgbCloud;
//ros::Publisher image_pub;
nav_msgs::Path unoptimized_path_msg;
nav_msgs::Path optimized_path_msg;
ros::ServiceServer srvSaveMap;
int show_load_map = 0;
lidar_slam::Control_status control_status;
bool localization_mode = false;
bool offline_mode = false;
bool just_show_mode = false;
string log_folder;
bool show_rviz = false;
bool fast_mode = false;
std::vector<Eigen::Isometry3d> keyPoses;


void livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg_in)
{
    if(control_status.reset||offline_mode)
       return;
	std::shared_ptr<livox_ros::LidarMsg> msg(new livox_ros::LidarMsg);
	msg->time_stamp = msg_in->header.stamp.toSec();
    msg->point_num = msg_in->point_num;
   // msg->lidar_id;
  //  msg->rsvd[3];
    msg->points.resize(msg_in->points.size());
    for (int i =0; i<msg_in->points.size(); i++){
		msg->points[i].x = msg_in->points[i].x;
        msg->points[i].y = msg_in->points[i].y;
        msg->points[i].z = msg_in->points[i].z;
        msg->points[i].reflectivity = msg_in->points[i].reflectivity;
        msg->points[i].offset_time = msg_in->points[i].offset_time;
		msg->points[i].tag = msg_in->points[i].tag;
        msg->points[i].line = msg_in->points[i].line;
	}
	slam -> livox_pcl_cbk(msg);
    
}
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    if(control_status.reset||offline_mode)
       return;

    std::shared_ptr<livox_ros::ImuMsg> msg(new livox_ros::ImuMsg);
	msg->time_stamp = msg_in->header.stamp.toSec();
	msg->angular_velocity << msg_in->angular_velocity.x,msg_in->angular_velocity.y,msg_in->angular_velocity.z;
	msg->linear_acceleration << msg_in->linear_acceleration.x,msg_in->linear_acceleration.y,msg_in->linear_acceleration.z;	
	slam -> imu_cbk(msg);
}
void pub_odom_cloud(PointCloudXYZI::Ptr msg_in)
{
	sensor_msgs::PointCloud2 laserCloudmsg;
	pcl::toROSMsg(*msg_in, laserCloudmsg);
	laserCloudmsg.header.stamp = ros::Time().now();
	laserCloudmsg.header.frame_id = "odom";
	pubOdomCloud.publish(laserCloudmsg);
}

void pub_lidar_cloud(PointCloudXYZI::Ptr msg_in)
{
	sensor_msgs::PointCloud2 laserCloudmsg;
	pcl::toROSMsg(*msg_in, laserCloudmsg);
	laserCloudmsg.header.stamp = ros::Time().now();
	laserCloudmsg.header.frame_id = "lidar";
	pubBodyCloud.publish(laserCloudmsg);
}

void pub_obstacle_cloud(PointCloudXYZI::Ptr msg_in)
{
	sensor_msgs::PointCloud2 laserCloudmsg;
	pcl::toROSMsg(*msg_in, laserCloudmsg);
	laserCloudmsg.header.stamp = ros::Time().now();
	laserCloudmsg.header.frame_id = "wheel";
	pubObstacleCloud.publish(laserCloudmsg);
}

void pub_filtered_obstacle_cloud(PointCloudXYZI::Ptr msg_in)
{
	sensor_msgs::PointCloud2 laserCloudmsg;
	pcl::toROSMsg(*msg_in, laserCloudmsg);
	laserCloudmsg.header.stamp = ros::Time().now();
	laserCloudmsg.header.frame_id = "wheel";
	pubFilteredObstacleCloud.publish(laserCloudmsg);
}

void pub_test_cloud(PointCloudXYZI::Ptr msg_in)
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

void pub_kdtree_cloud(PointCloudXYZI::Ptr msg_in)
{
	sensor_msgs::PointCloud2 laserCloudmsg;
	pcl::toROSMsg(*msg_in, laserCloudmsg);
	laserCloudmsg.header.stamp = ros::Time().now();
	laserCloudmsg.header.frame_id = "odom";
	pubKdtreeCloud.publish(laserCloudmsg);
}

void publish_unoptimized_path(const std::deque<Eigen::Isometry3d> path)
{
	geometry_msgs::PoseStamped msg;

	int size = path.size();
	unoptimized_path_msg.poses.clear();
    unoptimized_path_msg.header.stamp = ros::Time().now();
    unoptimized_path_msg.header.frame_id = "odom";
	for (int i = 0 ; i <path.size();i++){
		msg.header.stamp = ros::Time().now();
		msg.header.frame_id = "odom";
		msg.pose.position.x = path[i].translation().x();
		msg.pose.position.y = path[i].translation().y();
		msg.pose.position.z = path[i].translation().z();
		/*Eigen::Quaterniond quaternion = path[i].rotation();
		msg.pose.orientation.x = quaternion.x();
		msg.pose.orientation.y = quaternion.y();
		msg.pose.orientation.z = quaternion.z();
		msg.pose.orientation.w = quaternion.w();*/
		unoptimized_path_msg.poses.push_back(msg);
	}
    pubUnoptimizedPath.publish(unoptimized_path_msg);
}

void publish_optimized_path(const std::vector<Eigen::Isometry3d> path,std::string frame)
{
	geometry_msgs::PoseStamped msg;
    optimized_path_msg.poses.clear();
    optimized_path_msg.header.stamp = ros::Time().now();
    optimized_path_msg.header.frame_id = frame;
	int size = path.size();
	for (int i = 0 ; i <path.size();i++){
		msg.header.stamp = ros::Time().now();
		msg.header.frame_id = frame;
		msg.pose.position.x = path[i].translation().x();
		msg.pose.position.y = path[i].translation().y();
		msg.pose.position.z = path[i].translation().z();
		/*Eigen::Quaterniond quaternion = path[i].rotation();
		msg.pose.orientation.x = quaternion.x();
		msg.pose.orientation.y = quaternion.y();
		msg.pose.orientation.z = quaternion.z();
		msg.pose.orientation.w = quaternion.w();*/
		optimized_path_msg.poses.push_back(msg);
	}
    pubOptimizedPath.publish(optimized_path_msg);
}

void publish_odometry(const Eigen::Isometry3d lidar_in_odom)
{
	nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "odom";
    odomAftMapped.child_frame_id = "lidar";
    odomAftMapped.header.stamp = ros::Time().now(); // ros::Time().fromSec(lidar_end_time);
	odomAftMapped.pose.pose.position.x = lidar_in_odom.translation().x();
    odomAftMapped.pose.pose.position.y = lidar_in_odom.translation().y();
    odomAftMapped.pose.pose.position.z = lidar_in_odom.translation().z();
	Eigen::Quaterniond quaternion = Eigen::Quaterniond(lidar_in_odom.rotation());
    odomAftMapped.pose.pose.orientation.x = quaternion.x();
    odomAftMapped.pose.pose.orientation.y = quaternion.y();
    odomAftMapped.pose.pose.orientation.z = quaternion.z();
    odomAftMapped.pose.pose.orientation.w = quaternion.w();
    pubOdomAftMapped.publish(odomAftMapped);
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
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "odom", "lidar"));
}

void publish_static_transform(const Eigen::Isometry3d wheel_in_lidar)
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
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "lidar", "wheel"));
}

void publish_transform(const Eigen::Isometry3d& correction,string parent, string child)
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
    br.sendTransform(tf::StampedTransform(transform, ros::Time().now(), parent, child));
}

void publish_lidar_to_map(const Eigen::Isometry3d& lidar_in_map)
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
    br.sendTransform(tf::StampedTransform(transform, ros::Time().now(), "map", "lidar"));
}

void visualizeLoopClosure(map<int, int> loopIndexContainer)
{
    ros::Time timeLaserInfoStamp = ros::Time().now(); //  时间戳
    string odometryFrame = "odom";

    if (loopIndexContainer.empty())
        return;

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

    // 遍历闭环
    for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
    {
        int key_cur = it->first;
        int key_pre = it->second;
        geometry_msgs::Point p;
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
    pubLoopConstraintEdge.publish(markerArray);
}
void command_cbk(const std_msgs::Int32 &msg_in)
{
    switch(msg_in.data){
        case 0:
            printf("save_map\n");
            slam -> save_map(CURRENT_DIR+std::string("/lib/map/"),0.1);
            break;
        case 1:
            printf("load map\n");
            slam -> load_map(CURRENT_DIR+std::string("/lib/map/"));
            break;
        default:
           break;
    }

}

void show_keyframe(std::vector<ScInfo> loadKeyframe){
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
void pub_rgb_map(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud){
  sensor_msgs::PointCloud2 pub_cloud;
  pcl::toROSMsg(*rgb_cloud, pub_cloud);
  pub_cloud.header.frame_id = "odom";
  pub_cloud.header.stamp = ros::Time().now();
  pubRgbCloud.publish(pub_cloud);
}


void showThread()
{
    const int frequency = 5.0; // 频率为1Hz
    const std::chrono::milliseconds period(1000 / frequency);
    lidar_slam::Viewer test_view(localization_mode);
    while (ros::ok())
    {
        auto start = std::chrono::steady_clock::now();
        if (!control_status.reset){
            test_view.Start();
            control_status = test_view.getControl();
            if (!localization_mode){
                std::vector<Eigen::Isometry3d> optimized_poses = slam->get_optimized_path();
                test_view.DrawTrajectory(optimized_poses,Eigen::Vector3f(0,1,0));
                test_view.DrawTrajectory(slam->get_unoptimized_path(),Eigen::Vector3f(1,0,0));
                map<int, int> loopIndex = slam->getloopIndex();
                for (auto it = loopIndex.begin(); it != loopIndex.end(); ++it) {
                     test_view.DrawLine(optimized_poses[it->first],optimized_poses[it->second],Eigen::Vector3f(0,0,0));
                }
                
                if (control_status.showMap)
                    test_view.DrawCloud(slam->getCurrentMap(),Eigen::Vector3f(0,0,1),1);
                if (control_status.showLidar)
                   test_view.DrawCloud(slam->get_odom_cloud(),slam->getOdomToMap(),Eigen::Vector3f(1,0,0),2);
                if (control_status.showObstacle)
                    test_view.DrawCloud(slam->getFilteredObstacleCloud(),slam->getWheelInMap(),Eigen::Vector3f(0,1,0),2.0);
                test_view.DrawPose(slam->getWheelInMap());
            }
            else{
                if (control_status.showMap)
                   test_view.DrawCloud(slam->getLoadMapPoints(),Eigen::Vector3f(0,0,1),1.0);
                if (control_status.showLidar)
                   test_view.DrawCloud(slam->get_lidar_cloud(),slam->getLidarInMap(),Eigen::Vector3f(1,0,0),2.0);
                if (control_status.showObstacle)
                    test_view.DrawCloud(slam->getFilteredObstacleCloud(),slam->getWheelInMap(),Eigen::Vector3f(0,1,0),2.0);
                if (slam->isGloalLocalizationSuccess())
                    test_view.DrawPose(slam->getWheelInMap());
                test_view.DrawTrajectory(slam->get_unoptimized_path(),Eigen::Vector3f(1,0,0));
            }

            if (control_status.saveMap && !localization_mode)
                slam -> save_map(CURRENT_DIR+std::string("/map/"),0.1);
            test_view.Finish();  
        }
        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        if (elapsed < period)
        {
            std::this_thread::sleep_for(period - elapsed);
        }
    }
}

int main(int argc,char **argv)
{

    ROS_INFO("start interface");
	ros::init(argc,argv,"slam_interface");		
	ros::NodeHandle nh;
	ros::Rate loop_rate(200);
    nh.param<bool>("localization_mode", localization_mode, false);
    nh.param<bool>("just_show_mode", just_show_mode, false);
    nh.param<bool>("offline_mode", offline_mode, false);
    nh.param<bool>("show_rviz", show_rviz, false);
    nh.param<bool>("fast", fast_mode, false);
    nh.param<string>("log_folder", log_folder, " ");
 //   nh.param<int>("max_iteration", NUM_MAX_ITERATIONS, 4)

    pubOdomCloud = nh.advertise<sensor_msgs::PointCloud2>("/odom_cloud", 100000);  
	pubBodyCloud = nh.advertise<sensor_msgs::PointCloud2>("/body_cloud", 100000);
    pubObstacleCloud = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_cloud", 100000);
    pubFilteredObstacleCloud = nh.advertise<sensor_msgs::PointCloud2>("/filtered_obstacle_cloud", 100000);
    pubTestCloud = nh.advertise<sensor_msgs::PointCloud2>("/test_cloud", 100000);
	pubKdtreeCloud = nh.advertise<sensor_msgs::PointCloud2>("/kdtree_cloud", 100000); 
	pubOptimizedPath= nh.advertise<nav_msgs::Path>("/optimized_path", 1000);
    pubUnoptimizedPath= nh.advertise<nav_msgs::Path>("/unoptimized_path", 1000);
	pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/loop_closure_constraints", 1);
    pubKeyframePose = nh.advertise<visualization_msgs::MarkerArray>("/key_frame_pose", 1);
	pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    pubLoadMap = nh.advertise<sensor_msgs::PointCloud2>("/Load_map", 100000);
    pubRgbCloud= nh.advertise<sensor_msgs::PointCloud2>("rgb_cloud", 1);
  //  image_pub = nh.advertise<sensor_msgs::Image>("fisheye_image", 1);

    ROS_INFO("create lidar_slam");
	slam = std::make_unique<lidar_slam::LidarSlam>(CURRENT_DIR+std::string("/lib/"),localization_mode,offline_mode);	
    ROS_INFO("create lidar_slam success");
	ros::Subscriber sub_pcl = nh.subscribe("/livox/lidar", 200000, livox_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe("/livox/imu", 200000, imu_cbk);
   // ros::Subscriber sub_image = nh.subscribe("/fisheye_image", 200000, image_cbk);

    ros::Subscriber sub_command  = nh.subscribe("/command" ,200000,command_cbk);

       
    std::thread show_thread;
    if (!show_rviz)
        show_thread = std::thread(&showThread);
    std::thread load_data_thread;
    if (offline_mode){
        // 读取文件夹中的文件名
       
    }
    std::cout << "Current directory: " << CURRENT_DIR << std::endl;
   // MowerCamera fisheyeCamera("/dev/fisheyeCam", 1920, 1200, 2, "fisheye.mp4");
     

  //  Viewer viewer;
	while (ros::ok())
	{
		ros::spinOnce();

        if (control_status.reset){
            sleep(1);
            slam.reset();
            sleep(1);
            localization_mode = control_status.localizationMode;
            slam = std::make_unique<lidar_slam::LidarSlam>(CURRENT_DIR+std::string("/lib/"),localization_mode,offline_mode);
            show_load_map = 0;
            control_status.reset = false;

        }
        if (show_load_map==0 && localization_mode && (slam->getLoadMap())->points.size() > 0){
           // ROS_INFO("load map");
            sleep(1);
            sensor_msgs::PointCloud2 loadMap;
            pcl::toROSMsg(*(slam->getLoadMap()), loadMap); 
            loadMap.header.stamp = ros::Time::now();
            loadMap.header.frame_id = "map";
            pubLoadMap.publish(loadMap);
            show_keyframe(slam->getLoadKeyFrame());
            ROS_INFO("load map success");
            show_load_map ++;
            }
        if (just_show_mode){
            loop_rate.sleep();
            continue;
        } 
        if (slam->run()&&show_rviz){
             if (!localization_mode || slam->isGloalLocalizationSuccess())
                pub_odom_cloud(slam->get_odom_cloud());
                pub_test_cloud(slam->getTestCloud());
                pub_lidar_cloud(slam->get_lidar_cloud());
                pub_obstacle_cloud(slam->getObstacleCloud());
                pub_filtered_obstacle_cloud(slam->getFilteredObstacleCloud());
                publish_unoptimized_path(slam->get_unoptimized_path());
                publish_optimized_path(slam->get_optimized_path(),string("odom"));
                visualizeLoopClosure(slam->getloopIndex());
                publish_transform(slam->getOdomToMap(),string("map"),string("odom"));
	//     pub_kdtree_cloud(slam->get_kdtree_cloud());		//not used yet
		 }
         if(!localization_mode){
           // pub_rgb_map(slam->getCurrentRGBMap());
         }
         if (show_rviz){
            publish_static_transform(slam->getWheelInLidar());
            publish_odometry(slam->getLidarInOdom());
         }
     //   publish_lidar_to_map(slam->getLidarInMap()); //not used yet
		loop_rate.sleep();
	}
  //  stopRosbagRecord();
  //  show_thread.join();
  //  log_thread.join();
	return 0;
}


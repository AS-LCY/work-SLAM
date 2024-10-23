// #include "lidar_slam.hpp"
// // #include <Viewer.hpp>
// #include <iostream>
// #include <chrono>
// #include <vector>
// #include <csignal>
// #include <thread>
// #include <unistd.h>
// #include "include/livox_ros_driver2.h"
// #include "driver_node.h"
// #include "lddc.h"
// #include "lds_lidar.h"
// std::unique_ptr<lidar_slam::LidarSlam> slam;
// int show_load_map = 0;
// lidar_slam::Control_status control_status;
// bool localization_mode = false,offline_mode = false;
// std::string log_folder;
// bool just_show_mode = false;
// void showThread()
// {
//     const int frequency = 5.0; // 频率为1Hz
//     const std::chrono::milliseconds period(1000 / frequency);
//     lidar_slam::Viewer test_view(localization_mode);
//     while (true)
//     {
//         auto start = std::chrono::steady_clock::now();
//         if (!control_status.reset){
//             test_view.Start();
//             control_status = test_view.getControl();
//             if (!localization_mode){
//                 std::vector<Eigen::Isometry3d> optimized_poses = slam->get_optimized_path();
//                 test_view.DrawTrajectory(optimized_poses,Eigen::Vector3f(0,1,0));
//                 test_view.DrawTrajectory(slam->get_unoptimized_path(),Eigen::Vector3f(1,0,0));
//                 map<int, int> loopIndex = slam->getloopIndex();
//                 for (auto it = loopIndex.begin(); it != loopIndex.end(); ++it) {
//                      test_view.DrawLine(optimized_poses[it->first],optimized_poses[it->second],Eigen::Vector3f(0,0,0));
//                 }
                
//                 if (control_status.showMap)
//                     test_view.DrawCloud(slam->getCurrentMap(),Eigen::Vector3f(0,0,1),1);
//                 if (control_status.showLidar)
//                    test_view.DrawCloud(slam->get_odom_cloud(),slam->getOdomToMap(),Eigen::Vector3f(1,0,0),2);
//                 if (control_status.showObstacle)
//                     test_view.DrawCloud(slam->getFilteredObstacleCloud(),slam->getWheelInMap(),Eigen::Vector3f(0,1,0),2.0);
//                 test_view.DrawPose(slam->getWheelInMap());
//             }
//             else{
//                 if (control_status.showMap)
//                    test_view.DrawCloud(slam->getLoadMapPoints(),Eigen::Vector3f(0,0,1),1.0);
//                 if (control_status.showLidar)
//                    test_view.DrawCloud(slam->get_lidar_cloud(),slam->getLidarInMap(),Eigen::Vector3f(1,0,0),2.0);
//                 if (control_status.showObstacle)
//                     test_view.DrawCloud(slam->getFilteredObstacleCloud(),slam->getWheelInMap(),Eigen::Vector3f(0,1,0),2.0);
//                 if (slam->isGloalLocalizationSuccess())
//                     test_view.DrawPose(slam->getWheelInMap());
//                 test_view.DrawTrajectory(slam->get_unoptimized_path(),Eigen::Vector3f(1,0,0));
//             }

//             if (control_status.saveMap && !localization_mode)
//                 slam -> save_map(CURRENT_DIR+std::string("/map/"),0.1,0,0);
//             test_view.Finish();  
//         }
//         auto end = std::chrono::steady_clock::now();
//         auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

//         if (elapsed < period)
//         {
//             std::this_thread::sleep_for(period - elapsed);
//         }
//     }
// }
// /*void livox_ros::DriverNode::PointCloudDataPollThread()
// {
//   std::future_status status;
//   std::this_thread::sleep_for(std::chrono::seconds(3));
//   do {
//     lddc_ptr_->DistributePointCloudData();
//     status = future_.wait_for(std::chrono::microseconds(0));
//   } while (status == std::future_status::timeout);
// }

// void livox_ros::DriverNode::ImuDataPollThread()
// {
//   std::future_status status;
//   std::this_thread::sleep_for(std::chrono::seconds(3));
//   do {
//     lddc_ptr_->DistributeImuData();
//     status = future_.wait_for(std::chrono::microseconds(0));
//   } while (status == std::future_status::timeout);
// }*/


// int main(int argc,char **argv)
// {
//   if (argc > 1){
//     char *value = argv[1];
//     if ( strcmp(value, "localization") == 0){
//       localization_mode = true;
//       printf("localization mode\n");
//     }
//   }
//   if (argc > 3){
//       char *value = argv[2];
//       if (strcmp(value, "offline") == 0){
//          offline_mode = true;
//          printf("offline mode\n");
//        }
//       log_folder = argv[3];
//   }

     
//   printf("create lidar_slam\n");
//    std::cout << "Current directory: " << CURRENT_DIR << std::endl;
//   slam = std::make_unique<lidar_slam::LidarSlam>(CURRENT_DIR+std::string("/"),localization_mode,offline_mode);	
//   printf("create lidar_slam success\n");
//   std::thread show_thread;
//   show_thread = std::thread(&showThread);
//   std::thread load_data_thread;
//   if (offline_mode){
//         // 读取文件夹中的文件名
      
//   }

//     const int frequency = 100.0; // 频率为100Hz
//     const std::chrono::milliseconds period(1000 / frequency);
//   //  Viewer viewer;
// 	while (true)
// 	{
// 		auto start = std::chrono::steady_clock::now();
//         if (control_status.reset){
//             sleep(1);
//             localization_mode = control_status.localizationMode;
//             slam->reset(CURRENT_DIR+std::string("/"),localization_mode,false);
//             sleep(1);
//             control_status.reset = false;
//         }      
//         slam->run();
//         auto end = std::chrono::steady_clock::now();
//         auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

//         if (elapsed < period)
//         {
//             std::this_thread::sleep_for(period - elapsed);
//         }
// 	}
//   //  stopRosbagRecord();
//     show_thread.join();
    
//   //  log_thread.join();
// 	return 0;
// }
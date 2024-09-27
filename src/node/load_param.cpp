// #include "node/localization_module.h"


// namespace localization_module {


// void LocalizationModule::load_params(){
//     nh_.param<bool>("localization_mode", localization_mode_, false);
//     nh_.param<bool>("just_show_mode", just_show_mode_, false);
//     nh_.param<bool>("offline_mode_", offline_mode_, false);
//     nh_.param<bool>("show_rviz", show_rviz_, false);
//     // nh_.param<bool>("fast", fast_mode_, false);
//     // nh_.param<string>("log_folder", log_folder_, " ");

// }


// bool LocalizationModule::load_lidar_slam_param(){
//     std::string ns = "/flbot/lidar_slam/";
//     bool success = true;
//     /// common *******************************************
//     get_param(ns+ "common/time_sync_en", slam_param_.common.time_sync_en, &success);
//     get_param(ns+ "common/localization_mode", slam_param_.common.localization_mode, &success);
//     get_param(ns+ "common/offline_mode", slam_param_.common.offline_mode, &success);
//     get_param(ns+ "common/fast_mode", slam_param_.common.fast_mode, &success);
//     get_param(ns+ "common/just_show_mode", slam_param_.common.just_show_mode, &success);
//     get_param(ns+ "common/show_rviz", slam_param_.common.show_rviz, &success);
//     get_param(ns+ "common/save_log_dir", slam_param_.common.save_log_dir, &success);
//     get_param(ns+ "common/log_keep_time", slam_param_.common.log_keep_time, &success);
//     get_param(ns+ "common/map_relative_to", slam_param_.common.map_relative_to, &success);
//     get_param(ns+ "common/map_directory", slam_param_.common.map_directory, &success);
//     get_param(ns+ "common/sub_topic_ctrl_cmd", slam_param_.common.sub_topic_ctrl_cmd, &success);
//     get_param(ns+ "common/pub_topic_module_status", slam_param_.common.pub_topic_module_status, &success);
//     get_param(ns+ "common/pub_topic_module_loginfo", slam_param_.common.pub_topic_module_loginfo, &success);
//     get_param(ns+ "common/receive_lidar_freq", slam_param_.common.receive_lidar_freq, &success);
//     get_param(ns+ "common/slam_lose_rate_time_thr", slam_param_.common.slam_lose_rate_time_thr, &success);
//     get_param(ns+ "common/lidar_no_point_count_thr", slam_param_.common.lidar_no_point_count_thr, &success);
//     get_param(ns+ "common/feats_down_size", slam_param_.common.feats_down_size, &success);
//     get_param(ns+ "common/cpu_id", slam_param_.common.cpu_id, &success);
//     ROS_INFO("\033[1;32mset cpu_id size: %lu\033[0m", slam_param_.common.cpu_id.size());
//     // process map_dir
//     // std::cout << "C++ Standard: " << __cplusplus << std::endl;
//     std::string parent_dir;
//     if(slam_param_.common.map_relative_to == 0){//相对于 pkg
//         std::string package_path = ros::package::getPath("lidar_slam");
//         parent_dir = package_path;
//     }else if(slam_param_.common.map_relative_to == 1){//相对于catkin_ws
//         // std::filesystem::path package_path = ros::package::getPath("lidar_slam");..\\..\\
//         // parent_dir = package_path.parent_path().parent_path().generic_string();
//         std::string package_path = ros::package::getPath("lidar_slam");
//         parent_dir = package_path + "/../../";
//     }else if(slam_param_.common.map_relative_to == 2){//绝对路径
//         parent_dir="";
//     }

//     slam_param_.common.map_directory = parent_dir + slam_param_.common.map_directory;

//     /// extrinsic *******************************************
//     std::vector<double> extrinsic_T; // 1 * 3
//     std::vector<double> extrinsic_R; // 3 * 3
//     std::vector<double> Lidar_In_Wheel; // 4* 4
//     std::vector<double> extrinsic_euler_IMU_in_baselink; // 1 * 3
//     get_param(ns+ "extrinsic/extrinsic_est_en", slam_param_.extrinsic.extrinsic_est_en, &success);
//     get_param(ns+ "extrinsic/extrinsic_T", extrinsic_T, &success);//temp
//     get_param(ns+ "extrinsic/extrinsic_R", extrinsic_R, &success);//temp
//     get_param(ns+ "extrinsic/Lidar_In_Wheel", Lidar_In_Wheel, &success);//temp
//     get_param(ns+ "extrinsic/extrinsic_euler_IMU_in_baselink", extrinsic_euler_IMU_in_baselink, &success);//temp
   
//     // extrinT & extrinR
//     slam_param_.extrinsic.extrinT<< extrinsic_T[0],extrinsic_T[1],extrinsic_T[2];
//     double yaw   = extrinsic_R[0]/180 * M_PI;
//     double pitch = extrinsic_R[1]/180 * M_PI;
//     double roll  = extrinsic_R[2]/180 * M_PI;
//     slam_param_.extrinsic.extrinR = ypr2R(Eigen::Vector3d{yaw, pitch, roll});

//     // slam_param_.extrinsic.extrinR<< extrinsic_R[0],extrinsic_R[1],extrinsic_R[2],
//     //                                 extrinsic_R[3],extrinsic_R[4],extrinsic_R[5],
//     //                                 extrinsic_R[6],extrinsic_R[7],extrinsic_R[8];

//     // IMU in base_link
//     double yaw1   = extrinsic_euler_IMU_in_baselink[0]/180 * M_PI;
//     double pitch1 = extrinsic_euler_IMU_in_baselink[1]/180 * M_PI;
//     double roll1  = extrinsic_euler_IMU_in_baselink[2]/180 * M_PI;
//     // slam_param_.extrinsic.R_baselink_IMU = ypr2R(Eigen::Vector3d{yaw1, pitch1, roll1});
//     slam_param_.extrinsic.R_baselink_IMU = rpy2R(Eigen::Vector3d{roll1,pitch1, yaw1});

//     // T_wheel_lidar & T_lidar_wheel
//     Eigen::Matrix4d T_wheel_lidar;
//     T_wheel_lidar<< Lidar_In_Wheel[0], Lidar_In_Wheel[1], Lidar_In_Wheel[2], Lidar_In_Wheel[3],
//                     Lidar_In_Wheel[4], Lidar_In_Wheel[5], Lidar_In_Wheel[6], Lidar_In_Wheel[7],
//                     Lidar_In_Wheel[8], Lidar_In_Wheel[9], Lidar_In_Wheel[10],Lidar_In_Wheel[11],
//                     Lidar_In_Wheel[12],Lidar_In_Wheel[13],Lidar_In_Wheel[14],Lidar_In_Wheel[15];
//     slam_param_.extrinsic.T_wheel_lidar.matrix() = T_wheel_lidar;
//     slam_param_.extrinsic.T_lidar_wheel = slam_param_.extrinsic.T_wheel_lidar.inverse();

//     /// lidar_preproc params *******************************************
//     get_param(ns+ "lidar_preproc/lidar_type", slam_param_.lidar_preproc.lidar_type, &success);
//     get_param(ns+ "lidar_preproc/line_count", slam_param_.lidar_preproc.line_count, &success);
//     get_param(ns+ "lidar_preproc/blind_distance", slam_param_.lidar_preproc.blind_distance, &success);
//     get_param(ns+ "lidar_preproc/flag_keep_only_last_lidar", slam_param_.lidar_preproc.flag_keep_only_last_lidar, &success);
//     get_param(ns+ "lidar_preproc/point_filter_num", slam_param_.lidar_preproc.point_filter_num, &success);
//     get_param(ns+ "lidar_preproc/point_filter_distance", slam_param_.lidar_preproc.point_filter_distance, &success);
//     get_param(ns+ "lidar_preproc/feature_enabled", slam_param_.lidar_preproc.feature_enabled, &success);
//     // get_param(ns+ "lidar_preproc/simple_voxel_enabled", slam_param_.lidar_preproc.simple_voxel_enabled, &success);
//     get_param(ns+ "lidar_preproc/extract_cloud_method", slam_param_.lidar_preproc.extract_cloud_method, &success);
//     get_param(ns+ "lidar_preproc/leafsize", slam_param_.lidar_preproc.leafsize, &success);
//     get_param(ns+ "lidar_preproc/leafsize_vec", slam_param_.lidar_preproc.leafsize_vec, &success);
//     get_param(ns+ "lidar_preproc/voxel_region_xyz", slam_param_.lidar_preproc.voxel_region_xyz, &success);
//     get_param(ns+ "lidar_preproc/boundary_z", slam_param_.lidar_preproc.boundary_z, &success);
//     get_param(ns+ "lidar_preproc/obstacle_max_range", slam_param_.lidar_preproc.obstacle_max_range, &success);
//     // 
//     get_param(ns+ "lidar_preproc/obstacle_max_height", slam_param_.lidar_preproc.obstacle_max_height, &success);
//     get_param(ns+ "lidar_preproc/obstacle_min_height", slam_param_.lidar_preproc.obstacle_min_height, &success);
//     get_param(ns+ "lidar_preproc/obstacle_filter_size", slam_param_.lidar_preproc.obstacle_filter_size, &success);
//     get_param(ns+ "lidar_preproc/grid_size", slam_param_.lidar_preproc.grid_size, &success);

//     /// mapping params *******************************************
//     get_param(ns+ "mapping/acc_cov", slam_param_.mapping.acc_cov, &success);
//     get_param(ns+ "mapping/gyr_cov", slam_param_.mapping.gyr_cov, &success);
//     get_param(ns+ "mapping/b_acc_cov", slam_param_.mapping.b_acc_cov, &success);
//     get_param(ns+ "mapping/b_gyr_cov", slam_param_.mapping.b_gyr_cov, &success);
//     get_param(ns+ "mapping/cloud_leaf_size", slam_param_.mapping.cloud_leaf_size, &success);
//     get_param(ns+ "mapping/key_frame_distance", slam_param_.mapping.key_frame_distance, &success);
//     get_param(ns+ "mapping/key_frame_angle", slam_param_.mapping.key_frame_angle, &success);
//     get_param(ns+ "mapping/loopSearchDistance", slam_param_.mapping.loopSearchDistance, &success);
//     get_param(ns+ "mapping/use_ele_pcd_flag", slam_param_.mapping.use_ele_pcd_flag, &success);
//     get_param(ns+ "mapping/save_ele_pcd_flag", slam_param_.mapping.save_ele_pcd_flag, &success);
//     // get_param(ns+ "mapping/save_map_dir", slam_param_.mapping.save_map_dir, &success);
//     get_param(ns+ "mapping/save_map_resolution", slam_param_.mapping.save_map_resolution, &success);
    
//     /// sec_mapping params *******************************************
//     // get_param(ns+ "sec_mapping/load_map_dir", slam_param_.sec_mapping.load_map_dir, &success);


//     /// localization params *******************************************
//     // get_param(ns+ "localization/load_map_dir", slam_param_.localization.load_map_dir, &success);
//     get_param(ns+ "localization/fgicp_score_thr", slam_param_.localization.fgicp_score_thr, &success);
//     get_param(ns+ "localization/fgicp_freq", slam_param_.localization.fgicp_freq, &success);
//     get_param(ns+ "localization/filter_method", slam_param_.localization.filter_method, &success);
//     get_param(ns+ "localization/fst_order_k", slam_param_.localization.fst_order_k, &success);
//     get_param(ns+ "localization/odom2map_delta_thr", slam_param_.localization.odom2map_delta_thr, &success);
//     get_param(ns+ "localization/odom2map_delta_set", slam_param_.localization.odom2map_delta_set, &success);
//     get_param(ns+ "localization/lidar_ratio", slam_param_.localization.lidar_ratio, &success);
//     get_param(ns+ "localization/baselink_dy_thr", slam_param_.localization.baselink_dy_thr, &success);
//     get_param(ns+ "localization/baselink_dx_thr", slam_param_.localization.baselink_dx_thr, &success);
//     get_param(ns+ "localization/baselink_dyaw_thr", slam_param_.localization.baselink_dyaw_thr, &success);
//     get_param(ns+ "localization/window_size", slam_param_.localization.window_size, &success);
//     get_param(ns+ "localization/filter_freq", slam_param_.localization.filter_freq, &success);
    


//     /// re-localization params *******************************************
//     get_param(ns+ "re_localization/score_thr", slam_param_.re_localization.score_thr, &success);
//     get_param(ns+ "re_localization/time_out_thr", slam_param_.re_localization.time_out_thr, &success);


//     /// ikdtree params *******************************************
//     get_param(ns+ "ikdtree/cube_len", slam_param_.ikdtree.cube_len, &success);
//     get_param(ns+ "ikdtree/det_range", slam_param_.ikdtree.det_range, &success);
//     get_param(ns+ "ikdtree/kdTreeReconstructRadius", slam_param_.ikdtree.kdTreeReconstructRadius, &success);
//     get_param(ns+ "ikdtree/kdTreeReconstructKeyFrameLeafSize", slam_param_.ikdtree.kdTreeReconstructKeyFrameLeafSize, &success);
//     get_param(ns+ "ikdtree/kdTreeReconstructPointLeafSize", slam_param_.ikdtree.kdTreeReconstructPointLeafSize, &success);
//     get_param(ns+ "ikdtree/map_leaf_size", slam_param_.ikdtree.map_leaf_size, &success);

//     ROS_INFO("\033[1;32mset cpu_id size: %lu\033[0m", slam_param_.common.cpu_id.size());
//     return success;
// }

// }/// namespace localization_module
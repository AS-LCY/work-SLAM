#ifndef LOCALIZATION_MODULE_PARAM_MANAGER_H
#define LOCALIZATION_MODULE_PARAM_MANAGER_H

#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <vector>

#include "node/module_param_def.h"


using namespace std;

namespace localization_module{

class LocalizationModuleParamManager{
public:
	static LocalizationModuleParamManager* Instance(){
		static LocalizationModuleParamManager* instance;
		if(instance==nullptr){
			instance=new LocalizationModuleParamManager();
		}
		return instance;
	}

	bool load_config_params(){
        std::string ns = "/flbot/lidar_slam/";
        bool success = true;
        /// common *******************************************
        get_param(ns+ "common/run_on_mower", loaded_param_.common.run_on_mower, &success);
        get_param(ns+ "common/time_sync_en", loaded_param_.common.time_sync_en, &success);
        get_param(ns+ "common/localization_mode", loaded_param_.common.localization_mode, &success);
        get_param(ns+ "common/offline_mode", loaded_param_.common.offline_mode, &success);
        get_param(ns+ "common/fast_mode", loaded_param_.common.fast_mode, &success);
        get_param(ns+ "common/just_show_mode", loaded_param_.common.just_show_mode, &success);
        get_param(ns+ "common/show_rviz", loaded_param_.common.show_rviz, &success);
        get_param(ns+ "common/save_log_dir", loaded_param_.common.save_log_dir, &success);
        get_param(ns+ "common/log_keep_time", loaded_param_.common.log_keep_time, &success);
        get_param(ns+ "common/map_relative_to", loaded_param_.common.map_relative_to, &success);
        get_param(ns+ "common/map_directory", loaded_param_.common.map_directory, &success);
        get_param(ns+ "common/sub_topic_ctrl_cmd", loaded_param_.common.sub_topic_ctrl_cmd, &success);
        get_param(ns+ "common/pub_topic_module_status", loaded_param_.common.pub_topic_module_status, &success);
        get_param(ns+ "common/pub_topic_module_health", loaded_param_.common.pub_topic_module_health, &success);
        get_param(ns+ "common/pub_topic_module_loginfo", loaded_param_.common.pub_topic_module_loginfo, &success);
        get_param(ns+ "common/pub_topic_slipping", loaded_param_.common.pub_topic_slipping, &success);
        get_param(ns+ "common/receive_lidar_freq", loaded_param_.common.receive_lidar_freq, &success);
        get_param(ns+ "common/slam_lose_rate_time_thr", loaded_param_.common.slam_lose_rate_time_thr, &success);
        get_param(ns+ "common/lidar_no_point_count_thr", loaded_param_.common.lidar_no_point_count_thr, &success);
        get_param(ns+ "common/feats_down_size_thr", loaded_param_.common.feats_down_size_thr, &success);
        get_param(ns+ "common/use_pose_filter", loaded_param_.common.use_pose_filter, &success);
        get_param(ns+ "common/cpu_id", loaded_param_.common.cpu_id, &success);
        // process map_dir
        // std::cout << "C++ Standard: " << __cplusplus << std::endl;
        std::string parent_dir;
        if(loaded_param_.common.map_relative_to == 0){//相对于 pkg
            std::string package_path = ros::package::getPath("lidar_slam");
            parent_dir = package_path;
        }else if(loaded_param_.common.map_relative_to == 1){//相对于catkin_ws
            // std::filesystem::path package_path = ros::package::getPath("lidar_slam");..\\..\\
            // parent_dir = package_path.parent_path().parent_path().generic_string();
            std::string package_path = ros::package::getPath("lidar_slam");
            parent_dir = package_path + "/../../";
        }else if(loaded_param_.common.map_relative_to == 2){//绝对路径
            parent_dir="";
        }

        loaded_param_.common.map_directory = parent_dir + loaded_param_.common.map_directory;

        std::string map_directory_on_mower_temp = "";
        std::vector<int> cpu_id_on_mower_temp;
        get_param(ns+ "common/cpu_id_on_mower", cpu_id_on_mower_temp, &success);
        get_param(ns+ "common/map_directory_on_mower", map_directory_on_mower_temp, &success);

        if(loaded_param_.common.run_on_mower){
            loaded_param_.common.map_directory = map_directory_on_mower_temp;
            loaded_param_.common.cpu_id = cpu_id_on_mower_temp;
        }

        /// extrinsic *******************************************
        std::vector<double> extrinsic_T; // 1 * 3
        std::vector<double> extrinsic_R; // 3 * 3
        std::vector<double> Lidar_In_Wheel; // 4* 4
        // std::vector<double> extrinsic_euler_IMU_in_baselink; // 1 * 3
        std::vector<double> extrinsic_euler_IMU_in_lidar; // 1 * 3
        std::vector<double> extrinsic_euler_lidar_in_baselink; // 1 * 3
        
        // std::vector<double> quat_lidar_in_imu;
        get_param(ns+ "extrinsic/extrinsic_est_en", loaded_param_.extrinsic.extrinsic_est_en, &success);
        get_param(ns+ "extrinsic/extrinsic_T", extrinsic_T, &success);//temp
        get_param(ns+ "extrinsic/extrinsic_R", extrinsic_R, &success);//temp
        get_param(ns+ "extrinsic/Lidar_In_Wheel", Lidar_In_Wheel, &success);//temp
        // get_param(ns+ "extrinsic/extrinsic_euler_IMU_in_baselink", extrinsic_euler_IMU_in_baselink, &success);//temp
        get_param(ns+ "extrinsic/extrinsic_euler_IMU_in_lidar", extrinsic_euler_IMU_in_lidar, &success);//temp
        get_param(ns+ "extrinsic/extrinsic_euler_lidar_in_baselink", extrinsic_euler_lidar_in_baselink, &success);//temp
    
        // extrinT & extrinR
        loaded_param_.extrinsic.extrinT<< extrinsic_T[0],extrinsic_T[1],extrinsic_T[2];
        double yaw   = extrinsic_R[0]/180 * M_PI;
        double pitch = extrinsic_R[1]/180 * M_PI;
        double roll  = extrinsic_R[2]/180 * M_PI;
        loaded_param_.extrinsic.extrinR = ypr2R(Eigen::Vector3d{yaw, pitch, roll});
        // loaded_param_.extrinsic.extrinR = rpy2R(Eigen::Vector3d{roll, pitch, yaw});
        // std::cout<<"rpy2R(Eigen::Vector3d{roll, pitch, yaw}): "<<endl<<rpy2R(Eigen::Vector3d{roll, pitch, yaw})<<endl<<endl;
        // std::cout<<"ypr2R(Eigen::Vector3d{yaw, pitch, roll}): "<<endl<<ypr2R(Eigen::Vector3d{yaw, pitch, roll})<<endl<<endl;

        // loaded_param_.extrinsic.extrinR<< extrinsic_R[0],extrinsic_R[1],extrinsic_R[2],
        //                                 extrinsic_R[3],extrinsic_R[4],extrinsic_R[5],
        //                                 extrinsic_R[6],extrinsic_R[7],extrinsic_R[8];

        // // IMU in base_link
        // double yaw1   = extrinsic_euler_IMU_in_baselink[0]/180 * M_PI;
        // double pitch1 = extrinsic_euler_IMU_in_baselink[1]/180 * M_PI;
        // double roll1  = extrinsic_euler_IMU_in_baselink[2]/180 * M_PI;
        // loaded_param_.extrinsic.R_baselink_IMU = rpy2R(Eigen::Vector3d{roll1,pitch1, yaw1});

        // IMU in base_link
        double yaw2   = extrinsic_euler_IMU_in_lidar[0]/180 * M_PI;
        double pitch2 = extrinsic_euler_IMU_in_lidar[1]/180 * M_PI;
        double roll2  = extrinsic_euler_IMU_in_lidar[2]/180 * M_PI;
        auto R_imu_in_lidar = rpy2R(Eigen::Vector3d{roll2,pitch2, yaw2});
        double yaw3   = extrinsic_euler_lidar_in_baselink[0]/180 * M_PI;
        double pitch3 = extrinsic_euler_lidar_in_baselink[1]/180 * M_PI;
        double roll3  = extrinsic_euler_lidar_in_baselink[2]/180 * M_PI;
        auto R_lidar_in_base = rpy2R(Eigen::Vector3d{roll3,pitch3, yaw3});
        loaded_param_.extrinsic.R_baselink_IMU = R_lidar_in_base * R_imu_in_lidar;


        // T_wheel_lidar & T_lidar_wheel
        Eigen::Matrix4d T_wheel_lidar;
        T_wheel_lidar<< Lidar_In_Wheel[0], Lidar_In_Wheel[1], Lidar_In_Wheel[2], Lidar_In_Wheel[3],
                        Lidar_In_Wheel[4], Lidar_In_Wheel[5], Lidar_In_Wheel[6], Lidar_In_Wheel[7],
                        Lidar_In_Wheel[8], Lidar_In_Wheel[9], Lidar_In_Wheel[10],Lidar_In_Wheel[11],
                        Lidar_In_Wheel[12],Lidar_In_Wheel[13],Lidar_In_Wheel[14],Lidar_In_Wheel[15];
        loaded_param_.extrinsic.T_wheel_lidar.matrix() = T_wheel_lidar;
        loaded_param_.extrinsic.T_lidar_wheel = loaded_param_.extrinsic.T_wheel_lidar.inverse();

        /// lidar_preproc params *******************************************
        get_param(ns+ "lidar_preproc/lidar_type", loaded_param_.lidar_preproc.lidar_type, &success);
        get_param(ns+ "lidar_preproc/sub_lidar_topic", loaded_param_.lidar_preproc.sub_lidar_topic, &success);
        get_param(ns+ "lidar_preproc/sub_imu_topic", loaded_param_.lidar_preproc.sub_imu_topic, &success);
        get_param(ns+ "lidar_preproc/line_count", loaded_param_.lidar_preproc.line_count, &success);
        get_param(ns+ "lidar_preproc/blind_distance", loaded_param_.lidar_preproc.blind_distance, &success);
        get_param(ns+ "lidar_preproc/flag_keep_only_last_lidar", loaded_param_.lidar_preproc.flag_keep_only_last_lidar, &success);
        get_param(ns+ "lidar_preproc/point_filter_num", loaded_param_.lidar_preproc.point_filter_num, &success);
        get_param(ns+ "lidar_preproc/ring_filter_num", loaded_param_.lidar_preproc.ring_filter_num, &success);
        get_param(ns+ "lidar_preproc/point_filter_distance", loaded_param_.lidar_preproc.point_filter_distance, &success);
        get_param(ns+ "lidar_preproc/cloud_size_to_keep", loaded_param_.lidar_preproc.cloud_size_to_keep, &success);
        get_param(ns+ "lidar_preproc/feature_enabled", loaded_param_.lidar_preproc.feature_enabled, &success);
        // get_param(ns+ "lidar_preproc/simple_voxel_enabled", loaded_param_.lidar_preproc.simple_voxel_enabled, &success);
        get_param(ns+ "lidar_preproc/extract_cloud_method", loaded_param_.lidar_preproc.extract_cloud_method, &success);
        get_param(ns+ "lidar_preproc/leafsize", loaded_param_.lidar_preproc.leafsize, &success);
        get_param(ns+ "lidar_preproc/leafsize_vec", loaded_param_.lidar_preproc.leafsize_vec, &success);
        get_param(ns+ "lidar_preproc/voxel_region_xyz", loaded_param_.lidar_preproc.voxel_region_xyz, &success);
        get_param(ns+ "lidar_preproc/boundary_z", loaded_param_.lidar_preproc.boundary_z, &success);
        get_param(ns+ "lidar_preproc/obstacle_max_range", loaded_param_.lidar_preproc.obstacle_max_range, &success);
        // 
        get_param(ns+ "lidar_preproc/obstacle_max_height", loaded_param_.lidar_preproc.obstacle_max_height, &success);
        get_param(ns+ "lidar_preproc/obstacle_min_height", loaded_param_.lidar_preproc.obstacle_min_height, &success);
        get_param(ns+ "lidar_preproc/obstacle_filter_size", loaded_param_.lidar_preproc.obstacle_filter_size, &success);
        get_param(ns+ "lidar_preproc/grid_size", loaded_param_.lidar_preproc.grid_size, &success);

        /// mapping params *******************************************
        get_param(ns+ "mapping/acc_cov", loaded_param_.mapping.acc_cov, &success);
        get_param(ns+ "mapping/gyr_cov", loaded_param_.mapping.gyr_cov, &success);
        get_param(ns+ "mapping/b_acc_cov", loaded_param_.mapping.b_acc_cov, &success);
        get_param(ns+ "mapping/b_gyr_cov", loaded_param_.mapping.b_gyr_cov, &success);
        get_param(ns+ "mapping/cloud_leaf_size", loaded_param_.mapping.cloud_leaf_size, &success);
        get_param(ns+ "mapping/key_frame_distance", loaded_param_.mapping.key_frame_distance, &success);
        get_param(ns+ "mapping/key_frame_angle", loaded_param_.mapping.key_frame_angle, &success);
        get_param(ns+ "mapping/loopSearchDistance", loaded_param_.mapping.loopSearchDistance, &success);
        get_param(ns+ "mapping/loopSearchTimeDiff", loaded_param_.mapping.loopSearchTimeDiff, &success);
        get_param(ns+ "mapping/loopSearchSkipKey", loaded_param_.mapping.loopSearchSkipKey, &success);
        get_param(ns+ "mapping/loopIcpScore", loaded_param_.mapping.loopIcpScore, &success);
        // get_param(ns+ "mapping/use_ele_pcd_flag", loaded_param_.mapping.use_ele_pcd_flag, &success);
        // get_param(ns+ "mapping/save_ele_pcd_flag", loaded_param_.mapping.save_ele_pcd_flag, &success);
        // get_param(ns+ "mapping/save_map_dir", loaded_param_.mapping.save_map_dir, &success);
        get_param(ns+ "mapping/save_map_resolution", loaded_param_.mapping.save_map_resolution, &success);
        
        /// sec_mapping params *******************************************
        // get_param(ns+ "sec_mapping/load_map_dir", loaded_param_.sec_mapping.load_map_dir, &success);

        /// localization params *******************************************
        // get_param(ns+ "localization/load_map_dir", loaded_param_.localization.load_map_dir, &success);
        get_param(ns+ "localization/fgicp_score_thr", loaded_param_.localization.fgicp_score_thr, &success);
        get_param(ns+ "localization/fgicp_freq", loaded_param_.localization.fgicp_freq, &success);
        get_param(ns+ "localization/filter_method", loaded_param_.localization.filter_method, &success);
        get_param(ns+ "localization/fst_order_k", loaded_param_.localization.fst_order_k, &success);
        get_param(ns+ "localization/odom2map_delta_thr", loaded_param_.localization.odom2map_delta_thr, &success);
        get_param(ns+ "localization/odom2map_delta_set", loaded_param_.localization.odom2map_delta_set, &success);
        get_param(ns+ "localization/lidar_ratio", loaded_param_.localization.lidar_ratio, &success);
        get_param(ns+ "localization/baselink_dy_thr", loaded_param_.localization.baselink_dy_thr, &success);
        get_param(ns+ "localization/baselink_dx_thr", loaded_param_.localization.baselink_dx_thr, &success);
        get_param(ns+ "localization/baselink_dyaw_thr", loaded_param_.localization.baselink_dyaw_thr, &success);
        get_param(ns+ "localization/window_size", loaded_param_.localization.window_size, &success);
        get_param(ns+ "localization/filter_freq", loaded_param_.localization.filter_freq, &success);
        get_param(ns+ "localization/chassis_linear_velocity_thr", loaded_param_.localization.chassis_linear_velocity_thr, &success);
        get_param(ns+ "localization/motionless_chassis_ratio", loaded_param_.localization.motionless_chassis_ratio, &success);
        get_param(ns+ "localization/lidar_cbk_delay_thr", loaded_param_.localization.lidar_cbk_delay_thr, &success);
        
        /// re-localization params *******************************************
        get_param(ns+ "re_localization/score_thr", loaded_param_.re_localization.score_thr, &success);
        get_param(ns+ "re_localization/time_out_thr", loaded_param_.re_localization.time_out_thr, &success);


        /// ikdtree params *******************************************
        get_param(ns+ "ikdtree/cube_len", loaded_param_.ikdtree.cube_len, &success);
        get_param(ns+ "ikdtree/det_range", loaded_param_.ikdtree.det_range, &success);
        get_param(ns+ "ikdtree/kdTreeReconstructRadius", loaded_param_.ikdtree.kdTreeReconstructRadius, &success);
        get_param(ns+ "ikdtree/kdTreeReconstructKeyFrameLeafSize", loaded_param_.ikdtree.kdTreeReconstructKeyFrameLeafSize, &success);
        get_param(ns+ "ikdtree/kdTreeReconstructPointLeafSize", loaded_param_.ikdtree.kdTreeReconstructPointLeafSize, &success);
        get_param(ns+ "ikdtree/map_leaf_size", loaded_param_.ikdtree.map_leaf_size, &success);

        ///  detect slip params *******************************************
        get_param(ns+ "detect_slip/detect_window_time_range", loaded_param_.detect_slip.detect_window_time_range, &success);
        get_param(ns+ "detect_slip/slipping_count_thr", loaded_param_.detect_slip.slipping_count_thr, &success);
        get_param(ns+ "detect_slip/slipping_dist_thr", loaded_param_.detect_slip.slipping_dist_thr, &success);

        ROS_INFO_STREAM(BOLDGREEN<<"run_on_mower: "<<loaded_param_.common.run_on_mower<<RESET);
        ROS_INFO_STREAM(YELLOW<<"set cpu_id size: " <<loaded_param_.common.cpu_id.size()<<RESET);
        ROS_INFO_STREAM(YELLOW<<"map directory: " << loaded_param_.common.map_directory<<RESET);

        return success;
	}

	//////////////////////////////////////////////////////////////
	const lidar_slam::LidarSlamParam* get_loaded_param() const {
		return &loaded_param_;
	}


    /// @brief template function to get param for different types
    template <class T>
    void get_param(const std::string& param_str, T& param, bool* is_success){
        if(!nh_.getParamCached(param_str,param)){
            ROS_WARN_STREAM(YELLOW << "load param failed : "<< param_str.c_str()<< RESET);
            *is_success = false;
        }else{
            ROS_INFO("load param success: %s", param_str.c_str());
        }
    };



private:
	LocalizationModuleParamManager(){
		if(load_config_params()==false){
			ROS_ERROR_STREAM(RED << "load config params failed" <<RESET);
			exit(0);
		}
	}

	~LocalizationModuleParamManager(){};

private:
	ros::NodeHandle nh_;

    lidar_slam::LidarSlamParam loaded_param_;

};

} // namespace localization_module

#endif
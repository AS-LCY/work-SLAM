#ifndef FLBOT_LOCALIZTION_FUSION_FUSION_H
#define FLBOT_LOCALIZTION_FUSION_FUSION_H

#include <ros/ros.h>

#include <string>

namespace localization_module {
struct DetectSlipParam {
	double detect_window_time_range = 2;
	int slipping_count_thr = 5;
	double slipping_dist_thr = 0.2;
};

struct FusionTopicParams {
	std::string sub_imu_topic = "/livox/imu";
	std::string sub_chassis_topic = "/livox/imu";
	std::string sub_slam_odom_topic = "/livox/imu";
	std::string pub_localization_topic = "/flbot/localiztion/odometry";
	std::string pub_slipping_topic = "/flbot/localiztion/slipping";
};

struct EkfGatingParams {
	int mah_drift_thresh = 10;
	int mah_drift_lateral_thresh = 10;
	double dis_drift_lateral_thresh = 0.3;
	int mah_warn_thresh = 5;
	int mah_warn_lateral_thresh = 5;
	double turn_angular_thresh = 0.01;
};

struct LocalizationFusionParams {
	bool use_fusion = true;
	int status_num = 0;						  ///< the status number
	int measure_num = 0;					  ///< the measurement number
	int input_num = 0;						  ///< the input command number
	int localization_fusion_dt = 0;			  ///< the localization fusion period
	std::vector<double> status_cov_mat_diag;  ///< the status initial convariance
	std::vector<double> input_cov_mat_diag;	  ///< the input convariance
	std::vector<double> measure_cov_mat_diag; ///< the measurement convariance
	double w_thr = 0.15;
	bool ekf_use_chassis = true;
	bool use_ekf_yaw = false;
	std::vector<double> baselink_in_lidar;
	double time_lost_thr = 3.0;
	EkfGatingParams gating_params;
	FusionTopicParams topic_params;
	DetectSlipParam slip_params;
};

/// @brief the class to load the LocalizationFusion parameters
class LocalizationFusionParamsManager {
   public:
	/// @brief get LocalizationFusionParamsManager Instance
	/// @return LocalizationFusion class instance pointer
	static LocalizationFusionParamsManager* Instance() {
		static LocalizationFusionParamsManager* instance;
		if (instance == nullptr) {
			instance = new LocalizationFusionParamsManager();
		}
		return instance;
	}

	/// @brief load all LocalizationFusion parameters
	/// @return return true if load parameters success, otherwise return false
	bool load_config_params() {
		bool success = true;
		success &= load_localization_fusion_params();
		return success;
	}

	/// @brief load the parameters to init localization fusion from the ros parameter server
	/// @return return true if load localization fusion parameters success, otherwise return false
	bool load_localization_fusion_params() {
		bool success = true;
		const std::string title = "/flbot/localization/localization_fusion/";
		get_param(title + "use_fusion", localization_fusion_params_.use_fusion, &success);
		get_param(title + "status_num", localization_fusion_params_.status_num, &success);
		get_param(title + "measure_num", localization_fusion_params_.measure_num, &success);
		get_param(title + "input_num", localization_fusion_params_.input_num, &success);
		get_param(title + "localization_fusion_dt", localization_fusion_params_.localization_fusion_dt, &success);
		get_param(title + "status_cov_mat_diag", localization_fusion_params_.status_cov_mat_diag, &success);
		get_param(title + "input_cov_mat_diag", localization_fusion_params_.input_cov_mat_diag, &success);
		get_param(title + "measure_cov_mat_diag", localization_fusion_params_.measure_cov_mat_diag, &success);
		get_param(title + "w_thr", localization_fusion_params_.w_thr, &success);
		get_param(title + "ekf_use_chassis", localization_fusion_params_.ekf_use_chassis, &success);
		get_param(title + "use_ekf_yaw", localization_fusion_params_.use_ekf_yaw, &success);
		get_param(title + "baselink_in_lidar", localization_fusion_params_.baselink_in_lidar, &success);
		get_param(title + "time_lost_thr", localization_fusion_params_.time_lost_thr, &success);

		///  fusion topics params *******************************************
		get_param(title + "fusion_topics/sub_imu_topic", localization_fusion_params_.topic_params.sub_imu_topic,
				  &success);
		get_param(title + "fusion_topics/sub_chassis_topic", localization_fusion_params_.topic_params.sub_chassis_topic,
				  &success);
		get_param(title + "fusion_topics/sub_slam_odom_topic",
				  localization_fusion_params_.topic_params.sub_slam_odom_topic, &success);
		get_param(title + "fusion_topics/pub_localization_topic",
				  localization_fusion_params_.topic_params.pub_localization_topic, &success);
		get_param(title + "fusion_topics/pub_slipping_topic",
				  localization_fusion_params_.topic_params.pub_slipping_topic, &success);

		///  gating params *******************************************
		get_param(title + "gating/mah_drift_thresh", localization_fusion_params_.gating_params.mah_drift_thresh,
				  &success);
		get_param(title + "gating/mah_warn_thresh", localization_fusion_params_.gating_params.mah_warn_thresh,
				  &success);
		get_param(title + "gating/mah_drift_lateral_thresh",
				  localization_fusion_params_.gating_params.mah_drift_lateral_thresh, &success);
		get_param(title + "gating/mah_warn_lateral_thresh",
				  localization_fusion_params_.gating_params.mah_warn_lateral_thresh, &success);
		get_param(title + "gating/dis_drift_lateral_thresh",
				  localization_fusion_params_.gating_params.dis_drift_lateral_thresh, &success);
		get_param(title + "gating/turn_angular_thresh", localization_fusion_params_.gating_params.turn_angular_thresh,
				  &success);

		///  detect slip params *******************************************
		get_param(title + "detect_slip/detect_window_time_range",
				  localization_fusion_params_.slip_params.detect_window_time_range, &success);
		get_param(title + "detect_slip/slipping_count_thr", localization_fusion_params_.slip_params.slipping_count_thr,
				  &success);
		get_param(title + "detect_slip/slipping_dist_thr", localization_fusion_params_.slip_params.slipping_dist_thr,
				  &success);

		// success &= load_ekf_gating_params();
		// success &= load_fusion_topics_params();
		// success &= load_detect_slip_params();
		return success;
	}

	// bool load_ekf_gating_params(){
	//     bool success = true;
	//     const std::string title = "/flbot/localization/localization_fusion/gating/";
	//     EkfGatingParams& gating_params = localization_fusion_params_.gating_params;
	//     get_param(title + "mah_drift_thresh", gating_params.mah_drift_thresh, &success);
	//     get_param(title + "mah_warn_thresh", gating_params.mah_warn_thresh, &success);
	//     get_param(title + "mah_drift_lateral_thresh", gating_params.mah_drift_lateral_thresh, &success);
	//     get_param(title + "mah_warn_lateral_thresh", gating_params.mah_warn_lateral_thresh, &success);
	//     get_param(title + "dis_drift_lateral_thresh", gating_params.dis_drift_lateral_thresh, &success);
	//     get_param(title + "turn_angular_thresh", gating_params.turn_angular_thresh, &success);
	//     return success;
	// }

	// bool load_fusion_topics_params() {
	//     bool success = true;
	//     const std::string title = "/flbot/localization/localization_fusion/fusion_topics/";
	//     FusionTopicParams& topic_params = localization_fusion_params_.topic_params;
	//     get_param(title + "sub_imu_topic", topic_params.sub_imu_topic, &success);
	//     get_param(title + "sub_chassis_topic", topic_params.sub_chassis_topic, &success);
	//     get_param(title + "sub_slam_odom_topic", topic_params.sub_slam_odom_topic, &success);
	//     get_param(title + "pub_localization_topic", topic_params.pub_localization_topic, &success);
	//     get_param(title + "pub_slipping_topic", topic_params.pub_slipping_topic, &success);
	//     return success;
	// }

	// bool load_detect_slip_params() {
	//     bool success = true;
	//     const std::string title = "/flbot/localization/localization_fusion/detect_slip/";
	//     DetectSlipParams& slip_params = localization_fusion_params_.slip_params;
	//     ///  detect slip params *******************************************
	//     get_param(title+ "detect_slip/detect_window_time_range",
	//     localization_fusion_params_.slip_params.detect_window_time_range, &success); get_param(title+
	//     "detect_slip/slipping_count_thr", localization_fusion_params_.slip_params.slipping_count_thr, &success);
	//     get_param(title+ "detect_slip/slipping_dist_thr", localization_fusion_params_.slip_params.slipping_dist_thr,
	//     &success); return success;
	// }

	/// @brief get localization fusion parameters
	const LocalizationFusionParams* get_localization_fusion_params() const { return &localization_fusion_params_; }

	/// @brief template function to get param for different types
	template <class T>
	void get_param(const std::string& param_str, T& param, bool* is_success) {
		if (!nh_.getParamCached(param_str, param)) {
			ROS_WARN("load param failed : %s ", param_str.c_str());
			*is_success = false;
		} else {
			// ROS_INFO("load param success: %s", param_str.c_str());
		}
	};

   private:
	/// @brief constructor function
	LocalizationFusionParamsManager() {
		if (load_config_params() == false) {
			ROS_ERROR("Load LocalizationFusion params failed!");
			exit(0);
		}
	}

	/// @brief destructor function
	~LocalizationFusionParamsManager();

   private:
	ros::NodeHandle nh_;
	LocalizationFusionParams localization_fusion_params_; ///< the localization fusion params object
};

} // namespace localization_module

#endif // FLBOT_LOCALIZTION_FUSION_FUSION_H
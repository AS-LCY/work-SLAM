#include <ros/ros.h>
// ros-msg
// #include <std_msgs/UInt32.h>
// #include <geometry_msgs/Twist.h>
// #include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
// #include <sensor_msgs/NavSatFix.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>

// 另一个节点中定义
#include "fros_hardware_node/chassic_data.h"
#include "fairland_msgs/LocalizationPoseData.h"

#include "fusion_node/fusion_param.hpp"
#include "fusion_node/common/print_color.h"
#include "fusion_node/common/quaternion.h"
#include "fusion_node/common/numerical_process.h"
#include "pose_ekf.h"

namespace localization_module{


class EkfLocalizationFusion{

public:
    EkfLocalizationFusion();
    ~EkfLocalizationFusion();

    /// @brief init function when localization fusion needs initialization
    /// @param status the LeadgenStatus message in
    void init(const fairland_msgs::LocalizationPoseData& status);

    void reset();

    /// @brief localization fusion is init or not
    /// @return return true if is init, otherwise return false
    bool is_init();

    void localization_fusion_core(const fairland_msgs::LocalizationPoseData& status, fairland_msgs::LocalizationPoseData *status_out);


private:
    void set_localizationfusion_input();
    bool set_params();

    bool matrix_init(const LocalizationFusionParams* lf_params);


/////////////////////////////////////////////////////////////////////////////////////////////////

public:


private:
    std::shared_ptr<PoseEKF> ekf_ptr_; ///< the ekf pointer
    fairland_msgs::LocalizationPoseData status_;

    EkfGatingParams gating_params_;
    bool use_ekf_yaw_ = false;


    bool is_init_ = false; ///< if init or notLocalizationFusionParams
    double offset_x_=0.0; ///< offset for position x
    double offset_y_=0.0; ///< offset for posttion y
    double ts_; ///< timestamp for fusion

    int status_num_; ///< status num
    int measure_num_; ///< measure num
	int input_num_; ///< input num
    double dt_; ///< period of extended kalman filter

    Matrix input_; ///< the control command [v, w]
    Matrix measure_; ///< the measurement [x, y, theta]
    Matrix status_covariance_; ///< the status covariance
    Matrix measure_covariance_; ///< the measurement covariance
    Matrix input_covariance_; ///< the input covariance
    Matrix status_estimated_; ///< the status estimated matrix



};

} // namespace localization_module
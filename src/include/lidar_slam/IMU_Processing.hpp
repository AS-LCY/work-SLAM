#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H
#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>

#include <Eigen/Eigen>
// #include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
// #include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>


#include "lidar_slam/common_lib.h"
#include "lidar_slam/use-ikfom.hpp"
#include "lidar_slam/esekfom.hpp"
// #include "lddc.h"
#include "lidar/livox/ros_livox_datatype_def.h"

/*
这个hpp主要包含：
IMU数据预处理：IMU初始化，IMU正向传播，反向传播补偿运动失真   
*/

#define MAX_INI_COUNT (10)  //最大迭代次数
//判断点的时间先后顺序(注意curvature中存储的是时间戳)
static const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};
// 储存一帧lidar数据及imu数据序列
/*struct ImuMsg{
  double time_stamp;
  Vector3d angular_velocity;
  Vector3d linear_acceleration;
};*/
struct MeasureGroup     // Lidar data and imu dates for the curent process
{
    MeasureGroup()
    {
        lidar_beg_time = 0.0;
        lidar_end_time = 0.0;
        this->lidar.reset(new PointCloudType());
        imu.clear();
    };
    double lidar_beg_time; // lidar data begin time in the MeasureGroup
    double lidar_end_time; // lidar data end time in the MeasureGroup
    PointCloudType::Ptr lidar;
    deque<std::shared_ptr<livox_ros::ImuMsg>> imu;
};
struct Pose6D{
  float  offset_time;// # the offset time of IMU measurement w.r.t the first lidar point
  float acc[3];//       # the preintegrated total acceleration (global frame) at the Lidar origin
  float gyr[3];//       # the unbiased angular velocity (body frame) at the Lidar origin
  float vel[3];//      # the preintegrated velocity (global frame) at the Lidar origin
  float pos[3];//       # the preintegrated position (global frame) at the Lidar origin
  float rot[9];//       # the preintegrated rotation (global frame) at the Lidar origin
};


template<typename T>
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &g, \
                const Matrix<T, 3, 1> &v, const Matrix<T, 3, 1> &p, const Matrix<T, 3, 3> &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; j++)  rot_kp.rot[i*3+j] = R(i,j);
    }
    return move(rot_kp);
}



class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  void set_param(const V3D &transl, const M3D &rot, const V3D &gyr, const V3D &acc, const V3D &gyr_bias, const V3D &acc_bias);
  Eigen::Matrix<double, 12, 12> Q;    //噪声协方差矩阵  对应论文式(8)中的Q
  void Process(const MeasureGroup &meas, esekfom::esekf &kf_state, PointCloudType::Ptr &pcl_un_);

  V3D cov_acc;             //加速度测量协方差
  V3D cov_gyr;             //角速度测量协方差
  V3D cov_acc_scale;       //外部传入的 初始加速度协方差
  V3D cov_gyr_scale;       //外部传入的 初始角速度协方差
  V3D cov_bias_gyr;        //角速度bias的协方差
  V3D cov_bias_acc;        //加速度bias的协方差
  double first_lidar_time; //当前帧第一个点云时间
  V3D mean_acc;                           //加速度均值,用于计算方差
  Matrix3d initial_rotate;

 private:
  void IMU_init(const MeasureGroup &meas, esekfom::esekf &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf &kf_state, PointCloudType &pcl_in_out);

  PointCloudType::Ptr cur_pcl_un_;        //当前帧点云未去畸变
  std::shared_ptr<livox_ros::ImuMsg> last_imu_;     // 上一帧imu
  vector<Pose6D> IMUpose;                 // 存储imu位姿(反向传播用)
  M3D Lidar_R_wrt_IMU;                    // lidar到IMU的旋转外参
  V3D Lidar_T_wrt_IMU;                    // lidar到IMU的平移外参
  V3D mean_gyr;                           //角速度均值，用于计算方差
  V3D angvel_last;                        //上一帧角速度
  V3D acc_s_last;                         //上一帧加速度
  double start_timestamp_;                //开始时间戳
  double last_lidar_end_time_;            //上一帧结束时间戳
  int init_iter_num = 1;                  //初始化迭代次数
  bool b_first_frame_ = true;             //是否是第一帧
  bool imu_need_init_ = true;             //是否需要初始化imu
};
#endif

#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <logTracer/tracer.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sys/stat.h>

#include <Eigen/Core>

#include "kiss_matcher/KISSMatcher.hpp"
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

#define PI_M (3.14159265358)
#define G_m_s2 (9.81) // Gravaty const in GuangDong/China
// #define G_m_s2 (9.783)         // Gravaty const in GuangZhou/China
#define DIM_STATE (18)	// Dimension of states (Let Dim(SO(3)) = 3)
#define DIM_PROC_N (12) // Dimension of process noise (Let Dim(SO(3)) = 3)

#define LIDAR_SP_LEN (2)
#define INIT_COV (1)
#define NUM_MATCH_POINTS (5)
#define MAX_MEAS_DIM (10000)

#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0
#define CONSTRAIN(v, min, max) ((v > min) ? ((v < max) ? v : max) : min)
#define ARRAY_FROM_EIGEN(mat) mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat) vector<decltype(mat)::Scalar>(mat.data(), mat.data() + mat.rows() * mat.cols())
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "Log/" + name))

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudType;
typedef vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;

typedef Vector3d V3D;
typedef Matrix3d M3D;
typedef Vector3f V3F;
typedef Matrix3f M3F;

#define CASE_STR(x) \
	case x:         \
		return #x;  \
		break;

#define RESET "\033[0m"
#define BLACK "\033[30m"			  /* Black */
#define RED "\033[31m"				  /* Red */
#define GREEN "\033[32m"			  /* Green */
#define YELLOW "\033[33m"			  /* Yellow */
#define BLUE "\033[34m"				  /* Blue */
#define MAGENTA "\033[35m"			  /* Magenta */
#define CYAN "\033[36m"				  /* Cyan */
#define WHITE "\033[37m"			  /* White */
#define BOLDBLACK "\033[1m\033[30m"	  /* Bold Black */
#define BOLDRED "\033[1m\033[31m"	  /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"	  /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"	  /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"	  /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"	  /* Bold White */

constexpr double RAD2DEGREE = 180.f / M_PI;
constexpr double DEGREE2RAD = M_PI / 180.f;

extern bool USE_WHEEL;

namespace common_status {
enum class LocalizationStatus : int {
	Inactive = 0,		  // l_inactive
	Relocalizing = 1,	  // l_relocalizing
	RelocalizeFailed = 2, // l_relocalize failed
	Normal = 3,			  // l_normal
	LowAccuracy = 4,	  // l_low_accuracy
	Failed = 5			  // l_failed
};

enum class MappingStatus : int {
	Inactive = 0,		  // m_inactive
	Relocalizing = 1,	  // m_relocalizing
	RelocalizeFailed = 2, // m_relocalize failed
	Standby = 3,		  // m_standby
	CreatingEle = 4,	  // m_creating_ele (not used)
	Failed = 5			  // m_failed
};

enum class LocalNodeStatus : int {
	Inactive = 0,			// inactive
	Normal = 1,				// normal
	LidarCallbackDelay = 2, // lidar cbk delay
	LocalizeThreadDelay = 3 // localize thread delay
};

enum class MappingNodeStatus : int {
	Inactive = 0,				  // inactive
	Normal = 1,					  // normal
	LidarCallbackDelay = 2,		  // lidar cbk delay
	SecMapRelocalThreadDelay = 3, // secmap-relocal thread delay
	LoopClosureThreadDelay = 4	  // loop_closure_thread_delay
};

enum class HealthStatus : int {
	AllOk = 0,	   // all ok
	ErrorStop = 1, // error, stop pub tf & odom
	ErrorReset = 2 // error, reset slam to IDLE
};

enum class SlamRunStatus : int {
	Inactive = 0,
	Normal = 1,
	SyncFailed = 2,
	PointCloudEmpty = 3,
	BeforeDownSampleTooFewPoints = 4,
	AfterDownSampleTooFewPoints = 5,
	LidarOccluded = 6,
	LioVelAbnormalInPredict = 7,
	LioVelAbnormalInUpdate = 8,
};

enum class SecmapRelocalThrdStatus : int {
	Inactive = 0,		  // inactive
	Relocalizing = 1,	  // relocalizing
	RelocalizeFailed = 2, // relocalize failed
	Normal = 3			  // normal
};
} // namespace common_status

struct GICPConfig {
	int num_threads_ = 4;
	int correspondence_randomness_ = 20;
	int max_num_iter_ = 20;

	double max_corr_dist_ = 1.0;
	double scale_factor_for_corr_dist_ = 5.0;
	double overlap_threshold_ = 90.0;
};

struct LoopClosureConfig {
	bool verbose_ = true;
	bool enable_global_registration_ = true;
	bool is_multilayer_env_ = false;
	size_t num_submap_keyframes_ = 11;
	size_t num_inliers_threshold_ = 100;
	double voxel_res_ = 0.1;
	double loop_detection_radius_;
	double loop_detection_timediff_threshold_;
	GICPConfig gicp_config_;
	kiss_matcher::KISSMatcherConfig matcher_config_;
};

struct RegOutput {
	bool is_valid_ = false;
	bool is_converged_ = false;
	size_t num_final_inliers_ = 0;
	double overlapness_ = 0.0;
	Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();
};

struct LocalizeStatus {
	bool converged = false;
	double fit_score = 0.f;
	int num_inliers = 0;
	double inlier_fraction = 0.f;
	double cost_time = 0.f;
};

struct WheelOdomData {
	double timestamp = 0.0;		   // second
	double linear_velocity = 0.0;  // m/s
	double angular_velocity = 0.0; // rad/s
};

/*struct StatesGroup
{
	StatesGroup() {
		this->rot_end = M3D::Identity();
		this->pos_end = Zero3d;
		this->vel_end = Zero3d;
		this->bias_g  = Zero3d;
		this->bias_a  = Zero3d;
		this->gravity = Zero3d;
		this->cov     = MD(DIM_STATE,DIM_STATE)::Identity() * INIT_COV;
		this->cov.block<9,9>(9,9) = MD(9,9)::Identity() * 0.00001;
	};

	StatesGroup(const StatesGroup& b) {
		this->rot_end = b.rot_end;
		this->pos_end = b.pos_end;
		this->vel_end = b.vel_end;
		this->bias_g  = b.bias_g;
		this->bias_a  = b.bias_a;
		this->gravity = b.gravity;
		this->cov     = b.cov;
	};

	StatesGroup& operator=(const StatesGroup& b)
	{
		this->rot_end = b.rot_end;
		this->pos_end = b.pos_end;
		this->vel_end = b.vel_end;
		this->bias_g  = b.bias_g;
		this->bias_a  = b.bias_a;
		this->gravity = b.gravity;
		this->cov     = b.cov;
		return *this;
	};

	StatesGroup operator+(const Matrix<double, DIM_STATE, 1> &state_add)
	{
		StatesGroup a;
		a.rot_end = this->rot_end * Exp(state_add(0,0), state_add(1,0), state_add(2,0));
		a.pos_end = this->pos_end + state_add.block<3,1>(3,0);
		a.vel_end = this->vel_end + state_add.block<3,1>(6,0);
		a.bias_g  = this->bias_g  + state_add.block<3,1>(9,0);
		a.bias_a  = this->bias_a  + state_add.block<3,1>(12,0);
		a.gravity = this->gravity + state_add.block<3,1>(15,0);
		a.cov     = this->cov;
		return a;
	};

	StatesGroup& operator+=(const Matrix<double, DIM_STATE, 1> &state_add)
	{
		this->rot_end = this->rot_end * Exp(state_add(0,0), state_add(1,0), state_add(2,0));
		this->pos_end += state_add.block<3,1>(3,0);
		this->vel_end += state_add.block<3,1>(6,0);
		this->bias_g  += state_add.block<3,1>(9,0);
		this->bias_a  += state_add.block<3,1>(12,0);
		this->gravity += state_add.block<3,1>(15,0);
		return *this;
	};

	Matrix<double, DIM_STATE, 1> operator-(const StatesGroup& b)
	{
		Matrix<double, DIM_STATE, 1> a;
		M3D rotd(b.rot_end.transpose() * this->rot_end);
		a.block<3,1>(0,0)  = Log(rotd);
		a.block<3,1>(3,0)  = this->pos_end - b.pos_end;
		a.block<3,1>(6,0)  = this->vel_end - b.vel_end;
		a.block<3,1>(9,0)  = this->bias_g  - b.bias_g;
		a.block<3,1>(12,0) = this->bias_a  - b.bias_a;
		a.block<3,1>(15,0) = this->gravity - b.gravity;
		return a;
	};

	void resetpose()
	{
		this->rot_end = M3D::Identity();
		this->pos_end = Zero3d;
		this->vel_end = Zero3d;
	}

	M3D rot_end;      // the estimated attitude (rotation matrix) at the end lidar point
	V3D pos_end;      // the estimated position at the end lidar point (world frame)
	V3D vel_end;      // the estimated velocity at the end lidar point (world frame)
	V3D bias_g;       // gyroscope bias
	V3D bias_a;       // accelerator bias
	V3D gravity;      // the estimated gravity acceleration
	Matrix<double, DIM_STATE, DIM_STATE>  cov;     // states covariance
};*/

template <typename T>
T rad2deg(T radians) {
	return radians * 180.0 / PI_M;
}

template <typename T>
T deg2rad(T degrees) {
	return degrees * PI_M / 180.0;
}

inline void print_pose(const Eigen::Isometry3d& T) {
	Eigen::Vector3d t = T.translation();
	Eigen::Vector3d ypr = T.rotation().eulerAngles(2, 1, 0);
	double yaw = ypr[0] * 180.0 / M_PI;
	double pitch = ypr[1] * 180.0 / M_PI;
	double roll = ypr[2] * 180.0 / M_PI;
	TRACE_INFO("Translation (x, y, z): %f, %f, %f", t.x(), t.y(), t.z());
	TRACE_INFO("Rotation (yaw, pitch, roll) [deg]:  %f, %f, %f", yaw, pitch, roll);
}

/* comment
plane equation: Ax + By + Cz + D = 0
convert to: A/D*x + B/D*y + C/D*z = -1
solve: A0*x0 = b0
where A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1, ..., -1]^T
normvec:  normalized x0
*/

/***************************************************************************************************
template<typename T>
static bool esti_normvector(Matrix<T, 3, 1> &normvec, const PointVector &point, const T &threshold, const int
&point_num)
{
	MatrixXf A(point_num, 3);
	MatrixXf b(point_num, 1);
	b.setOnes();
	b *= -1.0f;

	for (int j = 0; j < point_num; j++)
	{
		A(j,0) = point[j].x;
		A(j,1) = point[j].y;
		A(j,2) = point[j].z;
	}
	normvec = A.colPivHouseholderQr().solve(b);

	for (int j = 0; j < point_num; j++)
	{
		if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y + normvec(2) * point[j].z + 1.0f) > threshold)
		{
			return false;
		}
	}

	normvec.normalize();
	return true;
}
**************************************************************************************************/

template <typename T>
static bool esti_plane(Matrix<T, 4, 1>& pca_result, const PointVector& point, const T& threshold) {
	Matrix<T, NUM_MATCH_POINTS, 3> A;
	Matrix<T, NUM_MATCH_POINTS, 1> b;
	A.setZero();
	b.setOnes();
	b *= -1.0f;

	for (int j = 0; j < NUM_MATCH_POINTS; j++) {
		A(j, 0) = point[j].x;
		A(j, 1) = point[j].y;
		A(j, 2) = point[j].z;
	}

	Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

	T n = normvec.norm();
	pca_result(0) = normvec(0) / n;
	pca_result(1) = normvec(1) / n;
	pca_result(2) = normvec(2) / n;
	pca_result(3) = 1.0 / n;

	for (int j = 0; j < NUM_MATCH_POINTS; j++) {
		if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) >
			threshold) {
			return false;
		}
	}
	return true;
}

Eigen::Vector3d R2ypr(const Eigen::Matrix3d& R);

Eigen::Matrix3d ypr2R(const Eigen::Vector3d& ypr);

void get_xyz_ypr(const Eigen::Isometry3d& eigen_transform, Eigen::Vector3d& xyz, Eigen::Vector3d& ypr);

Eigen::Matrix3d rpy2R(const Eigen::Vector3d& rpy);

Eigen::Matrix3d g2R(const Eigen::Vector3d& g);

template <typename T>
inline pcl::PointCloud<T> transformPcd(const pcl::PointCloud<T>& cloud_in, const Eigen::Matrix4d& pose) {
	if (cloud_in.empty()) {
		return cloud_in;
	}
	pcl::PointCloud<T> cloud_out;
	pcl::transformPointCloud(cloud_in, cloud_out, pose);
	return cloud_out;
}

std::vector<Eigen::Vector3f> convertCloudToVec(const pcl::PointCloud<pcl::PointXYZI>& cloud);

PointCloudType::Ptr transformPointCloud(PointCloudType::Ptr cloudIn, const Eigen::Isometry3d& transCur);

inline void orthonormalizeIsometry(Eigen::Isometry3d& T) {
	Eigen::Matrix3d R = T.rotation();

	// 对 R 做 SVD 分解
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d R_ortho = svd.matrixU() * svd.matrixV().transpose();

	// 确保 det(R) == +1（避免反射矩阵）
	if (R_ortho.determinant() < 0) {
		Eigen::Matrix3d U = svd.matrixU();
		U.col(2) *= -1;
		R_ortho = U * svd.matrixV().transpose();
	}

	T.linear() = R_ortho;
}

inline Eigen::Isometry3d convertSE3dToIsometry3d(const Sophus::SE3d& T_sophus) {
	Eigen::Matrix3d R = T_sophus.rotationMatrix();
	Eigen::Vector3d t = T_sophus.translation();
	Eigen::Isometry3d T_eigen = Eigen::Isometry3d::Identity();
	T_eigen.linear() = R;
	T_eigen.translation() = t;
	return T_eigen;
}

inline Sophus::SE3d convertIsometry3dToSE3d(Eigen::Isometry3d T_eigen) {
	orthonormalizeIsometry(T_eigen);
	Eigen::Matrix3d R = T_eigen.rotation();
	Eigen::Vector3d t = T_eigen.translation();
	Sophus::SE3d T_sophus(R, t);
	return T_sophus;
}

inline Eigen::Matrix3d skew_sym_matrix(const Eigen::Vector3d& v) {
	Eigen::Matrix3d m;
	m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
	return m;
}

// adjoint of SE3 (6x6)
inline Eigen::Matrix<double, 6, 6> adjointSE3(const Sophus::SE3d& T) {
	Eigen::Matrix3d R = T.rotationMatrix();
	Eigen::Vector3d t = T.translation();
	Eigen::Matrix<double, 6, 6> Ad = Eigen::Matrix<double, 6, 6>::Zero();
	Ad.template block<3, 3>(0, 0) = R;
	Ad.template block<3, 3>(0, 3) = skew_sym_matrix(t) * R;
	Ad.template block<3, 3>(3, 3) = R;
	return Ad;
}

inline void compute_relative_cov(const Sophus::SE3d& Tj, const Eigen::Matrix<double, 6, 6>& cov_Tj,
								 const Sophus::SE3d& Tk, const Eigen::Matrix<double, 6, 6>& cov_Tk, Sophus::SE3d& Tjk,
								 Eigen::Matrix<double, 6, 6>& Tjk_cov_local) {
	Tjk = Tj.inverse() * Tk;

	// 假设认为Tj和Tk两个位姿之间是相互独立的，不考虑联合分布，实际中两个位姿是有依赖关系的。
	// Tj，Tk两个位姿的方差是全局坐标系下的方差，不是在当前位姿下(局部坐标系下)的方差。
	// 返回的cov_Tjk是在位姿Tj下的方差。
	Eigen::Matrix<double, 6, 6> Tj_inv_adj = Tj.inverse().Adj();
	Eigen::Matrix<double, 6, 6> Tjk_cov_global =
		Tj_inv_adj * cov_Tj * Tj_inv_adj.transpose() + Tj_inv_adj * cov_Tk * Tj_inv_adj.transpose();

	Tjk_cov_local = Tj.Adj().inverse() * Tjk_cov_global * Tj.Adj().inverse().transpose();
}

inline Eigen::Matrix<double, 6, 6> compute_global_cov(const Sophus::SE3d& Ti,
													  const Eigen::Matrix<double, 6, 6>& cov_Ti_local) {
	//已知Ti在某全局坐标系下的位姿和在Ti local系下的方差，计算Ti在全局坐标系下的方差
	Eigen::Matrix<double, 6, 6> Ti_adj = Ti.Adj();
	Eigen::Matrix<double, 6, 6> cov_Ti_global = Ti_adj * cov_Ti_local * Ti_adj.transpose();
	return cov_Ti_global;
}

inline Eigen::Isometry3d makeTransform(const std::vector<double>& trans, const std::vector<double>& rot) {
	assert(trans.size() == 3 && rot.size() == 3);
	double tx = trans[0];
	double ty = trans[1];
	double tz = trans[2];
	double yaw = rot[0] * DEGREE2RAD;
	double pitch = rot[1] * DEGREE2RAD;
	double roll = rot[2] * DEGREE2RAD;

	// 按照 R = Rz(yaw) * Ry(pitch) * Rx(roll)
	Eigen::AngleAxisd Rz(yaw, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd Rx(roll, Eigen::Vector3d::UnitX());
	Eigen::Matrix3d R = (Rz * Ry * Rx).matrix();
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	T.linear() = R;
	T.translation() = Eigen::Vector3d(tx, ty, tz);
	return T;
}

Eigen::Vector3d calc_baselink_vel_from_lio_imu_state(const Eigen::Vector3d& imu_world_vel,
													 const Eigen::Matrix3d& imu_world_R,
													 const Eigen::Isometry3d& T_imu_baselink,
													 const Eigen::Vector3d& imu_gyro);
inline bool isSO3(const Eigen::Matrix3d& R, double tol = 1e-6, bool verbose = false) {
	Eigen::Matrix3d shouldBeIdentity = R.transpose() * R;
	Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
	double detR = R.determinant();

	bool orthogonal = shouldBeIdentity.isApprox(I, tol);
	bool det_one = std::abs(detR - 1.0) < tol;

	if (verbose) {
		std::cout << "R^T R:\n" << shouldBeIdentity << std::endl;
		std::cout << "det(R): " << detR << std::endl;
	}

	return orthogonal && det_one;
}

inline Eigen::Isometry3d smoothUpdateTransform(Eigen::Isometry3d T_old, Eigen::Isometry3d T_new,
											   const double alpha = 0.1) {
	orthonormalizeIsometry(T_old);
	orthonormalizeIsometry(T_new);
	Sophus::SE3d T_old_SE3(T_old.rotation(), T_old.translation());
	Sophus::SE3d T_new_SE3(T_new.rotation(), T_new.translation());

	Sophus::SE3d delta_T = T_new_SE3 * T_old_SE3.inverse();
	Eigen::Matrix<double, 6, 1> delta_se3 = delta_T.log();
	Eigen::Matrix<double, 6, 1> delta_smoothed = alpha * delta_se3; //指数平滑（插值）
	Sophus::SE3d T_smoothed_SE3 = Sophus::SE3d::exp(delta_smoothed) * T_old_SE3;

	Eigen::Isometry3d T_smoothed = Eigen::Isometry3d::Identity();
	T_smoothed.linear() = T_smoothed_SE3.rotationMatrix();
	T_smoothed.translation() = T_smoothed_SE3.translation();
	return T_smoothed;
}

static float angle_norm(float a) {
	if (a < -PI_M) {
		return a + PI_M * 2;
	} else if (a > PI_M) {
		return a - PI_M * 2;
	}
	return a;
}

bool mkdir_p(const std::string& path, mode_t mode);
bool create_directory_if_not_exists(const std::string& directory_path);

inline std::string LocalizationStatustoString(common_status::LocalizationStatus status) {
	switch (status) {
		case common_status::LocalizationStatus::Inactive:
			return "Inactive";
		case common_status::LocalizationStatus::Relocalizing:
			return "Relocalizing";
		case common_status::LocalizationStatus::RelocalizeFailed:
			return "RelocalizeFailed";
		case common_status::LocalizationStatus::Normal:
			return "Normal";
		case common_status::LocalizationStatus::LowAccuracy:
			return "LowAccuracy";
		case common_status::LocalizationStatus::Failed:
			return "Failed";
		default:
			return "Unknown";
	}
}

inline std::string MappingStatustoString(common_status::MappingStatus status) {
	switch (status) {
		case common_status::MappingStatus::Inactive:
			return "Inactive";
		case common_status::MappingStatus::Relocalizing:
			return "Relocalizing";
		case common_status::MappingStatus::RelocalizeFailed:
			return "RelocalizeFailed";
		case common_status::MappingStatus::Standby:
			return "Standby";
		case common_status::MappingStatus::CreatingEle:
			return "CreatingEle";
		case common_status::MappingStatus::Failed:
			return "Failed";
		default:
			return "Unknown";
	}
}

inline std::string SlamRunStatustoString(common_status::SlamRunStatus status) {
	switch (status) {
		case common_status::SlamRunStatus::Inactive:
			return "Inactive";
		case common_status::SlamRunStatus::Normal:
			return "Normal";
		case common_status::SlamRunStatus::SyncFailed:
			return "SyncFailed";
		case common_status::SlamRunStatus::PointCloudEmpty:
			return "PointCloudEmpty";
		case common_status::SlamRunStatus::BeforeDownSampleTooFewPoints:
			return "BeforeDownSampleTooFewPoints";
		case common_status::SlamRunStatus::AfterDownSampleTooFewPoints:
			return "AfterDownSampleTooFewPoints";
		case common_status::SlamRunStatus::LidarOccluded:
			return "LidarOccluded";
		case common_status::SlamRunStatus::LioVelAbnormalInPredict:
			return "LioVelAbnormalInPredict";
		case common_status::SlamRunStatus::LioVelAbnormalInUpdate:
			return "LioVelAbnormalInUpdate";
		default:
			return "Unknown";
	}
}

#endif


#ifndef FLBOT_LOCALIZATION_COMMON_QUATERNION_H
#define FLBOT_LOCALIZATION_COMMON_QUATERNION_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <Eigen/Dense>
#include <Eigen/Eigen>

namespace localization_module {
namespace common {

/// A const value that equals to 2*pi
const double M_PI2 = M_PI * 2;

/// @brief The Quaternion Class includes most of the commonly used quaternion operations, and convertion between Euler
/// Angle and Quaternions.
class Quaternion {
   public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static double rad2deg(double rad);
	static double deg2rad(double deg);

	static geometry_msgs::Quaternion eigen_quat_2_geo_quat(const Eigen::Quaterniond eq);
	static Eigen::Quaterniond geo_quat_2_eigen_quat(const geometry_msgs::Quaternion gq);

	/// @brief The static method rotate_vector3 rotates the input vector with the rotation decribed by the input
	/// quaterion.
	/// @param vin with type [geometry_msgs::Vector3](http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html) is
	/// the input vector to be rotated.
	/// @param quat with type
	/// [geometry_msgs::Quaternion](http://docs.ros.org/api/geometry_msgs/html/msg/Quaternion.html) is the rotation
	/// described in quaternion.
	/// @param is_reverse_flag with type bool is the flag to idicate whether to rotate backward, default to false.
	/// @return The function returns the rotated vector.
	static geometry_msgs::Vector3 rotate_vector3(const geometry_msgs::Vector3& vin,
												 const geometry_msgs::Quaternion& quat,
												 const bool is_reverse_flag = false);

	/// @brief The static method get_euler_zyx convers a rotation from quaternion form to euler form with the order
	/// z-y-x (or yaw-pitch-roll)
	/// @param quat with type
	/// [geometry_msgs::Quaternion](http://docs.ros.org/api/geometry_msgs/html/msg/Quaternion.html) is the rotation in
	/// quaternion form
	/// @return The function returns the rotation in euler form.
	static geometry_msgs::Vector3 get_euler_zyx(const geometry_msgs::Quaternion& quat);

	/// @brief The static method get_quaternion convers a rotation from euler form to quaternion form
	/// @param euler_zyx is the rotation in euler form with the rotation order z-y-x (or yaw-pitch-roll)
	/// @return The function returns the rotation in quaternion form.
	static geometry_msgs::Quaternion get_quaternion(const geometry_msgs::Vector3& euler_zyx);
	static geometry_msgs::Quaternion get_quaternion(const std::vector<double>& euler_zyx);
	static Eigen::Quaterniond get_eigen_quaternion(const std::vector<double>& euler_zyx);

	/// @brief Get the quaternion that can rotate vector v1 to vector v2
	/// @param v1 The first vector that the rotate begins
	/// @param v2 The second vector that the rotate ends
	/// @return the quaternion that can rotate vector v1 to vector v2
	static geometry_msgs::Quaternion get_quaternion(const geometry_msgs::Vector3& v1, const geometry_msgs::Vector3& v2);

	/// @brief This method multiplies two quaternions, such that the two rotations becomes cascaded
	/// @param q1 The first rotation described by a quaternion, must be the rotation happens first
	/// @param q2 The second rotation described by a quaternion, must be the rotation happers second
	/// @return The function returns a quaternion that describes the rotation equivalent to executing the two rotions
	/// one after the other
	static geometry_msgs::Quaternion multiply(const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2);

	/// @brief This method gets the conjugate quaternion
	/// @param quat The quaternion to get conjugate of
	/// @return The function returns the conjugate quaternion of the input quaternion
	static geometry_msgs::Quaternion get_conjugate(const geometry_msgs::Quaternion& quat);

	/// @brief This method gets the linear interpolation quaternion
	/// @param quat1 The first quaternion
	/// @param quat2 The second quaternion
	/// @param t The interpolation parameter, must be in range [0, 1]
	/// @return The function returns the quaternion that is the linear interpolation of the two input quaternions
	static geometry_msgs::Quaternion slerp(const geometry_msgs::Quaternion& quat1,
										   const geometry_msgs::Quaternion& quat2, const double t);

}; // class Quaternion

} // namespace common
} // namespace localization_module

#endif // FLBOT_LOCALIZATION_COMMON_QUATERNION_H

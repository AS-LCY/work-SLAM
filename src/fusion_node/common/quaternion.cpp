#include "quaternion.h"

#include <cmath>
#include <iostream>

#include "vector.h"

namespace localization_module {
namespace common {

double Quaternion::rad2deg(double rad) { return rad / M_PI * 180.0; }

double Quaternion::deg2rad(double deg) { return deg * M_PI / 180.0; }

/// @brief /////////////////////////////////////////////////////////////////////////////////////
/// @param e_quat
/// @return
geometry_msgs::Quaternion Quaternion::eigen_quat_2_geo_quat(const Eigen::Quaterniond e_quat) {
	geometry_msgs::Quaternion g_quat;
	g_quat.x = e_quat.x();
	g_quat.y = e_quat.y();
	g_quat.z = e_quat.z();
	g_quat.w = e_quat.w();

	return g_quat;
}

Eigen::Quaterniond Quaternion::geo_quat_2_eigen_quat(const geometry_msgs::Quaternion g_quat) {
	Eigen::Quaterniond e_quat;
	e_quat.x() = g_quat.x;
	e_quat.y() = g_quat.y;
	e_quat.z() = g_quat.z;
	e_quat.w() = g_quat.w;

	return e_quat;
}

/// @brief /////////////////////////////////////////////////////////////////////////////////////
/// @param quat
/// @return
geometry_msgs::Vector3 Quaternion::rotate_vector3(const geometry_msgs::Vector3&	   vin,
												  const geometry_msgs::Quaternion& quat, const bool is_reverse_flag) {
	double x = quat.x;
	double y = quat.y;
	double z = quat.z;
	double w = quat.w;
	double r[3][3];

	r[0][0] = 1 - 2 * y * y - 2 * z * z;
	r[0][1] = 2 * x * y - 2 * w * z;
	r[0][2] = 2 * w * y + 2 * x * z;

	r[1][0] = 2 * x * y + 2 * w * z;
	r[1][1] = 1 - 2 * x * x - 2 * z * z;
	r[1][2] = -2 * w * x + 2 * z * y;

	r[2][0] = -2 * w * y + 2 * x * z;
	r[2][1] = 2 * w * x + 2 * y * z;
	r[2][2] = 1 - 2 * x * x - 2 * y * y;

	geometry_msgs::Vector3 ans;
	if (false == is_reverse_flag) {
		ans.x = r[0][0] * vin.x + r[0][1] * vin.y +
				r[0][2] * vin.z; // To rotate a vector is to transform a point from one frame to another
		ans.y = r[1][0] * vin.x + r[1][1] * vin.y + r[1][2] * vin.z;
		ans.z = r[2][0] * vin.x + r[2][1] * vin.y + r[2][2] * vin.z;
	} else {
		ans.x = r[0][0] * vin.x + r[1][0] * vin.y +
				r[2][0] * vin.z; // This is the reverse of the above since the matrix is transposed.
		ans.y = r[0][1] * vin.x + r[1][1] * vin.y + r[2][1] * vin.z;
		ans.z = r[0][2] * vin.x + r[1][2] * vin.y + r[2][2] * vin.z;
	}
	return ans;
};

geometry_msgs::Vector3 Quaternion::get_euler_zyx(const geometry_msgs::Quaternion& quat) {
	double x = quat.x;
	double y = quat.y;
	double z = quat.z;
	double w = quat.w;

	geometry_msgs::Vector3 vect3;
	vect3.x = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
	vect3.y = asin(2 * (w * y - z * x));
	vect3.z = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

	return vect3;
};

Eigen::Quaterniond Quaternion::get_eigen_quaternion(const std::vector<double>& euler_zyx) {
	geometry_msgs::Quaternion quat		 = get_quaternion(euler_zyx);
	Eigen::Quaterniond		  eigen_quat = geo_quat_2_eigen_quat(quat);
	return eigen_quat;
}

geometry_msgs::Quaternion Quaternion::get_quaternion(const std::vector<double>& euler_zyx) {
	geometry_msgs::Vector3 geo_euler_zyx;
	geo_euler_zyx.x = euler_zyx[0];
	geo_euler_zyx.y = euler_zyx[1];
	geo_euler_zyx.z = euler_zyx[2];

	geometry_msgs::Quaternion quat;
	quat = get_quaternion(geo_euler_zyx);

	return quat;
}

geometry_msgs::Quaternion Quaternion::get_quaternion(const geometry_msgs::Vector3& euler_zyx) {
	double r = euler_zyx.x * 0.5;
	double p = euler_zyx.y * 0.5;
	double y = euler_zyx.z * 0.5;

	double sinr = sin(r);
	double cosr = cos(r);
	double sinp = sin(p);
	double cosp = cos(p);
	double siny = sin(y);
	double cosy = cos(y);

	geometry_msgs::Quaternion quat;
	quat.x = sinr * cosp * cosy - cosr * sinp * siny;
	quat.y = cosr * sinp * cosy + sinr * cosp * siny;
	quat.z = cosr * cosp * siny - sinr * sinp * cosy;
	quat.w = cosr * cosp * cosy + sinr * sinp * siny;

	return quat;
};

geometry_msgs::Quaternion Quaternion::multiply(const geometry_msgs::Quaternion& q1,
											   const geometry_msgs::Quaternion& q2) {
	geometry_msgs::Quaternion quat;
	quat.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
	quat.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
	quat.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
	quat.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
	return quat;
};

geometry_msgs::Quaternion Quaternion::get_quaternion(const geometry_msgs::Vector3& v1,
													 const geometry_msgs::Vector3& v2) {
	double					  len1	 = Vector::norm2(v1);
	double					  len2	 = Vector::norm2(v2);
	double					  lenlen = len1 * len2;
	geometry_msgs::Quaternion quat;
	if (lenlen < 1e-6) {
		std::cerr << "mango::common::Quaternion::get_quaternion()" << std::endl
				  << "Cannot generate quaternion with zero vector." << std::endl;
		throw "Cannot generate quaternion with zero vector.";
	}
	double				   angle = Vector::dot(v1, v2) / lenlen; // dot = len1 * len2 * cos(theta)
	geometry_msgs::Vector3 axis	 = Vector::cross(v1, v2);
	double				   alen	 = Vector::norm2(axis);
	double				   theta = 0.0;
	if (alen > 1e-6) {
		axis  = Vector::scale(axis, 1 / alen);
		theta = acos(angle) / 2;
	}
	double st = sin(theta);
	quat.x	  = axis.x * st;
	quat.y	  = axis.y * st;
	quat.z	  = axis.z * st;
	quat.w	  = cos(theta);
	return quat;
};

geometry_msgs::Quaternion Quaternion::get_conjugate(const geometry_msgs::Quaternion& quat) {
	geometry_msgs::Quaternion conjugate;
	conjugate.x = -quat.x;
	conjugate.y = -quat.y;
	conjugate.z = -quat.z;
	conjugate.w = quat.w;
	return conjugate;
};

geometry_msgs::Quaternion Quaternion::slerp(const geometry_msgs::Quaternion& quat1,
											const geometry_msgs::Quaternion& quat2, const double t) {
	double cosom = quat1.x * quat2.x + quat1.y * quat2.y + quat1.z * quat2.z + quat1.w * quat2.w;
	int	   sign	 = cosom > 0 ? 1 : -1;

	double sclp = 0;
	double sclq = 0;
	if ((1.0 - fabs(cosom)) > 1e-6) {
		double omega = acos(fabs(cosom));
		double sinom = sin(omega);
		sclp		 = sin((1.0 - t) * omega) / sinom;
		sclq		 = sin(t * omega) / sinom;
	} else {
		sclp = 1.0 - t;
		sclq = t;
	}

	geometry_msgs::Quaternion quat;
	quat.x = sclp * quat1.x + sclq * quat2.x * sign;
	quat.y = sclp * quat1.y + sclq * quat2.y * sign;
	quat.z = sclp * quat1.z + sclq * quat2.z * sign;
	quat.w = sclp * quat1.w + sclq * quat2.w * sign;
	return quat;
}

} // namespace common
} // namespace localization_module

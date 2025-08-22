#include "vector.h"

#include <math.h>

#include <iostream>

namespace localization_module {
namespace common {

////////////////////////////////////////////////////////////
///                ELEMENTS - OPERATIONS                 ///
////////////////////////////////////////////////////////////

void Vector::toggle_type(const geometry_msgs::Vector3& vct, geometry_msgs::Point* pt) {
	pt->x = vct.x;
	pt->y = vct.y;
	pt->z = vct.z;
};

geometry_msgs::Point Vector::toggle_type(const geometry_msgs::Vector3& vct) {
	geometry_msgs::Point pt;
	toggle_type(vct, &pt);
	return pt;
};

void Vector::toggle_type(const geometry_msgs::Point& pt, geometry_msgs::Vector3* vct) {
	vct->x = pt.x;
	vct->y = pt.y;
	vct->z = pt.z;
};

geometry_msgs::Vector3 Vector::toggle_type(const geometry_msgs::Point& pt) {
	geometry_msgs::Vector3 vct;
	toggle_type(pt, &vct);
	return vct;
};

////////////////////////////////////////////////////////////
///                 BASIC +/- OPERATIONS                 ///
////////////////////////////////////////////////////////////

std::vector<double> Vector::add(const std::vector<double>& vct1, const std::vector<double>& vct2) {
	std::vector<double> result;
	if (vct1.size() != vct2.size()) {
		std::cerr << "localization_module::common::Vector::add()" << std::endl
				  << "Vectors must have the same size when doing add. size1=" << vct1.size()
				  << ", size2=" << vct2.size() << std::endl;
		throw "Vectors must have the same size when doing add.";
		return result;
	}
	std::vector<double>::const_iterator it1;
	std::vector<double>::const_iterator it2;
	result.reserve(vct1.size());
	for (it1 = vct1.begin(), it2 = vct2.begin(); it1 != vct1.end() && it2 != vct2.end(); ++it1, ++it2) {
		result.push_back((*it1) + (*it2));
	};
	return result;
};

geometry_msgs::Vector3 Vector::add(const geometry_msgs::Vector3& vct1, const geometry_msgs::Vector3& vct2) {
	geometry_msgs::Vector3 result;
	result.x = vct1.x + vct2.x;
	result.y = vct1.y + vct2.y;
	result.z = vct1.z + vct2.z;
	return result;
};

geometry_msgs::Point Vector::add(const geometry_msgs::Point& pt1, const geometry_msgs::Point& pt2) {
	geometry_msgs::Point result;
	result.x = pt1.x + pt2.x;
	result.y = pt1.y + pt2.y;
	result.z = pt1.z + pt2.z;
	return result;
};

std::vector<double> Vector::sub(const std::vector<double>& vct1, const std::vector<double>& vct2) {
	std::vector<double> result;
	if (vct1.size() != vct2.size()) {
		std::cerr << "localization_module::common::Vector::sub()" << std::endl
				  << "Vectors must have the same size when doing sub. size1=" << vct1.size()
				  << ", size2=" << vct2.size() << std::endl;
		throw "Vectors must have the same size when doing sub.";
		return result;
	}
	std::vector<double>::const_iterator it1;
	std::vector<double>::const_iterator it2;
	result.reserve(vct1.size());
	for (it1 = vct1.begin(), it2 = vct2.begin(); it1 != vct1.end() && it2 != vct2.end(); ++it1, ++it2) {
		result.push_back((*it1) - (*it2));
	};
	return result;
};

geometry_msgs::Vector3 Vector::sub(const geometry_msgs::Vector3& vct1, const geometry_msgs::Vector3& vct2) {
	geometry_msgs::Vector3 result;
	result.x = vct1.x - vct2.x;
	result.y = vct1.y - vct2.y;
	result.z = vct1.z - vct2.z;
	return result;
};

geometry_msgs::Point Vector::sub(const geometry_msgs::Point& pt1, const geometry_msgs::Point& pt2) {
	geometry_msgs::Point result;
	result.x = pt1.x - pt2.x;
	result.y = pt1.y - pt2.y;
	result.z = pt1.z - pt2.z;
	return result;
};

double Vector::sum(const std::vector<double>& vct) {
	double rtn = 0.0;
	for (auto i : vct) {
		rtn += i;
	}
	return rtn;
}
////////////////////////////////////////////////////////////
///              VECTOR SCALING OPERATIONS               ///
////////////////////////////////////////////////////////////

std::vector<double> Vector::scale(const std::vector<double>& vct, const double& scaler) {
	std::vector<double>					result;
	std::vector<double>::const_iterator it;
	result.reserve(vct.size());
	for (it = vct.begin(); it != vct.end(); ++it) {
		result.push_back((*it) * scaler);
	};
	return result;
};

geometry_msgs::Vector3 Vector::scale(const geometry_msgs::Vector3& vct, const double& scaler) {
	geometry_msgs::Vector3 result;
	result.x = vct.x * scaler;
	result.y = vct.y * scaler;
	result.z = vct.z * scaler;
	return result;
};

geometry_msgs::Point Vector::scale(const geometry_msgs::Point& pt, const double& scaler) {
	geometry_msgs::Point result;
	result.x = pt.x * scaler;
	result.y = pt.y * scaler;
	result.z = pt.z * scaler;
	return result;
};

////////////////////////////////////////////////////////////
///        DOT - PRODUCTION - OPERATIONS                 ///
////////////////////////////////////////////////////////////

double Vector::dot(const std::vector<double>& vct1, const std::vector<double>& vct2) {
	if (vct1.size() != vct2.size()) {
		std::cerr << "localization_module::common::Vector::dot()" << std::endl
				  << "Vectors must have the same size when doing dot. size1=" << vct1.size()
				  << ", size2=" << vct2.size() << std::endl;
		throw "Vectors must have the same size when doing dot.";
		return 0;
	}
	std::vector<double>::const_iterator it1;
	std::vector<double>::const_iterator it2;
	double								sum = 0.0;
	for (it1 = vct1.begin(), it2 = vct2.begin(); it1 != vct1.end() && it2 != vct2.end(); ++it1, ++it2) {
		sum += (*it1) * (*it2);
	};
	return sum;
};

double Vector::dot(const geometry_msgs::Vector3& vct1, const geometry_msgs::Vector3& vct2, bool use_2D) {
	if (use_2D) {
		return vct1.x * vct2.x + vct1.y * vct2.y;
	} else {
		return vct1.x * vct2.x + vct1.y * vct2.y + vct1.z * vct2.z;
	}
};

double Vector::dot(const geometry_msgs::Point& pt1, const geometry_msgs::Point& pt2, bool use_2D) {
	return dot(toggle_type(pt1), toggle_type(pt2), use_2D);
};

////////////////////////////////////////////////////////////
///                  NORM2 - OPERATIONS                  ///
////////////////////////////////////////////////////////////

double Vector::norm2(const std::vector<double>& vct) { return sqrt(dot(vct, vct)); };

double Vector::norm2(const geometry_msgs::Vector3& vct3, bool use_2D) { return sqrt(dot(vct3, vct3, use_2D)); };

double Vector::norm2(const geometry_msgs::Point& pt, bool use_2D) { return sqrt(dot(pt, pt, use_2D)); };

double Vector::norm2(const geometry_msgs::Quaternion qt) {
	return sqrt(qt.x * qt.x + qt.y * qt.y + qt.z * qt.z + qt.w * qt.w);
};

////////////////////////////////////////////////////////////
///              CROSS- PRODUCT - OPERATIONS             ///
////////////////////////////////////////////////////////////

geometry_msgs::Vector3 Vector::cross(const geometry_msgs::Vector3& vct1, const geometry_msgs::Vector3& vct2) {
	geometry_msgs::Vector3 result;
	result.x = vct1.y * vct2.z - vct1.z * vct2.y;
	result.y = vct1.z * vct2.x - vct1.x * vct2.z;
	result.z = vct1.x * vct2.y - vct1.y * vct2.x;
	return result;
};

double Vector::cross(const geometry_msgs::Vector3& vct1, const geometry_msgs::Vector3& vct2, bool use_2D) {
	return vct1.x * vct2.y - vct1.y * vct2.x;
};

geometry_msgs::Point Vector::cross(const geometry_msgs::Point& pt1, const geometry_msgs::Point& pt2) {
	return toggle_type(cross(toggle_type(pt1), toggle_type(pt2)));
};

double Vector::cross(const geometry_msgs::Point& pt1, const geometry_msgs::Point& pt2, bool use_2D) {
	return cross(toggle_type(pt1), toggle_type(pt2), use_2D);
};

////////////////////////////////////////////////////////////
///                   OTHER - OPERATIONS                 ///
////////////////////////////////////////////////////////////
double Vector::distance(const geometry_msgs::Vector3& vct1, const geometry_msgs::Vector3& vct2, bool use_2D) {
	return norm2(sub(vct1, vct2), use_2D);
};

double Vector::distance(const geometry_msgs::Point& pt1, const geometry_msgs::Point& pt2, bool use_2D) {
	return distance(toggle_type(pt1), toggle_type(pt2), use_2D);
};

bool Vector::identical(const std::vector<double>& v1, const std::vector<double>& v2) {
	if (v1.size() != v2.size()) {
		return false;
	}
	std::vector<double>::const_iterator itr1, itr2;
	for (itr1 = v1.begin(), itr2 = v2.begin(); itr1 != v1.end() && itr2 != v2.end(); ++itr1, ++itr2) {
		if (fabs(*itr1 - *itr2) > 1e-9) {
			return false;
		}
	}
	return true;
}

bool Vector::identical(const geometry_msgs::Point& pt1, const geometry_msgs::Point& pt2) {
	if (fabs(pt1.x - pt2.x) > 1e-9 || fabs(pt1.y - pt2.y) > 1e-9 || fabs(pt1.z - pt2.z) > 1e-9) {
		return false;
	}
	return true;
}

} // namespace common
} // namespace localization_module

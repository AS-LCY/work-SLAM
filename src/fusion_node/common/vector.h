
#ifndef FLBOT_LOCALIZATION_COMMON_VECTOR_H
#define FLBOT_LOCALIZATION_COMMON_VECTOR_H

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

namespace localization_module {
namespace common {

/// @brief The basic operations of vectors
class Vector {
public:

////////////////////////////////////////////////////////////
///                ELEMENTS - OPERATIONS                 ///
////////////////////////////////////////////////////////////

/// @brief convert message type from geometry::Vector3 to geometry::Point
/// @param vct the input vector of type Vector3
/// @param pt the pointer to the variable with type Point
    static void toggle_type(const geometry_msgs::Vector3& vct, geometry_msgs::Point* pt);

/// @brief convert message type from geometry::Vector3 to geometry::Point
/// @param vct the input vector of type Vector3
/// @return the converted vector with the type of Point
    static geometry_msgs::Point toggle_type(const geometry_msgs::Vector3& vct);

/// @brief convert message type from geometry::Point to geometry::Vector3
/// @param pt the input vector of type Point
/// @param vct the pointer to the variable with type Vector3
    static void toggle_type(const geometry_msgs::Point& pt, geometry_msgs::Vector3* vct);

/// @brief convert message type from geometry::Point to geometry::Vector3
/// @param pt the input vector of type Point
/// @return the converted vector with the type of Vector3
    static geometry_msgs::Vector3 toggle_type(const geometry_msgs::Point& pt);

////////////////////////////////////////////////////////////
///                 BASIC +/- OPERATIONS                 ///
////////////////////////////////////////////////////////////

/// @brief add each element of two vectors
/// @param vct1 the first vector
/// @param vct2 the second vector
/// @return the sum of the two vectors
    static std::vector<double> add(const std::vector<double>& vct1,
                                   const std::vector<double>& vct2);

/// @brief add each element of two geometry_msgs::Vector3
/// @param vct1 the first vector
/// @param vct2 the second vector
/// @return the sum of the two given vectors
    static geometry_msgs::Vector3 add(const geometry_msgs::Vector3& vct1,
                                      const geometry_msgs::Vector3& vct2);

/// @brief add each element of two geometry_msgs::Point
/// @param pt1 the first point
/// @param pt2 the second point
/// @return the sum of the two given points
    static geometry_msgs::Point add(const geometry_msgs::Point& pt1,
                                    const geometry_msgs::Point& pt2);


/// @brief element-wise vector substraction: v = v1 - v2
/// @param vct1 the first vector
/// @param vct2 the second vector
/// @return v1 - v2
    static std::vector<double> sub(const std::vector<double>& vct1,
                                   const std::vector<double>& vct2);

/// @brief element-wise vector substraction: v = v1 - v2
/// @param vct1 the first vector
/// @param vct2 the second vector
/// @return v1 - v2
    static geometry_msgs::Vector3 sub(const geometry_msgs::Vector3& vct1,
                                      const geometry_msgs::Vector3& vct2);

/// @brief add each element of two geometry_msgs::Vector3
/// @param pt1 the first vector
/// @param pt2 the second vector
/// @return the sum of the two given vectors
    static geometry_msgs::Point sub(const geometry_msgs::Point& pt1,
                                    const geometry_msgs::Point& pt2);
/// @brief get cwise sum of all elements in the vector
/// @param vct the give  vector
/// @return the sum value
    static double sum(const std::vector<double>& vct);
////////////////////////////////////////////////////////////
///              VECTOR SCALING OPERATIONS               ///
////////////////////////////////////////////////////////////

/// @brief element-wise vector scaling
/// @param vct the imput vector
/// @param scaler the scaler multiplier
/// @return the scaled vector
    static std::vector<double> scale(const std::vector<double>& vct,
                                     const double& scaler);

/// @brief element-wise vector scaling
/// @param vct the imput vector
/// @param scaler the scaler multiplier
/// @return the scaled vector
    static geometry_msgs::Vector3 scale(const geometry_msgs::Vector3& vct,
                                        const double& scaler);

/// @brief element-wise vector devide
/// @param pt the imput vector
/// @param scaler the scaler multiplier
/// @return the scaled vector
    static geometry_msgs::Point scale(const geometry_msgs::Point& pt,
                                      const double& scaler);


////////////////////////////////////////////////////////////
///        DOT - PRODUCTION - OPERATIONS                 ///
////////////////////////////////////////////////////////////

/// @brief calculates dot product of two vectors
/// @param vct1 the first vector
/// @param vct2 the second vector
/// @return the dot product of the two given vectors
    static double dot(const std::vector<double>& vct1,
                      const std::vector<double>& vct2);

/// @brief calculates dot product of two geometry_msgs::Vector3
/// @param vct1 the first vector
/// @param vct2 the second vector
/// @param use_2D the marker to indicate using 2D distance
/// @return the dot product of the two given vectors
    static double dot(const geometry_msgs::Vector3& vct1,
                      const geometry_msgs::Vector3& vct2,
                      bool use_2D = false);

/// @brief calculates dot product of two geometry_msgs::Point
/// @param pt1 the first point
/// @param pt2 the second point
/// @param use_2D the marker to indicate using 2D distance
/// @return the dot product of the two given points
    static double dot(const geometry_msgs::Point& pt1,
                      const geometry_msgs::Point& pt2,
                      bool use_2D = false);

////////////////////////////////////////////////////////////
///                  NORM2 - OPERATIONS                  ///
////////////////////////////////////////////////////////////

/// @brief calculates norm2 of a C++ vector<double>
/// @param vct the input vector of uncentain length
/// @return the norm2 of the vector
    static double norm2(const std::vector<double>& vct);

/// @brief calculates norm2 of geometry_msgs::Vector3
/// @param vct3 the input Vector3
/// @param use_2D if true, use only x,y to calculate norm, defalut = false
/// @return the norm2 of the Vector3
    static double norm2(const geometry_msgs::Vector3& vct3, bool use_2D = false);

/// @brief calculates norm2 of geometry_msgs::Point
/// @param pt the input Point
/// @param use_2D if true, use only x,y to calculate norm, defalut = false
/// @return the norm2 of the Point
    static double norm2(const geometry_msgs::Point& pt, bool use_2D = false);

/// @brief calculates norm2 of geometry_msgs::Quaternion, which should always be 1.0
/// @param qt the quaternion to be checked
/// @return the norm2 of the Quaternion
    static double norm2(const geometry_msgs::Quaternion qt);

////////////////////////////////////////////////////////////
///              CROSS- PRODUCT - OPERATIONS             ///
////////////////////////////////////////////////////////////

/// @brief calculates cross product of two geometry_msgs::Vector3
/// @param vct1 the first vector
/// @param vct2 the second vector
/// @return the cross product (vector) of the two given vectors
    static geometry_msgs::Vector3 cross(const geometry_msgs::Vector3& vct1,
                                        const geometry_msgs::Vector3& vct2);

/// @brief calculates cross product of two geometry_msgs::Vector3, using 2D information
/// @param vct1 the first vector
/// @param vct2 the second vector
/// @param use_2D the marker for using only 2D information
/// @return the cross product (doube) of the two given vectors, using only x,y information
    static double cross(const geometry_msgs::Vector3& vct1,
                        const geometry_msgs::Vector3& vct2,
                        bool use_2D);

/// @brief calculates cross product of two geometry_msgs::Point
/// @param pt1 the first point
/// @param pt2 the second point
/// @return the cross product (vector) of the two given points
    static geometry_msgs::Point cross(const geometry_msgs::Point& pt1,
                                      const geometry_msgs::Point& pt2);

/// @brief calculates cross product of two geometry_msgs::Point, using 2D information
/// @param pt1 the first point
/// @param pt2 the second point
/// @param use_2D the marker for using only 2D information
/// @return the cross product (double) of the two given points, using only x,y information
    static double cross(const geometry_msgs::Point& pt1,
                        const geometry_msgs::Point& pt2,
                        bool use_2D);

////////////////////////////////////////////////////////////
///                   OTHER - OPERATIONS                 ///
////////////////////////////////////////////////////////////

/// @brief calculates the distance between two points
/// @param vct1 the first point
/// @param vct2 the second point
/// @param use_2D the marker to indicate using 2D distance
/// @return the distance between this two points
    static double distance(const geometry_msgs::Vector3& vct1,
                           const geometry_msgs::Vector3& vct2,
                           bool use_2D = false);

/// @brief calculates the distance between two points
/// @param pt1 the first point
/// @param pt2 the second point
/// @param use_2D the marker to indicate using 2D distance
/// @return the distance between this two points
    static double distance(const geometry_msgs::Point& pt1,
                           const geometry_msgs::Point& pt2,
                           bool use_2D = false);

/// @brief check if two vectors are the same
/// @param v1 the first vector
/// @param v2 the second vector
/// @return true if two vectors are identical
    static bool identical(const std::vector<double> & v1, const std::vector<double> & v2);

/// @brief check if two vectors are the same
/// @param pt1 the first vector
/// @param pt2 the second vector
/// @return true if two vectors are identical
    static bool identical(const geometry_msgs::Point& pt1,
                           const geometry_msgs::Point& pt2);

}; // class Vector

} // namespace common
} // namespace localization_module

#endif // FLBOT_LOCALIZATION_COMMON_VECTOR_H

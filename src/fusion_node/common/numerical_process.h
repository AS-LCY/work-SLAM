
#ifndef FLBOT_LOCALIZATION_COMMON_MATH_H
#define FLBOT_LOCALIZATION_COMMON_MATH_H

#include "math.h"

namespace localization_module {
namespace common {

/// @brief The class for some basic numerical processes
class NumericalProcess {
public:

/// @brief Unify the angle into - PI ~ PI
/// @param angle_in is the input angle
/// @return The unified value
    static double unify_angle(const double& angle_in, const bool& use_rad = true);

/// @brief sign function
/// @param value the input value
/// @return the output value
    static double sign(const double& value);

/// @brief dead zone function
/// @param input the input value
/// @param width the dead zone width
/// @return the output value
    static double dead_zone(const double& input, const double& width);

/// @brief range function
/// @param input the input value
/// @param lower the lower range
/// @param upper the upper range
/// @return the output value
    static double range(const double& input, const double& lower, const double& upper);
};

} // namespace common
} // namespace localization_module

#endif // FLBOT_LOCALIZATION_COMMON_MATH_H

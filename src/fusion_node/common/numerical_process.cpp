#include "numerical_process.h"

namespace localization_module {
namespace common {

double NumericalProcess::unify_angle(const double& angle_in, const bool& use_rad) {
	double angle_out = 0.0;
	if (use_rad) {
		angle_out = angle_in + M_PI;
		angle_out = fmod(angle_out, 2 * M_PI) - M_PI;

		while (angle_out < -M_PI) {
			angle_out += 2 * M_PI;
		}
		while (angle_out > M_PI) {
			angle_out -= 2 * M_PI;
		}
	} else {
		angle_out = angle_in + 180.0;
		angle_out = fmod(angle_out, 360.0) - 180.0;

		while (angle_out < -180.0) {
			angle_out += 360.0;
		}
		while (angle_out > 180.0) {
			angle_out -= 360.0;
		}
	}
	return angle_out;
}

double NumericalProcess::sign(const double& value) {
	if (value > 1e-6) {
		return 1.0;
	} else if (value < -1e-6) {
		return -1.0;
	} else {
		return 0.0;
	}
}

double NumericalProcess::dead_zone(const double& input, const double& width) {
	double zone_width = fabs(width);
	if (input > zone_width) {
		return input - zone_width;
	} else if (input < -zone_width) {
		return input + zone_width;
	} else {
		return 0.0;
	}
}

double NumericalProcess::range(const double& input, const double& lower, const double& upper) {
	if (upper < lower) {
		return input;
	}

	if (input > upper) {
		return upper;
	} else if (input < lower) {
		return lower;
	} else {
		return input;
	}
}

} // namespace common
} // namespace localization_module

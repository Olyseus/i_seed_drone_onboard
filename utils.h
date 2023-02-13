#ifndef UTILS_H_
#define UTILS_H_

#include <cmath>  // M_PI

static constexpr double pi_degree{180.0};
static constexpr double rad2deg{pi_degree / M_PI};
static constexpr double deg2rad{M_PI / pi_degree};

// return: yaw degree, pitch degree
std::pair<float, float> gimbal_rotation_params_with_heading_degree(double yaw_x_degree, double pitch_z_degree, double drone_heading_degree);

// return: yaw degree, pitch degree
std::pair<float, float> gimbal_rotation_params(double x_pixel, double y_pixel, double drone_heading_degree);

#endif // UTILS_H_

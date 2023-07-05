#ifndef UTILS_H_
#define UTILS_H_

#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Bbox_2.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Direction_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>

#include <cmath>  // M_PI

static const double pi_degree{180.0};
static const double rad2deg{pi_degree / M_PI};
static const double deg2rad{M_PI / pi_degree};

static const float pi_degree_f{180.0F};
static const float rad2deg_f{pi_degree_f / static_cast<float>(M_PI)};
static const float deg2rad_f{static_cast<float>(M_PI) / pi_degree_f};

static const std::size_t h20_img_width{5184};
static const std::size_t h20_img_height{3888};

// return: yaw degree, pitch degree
std::pair<float, float> gimbal_rotation_params_with_heading_degree(
    double yaw_x_degree, double pitch_z_degree, double drone_heading_degree);

// return: yaw degree, pitch degree
std::pair<float, float> gimbal_rotation_params(double x_pixel, double y_pixel,
                                               double drone_heading_degree);

namespace utils {

using kernel = CGAL::Cartesian<double>;
using kernel_ft = kernel::FT;

using point = kernel::Point_2;
using polygon = CGAL::Polygon_2<kernel>;
using polygon_with_holes = CGAL::Polygon_with_holes_2<kernel>;
using traits = CGAL::Arr_segment_traits_2<kernel>;
using segment = traits::Segment_2;
using arrangement = CGAL::Arrangement_2<traits>;
using direction = kernel::Direction_2;
using transformation = CGAL::Aff_transformation_2<kernel>;
using line = kernel::Line_2;
using vect = kernel::Vector_2;
using bbox = CGAL::Bbox_2;

enum class corner { left_down, left_up, right_down, right_up };

// https://github.com/Olyseus/i_seed_drone_onboard/issues/13#issuecomment-1253192301
// https://github.com/Olyseus/i_seed_drone_onboard/issues/13#issuecomment-1281932707
static const kernel_ft sensor_size_width_x1000_mm{7412};   // 7.412mm
static const kernel_ft sensor_size_height_x1000_mm{5559};  // 5.559mm
static const kernel_ft focal_length_x100_mm{1034};         // 10.34mm

static const kernel_ft sensor_size_width_mm{sensor_size_width_x1000_mm / 1000};
static const kernel_ft sensor_size_height_mm{sensor_size_height_x1000_mm /
                                             1000};
static const kernel_ft focal_length_mm{focal_length_x100_mm / 100};

static const kernel_ft sensor_size_width_m{sensor_size_width_mm / 1000};
static const kernel_ft sensor_size_height_m{sensor_size_height_mm / 1000};
static const kernel_ft focal_length_m{focal_length_mm / 1000};
static const kernel_ft camera_ground_distance_m{10};

static const kernel_ft camera_dist_scale{camera_ground_distance_m /
                                         focal_length_m};
static const kernel_ft camera_footprint_width_m{sensor_size_width_m *
                                                camera_dist_scale};
static const kernel_ft camera_footprint_height_m{sensor_size_height_m *
                                                 camera_dist_scale};

// Fix approximation error
void cleanup_collinear(polygon* poly);

}  // namespace utils

#endif  // UTILS_H_

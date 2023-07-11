#include "utils.h"

#include "olyseus_verify.h"  // OLYSEUS_VERIFY

// return: yaw degree, pitch degree
auto gimbal_rotation_params_with_heading_degree(double yaw_x_degree,
                                                double pitch_z_degree,
                                                double drone_heading_degree)
    -> std::pair<float, float> {
  constexpr double right_angle{90.0};
  constexpr double straight_angle{180.0};
  constexpr double full_angle{360.0};

  // standard quadrants orientation to rotation relative to North
  double yaw_result{right_angle - yaw_x_degree};

  // gimbal rotation is relative to North, not drone heading
  yaw_result += drone_heading_degree;

  if (yaw_result > straight_angle) {
    yaw_result -= full_angle;
  }
  if (yaw_result < -straight_angle) {
    yaw_result += full_angle;
  }
  OLYSEUS_VERIFY(yaw_result >= -straight_angle);
  OLYSEUS_VERIFY(yaw_result <= straight_angle);

  OLYSEUS_VERIFY(pitch_z_degree >= 0.0);
  OLYSEUS_VERIFY(pitch_z_degree < right_angle);

  return {yaw_result, pitch_z_degree - right_angle};
}

// return: yaw degree, pitch degree
auto gimbal_rotation_params(double x_pixel, double y_pixel,
                            double drone_heading_degree)
    -> std::pair<float, float> {
  const double sensor_size_width_mm{utils::sensor_size_width_x1000_mm / 1000};
  const double sensor_size_height_mm{utils::sensor_size_height_x1000_mm / 1000};
  const double focal_length_mm{utils::focal_length_x100_mm / 100};

  const double sensor_size_width_m{sensor_size_width_mm / 1000};
  const double sensor_size_height_m{sensor_size_height_mm / 1000};
  const double focal_length_m{focal_length_mm / 1000};

  OLYSEUS_VERIFY(x_pixel >= 0.0);
  OLYSEUS_VERIFY(y_pixel >= 0.0);
  OLYSEUS_VERIFY(x_pixel <= h20_img_width);
  OLYSEUS_VERIFY(y_pixel <= h20_img_height);

  static constexpr std::size_t width_half{h20_img_width / 2};
  static constexpr std::size_t height_half{h20_img_height / 2};

  const double x_c{x_pixel - width_half};
  const double y_c{height_half - y_pixel};

  constexpr double threshold{1e-1};
  if (x_c * x_c + y_c * y_c < threshold) {
    // 90 is north (in standard quadrants orientation)
    constexpr double north{90.0};
    return gimbal_rotation_params_with_heading_degree(north, 0.0,
                                                      drone_heading_degree);
  }

  const double yaw_x_rad{std::atan2(y_c, x_c)};

  const double pixel_to_m{sensor_size_width_m / h20_img_width};
  constexpr double eps{1e-8};
  OLYSEUS_VERIFY(std::abs(pixel_to_m - sensor_size_height_m / h20_img_height) <
                 eps);

  const double x_m{x_c * pixel_to_m};
  const double y_m{y_c * pixel_to_m};

  const double pitch_z_rad{
      std::atan(std::sqrt(x_m * x_m + y_m * y_m) / focal_length_m)};

  return gimbal_rotation_params_with_heading_degree(
      yaw_x_rad * rad2deg, pitch_z_rad * rad2deg, drone_heading_degree);
}

namespace utils {

// Fix approximation error
void cleanup_collinear(polygon* poly) {
  OLYSEUS_VERIFY(poly != nullptr);

  OLYSEUS_VERIFY(poly->is_simple());
  OLYSEUS_VERIFY(poly->is_counterclockwise_oriented());

  const double original_area{std::abs(CGAL::to_double(poly->area()))};

  std::vector<std::size_t> to_remove;

  for (std::size_t i{0}; i < poly->size(); ++i) {
    const std::size_t last{poly->size() - 1};
    const std::size_t prev_index{(i > 0) ? (i - 1) : last};
    const std::size_t next_index{(i == last) ? 0 : (i + 1)};
    const point prev{poly->vertex(prev_index)};
    const point curr{poly->vertex(i)};
    const point next{poly->vertex(next_index)};

    const kernel::Vector_3 v1{curr.x() - prev.x(), curr.y() - prev.y(), 0.0};
    const kernel::Vector_3 v2{next.x() - curr.x(), next.y() - curr.y(), 0.0};

    const double angle{CGAL::approximate_angle(v1, v2)};
    OLYSEUS_VERIFY(angle >= 0.0);
    constexpr double angle_eps{1e-3};
    if (angle < angle_eps) {
      to_remove.push_back(i);
    }
  }

  if (to_remove.empty()) {
    OLYSEUS_VERIFY(poly->is_convex());
    return;
  }

  for (auto it{to_remove.rbegin()}; it != to_remove.rend(); ++it) {
    // NOLINTNEXTLINE(*-narrowing-conversions)
    poly->erase(poly->vertices_begin() + *it);
  }

  if (poly->size() < 3) {
    poly->clear();
    OLYSEUS_VERIFY(poly->size() == 0);
    return;
  }

  OLYSEUS_VERIFY(poly->is_simple());
  OLYSEUS_VERIFY(poly->is_convex());
  OLYSEUS_VERIFY(poly->is_counterclockwise_oriented());

  constexpr double area_eps{1e-2};
  const double new_area{std::abs(CGAL::to_double(poly->area()))};
  OLYSEUS_VERIFY(std::abs(original_area - new_area) < area_eps);
}

}  // namespace utils

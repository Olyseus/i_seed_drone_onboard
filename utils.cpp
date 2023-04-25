#include "utils.h"

#include "inference.h"
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
  // https://github.com/Olyseus/i_seed_drone_onboard/issues/13#issuecomment-1253192301
  // https://github.com/Olyseus/i_seed_drone_onboard/issues/13#issuecomment-1281932707
  // https://sdk-forum.dji.net/hc/en-us/articles/11606411050265
  // https://github.com/Olyseus/i_seed_drone_onboard/issues/13#issuecomment-1325870404
  constexpr double sensor_size_width_x1000_mm{7412};   // 7.412mm
  constexpr double sensor_size_height_x1000_mm{5559};  // 5.559mm
  constexpr double focal_length_x100_mm{683};          // 6.83mm

  constexpr double sensor_size_width_mm{sensor_size_width_x1000_mm / 1000};
  constexpr double sensor_size_height_mm{sensor_size_height_x1000_mm / 1000};
  constexpr double focal_length_mm{focal_length_x100_mm / 100};

  constexpr double sensor_size_width_m{sensor_size_width_mm / 1000};
  constexpr double sensor_size_height_m{sensor_size_height_mm / 1000};
  constexpr double focal_length_m{focal_length_mm / 1000};

  constexpr std::size_t h20_img_width{inference::h20_img_width};
  constexpr std::size_t h20_img_height{inference::h20_img_height};

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

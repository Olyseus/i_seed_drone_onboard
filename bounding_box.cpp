#include "bounding_box.h"

#include <spdlog/spdlog.h>

#include <algorithm>  // std::max

#include "olyseus_verify.h"  // OLYSEUS_VERIFY

bounding_box::bounding_box(const float* p, std::size_t x_shift,
                           std::size_t y_shift)
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    : confidence_(p[4]) {
  // https://github.com/ultralytics/yolov5/issues/1277#issuecomment-1081657025
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  const float x{p[0] + static_cast<float>(x_shift)};
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  const float y{p[1] + static_cast<float>(y_shift)};

  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  const float w{p[2]};
  OLYSEUS_VERIFY(w >= 0.0);

  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  const float h{p[3]};
  OLYSEUS_VERIFY(h >= 0.0);

  constexpr float half{1.0F / 2.0F};

  xmax_ = x + w * half;
  xmin_ = x - w * half;

  ymax_ = y + h * half;
  ymin_ = y - h * half;

  OLYSEUS_VERIFY(confidence_ >= 0.0);
  OLYSEUS_VERIFY(confidence_ <= 1.0);

  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  const float iseed_blue{p[5]};
  OLYSEUS_VERIFY(iseed_blue >= 0.0);
  OLYSEUS_VERIFY(iseed_blue <= 1.0);

  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  const float iseed_original_color{p[6]};
  OLYSEUS_VERIFY(iseed_original_color >= 0.0);
  OLYSEUS_VERIFY(iseed_original_color <= 1.0);

  if (iseed_blue > iseed_original_color) {
    class_id_ = 0;
    class_name_ = "blue";
    confidence_ *= iseed_blue;
  } else {
    class_id_ = 1;
    class_name_ = "original_color";
    confidence_ *= iseed_original_color;
  }
}

auto bounding_box::intersect(const bounding_box& other) const -> bool {
  if (other.xmin_ >= xmin_) {
    if (other.xmin_ > xmax_) {
      return false;
    }
    // X intersection: true
  } else {
    if (other.xmax_ < xmin_) {
      return false;
    }
    // X intersection: true
  }

  if (other.ymin_ >= ymin_) {
    if (other.ymin_ > ymax_) {
      return false;
    }
    // Y intersection: true
  } else {
    if (other.ymax_ < ymin_) {
      return false;
    }
    // Y intersection: true
  }

  return true;
}

void bounding_box::print() const {
  spdlog::info(
      "xmin: {:.6f}, ymin: {:.6f}, xmax: {:.6f}, ymax: {:.6f}, confidence: "
      "{:.6f}, class: {}, {}",
      xmin_, ymin_, xmax_, ymax_, confidence_, class_id_, class_name_);
}

auto bounding_box::pmin() const -> cv::Point {
  return {static_cast<int>(xmin_), static_cast<int>(ymin_)};
}

auto bounding_box::pmax() const -> cv::Point {
  return {static_cast<int>(xmax_), static_cast<int>(ymax_)};
}

auto bounding_box::class_color() const -> cv::Scalar {
  // Return format is BGR
  switch (class_id_) {
    case 0:
      // blue
      return {255, 0, 0};  // NOLINT(*-magic-numbers)
    case 1:
      // original color, green
      return {0, 255, 0};  // NOLINT(*-magic-numbers)
    default:
      OLYSEUS_UNREACHABLE;
      return {0, 0, 0};
  }
}

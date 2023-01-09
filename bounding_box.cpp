#include "bounding_box.h"

#include <spdlog/spdlog.h>

#include <algorithm>         // std::max
#include <boost/assert.hpp>  // BOOST_VERIFY

bounding_box::bounding_box(float* p, std::size_t x_shift, std::size_t y_shift) {
  // https://github.com/ultralytics/yolov5/issues/1277#issuecomment-1081657025
  const float x{p[0] + x_shift};
  const float y{p[1] + y_shift};

  const float w{p[2]};
  BOOST_VERIFY(w >= 0.0);

  const float h{p[3]};
  BOOST_VERIFY(h >= 0.0);

  xmax_ = x + w / 2.0F;
  xmin_ = x - w / 2.0F;

  ymax_ = y + h / 2.0F;
  ymin_ = y - h / 2.0F;

  confidence_ = p[4];
  BOOST_VERIFY(confidence_ >= 0.0);
  BOOST_VERIFY(confidence_ <= 1.0);

  const float iseed_blue{p[5]};
  BOOST_VERIFY(iseed_blue >= 0.0);
  BOOST_VERIFY(iseed_blue <= 1.0);

  const float iseed_brown{p[6]};
  BOOST_VERIFY(iseed_brown >= 0.0);
  BOOST_VERIFY(iseed_brown <= 1.0);

  const float iseed_green{p[7]};
  BOOST_VERIFY(iseed_green >= 0.0);
  BOOST_VERIFY(iseed_green <= 1.0);

  if (iseed_blue > std::max(iseed_brown, iseed_green)) {
    class_id_ = 0;
    class_name_ = "blue";
    confidence_ *= iseed_blue;
  } else if (iseed_brown > std::max(iseed_blue, iseed_green)) {
    class_id_ = 1;
    class_name_ = "brown";
    confidence_ *= iseed_brown;
  } else {
    class_id_ = 2;
    class_name_ = "green";
    confidence_ *= iseed_green;
  }
}

bounding_box::~bounding_box() = default;

bool bounding_box::intersect(const bounding_box& other) const {
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

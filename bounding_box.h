#ifndef BOUNDING_BOX_H_
#define BOUNDING_BOX_H_

#include <cstddef>             // std::size_t
#include <opencv2/opencv.hpp>  // cv::Point

class bounding_box {
 public:
  bounding_box(const float*, std::size_t x_shift, std::size_t y_shift);
  ~bounding_box() = default;

  bool intersect(const bounding_box&) const;
  float confidence() const { return confidence_; }
  void print() const;

  float mid_x() const { return (xmin_ + xmax_) / 2.0F; }
  float mid_y() const { return (ymin_ + ymax_) / 2.0F; }

  cv::Point pmin() const;
  cv::Point pmax() const;
  cv::Scalar class_color() const;

 private:
  float xmin_{0.0};
  float ymin_{0.0};
  float xmax_{0.0};
  float ymax_{0.0};
  float confidence_{0.0};
  int class_id_{0};
  const char* class_name_{""};
};

#endif  // BOUNDING_BOX_H_

#ifndef BOUNDING_BOX_H_
#define BOUNDING_BOX_H_

#include <cstddef>             // std::size_t
#include <opencv2/opencv.hpp>  // cv::Point

/// \brief Reading bounding boxes objects from inference output
class bounding_box {
 public:
  /// \brief Read bounding box from memory
  /// \param[in] p Pointer to the memory
  /// \param[in] x_shift Tile shift by X-axis
  /// \param[in] y_shift Tile shift by Y-axis
  bounding_box(const float* p, std::size_t x_shift, std::size_t y_shift);

  ~bounding_box() = default;

  /// \brief Two boxes intersection
  /// \return \b true Boxes intersect each other
  bool intersect(const bounding_box&) const;

  /// \brief Model confidence about detected object
  /// \return 1.0 The maximum confidence about detected object class
  /// \return 0.0 The minimum confidence about detected object class
  float confidence() const { return confidence_; }

  /// \brief Print to console bounding box information
  void print() const;

  /// \name (x,y) coordinates of middle of bounding box
  /// @{
  float mid_x() const { return (xmin_ + xmax_) / 2.0F; }
  float mid_y() const { return (ymin_ + ymax_) / 2.0F; }
  /// @}

  /// \name min/max of the bounding box
  /// @{
  cv::Point pmin() const;
  cv::Point pmax() const;
  /// @}

  /// \brief Assigned color of a class of a detected object
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

#ifndef INFERENCE_H_
#define INFERENCE_H_

#include <NvInferRuntime.h>  // nvinfer1::ILogger

#include <boost/version.hpp>  // BOOST_VERSION
#include <memory>             // std::unique_ptr
#include <string>
#include <vector>

#if BOOST_VERSION >= 107400  // 1.74.0
#include <boost/gil/extension/io/jpeg/old.hpp>
#elif BOOST_VERSION >= 107200  // 1.72.0
#include <boost/gil/extension/io/jpeg.hpp>
#else
#include <boost/gil/extension/io/jpeg_io.hpp>
#endif

#include "utils.h"  // h20_img_width

class bounding_box;

/// \brief Run inference for images
class inference {
 public:
  /// \brief Load model from file
  inference(const std::string& model_file);
  ~inference();

  /// \cond private
  inference(const inference&) = delete;
  inference(inference&&) = delete;

  inference& operator=(const inference&) = delete;
  inference& operator=(inference&&) = delete;
  /// \endcond

  /// \brief Start inference on image
  /// \return Vector of \ref bounding_box objects
  /// \note \ref thread_inference "Thread: inference"
  std::vector<bounding_box> run(const std::string& image);

  static constexpr std::size_t inference_img_width{768};
  static constexpr std::size_t inference_img_height{768};

  static_assert(h20_img_width >= inference_img_width);
  static_assert(h20_img_height >= inference_img_height);

  static constexpr std::size_t n_batch_width{h20_img_width /
                                             inference_img_width};
  static constexpr std::size_t n_batch_height{h20_img_height /
                                              inference_img_height};

  static_assert(n_batch_width > 0);
  static_assert(n_batch_height > 0);

  static constexpr std::size_t n_batch{n_batch_width * n_batch_height};

 private:
  void init_input_data(
      const boost::gil::rgb8_image_t::const_view_t& image_view);
  std::vector<bounding_box> analyze_bboxes();
  static void analyze_entries(float* data, std::size_t x_shift,
                              std::size_t y_shift,
                              std::vector<bounding_box>& bboxes);

  static constexpr std::size_t height_crop_size{
      (h20_img_height - n_batch_height * inference_img_height) / 2};
  static_assert(h20_img_height - 2 * height_crop_size ==
                n_batch_height * inference_img_height);

  static constexpr std::size_t width_crop_size{
      (h20_img_width - n_batch_width * inference_img_width) / 2};
  static_assert(h20_img_width - 2 * width_crop_size ==
                n_batch_width * inference_img_width);

  static constexpr std::size_t rgb_size{3};

  // https://github.com/ultralytics/yolov5/issues/1277#issuecomment-1081657025
  static constexpr std::size_t output_params{5};  // x y w h objectness
  static constexpr std::size_t n_classes{
      3};  // i-seed blue, i-seed brown, i-seed green
  static constexpr std::size_t output_entry_len{output_params + n_classes};

  // https://github.com/ultralytics/yolov5/issues/1277#issuecomment-810458576
  static constexpr std::size_t p3_stride{8};
  static constexpr std::size_t p4_stride{16};
  static constexpr std::size_t p5_stride{32};
  static constexpr std::size_t n_anchors{3};

  static_assert((inference_img_height % p5_stride) == 0);
  static_assert((inference_img_width % p5_stride) == 0);

  static constexpr std::size_t p3_grid_size{inference_img_height / p3_stride *
                                            inference_img_width / p3_stride};
  static constexpr std::size_t p4_grid_size{inference_img_height / p4_stride *
                                            inference_img_width / p4_stride};
  static constexpr std::size_t p5_grid_size{inference_img_height / p5_stride *
                                            inference_img_width / p5_stride};

  static constexpr std::size_t n_output_entries{
      (p3_grid_size + p4_grid_size + p5_grid_size) * n_anchors};

  class logger : public nvinfer1::ILogger {
   public:
    using severity = nvinfer1::ILogger::Severity;
    using asciichar = nvinfer1::AsciiChar;

    void log(severity severity, asciichar const* msg) noexcept override;
  };

  logger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> exe_context_;

  std::vector<float> input_data_;
  std::vector<float> output_data_;

  void* dev_input_ptr_{nullptr};
  void* dev_output_ptr_{nullptr};
};

#endif  // INFERENCE_H_

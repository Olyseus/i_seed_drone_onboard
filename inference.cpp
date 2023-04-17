#include "inference.h"

#include <cuda_runtime_api.h>  // cudaError_t
#include <spdlog/spdlog.h>

#include <boost/assert.hpp>  // BOOST_VERIFY_MSG
#include <fstream>           // std::ifstream

#include "bounding_box.h"
#include "timer.h"

void inference::logger::log(severity severity, asciichar const* msg) noexcept {
  switch (severity) {
    case severity::kERROR:
      spdlog::error(msg);
      break;
    case severity::kWARNING:
      spdlog::warn(msg);
      break;
    case severity::kINFO:
      spdlog::info(msg);
      break;
    case severity::kVERBOSE:
      spdlog::debug(msg);
      break;
    default:
      BOOST_VERIFY_MSG(false, "unreachable");
      break;
  }
}

inference::inference(const std::string& model_file) {
  spdlog::info("Loading model from file: {}", model_file);

  spdlog::info("Expected model size: batch({}) w({}) h({})", n_batch,
               inference_img_width, inference_img_height);
  spdlog::info("batch_w({}) batch_h({})", n_batch_width, n_batch_height);
  spdlog::info("crop_w ({}) crop_h ({})", width_crop_size, height_crop_size);

  timer t;

  t.start();

  std::ifstream engine_file(model_file, std::ios::binary);
  BOOST_VERIFY(engine_file.good());
  engine_file.seekg(0, std::ifstream::end);
  const int64_t fsize{engine_file.tellg()};
  engine_file.seekg(0, std::ifstream::beg);

  std::vector<char> engine_data(fsize);
  engine_file.read(engine_data.data(), fsize);
  BOOST_VERIFY(engine_file.good());

  spdlog::info("File read in {}ms", t.elapsed_ms());

  t.start();

  runtime_.reset(nvinfer1::createInferRuntime(logger_));
  // runtime->setErrorRecorder(&gRecorder); // FIXME (implement error
  // recorder)
  engine_.reset(
      runtime_->deserializeCudaEngine(engine_data.data(), fsize, nullptr));
  BOOST_VERIFY(engine_);

  spdlog::info("Engine loaded in {}ms", t.elapsed_ms());

  exe_context_.reset(engine_->createExecutionContext());
  BOOST_VERIFY(exe_context_);

  BOOST_VERIFY(engine_->getNbOptimizationProfiles() == 1);
  BOOST_VERIFY(engine_->getNbBindings() == 2);  // input + output

  const int32_t bind_input{0};
  const int32_t bind_output{1};

  BOOST_VERIFY(engine_->bindingIsInput(bind_input));
  BOOST_VERIFY(!engine_->bindingIsInput(bind_output));

  // 32 bit floating points
  static_assert(sizeof(float) == 4);
  BOOST_VERIFY(engine_->getBindingDataType(bind_input) ==
               nvinfer1::DataType::kFLOAT);
  BOOST_VERIFY(engine_->getBindingDataType(bind_output) ==
               nvinfer1::DataType::kFLOAT);

  const nvinfer1::Dims input_dims{
      exe_context_->getBindingDimensions(bind_input)};
  BOOST_VERIFY(input_dims.nbDims == 4);
  BOOST_VERIFY(input_dims.d[0] == n_batch);
  BOOST_VERIFY(input_dims.d[1] == rgb_size);
  BOOST_VERIFY(input_dims.d[2] == inference_img_height);
  BOOST_VERIFY(input_dims.d[3] == inference_img_width);

  const nvinfer1::Dims output_dims{
      exe_context_->getBindingDimensions(bind_output)};
  BOOST_VERIFY(output_dims.nbDims == 3);
  BOOST_VERIFY(output_dims.d[0] == n_batch);
  BOOST_VERIFY(output_dims.d[1] == n_output_entries);
  BOOST_VERIFY(output_dims.d[2] == output_entry_len);

  const nvinfer1::Dims strides{exe_context_->getStrides(bind_input)};
  BOOST_VERIFY(strides.nbDims == 4);  // same as binding dimensions
  BOOST_VERIFY(strides.d[0] == inference_img_height * inference_img_width *
                                   rgb_size);  // stride between images
  BOOST_VERIFY(strides.d[1] ==
               inference_img_height *
                   inference_img_width);  // stride between RGB color components
  BOOST_VERIFY(strides.d[2] ==
               inference_img_width);  // stride between image rows
  BOOST_VERIFY(strides.d[3] == 1);    // color components have no padding

  const nvinfer1::Dims output_strides{exe_context_->getStrides(bind_output)};
  BOOST_VERIFY(output_strides.nbDims == 3);  // same as binding dimensions
  BOOST_VERIFY(output_strides.d[0] == n_output_entries * output_entry_len);
  BOOST_VERIFY(output_strides.d[1] == output_entry_len);
  BOOST_VERIFY(output_strides.d[2] == 1);  // no padding

  BOOST_VERIFY(engine_->getBindingVectorizedDim(bind_input) ==
               -1);  // Not vectorized
  BOOST_VERIFY(engine_->getBindingComponentsPerElement(bind_input) ==
               -1);  // Since not vectorized
  BOOST_VERIFY(engine_->getBindingVectorizedDim(bind_output) ==
               -1);  // Not vectorized
  BOOST_VERIFY(engine_->getBindingComponentsPerElement(bind_output) ==
               -1);  // Since not vectorized

  t.start();

  cudaError_t err{cudaMalloc(&dev_input_ptr_, n_batch * inference_img_height *
                                                  inference_img_width *
                                                  rgb_size * sizeof(float))};
  BOOST_VERIFY(dev_input_ptr_);
  BOOST_VERIFY(err == cudaSuccess);

  err = cudaMalloc(&dev_output_ptr_, n_batch * n_output_entries *
                                         output_entry_len * sizeof(float));
  BOOST_VERIFY(dev_output_ptr_);
  BOOST_VERIFY(err == cudaSuccess);

  output_data_.reserve(n_batch * n_output_entries * output_entry_len);
  input_data_.reserve(n_batch * inference_img_height * inference_img_width *
                      rgb_size);

  spdlog::info("Memory reserved in {}ms", t.elapsed_ms());
}

inference::~inference() {
  cudaError_t err = cudaFree(dev_input_ptr_);
  BOOST_VERIFY(err == cudaSuccess);

  err = cudaFree(dev_output_ptr_);
  BOOST_VERIFY(err == cudaSuccess);
}

auto inference::run(const std::string& image) -> std::vector<bounding_box> {
  spdlog::info("Inference for image {}", image);

  timer t;

  t.start();

  boost::gil::rgb8_image_t input_image;
  boost::gil::jpeg_read_image(image, input_image);
  const auto image_view{boost::gil::const_view(input_image)};
  BOOST_VERIFY(input_image.height() == h20_img_height);
  BOOST_VERIFY(input_image.width() == h20_img_width);

  spdlog::info("Image uint8 read from disk in {}ms", t.elapsed_ms());

  t.start();
  init_input_data(image_view);
  spdlog::info("Image float conversion and memory layout change in {}ms",
               t.elapsed_ms());

  t.start();

  cudaError_t err =
      cudaMemcpy(dev_input_ptr_, input_data_.data(),
                 input_data_.size() * sizeof(float), cudaMemcpyHostToDevice);
  BOOST_VERIFY(err == cudaSuccess);

  spdlog::info("Input data pushed to GPU in {}ms", t.elapsed_ms());

  t.start();

  std::array<void*, 2> exe_ptr{dev_input_ptr_, dev_output_ptr_};
  const bool ok{exe_context_->execute(n_batch, exe_ptr.data())};
  BOOST_VERIFY(ok);

  spdlog::info("Inference done in {}ms", t.elapsed_ms());

  t.start();

  output_data_.resize(n_batch * n_output_entries * output_entry_len);
  err = cudaMemcpy(output_data_.data(), dev_output_ptr_,
                   output_data_.size() * sizeof(float), cudaMemcpyDeviceToHost);
  BOOST_VERIFY(err == cudaSuccess);

  spdlog::info("Output data fetched from GPU in {}ms", t.elapsed_ms());

  return analyze_bboxes();
}

void inference::init_input_data(
    const boost::gil::rgb8_image_t::const_view_t& image_view) {
  input_data_.resize(n_batch * inference_img_height * inference_img_width *
                     rgb_size);

  constexpr std::size_t stride{inference_img_width * inference_img_height *
                               rgb_size};
  for (std::size_t batch_height_index{0}; batch_height_index < n_batch_height;
       ++batch_height_index) {
    for (std::size_t batch_width_index{0}; batch_width_index < n_batch_width;
         ++batch_width_index) {
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
      float* input{input_data_.data() +
                   (batch_height_index * n_batch_width + batch_width_index) *
                       stride};
      float* input_r{input};
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
      float* input_g{input_r + inference_img_width * inference_img_height};
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
      float* input_b{input_g + inference_img_width * inference_img_height};
      const std::size_t x_shift{batch_width_index * inference_img_width +
                                width_crop_size};
      const std::size_t y_shift{batch_height_index * inference_img_height +
                                height_crop_size};
      for (std::size_t y{0}; y < inference_img_height; ++y) {
        // NOLINTNEXTLINE(*-narrowing-conversions)
        const auto* y_it{image_view.row_begin(y + y_shift)};
        for (std::size_t x{0}; x < inference_img_width; ++x) {
          // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
          const boost::gil::rgb8c_pixel_t& p{y_it[x + x_shift]};
          const uint8_t r_int{boost::gil::get_color(p, boost::gil::red_t())};
          const uint8_t g_int{boost::gil::get_color(p, boost::gil::green_t())};
          const uint8_t b_int{boost::gil::get_color(p, boost::gil::blue_t())};

          constexpr float rgb_max{255.0F};
          *input_r = static_cast<float>(r_int) / rgb_max;
          *input_g = static_cast<float>(g_int) / rgb_max;
          *input_b = static_cast<float>(b_int) / rgb_max;

          // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
          ++input_r;
          // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
          ++input_g;
          // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
          ++input_b;
        }
      }
    }
  }
}

auto inference::analyze_bboxes() -> std::vector<bounding_box> {
  timer t;
  t.start();

  std::vector<bounding_box> bboxes;

  for (std::size_t batch_index{0}; batch_index < n_batch; ++batch_index) {
    const std::size_t batch_height_index{batch_index / n_batch_width};
    const std::size_t batch_width_index{batch_index -
                                        batch_height_index * n_batch_width};
    const std::size_t x_shift{batch_width_index * inference_img_width +
                              width_crop_size};
    const std::size_t y_shift{batch_height_index * inference_img_height +
                              height_crop_size};
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    float* data{output_data_.data() +
                batch_index * n_output_entries * output_entry_len};
    analyze_entries(data, x_shift, y_shift, bboxes);
  }

  spdlog::info("Bounding boxes analyzed in {}ms", t.elapsed_ms());
  return bboxes;
}

void inference::analyze_entries(float* data, std::size_t x_shift,
                                std::size_t y_shift,
                                std::vector<bounding_box>& bboxes) {
  for (std::size_t i{0}; i < n_output_entries; ++i) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    float* p{data + output_entry_len * i};
    const bounding_box box{p, x_shift, y_shift};
    constexpr double confidence_threshold{0.25};
    if (box.confidence() <= confidence_threshold) {
      continue;
    }

    bool keep{true};

    for (const bounding_box& x : bboxes) {
      if (!x.intersect(box)) {
        continue;
      }
      if (x.confidence() > box.confidence()) {
        keep = false;
        break;
      }
    }

    if (!keep) {
      continue;
    }

    // NOLINTNEXTLINE(altera-id-dependent-backward-branch)
    for (auto it{bboxes.begin()}; it != bboxes.end();) {
      if (!it->intersect(box)) {
        ++it;
        continue;
      }
      BOOST_VERIFY(box.confidence() >= it->confidence());
      it = bboxes.erase(it);
    }

    bboxes.push_back(box);
  }
}

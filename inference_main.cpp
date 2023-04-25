#include <spdlog/sinks/rotating_file_sink.h>  // spdlog::sinks::rotating_file_sink_mt
#include <spdlog/sinks/stdout_sinks.h>        // spdlog::sinks::stdout_sink_mt
#include <spdlog/spdlog.h>

#include <CLI/App.hpp>
#include <CLI/Config.hpp>        // CLI::App (link)
#include <CLI/Formatter.hpp>     // CLI::App (link)
#include <boost/filesystem.hpp>  // boost::filesystem::path
#include <iostream>              // std::cerr
#include <opencv2/opencv.hpp>    // cv::Mat

#include "bounding_box.h"
#include "inference.h"
#include "olyseus_verify.h"  // OLYSEUS_VERIFY

void setup_logging() {
  auto console_sink = std::make_shared<spdlog::sinks::stdout_sink_mt>();
  console_sink->set_level(spdlog::level::info);

  namespace fs = boost::filesystem;

  const fs::path log_path{fs::absolute("inference.log")};
  fs::remove(log_path);

  constexpr std::size_t max_file_size{10 * 1024 * 1024};
  constexpr std::size_t max_file_num{3};
  constexpr bool rotate_on_open{true};
  auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
      log_path.string(), max_file_size, max_file_num, rotate_on_open);
  file_sink->set_level(spdlog::level::info);

  auto logger = std::make_shared<spdlog::logger>(
      "", spdlog::sinks_init_list({console_sink, file_sink}));

  spdlog::set_default_logger(logger);
  spdlog::set_level(spdlog::level::trace);

  spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] (t:%t) [%l] %v");
  spdlog::flush_on(spdlog::level::info);

  spdlog::info("Logging to file: {}", log_path.string());
}

auto run_main(int argc, char** argv) -> int {
  CLI::App app{"inference"};

  std::string model;
  app.add_option("--model", model, "File with model weights")
      ->required()
      ->check(CLI::ExistingFile);

  std::string image;
  app.add_option("--image", image, "Image file")
      ->required()
      ->check(CLI::ExistingFile);

  std::string bbimage;
  app.add_option("--bbimage", bbimage, "Output image with bounding boxes");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  setup_logging();

  try {
    inference inf{model};
    const std::vector<bounding_box> bboxes{inf.run(image)};

    if (bboxes.empty()) {
      spdlog::info("No objects detected");
      return EXIT_SUCCESS;
    }

    for (const bounding_box& bb : bboxes) {
      bb.print();
    }

    if (!bbimage.empty()) {
      spdlog::info("Save bounding boxes to image: {}", bbimage);
      cv::Mat cv_image{cv::imread(image.c_str())};
      OLYSEUS_VERIFY(cv_image.data != nullptr);

      for (const bounding_box& bb : bboxes) {
        constexpr int thickness{3};
        cv::rectangle(cv_image, bb.pmin(), bb.pmax(), bb.class_color(),
                      thickness);

        cv::Point p_text{bb.pmin()};
        p_text.y -= 12;

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2) << bb.confidence() * 100.0
           << '%';

        cv::putText(cv_image, ss.str(), p_text, cv::FONT_HERSHEY_SIMPLEX, 2.0,
                    bb.class_color(), thickness);
      }

      const bool ok{cv::imwrite(bbimage, cv_image)};
      OLYSEUS_VERIFY(ok);
    }

    return EXIT_SUCCESS;
  } catch (const std::system_error& exc) {
    spdlog::critical("System error: {} {} ({})", exc.code().category().name(),
                     exc.code().value(), exc.what());
  } catch (const std::exception& exc) {
    spdlog::critical("Exception: {}", exc.what());
  } catch (...) {
    spdlog::critical("Unknown exception caught");
  }
  return EXIT_FAILURE;
}

auto main(int argc, char** argv) -> int {
  try {
    return run_main(argc, argv);
  } catch (const std::system_error& exc) {
    std::cerr << "System error " << exc.code() << " (" << exc.what() << ")\n";
  } catch (const std::exception& exc) {
    std::cerr << "Exception: " << exc.what() << '\n';
  } catch (...) {
    std::cerr << "Unknown exception caught\n";
  }
  return EXIT_FAILURE;
}

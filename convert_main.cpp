#include <NvInfer.h>                          // nvinfer1::createInferBuilder
#include <NvInferRuntime.h>                   // ICudaEngine
#include <NvOnnxParser.h>                     // nvonnxparser::createParser
#include <spdlog/sinks/rotating_file_sink.h>  // spdlog::sinks::rotating_file_sink_mt
#include <spdlog/sinks/stdout_sinks.h>        // spdlog::sinks::stdout_sink_mt
#include <spdlog/spdlog.h>

#include <CLI/App.hpp>
#include <CLI/Config.hpp>        // CLI::App (link)
#include <CLI/Formatter.hpp>     // CLI::App (link)
#include <boost/filesystem.hpp>  // boost::filesystem::path
#include <iostream>              // std::cerr

#include "inference.h"

class logger : public nvinfer1::ILogger {
 public:
  using severity = nvinfer1::ILogger::Severity;
  using asciichar = nvinfer1::AsciiChar;

  void log(severity severity, asciichar const* msg) noexcept override {
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
};

void setup_logging() {
  auto console_sink = std::make_shared<spdlog::sinks::stdout_sink_mt>();
  console_sink->set_level(spdlog::level::info);

  const boost::filesystem::path log_path{"convert.log"};
  boost::filesystem::remove(log_path);

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
  CLI::App app{"convert"};

  std::string onnx;
  app.add_option("--onnx", onnx, "File with ONNX model")
      ->required()
      ->check(CLI::ExistingFile);

  std::string engine;
  app.add_option("--engine", engine, "Output TensorRT engine weights")
      ->required();

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  setup_logging();

  try {
    logger infer_logger;

    std::unique_ptr<nvinfer1::IBuilder> builder{
        nvinfer1::createInferBuilder(infer_logger)};
    BOOST_VERIFY(builder);
    builder->setMaxBatchSize(inference::n_batch);

    constexpr uint32_t builder_flags{
        1U << static_cast<uint32_t>(
            nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH)};
    std::unique_ptr<nvinfer1::INetworkDefinition> network{
        builder->createNetworkV2(builder_flags)};
    BOOST_VERIFY(network);

    std::unique_ptr<nvonnxparser::IParser> parser{
        nvonnxparser::createParser(*network, infer_logger)};
    BOOST_VERIFY(parser);

    const bool parse_ok{parser->parseFromFile(
        onnx.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kINFO))};
    BOOST_VERIFY(parse_ok);

    std::unique_ptr<nvinfer1::IBuilderConfig> config{
        builder->createBuilderConfig()};
    BOOST_VERIFY(config);

    std::unique_ptr<nvinfer1::IHostMemory> serialized{
        builder->buildSerializedNetwork(*network, *config)};
    BOOST_VERIFY(serialized);

    std::ofstream engine_file(engine, std::ios::binary);
    BOOST_VERIFY(engine_file);

    engine_file.write(static_cast<char*>(serialized->data()),
                      serialized->size());
    BOOST_VERIFY(!engine_file.fail());

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

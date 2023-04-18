#include <spdlog/sinks/rotating_file_sink.h>  // spdlog::sinks::rotating_file_sink_mt
#include <spdlog/sinks/stdout_sinks.h>        // spdlog::sinks::stdout_sink_mt
#include <spdlog/spdlog.h>

#include <boost/filesystem.hpp>  // boost::filesystem::path
#include <iostream>              // std::cerr
#include <thread>                // std::this_thread

#include "application.hpp"  // Application
#include "drone.h"

void setup_logging() {
  auto console_sink = std::make_shared<spdlog::sinks::stdout_sink_mt>();
  console_sink->set_level(spdlog::level::info);

  namespace fs = boost::filesystem;

  const fs::path log_path{fs::absolute("i_seed_drone_onboard.log")};
  fs::remove(log_path);

  const fs::path log_path_debug{fs::absolute("i_seed_drone_onboard_debug.log")};
  fs::remove(log_path_debug);

  constexpr std::size_t max_file_size{10ULL * 1024ULL * 1024ULL};
  constexpr std::size_t max_file_num{3};
  constexpr bool rotate_on_open{true};

  auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
      log_path.string(), max_file_size, max_file_num, rotate_on_open);
  file_sink->set_level(spdlog::level::info);

  auto file_sink_debug = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
      log_path_debug.string(), max_file_size, max_file_num, rotate_on_open);
  file_sink_debug->set_level(spdlog::level::debug);

  auto logger = std::make_shared<spdlog::logger>(
      "", spdlog::sinks_init_list({console_sink, file_sink, file_sink_debug}));

  spdlog::set_default_logger(logger);
  spdlog::set_level(spdlog::level::trace);

  spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] (t:%t) [%l] %v");
  spdlog::flush_on(spdlog::level::info);

  spdlog::info("Logging to file: {}", log_path.string());
  spdlog::info("Logging to file (debug): {}", log_path_debug.string());
}

auto run_main(int argc, char** argv) -> int {
  setup_logging();

  try {
    BOOST_VERIFY(argc == 1);
    BOOST_VERIFY(argv != nullptr);
    auto app{std::make_unique<Application>()};

    T_DjiOsalHandler* osal{DjiPlatform_GetOsalHandler()};
    BOOST_VERIFY(osal);

    // Wait for SDK to start
    std::this_thread::sleep_for(std::chrono::seconds(2));

    drone x;
    x.start();
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

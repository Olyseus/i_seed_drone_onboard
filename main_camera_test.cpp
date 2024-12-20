#include <spdlog/sinks/rotating_file_sink.h>  // spdlog::sinks::rotating_file_sink_mt
#include <spdlog/sinks/stdout_sinks.h>        // spdlog::sinks::stdout_sink_mt
#include <spdlog/spdlog.h>

#include <boost/assert.hpp>
#include <boost/filesystem.hpp>  // boost::filesystem::path
#include <iostream>              // std::cerr
#include <thread>                // std::this_thread

#include "application.hpp"  // Application
#include "camera.h"
#include "mission.h"
#include "mission_state.h"

void setup_logging() {
  auto console_sink = std::make_shared<spdlog::sinks::stdout_sink_mt>();
  console_sink->set_level(spdlog::level::debug);

  namespace fs = boost::filesystem;

  const fs::path log_path{fs::absolute("i_seed_drone_camera.log")};
  fs::remove(log_path);

  constexpr std::size_t max_file_size{10 * 1024 * 1024};
  constexpr std::size_t max_file_num{3};
  constexpr bool rotate_on_open{true};
  auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
      log_path.string(), max_file_size, max_file_num, rotate_on_open);
  file_sink->set_level(spdlog::level::debug);

  auto logger = std::make_shared<spdlog::logger>(
      "", spdlog::sinks_init_list({console_sink, file_sink}));

  spdlog::set_default_logger(logger);
  spdlog::set_level(spdlog::level::trace);

  spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] (t:%t) [%l] %v");
  spdlog::flush_on(spdlog::level::debug);

  spdlog::info("Logging to file: {}", log_path.string());
}

auto run_main(int argc, char** argv) -> int {
  setup_logging();

  try {
    OLYSEUS_VERIFY(argc == 1);
    OLYSEUS_VERIFY(argv != nullptr);
    auto app{std::make_unique<Application>()};

    mission m;
    camera c{"/var/opt/i_seed_drone_onboard/best.engine", m};

    gps_coordinates gps;
    attitude drone_attitude;
    attitude gimbal_attitude;

    spdlog::info("Shoot photo");
    c.shoot_photo(gps, drone_attitude, gimbal_attitude, 0);
    spdlog::info("Shoot photo: DONE");
    while (true) {
      spdlog::info("Check SD card");
      const bool debug_launch{true};
      if (c.check_sdcard(debug_launch)) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::seconds{1});
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

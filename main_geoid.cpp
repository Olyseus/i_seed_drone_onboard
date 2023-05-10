#include <spdlog/sinks/rotating_file_sink.h>  // spdlog::sinks::rotating_file_sink_mt
#include <spdlog/sinks/stdout_sinks.h>        // spdlog::sinks::stdout_sink_mt
#include <spdlog/spdlog.h>

#include <CLI/App.hpp>
#include <CLI/Config.hpp>     // CLI::App (link)
#include <CLI/Formatter.hpp>  // CLI::App (link)
#include <GeographicLib/Geoid.hpp>
#include <boost/filesystem.hpp>  // boost::filesystem::path
#include <iostream>              // std::cerr

#include "olyseus_verify.h"  // OLYSEUS_VERIFY

void setup_logging() {
  auto console_sink = std::make_shared<spdlog::sinks::stdout_sink_mt>();
  console_sink->set_level(spdlog::level::info);

  namespace fs = boost::filesystem;

  const fs::path log_path{fs::absolute("geoid.log")};
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
  CLI::App app{"geoid"};

  double lat{0.0};
  app.add_option("--lat", lat, "Latitude (deg)")->required();

  double lon{0.0};
  app.add_option("--lon", lon, "Longitude (deg)")->required();

  double height{0.0};
  app.add_option("--height", height, "Height above mean sea level")->required();

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  setup_logging();

  try {
    spdlog::info("Create geoid");
    GeographicLib::Geoid geoid("egm96-5", "/usr/share/GeographicLib/geoids/");
    spdlog::info("Geoid created");

    spdlog::info("Cache geoid data");
    geoid.CacheAll();
    spdlog::info("Caching finished");

    const double altitude{geoid.ConvertHeight(
        lat, lon, height, GeographicLib::Geoid::GEOIDTOELLIPSOID)};
    spdlog::info("Altitude above WGS84 reference ellipsoid: {} (m)", altitude);

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

#include <boost/assert.hpp>
#include <boost/filesystem.hpp>  // boost::filesystem::path
#include <iostream>              // std::cerr
#include <spdlog/sinks/rotating_file_sink.h>  // spdlog::sinks::rotating_file_sink_mt
#include <spdlog/sinks/stdout_sinks.h>        // spdlog::sinks::stdout_sink_mt
#include <spdlog/spdlog.h>
#include <spdlog/spdlog.h>
#include <thread>  // std::this_thread

// Onboard SDK
#include <dji_linux_helpers.hpp>  // LinuxSetup

#include "api_code.h"

void setup_logging() {
  auto console_sink = std::make_shared<spdlog::sinks::stdout_sink_mt>();
  console_sink->set_level(spdlog::level::info);

  const boost::filesystem::path log_path{"i_seed_drone_onboard_quaternion.log"};
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
  setup_logging();

  try {
    constexpr bool enable_advanced_sensing{true};
    LinuxSetup linux_setup{argc, argv, enable_advanced_sensing};

    DJI::OSDK::Vehicle* vehicle{linux_setup.getVehicle()};
    BOOST_VERIFY(vehicle);

    constexpr int freq{5};
    DJI::OSDK::Telemetry::TopicName topic_list[] = {
        DJI::OSDK::Telemetry::TOPIC_QUATERNION};
    constexpr std::size_t num_topic{sizeof(topic_list) / sizeof(topic_list[0])};
    constexpr bool enable_timestamp{false};

    constexpr int timeout{20};

    api_code code{vehicle->subscribe->verify(timeout)};
    BOOST_VERIFY(code.success());

    constexpr int pkg_index{0};

    const bool pkg_status = vehicle->subscribe->initPackageFromTopicList(
        pkg_index, num_topic, topic_list, enable_timestamp, freq);
    BOOST_VERIFY(pkg_status);

    code = api_code{vehicle->subscribe->startPackage(pkg_index, timeout)};
    BOOST_VERIFY(code.success());

    BOOST_VERIFY(vehicle->gimbalManager);
    code = api_code{vehicle->gimbalManager->initGimbalModule(PAYLOAD_INDEX_0, "g0")};
    BOOST_VERIFY(code.success());

    std::this_thread::sleep_for(std::chrono::seconds(2));

    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      DJI::OSDK::Telemetry::Quaternion quaternion{
          vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_QUATERNION>()};

      // https://github.com/dji-sdk/Onboard-SDK/blob/2c38de17f7aad0064056f27eaa219d4ed30ab82a/sample/platform/STM32/OnBoardSDK_STM32/User/FlightControlSample.cpp#L800-L824
      const double q2sqr{quaternion.q2 * quaternion.q2};
      const double t0{-2.0 * (q2sqr + quaternion.q3 * quaternion.q3) + 1.0};
      const double t1{+2.0 * (quaternion.q1 * quaternion.q2 + quaternion.q0 * quaternion.q3)};
      double t2{-2.0 * (quaternion.q1 * quaternion.q3 - quaternion.q0 * quaternion.q2)};
      const double t3{+2.0 * (quaternion.q2 * quaternion.q3 + quaternion.q0 * quaternion.q1)};
      const double t4{-2.0 * (quaternion.q1 * quaternion.q1 + q2sqr) + 1.0};

      t2 = (t2 > 1.0) ? 1.0 : t2;
      t2 = (t2 < -1.0) ? -1.0 : t2;

      const double roll{asin(t2) * 180.0 / M_PI}; // X
      const double pitch{atan2(t3, t4) * 180.0 / M_PI}; // Y
      const double yaw{atan2(t1, t0) * 180.0 / M_PI}; // Z

      spdlog::info("roll: {}, pitch: {}, yaw: {}", roll, pitch, yaw);
    }

    // last
    vehicle->subscribe->removePackage(pkg_index, timeout);

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

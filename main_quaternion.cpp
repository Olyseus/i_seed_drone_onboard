#include <dji_fc_subscription.h>              // T_DjiFcSubscriptionQuaternion
#include <spdlog/sinks/rotating_file_sink.h>  // spdlog::sinks::rotating_file_sink_mt
#include <spdlog/sinks/stdout_sinks.h>        // spdlog::sinks::stdout_sink_mt
#include <spdlog/spdlog.h>

#include <boost/assert.hpp>
#include <boost/filesystem.hpp>  // boost::filesystem::path
#include <iostream>              // std::cerr
#include <thread>                // std::this_thread

#include "api_code.h"
#include "application.hpp"   // Application
#include "olyseus_verify.h"  // OLYSEUS_VERIFY

void setup_logging() {
  auto console_sink = std::make_shared<spdlog::sinks::stdout_sink_mt>();
  console_sink->set_level(spdlog::level::info);

  namespace fs = boost::filesystem;

  const fs::path log_path{fs::absolute("i_seed_drone_onboard_quaternion.log")};
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

T_DjiReturnCode quaternion_callback(const uint8_t* data, uint16_t data_size,
                                    const T_DjiDataTimestamp* timestamp) {
  OLYSEUS_VERIFY(data != nullptr);
  const auto quaternion{*(const T_DjiFcSubscriptionQuaternion*)data};
  (void)data_size;
  (void)timestamp;

  // https://github.com/dji-sdk/Onboard-SDK/blob/2c38de17f7aad0064056f27eaa219d4ed30ab82a/sample/platform/STM32/OnBoardSDK_STM32/User/FlightControlSample.cpp#L800-L824
  const double q2sqr{quaternion.q2 * quaternion.q2};
  const double t0{-2.0 * (q2sqr + quaternion.q3 * quaternion.q3) + 1.0};
  const double t1{
      +2.0 * (quaternion.q1 * quaternion.q2 + quaternion.q0 * quaternion.q3)};
  double t2{-2.0 *
            (quaternion.q1 * quaternion.q3 - quaternion.q0 * quaternion.q2)};
  const double t3{
      +2.0 * (quaternion.q2 * quaternion.q3 + quaternion.q0 * quaternion.q1)};
  const double t4{-2.0 * (quaternion.q1 * quaternion.q1 + q2sqr) + 1.0};

  t2 = (t2 > 1.0) ? 1.0 : t2;
  t2 = (t2 < -1.0) ? -1.0 : t2;

  // https://sdk-forum.dji.net/hc/en-us/requests/74003
  // https://sdk-forum.dji.net/hc/en-us/articles/360023657273
  const double roll{atan2(t3, t4) * 180.0 / M_PI};  // X
  const double pitch{asin(t2) * 180.0 / M_PI};      // Y
  const double yaw{atan2(t1, t0) * 180.0 / M_PI};   // Z

  spdlog::info("roll: {}, pitch: {}, yaw: {}", roll, pitch, yaw);

  OLYSEUS_VERIFY(yaw >= -180.0);
  OLYSEUS_VERIFY(yaw <= 180.0);
  OLYSEUS_VERIFY(pitch > -90.0);
  OLYSEUS_VERIFY(pitch < 90.0);
  OLYSEUS_VERIFY(roll > -90.0);
  OLYSEUS_VERIFY(roll < 90.0);

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

auto rtk_position_callback(const uint8_t* data, uint16_t data_size,
                           const T_DjiDataTimestamp* timestamp)
    -> T_DjiReturnCode {
  OLYSEUS_VERIFY(data != nullptr);
  const auto position{*(const T_DjiFcSubscriptionRtkPosition*)data};
  (void)data_size;
  (void)timestamp;

  // Note: position.hfsl;
  // In documentation it's "height above mean sea level" but in fact
  // the type is WGS 84 (outside of China)
  // - https://sdk-forum.dji.net/hc/en-us/requests/82680

  spdlog::info("RTK, drone latitude: {}, longitude: {}, altitude: {}",
               position.latitude, position.longitude, position.hfsl);

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

auto run_main(int argc, char** argv) -> int {
  setup_logging();

  try {
    OLYSEUS_VERIFY(argc == 1);
    OLYSEUS_VERIFY(argv != nullptr);
    auto app{std::make_unique<Application>()};

    T_DjiOsalHandler* osal{DjiPlatform_GetOsalHandler()};
    OLYSEUS_VERIFY(osal);

    // Wait for SDK to start
    std::this_thread::sleep_for(std::chrono::seconds(2));

    constexpr E_DjiDataSubscriptionTopicFreq topic_freq{
        DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ};

    T_DjiReturnCode code{DjiFcSubscription_Init()};
    OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, topic_freq, quaternion_callback);
    OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    code =
        DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
                                         topic_freq, rtk_position_callback);
    OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    while (true) {
      // Receive callbacks until interrupted
      std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    code = DjiFcSubscription_DeInit();
    OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

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

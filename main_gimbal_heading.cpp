#include <boost/assert.hpp>
#include <boost/filesystem.hpp>  // boost::filesystem::path
#include <dji_fc_subscription.h> // T_DjiFcSubscriptionQuaternion
#include <dji_gimbal_manager.h> // DjiGimbalManager_Init
#include <iostream>              // std::cerr
#include <spdlog/sinks/rotating_file_sink.h>  // spdlog::sinks::rotating_file_sink_mt
#include <spdlog/sinks/stdout_sinks.h>        // spdlog::sinks::stdout_sink_mt
#include <spdlog/spdlog.h>
#include <spdlog/spdlog.h>
#include <thread>  // std::this_thread

#include "application.hpp" // Application

#include "api_code.h"

void setup_logging() {
  auto console_sink = std::make_shared<spdlog::sinks::stdout_sink_mt>();
  console_sink->set_level(spdlog::level::info);

  const boost::filesystem::path log_path{"i_seed_drone_onboard_gimbal_heading.log"};
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

static double drone_yaw{0.0};
static double gimbal_roll{0.0};
static double gimbal_pitch{0.0};
static double gimbal_yaw{0.0};

T_DjiReturnCode quaternion_callback(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp* timestamp) {
  BOOST_VERIFY(data != nullptr);
  const auto quaternion{*(const T_DjiFcSubscriptionQuaternion*)data};;
  (void)data_size;
  (void)timestamp;

  // https://github.com/dji-sdk/Onboard-SDK/blob/2c38de17f7aad0064056f27eaa219d4ed30ab82a/sample/platform/STM32/OnBoardSDK_STM32/User/FlightControlSample.cpp#L800-L824
  const double q2sqr{quaternion.q2 * quaternion.q2};
  const double t0{-2.0 * (q2sqr + quaternion.q3 * quaternion.q3) + 1.0};
  const double t1{+2.0 * (quaternion.q1 * quaternion.q2 + quaternion.q0 * quaternion.q3)};

  // https://sdk-forum.dji.net/hc/en-us/requests/74003
  // https://sdk-forum.dji.net/hc/en-us/articles/360023657273
  drone_yaw = atan2(t1, t0) * 180.0 / M_PI; // Z

  spdlog::info("drone yaw: {}", drone_yaw);

  BOOST_VERIFY(drone_yaw >= -180.0);
  BOOST_VERIFY(drone_yaw <= 180.0);

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode gimbal_callback(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp* timestamp) {
  BOOST_VERIFY(data != nullptr);
  const auto angles{*(const T_DjiFcSubscriptionGimbalAngles*)data};;
  (void)data_size;
  (void)timestamp;

  gimbal_pitch = angles.x;
  gimbal_roll = angles.y;
  gimbal_yaw = angles.z;

  spdlog::info("gimbal pitch: {}, roll: {}, yaw: {}", gimbal_pitch, gimbal_roll, gimbal_yaw);

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
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

    T_DjiReturnCode code{DjiFcSubscription_Init()};
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
        quaternion_callback);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
        gimbal_callback);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    code = DjiGimbalManager_Init();
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    const E_DjiMountPosition m_pos{DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1};

    code = DjiGimbalManager_SetMode(m_pos, DJI_GIMBAL_MODE_FREE);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    code = DjiGimbalManager_Reset(m_pos);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      T_DjiGimbalManagerRotation rotation;
      rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE;
      rotation.pitch = 0.0; // -90.0: down, 0.0: forward
      rotation.roll = 0.0;
      rotation.yaw = drone_yaw;
      rotation.time = 0.1;

      constexpr float eps{1e-3};
      constexpr float rough_eps{0.1 + eps};
      const double d_roll{std::abs(rotation.roll - gimbal_roll)};
      const double d_pitch{std::abs(rotation.pitch - gimbal_pitch)};
      const double d_yaw{std::abs(rotation.yaw - gimbal_yaw)};
      if (d_roll < rough_eps && d_pitch < rough_eps && d_yaw < rough_eps) {
        continue;
      }

      spdlog::info("Run gimbal rotation");

      code = DjiGimbalManager_Rotate(m_pos, rotation);
      BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    }

    code = DjiFcSubscription_DeInit();
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

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

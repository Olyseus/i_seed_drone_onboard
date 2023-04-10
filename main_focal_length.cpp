#include <dji_camera_manager.h>  // DjiCameraManager_Init
#include <dji_payload_camera.h>  // DjiPayloadCamera_GetCameraHybridZoomFocalLengthOfPayload
#include <spdlog/sinks/rotating_file_sink.h>  // spdlog::sinks::rotating_file_sink_mt
#include <spdlog/sinks/stdout_sinks.h>        // spdlog::sinks::stdout_sink_mt
#include <spdlog/spdlog.h>

#include <boost/assert.hpp>
#include <boost/filesystem.hpp>  // boost::filesystem::path
#include <iostream>              // std::cerr
#include <thread>                // std::this_thread

#include "application.hpp"  // Application

void setup_logging() {
  auto console_sink = std::make_shared<spdlog::sinks::stdout_sink_mt>();
  console_sink->set_level(spdlog::level::info);

  namespace fs = boost::filesystem;

  const fs::path log_path{
      fs::absolute("i_seed_drone_onboard_focal_length.log")};
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
  setup_logging();

  try {
    BOOST_VERIFY(argc == 1);
    BOOST_VERIFY(argv != nullptr);
    auto app{std::make_unique<Application>()};

    T_DjiOsalHandler* osal{DjiPlatform_GetOsalHandler()};
    BOOST_VERIFY(osal);

    std::this_thread::sleep_for(std::chrono::seconds(2));

    T_DjiReturnCode code{DjiCameraManager_Init()};
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    E_DjiCameraType camera_type;
    constexpr E_DjiMountPosition m_pos{DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1};
    code = DjiCameraManager_GetCameraType(m_pos, &camera_type);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    BOOST_VERIFY(camera_type == DJI_CAMERA_TYPE_H20);

    T_DjiCameraManagerFirmwareVersion firmware_version;
    code = DjiCameraManager_GetFirmwareVersion(m_pos, &firmware_version);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    spdlog::info("Camera firmware version {:02d}.{:02d}.{:02d}.{:02d}",
                 firmware_version.firmware_version[0],
                 firmware_version.firmware_version[1],
                 firmware_version.firmware_version[2],
                 firmware_version.firmware_version[3]);

    // https://developer.dji.com/document/b0776c88-399e-4cd7-8142-681182de3dbd
    // At least: Matrice 300 RTK：V03.00.01.01
    BOOST_VERIFY(firmware_version.firmware_version[0] >= 4);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    T_DjiCameraOpticalZoomSpec optical_zoom_spec;
    code = DjiPayloadCamera_GetCameraOpticalZoomSpecOfPayload(
        m_pos, &optical_zoom_spec);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    spdlog::info("focal length min: {}, max: {}, step: {}",
                 optical_zoom_spec.minFocalLength,
                 optical_zoom_spec.maxFocalLength,
                 optical_zoom_spec.focalLengthStep);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    while (true) {
      const E_DjiCameraManagerFocusMode expected_focus_mode{
          DJI_CAMERA_MANAGER_FOCUS_MODE_MANUAL};

      // If failed, check camera is ZOOM and the pause is long enough:
      // - https://sdk-forum.dji.net/hc/en-us/requests/73828
      E_DjiCameraManagerFocusMode focus_mode;
      code = DjiCameraManager_GetFocusMode(m_pos, &focus_mode);
      BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

      if (focus_mode != expected_focus_mode) {
        spdlog::info("Changing focus mode to MF");
        code = DjiCameraManager_SetFocusMode(m_pos, expected_focus_mode);
        BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
      }

      uint16_t focal_length{0};
      code = DjiPayloadCamera_GetCameraHybridZoomFocalLengthOfPayload(
          m_pos, &focal_length);
      BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
      spdlog::info("Focal length: {}", focal_length);

      if (focal_length != optical_zoom_spec.minFocalLength) {
        T_DjiCameraManagerOpticalZoomParam optical_zoom_param;
        code = DjiCameraManager_GetOpticalZoomParam(m_pos, &optical_zoom_param);
        BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
        spdlog::info("zoom param current: {}",
                     optical_zoom_param.currentOpticalZoomFactor);

        code = DjiCameraManager_SetOpticalZoomParam(
            m_pos, DJI_CAMERA_ZOOM_DIRECTION_OUT,
            optical_zoom_param.currentOpticalZoomFactor *
                optical_zoom_spec.minFocalLength / focal_length);
        BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

        code = DjiPayloadCamera_GetCameraHybridZoomFocalLengthOfPayload(
            m_pos, &focal_length);
        BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

        BOOST_VERIFY(focal_length == optical_zoom_spec.minFocalLength);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    code = DjiCameraManager_DeInit();
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

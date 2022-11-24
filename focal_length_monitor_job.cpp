#include <boost/assert.hpp>  // BOOST_VERIFY
#include <spdlog/spdlog.h>
#include <thread> // std::this_thread

// Payload SDK
#include <dji_camera_manager.h>  // DjiCameraManager_Init
#include <dji_payload_camera.h>  // DjiPayloadCamera_GetCameraHybridZoomFocalLengthOfPayload

void focal_length_monitor_job() {
  T_DjiReturnCode code{DjiCameraManager_Init()};
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiPayloadCamera_Init();
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

  while (true) {
    if (false) { // FIXME (enable)
      const E_DjiCameraManagerFocusMode expected_focus_mode{DJI_CAMERA_MANAGER_FOCUS_MODE_AUTO};

      E_DjiCameraManagerFocusMode focus_mode;
      code = DjiCameraManager_GetFocusMode(m_pos, &focus_mode);
      BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

      if (focus_mode != expected_focus_mode) {
        spdlog::info("Changing focus mode");
        code = DjiCameraManager_SetFocusMode(m_pos, expected_focus_mode);
        BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
        spdlog::info("Changing focus mode: DONE");
      }
    }

    uint16_t focal_length{0};
    code = DjiPayloadCamera_GetCameraHybridZoomFocalLengthOfPayload(m_pos, &focal_length);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    spdlog::info("Focal length: {}", focal_length);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  code = DjiCameraManager_DeInit();
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
}

#include "camera_psdk.h"

#include <boost/assert.hpp>  // BOOST_VERIFY
#include <boost/filesystem.hpp>
#include <dji_camera_manager.h>  // DjiCameraManager_Init
#include <dji_liveview.h>  // DjiLiveview_Init
#include <dji_payload_camera.h>  // DjiPayloadCamera_GetCameraHybridZoomFocalLengthOfPayload
#include <spdlog/spdlog.h>

#include <sstream>  // std::ostringstream
#include <thread>  // std::this_thread

#include "application.hpp"
#include "bounding_box.h"

static std::string camera_psdk_file_dst;
static std::FILE* camera_psdk_file;

T_DjiReturnCode camera_psdk_data_callback(
    T_DjiDownloadFilePacketInfo packetInfo, const uint8_t *data, uint16_t len) {
  if (packetInfo.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_START) {
    BOOST_VERIFY(!camera_psdk_file_dst.empty());
    camera_psdk_file = fopen(camera_psdk_file_dst.c_str(), "wb+");
    camera_psdk_file_dst.clear();
  }

  BOOST_VERIFY(camera_psdk_file);
  std::size_t res{fwrite(data, 1, len, camera_psdk_file)};
  BOOST_VERIFY(res == len);

  if (packetInfo.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_END) {
    int res{fclose(camera_psdk_file)};
    BOOST_VERIFY(res == 0);
    camera_psdk_file = nullptr;
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

constexpr int camera_psdk::mount_position() {
  return DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;
}

camera_psdk::camera_psdk(const std::string& model_file) :
    inference_(model_file) {
  T_DjiOsalHandler* osal{DjiPlatform_GetOsalHandler()};
  BOOST_VERIFY(osal);

  T_DjiReturnCode code{DjiCameraManager_Init()};
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  E_DjiCameraType camera_type;

  constexpr auto m_pos{static_cast<E_DjiMountPosition>(mount_position())};
  code = DjiCameraManager_GetCameraType(m_pos, &camera_type);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
  BOOST_VERIFY(camera_type == DJI_CAMERA_TYPE_H20);

  T_DjiCameraManagerFirmwareVersion firmware_version;

  code = DjiCameraManager_GetFirmwareVersion(m_pos, &firmware_version);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  // https://developer.dji.com/document/b0776c88-399e-4cd7-8142-681182de3dbd
  // At least: Matrice 300 RTK：V03.00.01.01
  BOOST_VERIFY(firmware_version.firmware_version[0] >= 4);

  code = DjiCameraManager_SetMode(m_pos,
      DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiCameraManager_SetShootPhotoMode(
      m_pos, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiCameraManager_RegDownloadFileDataCallback(m_pos,
      camera_psdk_data_callback);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  /* FIXME (remove)
  code = DjiLiveview_Init();
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
  */

  T_DjiCameraManagerFileList media_file_list;
  spdlog::info("Downloading file list"); // FIXME (remove)
  code = DjiCameraManager_DownloadFileList(m_pos, &media_file_list);
  // code = DjiCameraManager_DownloadFileList(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, &media_file_list);
  // code = DjiCameraManager_DownloadFileList(static_cast<E_DjiMountPosition>(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1), &media_file_list);
  spdlog::info("Downloading file list: DONE"); // FIXME (remove)

  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
  for (int i = 0; i < media_file_list.totalCount; ++i) {
    const T_DjiCameraManagerFileListInfo& info{media_file_list.fileListInfo[i]};

    // FIXME (remove)
    spdlog::info(
                   "  Name: {}, index: {}, time:{}-{}-{}_{}:{}:{}, size: {:.2f} MB",
                   info.fileName,
                   info.fileIndex,
                   info.createTime.year,
                   info.createTime.month,
                   info.createTime.day,
                   info.createTime.hour,
                   info.createTime.minute,
                   info.createTime.second,
                   info.fileSize / (1024.0 * 1024.0));
    if (!std::regex_match(info.fileName, jpeg_regex)) {
      continue;
    }

    auto res{already_present_indexes_.insert(info.fileIndex)};
    BOOST_VERIFY(res.second);
  }

  spdlog::info("Number of JPEG files on SD card: {}", already_present_indexes_.size());
}

camera_psdk::~camera_psdk() {
  /* FIXME (remove)
  T_DjiReturnCode code{DjiLiveview_Init()};
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
  */

  spdlog::info("Deinit camera"); // FIXME (remove)
  T_DjiReturnCode code = DjiCameraManager_DeInit();
  spdlog::info("Deinit camera: DONE"); // FIXME (remove)
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
}

void camera_psdk::shoot_photo(const gps_coordinates& gps, const quaternion& quat,
    const gimbal_data& gimbal) {
  {
    std::lock_guard lock{queue_mutex_};
    queue_.push_back({gps, quat, gimbal});
    file_waiting_timer_.start();
  }

  {
    std::lock_guard lock{api_call_mutex_};
    spdlog::debug("Shoot photo request");

    constexpr auto m_pos{static_cast<E_DjiMountPosition>(mount_position())};

    T_DjiCameraOpticalZoomSpec optical_zoom_spec;
    T_DjiReturnCode code{DjiPayloadCamera_GetCameraOpticalZoomSpecOfPayload(m_pos, &optical_zoom_spec)};
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    spdlog::debug("focal length min: {}, max: {}, step: {}", optical_zoom_spec.minFocalLength, optical_zoom_spec.maxFocalLength, optical_zoom_spec.focalLengthStep);

    const E_DjiCameraManagerFocusMode expected_focus_mode{DJI_CAMERA_MANAGER_FOCUS_MODE_MANUAL};

    // If failed, check camera is ZOOM and the pause is long enough:
    // - https://sdk-forum.dji.net/hc/en-us/requests/73828
    E_DjiCameraManagerFocusMode focus_mode;
    code = DjiCameraManager_GetFocusMode(m_pos, &focus_mode);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    if (focus_mode != expected_focus_mode) {
      spdlog::debug("Changing focus mode to MF");
      code = DjiCameraManager_SetFocusMode(m_pos, expected_focus_mode);
      BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    }

    uint16_t focal_length{0};
    code = DjiPayloadCamera_GetCameraHybridZoomFocalLengthOfPayload(m_pos, &focal_length);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    spdlog::debug("Focal length: {}", focal_length);
    BOOST_VERIFY(focal_length == optical_zoom_spec.minFocalLength);

    // FIXME, try:
    // DJI_CAMERA_MANAGER_EXPOSURE_MODE_PROGRAM_AUTO
    // DJI_CAMERA_MANAGER_EXPOSURE_MODE_SHUTTER_PRIORITY
    // DJI_CAMERA_MANAGER_EXPOSURE_MODE_APERTURE_PRIORITY
    code = DjiCameraManager_SetExposureMode(m_pos, DJI_CAMERA_MANAGER_EXPOSURE_MODE_EXPOSURE_MANUAL);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    code = DjiCameraManager_SetISO(m_pos, DJI_CAMERA_MANAGER_ISO_100);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    code = DjiCameraManager_SetAperture(m_pos, DJI_CAMERA_MANAGER_APERTURE_F_1_DOT_6);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    code = DjiCameraManager_SetShutterSpeed(m_pos, DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_8000);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    spdlog::debug("Call DjiCameraManager_StartShootPhoto");
    code = DjiCameraManager_StartShootPhoto(m_pos,
            DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    spdlog::debug("DjiCameraManager_StartShootPhoto OK");

    E_DjiCameraManagerISO iso;
    code = DjiCameraManager_GetISO(m_pos, &iso);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    spdlog::info("ISO: {}", iso_name(iso));

    E_DjiCameraManagerAperture aperture;
    code = DjiCameraManager_GetAperture(m_pos, &aperture);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    spdlog::info("aperture: {}", aperture_name(aperture));

    E_DjiCameraManagerShutterSpeed shutter_speed;
    code = DjiCameraManager_GetShutterSpeed(m_pos, &shutter_speed);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    spdlog::info("shutter speed: {}", shutter_speed_name(shutter_speed));

    E_DjiCameraManagerExposureCompensation compensation;
    code = DjiCameraManager_GetExposureCompensation(m_pos, &compensation);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    spdlog::info("exposure compensation: {}", compensation_name(compensation));
  }
}

auto camera_psdk::check_sdcard() -> bool {
  {
    std::lock_guard lock{queue_mutex_};
    constexpr int64_t wait_ms{20 * 1000}; // 20 sec
    if (!queue_.empty() && file_waiting_timer_.elapsed_ms() > wait_ms) {
      throw std::runtime_error("Waiting for a file too long. SD Card is full?");
    }
  }

  T_DjiCameraManagerFileList media_file_list;
  {
    std::lock_guard lock{api_call_mutex_};
    constexpr auto m_pos{static_cast<E_DjiMountPosition>(mount_position())};
    spdlog::info("Downloading file list"); // FIXME (remove)
    const T_DjiReturnCode code{
      DjiCameraManager_DownloadFileList(m_pos, &media_file_list)};
    spdlog::info("Downloading file list: DONE"); // FIXME (remove)
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
  }

  std::vector<std::pair<uint32_t, T_DjiCameraManagerFileCreateTime>> inference_files;
  std::set<uint32_t> new_indexes;

  for (int i = 0; i < media_file_list.totalCount; ++i) {
    const T_DjiCameraManagerFileListInfo& info{media_file_list.fileListInfo[i]};

    // FIXME (remove)
    spdlog::info(
                   "  Name: {}, index: {}, time:{}-{}-{}_{}:{}:{}, size: {:.2f} MB",
                   info.fileName,
                   info.fileIndex,
                   info.createTime.year,
                   info.createTime.month,
                   info.createTime.day,
                   info.createTime.hour,
                   info.createTime.minute,
                   info.createTime.second,
                   info.fileSize / (1024.0 * 1024.0));

    if (!std::regex_match(info.fileName, jpeg_regex)) {
      continue;
    }
    BOOST_VERIFY(info.type == DJI_CAMERA_FILE_TYPE_JPEG);

    auto res{new_indexes.insert(info.fileIndex)};
    BOOST_VERIFY(res.second);

    if (already_present_indexes_.find(info.fileIndex) == already_present_indexes_.end()) {
      inference_files.emplace_back(info.fileIndex, info.createTime);
    }
  }

  already_present_indexes_ = new_indexes;

  if (inference_files.empty()) {
    // No files to process
    spdlog::info("No files to process"); // FIXME (remove)
    return false;
  }

  std::sort(inference_files.begin(), inference_files.end(),
      [](const auto& a_pair, const auto& b_pair) {
        const T_DjiCameraManagerFileCreateTime& a{a_pair.second};
        const T_DjiCameraManagerFileCreateTime& b{b_pair.second};
        return std::tie(a.year, a.month, a.day, a.hour, a.minute, a.second) <
               std::tie(b.year, b.month, b.day, b.hour, b.minute, b.second);
  });

  spdlog::info("{} files to process", inference_files.size());

  namespace fs = boost::filesystem;

  fs::path top_dir{"/var/opt/i_seed_drone_onboard"};
  fs::create_directories(top_dir);
  BOOST_VERIFY(fs::is_directory(top_dir));

  for (const auto& x : inference_files) {
    const uint32_t file_index{x.first};

    const T_DjiCameraManagerFileCreateTime& t{x.second};
    std::ostringstream file_dst;
    file_dst << t.year << '_' << static_cast<unsigned>(t.month) << '_'
        << static_cast<unsigned>(t.day) << '_'
        << static_cast<unsigned>(t.hour) << '_'
        << static_cast<unsigned>(t.minute) << '_'
        << static_cast<unsigned>(t.second) << ".jpg";

    const fs::path dst_path{top_dir / file_dst.str()};
    BOOST_VERIFY(camera_psdk_file_dst.empty());
    camera_psdk_file_dst = dst_path.string();
    spdlog::info("Download file with index {} to {}", file_index, camera_psdk_file_dst);
    constexpr auto m_pos{static_cast<E_DjiMountPosition>(mount_position())};
    T_DjiReturnCode code{DjiCameraManager_DownloadFileByIndex(m_pos, file_index)};
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      spdlog::critical("Error code: {}", code);
    }
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    // global variable should be cleared in callbacks for future use
    BOOST_VERIFY(camera_psdk_file_dst.empty());

    // check file is created at this point
    BOOST_VERIFY(fs::is_regular_file(dst_path));

    // The file can be deleted only after a download from sd card
    code = DjiCameraManager_DeleteFileByIndex(m_pos, file_index);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    queue_entry queue_head;
    {
      std::lock_guard lock{queue_mutex_};
      BOOST_VERIFY(!queue_.empty());
      queue_head = queue_.front();
      queue_.pop_front();
    }

    std::vector<bounding_box> bb{inference_.run(dst_path.string())};
    if (bb.empty()) {
      spdlog::info("No objects found");
      continue;
    }

    spdlog::info("{} objects detected", bb.size());
    for (const bounding_box& b : bb) {
      spdlog::info("x: {}, y: {}, {:.2f}%", b.mid_x(), b.mid_y(), b.confidence() * 100.0);
    }

    detection_result res;
    res.gps = queue_head.gps;
    res.quat = queue_head.quat;
    res.gimbal = queue_head.gimbal;
    for (const bounding_box& box: bb) {
      res.pixels.push_back({box.mid_x(), box.mid_y()});
    }

    detections_.push_back(res);
  }

  return true;
}

auto camera_psdk::iso_name(int value) -> const char* {
  switch (value) {
    case DJI_CAMERA_MANAGER_ISO_AUTO:
      return "auto";
    case DJI_CAMERA_MANAGER_ISO_100:
      return "100";
    case DJI_CAMERA_MANAGER_ISO_200:
      return "200";
    case DJI_CAMERA_MANAGER_ISO_400:
      return "400";
    case DJI_CAMERA_MANAGER_ISO_800:
      return "800";
    case DJI_CAMERA_MANAGER_ISO_1600:
      return "1600";
    case DJI_CAMERA_MANAGER_ISO_3200:
      return "3200";
    case DJI_CAMERA_MANAGER_ISO_6400:
      return "6400";
    case DJI_CAMERA_MANAGER_ISO_12800:
      return "12800";
    case DJI_CAMERA_MANAGER_ISO_25600:
      return "25600";
    case DJI_CAMERA_MANAGER_ISO_FIXED:
      return "fixed";
    default:
      BOOST_VERIFY(false);
      return "";
  }
}

auto camera_psdk::aperture_name(int value) -> const char* {
  switch (value) {
    case DJI_CAMERA_MANAGER_APERTURE_F_1_DOT_6:
      // It is only supported by Z30 camera
      return "f/1.6";
    case DJI_CAMERA_MANAGER_APERTURE_F_1_DOT_7:
      return "f/1.7";
    case DJI_CAMERA_MANAGER_APERTURE_F_1_DOT_8:
      return "f/1.8";
    case DJI_CAMERA_MANAGER_APERTURE_F_2:
      return "f/2";
    case DJI_CAMERA_MANAGER_APERTURE_F_2_DOT_2:
      return "f/2.2";
    case DJI_CAMERA_MANAGER_APERTURE_F_2_DOT_4:
      // It is only supported by Z30 camera
      return "f/2.4";
    case DJI_CAMERA_MANAGER_APERTURE_F_2_DOT_5:
      return "f/2.5";
    case DJI_CAMERA_MANAGER_APERTURE_F_2_DOT_6:
      return "f/2.6";
    case DJI_CAMERA_MANAGER_APERTURE_F_2_DOT_8:
      return "f/2.8";
    case DJI_CAMERA_MANAGER_APERTURE_F_3_DOT_2:
      return "f/3.2";
    case DJI_CAMERA_MANAGER_APERTURE_F_3_DOT_4:
      return "f/3.4";
    case DJI_CAMERA_MANAGER_APERTURE_F_3_DOT_5:
      return "f/3.5";
    case DJI_CAMERA_MANAGER_APERTURE_F_4:
      return "f/4";
    case DJI_CAMERA_MANAGER_APERTURE_F_4_DOT_5:
      return "f/4.5";
    case DJI_CAMERA_MANAGER_APERTURE_F_4_DOT_8:
      return "f/4.8";
    case DJI_CAMERA_MANAGER_APERTURE_F_5:
      return "f/5";
    case DJI_CAMERA_MANAGER_APERTURE_F_5_DOT_6:
      return "f/5.6";
    case DJI_CAMERA_MANAGER_APERTURE_F_6_DOT_3:
      return "f/6.3";
    case DJI_CAMERA_MANAGER_APERTURE_F_6_DOT_8:
      return "f/6.8";
    case DJI_CAMERA_MANAGER_APERTURE_F_7_DOT_1:
      return "f/7.1";
    case DJI_CAMERA_MANAGER_APERTURE_F_8:
      return "f/8";
    case DJI_CAMERA_MANAGER_APERTURE_F_9:
      return "f/9";
    case DJI_CAMERA_MANAGER_APERTURE_F_9_DOT_6:
      return "f/9.6";
    case DJI_CAMERA_MANAGER_APERTURE_F_10:
      return "f/10";
    case DJI_CAMERA_MANAGER_APERTURE_F_11:
      return "f/11";
    case DJI_CAMERA_MANAGER_APERTURE_F_13:
      return "f/13";
    case DJI_CAMERA_MANAGER_APERTURE_F_14:
      return "f/14";
    case DJI_CAMERA_MANAGER_APERTURE_F_16:
      return "f/16";
    case DJI_CAMERA_MANAGER_APERTURE_F_18:
      return "f/18";
    case DJI_CAMERA_MANAGER_APERTURE_F_19:
      return "f/19";
    case DJI_CAMERA_MANAGER_APERTURE_F_20:
      return "f/20";
    case DJI_CAMERA_MANAGER_APERTURE_F_22:
      return "f/22";
    case DJI_CAMERA_MANAGER_APERTURE_F_UNKNOWN:
      return "unknown";
    default:
      spdlog::critical("unknown value: {}", value);
      BOOST_VERIFY(false);
      return "";
  }
}

auto camera_psdk::shutter_speed_name(int value) -> const char* {
  switch (value) {
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_8000: return "1/8000 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_6400: return "1/6400 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_6000: return "1/6000 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_5000: return "1/5000 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_4000: return "1/4000 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_3200: return "1/3200 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_3000: return "1/3000 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_2500: return "1/2500 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_2000: return "1/2000 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_1600: return "1/1600 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_1500: return "1/1500 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_1250: return "1/1250 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_1000: return "1/1000 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_800: return "1/800 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_725: return "1/725 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_640: return "1/640 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_500: return "1/500 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_400: return "1/400 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_350: return "1/350 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_320: return "1/320 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_250: return "1/250 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_240: return "1/240 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_200: return "1/200 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_180: return "1/180 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_160: return "1/160 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_125: return "1/125 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_120: return "1/120 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_100: return "1/100 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_90: return "1/90 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_80: return "1/80 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_60: return "1/60 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_50: return "1/50 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_40: return "1/40 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_30: return "1/30 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_25: return "1/25 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_20: return "1/20 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_15: return "1/15 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_12DOT5: return "1/12.5 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_10: return "1/10 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_8: return "1/8 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_6DOT25: return "1/6.25 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_5: return "1/5 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_4: return "1/4 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_3: return "1/3 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_2DOT5: return "1/2.5 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_2: return "1/2 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_1DOT67: return "1/1.67 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_1DOT25: return "1/1.25 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1: return "1.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1DOT3: return "1.3 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1DOT6: return "1.6 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_2: return "2.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_2DOT5: return "2.5 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_3: return "3.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_3DOT2: return "3.2 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_4: return "4.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_5: return "5.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_6: return "6.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_7: return "7.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_8: return "8.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_9: return "9.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_10: return "10.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_13: return "13.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_15: return "15.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_20: return "20.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_25: return "25.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_30: return "30.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_UNKNOWN: return "";
    default:
      BOOST_VERIFY(false);
      return "";
  }
}

auto camera_psdk::compensation_name(int value) -> const char* {
  switch (value) {
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_5_0:
      return "-5.0ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_4_7:
      return "-4.7ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_4_3:
      return "-4.3ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_4_0:
      return "-4.0ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_3_7:
      return "-3.7ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_3_3:
      return "-3.3ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_3_0:
      return "-3.0ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_2_7:
      return "-2.7ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_2_3:
      return "-2.3ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_2_0:
      return "-2.0ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_1_7:
      return "-1.7ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_1_3:
      return "-1.3ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_1_0:
      return "-1.0ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_0_7:
      return "-0.7ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_0_3:
      return "-0.3ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_0_0:
      return "0.0ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_0_3:
      return "+0.3ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_0_7:
      return "+0.7ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_1_0:
      return "+1.0ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_1_3:
      return "+1.3ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_1_7:
      return "+1.7ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_2_0:
      return "+2.0ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_2_3:
      return "+2.3ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_2_7:
      return "+2.7ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_3_0:
      return "+3.0ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_3_3:
      return "+3.3ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_3_7:
      return "+3.7ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_4_0:
      return "+4.0ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_4_3:
      return "+4.3ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_4_7:
      return "+4.7ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_5_0:
      return "+5.0ev";
    case DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_FIXED:
      return "fixed";
    default:
      spdlog::critical("unknown value: {}", value);
      BOOST_VERIFY(false);
      return "";
  }
}

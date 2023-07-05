#include "camera.h"

#include <dji_liveview.h>  // DjiLiveview_Init
#include <dji_payload_camera.h>  // DjiPayloadCamera_GetCameraHybridZoomFocalLengthOfPayload
#include <spdlog/spdlog.h>

#include <boost/filesystem.hpp>
#include <sstream>  // std::ostringstream
#include <thread>   // std::this_thread

#include "application.hpp"
#include "bounding_box.h"
#include "drone.h"
#include "olyseus_verify.h"  // OLYSEUS_UNREACHABLE

namespace {

std::string camera_file_dst;
std::FILE* camera_file;

}  // namespace

auto camera_data_callback(T_DjiDownloadFilePacketInfo packetInfo,
                          const uint8_t* data, uint16_t len)
    -> T_DjiReturnCode {
  if (packetInfo.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_START) {
    OLYSEUS_VERIFY(!camera_file_dst.empty());
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    camera_file = fopen(camera_file_dst.c_str(), "wb+e");
    camera_file_dst.clear();
  }

  OLYSEUS_VERIFY(camera_file);
  const std::size_t res{fwrite(data, 1, len, camera_file)};
  OLYSEUS_VERIFY(res == len);

  if (packetInfo.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_END) {
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    const int res{fclose(camera_file)};
    OLYSEUS_VERIFY(res == 0);
    camera_file = nullptr;
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

camera::camera(const std::string& model_file, mission& m)
    : inference_(model_file), mission_(m) {
  T_DjiOsalHandler* osal{DjiPlatform_GetOsalHandler()};
  OLYSEUS_VERIFY(osal);

  T_DjiReturnCode code{DjiCameraManager_Init()};
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  E_DjiCameraType camera_type{DJI_CAMERA_TYPE_UNKNOWN};

  // Workaround for failed DjiCameraManager_GetCameraType
  constexpr int wait_ms{500};
  std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));

  constexpr E_DjiMountPosition m_pos{drone::m_pos};
  code = DjiCameraManager_GetCameraType(m_pos, &camera_type);
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
  OLYSEUS_VERIFY(camera_type == DJI_CAMERA_TYPE_H20);

  T_DjiCameraManagerFirmwareVersion firmware_version;

  code = DjiCameraManager_GetFirmwareVersion(m_pos, &firmware_version);
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  // https://developer.dji.com/document/b0776c88-399e-4cd7-8142-681182de3dbd
  // At least: Matrice 300 RTK：V03.00.01.01
  OLYSEUS_VERIFY(firmware_version.firmware_version[0] >= 4);

  code =
      DjiCameraManager_SetMode(m_pos, DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO);
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiCameraManager_SetShootPhotoMode(
      m_pos, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code =
      DjiCameraManager_RegDownloadFileDataCallback(m_pos, camera_data_callback);
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  /* FIXME (remove)
  code = DjiLiveview_Init();
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
  */

  T_DjiCameraManagerFileList media_file_list;
  spdlog::info("Downloading file list");  // FIXME (remove)
  code = DjiCameraManager_DownloadFileList(m_pos, &media_file_list);
  // code =
  // DjiCameraManager_DownloadFileList(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1,
  // &media_file_list); code =
  // DjiCameraManager_DownloadFileList(static_cast<E_DjiMountPosition>(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1),
  // &media_file_list);
  spdlog::info("Downloading file list: DONE");  // FIXME (remove)

  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
  for (int i = 0; i < media_file_list.totalCount; ++i) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    const T_DjiCameraManagerFileListInfo& info{media_file_list.fileListInfo[i]};

    // FIXME (remove)
    constexpr double KB{1024.0};
    constexpr double MB{1024.0 * KB};
    spdlog::info(
        "  Name: {}, index: {}, time:{}-{}-{}_{}:{}:{}, size: {:.2f} MB",
        info.fileName, info.fileIndex, info.createTime.year,
        info.createTime.month, info.createTime.day, info.createTime.hour,
        info.createTime.minute, info.createTime.second, info.fileSize / MB);
    if (!std::regex_match(info.fileName, jpeg_regex)) {
      continue;
    }

    auto res{already_present_indexes_.insert(info.fileIndex)};
    OLYSEUS_VERIFY(res.second);
  }

  spdlog::info("Number of JPEG files on SD card: {}",
               already_present_indexes_.size());
}

camera::~camera() {
  /* FIXME (remove)
  T_DjiReturnCode code{DjiLiveview_Init()};
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
  */

  spdlog::info("Deinit camera");  // FIXME (remove)
  const T_DjiReturnCode code = DjiCameraManager_DeInit();
  spdlog::info("Deinit camera: DONE");  // FIXME (remove)
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
}

void camera::shoot_photo(const gps_coordinates& gps,
                         const attitude& drone_attitude,
                         const attitude& gimbal_attitude,
                         std::size_t waypoint_index) {
  {
    const std::lock_guard lock{queue_mutex_};
    queue_.push_back({gps, drone_attitude, gimbal_attitude, waypoint_index});
    file_waiting_timer_.start();
  }

  {
    const std::lock_guard lock{api_call_mutex_};
    spdlog::info("Shoot photo request");

    constexpr E_DjiMountPosition m_pos{drone::m_pos};

    T_DjiCameraOpticalZoomSpec optical_zoom_spec;
    T_DjiReturnCode code{DjiPayloadCamera_GetCameraOpticalZoomSpecOfPayload(
        m_pos, &optical_zoom_spec)};
    OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    spdlog::debug("focal length min: {}, max: {}, step: {}",
                  optical_zoom_spec.minFocalLength,
                  optical_zoom_spec.maxFocalLength,
                  optical_zoom_spec.focalLengthStep);

    const E_DjiCameraManagerFocusMode expected_focus_mode{
        DJI_CAMERA_MANAGER_FOCUS_MODE_MANUAL};

    // If failed, check camera is ZOOM and the pause is long enough:
    // - https://sdk-forum.dji.net/hc/en-us/requests/73828
    E_DjiCameraManagerFocusMode focus_mode{
        DJI_CAMERA_MANAGER_FOCUS_MODE_UNKNOWN};
    code = DjiCameraManager_GetFocusMode(m_pos, &focus_mode);
    OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    if (focus_mode != expected_focus_mode) {
      spdlog::debug("Changing focus mode to MF");
      code = DjiCameraManager_SetFocusMode(m_pos, expected_focus_mode);
      OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    }

    uint16_t focal_length{0};
    code = DjiPayloadCamera_GetCameraHybridZoomFocalLengthOfPayload(
        m_pos, &focal_length);
    OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    spdlog::debug("Focal length: {}", focal_length);
    OLYSEUS_VERIFY(focal_length == optical_zoom_spec.minFocalLength);

    // FIXME, try:
    // DJI_CAMERA_MANAGER_EXPOSURE_MODE_PROGRAM_AUTO
    // DJI_CAMERA_MANAGER_EXPOSURE_MODE_EXPOSURE_MANUAL
    code = DjiCameraManager_SetExposureMode(
        m_pos, DJI_CAMERA_MANAGER_EXPOSURE_MODE_PROGRAM_AUTO);
    OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    // NOLINTNEXTLINE(readability-simplify-boolean-expr)
    if (false) {  // FIXME (upstream issue)
      code = DjiCameraManager_SetISO(m_pos, DJI_CAMERA_MANAGER_ISO_100);
      OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

      code = DjiCameraManager_SetAperture(
          m_pos, DJI_CAMERA_MANAGER_APERTURE_F_1_DOT_7);
      OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

      code = DjiCameraManager_SetShutterSpeed(
          m_pos, DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_8000);
      OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    }

    spdlog::debug("Call DjiCameraManager_StartShootPhoto");
    code = DjiCameraManager_StartShootPhoto(
        m_pos, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
    OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    spdlog::debug("DjiCameraManager_StartShootPhoto OK");

    // NOLINTNEXTLINE(readability-simplify-boolean-expr)
    if (false) {  // FIXME (upstream issue)
      E_DjiCameraManagerISO iso{DJI_CAMERA_MANAGER_ISO_FIXED};
      code = DjiCameraManager_GetISO(m_pos, &iso);
      OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
      spdlog::info("ISO: {}", iso_name(iso));

      E_DjiCameraManagerAperture aperture{
          DJI_CAMERA_MANAGER_APERTURE_F_UNKNOWN};
      code = DjiCameraManager_GetAperture(m_pos, &aperture);
      OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
      spdlog::info("aperture: {}", aperture_name(aperture));

      E_DjiCameraManagerShutterSpeed shutter_speed{
          DJI_CAMERA_MANAGER_SHUTTER_SPEED_UNKNOWN};
      code = DjiCameraManager_GetShutterSpeed(m_pos, &shutter_speed);
      OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
      spdlog::info("shutter speed: {}", shutter_speed_name(shutter_speed));

      E_DjiCameraManagerExposureCompensation compensation{
          DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_FIXED};
      code = DjiCameraManager_GetExposureCompensation(m_pos, &compensation);
      OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
      spdlog::info("exposure compensation: {}",
                   compensation_name(compensation));
    }
  }
}

// thread: inference
auto camera::check_sdcard(bool debug_launch) -> bool {
  {
    const std::lock_guard lock{queue_mutex_};
    constexpr int64_t wait_ms{20L * 1000L};  // 20 sec
    if (!queue_.empty() && file_waiting_timer_.elapsed_ms() > wait_ms) {
      throw std::runtime_error("Waiting for a file too long. SD Card is full?");
    }
  }

  T_DjiCameraManagerFileList media_file_list;
  {
    const std::lock_guard lock{api_call_mutex_};
    constexpr E_DjiMountPosition m_pos{drone::m_pos};
    spdlog::debug("Downloading file list");  // FIXME (remove)
    const T_DjiReturnCode code{
        DjiCameraManager_DownloadFileList(m_pos, &media_file_list)};
    spdlog::debug("Downloading file list: DONE");  // FIXME (remove)
    OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
  }

  std::vector<std::pair<uint32_t, T_DjiCameraManagerFileCreateTime>>
      inference_files;
  std::set<uint32_t> new_indexes;

  for (int i = 0; i < media_file_list.totalCount; ++i) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    const T_DjiCameraManagerFileListInfo& info{media_file_list.fileListInfo[i]};

    // FIXME (remove)
    constexpr double KB{1024.0};
    constexpr double MB{1024.0 * KB};
    spdlog::debug(
        "  Name: {}, index: {}, time:{}-{}-{}_{}:{}:{}, size: {:.2f} MB",
        info.fileName, info.fileIndex, info.createTime.year,
        info.createTime.month, info.createTime.day, info.createTime.hour,
        info.createTime.minute, info.createTime.second, info.fileSize / MB);

    if (!std::regex_match(info.fileName, jpeg_regex)) {
      continue;
    }
    OLYSEUS_VERIFY(info.type == DJI_CAMERA_FILE_TYPE_JPEG);

    auto res{new_indexes.insert(info.fileIndex)};
    OLYSEUS_VERIFY(res.second);

    if (already_present_indexes_.find(info.fileIndex) ==
        already_present_indexes_.end()) {
      inference_files.emplace_back(info.fileIndex, info.createTime);
    }
  }

  already_present_indexes_ = new_indexes;

  if (inference_files.empty()) {
    // No files to process
    spdlog::debug("No files to process");  // FIXME (remove)
    return false;
  }

  std::sort(
      inference_files.begin(), inference_files.end(),
      [](const auto& a_pair, const auto& b_pair) {
        const T_DjiCameraManagerFileCreateTime& a{a_pair.second};
        const T_DjiCameraManagerFileCreateTime& b{b_pair.second};
        return std::tie(a.year, a.month, a.day, a.hour, a.minute, a.second) <
               std::tie(b.year, b.month, b.day, b.hour, b.minute, b.second);
      });

  spdlog::info("{} files to process", inference_files.size());

  process_inference_files(inference_files, debug_launch);
  return true;
}

auto camera::queue_is_empty() const -> bool {
  const std::lock_guard lock{queue_mutex_};
  return queue_.empty();
}

void camera::process_inference_files(
    const std::vector<std::pair<uint32_t, T_DjiCameraManagerFileCreateTime>>&
        inference_files,
    bool debug_launch) {
  namespace fs = boost::filesystem;

  const fs::path top_dir{"/var/opt/i_seed_drone_onboard"};
  fs::create_directories(top_dir);
  OLYSEUS_VERIFY(fs::is_directory(top_dir));

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
    OLYSEUS_VERIFY(camera_file_dst.empty());
    camera_file_dst = dst_path.string();
    spdlog::info("Download file with index {} to {}", file_index,
                 camera_file_dst);
    constexpr E_DjiMountPosition m_pos{drone::m_pos};
    T_DjiReturnCode code{
        DjiCameraManager_DownloadFileByIndex(m_pos, file_index)};
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      spdlog::critical("Error code: {}", code);
    }
    OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    // global variable should be cleared in callbacks for future use
    OLYSEUS_VERIFY(camera_file_dst.empty());

    // check file is created at this point
    OLYSEUS_VERIFY(fs::is_regular_file(dst_path));

    // The file can be deleted only after a download from sd card
    code = DjiCameraManager_DeleteFileByIndex(m_pos, file_index);
    OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    queue_entry queue_head{};
    {
      const std::lock_guard lock{queue_mutex_};
      OLYSEUS_VERIFY(!queue_.empty());
      queue_head = queue_.front();
    }

    const std::vector<bounding_box> bb{inference_.run(dst_path.string())};
    if (bb.empty()) {
      spdlog::info("No objects found");
    } else {
      detection_result res;
      res.gps = queue_head.gps;
      res.drone_attitude = queue_head.drone_attitude;
      res.gimbal_attitude = queue_head.gimbal_attitude;
      for (const bounding_box& box : bb) {
        constexpr double threshold{0.5};  // 50%
        const bool ignored{box.confidence() < threshold};
        const char* s{ignored ? "(ignored)" : ""};
        spdlog::info("x: {}, y: {}, {:.2f}% {}", box.mid_x(), box.mid_y(),
                     box.confidence() * 100.0, s);
        if (!ignored) {
          res.pixels.push_back({box.mid_x(), box.mid_y()});
        }
      }

      if (debug_launch) {
        const fs::path bbox_path{top_dir / ("bbox_" + file_dst.str())};
        spdlog::info("Save bounding boxes to image: {}", bbox_path.string());
        cv::Mat cv_image{cv::imread(dst_path.string().c_str())};
        OLYSEUS_VERIFY(cv_image.data != nullptr);

        for (const bounding_box& bb : bb) {
          constexpr int thickness{3};
          cv::rectangle(cv_image, bb.pmin(), bb.pmax(), bb.class_color(),
                        thickness);

          cv::Point p_text{bb.pmin()};
          p_text.y -= 12;

          std::ostringstream ss;
          ss << std::fixed << std::setprecision(2) << bb.confidence() * 100.0
             << '%';

          cv::putText(cv_image, ss.str(), p_text, cv::FONT_HERSHEY_SIMPLEX, 2.0,
                      bb.class_color(), thickness);
        }

        const bool ok{cv::imwrite(bbox_path.string(), cv_image)};
        OLYSEUS_VERIFY(ok);
      } else if (!res.pixels.empty()) {
        mission_.save_detection(queue_head.waypoint_index, res);
      }
    }

    // Remove entry only after result saved in mission
    // Backward mission will only start when queue is empty
    {
      const std::lock_guard lock{queue_mutex_};
      OLYSEUS_VERIFY(!queue_.empty());
      queue_.pop_front();
    }
  }
}

auto camera::iso_name(int value) -> const char* {
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
      spdlog::critical("unknown ISO value: {}", value);
      OLYSEUS_UNREACHABLE;
      return "ERROR";
  }
}

auto camera::aperture_name(int value) -> const char* {
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
      spdlog::critical("unknown APERTURE value: {}", value);
      OLYSEUS_UNREACHABLE;
      return "ERROR";
  }
}

auto camera::shutter_speed_name(int value) -> const char* {
  switch (value) {
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_8000:
      return "1/8000 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_6400:
      return "1/6400 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_6000:
      return "1/6000 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_5000:
      return "1/5000 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_4000:
      return "1/4000 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_3200:
      return "1/3200 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_3000:
      return "1/3000 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_2500:
      return "1/2500 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_2000:
      return "1/2000 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_1600:
      return "1/1600 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_1500:
      return "1/1500 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_1250:
      return "1/1250 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_1000:
      return "1/1000 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_800:
      return "1/800 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_725:
      return "1/725 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_640:
      return "1/640 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_500:
      return "1/500 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_400:
      return "1/400 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_350:
      return "1/350 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_320:
      return "1/320 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_250:
      return "1/250 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_240:
      return "1/240 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_200:
      return "1/200 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_180:
      return "1/180 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_160:
      return "1/160 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_125:
      return "1/125 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_120:
      return "1/120 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_100:
      return "1/100 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_90:
      return "1/90 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_80:
      return "1/80 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_60:
      return "1/60 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_50:
      return "1/50 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_40:
      return "1/40 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_30:
      return "1/30 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_25:
      return "1/25 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_20:
      return "1/20 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_15:
      return "1/15 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_12DOT5:
      return "1/12.5 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_10:
      return "1/10 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_8:
      return "1/8 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_6DOT25:
      return "1/6.25 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_5:
      return "1/5 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_4:
      return "1/4 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_3:
      return "1/3 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_2DOT5:
      return "1/2.5 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_2:
      return "1/2 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_1DOT67:
      return "1/1.67 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_1DOT25:
      return "1/1.25 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1:
      return "1.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1DOT3:
      return "1.3 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_1DOT6:
      return "1.6 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_2:
      return "2.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_2DOT5:
      return "2.5 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_3:
      return "3.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_3DOT2:
      return "3.2 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_4:
      return "4.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_5:
      return "5.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_6:
      return "6.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_7:
      return "7.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_8:
      return "8.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_9:
      return "9.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_10:
      return "10.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_13:
      return "13.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_15:
      return "15.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_20:
      return "20.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_25:
      return "25.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_30:
      return "30.0 s";
    case DJI_CAMERA_MANAGER_SHUTTER_SPEED_UNKNOWN:
      return "unknown";
    default:
      spdlog::critical("unknown SHUTTER_SPEED value: {}", value);
      OLYSEUS_UNREACHABLE;
      return "ERROR";
  }
}

auto camera::compensation_name(int value) -> const char* {
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
      spdlog::critical("unknown EXPOSURE_COMPENSATION value: {}", value);
      OLYSEUS_UNREACHABLE;
      return "ERROR";
  }
}

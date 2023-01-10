#include "camera_psdk.h"

#include <boost/assert.hpp>  // BOOST_VERIFY
#include <boost/filesystem.hpp>
#include <dji_camera_manager.h>  // DjiCameraManager_Init
#include <dji_liveview.h>  // DjiLiveview_Init
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
    constexpr auto m_pos{static_cast<E_DjiMountPosition>(mount_position())};
    const T_DjiReturnCode code{
        DjiCameraManager_StartShootPhoto(m_pos,
            DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE)};
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
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

#include <boost/assert.hpp> // BOOST_VERIFY
#include <boost/filesystem.hpp>  // boost::filesystem::path
#include <dji_camera_manager.h>  // DjiCameraManager_Init
#include <iostream>              // std::cerr
#include <spdlog/sinks/rotating_file_sink.h>  // spdlog::sinks::rotating_file_sink_mt
#include <spdlog/sinks/stdout_sinks.h>        // spdlog::sinks::stdout_sink_mt
#include <spdlog/spdlog.h>
#include <spdlog/spdlog.h>

#include "application.hpp" // Application

static std::string file_dst;
static std::FILE* file;

T_DjiReturnCode camera_callback(
    T_DjiDownloadFilePacketInfo packetInfo, const uint8_t *data, uint16_t len) {
  if (packetInfo.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_START) {
    BOOST_VERIFY(!file_dst.empty());
    file = fopen(file_dst.c_str(), "wb+");
    file_dst.clear();
  }

  BOOST_VERIFY(file);
  std::size_t res{fwrite(data, 1, len, file)};
  BOOST_VERIFY(res == len);

  if (packetInfo.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_END) {
    int res{fclose(file)};
    BOOST_VERIFY(res == 0);
    file = nullptr;
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void setup_logging() {
  auto console_sink = std::make_shared<spdlog::sinks::stdout_sink_mt>();
  console_sink->set_level(spdlog::level::info);

  namespace fs = boost::filesystem;

  const fs::path log_path{fs::absolute("i_seed_drone_sdcard_clean.log")};
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

    T_DjiReturnCode code{DjiCameraManager_Init()};
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    constexpr E_DjiMountPosition m_pos{DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1};
    code = DjiCameraManager_RegDownloadFileDataCallback(m_pos,
        camera_callback);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    spdlog::info("Downloading file list");
    T_DjiCameraManagerFileList media_file_list;
    code = DjiCameraManager_DownloadFileList(m_pos, &media_file_list);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    std::vector<uint32_t> to_remove;

    for (int i = 0; i < media_file_list.totalCount; ++i) {
      const T_DjiCameraManagerFileListInfo& info{media_file_list.fileListInfo[i]};

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

      to_remove.push_back(info.fileIndex);
    }

    namespace fs = boost::filesystem;

    fs::path top_dir{"__temp_sdcard_clean"};
    fs::create_directories(top_dir);
    BOOST_VERIFY(fs::is_directory(top_dir));

    for (const uint32_t file_index : to_remove) {
      const fs::path dst_path{top_dir / std::to_string(file_index)};
      BOOST_VERIFY(file_dst.empty());
      file_dst = dst_path.string();
      spdlog::info("Download file with index {} to {}", file_index, file_dst);
      T_DjiReturnCode code{DjiCameraManager_DownloadFileByIndex(m_pos, file_index)};
      if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        spdlog::critical("Error code: {}", code);
      }
      BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

      // global variable should be cleared in callbacks for future use
      BOOST_VERIFY(file_dst.empty());

      // check file is created at this point
      BOOST_VERIFY(fs::is_regular_file(dst_path));

      // The file can be deleted only after a download from sd card
      code = DjiCameraManager_DeleteFileByIndex(m_pos, file_index);
      BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

      fs::remove(dst_path);
      BOOST_VERIFY(!fs::exists(dst_path));
    }

    fs::remove_all(top_dir);
    BOOST_VERIFY(!fs::exists(top_dir));

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

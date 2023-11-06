#include <dji_camera_manager.h>               // DjiCameraManager_Init
#include <spdlog/sinks/rotating_file_sink.h>  // spdlog::sinks::rotating_file_sink_mt
#include <spdlog/sinks/stdout_sinks.h>        // spdlog::sinks::stdout_sink_mt
#include <spdlog/spdlog.h>

#include <boost/filesystem.hpp>  // boost::filesystem::path
#include <iostream>              // std::cerr

#include "application.hpp"   // Application
#include "olyseus_verify.h"  // OLYSEUS_VERIFY

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
    OLYSEUS_VERIFY(argc == 1);
    OLYSEUS_VERIFY(argv != nullptr);
    auto app{std::make_unique<Application>()};

    T_DjiOsalHandler* osal{DjiPlatform_GetOsalHandler()};
    OLYSEUS_VERIFY(osal);

    T_DjiReturnCode code{DjiCameraManager_Init()};
    OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    constexpr E_DjiMountPosition m_pos{DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1};

    spdlog::info("Downloading file list");
    T_DjiCameraManagerFileList media_file_list;
    code = DjiCameraManager_DownloadFileList(m_pos, &media_file_list);
    OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    for (int i = 0; i < media_file_list.totalCount; ++i) {
      const T_DjiCameraManagerFileListInfo& info{
          media_file_list.fileListInfo[i]};

      spdlog::info(
          "  Name: {}, index: {}, time:{}-{}-{}_{}:{}:{}, size: {:.2f} MB",
          info.fileName, info.fileIndex, info.createTime.year,
          info.createTime.month, info.createTime.day, info.createTime.hour,
          info.createTime.minute, info.createTime.second,
          info.fileSize / (1024.0 * 1024.0));
    }

    code = DjiCameraManager_DeInit();
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

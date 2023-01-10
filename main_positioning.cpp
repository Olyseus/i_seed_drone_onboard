#include <boost/assert.hpp>
#include <boost/filesystem.hpp>  // boost::filesystem::path
#include <dji_positioning.h> // DjiPositioning_Init
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

  const boost::filesystem::path log_path{"i_seed_drone_onboard_positioning.log"};
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

T_DjiReturnCode get_newest_pps_callback(uint64_t* local_time_us) {
  BOOST_VERIFY(local_time_us != nullptr);

  /* FIXME (implement?)
  if (s_ppsNewestTriggerLocalTimeMs == 0) {
    USER_LOG_WARN("pps have not been triggered.");
    return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
  }

  *localTimeUs = (uint64_t) (s_ppsNewestTriggerLocalTimeMs * 1000);
  */

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

    // Will not work: https://sdk-forum.dji.net/hc/en-us/requests/75138
    T_DjiReturnCode code{DjiPositioning_Init()};
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    code = DjiTimeSync_Init();
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    constexpr uint8_t task_index{0};
    DjiPositioning_SetTaskIndex(task_index);

    /* FIXME (enable?)
    code = DjiTimeSync_RegGetNewestPpsTriggerTimeCallback(get_newest_pps_callback);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    */

    T_DjiTimeSyncAircraftTime aircraft_time;

    uint32_t current_time_ms{0};
    code = osal->GetTimeMs(&current_time_ms);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    // FIXME uint64_t pps_newest_trigger_time_us{0};
    // FIXME constexpr uint64_t dji_test_time_interval_among_events_us{200000};
    code = DjiTimeSync_TransferToAircraftTime(current_time_ms * 1000,
        // FIXME (???) pps_newest_trigger_time_us - 1000000 - dji_test_time_interval_among_events_us,
        &aircraft_time);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    T_DjiPositioningEventInfo event_info;
    event_info.eventSetIndex = 0;
    event_info.targetPointIndex = 0;
    event_info.eventTime = aircraft_time;

    T_DjiPositioningPositionInfo position_info;
    code = DjiPositioning_GetPositionInformationSync(1, &event_info, &position_info);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    spdlog::info("position solution property: {}", position_info.positionSolutionProperty);
    spdlog::info("pitchAttitudeAngle: {}, rollAttitudeAngle: {}, yawAttitudeAngle: {}",
        position_info.uavAttitude.pitch, position_info.uavAttitude.roll, position_info.uavAttitude.yaw);
    spdlog::info("northPositionOffset: {}, earthPositionOffset: {}, downPositionOffset: {}",
        position_info.offsetBetweenMainAntennaAndTargetPoint.x,
        position_info.offsetBetweenMainAntennaAndTargetPoint.y,
        position_info.offsetBetweenMainAntennaAndTargetPoint.z);
    spdlog::info("longitude: {}, latitude: {}, height: {}",
        position_info.targetPointPosition.longitude,
        position_info.targetPointPosition.latitude,
        position_info.targetPointPosition.height);
    spdlog::info("longStandardDeviation: {}, latStandardDeviation: {}, hgtStandardDeviation: {}",
        position_info.targetPointPositionStandardDeviation.longitude,
        position_info.targetPointPositionStandardDeviation.latitude,
        position_info.targetPointPositionStandardDeviation.height);

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

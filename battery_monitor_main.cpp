#include <boost/filesystem.hpp>   // boost::filesystem::path
#include <dji_fc_subscription.h> // DjiFcSubscription_Init
#include <spdlog/sinks/rotating_file_sink.h>  // spdlog::sinks::rotating_file_sink_mt
#include <spdlog/sinks/stdout_sinks.h>        // spdlog::sinks::stdout_sink_mt
#include <spdlog/spdlog.h>
#include <thread> // std::this_thread

#include "application.hpp" // Application

void battery_info(const uint8_t* data, int index) {
  BOOST_VERIFY(data != nullptr);
  const auto info{*(const T_DjiFcSubscriptionSingleBatteryInfo*)data};

  spdlog::info("Battery {}: {}%, {}V, {:.1f}C degrees", index,
               info.batteryCapacityPercent, info.currentVoltage / 1000,
               info.batteryTemperature / 10.0);
}

T_DjiReturnCode battery_callback_1(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp* timestamp) {
  (void)data_size;
  (void)timestamp;

  battery_info(data, 1);

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode battery_callback_2(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp* timestamp) {
  (void)data_size;
  (void)timestamp;

  battery_info(data, 2);

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void battery_monitor(int argc, char** argv) {
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
      DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1,
      DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
      battery_callback_1);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiFcSubscription_SubscribeTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX2,
      DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
      battery_callback_2);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  while (true) {
    // Keep receiving callbacks
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  code = DjiFcSubscription_DeInit();
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
}

void setup_logging() {
  auto console_sink = std::make_shared<spdlog::sinks::stdout_sink_mt>();
  console_sink->set_level(spdlog::level::info);

  const boost::filesystem::path log_path{"battery_monitor.log"};
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

auto run_main(int argc, char** argv) -> int {
  setup_logging();

  try {
    battery_monitor(argc, argv);
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

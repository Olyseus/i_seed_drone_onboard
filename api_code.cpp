#include "api_code.h"

#include <spdlog/spdlog.h>

#include <boost/assert.hpp>  // BOOST_VERIFY
#include <thread>            // std::this_thread

#include "server.h"  // pipeline_closed

api_code::api_code(const T_DjiReturnCode code) {
  switch (static_cast<DjiErrorCode>(code)) {
    case DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS:
      code_ = code::success;
      break;
    case DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT:
      // common read error
      spdlog::debug("DjiErrorCode: timeout");
      make_retry();
      break;
    case DJI_ERROR_SYSTEM_MODULE_CODE_BUSY:
      spdlog::error("DjiErrorCode: busy");
      make_retry();
      break;
    case DJI_ERROR_MOP_CHANNEL_MODULE_CODE_CONNECTION_CLOSE:
      spdlog::error("DjiErrorCode: connection close");
      throw pipeline_closed();
    case DJI_ERROR_SYSTEM_MODULE_CODE_MEMORY_ALLOC_FAILED:
      spdlog::error("DjiErrorCode: memory allocation failed");
      throw pipeline_closed();
    default:
      spdlog::error("Invalid DjiErrorCode: {}", code);
      break;
  }
}

void api_code::make_retry() {
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  code_ = code::retry;
}

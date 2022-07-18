#include "api_code.h"

#include <spdlog/spdlog.h>

#include <boost/assert.hpp>       // BOOST_VERIFY
#include <dji_linux_helpers.hpp>  // ACK::ErrorCode
#include <thread>                 // std::this_thread

#include "server.h"  // pipeline_closed

template <>
api_code::api_code(const ACK::ErrorCode& error_code) {
  if (ACK::getError(error_code) == ACK::SUCCESS) {
    code_ = code::success;
    return;
  }

  // Note: retry will not help
  // error_code.info.cmd_set == OpenProtocolCMD::CMDSet::subscribe
  // error_code.data ==
  // OpenProtocolCMD::ErrorCode::SubscribeACK::VERSION_DOES_NOT_MATCH

  BOOST_VERIFY(ACK::getError(error_code) == ACK::FAIL);
  ACK::getErrorCodeMessage(error_code, __func__);
}

template <>
api_code::api_code(const ErrorCode::ErrorCodeType& error_code) {
  if (error_code == ErrorCode::SysCommonErr::Success) {
    code_ = code::success;
    return;
  }
  ErrorCode::printErrorCodeMsg(error_code);
}

// https://github.com/dji-sdk/Onboard-SDK/blob/4.1.0/osdk-core/modules/inc/mop/dji_mop_define.hpp#L49
template <>
api_code::api_code(const DJI::OSDK::MOP::MopErrCode& error_code) {
  switch (error_code) {
    case MOP_PASSED:
      code_ = code::success;
      return;
    case MOP_FAILED:
      spdlog::error("MOP_FAILED");
      return;
    case MOP_CRC:
      spdlog::error("MOP_CRC");
      return;
    case MOP_PARM:
      spdlog::error("MOP_PARM");
      return;
    case MOP_NOMEM:
      spdlog::error("MOP_NOMEM");
      throw pipeline_closed();
    case MOP_NOTREADY:
      spdlog::error("MOP_NOTREADY");
      make_retry();
      return;
    case MOP_SEND:
      spdlog::error("MOP_SEND");
      return;
    case MOP_RECV:
      spdlog::error("MOP_RECV");
      return;
    case MOP_TIMEOUT:
      spdlog::error("MOP_TIMEOUT");
      make_retry();
      return;
    case MOP_RESBUSY:
      spdlog::error("MOP_RESBUSY");
      make_retry();
      return;
    case MOP_RESOCCUPIED:
      spdlog::error("MOP_RESOCCUPIED");
      make_retry();
      return;
    case MOP_CONNECTIONCLOSE:
      spdlog::error("MOP_CONNECTIONCLOSE");
      throw pipeline_closed();
    case MOP_NOTIMPLEMENT:
      spdlog::error("MOP_NOTIMPLEMENT");
      return;
    case MOP_UNKNOWN_ERR:
      spdlog::error("MOP_UNKNOWN_ERR");
      return;
    default:
      spdlog::error("Invalid MopErrCode: {}", error_code);
      return;
  }
}

void api_code::make_retry() {
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  code_ = code::retry;
}

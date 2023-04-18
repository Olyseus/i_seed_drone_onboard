#include "server.h"

#include <dji_mop_channel.h>  // DjiMopChannel_Create
#include <spdlog/spdlog.h>

#include <boost/assert.hpp>  // BOOST_VERIFY

server::server(uint16_t channel_id) {
  spdlog::info("Creating channel {}", channel_id);

  T_DjiMopChannelHandle channel_handle{nullptr};

  T_DjiReturnCode code{
      DjiMopChannel_Create(&channel_handle, DJI_MOP_CHANNEL_TRANS_RELIABLE)};
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiMopChannel_Bind(channel_handle, channel_id);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  BOOST_VERIFY(out_channel_handle_ == nullptr);
#else
  code = DjiMopChannel_Accept(channel_handle, &out_channel_handle_);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
  BOOST_VERIFY(out_channel_handle_ != nullptr);
#endif
}

server::~server() {
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  BOOST_VERIFY(out_channel_handle_ == nullptr);
#else
  spdlog::info("Close channel");

  T_DjiReturnCode code{DjiMopChannel_Close(out_channel_handle_)};
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiMopChannel_Destroy(out_channel_handle_);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
#endif
}

auto server::handle() const -> T_DjiMopChannelHandle {
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  BOOST_VERIFY(out_channel_handle_ == nullptr);
#else
  BOOST_VERIFY(out_channel_handle_ != nullptr);
#endif
  return out_channel_handle_;
}

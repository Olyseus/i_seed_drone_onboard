#include "server.h"

#include <dji_mop_channel.h>  // DjiMopChannel_Create
#include <spdlog/spdlog.h>

#include "olyseus_verify.h"  // OLYSEUS_VERIFY

server::server(uint16_t channel_id) {
  spdlog::info("Creating channel {}", channel_id);
  T_DjiReturnCode code{
      DjiMopChannel_Create(&channel_handle_, DJI_MOP_CHANNEL_TRANS_RELIABLE)};
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
  OLYSEUS_VERIFY(channel_handle_ != nullptr);

  spdlog::info("Binding channel {}", channel_id);
  static_assert(258 == DJI_ERROR_SYSTEM_MODULE_CODE_BUSY);
  code = DjiMopChannel_Bind(channel_handle_, channel_id);
  OLYSEUS_VERIFY(code != DJI_ERROR_SYSTEM_MODULE_CODE_BUSY); // retrying will not help
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
}

server::~server() {
#if defined(I_SEED_DRONE_ONBOARD_INTERCONNECTION)
  spdlog::info("Close channel");
  T_DjiReturnCode code{DjiMopChannel_Close(channel_handle_)};
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  spdlog::info("Destroy channel");
  code = DjiMopChannel_Destroy(channel_handle_);
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
#endif
}

auto server::handle() const -> T_DjiMopChannelHandle {
#if defined(I_SEED_DRONE_ONBOARD_INTERCONNECTION)
  OLYSEUS_VERIFY(channel_handle_ != nullptr);
#else
  OLYSEUS_VERIFY(channel_handle_ == nullptr);
#endif
  return channel_handle_;
}

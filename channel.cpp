#include "channel.h"

#include <dji_mop_channel.h>  // DjiMopChannel_Accept
#include <spdlog/spdlog.h>

#include "olyseus_verify.h"  // OLYSEUS_VERIFY
#include "server.h"

channel::channel(const server& s) {
#if defined(I_SEED_DRONE_ONBOARD_INTERCONNECTION)
  // That's where we will wait for connection from other side
  spdlog::info("Accept channel (create out)");
  const T_DjiReturnCode code{
      DjiMopChannel_Accept(s.handle(), &out_channel_handle_)};
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
  OLYSEUS_VERIFY(out_channel_handle_ != nullptr);
#else
  OLYSEUS_VERIFY(out_channel_handle_ == nullptr);
#endif
}

channel::~channel() {
#if defined(I_SEED_DRONE_ONBOARD_INTERCONNECTION)
  spdlog::info("Close channel (close out)");
  T_DjiReturnCode code{DjiMopChannel_Close(out_channel_handle_)};
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  spdlog::info("Destroy channel (destroy out)");
  code = DjiMopChannel_Destroy(out_channel_handle_);
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
#else
  OLYSEUS_VERIFY(out_channel_handle_ == nullptr);
#endif
}

auto channel::out_handle() const -> T_DjiMopChannelHandle {
#if defined(I_SEED_DRONE_ONBOARD_INTERCONNECTION)
  OLYSEUS_VERIFY(out_channel_handle_ != nullptr);
#else
  OLYSEUS_VERIFY(out_channel_handle_ == nullptr);
#endif
  return out_channel_handle_;
}

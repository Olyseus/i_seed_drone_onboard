#include "server.h"

#include <spdlog/spdlog.h>

#include <boost/assert.hpp>       // BOOST_VERIFY
#include <dji_linux_helpers.hpp>  // MopServer

#include "api_code.h"

server::server(uint16_t channel_id) : channel_id_{channel_id} {
  mop_server_.reset(new MopServer());
  spdlog::info("Creating channel {}", channel_id_);
  const api_code code{
      mop_server_->accept(channel_id_, MOP::PipelineType::RELIABLE, pipeline_)};
  BOOST_VERIFY(code.success());
  BOOST_VERIFY(pipeline_ != nullptr);
}

server::~server() {
  spdlog::info("Disconnect channel");
  const api_code code{mop_server_->close(channel_id_)};
  BOOST_VERIFY(code.success());
}

#ifndef SERVER_H_
#define SERVER_H_

#include <spdlog/spdlog.h>

#include <dji_linux_helpers.hpp>  // MopServer

#include "api_code.h"

class server {
 public:
  server(uint16_t channel_id) {
    mop_server_.reset(new MopServer());
    spdlog::info("Creating channel {}", channel_id);
    const api_code code{mop_server_->accept(
        channel_id, MOP::PipelineType::RELIABLE, pipeline_)};
    BOOST_VERIFY(code.success());
    BOOST_VERIFY(pipeline_ != nullptr);
  }

  ~server() {
    spdlog::info("Disconnect channel");
    const api_code code{mop_server_->close(channel_id)};
    BOOST_VERIFY(code.success());
  }

  server(const server&) = delete;
  server(server&&) = delete;

  server& operator=(const server&) = delete;
  server& operator=(server&&) = delete;

  DJI::OSDK::MopPipeline* pipeline() { return pipeline_; }

 private:
  std::unique_ptr<MopServer> mop_server_;
  DJI::OSDK::MopPipeline* pipeline_{nullptr};
};

#endif  // SERVER_H_

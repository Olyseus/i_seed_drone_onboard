#ifndef SERVER_H_
#define SERVER_H_

#include <memory>  // std::unique_ptr

namespace DJI {
namespace OSDK {
class MopServer;
class MopPipeline;
}  // namespace OSDK
}  // namespace DJI

class pipeline_closed {};

class server {
 public:
  server(uint16_t channel_id);
  ~server();

  server(const server&) = delete;
  server(server&&) = delete;

  server& operator=(const server&) = delete;
  server& operator=(server&&) = delete;

  DJI::OSDK::MopPipeline* pipeline() { return pipeline_; }

 private:
  const uint16_t channel_id_{0};
  std::unique_ptr<DJI::OSDK::MopServer> mop_server_;
  DJI::OSDK::MopPipeline* pipeline_{nullptr};
};

#endif  // SERVER_H_

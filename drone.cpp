#include "drone.h"

#include <future>

#include <boost/assert.hpp>
#include <spdlog/spdlog.h>

#include <dji_linux_helpers.hpp>  // LinuxSetup

drone::drone(int argc, char** argv) {
  spdlog::info("Setup for Linux");
  constexpr bool enable_advanced_sensing{true};
  linux_setup_.reset(new LinuxSetup(argc, argv, enable_advanced_sensing));
  spdlog::info("Setup finished");

  interconnection::command_type command;
  command.set_type(interconnection::command_type::PING);
  command.set_version(protocol_version);
  std::string buffer;
  const bool ok{command.SerializeToString(&buffer)};
  BOOST_VERIFY(ok);
  command_bytes_size_ = {static_cast<uint32_t>(buffer.size())};
  BOOST_VERIFY(command_bytes_size_ > 0);

  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

drone::~drone() = default;

void drone::start() {
  spdlog::info("Protocol version: {}", protocol_version);
  spdlog::info("Command bytes size: {}", command_bytes_size_);

  while (true) {
    connection_closed_ = false;

    std::unique_ptr<MopServer> server{new MopServer()};

    spdlog::info("Creating channel {}", channel_id);
    MopErrCode error{server->accept(channel_id, MOP::PipelineType::RELIABLE, pipeline_)};
    spdlog::info("Accept code: {}", error);
    BOOST_VERIFY(error == MOP_PASSED);
    BOOST_VERIFY(pipeline_ != nullptr);

    std::future<void> read_future{std::async(std::launch::async, &drone::read_job, this)};
    std::future<void> write_future{std::async(std::launch::async, &drone::write_job, this)};

    read_future.get();
    write_future.get();

    error = server->close(channel_id);
    BOOST_VERIFY(error == MOP_PASSED);
  }
}

void drone::read_job() {
  spdlog::info("Read job started");

  std::string buffer;
  buffer.resize(command_bytes_size_);

  while (true) {
    if (connection_closed_) {
      return;
    }
    BOOST_VERIFY(command_bytes_size_ == buffer.size());
    char* char_buffer{buffer.data()};
    static_assert(sizeof(char) == sizeof(uint8_t));
    uint8_t* recv_buf{reinterpret_cast<uint8_t*>(char_buffer)};
    MopPipeline::DataPackType read_pack = {recv_buf, command_bytes_size_};
    uint32_t len{0};
    MopErrCode result{pipeline_->recvData(read_pack, &len)};
    if (result == MOP_TIMEOUT) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }
    if (result == MOP_CONNECTIONCLOSE) {
      connection_closed_ = true;
      return;
    }
    spdlog::info("Read data code: {}", result);
    BOOST_VERIFY(result == MOP_PASSED);
    BOOST_VERIFY(len == command_bytes_size_);

    interconnection::command_type command;
    const bool ok{command.ParseFromString(buffer)};
    BOOST_VERIFY(ok);

    switch (command.type()) {
    case interconnection::command_type::PING:
      {
        std::lock_guard<std::mutex> lock(m_);
        execute_commands_.push_back(command.type());
      }
      break;
    default:
      BOOST_VERIFY(false);
    }
  }
}

void drone::write_job() {
  spdlog::info("Write job started");

  std::optional<interconnection::command_type::command_t> command;

  while (true) {
    if (connection_closed_) {
      return;
    }
    command.reset();

    {
      std::lock_guard<std::mutex> lock(m_);
      if (!execute_commands_.empty()) {
        command = execute_commands_.front();
      }
    }

    if (!command.has_value()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }

    switch (command.value()) {
      case interconnection::command_type::PING: {
        spdlog::info("Execute PING command");
        if (!send_command(interconnection::command_type::PING)) {
          continue;
        }
        break;
      }
      default:
        BOOST_VERIFY(false);
    }

    {
      std::lock_guard<std::mutex> lock(m_);
      execute_commands_.pop_front();
    }
  }
}

bool drone::send_command(interconnection::command_type::command_t command_type) {
  interconnection::command_type command;
  command.set_type(command_type);
  command.set_version(protocol_version);
  std::string buffer;
  const bool ok{command.SerializeToString(&buffer)};
  BOOST_VERIFY(ok);
  BOOST_VERIFY(buffer.size() == command_bytes_size_);

  char* char_buf{buffer.data()};
  static_assert(sizeof(char) == sizeof(uint8_t));
  uint8_t* send_buf{reinterpret_cast<uint8_t*>(char_buf)};
  MopPipeline::DataPackType req_pack = {send_buf, static_cast<uint32_t>(buffer.size())};
  uint32_t len{0};
  MopErrCode result = pipeline_->sendData(req_pack, &len);
  spdlog::info("Write data code: {} (size: {})", result, len);
  if (result == MOP_TIMEOUT) {
    spdlog::info("Write connection timeout (command)");
    return false;
  }
  if (result == MOP_CONNECTIONCLOSE) {
    connection_closed_ = true;
    spdlog::info("Write connection closed (command)");
    return false;
  }
  BOOST_VERIFY(result == MOP_PASSED);
  BOOST_VERIFY(len == buffer.size());
  return true;
}

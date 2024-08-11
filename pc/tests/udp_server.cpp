#include "espp.hpp"

using namespace std::chrono_literals;

static auto start = std::chrono::high_resolution_clock::now();

int main() {
  static auto server = espp::UdpSocket({.log_level = espp::Logger::Verbosity::DEBUG});
  auto server_task_config = espp::Task::Config{
      .name = "UdpServer",
      .callback = nullptr,
      .stack_size_bytes = 6 * 1024,
  };

  auto server_config = espp::UdpSocket::ReceiveConfig{
      .port = 5555, .buffer_size = 1024, .on_receive_callback = [
      ](auto &data, auto &source) -> auto{auto now = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
  fmt::print("Received {} bytes from {}:{}\n", data.size(), source.address, source.port);
  auto message =
      fmt::format("Received {} bytes from {}:{}\n", data.size(), source.address, source.port);
  // convert the message into a vector of bytes
  std::vector<uint8_t> message_bytes(message.begin(), message.end());
  // send the message back to the client
  return message_bytes;
}
}
;
server.start_receiving(server_task_config, server_config);

std::this_thread::sleep_for(10s);

return 0;
}

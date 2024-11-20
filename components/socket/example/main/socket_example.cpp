#include <algorithm>
#include <chrono>
#include <functional>
#include <iterator>
#include <thread>

#if CONFIG_ESP32_WIFI_NVS_ENABLED
#include "nvs_flash.h"
#endif

#include "logger.hpp"
#include "task.hpp"
#include "tcp_socket.hpp"
#include "udp_socket.hpp"
#include "wifi_ap.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

extern "C" void app_main(void) {
  fmt::print("Stating socket example!\n");

  auto test_duration = 3s;

#if CONFIG_ESP32_WIFI_NVS_ENABLED
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
#endif

  // create a wifi access point here so that LwIP will be init for this example
  espp::WifiAp wifi_ap({.ssid = "SocketExample",
                        .password = "", // no security
                        .log_level = espp::Logger::Verbosity::INFO});

  fmt::print(fg(fmt::terminal_color::yellow) | fmt::emphasis::bold, "Staring Basic UDP test.\n");

  // Unicast (client-server) example
  {
    //! [UDP Server example]
    std::string server_address = "127.0.0.1";
    size_t port = 5000;
    espp::UdpSocket server_socket({.log_level = espp::Logger::Verbosity::WARN});
    auto server_task_config = espp::Task::BaseConfig{
        .name = "UdpServer",
        .stack_size_bytes = 6 * 1024,
    };
    auto server_config = espp::UdpSocket::ReceiveConfig{
        .port = port,
        .buffer_size = 1024,
        .on_receive_callback =
            [](auto &data, auto &source) -> auto{fmt::print("Server received: {}\n"
                                                            "    from source: {}\n",
                                                            data, source);
    return std::nullopt;
  }
};
server_socket.start_receiving(server_task_config, server_config);
//! [UDP Server example]

//! [UDP Client example]
espp::UdpSocket client_socket({});
// create thread for sending data using the socket
auto client_task_fn = [&server_address, &client_socket, &port](auto &, auto &) {
  static size_t iterations = 0;
  std::vector<uint8_t> data{0, 1, 2, 3, 4};
  std::transform(data.begin(), data.end(), data.begin(),
                 [](const auto &d) { return d + iterations; });
  auto send_config = espp::UdpSocket::SendConfig{.ip_address = server_address, .port = port};
  client_socket.send(data, send_config);
  iterations++;
  std::this_thread::sleep_for(1s);
  // don't want to stop the task
  return false;
};
auto client_task =
    espp::Task::make_unique({.callback = client_task_fn,
                             .task_config = {.name = "Client Task", .stack_size_bytes = 5 * 1024}});
client_task->start();
//! [UDP Client example]
// now sleep for a while to let the monitor do its thing
std::this_thread::sleep_for(test_duration);
}

fmt::print(fg(fmt::terminal_color::green) | fmt::emphasis::bold, "Basic UDP test finished.\n");
std::this_thread::sleep_for(100ms);
fmt::print(fg(fmt::terminal_color::yellow) | fmt::emphasis::bold,
           "Staring UDP send waiting for response test.\n");

// Unicast (client-server) example with server response
{
  //! [UDP Server Response example]
  std::string server_address = "127.0.0.1";
  size_t port = 5000;
  espp::UdpSocket server_socket({.log_level = espp::Logger::Verbosity::WARN});
  auto server_task_config =
      espp::Task::BaseConfig{.name = "UdpServer", .stack_size_bytes = 6 * 1024};
  auto server_config = espp::UdpSocket::ReceiveConfig{
      .port = port,
      .buffer_size = 1024,
      .on_receive_callback =
          [](auto &data, auto &source) -> auto{fmt::print("Server received: {}\n"
                                                          "    from source: {}\n",
                                                          data, source);
  // reverse the data
  std::reverse(data.begin(), data.end());
  // and send it back
  return data;
}
}
;
server_socket.start_receiving(server_task_config, server_config);
//! [UDP Server Response example]

//! [UDP Client Response example]
espp::UdpSocket client_socket({.log_level = espp::Logger::Verbosity::WARN});
// create threads
auto client_task_fn = [&server_address, &client_socket, &port](auto &, auto &) {
  static size_t iterations = 0;
  std::vector<uint8_t> data{0, 1, 2, 3, 4};
  std::transform(data.begin(), data.end(), data.begin(),
                 [](const auto &d) { return d + iterations; });
  auto send_config = espp::UdpSocket::SendConfig{
      .ip_address = server_address,
      .port = port,
      .wait_for_response = true,
      .response_size = 128,
      .on_response_callback = [](auto &response) { fmt::print("Client received: {}\n", response); },
  };
  // NOTE: now this call blocks until the response is received
  client_socket.send(data, send_config);
  iterations++;
  std::this_thread::sleep_for(1s);
  // don't want to stop the task
  return false;
};
auto client_task =
    espp::Task::make_unique({.callback = client_task_fn,
                             .task_config = {.name = "Client Task", .stack_size_bytes = 5 * 1024}});
client_task->start();
//! [UDP Client Response example]
// now sleep for a while to let the monitor do its thing
std::this_thread::sleep_for(test_duration);
}

fmt::print(fg(fmt::terminal_color::green) | fmt::emphasis::bold,
           "UDP send waiting for response test finished.\n");
std::this_thread::sleep_for(100ms);
fmt::print(fg(fmt::terminal_color::yellow) | fmt::emphasis::bold, "Staring UDP multicast test.\n");

// Multicast example
{
  //! [UDP Multicast Server example]
  std::string multicast_group = "239.1.1.1";
  size_t port = 5000;
  espp::UdpSocket server_socket({.log_level = espp::Logger::Verbosity::WARN});
  auto server_task_config = espp::Task::BaseConfig{
      .name = "UdpServer",
      .stack_size_bytes = 6 * 1024,
  };
  auto server_config = espp::UdpSocket::ReceiveConfig{
      .port = port,
      .buffer_size = 1024,
      .is_multicast_endpoint = true,
      .multicast_group = multicast_group,
      .on_receive_callback =
          [](auto &data, auto &source) -> auto{fmt::print("Server received: {}\n"
                                                          "    from source: {}\n",
                                                          data, source);
  // reverse the data
  std::reverse(data.begin(), data.end());
  // and send it back
  return data;
}
}
;
server_socket.start_receiving(server_task_config, server_config);
//! [UDP Multicast Server example]

//! [UDP Multicast Client example]
espp::UdpSocket client_socket({});
// create threads
auto client_task_fn = [&client_socket, &port, &multicast_group](auto &, auto &) {
  static size_t iterations = 0;
  std::vector<uint8_t> data{0, 1, 2, 3, 4};
  std::transform(data.begin(), data.end(), data.begin(),
                 [](const auto &d) { return d + iterations; });
  auto send_config = espp::UdpSocket::SendConfig{.ip_address = multicast_group,
                                                 .port = port,
                                                 .is_multicast_endpoint = true,
                                                 .wait_for_response = true,
                                                 .response_size = 128,
                                                 .on_response_callback = [](auto &response) {
                                                   fmt::print("Client received: {}\n", response);
                                                 }};
  // NOTE: now this call blocks until the response is received
  client_socket.send(data, send_config);
  iterations++;
  std::this_thread::sleep_for(1s);
  // don't want to stop the task
  return false;
};
auto client_task =
    espp::Task::make_unique({.callback = client_task_fn,
                             .task_config = {.name = "Client Task", .stack_size_bytes = 5 * 1024}});
client_task->start();
//! [UDP Multicast Client example]
// now sleep for a while to let the monitor do its thing
std::this_thread::sleep_for(test_duration);
}

fmt::print(fg(fmt::terminal_color::green) | fmt::emphasis::bold, "UDP multicast test finished.\n");
std::this_thread::sleep_for(100ms);
fmt::print(fg(fmt::terminal_color::yellow) | fmt::emphasis::bold, "Staring Basic TCP test.\n");

// Unicast (client-server) example
{
  //! [TCP Server example]
  std::string server_address = "127.0.0.1";
  size_t port = 5000;
  int max_connections = 1;
  espp::TcpSocket server_socket({.log_level = espp::Logger::Verbosity::WARN});
  server_socket.bind(port);
  server_socket.listen(max_connections);
  auto server_task_fn = [&server_socket](auto &m, auto &cv) -> bool {
    static std::unique_ptr<espp::TcpSocket> client_socket;
    if (!client_socket) {
      client_socket = server_socket.accept();
      if (client_socket) {
        auto info = client_socket->get_remote_info();
        fmt::print("Server accepted connection from: {}\n", info);
      }
    }
    if (client_socket) {
      std::vector<uint8_t> data;
      size_t max_receive_size = 1024;
      if (client_socket->receive(data, max_receive_size)) {
        fmt::print("Server received: {}\n", data);
      }
    }
    {
      std::unique_lock<std::mutex> lock(m);
      cv.wait_for(lock, 10ms);
    }
    // don't want to stop the task
    return false;
  };

  auto server_task_config = espp::Task::Config{
      .callback = server_task_fn,
      .task_config =
          {
              .name = "TcpServer",
              .stack_size_bytes = 6 * 1024,
          },
  };
  auto server_task = espp::Task::make_unique(server_task_config);
  server_task->start();
  //! [TCP Server example]

  //! [TCP Client example]
  espp::TcpSocket client_socket({});
  client_socket.connect({.ip_address = server_address, .port = port});
  // create thread for sending data using the socket
  auto client_task_fn = [&server_address, &client_socket, &port](auto &, auto &) {
    static size_t iterations = 0;
    std::vector<uint8_t> data{0, 1, 2, 3, 4};
    std::transform(data.begin(), data.end(), data.begin(),
                   [](const auto &d) { return d + iterations; });
    client_socket.transmit(data);
    iterations++;
    std::this_thread::sleep_for(1s);
    // don't want to stop the task
    return false;
  };
  auto client_task = espp::Task::make_unique(
      {.callback = client_task_fn,
       .task_config = {.name = "Client Task", .stack_size_bytes = 5 * 1024}});
  client_task->start();
  //! [TCP Client example]
  // now sleep for a while to let the monitor do its thing
  std::this_thread::sleep_for(test_duration);
}

fmt::print(fg(fmt::terminal_color::green) | fmt::emphasis::bold, "Basic TCP test finished.\n");
std::this_thread::sleep_for(100ms);
fmt::print(fg(fmt::terminal_color::yellow) | fmt::emphasis::bold,
           "Staring TCP send waiting for response test.\n");

// Unicast (client-server) example with server response
{
  //! [TCP Server Response example]
  std::string server_address = "127.0.0.1";
  size_t port = 5000;
  int max_connections = 1;
  espp::TcpSocket server_socket({.log_level = espp::Logger::Verbosity::WARN});
  server_socket.bind(port);
  server_socket.listen(max_connections);
  auto server_task_fn = [&server_socket](auto &m, auto &cv) -> bool {
    static std::unique_ptr<espp::TcpSocket> client_socket;
    if (!client_socket) {
      client_socket = server_socket.accept();
      if (client_socket) {
        auto info = client_socket->get_remote_info();
        fmt::print("Server accepted connection from: {}\n", info);
      }
    }
    if (client_socket) {
      std::vector<uint8_t> data;
      size_t max_receive_size = 1024;
      if (client_socket->receive(data, max_receive_size)) {
        fmt::print("Server received: {}\n", data);
        // reverse the data
        std::reverse(data.begin(), data.end());
        // and send it back
        client_socket->transmit(data);
      }
    }
    {
      std::unique_lock<std::mutex> lock(m);
      cv.wait_for(lock, 10ms);
    }
    // don't want to stop the task
    return false;
  };

  auto server_task_config = espp::Task::Config{
      .callback = server_task_fn,
      .task_config =
          {
              .name = "TcpServer",
              .stack_size_bytes = 6 * 1024,
          },
  };
  auto server_task = espp::Task::make_unique(server_task_config);
  server_task->start();
  //! [TCP Server Response example]

  //! [TCP Client Response example]
  espp::TcpSocket client_socket({.log_level = espp::Logger::Verbosity::WARN});
  client_socket.connect({
      .ip_address = server_address,
      .port = port,
  });
  // create threads
  auto client_task_fn = [&server_address, &client_socket, &port](auto &, auto &) {
    static size_t iterations = 0;
    std::vector<uint8_t> data{0, 1, 2, 3, 4};
    std::transform(data.begin(), data.end(), data.begin(),
                   [](const auto &d) { return d + iterations; });
    auto transmit_config = espp::TcpSocket::TransmitConfig{
        .wait_for_response = true,
        .response_size = 128,
        .on_response_callback =
            [](auto &response) { fmt::print("Client received: {}\n", response); },
    };
    // NOTE: now this call blocks until the response is received
    client_socket.transmit(data, transmit_config);
    iterations++;
    std::this_thread::sleep_for(1s);
    // don't want to stop the task
    return false;
  };
  auto client_task = espp::Task::make_unique(
      {.callback = client_task_fn,
       .task_config = {.name = "Client Task", .stack_size_bytes = 5 * 1024}});
  client_task->start();
  //! [TCP Client Response example]
  // now sleep for a while to let the monitor do its thing
  std::this_thread::sleep_for(test_duration);
}

fmt::print(fg(fmt::terminal_color::green) | fmt::emphasis::bold,
           "TCP send waiting for response test finished.\n");
std::this_thread::sleep_for(100ms);
fmt::print(fg(fmt::terminal_color::green) | fmt::emphasis::bold, "Socket example finished!\n");

// sleep forever
while (true) {
  std::this_thread::sleep_for(1s);
}
}

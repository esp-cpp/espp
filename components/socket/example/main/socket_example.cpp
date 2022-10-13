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
  espp::WifiAp wifi_ap({
      .ssid = "SocketExample",
      .password = "", // no security
      .log_level = espp::Logger::Verbosity::INFO
    });

  fmt::print(fg(fmt::terminal_color::yellow) | fmt::emphasis::bold, "Staring Basic UDP test.\n");

  // Unicast (client-server) example
  {
    //! [UDP Server example]
    std::string server_address = "127.0.0.1";
    size_t port = 5000;
    espp::UdpSocket server_socket({.log_level=espp::Logger::Verbosity::WARN});
    auto server_task_config = espp::Task::Config{
      .name = "UdpServer",
      .callback = nullptr,
      .stack_size_bytes = 6 * 1024,
    };
    auto server_config = espp::UdpSocket::ReceiveConfig{
      .port = port,
      .buffer_size = 1024,
      .on_receive_callback = [](auto& data, auto& source) -> auto {
        fmt::print("Server received: {}\n"
                   "    from source: {}:{}\n",
                   data, source.address, source.port);
        return std::nullopt;
      }
    };
    server_socket.start_receiving(server_task_config, server_config);
    //! [UDP Server example]

    //! [UDP Client example]
    espp::UdpSocket client_socket({});
    // create thread for sending data using the socket
    auto client_task_fn = [&server_address, &client_socket, &port](auto&, auto&) {
      static size_t iterations=0;
      std::vector<uint8_t> data{0, 1, 2, 3, 4};
      for (auto& d : data) {
        d += iterations;
      }
      auto send_config = espp::UdpSocket::SendConfig{
        .ip_address = server_address,
        .port = port
      };
      client_socket.send(data, send_config);
      iterations++;
      std::this_thread::sleep_for(1s);
    };
    auto client_task = espp::Task::make_unique({
        .name = "Client Task",
        .callback = client_task_fn,
        .stack_size_bytes = 5*1024
      });
    client_task->start();
    //! [UDP Client example]
    // now sleep for a while to let the monitor do its thing
    std::this_thread::sleep_for(test_duration);
  }

  fmt::print(fg(fmt::terminal_color::green) | fmt::emphasis::bold, "Basic UDP test finished.\n");
  std::this_thread::sleep_for(100ms);
  fmt::print(fg(fmt::terminal_color::yellow) | fmt::emphasis::bold, "Staring UDP send waiting for response test.\n");

  // Unicast (client-server) example with server response
  {
    //! [UDP Server Response example]
    std::string server_address = "127.0.0.1";
    size_t port = 5000;
    espp::UdpSocket server_socket({.log_level=espp::Logger::Verbosity::WARN});
    auto server_task_config = espp::Task::Config{
      .name = "UdpServer",
      .callback = nullptr,
      .stack_size_bytes = 6 * 1024
    };
    auto server_config = espp::UdpSocket::ReceiveConfig{
      .port = port,
      .buffer_size = 1024,
      .on_receive_callback = [](auto& data, auto& source) -> auto {
        fmt::print("Server received: {}\n"
                   "    from source: {}:{}\n",
                   data, source.address, source.port);
        // reverse the data
        std::reverse(data.begin(), data.end());
        // and send it back
        return data;
      }
    };
    server_socket.start_receiving(server_task_config, server_config);
    //! [UDP Server Response example]

    //! [UDP Client Response example]
    espp::UdpSocket client_socket({.log_level=espp::Logger::Verbosity::WARN});
    // create threads
    auto client_task_fn = [&server_address, &client_socket, &port](auto&, auto&) {
      static size_t iterations=0;
      std::vector<uint8_t> data{0, 1, 2, 3, 4};
      for (auto& d : data) {
        d += iterations;
      }
      auto send_config = espp::UdpSocket::SendConfig{
        .ip_address = server_address,
        .port = port,
        .wait_for_response = true,
        .response_size = 128,
        .on_response_callback = [](auto& response) {
          fmt::print("Client received: {}\n", response);
        },
      };
      // NOTE: now this call blocks until the response is received
      client_socket.send(data, send_config);
      iterations++;
      std::this_thread::sleep_for(1s);
    };
    auto client_task = espp::Task::make_unique({
        .name = "Client Task",
        .callback = client_task_fn,
        .stack_size_bytes = 5*1024
      });
    client_task->start();
    //! [UDP Client Response example]
    // now sleep for a while to let the monitor do its thing
    std::this_thread::sleep_for(test_duration);
  }

  fmt::print(fg(fmt::terminal_color::green) | fmt::emphasis::bold, "UDP send waiting for response test finished.\n");
  std::this_thread::sleep_for(100ms);
  fmt::print(fg(fmt::terminal_color::yellow) | fmt::emphasis::bold, "Staring UDP multicast test.\n");

  // Multicast example
  {
    //! [UDP Multicast Server example]
    std::string multicast_group = "239.1.1.1";
    size_t port = 5000;
    espp::UdpSocket server_socket({.log_level=espp::Logger::Verbosity::WARN});
    auto server_task_config = espp::Task::Config{
      .name = "UdpServer",
      .callback = nullptr,
      .stack_size_bytes = 6 * 1024,
    };
    auto server_config = espp::UdpSocket::ReceiveConfig{
      .port = port,
      .buffer_size = 1024,
      .is_multicast_endpoint = true,
      .multicast_group = multicast_group,
      .on_receive_callback = [](auto& data, auto& source) -> auto {
        fmt::print("Server received: {}\n"
                   "    from source: {}:{}\n",
                   data, source.address, source.port);
        // reverse the data
        std::reverse(data.begin(), data.end());
        // and send it back
        return data;
      }
    };
    server_socket.start_receiving(server_task_config, server_config);
    //! [UDP Multicast Server example]

    //! [UDP Multicast Client example]
    espp::UdpSocket client_socket({});
    // create threads
    auto client_task_fn = [&client_socket, &port, &multicast_group](auto&, auto&) {
      static size_t iterations=0;
      std::vector<uint8_t> data{0, 1, 2, 3, 4};
      for (auto& d : data) {
        d += iterations;
      }
      auto send_config = espp::UdpSocket::SendConfig{
        .ip_address = multicast_group,
        .port = port,
        .is_multicast_endpoint = true,
        .wait_for_response = true,
        .response_size = 128,
        .on_response_callback = [](auto& response) {
          fmt::print("Client received: {}\n", response);
        }
      };
      // NOTE: now this call blocks until the response is received
      client_socket.send(data, send_config);
      iterations++;
      std::this_thread::sleep_for(1s);
    };
    auto client_task = espp::Task::make_unique({
        .name = "Client Task",
        .callback = client_task_fn,
        .stack_size_bytes = 5*1024
      });
    client_task->start();
    //! [UDP Multicast Client example]
    // now sleep for a while to let the monitor do its thing
    std::this_thread::sleep_for(test_duration);
  }

  fmt::print(fg(fmt::terminal_color::green) | fmt::emphasis::bold, "UDP multicast test finished.\n");
  std::this_thread::sleep_for(100ms);
  fmt::print(fg(fmt::terminal_color::green) | fmt::emphasis::bold, "Socket example finished!\n");

  // sleep forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <numeric>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "logger.hpp"
#include "task.hpp"
#include "tcp_socket.hpp"
#include "udp_socket.hpp"
#include "wifi_ap.hpp"

using namespace std::chrono_literals;

namespace {
using ByteVector = std::vector<uint8_t>;

constexpr auto kLoopbackAddress = "127.0.0.1";
constexpr auto kMulticastGroup = "239.1.1.1";
constexpr auto kPollInterval = 10ms;
constexpr auto kTaskInterval = 100ms;
constexpr auto kScenarioTimeout = 2500ms;
constexpr auto kSettleDelay = 50ms;
constexpr size_t kMaxPacketSize = 1536;
constexpr int kMaxConnections = 2;

struct ScenarioResult {
  std::string name;
  bool passed;
  std::string detail;
};

ByteVector make_payload(size_t size, uint8_t seed = 0) {
  ByteVector data(size);
  std::iota(data.begin(), data.end(), seed);
  return data;
}

ByteVector reversed(ByteVector data) {
  std::reverse(data.begin(), data.end());
  return data;
}

template <typename Predicate>
bool wait_until(Predicate &&predicate, std::chrono::milliseconds timeout,
                std::chrono::milliseconds poll_interval = kPollInterval) {
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(poll_interval);
  }
  return predicate();
}

bool wait_or_stop(std::mutex &m, std::condition_variable &cv, bool &notified,
                  std::chrono::milliseconds duration = kTaskInterval) {
  std::unique_lock<std::mutex> lock(m);
  auto stop_requested = cv.wait_for(lock, duration, [&notified] { return notified; });
  if (stop_requested) {
    notified = false;
  }
  return stop_requested;
}

ScenarioResult pass(std::string name, std::string detail) {
  return {.name = std::move(name), .passed = true, .detail = std::move(detail)};
}

ScenarioResult fail(std::string name, std::string detail) {
  return {.name = std::move(name), .passed = false, .detail = std::move(detail)};
}

void print_scenario_start(std::string_view name) {
  fmt::print(fg(fmt::terminal_color::yellow) | fmt::emphasis::bold, "Running: {}\n", name);
}

void print_scenario_result(const ScenarioResult &result) {
  auto color = result.passed ? fmt::terminal_color::green : fmt::terminal_color::red;
  auto status = result.passed ? "PASS" : "FAIL";
  fmt::print(fg(color) | fmt::emphasis::bold, "[{}] {}", status, result.name);
  if (!result.detail.empty()) {
    fmt::print(": {}", result.detail);
  }
  fmt::print("\n");
}

espp::Task::BaseConfig make_task_config(std::string_view name, size_t stack_size = 6 * 1024) {
  return {.name = std::string(name), .stack_size_bytes = stack_size};
}

ScenarioResult run_udp_unicast_teardown_scenario() {
  constexpr size_t port = 5000;
  constexpr size_t expected_messages = 3;
  std::atomic_size_t received_messages{0};

  {
    //! [UDP Server example]
    espp::UdpSocket server_socket({.log_level = espp::Logger::Verbosity::WARN});
    auto server_task_config = make_task_config("UdpServer");
    auto server_config = espp::UdpSocket::ReceiveConfig{
        .port = port,
        .buffer_size = kMaxPacketSize,
        .on_receive_callback = [&received_messages](const auto &data,
                                                    const auto &source) -> auto{received_messages++;
    fmt::print("UDP server received {} bytes from {}\n", data.size(), source);
    return std::nullopt;
  }
  ,
};
server_socket.start_receiving(server_task_config, server_config);
//! [UDP Server example]

//! [UDP Client example]
espp::UdpSocket client_socket({.log_level = espp::Logger::Verbosity::WARN});
auto client_task = espp::Task::make_unique({
    .callback = [&client_socket](std::mutex &m, std::condition_variable &cv,
                                 bool &notified) -> bool {
      static size_t iterations = 0;
      auto data = make_payload(32, static_cast<uint8_t>(iterations * 8));
      auto send_config = espp::UdpSocket::SendConfig{
          .ip_address = kLoopbackAddress,
          .port = port,
      };
      client_socket.send(data, send_config);
      iterations++;
      return wait_or_stop(m, cv, notified);
    },
    .task_config = make_task_config("UdpClient", 5 * 1024),
});
client_task->start();
//! [UDP Client example]

if (!wait_until([&received_messages] { return received_messages.load() >= expected_messages; },
                kScenarioTimeout)) {
  return fail("UDP unicast teardown",
              fmt::format("timed out after {} messages", received_messages.load()));
}
} // namespace

return pass("UDP unicast teardown",
            fmt::format("received {} messages and exited scope cleanly", received_messages.load()));
}

ScenarioResult run_udp_response_scenario() {
  constexpr size_t port = 5001;
  auto request = make_payload(768, 0x20);
  auto expected_response = reversed(request);
  ByteVector response;
  std::atomic_bool callback_invoked{false};

  {
    //! [UDP Server Response example]
    espp::UdpSocket server_socket({.log_level = espp::Logger::Verbosity::WARN});
    auto server_task_config = make_task_config("UdpResponseServer");
    auto server_config = espp::UdpSocket::ReceiveConfig{
        .port = port, .buffer_size = kMaxPacketSize, .on_receive_callback = [
        ](const auto &data, auto &) -> auto{return std::optional<ByteVector>(reversed(data));
  }
  ,
};
server_socket.start_receiving(server_task_config, server_config);
//! [UDP Server Response example]

//! [UDP Client Response example]
espp::UdpSocket client_socket({.log_level = espp::Logger::Verbosity::WARN});
auto send_config = espp::UdpSocket::SendConfig{
    .ip_address = kLoopbackAddress,
    .port = port,
    .wait_for_response = true,
    .response_size = kMaxPacketSize,
    .on_response_callback =
        [&response, &callback_invoked](const auto &received_response) {
          response = received_response;
          callback_invoked = true;
        },
    .response_timeout = 500ms,
};
auto ok = client_socket.send(request, send_config);
//! [UDP Client Response example]

if (!ok) {
  return fail("UDP request/response", "client send failed");
}
if (!callback_invoked.load()) {
  return fail("UDP request/response", "response callback was not invoked");
}
}

if (response != expected_response) {
  return fail("UDP request/response", fmt::format("response mismatch ({} bytes)", response.size()));
}
return pass("UDP request/response",
            fmt::format("round-tripped {} bytes with reversed response", response.size()));
}

ScenarioResult run_udp_multicast_scenario() {
  constexpr size_t port = 5002;
  auto request = make_payload(64, 0x40);
  auto expected_response = reversed(request);
  ByteVector response;

  {
    //! [UDP Multicast Server example]
    espp::UdpSocket server_socket({.log_level = espp::Logger::Verbosity::WARN});
    auto server_task_config = make_task_config("UdpMulticastServer");
    auto server_config = espp::UdpSocket::ReceiveConfig{
        .port = port,
        .buffer_size = kMaxPacketSize,
        .is_multicast_endpoint = true,
        .multicast_group = kMulticastGroup,
        .on_receive_callback =
            [](const auto &data, auto &) -> auto{return std::optional<ByteVector>(reversed(data));
  }
  ,
};
server_socket.start_receiving(server_task_config, server_config);
//! [UDP Multicast Server example]

//! [UDP Multicast Client example]
espp::UdpSocket client_socket({.log_level = espp::Logger::Verbosity::WARN});
auto send_config = espp::UdpSocket::SendConfig{
    .ip_address = kMulticastGroup,
    .port = port,
    .is_multicast_endpoint = true,
    .wait_for_response = true,
    .response_size = kMaxPacketSize,
    .on_response_callback =
        [&response](const auto &received_response) { response = received_response; },
    .response_timeout = 500ms,
};
auto ok = client_socket.send(request, send_config);
//! [UDP Multicast Client example]

if (!ok) {
  return fail("UDP multicast request/response", "multicast send failed");
}
}

if (response != expected_response) {
  return fail("UDP multicast request/response",
              fmt::format("response mismatch ({} bytes)", response.size()));
}
return pass("UDP multicast request/response",
            fmt::format("received {} byte response", response.size()));
}

ScenarioResult run_udp_timeout_scenario() {
  constexpr size_t port = 5003;
  espp::UdpSocket client_socket({.log_level = espp::Logger::Verbosity::WARN});
  auto send_config = espp::UdpSocket::SendConfig{
      .ip_address = kLoopbackAddress,
      .port = port,
      .wait_for_response = true,
      .response_size = 64,
      .response_timeout = 150ms,
  };

  auto ok = client_socket.send(make_payload(16, 0x55), send_config);
  if (ok) {
    return fail("UDP response timeout", "unexpectedly received a response");
  }
  return pass("UDP response timeout", "send timed out as expected with no server");
}

ScenarioResult run_udp_blocked_receive_teardown_scenario() {
  constexpr size_t port = 5004;
  {
    espp::UdpSocket server_socket({.log_level = espp::Logger::Verbosity::WARN});
    auto server_task_config = make_task_config("UdpBlockedServer");
    auto server_config = espp::UdpSocket::ReceiveConfig{
        .port = port, .buffer_size = kMaxPacketSize, .on_receive_callback = [
        ](auto &, auto &) -> auto{return std::nullopt;
  }
  ,
};
if (!server_socket.start_receiving(server_task_config, server_config)) {
  return fail("UDP blocked receive teardown", "failed to start receive task");
}
std::this_thread::sleep_for(kSettleDelay);
}
return pass("UDP blocked receive teardown", "receive task stopped while idle");
}

ScenarioResult run_tcp_unicast_teardown_scenario() {
  constexpr size_t port = 6000;
  constexpr size_t expected_messages = 3;
  std::atomic_size_t received_messages{0};

  {
    //! [TCP Server example]
    espp::TcpSocket server_socket({.log_level = espp::Logger::Verbosity::WARN});
    if (!server_socket.bind(port) || !server_socket.listen(kMaxConnections)) {
      return fail("TCP unicast teardown", "failed to bind/listen");
    }
    auto server_task = espp::Task::make_unique({
        .callback = [&server_socket, &received_messages](std::mutex &m, std::condition_variable &cv,
                                                         bool &notified) -> bool {
          static std::unique_ptr<espp::TcpSocket> client_socket;
          if (!client_socket) {
            client_socket = server_socket.accept();
            if (!client_socket) {
              return wait_or_stop(m, cv, notified, 10ms);
            }
          }

          ByteVector data;
          if (client_socket->receive(data, kMaxPacketSize)) {
            received_messages++;
            fmt::print("TCP server received {} bytes\n", data.size());
          } else if (!client_socket->is_connected()) {
            client_socket.reset();
          }
          return wait_or_stop(m, cv, notified, 10ms);
        },
        .task_config = make_task_config("TcpServer"),
    });
    server_task->start();
    //! [TCP Server example]

    //! [TCP Client example]
    espp::TcpSocket client_socket({.log_level = espp::Logger::Verbosity::WARN});
    if (!client_socket.connect({.ip_address = kLoopbackAddress, .port = port})) {
      server_socket.close();
      server_task->stop();
      return fail("TCP unicast teardown", "client failed to connect");
    }
    auto client_task = espp::Task::make_unique({
        .callback = [&client_socket](std::mutex &m, std::condition_variable &cv,
                                     bool &notified) -> bool {
          static size_t iterations = 0;
          auto data = make_payload(24, static_cast<uint8_t>(iterations * 5));
          client_socket.transmit(data);
          iterations++;
          return wait_or_stop(m, cv, notified);
        },
        .task_config = make_task_config("TcpClient", 5 * 1024),
    });
    client_task->start();
    //! [TCP Client example]

    if (!wait_until([&received_messages] { return received_messages.load() >= expected_messages; },
                    kScenarioTimeout)) {
      client_task->stop();
      client_socket.close();
      server_socket.close();
      server_task->stop();
      return fail("TCP unicast teardown",
                  fmt::format("timed out after {} messages", received_messages.load()));
    }

    client_task->stop();
    client_socket.close();
    server_socket.close();
    server_task->stop();
  }

  return pass("TCP unicast teardown", fmt::format("received {} messages and exited scope cleanly",
                                                  received_messages.load()));
}

ScenarioResult run_tcp_response_reconnect_scenario() {
  constexpr size_t port = 6001;
  std::atomic_size_t accepted_connections{0};

  {
    //! [TCP Server Response example]
    espp::TcpSocket server_socket({.log_level = espp::Logger::Verbosity::WARN});
    if (!server_socket.bind(port) || !server_socket.listen(kMaxConnections)) {
      return fail("TCP response/reconnect", "failed to bind/listen");
    }
    auto server_task = espp::Task::make_unique({
        .callback = [&server_socket, &accepted_connections](
                        std::mutex &m, std::condition_variable &cv, bool &notified) -> bool {
          static std::unique_ptr<espp::TcpSocket> client_socket;
          if (!client_socket) {
            client_socket = server_socket.accept();
            if (client_socket) {
              accepted_connections++;
            } else {
              return wait_or_stop(m, cv, notified, 10ms);
            }
          }

          ByteVector data;
          if (client_socket->receive(data, kMaxPacketSize)) {
            client_socket->transmit(reversed(data));
          } else if (!client_socket->is_connected()) {
            client_socket.reset();
          }
          return wait_or_stop(m, cv, notified, 10ms);
        },
        .task_config = make_task_config("TcpResponseServer"),
    });
    server_task->start();
    auto stop_server = [&server_socket, &server_task]() {
      server_socket.close();
      server_task->stop();
    };
    //! [TCP Server Response example]

    //! [TCP Client Response example]
    auto exchange_once = [port](uint8_t seed, ByteVector &response) -> bool {
      espp::TcpSocket client_socket({.log_level = espp::Logger::Verbosity::WARN});
      if (!client_socket.connect({.ip_address = kLoopbackAddress, .port = port})) {
        return false;
      }
      auto request = make_payload(40, seed);
      auto transmit_config = espp::TcpSocket::TransmitConfig{
          .wait_for_response = true,
          .response_size = kMaxPacketSize,
          .on_response_callback =
              [&response](const auto &received_response) { response = received_response; },
          .response_timeout = 500ms,
      };
      return client_socket.transmit(request, transmit_config);
    };
    //! [TCP Client Response example]

    ByteVector first_response;
    ByteVector second_response;
    if (!exchange_once(0x10, first_response)) {
      stop_server();
      return fail("TCP response/reconnect", "first request/response failed");
    }
    if (!wait_until([&accepted_connections] { return accepted_connections.load() >= 1; },
                    kScenarioTimeout)) {
      stop_server();
      return fail("TCP response/reconnect", "server never accepted the first connection");
    }
    if (!exchange_once(0x30, second_response)) {
      stop_server();
      return fail("TCP response/reconnect", "second request/response failed");
    }
    if (!wait_until([&accepted_connections] { return accepted_connections.load() >= 2; },
                    kScenarioTimeout)) {
      stop_server();
      return fail("TCP response/reconnect", "server never accepted the reconnect");
    }

    if (first_response != reversed(make_payload(40, 0x10))) {
      stop_server();
      return fail("TCP response/reconnect", "first response mismatch");
    }
    if (second_response != reversed(make_payload(40, 0x30))) {
      stop_server();
      return fail("TCP response/reconnect", "second response mismatch");
    }

    stop_server();
  }

  return pass("TCP response/reconnect",
              fmt::format("handled {} sequential client connections", accepted_connections.load()));
}

ScenarioResult run_tcp_blocked_accept_teardown_scenario() {
  constexpr size_t port = 6002;
  {
    espp::TcpSocket server_socket({.log_level = espp::Logger::Verbosity::WARN});
    if (!server_socket.bind(port) || !server_socket.listen(kMaxConnections)) {
      return fail("TCP blocked accept teardown", "failed to bind/listen");
    }
    auto server_task = espp::Task::make_unique({
        .callback = [&server_socket](std::mutex &m, std::condition_variable &cv,
                                     bool &notified) -> bool {
          auto accepted_socket = server_socket.accept();
          if (accepted_socket) {
            fmt::print("Unexpected TCP connection from {}\n", accepted_socket->get_remote_info());
          }
          return wait_or_stop(m, cv, notified, 10ms);
        },
        .task_config = make_task_config("TcpBlockedAccept"),
    });
    server_task->start();
    std::this_thread::sleep_for(kSettleDelay);

    server_socket.close();
    server_task->stop();
  }
  return pass("TCP blocked accept teardown", "accept task stopped while idle");
}

ScenarioResult run_tcp_connect_failure_scenario() {
  constexpr size_t port = 6003;
  espp::TcpSocket client_socket({.log_level = espp::Logger::Verbosity::WARN});
  if (client_socket.connect({.ip_address = kLoopbackAddress, .port = port})) {
    return fail("TCP connect failure", "unexpectedly connected to an unused port");
  }
  return pass("TCP connect failure", "connect failed as expected");
}
} // namespace

extern "C" void app_main(void) {
  fmt::print("Starting socket example!\n");

  // create a wifi access point here so that LwIP will be init for this example
  espp::WifiAp wifi_ap({.ssid = "SocketExample",
                        .password = "", // no security
                        .log_level = espp::Logger::Verbosity::INFO});

  std::vector<ScenarioResult> results;
  results.reserve(9);

  auto run_and_record = [&results](auto &&scenario_runner, std::string_view name) {
    print_scenario_start(name);
    auto result = scenario_runner();
    print_scenario_result(result);
    results.push_back(std::move(result));
    std::this_thread::sleep_for(kSettleDelay);
  };

  run_and_record(run_udp_unicast_teardown_scenario, "UDP unicast teardown");
  run_and_record(run_udp_response_scenario, "UDP request/response");
  run_and_record(run_udp_multicast_scenario, "UDP multicast request/response");
  run_and_record(run_udp_timeout_scenario, "UDP response timeout");
  run_and_record(run_udp_blocked_receive_teardown_scenario, "UDP blocked receive teardown");
  run_and_record(run_tcp_unicast_teardown_scenario, "TCP unicast teardown");
  run_and_record(run_tcp_response_reconnect_scenario, "TCP response/reconnect");
  run_and_record(run_tcp_blocked_accept_teardown_scenario, "TCP blocked accept teardown");
  run_and_record(run_tcp_connect_failure_scenario, "TCP connect failure");

  auto passed_count = std::count_if(results.begin(), results.end(),
                                    [](const auto &result) { return result.passed; });
  fmt::print(fg(fmt::terminal_color::cyan) | fmt::emphasis::bold,
             "Socket example summary: {}/{} scenarios passed\n", passed_count, results.size());
  for (const auto &result : results) {
    print_scenario_result(result);
  }

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

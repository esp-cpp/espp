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
#include "socket_reactor.hpp"
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

ScenarioResult run_socket_reactor_scenario() {
  // Two UDP receiver sockets, on two ports, both driven by ONE reactor (one
  // select() loop thread + a shared 2-worker pool) - no dedicated thread per
  // socket. Each server echoes the request back reversed.
  constexpr size_t port_a = 5020;
  constexpr size_t port_b = 5021;
  auto echo_reversed = [](const ByteVector &data, const espp::Socket::Info &) {
    return std::optional<ByteVector>(reversed(data));
  };

  //! [socket reactor example]
  espp::SocketReactor reactor({.log_level = espp::Logger::Verbosity::WARN});

  espp::UdpSocket server_a({.log_level = espp::Logger::Verbosity::WARN});
  espp::UdpSocket server_b({.log_level = espp::Logger::Verbosity::WARN});
  auto id_a = reactor.add_udp_receiver(
      server_a,
      {.port = port_a, .buffer_size = kMaxPacketSize, .on_receive_callback = echo_reversed});
  auto id_b = reactor.add_udp_receiver(
      server_b,
      {.port = port_b, .buffer_size = kMaxPacketSize, .on_receive_callback = echo_reversed});
  //! [socket reactor example]

  if (id_a == espp::SocketReactor::INVALID_ID || id_b == espp::SocketReactor::INVALID_ID) {
    return fail("Socket reactor", "failed to register one or both UDP receivers");
  }
  if (reactor.num_registered() != 2) {
    return fail("Socket reactor",
                fmt::format("expected 2 registrations, got {}", reactor.num_registered()));
  }

  // Send a distinct request to each server and verify each reversed response.
  auto send_and_check = [&](size_t port, uint8_t seed) -> bool {
    auto request = make_payload(512, seed);
    auto expected = reversed(request);
    ByteVector response;
    std::atomic_bool got_response{false};
    espp::UdpSocket client({.log_level = espp::Logger::Verbosity::WARN});
    client.send(request, {.ip_address = kLoopbackAddress,
                          .port = port,
                          .wait_for_response = true,
                          .response_size = kMaxPacketSize,
                          .on_response_callback =
                              [&](const ByteVector &r) {
                                response = r;
                                got_response = true;
                              },
                          .response_timeout = 500ms});
    return got_response.load() && response == expected;
  };

  if (!send_and_check(port_a, 0x10)) {
    return fail("Socket reactor", "server A did not echo correctly via the reactor");
  }
  if (!send_and_check(port_b, 0x80)) {
    return fail("Socket reactor", "server B did not echo correctly via the reactor");
  }

  // Unregister one socket and confirm the count drops.
  reactor.remove(id_a);
  if (!wait_until([&] { return reactor.num_registered() == 1; }, 500ms)) {
    return fail("Socket reactor", fmt::format("expected 1 registration after remove, got {}",
                                              reactor.num_registered()));
  }

  return pass("Socket reactor",
              "2 UDP sockets multiplexed on 1 select loop + shared pool, both echoed");
}

ScenarioResult run_reactor_priority_bands_scenario() {
  // Two UDP echo receivers on ONE reactor, registered at different QosBands.
  // The Critical receiver additionally marks its transmitted replies with
  // DSCP 46 (EF, "expedited forwarding"), exercising the IP_TOS setsockopt
  // path on lwIP. Functional on-target check of the band-aware registration +
  // dispatch: flood the Low-band port, then confirm the Critical-band port
  // still answers within the timeout, and that Low is still serviced (no band
  // is starved or unreachable).
  constexpr size_t critical_port = 5030;
  constexpr size_t low_port = 5031;
  auto echo_reversed = [](const ByteVector &data, const espp::Socket::Info &) {
    return std::optional<ByteVector>(reversed(data));
  };

  //! [socket reactor priority example]
  espp::SocketReactor reactor({.log_level = espp::Logger::Verbosity::WARN});

  espp::UdpSocket critical_server({.log_level = espp::Logger::Verbosity::WARN});
  espp::UdpSocket low_server({.log_level = espp::Logger::Verbosity::WARN});
  auto critical_id = reactor.add_udp_receiver(critical_server,
                                              {.port = critical_port,
                                               .buffer_size = kMaxPacketSize,
                                               .on_receive_callback = echo_reversed,
                                               .band = espp::QosBand::Critical,
                                               .dscp = espp::Dscp::EF}); // latency-critical replies
  auto low_id = reactor.add_udp_receiver(low_server, {.port = low_port,
                                                      .buffer_size = kMaxPacketSize,
                                                      .on_receive_callback = echo_reversed,
                                                      .band = espp::QosBand::Low});
  //! [socket reactor priority example]

  if (critical_id == espp::SocketReactor::INVALID_ID || low_id == espp::SocketReactor::INVALID_ID) {
    return fail("Reactor priority bands", "failed to register one or both banded UDP receivers");
  }

  // Flood the Low-band port with fire-and-forget packets so its receive
  // handling occupies the shared pool...
  espp::UdpSocket flooder({.log_level = espp::Logger::Verbosity::WARN});
  for (int i = 0; i < 8; ++i) {
    flooder.send(make_payload(512, static_cast<uint8_t>(i)),
                 {.ip_address = kLoopbackAddress, .port = low_port});
  }

  // ...then verify each banded port answers within the timeout.
  auto send_and_check = [&](size_t port, uint8_t seed) -> bool {
    auto request = make_payload(512, seed);
    auto expected = reversed(request);
    ByteVector response;
    std::atomic_bool got_response{false};
    espp::UdpSocket client({.log_level = espp::Logger::Verbosity::WARN});
    client.send(request, {.ip_address = kLoopbackAddress,
                          .port = port,
                          .wait_for_response = true,
                          .response_size = kMaxPacketSize,
                          .on_response_callback =
                              [&](const ByteVector &r) {
                                response = r;
                                got_response = true;
                              },
                          .response_timeout = 500ms});
    return got_response.load() && response == expected;
  };

  if (!send_and_check(critical_port, 0x20)) {
    return fail("Reactor priority bands",
                "Critical-band server did not echo during the Low-band flood");
  }
  // The Low band must still be serviced too (banded dispatch must not starve
  // or strand the least-urgent band).
  if (!send_and_check(low_port, 0x90)) {
    return fail("Reactor priority bands", "Low-band server did not echo after the flood");
  }

  return pass("Reactor priority bands",
              "Critical (DSCP EF) + Low banded receivers echoed during and after a Low flood");
}

ScenarioResult run_tcp_reactor_scenario() {
  // A TCP echo server built entirely on the reactor: one listener registration
  // accepts clients, and each accepted client is registered as a stream that
  // echoes bytes back - no accept thread and no thread-per-client.
  constexpr size_t port = 6010;
  auto request = make_payload(256, 0x40);
  std::atomic_bool got_echo{false};
  std::atomic_int accepted{0};
  std::atomic_bool closed{false};

  // Accepted server-side client sockets must outlive their reactor stream
  // registration, so keep them alive here (populated from a pool worker).
  std::mutex clients_mutex;
  std::vector<std::unique_ptr<espp::TcpSocket>> clients;

  {
    espp::SocketReactor reactor({.log_level = espp::Logger::Verbosity::WARN});

    espp::TcpSocket server({.log_level = espp::Logger::Verbosity::WARN});
    if (!server.bind(port) || !server.listen(kMaxConnections)) {
      return fail("TCP reactor", "failed to bind/listen");
    }

    //! [socket reactor tcp example]
    reactor.add_tcp_listener(server, [&](std::unique_ptr<espp::TcpSocket> client) {
      ++accepted;
      espp::TcpSocket *conn = nullptr;
      {
        std::lock_guard<std::mutex> lk(clients_mutex);
        clients.push_back(std::move(client));
        conn = clients.back().get();
      }
      // Register the accepted connection as a stream that echoes bytes back.
      reactor.add_tcp_stream(
          *conn, [](espp::TcpSocket &connection, ByteVector &data) { connection.transmit(data); },
          kMaxPacketSize, [&closed]() { closed = true; });
    });
    //! [socket reactor tcp example]

    ByteVector echo;
    espp::TcpSocket client({.log_level = espp::Logger::Verbosity::WARN});
    if (!client.connect({.ip_address = kLoopbackAddress, .port = port})) {
      return fail("TCP reactor", "client failed to connect");
    }
    client.transmit(request, {.wait_for_response = true,
                              .response_size = kMaxPacketSize,
                              .on_response_callback =
                                  [&](const ByteVector &r) {
                                    echo = r;
                                    got_echo = true;
                                  },
                              .response_timeout = 1s});

    if (!got_echo.load() || echo != request) {
      return fail("TCP reactor", "did not receive correct echo via the reactor");
    }
    if (accepted.load() != 1) {
      return fail("TCP reactor", fmt::format("expected 1 accept, got {}", accepted.load()));
    }

    // Close the client and verify the server-side stream notices the disconnect
    // and auto-unregisters, leaving only the listener.
    client.close();
    if (!wait_until([&] { return closed.load(); }, 1s)) {
      return fail("TCP reactor", "server did not observe client disconnect");
    }
    if (!wait_until([&] { return reactor.num_registered() == 1; }, 1s)) {
      return fail("TCP reactor", fmt::format("expected 1 registration after disconnect, got {}",
                                             reactor.num_registered()));
    }
  }
  return pass("TCP reactor", "listener + stream echo on one reactor, auto-removed on disconnect");
}

// Helper: send `request` to a UDP echo server on `port` and check the reversed
// reply comes back. Returns true on a correct round-trip.
bool udp_echo_roundtrip(size_t port, const ByteVector &request) {
  ByteVector response;
  std::atomic_bool got_response{false};
  espp::UdpSocket client({.log_level = espp::Logger::Verbosity::WARN});
  client.send(request, {.ip_address = kLoopbackAddress,
                        .port = port,
                        .wait_for_response = true,
                        .response_size = kMaxPacketSize,
                        .on_response_callback =
                            [&](const ByteVector &r) {
                              response = r;
                              got_response = true;
                            },
                        .response_timeout = 500ms});
  return got_response.load() && response == reversed(request);
}

ScenarioResult run_reactor_shared_pool_scenario() {
  // One externally-owned ThreadPool shared with the reactor, multiplexing three
  // UDP echo receivers, plus a dynamic remove() while running.
  constexpr size_t base_port = 5030;
  auto echo = [](const ByteVector &d, const espp::Socket::Info &) {
    return std::optional<ByteVector>(reversed(d));
  };
  auto pool = std::make_shared<espp::ThreadPool>(
      espp::ThreadPool::Config{.worker_count = 3, .log_level = espp::Logger::Verbosity::WARN});
  // Declare the sockets before the reactor so the reactor is destroyed first.
  std::vector<std::unique_ptr<espp::UdpSocket>> servers;
  {
    espp::SocketReactor reactor({.thread_pool = pool, .log_level = espp::Logger::Verbosity::WARN});
    std::vector<espp::SocketReactor::Id> ids;
    for (int i = 0; i < 3; ++i) {
      servers.push_back(std::make_unique<espp::UdpSocket>(
          espp::UdpSocket::Config{.log_level = espp::Logger::Verbosity::WARN}));
      auto id = reactor.add_udp_receiver(
          *servers.back(),
          {.port = base_port + i, .buffer_size = kMaxPacketSize, .on_receive_callback = echo});
      if (id == espp::SocketReactor::INVALID_ID) {
        return fail("Reactor shared pool", fmt::format("failed to register receiver {}", i));
      }
      ids.push_back(id);
    }
    if (reactor.num_registered() != 3) {
      return fail("Reactor shared pool", "expected 3 registrations");
    }
    for (int i = 0; i < 3; ++i) {
      if (!udp_echo_roundtrip(base_port + i, make_payload(200, static_cast<uint8_t>(i * 10 + 1)))) {
        return fail("Reactor shared pool", fmt::format("receiver {} echo failed", i));
      }
    }
    // Dynamically drop the middle receiver; the others keep working.
    reactor.remove(ids[1]);
    if (!wait_until([&] { return reactor.num_registered() == 2; }, 500ms)) {
      return fail("Reactor shared pool", "count did not drop to 2 after remove()");
    }
    if (!udp_echo_roundtrip(base_port + 0, make_payload(200, 0x01)) ||
        !udp_echo_roundtrip(base_port + 2, make_payload(200, 0x15))) {
      return fail("Reactor shared pool", "remaining receivers broke after remove()");
    }
    reactor.stop(); // quiesce before the sockets/pool go out of scope
  }
  return pass("Reactor shared pool", "3 UDP receivers on a shared pool; dynamic remove works");
}

ScenarioResult run_reactor_lifecycle_scenario() {
  espp::SocketReactor reactor({.auto_start = false, .log_level = espp::Logger::Verbosity::WARN});
  if (reactor.is_running()) {
    return fail("Reactor lifecycle", "running before start()");
  }
  // Invalid registrations are rejected with INVALID_ID.
  if (reactor.add_fd(-1, []() {}) != espp::SocketReactor::INVALID_ID) {
    return fail("Reactor lifecycle", "invalid fd was not rejected");
  }
  if (reactor.add_fd(0, nullptr) != espp::SocketReactor::INVALID_ID) {
    return fail("Reactor lifecycle", "null handler was not rejected");
  }
  if (!reactor.start() || !reactor.is_running()) {
    return fail("Reactor lifecycle", "start() failed");
  }
  {
    constexpr size_t port = 5040;
    espp::UdpSocket server({.log_level = espp::Logger::Verbosity::WARN});
    auto id = reactor.add_udp_receiver(
        server, {.port = port,
                 .buffer_size = kMaxPacketSize,
                 .on_receive_callback = [](const ByteVector &d, const espp::Socket::Info &) {
                   return std::optional<ByteVector>(reversed(d));
                 }});
    if (id == espp::SocketReactor::INVALID_ID) {
      return fail("Reactor lifecycle", "register after manual start failed");
    }
    if (!udp_echo_roundtrip(port, make_payload(128, 0x55))) {
      return fail("Reactor lifecycle", "echo after manual start failed");
    }
    reactor.stop();
  }
  if (reactor.is_running()) {
    return fail("Reactor lifecycle", "still running after stop()");
  }
  return pass("Reactor lifecycle",
              "auto_start=false then start(); invalid registrations rejected; stop() works");
}

ScenarioResult run_reactor_tcp_multiclient_scenario() {
  // One reactor accepts and services two concurrent TCP clients (no accept
  // thread, no thread-per-client).
  constexpr size_t port = 6020;
  std::atomic_int accepted{0};
  std::mutex clients_mutex;
  std::vector<std::unique_ptr<espp::TcpSocket>> clients;
  {
    espp::SocketReactor reactor({.log_level = espp::Logger::Verbosity::WARN});
    espp::TcpSocket server({.log_level = espp::Logger::Verbosity::WARN});
    if (!server.bind(port) || !server.listen(4)) {
      return fail("Reactor TCP multi-client", "failed to bind/listen");
    }
    reactor.add_tcp_listener(server, [&](std::unique_ptr<espp::TcpSocket> client) {
      ++accepted;
      espp::TcpSocket *conn = nullptr;
      {
        std::lock_guard<std::mutex> lk(clients_mutex);
        clients.push_back(std::move(client));
        conn = clients.back().get();
      }
      reactor.add_tcp_stream(
          *conn, [](espp::TcpSocket &c, ByteVector &data) { c.transmit(data); }, kMaxPacketSize);
    });

    // Two clients each round-trip a distinct payload concurrently.
    auto client_roundtrip = [&](uint8_t seed) -> bool {
      auto request = make_payload(200, seed);
      ByteVector echo;
      std::atomic_bool got{false};
      espp::TcpSocket client({.log_level = espp::Logger::Verbosity::WARN});
      if (!client.connect({.ip_address = kLoopbackAddress, .port = port})) {
        return false;
      }
      client.transmit(request, {.wait_for_response = true,
                                .response_size = kMaxPacketSize,
                                .on_response_callback =
                                    [&](const ByteVector &r) {
                                      echo = r;
                                      got = true;
                                    },
                                .response_timeout = 1s});
      return got.load() && echo == request;
    };
    if (!client_roundtrip(0x11) || !client_roundtrip(0x22)) {
      return fail("Reactor TCP multi-client", "one or both clients did not echo");
    }
    if (!wait_until([&] { return accepted.load() == 2; }, 1s)) {
      return fail("Reactor TCP multi-client",
                  fmt::format("expected 2 accepts, got {}", accepted.load()));
    }
    reactor.stop(); // quiesce before clients/server go out of scope
  }
  return pass("Reactor TCP multi-client", "2 concurrent clients accepted + echoed on one reactor");
}

ScenarioResult run_udp_send_overloads_scenario() {
  // Exercise the string_view and span send overloads and verify the server sees
  // the correct sender address/port.
  constexpr size_t port = 5050;
  std::atomic_bool saw_sender{false};
  std::string sender_addr;
  espp::UdpSocket server({.log_level = espp::Logger::Verbosity::WARN});
  auto server_task_config = make_task_config("UdpOverloadServer");
  server.start_receiving(
      server_task_config,
      {.port = port,
       .buffer_size = kMaxPacketSize,
       .on_receive_callback = [&](const ByteVector &d, const espp::Socket::Info &sender) {
         sender_addr = sender.address;
         saw_sender = true;
         return std::optional<ByteVector>(reversed(d));
       }});

  // string_view overload
  {
    std::string_view msg = "hello-string-view";
    ByteVector resp;
    std::atomic_bool got{false};
    espp::UdpSocket client({.log_level = espp::Logger::Verbosity::WARN});
    client.send(msg, {.ip_address = kLoopbackAddress,
                      .port = port,
                      .wait_for_response = true,
                      .response_size = kMaxPacketSize,
                      .on_response_callback =
                          [&](const ByteVector &r) {
                            resp = r;
                            got = true;
                          },
                      .response_timeout = 500ms});
    ByteVector expected(msg.begin(), msg.end());
    if (!got.load() || resp != reversed(expected)) {
      return fail("UDP send overloads", "string_view send/echo failed");
    }
  }
  // span overload
  {
    auto payload = make_payload(64, 0x33);
    ByteVector resp;
    std::atomic_bool got{false};
    espp::UdpSocket client({.log_level = espp::Logger::Verbosity::WARN});
    client.send(std::span<const uint8_t>{payload.data(), payload.size()},
                {.ip_address = kLoopbackAddress,
                 .port = port,
                 .wait_for_response = true,
                 .response_size = kMaxPacketSize,
                 .on_response_callback =
                     [&](const ByteVector &r) {
                       resp = r;
                       got = true;
                     },
                 .response_timeout = 500ms});
    if (!got.load() || resp != reversed(payload)) {
      return fail("UDP send overloads", "span send/echo failed");
    }
  }
  server.stop_receiving();
  if (!saw_sender.load() || sender_addr != kLoopbackAddress) {
    return fail("UDP send overloads", fmt::format("sender address wrong: '{}'", sender_addr));
  }
  return pass("UDP send overloads", "string_view + span overloads round-trip; sender info correct");
}
} // namespace

extern "C" void app_main(void) {
  fmt::print("Starting socket example!\n");

  // create a wifi access point here so that LwIP will be init for this example
  espp::WifiAp wifi_ap({.ssid = "SocketExample",
                        .password = "", // no security
                        .log_level = espp::Logger::Verbosity::INFO});

  std::vector<ScenarioResult> results;
  results.reserve(15);

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
  run_and_record(run_socket_reactor_scenario, "Socket reactor (select + thread pool)");
  run_and_record(run_reactor_priority_bands_scenario, "Reactor priority bands (QosBand + DSCP)");
  run_and_record(run_tcp_reactor_scenario, "TCP reactor (listener + streams)");
  run_and_record(run_reactor_shared_pool_scenario, "Reactor shared pool + dynamic remove");
  run_and_record(run_reactor_lifecycle_scenario, "Reactor lifecycle + input validation");
  run_and_record(run_reactor_tcp_multiclient_scenario, "Reactor TCP multi-client");
  run_and_record(run_udp_send_overloads_scenario, "UDP send overloads + sender info");

  // ---- final summary ----
  auto passed_count = std::count_if(results.begin(), results.end(),
                                    [](const auto &result) { return result.passed; });
  const auto failed_count = results.size() - static_cast<size_t>(passed_count);
  fmt::print("\n");
  fmt::print(fg(fmt::terminal_color::cyan) | fmt::emphasis::bold,
             "======== Socket example summary: {}/{} scenarios passed ========\n", passed_count,
             results.size());
  for (const auto &result : results) {
    print_scenario_result(result);
  }
  if (failed_count == 0) {
    fmt::print(fg(fmt::terminal_color::green) | fmt::emphasis::bold, "ALL {} SCENARIOS PASSED\n",
               results.size());
  } else {
    fmt::print(fg(fmt::terminal_color::red) | fmt::emphasis::bold, "{} SCENARIO(S) FAILED:\n",
               failed_count);
    for (const auto &result : results) {
      if (!result.passed) {
        fmt::print(fg(fmt::terminal_color::red), "  - {}: {}\n", result.name, result.detail);
      }
    }
  }

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "socket_reactor.hpp"
#include "tcp_socket.hpp"
#include "udp_socket.hpp"

using namespace std::chrono_literals;
using ByteVector = std::vector<uint8_t>;

namespace {
constexpr auto kLoopback = "127.0.0.1";
constexpr size_t kBufferSize = 1500;
constexpr auto WARN = espp::Logger::Verbosity::WARN;

ByteVector make_payload(size_t n, uint8_t seed) {
  ByteVector v(n);
  for (size_t i = 0; i < n; ++i) {
    v[i] = static_cast<uint8_t>(seed + i);
  }
  return v;
}

ByteVector reversed(ByteVector v) {
  std::reverse(v.begin(), v.end());
  return v;
}

template <typename Predicate>
bool wait_until(Predicate &&pred, std::chrono::milliseconds timeout = 1s,
                std::chrono::milliseconds interval = 10ms) {
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(interval);
  }
  return pred();
}

// Send `request` to a UDP echo server on `port` and return true if the reversed
// reply comes back.
bool udp_echo_roundtrip(size_t port, const ByteVector &request) {
  ByteVector response;
  std::atomic_bool got{false};
  espp::UdpSocket client({.log_level = WARN});
  client.send(request, {.ip_address = kLoopback,
                        .port = port,
                        .wait_for_response = true,
                        .response_size = kBufferSize,
                        .on_response_callback =
                            [&](const ByteVector &r) {
                              response = r;
                              got = true;
                            },
                        .response_timeout = 500ms});
  return got.load() && response == reversed(request);
}
} // namespace

// Mirrors python/socket_reactor_test.py, plus the TCP listener/stream paths
// (which are C++-only). Exit code 0 on full pass, 1 on any failure.
int main() {
  espp::Logger logger({.tag = "SocketReactor Test", .level = espp::Logger::Verbosity::INFO});

  int total_passed = 0;
  int total_tests = 0;
  auto check = [&](bool condition, const std::string &desc) -> bool {
    ++total_tests;
    if (condition) {
      ++total_passed;
      logger.info("  PASS: {}", desc);
    } else {
      logger.error("  FAIL: {}", desc);
    }
    return condition;
  };

  auto echo_reversed = [](const ByteVector &d, const espp::Socket::Info &) {
    return std::optional<ByteVector>(reversed(d));
  };

  // -------------------------------------------------------------------------
  // 1. Lifecycle + UDP echo + multiple receivers + dynamic remove
  // -------------------------------------------------------------------------
  logger.info("--- lifecycle + udp ---");
  {
    // sockets declared before the reactor so the reactor is destroyed first
    std::vector<std::unique_ptr<espp::UdpSocket>> servers;
    espp::SocketReactor reactor({.auto_start = false, .log_level = WARN});
    check(!reactor.is_running(), "not running before start()");
    check(reactor.start() && reactor.is_running(), "start() -> running");
    check(reactor.num_registered() == 0, "no registrations initially");

    std::vector<espp::SocketReactor::Id> ids;
    for (int i = 0; i < 3; ++i) {
      servers.push_back(
          std::make_unique<espp::UdpSocket>(espp::UdpSocket::Config{.log_level = WARN}));
      auto id = reactor.add_udp_receiver(*servers.back(), {.port = 6111 + static_cast<size_t>(i),
                                                           .buffer_size = kBufferSize,
                                                           .on_receive_callback = echo_reversed});
      check(id != espp::SocketReactor::INVALID_ID, "add_udp_receiver returns a valid id");
      ids.push_back(id);
    }
    check(reactor.num_registered() == 3, "three registrations");
    check(udp_echo_roundtrip(6111, make_payload(40, 0x01)), "receiver 0 echoes");
    check(udp_echo_roundtrip(6113, make_payload(40, 0x20)), "receiver 2 echoes");

    check(reactor.remove(ids[1]), "remove() returns true for a valid id");
    check(wait_until([&] { return reactor.num_registered() == 2; }), "count drops after remove");
    check(!reactor.remove(999999), "remove() of unknown id returns false");
    check(udp_echo_roundtrip(6111, make_payload(40, 0x02)), "remaining receiver still echoes");

    reactor.stop();
    check(!reactor.is_running(), "not running after stop()");
  }

  // -------------------------------------------------------------------------
  // 2. Input validation
  // -------------------------------------------------------------------------
  logger.info("--- input validation ---");
  {
    espp::SocketReactor reactor({.log_level = WARN});
    check(reactor.add_fd(-1, []() {}) == espp::SocketReactor::INVALID_ID, "invalid fd rejected");
    check(reactor.add_fd(0, nullptr) == espp::SocketReactor::INVALID_ID, "null handler rejected");
  }

  // -------------------------------------------------------------------------
  // 3. Shared external thread pool
  // -------------------------------------------------------------------------
  logger.info("--- shared pool ---");
  {
    auto pool = std::make_shared<espp::ThreadPool>(
        espp::ThreadPool::Config{.worker_count = 2, .log_level = WARN});
    espp::UdpSocket server({.log_level = WARN});
    {
      espp::SocketReactor reactor({.thread_pool = pool, .log_level = WARN});
      auto id = reactor.add_udp_receiver(
          server, {.port = 6120, .buffer_size = kBufferSize, .on_receive_callback = echo_reversed});
      check(id != espp::SocketReactor::INVALID_ID, "registered on a shared pool");
      check(udp_echo_roundtrip(6120, make_payload(64, 0x33)), "echoes via a shared pool");
      reactor.stop();
    }
  }

  // -------------------------------------------------------------------------
  // 4. TCP listener + stream echo + disconnect (C++-only paths)
  // -------------------------------------------------------------------------
  logger.info("--- tcp listener + stream ---");
  {
    constexpr size_t port = 6130;
    std::atomic_bool closed{false};
    std::mutex clients_mutex;
    std::vector<std::unique_ptr<espp::TcpSocket>> clients;
    {
      espp::SocketReactor reactor({.log_level = WARN});
      espp::TcpSocket server({.log_level = WARN});
      check(server.bind(port) && server.listen(2), "TCP server bind + listen");
      reactor.add_tcp_listener(server, [&](std::unique_ptr<espp::TcpSocket> client) {
        espp::TcpSocket *conn = nullptr;
        {
          std::lock_guard<std::mutex> lk(clients_mutex);
          clients.push_back(std::move(client));
          conn = clients.back().get();
        }
        reactor.add_tcp_stream(
            *conn, [](espp::TcpSocket &c, ByteVector &data) { c.transmit(data); }, kBufferSize,
            [&closed]() { closed = true; });
      });

      espp::TcpSocket client({.log_level = WARN});
      check(client.connect({.ip_address = kLoopback, .port = port}), "TCP client connects");
      auto request = make_payload(128, 0x40);
      ByteVector echo;
      std::atomic_bool got{false};
      client.transmit(request, {.wait_for_response = true,
                                .response_size = kBufferSize,
                                .on_response_callback =
                                    [&](const ByteVector &r) {
                                      echo = r;
                                      got = true;
                                    },
                                .response_timeout = 1s});
      check(got.load() && echo == request, "TCP echo via the reactor");
      client.close();
      check(wait_until([&] { return closed.load(); }), "server observed the client disconnect");
      check(wait_until([&] { return reactor.num_registered() == 1; }),
            "stream auto-removed, only the listener remains");
      reactor.stop();
    }
  }

  // -------------------------------------------------------------------------
  // Summary
  // -------------------------------------------------------------------------
  logger.info("");
  if (total_passed == total_tests) {
    logger.info("======== SocketReactor test: {}/{} checks passed ========", total_passed,
                total_tests);
    logger.info("ALL CHECKS PASSED");
    return 0;
  }
  logger.error("======== SocketReactor test: {}/{} checks passed ========", total_passed,
               total_tests);
  return 1;
}

// A service client's pending-request table is bounded, and a stop() releases
// every waiter.
//
// A request stayed in the pending table until its reply arrived, forever
// when a server took the request and died, or when a reply was lost (a
// VOLATILE reply writer does not replay): a periodic caller grew without
// bound, and a call_future() whose Impl was destroyed at stop() threw
// broken_promise at the waiter instead of the documented std::nullopt.
// Now ServiceConfig::request_timeout drops an entry when it expires and
// max_pending_requests refuses one too many; stop() abandons every entry
// (call() returns nullopt at once, a future resolves nullopt).
//
//  1. Against a deferred server that never replies, a client with a 200 ms
//     timeout and a cap of 8: 50 call_async() never have more than 8 pending,
//     and pending_count() is back to 0 within a second.
//  2. call_future() on that server, then stop(): the future resolves to
//     nullopt (no exception).
//  3. call() with a 10 s timeout returns nullopt within 500 ms of a stop()
//     on another thread.
//  4. The cap holds under concurrency, on both client kinds: with
//     max_pending_requests 1 and nothing answering, 8 threads x 20 call_async
//     get exactly ONE request through in total (the check used to run under
//     an earlier hold of the lock than the insert).
//
// Exits 0 on success, 1 on failure.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <future>
#include <thread>
#include <vector>

#include "rtps_participant.hpp"

using namespace std::chrono_literals;
using Rtps = espp::RtpsParticipant;

namespace {
const std::vector<uint8_t> kRequest{0x00, 0x01, 0x00, 0x00, 1, 0, 0, 0, 0, 0,
                                    0,    0,    2,    0,    0, 0, 0, 0, 0, 0};
constexpr const char *kService = "/black_hole";
constexpr const char *kType = "example_interfaces::srv::dds_::AddTwoInts";
} // namespace

int main() {
  Rtps server({.log_level = espp::Logger::Verbosity::ERROR});
  Rtps client({.log_level = espp::Logger::Verbosity::ERROR});
  if (!server.start() || !client.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }
  std::atomic<int> requests_seen{0};
  // A deferred handler that drops its responder: the request is taken and
  // never answered.
  if (!server.add_service_server_deferred(
          {.service = kService, .type_name = kType},
          [&requests_seen](std::span<const uint8_t>, Rtps::ServiceResponder) {
            requests_seen.fetch_add(1);
          })) {
    std::printf("FAIL: add_service_server_deferred\n");
    return 1;
  }
  auto cli = client.add_service_client({.service = kService,
                                        .type_name = kType,
                                        .request_timeout = 200ms,
                                        .max_pending_requests = 8});
  if (!cli || !cli->wait_for_server(10s)) {
    std::printf("FAIL: client / wait_for_server\n");
    return 1;
  }

  // 1. Bounded while nobody answers.
  std::size_t max_pending = 0;
  int accepted = 0;
  for (int i = 0; i < 50; ++i) {
    if (cli->call_async(kRequest, [](std::span<const uint8_t>) {})) {
      ++accepted;
    }
    max_pending = std::max(max_pending, cli->pending_count());
  }
  std::printf("50 call_async: %d accepted, pending peaked at %zu\n", accepted, max_pending);
  if (max_pending > 8) {
    std::printf("FAIL: pending table exceeded max_pending_requests (8): %zu\n", max_pending);
    return 1;
  }
  if (accepted < 8) {
    std::printf("FAIL: fewer than 8 requests accepted while the table had room\n");
    return 1;
  }
  const auto drain_deadline = std::chrono::steady_clock::now() + 1500ms;
  while (cli->pending_count() != 0 && std::chrono::steady_clock::now() < drain_deadline) {
    std::this_thread::sleep_for(20ms);
  }
  if (cli->pending_count() != 0) {
    std::printf("FAIL: expired requests were not dropped (pending %zu)\n", cli->pending_count());
    return 1;
  }
  // Expired entries go, and the table takes new requests again.
  if (!cli->call_async(kRequest, [](std::span<const uint8_t>) {})) {
    std::printf("FAIL: a request refused after the table drained\n");
    return 1;
  }
  std::printf("server saw %d requests\n", requests_seen.load());

  // 4. The cap under concurrency: one request in total gets through.
  {
    // A second never-answering service (a client per service name).
    if (!server.add_service_server_deferred(
            {.service = "/black_hole_capped", .type_name = kType},
            [](std::span<const uint8_t>, Rtps::ServiceResponder) {})) {
      std::printf("FAIL: add_service_server_deferred (capped)\n");
      return 1;
    }
    auto one = client.add_service_client({.service = "/black_hole_capped",
                                          .type_name = kType,
                                          .request_timeout = 30s,
                                          .max_pending_requests = 1});
    auto one_native = client.add_native_service_client({.service = "/nobody_native",
                                                        .type_name = "espp::native::Nobody",
                                                        .request_timeout = 30s,
                                                        .max_pending_requests = 1});
    if (!one || !one_native || !one->wait_for_server(10s)) {
      std::printf("FAIL: capped clients\n");
      return 1;
    }
    std::atomic<int> through_ros{0};
    std::atomic<int> through_native{0};
    std::atomic<bool> go{false};
    std::vector<std::thread> threads;
    for (int t = 0; t < 8; ++t) {
      threads.emplace_back([&]() {
        while (!go.load()) {
        }
        for (int i = 0; i < 20; ++i) {
          if (one->call_async(kRequest, [](std::span<const uint8_t>) {})) {
            through_ros.fetch_add(1);
          }
          if (one_native->call_async(kRequest, [](std::span<const uint8_t>) {})) {
            through_native.fetch_add(1);
          }
        }
      });
    }
    go = true;
    for (auto &t : threads) {
      t.join();
    }
    std::printf("cap 1, 8 threads x 20: %d ROS and %d native requests got through\n",
                through_ros.load(), through_native.load());
    if (through_ros.load() != 1 || one->pending_count() != 1 || through_native.load() != 1 ||
        one_native->pending_count() != 1) {
      std::printf("FAIL: max_pending_requests exceeded under concurrency\n");
      return 1;
    }
  }

  // 2. A future outlives its request: nullopt, not broken_promise.
  auto future = cli->call_future(kRequest);
  // 3. A blocked call() is released by stop().
  std::atomic<bool> call_returned{false};
  std::atomic<bool> call_was_nullopt{false};
  std::thread caller([&]() {
    const auto reply = cli->call(kRequest, 10s);
    call_was_nullopt = !reply.has_value();
    call_returned = true;
  });
  std::this_thread::sleep_for(100ms); // let the call() get queued and block
  const auto stop_at = std::chrono::steady_clock::now();
  client.stop();
  const auto waited_deadline = std::chrono::steady_clock::now() + 500ms;
  while (!call_returned.load() && std::chrono::steady_clock::now() < waited_deadline) {
    std::this_thread::sleep_for(5ms);
  }
  const auto call_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - stop_at)
                           .count();
  bool ok = true;
  if (!call_returned.load()) {
    std::printf("FAIL: call() did not return within 500 ms of stop()\n");
    ok = false;
  } else if (!call_was_nullopt.load()) {
    std::printf("FAIL: call() returned a reply from a server that never answered\n");
    ok = false;
  } else {
    std::printf("call() returned nullopt %lld ms after stop()\n", static_cast<long long>(call_ms));
  }
  caller.join();
  try {
    if (future.wait_for(500ms) != std::future_status::ready) {
      std::printf("FAIL: the future is still pending after stop()\n");
      ok = false;
    } else if (future.get().has_value()) {
      std::printf("FAIL: the future carries a reply from a server that never answered\n");
      ok = false;
    } else {
      std::printf("call_future() resolved to nullopt after stop()\n");
    }
  } catch (const std::future_error &e) {
    std::printf("FAIL: the future threw (%s) instead of resolving nullopt\n", e.what());
    ok = false;
  }
  server.stop();
  if (!ok) {
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}

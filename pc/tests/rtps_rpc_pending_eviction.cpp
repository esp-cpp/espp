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
//  5. call_async(request, on_reply, on_abandon), both client kinds: on_abandon
//     runs exactly once when the request expires (request_timeout) and exactly
//     once when stop() abandons it, and never for a request that was answered
//     (the echo service) - on_reply and on_abandon are mutually exclusive.
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

  // 5. on_abandon: once on expiry, once on stop(), never after a reply.
  if (!server.add_service_server({.service = "/echo", .type_name = kType},
                                 [](std::span<const uint8_t> req) {
                                   return std::vector<uint8_t>(req.begin(), req.end());
                                 }) ||
      !server.add_native_service_server(
          {.service = "/echo_native", .type_name = "espp::native::Echo"},
          [](std::span<const uint8_t> req) {
            return std::vector<uint8_t>(req.begin(), req.end());
          })) {
    std::printf("FAIL: echo servers\n");
    return 1;
  }
  auto echo = client.add_service_client({.service = "/echo", .type_name = kType});
  auto echo_native = client.add_native_service_client(
      {.service = "/echo_native", .type_name = "espp::native::Echo"});
  auto hole_native = client.add_native_service_client(
      {.service = "/hole_native", .type_name = "espp::native::Hole", .request_timeout = 200ms});
  if (!echo || !echo_native || !hole_native || !echo->wait_for_server(10s)) {
    std::printf("FAIL: abandon-case clients\n");
    return 1;
  }
  std::atomic<int> expired_ros{0}, expired_native{0};
  std::atomic<int> replied_ros{0}, replied_native{0};
  std::atomic<int> echoed_abandon{0};
  // Answered requests: on_reply, never on_abandon.
  if (!echo->call_async(
          kRequest, [&replied_ros](std::span<const uint8_t>) { replied_ros.fetch_add(1); },
          [&echoed_abandon]() { echoed_abandon.fetch_add(1); }) ||
      !echo_native->call_async(
          kRequest, [&replied_native](std::span<const uint8_t>) { replied_native.fetch_add(1); },
          [&echoed_abandon]() { echoed_abandon.fetch_add(1); })) {
    std::printf("FAIL: echo call_async\n");
    return 1;
  }
  // Unanswered requests with a 200 ms timeout: on_abandon once, on expiry.
  if (!cli->call_async(
          kRequest, [](std::span<const uint8_t>) {},
          [&expired_ros]() { expired_ros.fetch_add(1); }) ||
      !hole_native->call_async(
          kRequest, [](std::span<const uint8_t>) {},
          [&expired_native]() { expired_native.fetch_add(1); })) {
    std::printf("FAIL: black-hole call_async\n");
    return 1;
  }
  {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while ((replied_ros.load() < 1 || replied_native.load() < 1 || expired_ros.load() < 1 ||
            expired_native.load() < 1) &&
           std::chrono::steady_clock::now() < deadline) {
      (void)cli->pending_count(); // a touch prunes the expired entries
      (void)hole_native->pending_count();
      std::this_thread::sleep_for(20ms);
    }
  }
  std::this_thread::sleep_for(300ms); // room for a second (wrong) callback
  (void)cli->pending_count();
  (void)hole_native->pending_count();
  std::printf("on_abandon: expired ros=%d native=%d, replied ros=%d native=%d, abandon after "
              "reply=%d\n",
              expired_ros.load(), expired_native.load(), replied_ros.load(), replied_native.load(),
              echoed_abandon.load());
  if (expired_ros.load() != 1 || expired_native.load() != 1 || replied_ros.load() != 1 ||
      replied_native.load() != 1 || echoed_abandon.load() != 0) {
    std::printf("FAIL: on_abandon did not fire exactly once on expiry, or fired after a reply\n");
    return 1;
  }
  // ... and once on stop(), for a request still pending then (checked after).
  std::atomic<int> stopped_ros{0}, stopped_native{0};
  // Their own service names: a participant holds one client per service.
  if (!server.add_service_server_deferred(
          {.service = "/black_hole_stop", .type_name = kType},
          [](std::span<const uint8_t>, Rtps::ServiceResponder) {})) {
    std::printf("FAIL: add_service_server_deferred (stop case)\n");
    return 1;
  }
  auto long_native = client.add_native_service_client(
      {.service = "/hole_native_stop", .type_name = "espp::native::Hole", .request_timeout = 30s});
  auto long_ros = client.add_service_client(
      {.service = "/black_hole_stop", .type_name = kType, .request_timeout = 30s});
  if (!long_native || !long_ros || !long_ros->wait_for_server(10s) ||
      !long_ros->call_async(
          kRequest, [](std::span<const uint8_t>) {},
          [&stopped_ros]() { stopped_ros.fetch_add(1); }) ||
      !long_native->call_async(
          kRequest, [](std::span<const uint8_t>) {},
          [&stopped_native]() { stopped_native.fetch_add(1); })) {
    std::printf("FAIL: stop-case call_async\n");
    return 1;
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
  (void)long_ros->pending_count();
  (void)long_native->pending_count();
  std::printf("on_abandon at stop(): ros=%d native=%d\n", stopped_ros.load(),
              stopped_native.load());
  if (stopped_ros.load() != 1 || stopped_native.load() != 1) {
    std::printf("FAIL: on_abandon did not fire exactly once at stop()\n");
    ok = false;
  }
  if (!ok) {
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}

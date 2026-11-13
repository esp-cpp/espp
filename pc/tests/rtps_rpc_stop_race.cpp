// A service client handle is safe to use while, and after, its participant
// stops.
//
// ServiceClient::server_count(), wait_for_server() and call_async() walk the
// engine's request writer and reply reader. stop() frees both (phase 5). The
// client paths used to dereference them with no guard: a thread polling
// wait_for_server() while another thread stop()ped the participant, or a
// handle kept past stop() (its shared_ptr keeps the Impl alive), read freed
// memory. Now every engine touch from a client is pinned as an engine op
// (stop() waits for it) and checked against the run's liveness token (a
// handle from an earlier run is refused), so the calls return 0 / false.
//
//  1. One thread hammers server_count / wait_for_server / call_async on the
//     current client while the main thread stops and restarts the
//     participant 30 times, re-adding the client each time. No crash.
//  2. A handle kept from before a stop(): server_count() is 0, call_async()
//     and wait_for_server() are false, also after the participant restarted.
//  3. A handle (ROS and native) kept past the participant's DESTRUCTION: the
//     same refusals, silently, and a call_future() left pending resolves to
//     nullopt - the client's engine access is gated by the liveness token the
//     handle holds, never by the participant pointer alone (run under ASan
//     to see the old use-after-free).
//  4. call_future() hammered on both client kinds across stop/start rounds:
//     completion is single-shot (reply, abandonment at stop(), or a failed
//     send never fulfil the same promise twice - a std::future_error here
//     terminates the hammer thread).
//
// Exits 0 on success, 1 on failure.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "rtps_participant.hpp"

using namespace std::chrono_literals;

int main() {
  espp::RtpsParticipant participant({.log_level = espp::Logger::Verbosity::ERROR});
  if (!participant.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }
  const espp::RtpsParticipant::ServiceConfig cfg{
      .service = "/stop_race", .type_name = "example_interfaces::srv::dds_::AddTwoInts"};
  const espp::RtpsParticipant::ServiceConfig native_cfg{.service = "/stop_race_native",
                                                        .type_name = "espp::native::Add"};
  std::mutex client_mutex;
  std::shared_ptr<espp::RtpsParticipant::ServiceClient> client =
      participant.add_service_client(cfg);
  std::shared_ptr<espp::RtpsParticipant::NativeServiceClient> native_client =
      participant.add_native_service_client(native_cfg);
  if (!client || !native_client) {
    std::printf("FAIL: add_service_client\n");
    return 1;
  }
  const auto current = [&]() {
    std::lock_guard<std::mutex> lock(client_mutex);
    return client;
  };
  const auto current_native = [&]() {
    std::lock_guard<std::mutex> lock(client_mutex);
    return native_client;
  };

  const std::vector<uint8_t> request{0x00, 0x01, 0x00, 0x00, 1, 0, 0, 0, 0, 0,
                                     0,    0,    2,    0,    0, 0, 0, 0, 0, 0};
  std::atomic<bool> run{true};
  std::atomic<unsigned> calls{0};
  std::thread hammer([&]() {
    while (run.load()) {
      auto c = current();
      if (!c) {
        continue;
      }
      (void)c->server_count();
      (void)c->wait_for_server(5ms);
      (void)c->call_async(request, [](std::span<const uint8_t>) {});
      // 4. futures, on both client kinds: a double completion would throw
      // std::future_error out of this thread (and terminate the test).
      (void)c->call_future(request);
      if (auto n = current_native()) {
        (void)n->call_future(request);
      }
      calls.fetch_add(1);
    }
  });

  std::shared_ptr<espp::RtpsParticipant::ServiceClient> kept = client;
  for (int round = 0; round < 30; ++round) {
    participant.stop();
    if (!participant.start()) {
      std::printf("FAIL: restart %d\n", round);
      run = false;
      hammer.join();
      return 1;
    }
    auto fresh = participant.add_service_client(cfg);
    auto fresh_native = participant.add_native_service_client(native_cfg);
    if (!fresh || !fresh_native) {
      std::printf("FAIL: add_service_client after restart %d\n", round);
      run = false;
      hammer.join();
      return 1;
    }
    std::lock_guard<std::mutex> lock(client_mutex);
    client = fresh;
    native_client = fresh_native;
  }
  run = false;
  hammer.join();
  std::printf("30 restarts, %u hammer iterations\n", calls.load());

  // 2. The handle from the first run, with the participant running again.
  bool ok = true;
  if (kept->server_count() != 0) {
    std::printf("FAIL: a client from an earlier run reports matched servers\n");
    ok = false;
  }
  if (kept->call_async(request, [](std::span<const uint8_t>) {})) {
    std::printf("FAIL: a client from an earlier run accepted a request\n");
    ok = false;
  }
  const auto t0 = std::chrono::steady_clock::now();
  if (kept->wait_for_server(5s)) {
    std::printf("FAIL: a client from an earlier run reports a server\n");
    ok = false;
  }
  if (std::chrono::steady_clock::now() - t0 > 1s) {
    std::printf("FAIL: wait_for_server on a dead handle waited out its timeout\n");
    ok = false;
  }
  // ... and after the final stop, the current handle too.
  participant.stop();
  if (current()->server_count() != 0 || current()->call_async(request, nullptr)) {
    std::printf("FAIL: a client of a stopped participant is live\n");
    ok = false;
  }

  // 3. Handles that outlive their participant entirely.
  {
    auto doomed = std::make_unique<espp::RtpsParticipant>(
        espp::RtpsParticipant::Config{.log_level = espp::Logger::Verbosity::ERROR});
    if (!doomed->start()) {
      std::printf("FAIL: start (doomed)\n");
      return 1;
    }
    auto ros_handle = doomed->add_service_client(cfg);
    auto native_handle = doomed->add_native_service_client(native_cfg);
    if (!ros_handle || !native_handle) {
      std::printf("FAIL: clients on the doomed participant\n");
      return 1;
    }
    auto ros_future = ros_handle->call_future(request);       // refused: no server matched
    auto native_future = native_handle->call_future(request); // queued, never answered
    doomed.reset(); // stop() + destruction; the handles stay with us
    if (ros_handle->server_count() != 0 || ros_handle->call_async(request, nullptr) ||
        ros_handle->pending_count() != 0) {
      std::printf("FAIL: a handle of a destroyed participant is live (ROS)\n");
      ok = false;
    }
    const auto t1 = std::chrono::steady_clock::now();
    if (ros_handle->wait_for_server(5s) || std::chrono::steady_clock::now() - t1 > 1s) {
      std::printf("FAIL: wait_for_server on a handle of a destroyed participant\n");
      ok = false;
    }
    if (native_handle->call_async(request, nullptr) || native_handle->pending_count() != 0) {
      std::printf("FAIL: a handle of a destroyed participant is live (native)\n");
      ok = false;
    }
    try {
      if (ros_future.wait_for(0s) != std::future_status::ready || ros_future.get().has_value() ||
          native_future.wait_for(0s) != std::future_status::ready ||
          native_future.get().has_value()) {
        std::printf("FAIL: a future left at destruction did not resolve to nullopt\n");
        ok = false;
      }
    } catch (const std::future_error &e) {
      std::printf("FAIL: a future left at destruction threw (%s)\n", e.what());
      ok = false;
    }
    if (ros_handle->call_future(request).get().has_value() ||
        native_handle->call_future(request).get().has_value()) {
      std::printf("FAIL: call_future on a handle of a destroyed participant\n");
      ok = false;
    }
  }
  if (!ok) {
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}

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
//
// Exits 0 on success, 1 on failure.

#include <atomic>
#include <chrono>
#include <cstdio>
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
  std::mutex client_mutex;
  std::shared_ptr<espp::RtpsParticipant::ServiceClient> client =
      participant.add_service_client(cfg);
  if (!client) {
    std::printf("FAIL: add_service_client\n");
    return 1;
  }
  const auto current = [&]() {
    std::lock_guard<std::mutex> lock(client_mutex);
    return client;
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
    if (!fresh) {
      std::printf("FAIL: add_service_client after restart %d\n", round);
      run = false;
      hammer.join();
      return 1;
    }
    std::lock_guard<std::mutex> lock(client_mutex);
    client = fresh;
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
  if (!ok) {
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}

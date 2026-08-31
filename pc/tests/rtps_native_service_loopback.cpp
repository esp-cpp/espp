// In-process native (espp<->espp) service loopback: exercises the lean
// request/reply protocol (20-byte in-band correlation header over plain pub/sub,
// no ROS mangling / inline QoS) with all three client call styles. Uses an
// add_two_ints service (two int64 in, one int64 out). Exits 0 iff sync, async,
// and future calls all return the correlated sum.

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <thread>
#include <vector>

#include "rtps_participant.hpp"

using namespace std::chrono_literals;

namespace {
constexpr uint8_t kEncap[4] = {0x00, 0x01, 0x00, 0x00};
void put_i64(std::vector<uint8_t> &v, int64_t x) {
  for (int i = 0; i < 8; ++i)
    v.push_back(static_cast<uint8_t>((x >> (8 * i)) & 0xFF));
}
int64_t get_i64(std::span<const uint8_t> p, size_t off) {
  int64_t x = 0;
  for (int i = 0; i < 8; ++i)
    x |= static_cast<int64_t>(p[off + i]) << (8 * i);
  return x;
}
std::vector<uint8_t> request(int64_t a, int64_t b) {
  std::vector<uint8_t> v(kEncap, kEncap + 4);
  put_i64(v, a);
  put_i64(v, b);
  return v;
}
std::vector<uint8_t> response(int64_t sum) {
  std::vector<uint8_t> v(kEncap, kEncap + 4);
  put_i64(v, sum);
  return v;
}
} // namespace

int main() {
  const espp::RtpsParticipant::ServiceConfig cfg{"/add_two_ints", "espp::native::AddTwoInts"};

  espp::RtpsParticipant server({.log_level = espp::Logger::Verbosity::WARN});
  espp::RtpsParticipant client({.log_level = espp::Logger::Verbosity::WARN});
  if (!server.start() || !client.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }

  if (!server.add_native_service_server(cfg, [](std::span<const uint8_t> req) {
        if (req.size() < 4 + 16)
          return std::vector<uint8_t>{};
        return response(get_i64(req, 4) + get_i64(req, 12));
      })) {
    std::printf("FAIL: add_native_service_server\n");
    return 1;
  }
  auto call = client.add_native_service_client(cfg);
  if (!call) {
    std::printf("FAIL: add_native_service_client\n");
    return 1;
  }

  std::this_thread::sleep_for(2s); // SEDP match on es_rq/es_rr

  // Sync.
  bool sync_ok = false;
  {
    auto r = call->call(request(7, 35), 10s);
    sync_ok = r && r->size() >= 12 && get_i64(*r, 4) == 42;
    std::printf("native sync: 7+35 => %lld %s\n", r ? (long long)get_i64(*r, 4) : -1,
                sync_ok ? "ok" : "FAIL");
  }
  // Async.
  bool async_ok = false;
  {
    // The callback state is SHARED and captured by value: call_async retains
    // the callback, so if the wait below times out a late reply may still
    // invoke it - stack-captured references would then be use-after-scope.
    // The shared_ptr keeps the state alive as long as the callback exists, and
    // notifying under the lock orders any cv destruction after the notify.
    struct AsyncState {
      std::mutex m;
      std::condition_variable cv;
      bool done = false;
      int64_t got = 0;
    };
    auto st = std::make_shared<AsyncState>();
    call->call_async(request(1000, 337), [st](std::span<const uint8_t> r) {
      std::lock_guard<std::mutex> lk(st->m);
      if (r.size() >= 12) {
        st->got = get_i64(r, 4);
        st->done = true;
      }
      st->cv.notify_one();
    });
    std::unique_lock<std::mutex> lk(st->m);
    if (st->cv.wait_for(lk, 10s, [&] { return st->done; }))
      async_ok = (st->got == 1337);
    std::printf("native async: 1000+337 => %lld %s\n", (long long)st->got,
                async_ok ? "ok" : "FAIL");
  }
  // Future.
  bool future_ok = false;
  {
    auto fut = call->call_future(request(500, 500));
    if (fut.wait_for(10s) == std::future_status::ready) {
      auto r = fut.get();
      future_ok = r && r->size() >= 12 && get_i64(*r, 4) == 1000;
    }
    std::printf("native future: 500+500 => %s\n", future_ok ? "ok" : "FAIL");
  }

  server.stop();
  client.stop();

  if (sync_ok && async_ok && future_ok) {
    std::printf("PASS\n");
    return 0;
  }
  std::printf("FAIL\n");
  return 1;
}

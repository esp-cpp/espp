// In-process service (RMI) loopback: one participant hosts an add_two_ints
// service server, another calls it as a client. Exercises the full M1 request/
// reply path end to end - name mangling, the related_sample_identity inline QoS
// emit on both request and reply, the reader capturing it, and the pending-
// request correlation - without needing ROS 2. The wire encoding matches
// example_interfaces/srv/AddTwoInts (two int64 in, one int64 out) so the same
// payloads are valid against a real ROS 2 node in the docker interop leg.
//
// Exits 0 iff the correlated reply carries the expected sum.

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <optional>
#include <span>
#include <thread>
#include <vector>

#include "rtps_participant.hpp"

using namespace std::chrono_literals;

namespace {
using ServiceConfig = espp::RtpsParticipant::ServiceConfig;

// CDR_LE encapsulation header (little-endian, no options).
constexpr uint8_t kEncap[4] = {0x00, 0x01, 0x00, 0x00};

void put_i64(std::vector<uint8_t> &v, int64_t x) {
  for (int i = 0; i < 8; ++i) {
    v.push_back(static_cast<uint8_t>((x >> (8 * i)) & 0xFF));
  }
}
int64_t get_i64(std::span<const uint8_t> p, size_t off) {
  int64_t x = 0;
  for (int i = 0; i < 8; ++i) {
    x |= static_cast<int64_t>(p[off + i]) << (8 * i);
  }
  return x;
}

std::vector<uint8_t> encode_request(int64_t a, int64_t b) {
  std::vector<uint8_t> v(kEncap, kEncap + 4);
  put_i64(v, a);
  put_i64(v, b);
  return v;
}
std::vector<uint8_t> encode_response(int64_t sum) {
  std::vector<uint8_t> v(kEncap, kEncap + 4);
  put_i64(v, sum);
  return v;
}
} // namespace

int main() {
  const ServiceConfig cfg{"/add_two_ints", "example_interfaces::srv::dds_::AddTwoInts"};

  espp::RtpsParticipant server({.log_level = espp::Logger::Verbosity::WARN});
  espp::RtpsParticipant client({.log_level = espp::Logger::Verbosity::WARN});
  if (!server.start() || !client.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }

  // Server: sum = a + b.
  if (!server.add_service_server(cfg, [](std::span<const uint8_t> req) -> std::vector<uint8_t> {
        if (req.size() < 4 + 16) {
          return {};
        }
        const int64_t a = get_i64(req, 4);
        const int64_t b = get_i64(req, 12);
        return encode_response(a + b);
      })) {
    std::printf("FAIL: add_service_server\n");
    return 1;
  }

  auto call = client.add_service_client(cfg);
  if (!call) {
    std::printf("FAIL: add_service_client\n");
    return 1;
  }

  std::this_thread::sleep_for(2s); // SEDP match on rq/ + rr/ topics

  // Synchronous call.
  const int64_t a = 7, b = 35;
  auto reply = call->call(encode_request(a, b), 10s);
  bool ok = false;
  if (!reply.has_value()) {
    std::printf("FAIL: call timed out (no correlated reply)\n");
  } else if (reply->size() < 4 + 8) {
    std::printf("FAIL: reply too short (%zu bytes)\n", reply->size());
  } else {
    const int64_t sum = get_i64(*reply, 4);
    ok = (sum == a + b);
    std::printf("sync call: %lld + %lld = %lld (expected %lld) => %s\n", (long long)a, (long long)b,
                (long long)sum, (long long)(a + b), ok ? "ok" : "MISMATCH");
  }

  // Asynchronous call: a second request must correlate independently.
  bool async_ok = false;
  {
    std::mutex m;
    std::condition_variable cv;
    bool done = false;
    int64_t got = 0;
    const int64_t a2 = 1000, b2 = 337;
    if (call->call_async(encode_request(a2, b2), [&](std::span<const uint8_t> rep) {
          if (rep.size() >= 4 + 8) {
            std::lock_guard<std::mutex> lk(m);
            got = get_i64(rep, 4);
            done = true;
          }
          cv.notify_one();
        })) {
      std::unique_lock<std::mutex> lk(m);
      if (cv.wait_for(lk, 10s, [&] { return done; })) {
        async_ok = (got == a2 + b2);
      }
      std::printf("async call: got %lld (expected %lld) => %s\n", (long long)got,
                  (long long)(a2 + b2), async_ok ? "ok" : "MISMATCH/timeout");
    }
  }

  // Deferred server: a separate service whose handler replies from another
  // thread after a delay (exercises add_service_server_deferred + ServiceResponder).
  bool deferred_ok = false;
  {
    const char *dsvc = "/add_two_ints_deferred";
    std::vector<std::thread> workers;
    std::mutex wm;
    server.add_service_server_deferred(
        {dsvc, "example_interfaces::srv::dds_::AddTwoInts"},
        [&](std::span<const uint8_t> req, espp::RtpsParticipant::ServiceResponder responder) {
          std::vector<uint8_t> r(req.begin(), req.end());
          std::lock_guard<std::mutex> lk(wm);
          workers.emplace_back([r, responder]() {
            std::this_thread::sleep_for(300ms); // reply later, off the worker thread
            if (r.size() >= 4 + 16) {
              responder.reply(encode_response(get_i64(r, 4) + get_i64(r, 12)));
            }
          });
        });
    auto dcall = client.add_service_client({dsvc, "example_interfaces::srv::dds_::AddTwoInts"});
    std::this_thread::sleep_for(2s); // discovery for the new endpoints
    if (dcall) {
      auto dreply = dcall->call(encode_request(11, 31), 10s);
      if (dreply.has_value() && dreply->size() >= 4 + 8) {
        deferred_ok = (get_i64(*dreply, 4) == 42);
      }
      std::printf("deferred call: 11 + 31 = %lld => %s\n",
                  reply.has_value() ? (long long)get_i64(*reply, 4) : -1,
                  deferred_ok ? "ok" : "MISMATCH/timeout");
    }
    for (auto &t : workers) {
      if (t.joinable())
        t.join();
    }
  }

  // Future-based call: a third request must correlate independently.
  bool future_ok = false;
  {
    const int64_t a3 = 500, b3 = 500;
    auto fut = call->call_future(encode_request(a3, b3));
    if (fut.wait_for(10s) == std::future_status::ready) {
      auto reply3 = fut.get();
      if (reply3.has_value() && reply3->size() >= 4 + 8) {
        const int64_t sum = get_i64(*reply3, 4);
        future_ok = (sum == a3 + b3);
        std::printf("future call: got %lld (expected %lld) => %s\n", (long long)sum,
                    (long long)(a3 + b3), future_ok ? "ok" : "MISMATCH");
      }
    } else {
      std::printf("future call: timed out\n");
    }
  }

  server.stop();
  client.stop();

  if (ok && async_ok && future_ok && deferred_ok) {
    std::printf("PASS\n");
    return 0;
  }
  std::printf("FAIL\n");
  return 1;
}

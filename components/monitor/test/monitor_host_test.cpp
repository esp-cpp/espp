// Host-buildable unit tests for the MonitorService wire codec
// (include/detail/monitor_protocol.hpp). Build & run with:
//   c++ -std=c++20 -I../include -I../../stream_frame/include monitor_host_test.cpp -o test &&
//   ./test
//
// The codec needs no ESP-IDF headers; these are golden encode/decode tests so
// the browser console and any host tool can rely on the byte layout.

#include <cstdint>
#include <cstdio>
#include <span>
#include <string>
#include <vector>

#include "detail/monitor_protocol.hpp"

namespace mp = espp::detail::monitor_protocol;
namespace sf = espp::stream_frame;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

static void test_heap_roundtrip() {
  std::printf("test_heap_roundtrip\n");
  std::vector<mp::HeapRegion> regions = {
      {.flags = 0x1800,
       .free_bytes = 100000,
       .min_free_bytes = 90000,
       .largest_free_block = 65536,
       .allocated_bytes = 200000,
       .total_size = 300000},
      {.flags = 0x400,
       .free_bytes = 1,
       .min_free_bytes = 2,
       .largest_free_block = 3,
       .allocated_bytes = 4,
       .total_size = 5},
  };
  const auto p = mp::encode_heap(regions);
  CHECK(p.size() == 1 + 2 * 24);
  CHECK(p[0] == 2);
  // golden: first region, flags 0x1800 little-endian, free 100000 = 0x000186A0
  CHECK(p[1] == 0x00 && p[2] == 0x18 && p[3] == 0x00 && p[4] == 0x00);
  CHECK(p[5] == 0xA0 && p[6] == 0x86 && p[7] == 0x01 && p[8] == 0x00);
  const auto d = mp::decode_heap(p);
  CHECK(d && d->size() == 2);
  if (d && d->size() == 2) {
    CHECK((*d)[0].flags == 0x1800 && (*d)[0].free_bytes == 100000 && (*d)[0].total_size == 300000);
    CHECK((*d)[1].largest_free_block == 3 && (*d)[1].allocated_bytes == 4);
  }
  // truncated: declared 2 regions, only one present
  CHECK(!mp::decode_heap(std::span<const uint8_t>(p.data(), 1 + 24)));
  CHECK(!mp::decode_heap({}));
  // empty list
  const auto e = mp::encode_heap({});
  CHECK(e.size() == 1 && e[0] == 0);
  CHECK(mp::decode_heap(e) && mp::decode_heap(e)->empty());
}

static void test_tasks_roundtrip_and_cap() {
  std::printf("test_tasks_roundtrip_and_cap\n");
  std::vector<mp::TaskEntry> tasks = {
      {.name = "main", .cpu_percent = 12, .high_water_mark = 3000, .priority = 1, .core_id = 0},
      {.name = "IDLE1", .cpu_percent = 88, .high_water_mark = 500, .priority = 0, .core_id = 1},
      {.name = "tiT", .cpu_percent = 0, .high_water_mark = 1234, .priority = 18, .core_id = -1},
  };
  size_t encoded = 0;
  const auto p = mp::encode_tasks(tasks, sf::kMaxPayloadSize, &encoded);
  CHECK(encoded == 3 && p[0] == 3);
  // golden first record: [4]"main"[12][0xB8 0x0B 0 0][1][0]
  const uint8_t golden[] = {3, 4, 'm', 'a', 'i', 'n', 12, 0xB8, 0x0B, 0, 0, 1, 0};
  CHECK(p.size() >= sizeof(golden) && std::equal(golden, golden + sizeof(golden), p.begin()));
  const auto d = mp::decode_tasks(p);
  CHECK(d && d->size() == 3);
  if (d && d->size() == 3) {
    CHECK((*d)[1].name == "IDLE1" && (*d)[1].cpu_percent == 88 && (*d)[1].core_id == 1);
    CHECK((*d)[2].core_id == -1 && (*d)[2].priority == 18 && (*d)[2].high_water_mark == 1234);
  }
  // truncated payload is rejected
  CHECK(!mp::decode_tasks(std::span<const uint8_t>(p.data(), p.size() - 1)));
  // the cap drops whole entries from the end
  size_t n2 = 0;
  const auto capped = mp::encode_tasks(tasks, 1 + mp::task_entry_size("main") + 3, &n2);
  CHECK(n2 == 1 && capped[0] == 1 && capped.size() == 1 + mp::task_entry_size("main"));
  // the service caps the payload so the whole frame (9-byte header + CRC)
  // fits its max_frame_bytes: with the default 4096 that is a 4083-byte payload
  const size_t service_cap = 4096 - (sf::kHeaderSize + sf::kCrcSize);
  CHECK(service_cap == 4083);
  size_t n4 = 0;
  std::vector<mp::TaskEntry> lots(400, {.name = std::string(16, 'y'),
                                        .cpu_percent = 1,
                                        .high_water_mark = 1,
                                        .priority = 1,
                                        .core_id = 0});
  const auto fitted = mp::encode_tasks(lots, service_cap, &n4);
  CHECK(fitted.size() <= service_cap && n4 == fitted[0] && n4 < 400);
  CHECK(mp::build_frame(mp::Type::Tasks, fitted).size() <= 4096);
  // a 4096-byte payload never overflows: 500 tasks with long names
  std::vector<mp::TaskEntry> many(500, {.name = std::string(40, 'x'),
                                        .cpu_percent = 1,
                                        .high_water_mark = 1,
                                        .priority = 1,
                                        .core_id = 0});
  size_t n3 = 0;
  const auto big = mp::encode_tasks(many, sf::kMaxPayloadSize, &n3);
  CHECK(big.size() <= sf::kMaxPayloadSize && n3 < 500 && n3 == big[0]);
  CHECK(mp::decode_tasks(big) && mp::decode_tasks(big)->size() == n3);
  // a name longer than 255 bytes is truncated on the wire
  std::vector<mp::TaskEntry> longname = {{.name = std::string(300, 'n'),
                                          .cpu_percent = 0,
                                          .high_water_mark = 0,
                                          .priority = 0,
                                          .core_id = 0}};
  const auto ln = mp::encode_tasks(longname);
  CHECK(mp::decode_tasks(ln) && (*mp::decode_tasks(ln))[0].name.size() == 255);
}

static void test_set_stream_and_replies() {
  std::printf("test_set_stream_and_replies\n");
  const auto p = mp::encode_set_stream(true, 250, mp::kStreamHeap | mp::kStreamTasks);
  const uint8_t golden[] = {1, 0xFA, 0x00, 0x03};
  CHECK(p.size() == 4 && std::equal(golden, golden + 4, p.begin()));
  const auto r = mp::decode_set_stream(p);
  CHECK(r && r->enable && r->period_ms == 250 && r->what == 3);
  CHECK(!mp::decode_set_stream(std::span<const uint8_t>(p.data(), 3)));
  CHECK(mp::encode_ok(0x03) == std::vector<uint8_t>{0x03});
  const auto e = mp::encode_error(0x02, 95, "no");
  const uint8_t eg[] = {0x02, 95, 0, 0, 0, 'n', 'o'};
  CHECK(e.size() == 7 && std::equal(eg, eg + 7, e.begin()));
}

static void test_frames() {
  std::printf("test_frames\n");
  // device->host types set the frame reply flag; requests do not
  const auto req = mp::build_frame(mp::Type::GetHeap, {}, 8);
  CHECK(req.size() == 9 + 4 && req[2] == 0x10 && req[3] == 8 && req[4] == 0x01);
  const auto rep = mp::build_frame(mp::Type::Heap, mp::encode_heap({}), 9);
  CHECK(rep[2] == 0x11 && rep[3] == 9 && rep[4] == 0x81);
  sf::StreamParser parser;
  std::vector<uint8_t> stream(req);
  stream.insert(stream.end(), rep.begin(), rep.end());
  const auto frames = parser.feed(stream);
  CHECK(frames.size() == 2 && !frames[0].is_reply() && frames[1].is_reply() &&
        frames[1].module == 9 && frames[1].payload.size() == 1);
}

int main() {
  test_heap_roundtrip();
  test_tasks_roundtrip_and_cap();
  test_set_stream_and_replies();
  test_frames();
  if (g_failures) {
    std::printf("%d FAILURE(S)\n", g_failures);
    return 1;
  }
  std::printf("ALL TESTS PASSED\n");
  return 0;
}

// Host-buildable golden tests for the ODrive legacy native (Fibre) UART *stream*
// framing. Build & run with:
//   c++ -std=c++20 -I../include odrive_native_stream_test.cpp -o stream_test && ./stream_test
//
// These tests exercise espp::detail (stream_frame / StreamDeframer / odrive_crc8)
// directly, so they need no ESP-IDF headers. They freeze the wire framing that
// fibre's serial backend (Firmware/fibre/python/fibre/protocol.py) uses, verified
// against the fw-v0.5.1 reference.

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string>
#include <vector>

#include "detail/odrive_native_stream.hpp"

using espp::detail::odrive_crc8;
using espp::detail::stream_frame;
using espp::detail::StreamDeframer;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

static std::string hex(std::span<const uint8_t> b) {
  static const char *d = "0123456789ABCDEF";
  std::string s;
  for (size_t i = 0; i < b.size(); ++i) {
    if (i)
      s += ' ';
    s += d[b[i] >> 4];
    s += d[b[i] & 0xf];
  }
  return s;
}

static void test_crc8_golden() {
  std::printf("test_crc8_golden\n");
  // crc8("") == init == 0x42
  CHECK(odrive_crc8(std::span<const uint8_t>{}) == 0x42);
  const uint8_t zero = 0x00;
  CHECK(odrive_crc8(std::span<const uint8_t>(&zero, 1)) == 0xca);
  const char *s = "123456789";
  CHECK(odrive_crc8(std::span<const uint8_t>(reinterpret_cast<const uint8_t *>(s), 9)) == 0x8c);
  const uint8_t hdr[2] = {0xAA, 0x0A};
  CHECK(odrive_crc8(std::span<const uint8_t>(hdr, 2)) == 0x53);
}

static void test_frame_golden() {
  std::printf("test_frame_golden\n");
  // The endpoint-0 read request packet from the spec:
  //   seq=0x8080, endpoint=0x8000, output_len=512, offset u32=0, trailer=1
  const std::vector<uint8_t> packet = {0x80, 0x80, 0x00, 0x80, 0x00, 0x02,
                                       0x00, 0x00, 0x00, 0x00, 0x01, 0x00};
  const std::vector<uint8_t> expected = {0xAA, 0x0C, 0xE1, 0x80, 0x80, 0x00, 0x80, 0x00, 0x02,
                                         0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xA3, 0xAB};
  auto framed = stream_frame(packet);
  CHECK(framed == expected);
  if (framed != expected) {
    std::printf("    got: %s\n", hex(framed).c_str());
    std::printf("    exp: %s\n", hex(expected).c_str());
  }

  // Receiver validation trick: crc8 over [sync,len,crc8] == 0.
  const uint8_t hdr3[3] = {framed[0], framed[1], framed[2]};
  CHECK(odrive_crc8(std::span<const uint8_t>(hdr3, 3)) == 0);
  // and crc16 over [packet .. crc16 bytes] == 0.
  std::vector<uint8_t> pk_plus(framed.begin() + 3, framed.end());
  CHECK(espp::detail::odrive_crc16(pk_plus) == 0);
}

static void test_deframe_roundtrip() {
  std::printf("test_deframe_roundtrip\n");
  const std::vector<uint8_t> p1 = {0x80, 0x80, 0x00, 0x80, 0x00, 0x02,
                                   0x00, 0x00, 0x00, 0x00, 0x01, 0x00};
  const std::vector<uint8_t> p2 = {0x81, 0x80, 0x01, 0x80, 0x04, 0x00, 0xEC, 0x59};
  auto f1 = stream_frame(p1);
  auto f2 = stream_frame(p2);

  // Feed both frames concatenated in one push.
  std::vector<uint8_t> both = f1;
  both.insert(both.end(), f2.begin(), f2.end());
  StreamDeframer d;
  auto pkts = d.push(both);
  CHECK(pkts.size() == 2);
  if (pkts.size() == 2) {
    CHECK(pkts[0] == p1);
    CHECK(pkts[1] == p2);
  }
  CHECK(d.buffered() == 0);

  // Byte-at-a-time feed with leading garbage + a spurious 0xAA that fails header
  // CRC -> the deframer must resync and still recover the packet.
  StreamDeframer d2;
  std::vector<std::vector<uint8_t>> got;
  std::vector<uint8_t> stream = {0x00, 0xFF, 0xAA, 0x7F, 0x13}; // junk incl. bad 0xAA header
  stream.insert(stream.end(), f1.begin(), f1.end());
  for (uint8_t b : stream) {
    auto r = d2.push(std::span<const uint8_t>(&b, 1));
    for (auto &pk : r)
      got.push_back(pk);
  }
  CHECK(got.size() == 1);
  if (got.size() == 1)
    CHECK(got[0] == p1);

  // A frame with a corrupted CRC16 trailer must be dropped (no packet yielded).
  StreamDeframer d3;
  auto bad = f1;
  bad.back() ^= 0xFF; // corrupt low CRC16 byte
  auto r3 = d3.push(bad);
  CHECK(r3.empty());
}

int main() {
  test_crc8_golden();
  test_frame_golden();
  test_deframe_roundtrip();
  if (g_failures == 0) {
    std::printf("\nALL STREAM TESTS PASSED\n");
    return 0;
  }
  std::printf("\n%d CHECK(S) FAILED\n", g_failures);
  return 1;
}

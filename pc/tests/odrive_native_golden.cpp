// Golden wire-format test for the ODrive legacy native (Fibre endpoint) protocol
// -- the analogue of rtps_golden for components/odrive_native. It freezes, byte
// for byte, the CRC8 / CRC16 constants, the UART stream frame, and a packet
// round-trip, all verified against the fw-v0.5.1 reference (and proven end-to-end
// by the serial-loopback interop harness in components/odrive_native/interop).
//
// Built by the pc harness (see ../CMakeLists.txt, which adds the odrive_native
// include dir for odrive_native_* targets). Exits 0 when every golden matches.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <string>
#include <vector>

#include "detail/odrive_native_core.hpp"
#include "detail/odrive_native_stream.hpp"

using espp::detail::kProtocolVersion;
using espp::detail::odrive_crc16;
using espp::detail::odrive_crc8;
using espp::detail::OdriveNativeCore;
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

static std::span<const uint8_t> sv(const char *s) {
  return std::span<const uint8_t>(reinterpret_cast<const uint8_t *>(s), std::strlen(s));
}

static void golden_crc8() {
  std::printf("golden_crc8\n");
  CHECK(odrive_crc8(std::span<const uint8_t>{}) == 0x42); // init
  const uint8_t z = 0x00;
  CHECK(odrive_crc8(std::span<const uint8_t>(&z, 1)) == 0xca);
  CHECK(odrive_crc8(sv("123456789")) == 0x8c);
  const uint8_t hdr[2] = {0xAA, 0x0A};
  CHECK(odrive_crc8(std::span<const uint8_t>(hdr, 2)) == 0x53);
}

static void golden_crc16() {
  std::printf("golden_crc16\n");
  // Packet-CRC init (0x1337) -- the UART stream framing / packet trailer of ep 0.
  CHECK(odrive_crc16(std::string_view("")) == 0x1337);
  const uint8_t z = 0x00;
  CHECK(odrive_crc16(std::span<const uint8_t>(&z, 1)) == 0xe150);
  CHECK(odrive_crc16(std::string_view("123456789")) == 0xaa01);
}

static void golden_frame() {
  std::printf("golden_frame\n");
  const std::vector<uint8_t> packet = {0x80, 0x80, 0x00, 0x80, 0x00, 0x02,
                                       0x00, 0x00, 0x00, 0x00, 0x01, 0x00};
  const std::vector<uint8_t> expected = {0xAA, 0x0C, 0xE1, 0x80, 0x80, 0x00, 0x80, 0x00, 0x02,
                                         0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xA3, 0xAB};
  CHECK(stream_frame(packet) == expected);

  // Deframe round-trip (with the CRC16 trailer stripped back to the raw packet).
  StreamDeframer d;
  auto pkts = d.push(expected);
  CHECK(pkts.size() == 1);
  if (pkts.size() == 1)
    CHECK(pkts[0] == packet);
}

static void golden_packet_roundtrip() {
  std::printf("golden_packet_roundtrip\n");
  OdriveNativeCore core;
  float pos = 0.0f;
  core.register_float_property("vbus_voltage", [] { return 24.0f; });
  core.register_float_property(
      "axis0.controller.input_pos", [&] { return pos; },
      [&](float v, std::error_code &ec) {
        pos = v;
        ec.clear();
        return true;
      });

  // The endpoint canary is CRC16 over the JSON seeded with PROTOCOL_VERSION (1),
  // NOT the 0x1337 packet-CRC init -- this is what a real fibre client sends.
  const std::string json = core.json();
  CHECK(core.json_crc() == odrive_crc16(json, kProtocolVersion));
  CHECK(core.json_crc() != odrive_crc16(json)); // and it differs from the 0x1337 init

  // Full frame -> deframe -> process -> reframe -> deframe path for a value write
  // then read-back of endpoint 2 (input_pos), the exact loop the device shim runs.
  const uint16_t crc = core.json_crc();
  const uint16_t ep = 2;
  const float wrote = 12.5f;
  std::vector<uint8_t> req; // [seq][ep|0x8000][out_len][payload][trailer]
  auto put16 = [&](std::vector<uint8_t> &v, uint16_t x) {
    v.push_back(uint8_t(x & 0xff));
    v.push_back(uint8_t(x >> 8));
  };
  put16(req, 0x0080 | 0x0001);
  put16(req, ep | 0x8000);
  put16(req, 4); // want 4 bytes back
  uint8_t fb[4];
  std::memcpy(fb, &wrote, 4);
  req.insert(req.end(), fb, fb + 4);
  put16(req, crc);

  auto framed = stream_frame(req);
  StreamDeframer d;
  auto in = d.push(framed);
  CHECK(in.size() == 1);
  auto resp = core.process_bytes(in[0]);
  CHECK(resp.size() == 2 + 4);
  float rb = 0.0f;
  std::memcpy(&rb, resp.data() + 2, 4);
  CHECK(rb == wrote);

  auto resp_framed = stream_frame(resp);
  StreamDeframer d2;
  auto rp = d2.push(resp_framed);
  CHECK(rp.size() == 1);
  if (rp.size() == 1)
    CHECK(rp[0] == resp);
}

int main() {
  golden_crc8();
  golden_crc16();
  golden_frame();
  golden_packet_roundtrip();
  if (g_failures == 0) {
    std::printf("\nODRIVE_NATIVE GOLDEN: ALL PASSED\n");
    return 0;
  }
  std::printf("\nODRIVE_NATIVE GOLDEN: %d CHECK(S) FAILED\n", g_failures);
  return 1;
}

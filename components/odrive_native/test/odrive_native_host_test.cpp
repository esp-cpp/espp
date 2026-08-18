// Host-buildable unit tests for the ODrive legacy native (Fibre endpoint)
// protocol wire core. Build & run with:
//   c++ -std=c++20 -I../include odrive_native_host_test.cpp -o test && ./test
//
// These tests exercise espp::detail::OdriveNativeCore directly so they need no
// ESP-IDF headers.

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <string>
#include <vector>

#include "detail/odrive_native_core.hpp"

using espp::detail::odrive_crc16;
using espp::detail::OdriveNativeCore;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

// --- packet building helpers ---------------------------------------------
static void put_u16(std::vector<uint8_t> &v, uint16_t x) {
  v.push_back(uint8_t(x & 0xff));
  v.push_back(uint8_t((x >> 8) & 0xff));
}
static void put_u32(std::vector<uint8_t> &v, uint32_t x) {
  for (int i = 0; i < 4; ++i)
    v.push_back(uint8_t((x >> (8 * i)) & 0xff));
}

// Build a request packet: [seq][endpoint_field][output_len][payload][trailer]
static std::vector<uint8_t> make_packet(uint16_t seq, uint16_t endpoint_id, bool expect_response,
                                        uint16_t output_len, std::span<const uint8_t> payload,
                                        uint16_t trailer) {
  std::vector<uint8_t> p;
  put_u16(p, seq);
  put_u16(p, uint16_t(endpoint_id | (expect_response ? 0x8000 : 0)));
  put_u16(p, output_len);
  p.insert(p.end(), payload.begin(), payload.end());
  put_u16(p, trailer);
  return p;
}

static void test_crc_golden() {
  std::printf("test_crc_golden\n");
  CHECK(odrive_crc16(std::string_view("")) == 0x1337);
  const uint8_t zero = 0x00;
  CHECK(odrive_crc16(std::span<const uint8_t>(&zero, 1)) == 0xe150);
  CHECK(odrive_crc16(std::string_view("123456789")) == 0xaa01);
  std::vector<uint8_t> v0_19;
  for (int i = 0; i < 20; ++i)
    v0_19.push_back(uint8_t(i));
  CHECK(odrive_crc16(v0_19) == 0x94d3);
  CHECK(odrive_crc16(std::string_view(
            "[{\"name\":\"vbus_voltage\",\"id\":1,\"type\":\"float\",\"access\":\"r\"}]")) ==
        0x59ec);
}

static void test_endpoint0_read() {
  std::printf("test_endpoint0_read\n");
  OdriveNativeCore core;
  float vbus = 24.0f;
  core.register_float_property("vbus_voltage", [&]() { return vbus; });
  core.register_float_property(
      "axis0.controller.input_pos", [&]() { return 0.0f; },
      [&](float, std::error_code &) { return true; });

  const std::string json = core.json();
  // The endpoint canary (interface-definition CRC) is CRC-16 over the exact JSON
  // bytes seeded with PROTOCOL_VERSION (1) -- matching the fw-v0.5.1 firmware and
  // the reference fibre client (verified by the interop harness), NOT the 0x1337
  // packet-CRC init.
  CHECK(core.json_crc() == odrive_crc16(json, espp::detail::kProtocolVersion));

  // Read endpoint 0 from offset 0, want up to 512 bytes.
  std::vector<uint8_t> off0;
  put_u32(off0, 0);
  auto req = make_packet(0x0005, /*endpoint*/ 0, /*expect*/ true, /*output_len*/ 512, off0,
                         /*trailer*/ 1 /*PROTOCOL_VERSION*/);
  auto resp = core.process_bytes(req);
  CHECK(resp.size() >= 2);
  uint16_t resp_seq = uint16_t(resp[0] | (resp[1] << 8));
  CHECK((resp_seq & 0x8000) != 0);
  CHECK((resp_seq & 0x7fff) == 0x0005);
  std::string data(resp.begin() + 2, resp.end());
  CHECK(data == json);

  // Second read at the returned length -> empty data (terminates read loop).
  std::vector<uint8_t> off_end;
  put_u32(off_end, uint32_t(json.size()));
  auto req2 = make_packet(0x0006, 0, true, 512, off_end, 1);
  auto resp2 = core.process_bytes(req2);
  CHECK(resp2.size() == 2); // just the seq header, no data
  uint16_t resp2_seq = uint16_t(resp2[0] | (resp2[1] << 8));
  CHECK((resp2_seq & 0x8000) != 0);
}

static void test_float_write_then_read() {
  std::printf("test_float_write_then_read\n");
  OdriveNativeCore core;
  float stored = 0.0f;
  bool getter_called = false, setter_called = false;
  // vbus_voltage is endpoint id 1, input_pos is endpoint id 2 (rw).
  core.register_float_property("vbus_voltage", [&]() { return 24.0f; });
  core.register_float_property(
      "axis0.controller.input_pos",
      [&]() {
        getter_called = true;
        return stored;
      },
      [&](float v, std::error_code &ec) {
        setter_called = true;
        stored = v;
        ec.clear();
        return true;
      });
  const uint16_t crc = core.json_crc();
  const uint16_t ep = 2;

  // Write 12.5f to endpoint 2.
  const float wrote = 12.5f;
  std::vector<uint8_t> payload(4);
  std::memcpy(payload.data(), &wrote, 4);
  auto wreq = make_packet(0x0010, ep, /*expect*/ true, /*output_len*/ 0, payload, crc);
  auto wresp = core.process_bytes(wreq);
  CHECK(setter_called);
  CHECK(stored == wrote);
  CHECK(wresp.size() == 2); // header only, no data for a pure write

  // Read it back (output_len=4, empty payload).
  auto rreq = make_packet(0x0011, ep, true, 4, std::span<const uint8_t>{}, crc);
  auto rresp = core.process_bytes(rreq);
  CHECK(getter_called);
  CHECK(rresp.size() == 2 + 4);
  float readback = 0.0f;
  std::memcpy(&readback, rresp.data() + 2, 4);
  CHECK(readback == wrote);
}

static void test_canary_rejection() {
  std::printf("test_canary_rejection\n");
  OdriveNativeCore core;
  float stored = 1.0f;
  bool setter_called = false;
  core.register_float_property(
      "axis0.controller.input_pos", [&]() { return stored; },
      [&](float v, std::error_code &ec) {
        setter_called = true;
        stored = v;
        ec.clear();
        return true;
      });
  const uint16_t good_crc = core.json_crc();
  const uint16_t bad_crc = uint16_t(good_crc ^ 0xffff);
  const uint16_t ep = 1;

  const float wrote = 99.0f;
  std::vector<uint8_t> payload(4);
  std::memcpy(payload.data(), &wrote, 4);
  auto wreq = make_packet(0x0020, ep, true, 0, payload, bad_crc);
  auto wresp = core.process_bytes(wreq);
  CHECK(wresp.empty());  // ignored
  CHECK(!setter_called); // no callback
  CHECK(stored == 1.0f); // no state change
}

static void test_no_response() {
  std::printf("test_no_response\n");
  OdriveNativeCore core;
  core.register_float_property("vbus_voltage", [&]() { return 24.0f; });
  const uint16_t crc = core.json_crc();
  // Read endpoint 1 WITHOUT the expect-response bit -> empty output.
  auto req = make_packet(0x0030, /*endpoint*/ 1, /*expect*/ false, /*output_len*/ 4,
                         std::span<const uint8_t>{}, crc);
  auto resp = core.process_bytes(req);
  CHECK(resp.empty());
}

static void test_unknown_endpoint_ignored() {
  std::printf("test_unknown_endpoint_ignored\n");
  OdriveNativeCore core;
  core.register_float_property("vbus_voltage", [&]() { return 24.0f; });
  const uint16_t crc = core.json_crc();
  // An unknown endpoint id, WITH the expect-response bit and a valid canary,
  // must still be ignored (empty response) per PROTOCOL.md -- not ACKed.
  auto req = make_packet(0x0031, /*endpoint*/ 999, /*expect*/ true, /*output_len*/ 4,
                         std::span<const uint8_t>{}, crc);
  auto resp = core.process_bytes(req);
  CHECK(resp.empty());
}

int main() {
  test_crc_golden();
  test_endpoint0_read();
  test_float_write_then_read();
  test_canary_rejection();
  test_no_response();
  test_unknown_endpoint_ignored();
  if (g_failures == 0) {
    std::printf("\nALL TESTS PASSED\n");
    return 0;
  }
  std::printf("\n%d CHECK(S) FAILED\n", g_failures);
  return 1;
}

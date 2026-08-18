#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "logger.hpp"
#include "odrive_native.hpp"

using namespace espp;

namespace {
// Little-endian packet builder helpers.
void put_u16(std::vector<uint8_t> &v, uint16_t x) {
  v.push_back(uint8_t(x & 0xff));
  v.push_back(uint8_t((x >> 8) & 0xff));
}
void put_u32(std::vector<uint8_t> &v, uint32_t x) {
  for (int i = 0; i < 4; ++i)
    v.push_back(uint8_t((x >> (8 * i)) & 0xff));
}

// Build a request packet: [seq][endpoint|resp_bit][output_len][payload][trailer]
std::vector<uint8_t> make_packet(uint16_t seq, uint16_t endpoint_id, bool expect_response,
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

std::string to_hex(const std::vector<uint8_t> &v) {
  std::string s;
  char buf[4];
  for (uint8_t b : v) {
    snprintf(buf, sizeof(buf), "%02x ", b);
    s += buf;
  }
  return s;
}
} // namespace

extern "C" void app_main(void) {
  Logger logger({.tag = "ODriveNativeExample", .level = Logger::Verbosity::INFO});

  //! [odrive_native_basic_example]

  // Simulated motor state.
  struct {
    float vbus_voltage = 24.0f;
    float input_pos = 0.0f;
    int32_t axis_state = 1;
  } state;

  OdriveNative::Config cfg;
  cfg.log_level = Logger::Verbosity::INFO;
  OdriveNative proto(cfg);

  // Register a small object tree of simulated-motor properties. Endpoint ids
  // are assigned in registration order starting at 1.
  proto.register_float_property("vbus_voltage", [&]() { return state.vbus_voltage; }); // id 1
  proto.register_float_property(
      "axis0.controller.input_pos", [&]() { return state.input_pos; }, // id 2 (rw)
      [&](float v, std::error_code &ec) {
        ec.clear();
        state.input_pos = v;
        return true;
      });
  proto.register_int32_property(
      "axis0.current_state", [&]() { return state.axis_state; }, // id 3 (rw)
      [&](int32_t v, std::error_code &ec) {
        ec.clear();
        state.axis_state = v;
        return true;
      });

  const uint16_t json_crc = proto.json_crc();
  logger.info("Endpoint JSON descriptor ({} bytes, crc=0x{:04x}):\n{}", proto.json().size(),
              json_crc, proto.json());

  // 1) endpoint-0 read: fetch the JSON descriptor (offset 0, up to 512 bytes).
  {
    std::vector<uint8_t> offset;
    put_u32(offset, 0);
    auto req = make_packet(0x0001, /*endpoint*/ 0, /*expect*/ true, /*output_len*/ 512, offset,
                           /*trailer*/ 1 /*PROTOCOL_VERSION*/);
    auto resp = proto.process_bytes(req);
    std::string json(resp.begin() + 2, resp.end());
    logger.info("endpoint-0 read -> {} bytes: {}", resp.size(), json);
  }

  // 2) write axis0.controller.input_pos (endpoint 2) = 3.14f.
  {
    const float value = 3.14f;
    std::vector<uint8_t> payload(4);
    std::memcpy(payload.data(), &value, 4);
    auto req =
        make_packet(0x0002, /*endpoint*/ 2, /*expect*/ true, /*output_len*/ 0, payload, json_crc);
    (void)proto.process_bytes(req);
    logger.info("wrote input_pos=3.14 -> state.input_pos={}", state.input_pos);
  }

  // 3) read axis0.controller.input_pos back (endpoint 2, output_len=4).
  {
    auto req = make_packet(0x0003, /*endpoint*/ 2, /*expect*/ true, /*output_len*/ 4,
                           std::span<const uint8_t>{}, json_crc);
    auto resp = proto.process_bytes(req);
    float readback = 0.0f;
    if (resp.size() >= 6)
      std::memcpy(&readback, resp.data() + 2, 4);
    logger.info("read input_pos -> resp [{}] value={}", to_hex(resp), readback);
  }

  //! [odrive_native_basic_example]

  logger.info("ODrive native example complete.");
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

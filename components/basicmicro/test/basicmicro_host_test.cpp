// Host-buildable unit tests for the Basicmicro (MCP / RoboClaw-family) packet
// serial wire core. Build & run with:
//   c++ -std=c++20 -I../include basicmicro_host_test.cpp -o test && ./test
//
// These tests exercise the helpers in detail/basicmicro_core.hpp directly so
// they need no ESP-IDF headers. Golden CRC values were computed by executing
// the reference CRC16 C implementation printed in MCP Series User Manual
// section 2.2.6 (CRC-16/XMODEM: poly 0x1021, init 0, non-reflected).

#include <cstdint>
#include <cstdio>
#include <limits>
#include <span>
#include <string_view>
#include <vector>

#include "detail/basicmicro_core.hpp"

using namespace espp::detail;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

static uint16_t crc_of(std::string_view s) {
  return basicmicro_crc16(
      std::span<const uint8_t>(reinterpret_cast<const uint8_t *>(s.data()), s.size()));
}

static void test_crc_golden() {
  std::printf("test_crc_golden\n");
  // empty input leaves the initial remainder (0) untouched
  CHECK(crc_of("") == 0x0000);
  // a zero byte folded through a zero remainder stays zero
  const uint8_t zero = 0x00;
  CHECK(basicmicro_crc16(std::span<const uint8_t>(&zero, 1)) == 0x0000);
  // the classic CRC-16/XMODEM check value
  CHECK(crc_of("123456789") == 0x31C3);
  CHECK(crc_of("A") == 0x58E5);
  // real packets from the manual's command set:
  // [0x80, 0 (DriveForwardM1), 35]
  const uint8_t drive_fwd[] = {0x80, 0x00, 0x23};
  CHECK(basicmicro_crc16(drive_fwd) == 0x2F5B);
  // [0x80, 32 (DriveM1SignedDuty), duty=0x4000 (+50%)]
  const uint8_t drive_duty[] = {0x80, 0x20, 0x40, 0x00};
  CHECK(basicmicro_crc16(drive_duty) == 0x5632);
  // [0x80, 20 (ResetEncoders)] -- write command with an empty payload
  const uint8_t reset_enc[] = {0x80, 0x14};
  CHECK(basicmicro_crc16(reset_enc) == 0x492D);
  // seeded continuation equals one-shot over the concatenation
  const uint8_t head[] = {0x80, 0x18};
  const uint8_t tail[] = {0x00, 0x7B};
  CHECK(basicmicro_crc16(tail, basicmicro_crc16(head)) == 0xF806);
}

static void test_codecs() {
  std::printf("test_codecs\n");
  std::vector<uint8_t> v;
  append_u8(v, 0xAB);
  append_u16_be(v, 0x1234);
  append_u32_be(v, 0xDEADBEEF);
  append_i16_be(v, -2);    // 0xFFFE
  append_i32_be(v, -3000); // 0xFFFFF448
  const std::vector<uint8_t> expected = {0xAB, 0x12, 0x34, 0xDE, 0xAD, 0xBE, 0xEF,
                                         0xFF, 0xFE, 0xFF, 0xFF, 0xF4, 0x48};
  CHECK(v == expected);
  // decode round-trips (big-endian, "high byte first" per manual 2.2.9)
  CHECK(read_u16_be(v, 1) == 0x1234);
  CHECK(read_u32_be(v, 3) == 0xDEADBEEF);
  CHECK(read_i16_be(v, 7) == -2);
  CHECK(read_i32_be(v, 9) == -3000);
}

static void test_packet_build() {
  std::printf("test_packet_build\n");
  // write packet with payload: [addr, cmd, data..., crc_hi, crc_lo]
  std::vector<uint8_t> payload;
  append_u8(payload, 0x23);
  const auto pkt = build_write_packet(0x80, 0, payload);
  const std::vector<uint8_t> expected = {0x80, 0x00, 0x23, 0x2F, 0x5B};
  CHECK(pkt == expected);

  // write packet with an empty payload (ResetEncoders)
  const auto reset = build_write_packet(0x80, 20);
  const std::vector<uint8_t> expected_reset = {0x80, 0x14, 0x49, 0x2D};
  CHECK(reset == expected_reset);

  // drive M1 signed speed -3000 qpps: [0x80, 35, FF FF F4 48, crc]
  std::vector<uint8_t> speed_payload;
  append_i32_be(speed_payload, -3000);
  const auto speed_pkt = build_write_packet(0x80, 35, speed_payload);
  const std::vector<uint8_t> expected_speed = {0x80, 0x23, 0xFF, 0xFF, 0xF4, 0x48, 0xA0, 0x4F};
  CHECK(speed_pkt == expected_speed);

  // read requests carry no CRC
  const auto req = build_read_request(0x80, 24);
  const std::vector<uint8_t> expected_req = {0x80, 0x18};
  CHECK(req == expected_req);
}

static void test_reply_validation() {
  std::printf("test_reply_validation\n");
  // reply to ReadMainBatteryVoltage (cmd 24) at addr 0x80 with value 123
  // (12.3 V): data [0x00, 0x7B], CRC over [0x80, 0x18, 0x00, 0x7B] = 0xF806
  const std::vector<uint8_t> reply = {0x00, 0x7B, 0xF8, 0x06};
  CHECK(validate_reply(0x80, 24, reply));

  // wrong address / command seeds must fail
  CHECK(!validate_reply(0x81, 24, reply));
  CHECK(!validate_reply(0x80, 25, reply));

  // corrupt data byte must fail
  std::vector<uint8_t> bad_data = reply;
  bad_data[0] ^= 0x01;
  CHECK(!validate_reply(0x80, 24, bad_data));

  // corrupt CRC byte must fail
  std::vector<uint8_t> bad_crc = reply;
  bad_crc[3] ^= 0x01;
  CHECK(!validate_reply(0x80, 24, bad_crc));

  // too-short replies must fail (need at least the 2 CRC bytes)
  const std::vector<uint8_t> tiny = {0xF8};
  CHECK(!validate_reply(0x80, 24, tiny));

  // ReadEncoderM1 reply at addr 0x81: count 0x12345678, status 0x02, CRC over
  // [0x81, 0x10, 0x12, 0x34, 0x56, 0x78, 0x02] = 0xE201
  const std::vector<uint8_t> enc_reply = {0x12, 0x34, 0x56, 0x78, 0x02, 0xE2, 0x01};
  CHECK(validate_reply(0x81, 16, enc_reply));
  CHECK(read_u32_be(enc_reply, 0) == 0x12345678u);
  CHECK(enc_reply[4] == 0x02);
}

static void test_round_trip() {
  std::printf("test_round_trip\n");
  // Build a write packet, then check that the packet body validates against
  // its own trailing CRC using the reply-validation seeding rules: a write
  // packet [addr, cmd, data, crc] is equivalent to a "reply" of data bytes
  // whose CRC is seeded with [addr, cmd].
  std::vector<uint8_t> payload;
  append_u32_be(payload, 0x00010000); // P = 1.0 in 16.16 fixed point
  append_u32_be(payload, 0x00008000); // I = 0.5
  append_u32_be(payload, 0x00004000); // D = 0.25
  append_u32_be(payload, 44000);      // QPPS default
  const auto pkt = build_write_packet(0x80, 28, payload);
  CHECK(pkt.size() == 2 + 16 + 2);
  // whole packet CRCs to zero remainder... (property of appending the CRC)
  // more directly: the stored CRC matches a recomputation over addr+cmd+data
  const std::span<const uint8_t> body(pkt.data() + 2, pkt.size() - 4);
  CHECK(validate_reply(pkt[0], pkt[1], std::span<const uint8_t>(pkt.data() + 2, pkt.size() - 2)));
  CHECK(basicmicro_crc16(std::span<const uint8_t>(pkt.data(), pkt.size() - 2)) ==
        read_u16_be(pkt, pkt.size() - 2));
  CHECK(body.size() == 16);

  // command / status enums carry the verified wire values
  CHECK(static_cast<uint8_t>(BasicmicroCommand::ReadFirmwareVersion) == 21);
  CHECK(static_cast<uint8_t>(BasicmicroCommand::SetVelocityPidM1) == 28);
  CHECK(static_cast<uint8_t>(BasicmicroCommand::ReadVelocityPidM1) == 55);
  CHECK(static_cast<uint8_t>(BasicmicroCommand::SetPositionPidM1) == 61);
  CHECK(static_cast<uint8_t>(BasicmicroCommand::SetPositionPidM2) == 62);
  CHECK(static_cast<uint8_t>(BasicmicroCommand::ReadPositionPidM1) == 63);
  CHECK(static_cast<uint8_t>(BasicmicroCommand::ReadPositionPidM2) == 64);
  CHECK(static_cast<uint8_t>(BasicmicroCommand::ReadStatus) == 90);
  CHECK(static_cast<uint8_t>(BasicmicroCommand::EStopReset) == 200);
  CHECK(static_cast<uint16_t>(BasicmicroStatus::Temperature2Warning) == 0x2000);

  // position PID payload (command 61): D, P, I scaled by 1024, then MaxI,
  // Deadzone, MinPos, MaxPos as raw 32-bit -> 7 * 4 = 28 data bytes. Build the
  // gains through scale_pid_gain() exactly as the production path does.
  std::vector<uint8_t> pos;
  append_u32_be(pos, scale_pid_gain(4.0f, kBasicmicroPositionPidScale)); // D
  append_u32_be(pos, scale_pid_gain(2.0f, kBasicmicroPositionPidScale)); // P
  append_u32_be(pos, scale_pid_gain(0.0f, kBasicmicroPositionPidScale)); // I
  append_u32_be(pos, 0);                                                 // MaxI
  append_u32_be(pos, 10);                                                // Deadzone
  append_i32_be(pos, -20000);                                            // MinPos
  append_i32_be(pos, 20000);                                             // MaxPos
  CHECK(pos.size() == 28);
  CHECK(read_u32_be(pos, 0) == 4096); // 4.0 * 1024
  CHECK(read_u32_be(pos, 4) == 2048); // 2.0 * 1024
  CHECK(read_i32_be(pos, 20) == -20000);
  CHECK(read_i32_be(pos, 24) == 20000);
  const auto pos_pkt =
      build_write_packet(0x80, static_cast<uint8_t>(BasicmicroCommand::SetPositionPidM1), pos);
  CHECK(pos_pkt.size() == 2 + 28 + 2);
  CHECK(basicmicro_crc16(std::span<const uint8_t>(pos_pkt.data(), pos_pkt.size() - 2)) ==
        read_u16_be(pos_pkt, pos_pkt.size() - 2));

  // scale_pid_gain(): round-to-nearest (not truncate) and clamp non-negative.
  CHECK(scale_pid_gain(4.0f, kBasicmicroPositionPidScale) == 4096);
  CHECK(scale_pid_gain(1.5f, kBasicmicroPositionPidScale) == 1536);
  // 0.1 * 1024 = 102.4 -> rounds to 102 (truncation would also give 102);
  // 0.10009765625 (= 102.5/1024) rounds to 103, where truncation gives 102.
  CHECK(scale_pid_gain(102.5f / kBasicmicroPositionPidScale, kBasicmicroPositionPidScale) == 103);
  CHECK(scale_pid_gain(-1.0f, kBasicmicroPositionPidScale) == 0); // negatives clamp to 0
  CHECK(scale_pid_gain(0.0f, kBasicmicroPositionPidScale) == 0);
  CHECK(scale_pid_gain(1.0f, 65536.0f) == 65536); // velocity 16.16 scale too
  // non-finite and out-of-range inputs saturate rather than invoke UB
  CHECK(scale_pid_gain(std::numeric_limits<float>::infinity(), kBasicmicroPositionPidScale) ==
        0xFFFFFFFFu);
  CHECK(scale_pid_gain(std::numeric_limits<float>::quiet_NaN(), kBasicmicroPositionPidScale) == 0);
  CHECK(scale_pid_gain(1.0e12f, kBasicmicroPositionPidScale) == 0xFFFFFFFFu); // >> 2^32
  CHECK(scale_pid_gain(4194304.0f, 1024.0f) == 0xFFFFFFFFu); // 4194304 * 1024 == 2^32, saturates
  CHECK(scale_pid_gain(4194303.0f, 1024.0f) == 4294966272u); // just below 2^32, fits exactly

  // position PID REPLY (command 63/64): the read order is P, I, D, then MaxI,
  // Deadzone, MinPos, MaxPos -- distinct from the D, P, I write order above.
  // Build a 28-byte reply and confirm each field decodes from its offset.
  std::vector<uint8_t> reply;
  append_u32_be(reply, static_cast<uint32_t>(2.0f * kBasicmicroPositionPidScale)); // P (offset 0)
  append_u32_be(reply, static_cast<uint32_t>(0.5f * kBasicmicroPositionPidScale)); // I (offset 4)
  append_u32_be(reply, static_cast<uint32_t>(4.0f * kBasicmicroPositionPidScale)); // D (offset 8)
  append_u32_be(reply, 7);      // MaxI (offset 12)
  append_u32_be(reply, 10);     // Deadzone (offset 16)
  append_i32_be(reply, -20000); // MinPos (offset 20)
  append_i32_be(reply, 20000);  // MaxPos (offset 24)
  CHECK(reply.size() == 28);
  CHECK(static_cast<float>(read_u32_be(reply, 0)) / kBasicmicroPositionPidScale == 2.0f); // P
  CHECK(static_cast<float>(read_u32_be(reply, 4)) / kBasicmicroPositionPidScale == 0.5f); // I
  CHECK(static_cast<float>(read_u32_be(reply, 8)) / kBasicmicroPositionPidScale == 4.0f); // D
  CHECK(read_u32_be(reply, 12) == 7);
  CHECK(read_u32_be(reply, 16) == 10);
  CHECK(read_i32_be(reply, 20) == -20000);
  CHECK(read_i32_be(reply, 24) == 20000);
}

int main() {
  test_crc_golden();
  test_codecs();
  test_packet_build();
  test_reply_validation();
  test_round_trip();
  if (g_failures) {
    std::printf("%d FAILURES\n", g_failures);
    return 1;
  }
  std::printf("ALL PASSED\n");
  return 0;
}

// Host-buildable unit tests for the MCP266 CANopen mapping core. Build & run:
//   c++ -std=c++20 -I../include -I../../motor_controller/include \
//       mcp266_host_test.cpp -o test && ./test
// (the -I../../motor_controller/include is for the shared BasicmicroCommand table)
//
// These tests exercise detail/mcp266_core.hpp directly (no ESP-IDF headers).
// The object addresses were verified against a live MCP266's SDO object
// dictionary; the manufacturer region mirrors the packet-serial command set at
// index 0x2000 + command number.

#include <array>
#include <cstdint>
#include <cstdio>
#include <limits>

#include "detail/mcp266_core.hpp"

using namespace espp::detail::mcp266;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

static void test_command_object() {
  std::printf("test_command_object\n");
  // command N mirrors to 0x2000 + N (verified anchors)
  CHECK(command_object(61) == 0x203D);  // set M1 position PID
  CHECK(command_object(62) == 0x203E);  // set M2 position PID
  CHECK(command_object(63) == 0x203F);  // read M1 position PID
  CHECK(command_object(64) == 0x2040);  // read M2 position PID
  CHECK(command_object(32) == 0x2020);  // drive M1 duty
  CHECK(command_object(35) == 0x2023);  // drive M1 speed
  CHECK(command_object(24) == 0x2018);  // read main battery
  CHECK(command_object(82) == 0x2052);  // read temperature
  CHECK(command_object(200) == 0x20C8); // e-stop reset
  CHECK(command_object(20) == 0x2014);  // reset encoders
  CHECK(command_object(22) == 0x2016);  // set M1 encoder
  CHECK(command_object(23) == 0x2017);  // set M2 encoder
  CHECK(kMainBatteryObject == 0x2018);
  CHECK(kTemperatureObject == 0x2052);
  CHECK(kEStopResetObject == 0x20C8);
  CHECK(kResetEncodersObject == 0x2014);
  CHECK(kPositionGainScale == 1024);
}

static void test_axis_objects() {
  std::printf("test_axis_objects\n");
  const auto m1 = axis_m1();
  const auto m2 = axis_m2();
  // M1 at the standard offset, M2 mirrored at +0x800
  CHECK(m1.object_offset == 0x000);
  CHECK(m2.object_offset == 0x800);
  // per-axis command objects follow the 0x2000 + command mapping (cmd n / n+1)
  CHECK(m1.position_pid_set == 0x203D);
  CHECK(m1.position_pid_get == 0x203F);
  CHECK(m1.drive_duty == 0x2020);
  CHECK(m1.drive_speed == 0x2023);
  CHECK(m1.encoder_set == 0x2016);
  CHECK(m2.position_pid_set == 0x203E);
  CHECK(m2.position_pid_get == 0x2040);
  CHECK(m2.drive_duty == 0x2021);
  CHECK(m2.drive_speed == 0x2024);
  CHECK(m2.encoder_set == 0x2017);
  // the CiA 402 offset applied to a device-profile object selects the axis
  CHECK(static_cast<uint16_t>(0x6040 + m2.object_offset) == 0x6840); // controlword
  CHECK(static_cast<uint16_t>(0x607A + m2.object_offset) == 0x687A); // target position
}

static void test_position_pid_remap() {
  std::printf("test_position_pid_remap\n");
  // readback order [P, I, D, MaxI, Deadzone, MinPos, MaxPos] ->
  // setter order   [D, P, I, MaxI, Deadzone, MinPos, MaxPos]
  const std::array<int32_t, 7> readback{100, 20, 3, 4, 5, -1000, 1000};
  const auto setter = position_pid_readback_to_setter(readback);
  CHECK(setter[0] == 3);     // D
  CHECK(setter[1] == 100);   // P
  CHECK(setter[2] == 20);    // I
  CHECK(setter[3] == 4);     // MaxI
  CHECK(setter[4] == 5);     // Deadzone
  CHECK(setter[5] == -1000); // MinPos
  CHECK(setter[6] == 1000);  // MaxPos
  // constexpr-evaluable
  static_assert(position_pid_readback_to_setter({7, 8, 9, 0, 0, 0, 0})[0] == 9);
  static_assert(position_pid_readback_to_setter({7, 8, 9, 0, 0, 0, 0})[1] == 7);
}

static void test_scale_position_gain() {
  std::printf("test_scale_position_gain\n");
  // round-to-nearest (not truncate) and clamp to the non-negative i32 range,
  // matching espp::Basicmicro's scale_pid_gain()
  CHECK(scale_position_gain(4.0f) == 4096);
  CHECK(scale_position_gain(1.5f) == 1536);
  // 102.5 / 1024 scales back to exactly 102.5 -> rounds up to 103 (truncation: 102)
  CHECK(scale_position_gain(102.5f / 1024.0f) == 103);
  CHECK(scale_position_gain(15491.0f / 1024.0f) == 15491); // the default fallback P round-trips
  CHECK(scale_position_gain(0.0f) == 0);
  CHECK(scale_position_gain(-1.0f) == 0); // negatives clamp to 0
  CHECK(scale_position_gain(std::numeric_limits<float>::quiet_NaN()) == 0);
  CHECK(scale_position_gain(std::numeric_limits<float>::infinity()) == INT32_MAX);
  CHECK(scale_position_gain(1.0e12f) == INT32_MAX); // >> 2^31, saturates
  // constexpr-evaluable
  static_assert(scale_position_gain(2.0f) == 2048);
  static_assert(scale_position_gain(-2.0f) == 0);
}

int main() {
  test_command_object();
  test_axis_objects();
  test_position_pid_remap();
  test_scale_position_gain();
  if (g_failures) {
    std::printf("%d FAILURES\n", g_failures);
    return 1;
  }
  std::printf("ALL PASSED\n");
  return 0;
}

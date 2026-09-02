// Host-buildable unit tests for the MCP266 CANopen mapping core. Build & run:
//   c++ -std=c++20 -I../include mcp266_host_test.cpp -o test && ./test
//
// These tests exercise detail/mcp266_core.hpp directly (no ESP-IDF headers).
// The object addresses were verified against a live MCP266's SDO object
// dictionary; the manufacturer region mirrors the packet-serial command set at
// index 0x2000 + command number.

#include <array>
#include <cstdint>
#include <cstdio>

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
  CHECK(kMainBatteryObject == 0x2018);
  CHECK(kTemperatureObject == 0x2052);
  CHECK(kEStopResetObject == 0x20C8);
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
  CHECK(m2.position_pid_set == 0x203E);
  CHECK(m2.position_pid_get == 0x2040);
  CHECK(m2.drive_duty == 0x2021);
  CHECK(m2.drive_speed == 0x2024);
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

int main() {
  test_command_object();
  test_axis_objects();
  test_position_pid_remap();
  if (g_failures) {
    std::printf("%d FAILURES\n", g_failures);
    return 1;
  }
  std::printf("ALL PASSED\n");
  return 0;
}

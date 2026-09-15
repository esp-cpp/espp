// Host-buildable unit tests for the MCP266 console wire protocol
// (include/mcp266_protocol.hpp -- standard library only). Build & run:
//   c++ -std=c++20 -Wall -Wextra -Werror -I components/mcp266/include \
//       components/mcp266/test/mcp266_protocol_host_test.cpp -o test && ./test

#include <cstdio>
#include <vector>

#include "mcp266_protocol.hpp"

namespace proto = espp::mcp266_protocol;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

int main() {
  // ---- direction convention ----
  CHECK(!proto::is_reply(static_cast<uint8_t>(proto::Request::Start)));
  CHECK(proto::is_reply(static_cast<uint8_t>(proto::Reply::Status)));
  static_assert(proto::kModuleId == 6, "the MCP266 console web app expects module 6");

  // ---- axis selector: only 0 / 1 are valid ----
  CHECK(proto::parse_axis(0) == proto::Axis::M1 && proto::parse_axis(1) == proto::Axis::M2);
  CHECK(!proto::parse_axis(2).has_value() && !proto::parse_axis(0xFF).has_value());

  // ---- request payload round trips + exact sizes ----
  {
    const proto::ConfigurePositionLoop c{proto::Axis::M2, -1000, 1000, 42};
    const auto b = c.serialize();
    CHECK(b.size() == proto::ConfigurePositionLoop::kSize);
    const auto r = proto::ConfigurePositionLoop::parse(b);
    CHECK(r && r->axis == proto::Axis::M2 && r->min == -1000 && r->max == 1000 &&
          r->fallback_p == 42);
    CHECK(!proto::ConfigurePositionLoop::parse(std::vector<uint8_t>(b.begin(), b.end() - 1)));
    auto bad_axis = b;
    bad_axis[0] = 7;
    CHECK(!proto::ConfigurePositionLoop::parse(bad_axis)); // never falls through to M1
  }
  {
    const proto::SetPositionLimits l{proto::Axis::M1, -5, 5};
    const auto b = l.serialize();
    CHECK(b.size() == proto::SetPositionLimits::kSize);
    const auto r = proto::SetPositionLimits::parse(b);
    CHECK(r && r->axis == proto::Axis::M1 && r->min == -5 && r->max == 5);
  }
  {
    const proto::MoveToPosition m{proto::Axis::M2, -123456, 5000, 10000, 20000};
    const auto b = m.serialize();
    CHECK(b.size() == proto::MoveToPosition::kSize);
    CHECK(b[0] == 1 && b[1] == 0xC0 && b[2] == 0x1D && b[3] == 0xFE && b[4] == 0xFF); // LE i32
    const auto r = proto::MoveToPosition::parse(b);
    CHECK(r && r->target == -123456 && r->velocity == 5000 && r->accel == 10000 &&
          r->decel == 20000);
  }
  {
    const auto b = proto::DriveSpeed{proto::Axis::M1, -77}.serialize();
    CHECK(b.size() == proto::DriveSpeed::kSize);
    const auto r = proto::DriveSpeed::parse(b);
    CHECK(r && r->qpps == -77);
    const auto d = proto::DriveDuty{proto::Axis::M2, -32000}.serialize();
    CHECK(d.size() == proto::DriveDuty::kSize);
    const auto rd = proto::DriveDuty::parse(d);
    CHECK(rd && rd->axis == proto::Axis::M2 && rd->duty == -32000);
  }
  {
    const auto b = proto::SetStatusStream{true, 250}.serialize();
    CHECK(b.size() == proto::SetStatusStream::kSize && b[0] == 1 && b[1] == 0xFA && b[2] == 0);
    const auto r = proto::SetStatusStream::parse(b);
    CHECK(r && r->enabled && r->period_ms == 250);
    CHECK(!proto::SetStatusStream::parse(std::vector<uint8_t>{1, 2}));
  }

  // ---- STATUS: 25-byte layout round trip ----
  {
    proto::Status s;
    s.m1 = {1000, -20, 0x1237};
    s.m2 = {-1, 3, 0x0040};
    s.battery_decivolts = 245; // 24.5 V
    s.temp_decidegrees = 312;  // 31.2 C
    s.online = true;
    const auto b = s.serialize();
    CHECK(b.size() == proto::Status::kSize && proto::Status::kSize == 25);
    CHECK(b[24] == proto::Status::kFlagOnline);
    CHECK(b[8] == 0x37 && b[9] == 0x12); // M1 statusword LE at offset 8
    const auto r = proto::Status::parse(b);
    CHECK(r && *r == s);
    CHECK(!proto::Status::parse(std::vector<uint8_t>(b.begin(), b.end() - 1)));
  }

  // ---- DEVICE_INFO / OK / ERROR ----
  {
    const proto::DeviceInfo i{0x00020192, "MCP266"};
    const auto b = i.serialize();
    CHECK(b.size() == 4 + 6);
    const auto r = proto::DeviceInfo::parse(b);
    CHECK(r && *r == i);
    CHECK(!proto::DeviceInfo::parse(std::vector<uint8_t>{1, 2, 3}));

    CHECK(proto::make_ok_payload(0x65) == std::vector<uint8_t>{0x65});
    const auto e = proto::make_error_payload(0x63, 22, "configure failed: Invalid argument");
    const auto pe = proto::parse_error_payload(e);
    CHECK(pe && pe->request_type == 0x63 && pe->code == 22 &&
          pe->message == "configure failed: Invalid argument");
    CHECK(!proto::parse_error_payload(std::vector<uint8_t>{0x63, 1, 2}));
  }

  if (g_failures == 0) {
    std::printf("ALL MCP266 PROTOCOL TESTS PASSED\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", g_failures);
  return 1;
}

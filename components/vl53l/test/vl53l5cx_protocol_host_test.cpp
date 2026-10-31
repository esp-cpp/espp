// Host-buildable unit tests for the VL53L5CX console wire protocol
// (include/vl53l5cx_protocol.hpp -- standard library only). Build & run:
//   c++ -std=c++23 -Wall -Wextra -Werror -I components/vl53l/include \
//       components/vl53l/test/vl53l5cx_protocol_host_test.cpp -o test && ./test

#include <cstdio>
#include <string_view>
#include <vector>

#include "vl53l5cx_protocol.hpp"

namespace proto = espp::vl53l5cx_protocol;

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
  CHECK(!proto::is_reply(static_cast<uint8_t>(proto::Request::GetStatus)));
  CHECK(!proto::is_reply(static_cast<uint8_t>(proto::Request::SetI2cAddress)));
  CHECK(proto::is_reply(static_cast<uint8_t>(proto::Reply::Status)));
  CHECK(proto::is_reply(static_cast<uint8_t>(proto::Reply::Frame)));
  static_assert(proto::kModuleId == 10, "10 is the published default the console falls back to");
  static_assert(std::string_view(proto::kProtocol) == "espp.vl53l5cx",
                "the VL53L5CX console web app locates the module by this protocol id");
  static_assert(proto::kProtocolVersion == 1);

  // ---- request payloads ----
  {
    CHECK(!proto::ByteArg::parse({}).has_value());
    const uint8_t one[] = {64};
    CHECK(proto::ByteArg::parse(one)->value == 64);
    CHECK(proto::ByteArg{16}.serialize() == std::vector<uint8_t>{16});

    const proto::SetIntegrationTime t{0x01020304};
    const auto tb = t.serialize();
    CHECK(tb.size() == proto::SetIntegrationTime::kSize);
    CHECK((tb == std::vector<uint8_t>{0x04, 0x03, 0x02, 0x01})); // little-endian
    CHECK(proto::SetIntegrationTime::parse(tb)->time_ms == 0x01020304);
    CHECK(!proto::SetIntegrationTime::parse(std::span<const uint8_t>(tb).first(3)).has_value());

    const proto::EnableMotionIndicator m{400, 1500};
    const auto mb = m.serialize();
    CHECK(mb.size() == proto::EnableMotionIndicator::kSize);
    const auto mr = proto::EnableMotionIndicator::parse(mb);
    CHECK(mr && mr->min_mm == 400 && mr->max_mm == 1500);

    const proto::SetFrameStream s{true, 3};
    const auto sb = s.serialize();
    CHECK(sb.size() == proto::SetFrameStream::kSize);
    CHECK(proto::SetFrameStream::parse(sb)->enabled &&
          proto::SetFrameStream::parse(sb)->divider == 3);
    CHECK(!proto::SetFrameStream::parse(std::span<const uint8_t>(sb).first(1)).has_value());
  }

  // ---- STATUS round trip + exact size + flag bits ----
  {
    proto::Status s;
    s.initialized = true;
    s.ranging = true;
    s.motion_enabled = false;
    s.stream_enabled = true;
    s.variant = 1;
    s.address = 0x2A;
    s.zones = 64;
    s.frequency_hz = 15;
    s.integration_ms = 20;
    s.sharpener = 7;
    s.target_order = 1;
    s.ranging_mode = 1;
    s.power_mode = 1;
    s.motion_min_mm = 400;
    s.motion_max_mm = 1500;
    s.frame_count = 123456;
    s.frame_rate_chz = 1498;
    s.stream_divider = 2;
    const auto b = s.serialize();
    CHECK(b.size() == proto::Status::kSize);
    CHECK(b[0] == (proto::Status::kFlagInitialized | proto::Status::kFlagRanging |
                   proto::Status::kFlagStreamEnabled));
    const auto r = proto::Status::parse(b);
    CHECK(r && *r == s);
    CHECK(!proto::Status::parse(std::span<const uint8_t>(b).first(proto::Status::kSize - 1))
               .has_value());
  }

  // ---- FRAME: 4x4 without motion, 8x8 with motion, truncation ----
  {
    proto::Frame f;
    f.sequence = 42;
    f.timestamp_ms = 1000;
    f.zones = 16;
    for (size_t i = 0; i < 16; i++) {
      f.zone[i] = proto::Zone{.distance_mm = static_cast<int16_t>(100 * i),
                              .status = static_cast<uint8_t>(i == 3 ? 255 : 5),
                              .nb_targets = 1,
                              .reflectance_pct = static_cast<uint8_t>(i),
                              .sigma_mm = static_cast<uint16_t>(2 + i),
                              .signal_kcps = static_cast<uint32_t>(1000 + i),
                              .ambient_kcps = static_cast<uint32_t>(10 + i),
                              .spads = static_cast<uint32_t>(256 + i)};
    }
    const auto b = f.serialize();
    CHECK(b.size() == proto::Frame::kHeaderSize + 16 * proto::Zone::kSize);
    CHECK(b[8] == 16 && b[9] == 0);
    const auto r = proto::Frame::parse(b);
    CHECK(r && r->sequence == 42 && r->timestamp_ms == 1000 && r->zones == 16 &&
          !r->motion_present);
    CHECK(r && r->width() == 4);
    bool zones_equal = r.has_value();
    for (size_t i = 0; zones_equal && i < 16; i++)
      zones_equal = r->zone[i] == f.zone[i];
    CHECK(zones_equal);
    CHECK(!proto::Frame::parse(std::span<const uint8_t>(b).first(b.size() - 1)).has_value());

    proto::Frame g = f;
    g.zones = 64;
    g.motion_present = true;
    g.motion.global_indicator_1 = 7;
    g.motion.nb_of_aggregates = 16;
    g.motion.motion[9] = 180;
    const auto gb = g.serialize();
    CHECK(gb.size() == proto::Frame::kMaxSize);
    CHECK(gb.size() == 1366);
    const auto gr = proto::Frame::parse(gb);
    CHECK(gr && gr->zones == 64 && gr->width() == 8 && gr->motion_present &&
          gr->motion == g.motion);
    // a bad zone count is rejected
    std::vector<uint8_t> bad = gb;
    bad[8] = 32;
    CHECK(!proto::Frame::parse(bad).has_value());
  }

  // ---- OK / ERROR payloads ----
  {
    CHECK((proto::make_ok_payload(0x19) == std::vector<uint8_t>{0x19}));
    const auto e = proto::make_error_payload(0x11, 16, "busy");
    const auto r = proto::parse_error_payload(e);
    CHECK(r && r->request_type == 0x11 && r->code == 16 && r->message == "busy");
    CHECK(!proto::parse_error_payload(std::span<const uint8_t>(e).first(4)).has_value());
  }

  if (g_failures == 0)
    std::printf("vl53l5cx_protocol_host_test: all checks passed\n");
  else
    std::printf("vl53l5cx_protocol_host_test: %d check(s) FAILED\n", g_failures);
  return g_failures == 0 ? 0 : 1;
}

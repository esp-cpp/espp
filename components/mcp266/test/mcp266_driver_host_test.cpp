// Host-buildable tests for espp::Mcp266 against a fake MCP266 SDO server. Build
// & run from the repository root:
//   git submodule update --init components/format/detail/fmt
//   c++ -std=c++23 -Wall -Wextra -Werror -I components/mcp266/include \
//       -I components/canopen/include -I components/motor_controller/include \
//       -I components/base_component/include -I components/logger/include \
//       -I components/format/include -isystem components/format/detail/fmt/include \
//       components/mcp266/test/mcp266_driver_host_test.cpp \
//       components/logger/src/logger.cpp -o test && ./test
//
// The fake answers expedited SDO uploads / downloads inline from the client's
// send function (CanopenClient sends with no lock held that process_frame()
// takes, so an inline reply completes the transaction). It models the readback
// quirk seen on an MCP266 2x60A, software version 1.4.9: subindices 2..7 of the
// position-PID readback (0x203F / 0x2040) read 0 unless the reads of that object
// started at subindex 1 and nothing else was read in between.

#include <array>
#include <bit>
#include <cstdint>
#include <cstdio>
#include <map>
#include <utility>

#include "mcp266.hpp"

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

namespace {

namespace co = espp::detail::canopen;
namespace ds = espp::detail::ds402;
namespace mcp = espp::detail::mcp266;
using espp::CanopenClient;
using Frame = CanopenClient::CanFrame;

constexpr uint8_t kNode = 3;
constexpr uint16_t kM1PidSet = mcp::axis_m1().position_pid_set;
constexpr uint16_t kM2PidSet = mcp::axis_m2().position_pid_set;
constexpr uint16_t kM1PidGet = mcp::axis_m1().position_pid_get;
constexpr uint16_t kM2PidGet = mcp::axis_m2().position_pid_get;

/// A fake MCP266: an object dictionary of 4-byte values plus the readback quirk.
class FakeMcp266 {
public:
  FakeMcp266()
      : client_({.node_id = kNode,
                 .send = [this](const Frame &f) { return on_request(f); },
                 .sdo_timeout = std::chrono::milliseconds(50),
                 .log_level = espp::Logger::Verbosity::NONE}) {
    // the bench controller's stored records, readback order
    // [P, I, D, MaxI, Deadzone, MinPos, MaxPos]
    set_record(kM1PidGet, {103, 0, 868, 0, 10, 10000, 18000});
    set_record(kM2PidGet, {120, 0, 1086, 0, 10, 25000, 32000});
    store(ds::OBJ_POSITION_ACTUAL, 0, 13554); // M1 position actual
  }

  CanopenClient &client() { return client_; }

  /// A dictionary entry; 0 for one never written, as the controller reads it.
  int32_t field(uint16_t object, uint8_t sub) const {
    const auto it = od_.find({object, sub});
    return it == od_.end() ? 0 : it->second;
  }

private:
  using Key = std::pair<uint16_t, uint8_t>;

  void store(uint16_t object, uint8_t sub, int32_t value) {
    od_.insert_or_assign(Key{object, sub}, value);
  }

  void set_record(uint16_t object, const std::array<int32_t, 7> &readback) {
    for (uint8_t sub = 1; sub <= 7; ++sub) {
      store(object, sub, readback[sub - 1]);
    }
  }

  static bool is_readback(uint16_t object) { return object == kM1PidGet || object == kM2PidGet; }

  bool on_request(const Frame &req) {
    if (req.id != co::COB_SDO_RX_BASE + kNode || req.dlc != 8) {
      return true; // not an SDO request to us (NMT etc.): accept and ignore
    }
    const uint8_t cmd = req.data[0];
    const auto index = static_cast<uint16_t>(co::get_le(&req.data[1], 2));
    const uint8_t sub = req.data[3];
    Frame resp{};
    resp.id = co::COB_SDO_TX_BASE + kNode;
    resp.dlc = 8;
    resp.data[1] = req.data[1];
    resp.data[2] = req.data[2];
    resp.data[3] = sub;
    if (cmd == 0x40) { // upload (read) request
      int32_t value = 0;
      if (is_readback(index)) {
        // sub 1 runs the mirrored command; later subs are served from that
        // reply only while the reads of this object continue uninterrupted
        if (sub == 1) {
          snapshot_object_ = index;
        }
        value = (snapshot_object_ == index) ? field(index, sub) : 0;
      } else {
        snapshot_object_ = 0;
        value = field(index, sub);
      }
      resp.data[0] = 0x43; // expedited, size indicated, 4 bytes
      co::put_le(std::bit_cast<uint32_t>(value), &resp.data[4], 4);
    } else if ((cmd & 0xE3) == 0x23) { // expedited download (write), any size
      const size_t n = 4 - ((cmd >> 2) & 0x3);
      apply_write(index, sub, std::bit_cast<int32_t>(co::get_le(&req.data[4], n)));
      resp.data[0] = 0x60;
    } else {
      return true; // nothing else is used by these tests
    }
    client_.process_frame(resp);
    return true;
  }

  // the setter takes [D, P, I, MaxI, Deadzone, MinPos, MaxPos]; mirror it into
  // the readback's [P, I, D, ...]. Written out here rather than taken from
  // mcp::position_pid_readback_to_setter() so the driver's field shuffle is
  // checked against an independent copy, not against itself.
  void apply_write(uint16_t index, uint8_t sub, int32_t value) {
    const uint16_t get = index == kM1PidSet ? kM1PidGet : index == kM2PidSet ? kM2PidGet : 0;
    if (get == 0) {
      store(index, sub, value);
      return;
    }
    static constexpr std::array<uint8_t, 8> kSetterToReadback{0, 3, 1, 2, 4, 5, 6, 7};
    store(get, kSetterToReadback[sub], value);
  }

  std::map<Key, int32_t> od_;
  uint16_t snapshot_object_{0};
  CanopenClient client_;
};

} // namespace

static void test_fake_models_the_quirk() {
  std::printf("test_fake_models_the_quirk\n");
  FakeMcp266 fake;
  std::error_code ec;
  CHECK(fake.client().read_i32(kM1PidGet, 6, ec) == 0); // read alone: 0, as on the bench
  CHECK(!ec);
  for (uint8_t sub = 1; sub <= 5; ++sub) {
    fake.client().read_i32(kM1PidGet, sub, ec);
    CHECK(!ec);
  }
  CHECK(fake.client().read_i32(kM1PidGet, 6, ec) == 10000); // after subs 1..5: the clamp
  CHECK(!ec);
}

static void test_read_position_limits_cold() {
  std::printf("test_read_position_limits_cold\n");
  FakeMcp266 fake;
  espp::Mcp266 mcp(fake.client());
  std::error_code ec;
  int32_t min_pos = -1, max_pos = -1;
  CHECK(mcp.read_position_limits(espp::Mcp266::Axis::M1, min_pos, max_pos, ec));
  CHECK(!ec);
  CHECK(min_pos == 10000);
  CHECK(max_pos == 18000);
  CHECK(mcp.read_position_limits(espp::Mcp266::Axis::M2, min_pos, max_pos, ec));
  CHECK(min_pos == 25000);
  CHECK(max_pos == 32000);
}

static void test_read_position_limits_after_other_reads() {
  std::printf("test_read_position_limits_after_other_reads\n");
  FakeMcp266 fake;
  espp::Mcp266 mcp(fake.client());
  std::error_code ec;
  float p{}, i{}, d{};
  uint32_t max_i{}, deadzone{};
  int32_t min_pos{}, max_pos{}, count{};
  CHECK(mcp.read_position_pid(espp::Mcp266::Axis::M1, p, i, d, max_i, deadzone, min_pos, max_pos,
                              ec));
  CHECK(min_pos == 10000 && max_pos == 18000);
  CHECK(mcp.read_encoder(espp::Mcp266::Axis::M1, count, ec)); // another object in between
  CHECK(count == 13554);
  min_pos = max_pos = -1;
  CHECK(mcp.read_position_limits(espp::Mcp266::Axis::M1, min_pos, max_pos, ec));
  CHECK(min_pos == 10000);
  CHECK(max_pos == 18000);
}

static void test_configure_position_loop() {
  std::printf("test_configure_position_loop\n");
  FakeMcp266 fake;
  espp::Mcp266 mcp(fake.client());
  std::error_code ec;
  CHECK(mcp.configure_position_loop(espp::Mcp266::Axis::M1, -5000, 40000, ec));
  CHECK(!ec);
  CHECK(fake.field(kM1PidGet, 1) == 103); // stored P kept (non-zero)
  CHECK(fake.field(kM1PidGet, 3) == 868); // D kept in the D slot
  int32_t min_pos = 0, max_pos = 0;
  CHECK(mcp.read_position_limits(espp::Mcp266::Axis::M1, min_pos, max_pos, ec));
  CHECK(min_pos == -5000);
  CHECK(max_pos == 40000);
}

int main() {
  test_fake_models_the_quirk();
  test_read_position_limits_cold();
  test_read_position_limits_after_other_reads();
  test_configure_position_loop();
  if (g_failures == 0) {
    std::printf("ALL MCP266 DRIVER TESTS PASSED\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", g_failures);
  return 1;
}

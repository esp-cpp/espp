// Host-buildable unit tests for the CANopen (CiA 301) wire core and the
// CiA 402 statusword decoding. Build & run with:
//   c++ -std=c++20 -I../include canopen_host_test.cpp -o test && ./test
//
// These tests exercise espp::detail::canopen / espp::detail::ds402 directly so
// they need no ESP-IDF headers.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <string>
#include <vector>

#include "detail/canopen_core.hpp"

namespace co = espp::detail::canopen;
namespace ds = espp::detail::ds402;
using espp::detail::CanFrame;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

static bool bytes_equal(const CanFrame &f, std::span<const uint8_t> expected) {
  return f.dlc == expected.size() && std::equal(expected.begin(), expected.end(), f.data.begin());
}

static void test_nmt() {
  std::printf("test_nmt\n");
  // golden: NMT start node 5 -> id 0x000, data [0x01, 0x05]
  auto f = co::make_nmt(co::NmtCommand::Start, 5);
  CHECK(f.id == 0x000);
  CHECK(!f.extended && !f.rtr);
  const uint8_t start5[] = {0x01, 0x05};
  CHECK(bytes_equal(f, start5));

  // reset-node broadcast (all nodes)
  f = co::make_nmt(co::NmtCommand::ResetNode, 0);
  const uint8_t reset_all[] = {0x81, 0x00};
  CHECK(f.id == 0x000);
  CHECK(bytes_equal(f, reset_all));

  // stop / pre-operational / reset-communication command specifiers
  CHECK(co::make_nmt(co::NmtCommand::Stop, 7).data[0] == 0x02);
  CHECK(co::make_nmt(co::NmtCommand::PreOperational, 7).data[0] == 0x80);
  CHECK(co::make_nmt(co::NmtCommand::ResetCommunication, 7).data[0] == 0x82);
}

static void test_sync_and_pdo() {
  std::printf("test_sync_and_pdo\n");
  auto sync = co::make_sync();
  CHECK(sync.id == 0x080);
  CHECK(sync.dlc == 0);

  // RPDO1 to node 5, 4 bytes of packed data
  const uint8_t payload[] = {0x11, 0x22, 0x33, 0x44};
  auto pdo = co::make_pdo(co::COB_RPDO1_BASE + 5, payload);
  CHECK(pdo.id == 0x205);
  CHECK(bytes_equal(pdo, payload));
}

static void test_heartbeat() {
  std::printf("test_heartbeat\n");
  CanFrame f;
  f.id = 0x705;
  f.dlc = 1;
  uint8_t node = 0;

  f.data[0] = 0x05;
  auto state = co::parse_heartbeat(f, node);
  CHECK(state.has_value() && *state == co::NmtState::Operational && node == 5);

  f.data[0] = 0x00; // boot-up
  state = co::parse_heartbeat(f, node);
  CHECK(state.has_value() && *state == co::NmtState::BootUp);

  f.data[0] = 0x7F;
  state = co::parse_heartbeat(f, node);
  CHECK(state.has_value() && *state == co::NmtState::PreOperational);

  f.data[0] = 0x04;
  state = co::parse_heartbeat(f, node);
  CHECK(state.has_value() && *state == co::NmtState::Stopped);

  // not a heartbeat: SDO response id
  f.id = 0x585;
  CHECK(!co::parse_heartbeat(f, node).has_value());
  // not a heartbeat: 0x700 itself is not a valid node id (node 0)
  f.id = 0x700;
  CHECK(!co::parse_heartbeat(f, node).has_value());
}

static void test_sdo_expedited_download() {
  std::printf("test_sdo_expedited_download\n");
  // golden: write u32 0x12345678 to 0x60FF:00 on node 3
  //   -> id 0x603, data [0x23, 0xFF, 0x60, 0x00, 0x78, 0x56, 0x34, 0x12]
  const uint8_t val32[] = {0x78, 0x56, 0x34, 0x12}; // LE for 0x12345678
  auto f = co::make_sdo_expedited_download(3, 0x60FF, 0x00, val32);
  CHECK(f.id == 0x603);
  const uint8_t golden32[] = {0x23, 0xFF, 0x60, 0x00, 0x78, 0x56, 0x34, 0x12};
  CHECK(bytes_equal(f, golden32));

  // u16 write -> command 0x2B; u8 write -> command 0x2F
  const uint8_t val16[] = {0x0F, 0x00};
  f = co::make_sdo_expedited_download(3, 0x6040, 0x00, val16);
  const uint8_t golden16[] = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
  CHECK(bytes_equal(f, golden16));

  const uint8_t val8[] = {0x03};
  f = co::make_sdo_expedited_download(3, 0x6060, 0x00, val8);
  const uint8_t golden8[] = {0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00};
  CHECK(bytes_equal(f, golden8));

  // download response (scs=3)
  CanFrame resp;
  resp.id = 0x583;
  resp.dlc = 8;
  resp.data = {0x60, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  auto r = co::parse_sdo_response(resp);
  CHECK(r.type == co::SdoResponse::Type::DownloadOk);
  CHECK(r.index == 0x60FF && r.subindex == 0x00);
}

static void test_sdo_expedited_upload() {
  std::printf("test_sdo_expedited_upload\n");
  // request: read 0x6041:00 from node 3 -> id 0x603, [0x40, 0x41, 0x60, 0x00, ...]
  auto req = co::make_sdo_upload_request(3, 0x6041, 0x00);
  CHECK(req.id == 0x603);
  const uint8_t golden_req[] = {0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  CHECK(bytes_equal(req, golden_req));

  // response: expedited u16 0x0637 -> [0x4B, 0x41, 0x60, 0x00, 0x37, 0x06, 0, 0]
  CanFrame resp;
  resp.id = 0x583;
  resp.dlc = 8;
  resp.data = {0x4B, 0x41, 0x60, 0x00, 0x37, 0x06, 0x00, 0x00};
  auto r = co::parse_sdo_response(resp);
  CHECK(r.type == co::SdoResponse::Type::ExpeditedUpload);
  CHECK(r.index == 0x6041 && r.subindex == 0x00);
  CHECK(r.len == 2);
  CHECK(co::get_le(r.data.data(), r.len) == 0x0637);

  // response: expedited u32 (0x43) -> 4 valid bytes
  resp.data = {0x43, 0x00, 0x10, 0x00, 0x92, 0x01, 0x02, 0x00};
  r = co::parse_sdo_response(resp);
  CHECK(r.type == co::SdoResponse::Type::ExpeditedUpload);
  CHECK(r.len == 4);
  CHECK(co::get_le(r.data.data(), r.len) == 0x00020192);

  // response: expedited u8 (0x4F)
  resp.data = {0x4F, 0x01, 0x10, 0x00, 0xA5, 0x00, 0x00, 0x00};
  r = co::parse_sdo_response(resp);
  CHECK(r.type == co::SdoResponse::Type::ExpeditedUpload);
  CHECK(r.len == 1);
  CHECK(r.data[0] == 0xA5);
}

static void test_sdo_abort() {
  std::printf("test_sdo_abort\n");
  // abort: object does not exist (0x06020000) for 0x60FF:00
  CanFrame resp;
  resp.id = 0x583;
  resp.dlc = 8;
  resp.data = {0x80, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x02, 0x06};
  auto r = co::parse_sdo_response(resp);
  CHECK(r.type == co::SdoResponse::Type::Abort);
  CHECK(r.index == 0x60FF && r.subindex == 0x00);
  CHECK(r.abort_code == 0x06020000);
  CHECK(std::strcmp(co::sdo_abort_to_string(r.abort_code),
                    "object does not exist in the object dictionary") == 0);
  CHECK(std::strcmp(co::sdo_abort_to_string(0x06090011), "subindex does not exist") == 0);
  CHECK(std::strcmp(co::sdo_abort_to_string(0x06010002), "attempt to write a read-only object") ==
        0);
  CHECK(std::strcmp(co::sdo_abort_to_string(0x05040001), "invalid or unknown command specifier") ==
        0);
  CHECK(std::strcmp(co::sdo_abort_to_string(0xDEADBEEF), "unknown abort code") == 0);

  // abort frame builder (client-side abort, e.g. toggle error)
  auto f = co::make_sdo_abort(3, 0x1008, 0x00, 0x05030000);
  CHECK(f.id == 0x603);
  const uint8_t golden[] = {0x80, 0x08, 0x10, 0x00, 0x00, 0x00, 0x03, 0x05};
  CHECK(bytes_equal(f, golden));
}

static void test_sdo_segmented_upload() {
  std::printf("test_sdo_segmented_upload\n");
  // segmented upload of the 10-char device name "MCP266 2x6" from 0x1008:00
  const std::string name = "MCP266 2x6";
  CHECK(name.size() == 10);

  // initiate response: scs=2, e=0, s=1 -> 0x41, size = 10
  CanFrame init;
  init.id = 0x583;
  init.dlc = 8;
  init.data = {0x41, 0x08, 0x10, 0x00, 0x0A, 0x00, 0x00, 0x00};
  auto r = co::parse_sdo_response(init);
  CHECK(r.type == co::SdoResponse::Type::SegmentedUploadInit);
  CHECK(r.index == 0x1008 && r.subindex == 0x00);
  CHECK(r.size_indicated && r.total_size == 10);

  co::SdoSegmentedUpload assembler;
  CHECK(assembler.start(r));
  CHECK(!assembler.done());
  CHECK(assembler.next_toggle() == false);

  // first segment request: toggle 0 -> 0x60
  auto req = co::make_sdo_upload_segment_request(3, assembler.next_toggle());
  CHECK(req.id == 0x603);
  CHECK(req.data[0] == 0x60);

  // first segment response: toggle 0, 7 data bytes ("MCP266 "), not last
  //   -> byte0 = 0x00 | toggle<<4 | (7-7)<<1 | 0 = 0x00
  CanFrame seg1;
  seg1.id = 0x583;
  seg1.dlc = 8;
  seg1.data = {0x00, 'M', 'C', 'P', '2', '6', '6', ' '};
  r = co::parse_sdo_response(seg1);
  CHECK(r.type == co::SdoResponse::Type::UploadSegment);
  CHECK(r.toggle == false && !r.last && r.len == 7);
  CHECK(assembler.consume(r));
  CHECK(!assembler.done());
  CHECK(assembler.next_toggle() == true);

  // second segment request: toggle 1 -> 0x70
  req = co::make_sdo_upload_segment_request(3, assembler.next_toggle());
  CHECK(req.data[0] == 0x70);

  // second segment response: toggle 1, 3 data bytes ("2x6"), last
  //   -> byte0 = 0x00 | 1<<4 | (7-3)<<1 | 1 = 0x19
  CanFrame seg2;
  seg2.id = 0x583;
  seg2.dlc = 8;
  seg2.data = {0x19, '2', 'x', '6', 0x00, 0x00, 0x00, 0x00};
  r = co::parse_sdo_response(seg2);
  CHECK(r.type == co::SdoResponse::Type::UploadSegment);
  CHECK(r.toggle == true && r.last && r.len == 3);
  CHECK(assembler.consume(r));
  CHECK(assembler.done());
  CHECK(assembler.data() == name);

  // toggle-bit violation: replaying segment 1 (toggle 0) when 0 is expected
  // again must be rejected once the assembler expects toggle 0 but gets 1
  co::SdoSegmentedUpload bad;
  CHECK(bad.start(co::parse_sdo_response(init)));
  auto seg_toggle1 = co::parse_sdo_response(seg2); // toggle 1 first -> mismatch
  CHECK(!bad.consume(seg_toggle1));
  CHECK(!bad.done());
}

static void test_le_helpers() {
  std::printf("test_le_helpers\n");
  uint8_t buf[4] = {0, 0, 0, 0};
  co::put_le(0xAABBCCDD, buf, 4);
  CHECK(buf[0] == 0xDD && buf[1] == 0xCC && buf[2] == 0xBB && buf[3] == 0xAA);
  CHECK(co::get_le(buf, 4) == 0xAABBCCDD);
  co::put_le(0x1234, buf, 2);
  CHECK(buf[0] == 0x34 && buf[1] == 0x12);
  CHECK(co::get_le(buf, 2) == 0x1234);
}

static void test_ds402_decode() {
  std::printf("test_ds402_decode\n");
  // vectors derived from the CiA 402 state masks (0x4F / 0x6F)
  CHECK(ds::decode_state(0x0637) == ds::State::OperationEnabled);    // 0x37 & 0x6F == 0x27
  CHECK(ds::decode_state(0x0200) == ds::State::NotReadyToSwitchOn);  // & 0x4F == 0x00
  CHECK(ds::decode_state(0x0250) == ds::State::SwitchOnDisabled);    // & 0x4F == 0x40
  CHECK(ds::decode_state(0x0631) == ds::State::ReadyToSwitchOn);     // & 0x6F == 0x21
  CHECK(ds::decode_state(0x0633) == ds::State::SwitchedOn);          // & 0x6F == 0x23
  CHECK(ds::decode_state(0x0617) == ds::State::QuickStopActive);     // & 0x6F == 0x07 (bit 5 low)
  CHECK(ds::decode_state(0x061F) == ds::State::FaultReactionActive); // & 0x4F == 0x0F
  CHECK(ds::decode_state(0x0618) == ds::State::Fault);               // & 0x4F == 0x08
  // higher (mode-specific) bits must not affect the decode
  CHECK(ds::decode_state(0x1637) == ds::State::OperationEnabled);
  CHECK(ds::decode_state(0xF250) == ds::State::SwitchOnDisabled);

  CHECK(std::strcmp(ds::state_to_string(ds::State::OperationEnabled), "Operation enabled") == 0);
  CHECK(std::strcmp(ds::state_to_string(ds::State::Fault), "Fault") == 0);
}

// SDO frames are specified as exactly 8 bytes (CiA 301); anything shorter must
// parse as Unknown instead of decoding missing bytes as protocol fields.
static void test_sdo_malformed_dlc() {
  std::printf("test_sdo_malformed_dlc\n");
  CanFrame f;
  f.id = 0x583;
  f.data = {0x4B, 0x41, 0x60, 0x00, 0x37, 0x06, 0x00, 0x00};
  f.dlc = 7; // one byte short
  CHECK(co::parse_sdo_response(f).type == co::SdoResponse::Type::Unknown);
  f.dlc = 0;
  CHECK(co::parse_sdo_response(f).type == co::SdoResponse::Type::Unknown);
  f.dlc = 8; // and the full-length frame still parses
  CHECK(co::parse_sdo_response(f).type == co::SdoResponse::Type::ExpeditedUpload);
  // extended-id / RTR frames that merely collide with the COB-ID are not SDO
  f.extended = true;
  CHECK(co::parse_sdo_response(f).type == co::SdoResponse::Type::Unknown);
  f.extended = false;
  f.rtr = true;
  CHECK(co::parse_sdo_response(f).type == co::SdoResponse::Type::Unknown);
  f.rtr = false;
  CHECK(co::parse_sdo_response(f).type == co::SdoResponse::Type::ExpeditedUpload);
}

// The initiate response's total_size is remote-supplied: the assembler must
// refuse an oversized reservation up front and stop an un-sized (or lying)
// transfer from growing past the cap while appending.
static void test_sdo_segmented_size_cap() {
  std::printf("test_sdo_segmented_size_cap\n");
  co::SdoResponse init;
  init.type = co::SdoResponse::Type::SegmentedUploadInit;
  init.size_indicated = true;
  init.total_size = 0xFFFFFFFFu; // 4 GiB claim
  co::SdoSegmentedUpload a;
  CHECK(!a.start(init)); // refused up front (default 1 KiB cap)
  init.total_size = 2048;
  CHECK(!a.start(init));      // still over the default cap
  CHECK(a.start(init, 4096)); // fine under an explicit larger cap

  // un-sized transfer: growth is capped while appending
  co::SdoResponse init2;
  init2.type = co::SdoResponse::Type::SegmentedUploadInit;
  init2.size_indicated = false;
  co::SdoSegmentedUpload b;
  CHECK(b.start(init2, 10)); // 10-byte cap
  co::SdoResponse seg;
  seg.type = co::SdoResponse::Type::UploadSegment;
  seg.len = 7;
  seg.last = false;
  seg.toggle = false;
  std::fill(seg.data.begin(), seg.data.end(), 0x41);
  CHECK(b.consume(seg)); // 7 <= 10
  seg.toggle = true;
  CHECK(!b.consume(seg)); // 14 > 10 -> refused
}

int main() {
  test_nmt();
  test_sync_and_pdo();
  test_heartbeat();
  test_sdo_expedited_download();
  test_sdo_expedited_upload();
  test_sdo_abort();
  test_sdo_malformed_dlc();
  test_sdo_segmented_size_cap();
  test_sdo_segmented_upload();
  test_le_helpers();
  test_ds402_decode();

  if (g_failures == 0) {
    std::printf("ALL TESTS PASSED\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", g_failures);
  return 1;
}

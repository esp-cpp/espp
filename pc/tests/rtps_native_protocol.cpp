// Byte-level unit test for the native (espp<->espp) protocol codec
// (rtps/rpc/native_protocol.hpp): the 20-byte in-band request/reply header and
// the native-action goal-reply + feedback framing. No network - pure codec
// round-trips + fixed-layout assertions. Exits 0 iff every check passes.

#include <array>
#include <cstdint>
#include <cstdio>
#include <span>
#include <vector>

#include "rtps/rpc/native_protocol.hpp"

namespace {
int failures = 0;
void expect(bool cond, const char *what) {
  if (cond) {
    std::printf("ok   %s\n", what);
  } else {
    std::printf("FAIL %s\n", what);
    ++failures;
  }
}
} // namespace

int main() {
  using namespace rtps::rpc;

  // --- Request/reply header (20 bytes) round-trip + fixed layout. ---
  {
    NativeHeader h;
    h.client_prefix = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    h.request_id = 0x11223344;
    h.op = NativeOp::REPLY;
    h.flags = 0;
    const std::vector<uint8_t> payload{0x00, 0x01, 0x00, 0x00, 0xAA, 0xBB};
    auto frame = native_encode(h, payload);

    expect(frame.size() == NATIVE_HEADER_SIZE + payload.size(), "encode size");
    // request_id at offset 12 is little-endian.
    expect(frame[12] == 0x44 && frame[13] == 0x33 && frame[14] == 0x22 && frame[15] == 0x11,
           "request_id LE at offset 12");
    expect(frame[16] == static_cast<uint8_t>(NativeOp::REPLY), "op at offset 16");
    expect(std::equal(h.client_prefix.begin(), h.client_prefix.end(), frame.begin()),
           "client_prefix at offset 0");

    NativeHeader out;
    std::span<const uint8_t> out_payload;
    expect(native_decode(frame, out, out_payload), "decode ok");
    expect(out.client_prefix == h.client_prefix && out.request_id == h.request_id && out.op == h.op,
           "decode header round-trip");
    expect(out_payload.size() == payload.size() &&
               std::equal(out_payload.begin(), out_payload.end(), payload.begin()),
           "decode payload round-trip");

    // A frame shorter than the header must be rejected.
    std::vector<uint8_t> tooShort(NATIVE_HEADER_SIZE - 1, 0);
    NativeHeader dummy;
    std::span<const uint8_t> dummySpan;
    expect(!native_decode(tooShort, dummy, dummySpan), "decode rejects short frame");
  }

  // --- Native-action send_goal reply { accepted, goal_handle }. ---
  {
    auto reply = native_make_goal_reply(true, 0x0A0B0C0D);
    bool accepted = false;
    uint32_t handle = 0;
    expect(native_parse_goal_reply(reply, accepted, handle) && accepted && handle == 0x0A0B0C0D,
           "goal reply round-trip (accepted)");
    auto rej = native_make_goal_reply(false, 0);
    bool acc2 = true;
    uint32_t h2 = 99;
    expect(native_parse_goal_reply(rej, acc2, h2) && !acc2, "goal reply round-trip (rejected)");
  }

  // --- Native-action feedback/result { goal_handle, status, payload }. ---
  {
    const std::vector<uint8_t> body{0x00, 0x01, 0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF};
    auto msg = native_make_feedback(0x01020304, NativeGoalStatus::SUCCEEDED, body);
    uint32_t handle = 0;
    NativeGoalStatus status{};
    std::vector<uint8_t> payload;
    expect(native_parse_feedback(msg, handle, status, payload), "feedback parse ok");
    expect(handle == 0x01020304 && status == NativeGoalStatus::SUCCEEDED, "feedback handle+status");
    // payload is re-encapsulated (4-byte encap + the spliced body tail).
    expect(payload.size() == body.size() && payload[4] == 0xDE && payload.back() == 0xEF,
           "feedback payload round-trip");
  }

  if (failures == 0) {
    std::printf("PASS\n");
    return 0;
  }
  std::printf("FAIL: %d\n", failures);
  return 1;
}

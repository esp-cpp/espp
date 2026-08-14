// Validates the action envelope codec (rtps::rpc, action_types.hpp) against the
// EXACT bytes captured from a live rmw_fastrtps (ROS 2 Jazzy) Fibonacci action
// (RMI_AMI_DESIGN.md 3.4), goal UUID 93beb05274a946a6ae8f2540d15de68e, order=5.
// Confirms byte-for-byte wrap/parse of every action envelope.

#include <array>
#include <cstdint>
#include <cstdio>
#include <span>
#include <vector>

#include "rtps/rpc/action_types.hpp"

namespace {
int failures = 0;

std::vector<uint8_t> hex(const char *s) {
  std::vector<uint8_t> v;
  for (; s[0] && s[1]; s += 2) {
    auto nib = [](char c) -> int {
      if (c >= '0' && c <= '9')
        return c - '0';
      if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
      return 0;
    };
    v.push_back(static_cast<uint8_t>((nib(s[0]) << 4) | nib(s[1])));
  }
  return v;
}
void eq(const std::vector<uint8_t> &got, const std::vector<uint8_t> &want, const char *what) {
  if (got == want) {
    std::printf("ok   %s\n", what);
    return;
  }
  std::printf("FAIL %s\n  got  %zu bytes\n  want %zu bytes\n", what, got.size(), want.size());
  ++failures;
}
void expect(bool cond, const char *what) {
  if (cond) {
    std::printf("ok   %s\n", what);
  } else {
    std::printf("FAIL %s\n", what);
    ++failures;
  }
}
constexpr uint8_t kEncap[4] = {0x00, 0x01, 0x00, 0x00};
// kEncap + hex(h) as one vector - the shape of every CDR message here.
std::vector<uint8_t> encap_hex(const char *h) {
  std::vector<uint8_t> v(kEncap, kEncap + 4);
  const auto body = hex(h);
  v.insert(v.end(), body.begin(), body.end());
  return v;
}
} // namespace

int main() {
  using namespace rtps::rpc;
  const std::vector<uint8_t> uuid_bytes = hex("93beb05274a946a6ae8f2540d15de68e");
  GoalUuid uuid{};
  std::copy(uuid_bytes.begin(), uuid_bytes.end(), uuid.begin());

  // SendGoal_Request: UUID + int32 order=5. Captured serializedData (post-encap):
  //   93beb05274a946a6ae8f2540d15de68e 05000000
  {
    std::vector<uint8_t> goal = encap_hex("05000000"); // order=5 as its own CDR msg
    auto msg = wrap_send_goal_request(uuid, goal);
    std::vector<uint8_t> want = encap_hex("93beb05274a946a6ae8f2540d15de68e05000000");
    eq(msg, want, "wrap_send_goal_request");
    GoalUuid gid{};
    std::vector<uint8_t> goal_out;
    expect(unwrap_send_goal_request(msg, gid, goal_out) && gid == uuid && goal_out == goal,
           "unwrap_send_goal_request roundtrip");
  }

  // SendGoal_Response: accepted=true, stamp. Captured: 01000000 a3927e6a dc014325
  {
    auto msg = make_send_goal_response(true, 0x6a7e92a3, 0x254301dc);
    std::vector<uint8_t> want = encap_hex("01000000a3927e6adc014325");
    eq(msg, want, "make_send_goal_response");
    bool accepted = false;
    expect(parse_send_goal_response(msg, accepted) && accepted, "parse_send_goal_response");
  }

  // GetResult_Request: just the UUID.
  {
    auto msg = make_get_result_request(uuid);
    std::vector<uint8_t> want(kEncap, kEncap + 4);
    want.insert(want.end(), uuid_bytes.begin(), uuid_bytes.end());
    eq(msg, want, "make_get_result_request");
    GoalUuid gid{};
    expect(parse_get_result_request(msg, gid) && gid == uuid, "parse_get_result_request");
  }

  // GetResult_Response: status=SUCCEEDED(4), result=int32[] [0,1,1,2,3,5].
  // Captured: 04000000 06000000 00.. 01.. 01.. 02.. 03.. 05..
  {
    std::vector<uint8_t> result =
        encap_hex("06000000000000000100000001000000020000000300000005000000");
    auto msg = wrap_get_result_response(GoalStatus::SUCCEEDED, result);
    std::vector<uint8_t> want =
        encap_hex("0400000006000000000000000100000001000000020000000300000005000000");
    eq(msg, want, "wrap_get_result_response");
    GoalStatus st{};
    std::vector<uint8_t> res_out;
    expect(unwrap_get_result_response(msg, st, res_out) && st == GoalStatus::SUCCEEDED &&
               res_out == result,
           "unwrap_get_result_response roundtrip");
  }

  // FeedbackMessage: UUID + int32[] [0,1,1]. Captured:
  //   93beb05274a946a6ae8f2540d15de68e 03000000 000000000100000001000000
  {
    std::vector<uint8_t> fb = encap_hex("03000000000000000100000001000000");
    auto msg = wrap_feedback(uuid, fb);
    std::vector<uint8_t> want =
        encap_hex("93beb05274a946a6ae8f2540d15de68e03000000000000000100000001000000");
    eq(msg, want, "wrap_feedback");
  }

  // GoalStatusArray: 1 entry, ACCEPTED. Captured:
  //   01000000 93beb05274a946a6ae8f2540d15de68e a3927e6a 298f4425 01000000
  {
    GoalStatusEntry e;
    e.goal_id = uuid;
    e.sec = 0x6a7e92a3;
    e.nsec = 0x25448f29;
    e.status = GoalStatus::ACCEPTED;
    std::array<GoalStatusEntry, 1> entries{e};
    auto msg = make_goal_status_array(entries);
    std::vector<uint8_t> want =
        encap_hex("0100000093beb05274a946a6ae8f2540d15de68ea3927e6a298f442501000000");
    eq(msg, want, "make_goal_status_array");
    std::vector<GoalStatusEntry> out;
    expect(parse_goal_status_array(msg, out) && out.size() == 1 && out[0].goal_id == uuid &&
               out[0].status == GoalStatus::ACCEPTED,
           "parse_goal_status_array roundtrip");
  }

  if (failures == 0) {
    std::printf("PASS\n");
    return 0;
  }
  std::printf("FAIL: %d\n", failures);
  return 1;
}

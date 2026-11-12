// A typed action Result with int64 / float64 fields crosses intact.
//
// The byte-level action codec builds the GetResult_Response by splicing a
// standalone-serialized Result behind the status field at CDR offset 4. CDR
// aligns every field to the stream origin, so a result whose fields had been
// aligned relative to its own offset 0 is off by 4 bytes at every int64 /
// float64 field once it sits at offset 4: the typed espp::ActionServer /
// ActionClient produced wrong bytes for such a Result, and a ROS 2 client
// read garbage. The typed layer now serializes the whole response
// ({status, result} as one message, espp::GetResultResponse<Result>) and
// reads it the same way (ActionGoalHandle::terminate_with_response /
// ActionClient::send_goal_raw).
//
//  1. Byte-exact: GetResultResponse<Result>{SUCCEEDED, {1.5, 7, 2.5}} is the
//     hand-written xcdr1 layout, and the old splice would have been different.
//  2. End to end, two participants: for succeed, abort and cancel the typed
//     client receives exactly the values the typed server put in the Result
//     (and the feedback, whose double sits at an 8-aligned offset).
//
// Exits 0 on success, 1 on failure.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "rtps/rpc/action_types.hpp"
#include "rtps_action.hpp"

using namespace std::chrono_literals;

struct AlignGoal {
  int64_t x;
};
struct AlignResult {
  double first;
  int32_t n;
  double last;
};
struct AlignFeedback {
  int32_t a;
  double b;
};

namespace {
#define CHECK(cond, msg)                                                                           \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                                          \
      return false;                                                                                \
    }                                                                                              \
  } while (0)

void put_f64(std::vector<uint8_t> &v, double d) {
  uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(d));
  std::memcpy(&bits, &d, sizeof(bits));
  for (int i = 0; i < 8; ++i) {
    v.push_back(static_cast<uint8_t>((bits >> (8 * i)) & 0xFF));
  }
}
void put_i32(std::vector<uint8_t> &v, int32_t x) {
  for (int i = 0; i < 4; ++i) {
    v.push_back(static_cast<uint8_t>((static_cast<uint32_t>(x) >> (8 * i)) & 0xFF));
  }
}

bool bytes_exact() {
  const AlignResult result{1.5, 7, 2.5};
  const auto bytes = espp::detail::rtps_serialize<espp::GetResultResponse<AlignResult>>(
      espp::GetResultResponse<AlignResult>{4, result});
  // encapsulation; status int8 at body 0, pad to 8; 1.5; int32 7 + 4 pad; 2.5.
  std::vector<uint8_t> expected{0x00, 0x01, 0x00, 0x00, 0x04, 0, 0, 0, 0, 0, 0, 0};
  put_f64(expected, 1.5);
  put_i32(expected, 7);
  expected.insert(expected.end(), {0, 0, 0, 0});
  put_f64(expected, 2.5);
  CHECK(bytes == expected, "GetResultResponse<Result> is the hand-written xcdr1 layout");
  // What the byte-level splice makes of the same result: the standalone Result
  // (1.5 at its own offset 0) behind status + 3 pad -> 4 bytes shorter and the
  // doubles 4 bytes early. Documents why the typed layer no longer splices.
  const auto spliced = rtps::rpc::wrap_get_result_response(
      rtps::rpc::GoalStatus::SUCCEEDED, espp::detail::rtps_serialize<AlignResult>(result));
  CHECK(spliced != expected, "the byte-level splice differs for a Result with doubles");
  CHECK(spliced.size() == expected.size() - 4, "the splice is the 4 pad bytes short");
  // And the response reads back as one message.
  auto back = espp::detail::rtps_deserialize<espp::GetResultResponse<AlignResult>>(bytes);
  CHECK(back && back->status == 4 && back->result.first == 1.5 && back->result.n == 7 &&
            back->result.last == 2.5,
        "GetResultResponse<Result> round-trips");
  return true;
}

struct Outcome {
  std::mutex m;
  std::condition_variable cv;
  bool done{false};
  espp::GoalStatus status{};
  AlignResult result{};
  std::optional<AlignFeedback> feedback;
};

bool end_to_end() {
  espp::RtpsParticipant server({.log_level = espp::Logger::Verbosity::WARN});
  espp::RtpsParticipant client({.log_level = espp::Logger::Verbosity::WARN});
  CHECK(server.start() && client.start(), "start");

  // x > 0: succeed; x < 0: abort; x == 0: wait for the cancel, then canceled.
  espp::ActionServer<AlignGoal, AlignResult, AlignFeedback> act(
      server, {.action = "/align",
               .type_name = "espp_test::action::dds_::Align",
               .on_goal = [](const AlignGoal &) { return true; },
               .execute =
                   [](auto &h) {
                     const int64_t x = h.goal().x;
                     h.publish_feedback(AlignFeedback{static_cast<int32_t>(x), x * 0.5});
                     if (x > 0) {
                       h.succeed(AlignResult{1.5, 7, 2.5});
                     } else if (x < 0) {
                       h.abort(AlignResult{-1.5, -7, -2.5});
                     } else {
                       while (!h.is_canceling()) {
                         std::this_thread::sleep_for(10ms);
                       }
                       h.canceled(AlignResult{0.5, 1, 0.25});
                     }
                   }});
  espp::ActionClient<AlignGoal, AlignResult, AlignFeedback> cli(
      client, {.action = "/align", .type_name = "espp_test::action::dds_::Align"});
  CHECK(act.is_valid() && cli.is_valid(), "typed action endpoints");
  CHECK(cli.wait_for_server(10s), "wait_for_server");

  const auto run = [&](int64_t x, espp::GoalStatus want, AlignResult want_result,
                       const char *what) -> bool {
    auto out = std::make_shared<Outcome>();
    const bool sent = cli.send_goal(
        AlignGoal{x},
        [out](const AlignFeedback &fb) {
          std::lock_guard<std::mutex> lock(out->m);
          out->feedback = fb;
        },
        [out](espp::GoalStatus st, const AlignResult &r) {
          std::lock_guard<std::mutex> lock(out->m);
          out->status = st;
          out->result = r;
          out->done = true;
          out->cv.notify_all();
        });
    CHECK(sent, "send_goal");
    if (x == 0) {
      // Cancel once the goal is running (its feedback arrived).
      const auto deadline = std::chrono::steady_clock::now() + 10s;
      bool running = false;
      while (!running && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(10ms);
        std::lock_guard<std::mutex> lock(out->m);
        running = out->feedback.has_value();
      }
      CHECK(running, "feedback before cancel");
      CHECK(cli.cancel_goal(), "cancel_goal");
    }
    std::unique_lock<std::mutex> lock(out->m);
    CHECK(out->cv.wait_for(lock, 15s, [&] { return out->done; }), "result arrived");
    // The feedback is published the moment the goal starts, on a reliable
    // topic the client's feedback reader may not have matched yet at the
    // first goal (wait_for_server covers the three services only): the writer
    // replays it once the match lands, which can be after the result. Give
    // it a moment rather than racing discovery.
    if (!out->feedback) {
      const auto fb_deadline = std::chrono::steady_clock::now() + 5s;
      while (!out->feedback && std::chrono::steady_clock::now() < fb_deadline) {
        lock.unlock();
        std::this_thread::sleep_for(20ms);
        lock.lock();
      }
    }
    std::printf("%s: status %d, result {%g, %d, %g}, feedback {%d, %g}\n", what,
                static_cast<int>(out->status), out->result.first, out->result.n, out->result.last,
                out->feedback ? out->feedback->a : -1, out->feedback ? out->feedback->b : -1.0);
    CHECK(out->status == want, "terminal status");
    CHECK(out->result.first == want_result.first && out->result.n == want_result.n &&
              out->result.last == want_result.last,
          "the result's doubles and int32 arrived intact");
    CHECK(out->feedback && out->feedback->a == static_cast<int32_t>(x) &&
              out->feedback->b == x * 0.5,
          "the feedback's int32 and double arrived intact");
    return true;
  };
  CHECK(run(3, espp::GoalStatus::SUCCEEDED, AlignResult{1.5, 7, 2.5}, "succeed"), "succeed");
  CHECK(run(-3, espp::GoalStatus::ABORTED, AlignResult{-1.5, -7, -2.5}, "abort"), "abort");
  CHECK(run(0, espp::GoalStatus::CANCELED, AlignResult{0.5, 1, 0.25}, "cancel"), "cancel");
  return true;
}
} // namespace

int main() {
  if (!bytes_exact() || !end_to_end()) {
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}

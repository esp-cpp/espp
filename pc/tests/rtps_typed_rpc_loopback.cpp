// Showcase + self-test of the TYPED, espp-idiomatic RMI/AMI wrappers
// (espp::ServiceServer/ServiceClient, espp::ActionServer/ActionClient) - the
// service/action analogue of the typed Publisher<T>/Subscriber<T>. Reflectable
// structs are (de)serialized to CDR automatically by the `cdr` component, so
// application code never touches bytes. Exercises both the ROS 2-interoperable
// and the native protocol. Exits 0 iff every typed round-trip succeeds.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>

#include "rtps_action.hpp"
#include "rtps_service.hpp"

using namespace std::chrono_literals;

// Reflectable message structs - the fields map straight to CDR (ROS 2 wire form).
struct AddReq {
  int64_t a;
  int64_t b;
};
struct AddResp {
  int64_t sum;
};
struct FibGoal {
  int32_t order;
};
struct FibSeq {
  std::vector<int32_t> sequence;
}; // Result + Feedback

int main() {
  espp::RtpsParticipant server({.log_level = espp::Logger::Verbosity::WARN});
  espp::RtpsParticipant client({.log_level = espp::Logger::Verbosity::WARN});
  if (!server.start() || !client.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }
  using P = espp::RtpsProtocol;

  bool svc_ros = false, svc_native = false, act_ros = false, act_native = false;

  // --- Typed service (ROS 2 + native): AddTwoInts ---
  espp::ServiceServer<AddReq, AddResp> ros_srv(
      server, {.service = "/add_two_ints",
               .type_name = "example_interfaces::srv::dds_::AddTwoInts",
               .handler = [](const AddReq &r) { return AddResp{r.a + r.b}; }});
  espp::ServiceServer<AddReq, AddResp> nat_srv(
      server, {.service = "/mul",
               .type_name = "espp::native::Mul",
               .handler = [](const AddReq &r) { return AddResp{r.a * r.b}; },
               .protocol = P::NATIVE});
  espp::ServiceClient<AddReq, AddResp> ros_cli(
      client,
      {.service = "/add_two_ints", .type_name = "example_interfaces::srv::dds_::AddTwoInts"});
  espp::ServiceClient<AddReq, AddResp> nat_cli(
      client, {.service = "/mul", .type_name = "espp::native::Mul", .protocol = P::NATIVE});

  // --- Typed action (ROS 2 + native): Fibonacci-like sequence ---
  auto make_exec = [](auto &h) {
    const int32_t order = h.goal().order;
    std::vector<int32_t> seq{0, 1};
    for (int32_t i = 1; i < order; ++i) {
      seq.push_back(seq[i] + seq[i - 1]);
      h.publish_feedback(FibSeq{seq});
      std::this_thread::sleep_for(40ms);
    }
    h.succeed(FibSeq{seq});
  };
  espp::ActionServer<FibGoal, FibSeq, FibSeq> ros_act(
      server, {.action = "/fibonacci",
               .type_name = "example_interfaces::action::dds_::Fibonacci",
               .on_goal = [](const FibGoal &g) { return g.order > 0; },
               .execute = make_exec});
  espp::ActionServer<FibGoal, FibSeq, FibSeq> nat_act(
      server, {.action = "/countup",
               .type_name = "espp::native::CountUp",
               .on_goal = [](const FibGoal &g) { return g.order > 0; },
               .execute = make_exec,
               .protocol = P::NATIVE});
  espp::ActionClient<FibGoal, FibSeq, FibSeq> ros_act_cli(
      client, {.action = "/fibonacci", .type_name = "example_interfaces::action::dds_::Fibonacci"});
  espp::ActionClient<FibGoal, FibSeq, FibSeq> nat_act_cli(
      client, {.action = "/countup", .type_name = "espp::native::CountUp", .protocol = P::NATIVE});

  std::this_thread::sleep_for(2s); // discovery

  // Services: blocking typed calls.
  if (auto r = ros_cli.call(AddReq{7, 35}, 5s)) {
    svc_ros = (r->sum == 42);
  }
  if (auto r = nat_cli.call(AddReq{6, 7}, 5s)) {
    svc_native = (r->sum == 42);
  }
  std::printf("typed service ros=%d native=%d\n", svc_ros, svc_native);

  // Actions: typed feedback + result.
  auto run_action = [](auto &cli, int order, const std::vector<int32_t> &expected) {
    std::mutex m;
    std::condition_variable cv;
    bool done = false;
    std::atomic<int> fb{0};
    std::vector<int32_t> got;
    espp::GoalStatus status{};
    cli.send_goal(
        FibGoal{order}, [&](const FibSeq &) { fb.fetch_add(1); },
        [&](espp::GoalStatus st, const FibSeq &res) {
          std::lock_guard<std::mutex> lk(m);
          status = st;
          got = res.sequence;
          done = true;
          cv.notify_one();
        });
    std::unique_lock<std::mutex> lk(m);
    cv.wait_for(lk, 15s, [&] { return done; });
    return status == espp::GoalStatus::SUCCEEDED && got == expected && fb.load() > 0;
  };
  act_ros = run_action(ros_act_cli, 5, {0, 1, 1, 2, 3, 5});
  act_native = run_action(nat_act_cli, 5, {0, 1, 1, 2, 3, 5});
  std::printf("typed action ros=%d native=%d\n", act_ros, act_native);

  server.stop();
  client.stop();

  const bool ok = svc_ros && svc_native && act_ros && act_native;
  std::printf("%s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}

// rtps_embedded component example (ESP32 / esp32-ethernet-kit).
//
// Brings up an espp::RtpsParticipant over Ethernet and demonstrates the typed
// APIs: a Publisher<T>/Subscriber<T> pair (pub/sub), typed ServiceServer +
// ActionServer the device hosts, and typed ServiceClient + ActionClient the
// device runs. See components/rtps_embedded/example/README.md and the RMI/AMI
// docs (doc/en/protocols/rtps_rmi_ami.rst).

#include <atomic>
#include <chrono>
#include <thread>

#include "esp32-ethernet-kit.hpp"

#include "logger.hpp"
#include "rtps_action.hpp"
#include "rtps_participant.hpp"
#include "rtps_pubsub.hpp"
#include "rtps_service.hpp"
#include "timer.hpp"

using namespace std::chrono_literals;

// std_msgs/msg/String as a plain reflectable struct. The typed Publisher<T> /
// Subscriber<T> serialize any such struct to the DDS wire format (ROS 2 / classic
// CDR) with no manual (de)serialization in application code.
struct StringMsg {
  std::string data;
};

// Reflectable request/reply + goal/result structs for the typed service + action
// servers below. Their fields map straight to CDR, matching example_interfaces
// so a ROS 2 client (ros2 service call / ros2 action send_goal) can drive them.
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
};

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "rtps_example", .level = espp::Logger::Verbosity::INFO});

  //! [rtps example]
  // Bring up Ethernet (DHCP server on 192.168.4.1/24 so a directly-attached PC
  // gets an address); any espp network interface works - the RTPS participant
  // only needs the interface's IPv4 address.
  auto &board = espp::Esp32EthernetKit::get();
  bool eth_ok = board.initialize_ethernet({
      .mode = espp::Esp32EthernetKit::DhcpMode::SERVER,
      .on_link_up = [&]() { logger.info("Ethernet link up"); },
      .on_link_down = [&]() { logger.warn("Ethernet link down"); },
  });
  if (!eth_ok) {
    logger.error("Ethernet initialization failed");
    return;
  }
  logger.info("Waiting for Ethernet link...");
  while (!board.is_ethernet_connected()) {
    std::this_thread::sleep_for(100ms);
  }
  auto eth_ip = board.ethernet_ip();
  const std::string interface_address =
      fmt::format("{}.{}.{}.{}", esp_ip4_addr1_16(&eth_ip), esp_ip4_addr2_16(&eth_ip),
                  esp_ip4_addr3_16(&eth_ip), esp_ip4_addr4_16(&eth_ip));
  logger.info("Ethernet up, IP {}", interface_address);

  // RTPS/DDS participant (embeddedRTPS engine behind the espp facade). The
  // topics pair with the FastDDS host peer in example/pc/host_pubsub.cpp; for
  // ROS 2 instead, use topic "rt/<name>" with type "<pkg>::msg::dds_::<Type>_"
  // (e.g. "rt/chatter" + "std_msgs::msg::dds_::String_").
  constexpr const char *pub_topic = "mcu_to_pc";
  constexpr const char *sub_topic = "pc_to_mcu";
  constexpr const char *type_name = "std_msgs::msg::String";

  // Automatic locals: they RAII-clean up in reverse order on any early return
  // (subscriber/publisher stop referencing the participant before it is
  // destroyed), and the trailing while(true) keeps them alive in normal use.
  espp::RtpsParticipant participant({
      .interface_address = interface_address,
      .on_publisher_matched = [&]() { logger.info("publisher matched a remote reader"); },
      .on_subscriber_matched = [&]() { logger.info("subscriber matched a remote writer"); },
      .log_level = espp::Logger::Verbosity::INFO,
  });
  if (!participant.start()) {
    logger.error("Failed to start the RTPS participant");
    return;
  }

  // Typed reliable publisher: publish StringMsg structs directly (HEARTBEAT/
  // ACKNACK-acknowledged, retransmitted to matched readers). No manual CDR.
  using Reliability = espp::RtpsParticipant::Reliability;
  espp::Publisher<StringMsg> publisher(participant, {
                                                        .topic = pub_topic,
                                                        .type_name = type_name,
                                                        .reliability = Reliability::RELIABLE,
                                                    });
  // Typed subscriber: receive StringMsg structs directly.
  espp::Subscriber<StringMsg> subscriber(
      participant, {
                       .topic = sub_topic,
                       .type_name = type_name,
                       .on_message = [&](const StringMsg &msg) { logger.info("rx: {}", msg.data); },
                   });
  if (!publisher.is_valid() || !subscriber.is_valid()) {
    logger.error("Failed to create the typed publisher/subscriber");
    return;
  }

  // Publish a counter periodically via the typed publisher.
  uint32_t counter = 0;
  espp::Timer publish_timer({
      .name = "rtps_pub",
      .period = std::chrono::milliseconds(CONFIG_RTPS_EXAMPLE_ANNOUNCE_PERIOD_MS),
      .callback =
          [&]() {
            if (publisher.publish(StringMsg{fmt::format("msg {}", counter++)})) {
              logger.info("tx: msg {}", counter - 1);
            } else {
              logger.warn("tx dropped (history full)");
            }
            return false; // keep the timer running
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  logger.info("started: pub='{}' sub='{}' type='{}'", pub_topic, sub_topic, type_name);

#ifdef RTPS_WITH_RPC
  // Typed service (RMI) server: a ROS 2 client can `ros2 service call
  // /add_two_ints example_interfaces/srv/AddTwoInts "{a: 7, b: 35}"` and get 42.
  // No manual CDR - the reflectable AddReq/AddResp structs are (de)serialized for
  // us. (Compiled out when CONFIG_RTPS_ENABLE_RPC is disabled.)
  espp::ServiceServer<AddReq, AddResp> add_service(
      participant, {
                       .service = "/add_two_ints",
                       .type_name = "example_interfaces::srv::dds_::AddTwoInts",
                       .handler =
                           [&](const AddReq &r) {
                             logger.info("service add_two_ints: {} + {} = {}", r.a, r.b, r.a + r.b);
                             return AddResp{r.a + r.b};
                           },
                   });

  // Typed action (AMI) server: a ROS 2 client can `ros2 action send_goal
  // /fibonacci example_interfaces/action/Fibonacci "{order: 5}"` and receive
  // feedback + the [0,1,1,2,3,5] result. execute() runs on its own thread.
  espp::ActionServer<FibGoal, FibSeq, FibSeq> fib_action(
      participant, {
                       .action = "/fibonacci",
                       .type_name = "example_interfaces::action::dds_::Fibonacci",
                       .on_goal = [&](const FibGoal &g) { return g.order > 0; },
                       .execute =
                           [&](auto &h) {
                             const int32_t order = h.goal().order;
                             std::vector<int32_t> seq{0, 1};
                             for (int32_t i = 1; i < order; ++i) {
                               seq.push_back(seq[i] + seq[i - 1]);
                               h.publish_feedback(FibSeq{seq});
                               std::this_thread::sleep_for(200ms);
                             }
                             h.succeed(FibSeq{seq});
                             logger.info("action fibonacci({}) done", order);
                           },
                   });
  if (!add_service.is_valid() || !fib_action.is_valid()) {
    logger.error("Failed to create the typed service/action servers");
    return;
  }
  logger.info("service '/add_two_ints' + action '/fibonacci' ready");

  // Also demonstrate the CLIENT side on-device: a typed service client + action
  // client that call services a peer hosts ("/peer_add_two_ints", "/peer_fib").
  // Run a ROS 2 / rclpy server (or another espp device) for those names to see a
  // full round-trip; until then the calls simply time out (logged), which still
  // exercises the client API on-target. (Calling this device's OWN services is
  // not possible - a participant filters out its own messages.)
  espp::ServiceClient<AddReq, AddResp> add_client(
      participant,
      {.service = "/peer_add_two_ints", .type_name = "example_interfaces::srv::dds_::AddTwoInts"});
  espp::ActionClient<FibGoal, FibSeq, FibSeq> fib_client(
      participant,
      {.action = "/peer_fib", .type_name = "example_interfaces::action::dds_::Fibonacci"});

  // Only one action goal in flight at a time: without a peer the goal never
  // completes, so re-sending on every tick would leak a pending goal each time.
  // The service call() below self-cleans on its 1s timeout, so it can run freely.
  std::atomic<bool> fib_in_flight{false};
  espp::Timer rpc_client_timer({
      .name = "rtps_rpc_client",
      .period = 5s,
      .callback =
          [&]() {
            // Typed blocking service call (RMI).
            if (auto resp = add_client.call(AddReq{20, 22}, 1s)) {
              logger.info("[client] /peer_add_two_ints(20,22) = {}", resp->sum);
            } else {
              logger.info("[client] /peer_add_two_ints: no reply (peer serving it?)");
            }
            // Typed action goal (AMI) with typed feedback + result. Skip if the
            // previous goal has not finished (e.g. no peer is serving it).
            if (!fib_in_flight.exchange(true)) {
              fib_client.send_goal(
                  FibGoal{5}, [&](const FibSeq &) { /* per-feedback */ },
                  [&](espp::GoalStatus status, const FibSeq &res) {
                    logger.info("[client] /peer_fib result: status={} len={}",
                                static_cast<int>(status), res.sequence.size());
                    fib_in_flight.store(false);
                  });
            }
            return false; // keep the timer running
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  if (!add_client.is_valid() || !fib_client.is_valid()) {
    logger.error("Failed to create the typed service/action clients");
    return;
  }
  logger.info("client for '/peer_add_two_ints' + '/peer_fib' running");

#if CONFIG_RTPS_EXAMPLE_SECOND_PARTICIPANT
  // Purely additive on-device SELF-TEST (Kconfig, default off): a SECOND
  // participant with its own service + action clients that call THIS device's own
  // /add_two_ints and /fibonacci servers, for a full local round-trip (a
  // participant filters out its own messages, so the loopback needs a distinct
  // participant). This roughly doubles the RTPS engine RAM - only enable on a
  // target with headroom (e.g. PSRAM).
  espp::RtpsParticipant selftest_participant({
      .interface_address = interface_address,
      .log_level = espp::Logger::Verbosity::WARN,
  });
  if (!selftest_participant.start()) {
    logger.error("Failed to start the self-test participant");
    return;
  }
  espp::ServiceClient<AddReq, AddResp> selftest_add_client(
      selftest_participant,
      {.service = "/add_two_ints", .type_name = "example_interfaces::srv::dds_::AddTwoInts"});
  espp::ActionClient<FibGoal, FibSeq, FibSeq> selftest_fib_client(
      selftest_participant,
      {.action = "/fibonacci", .type_name = "example_interfaces::action::dds_::Fibonacci"});
  espp::Timer selftest_timer({
      .name = "rtps_selftest",
      .period = 5s,
      .callback =
          [&]() {
            if (auto resp = selftest_add_client.call(AddReq{20, 22}, 2s)) {
              logger.info("[self-test] /add_two_ints(20,22) = {} ({})", resp->sum,
                          resp->sum == 42 ? "PASS" : "FAIL");
            } else {
              logger.warn("[self-test] /add_two_ints: no reply");
            }
            selftest_fib_client.send_goal(
                FibGoal{5}, [&](const FibSeq &) {},
                [&](espp::GoalStatus status, const FibSeq &res) {
                  const std::vector<int32_t> expected{0, 1, 1, 2, 3, 5};
                  const bool ok = status == espp::GoalStatus::SUCCEEDED && res.sequence == expected;
                  logger.info("[self-test] /fibonacci(5) len={} ({})", res.sequence.size(),
                              ok ? "PASS" : "FAIL");
                });
            return false; // keep the timer running
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  if (!selftest_add_client.is_valid() || !selftest_fib_client.is_valid()) {
    logger.error("Failed to create the self-test clients");
    return;
  }
  logger.info("self-test participant round-tripping the local service + action");
#endif // CONFIG_RTPS_EXAMPLE_SECOND_PARTICIPANT
#endif // RTPS_WITH_RPC
  //! [rtps example]

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

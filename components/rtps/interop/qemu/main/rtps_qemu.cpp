// RTPS interop firmware for QEMU (components/rtps/interop/qemu).
//
// The device side of the QEMU interop harness (../run_qemu_interop.sh): the
// embedded RTPS engine - the esp32 configuration profile, lwIP, FreeRTOS - on
// QEMU's OpenCores Ethernet, talking to FastDDS, CycloneDDS / ROS 2 and the
// espp host engine on a tap device. The harness drives everything from the
// host side and asserts on the "RTPSTEST ..." lines this firmware logs:
//
//   RTPSTEST ready ip=<ip>           participant started
//   RTPSTEST incarnation cur=<hex> prev=<hex|none>   the GUID prefix incarnation of
//                                    this boot and, after a software reset, of the
//                                    previous one (which the engine disposes first)
//   RTPSTEST restart                 the restart case called /qemu_add_two_ints with a
//                                    magic request: the firmware tears its participant
//                                    down and creates a new one (a new GUID incarnation,
//                                    which first disposes the previous one). Not
//                                    esp_restart: QEMU's emulated chip panics in
//                                    ESP-IDF startup after a software reset. Ignored in
//                                    the first 5 s of a participant.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <esp_heap_caps.h>
#include <esp_netif.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "qemu.hpp"

#include "logger.hpp"
#include "rtps/entities/Domain.hpp"
#include "rtps_action.hpp"
#include "rtps_participant.hpp"
#include "rtps_pubsub.hpp"
#include "rtps_service.hpp"
#include "timer.hpp"

namespace {
// The memory suite (../run_qemu_interop.sh memory) reads these. The engine's
// objects are compile-time sized (the limits profile and the capacity knobs)
// but the Domain that holds them is allocated from the heap at start(), so
// their sizeof is the footprint a configuration commits to. The heap is split
// the way the ESP32 is: internal RAM, where every task stack and lwIP buffer
// must live and which runs out first, against PSRAM.
void log_mem_sizes(espp::Logger &logger) {
  logger.info("RTPSTEST mem sizeof domain={} participant={} stateful_writer={} "
              "stateless_writer={} stateful_reader={} stateless_reader={} facade={}",
              sizeof(rtps::Domain), sizeof(rtps::Participant), sizeof(rtps::StatefulWriter),
              sizeof(rtps::StatelessWriter), sizeof(rtps::StatefulReader),
              sizeof(rtps::StatelessReader), sizeof(espp::RtpsParticipant));
}

void log_mem_heap(espp::Logger &logger, const char *stage) {
  logger.info("RTPSTEST mem heap {} internal={} internal_min={} internal_largest={} psram={} "
              "psram_largest={}",
              stage, heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
              heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
              heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
              heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
              heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
}
} // namespace

using namespace std::chrono_literals;

// std_msgs/msg/String as a reflectable struct (ROS 2 CDR)
struct StringMsg {
  std::string data;
};

// example_interfaces/srv/AddTwoInts
struct AddReq {
  int64_t a;
  int64_t b;
};
struct AddResp {
  int64_t sum;
};

// example_interfaces/action/Fibonacci
struct FibGoal {
  int32_t order;
};
struct FibResult {
  std::vector<int32_t> sequence;
};
struct FibFeedback {
  std::vector<int32_t> sequence;
};

static std::string ip4_to_string(esp_ip4_addr_t ip) {
  char buf[16] = {};
  esp_ip4addr_ntoa(&ip, buf, sizeof(buf));
  return buf;
}

// One participant lifetime: endpoints, timers, the status loop. Returns when a
// RESTART sample asks for a new participant (the restart case); everything it
// owns is destroyed on the way out, and app_main builds the next one.
static void run_participant(espp::Logger &logger, const std::string &interface_address,
                            const std::string &interface_netmask);

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "rtps_qemu", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting the RTPS QEMU interop firmware");

  auto &qemu = espp::Qemu::get();

  // Network: a static address (CI, on the harness's tap device) or a DHCP
  // client (a developer's idf.py qemu on the user-mode NAT), per Kconfig.
  espp::Qemu::EthernetConfig eth_config{};
  // CLIENT is the right mode for both: espp::Ethernet's contract (DhcpMode
  // in ethernet.hpp) is that CLIENT with a non-zero ip_info uses that address
  // statically and never starts the DHCP client; SERVER would run a DHCP
  // server for peers. The CI run proves it: the tap has no DHCP server and
  // the device comes up at the configured address.
  eth_config.mode = espp::Qemu::DhcpMode::CLIENT;
#if CONFIG_RTPS_QEMU_STATIC_IP
  eth_config.ip_info.ip.addr = esp_ip4addr_aton(CONFIG_RTPS_QEMU_IP);
  eth_config.ip_info.netmask.addr = esp_ip4addr_aton(CONFIG_RTPS_QEMU_NETMASK);
  eth_config.ip_info.gw.addr = esp_ip4addr_aton(CONFIG_RTPS_QEMU_GATEWAY);
#endif
  std::atomic<bool> have_ip{false};
  eth_config.on_got_ip = [&](esp_ip4_addr_t ip) {
    logger.info("Ethernet up: {}", ip4_to_string(ip));
    have_ip = true;
  };
  eth_config.on_link_down = [&]() { logger.warn("Ethernet link down"); };
  if (!qemu.initialize_ethernet(eth_config)) {
    logger.error("RTPSTEST fatal ethernet");
    return;
  }
  // wait for the address (DHCP takes a second on the NAT; static is immediate)
  for (int i = 0; i < 300 && !have_ip; ++i) {
    std::this_thread::sleep_for(100ms);
  }
  if (!have_ip) {
    logger.error("RTPSTEST fatal no_ip");
    return;
  }
  const std::string interface_address = ip4_to_string(qemu.ethernet_ip());
  esp_netif_ip_info_t ip_info{};
  esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("ETH_DEF"), &ip_info);
  const std::string interface_netmask = ip4_to_string(ip_info.netmask);

  while (true) {
    run_participant(logger, interface_address, interface_netmask);
    logger.info("RTPSTEST participant torn down, starting a new one");
  }
}

static void run_participant(espp::Logger &logger, const std::string &interface_address,
                            const std::string &interface_netmask) {
  const auto boot_time = std::chrono::steady_clock::now();
  // The restart case calls /qemu_add_two_ints with this magic pair to ask the
  // device to recreate its participant (see the service handler below).
  constexpr int64_t kRestartMagicA = 424242;
  constexpr int64_t kRestartMagicB = 0;
  std::atomic<bool> restart_requested{false};
  // Counters for the status line (the callbacks run on the engine's threads)
  std::atomic<uint32_t> tx_count{0};
  std::atomic<uint32_t> rx_count{0};
  std::atomic<uint32_t> be_rx_count{0};
  std::atomic<uint32_t> big_rx_count{0};
  std::atomic<uint32_t> pub_matches{0};
  std::atomic<uint32_t> sub_matches{0};

  log_mem_sizes(logger);
  log_mem_heap(logger, "boot");
  espp::RtpsParticipant participant({
      .interface_address = interface_address,
      .interface_netmask = interface_netmask,
      .on_publisher_matched =
          [&]() {
            pub_matches++;
            logger.info("RTPSTEST pub_matched");
          },
      .on_subscriber_matched =
          [&]() {
            sub_matches++;
            logger.info("RTPSTEST sub_matched");
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  if (!participant.start()) {
    logger.error("RTPSTEST fatal participant");
    return;
  }
  log_mem_heap(logger, "started");

  using Reliability = espp::RtpsParticipant::Reliability;
  constexpr const char *type_name = "std_msgs::msg::dds_::String_";

  espp::Publisher<StringMsg> publisher(participant, {
                                                        .topic = "rt/qemu_tx",
                                                        .type_name = type_name,
                                                        .reliability = Reliability::RELIABLE,
                                                    });
  espp::Subscriber<StringMsg> subscriber(participant, {
                                                          .topic = "rt/qemu_rx",
                                                          .type_name = type_name,
                                                          .reliability = Reliability::RELIABLE,
                                                          .on_message =
                                                              [&](const StringMsg &msg) {
                                                                rx_count++;
                                                                logger.info("RTPSTEST rx {}",
                                                                            msg.data);
                                                              },
                                                      });
  espp::Publisher<StringMsg> be_publisher(participant, {
                                                           .topic = "rt/qemu_be_tx",
                                                           .type_name = type_name,
                                                           .reliability = Reliability::BEST_EFFORT,
                                                       });
  espp::Subscriber<StringMsg> be_subscriber(participant,
                                            {
                                                .topic = "rt/qemu_be_rx",
                                                .type_name = type_name,
                                                .reliability = Reliability::BEST_EFFORT,
                                                .on_message =
                                                    [&](const StringMsg &msg) {
                                                      be_rx_count++;
                                                      logger.info("RTPSTEST be_rx {}", msg.data);
                                                    },
                                            });
  // Samples larger than one UDP datagram arrive as DATA_FRAG and are
  // reassembled (CONFIG_RTPS_ENABLE_FRAGMENTATION); only the length is logged.
  espp::Subscriber<StringMsg> big_subscriber(
      participant, {
                       .topic = "rt/qemu_big",
                       .type_name = type_name,
                       .reliability = Reliability::RELIABLE,
                       .on_message =
                           [&](const StringMsg &msg) {
                             big_rx_count++;
                             logger.info("RTPSTEST big_rx len={}", msg.data.size());
                           },
                   });
  if (!publisher.is_valid() || !subscriber.is_valid() || !be_publisher.is_valid() ||
      !be_subscriber.is_valid() || !big_subscriber.is_valid()) {
    logger.error("RTPSTEST fatal endpoints");
    return;
  }

#ifdef RTPS_WITH_RPC
  espp::ServiceServer<AddReq, AddResp> add_service(
      participant, {
                       .service = "/qemu_add_two_ints",
                       .type_name = "example_interfaces::srv::dds_::AddTwoInts",
                       .handler =
                           [&](const AddReq &r) {
                             logger.info("RTPSTEST srv {}+{}={}", r.a, r.b, r.a + r.b);
                             // The restart case triggers through this reliable,
                             // already-matched request/reply channel (a topic
                             // sample to a freshly-matched reliable reader is
                             // not reliably delivered on every RMW). The magic
                             // request restarts the participant AFTER the reply
                             // goes out (the status loop tears down ~2 s later);
                             // ignored in the participant's first 5 s so an
                             // in-flight retry does not restart the new one.
                             if (r.a == kRestartMagicA && r.b == kRestartMagicB) {
                               const auto up_s = std::chrono::duration_cast<std::chrono::seconds>(
                                                     std::chrono::steady_clock::now() - boot_time)
                                                     .count();
                               if (up_s < 5) {
                                 logger.info("RTPSTEST restart ignored (participant {} s old)",
                                             up_s);
                               } else if (!restart_requested.exchange(true)) {
                                 logger.info("RTPSTEST restart");
                               }
                             }
                             return AddResp{r.a + r.b};
                           },
                   });
  if (!add_service.is_valid()) {
    logger.error("RTPSTEST fatal service");
    return;
  }
  // The action server's execute worker is a task created with the server (its
  // stack is part of the "endpoints" stage); a goal runs on it.
  espp::ActionServer<FibGoal, FibResult, FibFeedback> fib_action(
      participant, {
                       .action = "/qemu_fibonacci",
                       .type_name = "example_interfaces::action::dds_::Fibonacci",
                       .on_goal = [](const FibGoal &g) { return g.order >= 0 && g.order <= 20; },
                       .execute =
                           [&](auto &h) {
                             FibResult r;
                             // Reserved up front: the memory step measures the
                             // action, not the vector's growth.
                             r.sequence.reserve(static_cast<std::size_t>(h.goal().order));
                             int32_t a = 0, b = 1;
                             for (int32_t i = 0; i < h.goal().order && !h.is_canceling(); ++i) {
                               r.sequence.push_back(a);
                               const int32_t next = a + b;
                               a = b;
                               b = next;
                               FibFeedback fb;
                               fb.sequence.reserve(r.sequence.size());
                               fb.sequence = r.sequence;
                               h.publish_feedback(fb);
                             }
                             logger.info("RTPSTEST action order={} len={}", h.goal().order,
                                         r.sequence.size());
                             if (h.is_canceling()) {
                               h.canceled(r);
                             } else {
                               h.succeed(r);
                             }
                           },
                   });
  if (!fib_action.is_valid()) {
    logger.error("RTPSTEST fatal action");
    return;
  }
#endif

  log_mem_heap(logger, "endpoints");

  // Periodic publishers: one reliable, one best-effort
  uint32_t counter = 0;
  espp::Timer publish_timer({
      .name = "qemu_pub", // not rtps_*: the stack margin below is the engine's, not this test's
      .period = 200ms,
      .callback =
          [&]() {
            const uint32_t n = counter++;
            if (publisher.publish(StringMsg{"qemu " + std::to_string(n)})) {
              tx_count++;
              logger.info("RTPSTEST tx {}", n);
            }
            be_publisher.publish(StringMsg{"qemu_be " + std::to_string(n)});
            return false;
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });

  logger.info("RTPSTEST ready ip={}", interface_address);
  {
    // The prefix incarnation (bytes 10-11 of every GUID prefix this boot) and
    // the previous one (the previous boot's, kept across a software reset, or
    // the previous participant's after a RESTART): the engine disposes that
    // incarnation on its first announce cycles, so peers that still held it
    // forget it at once.
    const auto prev = rtps::Domain::currentPreviousGuidIncarnation();
    logger.info("RTPSTEST incarnation cur={:04x} prev={}", rtps::Domain::currentGuidIncarnation(),
                prev ? fmt::format("{:04x}", *prev) : std::string{"none"});
  }

  // The engine's tasks' stack margin: the smallest high-water mark (bytes of
  // stack never touched since the task started) over the rtps_* tasks (receive
  // workers, reactor, job retry, protocol, reply hold) and the action servers'
  // exec_* workers. Read every status period, the minimum so far is reported:
  // a thin margin is a crash waiting for a slightly deeper call.
  std::vector<TaskStatus_t> tasks;
  uint32_t stack_min = UINT32_MAX;
  uint32_t stack_min_worker = UINT32_MAX;
  std::string stack_min_task = "none";
  const auto sample_stacks = [&]() {
    const auto count = uxTaskGetNumberOfTasks();
    tasks.resize(count + 4);
    const auto n = uxTaskGetSystemState(tasks.data(), tasks.size(), nullptr);
    for (UBaseType_t i = 0; i < n; ++i) {
      const std::string_view name{tasks[i].pcTaskName};
      if (!name.starts_with("rtps_") && !name.starts_with("exec_")) {
        continue;
      }
      const uint32_t margin = tasks[i].usStackHighWaterMark * sizeof(StackType_t);
      if (margin < stack_min) {
        stack_min = margin;
        stack_min_task = std::string{name};
      }
      if (name.starts_with("rtps_worker") && margin < stack_min_worker) {
        stack_min_worker = margin;
      }
    }
  };

  // Status line every 2 s: the soak and leak checks read heap and min_heap
  const auto t0 = std::chrono::steady_clock::now();
  while (!restart_requested) {
    std::this_thread::sleep_for(2s);
    const auto up =
        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - t0)
            .count();
    sample_stacks();
    logger.info("RTPSTEST status up={} tx={} rx={} be_rx={} big_rx={} pub_matches={} "
                "sub_matches={} heap={} min_heap={} int={} int_min={} int_largest={} psram={} "
                "stack_min={} stack_min_task={} stack_min_worker={}",
                up, tx_count.load(), rx_count.load(), be_rx_count.load(), big_rx_count.load(),
                pub_matches.load(), sub_matches.load(), heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
                heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT),
                heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
                heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                heap_caps_get_free_size(MALLOC_CAP_SPIRAM), stack_min == UINT32_MAX ? 0 : stack_min,
                stack_min_task, stack_min_worker == UINT32_MAX ? 0 : stack_min_worker);
  }
}

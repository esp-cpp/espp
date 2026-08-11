// RTPS interop publisher (Phase 1 of components/rtps_embedded/REFACTOR_PLAN.md).
//
// Exercises the espp::RtpsParticipant facade end-to-end: publishes CDR string
// samples (serialized with espp::CdrWriter) so an external DDS peer (FastDDS or
// a ROS 2 node via rmw_fastrtps) can subscribe. Defaults follow the ROS 2
// conventions for std_msgs/String on /chatter.
//
// Usage: rtps_embedded_interop_pub [topic] [type] [reliable(0|1)] [count] [period_ms]
// [interface_ip] Exits 0 after publishing `count` samples.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "cdr.hpp"
#include "rtps_participant.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  const char *topic = (argc > 1) ? argv[1] : "rt/chatter";
  const char *type = (argc > 2) ? argv[2] : "std_msgs::msg::dds_::String_";
  const bool reliable = (argc > 3) ? (std::atoi(argv[3]) != 0) : true;
  const int count = (argc > 4) ? std::atoi(argv[4]) : 30;
  const int period_ms = (argc > 5) ? std::atoi(argv[5]) : 200;
  const char *interface_ip = (argc > 6) ? argv[6] : ""; // "" -> auto-detect

  espp::RtpsParticipant participant({
      .interface_address = interface_ip,
      .log_level = espp::Logger::Verbosity::INFO,
  });
  if (!participant.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }
  using Reliability = espp::RtpsParticipant::Reliability;
  if (!participant.add_writer({
          .topic = topic,
          .type_name = type,
          .reliability = reliable ? Reliability::RELIABLE : Reliability::BEST_EFFORT,
      })) {
    std::printf("FAIL: add_writer\n");
    return 1;
  }
  std::printf("interop_pub: topic=%s type=%s reliable=%d count=%d\n", topic, type, reliable ? 1 : 0,
              count);

  // Give SPDP/SEDP a moment to match before the first sample.
  std::this_thread::sleep_for(2s);

  espp::CdrWriter writer; // default: CDR_LE with encapsulation header
  int sent = 0;
  for (int i = 0; i < count; i++) {
    writer.reset();
    writer.write_string("espp interop " + std::to_string(i));
    if (participant.publish(topic, writer.buffer())) {
      sent++;
      std::printf("sent %d\n", sent);
      std::fflush(stdout);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }

  // Let reliable retransmits drain before tearing down.
  std::this_thread::sleep_for(1s);
  std::printf("DONE sent=%d\n", sent);
  participant.stop();
  return sent == count ? 0 : 1;
}

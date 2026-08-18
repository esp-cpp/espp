#include <atomic>
#include <chrono>
#include <thread>

#include "twai.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Twai Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting TWAI (CAN 2.0) example!");

  //! [twai example]
  // Count the frames we receive back, so we can assert the round-trip.
  static std::atomic<int> num_received{0};

  // The receive callback runs in task context (not ISR). It is invoked for
  // every frame the node receives - in loopback mode that is every frame we
  // transmit.
  auto on_receive = [&](const espp::Twai::Message &msg) {
    num_received++;
    logger.info("RX: {}", msg);
  };

  auto on_state_change = [&](const espp::Twai::StateChange &sc) {
    logger.warn("State change: {} -> {}", static_cast<int>(sc.old_state),
                static_cast<int>(sc.new_state));
  };

  // NOTE: We use Mode::LOOPBACK (internal loopback + self-test) so this example
  // runs on a bare devkit with NO CAN transceiver and NO other node on the bus.
  // To talk to a real bus, use Mode::NORMAL with a transceiver (e.g.
  // SN65HVD230) wired to the tx/rx GPIOs, and make sure at least one other node
  // is present to acknowledge frames.
  espp::Twai twai({
      .tx_gpio = 5, // GPIO5 (change to match your board / transceiver)
      .rx_gpio = 4, // GPIO4 (change to match your board / transceiver)
      .baudrate = 500000,
      .mode = espp::Twai::Mode::LOOPBACK,
      .tx_queue_depth = 5,
      .on_receive = on_receive,
      .on_state_change = on_state_change,
      .log_level = espp::Logger::Verbosity::INFO,
  });

  std::error_code ec;
  if (!twai.initialize(ec)) {
    logger.error("Failed to initialize TWAI: {}", ec.message());
    return;
  }

  // Transmit a few frames; in loopback mode each is received back via on_receive.
  static constexpr int num_to_send = 5;
  for (int i = 0; i < num_to_send; i++) {
    espp::Twai::Message msg;
    msg.id = 0x100 + i;
    msg.extended = false;
    msg.rtr = false;
    msg.dlc = 4;
    msg.data = {static_cast<uint8_t>(i), 0xDE, 0xAD, 0xBE, 0, 0, 0, 0};
    if (twai.transmit(msg, ec)) {
      logger.info("TX: {}", msg);
    } else {
      logger.error("Failed to transmit frame {}: {}", i, ec.message());
    }
    std::this_thread::sleep_for(50ms);
  }

  // give the RX task a moment to drain the queue
  std::this_thread::sleep_for(200ms);

  logger.info("Sent {} frames, received {} frames", num_to_send, num_received.load());

  // print out the node status / statistics
  twai_node_status_t status;
  twai_node_record_t record;
  if (twai.get_info(status, record, ec)) {
    logger.info("Node state={}, tx_err={}, rx_err={}, bus_err_num={}",
                static_cast<int>(status.state), status.tx_error_count, status.rx_error_count,
                record.bus_err_num);
  }

  // assert the round trip
  if (num_received.load() == num_to_send) {
    logger.info("SUCCESS: all {} frames were received back in loopback mode", num_to_send);
  } else {
    logger.error("FAILURE: expected {} frames, got {}", num_to_send, num_received.load());
  }
  //! [twai example]

  logger.info("TWAI example complete!");
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

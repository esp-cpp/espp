#include <chrono>
#include <thread>

#include "switch2_pro.hpp"

#include "logger.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "switch2_pro example", .level = espp::Logger::Verbosity::INFO});

  //! [switch2_pro example]
  // Bring up the emulated Switch 2 Pro Controller. init() verifies the pairing
  // crypto against a known-answer vector, builds the custom Nintendo GATT
  // services, configures security so the console (not BLE SMP) drives pairing,
  // and starts advertising with Nintendo manufacturer data.
  //
  // DEBUG log level traces every command write and response on the serial
  // monitor — flip to INFO for quieter output once things work.
  espp::Switch2Pro controller({
      .device_name = "Pro Controller",
      .log_level = espp::Logger::Verbosity::DEBUG,
  });

  if (!controller.init()) {
    logger.error("failed to initialize Switch2Pro");
    return;
  }
  logger.info("advertising — on the Switch 2, open Controllers > Pair, and watch "
              "the log for the connect interval, the 0x15 pairing exchange, and "
              "either 'pairing finalised' or a disconnect reason");

  // Report the current button/stick state once input streaming is enabled
  // (staged in a follow-up milestone). For now we just build a report to show
  // the API and let advertising/pairing run.
  espp::switch2::Pro2InputReport report;
  while (true) {
    report.reset();
    report.increment_counter();
    report.set_a(true);              // hold A as a placeholder
    report.set_left_stick(0.f, 0.f); // centered
    controller.set_input_report(report);

    logger.info("paired: {}", controller.is_paired());
    std::this_thread::sleep_for(1s);
  }
  //! [switch2_pro example]
}

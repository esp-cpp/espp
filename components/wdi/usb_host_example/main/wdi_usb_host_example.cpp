#include <chrono>
#include <thread>

#include "esp_random.h"

#include "logger.hpp"
#include "wdi_usb_host.hpp"

using namespace std::chrono_literals;

// WDI (Wheelchair Digital Interface) USB **host** example: act as the wheelchair
// (the USB host) and talk to an attached WDI HID accessory (for example another
// ESP32-S3 running the wdi usb_example). The host receives Control reports (the
// accessory's joystick + flags), replies to Keepalive / Request-Feedback, and
// runs the keepalive watchdog that drive-disables if the accessory goes quiet.
//
// SAFETY: this only *emulates* the wheelchair side for development. Do not wire a
// real chair's motion to on_control without the manufacturer's guidance.
extern "C" void app_main(void) {
  espp::Logger logger({.tag = "WDI USB Host", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting WDI USB host example");

  // Build this host's identity (manufacturer id + 14 random bytes).
  uint8_t rnd[14];
  esp_fill_random(rnd, sizeof(rnd));
  auto uuid = espp::WdiHost::make_host_uuid(
      static_cast<uint16_t>(espp::wdi::ManufacturerId::LuciMobility), rnd);

  espp::WdiUsbHost host({
      .on_control =
          [&](const espp::wdi::ControlReport &c) {
            logger.info("control: x={} y={} drive_enable={}", c.x, c.y,
                        c.has(espp::wdi::ControlBit::DriveEnable));
          },
      .on_connected = [&] { logger.info("WDI accessory connected"); },
      .on_disconnected = [&] { logger.warn("WDI accessory disconnected -> DRIVE DISABLE"); },
      .host_uuid = uuid,
      .log_level = espp::Logger::Verbosity::INFO,
  });

  // Report a plausible chair status back to the accessory.
  espp::wdi::FeedbackReport fb;
  fb.set(espp::wdi::FeedbackBit::DriveEnabled);
  fb.speed = 3;   // 0..15
  fb.profile = 1; // 0..15
  host.set_feedback(fb);

  std::error_code ec;
  if (!host.initialize(ec)) {
    logger.error("Failed to initialize USB host: {}", ec.message());
    return;
  }
  logger.info("USB host ready; plug in a WDI HID accessory.");

  // Run the keepalive watchdog. poll() fires on_disconnected if the accessory
  // stops sending (3 missed 257 ms windows).
  while (true) {
    host.poll();
    std::this_thread::sleep_for(50ms);
  }
}

#include <chrono>
#include <thread>

#include "esp_random.h"

#include "NimBLEDevice.h"

#include "logger.hpp"
#include "wdi_ble_central.hpp"

using namespace std::chrono_literals;

// WDI (Wheelchair Digital Interface) BLE **central** example: act as the
// wheelchair (BLE central) and talk to a WDI peripheral accessory (for example
// another ESP running the wdi ble_example). The central scans for the WDI
// service, connects, receives Control reports, replies to Keepalive /
// Request-Feedback, and runs the keepalive watchdog that drive-disables if the
// accessory goes quiet.
//
// SAFETY: this only *emulates* the wheelchair side for development. Do not wire a
// real chair's motion to on_control without the manufacturer's guidance.
extern "C" void app_main(void) {
  espp::Logger logger({.tag = "WDI BLE Host", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting WDI BLE central example");

  NimBLEDevice::init("espp WDI host");

  // Build this host's identity (manufacturer id + 14 random bytes).
  uint8_t rnd[14];
  esp_fill_random(rnd, sizeof(rnd));
  auto uuid = espp::WdiHost::make_host_uuid(
      static_cast<uint16_t>(espp::wdi::ManufacturerId::LuciMobility), rnd);

  espp::WdiBleCentral host({
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
  fb.speed = 3;
  fb.profile = 1;
  host.set_feedback(fb);

  // Scan + connect (retrying until a WDI peripheral is found), then run the
  // keepalive watchdog. If the link drops, scan again.
  while (true) {
    if (!host.is_connected()) {
      std::error_code ec;
      logger.info("scanning for a WDI peripheral...");
      if (!host.scan_and_connect(5000, ec)) {
        logger.warn("no peripheral yet ({}); retrying", ec.message());
        std::this_thread::sleep_for(1s);
        continue;
      }
    }
    host.poll();
    std::this_thread::sleep_for(50ms);
  }
}

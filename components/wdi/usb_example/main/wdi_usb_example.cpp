#include <chrono>
#include <cmath>
#include <thread>

#include "logger.hpp"
#include "wdi_usb.hpp"

using namespace std::chrono_literals;

// WDI (Wheelchair Digital Interface) USB peripheral example: enumerate as a WDI
// HID device (an accessory / alternative joystick) and drive a wheelchair (the USB
// host) over the standard WDI reports. The device sends Control reports +
// keepalives and receives Feedback; here we sweep a demo joystick pattern.
extern "C" void app_main(void) {
  espp::Logger logger({.tag = "WDI USB", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting WDI USB peripheral example");

  espp::WdiUsbPeripheral wdi({
      .on_feedback =
          [&](const espp::wdi::FeedbackReport &f) {
            logger.info("feedback: drive_enabled={} speed={} {:.1f} mph",
                        f.has(espp::wdi::FeedbackBit::DriveEnabled), f.speed, f.velocity_mph());
          },
      .on_keepalive_response =
          [&](const espp::wdi::HostUuid &u) {
            logger.info("host uuid: manufacturer=0x{:04x}", u.manufacturer_id());
          },
      .product = "espp WDI",
      .log_level = espp::Logger::Verbosity::INFO,
  });

  std::error_code ec;
  if (!wdi.initialize(ec)) {
    logger.error("Failed to initialize USB device: {}", ec.message());
    return;
  }
  logger.info("WDI HID device ready; connect it to a WDI host (wheelchair).");

  // Drive loop: sweep the joystick in a slow circle with drive enabled, poll for
  // keepalives, and ask for feedback once a second. A real accessory would map
  // physical inputs here instead. write_hid_report no-ops until the host mounts +
  // polls the interface, so this is safe to run before a host connects.
  int step = 0;
  while (true) {
    espp::wdi::ControlReport c;
    const float angle = (step % 60) / 60.0f * 2.0f * 3.14159265f;
    c.x = static_cast<int8_t>(80.0f * std::sin(angle));  // right/left
    c.y = static_cast<int8_t>(-80.0f * std::cos(angle)); // forward/reverse
    c.set(espp::wdi::ControlBit::DriveEnable);
    wdi.send_control(c); // resets the keepalive timer
    if (step % 20 == 0)
      wdi.request_feedback();
    wdi.poll(); // send a keepalive if one is due
    ++step;
    std::this_thread::sleep_for(50ms);
  }
}

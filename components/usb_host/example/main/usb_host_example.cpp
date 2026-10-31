#include <chrono>
#include <thread>

#include "logger.hpp"
#include "usb_host.hpp"

using namespace std::chrono_literals;

// USB Host (HID) example: act as a USB host, enumerate attached HID devices
// (mice, keyboards, gamepads, or vendor HID devices such as an espp WDI
// peripheral) and log each device plus a hex dump of every Input report it
// sends. Plug the ESP32-S3 (in host mode) into a USB HID device and watch the
// monitor.
//
// NOTE: the board must be able to source VBUS to the attached device (a board
// with a USB-A host port / VBUS switch, or a self-powered hub). The console
// runs on UART0 because the native USB-OTG port is used for the host role.

//! [usb_host_example]
extern "C" void app_main(void) {
  espp::Logger logger({.tag = "USB Host", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting USB HID host example");

  espp::UsbHost host({
      .on_device_connected =
          [&](const std::shared_ptr<espp::UsbHost::HidDevice> &device) {
            auto info = device->info();
            auto params = device->params();
            logger.info("connected: '{}' '{}' VID={:#06x} PID={:#06x} iface={} proto={}",
                        info.manufacturer, info.product, info.vid, info.pid,
                        params.interface_number, params.protocol);
            auto desc = device->report_descriptor();
            logger.info("  report descriptor: {} bytes", desc.size());

            // Log every Input report this device sends (device -> host).
            device->set_input_callback([&logger](std::span<const uint8_t> data) {
              logger.info("input report ({} bytes): {::#04x}", data.size(), data);
            });
          },
      .on_device_disconnected =
          [&](const std::shared_ptr<espp::UsbHost::HidDevice> &device) {
            logger.info("disconnected: PID={:#06x}", device->info().pid);
          },
      // .should_open = [](const auto &info, const auto &) { return info.vid == 0x1209; },
      .log_level = espp::Logger::Verbosity::INFO,
  });

  std::error_code ec;
  if (!host.initialize(ec)) {
    logger.error("Failed to initialize USB host: {}", ec.message());
    return;
  }
  logger.info("USB host ready; plug in a USB HID device.");

  while (true) {
    logger.debug("connected HID devices: {}", host.devices().size());
    std::this_thread::sleep_for(2s);
  }
}
//! [usb_host_example]

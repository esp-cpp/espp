// USB HID host example for the M5Stack Tab5: show a connected HID device's
// Input reports on the screen, decoding a 3Dconnexion SpaceMouse's six axes
// and buttons.
//
// The Tab5's USB-A jack is on the ESP32-P4's high-speed USB-OTG controller,
// which is the USB Host Library's default on that target, and the BSP turns the
// jack's 5 V on with the IO expanders. Plug a SpaceMouse (or any HID device:
// mouse, keyboard, gamepad) into it. The console is on the USB-C port
// (USB-Serial-JTAG, the other controller), so `idf.py monitor` keeps working.

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

#include "m5stack-tab5.hpp"

#include "logger.hpp"
#include "usb_host.hpp"

#include "gui.hpp"
#include "spacemouse_decoder.hpp"

using namespace std::chrono_literals;

//! [usb_host_tab5_example]
extern "C" void app_main(void) {
  espp::Logger logger({.tag = "USB Host Tab5", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting USB HID host (SpaceMouse) example");

  // --- board: IO expanders (USB-A 5 V), LCD, LVGL display, touch ---------------
  auto &tab5 = espp::M5StackTab5::get();
  if (!tab5.initialize_io_expanders()) {
    logger.error("Failed to initialize the IO expanders");
    return;
  }
  if (!tab5.initialize_lcd()) {
    logger.error("Failed to initialize the LCD");
    return;
  }
  const size_t pixel_buffer_size = tab5.display_width() * tab5.display_height();
  if (!tab5.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize the display");
    return;
  }
  tab5.brightness(75.0f);

  static Gui gui({.log_level = espp::Logger::Verbosity::INFO});
  gui.set_status_text("USB host starting...");

  // --- state shared between the USB dispatch task and the main loop ------------
  std::mutex state_mutex;
  SpaceMouseDecoder decoder;
  bool current_is_spacemouse = false;
  std::vector<uint8_t> last_report; // the most recent raw Input report
  std::atomic<uint32_t> report_count{0};

  // --- USB host --------------------------------------------------------------
  espp::UsbHost host({
      .on_device_connected =
          [&](const std::shared_ptr<espp::UsbHost::HidDevice> &device) {
            const auto info = device->info();
            const auto params = device->params();
            const bool is_spacemouse = SpaceMouseDecoder::is_spacemouse_vendor(info.vid);
            logger.info("connected: '{}' '{}' VID={:#06x} PID={:#06x} iface={} proto={}{}",
                        info.manufacturer, info.product, info.vid, info.pid,
                        params.interface_number, params.protocol,
                        is_spacemouse ? " (SpaceMouse)" : "");
            {
              std::lock_guard<std::mutex> lock(state_mutex);
              decoder.reset();
              current_is_spacemouse = is_spacemouse;
            }
            gui.set_device({.connected = true,
                            .product = info.product,
                            .manufacturer = info.manufacturer,
                            .vid = info.vid,
                            .pid = info.pid,
                            .interface_number = params.interface_number,
                            .protocol = params.protocol,
                            .report_descriptor_bytes = device->report_descriptor().size(),
                            .is_spacemouse = is_spacemouse});
            gui.set_status_text(is_spacemouse ? "Move the cap: translation (blue) and rotation "
                                                "(orange) axes; press the buttons."
                                              : "Raw Input reports are shown below.");

            // every Input report (device -> host): decode a SpaceMouse's, count
            // and show the raw bytes of all of them
            device->set_input_callback([&](std::span<const uint8_t> data) {
              report_count.fetch_add(1);
              bool decoded = false;
              SpaceMouseDecoder::State state;
              {
                std::lock_guard<std::mutex> lock(state_mutex);
                last_report.assign(data.begin(), data.end());
                if (current_is_spacemouse) {
                  decoded = decoder.decode(data);
                  state = decoder.state();
                }
              }
              if (decoded)
                gui.set_spacemouse_state(state);
              logger.debug("input report ({} bytes): {::#04x}", data.size(), data);
            });
          },
      .on_device_disconnected =
          [&](const std::shared_ptr<espp::UsbHost::HidDevice> &device) {
            logger.info("disconnected: PID={:#06x}", device->info().pid);
            {
              std::lock_guard<std::mutex> lock(state_mutex);
              current_is_spacemouse = false;
            }
            gui.set_device(Gui::DeviceInfo{});
            gui.set_status_text("Device removed. Plug a device into the USB-A port.");
          },
      .log_level = espp::Logger::Verbosity::INFO,
  });

  std::error_code ec;
  if (!host.initialize(ec)) {
    logger.error("Failed to initialize the USB host: {}", ec.message());
    gui.set_status_text(fmt::format("USB host failed: {}", ec.message()));
    return;
  }
  logger.info("USB host ready; plug a HID device into the USB-A port.");
  gui.set_status_text("Plug a device into the USB-A port.");
  //! [usb_host_tab5_example]

  // --- main loop: report rate + the latest raw report for the screen ----------
  // The raw-report line is refreshed here (a few times a second) rather than on
  // every report so a fast device does not swamp the LVGL label.
  uint32_t last_count = 0;
  auto last_time = std::chrono::steady_clock::now();
  while (true) {
    std::this_thread::sleep_for(250ms);
    const auto now = std::chrono::steady_clock::now();
    const uint32_t count = report_count.load();
    const float seconds = std::chrono::duration<float>(now - last_time).count();
    const float rate = seconds > 0 ? static_cast<float>(count - last_count) / seconds : 0.0f;
    last_count = count;
    last_time = now;

    const auto devices = host.devices();
    if (devices.empty())
      continue;
    std::string summary;
    std::vector<uint8_t> report;
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      report = last_report;
      if (current_is_spacemouse) {
        const auto &s = decoder.state();
        summary = fmt::format("T {} / R {} / B {} reports, {} unknown", s.translation_reports,
                              s.rotation_reports, s.button_reports, s.unknown_reports);
      } else {
        summary = fmt::format("{} reports", count);
      }
    }
    gui.set_last_report(report, rate);
    gui.set_status_text(summary);
  }
}

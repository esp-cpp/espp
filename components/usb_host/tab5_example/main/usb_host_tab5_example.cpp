// USB HID host example for the M5Stack Tab5: show a connected HID device's
// Input reports on the screen, decoding a 3Dconnexion SpaceMouse's six axes
// and buttons.
//
// The Tab5's USB-A jack is on the ESP32-P4's high-speed USB-OTG controller,
// which is the USB Host Library's default on that target, and the BSP turns the
// jack's 5 V on with the IO expanders. Plug a SpaceMouse (or any HID device:
// mouse, keyboard, gamepad) into it. The console is on the USB-C port
// (USB-Serial-JTAG, the other controller), so `idf.py monitor` keeps working.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

#include "esp_log.h"
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
  // the HID class driver only says at debug level which interfaces it found or
  // skipped; it logs on device events only, so this costs nothing at run time
  esp_log_level_set("hid-host", ESP_LOG_DEBUG);

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
  bool current_is_keyboard = false;
  uint8_t keyboard_modifiers = 0;
  std::array<uint8_t, 6> keyboard_keys{};
  std::vector<uint8_t> last_report; // the most recent raw Input report
  std::atomic<uint32_t> report_count{0};

  // --- USB host --------------------------------------------------------------
  espp::UsbHost host({
      .on_device_connected =
          [&](const std::shared_ptr<espp::UsbHost::HidDevice> &device) {
            const auto info = device->info();
            const auto params = device->params();
            const bool is_spacemouse = SpaceMouseDecoder::is_spacemouse_vendor(info.vid);
            // bInterfaceSubClass 1 = boot interface, bInterfaceProtocol 1 = keyboard
            const bool is_keyboard = params.sub_class == 1 && params.protocol == 1;
            logger.info("connected: '{}' '{}' VID={:#06x} PID={:#06x} iface={} proto={}{}",
                        info.manufacturer, info.product, info.vid, info.pid,
                        params.interface_number, params.protocol,
                        is_spacemouse ? " (SpaceMouse)" : "");
            // Ask a keyboard for the boot protocol: a fixed 8-byte report
            // ([modifier bits][reserved][6 key usage ids]) whatever its own
            // report descriptor says, so no per-keyboard parsing is needed.
            if (is_keyboard) {
              std::error_code proto_ec;
              if (!device->set_protocol(HID_REPORT_PROTOCOL_BOOT, proto_ec))
                logger.warn("keyboard did not accept the boot protocol: {}", proto_ec.message());
            }
            // A device may expose several HID interfaces (a keyboard usually adds a
            // mouse / media one): the card and the decoders follow the most
            // specific interface seen, so a generic sibling never downgrades a
            // keyboard or SpaceMouse to "raw reports".
            bool primary = false; // this interface is the one the card shows
            {
              std::lock_guard<std::mutex> lock(state_mutex);
              const bool have_specific = current_is_spacemouse || current_is_keyboard;
              primary = is_spacemouse || is_keyboard || !have_specific;
              if (primary) {
                decoder.reset();
                current_is_spacemouse = is_spacemouse;
                current_is_keyboard = is_keyboard;
                keyboard_modifiers = 0;
                keyboard_keys.fill(0);
              }
            }
            if (!primary) {
              // count its reports too (raw line), but leave the card alone
              device->set_input_callback([&](std::span<const uint8_t> data) {
                report_count.fetch_add(1);
                std::lock_guard<std::mutex> lock(state_mutex);
                last_report.assign(data.begin(), data.end());
              });
              return;
            }
            gui.set_device({.connected = true,
                            .product = info.product,
                            .manufacturer = info.manufacturer,
                            .vid = info.vid,
                            .pid = info.pid,
                            .interface_number = params.interface_number,
                            .protocol = params.protocol,
                            .report_descriptor_bytes = device->report_descriptor().size(),
                            .is_spacemouse = is_spacemouse,
                            .is_keyboard = is_keyboard});
            gui.set_status_text(is_spacemouse ? "Move the cap: translation (blue) and rotation "
                                                "(orange) axes; press the buttons."
                                : is_keyboard ? "Type: pressed keys light up on the keyboard."
                                              : "Raw Input reports are shown below.");

            // every Input report (device -> host): decode a SpaceMouse's, count
            // and show the raw bytes of all of them
            // Only decode here (USB dispatch task, at the device's report rate);
            // the screen is refreshed from the main loop at a fixed rate so a
            // fast device cannot flood LVGL.
            device->set_input_callback(
                [&, is_spacemouse, is_keyboard](std::span<const uint8_t> data) {
                  report_count.fetch_add(1);
                  std::lock_guard<std::mutex> lock(state_mutex);
                  last_report.assign(data.begin(), data.end());
                  // decode by THIS interface's kind (captured), not a shared flag
                  if (is_spacemouse) {
                    decoder.decode(data);
                  } else if (is_keyboard && data.size() >= 8) {
                    keyboard_modifiers = data[0];
                    std::copy_n(data.begin() + 2, keyboard_keys.size(), keyboard_keys.begin());
                  }
                });
          },
      .on_device_disconnected =
          [&](const std::shared_ptr<espp::UsbHost::HidDevice> &device) {
            logger.info("disconnected: PID={:#06x} iface={}", device->info().pid,
                        device->params().interface_number);
            // only the last interface of the device clears the screen
            if (!host.devices().empty())
              return;
            {
              std::lock_guard<std::mutex> lock(state_mutex);
              current_is_spacemouse = false;
              current_is_keyboard = false;
            }
            gui.set_device(Gui::DeviceInfo{});
            gui.set_status_text("Device removed. Plug a device into the USB-A port.");
          },
      // The Tab5's USB-A jack is on the P4's high-speed OTG controller, which is
      // peripheral 0 (the library default); say so explicitly so a board wired
      // the other way only has to change this number.
      .port = 0,
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

  // --- main loop: the screen is refreshed here at a fixed rate ----------------
  // Axes / buttons at ~30 Hz (plenty for the eye), the raw-report line, rate and
  // status a few times a second, whatever the device's report rate.
  constexpr auto kAxisRefresh = 33ms;
  constexpr int kSlowEvery = 8; // ~4 Hz
  uint32_t last_count = 0;
  size_t last_enumerated = 0;
  int slow_tick = 0;
  auto last_time = std::chrono::steady_clock::now();
  while (true) {
    std::this_thread::sleep_for(kAxisRefresh);
    bool spacemouse = false;
    bool keyboard = false;
    SpaceMouseDecoder::State state;
    uint8_t modifiers = 0;
    std::array<uint8_t, 6> keys{};
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      spacemouse = current_is_spacemouse;
      keyboard = current_is_keyboard;
      if (spacemouse)
        state = decoder.state();
      modifiers = keyboard_modifiers;
      keys = keyboard_keys;
    }
    if (spacemouse)
      gui.set_spacemouse_state(state);
    else if (keyboard)
      gui.set_keyboard_state(modifiers, keys);
    if (++slow_tick < kSlowEvery)
      continue;
    slow_tick = 0;

    const auto now = std::chrono::steady_clock::now();
    const uint32_t count = report_count.load();
    const float seconds = std::chrono::duration<float>(now - last_time).count();
    const float rate = seconds > 0 ? static_cast<float>(count - last_count) / seconds : 0.0f;
    last_count = count;
    last_time = now;

    // enumeration, HID or not: a device counted here but not opened as HID
    // (no HID interface / rejected) never reaches the callbacks above
    const size_t enumerated = host.num_usb_devices();
    const auto devices = host.devices();
    if (enumerated != last_enumerated) {
      logger.info("USB devices enumerated: {} (HID opened: {})", enumerated, devices.size());
      last_enumerated = enumerated;
      // a device that enumerated but was not opened as HID: show what it is
      // (the HID driver is silent when it finds no usable HID interface)
      if (enumerated > 0 && devices.empty()) {
        std::this_thread::sleep_for(1s); // give the HID driver time to open it first
        if (host.devices().empty())
          host.print_usb_devices();
      }
    }
    if (devices.empty()) {
      gui.set_status_text(
          enumerated == 0
              ? "Plug a device into the USB-A port."
              : fmt::format("{} USB device(s) enumerated, none opened as HID", enumerated));
      continue;
    }
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

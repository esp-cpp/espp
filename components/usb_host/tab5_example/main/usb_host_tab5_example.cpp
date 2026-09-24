// USB HID host example for the M5Stack Tab5: show what a connected HID device
// sends, decoded from its own report descriptor.
//
// Every HID interface the host opens is classified from its report descriptor
// with hid-rp's runtime ReportMap -- keyboard, mouse or gamepad -- and decoded
// with the matching hid-rp decoder; a 3Dconnexion SpaceMouse is recognised by
// vendor id and decoded into its six axes with the SpaceMouse report classes.
// The screen shows one panel per kind attached, so a keyboard with a mouse
// interface, or a wireless receiver carrying a keyboard, a mouse and a gamepad,
// shows all of them at once.
//
// The Tab5's USB-A jack is on the ESP32-P4's high-speed USB-OTG controller,
// which is the USB Host Library's default on that target, and the BSP switches
// the jack's 5 V through an IO expander. The console is on the USB-C port
// (USB-Serial-JTAG, the other controller). If devices on the jack stall in
// enumeration only while `idf.py monitor` is attached, see the README's
// hardware notes.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <thread>
#include <variant>
#include <vector>

#include "m5stack-tab5.hpp"

#include "hid-rp-report-map.hpp"
#include "logger.hpp"
#include "usb_host.hpp"

#include "gui.hpp"
#include "spacemouse_decoder.hpp"

using namespace std::chrono_literals;

namespace {
/// One opened HID interface: what it was recognised as, the decoder built from
/// its report descriptor, and (for a keyboard) its latest report, since a
/// keyboard may spread its keys over two interfaces (boot + NKRO).
struct Interface {
  Gui::Interface card; // what the device card lists
  std::variant<std::monostate, espp::hid_rp::KeyboardDecoder, espp::hid_rp::MouseDecoder,
               espp::hid_rp::GamepadDecoder>
      decoder;
  espp::hid_rp::KeyboardReport keys{};
};
} // namespace

//! [usb_host_tab5_example]
extern "C" void app_main(void) {
  espp::Logger logger({.tag = "USB Host Tab5", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting USB HID host example");

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
  if (!tab5.initialize_touch()) {
    logger.warn("Failed to initialize touch; the panel column will not scroll");
  }
  tab5.brightness(75.0f);
#if CONFIG_USB_HOST_TAB5_CONTROL_USB_A_POWER
  // the jack comes up powered with the IO expanders; keep it off until the host
  // is listening so a device attached at boot sees VBUS + host together (see
  // UsbHost::Config::vbus_control)
  tab5.set_usb_a_power(false);
#endif

  static Gui gui({.log_level = espp::Logger::Verbosity::INFO});
  gui.set_status_text("USB host starting...");

  // --- state shared between the USB dispatch task and the main loop ------------
  // Decoding happens in the input callbacks (USB dispatch task, at the device's
  // report rate) into this state; the screen is refreshed from the main loop at
  // a fixed rate, so a fast device cannot flood LVGL.
  std::mutex state_mutex;
  std::map<const espp::UsbHost::HidDevice *, Interface> interfaces; // every opened interface
  Gui::DeviceInfo device_card; // product / VID:PID of the attached device
  uint8_t device_address = 0;  // its USB address: the example shows one device at a time
  SpaceMouseDecoder spacemouse;
  Gui::MouseState mouse; // motion accumulated from the relative reports
  espp::hid_rp::GamepadReport gamepad;
  std::vector<uint8_t> last_report; // the most recent raw Input report, any interface
  std::atomic<uint32_t> report_count{0};

  // Rebuild the device card from the interfaces currently open (they connect
  // one at a time) and say what to do with what is attached. Call with the
  // state mutex held.
  const auto refresh_card = [&]() {
    device_card.interfaces.clear();
    for (const auto &[dev, iface] : interfaces) {
      (void)dev;
      device_card.interfaces.push_back(iface.card);
    }
    std::sort(device_card.interfaces.begin(), device_card.interfaces.end(),
              [](const auto &a, const auto &b) { return a.interface_number < b.interface_number; });
    device_card.connected = !interfaces.empty();
    gui.set_device(device_card);
    std::string hint;
    if (device_card.has(Gui::Kind::SpaceMouse))
      hint += "Move the cap and press its buttons. ";
    if (device_card.has(Gui::Kind::Keyboard))
      hint += "Type: pressed keys light up. ";
    if (device_card.has(Gui::Kind::Mouse))
      hint += "Move the mouse, scroll, click. ";
    if (device_card.has(Gui::Kind::Gamepad))
      hint += "Move the sticks and press the buttons. ";
    if (hint.empty())
      hint = "Not a kind this example decodes: raw Input reports are shown below.";
    gui.set_status_text(hint);
  };

  // --- USB host --------------------------------------------------------------
  // Two of the settings need a word:
  //  - dispatch_task_stack_size: the connect callback parses the report
  //    descriptor, builds the decoder and fills in the device card (LVGL +
  //    fmt) on the dispatch task, so that task gets a larger stack;
  //  - full_speed_only: with it the root port's link to a hub runs at full
  //    speed, so the full-speed HID devices behind the hub never need its
  //    transaction translator (which ESP-IDF's hub driver does not implement);
  //    without it those devices fail their first descriptor read and the hub
  //    port is disabled. A device plugged in directly is unaffected in
  //    practice, since HID needs nothing beyond full speed.
  espp::UsbHost host({
    .on_device_connected =
        [&](const std::shared_ptr<espp::UsbHost::HidDevice> &device) {
          const auto info = device->info();
          const auto params = device->params();
          Interface iface{};
          iface.card = {.interface_number = params.interface_number,
                        .protocol = params.protocol,
                        .report_descriptor_bytes = device->report_descriptor().size(),
                        .kind = Gui::Kind::Unknown};
          // A SpaceMouse first, by vendor id: its descriptor (six axes and
          // buttons) would otherwise pass for a gamepad. Everything else is
          // classified from its report descriptor; UsbHost has already asked a
          // boot-subclass interface for the report protocol, so the descriptor
          // describes what the device actually sends.
          if (SpaceMouseDecoder::is_spacemouse(info.vid, info.pid)) {
            iface.card.kind = Gui::Kind::SpaceMouse;
          } else if (auto map = espp::hid_rp::ReportMap::parse(device->report_descriptor())) {
            using namespace espp::hid_rp;
            if (KeyboardDecoder::looks_like_keyboard(*map)) {
              iface.decoder = KeyboardDecoder(std::move(*map));
              iface.card.kind = Gui::Kind::Keyboard;
            } else if (MouseDecoder::looks_like_mouse(*map)) {
              iface.decoder = MouseDecoder(std::move(*map));
              iface.card.kind = Gui::Kind::Mouse;
            } else if (GamepadDecoder::looks_like_gamepad(*map)) {
              GamepadDecoder pad(std::move(*map));
              pad.apply_quirks(info.vid, info.pid); // per-device layout fixes, by VID:PID
              logger.info("gamepad layout: {}", pad.quirks().layout == GamepadDecoder::Layout::Xbox
                                                    ? "Xbox-style"
                                                    : "DirectInput");
              iface.decoder = std::move(pad);
              iface.card.kind = Gui::Kind::Gamepad;
            }
          } else {
            logger.warn("interface {}: report descriptor could not be parsed",
                        params.interface_number);
          }
          logger.info("connected: '{}' '{}' VID={:#06x} PID={:#06x} iface={} proto={} -> {}",
                      info.manufacturer, info.product, info.vid, info.pid, params.interface_number,
                      params.protocol, Gui::kind_name(iface.card.kind));

          const auto kind = iface.card.kind;
          const auto *key = device.get();
          {
            std::lock_guard<std::mutex> lock(state_mutex);
            if (interfaces.empty()) {
              // the first interface names the device on the card
              device_address = params.address;
              device_card = Gui::DeviceInfo{};
              device_card.product = info.product;
              device_card.manufacturer = info.manufacturer;
              device_card.vid = info.vid;
              device_card.pid = info.pid;
              spacemouse.reset();
              mouse = {};
              gamepad = {};
            }
            interfaces[key] = std::move(iface);
            refresh_card();
          }

          // Every Input report (device -> host): decode by this interface's kind.
          device->set_input_callback([&, key, kind](std::span<const uint8_t> data) {
            report_count.fetch_add(1);
            std::lock_guard<std::mutex> lock(state_mutex);
            last_report.assign(data.begin(), data.end());
            auto it = interfaces.find(key);
            if (it == interfaces.end())
              return;
            switch (kind) {
            case Gui::Kind::SpaceMouse:
              spacemouse.decode(data);
              break;
            case Gui::Kind::Keyboard:
              std::get<espp::hid_rp::KeyboardDecoder>(it->second.decoder)
                  .decode(data, it->second.keys);
              break;
            case Gui::Kind::Mouse: {
              espp::hid_rp::MouseReport m;
              if (std::get<espp::hid_rp::MouseDecoder>(it->second.decoder).decode(data, m)) {
                mouse.x = std::clamp<int32_t>(mouse.x + m.dx, -Gui::kMouseRange, Gui::kMouseRange);
                mouse.y = std::clamp<int32_t>(mouse.y + m.dy, -Gui::kMouseRange, Gui::kMouseRange);
                mouse.wheel += m.wheel;
                mouse.buttons = m.buttons;
              }
              break;
            }
            case Gui::Kind::Gamepad:
              std::get<espp::hid_rp::GamepadDecoder>(it->second.decoder).decode(data, gamepad);
              break;
            default:
              break; // raw only
            }
          });
        },
    .on_device_disconnected =
        [&](const std::shared_ptr<espp::UsbHost::HidDevice> &device) {
          logger.info("disconnected: PID={:#06x} iface={}", device->info().pid,
                      device->params().interface_number);
          std::lock_guard<std::mutex> lock(state_mutex);
          interfaces.erase(device.get());
          if (!interfaces.empty()) {
            refresh_card(); // the device is still there with fewer interfaces
            return;
          }
          gui.set_device(Gui::DeviceInfo{});
          gui.set_status_text("Device removed. Plug a device into the USB-A port.");
        },
    // One physical device at a time (a hub can carry several): while one is
    // shown, the interfaces of any other USB address are not opened, so its
    // reports cannot mix into the card and panels. It is picked up when it
    // is re-plugged after the shown device goes away.
        .should_open =
            [&](const espp::UsbHost::HidDevice::Info &info,
                const espp::UsbHost::HidDevice::Params &params) {
              std::lock_guard<std::mutex> lock(state_mutex);
              if (interfaces.empty() || params.address == device_address)
                return true;
              logger.info("not opening '{}' VID={:#06x} PID={:#06x} (address {}): another device "
                          "is being shown",
                          info.product, info.vid, info.pid, params.address);
              return false;
            },
#if ESPP_USB_HOST_HAS_PORT_SELECT
    // The Tab5's USB-A jack is on the P4's high-speed OTG controller, which is
    // peripheral 0 (the library default); say so explicitly so a board wired
    // the other way only has to change this number. (Without port selection,
    // ESP-IDF 5.x, only the default -1 is accepted; it is the same controller.)
        .port = 0,
#endif
    .root_port_power_on_delay =
        std::chrono::milliseconds(CONFIG_USB_HOST_TAB5_ROOT_PORT_POWER_ON_DELAY_MS),
#if CONFIG_USB_HOST_TAB5_CONTROL_USB_A_POWER
    .vbus_control = [&](bool on) { tab5.set_usb_a_power(on); },
#endif
    .dispatch_task_stack_size = 16 * 1024, .full_speed_only = true,
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
  // The panels at ~30 Hz (plenty for the eye), the raw-report line, rate and
  // status a few times a second, whatever the device's report rate.
  constexpr auto kPanelRefresh = 33ms;
  constexpr int kSlowEvery = 8; // ~4 Hz
  uint32_t last_count = 0;
  size_t last_enumerated = 0;
  int slow_tick = 0;
  auto last_time = std::chrono::steady_clock::now();
  while (true) {
    std::this_thread::sleep_for(kPanelRefresh);
    bool has_spacemouse = false, has_keyboard = false, has_mouse = false, has_gamepad = false;
    SpaceMouseDecoder::State spacemouse_state{};
    espp::hid_rp::KeyboardReport keys{}; // the keyboard interfaces' reports, merged
    Gui::MouseState mouse_state{};
    espp::hid_rp::GamepadReport gamepad_state{};
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      for (const auto &[dev, iface] : interfaces) {
        (void)dev;
        switch (iface.card.kind) {
        case Gui::Kind::SpaceMouse:
          has_spacemouse = true;
          break;
        case Gui::Kind::Keyboard:
          has_keyboard = true;
          for (size_t i = 0; i < std::size(keys.keys); ++i)
            keys.keys[i] |= iface.keys.keys[i];
          break;
        case Gui::Kind::Mouse:
          has_mouse = true;
          break;
        case Gui::Kind::Gamepad:
          has_gamepad = true;
          break;
        default:
          break;
        }
      }
      if (has_spacemouse)
        spacemouse_state = spacemouse.state();
      mouse_state = mouse;
      gamepad_state = gamepad;
    }
    if (has_spacemouse)
      gui.set_spacemouse_state(spacemouse_state);
    if (has_keyboard)
      gui.set_keyboard_state(keys);
    if (has_mouse)
      gui.set_mouse_state(mouse_state);
    if (has_gamepad)
      gui.set_gamepad_state(gamepad_state);
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
    }
    if (devices.empty()) {
      gui.set_status_text(
          enumerated == 0
              ? "Plug a device into the USB-A port."
              : fmt::format("{} USB device(s) enumerated, none opened as HID", enumerated));
      continue;
    }
    std::vector<uint8_t> report;
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      report = last_report;
    }
    gui.set_last_report(report, rate);
  }
}

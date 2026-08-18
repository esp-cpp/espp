#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "logger.hpp"
#include "odrive_ascii.hpp"
#include "odrive_native.hpp"
#include "usb_device.hpp"

// hid-rp: build the HID gamepad report descriptor + serialize input reports.
#include "hid-rp-gamepad.hpp"
#include "hid-rp.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  using namespace espp;

  // The log console stays on the built-in USB-Serial-JTAG / UART (configured via
  // sdkconfig). The native USB device created below is a *separate* USB
  // peripheral that presents an ODrive-compatible device with TWO protocols on
  // TWO interfaces (matching how a real ODrive splits them), plus a HID gamepad:
  //   - CDC serial interface  -> ODrive ASCII protocol (text; terminal / WebSerial)
  //   - vendor interface (0xFF, WebUSB) -> ODrive native (Fibre) binary protocol,
  //     which is what odrivetool / the fibre library auto-discover over USB.
  //   - HID interface -> an animated gamepad (visualize with hid_visualizer.html)
  Logger logger({.tag = "OdriveUsbExample", .level = Logger::Verbosity::INFO});

  //! [usb_cdc_example]

  // One simulated motor state, shared by both protocol servers.
  struct {
    std::atomic<float> vbus{24.0f};
    std::atomic<float> position{0.0f};
    std::atomic<float> velocity{0.0f};
    std::atomic<float> torque{0.0f};
    std::atomic<float> vel_limit{20.0f};
    std::atomic<uint32_t> error{0};
    std::atomic<uint64_t> serial{0xA1B2C3D4E5ULL};
  } state;

  // --- ASCII protocol server (CDC / terminal / WebSerial) -------------------
  OdriveAscii::Config ascii_cfg;
  ascii_cfg.log_level = Logger::Verbosity::WARN;
  OdriveAscii ascii(ascii_cfg);
  ascii.register_float_property(
      "axis0.encoder.pos_estimate", [&]() { return state.position.load(); },
      [&](float v, std::error_code &ec) {
        ec.clear();
        state.position = v;
        return true;
      });
  ascii.register_float_property("axis0.encoder.vel_estimate",
                                [&]() { return state.velocity.load(); });
  ascii.register_float_property(
      "axis0.controller.input_pos", [&]() { return state.position.load(); },
      [&](float v, std::error_code &ec) {
        ec.clear();
        state.position = v;
        return true;
      });
  ascii.on_position_command([&](int axis, float pos, std::optional<float> vel_ff,
                                std::optional<float> torque_ff, std::error_code &ec) {
    (void)axis;
    ec.clear();
    state.position = pos;
    if (vel_ff.has_value())
      state.velocity = *vel_ff;
    if (torque_ff.has_value())
      state.torque = *torque_ff;
    return true;
  });
  ascii.on_feedback_request([&](int axis, float &pos_out, float &vel_out, std::error_code &ec) {
    (void)axis;
    ec.clear();
    pos_out = state.position.load();
    vel_out = state.velocity.load();
    return true;
  });

  // --- Native (Fibre) protocol server (vendor interface / odrivetool) -------
  // Register an ODrive-style endpoint tree; odrivetool / the fibre library
  // download this tree from endpoint 0 and read/write it by numeric id.
  OdriveNative::Config native_cfg;
  native_cfg.log_level = Logger::Verbosity::WARN;
  OdriveNative native(native_cfg);
  native.register_float_property("vbus_voltage", [&]() { return state.vbus.load(); });
  native.register_uint32_property("axis0.error", [&]() { return state.error.load(); });
  native.register_float_property("axis0.encoder.pos_estimate",
                                 [&]() { return state.position.load(); });
  native.register_float_property("axis0.encoder.vel_estimate",
                                 [&]() { return state.velocity.load(); });
  native.register_float_property(
      "axis0.controller.input_pos", [&]() { return state.position.load(); },
      [&](float v, std::error_code &ec) {
        ec.clear();
        state.position = v;
        return true;
      });
  native.register_float_property(
      "axis0.controller.config.vel_limit", [&]() { return state.vel_limit.load(); },
      [&](float v, std::error_code &ec) {
        ec.clear();
        state.vel_limit = v;
        return true;
      });
  native.register_uint64_property("serial_number", [&]() { return state.serial.load(); });

  // --- Composite native USB device ------------------------------------------
  UsbDevice::Config usb_cfg;
  usb_cfg.vid = 0x1209; // pid.codes VID used by ODrive
  usb_cfg.pid = 0x0d32; // ODrive v3-like PID
  usb_cfg.manufacturer = "espp";
  usb_cfg.product = "espp ODrive";
  usb_cfg.serial_number = "0001";
  usb_cfg.log_level = Logger::Verbosity::INFO;

  UsbDevice::CdcFunction cdc;
  cdc.interface_name = "espp ODrive ASCII (CDC)";
  usb_cfg.cdc = cdc;

  UsbDevice::VendorFunction vendor;
  vendor.interface_name = "espp ODrive native (Fibre)";
  vendor.webusb = true; // advertise BOS / WebUSB / MS OS 2.0 descriptors
  usb_cfg.vendor = vendor;

  // HID gamepad function. The report descriptor is built with the espp hid-rp
  // component: wrap espp::GamepadInputReport's descriptor fragment in a
  // generic-desktop GAMEPAD application collection and serialize it to bytes.
  static constexpr uint8_t kHidReportId = 1;
  static constexpr size_t kNumButtons = 15;
  using Gamepad = espp::GamepadInputReport<kNumButtons, std::uint16_t, std::uint16_t, 0, 65535, 0,
                                           1023, kHidReportId>;
  Gamepad gamepad;
  gamepad.reset();

  std::vector<uint8_t> hid_report_descriptor;
  {
    using namespace hid::page;
    using namespace hid::rdf;
    auto raw_descriptor = descriptor(usage_page<generic_desktop>(), usage(generic_desktop::GAMEPAD),
                                     collection::application(gamepad.get_descriptor()));
    hid_report_descriptor.assign(raw_descriptor.begin(), raw_descriptor.end());
  }
  logger.info("HID gamepad report descriptor: {} bytes", hid_report_descriptor.size());

  UsbDevice::HidFunction hid;
  hid.interface_name = "espp Gamepad HID";
  hid.report_descriptor = hid_report_descriptor;
  usb_cfg.hid = hid;

  UsbDevice usb(usb_cfg);

  // CDC RX -> ASCII protocol -> CDC write.
  usb.set_cdc_receive_callback([&](std::span<const uint8_t> data) {
    auto response = ascii.process_bytes(data);
    if (!response.empty())
      usb.write_cdc(response);
  });

  // Vendor RX -> native (Fibre) protocol -> vendor write.
  //
  // The Fibre packet protocol over USB relies on USB transfer boundaries: each
  // host bulk-OUT transfer is exactly one packet. odrivetool's requests are
  // small (< 64 B), so each vendor RX callback delivers one whole packet, which
  // is what process_bytes() expects. (If a future client sent packets larger
  // than a single bulk transfer, this callback would need a length-based
  // reassembly step.)
  usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) {
    auto response = native.process_bytes(data);
    if (!response.empty())
      usb.write_vendor(response);
  });

  std::error_code ec;
  if (!usb.initialize(ec)) {
    logger.error("Failed to initialize USB device: {}", ec.message());
    return;
  }
  logger.info("ODrive-compatible native USB device ready:");
  logger.info("  CDC serial interface  -> ODrive ASCII  (terminal / WebSerial)");
  logger.info("  vendor interface (WebUSB) -> ODrive native/Fibre (odrivetool over USB)");
  logger.info("  HID interface -> animated gamepad (visualize with hid_visualizer.html)");
  logger.info("Native endpoint tree ({} bytes, json_crc=0x{:04x})", native.json().size(),
              native.json_crc());

  //! [usb_cdc_example]

  // The CDC / vendor transports run off the TinyUSB task + their RX callbacks.
  // Here on the main task we animate the shared motor state (seen by the ASCII /
  // native clients) and push a HID gamepad input report a few times a second.
  float phase = 0.0f;
  size_t tick = 0;
  while (true) {
    // Animate the motor state so native / ASCII clients see live values.
    state.velocity = 0.5f * std::sin(phase);

    // Animate the two joysticks (values in [-1, 1]) and toggle a couple of buttons.
    const float lx = std::sin(phase);
    const float ly = std::cos(phase);
    const float rx = std::cos(phase);
    const float ry = std::sin(phase);
    const bool button_a = ((tick / 5) % 2) == 0;  // ~1 Hz at the 10 Hz loop
    const bool button_b = ((tick / 10) % 2) == 0; // ~0.5 Hz

    gamepad.reset();
    gamepad.set_left_joystick(lx, ly);
    gamepad.set_right_joystick(rx, ry);
    gamepad.set_button(1, button_a);
    gamepad.set_button(2, button_b);
    if (usb.is_hid_ready()) {
      auto report = gamepad.get_report();
      std::error_code hid_ec;
      usb.write_hid_report(kHidReportId, report, hid_ec);
      if (hid_ec)
        logger.warn_rate_limited("HID report send failed: {}", hid_ec.message());
    }

    // Log once per second (the loop runs at ~10 Hz).
    if ((tick % 10) == 0)
      logger.info("HID lx={:+.2f} ly={:+.2f} A={} B={} | motor pos={} vel={}", lx, ly, button_a,
                  button_b, state.position.load(), state.velocity.load());

    phase += 0.1f;
    ++tick;
    std::this_thread::sleep_for(100ms);
  }
}

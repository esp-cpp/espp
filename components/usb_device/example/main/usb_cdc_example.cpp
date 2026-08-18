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
#include "usb_device.hpp"

// hid-rp: build the HID gamepad report descriptor + serialize input reports.
#include "hid-rp-gamepad.hpp"
#include "hid-rp.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  using namespace espp;

  // The log console stays on the built-in USB-Serial-JTAG / UART (configured via
  // sdkconfig). The native USB device created below is a *separate* USB
  // peripheral that exposes a composite CDC serial + vendor/WebUSB + HID gamepad
  // device. CDC and vendor both feed the ODrive ASCII protocol; the HID function
  // presents an animated gamepad.
  Logger logger({.tag = "UsbDeviceExample", .level = Logger::Verbosity::INFO});

  //! [usb_cdc_example]

  // Simulated motor state driven by the ODrive ASCII commands.
  struct {
    float position = 0.0f;
    float velocity = 0.0f;
    float torque = 0.0f;
  } state;

  // Transport-agnostic ODrive ASCII protocol server. Both USB interfaces feed
  // the same server.
  OdriveAscii::Config proto_cfg;
  proto_cfg.log_level = Logger::Verbosity::WARN;
  OdriveAscii proto(proto_cfg);

  // Register a couple of demo properties and command callbacks (mirrors the
  // odrive_ascii example).
  proto.register_float_property(
      "axis0.encoder.pos_estimate", [&]() { return state.position; },
      [&](float v, std::error_code &ec) {
        ec.clear();
        state.position = v;
        return true;
      });
  proto.register_float_property("axis0.encoder.vel_estimate", [&]() { return state.velocity; });
  proto.register_float_property(
      "axis0.controller.input_pos", [&]() { return state.position; },
      [&](float v, std::error_code &ec) {
        ec.clear();
        state.position = v;
        return true;
      });
  proto.on_position_command([&](int axis, float pos, std::optional<float> vel_ff,
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
  proto.on_feedback_request([&](int axis, float &pos_out, float &vel_out, std::error_code &ec) {
    (void)axis;
    ec.clear();
    pos_out = state.position;
    vel_out = state.velocity;
    return true;
  });

  // Composite native USB device: CDC serial + vendor-specific (WebUSB) function.
  // We create it *before* wiring the RX callbacks so the callbacks can capture
  // the instance and write the response back out the *same* interface.
  UsbDevice::Config usb_cfg;
  usb_cfg.vid = 0x1209; // pid.codes VID used by ODrive
  usb_cfg.pid = 0x0d32; // ODrive-like PID
  usb_cfg.manufacturer = "espp";
  usb_cfg.product = "espp ODrive ASCII";
  usb_cfg.serial_number = "0001";
  usb_cfg.log_level = Logger::Verbosity::INFO;

  // CDC serial function.
  UsbDevice::CdcFunction cdc;
  cdc.interface_name = "espp ODrive CDC";
  usb_cfg.cdc = cdc;

  // Vendor-specific function with WebUSB so a browser can talk to it driverlessly.
  // The landing page defaults to the espp docs-hosted ODrive WebUSB console.
  UsbDevice::VendorFunction vendor;
  vendor.interface_name = "espp ODrive WebUSB";
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

  // Wire: CDC RX -> proto.process_bytes -> CDC write.
  usb.set_cdc_receive_callback([&](std::span<const uint8_t> data) {
    auto response = proto.process_bytes(data);
    if (!response.empty())
      usb.write_cdc(response);
  });

  // Wire: Vendor RX -> proto.process_bytes -> Vendor write (identical payload).
  usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) {
    auto response = proto.process_bytes(data);
    if (!response.empty())
      usb.write_vendor(response);
  });

  std::error_code ec;
  if (!usb.initialize(ec)) {
    logger.error("Failed to initialize USB device: {}", ec.message());
    return;
  }
  logger.info("Native USB device ready (CDC serial + vendor/WebUSB + HID gamepad).");
  logger.info("Serial: connect to the ODrive-like port and send commands, e.g.");
  logger.info("  'r axis0.encoder.pos_estimate' or 'p 0 1.0 0.5 0.1'");
  logger.info("WebUSB: open the browser console and connect to the vendor interface.");
  logger.info("HID: the host sees a live gamepad with animated sticks + toggling buttons.");

  //! [usb_cdc_example]

  // The CDC / vendor transports run off the TinyUSB task and their RX callbacks.
  // Here on the main task we animate the HID gamepad and push an input report a
  // few times a second so the host sees a live device.
  float phase = 0.0f;
  size_t tick = 0;
  while (true) {
    // Animate the two joysticks (values in [-1, 1]) and toggle a couple of
    // buttons at different rates.
    const float lx = std::sin(phase);
    const float ly = std::cos(phase);
    const float rx = std::cos(phase);
    const float ry = std::sin(phase);
    const bool button_a = ((tick / 5) % 2) == 0;  // toggles ~1 Hz at 10 Hz loop
    const bool button_b = ((tick / 10) % 2) == 0; // toggles ~0.5 Hz

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

    // Log the values we send once per second (the loop runs at ~10 Hz).
    if ((tick % 10) == 0) {
      logger.info("HID gamepad: lx={:+.2f} ly={:+.2f} rx={:+.2f} ry={:+.2f} A={} B={} ready={}", lx,
                  ly, rx, ry, button_a, button_b, usb.is_hid_ready());
    }

    phase += 0.1f;
    ++tick;
    std::this_thread::sleep_for(100ms);
  }
}

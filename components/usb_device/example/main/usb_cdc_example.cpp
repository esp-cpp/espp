#include <atomic>
#include <chrono>
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

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  using namespace espp;

  // The log console stays on the built-in USB-Serial-JTAG / UART (configured via
  // sdkconfig). The native USB device created below is a *separate* USB
  // peripheral that exposes two interfaces (CDC serial + vendor/WebUSB), both
  // dedicated to the ODrive ASCII protocol.
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
  logger.info("Native USB device ready (CDC serial + vendor/WebUSB).");
  logger.info("Serial: connect to the ODrive-like port and send commands, e.g.");
  logger.info("  'r axis0.encoder.pos_estimate' or 'p 0 1.0 0.5 0.1'");
  logger.info("WebUSB: open the browser console and connect to the vendor interface.");

  //! [usb_cdc_example]

  // Nothing else to do on the main task; the transport runs off the TinyUSB
  // task and its RX callbacks.
  while (true) {
    std::this_thread::sleep_for(1s);
    if (usb.is_cdc_connected() || usb.is_vendor_connected()) {
      logger.debug_rate_limited("USB host connected; pos={} vel={}", state.position,
                                state.velocity);
    }
  }
}

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

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  using namespace espp;

  // The log console stays on the built-in USB-Serial-JTAG / UART (configured via
  // sdkconfig). The native USB device created below is a *separate* USB
  // peripheral that presents an ODrive-compatible device with TWO protocols on
  // TWO interfaces, matching how a real ODrive splits them:
  //   - CDC serial interface  -> ODrive ASCII protocol (text; terminal / WebSerial)
  //   - vendor interface (0xFF, WebUSB) -> ODrive native (Fibre) binary protocol,
  //     which is what odrivetool / the fibre library auto-discover over USB.
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
  logger.info("Native endpoint tree ({} bytes, json_crc=0x{:04x})", native.json().size(),
              native.json_crc());

  //! [usb_cdc_example]

  // The transport runs off the TinyUSB task + its RX callbacks. Animate a little
  // state so a connected client sees live values.
  float t = 0.0f;
  while (true) {
    std::this_thread::sleep_for(100ms);
    t += 0.1f;
    state.velocity = 0.5f * std::sin(t);
    if (usb.is_cdc_connected() || usb.is_vendor_connected())
      logger.debug_rate_limited("USB host connected; pos={} vel={}", state.position.load(),
                                state.velocity.load());
  }
}

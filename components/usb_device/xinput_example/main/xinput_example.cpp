// Xbox 360 (X-Input) controller emulation example.
//
// Presents the ESP32-S3 as a wired Xbox 360 controller over the native USB-OTG
// peripheral using espp::UsbDevice's XInput function. A PC's XUSB driver binds
// it (VID/PID default to Microsoft's 0x045E:0x028E), so it shows up as an Xbox
// 360 controller in the OS gamepad tester / games. This demo cycles the buttons
// and sweeps the sticks/triggers so you can see live input, and logs any rumble
// / LED reports the host sends back. The console/logs go to UART0; the native
// USB port is reserved for the emulated controller.

#include <chrono>
#include <cmath>
#include <thread>

#include "logger.hpp"
#include "usb_device.hpp"

using namespace std::chrono_literals;
using espp::xinput::Button;
using espp::xinput::GamepadState;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "XInput", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting Xbox 360 (X-Input) controller example");

  espp::UsbDevice::Config cfg;
  cfg.product = "espp XInput Controller";
  cfg.log_level = espp::Logger::Verbosity::INFO;

  espp::UsbDevice::XInputFunction xinput;
  // Rumble / LED reports (8-byte host->device on the interrupt OUT endpoint).
  xinput.on_rumble = [&](std::span<const uint8_t> data) {
    if (data.size() >= 5 && data[0] == 0x00) // 0x00 = rumble report
      logger.info("rumble: left={} right={}", data[3], data[4]);
    else if (!data.empty() && data[0] == 0x01) // 0x01 = LED report
      logger.info("led pattern: {}", data.size() >= 3 ? data[2] : 0);
  };
  cfg.xinput = xinput; // XInput is the ONLY function -> Xbox identity + XUSB bind

  espp::UsbDevice usb(cfg);
  std::error_code ec;
  if (!usb.initialize(ec)) {
    logger.error("Failed to initialize USB device: {}", ec.message());
    return;
  }

  logger.info("Ready. Connect to a PC; it should enumerate as an Xbox 360 controller.");

  // Demo input generator: sweep the sticks/triggers in a circle and step the
  // face buttons A/B/X/Y one at a time each second.
  GamepadState state;
  const Button face[] = {Button::A, Button::B, Button::X, Button::Y};
  int tick = 0;
  while (true) {
    const float t = tick * 0.02f; // 50 Hz
    const float two_pi = 6.2831853f;
    state.lx = static_cast<int16_t>(std::sin(two_pi * 0.25f * t) * 32000);
    state.ly = static_cast<int16_t>(std::cos(two_pi * 0.25f * t) * 32000);
    state.rx = static_cast<int16_t>(std::sin(two_pi * 0.5f * t) * 20000);
    state.ry = static_cast<int16_t>(std::cos(two_pi * 0.5f * t) * 20000);
    const uint8_t tri = static_cast<uint8_t>((std::sin(two_pi * 0.5f * t) * 0.5f + 0.5f) * 255);
    state.left_trigger = tri;
    state.right_trigger = static_cast<uint8_t>(255 - tri);

    // One face button on at a time, changing each second.
    for (auto b : face)
      state.set(b, false);
    state.set(face[(tick / 50) % 4], true);

    usb.update_xinput_state(state); // no-op / retry-later while not mounted or busy

    tick++;
    std::this_thread::sleep_for(20ms);
  }
}

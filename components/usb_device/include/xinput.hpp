#pragma once

// Xbox 360 wired controller (X-Input) protocol helpers.
//
// This header is dependency-free and host-testable (no ESP-IDF / TinyUSB
// headers): it defines the X-Input wire constants, a gamepad-state model that
// packs the 20-byte input report, and a builder for the USB interface + XID +
// interrupt-endpoint descriptor bytes. `espp::UsbDevice`'s XInput function
// (usb_device.hpp) consumes these on-device; a host test exercises `report()`.
//
// X-Input is Microsoft's proprietary protocol for the Xbox 360 controller. The
// device presents a vendor-specific interface (bInterfaceClass 0xFF /
// bInterfaceSubClass 0x5D / bInterfaceProtocol 0x01) with one interrupt IN
// endpoint (20-byte input reports) and one interrupt OUT endpoint (8-byte
// rumble / LED reports). A PC's XUSB driver only binds a device whose VID/PID is
// a recognized Xbox 360 controller, so the defaults below are Microsoft's
// (0x045E:0x028E) -- use them only for emulation / testing of your own device.

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace espp {
namespace xinput {

/// Default identity of a wired Xbox 360 controller. These are Microsoft's IDs;
/// a host's XUSB driver only recognizes an X-Input controller by a known VID/PID,
/// so emulation requires advertising them (overridable via XInputFunction).
inline constexpr uint16_t kDefaultVid = 0x045E;       ///< Microsoft
inline constexpr uint16_t kDefaultPid = 0x028E;       ///< Xbox 360 Controller (wired)
inline constexpr uint16_t kDefaultBcdDevice = 0x0114; ///< reported device release

/// Vendor-specific interface class triple that identifies X-Input.
inline constexpr uint8_t kInterfaceClass = 0xFF;
inline constexpr uint8_t kInterfaceSubClass = 0x5D;
inline constexpr uint8_t kInterfaceProtocol = 0x01;

inline constexpr std::size_t kReportInSize = 20; ///< input report (device -> host)
inline constexpr std::size_t kReportOutSize = 8; ///< rumble / LED report (host -> device)
inline constexpr uint8_t kEpSize = 32;           ///< interrupt endpoint wMaxPacketSize
inline constexpr uint8_t kInInterval = 4;        ///< IN endpoint bInterval (ms, full speed)
inline constexpr uint8_t kOutInterval = 8;       ///< OUT endpoint bInterval (ms, full speed)

/// Full byte length of the interface + XID + two endpoint descriptors emitted by
/// interface_descriptor(): 9 (interface) + 17 (XID) + 7 (IN ep) + 7 (OUT ep).
inline constexpr std::size_t kInterfaceDescriptorLen = 9 + 17 + 7 + 7; // 40

/// Button bit positions within GamepadState::buttons (little-endian on the wire:
/// the low byte is report byte 2, the high byte is report byte 3).
///  byte 2 (low):  bit0 dpad-up, 1 down, 2 left, 3 right, 4 start, 5 back,
///                 6 left-stick (L3), 7 right-stick (R3)
///  byte 3 (high): bit0 LB, 1 RB, 2 Guide, (3 unused), 4 A, 5 B, 6 X, 7 Y
enum class Button : uint16_t {
  DpadUp = 1u << 0,
  DpadDown = 1u << 1,
  DpadLeft = 1u << 2,
  DpadRight = 1u << 3,
  Start = 1u << 4,
  Back = 1u << 5,
  LeftStick = 1u << 6,  ///< L3 (left stick click)
  RightStick = 1u << 7, ///< R3 (right stick click)
  LeftBumper = 1u << 8,
  RightBumper = 1u << 9,
  Guide = 1u << 10, ///< the center "Xbox" button
  // bit 11 is unused / reserved
  A = 1u << 12,
  B = 1u << 13,
  X = 1u << 14,
  Y = 1u << 15,
};

/// The full gamepad state, packed into the 20-byte X-Input input report.
struct GamepadState {
  uint16_t buttons{0};      ///< OR of Button values
  uint8_t left_trigger{0};  ///< LT analog, 0..255
  uint8_t right_trigger{0}; ///< RT analog, 0..255
  int16_t lx{0};            ///< left stick X, -32768..32767 (right positive)
  int16_t ly{0};            ///< left stick Y, -32768..32767 (up positive)
  int16_t rx{0};            ///< right stick X
  int16_t ry{0};            ///< right stick Y

  /// Set or clear a button.
  void set(Button b, bool on) {
    if (on)
      buttons |= static_cast<uint16_t>(b);
    else
      buttons &= static_cast<uint16_t>(~static_cast<uint16_t>(b));
  }
  bool get(Button b) const { return (buttons & static_cast<uint16_t>(b)) != 0; }

  /// Serialize the 20-byte X-Input input report (little-endian axes).
  std::array<uint8_t, kReportInSize> report() const {
    std::array<uint8_t, kReportInSize> r{};
    r[0] = 0x00; // message type (input report)
    r[1] = 0x14; // message length (20)
    r[2] = static_cast<uint8_t>(buttons & 0xFF);
    r[3] = static_cast<uint8_t>((buttons >> 8) & 0xFF);
    r[4] = left_trigger;
    r[5] = right_trigger;
    auto put16 = [&](std::size_t i, int16_t v) {
      const uint16_t u = static_cast<uint16_t>(v);
      r[i] = static_cast<uint8_t>(u & 0xFF);
      r[i + 1] = static_cast<uint8_t>((u >> 8) & 0xFF);
    };
    put16(6, lx);
    put16(8, ly);
    put16(10, rx);
    put16(12, ry);
    // bytes 14..19 are reserved (already zero)
    return r;
  }
};

/// Build the interface + XID + two interrupt-endpoint descriptor bytes for an
/// X-Input interface. @p ep_num is the endpoint NUMBER n; the IN endpoint is
/// 0x80|n and the OUT endpoint is n (the XID blob embeds the IN endpoint address
/// and the report sizes, so it is patched to match @p ep_num).
inline std::vector<uint8_t> interface_descriptor(uint8_t itf_num, uint8_t str_idx, uint8_t ep_num,
                                                 uint8_t in_interval = kInInterval,
                                                 uint8_t out_interval = kOutInterval) {
  const uint8_t ep_in = static_cast<uint8_t>(0x80 | ep_num);
  const uint8_t ep_out = ep_num;
  return {
      // clang-format off
      // Interface descriptor (9 bytes): vendor-specific 0xFF/0x5D/0x01, 2 endpoints.
      0x09, 0x04 /* INTERFACE */, itf_num, 0x00 /* alt */, 0x02 /* num endpoints */,
      kInterfaceClass, kInterfaceSubClass, kInterfaceProtocol, str_idx,
      // XID "unknown" vendor descriptor (17 bytes), matched byte-for-byte to a
      // real wired Xbox 360 controller (bLength 0x11, bDescriptorType 0x21). [2]
      // is 0x10 on the retail controller; [6] = IN endpoint address, [7] = IN
      // report size (0x14 = 20), [14] = OUT report size (0x08 = 8).
      0x11, 0x21, 0x10, 0x01, 0x01, 0x25,
      ep_in, 0x14, 0x00, 0x00, 0x00, 0x00, 0x13, 0x01, 0x08, 0x00, 0x00,
      // Endpoint IN (7 bytes): interrupt, wMaxPacketSize 32, bInterval.
      0x07, 0x05 /* ENDPOINT */, ep_in, 0x03 /* interrupt */, kEpSize, 0x00, in_interval,
      // Endpoint OUT (7 bytes): interrupt, wMaxPacketSize 32, bInterval.
      0x07, 0x05 /* ENDPOINT */, ep_out, 0x03 /* interrupt */, kEpSize, 0x00, out_interval,
      // clang-format on
  };
}

} // namespace xinput
} // namespace espp

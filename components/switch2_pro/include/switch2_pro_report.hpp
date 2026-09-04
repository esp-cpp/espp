#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "switch2_pro_protocol.hpp"

/// @file switch2_pro_report.hpp
/// @brief Nintendo Switch 2 Pro Controller input report (report id 0x09).

namespace espp::switch2 {

/// The 63-byte Pro Controller 2 input report (BLE omits the leading report-id
/// byte). Buttons are a 3-byte bitfield; sticks are two 12-bit axes packed into
/// 3 bytes each; the tail carries motion/IMU when enabled via feature-select.
///
/// Button bits (matching the reverse-engineered layout):
///   byte0: 0x80 RStick 0x40 Plus 0x20 ZR 0x10 R 0x08 X 0x04 Y 0x02 A 0x01 B
///   byte1: 0x80 LStick 0x40 Minus 0x20 ZL 0x10 L 0x08 Up 0x04 Left 0x02 Right 0x01 Down
///   byte2: 0x10 C 0x08 GL 0x04 GR 0x02 Capture 0x01 Home
class Pro2InputReport {
public:
  static constexpr uint8_t REPORT_ID = 0x09;
  static constexpr size_t SIZE = 63;
  static constexpr uint16_t STICK_CENTER = 0x800; ///< 12-bit midpoint (2048)
  static constexpr uint16_t STICK_MAX = 0xfff;

  Pro2InputReport() { reset(); }

  void reset() {
    data_.fill(0);
    data_[0x0b] = 0x30; // "unknown" byte: 0x30 unless feature bit 5 is set (0x38)
    set_left_stick(0.f, 0.f);
    set_right_stick(0.f, 0.f);
  }

  void increment_counter() { data_[0]++; }

  /// battery_level: 0..9; charging/external-power flags in the same byte.
  void set_power(uint8_t battery_level, bool charging, bool external_power) {
    data_[1] = static_cast<uint8_t>(((battery_level & 0x0f) << 2) | (charging ? 0x02 : 0x00) |
                                    (external_power ? 0x01 : 0x00));
  }

  // Face / shoulder / system buttons.
  void set_a(bool v) { set_bit(2, 0x02, v); }
  void set_b(bool v) { set_bit(2, 0x01, v); }
  void set_x(bool v) { set_bit(2, 0x08, v); }
  void set_y(bool v) { set_bit(2, 0x04, v); }
  void set_r(bool v) { set_bit(2, 0x10, v); }
  void set_zr(bool v) { set_bit(2, 0x20, v); }
  void set_plus(bool v) { set_bit(2, 0x40, v); }
  void set_rstick(bool v) { set_bit(2, 0x80, v); }
  void set_down(bool v) { set_bit(3, 0x01, v); }
  void set_right(bool v) { set_bit(3, 0x02, v); }
  void set_left(bool v) { set_bit(3, 0x04, v); }
  void set_up(bool v) { set_bit(3, 0x08, v); }
  void set_l(bool v) { set_bit(3, 0x10, v); }
  void set_zl(bool v) { set_bit(3, 0x20, v); }
  void set_minus(bool v) { set_bit(3, 0x40, v); }
  void set_lstick(bool v) { set_bit(3, 0x80, v); }
  void set_home(bool v) { set_bit(4, 0x01, v); }
  void set_capture(bool v) { set_bit(4, 0x02, v); }
  void set_gr(bool v) { set_bit(4, 0x04, v); } ///< right grip button
  void set_gl(bool v) { set_bit(4, 0x08, v); } ///< left grip button
  void set_c(bool v) { set_bit(4, 0x10, v); }  ///< Switch 2 "C" (chat) button

  /// Left/right stick, each axis in [-1, 1].
  void set_left_stick(float x, float y) { pack_stick(5, x, y); }
  void set_right_stick(float x, float y) { pack_stick(8, x, y); }

  const std::array<uint8_t, SIZE> &data() const { return data_; }

private:
  static uint16_t axis_to_u12(float v) {
    if (v < -1.f)
      v = -1.f;
    if (v > 1.f)
      v = 1.f;
    // Round (not truncate) so a neutral axis (v==0) maps to STICK_CENTER (2048),
    // not 2047, while ±1 still land exactly on 0 / STICK_MAX (4095).
    return static_cast<uint16_t>((v * 0.5f + 0.5f) * STICK_MAX + 0.5f);
  }
  void pack_stick(size_t offset, float x, float y) {
    const uint16_t xv = axis_to_u12(x);
    const uint16_t yv = axis_to_u12(y);
    data_[offset] = static_cast<uint8_t>(xv & 0xff);
    data_[offset + 1] = static_cast<uint8_t>(((yv & 0x0f) << 4) | ((xv >> 8) & 0x0f));
    data_[offset + 2] = static_cast<uint8_t>((yv >> 4) & 0xff);
  }
  void set_bit(size_t byte, uint8_t mask, bool v) {
    if (v)
      data_[byte] |= mask;
    else
      data_[byte] &= ~mask;
  }

  std::array<uint8_t, SIZE> data_{};
};

} // namespace espp::switch2

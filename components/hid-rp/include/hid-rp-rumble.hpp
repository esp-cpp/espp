#pragma once

// Rumble, both directions of a controller bridge:
//
//  - RumbleEncoder builds a controller's rumble OUTPUT report from its report
//    descriptor at runtime (descriptor-driven, like GamepadDecoder for inputs):
//    a device whose report map declares a Physical Interface Device (PID)
//    "Set Effect" style output report - DC Enable Actuators, one Magnitude per
//    actuator, Duration, Start Delay, Loop Count, as the Xbox Wireless
//    controller does over Bluetooth LE - can be driven without knowing its
//    VID/PID.
//  - decode_switch_rumble() reads the two motor levels a Nintendo Switch host
//    sends a Pro Controller / Joy-Con (HD rumble, in output reports 0x01 and
//    0x10).
//
// Levels are 0 (off) .. 255 (full) throughout. Header-only, no ESP-IDF headers.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "hid-rp-report-map.hpp"

namespace espp {
namespace hid_rp {

/// How strongly to drive each of a controller's rumble motors, 0 (off) .. 255.
struct RumbleLevels {
  uint8_t strong{0};        ///< the large, low-frequency motor (left grip)
  uint8_t weak{0};          ///< the small, high-frequency motor (right grip)
  uint8_t left_trigger{0};  ///< impulse-trigger motor behind the left trigger
  uint8_t right_trigger{0}; ///< impulse-trigger motor behind the right trigger

  /// Whether any motor is on.
  constexpr bool any() const { return strong || weak || left_trigger || right_trigger; }
  constexpr bool operator==(const RumbleLevels &) const = default;
};

/// Builds a controller's rumble output report from its report descriptor.
///
/// from_report_map() looks for an output report whose fields use the Physical
/// Interface Device page (0x0F) and include at least one Magnitude (0x70): one
/// Magnitude field per actuator, plus, when present, DC Enable Actuators
/// (0x97), Duration (0x50), Start Delay (0xA7) and Loop Count (0x7C). The
/// descriptor gives each field's position, size and range; what it does not
/// give is which Magnitude drives which motor, so that follows a convention:
///  - 4 or more actuators: left trigger, right trigger, strong, weak (the
///    Xbox Wireless controller's order, as Linux's hid-microsoft driver uses
///    it); further actuators are left off;
///  - 2 or 3 actuators: strong, weak (a third is left off);
///  - 1 actuator: the stronger of strong / weak.
/// set_motor() overrides that per actuator for a device that differs.
///
/// The enable field, when it has a bit per actuator, gets a bitmask in which
/// bit i enables actuator (count - 1 - i): the Xbox controller's bit 0 is its
/// weak motor and bit 1 its strong motor. A narrower enable field is a single
/// on / off flag. Duration is scaled by the field's unit (seconds with an
/// exponent: the Xbox controller's is 10 ms) and taken as milliseconds when the
/// field declares no time unit. Start Delay and Loop Count are 0: an effect
/// plays once, for its duration, starting at once.
class RumbleEncoder {
public:
  /// What an actuator is driven from.
  enum class Motor : uint8_t { Strong, Weak, LeftTrigger, RightTrigger, None };
  static constexpr size_t MAX_ACTUATORS = 8; ///< Magnitude fields beyond this are left at 0

  /// The encoder for a device's rumble report, or nullopt when its report map
  /// declares no PID-style output report with a Magnitude field.
  static std::optional<RumbleEncoder> from_report_map(const ReportMap &map) {
    namespace u = usage;
    const auto &outs = map.output_fields();
    for (const auto &first : outs) {
      if (first.usage_page != u::PAGE_PHYSICAL_INPUT_DEVICE || first.usage != u::PID_MAGNITUDE) {
        continue;
      }
      // the first report that carries a Magnitude: gather its PID fields
      RumbleEncoder e;
      e.report_id_ = first.report_id;
      e.report_size_ = map.output_report_bytes(first.report_id);
      for (const auto &f : outs) {
        if (f.report_id != e.report_id_ || f.usage_page != u::PAGE_PHYSICAL_INPUT_DEVICE) {
          continue;
        }
        switch (f.usage) {
        case u::PID_MAGNITUDE:
          if (e.actuator_count_ < MAX_ACTUATORS) {
            e.magnitudes_[e.actuator_count_++] = f;
          }
          break;
        case u::PID_DC_ENABLE_ACTUATORS:
          if (!e.enable_)
            e.enable_ = f;
          break;
        case u::PID_DURATION:
          if (!e.duration_)
            e.duration_ = f;
          break;
        default: // Start Delay, Loop Count and anything else: left at 0
          break;
        }
      }
      e.set_default_motors();
      return e;
    }
    return std::nullopt;
  }

  /// The report ID to send the report with (0: the device uses no report IDs).
  uint8_t report_id() const { return report_id_; }
  /// The report's size in bytes, without the report ID byte.
  size_t report_size() const { return report_size_; }
  /// How many actuators (Magnitude fields) the report drives.
  size_t actuator_count() const { return actuator_count_; }
  /// What actuator @p slot (in descriptor order) is driven from.
  Motor motor(size_t slot) const { return slot < actuator_count_ ? motors_[slot] : Motor::None; }
  /// Drive actuator @p slot from @p m instead of the default (see the class notes).
  void set_motor(size_t slot, Motor m) {
    if (slot < actuator_count_)
      motors_[slot] = m;
  }
  /// The longest effect the Duration field can express, in milliseconds (0
  /// when the report has no Duration field).
  uint32_t max_duration_ms() const {
    if (!duration_)
      return 0;
    return static_cast<uint32_t>(std::lround(field_max(*duration_) * unit_ms(*duration_)));
  }

  /// The report payload (without the report ID byte) that drives the motors at
  /// @p levels for @p duration_ms (clamped to what the Duration field can say;
  /// ignored when the report has none). All-zero levels make the stop report.
  std::vector<uint8_t> encode(const RumbleLevels &levels, uint32_t duration_ms) const {
    std::vector<uint8_t> out(report_size_, 0);
    uint32_t enable_mask = 0;
    bool any = false;
    for (size_t i = 0; i < actuator_count_; i++) {
      const uint8_t level = level_for(motors_[i], levels);
      if (level) {
        any = true;
        // bit i enables actuator (count - 1 - i): see the class notes
        const size_t bit = actuator_count_ - 1 - i;
        if (bit < 32)
          enable_mask |= 1u << bit;
      }
      const auto &f = magnitudes_[i];
      const int64_t lo = f.logical_min, hi = field_max(f);
      put(out, f, static_cast<uint32_t>(lo + (level * (hi - lo) + 127) / 255));
    }
    if (enable_) {
      const bool bitmask = enable_->bit_size >= actuator_count_;
      put(out, *enable_, bitmask ? enable_mask : (any ? 1u : 0u));
    }
    if (duration_) {
      const double units = duration_ms / unit_ms(*duration_);
      const int64_t hi = field_max(*duration_);
      const int64_t v = std::clamp<int64_t>(std::llround(units), duration_->logical_min, hi);
      put(out, *duration_, any ? static_cast<uint32_t>(v) : 0u);
    }
    return out;
  }

  /// The report that stops every motor.
  std::vector<uint8_t> encode_stop() const { return encode(RumbleLevels{}, 0); }

private:
  RumbleEncoder() = default;

  void set_default_motors() {
    motors_.fill(Motor::None);
    if (actuator_count_ >= 4) {
      motors_[0] = Motor::LeftTrigger;
      motors_[1] = Motor::RightTrigger;
      motors_[2] = Motor::Strong;
      motors_[3] = Motor::Weak;
    } else if (actuator_count_ >= 2) {
      motors_[0] = Motor::Strong;
      motors_[1] = Motor::Weak;
    } else if (actuator_count_ == 1) {
      motors_[0] = Motor::Strong; // driven by max(strong, weak), see level_for()
    }
  }

  uint8_t level_for(Motor m, const RumbleLevels &l) const {
    switch (m) {
    case Motor::Strong:
      return actuator_count_ == 1 ? std::max(l.strong, l.weak) : l.strong;
    case Motor::Weak:
      return l.weak;
    case Motor::LeftTrigger:
      return l.left_trigger;
    case Motor::RightTrigger:
      return l.right_trigger;
    default:
      return 0;
    }
  }

  // The field's top value: its logical maximum, or, when the descriptor leaves
  // that at 0, the largest value its bits hold.
  static int64_t field_max(const ReportField &f) {
    if (f.logical_max > f.logical_min)
      return f.logical_max;
    return f.bit_size >= 32 ? 0xFFFFFFFFll : (1ll << f.bit_size) - 1;
  }

  // Milliseconds per unit of a Duration field: a time unit (seconds, SI linear
  // or rotation system) scaled by its exponent, else milliseconds.
  static double unit_ms(const ReportField &f) {
    const uint32_t system = f.unit & 0xF, time = (f.unit >> 12) & 0xF;
    const bool seconds =
        (system == 1 || system == 2) && time == 1 && (f.unit & 0x0FF0) == 0 && (f.unit >> 16) == 0;
    return seconds ? std::pow(10.0, f.unit_exponent + 3) : 1.0;
  }

  // Write @p value into the field's bits (little-endian, as HID reports are).
  static void put(std::vector<uint8_t> &out, const ReportField &f, uint32_t value) {
    for (unsigned i = 0; i < f.bit_size && i < 32; i++) {
      const uint32_t bit = f.bit_offset + i;
      if ((bit >> 3) >= out.size())
        return;
      const uint8_t mask = static_cast<uint8_t>(1u << (bit & 7));
      if ((value >> i) & 1u)
        out[bit >> 3] |= mask;
      else
        out[bit >> 3] &= static_cast<uint8_t>(~mask);
    }
  }

  uint8_t report_id_{0};
  size_t report_size_{0};
  size_t actuator_count_{0};
  std::array<ReportField, MAX_ACTUATORS> magnitudes_{};
  std::array<Motor, MAX_ACTUATORS> motors_{};
  std::optional<ReportField> enable_;
  std::optional<ReportField> duration_;
};

/// The two motor levels of a Nintendo Switch host's rumble packet.
struct SwitchRumble {
  uint8_t left{0};  ///< the left Joy-Con / grip motor, 0..255
  uint8_t right{0}; ///< the right Joy-Con / grip motor, 0..255
  constexpr bool operator==(const SwitchRumble &) const = default;
};

namespace detail {
// HD rumble amplitude code (0..100, the 7-bit step the host encodes) to an
// amplitude in 0..~1: the inverse of the encoding in dekuNukem's
// Nintendo_Switch_Reverse_Engineering rumble_data_table (log2 segments above
// codes 16 and 32; linear below, where the table's steps are tiny).
inline float switch_rumble_amplitude(int code) {
  if (code <= 0)
    return 0.0f;
  code = std::min(code, 100);
  if (code >= 32)
    return std::exp2(code / 32.0f) / 8.7f;
  if (code >= 16)
    return std::exp2(code / 16.0f) / 17.0f;
  return code * (2.0f / 17.0f) / 16.0f;
}
// One motor's 4 bytes: [0] HF frequency (low byte), [1] bit 0 HF frequency
// MSB, bits 1-7 HF amplitude code; [2] bits 0-6 LF frequency, bit 7 LF
// amplitude LSB; [3] LF amplitude code / 2 + 0x40. The motor level is the
// stronger of the two bands.
inline uint8_t switch_rumble_motor(std::span<const uint8_t, 4> m) {
  const int hf_code = m[1] >> 1;
  const int lf_code = m[3] >= 0x40 ? ((m[3] - 0x40) << 1) | (m[2] >> 7) : 0;
  const float amp = std::max(switch_rumble_amplitude(hf_code), switch_rumble_amplitude(lf_code));
  return static_cast<uint8_t>(std::lround(std::clamp(amp, 0.0f, 1.0f) * 255.0f));
}
} // namespace detail

/// The motor levels in an 8-byte Switch rumble packet (left motor's 4 bytes,
/// then the right's). The neutral packet (00 01 40 40 per motor) is 0 / 0.
inline SwitchRumble decode_switch_rumble(std::span<const uint8_t, 8> data) {
  return {detail::switch_rumble_motor(data.subspan<0, 4>()),
          detail::switch_rumble_motor(data.subspan<4, 4>())};
}

/// The motor levels a Switch host's OUTPUT report carries, or nullopt when it
/// carries none. @p report starts with the report ID: 0x01 (rumble +
/// subcommand) and 0x10 (rumble only) both lay out [id][packet number][8
/// rumble bytes]...
inline std::optional<SwitchRumble> decode_switch_rumble_report(std::span<const uint8_t> report) {
  if (report.size() < 10 || (report[0] != 0x01 && report[0] != 0x10))
    return std::nullopt;
  return decode_switch_rumble(report.subspan<2, 8>());
}

} // namespace hid_rp
} // namespace espp

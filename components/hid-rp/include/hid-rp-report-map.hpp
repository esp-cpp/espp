#pragma once

// Runtime HID report descriptor mapping and report decoding.
//
// hid-rp generates report descriptors at compile time; this header is the
// other direction: given the report descriptor of a device found at runtime
// (e.g. from espp::UsbHost::HidDevice::report_descriptor() or a BLE HID
// service), work out where every Input field sits in the device's reports,
// then decode reports into something usable (a generic gamepad, a keyboard
// usage bitmap, mouse motion). It is built on hid-rp's `hid::rdf::parser`, the
// same item walker the library uses at compile time, so descriptors are read
// exactly as the generators write them (push/pop, usage ranges, array items,
// report IDs, long items).

#include <algorithm>
#include <bit>
#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <vector>

#include "hid/rdf/descriptor_view.hpp"
#include "hid/rdf/exception.hpp"
#include "hid/rdf/parser.hpp"

namespace espp {
namespace hid_rp {

/// One Input data field of a device's report(s), as laid out in the report.
struct ReportField {
  uint8_t report_id{0};   ///< 0 when the device does not use report IDs
  uint32_t bit_offset{0}; ///< bit position within the report (after the ID byte, if any)
  uint8_t bit_size{0};    ///< report size in bits, always 1..32 (see ReportMap::parse())
  uint16_t usage_page{0}; ///< global usage page
  uint16_t usage{0};      ///< usage ID; for array fields the usage minimum
  uint16_t usage_max{0};  ///< array fields: usage maximum (variable: unused)
  int32_t logical_min{0};
  int32_t logical_max{0};
  bool array{false};    ///< array item: the value selects a usage (usage + value - logical_min)
  bool relative{false}; ///< relative (mouse motion) rather than absolute
};

/// The Input fields of a report descriptor, parsed at runtime.
class ReportMap {
public:
  /// Parse a report descriptor. Returns nullopt for a descriptor the parser
  /// rejects (malformed items, unmatched push/pop). Only Input items are
  /// mapped; Output/Feature items are skipped (they live in other reports).
  static std::optional<ReportMap> parse(std::span<const uint8_t> descriptor) {
    ReportMap map;
    // hid-rp reports malformed descriptors through HID_RDF_ASSERT, which
    // throws when exceptions are enabled and is a no-op otherwise.
// The throw is hid-rp's (HID_RDF_ASSERT, gated on __EXCEPTIONS), so this has to
// cover every macro a toolchain may use to say exceptions are on; a try block
// that nothing throws through costs nothing.
#if defined(__cpp_exceptions) || defined(__EXCEPTIONS) || defined(_CPPUNWIND)
    try {
      [[maybe_unused]] Parser parser(
          map, ::hid::rdf::descriptor_view(descriptor.data(), descriptor.size()));
    } catch (const ::hid::rdf::exception &) {
      // Only the descriptor being malformed turns into "no map"; anything else
      // (std::bad_alloc from the field vector, say) is a real failure and is
      // left to propagate rather than being reported as an unparsable device.
      return std::nullopt;
    }
#else
    [[maybe_unused]] Parser parser(
        map, ::hid::rdf::descriptor_view(descriptor.data(), descriptor.size()));
#endif
    if (map.fields_.empty()) {
      return std::nullopt;
    }
    return map;
  }

  /// Every Input field, in descriptor order.
  const std::vector<ReportField> &fields() const { return fields_; }
  /// Whether the device's reports start with a report ID byte.
  bool uses_report_ids() const { return uses_report_ids_; }
  /// Size in bytes of the Input report with the given ID (0 = no report IDs),
  /// without the ID byte; 0 if unknown.
  size_t report_bytes(uint8_t report_id) const {
    auto it = bit_cursor_.find(report_id);
    return it == bit_cursor_.end() ? 0 : (it->second + 7) / 8;
  }

  /// Extract a field's value from a report (`report` excludes the ID byte;
  /// use split() first). Fields with a negative logical minimum are
  /// sign-extended. Returns nullopt when the field belongs to another report
  /// ID or the report is too short.
  static std::optional<int32_t> extract(const ReportField &f, uint8_t report_id,
                                        std::span<const uint8_t> report) {
    if (f.report_id != report_id || f.bit_size == 0) {
      return std::nullopt;
    }
    const uint32_t last_bit = f.bit_offset + f.bit_size - 1;
    if ((last_bit >> 3) >= report.size()) {
      return std::nullopt;
    }
    uint32_t raw = 0;
    if ((f.bit_offset & 7) == 0 && (f.bit_size & 7) == 0) {
      // byte-aligned whole bytes (8 / 16 / 24 / 32 bits: axes, wheels, keycode
      // array slots -- most of what a report carries besides single bits): take
      // them a byte at a time instead of a bit at a time. HID reports are
      // little-endian, which is the order the bit loop below builds as well.
      const size_t first = f.bit_offset >> 3;
      for (unsigned i = 0; i < f.bit_size >> 3; i++) {
        raw |= static_cast<uint32_t>(report[first + i]) << (8 * i);
      }
    } else {
      for (unsigned i = 0; i < f.bit_size && i < 32; i++) {
        const uint32_t bit = f.bit_offset + i;
        raw |= static_cast<uint32_t>((report[bit >> 3] >> (bit & 7)) & 1) << i;
      }
    }
    if (f.logical_min < 0 && f.bit_size < 32 && (raw & (1u << (f.bit_size - 1)))) {
      return std::bit_cast<int32_t>(raw | (~0u << f.bit_size)); // sign-extend
    }
    return std::bit_cast<int32_t>(raw); // a full 32-bit field: two's complement as-is
  }

  /// Split a raw report into its ID and payload according to the map.
  struct SplitReport {
    uint8_t report_id{0};
    std::span<const uint8_t> payload{};
  };
  SplitReport split(std::span<const uint8_t> raw) const {
    if (raw.empty()) {
      return {};
    }
    if (uses_report_ids_) {
      return {raw[0], raw.subspan(1)};
    }
    return {0, raw};
  }

private:
  using View = ::hid::rdf::descriptor_view;
  class Parser : public ::hid::rdf::parser<View::iterator> {
  public:
    using base = ::hid::rdf::parser<View::iterator>;
    using item_type = base::item_type;
    using items_view_type = base::items_view_type;
    using control = base::control;
    using global_item_store = ::hid::rdf::global_item_store;

    Parser(ReportMap &map, const View &view)
        : map_(map) {
      base::parse_items(view);
    }

    control parse_report_data_field(const item_type &main_item,
                                    const global_item_store &global_state,
                                    const items_view_type &main_section,
                                    unsigned /*tlc_number*/) override {
      namespace rdf = ::hid::rdf;
      if (main_item.main_tag() != rdf::main::tag::INPUT) {
        return control::CONTINUE;
      }
      const uint32_t flags = main_item.value_unsigned();
      const bool constant = flags & 0x1;
      const bool variable = flags & 0x2;
      const bool relative = flags & 0x4;
      auto global_u = [&](rdf::global::tag tag, uint32_t def) {
        const auto *it = global_state.get_item(tag);
        return it ? it->value_unsigned() : def;
      };
      auto global_s = [&](rdf::global::tag tag, int32_t def) {
        const auto *it = global_state.get_item(tag);
        return it ? it->value_signed() : def;
      };
      const uint32_t report_size = global_u(rdf::global::tag::REPORT_SIZE, 0);
      const uint32_t report_count = global_u(rdf::global::tag::REPORT_COUNT, 0);
      const uint8_t report_id = static_cast<uint8_t>(global_u(rdf::global::tag::REPORT_ID, 0));
      const uint16_t usage_page = static_cast<uint16_t>(global_u(rdf::global::tag::USAGE_PAGE, 0));
      const int32_t logical_min = global_s(rdf::global::tag::LOGICAL_MINIMUM, 0);
      const int32_t logical_max = global_s(rdf::global::tag::LOGICAL_MAXIMUM, 0);
      if (report_size == 0 || report_count == 0) {
        return control::CONTINUE; // a field without bits: nothing in the report to map
      }
      if (report_id != 0) {
        map_.uses_report_ids_ = true;
      }
      uint32_t &cursor = map_.bit_cursor_[report_id];

      // local items of this main section: a usage list and/or a usage range
      std::vector<uint32_t> usages; // full usage (page << 16 | id)
      uint32_t usage_min = 0, usage_max = 0;
      bool have_min = false, have_max = false;
      for (const auto &it : main_section) {
        if (it.has_tag(rdf::local::tag::USAGE)) {
          usages.push_back(base::get_usage(it, global_state));
        } else if (it.has_tag(rdf::local::tag::USAGE_MINIMUM)) {
          usage_min = base::get_usage(it, global_state);
          have_min = true;
        } else if (it.has_tag(rdf::local::tag::USAGE_MAXIMUM)) {
          usage_max = base::get_usage(it, global_state);
          have_max = true;
        }
      }
      // a range needs both bounds; infer a missing one from the field count
      // (usages are consecutive), which is what such descriptors mean. Only the
      // usage ID moves: a full usage is page << 16 | id, so adding to the
      // combined value would spill into (or borrow from) the page.
      const bool have_range = have_min || have_max;
      const uint32_t span = report_count - 1;
      if (have_min && !have_max) {
        usage_max =
            with_usage_id(usage_min, std::min<uint32_t>(0xFFFF, usage_id(usage_min) + span));
      } else if (have_max && !have_min) {
        const uint32_t id = usage_id(usage_max);
        usage_min = with_usage_id(usage_max, id >= span ? id - span : 0);
      }

      if (constant) {
        cursor += report_size * report_count; // padding
        return control::CONTINUE;
      }
      if (report_size == 0 || report_size > 32) {
        // extract() returns an int32_t, so a field wider than 32 bits cannot be
        // represented. Skip it instead of silently reporting its low 32 bits as
        // the value; the cursor still advances by the full width so every later
        // field keeps its correct offset. (A 0-bit field carries nothing.)
        cursor += report_size * report_count;
        return control::CONTINUE;
      }
      for (uint32_t n = 0; n < report_count; n++) {
        ReportField f;
        f.report_id = report_id;
        f.bit_offset = cursor;
        f.bit_size = static_cast<uint8_t>(report_size); // 1..32, checked above
        f.logical_min = logical_min;
        f.logical_max = logical_max;
        f.relative = relative;
        if (variable) {
          uint32_t u = 0;
          if (have_range) {
            // advance the usage ID only: adding n to the combined page << 16
            // | id would carry into the page near the top of it
            u = with_usage_id(usage_min, std::min<uint32_t>(usage_id(usage_min) + n, 0xFFFF));
            if ((u & 0xFFFF0000u) == (usage_max & 0xFFFF0000u)) {
              u = std::min(u, usage_max); // same page: the maximum still bounds it
            }
          } else if (!usages.empty()) {
            u = usages[std::min<size_t>(n, usages.size() - 1)];
          }
          f.usage_page = u ? static_cast<uint16_t>(u >> 16) : usage_page;
          f.usage = static_cast<uint16_t>(u & 0xFFFF);
        } else {
          // array item: each slot holds a usage index into [usage_min, usage_max]
          f.array = true;
          uint32_t lo = 0, hi = 0;
          if (have_range) {
            lo = usage_min;
            hi = usage_max;
          } else if (!usages.empty()) {
            // An explicit usage list need not be sorted, so the bounds come from
            // the whole list rather than its ends. A field carries one usage
            // page, so a list spanning pages contributes only the entries on the
            // first usage's page.
            const uint32_t page = usages.front() & 0xFFFF0000u;
            lo = hi = usages.front();
            for (uint32_t u : usages) {
              if ((u & 0xFFFF0000u) != page) {
                continue;
              }
              lo = std::min(lo, u);
              hi = std::max(hi, u);
            }
          }
          f.usage_page = lo ? static_cast<uint16_t>(lo >> 16) : usage_page;
          f.usage = static_cast<uint16_t>(lo & 0xFFFF);
          f.usage_max = static_cast<uint16_t>(hi & 0xFFFF);
        }
        cursor += report_size;
        map_.fields_.push_back(f);
      }
      return control::CONTINUE;
    }

  private:
    // A "full usage" is page << 16 | id; these keep the two halves apart so
    // arithmetic on the ID cannot walk into the page.
    static constexpr uint32_t usage_id(uint32_t full) { return full & 0xFFFFu; }
    static constexpr uint32_t with_usage_id(uint32_t full, uint32_t id) {
      return (full & 0xFFFF0000u) | (id & 0xFFFFu);
    }

    ReportMap &map_;
  };

  std::vector<ReportField> fields_;
  std::map<uint8_t, uint32_t> bit_cursor_; ///< input bit position per report ID
  bool uses_report_ids_{false};
};

// ---------------------------------------------------------------------------
// Decoders
// ---------------------------------------------------------------------------

/// HID usage pages / IDs the decoders look at.
namespace usage {
constexpr uint16_t PAGE_GENERIC_DESKTOP = 0x01;
constexpr uint16_t PAGE_KEYBOARD = 0x07;
constexpr uint16_t PAGE_BUTTON = 0x09;
constexpr uint16_t PAGE_CONSUMER = 0x0C;
constexpr uint16_t GD_X = 0x30, GD_Y = 0x31, GD_Z = 0x32, GD_RX = 0x33, GD_RY = 0x34, GD_RZ = 0x35;
constexpr uint16_t GD_WHEEL = 0x38, GD_HAT = 0x39;
constexpr uint16_t GD_DPAD_UP = 0x90, GD_DPAD_DOWN = 0x91, GD_DPAD_RIGHT = 0x92,
                   GD_DPAD_LEFT = 0x93;
constexpr uint16_t CONSUMER_MENU = 0x0040, CONSUMER_RECORD = 0x00B2, CONSUMER_AC_EXIT = 0x0204,
                   CONSUMER_AC_PROPERTIES = 0x0209, CONSUMER_AC_HOME = 0x0223,
                   CONSUMER_AC_BACK = 0x0224;
} // namespace usage

/// A gamepad report decoded into positions rather than button numbers.
struct GamepadReport {
  // face buttons by position (Nintendo naming of the positions: B is south)
  bool south{false}, east{false}, west{false}, north{false};
  bool l1{false}, r1{false}, l2{false}, r2{false}; ///< shoulders / triggers as buttons
  bool l3{false}, r3{false};                       ///< stick clicks
  bool select{false}, start{false}, home{false};
  bool up{false}, down{false}, left{false}, right{false}; ///< d-pad / hat / left stick
  int16_t lx{0}, ly{0}, rx{0}, ry{0}; ///< sticks, -32767..32767, y grows downwards
  int32_t hat{-1};                    ///< raw hat value (-1 when absent)
  uint64_t buttons{0};                ///< raw: bit n = button usage n pressed (usages 1..63)
  uint16_t consumer[4]{0, 0, 0, 0};   ///< raw consumer-page usages active
};

/// Decodes gamepad reports. Face-button numbering differs between controller
/// families, so the decoder carries a layout:
///  - Xbox-style: 1 = south, 2 = east, 3 = west, 4 = north (default)
///  - DirectInput / DualShock: 1 = west, 2 = south, 3 = east, 4 = north
///    (picked when the descriptor has a hat switch and >= 13 buttons)
/// Always: 5/6 = L1/R1, 7/8 = L2/R2, 9 = select, 10 = start, 11/12 = L3/R3.
/// Consumer-page controls (Menu, AC Back/Home, Record, AC Properties/Exit as
/// sent by Xbox-style pads) map to start / select / home. Per-device quirks
/// are keyed on VID/PID (apply_quirks()).
class GamepadDecoder {
public:
  enum class Layout { Xbox, DirectInput };
  struct Quirks {
    Layout layout{Layout::Xbox};
    bool dpad_updown_swapped{false}; ///< usages 0x90/0x91 carry down/up
    bool y_up_positive{false};       ///< stick Y grows upwards (HID convention is downwards)
  };

  /// Takes ownership of the map: pass an rvalue (`std::move(map)`) to hand it
  /// over, an lvalue to keep your own copy. Use the static looks_like_*()
  /// to classify a device without constructing a decoder per kind.
  explicit GamepadDecoder(ReportMap map)
      : map_(std::move(map)) {
    size_t buttons = 0;
    bool hat = false;
    for (const auto &f : map_.fields()) {
      if (f.usage_page == usage::PAGE_BUTTON && !f.array)
        buttons++;
      if (f.usage_page == usage::PAGE_GENERIC_DESKTOP && f.usage == usage::GD_HAT)
        hat = true;
    }
    if (hat && buttons >= 13) {
      quirks_.layout = Layout::DirectInput;
    }
  }

  /// Whether a map looks like a gamepad (buttons plus a direction source).
  static bool looks_like_gamepad(const ReportMap &map) {
    bool buttons = false, direction = false;
    for (const auto &f : map.fields()) {
      if (f.usage_page == usage::PAGE_BUTTON)
        buttons = true;
      if (f.usage_page == usage::PAGE_GENERIC_DESKTOP &&
          (f.usage == usage::GD_HAT || f.usage == usage::GD_X || f.usage == usage::GD_Y ||
           (f.usage >= usage::GD_DPAD_UP && f.usage <= usage::GD_DPAD_LEFT)))
        direction = true;
    }
    return buttons && direction;
  }
  /// Whether this decoder's map looks like a gamepad.
  bool looks_like_gamepad() const { return looks_like_gamepad(map_); }

  /// Apply the known per-controller quirks for a vendor / product ID.
  void apply_quirks(uint16_t vid, uint16_t pid) {
    struct Entry {
      uint16_t vid{0};
      uint16_t pid{0};
      Quirks quirks{};
    };
    static constexpr Entry table[] = {
        {0x054C, 0x05C4, {Layout::DirectInput, false, false}}, // Sony DualShock 4
        {0x054C, 0x09CC, {Layout::DirectInput, false, false}}, // Sony DualShock 4 (v2)
        {0x054C, 0x0CE6, {Layout::DirectInput, false, false}}, // Sony DualSense
        {0x358A, 0x0402, {Layout::Xbox, false, true}},         // Backbone Pro (stick Y grows up)
    };
    const auto *e =
        std::find_if(std::begin(table), std::end(table), [vid, pid](const Entry &entry) {
          return entry.vid == vid && entry.pid == pid;
        });
    if (e != std::end(table)) {
      quirks_ = e->quirks;
    }
  }
  const Quirks &quirks() const { return quirks_; }
  Quirks &quirks() { return quirks_; }
  const ReportMap &map() const { return map_; }

  /// Decode a raw report (with its ID byte, if the device uses IDs). Returns
  /// false when the report carries none of the mapped fields.
  bool decode(std::span<const uint8_t> raw, GamepadReport &out) const {
    const auto [id, report] = map_.split(raw);
    GamepadReport r{};
    bool any = false;
    const bool xbox = quirks_.layout == Layout::Xbox;
    const uint16_t south = xbox ? 1 : 2, east = xbox ? 2 : 3, west = xbox ? 3 : 1, north = 4;
    size_t n_consumer = 0;
    for (const auto &f : map_.fields()) {
      const auto v = ReportMap::extract(f, id, report);
      if (!v)
        continue;
      any = true;
      if (f.usage_page == usage::PAGE_BUTTON && !f.array) {
        const bool pressed = *v != 0;
        if (pressed && f.usage >= 1 && f.usage < 64)
          r.buttons |= uint64_t{1} << f.usage;
        if (f.usage == south)
          r.south |= pressed;
        else if (f.usage == east)
          r.east |= pressed;
        else if (f.usage == west)
          r.west |= pressed;
        else if (f.usage == north)
          r.north |= pressed;
        else if (f.usage == 5)
          r.l1 |= pressed;
        else if (f.usage == 6)
          r.r1 |= pressed;
        else if (f.usage == 7)
          r.l2 |= pressed;
        else if (f.usage == 8)
          r.r2 |= pressed;
        else if (f.usage == 9)
          r.select |= pressed;
        else if (f.usage == 10)
          r.start |= pressed;
        else if (f.usage == 11)
          r.l3 |= pressed;
        else if (f.usage == 12)
          r.r3 |= pressed;
      } else if (f.usage_page == usage::PAGE_GENERIC_DESKTOP) {
        if (f.usage == usage::GD_HAT) {
          r.hat = *v;
          const int32_t dir =
              *v - f.logical_min; // 0 = up, clockwise; 8 (or out of range) = centered
          if (*v >= f.logical_min && *v <= f.logical_max && dir >= 0 && dir < 8) {
            r.up |= dir == 7 || dir == 0 || dir == 1;
            r.right |= dir >= 1 && dir <= 3;
            r.down |= dir >= 3 && dir <= 5;
            r.left |= dir >= 5;
          }
        } else if (f.usage >= usage::GD_DPAD_UP && f.usage <= usage::GD_DPAD_LEFT) {
          const bool pressed = *v != 0;
          switch (f.usage) {
          case usage::GD_DPAD_UP:
            (quirks_.dpad_updown_swapped ? r.down : r.up) |= pressed;
            break;
          case usage::GD_DPAD_DOWN:
            (quirks_.dpad_updown_swapped ? r.up : r.down) |= pressed;
            break;
          case usage::GD_DPAD_RIGHT:
            r.right |= pressed;
            break;
          case usage::GD_DPAD_LEFT:
            r.left |= pressed;
            break;
          default:
            break;
          }
        } else if (f.usage >= usage::GD_X && f.usage <= usage::GD_RZ && !f.relative &&
                   f.logical_max > f.logical_min) {
          // 64-bit throughout: a full 32-bit logical range overflows int32_t
          const int64_t range = static_cast<int64_t>(f.logical_max) - f.logical_min;
          const int64_t centered = 2 * (static_cast<int64_t>(*v) - f.logical_min) - range;
          int16_t n =
              static_cast<int16_t>(std::clamp<int64_t>(centered * 32767 / range, -32767, 32767));
          const bool vertical =
              f.usage == usage::GD_Y || f.usage == usage::GD_RZ || f.usage == usage::GD_RY;
          if (vertical && quirks_.y_up_positive)
            n = static_cast<int16_t>(-n);
          switch (f.usage) {
          case usage::GD_X:
            r.lx = n;
            break;
          case usage::GD_Y:
            r.ly = n;
            break;
          case usage::GD_Z:
          case usage::GD_RX:
            r.rx = n;
            break;
          case usage::GD_RZ:
          case usage::GD_RY:
            r.ry = n;
            break;
          default:
            break;
          }
        }
      } else if (f.usage_page == usage::PAGE_CONSUMER) {
        uint16_t u = 0;
        if (f.array) {
          if (*v == f.logical_min && (f.usage != 0 || f.logical_min != 0))
            continue; // empty slot
          if (*v == 0 && f.usage == 0)
            continue;
          u = static_cast<uint16_t>(f.usage + (*v - f.logical_min));
        } else {
          if (!*v)
            continue;
          u = f.usage;
        }
        if (n_consumer < 4)
          r.consumer[n_consumer++] = u;
        switch (u) {
        case usage::CONSUMER_MENU:
        case usage::CONSUMER_AC_PROPERTIES:
          r.start = true;
          break;
        case usage::CONSUMER_AC_BACK:
        case usage::CONSUMER_AC_EXIT:
          r.select = true;
          break;
        case usage::CONSUMER_AC_HOME:
        case usage::CONSUMER_RECORD:
          r.home = true;
          break;
        default:
          break;
        }
      }
    }
    // the left stick as a d-pad with a 40% dead zone
    constexpr int16_t DEAD = 32767 * 2 / 5;
    r.left |= r.lx < -DEAD;
    r.right |= r.lx > DEAD;
    r.up |= r.ly < -DEAD;
    r.down |= r.ly > DEAD;
    if (!any)
      return false;
    out = r;
    return true;
  }

private:
  ReportMap map_;
  Quirks quirks_{};
};

/// A keyboard report: which usages (HID keyboard page, 0..255 incl. the
/// modifiers at 0xE0..0xE7) are pressed.
struct KeyboardReport {
  uint8_t keys[32]{}; ///< bit (usage & 7) of keys[usage >> 3]
  bool pressed(uint8_t usage) const { return (keys[usage >> 3] >> (usage & 7)) & 1; }
};

/// Decodes keyboard reports: boot-style 8-byte reports (modifier bitmap +
/// array of key usages) and NKRO bitmaps alike, from the descriptor.
class KeyboardDecoder {
public:
  /// Takes ownership of the map: pass an rvalue (`std::move(map)`) to hand it
  /// over, an lvalue to keep your own copy. Use the static looks_like_*()
  /// to classify a device without constructing a decoder per kind.
  explicit KeyboardDecoder(ReportMap map)
      : map_(std::move(map)) {}
  /// Whether a map looks like a keyboard (keyboard-page fields).
  static bool looks_like_keyboard(const ReportMap &map) {
    return std::any_of(map.fields().begin(), map.fields().end(),
                       [](const ReportField &f) { return f.usage_page == usage::PAGE_KEYBOARD; });
  }
  /// Whether this decoder's map looks like a keyboard.
  bool looks_like_keyboard() const { return looks_like_keyboard(map_); }
  bool decode(std::span<const uint8_t> raw, KeyboardReport &out) const {
    const auto [id, report] = map_.split(raw);
    KeyboardReport r{};
    bool any = false;
    for (const auto &f : map_.fields()) {
      if (f.usage_page != usage::PAGE_KEYBOARD)
        continue;
      const auto v = ReportMap::extract(f, id, report);
      if (!v)
        continue;
      any = true;
      if (f.array) {
        // an unused slot reads as the logical minimum (0 in the usual boot
        // descriptor, but not every keyboard uses 0)
        if (*v == f.logical_min)
          continue;
        const int32_t u = f.usage + (*v - f.logical_min);
        if (u > 0 && u < 256)
          r.keys[u >> 3] |= 1 << (u & 7);
      } else if (*v && f.usage < 256) {
        r.keys[f.usage >> 3] |= 1 << (f.usage & 7);
      }
    }
    if (!any)
      return false;
    out = r;
    return true;
  }
  const ReportMap &map() const { return map_; }

private:
  ReportMap map_;
};

/// A mouse report: relative motion and buttons.
struct MouseReport {
  int32_t dx{0}, dy{0}, wheel{0};
  uint8_t buttons{0}; ///< bit0 left, bit1 right, bit2 middle, ...
};

/// Decodes mouse reports (relative X / Y / wheel and buttons) from the
/// descriptor.
class MouseDecoder {
public:
  /// Takes ownership of the map: pass an rvalue (`std::move(map)`) to hand it
  /// over, an lvalue to keep your own copy. Use the static looks_like_*()
  /// to classify a device without constructing a decoder per kind.
  explicit MouseDecoder(ReportMap map)
      : map_(std::move(map)) {}
  /// Whether a map looks like a mouse (relative pointer axes).
  static bool looks_like_mouse(const ReportMap &map) {
    return std::any_of(map.fields().begin(), map.fields().end(), [](const ReportField &f) {
      return f.usage_page == usage::PAGE_GENERIC_DESKTOP && f.relative &&
             (f.usage == usage::GD_X || f.usage == usage::GD_Y);
    });
  }
  /// Whether this decoder's map looks like a mouse.
  bool looks_like_mouse() const { return looks_like_mouse(map_); }
  bool decode(std::span<const uint8_t> raw, MouseReport &out) const {
    const auto [id, report] = map_.split(raw);
    MouseReport r{};
    bool any = false;
    for (const auto &f : map_.fields()) {
      const auto v = ReportMap::extract(f, id, report);
      if (!v)
        continue;
      if (f.usage_page == usage::PAGE_BUTTON && !f.array) {
        any = true;
        if (*v && f.usage >= 1 && f.usage <= 8)
          r.buttons |= 1 << (f.usage - 1);
      } else if (f.usage_page == usage::PAGE_GENERIC_DESKTOP && f.relative) {
        any = true;
        if (f.usage == usage::GD_X)
          r.dx = *v;
        else if (f.usage == usage::GD_Y)
          r.dy = *v;
        else if (f.usage == usage::GD_WHEEL)
          r.wheel = *v;
      }
    }
    if (!any)
      return false;
    out = r;
    return true;
  }
  const ReportMap &map() const { return map_; }

private:
  ReportMap map_;
};

} // namespace hid_rp
} // namespace espp

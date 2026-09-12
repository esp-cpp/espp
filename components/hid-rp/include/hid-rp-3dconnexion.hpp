#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "format.hpp"
#include "hid-rp.hpp"

namespace espp {

/// HID 3Dconnexion SpaceMouse Translation Input Report
///
/// This class implements the HID Input Report used by 3Dconnexion SpaceMouse
/// devices (SpaceNavigator, SpaceMouse Wireless / Compact / Pro / Enterprise,
/// etc.) to report the translation (X, Y, Z panning) half of the device's
/// 6-DoF sensor. These devices enumerate on the standard Generic Desktop
/// (0x01) usage page as a Multi-Axis Controller (0x08) and split their 6
/// degrees of freedom across three separate Report IDs: translation (this
/// class, Report ID 1), rotation (see espp::SpaceMouseRotationInputReport,
/// Report ID 2), and buttons (see espp::SpaceMouseButtonsInputReport, Report
/// ID 3). See espp::spacemouse_descriptor() for how these are combined into a
/// single, complete report descriptor.
///
/// \note The report layout below (usages X/Y/Z, 16-bit signed logical range
///       of [-350, 350], physical range of [-1400, 1400] with a unit
///       exponent of 10^-4 x centimeter, and the "relative" data flag) was
///       verified against a genuine 3Dconnexion SpaceNavigator's HID report
///       descriptor, not guessed. Sources:
///       - https://github.com/AndunHH/spacemouse/blob/main/SpaceNavigator.md
///       -
///       https://udev-hid-bpf-bentiss-648236040b7c508ff54e7bc3510428536d7fd37b91.pages.freedesktop.org/case-study-spacenavigator.html
///         (raw descriptor bytes captured from real hardware, decoded below)
///       Real hardware marks these axes as *relative* (`Input (Data,Var,Rel)`)
///       even though the values are actually the instantaneous, absolute
///       displacement of the spring-loaded cap (it springs back to 0 when
///       released) rather than an accumulated delta. This is a well known,
///       documented quirk/bug of the 3Dconnexion firmware (see the
///       udev-hid-bpf case study above, which exists specifically to work
///       around it for the Linux Gamepad API). This class replicates the
///       flag faithfully so that host software written against the real
///       device parses this descriptor identically; callers on the espp side
///       still just set/get plain axis values via set_translation()/
///       get_translation().
///
/// \section hid_rp_3dconnexion_ex1 HID-RP 3Dconnexion SpaceMouse Example
/// \snippet hid_rp_example.cpp hid rp example
//
// NOTE: the whole class (including the base `hid::report::base`) must be
// defined with 1-byte packing so that no alignment padding is inserted
// before the 2-byte-aligned `axes` array below; get_report()/set_data() rely
// on the wire layout being exactly
// [report id][x_lo][x_hi][y_lo][y_hi][z_lo][z_hi] with no gaps. The pragma
// must precede the `class` keyword itself (not just appear inside the class
// body) for compilers to honor it for the class's own layout.
#pragma pack(push, 1)
template <std::int16_t LOGICAL_MIN = -350, std::int16_t LOGICAL_MAX = 350, uint8_t REPORT_ID = 1>
class SpaceMouseTranslationInputReport
    : public hid::report::base<hid::report::type::INPUT, REPORT_ID> {
public:
  static constexpr std::int16_t logical_min = LOGICAL_MIN; ///< Minimum raw axis value
  static constexpr std::int16_t logical_max = LOGICAL_MAX; ///< Maximum raw axis value
  static constexpr std::int16_t physical_min =
      static_cast<std::int16_t>(LOGICAL_MIN * 4); ///< Physical minimum
  static constexpr std::int16_t physical_max =
      static_cast<std::int16_t>(LOGICAL_MAX * 4); ///< Physical maximum
  /// Unit exponent (base 10) applied to the physical value, in the HID "SI
  /// Linear" unit system (whose base length unit is the centimeter); -4
  /// therefore expresses the physical range in units of 10^-4 cm (0.1 mm).
  static constexpr std::int8_t unit_exponent = -4;
  static constexpr std::size_t num_data_bytes = 3 * sizeof(std::int16_t); ///< X, Y, Z (int16 each)

protected:
  std::array<std::int16_t, 3> axes{0, 0, 0}; // X, Y, Z

public:
  /// Construct a new Translation Input Report object
  constexpr SpaceMouseTranslationInputReport() { reset(); }

  /// Reset the translation axes to their centered (0) value
  constexpr void reset() { axes.fill(0); }

  /// Set the X (left/right) translation axis value
  /// \param value The value to set the X axis to, in the range [logical_min, logical_max].
  constexpr void set_x(std::int16_t value) {
    axes[0] = std::clamp(value, logical_min, logical_max);
  }

  /// Set the Y (forward/backward) translation axis value
  /// \param value The value to set the Y axis to, in the range [logical_min, logical_max].
  constexpr void set_y(std::int16_t value) {
    axes[1] = std::clamp(value, logical_min, logical_max);
  }

  /// Set the Z (up/down) translation axis value
  /// \param value The value to set the Z axis to, in the range [logical_min, logical_max].
  constexpr void set_z(std::int16_t value) {
    axes[2] = std::clamp(value, logical_min, logical_max);
  }

  /// Set all three translation axes at once
  /// \param x The X axis value, in the range [logical_min, logical_max].
  /// \param y The Y axis value, in the range [logical_min, logical_max].
  /// \param z The Z axis value, in the range [logical_min, logical_max].
  constexpr void set_translation(std::int16_t x, std::int16_t y, std::int16_t z) {
    set_x(x);
    set_y(y);
    set_z(z);
  }

  /// Get the X (left/right) translation axis value
  /// \return The X axis value.
  constexpr std::int16_t get_x() const { return axes[0]; }

  /// Get the Y (forward/backward) translation axis value
  /// \return The Y axis value.
  constexpr std::int16_t get_y() const { return axes[1]; }

  /// Get the Z (up/down) translation axis value
  /// \return The Z axis value.
  constexpr std::int16_t get_z() const { return axes[2]; }

  /// Get all three translation axes at once
  /// \param x[out] The X axis value.
  /// \param y[out] The Y axis value.
  /// \param z[out] The Z axis value.
  constexpr void get_translation(std::int16_t &x, std::int16_t &y, std::int16_t &z) const {
    x = axes[0];
    y = axes[1];
    z = axes[2];
  }

  /// Get the input report as a vector of bytes
  /// \return The input report as a vector of bytes.
  /// \note The report id is not included in the returned vector.
  constexpr auto get_report() const {
    // the first byte is the id, which we don't want...
    size_t offset = 1;
    auto report_data = this->data() + offset;
    auto report_size = num_data_bytes;
    return std::vector<uint8_t>(report_data, report_data + report_size);
  }

  /// Set the input report data from a vector of bytes
  /// \param data The data to set the input report to.
  constexpr void set_data(const std::vector<uint8_t> &data) {
    // copy the data into our data array - skip the first byte, which is the
    // report id. Clamp the copy length to the report's payload size so an
    // over-long input cannot write past the backing storage.
    auto copy_size = std::min(data.size(), num_data_bytes);
    std::copy(data.begin(), data.begin() + copy_size, this->data() + 1);
  }

  /// Get the report descriptor as a hid::rdf::descriptor
  /// \return The report descriptor as a hid::rdf::descriptor.
  /// \note This is an incomplete descriptor, you will need to add it to a
  ///      collection::application descriptor to create a complete report descriptor.
  ///      See espp::spacemouse_descriptor() for a complete example.
  static constexpr auto get_descriptor() {
    using namespace hid::page;
    using namespace hid::rdf;

    // clang-format off
      return collection::physical(
                        conditional_report_id<REPORT_ID>(),
                        logical_limits<2, 2>(logical_min, logical_max),
                        physical_limits<2, 2>(physical_min, physical_max),
                        unit::centimeter(unit_exponent),
                        usage(generic_desktop::X),
                        usage(generic_desktop::Y),
                        usage(generic_desktop::Z),
                        report_size(16),
                        report_count(3),
                        input::relative_variable()
                        );
    // clang-format on
  }

  friend fmt::formatter<SpaceMouseTranslationInputReport<LOGICAL_MIN, LOGICAL_MAX, REPORT_ID>>;
};
#pragma pack(pop)

/// HID 3Dconnexion SpaceMouse Rotation Input Report
///
/// This class implements the HID Input Report used by 3Dconnexion SpaceMouse
/// devices to report the rotation (Rx, Ry, Rz tilt/twist) half of the
/// device's 6-DoF sensor. It is a sibling of
/// espp::SpaceMouseTranslationInputReport (see that class for the full
/// description of the device and citations for the verified layout) and uses
/// the same 16-bit signed logical/physical range and unit as the translation
/// report - the real hardware does not re-declare a distinct angular unit for
/// this report, instead simply reusing (via HID global item persistence)
/// whatever unit/range was last declared by the translation collection. This
/// is a device quirk, faithfully reproduced here, not an oversight.
///
/// \section hid_rp_3dconnexion_rot_ex1 HID-RP 3Dconnexion SpaceMouse Example
/// \snippet hid_rp_example.cpp hid rp example
// See espp::SpaceMouseTranslationInputReport for why the whole class (and
// the pragma preceding the `class` keyword) is needed: it removes the
// alignment padding that would otherwise be inserted before the
// 2-byte-aligned `axes` array.
#pragma pack(push, 1)
template <std::int16_t LOGICAL_MIN = -350, std::int16_t LOGICAL_MAX = 350, uint8_t REPORT_ID = 2>
class SpaceMouseRotationInputReport
    : public hid::report::base<hid::report::type::INPUT, REPORT_ID> {
public:
  static constexpr std::int16_t logical_min = LOGICAL_MIN; ///< Minimum raw axis value
  static constexpr std::int16_t logical_max = LOGICAL_MAX; ///< Maximum raw axis value
  static constexpr std::int16_t physical_min =
      static_cast<std::int16_t>(LOGICAL_MIN * 4); ///< Physical minimum
  static constexpr std::int16_t physical_max =
      static_cast<std::int16_t>(LOGICAL_MAX * 4);  ///< Physical maximum
  static constexpr std::int8_t unit_exponent = -4; ///< See espp::SpaceMouseTranslationInputReport
  static constexpr std::size_t num_data_bytes =
      3 * sizeof(std::int16_t); ///< Rx, Ry, Rz (int16 each)

protected:
  std::array<std::int16_t, 3> axes{0, 0, 0}; // Rx, Ry, Rz

public:
  /// Construct a new Rotation Input Report object
  constexpr SpaceMouseRotationInputReport() { reset(); }

  /// Reset the rotation axes to their centered (0) value
  constexpr void reset() { axes.fill(0); }

  /// Set the Rx (pitch, tilt forward/back) rotation axis value
  /// \param value The value to set the Rx axis to, in the range [logical_min, logical_max].
  constexpr void set_rx(std::int16_t value) {
    axes[0] = std::clamp(value, logical_min, logical_max);
  }

  /// Set the Ry (yaw, twist left/right) rotation axis value
  /// \param value The value to set the Ry axis to, in the range [logical_min, logical_max].
  constexpr void set_ry(std::int16_t value) {
    axes[1] = std::clamp(value, logical_min, logical_max);
  }

  /// Set the Rz (roll, tilt left/right) rotation axis value
  /// \param value The value to set the Rz axis to, in the range [logical_min, logical_max].
  constexpr void set_rz(std::int16_t value) {
    axes[2] = std::clamp(value, logical_min, logical_max);
  }

  /// Set all three rotation axes at once
  /// \param rx The Rx axis value, in the range [logical_min, logical_max].
  /// \param ry The Ry axis value, in the range [logical_min, logical_max].
  /// \param rz The Rz axis value, in the range [logical_min, logical_max].
  constexpr void set_rotation(std::int16_t rx, std::int16_t ry, std::int16_t rz) {
    set_rx(rx);
    set_ry(ry);
    set_rz(rz);
  }

  /// Get the Rx (pitch) rotation axis value
  /// \return The Rx axis value.
  constexpr std::int16_t get_rx() const { return axes[0]; }

  /// Get the Ry (yaw) rotation axis value
  /// \return The Ry axis value.
  constexpr std::int16_t get_ry() const { return axes[1]; }

  /// Get the Rz (roll) rotation axis value
  /// \return The Rz axis value.
  constexpr std::int16_t get_rz() const { return axes[2]; }

  /// Get all three rotation axes at once
  /// \param rx[out] The Rx axis value.
  /// \param ry[out] The Ry axis value.
  /// \param rz[out] The Rz axis value.
  constexpr void get_rotation(std::int16_t &rx, std::int16_t &ry, std::int16_t &rz) const {
    rx = axes[0];
    ry = axes[1];
    rz = axes[2];
  }

  /// Get the input report as a vector of bytes
  /// \return The input report as a vector of bytes.
  /// \note The report id is not included in the returned vector.
  constexpr auto get_report() const {
    // the first byte is the id, which we don't want...
    size_t offset = 1;
    auto report_data = this->data() + offset;
    auto report_size = num_data_bytes;
    return std::vector<uint8_t>(report_data, report_data + report_size);
  }

  /// Set the input report data from a vector of bytes
  /// \param data The data to set the input report to.
  constexpr void set_data(const std::vector<uint8_t> &data) {
    // copy the data into our data array - skip the first byte, which is the
    // report id. Clamp the copy length to the report's payload size so an
    // over-long input cannot write past the backing storage.
    auto copy_size = std::min(data.size(), num_data_bytes);
    std::copy(data.begin(), data.begin() + copy_size, this->data() + 1);
  }

  /// Get the report descriptor as a hid::rdf::descriptor
  /// \return The report descriptor as a hid::rdf::descriptor.
  /// \note This is an incomplete descriptor, you will need to add it to a
  ///      collection::application descriptor to create a complete report descriptor.
  ///      See espp::spacemouse_descriptor() for a complete example.
  static constexpr auto get_descriptor() {
    using namespace hid::page;
    using namespace hid::rdf;

    // clang-format off
      return collection::physical(
                        conditional_report_id<REPORT_ID>(),
                        usage(generic_desktop::RX),
                        usage(generic_desktop::RY),
                        usage(generic_desktop::RZ),
                        report_size(16),
                        report_count(3),
                        input::relative_variable()
                        );
    // clang-format on
  }

  friend fmt::formatter<SpaceMouseRotationInputReport<LOGICAL_MIN, LOGICAL_MAX, REPORT_ID>>;
};
#pragma pack(pop)

/// HID 3Dconnexion SpaceMouse Buttons Input Report
///
/// This class implements the HID Input Report used by 3Dconnexion SpaceMouse
/// devices to report the state of their buttons (Usage Page 0x09, Button).
/// The button count varies significantly across the SpaceMouse family: the
/// SpaceNavigator has 2 buttons, while the SpaceMouse Pro/Enterprise have
/// many more (15+); BUTTON_COUNT is therefore a template parameter, similar
/// to how espp::GamepadInputReport parameterizes its button count. The
/// SpaceNavigator's real 2-button report pads the fixed-size field out to a
/// full 2 bytes (2 button bits + 14 padding bits); this implementation
/// instead pads out to the next byte boundary for any BUTTON_COUNT (matching
/// espp::GamepadInputReport's convention) so that it generalizes cleanly
/// across the different models rather than hard-coding the 2-button case.
///
/// \note See espp::SpaceMouseTranslationInputReport for citations on the
///       verified SpaceMouse HID report layout.
///
/// \section hid_rp_3dconnexion_btn_ex1 HID-RP 3Dconnexion SpaceMouse Example
/// \snippet hid_rp_example.cpp hid rp example
template <std::size_t BUTTON_COUNT = 2, uint8_t REPORT_ID = 3>
class SpaceMouseButtonsInputReport : public hid::report::base<hid::report::type::INPUT, REPORT_ID> {
public:
  static constexpr std::size_t button_count = BUTTON_COUNT;
  static constexpr std::size_t num_button_bytes = (BUTTON_COUNT + 7) / 8;
  static constexpr std::size_t num_data_bytes = num_button_bytes;

protected:
  hid::report_bitset<hid::page::button, hid::page::button(1), hid::page::button(BUTTON_COUNT)>
      buttons;

public:
  /// Construct a new Buttons Input Report object
  constexpr SpaceMouseButtonsInputReport() { reset(); }

  /// Reset all buttons to the unpressed state
  constexpr void reset() { buttons.reset(); }

  /// Set the button value
  /// \param button_index The button for which you want to set the value.
  ///        Should be between 1 and BUTTON_COUNT, inclusive.
  /// \param value The true/false value you want to set the button to.
  constexpr void set_button(int button_index, bool value) {
    if (button_index < 1 || button_index > static_cast<int>(BUTTON_COUNT)) {
      return;
    }
    buttons.set(hid::page::button(button_index), value);
  }

  /// Get the button value
  /// \param button_index The button for which you want to get the value.
  ///        Should be between 1 and BUTTON_COUNT, inclusive.
  /// \return The true/false value of the button.
  constexpr bool get_button(int button_index) const {
    if (button_index < 1 || button_index > static_cast<int>(BUTTON_COUNT)) {
      return false;
    }
    return buttons.test(hid::page::button(button_index));
  }

  /// Get the input report as a vector of bytes
  /// \return The input report as a vector of bytes.
  /// \note The report id is not included in the returned vector.
  constexpr auto get_report() const {
    // the first byte is the id, which we don't want...
    size_t offset = 1;
    auto report_data = this->data() + offset;
    auto report_size = num_data_bytes;
    return std::vector<uint8_t>(report_data, report_data + report_size);
  }

  /// Set the input report data from a vector of bytes
  /// \param data The data to set the input report to.
  constexpr void set_data(const std::vector<uint8_t> &data) {
    // copy the data into our data array - skip the first byte, which is the
    // report id. Clamp the copy length to the report's payload size so an
    // over-long input cannot write past the backing storage.
    auto copy_size = std::min(data.size(), num_data_bytes);
    std::copy(data.begin(), data.begin() + copy_size, this->data() + 1);
  }

  /// Get the report descriptor as a hid::rdf::descriptor
  /// \return The report descriptor as a hid::rdf::descriptor.
  /// \note This is an incomplete descriptor, you will need to add it to a
  ///      collection::application descriptor to create a complete report descriptor.
  ///      See espp::spacemouse_descriptor() for a complete example.
  static constexpr auto get_descriptor() {
    using namespace hid::page;
    using namespace hid::rdf;

    // clang-format off
      return collection::logical(
                        conditional_report_id<REPORT_ID>(),
                        usage_page<button>(),
                        usage_limits(button(1), button(BUTTON_COUNT)),
                        logical_limits<1, 1>(0, 1),
                        report_size(1),
                        report_count(BUTTON_COUNT),
                        input::absolute_variable(),
                        logical_limits<1, 1>(0, 0),
                        input::byte_padding<BUTTON_COUNT>()
                        );
    // clang-format on
  }

  friend fmt::formatter<SpaceMouseButtonsInputReport<BUTTON_COUNT, REPORT_ID>>;
};

/// HID 3Dconnexion SpaceMouse LED Output Report
///
/// This class implements the (optional) HID Output Report used by some
/// 3Dconnexion SpaceMouse devices (e.g. the SpaceNavigator) to control a
/// single status LED. It is not required to emulate a functional SpaceMouse
/// (the translation, rotation, and buttons input reports are what matter for
/// that), but is included for completeness since real hardware exposes it as
/// Report ID 4.
///
/// \section hid_rp_3dconnexion_led_ex1 HID-RP 3Dconnexion SpaceMouse Example
/// \snippet hid_rp_example.cpp hid rp example
template <uint8_t REPORT_ID = 4>
class SpaceMouseLedOutputReport : public hid::report::base<hid::report::type::OUTPUT, REPORT_ID> {
protected:
  struct {
    std::uint8_t led : 1;
    std::uint8_t : 7;
  };

public:
  static constexpr std::size_t num_data_bytes = 1;

  /// Construct a new LED Output Report object
  constexpr SpaceMouseLedOutputReport() { reset(); }

  /// Turn the LED off
  constexpr void reset() { led = 0; }

  /// Set the LED state
  /// \param value True to turn the LED on, false to turn it off.
  constexpr void set_led(bool value) { led = value ? 1 : 0; }

  /// Get the LED state
  /// \return True if the LED is on, false otherwise.
  constexpr bool get_led() const { return led; }

  /// Get the output report as a vector of bytes
  /// \return The output report as a vector of bytes.
  /// \note The report id is not included in the returned vector.
  constexpr auto get_report() const {
    // the first byte is the id, which we don't want...
    size_t offset = 1;
    auto report_data = this->data() + offset;
    return std::vector<uint8_t>(report_data, report_data + num_data_bytes);
  }

  /// Set the output report data from a vector of bytes
  /// \param data The data to set the output report to.
  constexpr void set_data(const std::vector<uint8_t> &data) {
    // copy the data into our data array - skip the first byte, which is the
    // report id. Clamp the copy length to the report's payload size so an
    // over-long input cannot write past the backing storage.
    auto copy_size = std::min(data.size(), num_data_bytes);
    std::copy(data.begin(), data.begin() + copy_size, this->data() + 1);
  }

  /// Get the report descriptor as a hid::rdf::descriptor
  /// \return The report descriptor as a hid::rdf::descriptor.
  /// \note This is an incomplete descriptor, you will need to add it to a
  ///      collection::application descriptor to create a complete report descriptor.
  ///      See espp::spacemouse_descriptor() for a complete example.
  static constexpr auto get_descriptor() {
    using namespace hid::page;
    using namespace hid::rdf;

    // clang-format off
      return collection::logical(
                        conditional_report_id<REPORT_ID>(),
                        usage_page<leds>(),
                        usage(leds::GENERIC_INDICATOR),
                        logical_limits<1, 1>(0, 1),
                        report_count(1),
                        report_size(1),
                        output::absolute_variable(),
                        output::byte_padding<1>()
                        );
    // clang-format on
  }
};

/// Get the complete report descriptor for a 3Dconnexion SpaceMouse.
/// \tparam BUTTON_COUNT The number of buttons to report (2 for a
///         SpaceNavigator, more for other models in the family).
/// \return The complete report descriptor for a 3Dconnexion SpaceMouse,
///         combining the translation (Report ID 1), rotation (Report ID 2),
///         buttons (Report ID 3), and LED (Report ID 4) reports under the
///         standard Generic Desktop / Multi-Axis Controller application
///         collection.
template <std::size_t BUTTON_COUNT = 2>
[[maybe_unused]] static constexpr auto spacemouse_descriptor() {
  using namespace hid::page;
  using namespace hid::rdf;

  auto translation_descriptor = SpaceMouseTranslationInputReport<>::get_descriptor();
  auto rotation_descriptor = SpaceMouseRotationInputReport<>::get_descriptor();
  auto buttons_descriptor = SpaceMouseButtonsInputReport<BUTTON_COUNT>::get_descriptor();
  auto led_descriptor = SpaceMouseLedOutputReport<>::get_descriptor();

  return descriptor(usage_page<generic_desktop>(), usage(generic_desktop::MULTI_AXIS_CONTROLLER),
                    collection::application(translation_descriptor, rotation_descriptor,
                                            buttons_descriptor, led_descriptor));
} // spacemouse_descriptor

} // namespace espp

#include "hid-rp-3dconnexion-formatters.hpp"

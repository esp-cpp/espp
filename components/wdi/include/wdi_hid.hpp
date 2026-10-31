#pragma once

// WDI HID report descriptor, built with the espp `hid-rp` component.
//
// The five WDI reports live on the vendor usage page 0xFF00 ("Wheelchair Control
// Device"). Unlike an opaque byte-blob descriptor, this describes each report's
// real fields so a generic HID host can introspect them: the Control report as
// two signed-8-bit axes plus four 32-bit flag fields, the Feedback report as
// three 32-bit flag fields plus the packed speed/profile, velocity and odometer
// bytes, etc. The field decomposition is tied to the protocol core's report
// sizes (kControlSize, ...) with static_asserts below so the descriptor and the
// serialize()/parse() packing in detail/wdi_protocol.hpp cannot silently drift.
//
// The descriptor is used by BOTH transports: the USB HID interface embeds it in
// the configuration descriptor, and the BLE profile serves the identical bytes
// through its HID-over-GATT Report Map characteristic (10A50002).
//
// hid-rp is header-only and standard-library-only, so this is still host-testable
// (see test/wdi_hid_host_test.cpp).

#include <algorithm>
#include <cstdint>
#include <span>

#include "hid-rp.hpp"

#include "detail/wdi_protocol.hpp"

namespace espp {
namespace wdi {
/// @brief The WDI vendor HID usage page (0xFF00, "Wheelchair Control Device").
enum class hid_page : std::uint16_t;

/// @brief Vendor usage ids (on page 0xFF00) for the WDI report fields. The values
///        are arbitrary within the vendor page; they exist so the descriptor
///        names each field distinctly.
enum class HidUsage : std::uint8_t {
  WheelchairControlDevice = 0x01, ///< application collection usage
  // Control (0x01) fields
  AxisX = 0x30,     ///< lateral SInt8
  AxisY = 0x31,     ///< longitudinal SInt8
  Standard1 = 0x40, ///< Control standard1 u32
  Standard2 = 0x41, ///< Control standard2 u32
  Vendor1 = 0x42,   ///< Control vendor1 u32
  Vendor2 = 0x43,   ///< Control vendor2 u32
  // Feedback (0x02) fields
  FbStandard = 0x50,     ///< Feedback standard u32
  FbVendor1 = 0x51,      ///< Feedback vendor1 u32
  FbVendor2 = 0x52,      ///< Feedback vendor2 u32
  FbSpeedProfile = 0x53, ///< packed speed/profile u8
  FbVelocity = 0x54,     ///< packed velocity u8
  FbOdometer = 0x55,     ///< odometer u8
  FbReserved = 0x56,     ///< reserved u8[4]
  // Trigger / identity reports
  RequestFeedback = 0x60,  ///< Request-Feedback trigger u8
  Keepalive = 0x61,        ///< Keepalive trigger u8
  KeepaliveResponse = 0x62 ///< Host UUID u8[16]
};
} // namespace wdi
} // namespace espp

// Register the vendor page with hid-rp (page id 0xFF00), the same way the espp
// switch-pro descriptor registers its vendor page.
namespace hid {
namespace page {
template <> struct info<espp::wdi::hid_page> {
  constexpr static page_id_t page_id = 0xFF00;
  constexpr static usage_id_t max_usage_id = 0xFFFF;
  constexpr static const char *name = "WDI";
};
} // namespace page
} // namespace hid

namespace espp {
namespace wdi {
namespace detail {
// A raw vendor usage on page 0xFF00 (the typed usage() helper needs a page-typed
// usage; short_item emits `Usage(id)` directly, as the switch-pro descriptor
// does). The usage id must be a constant expression, so it is a template arg.
template <HidUsage U> constexpr auto usage() {
  return hid::rdf::short_item<1>(hid::rdf::local::tag::USAGE, static_cast<uint8_t>(U));
}

// One 32-bit WDI flag field, exposed as 32 individual bits (report_size 1 x 32)
// so a host sees the bitfield. `Output` selects host->device vs device->host.
template <bool Output, HidUsage U> constexpr auto flag_u32() {
  using namespace hid::rdf;
  if constexpr (Output)
    return descriptor(usage<U>(), report_count(32), output::absolute_variable());
  else
    return descriptor(usage<U>(), report_count(32), input::absolute_variable());
}

// One or more 8-bit byte fields. `Output` selects the direction.
template <bool Output, HidUsage U, uint16_t Count> constexpr auto bytes_u8() {
  using namespace hid::rdf;
  if constexpr (Output)
    return descriptor(usage<U>(), report_count(Count), output::absolute_variable());
  else
    return descriptor(usage<U>(), report_count(Count), input::absolute_variable());
}

// --- Control (0x01, Input): 2x SInt8 axes + 4x UInt32 flag fields = 18 bytes ---
constexpr auto control_report() {
  using namespace hid::rdf;
  return descriptor(report_id(static_cast<uint8_t>(ReportId::Control)),
                    // two signed-8-bit axes (X, Y)
                    usage<HidUsage::AxisX>(), usage<HidUsage::AxisY>(),
                    logical_limits<1, 1>(-127, 127), report_size(8), report_count(2),
                    input::absolute_variable(),
                    // four 32-bit flag fields (bit granularity)
                    logical_limits<1, 1>(0, 1), report_size(1),
                    flag_u32<false, HidUsage::Standard1>(), flag_u32<false, HidUsage::Standard2>(),
                    flag_u32<false, HidUsage::Vendor1>(), flag_u32<false, HidUsage::Vendor2>());
}

// --- Feedback (0x02, Output): 3x UInt32 + 3x UInt8 + 4x UInt8 reserved = 19 B ---
constexpr auto feedback_report() {
  using namespace hid::rdf;
  return descriptor(
      report_id(static_cast<uint8_t>(ReportId::Feedback)),
      // three 32-bit flag fields
      logical_limits<1, 1>(0, 1), report_size(1), flag_u32<true, HidUsage::FbStandard>(),
      flag_u32<true, HidUsage::FbVendor1>(), flag_u32<true, HidUsage::FbVendor2>(),
      // packed speed/profile, velocity, odometer, then 4 reserved bytes
      logical_limits<1, 2>(0, 255), report_size(8), bytes_u8<true, HidUsage::FbSpeedProfile, 1>(),
      bytes_u8<true, HidUsage::FbVelocity, 1>(), bytes_u8<true, HidUsage::FbOdometer, 1>(),
      bytes_u8<true, HidUsage::FbReserved, 4>());
}

// --- trigger / identity reports ---
constexpr auto request_feedback_report() {
  using namespace hid::rdf;
  return descriptor(report_id(static_cast<uint8_t>(ReportId::RequestFeedback)),
                    logical_limits<1, 2>(0, 255), report_size(8),
                    bytes_u8<false, HidUsage::RequestFeedback, kRequestFeedbackSize>());
}
constexpr auto keepalive_report() {
  using namespace hid::rdf;
  return descriptor(report_id(static_cast<uint8_t>(ReportId::Keepalive)),
                    logical_limits<1, 2>(0, 255), report_size(8),
                    bytes_u8<false, HidUsage::Keepalive, kKeepaliveSize>());
}
constexpr auto keepalive_response_report() {
  using namespace hid::rdf;
  return descriptor(report_id(static_cast<uint8_t>(ReportId::KeepaliveResponse)),
                    logical_limits<1, 2>(0, 255), report_size(8),
                    bytes_u8<true, HidUsage::KeepaliveResponse, kKeepaliveResponseSize>());
}

// Guard against the descriptor's field decomposition drifting from the protocol
// core's report sizes (detail/wdi_protocol.hpp). The byte totals must match.
static_assert(2 * 1 + 4 * 4 == kControlSize, "Control descriptor fields != kControlSize");
static_assert(3 * 4 + 3 * 1 + 4 * 1 == kFeedbackSize,
              "Feedback descriptor fields != kFeedbackSize");
static_assert(kRequestFeedbackSize == 1 && kKeepaliveSize == 1, "trigger report size changed");
static_assert(kKeepaliveResponseSize == 16, "Keepalive-Response size changed");
} // namespace detail

/// @brief Build the WDI HID report descriptor (usage page 0xFF00) with hid-rp.
inline constexpr auto make_hid_report_descriptor() {
  using namespace hid::rdf;
  return descriptor(usage_page<hid_page>(), detail::usage<HidUsage::WheelchairControlDevice>(),
                    collection::application(detail::control_report(), detail::feedback_report(),
                                            detail::request_feedback_report(),
                                            detail::keepalive_report(),
                                            detail::keepalive_response_report()));
}

/// @brief The WDI HID report descriptor bytes (a std::array), ready to hand to
///        espp::UsbDevice's HID function or a BLE HID Report Map characteristic.
inline constexpr auto kReportDescriptor = make_hid_report_descriptor();

/// @brief Does a HID report descriptor describe a WDI device?
///
/// True for an exact match against kReportDescriptor, or -- for another
/// implementation of the spec -- for a descriptor that declares the WDI vendor
/// usage page (0xFF00) immediately followed by usage 0x01 (Wheelchair Control
/// Device) and report ids 1..5. This is what a WDI host uses to decide which
/// HID device to adopt, so it walks the descriptor's items properly rather than
/// byte-scanning: item *data* (e.g. a Logical Maximum of 0x00FF0006) cannot
/// masquerade as a Usage Page item, long items (prefix 0xFE) are skipped, and a
/// truncated/malformed descriptor is rejected.
/// @param d The report descriptor bytes.
/// @return true if it looks like a WDI descriptor.
constexpr bool looks_like_wdi_descriptor(std::span<const uint8_t> d) {
  if (d.size() == kReportDescriptor.size() &&
      std::equal(d.begin(), d.end(), kReportDescriptor.begin()))
    return true;
  bool vendor_usage = false; // saw Usage Page 0xFF00 immediately followed by Usage 0x01
  uint8_t report_ids = 0;    // bit i-1 set when Report ID i (1..5) was seen
  bool prev_was_wdi_page = false;
  for (size_t i = 0; i < d.size();) {
    const uint8_t prefix = d[i];
    if (prefix == 0xFE) {
      // Long item: [0xFE][bDataSize][bLongItemTag][data...]. Valid HID (no
      // long items are defined today) -- skip it, but reject a truncated one.
      if (i + 2 >= d.size())
        return false;
      const size_t data_size = d[i + 1];
      if (i + 3 + data_size > d.size())
        return false;
      i += 3 + data_size;
      prev_was_wdi_page = false;
      continue;
    }
    const uint8_t size_code = prefix & 0x03;
    const size_t size = size_code == 3 ? 4 : size_code;
    if (i + 1 + size > d.size())
      return false; // malformed / truncated short item
    const uint8_t tag_type = prefix & 0xFC;
    const uint8_t *data = &d[i + 1];
    if (tag_type == 0x04 && size == 2 && data[0] == 0x00 && data[1] == 0xFF) {
      prev_was_wdi_page = true; // Global: Usage Page 0xFF00
    } else {
      if (tag_type == 0x08 && size == 1 && data[0] == 0x01 && prev_was_wdi_page)
        vendor_usage = true; // Local: Usage 0x01 (Wheelchair Control Device)
      prev_was_wdi_page = false;
    }
    if (tag_type == 0x84 && size == 1 && data[0] >= 1 && data[0] <= 5) // Global: Report ID
      report_ids |= static_cast<uint8_t>(1u << (data[0] - 1));
    i += 1 + size;
  }
  return vendor_usage && report_ids == 0x1F;
}

} // namespace wdi
} // namespace espp

#pragma once

// WDI HID report descriptor, built with the espp `hid-rp` component.
//
// The five WDI reports are vendor-defined opaque byte arrays on usage page
// 0xFF00 (Wheelchair Control Device), so this declares a custom hid-rp usage
// page and emits one report item per report id. Kept separate from the
// dependency-free protocol core (detail/wdi_protocol.hpp): only the USB HID
// transport needs a report descriptor (BLE carries the same reports as GATT
// characteristics), so only the USB binding pulls in hid-rp.
//
// hid-rp is header-only and standard-library-only, so this is still host-testable
// (see test/wdi_hid_host_test.cpp).

#include <cstdint>

#include "hid-rp.hpp"

#include "detail/wdi_protocol.hpp"

namespace espp {
namespace wdi {
/// @brief The WDI vendor HID usage page (0xFF00, "Wheelchair Control Device").
enum class hid_page : std::uint16_t;
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
// One WDI report: a report id + a raw vendor usage + `count` opaque 8-bit bytes,
// as an INPUT (device→host) or OUTPUT (host→device) item. report_size / logical
// limits are inherited from the enclosing application collection.
template <uint8_t ReportIdV, uint8_t UsageV, uint16_t Count, bool Output>
constexpr auto wdi_report_item() {
  using namespace hid::rdf;
  // A raw vendor usage (the typed usage() helper requires a page-typed usage);
  // short_item emits `Usage(UsageV)` directly, as the espp switch-pro descriptor
  // does for its vendor reports.
  const auto vendor_usage = short_item<1>(local::tag::USAGE, UsageV);
  if constexpr (Output)
    return descriptor(report_id(ReportIdV), vendor_usage, report_count(Count),
                      output::absolute_variable());
  else
    return descriptor(report_id(ReportIdV), vendor_usage, report_count(Count),
                      input::absolute_variable());
}
} // namespace detail

/// @brief Build the WDI HID report descriptor (usage page 0xFF00) with hid-rp.
inline constexpr auto make_hid_report_descriptor() {
  using namespace hid::rdf;
  return descriptor(usage_page<hid_page>(),
                    short_item<1>(local::tag::USAGE, 0x01), // Usage: Wheelchair Control Device
                    collection::application(
                        logical_limits<1, 2>(0, 255), // opaque bytes: 0..255
                        report_size(8),
                        // app -> host (Input) and host -> app (Output) reports:
                        detail::wdi_report_item<static_cast<uint8_t>(ReportId::Control), 0x01,
                                                kControlSize, false>(),
                        detail::wdi_report_item<static_cast<uint8_t>(ReportId::Feedback), 0x02,
                                                kFeedbackSize, true>(),
                        detail::wdi_report_item<static_cast<uint8_t>(ReportId::RequestFeedback),
                                                0x03, kRequestFeedbackSize, false>(),
                        detail::wdi_report_item<static_cast<uint8_t>(ReportId::Keepalive), 0x04,
                                                kKeepaliveSize, false>(),
                        detail::wdi_report_item<static_cast<uint8_t>(ReportId::KeepaliveResponse),
                                                0x05, kKeepaliveResponseSize, true>()));
}

/// @brief The WDI HID report descriptor bytes (a std::array), ready to hand to
///        espp::UsbDevice's HID function.
inline constexpr auto kReportDescriptor = make_hid_report_descriptor();

} // namespace wdi
} // namespace espp

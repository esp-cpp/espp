#pragma once

#include <algorithm>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// generated from intergatedcircuits/hid-usage-tables
#include "hid/page/battery_system.hpp"
#include "hid/page/button.hpp"
#include "hid/page/consumer.hpp"
#include "hid/page/generic_desktop.hpp"
#include "hid/page/generic_device.hpp"
#include "hid/page/leds.hpp"
#include "hid/page/physical_input_device.hpp"
#include "hid/page/simulation.hpp"

// from intergatedcircuits/hid-rp library
#include "hid/rdf/constants.hpp"
#include "hid/rdf/descriptor.hpp"
#include "hid/rdf/unit.hpp"
#include "hid/report.hpp"
#include "hid/report_bitset.hpp"
#include "hid/report_protocol.hpp"

namespace espp {
constexpr int num_bits(std::size_t x) {
  if (x == 0) {
    return 1;
  }

  int num_bits = 0;
  while (x > 0) {
    x >>= 1;
    num_bits++;
  }
  return num_bits;
}
namespace detail {
/// Copy a report payload received from the wire into `dest`, which holds
/// `dest_len` bytes: at most `dest_len` bytes are copied, so an over-long
/// vector can never write past the payload, and the bytes a short vector does
/// not cover are zeroed, so the report is fully determined by the last write
/// and get_report() is deterministic. Every hid-rp report's set_data() uses
/// this, so the clamp and the zero-fill cannot drift apart between reports.
/// \param dest First payload byte of the report object.
/// \param dest_len Payload size in bytes.
/// \param src The bytes received (any length).
constexpr void copy_report_payload(uint8_t *dest, std::size_t dest_len,
                                   const std::vector<uint8_t> &src) {
  const auto n = std::min(src.size(), dest_len);
  std::copy(src.begin(), src.begin() + static_cast<std::ptrdiff_t>(n), dest);
  std::fill(dest + n, dest + dest_len, uint8_t{0});
}
} // namespace detail
} // namespace espp

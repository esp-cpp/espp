#pragma once

#include <bitset>
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
} // namespace espp

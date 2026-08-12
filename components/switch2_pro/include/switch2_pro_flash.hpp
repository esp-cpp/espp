#pragma once

#include <array>
#include <cstdint>
#include <cstring>

/// @file switch2_pro_flash.hpp
/// @brief Simulated controller flash the console reads during init (command
///        0x02 memory reads): device info and stick calibration.
///
/// The console reads calibration/device-info blocks from the controller's
/// internal flash during bring-up. We emulate that flash in RAM and answer the
/// reads. The exact factory-calibration contents are controller-specific; the
/// values here are structurally valid placeholders (neutral stick calibration
/// centered at the 12-bit midpoint) sufficient for bring-up. Refine against a
/// real controller capture for pixel-accurate stick calibration.

namespace espp::switch2 {

/// Reads `len` bytes from the simulated flash at `addr` into `out`. Unknown
/// regions read back as zero. Returns the number of bytes written (== len).
inline size_t simulated_flash_read(uint32_t addr, size_t len, uint8_t *out) {
  std::memset(out, 0, len);

  // Neutral stick calibration: center at 0x800 (12-bit midpoint), symmetric
  // +/- range. Packed as the console expects (3 bytes per two 12-bit values).
  // NOTE: placeholder — replace with captured factory calibration for exact
  // stick behavior on real hardware.
  static constexpr std::array<uint8_t, 9> kNeutralStickCal = {0x00, 0x08, 0x80, 0x00, 0x08,
                                                              0x80, 0x00, 0x08, 0x80};

  // Device-info region (~0x13000): serial/colors/etc. Left mostly zero; the
  // console tolerates zeros here for bring-up.
  switch (addr) {
  case 0x0130A8: // primary stick calibration
  case 0x0130E8: // secondary stick calibration
    std::memcpy(out, kNeutralStickCal.data(),
                len < kNeutralStickCal.size() ? len : kNeutralStickCal.size());
    break;
  default:
    break;
  }
  return len;
}

} // namespace espp::switch2

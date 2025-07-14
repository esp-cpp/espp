#pragma once

#include <chrono>

namespace espp {
/**
 * @brief Convert a decimal number to BCD
 * @param dec The decimal number to convert
 * @return The BCD representation of the decimal number
 */
static inline uint8_t decimal_to_bcd(uint8_t dec) { return ((dec / 10 * 16) + (dec % 10)); }

/**
 * @brief Convert a BCD number to decimal
 * @param bcd The BCD number to convert
 * @return The decimal representation of the BCD number
 */
static inline uint8_t bcd_to_decimal(uint8_t bcd) { return ((bcd / 16 * 10) + (bcd % 16)); }
} // namespace espp
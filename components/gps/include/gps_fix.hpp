#pragma once

#include <cstdint>

namespace espp {
/// A GNSS fix, assembled from the parsed NMEA sentences.
struct GpsFix {
  bool valid{false};         ///< Whether the receiver reports a valid fix
  double latitude{0};        ///< Latitude in decimal degrees (+N / -S)
  double longitude{0};       ///< Longitude in decimal degrees (+E / -W)
  float altitude{0};         ///< Altitude above mean sea level, in meters
  float speed_knots{0};      ///< Speed over ground, in knots
  float course_degrees{0};   ///< Course over ground, in degrees true
  uint8_t num_satellites{0}; ///< Number of satellites used in the fix
  float hdop{0};             ///< Horizontal dilution of precision
  uint8_t fix_quality{0};    ///< GGA fix quality (0 = none, 1 = GPS, 2 = DGPS, ...)
  // UTC date / time of the fix
  uint16_t year{0};  ///< UTC year (e.g. 2026); 0 if no date received yet
  uint8_t month{0};  ///< UTC month (1-12)
  uint8_t day{0};    ///< UTC day of month (1-31)
  uint8_t hour{0};   ///< UTC hour (0-23)
  uint8_t minute{0}; ///< UTC minute (0-59)
  float second{0};   ///< UTC second, including fractional part

  /// Get the speed over ground in meters per second
  /// \return The speed in m/s
  float speed_mps() const { return speed_knots * 0.514444f; }

  /// Get the UTC time of the fix as a unix timestamp (seconds since epoch).
  /// \return The unix timestamp, or 0 if no date/time has been received
  uint64_t unix_timestamp() const {
    if (year < 2000) {
      return 0;
    }
    // days since epoch using the standard civil-date algorithm
    int y = year;
    int m = month;
    int d = day;
    y -= m <= 2;
    int era = y / 400;
    int yoe = y - era * 400;
    int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    int64_t days = era * 146097LL + doe - 719468LL;
    return (uint64_t)(days * 86400LL + hour * 3600LL + minute * 60LL + (int)second);
  }
};
} // namespace espp

#pragma once

#include <string_view>

#include "gps_fix.hpp"

namespace espp {
/// Incremental parser for NMEA-0183 sentences from a GNSS receiver.
///
/// Feed it complete sentences (with or without the trailing CR/LF) via
/// parse(); it validates the checksum and updates the fix state from the
/// sentences it understands (xxRMC, xxGGA - any talker id: GP, GN, BD, ...).
/// This class performs no I/O and is usable on any platform; see espp::Gps
/// for a UART-attached wrapper.
class NmeaParser {
public:
  /// Parse a single NMEA sentence and update the fix state.
  /// \param sentence The sentence, from '$' through the checksum (trailing
  ///        CR/LF permitted)
  /// \return True if the sentence was valid (good checksum) and recognized
  bool parse(std::string_view sentence);

  /// Parse a single NMEA sentence whose checksum has ALREADY been validated,
  /// updating the fix state. This skips the checksum check, so callers on a
  /// hot path that have already gated on checksum_valid() can avoid computing
  /// the checksum a second time. Use parse() instead if you want the checksum
  /// validated for you.
  /// \param sentence The sentence, from '$' through the checksum (trailing
  ///        CR/LF permitted)
  /// \return True if the sentence was recognized (RMC/GGA) and parsed
  bool parse_unchecked(std::string_view sentence);

  /// Get the current fix state, assembled from all sentences parsed so far.
  /// \return The current fix
  const GpsFix &fix() const { return fix_; }

  /// Verify the checksum of an NMEA sentence.
  /// \param sentence The sentence, from '$' through the checksum
  /// \return True if the sentence has a valid checksum
  static bool checksum_valid(std::string_view sentence);

protected:
  bool parse_rmc(std::string_view body);
  bool parse_gga(std::string_view body);

  GpsFix fix_{};
};
} // namespace espp

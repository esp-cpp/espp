#pragma once

#include <cstdint>

namespace espp {

/// @brief Standard DiffServ code points (DSCP) for IP traffic marking.
///
/// A DSCP is the 6-bit field carried in the upper bits of the IP TOS /
/// Traffic Class byte (RFC 2474); network devices use it to prioritize,
/// queue, or drop traffic. The named values below are the IANA-registered
/// per-hop behaviors, so application code can write `Dscp::Ef` instead of a
/// magic number:
///
/// - **CS0..CS7** - class selectors (RFC 2474): backwards-compatible with the
///   legacy IP precedence bits. CS0 (0) is default/best-effort forwarding;
///   higher classes are conventionally more important (CS6/CS7 are typically
///   reserved for network control traffic).
/// - **AF11..AF43** - assured forwarding (RFC 2597): four classes (AF1x
///   lowest priority .. AF4x highest), each with three drop precedences
///   (x1 = lowest drop probability .. x3 = highest). E.g. AF41 = high
///   priority, low drop probability.
/// - **EF** - expedited forwarding (RFC 3246): the standard marking for
///   low-latency, low-jitter traffic (e.g. voice / control loops).
/// - **VoiceAdmit** - capacity-admitted EF traffic (RFC 5865).
/// - **LE** - lower effort (RFC 8622): scavenger-class traffic that should
///   yield to everything else (e.g. bulk background transfers).
///
/// Enumerator names are CamelCase (Ef, Cs5, Af41) rather than the all-caps
/// RFC names (EF, CS5, AF41) because platform headers define macros with the
/// all-caps names (e.g. newlib's <sys/termios.h> defines CS5/CS6/CS7), which
/// would break the enum wherever both headers are included.
///
/// A custom (non-standard) code point can still be expressed with
/// `static_cast<Dscp>(value)` for values 0-63; consumers reject out-of-range
/// values.
enum class Dscp : uint8_t {
  Cs0 = 0,         ///< Class selector 0 - default / best-effort forwarding.
  Default = Cs0,   ///< Alias for CS0.
  Le = 1,          ///< Lower effort / scavenger (RFC 8622).
  Cs1 = 8,         ///< Class selector 1 (conventionally low-priority data).
  Af11 = 10,       ///< Assured forwarding: class 1, low drop precedence.
  Af12 = 12,       ///< Assured forwarding: class 1, medium drop precedence.
  Af13 = 14,       ///< Assured forwarding: class 1, high drop precedence.
  Cs2 = 16,        ///< Class selector 2 (conventionally OAM / management).
  Af21 = 18,       ///< Assured forwarding: class 2, low drop precedence.
  Af22 = 20,       ///< Assured forwarding: class 2, medium drop precedence.
  Af23 = 22,       ///< Assured forwarding: class 2, high drop precedence.
  Cs3 = 24,        ///< Class selector 3 (conventionally call signaling).
  Af31 = 26,       ///< Assured forwarding: class 3, low drop precedence.
  Af32 = 28,       ///< Assured forwarding: class 3, medium drop precedence.
  Af33 = 30,       ///< Assured forwarding: class 3, high drop precedence.
  Cs4 = 32,        ///< Class selector 4 (conventionally real-time interactive).
  Af41 = 34,       ///< Assured forwarding: class 4, low drop precedence.
  Af42 = 36,       ///< Assured forwarding: class 4, medium drop precedence.
  Af43 = 38,       ///< Assured forwarding: class 4, high drop precedence.
  Cs5 = 40,        ///< Class selector 5 (conventionally broadcast video).
  VoiceAdmit = 44, ///< Capacity-admitted EF traffic (RFC 5865).
  Ef = 46,         ///< Expedited forwarding (RFC 3246) - low-latency/low-jitter.
  Cs6 = 48,        ///< Class selector 6 (network control - use with care).
  Cs7 = 56,        ///< Class selector 7 (reserved network control).
};

/// @brief Convert a DSCP code point to the IP TOS / Traffic Class byte value
///        that carries it (DSCP occupies the upper 6 bits - RFC 2474).
/// @param dscp The code point to convert.
/// @return The TOS byte value (dscp << 2), e.g. Dscp::Ef -> 184 (0xB8).
constexpr uint8_t dscp_to_tos(Dscp dscp) { return static_cast<uint8_t>(dscp) << 2; }

} // namespace espp

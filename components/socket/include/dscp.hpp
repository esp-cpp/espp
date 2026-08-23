#pragma once

#include <cstdint>

namespace espp {

/// @brief Standard DiffServ code points (DSCP) for IP traffic marking.
///
/// A DSCP is the 6-bit field carried in the upper bits of the IP TOS /
/// Traffic Class byte (RFC 2474); network devices use it to prioritize,
/// queue, or drop traffic. The named values below are the IANA-registered
/// per-hop behaviors, so application code can write `Dscp::EF` instead of a
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
/// A custom (non-standard) code point can still be expressed with
/// `static_cast<Dscp>(value)` for values 0-63; consumers reject out-of-range
/// values.
enum class Dscp : uint8_t {
  CS0 = 0,         ///< Class selector 0 - default / best-effort forwarding.
  Default = CS0,   ///< Alias for CS0.
  LE = 1,          ///< Lower effort / scavenger (RFC 8622).
  CS1 = 8,         ///< Class selector 1 (conventionally low-priority data).
  AF11 = 10,       ///< Assured forwarding: class 1, low drop precedence.
  AF12 = 12,       ///< Assured forwarding: class 1, medium drop precedence.
  AF13 = 14,       ///< Assured forwarding: class 1, high drop precedence.
  CS2 = 16,        ///< Class selector 2 (conventionally OAM / management).
  AF21 = 18,       ///< Assured forwarding: class 2, low drop precedence.
  AF22 = 20,       ///< Assured forwarding: class 2, medium drop precedence.
  AF23 = 22,       ///< Assured forwarding: class 2, high drop precedence.
  CS3 = 24,        ///< Class selector 3 (conventionally call signaling).
  AF31 = 26,       ///< Assured forwarding: class 3, low drop precedence.
  AF32 = 28,       ///< Assured forwarding: class 3, medium drop precedence.
  AF33 = 30,       ///< Assured forwarding: class 3, high drop precedence.
  CS4 = 32,        ///< Class selector 4 (conventionally real-time interactive).
  AF41 = 34,       ///< Assured forwarding: class 4, low drop precedence.
  AF42 = 36,       ///< Assured forwarding: class 4, medium drop precedence.
  AF43 = 38,       ///< Assured forwarding: class 4, high drop precedence.
  CS5 = 40,        ///< Class selector 5 (conventionally broadcast video).
  VoiceAdmit = 44, ///< Capacity-admitted EF traffic (RFC 5865).
  EF = 46,         ///< Expedited forwarding (RFC 3246) - low-latency/low-jitter.
  CS6 = 48,        ///< Class selector 6 (network control - use with care).
  CS7 = 56,        ///< Class selector 7 (reserved network control).
};

/// @brief Convert a DSCP code point to the IP TOS / Traffic Class byte value
///        that carries it (DSCP occupies the upper 6 bits - RFC 2474).
/// @param dscp The code point to convert.
/// @return The TOS byte value (dscp << 2), e.g. Dscp::EF -> 184 (0xB8).
constexpr uint8_t dscp_to_tos(Dscp dscp) { return static_cast<uint8_t>(dscp) << 2; }

} // namespace espp

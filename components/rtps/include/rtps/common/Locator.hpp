/*
The MIT License
Copyright (c) 2019 Lehrstuhl Informatik 11 - RWTH Aachen University
Modifications Copyright (c) 2026 ATDev
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE

This file is part of embeddedRTPS.

Author: i11 - Embedded Software, RWTH Aachen University
*/

#ifndef RTPS_LOCATOR_T_H
#define RTPS_LOCATOR_T_H

#include "rtps/common/types.hpp"
#include "rtps/utils/CdrBuffer.hpp"
#include "rtps/utils/udpUtils.hpp"

#include <array>
#include <cstring>

namespace rtps {

#if defined(_MSC_VER)
#define RTPS_EMBEDDED_PACKED
#pragma pack(push, 1)
#else
#define RTPS_EMBEDDED_PACKED __attribute__((packed))
#endif

/// Prefix length (CIDR bits) assumed for the local network when none is
/// known: a /24, the historical behaviour of every subnet check below.
constexpr uint8_t DEFAULT_SUBNET_PREFIX_LENGTH = 24;

/// Whether `addr` is on the same IPv4 network as `local`, for a network of
/// `prefixLength` bits (0..32; larger values are treated as 32). A prefix of 0
/// puts every address on the local network.
inline bool isSameSubnetAddress(const std::array<uint8_t, 4> &addr,
                                const std::array<uint8_t, 4> &local,
                                uint8_t prefixLength = DEFAULT_SUBNET_PREFIX_LENGTH) {
  if (prefixLength > 32) {
    prefixLength = 32;
  }
  const uint32_t mask = prefixLength == 0 ? 0u : (0xFFFFFFFFu << (32u - prefixLength));
  const auto toU32 = [](const std::array<uint8_t, 4> &ip) {
    return (static_cast<uint32_t>(ip[0]) << 24) | (static_cast<uint32_t>(ip[1]) << 16) |
           (static_cast<uint32_t>(ip[2]) << 8) | static_cast<uint32_t>(ip[3]);
  };
  return (toU32(addr) & mask) == (toU32(local) & mask);
}

/// Prefix length of a dotted-quad netmask (255.255.128.0 -> 17). A mask whose
/// set bits are not contiguous (e.g. 255.0.255.0) is not a valid netmask and
/// yields DEFAULT_SUBNET_PREFIX_LENGTH.
inline uint8_t prefixLengthFromNetmask(const std::array<uint8_t, 4> &netmask) {
  const uint32_t mask =
      (static_cast<uint32_t>(netmask[0]) << 24) | (static_cast<uint32_t>(netmask[1]) << 16) |
      (static_cast<uint32_t>(netmask[2]) << 8) | static_cast<uint32_t>(netmask[3]);
  uint8_t prefixLength = 0;
  while (prefixLength < 32 && (mask & (0x80000000u >> prefixLength)) != 0) {
    ++prefixLength;
  }
  // every bit after the first clear one must be clear too
  const uint32_t expected = prefixLength == 0 ? 0u : (0xFFFFFFFFu << (32u - prefixLength));
  return mask == expected ? prefixLength : DEFAULT_SUBNET_PREFIX_LENGTH;
}

enum class LocatorKind_t : int32_t {
  LOCATOR_KIND_INVALID = -1,
  LOCATOR_KIND_RESERVED = 0,
  LOCATOR_KIND_UDPv4 = 1,
  LOCATOR_KIND_UDPv6 = 2,
  // Not in RTPS 2.5 (which defines the two above); the values FastDDS uses for
  // its other transports. A FastDDS peer lists its shared-memory locator in
  // every announcement, so they are recognised, if only to be skipped quietly.
  LOCATOR_KIND_TCPv4 = 4,
  LOCATOR_KIND_TCPv6 = 8,
  LOCATOR_KIND_SHM = 16
};

/// Name of a locator kind, for logs ("unknown" for a value not listed above).
constexpr const char *locatorKindName(LocatorKind_t kind) {
  switch (kind) {
  case LocatorKind_t::LOCATOR_KIND_INVALID:
    return "invalid";
  case LocatorKind_t::LOCATOR_KIND_RESERVED:
    return "reserved";
  case LocatorKind_t::LOCATOR_KIND_UDPv4:
    return "UDPv4";
  case LocatorKind_t::LOCATOR_KIND_UDPv6:
    return "UDPv6";
  case LocatorKind_t::LOCATOR_KIND_TCPv4:
    return "TCPv4";
  case LocatorKind_t::LOCATOR_KIND_TCPv6:
    return "TCPv6";
  case LocatorKind_t::LOCATOR_KIND_SHM:
    return "SHM";
  }
  return "unknown";
}

const uint32_t LOCATOR_PORT_INVALID = 0;
const std::array<uint8_t, 16> LOCATOR_ADDRESS_INVALID = {0, 0, 0, 0, 0, 0, 0, 0,
                                                         0, 0, 0, 0, 0, 0, 0, 0};

/*
 * This representation corresponds to the RTPS wire format
 */
struct FullLengthLocator {
  LocatorKind_t kind = LocatorKind_t::LOCATOR_KIND_INVALID;
  uint32_t port = LOCATOR_PORT_INVALID;
  std::array<uint8_t, 16> address = LOCATOR_ADDRESS_INVALID; // TODO make private such that kind and
                                                             // address always match?

  static FullLengthLocator createUDPv4Locator(uint8_t a, uint8_t b, uint8_t c, uint8_t d,
                                              uint32_t port) {
    FullLengthLocator locator;
    locator.kind = LocatorKind_t::LOCATOR_KIND_UDPv4;
    locator.address = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, a, b, c, d};
    locator.port = port;
    return locator;
  }

  void setInvalid() { kind = LocatorKind_t::LOCATOR_KIND_INVALID; }

  bool isValid() const { return kind != LocatorKind_t::LOCATOR_KIND_INVALID; }

  /// Reads the locator as the raw 24-byte RTPS wire representation
  /// (kind int32 LE + port uint32 LE + 16 address bytes), matching this
  /// struct's packed layout.
  bool readFromBuffer(CdrReader &reader) {
    if (reader.remaining() < sizeof(FullLengthLocator)) {
      return false;
    }
    return readBytes(reader, reinterpret_cast<uint8_t *>(this), sizeof(FullLengthLocator));
  }

  std::array<uint8_t, 4> getIp4Address() const { return getIp4AddressBytes(); }

  std::array<uint8_t, 4> getIp4AddressBytes() const {
    return {address[12], address[13], address[14], address[15]};
  }

  bool isSameAddress(const std::array<uint8_t, 4> &ipAddress) const {
    return getIp4AddressBytes() == ipAddress;
  }

  inline bool isSameSubnet(const std::array<uint8_t, 4> &localIp,
                           uint8_t prefixLength = DEFAULT_SUBNET_PREFIX_LENGTH) const {
    return isSameSubnetAddress(getIp4AddressBytes(), localIp, prefixLength);
  }

  inline bool isMulticastAddress() const {
    const auto ip = getIp4AddressBytes();
    return ip[0] >= 224 && ip[0] <= 239;
  }

  inline uint32_t getLocatorPort() const { return static_cast<Ip4Port_t>(port); }

} RTPS_EMBEDDED_PACKED;

inline FullLengthLocator getBuiltInUnicastLocator(ParticipantId_t participantId,
                                                  const std::array<uint8_t, 4> &localIp) {
  return FullLengthLocator::createUDPv4Locator(localIp[0], localIp[1], localIp[2], localIp[3],
                                               getBuiltInUnicastPort(participantId));
}

inline FullLengthLocator getBuiltInMulticastLocator() {
  return FullLengthLocator::createUDPv4Locator(239, 255, 0, 1, getBuiltInMulticastPort());
}

inline FullLengthLocator getUserUnicastLocator(ParticipantId_t participantId,
                                               const std::array<uint8_t, 4> &localIp) {
  return FullLengthLocator::createUDPv4Locator(localIp[0], localIp[1], localIp[2], localIp[3],
                                               getUserUnicastPort(participantId));
}

inline FullLengthLocator
getUserMulticastLocator(const std::array<uint8_t, 4> &localIp) { // this would be a unicastaddress,
                                                                 // as defined in config
  return FullLengthLocator::createUDPv4Locator(localIp[0], localIp[1], localIp[2], localIp[3],
                                               getUserMulticastPort());
}

inline FullLengthLocator getDefaultSendMulticastLocator() {
  return FullLengthLocator::createUDPv4Locator(239, 255, 0, 1, getBuiltInMulticastPort());
}

/*
 * This representation omits unnecessary 12 bytes of the full RTPS wire format
 */
struct LocatorIPv4 {
  LocatorKind_t kind = LocatorKind_t::LOCATOR_KIND_INVALID;
  std::array<uint8_t, 4> address = {0};
  uint32_t port = LOCATOR_PORT_INVALID;

  LocatorIPv4() = default;
  explicit LocatorIPv4(const FullLengthLocator &locator) {
    address[0] = locator.address[12];
    address[1] = locator.address[13];
    address[2] = locator.address[14];
    address[3] = locator.address[15];
    port = locator.port;
    kind = locator.kind;
  }

  std::array<uint8_t, 4> getIp4Address() const { return getIp4AddressBytes(); }

  const std::array<uint8_t, 4> &getIp4AddressBytes() const { return address; }

  void setInvalid() { kind = LocatorKind_t::LOCATOR_KIND_INVALID; }

  bool isValid() const { return kind != LocatorKind_t::LOCATOR_KIND_INVALID; }

  inline bool isSameSubnet(const std::array<uint8_t, 4> &localIp,
                           uint8_t prefixLength = DEFAULT_SUBNET_PREFIX_LENGTH) const {
    return isSameSubnetAddress(getIp4AddressBytes(), localIp, prefixLength);
  }

  inline bool isMulticastAddress() const {
    const auto ip = getIp4AddressBytes();
    return ip[0] >= 224 && ip[0] <= 239;
  }
};

} // namespace rtps

#if defined(_MSC_VER)
#pragma pack(pop)
#endif
#undef RTPS_EMBEDDED_PACKED

#endif // RTPS_LOCATOR_T_H

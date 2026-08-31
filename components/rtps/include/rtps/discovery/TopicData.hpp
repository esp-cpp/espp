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

#ifndef RTPS_DISCOVEREDWRITERDATA_H
#define RTPS_DISCOVEREDWRITERDATA_H

#define SUPPRESS_UNICAST 0

#include "dscp.hpp"
#include "qos_band.hpp"
#include "rtps/config.hpp"
#include "rtps/utils/CdrBuffer.hpp"
#include "rtps/utils/hash.hpp"
#include <array>
#include <optional>
#include <rtps/common/Locator.hpp>
#include <span>

namespace rtps {

struct BuiltInTopicKey {
  std::array<uint32_t, 3> value;
};

struct TopicData {
  Guid_t endpointGuid;
  char typeName[Config::MAX_TYPENAME_LENGTH];
  char topicName[Config::MAX_TOPICNAME_LENGTH];
  ReliabilityKind_t reliabilityKind;
  DurabilityKind_t durabilityKind;
  FullLengthLocator unicastLocator;
  FullLengthLocator multicastLocator;

  // --- Local-only endpoint scheduling attributes ----------------------------
  // These are NEVER serialized to (or parsed from) the wire: serializeInto()
  // and readFromBuffer() ignore them, so the SEDP encoding is unchanged. They
  // only steer how the LOCAL endpoint's traffic is scheduled (see
  // Domain::createWriter/createReader and EsppTransport).
  //
  /// Priority band for this endpoint's received-traffic dispatch (and, when a
  /// dedicated port is granted, for that port's reactor registration).
  espp::QosBand band{espp::QosBand::Normal};
  /// Optional DSCP code point applied to the endpoint's dedicated socket (marks
  /// the traffic it SENDS; requires a dedicated port to take effect).
  std::optional<espp::Dscp> dscp{};
  /// True when Domain granted this (local) endpoint a dedicated unicast port:
  /// unicastLocator then carries that port instead of the participant's shared
  /// user-unicast port. Always false for remote endpoints.
  bool hasDedicatedPort{false};

  uint8_t statusInfo = 0;
  bool statusInfoValid = false;
  // Use Case: Remotes communicates id of deleted endpoint through key_hash
  // parameter
  EntityId_t entityIdFromKeyHash = ENTITYID_UNKNOWN;
  bool entityIdFromKeyHashValid = false;

  TopicData()
      : endpointGuid(GUID_UNKNOWN)
      , typeName{'\0'}
      , topicName{'\0'}
      , reliabilityKind(ReliabilityKind_t::BEST_EFFORT)
      , durabilityKind(DurabilityKind_t::VOLATILE) {
    rtps::FullLengthLocator someLocator =
        rtps::FullLengthLocator::createUDPv4Locator(192, 168, 0, 42, rtps::getUserUnicastPort(0));
    unicastLocator = someLocator;
    multicastLocator = FullLengthLocator();
  };

  TopicData(Guid_t guid, ReliabilityKind_t reliability, FullLengthLocator loc)
      : endpointGuid(guid)
      , typeName{'\0'}
      , topicName{'\0'}
      , reliabilityKind(reliability)
      , durabilityKind(DurabilityKind_t::VOLATILE)
      , unicastLocator(loc) {}

  bool matchesTopicOf(const TopicData &other);

  /// Parses the SEDP PL_CDR parameter list in `data` (little-endian; a
  /// leading PL_CDR_LE encapsulation header, if present, is skipped as an
  /// unknown zero-length parameter, matching the historical behavior).
  bool readFromBuffer(std::span<const uint8_t> data);
  /// Appends this endpoint's SEDP PL_CDR parameter list (including the
  /// terminating PID_SENTINEL) to `writer`.
  bool serializeInto(CdrWriter &writer) const;

  bool isDisposedFlagSet() const;
  bool isUnregisteredFlagSet() const;
};

struct TopicDataCompressed {
  Guid_t endpointGuid = GUID_UNKNOWN;
  std::size_t topicHash = 0;
  std::size_t typeHash = 0;
  bool is_reliable = false;
  LocatorIPv4 unicastLocator;
  LocatorIPv4 multicastLocator;

  TopicDataCompressed() = default;
  explicit TopicDataCompressed(const TopicData &topic_data)
      : endpointGuid(topic_data.endpointGuid)
      , topicHash(hashCharArray(topic_data.topicName, Config::MAX_TOPICNAME_LENGTH))
      , typeHash(hashCharArray(topic_data.typeName, Config::MAX_TYPENAME_LENGTH))
      , is_reliable(topic_data.reliabilityKind == ReliabilityKind_t::RELIABLE)
      , unicastLocator(topic_data.unicastLocator)
      , multicastLocator(topic_data.multicastLocator) {}

  bool matchesTopicOf(const TopicData &topic_data) const;
};
} // namespace rtps

#endif // RTPS_DISCOVEREDWRITERDATA_H

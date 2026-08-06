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

#ifndef RTPS_PARTICIPANTPROXYDATA_H
#define RTPS_PARTICIPANTPROXYDATA_H

#include "base_component.hpp"
#include "rtps/config.hpp"
#include "rtps/messages/MessageTypes.hpp"
#include "ucdr/microcdr.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <rtps/common/Locator.hpp>

namespace rtps {

class Participant;
using SMElement::ParameterId;

using BuiltinEndpointSet_t = uint32_t;

class ParticipantProxyData : public espp::BaseComponent {
public:
  ParticipantProxyData()
      : espp::BaseComponent("RtpsParticipantProxy", espp::Logger::Verbosity::WARN) {
    onAliveSignal();
  }
  explicit ParticipantProxyData(Guid_t guid);

  ProtocolVersion_t m_protocolVersion = PROTOCOLVERSION;
  Guid_t m_guid = Guid_t{GUIDPREFIX_UNKNOWN, ENTITYID_UNKNOWN};
  VendorId_t m_vendorId = VENDOR_UNKNOWN;
  bool m_expectsInlineQos = false;
  BuiltinEndpointSet_t m_availableBuiltInEndpoints{0};
  std::array<LocatorIPv4, Config::SPDP_MAX_NUM_LOCATORS> m_metatrafficUnicastLocatorList;
  std::array<LocatorIPv4, Config::SPDP_MAX_NUM_LOCATORS> m_metatrafficMulticastLocatorList;
  std::array<LocatorIPv4, Config::SPDP_MAX_NUM_LOCATORS> m_defaultUnicastLocatorList;
  std::array<LocatorIPv4, Config::SPDP_MAX_NUM_LOCATORS> m_defaultMulticastLocatorList;
  Count_t m_manualLivelinessCount{1};
  Duration_t m_leaseDuration = Config::SPDP_DEFAULT_REMOTE_LEASE_DURATION;
  std::chrono::time_point<std::chrono::steady_clock> m_lastLivelinessReceivedTimestamp;
  void reset();

  bool readFromUcdrBuffer(ucdrBuffer &buffer, Participant *participant);

  inline bool hasParticipantWriter() const;
  inline bool hasParticipantReader() const;
  inline bool hasPublicationWriter() const;
  inline bool hasPublicationReader() const;
  inline bool hasSubscriptionWriter() const;
  inline bool hasSubscriptionReader() const;

  inline void onAliveSignal();
  inline bool isAlive() const;
  inline uint32_t getAliveSignalAgeInMilliseconds() const;

private:
  bool readLocatorIntoList(ucdrBuffer &buffer,
                           std::array<LocatorIPv4, Config::SPDP_MAX_NUM_LOCATORS> &list);

  static const BuiltinEndpointSet_t DISC_BUILTIN_ENDPOINT_PARTICIPANT_ANNOUNCER = 1 << 0;
  static const BuiltinEndpointSet_t DISC_BUILTIN_ENDPOINT_PARTICIPANT_DETECTOR = 1 << 1;
  static const BuiltinEndpointSet_t DISC_BUILTIN_ENDPOINT_PUBLICATION_ANNOUNCER = 1 << 2;
  static const BuiltinEndpointSet_t DISC_BUILTIN_ENDPOINT_PUBLICATION_DETECTOR = 1 << 3;
  static const BuiltinEndpointSet_t DISC_BUILTIN_ENDPOINT_SUBSCRIPTION_ANNOUNCER = 1 << 4;
  static const BuiltinEndpointSet_t DISC_BUILTIN_ENDPOINT_SUBSCRIPTION_DETECTOR = 1 << 5;
  static const BuiltinEndpointSet_t DISC_BUILTIN_ENDPOINT_PARTICIPANT_PROXY_ANNOUNCER = 1 << 6;
  static const BuiltinEndpointSet_t DISC_BUILTIN_ENDPOINT_PARTICIPANT_PROXY_DETECTOR = 1 << 7;
  static const BuiltinEndpointSet_t DISC_BUILTIN_ENDPOINT_PARTICIPANT_STATE_ANNOUNCER = 1 << 8;
  static const BuiltinEndpointSet_t DISC_BUILTIN_ENDPOINT_PARTICIPANT_STATE_DETECTOR = 1 << 9;
  static const BuiltinEndpointSet_t BUILTIN_ENDPOINT_PARTICIPANT_MESSAGE_DATA_WRITER = 1 << 10;
  static const BuiltinEndpointSet_t BUILTIN_ENDPOINT_PARTICIPANT_MESSAGE_DATA_READER = 1 << 11;
};

// Needs to be in header because they are marked with inline
bool ParticipantProxyData::hasParticipantWriter() const {
  return (m_availableBuiltInEndpoints & DISC_BUILTIN_ENDPOINT_PARTICIPANT_ANNOUNCER) == 1;
}

bool ParticipantProxyData::hasParticipantReader() const {
  return (m_availableBuiltInEndpoints & DISC_BUILTIN_ENDPOINT_PARTICIPANT_DETECTOR) != 0;
}

bool ParticipantProxyData::hasPublicationWriter() const {
  return (m_availableBuiltInEndpoints & DISC_BUILTIN_ENDPOINT_PUBLICATION_ANNOUNCER) != 0;
}

bool ParticipantProxyData::hasPublicationReader() const {
  return (m_availableBuiltInEndpoints & DISC_BUILTIN_ENDPOINT_PUBLICATION_DETECTOR) != 0;
}

bool ParticipantProxyData::hasSubscriptionWriter() const {
  return (m_availableBuiltInEndpoints & DISC_BUILTIN_ENDPOINT_SUBSCRIPTION_ANNOUNCER) != 0;
}

bool ParticipantProxyData::hasSubscriptionReader() const {
  return (m_availableBuiltInEndpoints & DISC_BUILTIN_ENDPOINT_SUBSCRIPTION_DETECTOR) != 0;
}

void ParticipantProxyData::onAliveSignal() {
  m_lastLivelinessReceivedTimestamp = std::chrono::steady_clock::now();
}

uint32_t ParticipantProxyData::getAliveSignalAgeInMilliseconds() const {
  auto now = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - m_lastLivelinessReceivedTimestamp);
  return static_cast<uint32_t>(duration.count());
}

/*
 *  Returns true if last heartbeat within lease duration, else false
 */
bool ParticipantProxyData::isAlive() const {
  uint32_t lease_in_ms = m_leaseDuration.seconds * 1000 + m_leaseDuration.fraction * 1e-6;

  uint32_t max_lease_in_ms = Config::SPDP_MAX_REMOTE_LEASE_DURATION.seconds * 1000 +
                             Config::SPDP_MAX_REMOTE_LEASE_DURATION.fraction * 1e-6;

  auto heatbeat_age_in_ms = getAliveSignalAgeInMilliseconds();

  if (heatbeat_age_in_ms > std::min(lease_in_ms, max_lease_in_ms)) {
    return false;
  }
  return true;
}

} // namespace rtps
#endif // RTPS_PARTICIPANTPROXYDATA_H

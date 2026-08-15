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

#include "rtps/discovery/ParticipantProxyData.hpp"
#include "rtps/entities/Participant.hpp"
#include "rtps/utils/Log.hpp"

using rtps::ParticipantProxyData;

#if SPDP_VERBOSE && RTPS_GLOBAL_VERBOSE
#define PPD_LOG(...) logger_.warn(__VA_ARGS__)
#else
#define PPD_LOG(...)                                                                               \
  do {                                                                                             \
  } while (0)
#endif

ParticipantProxyData::ParticipantProxyData(Guid_t guid)
    : espp::BaseComponent("RtpsParticipantProxy", espp::Logger::Verbosity::WARN)
    , m_guid(guid) {}

void ParticipantProxyData::reset() {
  m_guid = Guid_t{GUIDPREFIX_UNKNOWN, ENTITYID_UNKNOWN};
  m_manualLivelinessCount = Count_t{1};
  m_expectsInlineQos = false;
  onAliveSignal();
  for (int i = 0; i < Config::SPDP_MAX_NUM_LOCATORS; ++i) {
    m_metatrafficUnicastLocatorList[i].setInvalid();
    m_metatrafficMulticastLocatorList[i].setInvalid();
    m_defaultUnicastLocatorList[i].setInvalid();
    m_defaultMulticastLocatorList[i].setInvalid();
  }
}

bool ParticipantProxyData::readFromBuffer(CdrReader &buffer, Participant *participant) {
  reset();
  PPD_LOG("Start deserializing ParticipantProxyData");
  PPD_LOG("Buffer has {} bytes remaining", buffer.remaining());
  while (buffer.remaining() >= 4) {
    const auto pidRaw = buffer.read<uint16_t>();
    const auto lengthRaw = buffer.read<uint16_t>();
    if (!pidRaw || !lengthRaw) {
      return false;
    }
    const auto pid = static_cast<SMElement::ParameterId>(*pidRaw);
    const uint16_t length = *lengthRaw;
    PPD_LOG("Deserializing parameter with id {} and length {}", static_cast<uint16_t>(pid), length);
    if (buffer.remaining() < length) {
      PPD_LOG("Not enough data left in buffer to read parameter with id {} and length {}",
              static_cast<uint16_t>(pid), length);
      return false;
    }

    switch (pid) {
    case ParameterId::PID_KEY_HASH: {
      // TODO
      break;
    }

    case ParameterId::PID_PROTOCOL_VERSION: {
      const auto major = buffer.read<uint8_t>();
      if (!major) {
        return false;
      }
      m_protocolVersion.major = *major;
      if (m_protocolVersion.major < PROTOCOLVERSION.major) {
        PPD_LOG("Unsupported protocol version: {}.{}", m_protocolVersion.major,
                m_protocolVersion.minor);
        return false;
      } else {
        const auto minor = buffer.read<uint8_t>();
        if (!minor) {
          return false;
        }
        m_protocolVersion.minor = *minor;
      }
      PPD_LOG("Protocol version: {}.{}", m_protocolVersion.major, m_protocolVersion.minor);
      break;
    }
    case ParameterId::PID_VENDORID: {
      if (!readBytes(buffer, m_vendorId.vendorId.data(), m_vendorId.vendorId.size())) {
        return false;
      }
      PPD_LOG("vendor id struct size: {}", m_vendorId.vendorId.size());
      PPD_LOG("Vendor ID: {} {}", m_vendorId.vendorId[0], m_vendorId.vendorId[1]);
      break;
    }

    case ParameterId::PID_EXPECTS_INLINE_QOS: {
      const auto flag = buffer.read<uint8_t>();
      if (!flag) {
        return false;
      }
      m_expectsInlineQos = (*flag != 0);
      break;
    }
    case ParameterId::PID_PARTICIPANT_GUID: {
      if (!readBytes(buffer, m_guid.prefix.id.data(), m_guid.prefix.id.size()) ||
          !readBytes(buffer, m_guid.entityId.entityKey.data(), m_guid.entityId.entityKey.size())) {
        return false;
      }
      const auto kind = buffer.read<uint8_t>();
      if (!kind) {
        return false;
      }
      m_guid.entityId.entityKind = static_cast<EntityKind_t>(*kind);
      if (participant->findRemoteParticipant(m_guid.prefix)) {
        PPD_LOG("stopping deserialization early, participant is known");
        return true;
      }
      PPD_LOG("Participant GUID: {} {} {} {} {} {} {} {} {} {}", m_guid.prefix.id[0],
              m_guid.prefix.id[1], m_guid.prefix.id[2], m_guid.prefix.id[3], m_guid.prefix.id[4],
              m_guid.prefix.id[5], m_guid.prefix.id[6], m_guid.prefix.id[7], m_guid.prefix.id[8],
              m_guid.prefix.id[9]);
      break;
    }
    case ParameterId::PID_METATRAFFIC_MULTICAST_LOCATOR: {
      if (!readLocatorIntoList(buffer, m_metatrafficMulticastLocatorList)) {
        PPD_LOG("Failed to read metatraffic multicast locator");
        return false;
      }
      break;
    }
    case ParameterId::PID_METATRAFFIC_UNICAST_LOCATOR: {
      if (!readLocatorIntoList(buffer, m_metatrafficUnicastLocatorList)) {
        PPD_LOG("Failed to read metatraffic unicast locator");
        return false;
      }
      break;
    }
    case ParameterId::PID_DEFAULT_UNICAST_LOCATOR: {
      if (!readLocatorIntoList(buffer, m_defaultUnicastLocatorList)) {
        PPD_LOG("Failed to read default unicast locator");
        return false;
      }
      break;
    }
    case ParameterId::PID_DEFAULT_MULTICAST_LOCATOR: {
      if (!readLocatorIntoList(buffer, m_defaultMulticastLocatorList)) {
        PPD_LOG("Failed to read default multicast locator");
        return false;
      }
      break;
    }
    case ParameterId::PID_PARTICIPANT_LEASE_DURATION: {
      const auto seconds = buffer.read<int32_t>();
      const auto fraction = buffer.read<uint32_t>();
      if (!seconds || !fraction) {
        return false;
      }
      m_leaseDuration.seconds = *seconds;
      m_leaseDuration.fraction = *fraction;
      break;
    }
    case ParameterId::PID_BUILTIN_ENDPOINT_SET: {
      const auto endpointSet = buffer.read<uint32_t>();
      if (!endpointSet) {
        return false;
      }
      m_availableBuiltInEndpoints = *endpointSet;
      break;
    }
    case ParameterId::PID_ENTITY_NAME: {
      // TODO
      skipBytes(buffer, length);
      break;
    }
    case ParameterId::PID_PROPERTY_LIST: {
      // TODO
      skipBytes(buffer, length);
      break;
    }
    case ParameterId::PID_USER_DATA: {
      // TODO
      skipBytes(buffer, length);
      break;
    }
    case ParameterId::PID_PAD: {
      skipBytes(buffer, length);
      break;
    }
    case ParameterId::PID_SENTINEL: {
      return true;
    }
    default: {
      // Should not return false for unknown parameters, just skip them,
      // otherwise we might miss some important information if the remote
      // participant is using some vendor specific parameters that we do not
      // know about.
      skipBytes(buffer, length);
      break;
    }
    }
    // Parameter lists are 4-byte aligned
    alignTo4(buffer);
  }
  return true;
}

bool ParticipantProxyData::readLocatorIntoList(
    CdrReader &buffer, std::array<LocatorIPv4, Config::SPDP_MAX_NUM_LOCATORS> &list) {
  int valid_locators = 0;
  FullLengthLocator full_length_locator;
  for (auto &proxy_locator : list) {
    if (!proxy_locator.isValid()) {
      bool ret = full_length_locator.readFromBuffer(buffer);
      if (ret && full_length_locator.kind == LocatorKind_t::LOCATOR_KIND_UDPv4) {
        proxy_locator = LocatorIPv4(full_length_locator);
        PPD_LOG("Adding locator: {} {} {} {}", (int)proxy_locator.address[0],
                (int)proxy_locator.address[1], (int)proxy_locator.address[2],
                (int)proxy_locator.address[3]);
        return true;
      } else {
        PPD_LOG("Ignoring locator: {} {} {} {}", (int)full_length_locator.address[12],
                (int)full_length_locator.address[13], (int)full_length_locator.address[14],
                (int)full_length_locator.address[15]);
        return true;
      }
    } else {
      valid_locators++;
      if (valid_locators == Config::SPDP_MAX_NUM_LOCATORS) {
        if (buffer.remaining() < sizeof(FullLengthLocator)) {
          PPD_LOG("Not enough data left in buffer to read locator");
          return false;
        }
        skipBytes(buffer, sizeof(FullLengthLocator));
        PPD_LOG("Max number of valid locators exceeded, ignoring this locator as we have at least "
                "one valid locator");
        return true;
      }
    }
  }
  return false;
}

#undef PPD_LOG

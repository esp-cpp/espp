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

#include "rtps/discovery/SPDPAgent.hpp"
#include "rtps/discovery/ParticipantProxyData.hpp"
#include "rtps/entities/Participant.hpp"
#include "rtps/entities/Reader.hpp"
#include "rtps/entities/Writer.hpp"
#include "rtps/messages/MessageTypes.hpp"
#include "rtps/utils/Log.hpp"
#include "rtps/utils/udpUtils.hpp"

#include <chrono>
#include <mutex>
#include <thread>

using rtps::SPDPAgent;
using rtps::SMElement::BuildInEndpointSet;
using rtps::SMElement::ParameterId;

SPDPAgent::SPDPAgent()
    : espp::BaseComponent("RtpsSPDP", espp::Logger::Verbosity::WARN) {}

void SPDPAgent::init(Participant &participant, BuiltInEndpoints &endpoints) {
  mp_participant = &participant;
  m_buildInEndpoints = endpoints;
  m_buildInEndpoints.spdpReader->registerCallback(receiveCallback, this);

  addParticipantParameters();
  initialized = true;
}

void SPDPAgent::start() { m_running = true; }

void SPDPAgent::stop() { m_running = false; }

void SPDPAgent::announce() {
  // Exactly one announcement cycle; the Domain's protocol scheduler provides
  // the SPDP_RESEND_PERIOD_MS cadence (the pacing loop + sleep used to live
  // here when this was a dedicated thread body).
  const DataSize_t size = static_cast<DataSize_t>(m_outputSize);
  const uint8_t *payload = m_outputBuffer.data();
  // StatelessWriter drops already-sent history; enqueue a fresh SPDP sample
  // for each announce cycle.
  m_buildInEndpoints.spdpWriter->newChange(ChangeKind_t::ALIVE, payload, size);
  if (m_cycleHB == Config::SPDP_CYCLECOUNT_HEARTBEAT) {
    m_cycleHB = 0;
    mp_participant->checkAndResetHeartbeats();
  } else {
    m_cycleHB++;
  }
}

void SPDPAgent::receiveCallback(void *callee, const ReaderCacheChange &cacheChange) {
  auto agent = static_cast<SPDPAgent *>(callee);
  agent->handleSPDPPackage(cacheChange);
}

void SPDPAgent::handleSPDPPackage(const ReaderCacheChange &cacheChange) {
  if (!initialized) {
    SPDP_LOG("Callback called without initialization");
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  if (cacheChange.size > m_inputBuffer.size()) {
    SPDP_LOG("Input buffer too small");
    return;
  }

  // Something went wrong deserializing remote participant
  if (!cacheChange.copyInto(m_inputBuffer.data(), m_inputBuffer.size())) {
    return;
  }
  SPDP_LOG("SPDPPackage size: {}", cacheChange.size);

  if (cacheChange.kind == ChangeKind_t::ALIVE) {
    // The payload's endianness is selected by the encapsulation identifier
    // (first two bytes); endianness doesn't matter for reading those since
    // they are single bytes.
    const std::endian endianness = (m_inputBuffer[0] == SMElement::SCHEME_PL_CDR_LE[0] &&
                                    m_inputBuffer[1] == SMElement::SCHEME_PL_CDR_LE[1])
                                       ? std::endian::little
                                       : std::endian::big;
    CdrReader buffer(asBytes(m_inputBuffer.data(), m_inputBuffer.size()), endianness);
    // Skip the encapsulation identifier and options (2 + 2 bytes)
    skipBytes(buffer, 4);
    volatile bool success = m_proxyDataBuffer.readFromBuffer(buffer, mp_participant);
    if (success) {
      // TODO In case we store the history we can free the history mutex here
      processProxyData();
    } else {
      SPDP_LOG("ParticipantProxyData deserialization failed");
    }
  } else {
    // TODO RemoveParticipant
  }
}

void SPDPAgent::processProxyData() {
  if (m_proxyDataBuffer.m_guid.prefix.id == mp_participant->m_guidPrefix.id) {
    return; // Our own packet
  }

  SPDP_LOG("Message from GUID = {} {} {} {}", m_proxyDataBuffer.m_guid.prefix.id[4],
           m_proxyDataBuffer.m_guid.prefix.id[5], m_proxyDataBuffer.m_guid.prefix.id[6],
           m_proxyDataBuffer.m_guid.prefix.id[7]);
  const rtps::ParticipantProxyData *remote_part;
  remote_part = mp_participant->findRemoteParticipant(m_proxyDataBuffer.m_guid.prefix);
  if (remote_part != nullptr) {
    SPDP_LOG("Not adding this participant");
    mp_participant->refreshRemoteParticipantLiveliness(m_proxyDataBuffer.m_guid.prefix);
    return; // Already in our list
  }

  if (mp_participant->addNewRemoteParticipant(m_proxyDataBuffer)) {
    addProxiesForBuiltInEndpoints();
    const DataSize_t size = static_cast<DataSize_t>(m_outputSize);
    m_buildInEndpoints.spdpWriter->newChange(ChangeKind_t::ALIVE, m_outputBuffer.data(), size);
#if SPDP_VERBOSE && RTPS_GLOBAL_VERBOSE
    SPDP_LOG("Added new participant with guid: ");
    printGuidPrefix(m_proxyDataBuffer.m_guid.prefix);
  } else {
    SPDP_LOG("Failed to add new participant");
  }
#else
  } else {
    while (1) {
      SPDP_LOG("failed to add remote participant");
    }
  }
#endif
}

bool SPDPAgent::addProxiesForBuiltInEndpoints() {

  LocatorIPv4 *locator = nullptr;

  // Check if the remote participants has a locator in our subnet
  for (unsigned int i = 0; i < m_proxyDataBuffer.m_metatrafficUnicastLocatorList.size(); i++) {
    LocatorIPv4 *l = &(m_proxyDataBuffer.m_metatrafficUnicastLocatorList[i]);
    if (l->isValid() && l->isSameSubnet(mp_participant->m_localIpAddress)) {
      locator = l;
      break;
    }
  }

  // Fallback: if subnet check fails or local netif is not fully configured yet,
  // still use any valid unicast locator so SEDP matching can proceed.
  if (!locator) {
    for (unsigned int i = 0; i < m_proxyDataBuffer.m_metatrafficUnicastLocatorList.size(); i++) {
      LocatorIPv4 *l = &(m_proxyDataBuffer.m_metatrafficUnicastLocatorList[i]);
      if (l->isValid()) {
        locator = l;
        break;
      }
    }
  }

  if (!locator) {
    return false;
  }

  if (m_proxyDataBuffer.hasPublicationReader()) {
    const ReaderProxy proxy{
        {m_proxyDataBuffer.m_guid.prefix, ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER},
        *locator,
        true};
    m_buildInEndpoints.sedpPubWriter->addNewMatchedReader(proxy);
  }

  if (m_proxyDataBuffer.hasSubscriptionReader()) {
    const ReaderProxy proxy{
        {m_proxyDataBuffer.m_guid.prefix, ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER},
        *locator,
        true};
    m_buildInEndpoints.sedpSubWriter->addNewMatchedReader(proxy);
  }

  if (m_proxyDataBuffer.hasPublicationWriter()) {
    const WriterProxy proxy{
        {m_proxyDataBuffer.m_guid.prefix, ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER},
        *locator,
        true};
    m_buildInEndpoints.sedpPubReader->addNewMatchedWriter(proxy);
    m_buildInEndpoints.sedpPubReader->sendPreemptiveAckNack(proxy);
  }

  if (m_proxyDataBuffer.hasSubscriptionWriter()) {
    const WriterProxy proxy{
        {m_proxyDataBuffer.m_guid.prefix, ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER},
        *locator,
        true};
    m_buildInEndpoints.sedpSubReader->addNewMatchedWriter(proxy);
    m_buildInEndpoints.sedpSubReader->sendPreemptiveAckNack(proxy);
  }

  return true;
}

void SPDPAgent::addInlineQos(CdrWriter &writer) {
  writer.write<uint16_t>(ParameterId::PID_KEY_HASH);
  writer.write<uint16_t>(16);
  writeBytes(writer, mp_participant->m_guidPrefix.id.data(), sizeof(GuidPrefix_t::id));
  writeBytes(writer, ENTITYID_BUILD_IN_PARTICIPANT.entityKey.data(), sizeof(EntityId_t::entityKey));
  writer.write<uint8_t>(static_cast<uint8_t>(ENTITYID_BUILD_IN_PARTICIPANT.entityKind));

  endCurrentList(writer);
}

void SPDPAgent::endCurrentList(CdrWriter &writer) {
  writer.write<uint16_t>(ParameterId::PID_SENTINEL);
  writer.write<uint16_t>(0);
}

void SPDPAgent::addParticipantParameters() {
  const uint16_t zero_options = 0;
  const uint16_t protocolVersionSize =
      sizeof(PROTOCOLVERSION.major) + sizeof(PROTOCOLVERSION.minor);
  const uint16_t vendorIdSize = Config::VENDOR_ID.vendorId.size();
  const uint16_t locatorSize = sizeof(FullLengthLocator);
  const uint16_t durationSize = sizeof(Duration_t::seconds) + sizeof(Duration_t::fraction);
  const uint16_t entityKeySize = 3;
  const uint16_t entityKindSize = 1;
  const uint16_t entityIdSize = entityKeySize + entityKindSize;
  const uint16_t guidSize = sizeof(GuidPrefix_t::id) + entityIdSize;

  const FullLengthLocator userUniCastLocator =
      getUserUnicastLocator(mp_participant->m_participantId, mp_participant->m_localIpAddress);
  const FullLengthLocator builtInUniCastLocator =
      getBuiltInUnicastLocator(mp_participant->m_participantId, mp_participant->m_localIpAddress);
  const FullLengthLocator builtInMultiCastLocator = getBuiltInMulticastLocator();

  CdrSink sink{asWritableBytes(m_outputBuffer.data(), m_outputBuffer.size())};
  CdrWriter writer(sink);

  writeBytes(writer, rtps::SMElement::SCHEME_PL_CDR_LE.data(),
             rtps::SMElement::SCHEME_PL_CDR_LE.size());
  writer.write<uint16_t>(zero_options);

  writer.write<uint16_t>(ParameterId::PID_PROTOCOL_VERSION);
  writer.write<uint16_t>(protocolVersionSize + 2);
  writer.write<uint8_t>(PROTOCOLVERSION.major);
  writer.write<uint8_t>(PROTOCOLVERSION.minor);
  writer.align(4); // 2 bytes of padding to 4 byte boundary

  writer.write<uint16_t>(ParameterId::PID_VENDORID);
  writer.write<uint16_t>(vendorIdSize + 2);
  writeBytes(writer, Config::VENDOR_ID.vendorId.data(), vendorIdSize);
  writer.align(4); // 2 bytes of padding to 4 byte boundary

  writer.write<uint16_t>(ParameterId::PID_DEFAULT_UNICAST_LOCATOR);
  writer.write<uint16_t>(locatorSize);
  writeBytes(writer, reinterpret_cast<const uint8_t *>(&userUniCastLocator), locatorSize);

  writer.write<uint16_t>(ParameterId::PID_METATRAFFIC_UNICAST_LOCATOR);
  writer.write<uint16_t>(locatorSize);
  writeBytes(writer, reinterpret_cast<const uint8_t *>(&builtInUniCastLocator), locatorSize);

  writer.write<uint16_t>(ParameterId::PID_METATRAFFIC_MULTICAST_LOCATOR);
  writer.write<uint16_t>(locatorSize);
  writeBytes(writer, reinterpret_cast<const uint8_t *>(&builtInMultiCastLocator), locatorSize);

  writer.write<uint16_t>(ParameterId::PID_PARTICIPANT_LEASE_DURATION);
  writer.write<uint16_t>(durationSize);
  writer.write<int32_t>(Config::SPDP_DEFAULT_REMOTE_LEASE_DURATION.seconds);
  writer.write<uint32_t>(Config::SPDP_DEFAULT_REMOTE_LEASE_DURATION.fraction);

  writer.write<uint16_t>(ParameterId::PID_PARTICIPANT_GUID);
  writer.write<uint16_t>(guidSize);
  writeBytes(writer, mp_participant->m_guidPrefix.id.data(), sizeof(GuidPrefix_t::id));
  writeBytes(writer, ENTITYID_BUILD_IN_PARTICIPANT.entityKey.data(), entityKeySize);
  writer.write<uint8_t>(static_cast<uint8_t>(ENTITYID_BUILD_IN_PARTICIPANT.entityKind));

  writer.write<uint16_t>(ParameterId::PID_BUILTIN_ENDPOINT_SET);
  writer.write<uint16_t>(sizeof(BuildInEndpointSet));
  writer.write<uint32_t>(BuildInEndpointSet::DISC_BIE_PARTICIPANT_ANNOUNCER |
                         BuildInEndpointSet::DISC_BIE_PARTICIPANT_DETECTOR |
                         BuildInEndpointSet::DISC_BIE_PUBLICATION_ANNOUNCER |
                         BuildInEndpointSet::DISC_BIE_PUBLICATION_DETECTOR |
                         BuildInEndpointSet::DISC_BIE_SUBSCRIPTION_ANNOUNCER |
                         BuildInEndpointSet::DISC_BIE_SUBSCRIPTION_DETECTOR);

  endCurrentList(writer);

  m_outputSize = sink.size();
}

#undef SPDP_VERBOSE

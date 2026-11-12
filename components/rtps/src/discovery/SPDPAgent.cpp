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
#include <cstring>
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
    // The peer disposed its participant instance (it is leaving the domain):
    // forget it now, with every proxy of its endpoints, instead of when its
    // lease runs out. The instance is named by the inline-QoS key hash; a
    // peer that leaves the hash out carries the GUID in the payload.
    // Only the peer's own SPDP writer may dispose it, and only itself: a
    // DATA from another writer, or naming another participant, is malformed
    // (or hostile) and must not remove anyone.
    if (!(cacheChange.writerGuid.entityId == ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER)) {
      return;
    }
    GuidPrefix_t prefix = GUIDPREFIX_UNKNOWN;
    if (cacheChange.hasKeyHash) {
      if (cacheChange.keyHash.entityId == ENTITYID_BUILD_IN_PARTICIPANT) {
        prefix = cacheChange.keyHash.prefix;
      }
    } else {
      prefix = participantPrefixInPayload(m_inputBuffer.data(), cacheChange.size);
    }
    if (prefix == GUIDPREFIX_UNKNOWN || prefix == mp_participant->m_guidPrefix) {
      return;
    }
    if (!(prefix == cacheChange.writerGuid.prefix)) {
      logger_.warn("Participant dispose from {:02x}{:02x}{:02x}{:02x} names another participant, "
                   "ignored",
                   cacheChange.writerGuid.prefix.id[0], cacheChange.writerGuid.prefix.id[1],
                   cacheChange.writerGuid.prefix.id[2], cacheChange.writerGuid.prefix.id[3]);
      return;
    }
    if (mp_participant->findRemoteParticipant(prefix) == nullptr) {
      return; // never known, or already gone
    }
    SPDP_LOG("Remote participant disposed, removing");
    // Lock order as on eviction (processProxyData): this agent's mutex, then
    // the SEDP agent's and the participant's inside removeRemoteParticipant.
    mp_participant->removeRemoteParticipant(prefix);
  }
}

rtps::GuidPrefix_t SPDPAgent::participantPrefixInPayload(const uint8_t *data, DataSize_t size) {
  // A PL_CDR_LE parameter list behind the 4-byte encapsulation header; only
  // PID_PARTICIPANT_GUID is of interest (the dispose carries little else).
  if (size < 4 ||
      !(data[0] == SMElement::SCHEME_PL_CDR_LE[0] && data[1] == SMElement::SCHEME_PL_CDR_LE[1])) {
    return GUIDPREFIX_UNKNOWN;
  }
  DataSize_t pos = 4;
  while (pos + 4 <= size) {
    uint16_t pid = 0;
    uint16_t length = 0;
    memcpy(&pid, data + pos, sizeof(pid));
    memcpy(&length, data + pos + 2, sizeof(length));
    pos += 4;
    if (pid == ParameterId::PID_SENTINEL) {
      break;
    }
    if (pos + length > size) {
      break;
    }
    if (pid == ParameterId::PID_PARTICIPANT_GUID && length == 16) {
      GuidPrefix_t prefix;
      memcpy(prefix.id.data(), data + pos, prefix.id.size());
      return prefix;
    }
    pos += length;
    pos += (4 - (pos % 4)) % 4; // parameters are 4-byte aligned
  }
  return GUIDPREFIX_UNKNOWN;
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

  // A newcomer without a usable metatraffic unicast locator could never be
  // matched (addProxiesForBuiltInEndpoints() refuses it below): check that
  // first, so such an announcement cannot evict a participant and spend the
  // eviction cooldown on its way to being forgotten anyway.
  bool usableLocator = false;
  for (unsigned int i = 0; i < m_proxyDataBuffer.m_metatrafficUnicastLocatorList.size(); i++) {
    if (m_proxyDataBuffer.m_metatrafficUnicastLocatorList[i].isValid()) {
      usableLocator = true;
      break;
    }
  }
  if (!usableLocator) {
    SPDP_LOG("No usable metatraffic unicast locator in this announcement, not adding it");
    return;
  }

  if (!mp_participant->addNewRemoteParticipant(m_proxyDataBuffer)) {
    // The remote-participant pool is full. Make room by forgetting the
    // participant we have the least use for (Config::SPDP_EVICT_UNUSED_PARTICIPANTS;
    // the builtin proxy pools below get the same treatment,
    // see Participant::evictUnusedRemoteParticipant; lock order here is the
    // SPDP agent mutex, held by handleSPDPPackage, -> SEDP agent -> Participant)
    // and retry. When nothing can be evicted (every known participant is one
    // we talk to, or one was evicted this announcement period already) the
    // peer announces again (SPDP is periodic), and the liveliness sweep frees
    // a slot once a known peer's lease runs out, so it gets in later; nothing
    // to do now. (This used to spin forever in non-verbose builds, with the
    // agent mutex held: one participant too many on the domain froze discovery.)
    GuidPrefix_t evicted{};
    if (!mp_participant->evictUnusedRemoteParticipant(evicted, "Remote-participant pool full") ||
        !mp_participant->addNewRemoteParticipant(m_proxyDataBuffer)) {
      SPDP_LOG("Remote-participant pool full, not adding this participant");
      return;
    }
  }

  if (!addProxiesForBuiltInEndpoints()) {
    // Its SEDP proxies do not fit the builtin endpoints' per-endpoint proxy
    // pools (or it has no usable locator). Those pools are usually the first
    // to fill on a busy domain (they are sized by the per-endpoint proxy
    // knobs, not by the participant pool), so make room the same way as for a
    // full participant pool: forget the participant we have the least use for
    // - which frees its builtin proxies too - and try once more. (The proxies
    // that did fit the first time were removed again by the failure.) When
    // nothing can be evicted, forget the newcomer: a participant left in the
    // list without its builtin proxies would never be matched, because every
    // later announcement only refreshes its lease. Forgotten, it is retried on
    // its next announcement, by which time the sweep or an eviction may have
    // freed a slot. Lock order here is the SPDP agent mutex (held by
    // handleSPDPPackage) -> SEDP agent mutex -> Participant mutex; nothing
    // takes the SPDP mutex while holding either of the others.
    GuidPrefix_t evicted{};
    const bool fits = mp_participant->evictUnusedRemoteParticipant(
                          evicted, "Builtin proxies of a new remote participant do not fit") &&
                      addProxiesForBuiltInEndpoints();
    if (!fits) {
      SPDP_LOG("Builtin proxies for the new participant did not fit, forgetting it");
      mp_participant->removeRemoteParticipant(m_proxyDataBuffer.m_guid.prefix);
      return;
    }
  }

  const DataSize_t size = static_cast<DataSize_t>(m_outputSize);
  m_buildInEndpoints.spdpWriter->newChange(ChangeKind_t::ALIVE, m_outputBuffer.data(), size);
#if SPDP_VERBOSE && RTPS_GLOBAL_VERBOSE
  SPDP_LOG("Added new participant with guid: ");
  printGuidPrefix(m_proxyDataBuffer.m_guid.prefix);
#endif
}

bool SPDPAgent::addProxiesForBuiltInEndpoints() {

  LocatorIPv4 *locator = nullptr;

  // Check if the remote participants has a locator in our subnet
  for (unsigned int i = 0; i < m_proxyDataBuffer.m_metatrafficUnicastLocatorList.size(); i++) {
    LocatorIPv4 *l = &(m_proxyDataBuffer.m_metatrafficUnicastLocatorList[i]);
    if (l->isValid() &&
        l->isSameSubnet(mp_participant->m_localIpAddress, mp_participant->m_localPrefixLength)) {
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

  // All four proxies or none: each builtin endpoint's proxy pool holds
  // NUM_*_PROXIES_PER_* entries, one per remote participant, and a peer that
  // only partly fits could never complete SEDP with us. On any failure the
  // ones that did fit are removed again and the caller forgets the peer.
  const GuidPrefix_t &prefix = m_proxyDataBuffer.m_guid.prefix;
  bool ok = true;

  if (m_proxyDataBuffer.hasPublicationReader()) {
    const ReaderProxy proxy{{prefix, ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER}, *locator, true};
    ok = m_buildInEndpoints.sedpPubWriter->addNewMatchedReader(proxy) && ok;
  }

  if (m_proxyDataBuffer.hasSubscriptionReader()) {
    const ReaderProxy proxy{{prefix, ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER}, *locator, true};
    ok = m_buildInEndpoints.sedpSubWriter->addNewMatchedReader(proxy) && ok;
  }

  if (m_proxyDataBuffer.hasPublicationWriter()) {
    const WriterProxy proxy{{prefix, ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER}, *locator, true};
    if (m_buildInEndpoints.sedpPubReader->addNewMatchedWriter(proxy)) {
      m_buildInEndpoints.sedpPubReader->sendPreemptiveAckNack(proxy);
    } else {
      ok = false;
    }
  }

  if (m_proxyDataBuffer.hasSubscriptionWriter()) {
    const WriterProxy proxy{{prefix, ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER}, *locator, true};
    if (m_buildInEndpoints.sedpSubReader->addNewMatchedWriter(proxy)) {
      m_buildInEndpoints.sedpSubReader->sendPreemptiveAckNack(proxy);
    } else {
      ok = false;
    }
  }

  if (!ok) {
    m_buildInEndpoints.sedpPubWriter->removeProxy(
        {prefix, ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER});
    m_buildInEndpoints.sedpSubWriter->removeProxy(
        {prefix, ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER});
    m_buildInEndpoints.sedpPubReader->removeProxy(
        {prefix, ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER});
    m_buildInEndpoints.sedpSubReader->removeProxy(
        {prefix, ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER});
  }
  return ok;
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

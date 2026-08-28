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

#include "rtps/entities/Participant.hpp"
#include "rtps/entities/Reader.hpp"
#include "rtps/entities/Writer.hpp"
#include "rtps/messages/MessageReceiver.hpp"
#include "rtps/utils/Log.hpp"
#include <mutex>

#if PARTICIPANT_VERBOSE && RTPS_GLOBAL_VERBOSE
#define PARTICIPANT_LOG(...) logger_.warn(__VA_ARGS__)
#else
#define PARTICIPANT_LOG(...)                                                                       \
  do {                                                                                             \
  } while (0)
#endif

using rtps::Participant;

Participant::Participant()
    : espp::BaseComponent("RtpsParticipant", espp::Logger::Verbosity::WARN)
    , m_guidPrefix(GUIDPREFIX_UNKNOWN)
    , m_participantId(PARTICIPANT_ID_INVALID)
    , m_receiver(this) {}
Participant::Participant(const GuidPrefix_t &guidPrefix, ParticipantId_t participantId)
    : espp::BaseComponent("RtpsParticipant", espp::Logger::Verbosity::WARN)
    , m_guidPrefix(guidPrefix)
    , m_participantId(participantId)
    , m_receiver(this) {}

Participant::~Participant() { m_spdpAgent.stop(); }

void Participant::reuse(const GuidPrefix_t &guidPrefix, ParticipantId_t participantId) {
  m_guidPrefix = guidPrefix;
  m_participantId = participantId;
  m_localIpAddress = {Config::IP_ADDRESS[0], Config::IP_ADDRESS[1], Config::IP_ADDRESS[2],
                      Config::IP_ADDRESS[3]};
}

void Participant::reuse(const GuidPrefix_t &guidPrefix, ParticipantId_t participantId,
                        const Ip4AddressBytes &localIpAddress) {
  m_guidPrefix = guidPrefix;
  m_participantId = participantId;
  m_localIpAddress = localIpAddress;
}

bool Participant::isValid() { return m_participantId != PARTICIPANT_ID_INVALID; }

std::array<uint8_t, 3> Participant::getNextUserEntityKey() {
  const auto result = m_nextUserEntityId;

  ++m_nextUserEntityId[2];
  if (m_nextUserEntityId[2] == 0) {
    ++m_nextUserEntityId[1];
    if (m_nextUserEntityId[1] == 0) {
      ++m_nextUserEntityId[0];
    }
  }
  return result;
}

bool Participant::registerOnNewPublisherMatchedCallback(void (*callback)(void *arg), void *args) {
  if (!m_hasBuilInEndpoints) {
    return false;
  }

  m_sedpAgent.registerOnNewPublisherMatchedCallback(callback, args);
  return true;
}

bool Participant::registerOnNewSubscriberMatchedCallback(void (*callback)(void *arg), void *args) {
  if (!m_hasBuilInEndpoints) {
    return false;
  }

  m_sedpAgent.registerOnNewSubscriberMatchedCallback(callback, args);
  return true;
}

rtps::Writer *Participant::addWriter(Writer *pWriter) {
  // Reserve the slot under m_mutex, but announce via the SEDP agent OUTSIDE
  // it: the agent locks SEDPAgent::m_mutex and then this mutex (its receive
  // handlers and tryMatchUnmatchedEndpoints() call back into this
  // participant), so nesting the agent call under m_mutex is a lock-order
  // inversion that deadlocks under concurrent discovery traffic. The global
  // order is SEDPAgent::m_mutex -> Participant::m_mutex, never the reverse.
  bool inserted = false;
  {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    for (unsigned int i = 0; i < m_writers.size(); i++) {
      if (m_writers[i] == nullptr) {
        m_writers[i] = pWriter;
        inserted = true;
        break;
      }
    }
  }
  if (!inserted) {
    return nullptr;
  }
  if (m_hasBuilInEndpoints) {
    m_sedpAgent.addWriter(*pWriter);
  }
  return pWriter;
}

bool Participant::isWritersFull() {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (unsigned int i = 0; i < m_writers.size(); i++) {
    if (m_writers[i] == nullptr) {
      return false;
    }
  }

  return true;
}

rtps::Reader *Participant::addReader(Reader *pReader) {
  // Slot under m_mutex, SEDP announcement outside it - see addWriter() for
  // the lock-order rationale (SEDPAgent::m_mutex must never be acquired while
  // holding m_mutex).
  bool inserted = false;
  {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    for (unsigned int i = 0; i < m_readers.size(); i++) {
      if (m_readers[i] == nullptr) {
        m_readers[i] = pReader;
        inserted = true;
        break;
      }
    }
  }
  if (!inserted) {
    return nullptr;
  }
  if (m_hasBuilInEndpoints) {
    m_sedpAgent.addReader(*pReader);
  }
  return pReader;
}

bool Participant::deleteReader(Reader *reader) {
  // Membership check under m_mutex first (pointer identity - endpoints are
  // pooled objects owned by the Domain - which also guards the empty (nullptr)
  // slots the previous sequence-number comparison dereferenced).
  bool found = false;
  {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    for (unsigned int i = 0; i < m_readers.size(); i++) {
      if (m_readers[i] == reader) {
        found = true;
        break;
      }
    }
  }
  if (!found || reader == nullptr) {
    return false;
  }
  // Make the SEDP disposal and the slot removal ATOMIC by holding the agent's
  // mutex across both, in the global lock order (SEDPAgent::m_mutex ->
  // Participant::m_mutex, see addWriter()). Without this, a concurrent SEDP
  // receive handler could match a newly announced remote writer to this reader
  // in the window between its disposal and the slot-clear - consuming the
  // remote from the unmatched registry just before the reader vanishes, losing
  // that match for any replacement reader. Both mutexes are recursive, so the
  // agent's own lock in deleteReader() nests harmlessly.
  std::lock_guard<std::recursive_mutex> sedp_lock(m_sedpAgent.getMutex());
  if (!m_sedpAgent.deleteReader(reader)) {
    PARTICIPANT_LOG("Found reader but SEDP deletion failed");
    return false;
  }
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (unsigned int i = 0; i < m_readers.size(); i++) {
    if (m_readers[i] == reader) {
      m_readers[i] = nullptr;
      return true;
    }
  }
  return false;
}

bool Participant::deleteWriter(Writer *writer) {
  // Same structure and lock-order rationale as deleteReader().
  bool found = false;
  {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    for (unsigned int i = 0; i < m_writers.size(); i++) {
      if (m_writers[i] == writer) {
        found = true;
        break;
      }
    }
  }
  if (!found || writer == nullptr) {
    return false;
  }
  // Atomic disposal + slot removal under the agent's mutex, in the global lock
  // order (SEDPAgent::m_mutex -> Participant::m_mutex) - see deleteReader().
  std::lock_guard<std::recursive_mutex> sedp_lock(m_sedpAgent.getMutex());
  if (!m_sedpAgent.deleteWriter(writer)) {
    PARTICIPANT_LOG("Found writer but SEDP deletion failed");
    return false;
  }
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (unsigned int i = 0; i < m_writers.size(); i++) {
    if (m_writers[i] == writer) {
      m_writers[i] = nullptr;
      return true;
    }
  }
  return false;
}

bool Participant::isReadersFull() {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (unsigned int i = 0; i < m_readers.size(); i++) {
    if (m_readers[i] == nullptr) {
      return false;
    }
  }

  return true;
}

rtps::Writer *Participant::getWriter(EntityId_t id) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (size_t i = 0; i < m_writers.size(); ++i) {
    if (m_writers[i] == nullptr) {
      continue;
    }
    if (m_writers[i]->m_attributes.endpointGuid.entityId == id) {
      return m_writers[i];
    }
  }
  return nullptr;
}

rtps::Reader *Participant::getReader(EntityId_t id) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (size_t i = 0; i < m_readers.size(); ++i) {
    if (m_readers[i] == nullptr) {
      continue;
    }
    if (m_readers[i]->m_attributes.endpointGuid.entityId == id) {
      return m_readers[i];
    }
  }
  return nullptr;
}

rtps::Reader *Participant::getReaderByWriterId(const Guid_t &guid) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (size_t i = 0; i < m_readers.size(); ++i) {
    if (m_readers[i] == nullptr) {
      continue;
    }
    if (m_readers[i]->isProxy(guid)) {
      return m_readers[i];
    }
  }
  return nullptr;
}

// Generation-capturing lookup variants for the receive path. The generation is
// read while m_mutex is held, i.e. atomically with the slot still being
// registered: deleteReader()/deleteWriter() clear the slot under this mutex
// BEFORE reset() bumps the generation, so a pointer returned here always comes
// with the pre-deletion generation and a stale dispatch is rejected by the
// endpoint's *IfCurrent check.
rtps::Writer *Participant::getWriter(EntityId_t id, uint32_t &generation_out) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  Writer *writer = getWriter(id);
  if (writer != nullptr) {
    generation_out = writer->generation();
  }
  return writer;
}

rtps::Reader *Participant::getReader(EntityId_t id, uint32_t &generation_out) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  Reader *reader = getReader(id);
  if (reader != nullptr) {
    generation_out = reader->generation();
  }
  return reader;
}

rtps::Reader *Participant::getReaderByWriterId(const Guid_t &guid, uint32_t &generation_out) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  Reader *reader = getReaderByWriterId(guid);
  if (reader != nullptr) {
    generation_out = reader->generation();
  }
  return reader;
}

rtps::Writer *Participant::getMatchingWriter(const TopicData &readerTopicData) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (size_t i = 0; i < m_writers.size(); ++i) {
    if (m_writers[i] == nullptr) {
      continue;
    }
    if (m_writers[i]->m_attributes.matchesTopicOf(readerTopicData) &&
        (readerTopicData.reliabilityKind == ReliabilityKind_t::BEST_EFFORT ||
         m_writers[i]->m_attributes.reliabilityKind == ReliabilityKind_t::RELIABLE)) {
      return m_writers[i];
    }
  }
  return nullptr;
}

rtps::Reader *Participant::getMatchingReader(const TopicData &writerTopicData) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (size_t i = 0; i < m_readers.size(); ++i) {
    if (m_readers[i] == nullptr) {
      continue;
    }
    if (m_readers[i]->m_attributes.matchesTopicOf(writerTopicData) &&
        (writerTopicData.reliabilityKind == ReliabilityKind_t::RELIABLE ||
         m_readers[i]->m_attributes.reliabilityKind == ReliabilityKind_t::BEST_EFFORT)) {
      return m_readers[i];
    }
  }
  return nullptr;
}

rtps::Writer *Participant::getMatchingWriter(const TopicDataCompressed &readerTopicData) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (size_t i = 0; i < m_writers.size(); ++i) {
    if (m_writers[i] == nullptr) {
      continue;
    }
    if (readerTopicData.matchesTopicOf(m_writers[i]->m_attributes) &&
        (readerTopicData.is_reliable == false ||
         m_writers[i]->m_attributes.reliabilityKind == ReliabilityKind_t::RELIABLE)) {
      return m_writers[i];
    }
  }
  return nullptr;
}

rtps::Reader *Participant::getMatchingReader(const TopicDataCompressed &writerTopicData) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (size_t i = 0; i < m_readers.size(); ++i) {
    if (m_readers[i] == nullptr) {
      continue;
    }
    if (writerTopicData.matchesTopicOf(m_readers[i]->m_attributes) &&
        (writerTopicData.is_reliable == true ||
         m_readers[i]->m_attributes.reliabilityKind == ReliabilityKind_t::BEST_EFFORT)) {
      return m_readers[i];
    }
  }
  return nullptr;
}

bool Participant::addNewRemoteParticipant(const ParticipantProxyData &remotePart) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  return m_remoteParticipants.add(remotePart);
}

bool Participant::removeRemoteParticipant(const GuidPrefix_t &prefix) {
  // Documented global lock order: SEDPAgent::m_mutex BEFORE
  // Participant::m_mutex. The removal below calls into the SEDP agent
  // (removeUnmatchedEntitiesOfParticipant takes its mutex), so acquire the
  // agent mutex first - taking m_mutex alone here and letting the nested call
  // grab the agent mutex would be an ABBA inversion against the SEDP receive
  // handlers, which hold the agent mutex and call findRemoteParticipant()
  // (m_mutex). Callers must not already hold m_mutex without the agent mutex.
  std::lock_guard<std::recursive_mutex> sedp_lock(m_sedpAgent.getMutex());
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  auto isElementToRemove = [&](const ParticipantProxyData &proxy) {
    return proxy.m_guid.prefix == prefix;
  };
  auto thunk = [](void *arg, const ParticipantProxyData &value) {
    return (*static_cast<decltype(isElementToRemove) *>(arg))(value);
  };
  removeAllProxiesOfParticipant(prefix);
  m_sedpAgent.removeUnmatchedEntitiesOfParticipant(prefix);
  return m_remoteParticipants.remove(thunk, &isElementToRemove);
}

void Participant::removeAllProxiesOfParticipant(const GuidPrefix_t &prefix) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (unsigned int i = 0; i < m_readers.size(); i++) {
    if (m_readers[i] == nullptr) {
      continue;
    }
    m_readers[i]->removeAllProxiesOfParticipant(prefix);
  }

  for (unsigned int i = 0; i < m_writers.size(); i++) {
    if (m_writers[i] == nullptr) {
      continue;
    }
    m_writers[i]->removeAllProxiesOfParticipant(prefix);
  }
}

void Participant::removeProxyFromAllEndpoints(const Guid_t &guid) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (unsigned int i = 0; i < m_writers.size(); i++) {
    if (m_writers[i] == nullptr) {
      continue;
    }
    if (m_writers[i]->removeProxy(guid)) {
      PARTICIPANT_LOG("Removing proxy for writer [{}, {}], proxies left = {}",
                      m_writers[i]->m_attributes.topicName, m_writers[i]->m_attributes.typeName,
                      (int)m_writers[i]->getProxiesCount());
    }
  }

  for (unsigned int i = 0; i < m_readers.size(); i++) {
    if (m_readers[i] == nullptr) {
      continue;
    }
    if (m_readers[i]->removeProxy(guid)) {
      PARTICIPANT_LOG("Removing proxy for reader [{}, {}], proxies left = {}",
                      m_readers[i]->m_attributes.topicName, m_readers[i]->m_attributes.typeName,
                      (int)m_readers[i]->getProxiesCount());
    }
  }
}

const rtps::ParticipantProxyData *Participant::findRemoteParticipant(const GuidPrefix_t &prefix) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  auto isElementToFind = [&](const ParticipantProxyData &proxy) {
    return proxy.m_guid.prefix == prefix;
  };
  auto thunk = [](void *arg, const ParticipantProxyData &value) {
    return (*static_cast<decltype(isElementToFind) *>(arg))(value);
  };
  return m_remoteParticipants.find(thunk, &isElementToFind);
}

void Participant::refreshRemoteParticipantLiveliness(const GuidPrefix_t &prefix) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  auto isElementToFind = [&](const ParticipantProxyData &proxy) {
    return proxy.m_guid.prefix == prefix;
  };
  auto thunk = [](void *arg, const ParticipantProxyData &value) {
    return (*static_cast<decltype(isElementToFind) *>(arg))(value);
  };

  auto remoteParticipant = m_remoteParticipants.find(thunk, &isElementToFind);
  if (remoteParticipant != nullptr) {
    remoteParticipant->onAliveSignal();
  }
}

bool Participant::hasReaderWithMulticastLocator(const std::array<uint8_t, 4> &address) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (size_t i = 0; i < m_readers.size(); i++) {
    if (m_readers[i] == nullptr) {
      continue;
    }
    if (m_readers[i]->m_attributes.multicastLocator.getIp4AddressBytes() == address) {
      return true;
    }
  }
  return false;
}

uint32_t Participant::getRemoteParticipantCount() {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  return m_remoteParticipants.getNumElements();
}

rtps::MessageReceiver *Participant::getMessageReceiver() { return &m_receiver; }

bool Participant::checkAndResetHeartbeats() {
  // Phase 1 - SCAN ONLY. Lock order: SPDP-agent mutex BEFORE the participant
  // mutex, matching the SPDP receive path (handleSPDPPackage holds the agent
  // mutex and then calls findRemoteParticipant, which takes m_mutex). The
  // expired participant is only SELECTED here; the removal itself must run
  // with these locks RELEASED, because removeRemoteParticipant() acquires the
  // SEDP-agent mutex before m_mutex (the documented global order) - removing
  // while m_mutex is held would be an ABBA inversion against the SEDP receive
  // handlers, which hold the SEDP mutex and call findRemoteParticipant().
  GuidPrefix_t expiredPrefix{};
  bool haveExpired = false;
  {
    std::lock_guard<std::recursive_mutex> lock1(m_spdpAgent.m_mutex);
    std::lock_guard<std::recursive_mutex> lock2(m_mutex);
    PARTICIPANT_LOG("Have {} remote participants",
                    (unsigned int)m_remoteParticipants.getNumElements());
    PARTICIPANT_LOG("Unmatched remote writers/readers, {} / {}",
                    static_cast<unsigned int>(m_sedpAgent.getNumRemoteUnmatchedWriters()),
                    static_cast<unsigned int>(m_sedpAgent.getNumRemoteUnmatchedReaders()));
    for (auto &remote : m_remoteParticipants) {
      PARTICIPANT_LOG("Remote GUID = {} {} {} {} | Age = {} [ms]", remote.m_guid.prefix.id[4],
                      remote.m_guid.prefix.id[5], remote.m_guid.prefix.id[6],
                      remote.m_guid.prefix.id[7],
                      (unsigned int)remote.getAliveSignalAgeInMilliseconds());
      if (remote.isAlive()) {
        continue;
      }
      PARTICIPANT_LOG("removing remote participant");
      expiredPrefix = remote.m_guid.prefix;
      haveExpired = true;
      break;
    }
  }
  if (!haveExpired) {
    return true;
  }
  // Phase 2 - remove with no locks held (a liveness refresh racing this
  // window loses by design: the lease already expired, and SPDP rediscovery
  // re-adds the participant).
  return removeRemoteParticipant(expiredPrefix);
}

void Participant::printInfo() {

  uint32_t max_reader_proxies = 0;
  for (unsigned int i = 0; i < m_readers.size(); i++) {
    if (m_readers[i] != nullptr && m_readers[i]->isInitialized()) {
      if (m_hasBuilInEndpoints && i < 3) {
#ifdef PARTICIPANT_PRINTINFO_LONG
        if (m_readers[i]->m_attributes.endpointGuid.entityId ==
            ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER) {
          PARTICIPANT_LOG("Reader {}: SPDP BUILTIN READER | Remote Proxies = {}", i,
                          static_cast<int>(m_readers[i]->getProxiesCount()));
        }
        if (m_readers[i]->m_attributes.endpointGuid.entityId ==
            ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER) {
          PARTICIPANT_LOG("Reader {}: SEDP PUBLICATION READER | Remote Proxies = {}", i,
                          static_cast<int>(m_readers[i]->getProxiesCount()));
        }
        if (m_readers[i]->m_attributes.endpointGuid.entityId ==
            ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER) {
          PARTICIPANT_LOG("Reader {}: SEDP SUBSCRIPTION READER | Remote Proxies = {}", i,
                          static_cast<int>(m_readers[i]->getProxiesCount()));
        }
#endif
        continue;
      }

      max_reader_proxies = std::max(max_reader_proxies, m_readers[i]->getProxiesCount());
#ifdef PARTICIPANT_PRINTINFO_LONG
      PARTICIPANT_LOG("Reader {}: Topic = {} | Type = {} | Remote Proxies = {} | SEDP "
                      "SN = {}",
                      i, m_readers[i]->m_attributes.topicName, m_readers[i]->m_attributes.typeName,
                      static_cast<int>(m_readers[i]->getProxiesCount()),
                      static_cast<int>(m_readers[i]->getSEDPSequenceNumber().low));
#endif
    }
  }

  uint32_t max_writer_proxies = 0;
  for (unsigned int i = 0; i < m_writers.size(); i++) {

    if (m_hasBuilInEndpoints && i < 3) {
#ifdef PARTICIPANT_PRINTINFO_LONG
      if (m_writers[i]->m_attributes.endpointGuid.entityId ==
          ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER) {
        PARTICIPANT_LOG("Writer {}: SPDP WRITER | Remote Proxies = {}", i,
                        static_cast<int>(m_writers[i]->getProxiesCount()));
      }
      if (m_writers[i]->m_attributes.endpointGuid.entityId ==
          ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER) {
        PARTICIPANT_LOG("Writer {}: SEDP PUBLICATION WRITER | Remote Proxies = {}", i,
                        static_cast<int>(m_writers[i]->getProxiesCount()));
      }
      if (m_writers[i]->m_attributes.endpointGuid.entityId ==
          ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER) {
        PARTICIPANT_LOG("Writer {}: SEDP SUBSCRIPTION WRITER | Remote Proxies = {}", i,
                        static_cast<int>(m_writers[i]->getProxiesCount()));
      }
#endif
      continue;
    }

    if (m_writers[i] != nullptr && m_writers[i]->isInitialized()) {
      max_writer_proxies = std::max(max_writer_proxies, m_writers[i]->getProxiesCount());
#ifdef PARTICIPANT_PRINTINFO_LONG
      PARTICIPANT_LOG("Writer {}: Topic = {} | Type = {} | Remote Proxies = {} | SEDP "
                      "SN = {}",
                      i, m_writers[i]->m_attributes.topicName, m_writers[i]->m_attributes.typeName,
                      static_cast<int>(m_writers[i]->getProxiesCount()),
                      static_cast<int>(m_writers[i]->getSEDPSequenceNumber().low));
#endif
    }
  }

  PARTICIPANT_LOG("Max Writer Proxies {}", max_writer_proxies);
  PARTICIPANT_LOG("Max Reader Proxies {}", max_reader_proxies);
  PARTICIPANT_LOG("Unmatched Remote Readers = {}",
                  static_cast<int>(m_sedpAgent.getNumRemoteUnmatchedReaders()));
  PARTICIPANT_LOG("Unmatched Remote Writers = {}",
                  static_cast<int>(m_sedpAgent.getNumRemoteUnmatchedWriters()));
  PARTICIPANT_LOG("Remote Participants = {}",
                  static_cast<int>(m_remoteParticipants.getNumElements()));
}

rtps::SPDPAgent &Participant::getSPDPAgent() { return m_spdpAgent; }

void Participant::addBuiltInEndpoints(BuiltInEndpoints &endpoints) {
  // Only the flag needs m_mutex. The agent init()s register reader callbacks
  // (Reader::m_callback_mutex) and the add*() calls take the SEDP/participant
  // locks themselves; running them under m_mutex would establish a
  // participant -> callback-mutex (and participant -> SEDP) order that
  // inverts the discovery receive path (callback/SEDP mutex -> participant)
  // and could deadlock init against a concurrently arriving SPDP/SEDP
  // datagram.
  {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    m_hasBuilInEndpoints = true;
  }
  m_spdpAgent.init(*this, endpoints);
  m_sedpAgent.init(*this, endpoints);

  // This needs to be done after initializing the agents
  addWriter(endpoints.spdpWriter);
  addReader(endpoints.spdpReader);
  addWriter(endpoints.sedpPubWriter);
  addReader(endpoints.sedpPubReader);
  addWriter(endpoints.sedpSubWriter);
  addReader(endpoints.sedpSubReader);
}

void Participant::newMessage(const uint8_t *data, DataSize_t size) {
  if (!m_receiver.processMessage(data, size)) {
    PARTICIPANT_LOG("MESSAGE PROCESSING FAILED");
  }
}

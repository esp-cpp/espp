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
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (unsigned int i = 0; i < m_writers.size(); i++) {
    if (m_writers[i] == nullptr) {
      m_writers[i] = pWriter;
      if (m_hasBuilInEndpoints) {
        m_sedpAgent.addWriter(*pWriter);
      }
      return pWriter;
    }
  }
  return nullptr;
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
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (unsigned int i = 0; i < m_readers.size(); i++) {
    if (m_readers[i] == nullptr) {
      m_readers[i] = pReader;
      if (m_hasBuilInEndpoints) {
        m_sedpAgent.addReader(*pReader);
      }
      return pReader;
    }
  }

  return nullptr;
}

bool Participant::deleteReader(Reader *reader) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (unsigned int i = 0; i < m_readers.size(); i++) {
    if (m_readers[i]->getSEDPSequenceNumber() == reader->getSEDPSequenceNumber()) {
      if (m_sedpAgent.deleteReader(reader)) {
        m_readers[i] = nullptr;
        return true;
      }
      PARTICIPANT_LOG("Found reader but SEDP deletion failed");
    }
  }
  return false;
}

bool Participant::deleteWriter(Writer *writer) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (unsigned int i = 0; i < m_writers.size(); i++) {
    if (m_writers[i]->getSEDPSequenceNumber() == writer->getSEDPSequenceNumber()) {
      if (m_sedpAgent.deleteWriter(writer)) {
        m_writers[i] = nullptr;
        return true;
      }
      PARTICIPANT_LOG("Found reader but SEDP deletion failed");
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
  for (uint8_t i = 0; i < m_writers.size(); ++i) {
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
  for (uint8_t i = 0; i < m_readers.size(); ++i) {
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
  for (uint8_t i = 0; i < m_readers.size(); ++i) {
    if (m_readers[i] == nullptr) {
      continue;
    }
    if (m_readers[i]->isProxy(guid)) {
      return m_readers[i];
    }
  }
  return nullptr;
}

rtps::Writer *Participant::getMatchingWriter(const TopicData &readerTopicData) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  for (uint8_t i = 0; i < m_writers.size(); ++i) {
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
  for (uint8_t i = 0; i < m_readers.size(); ++i) {
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
  for (uint8_t i = 0; i < m_writers.size(); ++i) {
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
  for (uint8_t i = 0; i < m_readers.size(); ++i) {
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
  for (uint8_t i = 0; i < m_readers.size(); i++) {
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
  std::lock_guard<std::recursive_mutex> lock1(m_mutex);
  std::lock_guard<std::recursive_mutex> lock2(m_spdpAgent.m_mutex);
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
    bool success = removeRemoteParticipant(remote.m_guid.prefix);
    if (!success) {
      return false;
    } else {
      return true;
    }
  }
  return true;
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
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  m_hasBuilInEndpoints = true;
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

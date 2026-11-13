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

#include "rtps/discovery/SEDPAgent.hpp"
#include "rtps/discovery/TopicData.hpp"
#include "rtps/entities/Participant.hpp"
#include "rtps/entities/Reader.hpp"
#include "rtps/entities/Writer.hpp"
#include "rtps/messages/MessageTypes.hpp"
#include "rtps/utils/CdrBuffer.hpp"
#include "rtps/utils/Log.hpp"
#include <algorithm>
#include <array>
#include <mutex>
#include <span>

using rtps::SEDPAgent;

#if SEDP_VERBOSE && RTPS_GLOBAL_VERBOSE
#define SEDP_LOG(...) logger_.warn(__VA_ARGS__)
#else
#define SEDP_LOG(...)                                                                              \
  do {                                                                                             \
  } while (0)
#endif

SEDPAgent::SEDPAgent()
    : espp::BaseComponent("RtpsSEDP", espp::Logger::Verbosity::WARN) {}

void SEDPAgent::init(Participant &part, const BuiltInEndpoints &endpoints) {
  m_part = &part;
  m_endpoints = endpoints;
  if (m_endpoints.sedpPubReader != nullptr) {
    m_endpoints.sedpPubReader->registerCallback(jumppadPublisherReader, this);
  }
  if (m_endpoints.sedpSubReader != nullptr) {
    m_endpoints.sedpSubReader->registerCallback(jumppadSubscriptionReader, this);
  }
}

void SEDPAgent::registerOnNewPublisherMatchedCallback(void (*callback)(void *arg), void *args) {
  mfp_onNewPublisherCallback = callback;
  m_onNewPublisherArgs = args;
}

void SEDPAgent::registerOnNewSubscriberMatchedCallback(void (*callback)(void *arg), void *args) {
  mfp_onNewSubscriberCallback = callback;
  m_onNewSubscriberArgs = args;
}

void SEDPAgent::jumppadPublisherReader(void *callee, const ReaderCacheChange &cacheChange) {
  auto agent = static_cast<SEDPAgent *>(callee);
  agent->handlePublisherReaderMessage(cacheChange);
}

void SEDPAgent::jumppadSubscriptionReader(void *callee, const ReaderCacheChange &cacheChange) {
  auto agent = static_cast<SEDPAgent *>(callee);
  agent->handleSubscriptionReaderMessage(cacheChange);
}

void SEDPAgent::handlePublisherReaderMessage(const ReaderCacheChange &change) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
#if SEDP_VERBOSE
  SEDP_LOG("New publisher");
#endif

  if (change.kind != ChangeKind_t::ALIVE) {
    // A dispose/unregister (PID_STATUS_INFO in the inline QoS): the peer
    // deleted the endpoint. Usually no payload at all, so it is handled before
    // the parse below.
    handleRemoteEndpointDeletion(change, /*writers=*/true);
    return;
  }

  if (!change.copyInto(m_buffer, sizeof(m_buffer) / sizeof(m_buffer[0]))) {
#if SEDP_VERBOSE
    SEDP_LOG("EDPAgent: Buffer too small.");
#endif
    return;
  }

  TopicData topicData;
  if (topicData.readFromBuffer(std::span<const uint8_t>(m_buffer, change.getDataSize()),
                               m_part->m_localIpAddress, m_part->m_localPrefixLength)) {
    handlePublisherReaderMessage(topicData, change);
  }
}

namespace {
// The unicast locator a remote endpoint's traffic goes to. FastDDS puts
// PID_UNICAST_LOCATOR in every SEDP sample; CycloneDDS omits it and relies on
// the participant's default unicast locator from SPDP (RTPS 2.5 8.5.3.2 allows
// both). Without this fallback a CycloneDDS endpoint gets an invalid locator
// and nothing is ever sent to it. Of the defaults, prefer one on our subnet:
// a peer on several networks lists one per network, and only that one is
// reachable.
rtps::LocatorIPv4 endpointUnicastLocator(rtps::Participant *part, const rtps::TopicData &data) {
  rtps::LocatorIPv4 loc(data.unicastLocator);
  if (loc.isValid()) {
    return loc;
  }
  const rtps::ParticipantProxyData *remote = part->findRemoteParticipant(data.endpointGuid.prefix);
  if (remote != nullptr) {
    const auto &defaults = remote->m_defaultUnicastLocatorList;
    auto found = std::find_if(defaults.begin(), defaults.end(), [part](const auto &candidate) {
      return candidate.isValid() &&
             candidate.isSameSubnet(part->m_localIpAddress, part->m_localPrefixLength);
    });
    if (found == defaults.end()) {
      found = std::find_if(defaults.begin(), defaults.end(),
                           [](const auto &candidate) { return candidate.isValid(); });
    }
    if (found != defaults.end()) {
      return *found;
    }
  }
  return loc;
}
} // namespace

void SEDPAgent::rememberRemoteWriter(const TopicData &writerData) {
  // The record keeps the locator the endpoint's traffic goes to, resolved as
  // for a match now (endpointUnicastLocator): a peer that announces no
  // PID_UNICAST_LOCATOR (Cyclone DDS) uses its participant's default, and a
  // local reader created later must get that one, not an invalid locator.
  TopicDataCompressed record(writerData);
  record.unicastLocator = endpointUnicastLocator(m_part, writerData);
  rememberRemoteWriter(record);
}

void SEDPAgent::rememberRemoteReader(const TopicData &readerData) {
  TopicDataCompressed record(readerData);
  record.unicastLocator = endpointUnicastLocator(m_part, readerData); // see above
  rememberRemoteReader(record);
}

void SEDPAgent::rememberRemoteWriter(const TopicDataCompressed &writerData) {
  if (isKnown(m_knownRemoteWriters, writerData.endpointGuid)) {
    return; // a re-announcement
  }
  if (m_knownRemoteWriters.isFull()) {
    // Not remembered: a local reader created later cannot match it. Said once
    // per fill; the flag resets when a record is forgotten.
    if (!m_knownRemoteWritersFullWarned) {
      m_knownRemoteWritersFullWarned = true;
      logger_.warn("Known remote writers full (MAX_NUM_UNMATCHED_REMOTE_WRITERS={}): a remote "
                   "writer cannot be remembered for readers created later",
                   static_cast<unsigned>(Config::MAX_NUM_UNMATCHED_REMOTE_WRITERS));
    }
    return;
  }
  SEDP_LOG("Remembering remote writer {:x} {:x}.", writerData.topicHash, writerData.typeHash);
  m_knownRemoteWriters.add(writerData);
}

void SEDPAgent::rememberRemoteReader(const TopicDataCompressed &readerData) {
  if (isKnown(m_knownRemoteReaders, readerData.endpointGuid)) {
    return;
  }
  if (m_knownRemoteReaders.isFull()) {
    if (!m_knownRemoteReadersFullWarned) {
      m_knownRemoteReadersFullWarned = true;
      logger_.warn("Known remote readers full (MAX_NUM_UNMATCHED_REMOTE_READERS={}): a remote "
                   "reader cannot be remembered for writers created later",
                   static_cast<unsigned>(Config::MAX_NUM_UNMATCHED_REMOTE_READERS));
    }
    return;
  }
  SEDP_LOG("Remembering remote reader {:x} {:x}.", readerData.topicHash, readerData.typeHash);
  m_knownRemoteReaders.add(readerData);
}

void SEDPAgent::forgetRemoteEndpoint(const Guid_t &guid) {
  auto isElementToRemove = [&](const TopicDataCompressed &topicData) {
    return topicData.endpointGuid == guid;
  };

  auto thunk = [](void *arg, const TopicDataCompressed &value) {
    return (*static_cast<decltype(isElementToRemove) *>(arg))(value);
  };

  if (m_knownRemoteReaders.remove(thunk, &isElementToRemove)) {
    m_knownRemoteReadersFullWarned = false;
  }
  if (m_knownRemoteWriters.remove(thunk, &isElementToRemove)) {
    m_knownRemoteWritersFullWarned = false;
  }
}

void SEDPAgent::forgetRemoteEndpointsOfParticipant(const GuidPrefix_t &guidPrefix) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  auto isElementToRemove = [&](const TopicDataCompressed &topicData) {
    return topicData.endpointGuid.prefix == guidPrefix;
  };

  auto thunk = [](void *arg, const TopicDataCompressed &value) {
    return (*static_cast<decltype(isElementToRemove) *>(arg))(value);
  };

  if (m_knownRemoteReaders.remove(thunk, &isElementToRemove)) {
    m_knownRemoteReadersFullWarned = false;
  }
  if (m_knownRemoteWriters.remove(thunk, &isElementToRemove)) {
    m_knownRemoteWritersFullWarned = false;
  }
}

uint32_t SEDPAgent::getNumKnownRemoteReaders() { return m_knownRemoteReaders.getNumElements(); }

uint32_t SEDPAgent::getNumKnownRemoteWriters() { return m_knownRemoteWriters.getNumElements(); }

void SEDPAgent::handlePublisherReaderMessage(const TopicData &writerData,
                                             const ReaderCacheChange &change) {
  // TODO Is it okay to add Endpoint if the respective participant is unknown
  // participant?
  if (!m_part->findRemoteParticipant(writerData.endpointGuid.prefix)) {
    return;
  }

  if (writerData.isDisposedFlagSet() || writerData.isUnregisteredFlagSet()) {
    handleRemoteEndpointDeletion(writerData, change, /*writers=*/true);
    return;
  }

#if SEDP_VERBOSE
  SEDP_LOG("PUB T/D {}/{}", writerData.topicName, writerData.typeName);
#endif
  // Remembered first, whatever matches now: a peer announces an endpoint
  // once, and a local reader created later matches it from the registry
  // (addReader -> matchKnownRemoteEndpoints). Then every local reader on the
  // topic gets the writer (several readers may share a topic; a DATA the
  // writer addresses to ENTITYID_UNKNOWN is for all of them, see
  // Participant::getReadersByWriterId).
  rememberRemoteWriter(writerData);
  Participant::Readers readers;
  const std::size_t count = m_part->getMatchingReaders(writerData, readers);
  if (count == 0) {
#if SEDP_VERBOSE
    SEDP_LOG("SEDPAgent: Couldn't find reader for new Publisher[{}, {}]", writerData.topicName,
             writerData.typeName);
#endif
    return;
  }
  // TODO check policies
#if SEDP_VERBOSE
  if (writerData.reliabilityKind == ReliabilityKind_t::RELIABLE) {
    SEDP_LOG("Found a new reliable publisher");
  } else {
    SEDP_LOG("Found a new best-effort publisher");
  }
#endif
  const WriterProxy proxy{writerData.endpointGuid, endpointUnicastLocator(m_part, writerData),
                          (writerData.reliabilityKind == ReliabilityKind_t::RELIABLE)};
  for (std::size_t i = 0; i < count; ++i) {
    // A reader whose proxy pool is full is retried from the registry when a
    // proxy goes away (removeRemoteEndpoint) or an endpoint is created.
    matchRemoteWriter(*readers[i], proxy);
  }
  if (mfp_onNewPublisherCallback != nullptr) {
    mfp_onNewPublisherCallback(m_onNewPublisherArgs);
  }
}

bool SEDPAgent::matchRemoteWriter(Reader &reader, const WriterProxy &proxy) {
  if (reader.isProxy(proxy.remoteWriterGuid)) {
    return true; // a re-announcement, or a retry from the registry
  }
  if (!reader.addNewMatchedWriter(proxy)) {
    logger_.warn(
        "Reader '{}' has no room for another remote writer "
        "(NUM_WRITER_PROXIES_PER_READER={}): it stays known and is retried when one goes away",
        reader.m_attributes.topicName, static_cast<int>(Config::NUM_WRITER_PROXIES_PER_READER));
    return false;
  }
  if (proxy.is_reliable) {
    // Tell the writer we exist, as the SEDP readers do on participant discovery
    // and as Fast DDS and Cyclone DDS readers do on every match: the writer's
    // ReaderProxy::acknowledged is what a service client waits for before it
    // sends (ServiceClient::wait_for_server), and a writer with an empty history
    // sends no HEARTBEAT for us to answer otherwise.
    reader.sendPreemptiveAckNack(proxy);
  }
  return true;
}

bool SEDPAgent::matchRemoteReader(Writer &writer, const ReaderProxy &proxy) {
  if (writer.isProxy(proxy.remoteReaderGuid)) {
    return true;
  }
  if (!writer.addNewMatchedReader(proxy)) {
    logger_.warn(
        "Writer '{}' has no room for another remote reader "
        "(NUM_READER_PROXIES_PER_WRITER={}): it stays known and is retried when one goes away",
        writer.m_attributes.topicName, static_cast<int>(Config::NUM_READER_PROXIES_PER_WRITER));
    return false;
  }
  return true;
}

template <typename Pool> bool SEDPAgent::isKnown(Pool &pool, const rtps::Guid_t &guid) {
  return std::any_of(pool.begin(), pool.end(),
                     [&](const auto &entry) { return entry.endpointGuid == guid; });
}

void SEDPAgent::retryKnownRemoteEndpoints() {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  matchKnownRemoteEndpoints();
}

void SEDPAgent::handleSubscriptionReaderMessage(const ReaderCacheChange &change) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
#if SEDP_VERBOSE
  SEDP_LOG("New subscriber");
#endif

  if (change.kind != ChangeKind_t::ALIVE) {
    handleRemoteEndpointDeletion(change, /*writers=*/false); // see handlePublisherReaderMessage
    return;
  }

  if (!change.copyInto(m_buffer, sizeof(m_buffer) / sizeof(m_buffer[0]))) {
#if SEDP_VERBOSE
    SEDP_LOG("SEDPAgent: Buffer too small.");
#endif
    return;
  }

  TopicData topicData;
  if (topicData.readFromBuffer(std::span<const uint8_t>(m_buffer, change.getDataSize()),
                               m_part->m_localIpAddress, m_part->m_localPrefixLength)) {
    handleSubscriptionReaderMessage(topicData, change);
  }
}

void SEDPAgent::handleRemoteEndpointDeletion(const ReaderCacheChange &change, bool writers) {
  SEDP_LOG("Endpoint deletion message SN {}.{} GUID {} {} {} {}", (int)change.sn.high,
           (int)change.sn.low, change.writerGuid.prefix.id[0], change.writerGuid.prefix.id[1],
           change.writerGuid.prefix.id[2], change.writerGuid.prefix.id[3]);
  // The instance is named by the inline-QoS key hash (Fast DDS, this engine,
  // Cyclone DDS); a peer that leaves the hash out still carries the endpoint
  // GUID in its (PL_CDR) payload, where a key hash may also sit.
  Guid_t guid = GUID_UNKNOWN;
  if (change.hasKeyHash) {
    guid = change.keyHash;
  } else if (change.getDataSize() > 0 &&
             change.copyInto(m_buffer, sizeof(m_buffer) / sizeof(m_buffer[0]))) {
    TopicData topic;
    if (topic.readFromBuffer(std::span<const uint8_t>(m_buffer, change.getDataSize()),
                             m_part->m_localIpAddress, m_part->m_localPrefixLength)) {
      guid = disposedEndpointGuid(topic);
    }
  }
  if (guid == GUID_UNKNOWN) {
    logger_.warn("Endpoint deletion without a key hash or endpoint GUID, ignored");
    return;
  }
  if (!isDisposableBy(guid, change, writers)) {
    return;
  }
  removeRemoteEndpoint(guid);
}

void SEDPAgent::handleRemoteEndpointDeletion(const TopicData &topic,
                                             const ReaderCacheChange &change, bool writers) {
  // The payload itself said disposed (PID_STATUS_INFO inside the parameter
  // list rather than the inline QoS): the historical path, kept for peers
  // that write it there.
  SEDP_LOG("Endpoint deletion (payload status) SN {}.{}", (int)change.sn.high, (int)change.sn.low);
  const Guid_t guid = change.hasKeyHash ? change.keyHash : disposedEndpointGuid(topic);
  if (guid == GUID_UNKNOWN || !isDisposableBy(guid, change, writers)) {
    return;
  }
  removeRemoteEndpoint(guid);
}

bool SEDPAgent::isDisposableBy(const rtps::Guid_t &guid, const ReaderCacheChange &change,
                               bool writers) {
  // A participant may only dispose its own endpoints, and the publications
  // writer only writers, the subscriptions writer only readers: anything else
  // is a malformed (or hostile) datagram and must not touch our matches.
  const EntityKind_t kind = guid.entityId.entityKind;
  const bool user_writer = kind == EntityKind_t::USER_DEFINED_WRITER_WITH_KEY ||
                           kind == EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY;
  const bool user_reader = kind == EntityKind_t::USER_DEFINED_READER_WITH_KEY ||
                           kind == EntityKind_t::USER_DEFINED_READER_WITHOUT_KEY;
  if (!(guid.prefix == change.writerGuid.prefix) || (writers ? !user_writer : !user_reader)) {
    logger_.warn("Endpoint deletion from participant {:02x}{:02x}{:02x}{:02x} names an endpoint it "
                 "does not own (or of the wrong kind), ignored",
                 change.writerGuid.prefix.id[0], change.writerGuid.prefix.id[1],
                 change.writerGuid.prefix.id[2], change.writerGuid.prefix.id[3]);
    return false;
  }
  return true;
}

rtps::Guid_t SEDPAgent::disposedEndpointGuid(const TopicData &topic) {
  if (topic.entityIdFromKeyHashValid) {
    Guid_t guid;
    guid.prefix = topic.endpointGuid.prefix; // PID_KEY_HASH fills the prefix too
    guid.entityId = topic.entityIdFromKeyHash;
    return guid;
  }
  return topic.endpointGuid; // PID_ENDPOINT_GUID, or GUID_UNKNOWN when absent
}

void SEDPAgent::removeRemoteEndpoint(const Guid_t &guid) {
  // Remove entity ID from all proxies of local endpoints
  m_part->removeProxyFromAllEndpoints(guid);

  // Forget it: it is gone, nothing created later should match it.
  forgetRemoteEndpoint(guid);

  // A proxy slot may have come free: an endpoint that waited for one takes it.
  matchKnownRemoteEndpoints();
}

void SEDPAgent::handleSubscriptionReaderMessage(const TopicData &readerData,
                                                const ReaderCacheChange &change) {
  if (!m_part->findRemoteParticipant(readerData.endpointGuid.prefix)) {
    return;
  }

  if (readerData.isDisposedFlagSet() || readerData.isUnregisteredFlagSet()) {
    handleRemoteEndpointDeletion(readerData, change, /*writers=*/false);
    return;
  }

  rememberRemoteReader(readerData); // see handlePublisherReaderMessage
  Participant::Writers writers;
  const std::size_t count = m_part->getMatchingWriters(readerData, writers);
#if SEDP_VERBOSE
  SEDP_LOG("SUB T/D {}/{}", readerData.topicName, readerData.typeName);
#endif
  if (count == 0) {
#if SEDP_VERBOSE
    SEDP_LOG("SEDPAgent: Couldn't find writer for new subscriber[{}, {}]", readerData.topicName,
             readerData.typeName);
#endif
    return;
  }

  // TODO check policies
#if SEDP_VERBOSE
  if (readerData.reliabilityKind == ReliabilityKind_t::RELIABLE) {
    SEDP_LOG("Found a new reliable subscriber");
  } else {
    SEDP_LOG("Found a new best-effort subscriber");
  }
#endif
  const bool reliable = readerData.reliabilityKind == ReliabilityKind_t::RELIABLE;
  const ReaderProxy proxy =
      readerData.multicastLocator.kind == rtps::LocatorKind_t::LOCATOR_KIND_UDPv4
          ? ReaderProxy{readerData.endpointGuid, endpointUnicastLocator(m_part, readerData),
                        LocatorIPv4(readerData.multicastLocator), reliable}
          : ReaderProxy{readerData.endpointGuid, endpointUnicastLocator(m_part, readerData),
                        reliable};
  for (std::size_t i = 0; i < count; ++i) { // every local writer on the topic
    matchRemoteReader(*writers[i], proxy);
  }

  if (mfp_onNewSubscriberCallback != nullptr) {
    mfp_onNewSubscriberCallback(m_onNewSubscriberArgs);
  }
}

void SEDPAgent::matchKnownRemoteEndpoints() {
  // Every remembered remote endpoint against every compatible local one;
  // matchRemote*() skips the pairs that already hold a proxy, so this is
  // cheap to run on each local endpoint creation and each freed proxy. The
  // registry is not mutated here (records only leave it on dispose / the
  // participant's departure), so iterating the pool is safe.
  for (auto &remote : m_knownRemoteReaders) {
    Participant::Writers writers;
    const std::size_t count = m_part->getMatchingWriters(remote, writers);
    const ReaderProxy proxy{remote.endpointGuid, remote.unicastLocator, remote.multicastLocator,
                            remote.is_reliable};
    for (std::size_t i = 0; i < count; ++i) {
      matchRemoteReader(*writers[i], proxy);
    }
  }
  for (auto &remote : m_knownRemoteWriters) {
    Participant::Readers readers;
    const std::size_t count = m_part->getMatchingReaders(remote, readers);
    const WriterProxy proxy{remote.endpointGuid, remote.unicastLocator, remote.is_reliable};
    for (std::size_t i = 0; i < count; ++i) {
      matchRemoteWriter(*readers[i], proxy);
    }
  }
}

bool SEDPAgent::addWriter(Writer &writer) {
  if (m_endpoints.sedpPubWriter == nullptr) {
    return true;
  }
  EntityKind_t writerKind = writer.m_attributes.endpointGuid.entityId.entityKind;
  if (writerKind == EntityKind_t::BUILD_IN_WRITER_WITH_KEY ||
      writerKind == EntityKind_t::BUILD_IN_WRITER_WITHOUT_KEY) {
    return true; // No need to announce builtin endpoints
  }

  std::lock_guard<std::recursive_mutex> lock(m_mutex);

  CdrSink sink{asWritableBytes(m_buffer, sizeof(m_buffer) / sizeof(m_buffer[0]))};
  CdrWriter microbuffer(sink);
  const uint16_t zero_options = 0;

  writeBytes(microbuffer, rtps::SMElement::SCHEME_PL_CDR_LE.data(),
             rtps::SMElement::SCHEME_PL_CDR_LE.size());
  microbuffer.write<uint16_t>(zero_options);
  writer.m_attributes.serializeInto(microbuffer);
  auto change = m_endpoints.sedpPubWriter->newChange(ChangeKind_t::ALIVE, m_buffer, sink.size());
  if (change == nullptr) {
    // The SEDP writer refused the announcement (its history is full of live
    // endpoints, see StatefulWriter::newChange): an unannounced writer is
    // undiscoverable, so the caller fails the creation.
    logger_.error("Could not announce writer '{}': the SEDP publications writer refused the change",
                  writer.m_attributes.topicName);
    return false;
  }
  writer.setSEDPSequenceNumber(change->sequenceNumber);
#if SEDP_VERBOSE
  SEDP_LOG("Added new change to sedpPubWriter.\n");
#endif
  // Only now give the new writer the remembered remote readers: a refused
  // announcement resets the writer, and the proxies would be gone with it.
  matchKnownRemoteEndpoints();
  return true;
}

template <typename A>
bool SEDPAgent::disposeEndpointInSEDPHistory(A *local_endpoint, Writer *sedp_writer) {
  return sedp_writer->removeFromHistory(local_endpoint->getSEDPSequenceNumber());
}

template <typename A>
bool SEDPAgent::announceEndpointDeletion(A *local_endpoint, Writer *sedp_endpoint) {
  CdrSink sink{asWritableBytes(m_buffer, sizeof(m_buffer) / sizeof(m_buffer[0]))};
  CdrWriter microbuffer(sink);

  microbuffer.write<uint16_t>(ParameterId::PID_KEY_HASH);
  microbuffer.write<uint16_t>(16);
  writeBytes(microbuffer, local_endpoint->m_attributes.endpointGuid.prefix.id.data(),
             sizeof(GuidPrefix_t::id));
  writeBytes(microbuffer, local_endpoint->m_attributes.endpointGuid.entityId.entityKey.data(), 3);
  microbuffer.write<uint8_t>(
      static_cast<uint8_t>(local_endpoint->m_attributes.endpointGuid.entityId.entityKind));

  microbuffer.write<uint16_t>(ParameterId::PID_STATUS_INFO);
  microbuffer.write<uint16_t>(static_cast<uint16_t>(4));
  microbuffer.write<uint8_t>(0);
  microbuffer.write<uint8_t>(0);
  microbuffer.write<uint8_t>(0);
  microbuffer.write<uint8_t>(3);

  // Sentinel to terminate inline qos
  microbuffer.write<uint16_t>(ParameterId::PID_SENTINEL);
  microbuffer.write<uint16_t>(0);

  // Sentinel to terminate serialized data
  microbuffer.write<uint16_t>(ParameterId::PID_SENTINEL);
  microbuffer.write<uint16_t>(0);

  auto ret = sedp_endpoint->newChange(ChangeKind_t::ALIVE, m_buffer, sink.size(), true, true);
  SEDP_LOG("Announcing endpoint delete, SN = {}.{}", (int)ret->sequenceNumber.low,
           (int)ret->sequenceNumber.high);
  return (ret != nullptr);
}

void SEDPAgent::jumppadTakeProxyOfDisposedReader(const Reader *reader, const WriterProxy &proxy,
                                                 void *arg) {
  auto agent = static_cast<SEDPAgent *>(arg);
  TopicDataCompressed topic_data(reader->m_attributes);
  topic_data.endpointGuid = proxy.remoteWriterGuid;
  topic_data.is_reliable = proxy.is_reliable;
  topic_data.multicastLocator.kind = LocatorKind_t::LOCATOR_KIND_INVALID;
  topic_data.unicastLocator = proxy.remoteLocator;
  agent->rememberRemoteWriter(topic_data); // already known, unless the registry was full then
}

void SEDPAgent::jumppadTakeProxyOfDisposedWriter(const Writer *writer, const ReaderProxy &proxy,
                                                 void *arg) {
  auto agent = static_cast<SEDPAgent *>(arg);
  TopicDataCompressed topic_data(writer->m_attributes);
  topic_data.endpointGuid = proxy.remoteReaderGuid;
  topic_data.is_reliable = proxy.is_reliable;
  topic_data.multicastLocator.kind = LocatorKind_t::LOCATOR_KIND_INVALID;
  topic_data.unicastLocator = proxy.remoteLocator;
  agent->rememberRemoteReader(topic_data);
}

bool SEDPAgent::deleteReader(Reader *reader) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  // Drop the announcement from the SEDP history. Not finding it there (it was
  // never stored, or an older engine overwrote it) is no reason to keep the
  // reader: the peers that matched it still get the dispose below, and the
  // local slot must be freed either way.
  if (!disposeEndpointInSEDPHistory(reader, m_endpoints.sedpSubWriter)) {
    logger_.warn("SEDP announcement of reader '{}' not in history, announcing its deletion anyway",
                 reader->m_attributes.topicName);
  }

  // Create Deletion Message [UD] and add to corret builtin endpoint
  if (!announceEndpointDeletion(reader, m_endpoints.sedpSubWriter)) {
    return false;
  }

  // The remote writers it matched stay known for a reader created later
  // (they are, unless the registry was full when they were announced).
  reader->dumpAllProxies(SEDPAgent::jumppadTakeProxyOfDisposedReader, this);

  return true;
}

bool SEDPAgent::deleteWriter(Writer *writer) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  if (!disposeEndpointInSEDPHistory(writer, m_endpoints.sedpPubWriter)) {
    logger_.warn("SEDP announcement of writer '{}' not in history, announcing its deletion anyway",
                 writer->m_attributes.topicName); // see deleteReader()
  }

  // Create Deletion Mesasge [UD] and add to corret builtin endpoint
  if (!announceEndpointDeletion(writer, m_endpoints.sedpPubWriter)) {
    return false;
  }

  // The remote readers it matched stay known for a writer created later.
  writer->dumpAllProxies(SEDPAgent::jumppadTakeProxyOfDisposedWriter, this);

  return true;
}

bool SEDPAgent::addReader(Reader &reader) {
  if (m_endpoints.sedpSubWriter == nullptr) {
    return true;
  }

  EntityKind_t readerKind = reader.m_attributes.endpointGuid.entityId.entityKind;
  if (readerKind == EntityKind_t::BUILD_IN_READER_WITH_KEY ||
      readerKind == EntityKind_t::BUILD_IN_READER_WITHOUT_KEY) {
    return true; // No need to announce builtin endpoints
  }

  std::lock_guard<std::recursive_mutex> lock(m_mutex);

  CdrSink sink{asWritableBytes(m_buffer, sizeof(m_buffer) / sizeof(m_buffer[0]))};
  CdrWriter microbuffer(sink);
  const uint16_t zero_options = 0;

  writeBytes(microbuffer, rtps::SMElement::SCHEME_PL_CDR_LE.data(),
             rtps::SMElement::SCHEME_PL_CDR_LE.size());
  microbuffer.write<uint16_t>(zero_options);
  reader.m_attributes.serializeInto(microbuffer);
  auto change = m_endpoints.sedpSubWriter->newChange(ChangeKind_t::ALIVE, m_buffer, sink.size());
  if (change == nullptr) {
    logger_.error(
        "Could not announce reader '{}': the SEDP subscriptions writer refused the change",
        reader.m_attributes.topicName); // see addWriter()
    return false;
  }
  reader.setSEDPSequenceNumber(change->sequenceNumber);
#if SEDP_VERBOSE
  SEDP_LOG("Added new change to sedpSubWriter.\n");
#endif
  matchKnownRemoteEndpoints(); // see addWriter()
  return true;
}

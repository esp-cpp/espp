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

#include "rtps/entities/Domain.hpp"
#include "rtps/utils/Log.hpp"
#include "rtps/utils/udpUtils.hpp"
#include <cassert>
#include <mutex>
#include <random>

#if defined(ESP_PLATFORM)
#include "esp_mac.h"
#endif

#if DOMAIN_VERBOSE && RTPS_GLOBAL_VERBOSE
#define DOMAIN_LOG(...) logger_.warn(__VA_ARGS__)
#else
#define DOMAIN_LOG(...)                                                                            \
  do {                                                                                             \
  } while (0)
#endif

using rtps::Domain;

Domain::Domain(const rtps::Ip4AddressBytes &localIpAddress)
    : espp::BaseComponent("RtpsDomain", espp::Logger::Verbosity::WARN)
    , m_defaultTransport(&Domain::datagramJumppad, this)
    , m_transport(&m_defaultTransport)
    , m_localIpAddress(localIpAddress) {
  m_transportSetupOk = initializeTransport();
}

Domain::Domain(rtps::EsppTransport &transport, const rtps::Ip4AddressBytes &localIpAddress)
    : espp::BaseComponent("RtpsDomain", espp::Logger::Verbosity::WARN)
    , m_defaultTransport(&Domain::datagramJumppad, this)
    , m_transport(&transport)
    , m_localIpAddress(localIpAddress) {
  m_transportSetupOk = initializeTransport();
}

bool Domain::initializeTransport() {
  assert(m_transport != nullptr);
  bool success = true;
  success =
      m_transport->ensureReceivePort(getUserMulticastPort(), /*is_multicast=*/true) && success;
  success =
      m_transport->ensureReceivePort(getBuiltInMulticastPort(), /*is_multicast=*/true) && success;
  success = m_transport->joinMultiCastGroup({239, 255, 0, 1}) && success;
  return success;
}

Domain::~Domain() { stop(); }

bool Domain::completeInit() {
  if (!m_transportSetupOk) {
    DOMAIN_LOG("Failed transport setup. Domain initialization aborted.");
    m_initComplete = false;
    return false;
  }

  m_initComplete = true;

  for (uint8_t slot = 0; slot < m_numParticipants; ++slot) {
    m_participants[slot].getSPDPAgent().start();
  }
  return m_initComplete;
}

void Domain::stop() {
  for (uint8_t slot = 0; slot < m_numParticipants; ++slot) {
    m_participants[slot].getSPDPAgent().stop();
  }
  // Stop receive dispatch + the worker pool BEFORE participants/writers are
  // torn down: queued jobs and in-flight datagram handlers reference them.
  m_transport->stop();
}

void Domain::receiveJumppad(void *callee, const PacketInfo &packet) {
  auto domain = static_cast<Domain *>(callee);
  domain->receiveCallback(packet);
}

void Domain::datagramJumppad(void *arg, const uint8_t *data, std::size_t size, Ip4Port_t localPort,
                             Ip4Port_t remotePort, const Ip4AddressBytes &remoteAddress) {
  auto *domain = static_cast<Domain *>(arg);

  PacketInfo packet;
  packet.destAddr = remoteAddress;
  packet.destPort = localPort;
  packet.srcPort = remotePort;
  if (size > 0 && data != nullptr) {
    packet.payload.assign(data, data + size);
  }
  // Process inline on the transport worker that delivered the datagram: the
  // reactor's one-shot arming already serializes per socket, replacing the
  // former queue + reader-worker indirection.
  domain->receiveCallback(packet);
}

void Domain::receiveCallback(const PacketInfo &packet) {
  if (packet.payload.empty()) {
    DOMAIN_LOG("Dropping packet without payload");
    return;
  }

  const uint8_t *payload = packet.payload.data();
  DataSize_t payload_size = static_cast<DataSize_t>(packet.payload.size());

  if (isMetaMultiCastPort(packet.destPort)) {
    // Pass to all
    DOMAIN_LOG("Domain: Multicast to port {}", packet.destPort);
    for (uint8_t slot = 0; slot < m_numParticipants; ++slot) {
      m_participants[slot].newMessage(payload, payload_size);
    }
    // First Check if UserTraffic Multicast
  } else if (isUserMultiCastPort(packet.destPort)) {
    // Pass to Participant with assigned Multicast Adress (Port ist everytime
    // the same)
    DOMAIN_LOG("Domain: Got user multicast message on port {}", packet.destPort);
    for (uint8_t slot = 0; slot < m_numParticipants; ++slot) {
      if (m_participants[slot].hasReaderWithMulticastLocator(packet.destAddr)) {
        DOMAIN_LOG("Domain: Forward Multicast only to Participant: {}", slot);
        m_participants[slot].newMessage(payload, payload_size);
      }
    }
  } else {
    // Pass to addressed one only (Unicast, by Port)
    ParticipantId_t id =
        getParticipantIdFromUnicastPort(packet.destPort, isUserPort(packet.destPort));
    if (id != PARTICIPANT_ID_INVALID) {
      DOMAIN_LOG("Domain: Got unicast message on port {}", packet.destPort);
      // Ids may be non-contiguous after port probing, so look the participant
      // up by id rather than indexing slots arithmetically.
      Participant *target = findParticipantById(id);
      if (target != nullptr) {
        target->newMessage(payload, payload_size);
      } else {
        DOMAIN_LOG("Domain: No local participant with id {}.", id);
      }
    } else {
      DOMAIN_LOG("Domain: Got message to port {}: no matching participant", packet.destPort);
    }
  }
}

rtps::Participant *Domain::createParticipant() {

  DOMAIN_LOG("Domain: Creating new participant.");

  if (m_initComplete || m_participants.size() <= m_numParticipants) {
    return nullptr;
  }

  // Probe for a participant id whose unicast ports are free on this host.
  // Unicast channels bind with reuse disabled, so an id already used by
  // another process fails loudly here and we advance to the next id - the
  // same strategy FastDDS uses. Ids may therefore skip values; slots are
  // tracked separately (m_numParticipants).
  ParticipantId_t candidate = m_nextParticipantId;
  const ParticipantId_t last_candidate = m_nextParticipantId + PARTICIPANT_PORT_PROBE_LIMIT;
  bool ports_ok = false;
  for (; candidate < last_candidate; ++candidate) {
    if (!m_transport->ensureReceivePort(getUserUnicastPort(candidate), /*is_multicast=*/false)) {
      continue;
    }
    if (m_transport->ensureReceivePort(getBuiltInUnicastPort(candidate), /*is_multicast=*/false)) {
      ports_ok = true;
      break;
    }
    // unwind the half-registered probe before trying the next id
    m_transport->releaseReceivePort(getUserUnicastPort(candidate));
  }
  if (!ports_ok) {
    DOMAIN_LOG("No free unicast ports for a new participant (probed {} ids from {})",
               PARTICIPANT_PORT_PROBE_LIMIT, m_nextParticipantId);
    m_transportSetupOk = false;
    return nullptr;
  }

  auto &entry = m_participants[m_numParticipants];
  ++m_numParticipants;
  entry.reuse(generateGuidPrefix(candidate), candidate, m_localIpAddress);
  createBuiltinWritersAndReaders(entry);
  m_nextParticipantId = static_cast<ParticipantId_t>(candidate + 1);
  return &entry;
}

void Domain::createBuiltinWritersAndReaders(Participant &part) {
  // SPDP
  StatelessWriter *spdpWriter =
      getNextUnusedEndpoint<decltype(m_statelessWriters), StatelessWriter>(m_statelessWriters);
  StatelessReader *spdpReader =
      getNextUnusedEndpoint<decltype(m_statelessReaders), StatelessReader>(m_statelessReaders);

  TopicData spdpWriterAttributes;
  spdpWriterAttributes.topicName[0] = '\0';
  spdpWriterAttributes.typeName[0] = '\0';
  spdpWriterAttributes.reliabilityKind = ReliabilityKind_t::BEST_EFFORT;
  spdpWriterAttributes.durabilityKind = DurabilityKind_t::TRANSIENT_LOCAL;
  spdpWriterAttributes.endpointGuid.prefix = part.m_guidPrefix;
  spdpWriterAttributes.endpointGuid.entityId = ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER;
  spdpWriterAttributes.unicastLocator = getBuiltInMulticastLocator();

  spdpWriter->init(spdpWriterAttributes, TopicKind_t::WITH_KEY, *m_transport);
  spdpWriter->addNewMatchedReader(
      ReaderProxy{{part.m_guidPrefix, ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER},
                  LocatorIPv4(getBuiltInMulticastLocator()),
                  false});

  TopicData spdpReaderAttributes;
  spdpReaderAttributes.endpointGuid = {part.m_guidPrefix, ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER};
  spdpReader->init(spdpReaderAttributes);

  // SEDP

  // Prepare attributes
  TopicData sedpAttributes;
  sedpAttributes.topicName[0] = '\0';
  sedpAttributes.typeName[0] = '\0';
  sedpAttributes.reliabilityKind = ReliabilityKind_t::RELIABLE;
  sedpAttributes.durabilityKind = DurabilityKind_t::TRANSIENT_LOCAL;
  sedpAttributes.endpointGuid.prefix = part.m_guidPrefix;
  sedpAttributes.unicastLocator = getBuiltInUnicastLocator(part.m_participantId, m_localIpAddress);

  // READER
  StatefulReader *sedpPubReader =
      getNextUnusedEndpoint<decltype(m_statefulReaders), StatefulReader>(m_statefulReaders);
  sedpAttributes.endpointGuid.entityId = ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER;
  sedpPubReader->init(sedpAttributes, *m_transport);

  StatefulReader *sedpSubReader =
      getNextUnusedEndpoint<decltype(m_statefulReaders), StatefulReader>(m_statefulReaders);
  sedpAttributes.endpointGuid.entityId = ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER;
  sedpSubReader->init(sedpAttributes, *m_transport);

  // WRITER
  StatefulWriter *sedpPubWriter =
      getNextUnusedEndpoint<decltype(m_statefulWriters), StatefulWriter>(m_statefulWriters);
  sedpAttributes.endpointGuid.entityId = ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER;
  sedpPubWriter->init(sedpAttributes, TopicKind_t::NO_KEY, *m_transport);

  StatefulWriter *sedpSubWriter =
      getNextUnusedEndpoint<decltype(m_statefulWriters), StatefulWriter>(m_statefulWriters);
  sedpAttributes.endpointGuid.entityId = ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER;
  sedpSubWriter->init(sedpAttributes, TopicKind_t::NO_KEY, *m_transport);

  // COLLECT
  BuiltInEndpoints endpoints{};
  endpoints.spdpWriter = spdpWriter;
  endpoints.spdpReader = spdpReader;
  endpoints.sedpPubReader = sedpPubReader;
  endpoints.sedpSubReader = sedpSubReader;
  endpoints.sedpPubWriter = sedpPubWriter;
  endpoints.sedpSubWriter = sedpSubWriter;

  part.addBuiltInEndpoints(endpoints);
}

rtps::Participant *Domain::findParticipantById(ParticipantId_t id) {
  for (uint8_t slot = 0; slot < m_numParticipants; ++slot) {
    if (m_participants[slot].m_participantId == id) {
      return &m_participants[slot];
    }
  }
  return nullptr;
}

void Domain::registerMulticastPort(FullLengthLocator mcastLocator) {
  if (mcastLocator.kind == LocatorKind_t::LOCATOR_KIND_UDPv4) {
    m_transportSetupOk =
        m_transport->ensureReceivePort(mcastLocator.getLocatorPort(), /*is_multicast=*/true) &&
        m_transportSetupOk;
  }
}

rtps::Reader *Domain::readerExists(Participant &part, const char *topicName, const char *typeName,
                                   bool reliable) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  if (reliable) {
    for (unsigned int i = 0; i < m_statefulReaders.size(); i++) {
      if (m_statefulReaders[i].isInitialized()) {
        if (strncmp(m_statefulReaders[i].m_attributes.topicName, topicName,
                    Config::MAX_TYPENAME_LENGTH) != 0) {
          continue;
        }

        if (strncmp(m_statefulReaders[i].m_attributes.typeName, typeName,
                    Config::MAX_TYPENAME_LENGTH) != 0) {
          continue;
        }

        DOMAIN_LOG("StatefulReader exists already [{}, {}]", topicName, typeName);

        return &m_statefulReaders[i];
      }
    }
  } else {
    for (unsigned int i = 0; i < m_statelessReaders.size(); i++) {
      if (m_statelessReaders[i].isInitialized()) {
        if (strncmp(m_statelessReaders[i].m_attributes.topicName, topicName,
                    Config::MAX_TYPENAME_LENGTH) != 0) {
          continue;
        }

        if (strncmp(m_statelessReaders[i].m_attributes.typeName, typeName,
                    Config::MAX_TYPENAME_LENGTH) != 0) {
          continue;
        }

        DOMAIN_LOG("StatelessReader exists [{}, {}]", topicName, typeName);

        return &m_statelessReaders[i];
      }
    }
  }

  return nullptr;
}

rtps::Writer *Domain::writerExists(Participant &part, const char *topicName, const char *typeName,
                                   bool reliable) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  if (reliable) {
    for (unsigned int i = 0; i < m_statefulWriters.size(); i++) {
      if (m_statefulWriters[i].isInitialized()) {
        if (strncmp(m_statefulWriters[i].m_attributes.topicName, topicName,
                    Config::MAX_TYPENAME_LENGTH) != 0) {
          continue;
        }

        if (strncmp(m_statefulWriters[i].m_attributes.typeName, typeName,
                    Config::MAX_TYPENAME_LENGTH) != 0) {
          continue;
        }

        DOMAIN_LOG("StatefulWriter exists [{}, {}]", topicName, typeName);

        return &m_statefulWriters[i];
      }
    }
  } else {
    for (unsigned int i = 0; i < m_statelessWriters.size(); i++) {
      if (m_statelessWriters[i].isInitialized()) {
        if (strncmp(m_statelessWriters[i].m_attributes.topicName, topicName,
                    Config::MAX_TYPENAME_LENGTH) != 0) {
          continue;
        }

        if (strncmp(m_statelessWriters[i].m_attributes.typeName, typeName,
                    Config::MAX_TYPENAME_LENGTH) != 0) {
          continue;
        }

        DOMAIN_LOG("StatelessWriter exists [{}, {}]", topicName, typeName);

        return &m_statelessWriters[i];
      }
    }
  }

  return nullptr;
}

rtps::Writer *Domain::createWriter(Participant &part, const char *topicName, const char *typeName,
                                   bool reliable, bool enforceUnicast) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  StatelessWriter *statelessWriter =
      getNextUnusedEndpoint<decltype(m_statelessWriters), StatelessWriter>(m_statelessWriters);
  StatefulWriter *statefulWriter =
      getNextUnusedEndpoint<decltype(m_statefulWriters), StatefulWriter>(m_statefulWriters);

  // Check if there is enough capacity for more writers
  if ((reliable && statefulWriter == nullptr) || (!reliable && statelessWriter == nullptr) ||
      part.isWritersFull()) {

    DOMAIN_LOG("No Writer created. Max Number of Writers reached.");

    return nullptr;
  }

  // TODO Distinguish WithKey and NoKey (Also changes EntityKind)
  TopicData attributes;

  if (strlen(topicName) >= Config::MAX_TOPICNAME_LENGTH ||
      strlen(typeName) >= Config::MAX_TYPENAME_LENGTH) {
    return nullptr;
  }
  strncpy(attributes.topicName, topicName, Config::MAX_TOPICNAME_LENGTH);
  strncpy(attributes.typeName, typeName, Config::MAX_TYPENAME_LENGTH);
  attributes.topicName[Config::MAX_TOPICNAME_LENGTH - 1] = '\0';
  attributes.typeName[Config::MAX_TYPENAME_LENGTH - 1] = '\0';
  attributes.endpointGuid.prefix = part.m_guidPrefix;
  attributes.endpointGuid.entityId = {part.getNextUserEntityKey(),
                                      EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY};
  attributes.unicastLocator = getUserUnicastLocator(part.m_participantId, m_localIpAddress);
  attributes.durabilityKind = DurabilityKind_t::TRANSIENT_LOCAL;

  DOMAIN_LOG("Creating writer[{}, {}]", topicName, typeName);

  if (reliable) {
    attributes.reliabilityKind = ReliabilityKind_t::RELIABLE;

    if (!statefulWriter->init(attributes, TopicKind_t::NO_KEY, *m_transport, enforceUnicast)) {
      DOMAIN_LOG("StatefulWriter init failed.");
      return nullptr;
    }

    if (!part.addWriter(statefulWriter)) {
      return nullptr;
    }
    return statefulWriter;
  } else {
    attributes.reliabilityKind = ReliabilityKind_t::BEST_EFFORT;

    if (!statelessWriter->init(attributes, TopicKind_t::NO_KEY, *m_transport, enforceUnicast)) {
      DOMAIN_LOG("StatelessWriter init failed.");
      return nullptr;
    }

    if (!part.addWriter(statelessWriter)) {
      return nullptr;
    }
    return statelessWriter;
  }
}

rtps::Reader *Domain::createReader(Participant &part, const char *topicName, const char *typeName,
                                   bool reliable, rtps::Ip4AddressBytes mcastaddress) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  StatelessReader *statelessReader =
      getNextUnusedEndpoint<decltype(m_statelessReaders), StatelessReader>(m_statelessReaders);
  StatefulReader *statefulReader =
      getNextUnusedEndpoint<decltype(m_statefulReaders), StatefulReader>(m_statefulReaders);

  if ((reliable && statefulReader == nullptr) || (!reliable && statelessReader == nullptr) ||
      part.isReadersFull()) {

    DOMAIN_LOG("No Reader created. Max Number of Readers reached.");

    return nullptr;
  }

  // TODO Distinguish WithKey and NoKey (Also changes EntityKind)
  TopicData attributes;

  if (strlen(topicName) >= Config::MAX_TOPICNAME_LENGTH ||
      strlen(typeName) >= Config::MAX_TYPENAME_LENGTH) {
    return nullptr;
  }
  strncpy(attributes.topicName, topicName, Config::MAX_TOPICNAME_LENGTH);
  strncpy(attributes.typeName, typeName, Config::MAX_TYPENAME_LENGTH);
  attributes.topicName[Config::MAX_TOPICNAME_LENGTH - 1] = '\0';
  attributes.typeName[Config::MAX_TYPENAME_LENGTH - 1] = '\0';
  attributes.endpointGuid.prefix = part.m_guidPrefix;
  attributes.endpointGuid.entityId = {part.getNextUserEntityKey(),
                                      EntityKind_t::USER_DEFINED_READER_WITHOUT_KEY};
  attributes.unicastLocator = getUserUnicastLocator(part.m_participantId, m_localIpAddress);
  if (!isZeroAddress(mcastaddress)) {
    if (isMulticastAddress(mcastaddress)) {
      attributes.multicastLocator = rtps::FullLengthLocator::createUDPv4Locator(
          mcastaddress[0], mcastaddress[1], mcastaddress[2], mcastaddress[3],
          getUserMulticastPort());
      m_transportSetupOk =
          m_transport->joinMultiCastGroup(
              {attributes.multicastLocator.address[12], attributes.multicastLocator.address[13],
               attributes.multicastLocator.address[14], attributes.multicastLocator.address[15]}) &&
          m_transportSetupOk;
      registerMulticastPort(attributes.multicastLocator);

      DOMAIN_LOG("Multicast enabled!");

    } else {

      DOMAIN_LOG("This is not a Multicastaddress!");
    }
  }
  attributes.durabilityKind = DurabilityKind_t::VOLATILE;

  DOMAIN_LOG("Creating reader[{}, {}]", topicName, typeName);

  if (reliable) {

    attributes.reliabilityKind = ReliabilityKind_t::RELIABLE;

    statefulReader->init(attributes, *m_transport);

    if (!part.addReader(statefulReader)) {
      DOMAIN_LOG("Failed to add reader to participant.");

      return nullptr;
    }
    return statefulReader;
  } else {

    attributes.reliabilityKind = ReliabilityKind_t::BEST_EFFORT;

    statelessReader->init(attributes);

    if (!part.addReader(statelessReader)) {
      return nullptr;
    }
    return statelessReader;
  }
}

bool rtps::Domain::deleteReader(Participant &part, Reader *reader) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  if (reader == nullptr || !reader->isInitialized()) {
    return false;
  }
  if (!part.deleteReader(reader)) {
    return false;
  }

  reader->reset();
  return true;
}

bool rtps::Domain::deleteWriter(Participant &part, Writer *writer) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  if (writer == nullptr || !writer->isInitialized()) {
    return false;
  }
  if (!part.deleteWriter(writer)) {
    return false;
  }

  writer->reset();
  return true;
}

void rtps::Domain::printInfo() {
  for (unsigned int i = 0; i < m_participants.size(); i++) {
    DOMAIN_LOG("Participant {}", i);
    m_participants[i].printInfo();
  }
}

rtps::GuidPrefix_t Domain::generateGuidPrefix(ParticipantId_t id) const {
  GuidPrefix_t prefix;
#if defined(ESP_PLATFORM)
  uint8_t mac[6] = {0};
  esp_err_t mac_err = esp_read_mac(mac, ESP_MAC_ETH);
  if (mac_err != ESP_OK) {
    mac_err = esp_read_mac(mac, ESP_MAC_WIFI_STA);
  }
  if (mac_err == ESP_OK) {
    // Make participant GUID unique per board while keeping a stable layout.
    prefix.id[0] = mac[0];
    prefix.id[1] = mac[1];
    prefix.id[2] = mac[2];
    prefix.id[3] = mac[3];
    prefix.id[4] = mac[4];
    prefix.id[5] = mac[5];
    prefix.id[6] = static_cast<uint8_t>(id);
    prefix.id[7] = Config::VENDOR_ID.vendorId[0];
    prefix.id[8] = Config::VENDOR_ID.vendorId[1];
    prefix.id[9] = Config::DOMAIN_ID;
    prefix.id[10] = 0xA5;
    prefix.id[11] = 0x5A;
    return prefix;
  }
#endif

  if (Config::BASE_GUID_PREFIX == GUID_RANDOM) {
    // Use OS entropy, not rand(): unseeded rand() yields the identical sequence
    // in every process, so two host processes would share a GUID prefix and drop
    // each other's packets as their own.
    std::random_device rd;
    for (unsigned int i = 0; i < prefix.id.size(); i++) {
      prefix.id[i] = static_cast<uint8_t>(rd());
    }
  } else {
    for (unsigned int i = 0; i < rtps::Config::BASE_GUID_PREFIX.id.size(); i++) {
      prefix.id[i] = Config::BASE_GUID_PREFIX.id[i];
    }
  }
  // Stamp the participant id (and vendor/domain, mirroring the ESP path) so
  // multiple participants in one process always get distinct GUID prefixes -
  // identical prefixes make peers discard each other's SPDP as "own message".
  prefix.id[6] = static_cast<uint8_t>(id);
  prefix.id[7] = Config::VENDOR_ID.vendorId[0];
  prefix.id[8] = Config::VENDOR_ID.vendorId[1];
  prefix.id[9] = Config::DOMAIN_ID;
  return prefix;
}

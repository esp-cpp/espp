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
#include <algorithm>
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

Domain::Domain(const rtps::Ip4AddressBytes &localIpAddress, const DomainConfig &config)
    : espp::BaseComponent("RtpsDomain", espp::Logger::Verbosity::WARN)
    , m_defaultTransport(&Domain::datagramJumppad, this)
    , m_transport(&m_defaultTransport)
    , m_localIpAddress(localIpAddress)
    , m_config(config) {
  m_transportSetupOk = initializeTransport();
}

Domain::Domain(rtps::EsppTransport &transport, const rtps::Ip4AddressBytes &localIpAddress,
               const DomainConfig &config)
    : espp::BaseComponent("RtpsDomain", espp::Logger::Verbosity::WARN)
    , m_defaultTransport(&Domain::datagramJumppad, this)
    , m_transport(&transport)
    , m_localIpAddress(localIpAddress)
    , m_config(config) {
  m_transportSetupOk = initializeTransport();
}

bool Domain::initializeTransport() {
  assert(m_transport != nullptr);
  // Metatraffic (SPDP discovery multicast) is registered at the configured
  // metatraffic band (High by default) so discovery dispatch overtakes queued
  // user-traffic handling; user multicast runs at the user-traffic band.
  bool success = m_transport->ensureReceivePort(getUserMulticastPort(), /*is_multicast=*/true,
                                                {.band = m_config.user_traffic_band});
  success = m_transport->ensureReceivePort(getBuiltInMulticastPort(), /*is_multicast=*/true,
                                           {.band = m_config.metatraffic_band}) &&
            success;
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

  // Start the protocol scheduler: one task drives every participant's SPDP
  // announcements and every stateful writer's heartbeat cadence.
  m_nextSpdpAnnounce = std::chrono::steady_clock::now();
  for (auto &writer : m_statefulWriters) {
    writer.setProtocolNudge([this]() { nudgeProtocol(); });
  }
  espp::Task::Config task_config;
  task_config.callback = [this](std::mutex &m, std::condition_variable &cv, bool &notified) {
    return protocolLoop(m, cv, notified);
  };
  task_config.task_config.name = "rtps_protocol";
  task_config.task_config.stack_size_bytes =
      Config::SPDP_WRITER_STACKSIZE > 4096 ? Config::SPDP_WRITER_STACKSIZE : 4096;
  task_config.task_config.priority = Config::SPDP_WRITER_PRIO;
  task_config.log_level = espp::Logger::Verbosity::WARN;
  m_protocolTask = espp::Task::make_unique(task_config);
  (void)m_protocolTask->start();

  for (uint8_t slot = 0; slot < m_numParticipants; ++slot) {
    m_participants[slot].getSPDPAgent().start();
  }
  return m_initComplete;
}

void Domain::stop() {
  if (m_protocolTask) {
    m_protocolStopRequested = true;
    nudgeProtocol();        // wake the loop so it observes the stop flag
    m_protocolTask->stop(); // returns promptly
    // Retract the published task synchronization pointers BEFORE the task (and
    // the mutex/cv they point into) is destroyed: a late nudge from a racing
    // publisher then no-ops instead of locking a destroyed mutex.
    {
      std::lock_guard<std::mutex> nudge_lock(m_protocolNudgeMutex);
      m_protocolMutex = nullptr;
      m_protocolCv = nullptr;
      m_protocolNotified = nullptr;
    }
    m_protocolTask.reset();
    m_protocolStopRequested = false;
  }
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

bool Domain::protocolLoop(std::mutex &m, std::condition_variable &cv, bool &notified) {
  using clock = std::chrono::steady_clock;
  const auto now = clock::now();

  // SPDP announcements for every running participant, at the configured cadence.
  if (now >= m_nextSpdpAnnounce) {
    for (uint8_t slot = 0; slot < m_numParticipants; ++slot) {
      auto &agent = m_participants[slot].getSPDPAgent();
      if (agent.isRunning()) {
        agent.announce();
      }
    }
    m_nextSpdpAnnounce = now + std::chrono::milliseconds(Config::SPDP_RESEND_PERIOD_MS);
  }

  // Heartbeat ticks for every (initialized) stateful writer; each returns its
  // next deadline. Uninitialized writers report a far-future deadline.
  auto next_deadline = m_nextSpdpAnnounce;
  for (auto &writer : m_statefulWriters) {
    const auto writer_deadline = writer.heartbeatTick(now);
    next_deadline = std::min(next_deadline, writer_deadline);
  }

  // Publish the task's synchronization objects for nudgeProtocol() under the
  // nudge mutex: nudges arrive from arbitrary publisher threads, so unguarded
  // pointer writes here would race their reads (and re-writing every
  // iteration raced even after the first). The values are stable for the
  // task's lifetime, so this is one uncontended lock per tick.
  {
    std::lock_guard<std::mutex> nudge_lock(m_protocolNudgeMutex);
    m_protocolMutex = &m;
    m_protocolCv = &cv;
    m_protocolNotified = &notified;
  }
  // Sleep until the earliest deadline; a publish on a reliable writer (or
  // stop()) notifies the cv to re-evaluate immediately.
  std::unique_lock<std::mutex> lock(m);
  cv.wait_until(lock, next_deadline, [&notified] { return notified; });
  if (notified) {
    notified = false;
    // Both Domain::stop() and a heartbeat nudge arrive via this cv; the
    // explicit flag (set before the stop notification) disambiguates. A nudge
    // simply re-evaluates deadlines on the next iteration.
    if (m_protocolStopRequested.load()) {
      return true; // stop requested
    }
  }
  return m_protocolStopRequested.load(); // keep running unless stopping
}

void Domain::nudgeProtocol() {
  // Read the published pointers under the nudge mutex (see the member doc):
  // this synchronizes with protocolLoop()'s publication and with stop()'s
  // nulling, so a nudge racing either is a safe no-op rather than a torn read
  // or a use-after-free of the destroyed task's mutex/cv.
  std::lock_guard<std::mutex> nudge_lock(m_protocolNudgeMutex);
  if (m_protocolMutex != nullptr && m_protocolCv != nullptr && m_protocolNotified != nullptr) {
    std::lock_guard<std::mutex> lock(*m_protocolMutex);
    *m_protocolNotified = true;
    m_protocolCv->notify_all();
  }
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
  } else if (Participant *dedicated = findParticipantByDedicatedPort(packet.destPort);
             dedicated != nullptr) {
    // A dedicated (per-endpoint) unicast port: route straight to the owning
    // participant; the engine's MessageReceiver demuxes by entity id, so the
    // local port the datagram arrived on is otherwise irrelevant.
    DOMAIN_LOG("Domain: Got message on dedicated endpoint port {}", packet.destPort);
    dedicated->newMessage(payload, payload_size);
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
    if (!m_transport->ensureReceivePort(getUserUnicastPort(candidate), /*is_multicast=*/false,
                                        {.band = m_config.user_traffic_band})) {
      continue;
    }
    // SEDP unicast is metatraffic: keep discovery dispatch above user traffic.
    if (m_transport->ensureReceivePort(getBuiltInUnicastPort(candidate), /*is_multicast=*/false,
                                       {.band = m_config.metatraffic_band})) {
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
  if (!createBuiltinWritersAndReaders(entry)) {
    // Pool exhaustion (see createBuiltinWritersAndReaders): unwind the slot
    // and the probed ports so the failure is clean and the caller gets a
    // truthful nullptr instead of a participant missing its discovery
    // endpoints (or a crash).
    --m_numParticipants;
    m_transport->releaseReceivePort(getUserUnicastPort(candidate));
    m_transport->releaseReceivePort(getBuiltInUnicastPort(candidate));
    return nullptr;
  }
  m_nextParticipantId = static_cast<ParticipantId_t>(candidate + 1);
  return &entry;
}

bool Domain::createBuiltinWritersAndReaders(Participant &part) {
  // Every allocation below is null-checked: the pools are GLOBAL, so a limits
  // override (or profile) sized for fewer participants than
  // MAX_NUM_PARTICIPANTS legitimately runs out here - that must fail the
  // participant cleanly, not crash. Allocation and init() interleave (the
  // allocator returns the first UNINITIALIZED slot, so a second allocation
  // from the same pool must come after the first one's init()); on a failure
  // partway, the endpoints this invocation already initialized are reset() so
  // nothing leaks from the pools.
  StatelessWriter *spdpWriter = nullptr;
  StatelessReader *spdpReader = nullptr;
  StatefulReader *sedpPubReader = nullptr;
  StatefulReader *sedpSubReader = nullptr;
  StatefulWriter *sedpPubWriter = nullptr;
  const auto fail = [&](const char *what) {
    DOMAIN_LOG("Builtin endpoint pool exhausted creating participant ({}): each participant "
               "consumes 1 stateless writer/reader and 2 stateful writers/readers from the "
               "GLOBAL pools - raise the NUM_STATELESS_*/NUM_STATEFUL_* limits or lower "
               "MAX_NUM_PARTICIPANTS",
               what);
    (void)what;
    if (spdpWriter != nullptr) {
      spdpWriter->reset();
    }
    if (spdpReader != nullptr) {
      spdpReader->reset();
    }
    if (sedpPubReader != nullptr) {
      sedpPubReader->reset();
    }
    if (sedpSubReader != nullptr) {
      sedpSubReader->reset();
    }
    if (sedpPubWriter != nullptr) {
      sedpPubWriter->reset();
    }
    return false;
  };

  // SPDP
  spdpWriter =
      getNextUnusedEndpoint<decltype(m_statelessWriters), StatelessWriter>(m_statelessWriters);
  if (spdpWriter == nullptr) {
    return fail("stateless writer for SPDP");
  }
  spdpReader =
      getNextUnusedEndpoint<decltype(m_statelessReaders), StatelessReader>(m_statelessReaders);
  if (spdpReader == nullptr) {
    return fail("stateless reader for SPDP");
  }

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
  sedpPubReader =
      getNextUnusedEndpoint<decltype(m_statefulReaders), StatefulReader>(m_statefulReaders);
  if (sedpPubReader == nullptr) {
    return fail("stateful reader for SEDP publications");
  }
  sedpAttributes.endpointGuid.entityId = ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER;
  sedpPubReader->init(sedpAttributes, *m_transport);

  sedpSubReader =
      getNextUnusedEndpoint<decltype(m_statefulReaders), StatefulReader>(m_statefulReaders);
  if (sedpSubReader == nullptr) {
    return fail("stateful reader for SEDP subscriptions");
  }
  sedpAttributes.endpointGuid.entityId = ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER;
  sedpSubReader->init(sedpAttributes, *m_transport);

  // WRITER
  sedpPubWriter =
      getNextUnusedEndpoint<decltype(m_statefulWriters), StatefulWriter>(m_statefulWriters);
  if (sedpPubWriter == nullptr) {
    return fail("stateful writer for SEDP publications");
  }
  sedpAttributes.endpointGuid.entityId = ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER;
  sedpPubWriter->init(sedpAttributes, TopicKind_t::NO_KEY, *m_transport);

  StatefulWriter *sedpSubWriter =
      getNextUnusedEndpoint<decltype(m_statefulWriters), StatefulWriter>(m_statefulWriters);
  if (sedpSubWriter == nullptr) {
    return fail("stateful writer for SEDP subscriptions");
  }
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
  return true;
}

rtps::Participant *Domain::findParticipantById(ParticipantId_t id) {
  for (uint8_t slot = 0; slot < m_numParticipants; ++slot) {
    if (m_participants[slot].m_participantId == id) {
      return &m_participants[slot];
    }
  }
  return nullptr;
}

rtps::Participant *Domain::findParticipantByDedicatedPort(Ip4Port_t port) {
  // Receive-path lookup: guarded by the registry's own small mutex, NOT
  // m_mutex - an API caller holding m_mutex (create/delete) must never be
  // able to stall the receive workers (see m_dedicatedPorts).
  std::lock_guard<std::mutex> lock(m_dedicatedPortsMutex);
  const auto it = std::find_if(m_dedicatedPorts.begin(), m_dedicatedPorts.end(),
                               [port](const DedicatedPort &entry) { return entry.port == port; });
  return (it != m_dedicatedPorts.end()) ? it->participant : nullptr;
}

rtps::Ip4Port_t Domain::allocateDedicatedEndpointPort(Participant &part, espp::QosBand band,
                                                      const std::optional<espp::Dscp> &dscp) {
  // Caller holds m_mutex (createWriter/createReader).
  if (!m_config.enable_dedicated_endpoint_ports) {
    return 0;
  }
  // The ration must be a TRUE fd bound: count both the active dedicated
  // ports AND released sockets whose fd is still open awaiting the reactor's
  // removal completion (retired sockets close promptly, but a handler in
  // flight defers the close - without counting them, churn could push the
  // real fd usage past the cap).
  std::size_t active = 0;
  {
    std::lock_guard<std::mutex> registry_lock(m_dedicatedPortsMutex);
    active = m_dedicatedPorts.size();
  }
  const std::size_t retired = m_transport->retiredSocketCount();
  if (active + retired >= m_config.max_prioritized_endpoint_ports) {
    logger_.warn("Dedicated endpoint port ration exhausted ({} active + {} retired, cap {}); "
                 "falling back to the shared user-unicast port",
                 active, retired,
                 static_cast<unsigned int>(m_config.max_prioritized_endpoint_ports));
    return 0;
  }
  const Ip4Port_t base = 7400 + 250 * Config::DOMAIN_ID + DEDICATED_PORT_OFFSET;
  const uint16_t first_offset = m_nextDedicatedPortOffset;
  uint16_t probed = 0;
  for (uint16_t probe = 0; probe < DEDICATED_PORT_PROBE_LIMIT; ++probe) {
    const uint16_t offset = first_offset + probe;
    if (DEDICATED_PORT_OFFSET + offset > 249) {
      break; // stay inside this domain's 250-port block
    }
    ++probed;
    const Ip4Port_t port = base + offset;
    // Reuse-disabled unicast bind: a port taken by another process fails
    // loudly here and the next candidate is probed (same strategy as the
    // participant-id port probe above).
    if (m_transport->ensureReceivePort(port, /*is_multicast=*/false,
                                       {.band = band, .dscp = dscp})) {
      m_nextDedicatedPortOffset = offset + 1;
      std::lock_guard<std::mutex> registry_lock(m_dedicatedPortsMutex);
      m_dedicatedPorts.push_back(DedicatedPort{port, &part});
      return port;
    }
  }
  // Advance PAST the failed window so the next allocation probes fresh ports;
  // without this a fully-occupied window (e.g. ports taken by another
  // process) would be retried forever and dedicated allocation would be
  // permanently stuck even though later ports in the 100..249 block are free.
  m_nextDedicatedPortOffset = first_offset + probed;
  logger_.warn("No free dedicated endpoint port (probed {} from offset {}; next allocation "
               "starts at offset {}); falling back to the shared user-unicast port",
               probed, first_offset, m_nextDedicatedPortOffset);
  return 0;
}

void Domain::releaseDedicatedEndpointPort(Ip4Port_t port) {
  // Caller holds m_mutex (deleteWriter/deleteReader); the registry has its
  // own mutex, held only for the erase (never across the transport call).
  if (port == 0) {
    return;
  }
  bool releasing = false;
  {
    std::lock_guard<std::mutex> registry_lock(m_dedicatedPortsMutex);
    const auto it = std::find_if(m_dedicatedPorts.begin(), m_dedicatedPorts.end(),
                                 [port](const DedicatedPort &entry) { return entry.port == port; });
    if (it != m_dedicatedPorts.end()) {
      m_dedicatedPorts.erase(it);
      releasing = true;
    }
  }
  if (releasing) {
    m_transport->releaseReceivePort(port);
  }
}

void Domain::applyEndpointOptions(Participant &part, TopicData &attributes,
                                  const EndpointOptions &options) {
  attributes.band = options.band;
  attributes.dscp = options.dscp;
  // A non-default band (or a DSCP marking, which is per-socket) requests a
  // dedicated unicast port. On success the endpoint's SEDP announcement
  // carries the dedicated port as its per-endpoint unicast locator (a standard
  // DDS parameter - PID_UNICAST_LOCATOR - so FastDDS/ROS 2 peers send this
  // endpoint's traffic there), and the endpoint also SENDS from that socket
  // (m_srcPort follows the unicast locator), so the DSCP marking applies to
  // its outgoing traffic. On failure (disabled / ration exhausted / no free
  // port) the endpoint keeps the shared user-unicast locator; a higher layer
  // may then apply deferred banded dispatch (see the espp facade).
  if (options.band == espp::QosBand::Normal && !options.dscp.has_value()) {
    return;
  }
  const Ip4Port_t port = allocateDedicatedEndpointPort(part, options.band, options.dscp);
  if (port == 0) {
    return;
  }
  attributes.unicastLocator = FullLengthLocator::createUDPv4Locator(
      m_localIpAddress[0], m_localIpAddress[1], m_localIpAddress[2], m_localIpAddress[3], port);
  attributes.hasDedicatedPort = true;
  DOMAIN_LOG("Granted dedicated endpoint port {} (band {})", port, static_cast<int>(options.band));
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
                                   bool reliable, bool enforceUnicast,
                                   const EndpointOptions &options) {
  // Validate the DSCP up front: an out-of-range code point can never be
  // applied (Socket::set_dscp rejects > 63), so probing would burn dedicated
  // -port offsets on binds that fail their marking and then silently fall
  // back without the requested behavior. Reject the writer instead so the
  // misconfiguration is an explicit endpoint-creation error.
  if (options.dscp.has_value() && static_cast<uint8_t>(options.dscp.value()) > 63) {
    logger_.error("Invalid DSCP {} for writer '{}' (valid code points are 0..63); rejecting",
                  static_cast<int>(options.dscp.value()), topicName);
    return nullptr;
  }

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
  applyEndpointOptions(part, attributes, options);

  DOMAIN_LOG("Creating writer[{}, {}]", topicName, typeName);

  // On any failure below, return the endpoint's dedicated port (if one was
  // granted) so it is not leaked.
  const Ip4Port_t dedicated_port =
      attributes.hasDedicatedPort ? static_cast<Ip4Port_t>(attributes.unicastLocator.port) : 0;

  if (reliable) {
    attributes.reliabilityKind = ReliabilityKind_t::RELIABLE;

    if (!statefulWriter->init(attributes, TopicKind_t::NO_KEY, *m_transport, enforceUnicast)) {
      DOMAIN_LOG("StatefulWriter init failed.");
      releaseDedicatedEndpointPort(dedicated_port);
      return nullptr;
    }

    if (!part.addWriter(statefulWriter)) {
      releaseDedicatedEndpointPort(dedicated_port);
      return nullptr;
    }
    return statefulWriter;
  } else {
    attributes.reliabilityKind = ReliabilityKind_t::BEST_EFFORT;

    if (!statelessWriter->init(attributes, TopicKind_t::NO_KEY, *m_transport, enforceUnicast)) {
      DOMAIN_LOG("StatelessWriter init failed.");
      releaseDedicatedEndpointPort(dedicated_port);
      return nullptr;
    }

    if (!part.addWriter(statelessWriter)) {
      releaseDedicatedEndpointPort(dedicated_port);
      return nullptr;
    }
    return statelessWriter;
  }
}

rtps::Reader *Domain::createReader(Participant &part, const char *topicName, const char *typeName,
                                   bool reliable, rtps::Ip4AddressBytes mcastaddress,
                                   const EndpointOptions &options) {
  // Validate the DSCP up front: an out-of-range code point can never be
  // applied (Socket::set_dscp rejects > 63), so probing would burn dedicated
  // -port offsets on binds that fail their marking and then silently fall
  // back without the requested behavior. Reject the reader instead so the
  // misconfiguration is an explicit endpoint-creation error.
  if (options.dscp.has_value() && static_cast<uint8_t>(options.dscp.value()) > 63) {
    logger_.error("Invalid DSCP {} for reader '{}' (valid code points are 0..63); rejecting",
                  static_cast<int>(options.dscp.value()), topicName);
    return nullptr;
  }

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
  applyEndpointOptions(part, attributes, options);

  DOMAIN_LOG("Creating reader[{}, {}]", topicName, typeName);

  // On any failure below, return the endpoint's dedicated port (if one was
  // granted) so it is not leaked.
  const Ip4Port_t dedicated_port =
      attributes.hasDedicatedPort ? static_cast<Ip4Port_t>(attributes.unicastLocator.port) : 0;

  if (reliable) {

    attributes.reliabilityKind = ReliabilityKind_t::RELIABLE;

    statefulReader->init(attributes, *m_transport);

    if (!part.addReader(statefulReader)) {
      DOMAIN_LOG("Failed to add reader to participant.");

      releaseDedicatedEndpointPort(dedicated_port);
      return nullptr;
    }
    return statefulReader;
  } else {

    attributes.reliabilityKind = ReliabilityKind_t::BEST_EFFORT;

    statelessReader->init(attributes);

    if (!part.addReader(statelessReader)) {
      releaseDedicatedEndpointPort(dedicated_port);
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

  // Capture the dedicated port BEFORE reset() wipes the attributes, but
  // release it AFTER reset(): reset() drains the in-flight guarded dispatches,
  // and a still-running HEARTBEAT/GAP handler may sendPacket() from this
  // source port - releasing first would let EsppTransport::sendPacket()
  // recreate it as an ordinary untracked channel (default band/DSCP) and leak
  // the fd. Mirrors the writer teardown below.
  const bool had_dedicated_port = reader->m_attributes.hasDedicatedPort;
  const auto dedicated_port = static_cast<Ip4Port_t>(reader->m_attributes.unicastLocator.port);
  reader->reset();
  if (had_dedicated_port) {
    releaseDedicatedEndpointPort(dedicated_port);
  }
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

  // Cancel any PARKED guaranteed progress() job for this writer so the retry
  // timer cannot resurrect it after deletion. A job already handed to the pool
  // is made safe by reset() below (progress() no-ops once !initialized).
  if (m_transport != nullptr) {
    m_transport->cancelGuaranteed(writer);
  }

  // Capture the dedicated port BEFORE reset() wipes the attributes, but release
  // it AFTER reset(): reset() quiesces any in-flight progress() (it takes the
  // writer's m_mutex and clears m_is_initialized_), so once it returns no job
  // can still send on this port.
  const bool had_dedicated_port = writer->m_attributes.hasDedicatedPort;
  const auto dedicated_port = static_cast<Ip4Port_t>(writer->m_attributes.unicastLocator.port);
  writer->reset();
  if (had_dedicated_port) {
    releaseDedicatedEndpointPort(dedicated_port);
  }
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

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

#ifndef RTPS_DOMAIN_H
#define RTPS_DOMAIN_H

#include "base_component.hpp"
#include "rtps/common/types.hpp"
#include "rtps/communication/EsppTransport.hpp"
#include "rtps/config.hpp"
#include "rtps/entities/Participant.hpp"
#include "rtps/entities/StatefulReader.hpp"
#include "rtps/entities/StatefulWriter.hpp"
#include "rtps/entities/StatelessReader.hpp"
#include "rtps/entities/StatelessWriter.hpp"
#include "task.hpp"
#include <atomic>
#include <mutex>
#include <rtps/common/Locator.hpp>

namespace rtps {

/// Runtime scheduling configuration for a Domain (all fields optional; the
/// defaults preserve pre-band behavior except that metatraffic - SPDP/SEDP
/// discovery - is dispatched at QosBand::High so discovery stays responsive
/// under user-traffic load).
struct DomainConfig {
  /// Band for the metatraffic (SPDP multicast + SEDP unicast) channels.
  espp::QosBand metatraffic_band{espp::QosBand::High};
  /// Band for the shared user-traffic (user unicast + user multicast) channels.
  espp::QosBand user_traffic_band{espp::QosBand::Normal};
  /// Allow endpoints with a non-default band (or a dscp) to be granted their
  /// own dedicated unicast port (announced via their SEDP per-endpoint unicast
  /// locator). Disable to force banded endpoints onto the shared user port.
  bool enable_dedicated_endpoint_ports{true};
  /// Ration for dedicated endpoint ports (each one consumes a UDP socket/fd;
  /// lwIP on ESP32 defaults to ~10 sockets total). When exhausted, further
  /// banded endpoints fall back to the shared user port (with a warning).
  /// The cap is a TRUE fd bound: released sockets whose fd is still open
  /// awaiting the reactor's removal completion (retired) count toward it.
  uint8_t max_prioritized_endpoint_ports{4};
};

/// Per-endpoint scheduling options for createWriter()/createReader().
struct EndpointOptions {
  /// Priority band for the endpoint's received-traffic dispatch. A non-Normal
  /// band requests a dedicated unicast port (see DomainConfig).
  espp::QosBand band{espp::QosBand::Normal};
  /// Optional DSCP code point for traffic the endpoint sends; requires (and by
  /// itself requests) a dedicated port, since DSCP is per-socket.
  std::optional<espp::Dscp> dscp{};
};

class Domain : public espp::BaseComponent {
public:
  explicit Domain(const Ip4AddressBytes &localIpAddress, const DomainConfig &config = {});
  Domain(EsppTransport &transport, const Ip4AddressBytes &localIpAddress,
         const DomainConfig &config = {});
  ~Domain();

  bool completeInit();
  void stop();

  Participant *createParticipant();
  Writer *createWriter(Participant &part, const char *topicName, const char *typeName,
                       bool reliable, bool enforceUnicast = false,
                       const EndpointOptions &options = {});
  Reader *createReader(Participant &part, const char *topicName, const char *typeName,
                       bool reliable, Ip4AddressBytes mcastaddress = {0, 0, 0, 0},
                       const EndpointOptions &options = {});

  Writer *writerExists(Participant &part, const char *topicName, const char *typeName,
                       bool reliable);
  Reader *readerExists(Participant &part, const char *topicName, const char *typeName,
                       bool reliable);

  bool deleteWriter(Participant &part, Writer *writer);
  bool deleteReader(Participant &part, Reader *reader);

  /// The transport the domain receives/sends through. Exposed so higher layers
  /// (e.g. the espp facade's banded deferred dispatch) can submit work onto the
  /// transport's worker pool at a chosen priority band.
  EsppTransport &getTransport() { return *m_transport; }

  void printInfo();

private:
  friend class SizeInspector;
  /// ReceiveCallback adapter: builds a PacketInfo from a raw datagram and
  /// processes it inline on the transport worker that delivered it (the
  /// reactor guarantees per-socket ordering).
  static void datagramJumppad(void *arg, const uint8_t *data, std::size_t size, Ip4Port_t localPort,
                              Ip4Port_t remotePort, const Ip4AddressBytes &remoteAddress);
  using DefaultTransport = EsppTransport;
  DefaultTransport m_defaultTransport;
  EsppTransport *m_transport = nullptr;
  std::array<Participant, Config::MAX_NUM_PARTICIPANTS> m_participants;
  Ip4AddressBytes m_localIpAddress{{0, 0, 0, 0}};
  const uint8_t PARTICIPANT_START_ID = 0;
  /// Next participant id to try; ids may skip values when a unicast port is
  /// already taken on this host (another process) and the id is probed forward.
  ParticipantId_t m_nextParticipantId = PARTICIPANT_START_ID;
  /// Number of participants created in this domain (slot count into
  /// m_participants; independent of the probed participant ids).
  uint8_t m_numParticipants = 0;
  /// How many participant ids to probe for free unicast ports before giving up.
  static constexpr uint8_t PARTICIPANT_PORT_PROBE_LIMIT = 16;
  Participant *findParticipantById(ParticipantId_t id);

  DomainConfig m_config{};

  // --- Dedicated endpoint ports (per-endpoint priority) ---------------------
  // Deterministic allocation strategy: dedicated ports live in this domain's
  // RTPS port block at offset DEDICATED_PORT_OFFSET, i.e.
  //   port = 7400 + 250*DOMAIN_ID + DEDICATED_PORT_OFFSET + n
  // with n probed linearly (reuse-disabled bind, so a port taken by another
  // process on this host fails loudly and the next one is tried). The standard
  // RTPS offsets (builtin/user, multicast/unicast) stay below 100 for
  // participant ids 0..44 only, so createParticipant() ENFORCES that cap on
  // the id probe while dedicated ports are enabled - an id past it would bind
  // its shared user-unicast port inside this range, where a dedicated-port
  // probe would mistake the existing channel for a fresh allocation and
  // misroute that participant's traffic. The whole range stays inside this
  // domain's 250-port block (offsets 100..249 -> up to 150 candidate ports;
  // allocation is additionally rationed by
  // DomainConfig::max_prioritized_endpoint_ports).
  static constexpr uint16_t DEDICATED_PORT_OFFSET = 100;
  static constexpr uint16_t DEDICATED_PORT_PROBE_LIMIT = 16;
  struct DedicatedPort {
    Ip4Port_t port{0};
    Participant *participant{nullptr};
  };
  /// Active dedicated ports, for receive routing (port -> owning participant)
  /// and for release on endpoint deletion. Bounded by the ration. Guarded by
  /// m_dedicatedPortsMutex - its OWN small mutex, NOT m_mutex: the lookup runs
  /// on the receive path (receiveCallback on a pool worker), and taking
  /// m_mutex there would let an API caller holding m_mutex across a blocking
  /// operation stall every receive worker.
  std::vector<DedicatedPort> m_dedicatedPorts;
  mutable std::mutex m_dedicatedPortsMutex;
  /// Next port offset to try, so allocation walks forward deterministically.
  uint16_t m_nextDedicatedPortOffset = 0;
  /// Allocate (bind + register) a dedicated unicast port for an endpoint of
  /// `part` at `band` (optionally DSCP-marked). Returns 0 when disabled, the
  /// ration is exhausted, or no free port was found - callers then fall back
  /// to the shared user-unicast port.
  Ip4Port_t allocateDedicatedEndpointPort(Participant &part, espp::QosBand band,
                                          const std::optional<espp::Dscp> &dscp);
  /// Release an endpoint's dedicated port (no-op for port 0 / unknown ports).
  void releaseDedicatedEndpointPort(Ip4Port_t port);
  Participant *findParticipantByDedicatedPort(Ip4Port_t port);
  /// Apply EndpointOptions to freshly-built endpoint attributes: copies
  /// band/dscp and, when the options request priority, tries to allocate a
  /// dedicated port and rewrites attributes.unicastLocator to it.
  void applyEndpointOptions(Participant &part, TopicData &attributes,
                            const EndpointOptions &options);

  /// Single deadline-scheduled protocol task: drives SPDP announcements for
  /// every participant and heartbeat ticks for every stateful writer,
  /// replacing one SPDP thread per participant plus one heartbeat thread per
  /// stateful writer. Sleeps until the earliest deadline; publishes on
  /// reliable writers nudge it awake (heartbeat piggyback).
  bool protocolLoop(std::mutex &m, std::condition_variable &cv, bool &notified);
  void nudgeProtocol();
  std::unique_ptr<espp::Task> m_protocolTask;
  /// Leaf mutex guarding the publication, use and teardown of the three task
  /// synchronization pointers below. protocolLoop() publishes them (under this
  /// mutex) before its first wait; nudgeProtocol() - called from arbitrary
  /// publisher threads via the writers' protocol nudge - reads them under it;
  /// stop() nulls them under it before the task (and with it the pointed-to
  /// mutex/cv) is destroyed, so a late nudge is a safe no-op instead of a
  /// use-after-free. Lock order: this mutex may be held while taking the
  /// task's mutex, never the reverse.
  std::mutex m_protocolNudgeMutex;
  std::mutex *m_protocolMutex = nullptr;
  std::condition_variable *m_protocolCv = nullptr;
  bool *m_protocolNotified = nullptr;
  std::chrono::steady_clock::time_point m_nextSpdpAnnounce{};
  /// Set by stop() BEFORE the task is notified, so protocolLoop can tell a
  /// stop apart from a heartbeat nudge (both arrive via the task cv).
  std::atomic<bool> m_protocolStopRequested{false};

  std::array<StatelessWriter, Config::NUM_STATELESS_WRITERS> m_statelessWriters;
  std::array<StatelessReader, Config::NUM_STATELESS_READERS> m_statelessReaders;
  std::array<StatefulReader, Config::NUM_STATEFUL_READERS> m_statefulReaders;
  std::array<StatefulWriter, Config::NUM_STATEFUL_WRITERS> m_statefulWriters;
  template <typename A, typename B> B *getNextUnusedEndpoint(A &a) {
    for (unsigned int i = 0; i < a.size(); i++) {
      if (!a[i].isInitialized()) {
        return &(a[i]);
      }
    }
    return nullptr;
  }

  bool m_initComplete = false;
  bool m_transportSetupOk = true;
  std::recursive_mutex m_mutex;

  void receiveCallback(const PacketInfo &packet);
  GuidPrefix_t generateGuidPrefix(ParticipantId_t id) const;
  //! Allocate + wire the participant's builtin discovery endpoints. Returns
  //! false (after rolling back any endpoint this invocation initialized) when
  //! a GLOBAL endpoint pool is exhausted - each participant consumes 1
  //! stateless writer/reader and 2 stateful writers/readers, so undersized
  //! limits (or too many participants) must fail participant creation cleanly
  //! instead of dereferencing a null builtin.
  bool createBuiltinWritersAndReaders(Participant &part);
  bool initializeTransport();
  void registerMulticastPort(FullLengthLocator mcastLocator);
  static void receiveJumppad(void *callee, const PacketInfo &packet);
};
} // namespace rtps

#endif // RTPS_DOMAIN_H

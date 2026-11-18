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

#ifndef RTPS_SPDP_H
#define RTPS_SPDP_H

#include "base_component.hpp"
#include "rtps/common/types.hpp"
#include "rtps/config.hpp"
#include "rtps/discovery/BuiltInEndpoints.hpp"
#include "rtps/discovery/ParticipantProxyData.hpp"
#include "rtps/utils/CdrBuffer.hpp"
#include "rtps/utils/Log.hpp"
#include "task.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>

#if SPDP_VERBOSE && RTPS_GLOBAL_VERBOSE
#include "rtps/utils/printutils.hpp"
#define SPDP_LOG(...) logger_.warn(__VA_ARGS__)
#else
#define SPDP_LOG(...)                                                                              \
  do {                                                                                             \
  } while (0)
#endif

namespace rtps {
class Participant;
class Writer;
class Reader;
class ReaderCacheChange;

class SPDPAgent : public espp::BaseComponent {
public:
  SPDPAgent();
  void init(Participant &participant, BuiltInEndpoints &endpoints);
  void start();
  void stop();
  /// True between start() and stop(); the Domain's protocol scheduler only
  /// announces for running agents.
  bool isRunning() const { return m_running; }
  /// Send one SPDP announcement (+ periodic remote-liveliness check every
  /// SPDP_CYCLECOUNT_HEARTBEAT rounds). Called by the Domain's protocol
  /// scheduler at SPDP_RESEND_PERIOD_MS cadence instead of a dedicated
  /// broadcast thread.
  void announce();
  /// `announced` is the sequence number of the announcement just received from
  /// a known participant whose highest so far is `last`. A number far below
  /// the highest (at most half of it, once the peer has announced at least
  /// three times) is not a reordered or duplicated datagram but a peer that
  /// restarted with the same GUID prefix and numbers its new SPDP writer from
  /// 1 again; such a peer is forgotten and rediscovered (its SEDP and user
  /// writers restarted too, and the stateful readers' proxies would keep
  /// waiting for the old numbers). Edges: a peer seen fewer than three times
  /// is never judged (a reordered second announcement must not tear down a
  /// participant just discovered); a restarted peer whose first
  /// announcements were lost is caught as long as its number is still at most
  /// half of what was seen before, else its lease decides; number 0 never
  /// qualifies (not a valid RTPS sequence number). This engine's own boards
  /// never reuse a prefix (Domain::guidIncarnation), so this guards against
  /// stacks that do. Security: a spoofed low-numbered announcement can tear
  /// down and re-add a healthy peer - no more than a spoofed dispose, which
  /// the protocol has always allowed from any multicast sender.
  static bool isRestart(const SequenceNumber_t &announced, const SequenceNumber_t &last);
  std::recursive_mutex m_mutex;

private:
  Participant *mp_participant = nullptr;
  BuiltInEndpoints m_buildInEndpoints;
  // Atomic: start()/stop() flip it from the app thread while the Domain's
  // protocol-scheduler thread polls isRunning() every announce cycle.
  std::atomic<bool> m_running{false};
  std::array<uint8_t, 1000> m_outputBuffer{}; // TODO check required size
  std::array<uint8_t, 1000> m_inputBuffer{};
  ParticipantProxyData m_proxyDataBuffer{};
  /// Number of valid bytes of the pre-built SPDP announcement in
  /// m_outputBuffer (built once by addParticipantParameters()).
  size_t m_outputSize = 0;
  uint8_t m_cycleHB = 0;

  bool initialized = false;
  static void receiveCallback(void *callee, const ReaderCacheChange &cacheChange);
  void handleSPDPPackage(const ReaderCacheChange &cacheChange);
  /// The prefix of the PID_PARTICIPANT_GUID in a PL_CDR payload (big- or
  /// little-endian, as its encapsulation header says) when that GUID names a
  /// participant (ENTITYID_BUILD_IN_PARTICIPANT, as the inline key hash must),
  /// else GUIDPREFIX_UNKNOWN. For a dispose that carries the key in its
  /// payload rather than inline.
  static GuidPrefix_t participantPrefixInPayload(const uint8_t *data, DataSize_t size);
  /// Parse the announcement in m_inputBuffer into m_proxyDataBuffer. The
  /// parser stops at the GUID of a participant that is already known.
  bool parseAnnouncement();
  /// Participants that disposed themselves lately: the prefix, the dispose's
  /// sequence number and when it came. An announcement of such a prefix with
  /// a lower number is a delayed or duplicated copy from before the dispose
  /// and must not bring the participant back: the SPDP reader filters
  /// sequence numbers only for writers it knows, and a removed participant's
  /// went with it. Forgotten after DISPOSE_MEMORY_MS - a peer whose prefix is
  /// stable across restarts (this engine's, on ESP targets) starts a new
  /// writer at 1 and must be discovered again promptly.
  struct DisposedParticipant {
    GuidPrefix_t prefix{};
    SequenceNumber_t sn{};
    std::chrono::steady_clock::time_point at{};
  };
  static constexpr uint32_t DISPOSE_MEMORY_MS = 2 * Config::SPDP_RESEND_PERIOD_MS;
  std::array<DisposedParticipant, 8> m_disposed{};
  size_t m_nextDisposed = 0;
  void rememberDispose(const GuidPrefix_t &prefix, const SequenceNumber_t &sn);
  bool isFromBeforeDispose(const GuidPrefix_t &prefix, const SequenceNumber_t &sn) const;
  /// Send the dispose of the participant we were in the previous boot, AS that
  /// participant: an RTPS message whose header prefix and SPDP writer are the
  /// old incarnation's, with its participant GUID as key hash + payload and
  /// a sequence number above anything it could have sent (every stack's SPDP
  /// reader takes a participant's own dispose; one from another writer
  /// naming it is refused by Cyclone DDS, Fast DDS and this engine alike).
  /// Sent on the first PREVIOUS_INCARNATION_DISPOSES announce cycles - best
  /// effort, multicast, a lost datagram costs a peer one lease.
  void sendPreviousIncarnationDispose();
  static constexpr uint8_t PREVIOUS_INCARNATION_DISPOSES = 3;
  uint8_t m_previousIncarnationDisposesLeft = 0;
  void processProxyData(const SequenceNumber_t &announcementSn);
  bool addProxiesForBuiltInEndpoints();

  void addInlineQos(CdrWriter &writer);
  void addParticipantParameters();
  void endCurrentList(CdrWriter &writer);
};
} // namespace rtps

#endif // RTPS_SPDP_H

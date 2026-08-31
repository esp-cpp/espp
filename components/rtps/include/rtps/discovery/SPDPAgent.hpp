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
  void processProxyData();
  bool addProxiesForBuiltInEndpoints();

  void addInlineQos(CdrWriter &writer);
  void addParticipantParameters();
  void endCurrentList(CdrWriter &writer);
};
} // namespace rtps

#endif // RTPS_SPDP_H

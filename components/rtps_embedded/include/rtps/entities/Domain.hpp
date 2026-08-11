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
#include "rtps/ThreadPool.hpp"
#include "rtps/common/types.hpp"
#include "rtps/communication/EsppTransport.hpp"
#include "rtps/config.hpp"
#include "rtps/entities/Participant.hpp"
#include "rtps/entities/StatefulReader.hpp"
#include "rtps/entities/StatefulWriter.hpp"
#include "rtps/entities/StatelessReader.hpp"
#include "rtps/entities/StatelessWriter.hpp"
#include <mutex>
#include <rtps/common/Locator.hpp>

namespace rtps {
class Domain : public espp::BaseComponent {
public:
  explicit Domain(const Ip4AddressBytes &localIpAddress);
  Domain(EsppTransport &transport, const Ip4AddressBytes &localIpAddress);
  ~Domain();

  bool completeInit();
  void stop();

  Participant *createParticipant();
  Writer *createWriter(Participant &part, const char *topicName, const char *typeName,
                       bool reliable, bool enforceUnicast = false);
  Reader *createReader(Participant &part, const char *topicName, const char *typeName,
                       bool reliable, Ip4AddressBytes mcastaddress = {0, 0, 0, 0});

  Writer *writerExists(Participant &part, const char *topicName, const char *typeName,
                       bool reliable);
  Reader *readerExists(Participant &part, const char *topicName, const char *typeName,
                       bool reliable);

  bool deleteWriter(Participant &part, Writer *writer);
  bool deleteReader(Participant &part, Reader *reader);

  void printInfo();

private:
  friend class SizeInspector;
  ThreadPool m_threadPool;
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
  void createBuiltinWritersAndReaders(Participant &part);
  bool initializeTransport();
  void registerMulticastPort(FullLengthLocator mcastLocator);
  static void receiveJumppad(void *callee, const PacketInfo &packet);
};
} // namespace rtps

#endif // RTPS_DOMAIN_H

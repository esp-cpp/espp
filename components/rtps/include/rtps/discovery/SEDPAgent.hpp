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

#ifndef RTPS_SEDPAGENT_H
#define RTPS_SEDPAGENT_H

#include "base_component.hpp"
#include "rtps/config.hpp"
#include "rtps/discovery/BuiltInEndpoints.hpp"
#include "rtps/discovery/TopicData.hpp"

#include <cstdint>
#include <mutex>

namespace rtps {

class Participant;
class ReaderCacheChange;
class Writer;
class Reader;

class SEDPAgent : public espp::BaseComponent {
public:
  SEDPAgent();
  void init(Participant &part, const BuiltInEndpoints &endpoints);
  bool addWriter(Writer &writer);
  bool addReader(Reader &reader);
  bool deleteReader(Reader *reader);
  bool deleteWriter(Writer *reader);

  void registerOnNewPublisherMatchedCallback(void (*callback)(void *arg), void *args);
  void registerOnNewSubscriberMatchedCallback(void (*callback)(void *arg), void *args);
  void removeUnmatchedEntitiesOfParticipant(const GuidPrefix_t &guidPrefix);
  void removeUnmatchedEntity(const Guid_t &guid);

  uint32_t getNumRemoteUnmatchedReaders();
  uint32_t getNumRemoteUnmatchedWriters();
  /// Match what waits in the unmatched registry against the local endpoints:
  /// called after proxies were freed (a remote participant left), so an
  /// endpoint that found no room in a proxy pool takes the slot.
  void retryUnmatchedEndpoints();

  /// The agent's discovery mutex (recursive). Exposed so an endpoint deletion
  /// can be made ATOMIC with the participant's slot removal under the global
  /// lock order (SEDPAgent::m_mutex -> Participant::m_mutex): holding it across
  /// deleteReader/deleteWriter() AND the slot-clear prevents a concurrent SEDP
  /// receive handler from matching a remote to the endpoint in the window
  /// between its disposal and its removal from the participant's tables.
  std::recursive_mutex &getMutex() { return m_mutex; }

protected: // For testing purposes
  void handlePublisherReaderMessage(const TopicData &writerData, const ReaderCacheChange &change);
  void handleSubscriptionReaderMessage(const TopicData &writerData,
                                       const ReaderCacheChange &change);

private:
  Participant *m_part = nullptr;
  std::recursive_mutex m_mutex;
  uint8_t m_buffer[600]; // TODO check size, currently changed from 300 to 600
                         // (FastDDS gives too many options)
  BuiltInEndpoints m_endpoints;
  /*
   * If we add readers later on, remote participants will not send matching
   * writer proxies again (and vice versa). This is done only once during
   * discovery. Therefore, we need to keep track of remote endpoints. Topic and
   * type are represented as hash values to save memory.
   */
  MemoryPool<TopicDataCompressed, Config::MAX_NUM_UNMATCHED_REMOTE_WRITERS>
      m_unmatchedRemoteWriters;
  size_t m_numUnmatchedRemoteWriters = 0;
  MemoryPool<TopicDataCompressed, Config::MAX_NUM_UNMATCHED_REMOTE_READERS>
      m_unmatchedRemoteReaders;
  size_t m_numMatchedRemoteReaders = 0;

  void tryMatchUnmatchedEndpoints();
  void addUnmatchedRemoteWriter(const TopicData &writerData);
  void addUnmatchedRemoteReader(const TopicData &readerData);
  void addUnmatchedRemoteWriter(const TopicDataCompressed &writerData);
  void addUnmatchedRemoteReader(const TopicDataCompressed &readerData);

  /// A dispose/unregister of a remote endpoint, told by the DATA's inline QoS
  /// (change.kind != ALIVE; the instance in change.keyHash or, failing that,
  /// in the payload's endpoint GUID).
  /// `writers`: the change came over the publications leg (else subscriptions).
  void handleRemoteEndpointDeletion(const ReaderCacheChange &change, bool writers);
  /// Give a local reader the proxy of a remote writer (skipping one it already
  /// has) and, for a reliable writer, send the pre-emptive ACKNACK. False when
  /// the reader's proxy pool is full.
  bool matchRemoteWriter(Reader &reader, const WriterProxy &proxy);
  /// The same for a remote reader on a local writer.
  bool matchRemoteReader(Writer &writer, const ReaderProxy &proxy);
  template <typename Pool> static bool isUnmatched(Pool &pool, const Guid_t &guid);
  /// The same told by a status-info parameter inside the payload.
  void handleRemoteEndpointDeletion(const TopicData &topic, const ReaderCacheChange &change,
                                    bool writers);
  static Guid_t disposedEndpointGuid(const TopicData &topic);
  /// Whether the sender of `change` may dispose `guid`: its own user endpoint,
  /// of the kind the leg announces.
  bool isDisposableBy(const Guid_t &guid, const ReaderCacheChange &change, bool writers);
  void removeRemoteEndpoint(const Guid_t &guid);

  void (*mfp_onNewPublisherCallback)(void *arg) = nullptr;
  void *m_onNewPublisherArgs = nullptr;
  void (*mfp_onNewSubscriberCallback)(void *arg) = nullptr;
  void *m_onNewSubscriberArgs = nullptr;

  static void jumppadPublisherReader(void *callee, const ReaderCacheChange &cacheChange);
  static void jumppadSubscriptionReader(void *callee, const ReaderCacheChange &cacheChange);

  static void jumppadTakeProxyOfDisposedReader(const Reader *reader, const WriterProxy &proxy,
                                               void *arg);
  static void jumppadTakeProxyOfDisposedWriter(const Writer *writer, const ReaderProxy &proxy,
                                               void *arg);

  void handlePublisherReaderMessage(const ReaderCacheChange &change);
  void handleSubscriptionReaderMessage(const ReaderCacheChange &change);

  template <typename A> bool deleteEndpoint(A *endpoint, Writer *sedp_endpoint);

  template <typename A> bool announceEndpointDeletion(A *local_endpoint, Writer *sedp_endpoint);

  template <typename A> bool disposeEndpointInSEDPHistory(A *local_endpoint, Writer *sedp_writer);
};
} // namespace rtps

#endif // RTPS_SEDPAGENT_H

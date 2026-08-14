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

#include "rtps/messages/MessageReceiver.hpp"
#include <rtps/entities/Participant.hpp>

#include "rtps/entities/Reader.hpp"
#include "rtps/entities/Writer.hpp"
#include "rtps/messages/MessageTypes.hpp"
#include "rtps/utils/Log.hpp"

#include <cstring>

using rtps::MessageReceiver;

#if RECV_VERBOSE && RTPS_GLOBAL_VERBOSE
#include "rtps/utils/printutils.hpp"
#define RECV_LOG(...) logger_.warn(__VA_ARGS__)
#else
#define RECV_LOG(...)                                                                              \
  do {                                                                                             \
  } while (0)
#endif

MessageReceiver::MessageReceiver(Participant *part)
    : espp::BaseComponent("RtpsMessageReceiver", espp::Logger::Verbosity::WARN)
    , mp_part(part) {}

bool MessageReceiver::processMessage(const uint8_t *data, DataSize_t size) {
  rtps::MessageSourceState sourceState;
  MessageProcessingInfo msgInfo(data, size);

  if (!processHeader(msgInfo, sourceState)) {
    return false;
  }
  SubmessageHeader submsgHeader;
  while (msgInfo.nextPos < msgInfo.size) {
    if (!deserializeMessage(msgInfo, submsgHeader)) {
      return false;
    }
    processSubmessage(msgInfo, submsgHeader, sourceState);
  }

  return true;
}

bool MessageReceiver::processHeader(MessageProcessingInfo &msgInfo,
                                    rtps::MessageSourceState &sourceState) {
  Header header;
  if (!deserializeMessage(msgInfo, header)) {
    return false;
  }

  if (header.guidPrefix.id == mp_part->m_guidPrefix.id) {
    RECV_LOG("[MessageReceiver]: Received own message.");
    return false; // Don't process our own packet
  }

  if (header.protocolName != RTPS_PROTOCOL_NAME ||
      header.protocolVersion.major != PROTOCOLVERSION.major) {
    return false;
  }

  sourceState.sourceGuidPrefix = header.guidPrefix;
  sourceState.sourceVendor = header.vendorId;
  sourceState.sourceVersion = header.protocolVersion;

  msgInfo.nextPos += Header::getRawSize();
  return true;
}

bool MessageReceiver::processSubmessage(MessageProcessingInfo &msgInfo,
                                        const SubmessageHeader &submsgHeader,
                                        const rtps::MessageSourceState &sourceState) {
  bool success = false;

  switch (submsgHeader.submessageId) {
  case SubmessageKind::ACKNACK:
    RECV_LOG("Processing AckNack submessage");
    success = processAckNackSubmessage(msgInfo, sourceState);
    break;
  case SubmessageKind::DATA:
    RECV_LOG("Processing Data submessage");
    success = processDataSubmessage(msgInfo, submsgHeader, sourceState);
    break;
#ifdef RTPS_ENABLE_FRAGMENTATION
  case SubmessageKind::DATA_FRAG:
    RECV_LOG("Processing DataFrag submessage");
    success = processDataFragSubmessage(msgInfo, submsgHeader, sourceState);
    break;
#endif
  case SubmessageKind::HEARTBEAT:
    RECV_LOG("Processing Heartbeat submessage");
    success = processHeartbeatSubmessage(msgInfo, sourceState);
    break;
  case SubmessageKind::INFO_DST:
    RECV_LOG("Info_DST submessage not relevant.");
    success = true; // Not relevant
    break;
  case SubmessageKind::GAP:
    RECV_LOG("Processing GAP submessage");
    success = processGapSubmessage(msgInfo, sourceState);
    break;
  case SubmessageKind::INFO_TS:
    RECV_LOG("Info_TS submessage not relevant.");
    success = true; // Not relevant now
    break;
  default:
    RECV_LOG("Submessage of type {} currently not supported. Skipping..",
             static_cast<uint8_t>(submsgHeader.submessageId));
    success = false;
  }
  msgInfo.nextPos += submsgHeader.octetsToNextHeader + SubmessageHeader::getRawSize();
  return success;
}

bool MessageReceiver::processDataSubmessage(MessageProcessingInfo &msgInfo,
                                            const SubmessageHeader &submsgHeader,
                                            const rtps::MessageSourceState &sourceState) {
  SubmessageData dataSubmsg;
  if (!deserializeMessage(msgInfo, dataSubmsg)) {
    return false;
  }

  const uint8_t *submessageStart = msgInfo.getPointerToCurrentPos();
  const uint8_t *submessageEnd =
      submessageStart + SubmessageHeader::getRawSize() + submsgHeader.octetsToNextHeader;
  const uint16_t submessageBodyOffset =
      static_cast<uint16_t>(sizeof(dataSubmsg.extraFlags) + sizeof(dataSubmsg.octetsToInlineQos) +
                            dataSubmsg.octetsToInlineQos);
  const uint8_t *serializedData =
      submessageStart + SubmessageHeader::getRawSize() + submessageBodyOffset;

  if (serializedData > submessageEnd) {
    return false;
  }

  // ROS 2 service correlation: capture a related_sample_identity inline QoS if
  // present (PID 0x0083 or the eProsima legacy 0x800f, both a 24-byte
  // SampleIdentity). Plain pub/sub leaves this unset.
  bool hasRelatedSampleIdentity = false;
  rpc::SampleIdentity relatedSampleIdentity{};

  if ((submsgHeader.flags & FLAG_INLINE_QOS) != 0) {
    const uint8_t *cursor = serializedData;
    bool foundSentinel = false;

    while ((cursor + sizeof(uint16_t) + sizeof(uint16_t)) <= submessageEnd) {
      uint16_t pid = 0;
      uint16_t length = 0;
      memcpy(&pid, cursor, sizeof(pid));
      cursor += sizeof(pid);
      memcpy(&length, cursor, sizeof(length));
      cursor += sizeof(length);

      if (pid == SMElement::PID_SENTINEL) {
        foundSentinel = true;
        serializedData = cursor;
        break;
      }

      if (cursor + length > submessageEnd) {
        return false;
      }

      if ((pid == rpc::PID_RELATED_SAMPLE_IDENTITY ||
           pid == rpc::PID_CUSTOM_RELATED_SAMPLE_IDENTITY) &&
          length >= rpc::SAMPLE_IDENTITY_CDR_SIZE && !hasRelatedSampleIdentity) {
        // 24-byte value: Guid_t (prefix 12 + entityKey 3 + entityKind 1) then
        // SequenceNumber_t (high int32, low uint32), all little-endian.
        Guid_t g{};
        memcpy(g.prefix.id.data(), cursor, g.prefix.id.size());
        memcpy(g.entityId.entityKey.data(), cursor + 12, g.entityId.entityKey.size());
        memcpy(&g.entityId.entityKind, cursor + 15, sizeof(EntityKind_t));
        SequenceNumber_t seq{};
        memcpy(&seq.high, cursor + 16, sizeof(seq.high));
        memcpy(&seq.low, cursor + 20, sizeof(seq.low));
        relatedSampleIdentity = rpc::SampleIdentity{g, seq};
        hasRelatedSampleIdentity = true;
      }

      cursor += length;

      const std::size_t consumed = static_cast<std::size_t>(cursor - serializedData);
      const std::size_t alignment = (4 - (consumed % 4)) % 4;
      if (cursor + alignment > submessageEnd) {
        return false;
      }
      cursor += alignment;
    }

    if (!foundSentinel) {
      return false;
    }
  }

  const DataSize_t size = static_cast<DataSize_t>(submessageEnd - serializedData);

  RECV_LOG("Received data message size {}", static_cast<int>(size));

  Reader *reader;
  if (dataSubmsg.readerId == ENTITYID_UNKNOWN) {
#if RECV_VERBOSE && RTPS_GLOBAL_VERBOSE
    RECV_LOG("Received ENTITYID_UNKNOWN readerID, searching for writer ID = ");
    printGuid(Guid_t{sourceState.sourceGuidPrefix, dataSubmsg.writerId});
#endif
    reader =
        mp_part->getReaderByWriterId(Guid_t{sourceState.sourceGuidPrefix, dataSubmsg.writerId});
    if (reader != nullptr)
      RECV_LOG("Found reader!");
  } else {
    reader = mp_part->getReader(dataSubmsg.readerId);
#if RECV_VERBOSE && RTPS_GLOBAL_VERBOSE
    auto reader_by_writer =
        mp_part->getReaderByWriterId(Guid_t{sourceState.sourceGuidPrefix, dataSubmsg.writerId});

    if (reader_by_writer == nullptr && reader != nullptr) {
      RECV_LOG("FOUND By READER ID, NOT BY WRITER ID =");
      printGuid(Guid_t{sourceState.sourceGuidPrefix, dataSubmsg.writerId});
    }
#endif
  }
  if (reader != nullptr) {
    Guid_t writerGuid{sourceState.sourceGuidPrefix, dataSubmsg.writerId};
    ReaderCacheChange change{ChangeKind_t::ALIVE,  writerGuid, dataSubmsg.writerSN,
                             serializedData,       size,       hasRelatedSampleIdentity,
                             relatedSampleIdentity};
    reader->newChange(change);
  } else {
#if RECV_VERBOSE && RTPS_GLOBAL_VERBOSE
    RECV_LOG("Couldn't find a reader with id: ");
    printEntityId(dataSubmsg.readerId);
#endif
  }

  return true;
}

#ifdef RTPS_ENABLE_FRAGMENTATION
bool MessageReceiver::processDataFragSubmessage(MessageProcessingInfo &msgInfo,
                                                const SubmessageHeader &submsgHeader,
                                                const rtps::MessageSourceState &sourceState) {
  SubmessageDataFrag frag;
  if (!deserializeMessage(msgInfo, frag)) {
    return false;
  }

  const uint8_t *submessageStart = msgInfo.getPointerToCurrentPos();
  const uint8_t *submessageEnd =
      submessageStart + SubmessageHeader::getRawSize() + submsgHeader.octetsToNextHeader;
  const uint16_t submessageBodyOffset = static_cast<uint16_t>(
      sizeof(frag.extraFlags) + sizeof(frag.octetsToInlineQos) + frag.octetsToInlineQos);
  const uint8_t *serializedData =
      submessageStart + SubmessageHeader::getRawSize() + submessageBodyOffset;

  if (serializedData > submessageEnd) {
    return false;
  }

  // Skip an inlineQos ParameterList if present (a peer may attach a key hash to
  // the first fragment), mirroring processDataSubmessage.
  if ((submsgHeader.flags & FLAG_INLINE_QOS) != 0) {
    const uint8_t *cursor = serializedData;
    bool foundSentinel = false;
    while ((cursor + sizeof(uint16_t) + sizeof(uint16_t)) <= submessageEnd) {
      uint16_t pid = 0;
      uint16_t length = 0;
      memcpy(&pid, cursor, sizeof(pid));
      cursor += sizeof(pid);
      memcpy(&length, cursor, sizeof(length));
      cursor += sizeof(length);
      if (pid == SMElement::PID_SENTINEL) {
        foundSentinel = true;
        serializedData = cursor;
        break;
      }
      if (cursor + length > submessageEnd) {
        return false;
      }
      cursor += length;
      const std::size_t consumed = static_cast<std::size_t>(cursor - serializedData);
      const std::size_t alignment = (4 - (consumed % 4)) % 4;
      if (cursor + alignment > submessageEnd) {
        return false;
      }
      cursor += alignment;
    }
    if (!foundSentinel) {
      return false;
    }
  }

  if (serializedData > submessageEnd) {
    return false;
  }
  const DataSize_t fragDataLen = static_cast<DataSize_t>(submessageEnd - serializedData);

  Reader *reader;
  if (frag.readerId == ENTITYID_UNKNOWN) {
    reader = mp_part->getReaderByWriterId(Guid_t{sourceState.sourceGuidPrefix, frag.writerId});
  } else {
    reader = mp_part->getReader(frag.readerId);
  }
  if (reader != nullptr) {
    Guid_t writerGuid{sourceState.sourceGuidPrefix, frag.writerId};
    reader->newFragment(writerGuid, frag.writerSN, frag.fragmentStartingNum,
                        frag.fragmentsInSubmessage, frag.fragmentSize, frag.sampleSize,
                        serializedData, fragDataLen);
  }
  return true;
}
#endif

bool MessageReceiver::processHeartbeatSubmessage(MessageProcessingInfo &msgInfo,
                                                 const rtps::MessageSourceState &sourceState) {
  SubmessageHeartbeat submsgHB;
  if (!deserializeMessage(msgInfo, submsgHB)) {
    return false;
  }

  Reader *reader = mp_part->getReader(submsgHB.readerId);
  if (reader != nullptr) {
    reader->onNewHeartbeat(submsgHB, sourceState.sourceGuidPrefix);
    mp_part->refreshRemoteParticipantLiveliness(sourceState.sourceGuidPrefix);
    return true;
  } else {
    return false;
  }
}

bool MessageReceiver::processAckNackSubmessage(MessageProcessingInfo &msgInfo,
                                               const rtps::MessageSourceState &sourceState) {
  SubmessageAckNack submsgAckNack;
  if (!deserializeMessage(msgInfo, submsgAckNack)) {
    return false;
  }

  Writer *writer = mp_part->getWriter(submsgAckNack.writerId);
  if (writer != nullptr) {
    writer->onNewAckNack(submsgAckNack, sourceState.sourceGuidPrefix);
    return true;
  } else {
    return false;
  }
}

bool MessageReceiver::processGapSubmessage(MessageProcessingInfo &msgInfo,
                                           const rtps::MessageSourceState &sourceState) {
  SubmessageGap submsgGap;
  if (!deserializeMessage(msgInfo, submsgGap)) {
    return false;
  }

  Reader *reader = mp_part->getReader(submsgGap.readerId);
  if (reader != nullptr) {
    reader->onNewGapMessage(submsgGap, sourceState.sourceGuidPrefix);
    return true;
  } else {
    return false;
  }
}
#undef RECV_VERBOSE

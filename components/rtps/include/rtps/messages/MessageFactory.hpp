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

// Copyright 2023 Apex.AI, Inc.
// All rights reserved.

#ifndef RTPS_MESSAGEFACTORY_H
#define RTPS_MESSAGEFACTORY_H

#include "rtps/common/types.hpp"
#include "rtps/config.hpp"
#include "rtps/messages/MessageTypes.hpp"
#include "rtps/rpc/sample_identity.hpp"
#include "rtps/utils/sysFunctions.hpp"

#include <array>
#include <cstdint>

namespace rtps {
namespace MessageFactory {
const std::array<uint8_t, 4> PROTOCOL_TYPE{'R', 'T', 'P', 'S'};
const uint8_t numBytesUntilEndOfLength = 4; // The first bytes incl. submessagelength don't count

template <class Buffer> void addHeader(Buffer &buffer, const GuidPrefix_t &guidPrefix) {

  Header header;
  header.protocolName = PROTOCOL_TYPE;
  header.protocolVersion = PROTOCOLVERSION;
  header.vendorId = Config::VENDOR_ID;
  header.guidPrefix = guidPrefix;

  serializeMessage(buffer, header);
}

template <class Buffer> bool addSubMessageInfoDST(Buffer &buffer, GuidPrefix_t &dst) {
  SubmessageInfoDST msg;
  msg.header.submessageId = SubmessageKind::INFO_DST;

#if IS_LITTLE_ENDIAN
  msg.header.flags = FLAG_LITTLE_ENDIAN;
#else
  msg.header.flags = FLAG_BIG_ENDIAN;
#endif

  msg.header.octetsToNextHeader = sizeof(GuidPrefix_t);
  msg.guidPrefix = dst;

  return serializeMessage(buffer, msg);
}

template <class Buffer> void addSubMessageTimeStamp(Buffer &buffer, bool setInvalid = false) {
  SubmessageHeader header;
  header.submessageId = SubmessageKind::INFO_TS;

#if IS_LITTLE_ENDIAN
  header.flags = FLAG_LITTLE_ENDIAN;
#else
  header.flags = FLAG_BIG_ENDIAN;
#endif

  if (setInvalid) {
    header.flags |= FLAG_INVALIDATE;
    header.octetsToNextHeader = 0;
  } else {
    header.octetsToNextHeader = sizeof(Time_t);
  }

  serializeMessage(buffer, header);

  if (!setInvalid) {
    buffer.reserve(header.octetsToNextHeader);
    Time_t now = getCurrentTimeStamp();
    buffer.append(reinterpret_cast<uint8_t *>(&now.seconds), sizeof(Time_t::seconds));
    buffer.append(reinterpret_cast<uint8_t *>(&now.fraction), sizeof(Time_t::fraction));
  }
}

template <class Buffer, class PayloadBuffer>
void addSubMessageData(Buffer &buffer, const PayloadBuffer &filledPayload, bool containsInlineQos,
                       const SequenceNumber_t &SN, const EntityId_t &writerID,
                       const EntityId_t &readerID) {
  SubmessageData msg;
  msg.header.submessageId = SubmessageKind::DATA;
#if IS_LITTLE_ENDIAN
  msg.header.flags = FLAG_LITTLE_ENDIAN;
#else
  msg.header.flags = FLAG_BIG_ENDIAN;
#endif

  // octetsToNextHeader is a 16-bit wire field. spaceUsed() is now DataSize_t
  // (uint32) but the non-fragmented DATA path is only reached for samples that
  // fit a single submessage (< 64 KB), so the narrowing to uint16 is correct
  // and required for wire-format neutrality. (DATA_FRAG, when it lands, will
  // carry oversized samples via fragment-sized submessages instead.)
  msg.header.octetsToNextHeader = static_cast<uint16_t>(
      SubmessageData::getRawSize() + filledPayload.spaceUsed() - numBytesUntilEndOfLength);

  if (containsInlineQos) {
    msg.header.flags |= FLAG_INLINE_QOS;
  }
  if (filledPayload.isValid()) {
    msg.header.flags |= FLAG_DATA_PAYLOAD;
  }

  msg.writerSN = SN;
  msg.extraFlags = 0;
  msg.readerId = readerID;
  msg.writerId = writerID;

  constexpr uint16_t octetsToInlineQoS = 4 + 4 + 8; // EntityIds + SequenceNumber
  msg.octetsToInlineQos = octetsToInlineQoS;

  serializeMessage(buffer, msg);

  if (filledPayload.isValid()) {
    buffer.append(filledPayload);
  }
}

// Append a DATA submessage that carries a related_sample_identity inline QoS, as
// rmw_fastrtps uses for ROS 2 service request/reply correlation (see
// rpc/sample_identity.hpp and RMI_AMI_DESIGN.md 3.2). Distinct from
// addSubMessageData so the plain pub/sub path stays byte-identical; only the
// service request/reply writers call this.
//
// Layout after the SubmessageData header (readerId..writerSN): an inline QoS
// ParameterList of two 24-byte related_sample_identity parameters (PID 0x0083
// and 0x800f, both emitted with the identical value) terminated by PID_SENTINEL,
// followed by the serialized payload.
template <class Buffer, class PayloadBuffer>
void addSubMessageDataWithRelatedSampleIdentity(Buffer &buffer, const PayloadBuffer &filledPayload,
                                                const rpc::SampleIdentity &relatedSampleIdentity,
                                                const SequenceNumber_t &SN,
                                                const EntityId_t &writerID,
                                                const EntityId_t &readerID) {
  // Inline QoS parameter list byte count: two (PID + length + 24-byte value)
  // parameters plus a (PID + length) sentinel.
  constexpr uint16_t kParamHeader = 2 * sizeof(uint16_t); // parameterId + length
  constexpr uint16_t kInlineQosBytes =
      2 * (kParamHeader + rpc::SAMPLE_IDENTITY_CDR_SIZE) + kParamHeader; // + PID_SENTINEL

  SubmessageData msg;
  msg.header.submessageId = SubmessageKind::DATA;
#if IS_LITTLE_ENDIAN
  msg.header.flags = FLAG_LITTLE_ENDIAN;
#else
  msg.header.flags = FLAG_BIG_ENDIAN;
#endif
  msg.header.flags |= FLAG_INLINE_QOS;
  if (filledPayload.isValid()) {
    msg.header.flags |= FLAG_DATA_PAYLOAD;
  }

  // octetsToNextHeader spans the fixed DATA body + inline QoS + payload (the
  // narrowing to uint16 is safe: a service request/reply always fits one
  // submessage). See addSubMessageData for the base calculation.
  msg.header.octetsToNextHeader =
      static_cast<uint16_t>(SubmessageData::getRawSize() + kInlineQosBytes +
                            filledPayload.spaceUsed() - numBytesUntilEndOfLength);

  msg.writerSN = SN;
  msg.extraFlags = 0;
  msg.readerId = readerID;
  msg.writerId = writerID;
  constexpr uint16_t octetsToInlineQoS = 4 + 4 + 8; // EntityIds + SequenceNumber
  msg.octetsToInlineQos = octetsToInlineQoS;

  serializeMessage(buffer, msg);

  // Inline QoS ParameterList: same 24-byte value under both PIDs, then sentinel.
  const auto appendParam = [&](uint16_t pid) {
    uint16_t length = rpc::SAMPLE_IDENTITY_CDR_SIZE;
    buffer.append(reinterpret_cast<uint8_t *>(&pid), sizeof(pid));
    buffer.append(reinterpret_cast<uint8_t *>(&length), sizeof(length));
    rpc::serializeSampleIdentity(buffer, relatedSampleIdentity);
  };
  appendParam(rpc::PID_RELATED_SAMPLE_IDENTITY);
  appendParam(rpc::PID_CUSTOM_RELATED_SAMPLE_IDENTITY);
  uint16_t sentinel = SMElement::PID_SENTINEL;
  uint16_t sentinelLen = 0;
  buffer.append(reinterpret_cast<uint8_t *>(&sentinel), sizeof(sentinel));
  buffer.append(reinterpret_cast<uint8_t *>(&sentinelLen), sizeof(sentinelLen));

  if (filledPayload.isValid()) {
    buffer.append(filledPayload);
  }
}

#ifdef RTPS_ENABLE_FRAGMENTATION
// Append one DATA_FRAG submessage carrying [fragData, fragData+fragLen) as the
// serializedData for fragment(s) starting at fragStartNum (1-based). fragLen must
// be <= 65507 - overhead (guaranteed because callers pass fragmentSize-bounded
// chunks). fragmentsInSubmessage is normally 1 (simplest, interop-friendly).
template <class Buffer>
void addSubMessageDataFrag(Buffer &buffer, const uint8_t *fragData, uint16_t fragLen,
                           uint32_t fragStartNum, uint16_t fragsInSubmsg, uint16_t fragmentSize,
                           uint32_t sampleSize, const SequenceNumber_t &SN,
                           const EntityId_t &writerID, const EntityId_t &readerID) {
  SubmessageDataFrag msg;
  msg.header.submessageId = SubmessageKind::DATA_FRAG;
#if IS_LITTLE_ENDIAN
  msg.header.flags = FLAG_LITTLE_ENDIAN;
#else
  msg.header.flags = FLAG_BIG_ENDIAN;
#endif
  // DATA_FRAG has no separate "data present" flag (bit 2 is the Key flag); the
  // serializedData is always present. Alive CDR samples set only E (+ Q if
  // inlineQos, which we do not emit).
  msg.header.octetsToNextHeader =
      static_cast<uint16_t>(SubmessageDataFrag::getRawSize() + fragLen - numBytesUntilEndOfLength);
  msg.extraFlags = 0;
  msg.octetsToInlineQos = SubmessageDataFrag::octetsToInlineQosValue();
  msg.readerId = readerID;
  msg.writerId = writerID;
  msg.writerSN = SN;
  msg.fragmentStartingNum = fragStartNum;
  msg.fragmentsInSubmessage = fragsInSubmsg;
  msg.fragmentSize = fragmentSize;
  msg.sampleSize = sampleSize;

  serializeMessage(buffer, msg);

  buffer.reserve(fragLen);
  buffer.append(fragData, fragLen);
}
#endif // RTPS_ENABLE_FRAGMENTATION

template <class Buffer>
void addHeartbeat(Buffer &buffer, EntityId_t writerId, EntityId_t readerId,
                  SequenceNumber_t firstSN, SequenceNumber_t lastSN, Count_t count) {
  SubmessageHeartbeat subMsg;
  subMsg.header.submessageId = SubmessageKind::HEARTBEAT;
  subMsg.header.octetsToNextHeader = SubmessageHeartbeat::getRawSize() - numBytesUntilEndOfLength;
#if IS_LITTLE_ENDIAN
  subMsg.header.flags = FLAG_LITTLE_ENDIAN;
#else
  subMsg.header.flags = FLAG_BIG_ENDIAN;
#endif
  // Force response by not setting final flag.

  subMsg.writerId = writerId;
  subMsg.readerId = readerId;
  subMsg.firstSN = firstSN;
  subMsg.lastSN = lastSN;
  subMsg.count = count;

  serializeMessage(buffer, subMsg);
}

template <class Buffer>
void addAckNack(Buffer &buffer, EntityId_t writerId, EntityId_t readerId,
                SequenceNumberSet readerSNState, Count_t count, bool final_flag) {
  SubmessageAckNack subMsg;
  subMsg.header.submessageId = SubmessageKind::ACKNACK;
#if IS_LITTLE_ENDIAN
  subMsg.header.flags = FLAG_LITTLE_ENDIAN;
#else
  subMsg.header.flags = FLAG_BIG_ENDIAN;
#endif
  if (final_flag) {
    subMsg.header.flags |= FLAG_FINAL; // For now, we don't want any response
  } else {
    subMsg.header.flags &= ~FLAG_FINAL; // Send future heartbeats, even if no change occured
  }
  subMsg.header.octetsToNextHeader =
      SubmessageAckNack::getRawSize(readerSNState) - numBytesUntilEndOfLength;

  subMsg.writerId = writerId;
  subMsg.readerId = readerId;
  subMsg.readerSNState = readerSNState;
  subMsg.count = count;

  serializeMessage(buffer, subMsg);
}

template <class Buffer>
void addSubmessageGap(Buffer &buffer, EntityId_t writerId, EntityId_t readerId,
                      const SequenceNumber_t &firstMissing, const SequenceNumber_t &nextValid) {
  SubmessageGap subMsg;
  subMsg.header.submessageId = SubmessageKind::GAP;
#if IS_LITTLE_ENDIAN
  subMsg.header.flags = FLAG_LITTLE_ENDIAN;
#else
  subMsg.header.flags = FLAG_BIG_ENDIAN;
#endif
  subMsg.header.octetsToNextHeader = 32;

  subMsg.writerId = writerId;
  subMsg.readerId = readerId;
  subMsg.gapStart = firstMissing;
  subMsg.gapList.base = nextValid;
  subMsg.gapList.numBits = 0;

  serializeMessage(buffer, subMsg);
}
} // namespace MessageFactory
} // namespace rtps

#endif // RTPS_MESSAGEFACTORY_H

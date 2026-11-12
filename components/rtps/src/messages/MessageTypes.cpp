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

#include "rtps/messages/MessageTypes.hpp"

#include <cstddef>
#include <cstring>

#include <stdio.h>
using namespace rtps;

void doCopyAndMoveOn(uint8_t *dst, const uint8_t *&src, size_t size) {
  memcpy(dst, src, size);
  src += size;
}

bool rtps::deserializeMessage(const MessageProcessingInfo &info, Header &header) {
  if (info.getRemainingSize() < Header::getRawSize()) {
    return false;
  }

  const uint8_t *currentPos = info.getPointerToCurrentPos();
  doCopyAndMoveOn(header.protocolName.data(), currentPos, sizeof(std::array<uint8_t, 4>));
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&header.protocolVersion), currentPos,
                  sizeof(ProtocolVersion_t));
  doCopyAndMoveOn(header.vendorId.vendorId.data(), currentPos, header.vendorId.vendorId.size());
  doCopyAndMoveOn(header.guidPrefix.id.data(), currentPos, header.guidPrefix.id.size());
  return true;
}

bool rtps::deserializeMessage(const MessageProcessingInfo &info, SubmessageHeader &header) {
  if (info.getRemainingSize() < SubmessageHeader::getRawSize()) {
    return false;
  }

  const uint8_t *currentPos = info.getPointerToCurrentPos();
  header.submessageId = static_cast<SubmessageKind>(*currentPos++);
  header.flags = *(currentPos++);
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&header.octetsToNextHeader), currentPos,
                  sizeof(uint16_t));
  return true;
}

bool rtps::deserializeMessage(const MessageProcessingInfo &info, SubmessageData &msg) {
  if (info.getRemainingSize() < SubmessageHeader::getRawSize()) {
    return false;
  }
  if (!deserializeMessage(info, msg.header)) {
    return false;
  }

  // Check for length including data
  constexpr auto min_body_size = SubmessageData::getRawSize() - SubmessageHeader::getRawSize();
  if (msg.header.octetsToNextHeader < min_body_size) {
    return false;
  }
  if (info.getRemainingSize() < SubmessageHeader::getRawSize() + msg.header.octetsToNextHeader) {
    return false;
  }

  const uint8_t *currentPos = info.getPointerToCurrentPos() + SubmessageHeader::getRawSize();

  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&msg.extraFlags), currentPos, sizeof(uint16_t));
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&msg.octetsToInlineQos), currentPos,
                  sizeof(uint16_t));
  doCopyAndMoveOn(msg.readerId.entityKey.data(), currentPos, msg.readerId.entityKey.size());
  msg.readerId.entityKind = static_cast<EntityKind_t>(*currentPos++);
  doCopyAndMoveOn(msg.writerId.entityKey.data(), currentPos, msg.writerId.entityKey.size());
  msg.writerId.entityKind = static_cast<EntityKind_t>(*currentPos++);
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&msg.writerSN.high), currentPos,
                  sizeof(msg.writerSN.high));
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&msg.writerSN.low), currentPos,
                  sizeof(msg.writerSN.low));
  return true;
}

#ifdef RTPS_ENABLE_FRAGMENTATION
bool rtps::deserializeMessage(const MessageProcessingInfo &info, SubmessageDataFrag &msg) {
  if (info.getRemainingSize() < SubmessageHeader::getRawSize()) {
    return false;
  }
  if (!deserializeMessage(info, msg.header)) {
    return false;
  }

  // The fixed DATA_FRAG header (without inlineQos / serializedData) must be
  // present in full.
  constexpr auto min_body_size = SubmessageDataFrag::getRawSize() - SubmessageHeader::getRawSize();
  if (msg.header.octetsToNextHeader < min_body_size) {
    return false;
  }
  if (info.getRemainingSize() < SubmessageHeader::getRawSize() + msg.header.octetsToNextHeader) {
    return false;
  }

  const uint8_t *currentPos = info.getPointerToCurrentPos() + SubmessageHeader::getRawSize();

  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&msg.extraFlags), currentPos, sizeof(uint16_t));
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&msg.octetsToInlineQos), currentPos,
                  sizeof(uint16_t));
  doCopyAndMoveOn(msg.readerId.entityKey.data(), currentPos, msg.readerId.entityKey.size());
  msg.readerId.entityKind = static_cast<EntityKind_t>(*currentPos++);
  doCopyAndMoveOn(msg.writerId.entityKey.data(), currentPos, msg.writerId.entityKey.size());
  msg.writerId.entityKind = static_cast<EntityKind_t>(*currentPos++);
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&msg.writerSN.high), currentPos,
                  sizeof(msg.writerSN.high));
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&msg.writerSN.low), currentPos,
                  sizeof(msg.writerSN.low));
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&msg.fragmentStartingNum), currentPos,
                  sizeof(uint32_t));
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&msg.fragmentsInSubmessage), currentPos,
                  sizeof(uint16_t));
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&msg.fragmentSize), currentPos, sizeof(uint16_t));
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&msg.sampleSize), currentPos, sizeof(uint32_t));
  return true;
}
#endif

bool rtps::deserializeMessage(const MessageProcessingInfo &info, SubmessageHeartbeat &msg) {
  if (info.getRemainingSize() < SubmessageHeartbeat::getRawSize()) {
    return false;
  }
  if (!deserializeMessage(info, msg.header)) {
    return false;
  }
  if (msg.header.octetsToNextHeader <
      SubmessageHeartbeat::getRawSize() - SubmessageHeader::getRawSize()) {
    return false; // declared shorter than its fixed body
  }

  const uint8_t *currentPos = info.getPointerToCurrentPos() + SubmessageHeader::getRawSize();

  doCopyAndMoveOn(msg.readerId.entityKey.data(), currentPos, msg.readerId.entityKey.size());
  msg.readerId.entityKind = static_cast<EntityKind_t>(*currentPos++);
  doCopyAndMoveOn(msg.writerId.entityKey.data(), currentPos, msg.writerId.entityKey.size());
  msg.writerId.entityKind = static_cast<EntityKind_t>(*currentPos++);
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&msg.firstSN.high), currentPos,
                  sizeof(msg.firstSN.high));
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&msg.firstSN.low), currentPos,
                  sizeof(msg.firstSN.low));
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&msg.lastSN.high), currentPos,
                  sizeof(msg.lastSN.high));
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&msg.lastSN.low), currentPos, sizeof(msg.lastSN.low));
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&msg.count.value), currentPos,
                  sizeof(msg.count.value));
  return true;
}

bool rtps::deserializeSNS(const uint8_t *&position, SequenceNumberSet &set,
                          std::size_t num_bitfields) {
  // `num_bitfields` is what the submessage length leaves for the bitmap after
  // the base and numBits; the caller has checked that many bytes exist. The
  // bitmap holds ceil(numBits / 32) words (RTPS 2.5 9.4.2.6), so a numBits that
  // needs more bytes than are there is malformed - the old code read the
  // declared count regardless and advanced past the submessage.
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&set.base.high), position,
                  sizeof(SequenceNumber_t::high));
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&set.base.low), position,
                  sizeof(SequenceNumber_t::low));
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&set.numBits), position, sizeof(uint32_t));

  if (set.numBits > SNS_MAX_NUM_BITS) {
    // More than the set can represent: a peer may legitimately announce up to
    // 256 (the spec's maximum); anything above is bogus. Clamp, so the words
    // that are there are used and nothing beyond the set is ever addressed.
    set.numBits = SNS_MAX_NUM_BITS;
  }
  const std::size_t declared = 4 * ((set.numBits + 31) / 32); // bytes the bitmap needs
  if (declared > num_bitfields) {
    return false;
  }
  if (declared > 0) {
    doCopyAndMoveOn(reinterpret_cast<uint8_t *>(set.bitMap.data()), position, declared);
  }
  position += (num_bitfields - declared); // trailing bytes the peer appended
  return true;
}

bool rtps::deserializeMessage(const MessageProcessingInfo &info, SubmessageAckNack &msg) {
  const DataSize_t remainingSizeAtBeginning = info.getRemainingSize();
  if (remainingSizeAtBeginning <
      SubmessageAckNack::getRawSizeWithoutSNSet()) { // Size of SequenceNumberSet unknown
    return false;
  }
  if (!deserializeMessage(info, msg.header)) {
    return false;
  }
  // The declared length must hold the fixed fields (two entity ids, the set's
  // base and numBits, the count) and must not run past the datagram: the
  // bitmap length below is derived from it, and without these checks a short
  // ACKNACK underflowed that length (size_t) and the parser read far beyond
  // the buffer. Regression test: pc/tests/rtps_receiver_malformed.cpp.
  constexpr uint16_t kFixedBody = 4 + 4 + 8 + 4 + 4;
  if (msg.header.octetsToNextHeader < kFixedBody ||
      info.getRemainingSize() < SubmessageHeader::getRawSize() + msg.header.octetsToNextHeader) {
    return false;
  }

  const uint8_t *currentPos = info.getPointerToCurrentPos() + SubmessageHeader::getRawSize();

  doCopyAndMoveOn(msg.readerId.entityKey.data(), currentPos, msg.readerId.entityKey.size());
  msg.readerId.entityKind = static_cast<EntityKind_t>(*currentPos++);
  doCopyAndMoveOn(msg.writerId.entityKey.data(), currentPos, msg.writerId.entityKey.size());
  msg.writerId.entityKind = static_cast<EntityKind_t>(*currentPos++);

  const std::size_t num_bitfields = msg.header.octetsToNextHeader - kFixedBody;
  if (!deserializeSNS(currentPos, msg.readerSNState, num_bitfields)) {
    return false;
  }

  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&msg.count.value), currentPos,
                  sizeof(msg.count.value));
  return true;
}

bool rtps::deserializeMessage(const MessageProcessingInfo &info, SubmessageGap &msg) {

  const DataSize_t remainingSizeAtBeginning = info.getRemainingSize();
  if (remainingSizeAtBeginning <
      SubmessageGap::getRawSizeWithoutSNSet()) { // Size of SequenceNumberSet
                                                 // unknown
    return false;
  }
  if (!deserializeMessage(info, msg.header)) {
    return false;
  }
  if (msg.header.octetsToNextHeader <
      (SubmessageGap::getRawSizeWithSingleElementSNSet() - SubmessageHeader::getRawSize())) {
    return false;
  }
  if (info.getRemainingSize() < SubmessageHeader::getRawSize() + msg.header.octetsToNextHeader) {
    return false; // declared longer than the datagram (see the ACKNACK check)
  }
  const uint8_t *currentPos = info.getPointerToCurrentPos() + SubmessageHeader::getRawSize();

  doCopyAndMoveOn(msg.readerId.entityKey.data(), currentPos, msg.readerId.entityKey.size());
  msg.readerId.entityKind = static_cast<EntityKind_t>(*currentPos++);
  doCopyAndMoveOn(msg.writerId.entityKey.data(), currentPos, msg.writerId.entityKey.size());
  msg.writerId.entityKind = static_cast<EntityKind_t>(*currentPos++);

  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&msg.gapStart.high), currentPos,
                  sizeof(msg.gapStart.high));
  doCopyAndMoveOn(reinterpret_cast<uint8_t *>(&msg.gapStart.low), currentPos,
                  sizeof(msg.gapStart.low));

  const std::size_t num_bitfields = msg.header.octetsToNextHeader - 4 - 4 - 8 - 8 - 4;
  return deserializeSNS(currentPos, msg.gapList, num_bitfields);
}

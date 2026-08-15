/*
The MIT License
Copyright (c) 2026 ATDev
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

This file is part of the espp embeddedRTPS port.
*/

#ifndef RTPS_RPC_SAMPLE_IDENTITY_H
#define RTPS_RPC_SAMPLE_IDENTITY_H

// ---------------------------------------------------------------------------
// SampleIdentity + related_sample_identity inline-QoS wire constants for ROS 2
// (rmw_fastrtps) request/reply correlation.
//
// A SampleIdentity is a {GUID, SequenceNumber}. rmw_fastrtps correlates a reply
// to its request by carrying the request's identity on BOTH the request and the
// reply as inline QoS, under two parameter IDs (see RMI_AMI_DESIGN.md 3.2,
// confirmed against a live ROS 2 Jazzy AddTwoInts capture):
//
//   PID 0x0083  PID_RELATED_SAMPLE_IDENTITY         (OMG DDS-RPC standard)
//   PID 0x800f  PID_CUSTOM_RELATED_SAMPLE_IDENTITY  (eProsima legacy)
//
// Both carry the identical 24-byte value and both are emitted; a receiver
// accepts either. The 24-byte CDR_LE serialization is:
//
//   guidPrefix (12) | entityKey (3) | entityKind (1) | seq.high (int32) | seq.low (uint32)
//
// i.e. the raw Guid_t layout (16) followed by SequenceNumber_t (8), all little-
// endian. UNKNOWN sequence number = {high=-1, low=0} (client's request value;
// the server echoes the request's RTPS writerSeqNumber instead).
// ---------------------------------------------------------------------------

#include "rtps/common/types.hpp"

#include <cstdint>

namespace rtps {
namespace rpc {

constexpr uint16_t PID_RELATED_SAMPLE_IDENTITY = 0x0083;
constexpr uint16_t PID_CUSTOM_RELATED_SAMPLE_IDENTITY = 0x800f;

// On-wire size of a serialized SampleIdentity: Guid_t (16) + SequenceNumber_t (8).
constexpr uint16_t SAMPLE_IDENTITY_CDR_SIZE = 24;

struct SampleIdentity {
  Guid_t writer_guid;
  SequenceNumber_t sequence_number;

  bool operator==(const SampleIdentity &o) const {
    return writer_guid == o.writer_guid && sequence_number == o.sequence_number;
  }
};

// The SequenceNumber value rmw uses for an as-yet-unassigned identity (the value
// a client stamps on its outgoing request).
inline SampleIdentity unknown_sample_identity(const Guid_t &reply_reader_guid) {
  return SampleIdentity{reply_reader_guid, SequenceNumber_t{-1, 0}};
}

// Append the 24-byte CDR_LE SampleIdentity value to a MessageFactory-style
// Buffer (append(const uint8_t*, len)). Little-endian integers match the wire.
template <class Buffer> void serializeSampleIdentity(Buffer &buffer, const SampleIdentity &id) {
  buffer.append(id.writer_guid.prefix.id.data(), id.writer_guid.prefix.id.size());
  buffer.append(id.writer_guid.entityId.entityKey.data(), id.writer_guid.entityId.entityKey.size());
  buffer.append(reinterpret_cast<const uint8_t *>(&id.writer_guid.entityId.entityKind),
                sizeof(EntityKind_t));
  buffer.append(reinterpret_cast<const uint8_t *>(&id.sequence_number.high),
                sizeof(id.sequence_number.high));
  buffer.append(reinterpret_cast<const uint8_t *>(&id.sequence_number.low),
                sizeof(id.sequence_number.low));
}

} // namespace rpc
} // namespace rtps

#endif // RTPS_RPC_SAMPLE_IDENTITY_H

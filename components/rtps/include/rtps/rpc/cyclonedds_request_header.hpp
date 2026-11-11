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

#ifndef RTPS_RPC_CYCLONEDDS_REQUEST_HEADER_H
#define RTPS_RPC_CYCLONEDDS_REQUEST_HEADER_H

// ---------------------------------------------------------------------------
// rmw_cyclonedds_cpp request/reply correlation: an in-payload header.
//
// rmw_fastrtps_cpp (and espp's default) correlates a reply to its request with
// the related_sample_identity inline QoS (see sample_identity.hpp).
// rmw_cyclonedds_cpp does not: it prepends a 16-byte header to the CDR body of
// EVERY request and EVERY reply (rmw_cyclonedds_cpp/src/serdata.hpp,
// cdds_request_header_t, identical from humble through jazzy):
//
//   encapsulation (4) | guid (uint64) | seq (int64) | <Service>_Request / _Response
//
// The header is written in the payload's own CDR endianness (the second
// encapsulation byte, 0x01 = little-endian). Because it is exactly 16 bytes
// the message body still starts 8-byte aligned, so the body's own CDR bytes
// are identical to a standalone sample: stripping or prepending the header is
// a plain byte splice with no re-encoding.
//
// Semantics (rmw_cyclonedds_cpp/src/rmw_node.cpp):
//   client  : guid = the client's request-writer instance handle (opaque to
//             everyone else), seq = a client-wide counter.
//   server  : echoes both values unchanged into the reply.
//   client  : takes a reply iff reply.guid == its own guid; seq finds the
//             pending request.
// The reply is a plain DATA on the shared rr/ topic - no inline QoS - so a
// server speaking to a Cyclone client must answer with this header and NOT
// with related_sample_identity, and vice versa. Which one a peer speaks is
// decided by its vendor id (01.10 = Eclipse Cyclone DDS; the RTPS message
// header carries it on every packet, SPDP records it per participant).
// ---------------------------------------------------------------------------

#include "rtps/common/types.hpp"

#include <bit>
#include <cstdint>
#include <cstring>
#include <span>
#include <vector>

namespace rtps {
namespace rpc {

/// Size of the CDR encapsulation header that starts every payload.
constexpr std::size_t CDR_ENCAPSULATION_SIZE = 4;
/// Size of the in-payload header rmw_cyclonedds_cpp puts after the encapsulation.
constexpr std::size_t CYCLONEDDS_REQUEST_HEADER_SIZE = 16;

/// cdds_request_header_t: {guid, seq}, both opaque to the server, which echoes
/// them. A client chooses its own guid (unique among the clients of a service)
/// and a seq per request.
struct CycloneDdsRequestHeader {
  uint64_t guid{0};
  int64_t seq{0};

  bool operator==(const CycloneDdsRequestHeader &o) const { return guid == o.guid && seq == o.seq; }
};

/// True for a participant whose vendor id is Eclipse Cyclone DDS, i.e. a peer
/// whose services use the in-payload header.
inline bool isCycloneDds(const VendorId_t &vendor) { return vendor == VENDOR_ECLIPSE_CYCLONEDDS; }

/// True when `payload` starts with a CDR encapsulation header, {0x00, kind,
/// options(2)} with kind CDR_BE 0x00, CDR_LE 0x01, PL_CDR_BE 0x02 or PL_CDR_LE
/// 0x03 (XCDR1, what rmw_cyclonedds_cpp and rmw_fastrtps_cpp use for services).
/// Anything else is not an rmw payload: the codec refuses it rather than read a
/// header out of it, and a responder checks it before replying. The options
/// bytes are not checked: the spec leaves them to the encoder.
inline bool isCdrEncapsulation(std::span<const uint8_t> payload) {
  return payload.size() >= CDR_ENCAPSULATION_SIZE && payload[0] == 0x00 && payload[1] <= 0x03;
}

namespace detail {
// The encapsulation's endianness flag: bit 0 of the kind byte (CDR_LE 0x0001,
// PL_CDR_LE 0x0003 are little-endian; CDR_BE 0x0000, PL_CDR_BE 0x0002 big).
// The caller has checked isCdrEncapsulation().
inline bool encapsulationIsLittleEndian(std::span<const uint8_t> payload) {
  return (payload[1] & 0x01) != 0;
}
inline uint64_t loadU64(const uint8_t *p, bool littleEndian) {
  uint64_t v = 0;
  for (int i = 0; i < 8; ++i) {
    const int shift = littleEndian ? 8 * i : 8 * (7 - i);
    v |= static_cast<uint64_t>(p[i]) << shift;
  }
  return v;
}
inline void storeU64(uint8_t *p, uint64_t v, bool littleEndian) {
  for (int i = 0; i < 8; ++i) {
    const int shift = littleEndian ? 8 * i : 8 * (7 - i);
    p[i] = static_cast<uint8_t>((v >> shift) & 0xFF);
  }
}
// seq is an int64 on the wire as its two's-complement bit pattern; bit_cast
// keeps the bits as they are (a static_cast of a value above INT64_MAX would
// rely on the implementation's conversion).
inline int64_t loadI64(const uint8_t *p, bool littleEndian) {
  return std::bit_cast<int64_t>(loadU64(p, littleEndian));
}
inline void storeI64(uint8_t *p, int64_t v, bool littleEndian) {
  storeU64(p, std::bit_cast<uint64_t>(v), littleEndian);
}
} // namespace detail

/// Parse the header of an rmw_cyclonedds_cpp request/reply payload
/// (encapsulation + 16-byte header + body). Returns false when the payload does
/// not start with a CDR encapsulation or is too short to hold the header.
inline bool parseCycloneDdsRequestHeader(std::span<const uint8_t> payload,
                                         CycloneDdsRequestHeader &out) {
  if (payload.size() < CDR_ENCAPSULATION_SIZE + CYCLONEDDS_REQUEST_HEADER_SIZE ||
      !isCdrEncapsulation(payload)) {
    return false;
  }
  const bool le = detail::encapsulationIsLittleEndian(payload);
  const uint8_t *p = payload.data() + CDR_ENCAPSULATION_SIZE;
  out.guid = detail::loadU64(p, le);
  out.seq = detail::loadI64(p + 8, le);
  return true;
}

/// The payload without its Cyclone header: encapsulation + body, i.e. the
/// bytes rmw_fastrtps_cpp / espp would have sent for the same message. Empty
/// when the payload is not a CDR encapsulation or too short to hold the header
/// (the same contract as prependCycloneDdsRequestHeader()).
inline std::vector<uint8_t> stripCycloneDdsRequestHeader(std::span<const uint8_t> payload) {
  if (payload.size() < CDR_ENCAPSULATION_SIZE + CYCLONEDDS_REQUEST_HEADER_SIZE ||
      !isCdrEncapsulation(payload)) {
    return {};
  }
  std::vector<uint8_t> out;
  out.reserve(payload.size() - CYCLONEDDS_REQUEST_HEADER_SIZE);
  out.insert(out.end(), payload.begin(), payload.begin() + CDR_ENCAPSULATION_SIZE);
  out.insert(out.end(), payload.begin() + CDR_ENCAPSULATION_SIZE + CYCLONEDDS_REQUEST_HEADER_SIZE,
             payload.end());
  return out;
}

/// The payload with a Cyclone header spliced in after its encapsulation, in the
/// encapsulation's endianness. `payload` must start with a CDR encapsulation
/// header; an empty result signals one that is missing or not recognized.
inline std::vector<uint8_t> prependCycloneDdsRequestHeader(const CycloneDdsRequestHeader &header,
                                                           std::span<const uint8_t> payload) {
  if (!isCdrEncapsulation(payload)) {
    return {};
  }
  const bool le = detail::encapsulationIsLittleEndian(payload);
  std::vector<uint8_t> out(payload.size() + CYCLONEDDS_REQUEST_HEADER_SIZE);
  std::memcpy(out.data(), payload.data(), CDR_ENCAPSULATION_SIZE);
  detail::storeU64(out.data() + CDR_ENCAPSULATION_SIZE, header.guid, le);
  detail::storeI64(out.data() + CDR_ENCAPSULATION_SIZE + 8, header.seq, le);
  std::memcpy(out.data() + CDR_ENCAPSULATION_SIZE + CYCLONEDDS_REQUEST_HEADER_SIZE,
              payload.data() + CDR_ENCAPSULATION_SIZE, payload.size() - CDR_ENCAPSULATION_SIZE);
  return out;
}

/// A 64-bit client id for the header's guid field, derived from an endpoint
/// GUID (FNV-1a over its 16 bytes). Replies travel on the shared rr/ topic and
/// a client takes every reply whose guid is its own, so two espp clients of one
/// service that hashed to the same id would accept each other's replies
/// whenever their seq counters coincide (both start at 1): a misdelivery, not
/// noise. The chance is 2^-64 per pair of clients, which is why nothing more is
/// done about it; 0 is not special to the filter, so it needs no remapping.
inline uint64_t cycloneDdsClientId(const Guid_t &guid) {
  uint64_t h = 1469598103934665603ULL;
  auto mix = [&h](uint8_t b) {
    h ^= b;
    h *= 1099511628211ULL;
  };
  for (const auto b : guid.prefix.id) {
    mix(b);
  }
  for (const auto b : guid.entityId.entityKey) {
    mix(b);
  }
  mix(static_cast<uint8_t>(guid.entityId.entityKind));
  return h;
}

} // namespace rpc
} // namespace rtps

#endif // RTPS_RPC_CYCLONEDDS_REQUEST_HEADER_H

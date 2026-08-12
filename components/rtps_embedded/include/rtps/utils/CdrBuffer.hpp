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

This file is part of embeddedRTPS.
*/

#ifndef RTPS_UTILS_CDRBUFFER_H
#define RTPS_UTILS_CDRBUFFER_H

// Thin aliases/helpers over espp's `cdr` stream primitives that reproduce the
// exact byte behavior the engine previously got from the Micro-CDR library:
//  - little-endian by default,
//  - primitive writes/reads natural-aligned relative to the buffer start,
//  - raw byte arrays written/read without alignment,
//  - fixed caller-owned buffers (span_sink), no heap.
// Used for the SPDP/SEDP PL_CDR discovery parameter lists; the byte-identity
// of the output is frozen by pc/tests/rtps_embedded_golden.cpp.

#include <cstdint>
#include <cstring>
#include <span>

#include "cdr/stream.hpp"

namespace rtps {

using CdrSink = cdr::span_sink;
using CdrWriter = cdr::basic_writer<cdr::version::xcdr1, cdr::span_sink>;
using CdrReader = cdr::reader<cdr::version::xcdr1>;

inline std::span<std::byte> asWritableBytes(uint8_t *data, size_t size) {
  return {reinterpret_cast<std::byte *>(data), size};
}

inline std::span<const std::byte> asBytes(const uint8_t *data, size_t size) {
  return {reinterpret_cast<const std::byte *>(data), size};
}

/// Raw (unaligned) byte-array write (Micro-CDR's serialize_array_uint8_t).
inline void writeBytes(CdrWriter &writer, const uint8_t *data, size_t size) {
  writer.write_bytes(asBytes(data, size));
}

/// Raw (unaligned) byte-array read (Micro-CDR's deserialize_array_uint8_t).
inline bool readBytes(CdrReader &reader, uint8_t *dst, size_t size) {
  auto bytes = reader.read_bytes(size);
  if (!bytes) {
    return false;
  }
  std::memcpy(dst, bytes->data(), size);
  return true;
}

/// Advance the read position by up to `size` bytes, clamped to the end of the
/// buffer (Micro-CDR's advance_buffer).
inline void skipBytes(CdrReader &reader, size_t size) {
  (void)reader.seek(std::min(reader.position() + size, reader.total_size()));
}

/// Advance the read position to the next 4-byte boundary (parameter-list
/// element alignment), clamped to the end of the buffer (Micro-CDR's
/// align_to(buffer, 4)).
inline void alignTo4(CdrReader &reader) {
  (void)reader.seek(std::min((reader.position() + 3) & ~size_t{3}, reader.total_size()));
}

} // namespace rtps

#endif // RTPS_UTILS_CDRBUFFER_H

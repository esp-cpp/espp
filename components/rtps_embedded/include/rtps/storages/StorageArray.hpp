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

#ifndef RTPS_STORAGE_ARRAY_H
#define RTPS_STORAGE_ARRAY_H

// ---------------------------------------------------------------------------
// StorageArray<T, N>: the compile-time storage policy for the engine's VALUE
// pools (MemoryPool, SimpleHistoryCache, HistoryCacheWithDeletion,
// ThreadSafeCircularBuffer).
//
// The policy is selected purely at compile time by the limits profile header
// (rtps/config.hpp -> config_*.hpp):
//
//   * ESP32 / "embedded" profile  -> RTPS_STORAGE_DYNAMIC is NOT defined
//         => backed by a fixed std::array<T, N>. This is byte-identical, in
//            behaviour and footprint, to the raw C arrays the engine shipped
//            with. No heap, no runtime capacity, no growth path is compiled in.
//            Determinism for the MCU is preserved.
//
//   * host / host_large profiles  -> RTPS_STORAGE_DYNAMIC IS defined
//         => backed by a std::vector<T> reserved (sized) to N up front, that
//            can grow past the profile cap instead of hard-failing when a pool
//            fills. Heap-backed, non-deterministic, host-only.
//
// StorageArray only stores capacity; it never touches serialized bytes, so the
// golden wire tests stay byte-identical on both paths.
//
// The surface is intentionally minimal - exactly what the pools need:
//   operator[](i), size(), and (dynamic only) ensureSize(n) to grow.
// ---------------------------------------------------------------------------

#include <array>
#include <cstddef>

#include "rtps/config.hpp" // pulls in the profile header that (does not) define
                           // RTPS_STORAGE_DYNAMIC

#ifdef RTPS_STORAGE_DYNAMIC
#include <deque>
#endif

namespace rtps {

#ifdef RTPS_STORAGE_DYNAMIC

// Dynamic (host) storage: heap-backed, sized (reserved) to N up front, growable
// past N.
//
// Backed by std::deque rather than std::vector deliberately: some element types
// stored here (notably CacheChange) are move-ASSIGNABLE but not
// move-CONSTRUCTIBLE, so a std::vector could not relocate them when it grows.
// std::deque grows by appending default-constructed elements and never
// relocates the existing ones, so resize() needs only DefaultConstructible and
// previously handed-out indices/references stay valid. Callers that keep ring
// indices (the history caches / circular buffer) re-linearize themselves after
// ensureSize() using move-assignment only.
template <typename T, std::size_t N> class StorageArray {
public:
  StorageArray()
      : m_data(N) {} // initial size N (value-initialized) == "reserved to N"

  T &operator[](std::size_t i) { return m_data[i]; }
  const T &operator[](std::size_t i) const { return m_data[i]; }

  std::size_t size() const { return m_data.size(); }

  /// Grow so that at least new_size elements are addressable. Existing elements
  /// are preserved in place; freshly added elements are value-initialized. Only
  /// ever grows (never shrinks).
  void ensureSize(std::size_t new_size) {
    if (new_size > m_data.size()) {
      m_data.resize(new_size);
    }
  }

private:
  std::deque<T> m_data;
};

#else

// Static (embedded) storage: fixed array, no heap, no growth. Zero-overhead;
// codegen is identical to the raw `T m_data[N]` the engine used before.
template <typename T, std::size_t N> class StorageArray {
public:
  T &operator[](std::size_t i) { return m_data[i]; }
  const T &operator[](std::size_t i) const { return m_data[i]; }

  static constexpr std::size_t size() { return N; }

  // ensureSize() intentionally does not exist on the static path: any attempt
  // to grow a fixed pool is a compile error, guaranteeing no growth code is
  // reachable on the MCU.

private:
  std::array<T, N> m_data{};
};

#endif // RTPS_STORAGE_DYNAMIC

} // namespace rtps

#endif // RTPS_STORAGE_ARRAY_H

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

#ifndef RTPS_PAYLOADALLOCATOR_H
#define RTPS_PAYLOADALLOCATOR_H

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>
#include <vector>

#if defined(ESP_PLATFORM)
#include "esp_heap_caps.h"
#include "sdkconfig.h"
#endif

namespace rtps {

/// Allocator for the engine's payload bytes: the history caches' samples, the
/// messages it builds to send and the datagrams it parses.
///
/// Those buffers are mostly a few hundred bytes to a datagram, below the usual
/// CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL threshold, so a plain malloc() puts
/// them in internal RAM first. On a busy domain they add up: a participant
/// that joins (or restarts into) a domain with a dozen live peers takes every
/// peer's discovery announcements at once, and the internal RAM a task stack
/// or a FreeRTOS lock needs runs out. With CONFIG_RTPS_PAYLOAD_PREFER_PSRAM
/// (default on when PSRAM is enabled) they are allocated from PSRAM first and
/// from internal RAM only when PSRAM cannot satisfy them. Elsewhere (no PSRAM,
/// the option off, the host) this is plain operator new / delete.
template <class T> struct PayloadAllocator {
  using value_type = T;

  PayloadAllocator() noexcept = default;
  template <class U> constexpr PayloadAllocator(const PayloadAllocator<U> &) noexcept {}

  T *allocate(std::size_t n) {
#if defined(ESP_PLATFORM) && CONFIG_RTPS_PAYLOAD_PREFER_PSRAM
    if (n > static_cast<std::size_t>(-1) / sizeof(T)) {
      allocation_failed();
    }
    void *p = heap_caps_malloc_prefer(n * sizeof(T), 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
                                      MALLOC_CAP_DEFAULT);
    if (p == nullptr) {
      allocation_failed();
    }
    return static_cast<T *>(p);
#else
    return static_cast<T *>(::operator new(n * sizeof(T)));
#endif
  }

  void deallocate(T *p, std::size_t) noexcept {
#if defined(ESP_PLATFORM) && CONFIG_RTPS_PAYLOAD_PREFER_PSRAM
    heap_caps_free(p);
#else
    ::operator delete(p);
#endif
  }

  template <class U> bool operator==(const PayloadAllocator<U> &) const noexcept { return true; }
  template <class U> bool operator!=(const PayloadAllocator<U> &) const noexcept { return false; }

private:
  [[noreturn]] static void allocation_failed() {
    // what operator new does when it cannot allocate: bad_alloc, or (built
    // without exceptions) the program ends
#if defined(__cpp_exceptions) || defined(__EXCEPTIONS)
    throw std::bad_alloc();
#else
    std::abort();
#endif
  }
};

/// The byte vector the engine keeps payloads in (see PayloadAllocator).
using PayloadBytes = std::vector<uint8_t, PayloadAllocator<uint8_t>>;

} // namespace rtps

#endif // RTPS_PAYLOADALLOCATOR_H

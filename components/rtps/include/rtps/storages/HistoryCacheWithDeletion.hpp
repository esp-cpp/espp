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

#ifndef HISTORYCACHEWITHDELETION_H
#define HISTORYCACHEWITHDELETION_H

#include <array>
#include <limits>
#include <stdint.h>

#include "rtps/config.hpp"
#include "rtps/storages/CacheChange.hpp"
#include "rtps/storages/StorageArray.hpp"

namespace rtps {

/**
 * Extension of the SimpleHistoryCache that allows for deletion operation at the
 * cost of efficieny
 * TODO: Replace with something better in the future!
 * Likely only used for SEDP
 */
template <uint16_t SIZE> class HistoryCacheWithDeletion {
public:
  HistoryCacheWithDeletion() = default;

  uint32_t m_dispose_after_write_cnt = 0;

  bool isFull() const {
    uint16_t it = m_head;
    incrementIterator(it);
    return it == m_tail;
  }

  const CacheChange *addChange(const uint8_t *data, DataSize_t size, bool inLineQoS,
                               bool disposeAfterWrite, bool hasRelatedSampleIdentity = false,
                               const rpc::SampleIdentity &relatedSampleIdentity = {}) {
    CacheChange change;
    change.kind = ChangeKind_t::ALIVE;
    change.inLineQoS = inLineQoS;
    change.disposeAfterWrite = disposeAfterWrite;
    change.hasRelatedSampleIdentity = hasRelatedSampleIdentity;
    change.relatedSampleIdentity = relatedSampleIdentity;
    change.data.reserve(size);
    change.data.append(data, size);
    change.sequenceNumber = ++m_lastUsedSequenceNumber;

    if (disposeAfterWrite) {
      m_dispose_after_write_cnt++;
    }

#ifdef RTPS_STORAGE_DYNAMIC
    // Host: grow (retain) instead of dropping the oldest change when full. Done
    // before capturing `place` so growth cannot invalidate the pointer.
    if (isFull()) {
      grow();
    }
#endif

    CacheChange *place = &m_buffer[m_head];
    incrementHead();

    *place = std::move(change);
    return place;
  }

  const CacheChange *addChange(const uint8_t *data, DataSize_t size) {
    return addChange(data, size, 0, false);
  }

  void removeUntilIncl(SequenceNumber_t sn) {
    if (m_head == m_tail) {
      return;
    }

    if (getCurrentSeqNumMax() <= sn) { // We won't overrun head
      m_head = m_tail;
      return;
    }

    while (m_buffer[m_tail].sequenceNumber <= sn) {
      incrementTail();
    }
  }

  void dropOldest() { removeUntilIncl(getCurrentSeqNumMin()); }

  bool dropChange(const SequenceNumber_t &sn) {
    uint16_t idx_to_clear;
    CacheChange *change;
    if (!getChangeBySN(sn, &change, idx_to_clear)) {
      return false; // sn does not exist, nothing to do
    }

    if (idx_to_clear == m_tail) {
      m_buffer[m_tail].reset();
      incrementTail();
      return true;
    }

    uint16_t prev = idx_to_clear;
    do {
      prev = idx_to_clear - 1;
      if (prev >= m_buffer.size()) {
        prev = m_buffer.size() - 1;
      }

      m_buffer[idx_to_clear] = std::move(m_buffer[prev]);
      idx_to_clear = prev;

    } while (prev != m_tail);

    incrementTail();

    return true;
  }

  bool setCacheChangeKind(const SequenceNumber_t &sn, ChangeKind_t kind) {
    CacheChange *change = getChangeBySN(sn);
    if (change == nullptr) {
      return false;
    }

    change->kind = kind;
    return true;
  }

  CacheChange *getChangeBySN(SequenceNumber_t sn) {
    CacheChange *change;
    uint16_t position;
    if (getChangeBySN(sn, &change, position)) {
      return change;
    } else {
      return nullptr;
    }
  }

  bool isEmpty() const { return (m_head == m_tail); }

  const SequenceNumber_t &getCurrentSeqNumMin() const {
    if (m_head == m_tail) {
      return SEQUENCENUMBER_UNKNOWN;
    } else {
      return m_buffer[m_tail].sequenceNumber;
    }
  }

  const SequenceNumber_t &getCurrentSeqNumMax() const {
    if (m_head == m_tail) {
      return SEQUENCENUMBER_UNKNOWN;
    } else {
      return m_lastUsedSequenceNumber;
    }
  }

  const SequenceNumber_t &getLastUsedSequenceNumber() const { return m_lastUsedSequenceNumber; }

  void clear() {
    m_head = 0;
    m_tail = 0;
    m_lastUsedSequenceNumber = {0, 0};
  }
#ifdef DEBUG_HISTORY_CACHE_WITH_DELETION
  void print() const {
    for (unsigned int i = 0; i < m_buffer.size(); i++) {
      std::cout << "[" << i << "] "
                << " SN = " << m_buffer[i].sequenceNumber.low;
      switch (m_buffer[i].kind) {
      case ChangeKind_t::ALIVE:
        std::cout << " Type = ALIVE";
        break;
      case ChangeKind_t::INVALID:
        std::cout << " Type = INVALID";
        break;
      case ChangeKind_t::NOT_ALIVE_DISPOSED:
        std::cout << " Type = DISPOSED";
        break;
      }
      if (m_head == i) {
        std::cout << " <- HEAD";
      }
      if (m_tail == i) {
        std::cout << " <- TAIL";
      }
      std::cout << std::endl;
    }
  }
#endif
  bool isSNInRange(const SequenceNumber_t &sn) const {
    if (isEmpty()) {
      return false;
    }
    SequenceNumber_t minSN = getCurrentSeqNumMin();
    if (sn < minSN || getCurrentSeqNumMax() < sn) {
      return false;
    }
    return true;
  }

private:
  // Static path: fixed std::array<CacheChange, SIZE + 1> (byte-identical to
  // before). Dynamic path: heap-backed, reserved to SIZE + 1, grown by grow().
  StorageArray<CacheChange, SIZE + 1> m_buffer;
  uint16_t m_head = 0;
  uint16_t m_tail = 0;
  static_assert(sizeof(SIZE) <= sizeof(m_head), "Iterator is large enough for given size");

  SequenceNumber_t m_lastUsedSequenceNumber{0, 0};

#ifdef RTPS_STORAGE_DYNAMIC
  // Host only: double the ring capacity, then re-linearize the live changes so
  // the head/tail indices remain valid after the (append-only) resize. The live
  // range [m_tail, m_head) is move-assigned into the freshly grown region
  // [oldCap, ...) in logical order. Uses move-assignment only (CacheChange is
  // move-assignable but not move-constructible).
  void grow() {
    const std::size_t oldCap = m_buffer.size();
    // 16-bit ring indices: refuse to grow past what they can represent, else the
    // m_head/m_tail casts below truncate and corrupt the ring. A history cache
    // never legitimately needs this many entries; it then behaves as full
    // (drop-oldest) at this bound.
    if (oldCap * 2 > std::numeric_limits<uint16_t>::max()) {
      return;
    }
    m_buffer.ensureSize(oldCap * 2);
    std::size_t dst = oldCap;
    uint16_t src = m_tail;
    while (src != m_head) {
      m_buffer[dst] = std::move(m_buffer[src]);
      ++dst;
      ++src;
      if (src >= oldCap) {
        src = 0; // wrap at the OLD capacity
      }
    }
    m_tail = static_cast<uint16_t>(oldCap);
    m_head = static_cast<uint16_t>(dst);
  }
#endif

  bool getChangeBySN(const SequenceNumber_t &sn, CacheChange **out_change,
                     uint16_t &out_buffer_position) {
    if (!isSNInRange(sn)) {
      return false;
    }
    static_assert(std::is_unsigned<decltype(sn.low)>::value, "Underflow well defined");
    static_assert(sizeof(m_tail) <= sizeof(uint16_t), "Cast ist well defined");

    uint16_t cur_idx = m_tail;
    while (cur_idx != m_head) {
      if (m_buffer[cur_idx].sequenceNumber == sn) {
        *out_change = &m_buffer[cur_idx];
        out_buffer_position = cur_idx;
        return true;
      }
      // Sequence numbers are consecutive
      if (m_buffer[cur_idx].sequenceNumber > sn) {
        *out_change = nullptr;
        return false;
      }

      incrementIterator(cur_idx);
    }

    *out_change = nullptr;
    return false;
  }

  inline void incrementHead() {
    incrementIterator(m_head);
    if (m_head == m_tail) {
      // Move without check
      incrementIterator(m_tail); // drop one
    }
  }

  inline void incrementIterator(uint16_t &iterator) const {
    ++iterator;
    if (iterator >= m_buffer.size()) {
      iterator = 0;
    }
  }

  inline void incrementTail() {
    if (m_buffer[m_tail].disposeAfterWrite) {
      m_dispose_after_write_cnt--;
    }
    if (m_head != m_tail) {
      m_buffer[m_tail].reset();
      incrementIterator(m_tail);
    }
  }

protected:
  // This constructor was created for unit testing
  explicit HistoryCacheWithDeletion(SequenceNumber_t lastUsed)
      : HistoryCacheWithDeletion() {
    m_lastUsedSequenceNumber = lastUsed;
  }
};
} // namespace rtps

#endif // HISTORYCACHEWITHDELETION_H

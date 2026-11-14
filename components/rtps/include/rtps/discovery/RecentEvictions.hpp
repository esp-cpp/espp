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

#pragma once

#include "rtps/common/types.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <optional>

namespace rtps {

//! The remote participants this participant evicted lately, and the rule that
//! decides whether an announcing participant may have another one evicted for
//! it (Participant::evictUnusedRemoteParticipant).
//!
//! Why: on a domain that stays bigger than the pool, the participants that do
//! not fit re-announce every period, and with a plain "one eviction per
//! period" cooldown they take turns displacing each other at the eviction
//! rate - one is forgotten, announces again a period later, and takes the slot
//! of the next-oldest, forever. A participant never seen before (the one this
//! participant may actually need) competes with all of them for that one slot
//! per period and can lose every round (it did: a late publisher went
//! unmatched for 90 s on a domain of 28 against 20 slots). So:
//!  - a participant evicted less than `holdoff` ago is HELD OFF: nothing is
//!    evicted for it (it is unused, and it still gets a free slot at once);
//!  - a participant this one remembers evicting, past its hold-off, waits for
//!    the eviction cooldown like before;
//!  - a participant never evicted (or forgotten by this ring) may have one
//!    evicted for it at once, cooldown or not: it is new only once, so this
//!    does not unbound the churn, and it is admitted on its first announcement.
//! A ring of N entries, oldest overwritten; no allocation.
template <std::size_t N> class RecentEvictions {
public:
  using time_point = std::chrono::steady_clock::time_point;
  static_assert(N > 0, "RecentEvictions needs at least one entry");

  enum class Gate {
    Proceed, //!< may evict for this participant now
    HeldOff, //!< evicted lately: do not evict for it
    Cooldown //!< a former evictee, but an eviction happened this period already
  };

  //! Remember that `prefix` was evicted at `now` (refreshes an existing entry,
  //! else takes the ring's next slot).
  void note(const GuidPrefix_t &prefix, const time_point &now) {
    if (auto *entry = find(prefix); entry != nullptr) {
      entry->at = now;
      return;
    }
    auto &slot = m_entries[m_next];
    slot.prefix = prefix;
    slot.at = now;
    slot.valid = true;
    m_next = (m_next + 1) % N;
  }

  //! When `prefix` was last evicted, if the ring still remembers it.
  std::optional<time_point> lastEviction(const GuidPrefix_t &prefix) const {
    const auto *entry = find(prefix);
    return entry != nullptr ? std::optional<time_point>{entry->at} : std::nullopt;
  }

  //! Whether an eviction may be made for `newcomer` at `now`, given the time
  //! of the last eviction (`lastEvictionAt`, a default time_point when none
  //! happened yet), the per-period `cooldown` and the readmission `holdoff`.
  Gate gate(const GuidPrefix_t &newcomer, const time_point &now, const time_point &lastEvictionAt,
            std::chrono::milliseconds cooldown, std::chrono::milliseconds holdoff) const {
    const auto evictedAt = lastEviction(newcomer);
    if (!evictedAt) {
      return Gate::Proceed;
    }
    if (now - *evictedAt < holdoff) {
      return Gate::HeldOff;
    }
    if (lastEvictionAt != time_point{} && now - lastEvictionAt < cooldown) {
      return Gate::Cooldown;
    }
    return Gate::Proceed;
  }

  static constexpr std::size_t capacity() { return N; }

private:
  struct Entry {
    GuidPrefix_t prefix{};
    time_point at{};
    bool valid{false};
  };
  std::array<Entry, N> m_entries{};
  std::size_t m_next{0};

  const Entry *find(const GuidPrefix_t &prefix) const {
    const auto it = std::find_if(m_entries.begin(), m_entries.end(), [&prefix](const Entry &e) {
      return e.valid && e.prefix == prefix;
    });
    return it != m_entries.end() ? &*it : nullptr;
  }
  Entry *find(const GuidPrefix_t &prefix) {
    return const_cast<Entry *>(static_cast<const RecentEvictions &>(*this).find(prefix));
  }
};

} // namespace rtps

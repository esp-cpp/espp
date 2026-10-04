#pragma once

// The byte ring behind espp::ConsoleCapture: a fixed capacity, a monotonic
// byte count, cursor-based reads that report what the ring EVICTED before the
// reader got to it, and a clear() that hides older bytes from readers WITHOUT
// counting them as lost (an intentional discard is not a loss). Optional
// stripping of ANSI CSI sequences on the way in. Host-buildable (standard
// library only) so it is unit-tested in components/desktop/test; does no
// locking (ConsoleCapture holds its mutex around every call).

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace espp::detail {

class ConsoleRing {
public:
  /// @param capacity Bytes kept (at least 1).
  explicit ConsoleRing(size_t capacity = 1)
      : ring_(capacity ? capacity : 1, 0) {}

  size_t capacity() const { return ring_.size(); }
  /// Bytes ever written (monotonic; the cursor space).
  uint64_t total() const { return total_; }
  /// Bytes readable right now: written since the last clear(), at most capacity().
  size_t available() const { return static_cast<size_t>(total_ - oldest()); }

  /// Append bytes; with `strip_ansi` an ESC '[' ... final (0x40..0x7E)
  /// sequence is dropped (a lone ESC that turns out not to start one is kept).
  void write(const uint8_t *data, size_t size, bool strip_ansi) {
    for (size_t i = 0; i < size; ++i) {
      const uint8_t c = data[i];
      if (strip_ansi) {
        if (ansi_state_ == 1) {
          if (c == '[') {
            ansi_state_ = 2;
            continue;
          }
          ansi_state_ = 0;
          put(0x1B); // the pending ESC was not a CSI: keep it
        } else if (ansi_state_ == 2) {
          if (c >= 0x40 && c <= 0x7E)
            ansi_state_ = 0;
          continue;
        }
        if (c == 0x1B) {
          ansi_state_ = 1;
          continue;
        }
      }
      put(c);
    }
  }

  /**
   * @brief Copy the bytes after `*cursor` (at most max_bytes), advancing it.
   * @param cursor In: the reader's position (0 = the oldest readable byte);
   *        out: the position after the bytes returned. A cursor behind the
   *        oldest readable byte is moved forward.
   * @param dropped If not null: how many bytes the ring EVICTED (capacity
   *        overwrite) before the reader consumed them; bytes hidden by a
   *        clear() are skipped without being counted.
   * @return The number of bytes appended to `out`.
   */
  size_t read_since(uint64_t *cursor, std::string &out, size_t max_bytes,
                    size_t *dropped = nullptr) const {
    const uint64_t evicted = total_ > ring_.size() ? total_ - ring_.size() : 0;
    uint64_t pos = (cursor && *cursor) ? *cursor : oldest();
    size_t lost = 0;
    if (pos < evicted) {
      lost = static_cast<size_t>(evicted - pos);
      pos = evicted;
    }
    if (pos < cleared_at_)
      pos = cleared_at_; // an intentional discard: skipped, not lost
    if (pos > total_)
      pos = total_;
    if (dropped)
      *dropped = lost;
    const size_t n = std::min(static_cast<size_t>(total_ - pos), max_bytes);
    out.reserve(out.size() + n);
    for (size_t i = 0; i < n; ++i)
      out.push_back(static_cast<char>(ring_[static_cast<size_t>((pos + i) % ring_.size())]));
    if (cursor)
      *cursor = pos + n;
    return n;
  }

  /// Hide everything written so far from readers (they resume at total()).
  void clear() { cleared_at_ = total_; }

private:
  /// The oldest byte a reader may still get.
  uint64_t oldest() const {
    const uint64_t evicted = total_ > ring_.size() ? total_ - ring_.size() : 0;
    return std::max(evicted, cleared_at_);
  }
  void put(uint8_t c) {
    ring_[static_cast<size_t>(total_ % ring_.size())] = c;
    ++total_;
  }

  std::vector<uint8_t> ring_;
  uint64_t total_{0};
  uint64_t cleared_at_{0};
  uint8_t ansi_state_{0}; ///< 0 text, 1 after ESC, 2 inside a CSI
};

} // namespace espp::detail

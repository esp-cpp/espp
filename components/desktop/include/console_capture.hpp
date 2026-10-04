#pragma once

// espp::ConsoleCapture -- keeps the last N bytes of everything the firmware
// prints (ESP_LOG, espp::Logger / fmt::print, printf, stderr) in a byte ring
// that an application can read back, e.g. the desktop's Log Viewer app or a
// "download logs" console. The original console keeps working: the capture
// is a tee, not a redirect.
//
// How: a tiny write-only VFS device (/dev/logcap) is registered and stdout /
// stderr are freopen'ed onto it; its write() forwards the bytes to the
// original console (/dev/console, which carries the primary and any secondary
// console; falling back to the raw UART / USB-Serial-JTAG device) and appends
// them to the ring. Nothing blocks on a reader: a write costs a memcpy under
// a short mutex hold plus the console write it would have done anyway.
//
// Mutually exclusive with espp::UsbDevice::route_console_to_cdc(): both
// re-point stdout, whichever runs last wins. Install this first thing in
// app_main() so the boot log is captured.

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <system_error>
#include <vector>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_vfs.h"
#include "sdkconfig.h"

#include "detail/console_ring.hpp"

namespace espp {

/**
 * @brief Captures stdout / stderr into a bounded byte ring while teeing them
 *        to the original console.
 *
 * A process-wide singleton (there is one stdout): install() once, then any
 * task may call read_since() with its own cursor to page through the log
 * without blocking the writers. The cursor is an absolute byte position
 * (total_bytes()); when the ring overwrote bytes the reader had not consumed,
 * read_since() reports how many were dropped and resumes at the oldest byte
 * still kept.
 *
 * \section console_capture_ex1 ConsoleCapture Example
 * \snippet desktop_example.cpp console_capture
 */
class ConsoleCapture {
public:
  /// Configuration for install().
  struct Config {
    size_t capacity_bytes{16 * 1024}; ///< Ring size: the newest bytes kept.
    /// Keep writing to the original console (the UART / USB-Serial-JTAG
    /// monitor). Off = capture only (the console goes quiet).
    bool tee_to_console{true};
    /// Remove ANSI CSI escape sequences (colors) from the captured bytes; the
    /// tee still gets them. Leave off when the consumer renders ANSI itself
    /// (the desktop Log Viewer does).
    bool strip_ansi{false};
  };

  /// VFS path of the capture device (must be <= ESP_VFS_PATH_MAX).
  static constexpr const char *kVfsPath = "/dev/logcap";

  /**
   * @brief Register the capture device and re-point stdout / stderr at it.
   * @param config Ring size and tee / strip options.
   * @param ec Set on failure (io_error: the VFS could not be registered or
   *        stdout could not be re-opened -- the original console is restored
   *        on a best-effort basis).
   * @return true on success, or when already installed (the config of the
   *         first install stays; the tee / strip flags are updated).
   */
  static bool install(const Config &config, std::error_code &ec) {
    ec.clear();
    State &s = state();
    std::lock_guard<std::mutex> lock(s.mutex);
    if (s.installed) {
      s.tee.store(config.tee_to_console);
      s.strip_ansi.store(config.strip_ansi);
      reconcile_tee(s, config.tee_to_console);
      return true;
    }
    s.ring = detail::ConsoleRing(config.capacity_bytes);
    s.tee.store(config.tee_to_console);
    s.strip_ansi.store(config.strip_ansi);
    std::fflush(stdout);
    std::fflush(stderr);
    reconcile_tee(s, config.tee_to_console);
    esp_vfs_t vfs = {};
    vfs.flags = ESP_VFS_FLAG_DEFAULT;
    // The classic (context-pointer-less) esp_vfs_t members are deprecated in
    // IDF v6 but exactly right for this write-only sink (same as usb_device's
    // console routing); use them deliberately and silence the notice.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    vfs.open = &vfs_open;
    vfs.write = &vfs_write;
    vfs.close = &vfs_close;
    vfs.fstat = &vfs_fstat;
#pragma GCC diagnostic pop
    if (esp_vfs_register(kVfsPath, &vfs, nullptr) != ESP_OK) {
      reconcile_tee(s, false);
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    s.installed = true; // published before freopen: the write callback is live
    // freopen closes the stream's previous target even when opening the new
    // one fails, so on any failure restore both streams to the original
    // console (best effort), undo the VFS + tee, and report.
    auto rollback = [&]() {
      s.installed = false;
      // cppcheck-suppress ignoredReturnValue
      std::freopen(original_console_path(), "w", stdout);
      // cppcheck-suppress ignoredReturnValue
      std::freopen(original_console_path(), "w", stderr);
      esp_vfs_unregister(kVfsPath);
      reconcile_tee(s, false);
      ec = std::make_error_code(std::errc::io_error);
      return false;
    };
    if (std::freopen(kVfsPath, "w", stdout) == nullptr)
      return rollback();
    // stderr too (ESP-IDF's panic / abort paths bypass stdio and are not
    // captured; everything that goes through stdio is).
    if (std::freopen(kVfsPath, "w", stderr) == nullptr)
      return rollback();
    // line-buffered: each log line reaches the ring (and the console) as one
    // write, so a reader never sees a torn line for ordinary logging
    std::setvbuf(stdout, nullptr, _IOLBF, 256);
    std::setvbuf(stderr, nullptr, _IONBF, 0);
    return true;
  }

  /// @brief Whether install() succeeded.
  static bool installed() {
    State &s = state();
    std::lock_guard<std::mutex> lock(s.mutex);
    return s.installed;
  }

  /// @brief The ring capacity in bytes.
  static size_t capacity() {
    State &s = state();
    std::lock_guard<std::mutex> lock(s.mutex);
    return s.ring.capacity();
  }

  /// @brief Bytes captured since boot (monotonic; a cursor value).
  static uint64_t total_bytes() {
    State &s = state();
    std::lock_guard<std::mutex> lock(s.mutex);
    return s.ring.total();
  }

  /// @brief Bytes currently kept in the ring (readable).
  static size_t available() {
    State &s = state();
    std::lock_guard<std::mutex> lock(s.mutex);
    return s.ring.available();
  }

  /**
   * @brief Copy the bytes captured after `*cursor` (at most max_bytes) and
   *        advance the cursor.
   * @param cursor In: where the reader is (0 = from the oldest readable byte);
   *        out: the position after the bytes returned.
   * @param out Appended with the bytes.
   * @param max_bytes Upper bound on the bytes appended in this call.
   * @param dropped If not null, set to the number of bytes the ring EVICTED
   *        (capacity overwrite) before the reader got to them (0 when none);
   *        bytes hidden by clear() are skipped without being counted.
   * @return The number of bytes appended.
   */
  static size_t read_since(uint64_t *cursor, std::string &out, size_t max_bytes,
                           size_t *dropped = nullptr) {
    State &s = state();
    std::lock_guard<std::mutex> lock(s.mutex);
    return s.ring.read_since(cursor, out, max_bytes, dropped);
  }

  /// @brief Forget everything captured so far (readers resume at total_bytes()).
  static void clear() {
    State &s = state();
    std::lock_guard<std::mutex> lock(s.mutex);
    s.ring.clear();
  }

  /// @brief Switch the tee to the original console on / off.
  static void set_tee_to_console(bool enable) {
    State &s = state();
    std::lock_guard<std::mutex> lock(s.mutex);
    s.tee.store(enable);
    reconcile_tee(s, enable);
  }

  static bool tee_to_console() { return state().tee.load(); }

private:
  struct State {
    std::mutex mutex; ///< guards the ring
    detail::ConsoleRing ring{1};
    bool installed{false};
    std::atomic<bool> tee{true};
    std::atomic<bool> strip_ansi{false};
    std::mutex tee_mutex; ///< holds tee_fd open across a write (reconcile closes under it)
    int tee_fd{-1};
  };

  static State &state() {
    static State s;
    return s;
  }

  /// The device the console was on before install(): /dev/console (primary +
  /// secondary) when the SDK provides it, else the raw device.
  static const char *original_console_path() { return "/dev/console"; }

  static int open_original_console() {
    int fd = ::open(original_console_path(), O_WRONLY);
    if (fd >= 0)
      return fd;
#if defined(CONFIG_ESP_CONSOLE_UART_NUM)
    char path[16];
    std::snprintf(path, sizeof(path), "/dev/uart/%d", CONFIG_ESP_CONSOLE_UART_NUM);
    fd = ::open(path, O_WRONLY);
    if (fd >= 0)
      return fd;
#endif
#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG) ||                                                 \
    defined(CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG)
    fd = ::open("/dev/usbserjtag", O_WRONLY);
#endif
    return fd;
  }

  /// Open / close the tee fd to match `want`. Serialized with the writers on
  /// tee_mutex so an fd is never closed under a write() in flight.
  static void reconcile_tee(State &s, bool want) {
    std::lock_guard<std::mutex> lock(s.tee_mutex);
    if (want && s.tee_fd < 0) {
      s.tee_fd = open_original_console();
    } else if (!want && s.tee_fd >= 0) {
      ::close(s.tee_fd);
      s.tee_fd = -1;
    }
  }

  static int vfs_open(const char *, int, int) { return 0; }
  static int vfs_close(int) { return 0; }
  static int vfs_fstat(int, struct stat *st) {
    *st = {};
    st->st_mode = S_IFCHR; // a character device: stdio line-buffers it
    return 0;
  }

  /// Runs on whichever task writes to stdout / stderr: tee, then ring.
  static ssize_t vfs_write(int, const void *data, size_t size) {
    State &s = state();
    if (s.tee.load()) {
      // the console write (possibly blocking on the UART) is serialized on
      // its own mutex, kept apart from the ring's so readers never wait on it
      std::lock_guard<std::mutex> tee_lock(s.tee_mutex);
      if (s.tee_fd >= 0)
        ::write(s.tee_fd, data, size);
    }
    std::lock_guard<std::mutex> lock(s.mutex);
    s.ring.write(static_cast<const uint8_t *>(data), size, s.strip_ansi.load());
    return static_cast<ssize_t>(size);
  }
};

} // namespace espp

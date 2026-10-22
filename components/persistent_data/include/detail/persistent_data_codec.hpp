#pragma once

// File format, version list and file I/O helpers for espp::PersistentData.
// Standard library + alpaca only so it can be built and tested on the host
// (see test/persistent_data_host_test.cpp); espp::PersistentData adds the
// logger, the mutex and the Config on top of these functions.

#include <algorithm>
#include <array>
#include <cerrno>
#include <concepts>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <span>
#include <string>
#include <system_error>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#if __has_include(<unistd.h>)
#include <unistd.h>
#endif

#include "serialization.hpp"

#include "persistent_data_errc.hpp"

namespace espp::persistent_data {

/// @brief Serialization options used for the payload.
/// @details `with_version` embeds a hash of the type's field types, so a
///          payload never decodes as a type with a different layout even if a
///          version number was reused by mistake; `strict` rejects payloads
///          that are shorter than the type; `fixed_length_encoding` keeps the
///          encoding independent of the values stored.
inline constexpr alpaca::options SERIALIZATION_OPTIONS = alpaca::options::fixed_length_encoding |
                                                         alpaca::options::with_version |
                                                         alpaca::options::strict;

/// @brief Magic number at the start of every file ("ESPD" stored little-endian).
inline constexpr uint32_t MAGIC = 0x44505345;

/// @brief Size of the file header in bytes.
/// @details Layout (all fields little-endian):
///   - u32 magic   (MAGIC)
///   - u32 version (schema version of the payload, the type's `VERSION`)
///   - u32 length  (payload length in bytes)
///   - u32 crc32   (over magic, version, length and the payload)
///   - payload     (alpaca, SERIALIZATION_OPTIONS)
inline constexpr size_t HEADER_SIZE = 16;

inline void put_u32(uint8_t *p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
  p[2] = static_cast<uint8_t>(v >> 16);
  p[3] = static_cast<uint8_t>(v >> 24);
}

inline uint32_t get_u32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

/// @brief A type usable as a schema version: an aggregate with a `static constexpr uint32_t
///        VERSION` member.
template <typename T>
concept VersionedType = std::is_aggregate_v<T> && requires {
  { T::VERSION } -> std::convertible_to<uint32_t>;
};

/// @brief `Next` can be produced from `Prev` by `Next::migrate(const Prev &)`.
template <typename Prev, typename Next>
concept MigratableFrom = requires(const Prev &prev) {
  { Next::migrate(prev) } -> std::convertible_to<Next>;
};

/// @brief Encode `value` (schema version `version`) into the file format.
/// @return true on success; on failure ec is set and `out` is unspecified.
template <typename T>
bool encode(const T &value, uint32_t version, std::vector<uint8_t> &out, std::error_code &ec) {
  out.clear();
  out.resize(HEADER_SIZE);
  // alpaca appends to the vector, so the payload lands right after the header
  size_t payload_len = espp::serialize<SERIALIZATION_OPTIONS>(value, out);
  if (payload_len == 0 || out.size() != HEADER_SIZE + payload_len) {
    ec = PersistentDataErrc::serialize_failed;
    return false;
  }
  put_u32(out.data(), MAGIC);
  put_u32(out.data() + 4, version);
  put_u32(out.data() + 8, static_cast<uint32_t>(payload_len));
  // crc over magic, version and length (the crc field itself is skipped) plus the payload
  uint32_t crc = crc32_fast(out.data(), 12);
  crc = crc32_fast(out.data() + HEADER_SIZE, payload_len, crc);
  put_u32(out.data() + 12, crc);
  ec.clear();
  return true;
}

/// @brief Validate the header + checksum of an encoded file.
/// @param[in] bytes The whole file
/// @param[out] version The schema version recorded in the header
/// @param[out] payload The payload bytes (a view into `bytes`)
/// @return true if the file is well-formed; ec says why not otherwise.
inline bool parse(std::span<const uint8_t> bytes, uint32_t &version,
                  std::span<const uint8_t> &payload, std::error_code &ec) {
  if (bytes.size() < HEADER_SIZE || get_u32(bytes.data()) != MAGIC) {
    ec = PersistentDataErrc::bad_header;
    return false;
  }
  uint32_t length = get_u32(bytes.data() + 8);
  if (length != bytes.size() - HEADER_SIZE) {
    // truncated write, or trailing bytes that we never wrote
    ec = PersistentDataErrc::bad_header;
    return false;
  }
  uint32_t crc = crc32_fast(bytes.data(), 12);
  crc = crc32_fast(bytes.data() + HEADER_SIZE, length, crc);
  if (crc != get_u32(bytes.data() + 12)) {
    ec = PersistentDataErrc::bad_checksum;
    return false;
  }
  version = get_u32(bytes.data() + 4);
  payload = bytes.subspan(HEADER_SIZE);
  ec.clear();
  return true;
}

/// @brief Decode a payload as type T.
template <typename T>
bool decode_payload(std::span<const uint8_t> payload, T &out, std::error_code &ec) {
  std::error_code alpaca_ec;
  out = alpaca::deserialize<SERIALIZATION_OPTIONS, T>(payload, payload.size(), alpaca_ec);
  if (alpaca_ec) {
    ec = PersistentDataErrc::deserialize_failed;
    return false;
  }
  ec.clear();
  return true;
}

/**
 * @brief An ordered list of schema versions of one data structure, newest first.
 * @tparam Versions Aggregate types, each with `static constexpr uint32_t VERSION`
 *         (strictly decreasing along the list); the first is the current type.
 *         Every type except the last must provide `static T migrate(const Older &)`
 *         from the type after it, so an old file is migrated one step at a
 *         time toward the front of the list.
 */
template <VersionedType... Versions> struct VersionList {
  static_assert(sizeof...(Versions) > 0, "at least one version type is required");

  static constexpr size_t count = sizeof...(Versions);
  template <size_t I> using At = std::tuple_element_t<I, std::tuple<Versions...>>;
  using Current = At<0>;
  static constexpr std::array<uint32_t, count> versions{
      static_cast<uint32_t>(Versions::VERSION)...};
  static constexpr uint32_t current_version = versions[0];

private:
  static constexpr bool strictly_decreasing() {
    for (size_t i = 1; i < count; i++) {
      if (versions[i] >= versions[i - 1])
        return false;
    }
    return true;
  }
  static_assert(strictly_decreasing(),
                "VERSION values must strictly decrease along the list (newest first)");

  template <size_t... Is> static constexpr bool chain_migratable(std::index_sequence<Is...>) {
    return (MigratableFrom<At<Is + 1>, At<Is>> && ...);
  }
  static_assert(chain_migratable(std::make_index_sequence<count - 1>{}),
                "every version except the last needs 'static T migrate(const Older &)' from "
                "the version after it in the list");

public:
  /// @brief Whether a file of schema version `version` can be loaded.
  static constexpr bool supports(uint32_t version) {
    return std::any_of(versions.begin(), versions.end(),
                       [version](uint32_t v) { return v == version; });
  }

  /// @brief Run the migration chain from the version at index I up to Current (index 0).
  template <size_t I> static Current migrate_from(At<I> &&value) {
    if constexpr (I == 0) {
      return std::move(value);
    } else {
      return migrate_from<I - 1>(At<I - 1>::migrate(value));
    }
  }

  /// @brief Decode a payload of schema version `version` and migrate it to Current.
  /// @return true on success; ec is unknown_version or deserialize_failed otherwise.
  static bool decode(uint32_t version, std::span<const uint8_t> payload, Current &out,
                     std::error_code &ec) {
    return decode_impl(version, payload, out, ec, std::make_index_sequence<count>{});
  }

private:
  template <size_t I>
  static bool decode_at(std::span<const uint8_t> payload, Current &out, std::error_code &ec) {
    At<I> value{};
    if (!decode_payload(payload, value, ec))
      return false;
    out = migrate_from<I>(std::move(value));
    return true;
  }

  template <size_t... Is>
  static bool decode_impl(uint32_t version, std::span<const uint8_t> payload, Current &out,
                          std::error_code &ec, std::index_sequence<Is...>) {
    bool ok = false;
    bool matched =
        ((versions[Is] == version ? (ok = decode_at<Is>(payload, out, ec), true) : false) || ...);
    if (!matched) {
      ec = PersistentDataErrc::unknown_version;
      return false;
    }
    return ok;
  }
};

// ------------------------------- file helpers --------------------------------

/// @brief Whether a regular file exists at `path`.
inline bool file_exists(const std::filesystem::path &path) {
  std::error_code ec;
  return std::filesystem::is_regular_file(path, ec);
}

/// @brief Read a whole file.
/// @param max_size Files larger than this are rejected (file_too_large) before
///        any allocation, so a corrupt directory entry cannot exhaust memory.
inline bool read_file(const std::filesystem::path &path, size_t max_size, std::vector<uint8_t> &out,
                      std::error_code &ec) {
  FILE *f = std::fopen(path.c_str(), "rb");
  if (!f) {
    ec = errno == ENOENT ? PersistentDataErrc::file_not_found : PersistentDataErrc::io_error;
    return false;
  }
  bool ok = false;
  do {
    if (std::fseek(f, 0, SEEK_END) != 0) {
      ec = PersistentDataErrc::io_error;
      break;
    }
    long size = std::ftell(f);
    if (size < 0 || std::fseek(f, 0, SEEK_SET) != 0) {
      ec = PersistentDataErrc::io_error;
      break;
    }
    if (static_cast<size_t>(size) > max_size) {
      ec = PersistentDataErrc::file_too_large;
      break;
    }
    out.resize(static_cast<size_t>(size));
    if (size > 0 && std::fread(out.data(), 1, out.size(), f) != out.size()) {
      ec = PersistentDataErrc::io_error;
      break;
    }
    ok = true;
  } while (false);
  std::fclose(f);
  if (ok)
    ec.clear();
  return ok;
}

/// @brief Write `bytes` to `path` atomically.
/// @details Writes `<path>.tmp`, flushes and syncs it, then renames it over
///          `path`, so a reset in the middle of a save leaves the previous file
///          intact. If the filesystem cannot rename over an existing file
///          (SPIFFS), the old file is removed first and the rename retried;
///          that short window is the only non-atomic step.
inline bool write_file_atomic(const std::filesystem::path &path, std::span<const uint8_t> bytes,
                              std::error_code &ec) {
  std::filesystem::path tmp = path;
  tmp += ".tmp";
  FILE *f = std::fopen(tmp.c_str(), "wb");
  if (!f) {
    ec = PersistentDataErrc::io_error;
    return false;
  }
  bool ok = std::fwrite(bytes.data(), 1, bytes.size(), f) == bytes.size();
  ok = (std::fflush(f) == 0) && ok;
#if __has_include(<unistd.h>)
  ok = (fsync(fileno(f)) == 0) && ok;
#endif
  ok = (std::fclose(f) == 0) && ok;
  if (!ok) {
    std::remove(tmp.c_str());
    ec = PersistentDataErrc::io_error;
    return false;
  }
  if (std::rename(tmp.c_str(), path.c_str()) != 0) {
    // some filesystems (SPIFFS) refuse to replace an existing file
    bool retried = false;
    if (errno == EEXIST || errno == ENOTEMPTY) {
      std::remove(path.c_str());
      retried = std::rename(tmp.c_str(), path.c_str()) == 0;
    }
    if (!retried) {
      std::remove(tmp.c_str());
      ec = PersistentDataErrc::io_error;
      return false;
    }
  }
  ec.clear();
  return true;
}

/// @brief Remove the file at `path` (and a leftover `<path>.tmp`).
/// @return true if the file is gone afterwards, including when there was none.
inline bool remove_file(const std::filesystem::path &path, std::error_code &ec) {
  std::filesystem::path tmp = path;
  tmp += ".tmp";
  std::remove(tmp.c_str());
  if (std::remove(path.c_str()) != 0 && errno != ENOENT) {
    ec = PersistentDataErrc::io_error;
    return false;
  }
  ec.clear();
  return true;
}

// --------------------------- load / save a VersionList ------------------------

/// @brief Result of a successful load.
struct LoadInfo {
  uint32_t file_version{0}; ///< Schema version found in the file
  bool migrated{false};     ///< The file was older than Current and was migrated
};

/// @brief Read, validate, decode and migrate the file at `path` into `out`.
template <typename List>
bool load_file(const std::filesystem::path &path, size_t max_size, typename List::Current &out,
               LoadInfo &info, std::error_code &ec) {
  std::vector<uint8_t> bytes;
  if (!read_file(path, max_size, bytes, ec))
    return false;
  uint32_t version = 0;
  std::span<const uint8_t> payload;
  if (!parse(bytes, version, payload, ec))
    return false;
  if (!List::decode(version, payload, out, ec))
    return false;
  info.file_version = version;
  info.migrated = version != List::current_version;
  return true;
}

/// @brief Encode `value` as the current version and write it atomically to `path`.
template <typename List>
bool save_file(const std::filesystem::path &path, const typename List::Current &value,
               std::error_code &ec) {
  std::vector<uint8_t> bytes;
  if (!encode(value, List::current_version, bytes, ec))
    return false;
  return write_file_atomic(path, bytes, ec);
}

} // namespace espp::persistent_data

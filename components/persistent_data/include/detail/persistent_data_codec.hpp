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
#include <deque>
#include <filesystem>
#include <list>
#include <map>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <system_error>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#if __has_include(<unistd.h>)
#include <unistd.h>
#endif

#include "serialization.hpp"

#include "persistent_data_errc.hpp"

namespace espp::persistent_data {

/// @brief Serialization options used for the payload.
/// @details `strict` rejects payloads that are shorter than the type;
///          `fixed_length_encoding` keeps the encoding independent of the
///          values stored. alpaca's own `with_version` is deliberately not used:
///          its hash folds `sizeof(struct)` in, which differs between a 32-bit
///          ESP32 and a 64-bit host, so a file written on one would not load on
///          the other. The header's layout fingerprint (see layout::fingerprint)
///          plays that role portably.
inline constexpr alpaca::options SERIALIZATION_OPTIONS =
    alpaca::options::fixed_length_encoding | alpaca::options::strict;

/// @brief Magic number at the start of every file ("ESPD" stored little-endian).
inline constexpr uint32_t MAGIC = 0x44505345;

/// @brief Size of the file header in bytes.
/// @details Layout (all fields little-endian):
///   - u32 magic   (MAGIC)
///   - u32 version (schema version of the payload, the type's `VERSION`)
///   - u32 layout  (fingerprint of the payload type's layout, layout::fingerprint)
///   - u32 length  (payload length in bytes)
///   - u32 crc32   (over the four fields above and the payload)
///   - payload     (alpaca, SERIALIZATION_OPTIONS)
inline constexpr size_t HEADER_SIZE = 20;

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

/**
 * @brief Portable description of a type's serialized layout.
 *
 * @details The fingerprint is a CRC32 over a byte sequence that describes the
 *          type exactly as alpaca serializes it, using alpaca's own
 *          `field_type` codes, and nothing platform-dependent (no sizeof, no
 *          padding). It is the same on every target and host, and a tool
 *          without the C++ type (a Python reader) can derive it from a field
 *          spec. Grammar (one entry per type, recursive):
 *            - scalar                 code
 *            - enum                   enum_class, <underlying integer>
 *            - std::string            string
 *            - std::vector<T>         vector, <T>
 *            - std::array<T, N>       array, u32 N, <T>
 *            - std::list / deque<T>   list / deque, <T>
 *            - std::map<K, V>         map, <K>, <V>   (unordered_map likewise)
 *            - std::set<T>            set, <T>        (unordered_set likewise)
 *            - std::optional<T>       optional, <T>
 *            - std::pair<A, B>        pair, <A>, <B>
 *            - std::tuple<Ts...>      tuple, u16 count, <Ts>...
 *            - std::variant<Ts...>    variant, u16 count, <Ts>...
 *            - aggregate              struct_, u16 field count, <fields>...
 *          Multi-byte counts are little-endian.
 */
namespace layout {

using alpaca::detail::field_type;

template <typename T> struct always_false : std::false_type {};

inline void code(std::vector<uint8_t> &out, field_type t) {
  out.push_back(static_cast<uint8_t>(t));
}
inline void put_u16(std::vector<uint8_t> &out, uint16_t v) {
  out.push_back(static_cast<uint8_t>(v));
  out.push_back(static_cast<uint8_t>(v >> 8));
}
inline void put_u32(std::vector<uint8_t> &out, uint32_t v) {
  out.push_back(static_cast<uint8_t>(v));
  out.push_back(static_cast<uint8_t>(v >> 8));
  out.push_back(static_cast<uint8_t>(v >> 16));
  out.push_back(static_cast<uint8_t>(v >> 24));
}

template <typename T> void describe(std::vector<uint8_t> &out);

template <typename T, size_t N, size_t I> void describe_fields(std::vector<uint8_t> &out) {
  if constexpr (I < N) {
    T ref{};
    using Field = std::decay_t<decltype(alpaca::detail::get<I, T, N>(ref))>;
    describe<Field>(out);
    describe_fields<T, N, I + 1>(out);
  }
}

template <typename T, size_t N, size_t I> void describe_tuple(std::vector<uint8_t> &out) {
  if constexpr (I < N) {
    describe<std::tuple_element_t<I, T>>(out);
    describe_tuple<T, N, I + 1>(out);
  }
}

template <typename T, size_t N, size_t I> void describe_variant(std::vector<uint8_t> &out) {
  if constexpr (I < N) {
    describe<std::variant_alternative_t<I, T>>(out);
    describe_variant<T, N, I + 1>(out);
  }
}

template <typename T> void describe(std::vector<uint8_t> &out) {
  using U = std::remove_cv_t<T>;
  using alpaca::detail::is_specialization;
  if constexpr (std::is_same_v<U, bool>) {
    code(out, field_type::bool_);
  } else if constexpr (std::is_same_v<U, char>) {
    code(out, field_type::char_);
  } else if constexpr (std::is_same_v<U, uint8_t>) {
    code(out, field_type::uint8);
  } else if constexpr (std::is_same_v<U, uint16_t>) {
    code(out, field_type::uint16);
  } else if constexpr (std::is_same_v<U, uint32_t>) {
    code(out, field_type::uint32);
  } else if constexpr (std::is_same_v<U, uint64_t>) {
    code(out, field_type::uint64);
  } else if constexpr (std::is_same_v<U, int8_t>) {
    code(out, field_type::int8);
  } else if constexpr (std::is_same_v<U, int16_t>) {
    code(out, field_type::int16);
  } else if constexpr (std::is_same_v<U, int32_t>) {
    code(out, field_type::int32);
  } else if constexpr (std::is_same_v<U, int64_t>) {
    code(out, field_type::int64);
  } else if constexpr (std::is_same_v<U, float>) {
    code(out, field_type::float32);
  } else if constexpr (std::is_same_v<U, double>) {
    code(out, field_type::float64);
  } else if constexpr (std::is_enum_v<U>) {
    code(out, field_type::enum_class);
    describe<std::underlying_type_t<U>>(out);
  } else if constexpr (is_specialization<U, std::basic_string>::value) {
    code(out, field_type::string);
  } else if constexpr (is_specialization<U, std::vector>::value) {
    code(out, field_type::vector);
    describe<typename U::value_type>(out);
  } else if constexpr (alpaca::detail::is_array_type<U>::value) {
    code(out, field_type::array);
    put_u32(out, static_cast<uint32_t>(std::tuple_size_v<U>));
    describe<typename U::value_type>(out);
  } else if constexpr (is_specialization<U, std::list>::value) {
    code(out, field_type::list);
    describe<typename U::value_type>(out);
  } else if constexpr (is_specialization<U, std::deque>::value) {
    code(out, field_type::deque);
    describe<typename U::value_type>(out);
  } else if constexpr (is_specialization<U, std::map>::value) {
    code(out, field_type::map);
    describe<typename U::key_type>(out);
    describe<typename U::mapped_type>(out);
  } else if constexpr (is_specialization<U, std::unordered_map>::value) {
    code(out, field_type::unordered_map);
    describe<typename U::key_type>(out);
    describe<typename U::mapped_type>(out);
  } else if constexpr (is_specialization<U, std::set>::value) {
    code(out, field_type::set);
    describe<typename U::value_type>(out);
  } else if constexpr (is_specialization<U, std::unordered_set>::value) {
    code(out, field_type::unordered_set);
    describe<typename U::value_type>(out);
  } else if constexpr (is_specialization<U, std::optional>::value) {
    code(out, field_type::optional);
    describe<typename U::value_type>(out);
  } else if constexpr (is_specialization<U, std::pair>::value) {
    code(out, field_type::pair);
    describe<typename U::first_type>(out);
    describe<typename U::second_type>(out);
  } else if constexpr (is_specialization<U, std::tuple>::value) {
    code(out, field_type::tuple);
    put_u16(out, static_cast<uint16_t>(std::tuple_size_v<U>));
    describe_tuple<U, std::tuple_size_v<U>, 0>(out);
  } else if constexpr (is_specialization<U, std::variant>::value) {
    code(out, field_type::variant);
    put_u16(out, static_cast<uint16_t>(std::variant_size_v<U>));
    describe_variant<U, std::variant_size_v<U>, 0>(out);
  } else if constexpr (std::is_aggregate_v<U>) {
    constexpr size_t N = alpaca::detail::aggregate_arity<U>::size();
    code(out, field_type::struct_);
    put_u16(out, static_cast<uint16_t>(N));
    describe_fields<U, N, 0>(out);
  } else {
    static_assert(always_false<T>::value,
                  "persistent_data: this field type has no layout description (add it to "
                  "layout::describe, or use a type alpaca serializes)");
  }
}

/// @brief The layout description bytes of T (see the grammar above).
template <typename T> std::vector<uint8_t> describe() {
  std::vector<uint8_t> out;
  describe<T>(out);
  return out;
}

/// @brief CRC32 of describe<T>(); the value stored in the file header.
template <typename T> uint32_t fingerprint() {
  static const uint32_t value = [] {
    auto bytes = describe<T>();
    return crc32_fast(bytes.data(), bytes.size());
  }();
  return value;
}

} // namespace layout

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

/// @brief What the header of a well-formed file says.
struct Header {
  uint32_t version{0}; ///< Schema version of the payload
  uint32_t layout{0};  ///< Layout fingerprint of the type that wrote the payload
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
  put_u32(out.data() + 8, layout::fingerprint<T>());
  put_u32(out.data() + 12, static_cast<uint32_t>(payload_len));
  // crc over the header fields (the crc field itself is skipped) plus the payload
  uint32_t crc = crc32_fast(out.data(), 16);
  crc = crc32_fast(out.data() + HEADER_SIZE, payload_len, crc);
  put_u32(out.data() + 16, crc);
  ec.clear();
  return true;
}

/// @brief Validate the header + checksum of an encoded file.
/// @param[in] bytes The whole file
/// @param[out] header The schema version and layout fingerprint recorded in the header
/// @param[out] payload The payload bytes (a view into `bytes`)
/// @return true if the file is well-formed; ec says why not otherwise.
inline bool parse(std::span<const uint8_t> bytes, Header &header, std::span<const uint8_t> &payload,
                  std::error_code &ec) {
  if (bytes.size() < HEADER_SIZE || get_u32(bytes.data()) != MAGIC) {
    ec = PersistentDataErrc::bad_header;
    return false;
  }
  uint32_t length = get_u32(bytes.data() + 12);
  if (length != bytes.size() - HEADER_SIZE) {
    // truncated write, or trailing bytes that we never wrote
    ec = PersistentDataErrc::bad_header;
    return false;
  }
  uint32_t crc = crc32_fast(bytes.data(), 16);
  crc = crc32_fast(bytes.data() + HEADER_SIZE, length, crc);
  if (crc != get_u32(bytes.data() + 16)) {
    ec = PersistentDataErrc::bad_checksum;
    return false;
  }
  header.version = get_u32(bytes.data() + 4);
  header.layout = get_u32(bytes.data() + 8);
  payload = bytes.subspan(HEADER_SIZE);
  ec.clear();
  return true;
}

/// @brief Decode a payload as type T after checking its layout fingerprint.
template <typename T>
bool decode_payload(uint32_t layout, std::span<const uint8_t> payload, T &out,
                    std::error_code &ec) {
  if (layout != layout::fingerprint<T>()) {
    ec = PersistentDataErrc::layout_mismatch;
    return false;
  }
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

  /// @brief Decode a payload described by `header` and migrate it to Current.
  /// @return true on success; ec is unknown_version, layout_mismatch or
  ///         deserialize_failed otherwise.
  static bool decode(const Header &header, std::span<const uint8_t> payload, Current &out,
                     std::error_code &ec) {
    return decode_impl(header, payload, out, ec, std::make_index_sequence<count>{});
  }

private:
  template <size_t I>
  static bool decode_at(uint32_t layout, std::span<const uint8_t> payload, Current &out,
                        std::error_code &ec) {
    At<I> value{};
    if (!decode_payload(layout, payload, value, ec))
      return false;
    out = migrate_from<I>(std::move(value));
    return true;
  }

  template <size_t... Is>
  static bool decode_impl(const Header &header, std::span<const uint8_t> payload, Current &out,
                          std::error_code &ec, std::index_sequence<Is...>) {
    bool ok = false;
    bool matched = ((versions[Is] == header.version
                         ? (ok = decode_at<Is>(header.layout, payload, out, ec), true)
                         : false) ||
                    ...);
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
  // path.c_str() is wchar_t on Windows; string() gives the narrow form everywhere
  const std::string path_str = path.string();
  FILE *f = std::fopen(path_str.c_str(), "rb");
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
  const std::string tmp_str = tmp.string();
  FILE *f = std::fopen(tmp_str.c_str(), "wb");
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
  std::error_code fs_ec;
  if (!ok) {
    std::filesystem::remove(tmp, fs_ec);
    ec = PersistentDataErrc::io_error;
    return false;
  }
  // std::filesystem::rename replaces an existing file atomically on POSIX
  // (rename(2)) and on Windows (MoveFileEx REPLACE_EXISTING)
  std::filesystem::rename(tmp, path, fs_ec);
  if (fs_ec) {
    // some filesystems (SPIFFS) refuse to replace an existing file: remove it
    // first and retry; that short window is the only non-atomic step
    std::filesystem::remove(path, fs_ec);
    std::filesystem::rename(tmp, path, fs_ec);
    if (fs_ec) {
      std::filesystem::remove(tmp, fs_ec);
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
  std::error_code fs_ec;
  std::filesystem::remove(tmp, fs_ec);
  // remove() returns false without an error when there is nothing to remove
  std::filesystem::remove(path, fs_ec);
  if (fs_ec) {
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
  Header header;
  std::span<const uint8_t> payload;
  if (!parse(bytes, header, payload, ec))
    return false;
  if (!List::decode(header, payload, out, ec))
    return false;
  info.file_version = header.version;
  info.migrated = header.version != List::current_version;
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

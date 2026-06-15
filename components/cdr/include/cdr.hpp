#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace espp {

/// Supported CDR encapsulation identifiers.
enum class CdrEncapsulation : uint16_t {
  CDR_BE = 0x0000,    ///< Big-endian Common Data Representation.
  CDR_LE = 0x0001,    ///< Little-endian Common Data Representation.
  PL_CDR_BE = 0x0002, ///< Big-endian parameter-list CDR.
  PL_CDR_LE = 0x0003, ///< Little-endian parameter-list CDR.
};

namespace detail {
template <typename T> inline T swap_endian(T value) {
  auto bytes = std::bit_cast<std::array<std::byte, sizeof(T)>>(value);
  std::reverse(bytes.begin(), bytes.end());
  return std::bit_cast<T>(bytes);
}

template <typename T> inline T convert_endian(T value, bool target_little_endian) {
  if constexpr (sizeof(T) == 1) {
    return value;
  } else {
    if ((std::endian::native == std::endian::little) == target_little_endian) {
      return value;
    }
    return swap_endian(value);
  }
}

constexpr size_t cdr_alignment_for_size(size_t size) {
  return size >= 8 ? 8 : (size >= 4 ? 4 : (size >= 2 ? 2 : 1));
}

template <typename T> constexpr size_t cdr_alignment() { return cdr_alignment_for_size(sizeof(T)); }
} // namespace detail

/// Small helper for building CDR/XCDR1-style byte streams.
///
/// \section cdr_ex1 CDR Example
/// \snippet cdr_example.cpp cdr example
class CdrWriter {
public:
  /// @brief Configuration for a CDR writer instance.
  struct Config {
    CdrEncapsulation encapsulation{
        CdrEncapsulation::CDR_LE}; ///< Encapsulation kind to emit when writing.
    bool include_encapsulation{
        true}; ///< If true, prepend the 4-byte encapsulation header to the buffer.
  };

  /// @brief Create a configuration for writing a CDR body without an encapsulation header.
  /// @param encapsulation Endianness/encapsulation rules to use for the body payload.
  /// @return A configuration with encapsulation emission disabled.
  [[nodiscard]] static Config
  body_config(CdrEncapsulation encapsulation = CdrEncapsulation::CDR_LE) {
    return {
        .encapsulation = encapsulation,
        .include_encapsulation = false,
    };
  }

  /// @brief Create a writer configured for a headerless/body-only CDR payload.
  /// @param encapsulation Endianness/encapsulation rules to use for the body payload.
  /// @return A ready-to-use writer with no encapsulation header in its output.
  [[nodiscard]] static CdrWriter
  make_body_writer(CdrEncapsulation encapsulation = CdrEncapsulation::CDR_LE) {
    return CdrWriter(body_config(encapsulation));
  }

  /// @brief Wrap an existing payload with a CDR encapsulation header.
  /// @param payload Raw bytes to append after the generated encapsulation header.
  /// @param encapsulation Encapsulation header to prepend.
  /// @return A new byte buffer containing the encapsulation header followed by the payload.
  [[nodiscard]] static std::vector<uint8_t>
  encapsulate(std::span<const uint8_t> payload,
              CdrEncapsulation encapsulation = CdrEncapsulation::CDR_LE) {
    CdrWriter writer({
        .encapsulation = encapsulation,
        .include_encapsulation = true,
    });
    writer.write_bytes(payload);
    return writer.take_buffer();
  }

  /// @brief Construct a writer using the default little-endian encapsulated configuration.
  CdrWriter() { reset(); }

  /// @brief Construct a writer using an explicit configuration.
  /// @param config Writer configuration controlling encapsulation and endianness behavior.
  explicit CdrWriter(const Config &config)
      : config_(config) {
    reset();
  }

  /// @brief Clear the current buffer and reinitialize the encapsulation header if configured.
  void reset() {
    data_.clear();
    if (config_.include_encapsulation) {
      auto value = static_cast<uint16_t>(config_.encapsulation);
      data_.push_back(static_cast<uint8_t>((value >> 8) & 0xff));
      data_.push_back(static_cast<uint8_t>(value & 0xff));
      data_.push_back(0);
      data_.push_back(0);
    }
  }

  /// @brief Get the configured encapsulation kind for this writer.
  /// @return The encapsulation kind associated with this writer.
  [[nodiscard]] CdrEncapsulation encapsulation() const { return config_.encapsulation; }

  /// @brief Determine whether values are encoded in little-endian order.
  /// @return True for little-endian encapsulations, false for big-endian ones.
  [[nodiscard]] bool uses_little_endian() const {
    return config_.encapsulation == CdrEncapsulation::CDR_LE ||
           config_.encapsulation == CdrEncapsulation::PL_CDR_LE;
  }

  /// @brief Get the total number of bytes currently written.
  /// @return The size of the backing byte buffer, including any encapsulation header.
  [[nodiscard]] size_t size() const { return data_.size(); }

  /// @brief Access the full serialized buffer built so far.
  /// @return A const reference to the complete buffer, including any encapsulation header.
  [[nodiscard]] const std::vector<uint8_t> &buffer() const { return data_; }

  /// @brief Access only the payload portion of the buffer.
  /// @return A view over the serialized bytes after any encapsulation header.
  [[nodiscard]] std::span<const uint8_t> payload() const {
    auto bytes = std::span<const uint8_t>{data_.data(), data_.size()};
    return bytes.subspan(std::min(payload_offset(), bytes.size()));
  }

  /// @brief Move the complete serialized buffer out of the writer.
  /// @return The current buffer contents, including any encapsulation header.
  [[nodiscard]] std::vector<uint8_t> take_buffer() { return std::move(data_); }

  /// @brief Pad the buffer with zeros until it satisfies the requested alignment.
  /// @param alignment Required alignment in bytes. Values less than or equal to 1 are ignored.
  /// @return Always returns true. This matches the reader API even though writer-side alignment
  /// cannot currently fail.
  bool align(size_t alignment) {
    if (alignment <= 1) {
      return true;
    }
    while (data_.size() % alignment != 0) {
      data_.push_back(0);
    }
    return true;
  }

  /// @brief Append a primitive scalar using CDR alignment and endianness rules.
  /// @tparam T Primitive integral or floating-point type to encode.
  /// @param value Value to append to the serialized buffer.
  /// @return True after the value has been encoded and appended.
  template <typename T>
  requires(std::is_integral_v<T> || std::is_floating_point_v<T>) bool write(T value) {
    align(detail::cdr_alignment<T>());
    auto encoded = detail::convert_endian(value, uses_little_endian());
    auto bytes = std::bit_cast<std::array<std::byte, sizeof(T)>>(encoded);
    auto *raw = reinterpret_cast<const uint8_t *>(bytes.data());
    data_.insert(data_.end(), raw, raw + bytes.size());
    return true;
  }

  /// @brief Append a boolean value using the standard CDR 1-byte representation.
  /// @param value Boolean value to encode.
  /// @return True after the value has been appended.
  bool write_bool(bool value) { return write<uint8_t>(value ? 1 : 0); }

  /// @brief Append a CDR string.
  /// @param text UTF-8 text to encode. A terminating null byte is written automatically.
  /// @return True after the string length, contents, terminator, and alignment padding are written.
  bool write_string(std::string_view text) {
    align(4);
    write<uint32_t>(static_cast<uint32_t>(text.size() + 1));
    data_.insert(data_.end(), text.begin(), text.end());
    data_.push_back(0);
    align(4);
    return true;
  }

  /// @brief Append raw bytes with optional alignment.
  /// @param bytes Bytes to copy into the serialized buffer.
  /// @param alignment Alignment in bytes to satisfy before appending the data.
  /// @return True after the bytes have been appended.
  bool write_bytes(std::span<const uint8_t> bytes, size_t alignment = 1) {
    align(alignment);
    data_.insert(data_.end(), bytes.begin(), bytes.end());
    return true;
  }

  /// @brief Append a fixed-size array of primitive values.
  /// @tparam T Primitive integral or floating-point element type.
  /// @tparam N Number of elements in the array.
  /// @param values Array to encode element-by-element.
  /// @return True after all elements have been encoded.
  template <typename T, size_t N>
  requires(std::is_integral_v<T> ||
           std::is_floating_point_v<T>) bool write_array(const std::array<T, N> &values) {
    return std::all_of(values.begin(), values.end(),
                       [this](const auto &value) { return write<T>(value); });
  }

  /// @brief Append a variable-length CDR sequence of primitive values.
  /// @tparam T Primitive integral or floating-point element type.
  /// @param values Sequence elements to encode.
  /// @return True after the sequence length and all elements have been encoded.
  template <typename T>
  requires(std::is_integral_v<T> ||
           std::is_floating_point_v<T>) bool write_sequence(std::span<const T> values) {
    align(4);
    write<uint32_t>(static_cast<uint32_t>(values.size()));
    for (const auto &value : values) {
      write<T>(value);
    }
    return true;
  }

private:
  [[nodiscard]] size_t payload_offset() const { return config_.include_encapsulation ? 4 : 0; }

  Config config_;
  std::vector<uint8_t> data_{};
};

/// Small helper for parsing CDR/XCDR1-style byte streams.
class CdrReader {
public:
  /// @brief Configuration for a CDR reader instance.
  struct Config {
    bool expect_encapsulation{
        true}; ///< If true, consume and validate a 4-byte encapsulation header on reset.
    CdrEncapsulation default_encapsulation{
        CdrEncapsulation::CDR_LE}; ///< Encapsulation to assume when no header is expected.
  };

  /// @brief Create a configuration for reading a headerless/body-only CDR payload.
  /// @param encapsulation Encapsulation/endian rules to assume for the payload body.
  /// @return A configuration with encapsulation consumption disabled.
  [[nodiscard]] static Config
  body_config(CdrEncapsulation encapsulation = CdrEncapsulation::CDR_LE) {
    return {
        .expect_encapsulation = false,
        .default_encapsulation = encapsulation,
    };
  }

  /// @brief Create a reader for a headerless/body-only CDR payload.
  /// @param data Serialized payload bytes to read.
  /// @param encapsulation Encapsulation/endian rules to assume for the payload body.
  /// @return A reader initialized for body-only parsing.
  [[nodiscard]] static CdrReader
  make_body_reader(std::span<const uint8_t> data,
                   CdrEncapsulation encapsulation = CdrEncapsulation::CDR_LE) {
    return CdrReader(data, body_config(encapsulation));
  }

  /// @brief Construct a reader that expects a standard encapsulated CDR payload.
  /// @param data Serialized bytes to parse.
  explicit CdrReader(std::span<const uint8_t> data) { reset(data); }

  /// @brief Construct a reader with an explicit configuration.
  /// @param data Serialized bytes to parse.
  /// @param config Reader configuration controlling encapsulation handling.
  CdrReader(std::span<const uint8_t> data, const Config &config)
      : config_(config) {
    reset(data);
  }

  /// @brief Reset the reader to the beginning of a new serialized buffer.
  /// @param data Serialized bytes to parse.
  void reset(std::span<const uint8_t> data) {
    data_ = data;
    offset_ = 0;
    valid_ = true;
    if (config_.expect_encapsulation) {
      if (data_.size() < 4) {
        valid_ = false;
        return;
      }
      uint16_t raw = static_cast<uint16_t>(data_[0] << 8) | static_cast<uint16_t>(data_[1]);
      switch (raw) {
      case static_cast<uint16_t>(CdrEncapsulation::CDR_BE):
      case static_cast<uint16_t>(CdrEncapsulation::CDR_LE):
      case static_cast<uint16_t>(CdrEncapsulation::PL_CDR_BE):
      case static_cast<uint16_t>(CdrEncapsulation::PL_CDR_LE):
        encapsulation_ = static_cast<CdrEncapsulation>(raw);
        break;
      default:
        valid_ = false;
        return;
      }
      offset_ = 4;
    } else {
      encapsulation_ = config_.default_encapsulation;
    }
  }

  /// @brief Check whether the reader is still in a valid state.
  /// @return True if parsing can continue, false if a prior operation failed.
  [[nodiscard]] bool valid() const { return valid_; }

  /// @brief Get the active encapsulation for the current buffer.
  /// @return The parsed or assumed encapsulation value.
  [[nodiscard]] CdrEncapsulation encapsulation() const { return encapsulation_; }

  /// @brief Determine whether values are decoded as little-endian.
  /// @return True for little-endian encapsulations, false for big-endian ones.
  [[nodiscard]] bool uses_little_endian() const {
    return encapsulation_ == CdrEncapsulation::CDR_LE ||
           encapsulation_ == CdrEncapsulation::PL_CDR_LE;
  }

  /// @brief Access only the payload bytes after any encapsulation header.
  /// @return A view over the unread buffer excluding the encapsulation header.
  [[nodiscard]] std::span<const uint8_t> payload() const {
    return data_.subspan(std::min(payload_offset(), data_.size()));
  }

  /// @brief Get the number of unread bytes remaining.
  /// @return Remaining unread byte count, or 0 if the offset is beyond the buffer.
  [[nodiscard]] size_t remaining() const {
    return offset_ <= data_.size() ? data_.size() - offset_ : 0;
  }

  /// @brief Access a view of the unread bytes without copying.
  /// @return A span over the unread tail of the buffer, or an empty span if the reader is invalid.
  [[nodiscard]] std::span<const uint8_t> remaining_view() const {
    if (!valid_) {
      return {};
    }
    return data_.subspan(std::min(offset_, data_.size()));
  }

  /// @brief Advance the read cursor by a fixed number of bytes.
  /// @param length Number of bytes to skip.
  /// @return True if the bytes were skipped, false if the reader became invalid.
  bool skip(size_t length) {
    if (!valid_ || remaining() < length) {
      valid_ = false;
      return false;
    }
    offset_ += length;
    return true;
  }

  /// @brief Advance the read cursor to satisfy an alignment requirement.
  /// @param alignment Required alignment in bytes. Values less than or equal to 1 are ignored.
  /// @return True if the padding bytes were skipped successfully, false if the reader became
  /// invalid.
  bool align(size_t alignment) {
    if (alignment <= 1) {
      return true;
    }
    size_t padding = (alignment - (offset_ % alignment)) % alignment;
    return skip(padding);
  }

  /// @brief Read a primitive scalar using CDR alignment and endianness rules.
  /// @tparam T Primitive integral or floating-point type to decode.
  /// @param value Output variable that receives the decoded value on success.
  /// @return True if a complete value was decoded, false otherwise.
  template <typename T>
  requires(std::is_integral_v<T> || std::is_floating_point_v<T>) bool read(T &value) {
    constexpr size_t alignment = detail::cdr_alignment<T>();
    if constexpr (alignment > 1) {
      if (!align(alignment)) { // cppcheck-suppress knownConditionTrueFalse
        valid_ = false;
        return false;
      }
    }
    if (remaining() < sizeof(T)) {
      valid_ = false;
      return false;
    }
    std::array<std::byte, sizeof(T)> bytes{};
    std::memcpy(bytes.data(), data_.data() + offset_, sizeof(T));
    offset_ += sizeof(T);
    auto encoded = std::bit_cast<T>(bytes);
    value = detail::convert_endian(encoded, uses_little_endian());
    return true;
  }

  /// @brief Read a boolean value encoded with the CDR 1-byte representation.
  /// @param value Output variable that receives the decoded boolean on success.
  /// @return True if the boolean was read successfully, false otherwise.
  bool read_bool(bool &value) {
    uint8_t raw = 0;
    if (!read<uint8_t>(raw)) {
      return false;
    }
    value = raw != 0;
    return true;
  }

  /// @brief Read a CDR string.
  /// @param text Output string receiving the decoded text without the trailing null terminator.
  /// @return True if the string was decoded successfully, false otherwise.
  bool read_string(std::string &text) {
    if (!align(4)) {
      return false;
    }
    uint32_t length = 0;
    if (!read<uint32_t>(length) || length == 0 || remaining() < length) {
      valid_ = false;
      return false;
    }
    auto span = data_.subspan(offset_, length);
    offset_ += length;
    if (span.back() == 0) {
      span = span.first(span.size() - 1);
    }
    text.assign(reinterpret_cast<const char *>(span.data()), span.size());
    return align(4);
  }

  /// @brief Read a fixed number of bytes as a zero-copy span.
  /// @param length Number of bytes to expose.
  /// @param alignment Alignment in bytes to satisfy before reading.
  /// @return A span over the requested bytes, or an empty span if the read failed.
  std::span<const uint8_t> read_span(size_t length, size_t alignment = 1) {
    if (!align(alignment) || remaining() < length) {
      valid_ = false;
      return {};
    }
    auto span = data_.subspan(offset_, length);
    offset_ += length;
    return span;
  }

  /// @brief Read bytes into a caller-provided mutable span.
  /// @param bytes Destination span that receives the copied bytes.
  /// @param alignment Alignment in bytes to satisfy before reading.
  /// @return True if all requested bytes were read successfully, false otherwise.
  bool read_bytes(std::span<uint8_t> bytes, size_t alignment = 1) {
    auto span = read_span(bytes.size(), alignment);
    if (span.size() != bytes.size()) {
      return false;
    }
    std::memcpy(bytes.data(), span.data(), bytes.size());
    return true;
  }

  /// @brief Read bytes into a vector.
  /// @param bytes Output vector replaced with the decoded bytes on success.
  /// @param length Number of bytes to read.
  /// @param alignment Alignment in bytes to satisfy before reading.
  /// @return True if the requested bytes were read successfully, false otherwise.
  bool read_bytes(std::vector<uint8_t> &bytes, size_t length, size_t alignment = 1) {
    auto span = read_span(length, alignment);
    if (span.size() != length) {
      return false;
    }
    bytes.assign(span.begin(), span.end());
    return true;
  }

  /// @brief Read a fixed-size array of primitive values.
  /// @tparam T Primitive integral or floating-point element type.
  /// @tparam N Number of elements in the array.
  /// @param values Output array receiving the decoded elements.
  /// @return True if all array elements were read successfully, false otherwise.
  template <typename T, size_t N>
  requires(std::is_integral_v<T> ||
           std::is_floating_point_v<T>) bool read_array(std::array<T, N> &values) {
    for (auto &value : values) {
      if (!read<T>(value)) {
        return false;
      }
    }
    return true;
  }

  /// @brief Read a variable-length CDR sequence of primitive values.
  /// @tparam T Primitive integral or floating-point element type.
  /// @param values Output vector replaced with the decoded sequence elements on success.
  /// @return True if the sequence length and all elements were decoded successfully, false
  /// otherwise.
  template <typename T>
  requires(std::is_integral_v<T> ||
           std::is_floating_point_v<T>) bool read_sequence(std::vector<T> &values) {
    if (!align(4)) {
      return false;
    }
    uint32_t length = 0;
    if (!read<uint32_t>(length)) {
      return false;
    }
    values.clear();
    values.reserve(length);
    for (uint32_t i = 0; i < length; i++) {
      T value{};
      if (!read<T>(value)) {
        return false;
      }
      values.push_back(value);
    }
    return true;
  }

private:
  [[nodiscard]] size_t payload_offset() const { return config_.expect_encapsulation ? 4 : 0; }

  Config config_{};
  std::span<const uint8_t> data_{};
  size_t offset_{0};
  bool valid_{false};
  CdrEncapsulation encapsulation_{CdrEncapsulation::CDR_LE};
};

} // namespace espp

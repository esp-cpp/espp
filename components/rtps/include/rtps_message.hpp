#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "cdr.hpp"

namespace espp {

/// @brief A type usable with the typed RTPS layer (pub/sub, services, actions).
///
/// Any reflectable struct the `cdr` component can serialize and deserialize
/// qualifies - no base class, macros, or member functions required. This mirrors
/// the ROS 2 / DDS message model: a plain data struct whose fields map to CDR.
template <typename T>
concept RtpsMessage = requires(const T &value, std::span<const std::byte> bytes) {
  { cdr::serialized_size<cdr::xcdr1>(value) } -> std::convertible_to<std::size_t>;
  {cdr::deserialize<T>(bytes)};
};

/// @brief Which request/reply protocol a typed service or action endpoint uses.
enum class RtpsProtocol {
  ROS2,   ///< ROS 2-interoperable (rq/rr topics + related_sample_identity).
  NATIVE, ///< Lean espp<->espp protocol (in-band header, es_rq/es_rr topics).
};

namespace detail {
/// Serialize a reflectable message to CDR (ROS 2 / classic XCDR1) bytes. Returns
/// an empty vector on failure.
template <RtpsMessage T> std::vector<uint8_t> rtps_serialize(const T &value) {
  std::vector<uint8_t> buf(cdr::serialized_size<cdr::xcdr1>(value));
  const auto written =
      cdr::serialize_into<cdr::xcdr1>(value, std::as_writable_bytes(std::span(buf)));
  if (!written) {
    return {};
  }
  buf.resize(*written);
  return buf;
}

/// Deserialize CDR bytes to a reflectable message. cdr::deserialize returns a
/// std::expected; this collapses it to std::optional (nullopt on failure).
template <RtpsMessage T> std::optional<T> rtps_deserialize(std::span<const uint8_t> bytes) {
  auto result = cdr::deserialize<T>(std::as_bytes(bytes));
  if (!result) {
    return std::nullopt;
  }
  return std::move(*result);
}
} // namespace detail

} // namespace espp

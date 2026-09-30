#pragma once

// Wire protocol of espp::MonitorService: heap-region and task statistics over
// the espp stream_frame codec, routed by an espp::Dispatcher on module 8 by
// default (`espp.monitor` v1 through discovery).
//
// This header is deliberately host-buildable (stream_frame.hpp + the standard
// library only) so the codec is unit-tested on the host
// (components/monitor/test/monitor_host_test.cpp) and so host tools can reuse
// it. All multi-byte fields are little-endian; a `str` is [len u8][bytes].
//
// Requests (host -> device, reply flag clear):
//   0x01 GET_HEAP    (no payload)
//   0x02 GET_TASKS   (no payload)
//   0x03 SET_STREAM  [enable u8][period_ms u16][what u8: bit0 heap, bit1 tasks]
//                    enable = 0 stops the periodic HEAP / TASKS events; the
//                    device clamps the period to its Config::min_stream_period.
// Replies / events (device -> host, high bit set = frame reply flag):
//   0x81 HEAP   [count u8]{[flags u32][free u32][min_free u32]
//                          [largest_free_block u32][allocated u32][total u32]}
//   0x82 TASKS  [count u8]{[name str][cpu_percent u8][high_water_mark u32]
//                          [priority u8][core i8]}
//   0x83 OK     [request_type u8]
//   0x84 ERROR  [request_type u8][code u32][utf8 message]
// HEAP / TASKS answer the matching GET_* request and are also sent
// unsolicited while streaming is enabled (same encoding, so a host decodes
// both the same way). A TASKS payload is capped at the frame payload limit:
// tasks that would not fit are dropped from the END of the list.

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "stream_frame.hpp"

namespace espp::detail::monitor_protocol {

/// Default dispatcher module id (a routing key only; see MonitorService::Config::module).
inline constexpr uint8_t kModule = 8;
/// Stable protocol identifier + version advertised through discovery.
inline constexpr const char *kProtocol = "espp.monitor";
inline constexpr uint16_t kProtocolVersion = 1;

/// Frame `type` values within the monitor module.
enum class Type : uint8_t {
  // host -> device
  GetHeap = 0x01,
  GetTasks = 0x02,
  SetStream = 0x03,
  // device -> host (high bit set)
  Heap = 0x81,
  Tasks = 0x82,
  Ok = 0x83,
  Error = 0x84,
};

/// SET_STREAM `what` bits.
inline constexpr uint8_t kStreamHeap = 0x01;
inline constexpr uint8_t kStreamTasks = 0x02;

/// One heap region as carried in a HEAP payload.
struct HeapRegion {
  uint32_t flags{0}; ///< MALLOC_CAP_* bitmask the region was queried with.
  uint32_t free_bytes{0};
  uint32_t min_free_bytes{0};
  uint32_t largest_free_block{0};
  uint32_t allocated_bytes{0};
  uint32_t total_size{0};
};

/// One task as carried in a TASKS payload.
struct TaskEntry {
  std::string name;
  uint8_t cpu_percent{0};
  uint32_t high_water_mark{0};
  uint8_t priority{0};
  int8_t core_id{-2}; ///< 0 / 1, -1 = unpinned, -2 = unknown (core ids not compiled in).
};

/// Decoded SET_STREAM request.
struct StreamRequest {
  bool enable{false};
  uint16_t period_ms{0};
  uint8_t what{0}; ///< kStreamHeap | kStreamTasks
};

/// Bytes one TaskEntry occupies on the wire (name capped at 255).
inline size_t task_entry_size(std::string_view name) {
  return 1 + (name.size() > 255 ? 255 : name.size()) + 1 + 4 + 1 + 1;
}

/// Append a [len u8][bytes] string (truncated to 255 bytes).
inline void put_str(std::vector<uint8_t> &out, std::string_view s) {
  const size_t n = s.size() > 255 ? 255 : s.size();
  out.push_back(static_cast<uint8_t>(n));
  out.insert(out.end(), s.begin(), s.begin() + static_cast<std::ptrdiff_t>(n));
}

/// Whether a type value is a device->host reply / event.
inline constexpr bool is_reply(Type type) { return (static_cast<uint8_t>(type) & 0x80) != 0; }

/// Build an encoded frame for a monitor message (device->host types map to the
/// frame reply flag).
inline std::vector<uint8_t> build_frame(Type type, std::span<const uint8_t> payload = {},
                                        uint8_t module = kModule) {
  return espp::stream_frame::build_frame(is_reply(type), module, static_cast<uint8_t>(type),
                                         payload);
}

// ---- encoders ---------------------------------------------------------------

/// Encode a HEAP payload. At most 255 regions are encoded.
inline std::vector<uint8_t> encode_heap(std::span<const HeapRegion> regions) {
  std::vector<uint8_t> p;
  const size_t n = regions.size() > 255 ? 255 : regions.size();
  p.reserve(1 + 24 * n);
  p.push_back(static_cast<uint8_t>(n));
  for (size_t i = 0; i < n; ++i) {
    const auto &r = regions[i];
    espp::stream_frame::put_u32(p, r.flags);
    espp::stream_frame::put_u32(p, r.free_bytes);
    espp::stream_frame::put_u32(p, r.min_free_bytes);
    espp::stream_frame::put_u32(p, r.largest_free_block);
    espp::stream_frame::put_u32(p, r.allocated_bytes);
    espp::stream_frame::put_u32(p, r.total_size);
  }
  return p;
}

/// Encode a TASKS payload, keeping it within @p max_bytes (the frame payload
/// limit by default): tasks that would not fit are dropped from the end.
/// @param[out] encoded_count Set to the number of tasks encoded, if non-null.
inline std::vector<uint8_t> encode_tasks(std::span<const TaskEntry> tasks,
                                         size_t max_bytes = espp::stream_frame::kMaxPayloadSize,
                                         size_t *encoded_count = nullptr) {
  std::vector<uint8_t> p;
  p.push_back(0); // count, patched below
  size_t n = 0;
  for (const auto &t : tasks) {
    if (n == 255 || p.size() + task_entry_size(t.name) > max_bytes)
      break;
    put_str(p, t.name);
    p.push_back(t.cpu_percent);
    espp::stream_frame::put_u32(p, t.high_water_mark);
    p.push_back(t.priority);
    p.push_back(static_cast<uint8_t>(t.core_id));
    ++n;
  }
  p[0] = static_cast<uint8_t>(n);
  if (encoded_count)
    *encoded_count = n;
  return p;
}

/// Encode a SET_STREAM request payload.
inline std::vector<uint8_t> encode_set_stream(bool enable, uint16_t period_ms, uint8_t what) {
  std::vector<uint8_t> p;
  p.push_back(enable ? 1 : 0);
  espp::stream_frame::put_u16(p, period_ms);
  p.push_back(what);
  return p;
}

/// Encode an OK payload.
inline std::vector<uint8_t> encode_ok(uint8_t request_type) { return {request_type}; }

/// Encode an ERROR payload.
inline std::vector<uint8_t> encode_error(uint8_t request_type, uint32_t code,
                                         std::string_view message) {
  std::vector<uint8_t> p;
  p.push_back(request_type);
  espp::stream_frame::put_u32(p, code);
  p.insert(p.end(), message.begin(), message.end());
  return p;
}

// ---- decoders (nullopt on a malformed payload) ---------------------------------

inline std::optional<std::vector<HeapRegion>> decode_heap(std::span<const uint8_t> p) {
  if (p.empty())
    return std::nullopt;
  const size_t n = p[0];
  if (p.size() < 1 + 24 * n)
    return std::nullopt;
  std::vector<HeapRegion> out;
  out.reserve(n);
  size_t i = 1;
  for (size_t k = 0; k < n; ++k, i += 24) {
    HeapRegion r;
    r.flags = espp::stream_frame::get_u32(p.subspan(i));
    r.free_bytes = espp::stream_frame::get_u32(p.subspan(i + 4));
    r.min_free_bytes = espp::stream_frame::get_u32(p.subspan(i + 8));
    r.largest_free_block = espp::stream_frame::get_u32(p.subspan(i + 12));
    r.allocated_bytes = espp::stream_frame::get_u32(p.subspan(i + 16));
    r.total_size = espp::stream_frame::get_u32(p.subspan(i + 20));
    out.push_back(r);
  }
  return out;
}

inline std::optional<std::vector<TaskEntry>> decode_tasks(std::span<const uint8_t> p) {
  if (p.empty())
    return std::nullopt;
  const size_t n = p[0];
  std::vector<TaskEntry> out;
  out.reserve(n);
  size_t i = 1;
  for (size_t k = 0; k < n; ++k) {
    if (i >= p.size())
      return std::nullopt;
    const size_t len = p[i++];
    if (i + len + 7 > p.size())
      return std::nullopt;
    TaskEntry t;
    t.name.assign(reinterpret_cast<const char *>(p.data() + i), len);
    i += len;
    t.cpu_percent = p[i++];
    t.high_water_mark = espp::stream_frame::get_u32(p.subspan(i));
    i += 4;
    t.priority = p[i++];
    t.core_id = static_cast<int8_t>(p[i++]);
    out.push_back(std::move(t));
  }
  return out;
}

inline std::optional<StreamRequest> decode_set_stream(std::span<const uint8_t> p) {
  if (p.size() < 4)
    return std::nullopt;
  StreamRequest r;
  r.enable = p[0] != 0;
  r.period_ms = espp::stream_frame::get_u16(p.subspan(1));
  r.what = p[3];
  return r;
}

} // namespace espp::detail::monitor_protocol

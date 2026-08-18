#pragma once

// ODrive legacy native (Fibre endpoint) protocol — wire core.
//
// This header is intentionally free of any ESP-IDF / FreeRTOS dependency so
// that the wire logic (CRC, packet packing, type codecs, JSON descriptor,
// endpoint dispatch) can be built and unit-tested on a host with nothing more
// than a C++20 standard library. The `espp::OdriveNative` component composes
// this core together with `espp::BaseComponent` for logging.
//
// See PROTOCOL.md for the authoritative wire specification.

#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace espp {
namespace detail {

/// ODrive legacy CRC-16 (poly 0x3d65, init 0x1337, non-reflected, MSB-first).
/// Fold a single byte through the running remainder.
inline uint16_t odrive_crc16_byte(uint16_t rem, uint8_t val) {
  rem ^= static_cast<uint16_t>(static_cast<uint16_t>(val) << 8);
  for (int i = 0; i < 8; i++)
    rem = (rem & 0x8000) ? static_cast<uint16_t>((rem << 1) ^ 0x3d65)
                         : static_cast<uint16_t>(rem << 1);
  return rem;
}

/// CRC-16 over a buffer using the ODrive init value (0x1337).
inline uint16_t odrive_crc16(std::span<const uint8_t> data, uint16_t init = 0x1337) {
  uint16_t r = init;
  for (uint8_t b : data)
    r = odrive_crc16_byte(r, b);
  return r;
}

/// CRC-16 convenience overload for a string_view.
inline uint16_t odrive_crc16(std::string_view s, uint16_t init = 0x1337) {
  return odrive_crc16(
      std::span<const uint8_t>(reinterpret_cast<const uint8_t *>(s.data()), s.size()), init);
}

/// The legacy protocol version (canary for endpoint 0).
static constexpr uint16_t kProtocolVersion = 1;

/// Endianness helpers. Both ESP32 and the host dev machines are little-endian,
/// and the wire format is little-endian, so a raw byte copy is correct. The
/// static_assert guards against ever building on a big-endian target.
static_assert(
    []() {
// portable little-endian check evaluated at compile time via union punning
// is not constexpr-friendly; instead rely on __BYTE_ORDER__ when available.
#if defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__)
      return __BYTE_ORDER__ != __ORDER_BIG_ENDIAN__;
#else
      return true;
#endif
    }(),
    "OdriveNative wire core assumes a little-endian target");

inline uint16_t read_u16_le(std::span<const uint8_t> s, size_t off) {
  return static_cast<uint16_t>(s[off]) | (static_cast<uint16_t>(s[off + 1]) << 8);
}

inline void append_u16_le(std::vector<uint8_t> &v, uint16_t val) {
  v.push_back(static_cast<uint8_t>(val & 0xff));
  v.push_back(static_cast<uint8_t>((val >> 8) & 0xff));
}

template <typename T> inline void append_le(std::vector<uint8_t> &v, T val) {
  static_assert(std::is_trivially_copyable_v<T>, "append_le requires trivially copyable type");
  uint8_t buf[sizeof(T)];
  std::memcpy(buf, &val, sizeof(T));
  for (size_t i = 0; i < sizeof(T); ++i)
    v.push_back(buf[i]);
}

template <typename T> inline bool read_le(std::span<const uint8_t> s, T &out) {
  static_assert(std::is_trivially_copyable_v<T>, "read_le requires trivially copyable type");
  if (s.size() < sizeof(T))
    return false;
  uint8_t buf[sizeof(T)];
  for (size_t i = 0; i < sizeof(T); ++i)
    buf[i] = s[i];
  std::memcpy(&out, buf, sizeof(T));
  return true;
}

/// Escape a string for inclusion in the compact JSON descriptor. Endpoint names
/// are normally plain identifiers, but escaping keeps json_crc correct if a
/// name ever contains a quote or backslash.
inline std::string json_escape(std::string_view s) {
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    switch (c) {
    case '"':
      out += "\\\"";
      break;
    case '\\':
      out += "\\\\";
      break;
    case '\n':
      out += "\\n";
      break;
    case '\r':
      out += "\\r";
      break;
    case '\t':
      out += "\\t";
      break;
    default:
      out += c;
      break;
    }
  }
  return out;
}

/**
 * @brief Transport-agnostic server for the ODrive legacy native (Fibre
 *        endpoint) binary protocol.
 *
 * This is the host-buildable core. Register typed properties from dotted paths;
 * the core assigns sequential endpoint ids (starting at 1; endpoint 0 is the
 * JSON descriptor), builds the compact JSON descriptor and its CRC, and
 * dispatches inbound packets via process_bytes().
 */
class OdriveNativeCore {
public:
  /// Read accessor: return the current typed value.
  template <typename T> using getter_fn = std::function<T()>;
  /// Write accessor: apply a typed value, set ec on error, return true on ok.
  template <typename T> using setter_fn = std::function<bool(T, std::error_code &)>;

  OdriveNativeCore() = default;

  void register_float_property(const std::string &path, const getter_fn<float> &getter,
                               const setter_fn<float> &setter = nullptr) {
    register_typed<float>(path, "float", getter, setter);
  }
  void register_int8_property(const std::string &path, const getter_fn<int8_t> &getter,
                              const setter_fn<int8_t> &setter = nullptr) {
    register_typed<int8_t>(path, "int8", getter, setter);
  }
  void register_uint8_property(const std::string &path, const getter_fn<uint8_t> &getter,
                               const setter_fn<uint8_t> &setter = nullptr) {
    register_typed<uint8_t>(path, "uint8", getter, setter);
  }
  void register_int16_property(const std::string &path, const getter_fn<int16_t> &getter,
                               const setter_fn<int16_t> &setter = nullptr) {
    register_typed<int16_t>(path, "int16", getter, setter);
  }
  void register_uint16_property(const std::string &path, const getter_fn<uint16_t> &getter,
                                const setter_fn<uint16_t> &setter = nullptr) {
    register_typed<uint16_t>(path, "uint16", getter, setter);
  }
  void register_int32_property(const std::string &path, const getter_fn<int32_t> &getter,
                               const setter_fn<int32_t> &setter = nullptr) {
    register_typed<int32_t>(path, "int32", getter, setter);
  }
  void register_uint32_property(const std::string &path, const getter_fn<uint32_t> &getter,
                                const setter_fn<uint32_t> &setter = nullptr) {
    register_typed<uint32_t>(path, "uint32", getter, setter);
  }
  void register_int64_property(const std::string &path, const getter_fn<int64_t> &getter,
                               const setter_fn<int64_t> &setter = nullptr) {
    register_typed<int64_t>(path, "int64", getter, setter);
  }
  void register_uint64_property(const std::string &path, const getter_fn<uint64_t> &getter,
                                const setter_fn<uint64_t> &setter = nullptr) {
    register_typed<uint64_t>(path, "uint64", getter, setter);
  }

  /// Register a bool property. Wire size is 1 byte, serialized as 0/1.
  void register_bool_property(const std::string &path, const getter_fn<bool> &getter,
                              const setter_fn<bool> &setter = nullptr) {
    // An endpoint with neither accessor cannot be read or written; registering
    // it would misrepresent its access as "r" in the schema. Reject it.
    if (!getter && !setter)
      return;
    std::scoped_lock lk(mutex_);
    Endpoint ep;
    ep.id = next_id_++;
    ep.path = path;
    ep.type = "bool";
    ep.size = 1;
    ep.readable = static_cast<bool>(getter);
    ep.writable = static_cast<bool>(setter);
    if (getter) {
      ep.serialize = [getter](std::vector<uint8_t> &v) { v.push_back(getter() ? 1 : 0); };
    }
    if (setter) {
      ep.deserialize = [setter](std::span<const uint8_t> s) -> bool {
        if (s.empty())
          return false;
        std::error_code ec;
        return setter(s[0] != 0, ec);
      };
    }
    endpoints_.push_back(std::move(ep));
    finalized_ = false;
  }

  /// Build (or rebuild) the JSON descriptor and its CRC. Called lazily by
  /// process_bytes(); safe to call explicitly.
  void finalize() {
    std::scoped_lock lk(mutex_);
    finalize_locked();
  }

  /// The compact JSON descriptor bytes (endpoint 0 blob).
  std::string json() {
    std::scoped_lock lk(mutex_);
    finalize_locked();
    return json_;
  }

  /// CRC-16 over the JSON descriptor (the canary for endpoints > 0).
  uint16_t json_crc() {
    std::scoped_lock lk(mutex_);
    finalize_locked();
    return json_crc_;
  }

  /**
   * @brief Process exactly one inbound packet and return the response packet.
   * @param in One complete request packet (one USB bulk transfer).
   * @return Response packet bytes, or empty if no response is expected / the
   *         packet is ignored.
   */
  std::vector<uint8_t> process_bytes(std::span<const uint8_t> in) {
    // Minimum packet: seq(2) + endpoint(2) + output_len(2) + trailer(2).
    if (in.size() < 8)
      return {};

    const uint16_t seq_no = read_u16_le(in, 0);
    const uint16_t endpoint_field = read_u16_le(in, 2);
    const uint16_t output_len = read_u16_le(in, 4);
    const bool expect_response = (endpoint_field & 0x8000) != 0;
    const uint16_t endpoint_id = endpoint_field & 0x7fff;
    const uint16_t trailer = read_u16_le(in, in.size() - 2);
    const std::span<const uint8_t> payload = in.subspan(6, in.size() - 8);

    // Snapshot everything we need under the lock, then invoke user callbacks
    // (getter/setter) with the lock released.
    std::string json_snapshot;
    uint16_t json_crc_snapshot = 0;
    bool have_endpoint = false;
    bool writable = false;
    size_t ep_size = 0;
    std::function<void(std::vector<uint8_t> &)> serialize;
    std::function<bool(std::span<const uint8_t>)> deserialize;
    {
      std::scoped_lock lk(mutex_);
      finalize_locked();
      json_crc_snapshot = json_crc_;
      if (endpoint_id == 0) {
        json_snapshot = json_;
      } else {
        for (const auto &ep : endpoints_) {
          if (ep.id == endpoint_id) {
            have_endpoint = true;
            writable = ep.writable;
            ep_size = ep.size;
            serialize = ep.serialize;
            deserialize = ep.deserialize;
            break;
          }
        }
      }
    }

    // Canary check: PROTOCOL_VERSION for endpoint 0, else json_crc. A mismatch
    // means client and server disagree on the object model — ignore silently.
    const uint16_t expected_canary = (endpoint_id == 0) ? kProtocolVersion : json_crc_snapshot;
    if (trailer != expected_canary)
      return {};

    std::vector<uint8_t> data;

    if (endpoint_id == 0) {
      // JSON blob read: payload is a u32 LE offset.
      uint32_t offset = 0;
      read_le<uint32_t>(payload, offset); // leaves offset=0 if payload too short
      const size_t len = json_snapshot.size();
      if (offset < len) {
        const size_t chunk = std::min<size_t>(output_len, 512);
        const size_t end = std::min<size_t>(len, static_cast<size_t>(offset) + chunk);
        data.assign(json_snapshot.begin() + offset, json_snapshot.begin() + end);
      }
      // offset >= len -> empty (terminates the client's read loop)
    } else if (have_endpoint) {
      // Property endpoint: write first (if payload present and writable), then
      // read the current value into the response (if output_len > 0).
      if (!payload.empty() && writable && deserialize) {
        deserialize(payload);
      }
      if (output_len > 0 && serialize) {
        std::vector<uint8_t> value;
        serialize(value);
        const size_t n = std::min<size_t>(output_len, value.size());
        data.assign(value.begin(), value.begin() + n);
      }
      (void)ep_size;
    }
    // unknown endpoint -> data stays empty

    if (!expect_response)
      return {};

    std::vector<uint8_t> out;
    append_u16_le(out, static_cast<uint16_t>(seq_no | 0x8000));
    out.insert(out.end(), data.begin(), data.end());
    return out;
  }

private:
  struct Endpoint {
    uint16_t id{0};
    std::string path; // dotted, e.g. "axis0.controller.input_pos"
    std::string type; // JSON primitive type name
    size_t size{0};   // wire size in bytes
    bool readable{false};
    bool writable{false};
    std::function<void(std::vector<uint8_t> &)> serialize;     // append value LE
    std::function<bool(std::span<const uint8_t>)> deserialize; // read + apply
  };

  template <typename T>
  void register_typed(const std::string &path, const char *type_name, const getter_fn<T> &getter,
                      const setter_fn<T> &setter) {
    // An endpoint with neither accessor cannot be read or written; registering
    // it would misrepresent its access as "r" in the schema. Reject it.
    if (!getter && !setter)
      return;
    std::scoped_lock lk(mutex_);
    Endpoint ep;
    ep.id = next_id_++;
    ep.path = path;
    ep.type = type_name;
    ep.size = sizeof(T);
    ep.readable = static_cast<bool>(getter);
    ep.writable = static_cast<bool>(setter);
    if (getter) {
      ep.serialize = [getter](std::vector<uint8_t> &v) { append_le<T>(v, getter()); };
    }
    if (setter) {
      ep.deserialize = [setter](std::span<const uint8_t> s) -> bool {
        T val{};
        if (!read_le<T>(s, val))
          return false;
        std::error_code ec;
        return setter(val, ec);
      };
    }
    endpoints_.push_back(std::move(ep));
    finalized_ = false;
  }

  // ---- JSON descriptor generation (mutex_ held by caller) ----
  struct JsonNode {
    std::string name;
    bool is_property{false};
    // property fields:
    uint16_t id{0};
    std::string type;
    std::string access;
    // object children (ordered by first registration):
    std::vector<JsonNode> children;
  };

  static JsonNode *find_child(JsonNode &parent, std::string_view name) {
    for (auto &c : parent.children) {
      if (c.name == name)
        return &c;
    }
    return nullptr;
  }

  static void append_entry(std::string &out, const JsonNode &node) {
    out += "{\"name\":\"";
    out += json_escape(node.name);
    out += '"';
    if (node.is_property) {
      out += ",\"id\":";
      out += std::to_string(node.id);
      out += ",\"type\":\"";
      out += node.type;
      out += "\",\"access\":\"";
      out += node.access;
      out += "\"}";
    } else {
      out += ",\"type\":\"object\",\"members\":";
      append_members(out, node);
      out += '}';
    }
  }

  static void append_members(std::string &out, const JsonNode &node) {
    out += '[';
    bool first = true;
    for (const auto &c : node.children) {
      if (!first)
        out += ',';
      first = false;
      append_entry(out, c);
    }
    out += ']';
  }

  void finalize_locked() {
    if (finalized_)
      return;
    JsonNode root;
    for (const auto &ep : endpoints_) {
      // split dotted path
      std::vector<std::string_view> parts;
      size_t start = 0;
      std::string_view p(ep.path);
      while (true) {
        size_t dot = p.find('.', start);
        if (dot == std::string_view::npos) {
          parts.push_back(p.substr(start));
          break;
        }
        parts.push_back(p.substr(start, dot - start));
        start = dot + 1;
      }
      JsonNode *cur = &root;
      for (size_t i = 0; i + 1 < parts.size(); ++i) {
        JsonNode *child = find_child(*cur, parts[i]);
        if (!child) {
          JsonNode obj;
          obj.name = std::string(parts[i]);
          obj.is_property = false;
          cur->children.push_back(std::move(obj));
          child = &cur->children.back();
        }
        cur = child;
      }
      JsonNode prop;
      prop.name = std::string(parts.back());
      prop.is_property = true;
      prop.id = ep.id;
      prop.type = ep.type;
      prop.access = ep.readable ? (ep.writable ? "rw" : "r") : (ep.writable ? "w" : "r");
      cur->children.push_back(std::move(prop));
    }
    json_.clear();
    append_members(json_, root);
    // The endpoint canary (interface-definition CRC) is CRC-16 over the JSON
    // descriptor seeded with PROTOCOL_VERSION as the init value -- NOT the
    // 0x1337 packet-CRC init. This matches the fw-v0.5.1 firmware
    //   json_crc_ = calc_crc16(PROTOCOL_VERSION, embedded_json, len)   (endpoints_template.j2)
    // and the reference fibre client
    //   json_crc16 = calc_crc16(PROTOCOL_VERSION, json_bytes)          (discovery.py)
    // so that a real fibre client's endpoint-N trailer matches. Verified by the
    // serial-loopback interop harness (components/odrive_native/interop).
    json_crc_ = odrive_crc16(json_, kProtocolVersion);
    finalized_ = true;
  }

  std::mutex mutex_;
  std::vector<Endpoint> endpoints_;
  uint16_t next_id_{1}; // 0 reserved for JSON blob
  bool finalized_{false};
  std::string json_;
  uint16_t json_crc_{0};
};

} // namespace detail
} // namespace espp

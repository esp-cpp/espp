#include "meshtastic_protobuf.hpp"

#include <cstring>
#include <string_view>

namespace espp::meshtastic {

namespace {

// Minimal protobuf wire-format helpers. The protobuf encoding is a public,
// stable format: each field is a tag (field number << 3 | wire type)
// followed by a varint (type 0), a length-delimited byte string (type 2), or
// a fixed 32/64-bit value (types 5 / 1).

enum WireType : uint8_t {
  WIRE_VARINT = 0,
  WIRE_FIXED64 = 1,
  WIRE_LENGTH = 2,
  WIRE_FIXED32 = 5,
};

void put_varint(std::vector<uint8_t> &out, uint64_t value) {
  while (value >= 0x80) {
    out.push_back((uint8_t)(value & 0x7f) | 0x80);
    value >>= 7;
  }
  out.push_back((uint8_t)value);
}

void put_tag(std::vector<uint8_t> &out, uint32_t field, WireType wire) {
  put_varint(out, ((uint64_t)field << 3) | wire);
}

void put_varint_field(std::vector<uint8_t> &out, uint32_t field, uint64_t value) {
  put_tag(out, field, WIRE_VARINT);
  put_varint(out, value);
}

void put_fixed32_field(std::vector<uint8_t> &out, uint32_t field, uint32_t value) {
  put_tag(out, field, WIRE_FIXED32);
  out.push_back(value & 0xff);
  out.push_back((value >> 8) & 0xff);
  out.push_back((value >> 16) & 0xff);
  out.push_back((value >> 24) & 0xff);
}

void put_bytes_field(std::vector<uint8_t> &out, uint32_t field, std::span<const uint8_t> bytes) {
  put_tag(out, field, WIRE_LENGTH);
  put_varint(out, bytes.size());
  out.insert(out.end(), bytes.begin(), bytes.end());
}

void put_string_field(std::vector<uint8_t> &out, uint32_t field, std::string_view str) {
  put_bytes_field(out, field, std::span{reinterpret_cast<const uint8_t *>(str.data()), str.size()});
}

// int32 fields are encoded as (sign-extended) 64-bit varints
void put_int32_field(std::vector<uint8_t> &out, uint32_t field, int32_t value) {
  put_varint_field(out, field, (uint64_t)(int64_t)value);
}

struct Reader {
  const uint8_t *data;
  size_t size;
  size_t pos{0};

  bool read_varint(uint64_t &value) {
    value = 0;
    int shift = 0;
    while (pos < size && shift < 64) {
      uint8_t b = data[pos++];
      value |= (uint64_t)(b & 0x7f) << shift;
      if (!(b & 0x80)) {
        return true;
      }
      shift += 7;
    }
    return false;
  }

  bool read_fixed32(uint32_t &value) {
    if (pos + 4 > size) {
      return false;
    }
    value = (uint32_t)data[pos] | ((uint32_t)data[pos + 1] << 8) | ((uint32_t)data[pos + 2] << 16) |
            ((uint32_t)data[pos + 3] << 24);
    pos += 4;
    return true;
  }

  bool read_bytes(std::span<const uint8_t> &bytes) {
    uint64_t length = 0;
    if (!read_varint(length) || pos + length > size) {
      return false;
    }
    bytes = std::span{&data[pos], (size_t)length};
    pos += length;
    return true;
  }

  bool skip(WireType wire) {
    switch (wire) {
    case WIRE_VARINT: {
      uint64_t v;
      return read_varint(v);
    }
    case WIRE_FIXED64:
      if (pos + 8 > size) {
        return false;
      }
      pos += 8;
      return true;
    case WIRE_LENGTH: {
      std::span<const uint8_t> b;
      return read_bytes(b);
    }
    case WIRE_FIXED32: {
      uint32_t v;
      return read_fixed32(v);
    }
    }
    return false;
  }

  bool done() const { return pos >= size; }
};

} // namespace

std::vector<uint8_t> encode_data(const DataMessage &data) {
  std::vector<uint8_t> out;
  out.reserve(data.payload.size() + 16);
  if (data.portnum != PortNum::UNKNOWN_APP) {
    put_varint_field(out, 1, (uint64_t)data.portnum);
  }
  if (!data.payload.empty()) {
    put_bytes_field(out, 2, data.payload);
  }
  if (data.want_response) {
    put_varint_field(out, 3, 1);
  }
  if (data.dest) {
    put_fixed32_field(out, 4, data.dest);
  }
  if (data.source) {
    put_fixed32_field(out, 5, data.source);
  }
  if (data.request_id) {
    put_fixed32_field(out, 6, data.request_id);
  }
  if (data.reply_id) {
    put_fixed32_field(out, 7, data.reply_id);
  }
  if (data.emoji) {
    put_fixed32_field(out, 8, data.emoji);
  }
  return out;
}

std::optional<DataMessage> decode_data(std::span<const uint8_t> bytes) {
  DataMessage data;
  Reader reader{bytes.data(), bytes.size()};
  while (!reader.done()) {
    uint64_t tag;
    if (!reader.read_varint(tag)) {
      return std::nullopt;
    }
    uint32_t field = tag >> 3;
    WireType wire = (WireType)(tag & 0x07);
    uint64_t varint;
    uint32_t fixed;
    std::span<const uint8_t> span;
    switch (field) {
    case 1:
      if (!reader.read_varint(varint)) {
        return std::nullopt;
      }
      data.portnum = (PortNum)varint;
      break;
    case 2:
      if (!reader.read_bytes(span)) {
        return std::nullopt;
      }
      data.payload.assign(span.begin(), span.end());
      break;
    case 3:
      if (!reader.read_varint(varint)) {
        return std::nullopt;
      }
      data.want_response = varint != 0;
      break;
    case 4:
      if (!reader.read_fixed32(fixed)) {
        return std::nullopt;
      }
      data.dest = fixed;
      break;
    case 5:
      if (!reader.read_fixed32(fixed)) {
        return std::nullopt;
      }
      data.source = fixed;
      break;
    case 6:
      if (!reader.read_fixed32(fixed)) {
        return std::nullopt;
      }
      data.request_id = fixed;
      break;
    case 7:
      if (!reader.read_fixed32(fixed)) {
        return std::nullopt;
      }
      data.reply_id = fixed;
      break;
    case 8:
      if (!reader.read_fixed32(fixed)) {
        return std::nullopt;
      }
      data.emoji = fixed;
      break;
    default:
      if (!reader.skip(wire)) {
        return std::nullopt;
      }
      break;
    }
  }
  return data;
}

std::vector<uint8_t> encode_user(const User &user) {
  std::vector<uint8_t> out;
  put_string_field(out, 1, user.id);
  put_string_field(out, 2, user.long_name);
  put_string_field(out, 3, user.short_name);
  put_varint_field(out, 5, (uint64_t)user.hw_model);
  if (user.is_licensed) {
    put_varint_field(out, 6, 1);
  }
  if (user.role) {
    put_varint_field(out, 7, user.role);
  }
  if (!user.public_key.empty()) {
    put_bytes_field(out, 8, user.public_key);
  }
  return out;
}

std::optional<User> decode_user(std::span<const uint8_t> bytes) {
  User user;
  Reader reader{bytes.data(), bytes.size()};
  while (!reader.done()) {
    uint64_t tag;
    if (!reader.read_varint(tag)) {
      return std::nullopt;
    }
    uint32_t field = tag >> 3;
    WireType wire = (WireType)(tag & 0x07);
    uint64_t varint;
    std::span<const uint8_t> span;
    switch (field) {
    case 1:
      if (!reader.read_bytes(span)) {
        return std::nullopt;
      }
      user.id.assign(span.begin(), span.end());
      break;
    case 2:
      if (!reader.read_bytes(span)) {
        return std::nullopt;
      }
      user.long_name.assign(span.begin(), span.end());
      break;
    case 3:
      if (!reader.read_bytes(span)) {
        return std::nullopt;
      }
      user.short_name.assign(span.begin(), span.end());
      break;
    case 5:
      if (!reader.read_varint(varint)) {
        return std::nullopt;
      }
      user.hw_model = (HardwareModel)varint;
      break;
    case 6:
      if (!reader.read_varint(varint)) {
        return std::nullopt;
      }
      user.is_licensed = varint != 0;
      break;
    case 7:
      if (!reader.read_varint(varint)) {
        return std::nullopt;
      }
      user.role = (uint32_t)varint;
      break;
    case 8:
      if (!reader.read_bytes(span)) {
        return std::nullopt;
      }
      user.public_key.assign(span.begin(), span.end());
      break;
    default:
      if (!reader.skip(wire)) {
        return std::nullopt;
      }
      break;
    }
  }
  return user;
}

std::vector<uint8_t> encode_position(const Position &position) {
  std::vector<uint8_t> out;
  if (position.has_latitude) {
    put_fixed32_field(out, 1, (uint32_t)position.latitude_i);
  }
  if (position.has_longitude) {
    put_fixed32_field(out, 2, (uint32_t)position.longitude_i);
  }
  if (position.has_altitude) {
    put_int32_field(out, 3, position.altitude);
  }
  if (position.time) {
    put_fixed32_field(out, 4, position.time);
  }
  if (position.ground_speed) {
    put_varint_field(out, 15, position.ground_speed);
  }
  if (position.ground_track) {
    put_varint_field(out, 16, position.ground_track);
  }
  if (position.sats_in_view) {
    put_varint_field(out, 19, position.sats_in_view);
  }
  if (position.precision_bits) {
    put_varint_field(out, 23, position.precision_bits);
  }
  return out;
}

std::optional<Position> decode_position(std::span<const uint8_t> bytes) {
  Position position;
  Reader reader{bytes.data(), bytes.size()};
  while (!reader.done()) {
    uint64_t tag;
    if (!reader.read_varint(tag)) {
      return std::nullopt;
    }
    uint32_t field = tag >> 3;
    WireType wire = (WireType)(tag & 0x07);
    uint64_t varint;
    uint32_t fixed;
    switch (field) {
    case 1:
      if (!reader.read_fixed32(fixed)) {
        return std::nullopt;
      }
      position.latitude_i = (int32_t)fixed;
      position.has_latitude = true;
      break;
    case 2:
      if (!reader.read_fixed32(fixed)) {
        return std::nullopt;
      }
      position.longitude_i = (int32_t)fixed;
      position.has_longitude = true;
      break;
    case 3:
      if (!reader.read_varint(varint)) {
        return std::nullopt;
      }
      position.altitude = (int32_t)(int64_t)varint;
      position.has_altitude = true;
      break;
    case 4:
      if (!reader.read_fixed32(fixed)) {
        return std::nullopt;
      }
      position.time = fixed;
      break;
    case 15:
      if (!reader.read_varint(varint)) {
        return std::nullopt;
      }
      position.ground_speed = (uint32_t)varint;
      break;
    case 16:
      if (!reader.read_varint(varint)) {
        return std::nullopt;
      }
      position.ground_track = (uint32_t)varint;
      break;
    case 19:
      if (!reader.read_varint(varint)) {
        return std::nullopt;
      }
      position.sats_in_view = (uint32_t)varint;
      break;
    case 23:
      if (!reader.read_varint(varint)) {
        return std::nullopt;
      }
      position.precision_bits = (uint32_t)varint;
      break;
    default:
      if (!reader.skip(wire)) {
        return std::nullopt;
      }
      break;
    }
  }
  return position;
}

} // namespace espp::meshtastic

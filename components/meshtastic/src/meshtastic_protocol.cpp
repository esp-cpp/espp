#include "meshtastic_protocol.hpp"

#include <cmath>

namespace espp::meshtastic {

namespace {
struct RegionInfo {
  float freq_start_mhz;
  float freq_end_mhz;
  float spacing_mhz;
};

RegionInfo region_info(Region region) {
  switch (region) {
  case Region::US:
    return {902.0f, 928.0f, 0.0f};
  case Region::EU_868:
    return {869.4f, 869.65f, 0.0f};
  case Region::EU_433:
    return {433.0f, 434.0f, 0.0f};
  case Region::ANZ:
    return {915.0f, 928.0f, 0.0f};
  }
  return {902.0f, 928.0f, 0.0f};
}

struct PresetParams {
  uint32_t bandwidth_hz;
  uint8_t spreading_factor;
  uint8_t coding_rate; // denominator of 4/x
};

PresetParams preset_params(ModemPreset preset) {
  switch (preset) {
  case ModemPreset::SHORT_TURBO:
    return {500000, 7, 5};
  case ModemPreset::SHORT_FAST:
    return {250000, 7, 5};
  case ModemPreset::SHORT_SLOW:
    return {250000, 8, 5};
  case ModemPreset::MEDIUM_FAST:
    return {250000, 9, 5};
  case ModemPreset::MEDIUM_SLOW:
    return {250000, 10, 5};
  case ModemPreset::LONG_MODERATE:
    return {125000, 11, 8};
  case ModemPreset::LONG_SLOW:
    return {125000, 12, 8};
  case ModemPreset::LONG_FAST:
  default:
    return {250000, 11, 5};
  }
}
} // namespace

uint32_t djb2_hash(std::string_view str) {
  uint32_t hash = 5381;
  for (unsigned char c : str) {
    hash = ((hash << 5) + hash) + c; // hash * 33 + c
  }
  return hash;
}

uint8_t xor_hash(std::span<const uint8_t> data) {
  uint8_t hash = 0;
  for (auto b : data) {
    hash ^= b;
  }
  return hash;
}

uint8_t channel_hash(std::string_view channel_name, std::span<const uint8_t> psk) {
  uint8_t name_hash = xor_hash(
      std::span{reinterpret_cast<const uint8_t *>(channel_name.data()), channel_name.size()});
  return name_hash ^ xor_hash(psk);
}

const char *preset_display_name(ModemPreset preset) {
  switch (preset) {
  case ModemPreset::SHORT_TURBO:
    return "ShortTurbo";
  case ModemPreset::SHORT_FAST:
    return "ShortFast";
  case ModemPreset::SHORT_SLOW:
    return "ShortSlow";
  case ModemPreset::MEDIUM_FAST:
    return "MediumFast";
  case ModemPreset::MEDIUM_SLOW:
    return "MediumSlow";
  case ModemPreset::LONG_MODERATE:
    return "LongMod";
  case ModemPreset::LONG_SLOW:
    return "LongSlow";
  case ModemPreset::LONG_FAST:
  default:
    return "LongFast";
  }
}

ModemConfig compute_modem_config(Region region, ModemPreset preset, std::string_view channel_name,
                                 uint32_t frequency_slot_override) {
  auto params = preset_params(preset);
  auto info = region_info(region);
  float bw_mhz = params.bandwidth_hz / 1e6f;
  uint32_t num_channels =
      (uint32_t)std::floor((info.freq_end_mhz - info.freq_start_mhz) / (info.spacing_mhz + bw_mhz));
  if (num_channels == 0) {
    num_channels = 1;
  }
  std::string name(channel_name);
  if (name.empty()) {
    name = preset_display_name(preset);
  }
  uint32_t slot;
  if (frequency_slot_override > 0) {
    slot = (frequency_slot_override - 1) % num_channels;
  } else {
    slot = djb2_hash(name) % num_channels;
  }
  // freq = start + bw/2 + slot * bw (all in MHz); compute in Hz to avoid
  // floating point error at the kHz level
  uint64_t start_hz = (uint64_t)std::llround((double)info.freq_start_mhz * 1e6);
  uint64_t freq_hz = start_hz + params.bandwidth_hz / 2 + (uint64_t)slot * params.bandwidth_hz;

  ModemConfig config;
  config.frequency_hz = (uint32_t)freq_hz;
  config.bandwidth_hz = params.bandwidth_hz;
  config.spreading_factor = params.spreading_factor;
  config.coding_rate = params.coding_rate;
  config.preamble_length = PREAMBLE_LENGTH;
  config.sync_word = SYNC_WORD;
  config.crc_enabled = true;
  return config;
}

void pack_header(const PacketHeader &header, uint8_t *out) {
  auto put_u32 = [](uint8_t *p, uint32_t v) {
    p[0] = v & 0xff;
    p[1] = (v >> 8) & 0xff;
    p[2] = (v >> 16) & 0xff;
    p[3] = (v >> 24) & 0xff;
  };
  put_u32(&out[0], header.to);
  put_u32(&out[4], header.from);
  put_u32(&out[8], header.id);
  out[12] = (header.hop_limit & 0x07) | (header.want_ack ? 0x08 : 0x00) |
            (header.via_mqtt ? 0x10 : 0x00) | ((header.hop_start & 0x07) << 5);
  out[13] = header.channel;
  out[14] = header.next_hop;
  out[15] = header.relay_node;
}

PacketHeader unpack_header(const uint8_t *data) {
  auto get_u32 = [](const uint8_t *p) -> uint32_t {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  };
  PacketHeader header;
  header.to = get_u32(&data[0]);
  header.from = get_u32(&data[4]);
  header.id = get_u32(&data[8]);
  header.hop_limit = data[12] & 0x07;
  header.want_ack = (data[12] & 0x08) != 0;
  header.via_mqtt = (data[12] & 0x10) != 0;
  header.hop_start = (data[12] >> 5) & 0x07;
  header.channel = data[13];
  header.next_hop = data[14];
  header.relay_node = data[15];
  return header;
}

} // namespace espp::meshtastic

#include "meshtastic.hpp"

#include <algorithm>
#include <cstdio>

#include <esp_mac.h>
#include <esp_random.h>

using namespace espp;
using namespace espp::meshtastic;

MeshtasticNode::MeshtasticNode(const Config &config)
    : BaseComponent("Meshtastic", config.log_level)
    , config_(config) {
  // derive a node number from the MAC if one wasn't provided
  if (config_.node_num == 0) {
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    config_.node_num =
        ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
    if (config_.node_num == 0 || config_.node_num == BROADCAST_ADDR) {
      config_.node_num = 0x0badf00d;
    }
  }

  // compute the modem configuration for the region / preset / channel
  modem_config_ = compute_modem_config(config_.region, config_.preset, config_.channel_name);

  // expand the PSK and compute the channel hash
  key_ = expand_psk(config_.psk);
  // expand_psk() returns an empty key both when encryption is intentionally
  // disabled (empty PSK, or the single sentinel byte 0) and when the PSK length
  // is invalid. Distinguish the two so an invalid key does not silently fall
  // back to an unencrypted channel without any indication.
  const bool psk_disabled = config_.psk.empty() || (config_.psk.size() == 1 && config_.psk[0] == 0);
  if (key_.empty() && !psk_disabled) {
    logger_.error("Invalid PSK length {} (expected 1, 16, or 32 bytes); channel will be "
                  "UNENCRYPTED",
                  config_.psk.size());
  }
  resolved_channel_name_ =
      config_.channel_name.empty() ? preset_display_name(config_.preset) : config_.channel_name;
  // the channel hash uses the expanded key; for an unencrypted channel the
  // PSK contribution is empty
  channel_hash_ = channel_hash(resolved_channel_name_, key_);

  logger_.info("Meshtastic node {} ({}) on {} channel '{}' @ {:.3f} MHz (hash 0x{:02x})", node_id(),
               config_.long_name, key_.empty() ? "unencrypted" : "encrypted",
               resolved_channel_name_, modem_config_.frequency_hz / 1e6f, channel_hash_);
}

std::string MeshtasticNode::node_id() const {
  char buffer[12];
  std::snprintf(buffer, sizeof(buffer), "!%08lx", (unsigned long)config_.node_num);
  return std::string(buffer);
}

uint32_t MeshtasticNode::next_packet_id() {
  // Packet ids must be nonzero and non-repeating within a session. Seed the
  // counter once from the hardware RNG (so two nodes are unlikely to start on
  // the same id) then increment monotonically. Returning a fresh esp_random()
  // each time can repeat, which makes peers treat new packets as duplicates
  // and breaks ACK / response matching.
  if (packet_id_counter_ == 0) {
    packet_id_counter_ = esp_random() | 1u; // nonzero seed
  }
  uint32_t id = packet_id_counter_++;
  if (packet_id_counter_ == 0) {
    packet_id_counter_ = 1; // skip 0 on wrap so it stays the "unseeded" sentinel
  }
  return id;
}

bool MeshtasticNode::already_seen(uint32_t from, uint32_t id) {
  uint64_t key = ((uint64_t)from << 32) | id;
  if (std::any_of(seen_packets_.begin(), seen_packets_.end(),
                  [key](uint64_t seen) { return seen == key; })) {
    return true;
  }
  seen_packets_.push_back(key);
  if (seen_packets_.size() > MAX_SEEN_PACKETS) {
    seen_packets_.pop_front();
  }
  return false;
}

bool MeshtasticNode::send_text(std::string_view text, uint32_t destination, bool want_ack) {
  DataMessage data;
  data.portnum = PortNum::TEXT_MESSAGE_APP;
  data.payload.assign(text.begin(), text.end());
  return send_packet(data, destination, want_ack);
}

bool MeshtasticNode::send_node_info() {
  User user;
  user.id = node_id();
  user.long_name = config_.long_name;
  user.short_name = config_.short_name;
  user.hw_model = config_.hw_model;
  DataMessage data;
  data.portnum = PortNum::NODEINFO_APP;
  data.payload = encode_user(user);
  return send_packet(data, BROADCAST_ADDR, false);
}

bool MeshtasticNode::send_position(const Position &position) {
  DataMessage data;
  data.portnum = PortNum::POSITION_APP;
  data.payload = encode_position(position);
  return send_packet(data, BROADCAST_ADDR, false);
}

bool MeshtasticNode::send_data(const DataMessage &data, uint32_t destination, bool want_ack) {
  return send_packet(data, destination, want_ack);
}

bool MeshtasticNode::send_packet(const DataMessage &data, uint32_t destination, bool want_ack) {
  if (!config_.transmit) {
    logger_.error("No transmit function configured");
    return false;
  }
  std::vector<uint8_t> plaintext = encode_data(data);
  if (plaintext.size() > MAX_DATA_LENGTH) {
    logger_.error("Payload too large ({} > {} bytes)", plaintext.size(), MAX_DATA_LENGTH);
    return false;
  }

  PacketHeader header;
  header.to = destination;
  header.from = config_.node_num;
  header.want_ack = want_ack;
  header.via_mqtt = false;
  header.channel = channel_hash_;
  header.next_hop = 0;
  header.relay_node = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    header.id = next_packet_id();
    header.hop_limit = config_.hop_limit;
    header.hop_start = config_.hop_limit;
    // remember our own packet so we don't act on our own rebroadcasts
    already_seen(header.from, header.id);
  }

  // encrypt the payload in place (CTR mode; no-op copy if unencrypted)
  if (!key_.empty()) {
    if (!crypt_payload(key_, header.id, header.from, plaintext)) {
      logger_.error("Failed to encrypt payload");
      return false;
    }
  }

  std::vector<uint8_t> frame(HEADER_LENGTH + plaintext.size());
  pack_header(header, frame.data());
  std::copy(plaintext.begin(), plaintext.end(), frame.begin() + HEADER_LENGTH);

  logger_.debug("Transmitting packet id 0x{:08x} to 0x{:08x} ({} bytes)", header.id, header.to,
                frame.size());
  return config_.transmit(frame);
}

bool MeshtasticNode::handle_frame(std::span<const uint8_t> frame, float rssi, float snr) {
  if (frame.size() < HEADER_LENGTH || frame.size() > MAX_FRAME_LENGTH) {
    logger_.debug("Ignoring frame of invalid size {}", frame.size());
    return false;
  }
  PacketHeader header = unpack_header(frame.data());

  // ignore our own transmissions
  if (header.from == config_.node_num) {
    return false;
  }

  // only handle packets on our channel (the hash is a hint, not a guarantee -
  // decryption/decoding is the real check, but this cheaply rejects other
  // channels)
  if (header.channel != channel_hash_) {
    logger_.debug("Ignoring packet for channel 0x{:02x} (ours is 0x{:02x})", header.channel,
                  channel_hash_);
    return false;
  }

  bool seen;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    seen = already_seen(header.from, header.id);
  }
  if (seen) {
    logger_.debug("Ignoring already-seen packet 0x{:08x} from 0x{:08x}", header.id, header.from);
    return false;
  }

  // decrypt the payload
  std::vector<uint8_t> payload(frame.begin() + HEADER_LENGTH, frame.end());
  if (!key_.empty()) {
    if (!crypt_payload(key_, header.id, header.from, payload)) {
      logger_.warn("Failed to decrypt packet 0x{:08x}", header.id);
      return false;
    }
  }

  auto data = decode_data(payload);
  if (!data) {
    logger_.debug("Packet 0x{:08x} did not decode as a Data message (wrong channel key?)",
                  header.id);
    return false;
  }

  PacketMetadata metadata;
  metadata.header = header;
  metadata.rssi = rssi;
  metadata.snr = snr;
  metadata.hops_taken =
      header.hop_start >= header.hop_limit ? header.hop_start - header.hop_limit : 0;

  logger_.debug("Received {} packet 0x{:08x} from 0x{:08x} ({} hops, RSSI {:.0f}, SNR {:.1f})",
                (int)data->portnum, header.id, header.from, metadata.hops_taken, rssi, snr);

  // Snapshot the callbacks (and rebroadcast flag) under the lock so a
  // concurrent set_*_callback() cannot race with dispatch, then invoke the
  // copies *outside* the lock so a callback that calls back into the node
  // cannot deadlock.
  text_callback_fn on_text;
  nodeinfo_callback_fn on_nodeinfo;
  position_callback_fn on_position;
  data_callback_fn on_data;
  bool rebroadcast;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    on_text = config_.on_text;
    on_nodeinfo = config_.on_nodeinfo;
    on_position = config_.on_position;
    on_data = config_.on_data;
    rebroadcast = config_.rebroadcast;
  }

  // dispatch to the specific callbacks
  switch (data->portnum) {
  case PortNum::TEXT_MESSAGE_APP:
    if (on_text) {
      on_text(std::string(data->payload.begin(), data->payload.end()), metadata);
    }
    break;
  case PortNum::NODEINFO_APP:
    if (on_nodeinfo) {
      auto user = decode_user(data->payload);
      if (user) {
        on_nodeinfo(*user, metadata);
      }
    }
    break;
  case PortNum::POSITION_APP:
    if (on_position) {
      auto position = decode_position(data->payload);
      if (position) {
        on_position(*position, metadata);
      }
    }
    break;
  default:
    break;
  }
  if (on_data) {
    on_data(*data, metadata);
  }

  // rebroadcast (managed flood) if configured and there are hops remaining
  if (rebroadcast && header.to != config_.node_num) {
    maybe_rebroadcast(header, frame);
  }

  return true;
}

void MeshtasticNode::maybe_rebroadcast(const PacketHeader &header, std::span<const uint8_t> frame) {
  if (header.hop_limit == 0 || header.id == 0) {
    return;
  }
  if (!config_.transmit) {
    return;
  }
  // decrement the hop limit and re-pack the header, keeping the (still
  // encrypted) payload untouched
  std::vector<uint8_t> out(frame.begin(), frame.end());
  PacketHeader relayed = header;
  relayed.hop_limit = header.hop_limit - 1;
  relayed.relay_node = config_.node_num & 0xff;
  pack_header(relayed, out.data());
  logger_.debug("Rebroadcasting packet 0x{:08x} (hop limit {} -> {})", header.id, header.hop_limit,
                relayed.hop_limit);
  config_.transmit(out);
}

void MeshtasticNode::set_text_callback(const text_callback_fn &callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_.on_text = callback;
}

void MeshtasticNode::set_nodeinfo_callback(const nodeinfo_callback_fn &callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_.on_nodeinfo = callback;
}

void MeshtasticNode::set_position_callback(const position_callback_fn &callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_.on_position = callback;
}

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace espp::meshtastic {

/// The broadcast node address
static constexpr uint32_t BROADCAST_ADDR = 0xFFFFFFFF;

/// The length of the unencrypted packet header
static constexpr size_t HEADER_LENGTH = 16;

/// The maximum length of a LoRa frame (header + encrypted payload)
static constexpr size_t MAX_FRAME_LENGTH = 255;

/// The maximum length of the encoded (plaintext) Data protobuf
static constexpr size_t MAX_DATA_LENGTH = 233;

/// The LoRa sync word used by Meshtastic networks
static constexpr uint8_t SYNC_WORD = 0x2B;

/// The preamble length (in symbols) used by Meshtastic networks
static constexpr uint16_t PREAMBLE_LENGTH = 16;

/// LoRa regulatory regions (frequency plans)
enum class Region : uint8_t {
  US,     ///< United States (902.0 - 928.0 MHz)
  EU_868, ///< Europe 868 (869.4 - 869.65 MHz)
  EU_433, ///< Europe 433 (433.0 - 434.0 MHz)
  ANZ,    ///< Australia / New Zealand (915.0 - 928.0 MHz)
};

/// Modem presets (named LoRa modulation configurations)
enum class ModemPreset : uint8_t {
  LONG_FAST = 0,     ///< SF11 / 250 kHz / CR 4/5 (the network default)
  LONG_SLOW = 1,     ///< SF12 / 125 kHz / CR 4/8
  MEDIUM_SLOW = 3,   ///< SF10 / 250 kHz / CR 4/5
  MEDIUM_FAST = 4,   ///< SF9 / 250 kHz / CR 4/5
  SHORT_SLOW = 5,    ///< SF8 / 250 kHz / CR 4/5
  SHORT_FAST = 6,    ///< SF7 / 250 kHz / CR 4/5
  LONG_MODERATE = 7, ///< SF11 / 125 kHz / CR 4/8
  SHORT_TURBO = 8,   ///< SF7 / 500 kHz / CR 4/5
};

/// Application port numbers (meshtastic.PortNum)
enum class PortNum : uint32_t {
  UNKNOWN_APP = 0,
  TEXT_MESSAGE_APP = 1,
  REMOTE_HARDWARE_APP = 2,
  POSITION_APP = 3,
  NODEINFO_APP = 4,
  ROUTING_APP = 5,
  ADMIN_APP = 6,
  WAYPOINT_APP = 8,
  TELEMETRY_APP = 67,
  TRACEROUTE_APP = 70,
  NEIGHBORINFO_APP = 71,
};

/// Hardware models (meshtastic.HardwareModel, subset)
enum class HardwareModel : uint32_t {
  UNSET = 0,
  T_DECK = 50,
  M5STACK_CARDPUTER_ADV = 112,
  PRIVATE_HW = 255,
};

/// The unencrypted 16-byte packet header, present at the start of every
/// frame. All multi-byte fields are little-endian on the air.
struct PacketHeader {
  uint32_t to{BROADCAST_ADDR}; ///< Destination node number
  uint32_t from{0};            ///< Source node number
  uint32_t id{0};              ///< Packet id (unique per sender)
  uint8_t hop_limit{0};        ///< Remaining hops (0-7)
  bool want_ack{false};        ///< Whether the sender wants an ack
  bool via_mqtt{false};        ///< Whether the packet passed through MQTT
  uint8_t hop_start{0};        ///< The hop limit the packet started with
  uint8_t channel{0};          ///< Channel hash byte
  uint8_t next_hop{0};         ///< Next-hop node (low byte), 0 = any
  uint8_t relay_node{0};       ///< Relaying node (low byte)
};

/// The meshtastic.Data protobuf message (the decrypted packet payload)
struct DataMessage {
  PortNum portnum{PortNum::UNKNOWN_APP}; ///< Which app the payload is for
  std::vector<uint8_t> payload{};        ///< The app payload
  bool want_response{false};             ///< Whether the sender wants a response
  uint32_t dest{0};                      ///< Original destination (for multi-hop)
  uint32_t source{0};                    ///< Original source (for multi-hop)
  uint32_t request_id{0};                ///< The request this is a response to
  uint32_t reply_id{0};                  ///< The message this is a reply to
  uint32_t emoji{0};                     ///< Emoji tapback code point
};

/// The meshtastic.User protobuf message (node info)
struct User {
  std::string id{};                             ///< Node id string, e.g. "!a1b2c3d4"
  std::string long_name{};                      ///< Full name of the node
  std::string short_name{};                     ///< Short name (up to 4 characters)
  HardwareModel hw_model{HardwareModel::UNSET}; ///< The hardware model
  bool is_licensed{false};                      ///< Whether the operator is a licensed ham
  uint32_t role{0};                  ///< Device role (meshtastic.Config.DeviceConfig.Role)
  std::vector<uint8_t> public_key{}; ///< X25519 public key (optional)
};

/// The meshtastic.Position protobuf message (subset)
struct Position {
  int32_t latitude_i{0};      ///< Latitude * 1e7
  int32_t longitude_i{0};     ///< Longitude * 1e7
  int32_t altitude{0};        ///< Altitude above MSL, meters
  uint32_t time{0};           ///< Unix timestamp of the fix
  uint32_t ground_speed{0};   ///< Speed over ground, m/s
  uint32_t ground_track{0};   ///< Course over ground, degrees * 1e5
  uint32_t sats_in_view{0};   ///< Number of satellites used
  uint32_t precision_bits{0}; ///< Position precision (32 = full precision)
  bool has_latitude{false};   ///< Whether latitude_i is set
  bool has_longitude{false};  ///< Whether longitude_i is set
  bool has_altitude{false};   ///< Whether altitude is set

  /// Get the latitude in decimal degrees
  double latitude() const { return latitude_i / 1e7; }
  /// Get the longitude in decimal degrees
  double longitude() const { return longitude_i / 1e7; }
};

/// LoRa modem configuration needed to join a Meshtastic channel; apply this
/// to your radio driver (see espp::Sx126x::RadioConfig).
struct ModemConfig {
  uint32_t frequency_hz{0};                  ///< RF center frequency, Hz
  uint32_t bandwidth_hz{0};                  ///< Signal bandwidth, Hz
  uint8_t spreading_factor{0};               ///< Spreading factor (7-12)
  uint8_t coding_rate{0};                    ///< Coding rate denominator (5-8, i.e. 4/x)
  uint16_t preamble_length{PREAMBLE_LENGTH}; ///< Preamble length, symbols
  uint8_t sync_word{SYNC_WORD};              ///< LoRa sync word
  bool crc_enabled{true};                    ///< Payload CRC enabled
};

/// Metadata for a received packet, passed alongside decoded payloads
struct PacketMetadata {
  PacketHeader header{}; ///< The packet header
  float rssi{0};         ///< Receive RSSI, dBm
  float snr{0};          ///< Receive SNR, dB
  uint8_t hops_taken{0}; ///< hop_start - hop_limit (0 = heard directly)
};

} // namespace espp::meshtastic

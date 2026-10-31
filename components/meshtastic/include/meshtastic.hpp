#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "base_component.hpp"
#include "meshtastic_crypto.hpp"
#include "meshtastic_protobuf.hpp"
#include "meshtastic_protocol.hpp"
#include "meshtastic_types.hpp"

namespace espp {
/// A minimal, radio-agnostic Meshtastic-compatible mesh node.
///
/// This class implements enough of the Meshtastic over-the-air protocol to
/// interoperate with stock Meshtastic devices on a shared channel (the public
/// "LongFast" channel by default): it frames, encrypts (AES-CTR with the
/// well-known default key), sends and receives text messages, node info and
/// positions, and performs receive-side deduplication and (optionally)
/// managed-flood rebroadcasting.
///
/// It is decoupled from any particular radio: you provide a transmit function
/// and feed it received frames (e.g. from an espp::Sx126x driver). Call
/// modem_config() to get the exact LoRa modulation parameters to apply to
/// your radio for the configured region / preset / channel.
///
/// \note This is an independent, clean-room implementation of the published
///       Meshtastic protocol; it is not affiliated with or endorsed by
///       Meshtastic LLC. "Meshtastic" is a registered trademark of Meshtastic
///       LLC. See the component README for details.
///
/// \note Scope: this implements the public default channel with PSK
///       encryption. It does not implement PKI-encrypted direct messages,
///       MQTT, store-and-forward, or the admin protocol.
///
/// \section meshtastic_example Example
/// \snippet meshtastic_example.cpp meshtastic example
class MeshtasticNode : public BaseComponent {
public:
  /// Function used to transmit a framed packet over the radio. Return true on
  /// success.
  typedef std::function<bool(std::span<const uint8_t> frame)> transmit_fn;

  /// Callback invoked when a text message is received.
  /// \param text The message text
  /// \param metadata The packet metadata (sender, RSSI, SNR, ...)
  typedef std::function<void(const std::string &text, const meshtastic::PacketMetadata &metadata)>
      text_callback_fn;

  /// Callback invoked when node info (a User message) is received.
  typedef std::function<void(const meshtastic::User &user,
                             const meshtastic::PacketMetadata &metadata)>
      nodeinfo_callback_fn;

  /// Callback invoked when a position is received.
  typedef std::function<void(const meshtastic::Position &position,
                             const meshtastic::PacketMetadata &metadata)>
      position_callback_fn;

  /// Callback invoked for any received Data message (after decryption and
  /// decode), for handling port numbers not covered by the specific
  /// callbacks above.
  typedef std::function<void(const meshtastic::DataMessage &data,
                             const meshtastic::PacketMetadata &metadata)>
      data_callback_fn;

  /// Configuration for the Meshtastic node.
  struct Config {
    uint32_t node_num{0};               ///< This node's number. If 0, a stable pseudo-random
                                        ///< number is derived from the ESP32's MAC address.
    std::string long_name{"espp node"}; ///< This node's long name
    std::string short_name{"espp"};     ///< This node's short name (<= 4 chars)
    meshtastic::HardwareModel hw_model{
        meshtastic::HardwareModel::PRIVATE_HW}; ///< This node's hardware model

    meshtastic::Region region{meshtastic::Region::US};                  ///< Regulatory region
    meshtastic::ModemPreset preset{meshtastic::ModemPreset::LONG_FAST}; ///< Modem preset
    std::string channel_name{};  ///< Channel name; empty uses the default
                                 ///< channel (the preset's display name)
    std::vector<uint8_t> psk{1}; ///< The channel PSK. Default {1} = the public
                                 ///< default key. See meshtastic::expand_psk.

    transmit_fn transmit{nullptr}; ///< Function used to transmit frames

    uint8_t hop_limit{3};    ///< Hop limit for originated broadcasts (0-7)
    bool rebroadcast{false}; ///< Whether to rebroadcast (relay) others' packets
                             ///< (managed flood). False = receive/originate only,
                             ///< which still fully interoperates.

    text_callback_fn on_text{nullptr};         ///< Called on received text messages
    nodeinfo_callback_fn on_nodeinfo{nullptr}; ///< Called on received node info
    position_callback_fn on_position{nullptr}; ///< Called on received positions
    data_callback_fn on_data{nullptr};         ///< Called on any received Data message

    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log verbosity
  };

  /// Constructor
  /// \param config The configuration for the node
  explicit MeshtasticNode(const Config &config);

  /// Get the LoRa modem configuration to apply to the radio for the
  /// configured region / preset / channel (frequency, bandwidth, spreading
  /// factor, coding rate, sync word).
  /// \return The modem configuration
  const meshtastic::ModemConfig &modem_config() const { return modem_config_; }

  /// Get this node's number.
  /// \return The node number
  uint32_t node_num() const { return config_.node_num; }

  /// Get this node's id string (e.g. "!a1b2c3d4").
  /// \return The node id string
  std::string node_id() const;

  /// Send a text message on the channel.
  /// \param text The message text (UTF-8, up to ~200 bytes)
  /// \param destination The destination node, or BROADCAST_ADDR (default) to
  ///        broadcast to the channel
  /// \param want_ack Whether to request an acknowledgement
  /// \return True if the message was transmitted
  bool send_text(std::string_view text, uint32_t destination = meshtastic::BROADCAST_ADDR,
                 bool want_ack = false);

  /// Broadcast this node's info (User message). Stock devices need this to
  /// show your node's name in their node lists.
  /// \return True if the node info was transmitted
  bool send_node_info();

  /// Broadcast a position.
  /// \param position The position to broadcast
  /// \return True if the position was transmitted
  bool send_position(const meshtastic::Position &position);

  /// Send an arbitrary Data message on the channel (advanced use).
  /// \param data The Data message to send
  /// \param destination The destination node, or BROADCAST_ADDR to broadcast
  /// \param want_ack Whether to request an acknowledgement
  /// \return True if the message was transmitted
  bool send_data(const meshtastic::DataMessage &data,
                 uint32_t destination = meshtastic::BROADCAST_ADDR, bool want_ack = false);

  /// Handle a raw frame received from the radio. Decrypts, decodes, performs
  /// deduplication, dispatches callbacks, and (if configured) rebroadcasts.
  /// \param frame The raw received frame (header + encrypted payload)
  /// \param rssi The receive RSSI in dBm
  /// \param snr The receive SNR in dB
  /// \return True if the frame was a valid, newly-seen packet for our channel
  bool handle_frame(std::span<const uint8_t> frame, float rssi = 0, float snr = 0);

  /// Set the callback invoked when a text message is received.
  /// \param callback The callback
  void set_text_callback(const text_callback_fn &callback);

  /// Set the callback invoked when node info is received.
  /// \param callback The callback
  void set_nodeinfo_callback(const nodeinfo_callback_fn &callback);

  /// Set the callback invoked when a position is received.
  /// \param callback The callback
  void set_position_callback(const position_callback_fn &callback);

protected:
  uint32_t next_packet_id();
  bool already_seen(uint32_t from, uint32_t id);
  bool send_packet(const meshtastic::DataMessage &data, uint32_t destination, bool want_ack);
  void maybe_rebroadcast(const meshtastic::PacketHeader &header, std::span<const uint8_t> frame);

  Config config_;
  meshtastic::ModemConfig modem_config_{};
  std::vector<uint8_t> key_{};        // expanded AES key (empty = no encryption)
  uint8_t channel_hash_{0};           // our channel's hash byte
  std::string resolved_channel_name_; // channel name used for hashing

  std::mutex mutex_;
  uint32_t packet_id_counter_{0};
  // recently-seen (from, id) pairs for deduplication
  std::deque<uint64_t> seen_packets_;
  static constexpr size_t MAX_SEEN_PACKETS = 128;
};
} // namespace espp

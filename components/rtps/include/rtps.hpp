#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "base_component.hpp"
#include "task.hpp"
#include "udp_socket.hpp"

namespace espp {
/// Cross-platform RTPS protocol foundation built on top of the socket component.
class RtpsParticipant : public BaseComponent {
public:
  /// @brief Delivery semantics advertised for a writer or reader endpoint.
  enum class ReliabilityKind : uint8_t {
    BEST_EFFORT = 0, ///< Best-effort delivery semantics.
    RELIABLE = 1,    ///< Reliable delivery semantics.
  };

  /// @brief RTPS protocol version carried in the RTPS message header.
  struct ProtocolVersion {
    uint8_t major{2}; ///< Major RTPS version number.
    uint8_t minor{3}; ///< Minor RTPS version number.
  };

  /// @brief RTPS vendor identifier carried in the RTPS message header.
  struct VendorId {
    std::array<uint8_t, 2> value{0xca, 0xfe}; ///< Two-byte vendor identifier.
  };

  /// @brief 12-byte prefix that identifies an RTPS participant.
  struct GuidPrefix {
    std::array<uint8_t, 12> value{}; ///< Raw 12-byte GUID prefix value.

    /// @brief Compare two GUID prefixes for equality.
    /// @param other Prefix to compare against.
    /// @return True if both prefixes contain identical bytes.
    bool operator==(const GuidPrefix &other) const = default;

    /// @brief Convert the GUID prefix to a printable hex string.
    /// @return Colon-separated hexadecimal representation of the prefix.
    std::string to_string() const;
  };

  /// @brief 4-byte entity identifier within a participant.
  struct EntityId {
    std::array<uint8_t, 4> value{}; ///< Raw 4-byte entity identifier value.

    /// @brief Compare two entity identifiers for equality.
    /// @param other Entity identifier to compare against.
    /// @return True if both entity identifiers contain identical bytes.
    bool operator==(const EntityId &other) const = default;

    /// @brief Convert the entity identifier to a printable hex string.
    /// @return Colon-separated hexadecimal representation of the entity identifier.
    std::string to_string() const;
  };

  /// @brief Globally unique identifier for an RTPS entity.
  struct Guid {
    GuidPrefix prefix{};  ///< Participant GUID prefix portion.
    EntityId entity_id{}; ///< Entity identifier portion within the participant.

    /// @brief Compare two GUIDs for equality.
    /// @param other GUID to compare against.
    /// @return True if both GUIDs have the same prefix and entity identifier.
    bool operator==(const Guid &other) const = default;

    /// @brief Convert the GUID to a printable string.
    /// @return Combined printable representation of the prefix and entity identifier.
    std::string to_string() const;
  };

  /// @brief RTPS sequence number wrapper.
  struct SequenceNumber {
    int64_t value{1}; ///< Signed 64-bit RTPS sequence number value.
  };

  /// @brief RTPS network locator for a unicast or multicast transport endpoint.
  struct Locator {
    /// @brief Supported locator transport kinds.
    enum class Kind : int32_t {
      INVALID = -1, ///< Locator is not initialized or not valid.
      UDP_V4 = 1,   ///< UDP over IPv4 locator.
    };

    Kind kind{Kind::INVALID};          ///< Transport kind of this locator.
    uint32_t port{0};                  ///< Transport port number in host byte order.
    std::array<uint8_t, 16> address{}; ///< Raw 16-byte RTPS locator address field.

    /// @brief Build a UDPv4 locator from a dotted IPv4 address and port.
    /// @param ipv4_address Dotted-decimal IPv4 address string.
    /// @param port UDP port number to advertise.
    /// @return A locator configured for UDPv4 with the IPv4 address stored in RTPS locator form.
    static Locator udp_v4(std::string_view ipv4_address, uint16_t port);

    /// @brief Convert the locator address to a printable IPv4 string.
    /// @return Dotted-decimal IPv4 address, or `0.0.0.0` if the locator is not UDPv4.
    std::string address_string() const;
  };

  /// @brief RTPS message header fields.
  struct Header {
    ProtocolVersion protocol_version{}; ///< RTPS protocol version.
    VendorId vendor_id{};               ///< Sender vendor identifier.
    GuidPrefix guid_prefix{};           ///< Sender participant GUID prefix.
  };

  /// @brief Supported RTPS submessage kinds used by the current implementation.
  enum class SubmessageKind : uint8_t {
    PAD = 0x01,       ///< Padding submessage.
    ACKNACK = 0x06,   ///< Reliable-reader acknowledgement submessage.
    HEARTBEAT = 0x07, ///< Reliable-writer heartbeat submessage.
    INFO_TS = 0x09,   ///< Timestamp information submessage.
    INFO_DST = 0x0e,  ///< Destination GUID-prefix information submessage.
    DATA = 0x15,      ///< User or discovery data submessage.
  };

  /// @brief One RTPS submessage within an RTPS message.
  struct Submessage {
    SubmessageKind kind{SubmessageKind::PAD}; ///< Submessage kind discriminator.
    uint8_t flags{0x01};                      ///< Raw RTPS submessage flags byte.
    std::vector<uint8_t>
        payload{}; ///< Serialized submessage payload bytes without the 4-byte submessage header.
  };

  /// @brief RTPS message consisting of a header and a sequence of submessages.
  struct Message {
    Header header{};                       ///< RTPS message header.
    std::vector<Submessage> submessages{}; ///< Serialized submessages carried by this RTPS message.

    /// @brief Serialize the RTPS message to bytes.
    /// @return A complete RTPS message buffer ready to send on the network.
    std::vector<uint8_t> serialize() const;

    /// @brief Parse an RTPS message from bytes.
    /// @param data Serialized RTPS message bytes.
    /// @return A parsed message on success, or `std::nullopt` if the input is invalid.
    static std::optional<Message> parse(std::span<const uint8_t> data);
  };

  /// @brief Standard RTPS UDP port mapping derived from domain and participant IDs.
  struct PortMapping {
    uint16_t metatraffic_multicast{0}; ///< Multicast discovery/metatraffic port.
    uint16_t metatraffic_unicast{0};   ///< Unicast discovery/metatraffic port for this participant.
    uint16_t user_multicast{0};        ///< Multicast user-data port.
    uint16_t user_unicast{0};          ///< Unicast user-data port for this participant.
  };

  /// @brief Configuration for a locally advertised writer endpoint.
  struct WriterConfig {
    std::string topic_name{};                     ///< Topic name advertised through SEDP.
    std::string type_name{"std_msgs/msg/UInt32"}; ///< Type name advertised through SEDP.
    ReliabilityKind reliability{
        ReliabilityKind::BEST_EFFORT}; ///< Reliability QoS advertised for the writer.
    uint32_t entity_index{0};          ///< Local entity slot used to derive the RTPS entity ID.
  };

  /// @brief Configuration for a locally advertised reader endpoint.
  struct ReaderConfig {
    std::string topic_name{};                     ///< Topic name advertised through SEDP.
    std::string type_name{"std_msgs/msg/UInt32"}; ///< Type name advertised through SEDP.
    ReliabilityKind reliability{
        ReliabilityKind::BEST_EFFORT}; ///< Reliability QoS advertised for the reader.
    uint32_t entity_index{0};          ///< Local entity slot used to derive the RTPS entity ID.
    std::function<void(uint32_t)> on_uint32_sample{
        nullptr}; ///< Callback invoked when a matching temporary UInt32 sample is received.
  };

  /// @brief Cached information about a discovered remote participant.
  struct ParticipantProxy {
    Guid participant_guid{};       ///< Discovered participant GUID.
    GuidPrefix guid_prefix{};      ///< Discovered participant GUID prefix.
    std::string name{};            ///< Remote participant name, if advertised.
    std::string enclave{"/"};      ///< Remote ROS 2 enclave/user-data hint, if advertised.
    std::string address{};         ///< Preferred remote IPv4 address for user traffic.
    PortMapping ports{};           ///< Remote participant port mapping derived from discovery data.
    uint32_t builtin_endpoints{0}; ///< Remote builtin-endpoint bitmask from SPDP.
  };

  /// @brief Cached information about a discovered remote reader or writer endpoint.
  struct EndpointProxy {
    Guid guid{};              ///< Discovered endpoint GUID.
    Guid participant_guid{};  ///< GUID of the participant that owns this endpoint.
    std::string topic_name{}; ///< Discovered topic name.
    std::string type_name{"std_msgs/msg/UInt32"};              ///< Discovered type name.
    ReliabilityKind reliability{ReliabilityKind::BEST_EFFORT}; ///< Advertised endpoint reliability.
    bool is_reader{false};          ///< True for discovered readers, false for discovered writers.
    bool expects_inline_qos{false}; ///< Whether the remote endpoint requested inline QoS.
    Locator unicast_locator{};      ///< Preferred unicast locator advertised by the endpoint.
  };

  /// @brief Top-level participant configuration.
  struct Config {
    std::string node_name{"espp_rtps"}; ///< Local participant name advertised in discovery.
    uint16_t domain_id{0};      ///< RTPS domain ID used for port derivation and discovery scope.
    uint16_t participant_id{0}; ///< RTPS participant ID used for GUID and port derivation.
    std::string bind_address{"0.0.0.0"}; ///< Local IPv4 address to bind sockets to.
    std::string advertised_address{
        "127.0.0.1"}; ///< IPv4 address advertised to peers for unicast traffic.
    std::string metatraffic_multicast_group{
        "239.255.0.1"}; ///< Multicast group used for RTPS metatraffic discovery.
    Task::BaseConfig receive_task_config{
        .name = "RtpsRx",
        .stack_size_bytes = 6 * 1024}; ///< Base task configuration for receive sockets.
    Task::BaseConfig announce_task_config{
        .name = "RtpsAnnounce",
        .stack_size_bytes = 6 * 1024}; ///< Task configuration for periodic discovery announcements.
    std::chrono::milliseconds announce_period{1000}; ///< Interval between periodic SPDP/SEDP sends.
    std::string enclave{"/"}; ///< User-data enclave string advertised in SPDP.
    std::function<void(const ParticipantProxy &participant)> on_participant_discovered{
        nullptr}; ///< Callback invoked when a remote participant is first discovered.
    std::function<void(const EndpointProxy &endpoint)> on_endpoint_discovered{
        nullptr}; ///< Callback invoked when a remote endpoint is first discovered.
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::INFO}; ///< Participant log verbosity.
  };

  /// @brief Construct an RTPS participant.
  /// @param config Participant configuration controlling ports, addresses, discovery, and
  /// callbacks.
  explicit RtpsParticipant(const Config &config);

  /// @brief Destroy the participant and stop any active sockets/tasks.
  ~RtpsParticipant();

  /// @brief Start discovery sockets, user-data sockets, and periodic announcements.
  /// @return True if startup succeeded, false if the participant was already started or
  /// initialization failed.
  bool start();

  /// @brief Stop sockets and background tasks associated with the participant.
  void stop();

  /// @brief Check whether the participant is currently started.
  /// @return True if the participant has been started and not yet stopped.
  bool is_started() const;

  /// @brief Register a local writer endpoint to advertise through SEDP.
  /// @param writer_config Writer configuration to add.
  /// @return True after the writer has been stored.
  bool add_writer(const WriterConfig &writer_config);

  /// @brief Register a local reader endpoint to advertise through SEDP.
  /// @param reader_config Reader configuration to add.
  /// @return True after the reader has been stored.
  bool add_reader(const ReaderConfig &reader_config);

  /// @brief Get the currently discovered remote participants.
  /// @return A snapshot copy of the discovered participant list.
  std::vector<ParticipantProxy> discovered_participants() const;

  /// @brief Get the currently discovered remote writer endpoints.
  /// @return A snapshot copy of the discovered writer list.
  std::vector<EndpointProxy> discovered_writers() const;

  /// @brief Get the currently discovered remote reader endpoints.
  /// @return A snapshot copy of the discovered reader list.
  std::vector<EndpointProxy> discovered_readers() const;

  /// @brief Access the registered local writer configurations.
  /// @return A const reference to the local writer list.
  const std::vector<WriterConfig> &writers() const;

  /// @brief Access the registered local reader configurations.
  /// @return A const reference to the local reader list.
  const std::vector<ReaderConfig> &readers() const;

  /// @brief Compute the standard RTPS UDP port mapping for this participant.
  /// @return The derived metatraffic and user-data ports for the configured domain and participant
  /// IDs.
  PortMapping ports() const;

  /// @brief Get the local participant GUID.
  /// @return GUID built from the local GUID prefix and participant entity ID.
  Guid participant_guid() const;

  /// @brief Get the GUID for a local writer entity slot.
  /// @param index Zero-based local writer entity index.
  /// @return GUID for the derived local writer entity.
  Guid writer_guid(size_t index) const;

  /// @brief Get the GUID for a local reader entity slot.
  /// @param index Zero-based local reader entity index.
  /// @return GUID for the derived local reader entity.
  Guid reader_guid(size_t index) const;

  /// @brief Build the default participant announce message.
  /// @return Serialized SPDP participant announce message.
  std::vector<uint8_t> build_announce_message() const;

  /// @brief Build the SPDP participant announcement message for this participant.
  /// @return Serialized SPDP message describing the local participant.
  std::vector<uint8_t> build_spdp_announce_message() const;

  /// @brief Build an SEDP publication announcement for a local writer.
  /// @param writer_config Writer configuration to serialize.
  /// @return Serialized SEDP publication message for the writer.
  std::vector<uint8_t> build_sedp_publication_message(const WriterConfig &writer_config) const;

  /// @brief Build an SEDP subscription announcement for a local reader.
  /// @param reader_config Reader configuration to serialize.
  /// @return Serialized SEDP subscription message for the reader.
  std::vector<uint8_t> build_sedp_subscription_message(const ReaderConfig &reader_config) const;

  /// @brief Build a temporary ESPP UInt32 user-data message.
  /// @param topic_name Topic name to embed in the message payload.
  /// @param value UInt32 sample value to serialize.
  /// @param reliability Reliability flag to encode in the temporary payload header.
  /// @return Serialized RTPS DATA message containing the temporary ESPP UInt32 payload.
  std::vector<uint8_t> build_uint32_data_message(std::string_view topic_name, uint32_t value,
                                                 ReliabilityKind reliability) const;

  /// @brief Publish a temporary ESPP UInt32 sample to discovered participants.
  /// @param topic_name Topic name to publish on. Must match a registered local writer.
  /// @param value UInt32 sample value to send.
  /// @return True if at least one send call succeeded, false otherwise.
  bool publish_uint32(std::string_view topic_name, uint32_t value);

  /// @brief Serialize a UInt32 value into a standalone CDR payload.
  /// @param value Value to serialize.
  /// @return Encapsulated little-endian CDR payload containing the value.
  static std::vector<uint8_t> serialize_uint32_cdr(uint32_t value);

  /// @brief Parse a standalone CDR payload containing a UInt32 value.
  /// @param data Encapsulated CDR payload bytes.
  /// @return Parsed UInt32 value on success, or `std::nullopt` if the payload is invalid.
  static std::optional<uint32_t> deserialize_uint32_cdr(std::span<const uint8_t> data);

  /// @brief Compute the standard RTPS UDP port mapping for a domain/participant pair.
  /// @param domain_id RTPS domain ID.
  /// @param participant_id RTPS participant ID.
  /// @return Derived RTPS metatraffic and user-data ports.
  static PortMapping compute_port_mapping(uint16_t domain_id, uint16_t participant_id);

private:
  bool handle_metatraffic_message(std::vector<uint8_t> &data, const Socket::Info &sender);
  bool handle_user_message(std::vector<uint8_t> &data, const Socket::Info &sender);
  bool send_spdp_announce_now();
  bool send_sedp_announcements_to(const ParticipantProxy &participant);
  bool send_discovery_now();
  ParticipantProxy make_local_participant_proxy() const;

  Config config_;
  GuidPrefix guid_prefix_{};
  std::atomic_bool started_{false};

  std::unique_ptr<UdpSocket> metatraffic_multicast_receiver_;
  std::unique_ptr<UdpSocket> metatraffic_unicast_receiver_;
  std::unique_ptr<UdpSocket> user_unicast_receiver_;
  std::unique_ptr<Task> announce_task_;

  mutable std::mutex mutex_;
  std::vector<WriterConfig> writers_;
  std::vector<ReaderConfig> readers_;
  std::vector<ParticipantProxy> discovered_participants_;
  std::vector<EndpointProxy> discovered_writers_;
  std::vector<EndpointProxy> discovered_readers_;
};
} // namespace espp

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "base_component.hpp"
#include "task.hpp"
#include "udp_socket.hpp"

namespace espp {
/// Cross-platform RTPS protocol foundation built on top of the socket component.
///
/// \section rtps_ex1 RTPS Example
/// \snippet rtps_example.cpp rtps example
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
    GAP = 0x08,       ///< Writer notification that sequence numbers are irrelevant/unavailable.
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
    std::string multicast_group{}; ///< Optional multicast group advertised for this writer and used
                                   ///< by `publish()` when set.
    uint32_t entity_index{0};      ///< Local entity slot used to derive the RTPS entity ID.
    uint32_t history_depth{16};    ///< KEEP_LAST history depth advertised through SEDP and used to
                                   ///< bound the reliable-writer history cache (number of recent
                                   ///< samples retained for retransmission). Only used when
                                   ///< reliability is RELIABLE.
  };

  /// @brief Configuration for a locally advertised reader endpoint.
  struct ReaderConfig {
    std::string topic_name{};                     ///< Topic name advertised through SEDP.
    std::string type_name{"std_msgs/msg/UInt32"}; ///< Type name advertised through SEDP.
    ReliabilityKind reliability{
        ReliabilityKind::BEST_EFFORT}; ///< Reliability QoS advertised for the reader.
    std::string multicast_group{};     ///< Optional multicast group advertised for this reader and
                                   ///< joined on the standard RTPS user-multicast port when set.
    uint32_t entity_index{0}; ///< Local entity slot used to derive the RTPS entity ID.
    std::function<void(std::span<const uint8_t>)> on_sample{
        nullptr}; ///< Callback invoked with the raw CDR-encapsulated serialized payload
                  ///< (encapsulation header + body) of a matching received sample. The span is only
                  ///< valid for the duration of the callback.
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
    Locator metatraffic_unicast_locator{};   ///< Full metatraffic unicast locator (address + port).
    Locator metatraffic_multicast_locator{}; ///< Full metatraffic multicast locator.
    Locator default_unicast_locator{};       ///< Full default (user-data) unicast locator.
    Locator default_multicast_locator{};     ///< Full default (user-data) multicast locator.
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
    std::vector<Locator> multicast_locators{}; ///< Multicast locators advertised by the endpoint
                                               ///< for user-data traffic.
  };

  /// @brief Top-level participant configuration.
  struct Config {
    std::string node_name{"espp_rtps"}; ///< Local participant name advertised in discovery.
    uint16_t domain_id{0};      ///< RTPS domain ID used for port derivation and discovery scope.
    uint16_t participant_id{0}; ///< RTPS participant ID used for GUID and port derivation.
    bool randomize_guid_prefix{true}; ///< When true (default), mix per-instance entropy into the
                                      ///< participant GUID so a restarted participant is seen as a
                                      ///< new participant — DDS/ROS 2 peers then accept its
                                      ///< republished samples instead of dropping them as
                                      ///< already-seen duplicates. Set false for a deterministic
                                      ///< GUID derived only from node_name/domain/participant id
                                      ///< (e.g. reproducible tests).
    std::string bind_address{"0.0.0.0"}; ///< Local IPv4 address to bind sockets to.
    std::string advertised_address{
        "127.0.0.1"}; ///< IPv4 address advertised to peers for unicast traffic.
    std::string metatraffic_multicast_group{
        "239.255.0.1"}; ///< Multicast group used for RTPS metatraffic discovery.
    std::string user_multicast_group{
        "239.255.0.1"}; ///< Multicast group used for best-effort user-data multicast when enabled.
    bool use_multicast_for_user_data{false}; ///< If true, join the user multicast group and publish
                                             ///< temporary user-data samples via multicast.
    Task::BaseConfig receive_task_config{
        .name = "RtpsRx",
        .stack_size_bytes = 6 * 1024}; ///< Base task configuration for receive sockets.
    Task::BaseConfig announce_task_config{
        .name = "RtpsAnnounce",
        .stack_size_bytes = 6 * 1024}; ///< Task configuration for periodic discovery announcements.
    std::chrono::milliseconds announce_period{1000}; ///< Interval between periodic SPDP/SEDP sends.
    Task::BaseConfig heartbeat_task_config{
        .name = "RtpsHeartbeat", .stack_size_bytes = 6 * 1024}; ///< Task configuration for periodic
                                                                ///< reliable-writer heartbeats.
    std::chrono::milliseconds heartbeat_period{200}; ///< Interval between periodic HEARTBEATs sent
                                                     ///< for reliable writers with cached samples.
    uint32_t reliable_reorder_depth{
        32}; ///< Max number of out-of-order samples buffered per reliable reader for in-order
             ///< delivery. Samples beyond this are dropped and re-requested via ACKNACK.
    std::string enclave{"/"}; ///< User-data enclave string advertised in SPDP.
    std::function<void(const ParticipantProxy &participant)> on_participant_discovered{
        nullptr}; ///< Callback invoked when a remote participant is first discovered.
    std::function<void(const EndpointProxy &endpoint)> on_endpoint_discovered{
        nullptr}; ///< Callback invoked when a remote endpoint is first discovered.
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::INFO}; ///< Participant log verbosity.
    espp::Logger::Verbosity socket_log_level{
        espp::Logger::Verbosity::WARN}; ///< Log verbosity for the participant's underlying UDP
                                        ///< sockets. Defaults to WARN so routine socket activity
                                        ///< does not clutter the logs; raise it to debug transport
                                        ///< issues independently of the participant log level.
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
  /// @return A snapshot copy of the local writer list.
  std::vector<WriterConfig> writers() const;

  /// @brief Access the registered local reader configurations.
  /// @return A snapshot copy of the local reader list.
  std::vector<ReaderConfig> readers() const;

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

  /// @brief Build a standard RTPS user-data DATA message carrying a CDR-encoded sample.
  /// @param writer_config Local writer configuration used for the topic and writer entity ID.
  /// @param cdr_payload CDR-encapsulated serialized payload (encapsulation header + body) to carry
  /// as the DATA submessage serializedPayload.
  /// @return Serialized RTPS DATA message containing the sample.
  std::vector<uint8_t> build_data_message(const WriterConfig &writer_config,
                                          std::span<const uint8_t> cdr_payload) const;

  /// @brief Publish a CDR-encoded sample on a topic using the configured user-data transport.
  /// @param topic_name Topic name to publish on. Must match a registered local writer.
  /// @param cdr_payload CDR-encapsulated serialized payload (encapsulation header + body) to send.
  /// @return True if at least one send call succeeded, false otherwise.
  bool publish(std::string_view topic_name, std::span<const uint8_t> cdr_payload);

  /// @brief Compute the standard RTPS UDP port mapping for a domain/participant pair.
  /// @param domain_id RTPS domain ID.
  /// @param participant_id RTPS participant ID.
  /// @return Derived RTPS metatraffic and user-data ports.
  static PortMapping compute_port_mapping(uint16_t domain_id, uint16_t participant_id);

private:
  struct UserMulticastReceiver {
    std::string multicast_group{};
    std::unique_ptr<UdpSocket> socket{};
  };

  bool handle_metatraffic_message(std::vector<uint8_t> &data, const Socket::Info &sender);
  bool handle_user_message(std::vector<uint8_t> &data, const Socket::Info &sender);
  bool ensure_user_multicast_receivers_started(const std::string &extra_group = {});

  /// @brief A user-data send destination plus the RTPS reader entity it targets.
  /// @details For unicast sends to a specific matched reader, reader_id is that
  ///          reader's entity id so the DATA submessage addresses it directly;
  ///          for multicast sends (which target all matched readers) it is left
  ///          as ENTITYID_UNKNOWN.
  struct UserDataDestination {
    UdpSocket::SendConfig send_config{};
    EntityId reader_id{};
  };
  std::vector<UserDataDestination> build_user_send_configs(std::string_view topic_name,
                                                           const WriterConfig &writer_config) const;
  int64_t next_spdp_sequence_number() const;
  int64_t next_sedp_publication_sequence_number() const;
  int64_t next_sedp_subscription_sequence_number() const;
  int64_t next_user_data_sequence_number(uint32_t entity_index) const;
  bool send_spdp_announce_now();
  bool send_sedp_announcements_to(const ParticipantProxy &participant);
  bool send_discovery_now();

  // SEDP serialized-payload builders (the parameter list of a publication/subscription sample,
  // without the DATA submessage framing) so the same sample can be re-sent under a stable sequence
  // number and retransmitted on ACKNACK.
  std::vector<uint8_t> build_sedp_publication_payload(const WriterConfig &writer_config) const;
  std::vector<uint8_t> build_sedp_subscription_payload(const ReaderConfig &reader_config) const;
  // Build an INFO_DST + DATA message directed at a specific reader (used for both user-data and
  // SEDP retransmission).
  std::vector<uint8_t>
  build_directed_data_message(const GuidPrefix &dest_prefix, EntityId reader_id, EntityId writer_id,
                              int64_t sequence_number,
                              std::span<const uint8_t> serialized_payload) const;
  void send_sedp_heartbeats_to(const std::string &dest_address, uint16_t dest_port,
                               const GuidPrefix &dest_prefix, size_t writer_count,
                               size_t reader_count);
  // Writer-side ACKNACK response: resend the requested sequence numbers to the requesting reader.
  void retransmit_user_data(const GuidPrefix &reader_prefix, EntityId reader_id, EntityId writer_id,
                            const std::vector<int64_t> &requested_sequence_numbers);
  void retransmit_sedp(const GuidPrefix &reader_prefix, EntityId reader_id, EntityId writer_id,
                       const std::vector<int64_t> &requested_sequence_numbers);
  // Reader-side GAP handling: mark the irrelevant sequence numbers as skipped and advance.
  // [gap_start, gap_list_base) is the contiguous irrelevant range; bitmap_irrelevant are individual
  // irrelevant sequence numbers >= gap_list_base.
  void handle_user_gap(const GuidPrefix &writer_prefix, EntityId writer_id, int64_t gap_start,
                       int64_t gap_list_base, const std::vector<int64_t> &bitmap_irrelevant);
  void handle_builtin_gap(const GuidPrefix &writer_prefix, EntityId writer_id, int64_t gap_start,
                          int64_t gap_list_base, const std::vector<int64_t> &bitmap_irrelevant);

  /// @brief Per-writer reliable-QoS state: the history of recently-sent samples
  ///        (keyed by sequence number) plus heartbeat bookkeeping. Guarded by
  ///        reliable_mutex_.
  struct WriterReliableState {
    std::map<int64_t, std::vector<uint8_t>>
        history{};                   ///< Cached CDR payloads keyed by sequence number.
    int64_t last_sequence_number{0}; ///< Highest sequence number written so far (0 = none).
    uint32_t heartbeat_count{0};     ///< Monotonic HEARTBEAT count for this writer.
  };

  /// @brief Per-(local reader, remote writer) reliable-QoS receive state used for
  ///        duplicate suppression, in-order delivery, and ACKNACK generation.
  ///        Guarded by reliable_mutex_.
  struct ReaderReliableState {
    int64_t highest_delivered{0}; ///< Highest sequence number delivered in order (0 = none).
    std::map<int64_t, std::vector<uint8_t>>
        reorder{};                    ///< Out-of-order samples awaiting their predecessors.
    std::set<int64_t> irrelevant{};   ///< Out-of-order sequence numbers a GAP marked irrelevant
                                      ///< (skipped, not delivered, when the frontier reaches them).
    uint32_t last_heartbeat_count{0}; ///< Highest HEARTBEAT count seen (stale-heartbeat detection).
    uint32_t acknack_count{0};        ///< Monotonic ACKNACK count emitted by this reader.
  };

  // Advance the reader's in-order frontier over any now-contiguous buffered samples (collected into
  // `delivered`) and irrelevant (GAP-skipped) sequence numbers. Caller must hold reliable_mutex_.
  static void drain_reader_frontier(ReaderReliableState &state,
                                    std::vector<std::vector<uint8_t>> &delivered);
  // Mark a GAP's irrelevant sequence numbers as skipped, then drain the frontier. Caller must hold
  // reliable_mutex_.
  static void apply_gap(ReaderReliableState &state, int64_t gap_start, int64_t gap_list_base,
                        const std::vector<int64_t> &bitmap_irrelevant,
                        std::vector<std::vector<uint8_t>> &delivered);

  /// @brief Hash functor for using a Guid as an unordered_map key.
  struct GuidHash {
    size_t operator()(const Guid &guid) const;
  };

  /// @brief Owns the discovered-participant and discovered-endpoint records and
  ///        the lock that guards them.
  /// @details Identity is the GUID. Updates *merge* the incoming fields into the
  ///          existing record (the caller's `apply` mutator sets only the fields
  ///          actually present in the received message), so a later announcement
  ///          that omits a locator/QoS/name does not erase previously-learned
  ///          values. Centralizing storage + merge + locking here keeps the
  ///          participant's discovery bookkeeping in one place (and is the
  ///          natural home for future lease-based expiry).
  class DiscoveryDb {
  public:
    /// @brief Result of an upsert: whether the record was newly created and a
    ///        snapshot of the merged record.
    template <typename Proxy> struct UpsertResult {
      bool is_new{false};
      Proxy value{};
    };

    UpsertResult<ParticipantProxy>
    upsert_participant(const Guid &participant_guid,
                       const std::function<void(ParticipantProxy &)> &apply);
    UpsertResult<EndpointProxy> upsert_endpoint(bool is_reader, const Guid &endpoint_guid,
                                                const std::function<void(EndpointProxy &)> &apply);

    std::optional<ParticipantProxy> find_participant_by_prefix(const GuidPrefix &prefix) const;
    std::optional<EndpointProxy> find_writer(const Guid &guid) const;

    std::vector<ParticipantProxy> participants() const;
    std::vector<EndpointProxy> writers() const;
    std::vector<EndpointProxy> readers() const;

    void clear();

  private:
    mutable std::mutex mutex_;
    std::unordered_map<Guid, ParticipantProxy, GuidHash> participants_;
    std::unordered_map<Guid, EndpointProxy, GuidHash> writers_;
    std::unordered_map<Guid, EndpointProxy, GuidHash> readers_;
  };

  std::vector<uint8_t> build_data_message_with_sequence_number(const WriterConfig &writer_config,
                                                               std::span<const uint8_t> cdr_payload,
                                                               int64_t sequence_number,
                                                               EntityId reader_id) const;
  void store_reliable_sample(const WriterConfig &writer_config, int64_t sequence_number,
                             std::span<const uint8_t> cdr_payload);
  bool send_heartbeat_for_writer(const WriterConfig &writer_config);
  bool send_heartbeats_now();
  void deliver_reliable_sample(uint32_t reader_entity_index, const Guid &writer_guid,
                               int64_t sequence_number, std::span<const uint8_t> payload,
                               const std::function<void(std::span<const uint8_t>)> &on_sample);
  void send_acknack_for_heartbeat(const GuidPrefix &writer_prefix, const EntityId &writer_id,
                                  int64_t first_sn, int64_t last_sn, uint32_t heartbeat_count,
                                  bool heartbeat_final);
  // Builtin (SPDP/SEDP) reliability: track received discovery samples and ACKNACK the heartbeats a
  // reliable peer (e.g. Fast DDS) sends for its builtin SEDP writers, so it (re)sends us the
  // endpoint discovery data we need to match it.
  void record_builtin_sample(const Guid &writer_guid, int64_t sequence_number);
  void send_builtin_acknack(const GuidPrefix &writer_prefix, const EntityId &writer_id,
                            int64_t first_sn, int64_t last_sn, uint32_t heartbeat_count,
                            bool heartbeat_final);

  Config config_;
  GuidPrefix guid_prefix_{};
  std::atomic_bool started_{false};

  std::unique_ptr<UdpSocket> metatraffic_multicast_receiver_;
  std::unique_ptr<UdpSocket> metatraffic_unicast_receiver_;
  std::vector<UserMulticastReceiver> user_multicast_receivers_;
  std::unique_ptr<UdpSocket> user_unicast_receiver_;
  std::unique_ptr<Task> announce_task_;
  std::unique_ptr<Task> heartbeat_task_;

  mutable std::mutex mutex_;
  mutable std::mutex receivers_mutex_; ///< Guards user_multicast_receivers_ against concurrent
                                       ///< add_reader()/stop() access.
  mutable std::mutex sequence_mutex_;
  mutable std::atomic<int64_t> spdp_sequence_number_{1};
  mutable std::atomic<int64_t> sedp_publications_sequence_number_{1};
  mutable std::atomic<int64_t> sedp_subscriptions_sequence_number_{1};
  mutable std::unordered_map<uint32_t, int64_t> user_data_sequence_numbers_;
  mutable std::mutex reliable_mutex_; ///< Guards writer_reliable_states_ against concurrent
                                      ///< publish()/heartbeat/receive access.
  std::unordered_map<uint32_t, WriterReliableState>
      writer_reliable_states_; ///< Reliable-QoS state keyed by writer entity_index.
  std::unordered_map<std::string, ReaderReliableState>
      reader_reliable_states_; ///< Reliable-QoS receive state keyed by "<reader index>#<writer
                               ///< GUID>".
  std::unordered_map<std::string, ReaderReliableState>
      builtin_reader_states_; ///< Received-SEDP-sample state keyed by remote builtin writer GUID,
                              ///< used to ACKNACK a reliable peer's discovery heartbeats.
  mutable std::atomic<uint32_t> sedp_pub_heartbeat_count_{0}; ///< HEARTBEAT count for our builtin
                                                              ///< SEDP publications writer.
  mutable std::atomic<uint32_t> sedp_sub_heartbeat_count_{0}; ///< HEARTBEAT count for our builtin
                                                              ///< SEDP subscriptions writer.
  std::vector<WriterConfig> writers_;
  std::vector<ReaderConfig> readers_;
  DiscoveryDb discovery_; ///< Discovered participants + endpoints (owns its own lock).
};
} // namespace espp

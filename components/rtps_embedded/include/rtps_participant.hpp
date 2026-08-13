#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "base_component.hpp"

// Forward declarations of the embeddedRTPS engine types (see
// components/rtps_embedded/include/rtps/). The engine headers are only needed
// by the implementation; users of this facade never touch them directly.
namespace rtps {
class Domain;
class Participant;
class Writer;
class Reader;
class ReaderCacheChange;
} // namespace rtps

namespace espp {

/// @brief RTPS/DDS participant for pub/sub interop with FastDDS and ROS 2.
///
/// An espp-idiomatic facade over the embeddedRTPS engine (the FastDDS/ROS 2
/// interop-proven RTPS implementation vendored in components/rtps_embedded).
/// One RtpsParticipant owns one RTPS domain participant: create it with a
/// Config, start() it, then add writers/readers and publish CDR-encapsulated
/// samples. Samples arriving on readers are delivered via the on_sample
/// callback as CDR-encapsulated payload bytes (use the reflection-driven `cdr`
/// component - cdr::serialize / cdr::deserialize - to (de)serialize them).
///
/// For ROS 2 interop, use ROS 2 naming conventions: topic "rt/<name>" and type
/// "<pkg>::msg::dds_::<Type>_" (e.g. topic "rt/chatter" with type
/// "std_msgs::msg::dds_::String_" matches a ROS 2 std_msgs/String subscriber
/// on /chatter).
///
/// Phase 1 facade (see components/rtps_embedded/REFACTOR_PLAN.md): the engine
/// beneath is unchanged, so its current limitations apply - domain id is fixed
/// at compile time (Config::DOMAIN_ID, default 0), announcement/heartbeat
/// periods are compile-time constants, endpoint counts are bounded by the
/// engine's pools, and a second RtpsParticipant in the same process will
/// collide on unicast ports (scheduled fix in Phase 2).
class RtpsParticipant : public BaseComponent {
public:
  /// Callback for samples received on a reader. The span holds the
  /// CDR-encapsulated payload (4-byte encapsulation header + CDR body) and is
  /// only valid for the duration of the callback; copy it if you keep it.
  /// \note Runs on an engine worker thread - return quickly, do not block.
  using sample_callback_t = std::function<void(std::span<const uint8_t> cdr_payload)>;

  /// Callback invoked when a remote endpoint matches one of this participant's
  /// writers (publisher matched) or readers (subscriber matched).
  /// \note Runs on an engine worker thread - return quickly, do not block.
  using matched_callback_t = std::function<void()>;

  /// Reliability QoS for a writer or reader.
  enum class Reliability {
    BEST_EFFORT, ///< Fire-and-forget delivery (stateless endpoint).
    RELIABLE,    ///< HEARTBEAT/ACKNACK acknowledged delivery (stateful endpoint).
  };

  /// Configuration for a writer (publishing endpoint).
  struct WriterConfig {
    std::string topic;     ///< DDS topic name (e.g. "rt/chatter" for ROS 2).
    std::string type_name; ///< DDS type name (e.g. "std_msgs::msg::dds_::String_").
    Reliability reliability{Reliability::BEST_EFFORT}; ///< Reliability QoS.
    /// Nominal per-fragment payload size (bytes) used when a published sample is
    /// too large for a single DATA submessage and is split into DATA_FRAG
    /// submessages. Default 63000 (large: fewer fragments). Lower it toward the
    /// path MTU (e.g. ~1400) for lossy links. Only relevant when fragmentation is
    /// compiled in (always on host; opt-in on ESP32). Ignored for samples that
    /// fit a single DATA submessage.
    uint16_t fragment_size{63000};
  };

  /// Configuration for a reader (subscribing endpoint).
  struct ReaderConfig {
    std::string topic;     ///< DDS topic name (e.g. "rt/chatter" for ROS 2).
    std::string type_name; ///< DDS type name (e.g. "std_msgs::msg::dds_::String_").
    Reliability reliability{Reliability::BEST_EFFORT}; ///< Reliability QoS.
    sample_callback_t on_sample{nullptr};              ///< Called for each received sample.
  };

  /// Configuration for the participant.
  struct Config {
    /// IPv4 address of the network interface to use. On the host, leave empty
    /// to auto-detect the first non-loopback IPv4 interface. On ESP targets it
    /// must be set explicitly (e.g. from the WiFi/Ethernet netif IP).
    std::string interface_address{};
    matched_callback_t on_publisher_matched{nullptr};     ///< A writer gained a remote reader.
    matched_callback_t on_subscriber_matched{nullptr};    ///< A reader gained a remote writer.
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Facade log verbosity.
  };

  /// Construct the participant (does not open sockets; see start()).
  /// \param config The participant configuration.
  explicit RtpsParticipant(const Config &config);

  /// Stops the participant (see stop()).
  ~RtpsParticipant();

  RtpsParticipant(const RtpsParticipant &) = delete;
  RtpsParticipant &operator=(const RtpsParticipant &) = delete;
  RtpsParticipant(RtpsParticipant &&) = delete;
  RtpsParticipant &operator=(RtpsParticipant &&) = delete;

  /// Start the participant: bring up the RTPS transport and begin SPDP/SEDP
  /// discovery. Writers and readers can only be added after a successful
  /// start().
  /// \return True on success (false if already started or bring-up failed).
  bool start();

  /// Stop the participant and its discovery/transport threads. Registered
  /// callbacks will not be invoked after stop() returns.
  void stop();

  /// \return True if the participant has been started and not stopped.
  bool is_started() const { return started_; }

  /// Add a publishing endpoint.
  /// \param config The writer configuration.
  /// \return True on success (false when not started, on duplicate topic, or
  ///         when the engine's writer pool is exhausted).
  bool add_writer(const WriterConfig &config);

  /// Add a subscribing endpoint.
  /// \param config The reader configuration.
  /// \return True on success (false when not started or when the engine's
  ///         reader pool is exhausted).
  bool add_reader(const ReaderConfig &config);

  /// Maximum size of a single published CDR payload, in bytes.
  ///
  /// When fragmentation is compiled in (RTPS_ENABLE_FRAGMENTATION - always on
  /// host, opt-in on ESP32) this is the large-sample reassembly cap
  /// (RTPS_MAX_SAMPLE_SIZE: 8 MB host, 256 KB ESP32): samples above one DATA
  /// submessage are split into DATA_FRAG submessages and reassembled by the peer.
  /// When fragmentation is compiled out (ESP32 default) it is bounded by the RTPS
  /// wire format instead: a DATA submessage's length field (octetsToNextHeader)
  /// is 16-bit, so one unfragmented sample cannot exceed 65535 bytes, and
  /// publish() rejects larger samples rather than truncating them.
#if defined(RTPS_ENABLE_FRAGMENTATION)
  static constexpr std::size_t max_payload_size = RTPS_MAX_SAMPLE_SIZE;
#else
  // The largest serialized payload that fits one unfragmented DATA submessage in
  // a single UDP datagram: 65507 (max UDP payload) - 20 (RTPS header) - 12
  // (INFO_TS) - 24 (DATA submessage header) = 65451. Kept in sync with the
  // engine's rtps::MAX_UNFRAGMENTED_PAYLOAD by a static_assert in the .cpp. A
  // larger sample would overflow the datagram and wrap the 16-bit submessage
  // length, so publish() rejects it.
  static constexpr std::size_t max_payload_size = 65451;
#endif

  /// Publish a CDR-encapsulated sample on a topic previously registered with
  /// add_writer().
  /// \param topic The topic name used in add_writer().
  /// \param cdr_payload The CDR-encapsulated sample (4-byte encapsulation
  ///        header + CDR body); copied into the writer's history. Must not
  ///        exceed max_payload_size bytes.
  /// \return True if the sample was accepted into the writer history; false if
  ///         the payload exceeds max_payload_size (see that constant).
  bool publish(std::string_view topic, std::span<const uint8_t> cdr_payload);

protected:
  /// Per-reader context bridging the engine's C function-pointer callback to
  /// the std::function callback; heap-allocated so its address stays stable
  /// for the lifetime of the reader.
  struct ReaderContext {
    RtpsParticipant *self{nullptr};
    sample_callback_t on_sample{nullptr};
    std::mutex buffer_mutex;
    std::vector<uint8_t> buffer;
  };

  static void reader_trampoline(void *arg, const rtps::ReaderCacheChange &change);
  static void publisher_matched_trampoline(void *arg);
  static void subscriber_matched_trampoline(void *arg);

  bool resolve_interface_address(std::array<uint8_t, 4> &ip_bytes) const;

  Config config_;
  std::atomic<bool> started_{false};
  mutable std::mutex mutex_; ///< guards domain_/participant_/writers_/reader_contexts_
  std::unique_ptr<rtps::Domain> domain_;
  rtps::Participant *participant_{nullptr};
  std::unordered_map<std::string, rtps::Writer *> writers_;
  std::vector<std::unique_ptr<ReaderContext>> reader_contexts_;
};

} // namespace espp

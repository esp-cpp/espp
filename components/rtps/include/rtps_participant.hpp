#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "base_component.hpp"
#include "dscp.hpp"     // espp::Dscp (per-endpoint outbound marking)
#include "qos_band.hpp" // espp::QosBand (per-endpoint / per-channel priority)
#include "timer.hpp"    // retry timer for rejected deferred-drain arms

// Forward declarations of the embeddedRTPS engine types (see
// components/rtps/include/rtps/). The engine headers are only needed
// by the implementation; users of this facade never touch them directly.
namespace rtps {
class Domain;
class Participant;
class Writer;
class Reader;
class ReaderCacheChange;
class EsppTransport;
} // namespace rtps

// The RPC layer (services + actions, ROS-interoperable and native) is compiled
// in by default. Define RTPS_NO_RPC (the ESP Kconfig option RTPS_ENABLE_RPC=n
// does this via CMake) to exclude it and its std::thread/std::future use, saving
// flash on devices that only need pub/sub. Pure pub/sub is unaffected.
#if !defined(RTPS_NO_RPC)
#define RTPS_WITH_RPC 1
#endif

namespace espp {

/// @brief RTPS/DDS participant for pub/sub interop with FastDDS and ROS 2.
///
/// An espp-idiomatic facade over the embeddedRTPS engine (the FastDDS/ROS 2
/// interop-proven RTPS implementation vendored in components/rtps).
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
/// Engine limitations that still apply (see components/rtps/REFACTOR_PLAN.md):
/// domain id is fixed at compile time (Config::DOMAIN_ID, default 0),
/// announcement/heartbeat periods are compile-time constants, and endpoint
/// counts are bounded by the engine's pools. Multiple RtpsParticipants per
/// process/host work - each probes forward to free unicast ports.
///
/// Priority scheduling: transport channels dispatch at espp::QosBand bands
/// (metatraffic High by default - Config::metatraffic_band; user traffic
/// Normal). Endpoints get per-endpoint priority via WriterConfig::band /
/// ReaderConfig::band (and ServiceConfig / ActionConfig): a banded endpoint is
/// granted a dedicated, band-scheduled (optionally DSCP-marked) unicast port,
/// announced to peers via its SEDP unicast locator; when the dedicated-port
/// ration (Config::max_prioritized_endpoint_ports) is exhausted, banded
/// readers fall back to deferred banded dispatch. See the component README's
/// "Priority scheduling" section.
///
/// \section rtps_participant_ex1 RtpsParticipant Example
/// \snippet rtps_example.cpp rtps participant example
///
/// \section rtps_full_ex1 Full Example (network bring-up + typed pub/sub + RPC)
/// \snippet rtps_example.cpp rtps example
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
    /// Priority band for this writer's endpoint (see espp::QosBand). Two
    /// distinct effects:
    ///   - Outbound pool scheduling (ALWAYS applies, no dedicated port
    ///     needed): the writer's progress()/send work is submitted to the
    ///     transport pool at this band, so a higher-band writer's outgoing
    ///     DATA is scheduled ahead of lower-band work under load.
    ///   - Inbound socket dispatch (requires a DEDICATED unicast port): a
    ///     non-Normal band (or a set dscp) requests a dedicated port so
    ///     inbound protocol traffic addressed to it (ACKNACKs from reliable
    ///     readers) is dispatched at this band and outgoing DATA leaves from
    ///     that socket. Rationed - see Config::max_prioritized_endpoint_ports;
    ///     when no dedicated port is available only the inbound-dispatch part
    ///     is lost (ACKNACKs share the participant's user-unicast port), while
    ///     the outbound pool scheduling above still applies.
    espp::QosBand band{espp::QosBand::Normal};
    /// Optional DSCP code point (e.g. espp::Dscp::Ef) marking the traffic this
    /// writer SENDS. Requires (and by itself requests) a dedicated port, since
    /// DSCP is per-socket; ignored when none could be allocated.
    std::optional<espp::Dscp> dscp{};
  };

  /// Configuration for a reader (subscribing endpoint).
  struct ReaderConfig {
    std::string topic;     ///< DDS topic name (e.g. "rt/chatter" for ROS 2).
    std::string type_name; ///< DDS type name (e.g. "std_msgs::msg::dds_::String_").
    Reliability reliability{Reliability::BEST_EFFORT}; ///< Reliability QoS.
    sample_callback_t on_sample{nullptr};              ///< Called for each received sample.
    /// Priority band for this reader's endpoint (see espp::QosBand). A
    /// non-Normal band (or a set dscp) requests a DEDICATED unicast port,
    /// announced to peers via the endpoint's SEDP unicast locator (standard
    /// DDS, honored by FastDDS/ROS 2), so this reader's samples arrive on
    /// their own socket and are dispatched at this band ahead of Normal
    /// traffic. Rationed - see Config::max_prioritized_endpoint_ports; when no
    /// dedicated port is available (or dedicated ports are disabled) the
    /// reader falls back to DEFERRED banded dispatch: its on_sample runs from
    /// a bounded per-reader queue re-submitted to the transport pool at this
    /// band (ordering preserved, one in-flight callback per reader) instead of
    /// inline on the shared-port receive worker.
    espp::QosBand band{espp::QosBand::Normal};
    /// Optional DSCP code point marking the traffic this reader SENDS (its
    /// ACKNACKs, when reliable). Requires a dedicated port; ignored when none
    /// could be allocated.
    std::optional<espp::Dscp> dscp{};
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
    /// Priority band for the metatraffic (SPDP/SEDP discovery) channels. High
    /// by default so discovery dispatch stays responsive when user traffic
    /// backs the worker pool up; set to QosBand::Normal for the exact pre-band
    /// behavior.
    espp::QosBand metatraffic_band{espp::QosBand::High};
    /// Priority band for the shared user-traffic channels (user unicast +
    /// user multicast). Normal by default (pre-band behavior).
    espp::QosBand user_traffic_band{espp::QosBand::Normal};
    /// Allow endpoints with a non-Normal band (or a dscp) to get a dedicated
    /// unicast port (see WriterConfig::band / ReaderConfig::band). Disable to
    /// force every banded endpoint onto the shared user port (readers then use
    /// deferred banded dispatch).
    bool enable_dedicated_endpoint_ports{true};
    /// Cap on dedicated endpoint ports. Each consumes one UDP socket/fd - on
    /// ESP32, lwIP's CONFIG_LWIP_MAX_SOCKETS defaults to ~10 total and the
    /// participant already uses 4 - so dedicated ports are deliberately
    /// rationed. When exhausted, further banded endpoints log a warning and
    /// fall back to the shared port (readers: deferred banded dispatch). The
    /// cap is a TRUE fd bound: a released endpoint's socket counts against it
    /// until its fd actually closes (normally immediate).
    uint8_t max_prioritized_endpoint_ports{4};
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

  // ---------------------------------------------------------------------------
  // Services (RMI: request/reply), ROS 2 (rmw_fastrtps) interoperable.
  //
  // A service maps onto a request topic (rq/<name>Request) + a reply topic
  // (rr/<name>Reply), with replies correlated to requests via the
  // related_sample_identity inline QoS (see RMI_AMI_DESIGN.md). Payloads are
  // CDR-encapsulated, exactly like publish()/on_sample: for ROS 2 the request is
  // a <Service>_Request and the reply a <Service>_Response.
  // ---------------------------------------------------------------------------

#ifdef RTPS_WITH_RPC
  /// Handler for a service server: given a CDR-encapsulated request, return the
  /// CDR-encapsulated reply. Runs on an engine worker thread - return promptly.
  using service_handler_t = std::function<std::vector<uint8_t>(std::span<const uint8_t> request)>;

  /// Configuration for a service server or client.
  struct ServiceConfig {
    std::string service; ///< ROS 2 service name, e.g. "/add_two_ints".
    /// Base DDS service type, e.g. "example_interfaces::srv::dds_::AddTwoInts".
    /// The _Request_/_Response_ suffixes are derived internally.
    std::string type_name;
    /// Priority band applied to BOTH of the service's endpoints: for a server
    /// its request reader + reply writer, for a client its request writer +
    /// reply reader (see WriterConfig::band / ReaderConfig::band for the
    /// dedicated-port / deferred-dispatch semantics; note each banded endpoint
    /// counts against Config::max_prioritized_endpoint_ports).
    espp::QosBand band{espp::QosBand::Normal};
    /// Optional DSCP code point applied to both endpoints' dedicated sockets
    /// (marks the requests/replies this side SENDS).
    std::optional<espp::Dscp> dscp{};
  };

  /// Handle to reply to a service request later (deferred reply). Copyable and
  /// movable; safe to store and fulfill from any thread. reply() sends the
  /// correlated response exactly once (subsequent calls are ignored). Used when
  /// the response is not ready when the request arrives - e.g. an action's
  /// get_result, which must wait for the goal to finish. See
  /// add_service_server_deferred().
  class ServiceResponder {
  public:
    ServiceResponder() = default; ///< Empty/invalid responder.
    /// Send the CDR-encapsulated response, correlated to the original request.
    /// No-op if invalid or already replied.
    void reply(std::span<const uint8_t> response) const;
    bool valid() const { return static_cast<bool>(state_); }

  private:
    friend class RtpsParticipant;
    struct State;
    explicit ServiceResponder(std::shared_ptr<State> state)
        : state_(std::move(state)) {}
    std::shared_ptr<State> state_;
  };

  /// Deferred service handler: invoked with the request and a responder. The
  /// handler may call responder.reply() immediately or store the responder and
  /// reply later (from any thread). Unlike service_handler_t this never blocks a
  /// worker waiting for a slow response.
  using service_deferred_handler_t =
      std::function<void(std::span<const uint8_t> request, ServiceResponder responder)>;

  /// Client handle for calling a service. Obtain one from add_service_client();
  /// it stays valid until the participant is stopped/destroyed.
  class ServiceClient {
  public:
    /// Callback delivering a CDR-encapsulated reply for a call_async() request.
    using reply_callback_t = std::function<void(std::span<const uint8_t> reply)>;

    ~ServiceClient();
    ServiceClient(const ServiceClient &) = delete;
    ServiceClient &operator=(const ServiceClient &) = delete;

    /// Send a request and invoke on_reply when the correlated reply arrives.
    /// \return False if the participant is not started or the request could not
    ///         be queued. The callback runs on an engine worker thread.
    bool call_async(std::span<const uint8_t> request, reply_callback_t on_reply);

    /// Send a request and block until the correlated reply arrives or timeout.
    /// \return The CDR-encapsulated reply, or std::nullopt on timeout/failure.
    ///         Do not call from within an engine callback (it would deadlock).
    std::optional<std::vector<uint8_t>> call(std::span<const uint8_t> request,
                                             std::chrono::milliseconds timeout);

    /// Send a request and return a future that becomes ready with the correlated
    /// reply (or std::nullopt if the request could not be queued). The future
    /// never blocks a worker thread; wait on it (or wait_for a timeout) from the
    /// caller. A pending request without a reply leaves the future unfulfilled
    /// until the participant stops.
    std::future<std::optional<std::vector<uint8_t>>> call_future(std::span<const uint8_t> request);

  private:
    friend class RtpsParticipant;
    struct Impl;
    explicit ServiceClient(std::shared_ptr<Impl> impl);
    /// shared_ptr: deferred drain jobs capture the Impl (see DeferredDispatch),
    /// so it must be shareable and outlive queued work.
    std::shared_ptr<Impl> impl_;
  };

  /// Add a service server. The handler is invoked for each request; its return
  /// value is sent back as the reply, correlated to the requesting client.
  /// \return True on success (false when not started or endpoint creation fails).
  bool add_service_server(const ServiceConfig &config, service_handler_t handler);

  /// Add a service server that replies asynchronously via a ServiceResponder.
  /// Use this when the response may not be ready when the request arrives (e.g.
  /// an action's get_result). \return True on success.
  bool add_service_server_deferred(const ServiceConfig &config, service_deferred_handler_t handler);

  /// Add a service client for calling a service.
  /// \return A client handle, or nullptr on failure (not started / endpoint
  ///         creation failed). Owned by the participant; valid until stop().
  std::shared_ptr<ServiceClient> add_service_client(const ServiceConfig &config);

  // ---------------------------------------------------------------------------
  // Actions (AMI: goal server), ROS 2 (rmw_fastrtps) interoperable.
  //
  // An action maps onto 3 services (send_goal, cancel_goal, get_result) + 2
  // topics (feedback, status) - no new wire primitive. Goal/result/feedback
  // payloads are CDR-encapsulated exactly like publish()/services (for ROS 2, the
  // action's Goal/Result/Feedback messages). See RMI_AMI_DESIGN.md.
  // ---------------------------------------------------------------------------

  /// 16-byte action goal id (unique_identifier_msgs/UUID).
  using GoalId = std::array<uint8_t, 16>;

  /// Configuration for an action server or client.
  struct ActionConfig {
    std::string action; ///< ROS 2 action name, e.g. "/fibonacci".
    /// Base DDS action type, e.g. "example_interfaces::action::dds_::Fibonacci".
    std::string type_name;
    /// Priority band inherited by ALL of the action's underlying endpoints:
    /// the send_goal/cancel_goal/get_result service endpoints and the
    /// feedback/status topic endpoints (a ROS action server is ~8 endpoints,
    /// a client ~7 - far more than the default dedicated-port ration of
    /// Config::max_prioritized_endpoint_ports, so most of a banded action's
    /// endpoints will use the shared port; readers there get deferred banded
    /// dispatch. Raise the cap if you want dedicated ports for a whole
    /// action.) Native actions inherit it on their ~3 endpoints likewise.
    espp::QosBand band{espp::QosBand::Normal};
    /// Optional DSCP code point for the endpoints' dedicated sockets.
    std::optional<espp::Dscp> dscp{};
  };

  /// Server-side handle to a running goal, passed to the execute callback (which
  /// runs on its own thread). Publish feedback and terminate the goal through it.
  class ActionGoalHandle {
  public:
    const GoalId &goal_id() const;
    /// The CDR-encapsulated goal payload.
    std::span<const uint8_t> goal() const;
    /// Publish a CDR-encapsulated feedback message for this goal.
    void publish_feedback(std::span<const uint8_t> feedback) const;
    /// Terminate the goal SUCCEEDED/ABORTED/CANCELED with a CDR result payload.
    void succeed(std::span<const uint8_t> result) const;
    void abort(std::span<const uint8_t> result) const;
    void canceled(std::span<const uint8_t> result) const;
    /// True if a cancel has been requested for this goal.
    bool is_canceling() const;

  private:
    friend class RtpsParticipant;
    struct State;
    explicit ActionGoalHandle(std::shared_ptr<State> state)
        : state_(std::move(state)) {}
    void terminate(int status_value, std::span<const uint8_t> result) const;
    std::shared_ptr<State> state_;
  };

  /// Called when a goal arrives; return true to accept, false to reject.
  using action_goal_callback_t =
      std::function<bool(const GoalId &goal_id, std::span<const uint8_t> goal)>;
  /// Called (on its own thread) to run an accepted goal to completion.
  using action_execute_callback_t = std::function<void(ActionGoalHandle handle)>;
  /// Called when a cancel is requested for a goal; return true to accept.
  using action_cancel_callback_t = std::function<bool(const GoalId &goal_id)>;

  /// Add an action server. on_goal decides acceptance; execute runs each accepted
  /// goal on its own thread; on_cancel (optional) accepts/rejects cancellations.
  /// \return True on success (false when not started or endpoint creation fails).
  bool add_action_server(const ActionConfig &config, action_goal_callback_t on_goal,
                         action_execute_callback_t execute,
                         action_cancel_callback_t on_cancel = nullptr);

  /// Client handle for calling an action. Obtain from add_action_client().
  class ActionClient {
  public:
    /// CDR-encapsulated feedback for an in-progress goal.
    using feedback_callback_t = std::function<void(std::span<const uint8_t> feedback)>;
    /// Terminal result: the GoalStatus value + the CDR-encapsulated result.
    using result_callback_t = std::function<void(int8_t status, std::span<const uint8_t> result)>;

    ~ActionClient();
    ActionClient(const ActionClient &) = delete;
    ActionClient &operator=(const ActionClient &) = delete;

    /// Send a goal. on_feedback is invoked for each feedback message; on_result
    /// once when the goal terminates (or is rejected, with an empty result).
    /// \return The generated goal id, or std::nullopt on failure.
    std::optional<GoalId> send_goal(std::span<const uint8_t> goal, feedback_callback_t on_feedback,
                                    result_callback_t on_result);
    /// Request cancellation of a previously sent goal.
    bool cancel_goal(const GoalId &goal_id);

  private:
    friend class RtpsParticipant;
    struct Impl;
    explicit ActionClient(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
  };

  /// Add an action client. \return A handle, or nullptr on failure.
  std::shared_ptr<ActionClient> add_action_client(const ActionConfig &config);

  // ---------------------------------------------------------------------------
  // Native services (espp<->espp): a lean request/reply that trades ROS 2
  // interop for simplicity. Correlation is a 20-byte in-band header (no inline
  // QoS, no rq/rr mangling), riding plain reliable pub/sub on es_rq/es_rr topics.
  // Same client ergonomics as the ROS services (sync / callback / future). NOT
  // interoperable with ROS 2 - use add_service_* for that. See RMI_AMI_DESIGN.md.
  // ---------------------------------------------------------------------------

  /// Client handle for a native service (see add_native_service_client()).
  class NativeServiceClient {
  public:
    using reply_callback_t = std::function<void(std::span<const uint8_t> reply)>;
    ~NativeServiceClient();
    NativeServiceClient(const NativeServiceClient &) = delete;
    NativeServiceClient &operator=(const NativeServiceClient &) = delete;

    /// Send a request; invoke on_reply when the correlated reply arrives.
    bool call_async(std::span<const uint8_t> request, reply_callback_t on_reply);
    /// Send a request and block for the reply (std::nullopt on timeout/failure).
    std::optional<std::vector<uint8_t>> call(std::span<const uint8_t> request,
                                             std::chrono::milliseconds timeout);
    /// Send a request and return a future for the reply.
    std::future<std::optional<std::vector<uint8_t>>> call_future(std::span<const uint8_t> request);

  private:
    friend class RtpsParticipant;
    struct Impl;
    explicit NativeServiceClient(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
  };

  /// Add a native (espp<->espp) service server. \return True on success.
  bool add_native_service_server(const ServiceConfig &config, service_handler_t handler);
  /// Add a native (espp<->espp) service client. \return A handle, or nullptr.
  std::shared_ptr<NativeServiceClient> add_native_service_client(const ServiceConfig &config);

  // ---------------------------------------------------------------------------
  // Native actions (espp<->espp): a lean AMI - one native request/reply
  // (send_goal -> {accepted, goal_handle}) + one feedback topic carrying the
  // terminal result. ~3 endpoints vs the ROS action's ~10. NOT ROS-interoperable.
  // ---------------------------------------------------------------------------

  /// Server-side handle to a running native goal (passed to the execute callback,
  /// which runs on its own thread).
  class NativeGoalHandle {
  public:
    uint32_t goal_handle() const;
    std::span<const uint8_t> goal() const;
    void publish_feedback(std::span<const uint8_t> feedback) const;
    void succeed(std::span<const uint8_t> result) const;
    void abort(std::span<const uint8_t> result) const;
    /// Terminate the goal CANCELED (in response to a cancel request).
    void canceled(std::span<const uint8_t> result) const;
    /// True if the client has requested cancellation of this goal (and the
    /// server's on_cancel, if any, accepted it). A long-running execute callback
    /// should poll this and wind down - calling canceled()/abort() - when set.
    bool is_canceling() const;

  private:
    friend class RtpsParticipant;
    struct State;
    explicit NativeGoalHandle(std::shared_ptr<State> state)
        : state_(std::move(state)) {}
    void terminate(uint8_t status, std::span<const uint8_t> result) const;
    std::shared_ptr<State> state_;
  };

  using native_goal_callback_t = std::function<bool(std::span<const uint8_t> goal)>;
  using native_execute_callback_t = std::function<void(NativeGoalHandle handle)>;
  /// Cancel policy: return true to accept a cancel request for the goal_handle
  /// (the execute callback then observes is_canceling()). Default (nullptr)
  /// accepts every cancel.
  using native_cancel_callback_t = std::function<bool(uint32_t goal_handle)>;

  /// Add a native action server. on_goal accepts/rejects; execute runs each
  /// accepted goal on its own thread; on_cancel (optional) gates cancel requests.
  /// \return True on success.
  bool add_native_action_server(const ActionConfig &config, native_goal_callback_t on_goal,
                                native_execute_callback_t execute,
                                native_cancel_callback_t on_cancel = nullptr);

  /// Client handle for a native action.
  class NativeActionClient {
  public:
    using feedback_callback_t = std::function<void(std::span<const uint8_t> feedback)>;
    /// Terminal result: the NativeGoalStatus value + the CDR result payload.
    using result_callback_t = std::function<void(uint8_t status, std::span<const uint8_t> result)>;
    /// Invoked once when the server accepts the goal, with the server-assigned
    /// goal_handle - keep it to cancel_goal() the goal later.
    using accepted_callback_t = std::function<void(uint32_t goal_handle)>;

    ~NativeActionClient();
    NativeActionClient(const NativeActionClient &) = delete;
    NativeActionClient &operator=(const NativeActionClient &) = delete;

    /// Send a goal; on_feedback per feedback message, on_result once at the end,
    /// on_accepted (optional) with the assigned goal_handle when accepted.
    /// \return True if the goal was queued.
    bool send_goal(std::span<const uint8_t> goal, feedback_callback_t on_feedback,
                   result_callback_t on_result, accepted_callback_t on_accepted = nullptr);

    /// Request cancellation of a previously accepted goal by its goal_handle
    /// (from on_accepted). \return True if the cancel request was queued.
    bool cancel_goal(uint32_t goal_handle);

  private:
    friend class RtpsParticipant;
    struct Impl;
    explicit NativeActionClient(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
  };

  /// Add a native action client. \return A handle, or nullptr.
  std::shared_ptr<NativeActionClient> add_native_action_client(const ActionConfig &config);
#endif // RTPS_WITH_RPC

protected:
  /// Deferred banded dispatch for a banded endpoint that did NOT get a
  /// dedicated port (ration exhausted or dedicated ports disabled): instead of
  /// running the user callback inline on the shared-port receive worker, each
  /// delivery is queued (bounded) and drained by a single in-flight job
  /// re-submitted to the transport's worker pool at `band` - one delivery per
  /// job, mirroring the reactor's one-shot pattern - preserving per-endpoint
  /// ordering while letting the pool schedule it against other bands. When
  /// disabled (the default path and dedicated-port endpoints), run() executes
  /// the delivery inline, exactly as before.
  struct DeferredDispatch {
    bool enabled{false};
    espp::QosBand band{espp::QosBand::Normal};
    rtps::EsppTransport *transport{nullptr};
    std::mutex mutex;                        ///< guards queue / flags / dropped
    std::deque<std::function<void()>> queue; ///< pending deliveries (bounded)
    bool in_flight{false};                   ///< a drain job is queued/running
    bool needs_arm{false};                   ///< an arm was rejected; retried by the timer
    bool closed{false};                      ///< close() ran: drop queue, refuse new work
    bool delivering{false};                  ///< a delivery callback is executing right now
    std::condition_variable drain_done;      ///< signalled when a delivery finishes (close() waits)
    std::size_t dropped{0};                  ///< deliveries dropped (queue full)
    /// Lazy retry timer, created only when the transport pool rejects a drain
    /// arm: guarantees a queued (possibly lone/last) delivery is re-armed even
    /// if no further traffic arrives. Its callback captures the OWNING context
    /// weakly (no cycle). It NEVER self-cancels (the callback always returns
    /// false, no-op'ing once there is nothing to re-arm); only close() cancels
    /// it, synchronously. A self-cancelling espp::Timer leaves running_ set
    /// while its task exits, so a later start() would no-op against a dead task
    /// and strand the parked delivery - keeping it alive avoids that race.
    std::unique_ptr<espp::Timer> retry_timer;
    /// Pending-delivery bound per endpoint: beyond it the NEWEST delivery is
    /// dropped (with a warning), so a stalled callback cannot queue without
    /// limit beyond the pool's own bounds.
    static constexpr std::size_t max_queued = 32;

    /// Run `delivery` inline (when not enabled) or enqueue it and arm the
    /// single drain job at `band`. `owner` is the shared context this
    /// dispatcher is embedded in: every drain job (and the retry timer)
    /// captures it, so queued work can never outlive the context (the
    /// lifetime model for all deferred work - see close()).
    void run_or_defer(std::function<void()> delivery, std::shared_ptr<void> owner);

    /// Quiesce: drop all queued deliveries, refuse new ones, and cancel the
    /// retry timer (synchronously - after close() returns no timer callback
    /// is running). MUST be called before the owning context's references are
    /// released and before its engine endpoints are deleted; the at-most-one
    /// in-flight delivery finishes against memory kept alive by its shared
    /// owner capture.
    void close();

  private:
    /// Submit the single drain job (in_flight must already be true). On pool
    /// rejection: flags needs_arm and starts the retry timer.
    void arm(std::shared_ptr<void> owner);
    void drain(std::shared_ptr<void> owner); ///< one delivery, then re-arm if queued
    void ensure_retry_timer_locked(const std::shared_ptr<void> &owner); ///< mutex held
  };

  /// Per-reader context bridging the engine's C function-pointer callback to
  /// the std::function callback; heap-allocated so its address stays stable
  /// for the lifetime of the reader.
  struct ReaderContext : std::enable_shared_from_this<ReaderContext> {
    RtpsParticipant *self{nullptr};
    sample_callback_t on_sample{nullptr};
    std::mutex buffer_mutex;
    std::vector<uint8_t> buffer;
    DeferredDispatch deferred;     ///< banded shared-port readers only
    std::string topic;             ///< for rollback of partially-built composites
    rtps::Reader *reader{nullptr}; ///< engine endpoint (for rollback deletion)
  };

  static void reader_trampoline(void *arg, const rtps::ReaderCacheChange &change);
  static void publisher_matched_trampoline(void *arg);
  static void subscriber_matched_trampoline(void *arg);

#ifdef RTPS_WITH_RPC
  /// Per-server bridge from the engine request-reader callback to the user
  /// handler + reply writer. Heap-allocated for a stable address; defined in the
  /// .cpp so engine types stay out of this header.
  struct ServiceServerContext;
  static void service_request_trampoline(void *arg, const rtps::ReaderCacheChange &change);
  static void service_reply_trampoline(void *arg, const rtps::ReaderCacheChange &change);

  /// Composite (action) transaction support: the internal add_* variants
  /// return the exact handle they created, and the remove_* helpers remove
  /// exactly that handle - so a failed composite rolls back only what THIS
  /// invocation built (never a concurrently added endpoint), and calling a
  /// composite twice cannot delete the first instance's endpoints. Each
  /// removal closes the handle's deferred dispatcher before deleting its
  /// engine endpoints (the deferred-work lifetime discipline).
  struct NativeServiceServerContext;
  std::shared_ptr<ServiceServerContext>
  add_service_server_deferred_internal(const ServiceConfig &config,
                                       service_deferred_handler_t handler);
  std::shared_ptr<NativeServiceServerContext>
  add_native_service_server_internal(const ServiceConfig &config, service_handler_t handler);
  /// Removal invariant (all remove_* helpers): ENGINE deletion first, facade
  /// state (registry entry, deferred close, handle fields) mutated only after
  /// the deletion is CONFIRMED. A failed deletion leaves every remaining
  /// handle - and the still-live callbacks/dispatchers they anchor - intact
  /// for retry; partial progress is recorded by nulling/clearing the already
  /// deleted endpoint's field so a retry resumes where it left off.
  bool remove_service_server(const std::shared_ptr<ServiceServerContext> &server);
  bool remove_service_client(const std::shared_ptr<ServiceClient> &client);
  bool remove_native_service_server(const std::shared_ptr<NativeServiceServerContext> &server);
  bool remove_native_service_client(const std::shared_ptr<NativeServiceClient> &client);
#endif // RTPS_WITH_RPC

  bool resolve_interface_address(std::array<uint8_t, 4> &ip_bytes) const;

  /// Rollback helpers for partially-built composite endpoints (services /
  /// actions): remove a previously added writer/reader so a failed composite
  /// leaves no announced endpoint and no consumed dedicated-port ration slot
  /// behind. Both lock mutex_ internally - callers must NOT hold it.
  bool remove_writer(const std::string &topic);
  bool remove_reader(const std::string &topic);

  Config config_;
  std::atomic<bool> started_{false};
  mutable std::mutex mutex_; ///< guards domain_/participant_/writers_/reader_contexts_
  std::unique_ptr<rtps::Domain> domain_;
  rtps::Participant *participant_{nullptr};
  std::unordered_map<std::string, rtps::Writer *> writers_;
  /// shared_ptr (not unique_ptr): deferred deliveries and drain jobs capture
  /// the context, so it stays alive until the last queued job releases it
  /// even if it is removed/rolled back first (see DeferredDispatch).
  std::vector<std::shared_ptr<ReaderContext>> reader_contexts_;

  // Endpoints whose deletion failed during a creation-time rollback (e.g. the
  // SEDP dispose could not be sent, so Domain::deleteWriter/deleteReader
  // returned false and the endpoint stayed registered with its dedicated port).
  // Retained (not dropped) so stop() can retry the deletion instead of leaking
  // an untracked endpoint. Guarded by mutex_.
  std::vector<rtps::Writer *> orphaned_writers_;
  std::vector<rtps::Reader *> orphaned_readers_;
  // Delete an endpoint during a rollback; on failure retain it in the orphan
  // list above. Called with mutex_ held.
  void rollback_delete_writer(rtps::Writer *writer);
  void rollback_delete_reader(rtps::Reader *reader);

  // Shared liveness token for async RPC reply paths. A deferred service responder
  // (which user code may hold and fulfill arbitrarily long after the request)
  // checks `alive` under this mutex before writing through its engine reply
  // writer; stop() flips it false under the same mutex before destroying the
  // domain, so a reply that races shutdown safely no-ops instead of using a
  // freed writer. A shared_ptr so it outlives the participant - a late reply
  // then sees `alive == false` and never dereferences freed state.
  struct Liveness {
    std::mutex m;
    bool alive{true};
  };
  std::shared_ptr<Liveness> live_;
#ifdef RTPS_WITH_RPC
  // shared_ptr (not unique_ptr) so an incomplete ServiceServerContext can be held
  // here: shared_ptr's destructor is type-erased, so the member vector needs no
  // complete type at the participant's construction/destruction point (the
  // context is defined in the .cpp). All the other RPC context containers below
  // follow the same rule.
  std::vector<std::shared_ptr<ServiceServerContext>> service_servers_;
  std::vector<std::shared_ptr<ServiceClient>> service_clients_;

  struct ActionServerContext;
  std::vector<std::shared_ptr<ActionServerContext>> action_servers_;
  std::vector<std::shared_ptr<ActionClient>> action_clients_;

  std::vector<std::shared_ptr<NativeServiceServerContext>> native_service_servers_;
  std::vector<std::shared_ptr<NativeServiceClient>> native_service_clients_;

  struct NativeActionServerContext;
  std::vector<std::shared_ptr<NativeActionServerContext>> native_action_servers_;
  std::vector<std::shared_ptr<NativeActionClient>> native_action_clients_;
#endif // RTPS_WITH_RPC
};

} // namespace espp

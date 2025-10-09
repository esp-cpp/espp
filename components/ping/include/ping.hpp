#pragma once

#include <condition_variable>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <lwip/inet.h>
#include <lwip/netdb.h>
#include <ping/ping_sock.h>

#include "base_component.hpp"
#include "cli.hpp"

namespace espp {

/**
 * @brief ICMP Echo (Ping) helper wrapping ESP-IDF ping API.
 *
 * This class owns the ping session lifecycle (create/start/delete) and exposes a
 * simple C++ functional interface for receiving ping results. The call to run()
 * is synchronous and returns when the session finishes or an error occurs.
 *
 * Example usage:
 * \section ping_ex_1 Simple Example
 * \snippet ping_example.cpp ping_simple_example
 * \section ping_ex_2 CLI Example
 * \snippet ping_example.cpp ping_cli_example
 */
class Ping : public BaseComponent {
public:
  /// Statistics from a completed ping session
  struct Stats {
    uint32_t transmitted{0}; ///< Number of packets transmitted
    uint32_t received{0};    ///< Number of packets received
    float loss_pct{0.0f};    ///< Packet loss percentage
    uint32_t avg_ms{0};      ///< Average round-trip time in milliseconds
    uint32_t min_ms{
        std::numeric_limits<uint32_t>::max()}; ///< Minimum round-trip time in milliseconds
    uint32_t max_ms{0};                        ///< Maximum round-trip time in milliseconds
  };

  /// Alias for the ping session start callback
  typedef std::function<void()> session_start_cb_t;
  /// Alias for the ping reply callback
  typedef std::function<void(uint32_t seq, uint32_t ttl, uint32_t time_ms, uint32_t bytes)>
      reply_cb_t;
  /// Alias for the ping timeout callback
  typedef std::function<void()> timeout_cb_t;
  /// Alias for the ping session end callback
  typedef std::function<void(const Stats &stats)> end_cb_t;

  /**
   * @brief User-provided callbacks for ping events.
   */
  struct Callbacks {
    session_start_cb_t
        on_session_start;    ///< Called right before the first probe is sent (optional).
    reply_cb_t on_reply;     ///< Called for each successful reply.
    timeout_cb_t on_timeout; ///< Called when a probe times out. One call per timed out probe.
    end_cb_t on_end;         ///< Called at session end with summary statistics.
  };

  /**
   * @brief Configuration for an individual ping session.
   */
  struct SessionConfig {
    std::string target_host;      ///< Hostname or dotted-quad IPv4 address
    size_t task_stack_size{4096}; ///< Stack size for the ping task
    uint32_t count{5};            ///< Number of echo requests to send
    uint32_t interval_ms{1000};   ///< Interval between requests in milliseconds
    uint32_t timeout_ms{1000};    ///< Per-request timeout in milliseconds
    uint32_t data_size{0};        ///< Payload size in bytes
    uint8_t ttl{64};              ///< Time-to-live in the range [1, 255]
    uint8_t tos{0};               ///< Type of service in the range [0, 255]
  };

  /**
   * @brief Configuration for a ping session.
   */
  struct Config {
    SessionConfig session{};                              ///< Ping session parameters
    Callbacks callbacks{};                                ///< Optional user callbacks
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Component log level
  };

  /**
   * @brief Construct a Ping helper.
   * @param config Ping configuration and callbacks.
   */
  explicit Ping(const Config &config);

  /**
   * @brief Destructor cleans up any active session.
   */
  ~Ping();

  /**
   * @brief Run the ping session synchronously, blocking until complete or stopped.
   * @param ec Error code set on failure.
   * @return true if session completed successfully, false otherwise.
   */
  bool run(std::error_code &ec);

  /**
   * @brief Set new session configuration and run the ping session synchronously.
   * @param session_config New session configuration.
   * @param ec Error code set on failure.
   * @return true if session completed successfully, false otherwise.
   */
  bool run(const SessionConfig &session_config, std::error_code &ec) {
    std::lock_guard<std::mutex> clk(config_mutex_);
    config_.session = session_config;
    return run(ec);
  }

  /**
   * @brief Start the ping session asynchronously, returning immediately.
   * @param ec Error code set on failure.
   * @return true if the session started, false otherwise.
   */
  bool run_async(std::error_code &ec);

  /**
   * @brief Run the ping session synchronously, blocking until complete or stopped.
   * @param ec Error code set on failure.
   * @return true if the session completed, false otherwise.
   */
  bool run_sync(std::error_code &ec);

  /**
   * @brief Stop an active ping session.
   */
  void stop();

  /**
   * @brief Check if a ping session is currently running.
   * @return true if a session is active, false otherwise.
   */
  bool is_running() const;

  /**
   * @brief Set the target host (thread-safe).
   * @param host Hostname or dotted-quad IPv4 address.
   */
  void set_target_host(std::string_view host);

  /**
   * @brief Get the current target host.
   * @return Target host (empty string if not set).
   */
  std::string get_target_host() const;

  /**
   * @brief Set number of echo requests to send.
   * @param count Number of requests (0 = infinite).
   */
  void set_count(uint32_t count);

  /**
   * @brief Get number of echo requests to send.
   * @return Number of requests (0 = infinite).
   */
  uint32_t get_count() const;

  /**
   * @brief Set interval between requests in milliseconds.
   * @param interval_ms Interval in milliseconds.
   */
  void set_interval_ms(uint32_t interval_ms);

  /**
   * @brief Get interval between requests in milliseconds.
   * @return Interval in milliseconds.
   */
  uint32_t get_interval_ms() const;

  /**
   * @brief Set per-request timeout in milliseconds.
   * @param timeout_ms Timeout in milliseconds.
   */
  void set_timeout_ms(uint32_t timeout_ms);

  /**
   * @brief Get per-request timeout in milliseconds.
   * @return Timeout in milliseconds.
   */
  uint32_t get_timeout_ms() const;

  /**
   * @brief Set payload size in bytes.
   * @param data_size Payload size in bytes.
   */
  void set_data_size(uint32_t data_size);

  /**
   * @brief Get payload size in bytes.
   * @return Payload size in bytes.
   */
  uint32_t get_data_size() const;

  /**
   * @brief Set time-to-live value.
   * @param ttl Time-to-live value.
   */
  void set_ttl(uint8_t ttl);

  /**
   * @brief Get time-to-live value.
   * @return Time-to-live value.
   */
  uint8_t get_ttl() const;

  /**
   * @brief Set IP type-of-service value.
   * @param tos Type-of-service value.
   */
  void set_tos(uint8_t tos);

  /**
   * @brief Get IP type-of-service value.
   * @return Type-of-service value.
   */
  uint8_t get_tos() const;

  /**
   * @brief Set ping task stack size.
   * @param bytes Stack size in bytes.
   */
  void set_task_stack_size(size_t bytes);

  /**
   * @brief Get ping task stack size.
   * @return Stack size in bytes.
   */
  size_t get_task_stack_size() const;

  /**
   * @brief Get a copy of the current session configuration (thread-safe).
   * @return Current session configuration.
   */
  SessionConfig get_session_config() const;

  /**
   * @brief Get statistics from the last completed ping session.
   * @return Statistics structure (all zeros if no session completed).
   */
  Stats get_stats() const {
    std::lock_guard<std::mutex> slk(stats_mutex_);
    return stats_;
  }

  // Create a CLI submenu to run ping
  /**
   * @brief Small CLI wrapper for invoking ping from a menu.
   */
  class Menu {
  public:
    /**
     * @brief Construct a CLI wrapper bound to a Ping instance.
     * @param ping Reference to a Ping instance.
     */
    explicit Menu(std::reference_wrapper<Ping> ping)
        : ping_(ping) {}

    /**
     * @brief Create a CLI submenu with a `run \<host\>` command.
     * @param name Menu name (default: "ping")
     * @param description Menu description (default: "Ping menu")
     * @return Unique pointer to the menu instance.
     */
    std::unique_ptr<cli::Menu> get(std::string_view name = "ping",
                                   std::string_view description = "Ping menu");

  private:
    std::reference_wrapper<Ping> ping_;
  };

private:
  /** @cond INTERNAL */
  static void on_ping_success(void *h, void *args);
  static void on_ping_timeout(void *h, void *args);
  static void on_ping_end(void *h, void *args);
  /** @endcond */

  void handle_success(void *h);
  void handle_timeout(void *h);
  void handle_end(void *h);

  /** @cond INTERNAL */
  bool create_session(std::error_code &ec);
  void destroy_session();

  ip_addr_t resolve_target(std::error_code &ec) const;
  /** @endcond */

  mutable std::mutex stats_mutex_;
  Stats stats_{};

  // configuration and session state
  mutable std::mutex config_mutex_;
  Config config_;

  mutable std::mutex state_mutex_;
  std::condition_variable state_cv_;
  bool session_active_{false};
  bool session_completed_{false};
  esp_ping_handle_t handle_{nullptr};
};

} // namespace espp

// fmt formatters for espp::Ping::Stats, espp::Ping::SessionConfig, and espp::Ping::Config
template <> struct fmt::formatter<espp::Ping::Stats> : fmt::formatter<std::string_view> {
  template <typename FormatContext>
  auto format(const espp::Ping::Stats &s, FormatContext &ctx) const {
    return fmt::format_to(
        ctx.out(), "transmitted={} received={} loss_pct={:.02f}% avg_ms={} min_ms={} max_ms={}",
        s.transmitted, s.received, s.loss_pct, s.avg_ms, s.min_ms, s.max_ms);
  }
};

template <> struct fmt::formatter<espp::Ping::SessionConfig> : fmt::formatter<std::string_view> {
  template <typename FormatContext>
  auto format(const espp::Ping::SessionConfig &s, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(),
                          "host={} count={} interval_ms={} timeout_ms={} size={} ttl={} tos={} "
                          "stack={}",
                          s.target_host, s.count, s.interval_ms, s.timeout_ms, s.data_size,
                          (int)s.ttl, (int)s.tos, s.task_stack_size);
  }
};

template <> struct fmt::formatter<espp::Ping::Config> : fmt::formatter<std::string_view> {
  template <typename FormatContext>
  auto format(const espp::Ping::Config &c, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "{{ session: ({}) }}", c.session);
  }
};

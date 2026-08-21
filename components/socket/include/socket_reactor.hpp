#pragma once

#include "socket_msvc.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include "base_component.hpp"
#include "socket.hpp"
#include "task.hpp"
#include "tcp_socket.hpp"
#include "thread_pool.hpp"
#include "udp_socket.hpp"

namespace espp {
/**
 * @brief A single-threaded select() event loop that multiplexes many receiver
 *        sockets and dispatches their read handling onto a shared thread pool.
 *
 * @details Instead of dedicating one background thread (one @ref espp::Task) to
 *          every receiving socket, register the sockets with a single
 *          SocketReactor. One loop thread waits in @c ::select() on all
 *          registered file descriptors at once; when a socket becomes readable
 *          the reactor hands that socket's read + user callback to a
 *          @ref espp::ThreadPool worker. This collapses "N receiver threads"
 *          down to "1 loop thread + a small fixed pool", which can additionally
 *          be shared across several subsystems.
 *
 *          To keep per-socket handling correct under level-triggered
 *          @c select(), the reactor is one-shot: when a socket is reported
 *          readable it is @b disarmed (removed from the interest set) and a job
 *          is submitted; the socket is @b re-armed only after that job
 *          completes. This guarantees at most one in-flight handler per socket
 *          (so per-socket ordering is preserved and there is no concurrent
 *          @c recv on the same fd), while different sockets' handlers run
 *          concurrently on the pool.
 *
 *          Registration changes, re-arming, and stop are made immediately
 *          responsive by a loopback UDP "wakeup" socket that is also in the
 *          select set: poking it interrupts @c select() at once.
 *
 *          Each registration carries a priority band (@ref espp::QosBand,
 *          default Normal). When one @c select() wakeup reports several
 *          sockets readable at once, their handlers are submitted to the pool
 *          in band order (most urgent first) and each is submitted AT its band,
 *          so band-aware pools (see @ref espp::ThreadPool) run urgent sockets'
 *          handlers first. This also shapes the saturation policy: when the
 *          pool is (nearly) full, urgent sockets win the remaining queue slots
 *          while less urgent ones simply stay readable and are re-reported by
 *          the next @c select() (kernel-buffer backpressure, no data loss for
 *          TCP / bounded loss semantics identical to before for UDP). With the
 *          default band on every registration the dispatch order and behavior
 *          are unchanged from the pre-band reactor.
 *
 * @note Lifetime. Registered sockets and callbacks must outlive their
 *       registration. @ref stop (and the destructor) waits for any in-flight
 *       handler to finish, so the guaranteed-safe teardown order is: stop() /
 *       destroy the reactor first, then destroy the sockets. @ref remove is
 *       *asynchronous* with respect to a handler that is already running for
 *       that id (it unregisters, but a handler mid-recv() will still finish);
 *       do not free a socket immediately after remove() while its handler may
 *       be executing - unregister and then rely on reactor teardown, or ensure
 *       the socket outlives the reactor. @ref stop (and destroying the reactor)
 *       must NOT be called from within a handler: it waits for the calling
 *       handler to finish (and joins an owned pool), which would deadlock - stop
 *       from another thread. A stop() invoked from a handler is refused + logged.
 *
 * @note The select() backend uses @c fd_set, which on POSIX/lwip can only hold
 *       file descriptors with value < @c FD_SETSIZE. Registration rejects an fd
 *       at or above that limit. On lwip, socket fds occupy
 *       [FD_SETSIZE - CONFIG_LWIP_MAX_SOCKETS, FD_SETSIZE), so this is only hit
 *       if those limits are raised past the compiled @c FD_SETSIZE.
 *
 * \section socket_reactor_ex1 Socket Reactor UDP Example
 * \snippet socket_example.cpp socket reactor example
 * \section socket_reactor_ex2 Socket Reactor TCP Example
 * \snippet socket_example.cpp socket reactor tcp example
 */
class SocketReactor : public BaseComponent {
public:
  /// Opaque handle for a registration, returned by the add_* methods and
  /// passed to @ref remove. INVALID_ID (0) is never returned on success.
  using Id = std::uint32_t;
  static constexpr Id INVALID_ID = 0; ///< Never a valid registration id.

  /**
   * @brief Low-level read handler, invoked on a thread-pool worker when the
   *        registered socket is readable.
   * @note The reactor guarantees at most one in-flight invocation per
   *       registration (disarm-on-dispatch, re-arm on completion), so the
   *       handler need not guard against concurrent calls for the same socket.
   *       The handler is expected to read from the socket (draining it as
   *       appropriate) and process the data.
   */
  using ReadHandler = std::function<void()>;

  /// Called (on a pool worker) when a listening TcpSocket accepts a new client.
  /// The consumer takes ownership of @p client and typically registers it with
  /// add_tcp_stream() on this same reactor.
  using AcceptCallback = std::function<void(std::unique_ptr<espp::TcpSocket> client)>;

  /// Called (on a pool worker) with data read from a connected TcpSocket.
  using StreamCallback =
      std::function<void(espp::TcpSocket &connection, std::vector<uint8_t> &data)>;

  /// Called (on a pool worker) when a connected TcpSocket reaches EOF / closes.
  /// After this fires the stream is automatically unregistered.
  using CloseCallback = std::function<void()>;

  /**
   * @brief Configuration for the SocketReactor.
   */
  struct Config {
    std::shared_ptr<espp::ThreadPool> thread_pool{
        nullptr}; ///< Pool to dispatch handlers on. If null, the reactor creates and owns one from
                  ///< @ref pool_config; provide a shared pool to share workers across subsystems.
    espp::ThreadPool::Config pool_config{
        .worker_count = 2,
        .worker_task_config = {.name = "SocketReactor pool",
                               .stack_size_bytes = 4096,
                               .priority = 5}}; ///< Used only when @ref thread_pool is null.
    espp::Task::BaseConfig loop_task_config{.name = "SocketReactor",
                                            .stack_size_bytes = 4096,
                                            .priority = 5,
                                            .core_id =
                                                -1}; ///< Config for the single select() loop task.
    std::chrono::microseconds select_timeout{
        std::chrono::seconds(1)}; ///< Max time select() blocks; the wakeup socket makes
                                  ///< registration/stop responsive regardless of this.
    bool auto_start{true};        ///< Start the loop (and owned pool) on construction.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Logger verbosity.
  };

  /**
   * @brief Construct the reactor (and, if configured, start it).
   * @param config Configuration for the reactor.
   */
  explicit SocketReactor(const Config &config);

  /// Stop the loop and unregister everything.
  ~SocketReactor();

  /**
   * @brief Start the select() loop and (if owned) the thread pool.
   * @return true if the reactor is running.
   */
  bool start();

  /**
   * @brief Stop the loop, wait for any in-flight handlers to finish, and (if
   *        owned) stop the thread pool.
   */
  void stop();

  /// @return true if the loop is running.
  bool is_running() const;

  /**
   * @brief Bind @p socket per @p receive_config and register it to receive on
   *        this reactor. When a datagram arrives the reactor calls
   *        @c socket.receive() and invokes @c receive_config.on_receive_callback
   *        with the data and sender on a pool worker; if the callback returns
   *        data, it is sent back to the sender.
   * @note This replaces UdpSocket::start_receiving() (which spawns a dedicated
   *       thread) - the reactor drives the socket instead.
   * @note The receive_config's band field selects the priority band this
   *       socket's handling is dispatched at, and its dscp field (if set) is
   *       applied to the socket via IP_TOS here (best-effort, marks
   *       transmitted packets - see UdpSocket::ReceiveConfig).
   * @param socket A UdpSocket to bind and receive on. Must outlive the
   *        registration (call remove() before destroying it).
   * @param receive_config Port / multicast / buffer_size / callback / band /
   *        dscp config.
   * @return A registration Id, or INVALID_ID on failure.
   */
  Id add_udp_receiver(espp::UdpSocket &socket,
                      const espp::UdpSocket::ReceiveConfig &receive_config);

  /**
   * @brief Register a listening TcpSocket. When a connection is pending the
   *        reactor calls @c listener.accept() and invokes @p on_accept with the
   *        new client on a pool worker. The listener stays registered.
   * @param listener A TcpSocket that has already been bind()+listen()'d. Must
   *        outlive the registration.
   * @param on_accept Callback given ownership of each accepted client.
   * @param band Priority band to dispatch accept handling at (see
   *        espp::QosBand; default Normal = pre-band behavior).
   * @return A registration Id, or INVALID_ID on failure.
   */
  Id add_tcp_listener(espp::TcpSocket &listener, const AcceptCallback &on_accept,
                      QosBand band = QosBand::Normal);

  /**
   * @brief Register a connected TcpSocket for reading. When it is readable the
   *        reactor reads up to @p buffer_size bytes and invokes @p on_data on a
   *        pool worker. On EOF / disconnect it invokes @p on_close (if set) and
   *        automatically unregisters the stream.
   * @param connection A connected TcpSocket (e.g. from add_tcp_listener's
   *        callback or TcpSocket::connect). Must outlive the registration.
   * @param on_data Callback given the connection and the bytes read.
   * @param buffer_size Max bytes to read per readable event.
   * @param on_close Optional callback fired once when the peer closes.
   * @param band Priority band to dispatch read handling at (see espp::QosBand;
   *        default Normal = pre-band behavior).
   * @return A registration Id, or INVALID_ID on failure.
   */
  Id add_tcp_stream(espp::TcpSocket &connection, const StreamCallback &on_data, size_t buffer_size,
                    const CloseCallback &on_close = {}, QosBand band = QosBand::Normal);

  /**
   * @brief Low-level registration: watch @p fd for readability and run
   *        @p handler (on a pool worker) each time it is readable.
   * @param fd A valid socket file descriptor (see Socket::native_handle()).
   * @param handler Handler that reads/processes the socket.
   * @param band Priority band to dispatch the handler at (see espp::QosBand;
   *        default Normal = pre-band behavior).
   * @return A registration Id, or INVALID_ID on failure.
   */
  Id add_fd(sock_type_t fd, ReadHandler handler, QosBand band = QosBand::Normal);

  /**
   * @brief Unregister a socket. Safe to call from any thread, including from
   *        within a running handler. If a handler for this id is currently
   *        in-flight, the entry is erased once it completes.
   * @param id The registration Id returned by an add_* method.
   * @return true if the id was found.
   */
  bool remove(Id id);

  /// @return the number of currently registered sockets.
  size_t num_registered() const;

protected:
  struct Entry {
    sock_type_t fd{static_cast<sock_type_t>(-1)}; ///< Watched file descriptor.
    ReadHandler handler;                          ///< Handler run on the pool.
    QosBand band{QosBand::Normal};                ///< Priority band for dispatch.
    bool armed{true};                             ///< In the select set (not currently dispatched).
    bool in_flight{false};                        ///< A pool job is currently running the handler.
    bool remove_requested{false};                 ///< remove() was called while in-flight.
  };

  /// Validate an fd for registration: must be valid and (for the select()
  /// backend, on POSIX/lwip) below FD_SETSIZE so FD_SET is not UB. Logs on
  /// failure.
  bool check_fd(sock_type_t fd) const;

  /// Allocate a fresh registration id (thread-safe, never INVALID_ID).
  Id allocate_id();
  /// Insert an entry for a pre-allocated id and wake the loop. Used so a
  /// stream handler can capture its own id before the entry becomes reachable.
  void insert_entry(Id id, sock_type_t fd, ReadHandler handler, QosBand band);

  /// One iteration of the select() loop (the loop task callback body).
  bool loop_iteration(std::mutex &m, std::condition_variable &cv, bool &task_notified);
  /// Run an entry's handler on the pool, then re-arm (or erase) it.
  void dispatch(Id id);
  /// Interrupt select() by poking the wakeup socket.
  void wake();
  bool create_wakeup_socket();
  void close_wakeup_socket();
  static bool set_nonblocking(sock_type_t fd);

  Config config_;
  std::shared_ptr<espp::ThreadPool> pool_;
  bool owns_pool_{false};
  std::unique_ptr<espp::Task> loop_task_;
  std::atomic<bool> running_{false};
  std::atomic<int> in_flight_count_{0};

  mutable std::mutex mutex_; ///< Guards entries_ and next_id_.
  std::map<Id, Entry> entries_;
  Id next_id_{INVALID_ID};

  std::atomic<sock_type_t> wakeup_recv_{
      static_cast<sock_type_t>(-1)};  ///< Loopback UDP socket in the select set. Atomic so wake()
                                      ///< (called from arbitrary threads) races safely with the
                                      ///< create/close in start()/stop().
  struct sockaddr_in wakeup_addr_ {}; ///< Its own address, for self-sendto (stable after create).
};
} // namespace espp

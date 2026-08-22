#include "socket_reactor.hpp"

#include <algorithm>
#include <exception>
#include <thread>
#include <utility>

#ifndef _WIN32
#include <fcntl.h>
#endif

using namespace espp;

namespace {
// True while the current thread is running a reactor dispatch handler. Used to
// detect (and refuse) a stop() called from within a handler, which would
// otherwise deadlock waiting for the calling handler to finish.
thread_local bool t_in_reactor_dispatch = false;
struct DispatchGuard {
  bool prev;
  DispatchGuard()
      : prev(t_in_reactor_dispatch) {
    t_in_reactor_dispatch = true;
  }
  ~DispatchGuard() { t_in_reactor_dispatch = prev; }
};

// Close a socket with the platform-correct call (Winsock sockets must use
// closesocket(), not ::close()).
void close_socket(sock_type_t fd) {
#if defined(_WIN32)
  closesocket(fd);
#else
  ::close(fd);
#endif
}
} // namespace

SocketReactor::SocketReactor(const SocketReactor::Config &config)
    : BaseComponent(config.loop_task_config.name, config.log_level)
    , config_(config) {
  if (config_.thread_pool) {
    pool_ = config_.thread_pool;
    owns_pool_ = false;
  } else {
    auto pool_config = config_.pool_config;
    pool_config.log_level = config_.log_level;
    pool_ = std::make_shared<espp::ThreadPool>(pool_config);
    owns_pool_ = true;
  }
  if (config_.auto_start) {
    start();
  }
}

SocketReactor::~SocketReactor() {
  // Destroying the reactor from within one of its own handlers is a fatal
  // lifetime violation: the handler is executing inside the object being freed,
  // so there is no safe teardown (stop() would refuse, and we would then clear
  // live loop/pool state - a use-after-free). Fail loudly instead.
  if (t_in_reactor_dispatch) {
    logger_.error("SocketReactor destroyed from within a reactor handler; terminating "
                  "(destroy/stop the reactor from another thread).");
    std::terminate();
  }
  stop();
  std::lock_guard<std::mutex> lock(mutex_);
  entries_.clear();
}

bool SocketReactor::start() {
  if (running_) {
    return true;
  }
  if (!create_wakeup_socket()) {
    logger_.error("Could not create wakeup socket, not starting");
    return false;
  }
  // Make sure the (owned) pool is running before we start dispatching to it.
  if (owns_pool_ && !pool_->is_running()) {
    pool_->start();
  }
  running_ = true;
  loop_task_ = espp::Task::make_unique({
      .callback = [this](std::mutex &m, std::condition_variable &cv, bool &task_notified) -> bool {
        return loop_iteration(m, cv, task_notified);
      },
      .task_config = config_.loop_task_config,
      .log_level = config_.log_level,
  });
  if (!loop_task_->start()) {
    logger_.error("Could not start reactor loop task");
    running_ = false;
    close_wakeup_socket();
    return false;
  }
  logger_.info("SocketReactor started");
  return true;
}

void SocketReactor::stop() {
  // stop() waits for in-flight handlers to finish (and, for an owned pool, joins
  // the pool workers). Calling it from within a handler would therefore wait for
  // the calling handler - a deadlock (and, for an owned pool, a self-join).
  // Refuse rather than hang; the caller must stop the reactor from another thread.
  if (t_in_reactor_dispatch) {
    logger_.error("stop() called from within a reactor handler; refusing (it would deadlock). "
                  "Stop/destroy the reactor from another thread.");
    return;
  }
  if (!running_ && !loop_task_) {
    return;
  }
  // Signal the loop to exit and interrupt select() so it notices immediately.
  running_ = false;
  wake();
  if (loop_task_) {
    loop_task_->stop(); // joins the loop thread
    loop_task_.reset();
  }
  // Wait for any handlers still running on the pool to finish before we tear
  // down, so their captured `this` stays valid. The loop task is stopped, so no
  // new dispatches begin and in_flight_count_ only decreases. For an owned pool
  // pool_->stop() below also joins the workers; for a shared pool this wait is
  // the only thing preventing a use-after-free, so it must not give up early -
  // handlers are required to be finite (see the class documentation).
  auto warn_at = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (in_flight_count_ > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (std::chrono::steady_clock::now() > warn_at) {
      logger_.warn("Still waiting for {} in-flight handler(s) to finish", in_flight_count_.load());
      warn_at += std::chrono::seconds(2);
    }
  }
  if (owns_pool_ && pool_) {
    pool_->stop();
  }
  close_wakeup_socket();
  logger_.info("SocketReactor stopped");
}

bool SocketReactor::is_running() const { return running_; }

bool SocketReactor::check_fd(sock_type_t fd) const {
  if (!Socket::is_valid_fd(fd)) {
    logger_.error("register: invalid socket fd");
    return false;
  }
#if !defined(_WIN32)
  // The select() backend uses fd_set, a bitmap indexed by fd value on
  // POSIX/lwip; FD_SET(fd) with fd >= FD_SETSIZE is undefined behavior. (On
  // Winsock, fd_set is a bounded array of SOCKETs, so the value is not the
  // limit - the count is - and this check does not apply.)
  if (fd >= FD_SETSIZE) {
    logger_.error("register: socket fd {} is >= FD_SETSIZE ({}); raise FD_SETSIZE (and "
                  "CONFIG_LWIP_MAX_SOCKETS on ESP) to watch this many sockets",
                  static_cast<int>(fd), static_cast<int>(FD_SETSIZE));
    return false;
  }
#endif
  return true;
}

SocketReactor::Id SocketReactor::allocate_id() {
  std::lock_guard<std::mutex> lock(mutex_);
  // Monotonic ids, skipping INVALID_ID on wrap.
  do {
    ++next_id_;
  } while (next_id_ == INVALID_ID);
  return next_id_;
}

void SocketReactor::insert_entry(Id id, sock_type_t fd, ReadHandler handler, QosBand band) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    entries_[id] = Entry{.fd = fd, .handler = std::move(handler), .band = band};
  }
  wake(); // interrupt select() so the new fd is picked up
}

SocketReactor::Id SocketReactor::add_fd(sock_type_t fd, SocketReactor::ReadHandler handler,
                                        QosBand band) {
  if (!handler) {
    logger_.error("add_fd: null handler");
    return INVALID_ID;
  }
  if (!check_fd(fd)) {
    return INVALID_ID;
  }
  Id id = allocate_id();
  insert_entry(id, fd, std::move(handler), band);
  return id;
}

SocketReactor::Id
SocketReactor::add_udp_receiver(espp::UdpSocket &socket,
                                const espp::UdpSocket::ReceiveConfig &receive_config) {
  if (!socket.bind(receive_config)) {
    logger_.error("add_udp_receiver: could not bind socket to port {}", receive_config.port);
    return INVALID_ID;
  }
  const auto callback = receive_config.on_receive_callback;
  const auto buffer_size = receive_config.buffer_size;
  sock_type_t fd = socket.native_handle();
  if (receive_config.dscp.has_value()) {
    // Mark this socket's transmitted packets (e.g. echo responses) with the
    // requested DSCP code point. The TOS byte carries the 6-bit DSCP in its
    // upper bits (RFC 2474). Best-effort: network / driver treatment only, no
    // effect on local scheduling (that is what `band` is for).
    const int tos = (receive_config.dscp.value() & 0x3F) << 2;
    if (::setsockopt(fd, IPPROTO_IP, IP_TOS, reinterpret_cast<const char *>(&tos), sizeof(tos)) <
        0) {
      logger_.warn("add_udp_receiver: could not set IP_TOS (DSCP {}) on port {}",
                   receive_config.dscp.value(), receive_config.port);
    }
  }
  auto handler = [this, &socket, callback, buffer_size]() {
    std::vector<uint8_t> data;
    Socket::Info sender;
    if (!socket.receive(buffer_size, data, sender)) {
      // Transient (e.g. spurious wakeup) - the socket will be re-armed and we
      // will try again on the next readable event.
      return;
    }
    if (!callback) {
      return;
    }
    auto maybe_response = callback(data, sender);
    if (!maybe_response.has_value() || maybe_response->empty()) {
      return;
    }
    auto *sender_address = sender.ipv4_ptr();
    auto &response = maybe_response.value();
    int sent = ::sendto(socket.native_handle(), reinterpret_cast<const char *>(response.data()),
                        response.size(), 0, reinterpret_cast<struct sockaddr *>(sender_address),
                        sizeof(*sender_address));
    if (sent < 0) {
      logger_.warn("Failed to send UDP response to {}", sender);
    }
  };
  return add_fd(fd, std::move(handler), receive_config.band);
}

SocketReactor::Id SocketReactor::add_tcp_listener(espp::TcpSocket &listener,
                                                  const AcceptCallback &on_accept, QosBand band) {
  sock_type_t fd = listener.native_handle();
  auto handler = [this, &listener, on_accept]() {
    // select() reported the listener readable, so accept() returns immediately.
    auto client = listener.accept();
    if (!client) {
      return; // transient (e.g. the pending connection went away)
    }
    if (on_accept) {
      on_accept(std::move(client));
    }
  };
  return add_fd(fd, std::move(handler), band);
}

SocketReactor::Id SocketReactor::add_tcp_stream(espp::TcpSocket &connection,
                                                const StreamCallback &on_data, size_t buffer_size,
                                                const CloseCallback &on_close, QosBand band) {
  sock_type_t fd = connection.native_handle();
  if (!check_fd(fd)) {
    return INVALID_ID;
  }
  // Reserve the id first so the handler can unregister itself on disconnect.
  Id id = allocate_id();
  auto handler = [this, &connection, on_data, on_close, buffer_size, id]() {
    std::vector<uint8_t> data;
    if (connection.receive(data, buffer_size)) {
      if (on_data) {
        on_data(connection, data);
      }
      return;
    }
    // receive() returned false after a readable event: this is terminal - either
    // a clean EOF (recv == 0, is_connected() now false) or a socket error such
    // as a peer RST (recv < 0). Both mean stop watching this connection;
    // treating a non-EOF error as "transient" here would busy-loop the reactor
    // (select keeps reporting the fd readable). Fire on_close and unregister.
    if (on_close) {
      on_close();
    }
    remove(id);
  };
  insert_entry(id, fd, std::move(handler), band);
  return id;
}

bool SocketReactor::remove(SocketReactor::Id id) {
  bool found = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = entries_.find(id);
    if (it != entries_.end()) {
      found = true;
      if (it->second.in_flight) {
        // A handler is running; defer erasure until dispatch() completes.
        it->second.remove_requested = true;
      } else {
        entries_.erase(it);
      }
    }
  }
  if (found) {
    wake();
  }
  return found;
}

size_t SocketReactor::num_registered() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return entries_.size();
}

void SocketReactor::dispatch(SocketReactor::Id id) {
  // Decrement the in-flight count when this dispatch finishes by ANY path (early
  // return, or an exception thrown from the handler), so stop()'s wait on
  // in_flight_count_ can never hang.
  struct CountGuard {
    std::atomic<int> &count;
    ~CountGuard() { --count; }
  } count_guard{in_flight_count_};

  ReadHandler handler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = entries_.find(id);
    if (it == entries_.end()) {
      return; // removed before we got here
    }
    handler = it->second.handler; // copy so we can run it unlocked
  }
  // Run the user handler without holding the lock (it may call back into the
  // reactor, block on a mutex, etc.). Mark the thread so a stop() invoked from
  // within the handler is detected rather than deadlocking, and never let a
  // handler exception escape into the pool worker (it would kill the worker and
  // leave the entry stuck in_flight).
  if (handler) {
    DispatchGuard guard;
#if defined(__cpp_exceptions) && __cpp_exceptions
    try {
      handler();
    } catch (const std::exception &e) {
      logger_.error("Exception in reactor handler: {}", e.what());
    } catch (...) {
      logger_.error("Unknown exception in reactor handler");
    }
#else
    // C++ exceptions are disabled (e.g. the ESP-IDF default), so a throwing
    // handler would abort regardless; call it directly. The RAII CountGuard
    // still keeps in_flight_count_ consistent for normal returns.
    handler();
#endif
  }
  bool wake_needed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = entries_.find(id);
    if (it != entries_.end()) {
      it->second.in_flight = false;
      if (it->second.remove_requested) {
        entries_.erase(it);
      } else {
        it->second.armed = true; // re-arm so the loop watches it again
        wake_needed = true;
      }
    }
  }
  if (wake_needed) {
    wake();
  }
}

bool SocketReactor::loop_iteration(std::mutex &, std::condition_variable &, bool &) {
  if (!running_) {
    return true; // stop the task
  }

  const sock_type_t wakeup_fd = wakeup_recv_.load();
  fd_set readfds;
  fd_set exceptfds;
  FD_ZERO(&readfds);
  FD_ZERO(&exceptfds);
  sock_type_t max_fd = wakeup_fd;
  FD_SET(wakeup_fd, &readfds);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &[id, entry] : entries_) {
      if (entry.armed && !entry.in_flight && Socket::is_valid_fd(entry.fd)) {
        FD_SET(entry.fd, &readfds);
        FD_SET(entry.fd, &exceptfds);
        if (entry.fd > max_fd) {
          max_fd = entry.fd;
        }
      }
    }
  }

  struct timeval tv;
  tv.tv_sec = std::chrono::duration_cast<std::chrono::seconds>(config_.select_timeout).count();
  tv.tv_usec = config_.select_timeout.count() % 1000000;
  int num_ready = ::select(static_cast<int>(max_fd) + 1, &readfds, nullptr, &exceptfds, &tv);
  if (num_ready < 0) {
    // Interrupted or transient error; just loop again.
    return !running_;
  }
  if (num_ready == 0) {
    return !running_; // timeout
  }

  // Drain the wakeup socket if it fired (registration/stop/re-arm signal).
  if (FD_ISSET(wakeup_fd, &readfds)) {
    char buf[64];
    while (::recvfrom(wakeup_fd, buf, sizeof(buf), 0, nullptr, nullptr) > 0) {
    }
  }

  // Collect readable entries, disarming each so it is not dispatched again
  // until its handler completes.
  std::vector<std::pair<Id, QosBand>> ready;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto &[id, entry] : entries_) {
      if (entry.armed && !entry.in_flight && Socket::is_valid_fd(entry.fd) &&
          (FD_ISSET(entry.fd, &readfds) || FD_ISSET(entry.fd, &exceptfds))) {
        entry.armed = false;
        entry.in_flight = true;
        ready.emplace_back(id, entry.band);
      }
    }
  }

  // Dispatch in band order (most urgent first; stable, so same-band sockets
  // keep their registration order). Submitting urgent sockets first also means
  // they win the remaining pool slots when the pool is nearly saturated.
  std::stable_sort(ready.begin(), ready.end(),
                   [](const auto &a, const auto &b) { return a.second < b.second; });

  for (const auto &[id, band] : ready) {
    ++in_flight_count_;
    bool submitted = pool_->submit([this, id]() { dispatch(id); }, band);
    if (!submitted) {
      // Pool is saturated; revert and let the next select() re-report this fd
      // (the data stays buffered in the socket - natural backpressure).
      --in_flight_count_;
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = entries_.find(id);
      if (it != entries_.end()) {
        it->second.in_flight = false;
        // Honor a remove() that arrived while this entry was marked in_flight,
        // rather than blindly re-arming a logically-removed registration.
        if (it->second.remove_requested) {
          entries_.erase(it);
        } else {
          it->second.armed = true;
        }
      }
    }
  }

  return !running_;
}

void SocketReactor::wake() {
  const sock_type_t fd = wakeup_recv_.load();
  if (!Socket::is_valid_fd(fd)) {
    return;
  }
  const char byte = 'x';
  ::sendto(fd, &byte, 1, 0, reinterpret_cast<struct sockaddr *>(&wakeup_addr_),
           sizeof(wakeup_addr_));
}

bool SocketReactor::create_wakeup_socket() {
  const sock_type_t fd = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (!Socket::is_valid_fd(fd)) {
    logger_.error("Could not create wakeup socket");
    return false;
  }
#if !defined(_WIN32)
  // The wakeup fd is FD_SET into the select set every iteration, so it too must
  // be below FD_SETSIZE (see check_fd). It is created before any registrations,
  // so on lwip it takes a low-offset fd - but guard anyway.
  if (fd >= FD_SETSIZE) {
    logger_.error("wakeup socket fd {} is >= FD_SETSIZE ({})", static_cast<int>(fd),
                  static_cast<int>(FD_SETSIZE));
    close_socket(fd);
    return false;
  }
#endif
  struct sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0; // ask the OS for an ephemeral port
  if (::bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    logger_.error("Could not bind wakeup socket");
    close_socket(fd);
    return false;
  }
  // Learn the assigned address/port so wake() can send to ourselves.
  wakeup_addr_ = {};
  socklen_t len = sizeof(wakeup_addr_);
  if (::getsockname(fd, reinterpret_cast<struct sockaddr *>(&wakeup_addr_), &len) < 0) {
    logger_.error("Could not getsockname on wakeup socket");
    close_socket(fd);
    return false;
  }
  // Must be non-blocking, otherwise the loop thread's drain-recvfrom() loop can
  // block forever and stall the reactor.
  if (!set_nonblocking(fd)) {
    logger_.error("Could not set wakeup socket non-blocking");
    close_socket(fd);
    return false;
  }
  wakeup_recv_ = fd; // publish only once fully set up
  return true;
}

void SocketReactor::close_wakeup_socket() {
  const sock_type_t fd = wakeup_recv_.exchange(static_cast<sock_type_t>(-1));
  if (Socket::is_valid_fd(fd)) {
    close_socket(fd);
  }
}

bool SocketReactor::set_nonblocking(sock_type_t fd) {
#if defined(_WIN32)
  u_long mode = 1;
  return ioctlsocket(fd, FIONBIO, &mode) == 0;
#else
  int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    return false;
  }
  return ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
#endif
}

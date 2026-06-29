#include "socket.hpp"

using namespace espp;

void Socket::Info::init_ipv4(const std::string &addr, size_t prt) {
  address = addr;
  port = prt;
  auto server_address = ipv4_ptr();
  server_address->sin_family = AF_INET;
  server_address->sin_addr.s_addr = inet_addr(address.c_str());
  server_address->sin_port = htons(port);
}

struct sockaddr_in *Socket::Info::ipv4_ptr() {
  return reinterpret_cast<struct sockaddr_in *>(&raw);
}

struct sockaddr_in6 *Socket::Info::ipv6_ptr() {
  return reinterpret_cast<struct sockaddr_in6 *>(&raw);
}

void Socket::Info::update() {
  if (raw.ss_family == PF_INET) {
    const auto *ipv4 = reinterpret_cast<const struct sockaddr_in *>(&raw);
    address = inet_ntoa(ipv4->sin_addr);
    port = ipv4->sin_port;
  } else if (raw.ss_family == PF_INET6) {
    const auto *ipv6 = reinterpret_cast<const struct sockaddr_in6 *>(&raw);
#if defined(ESP_PLATFORM)
    address = inet_ntoa(ipv6->sin6_addr);
#else
    char str[INET6_ADDRSTRLEN];
    inet_ntop(AF_INET6, &(ipv6->sin6_addr), str, INET6_ADDRSTRLEN);
    address = str;
#endif
    port = ipv6->sin6_port;
  }
}

void Socket::Info::from_sockaddr(const struct sockaddr_storage &source_address) {
  memcpy(&raw, &source_address, sizeof(source_address));
  update();
}

void Socket::Info::from_sockaddr(const struct sockaddr_in &source_address) {
  memcpy(&raw, &source_address, sizeof(source_address));
  address = inet_ntoa(source_address.sin_addr);
  port = source_address.sin_port;
}

void Socket::Info::from_sockaddr(const struct sockaddr_in6 &source_address) {
#if defined(ESP_PLATFORM)
  address = inet_ntoa(source_address.sin6_addr);
#else
  char str[INET6_ADDRSTRLEN];
  inet_ntop(AF_INET6, &(source_address.sin6_addr), str, INET6_ADDRSTRLEN);
  address = str;
#endif
  port = source_address.sin6_port;
  memcpy(&raw, &source_address, sizeof(source_address));
}

[[maybe_unused]] static bool _socket_initialized = false;
Socket::Socket(sock_type_t socket_fd, const Logger::Config &logger_config)
    : BaseComponent(logger_config) {
#ifdef _MSC_VER
  if (!_socket_initialized) {
    logger_.debug("Initializing Winsock");
    WSADATA wsa_data;
    int err = WSAStartup(MAKEWORD(1, 1), &wsa_data);
    if (err != 0) {
      logger_.error("WSAStartup failed: {}", error_string(err));
    }
    _socket_initialized = true;
  }
#endif
  socket_ = socket_fd;
}

Socket::Socket(Type type, const Logger::Config &logger_config)
    : BaseComponent(logger_config) {
#ifdef _MSC_VER
  if (!_socket_initialized) {
    logger_.debug("Initializing Winsock");
    WSADATA wsa_data;
    int err = WSAStartup(MAKEWORD(1, 1), &wsa_data);
    if (err != 0) {
      logger_.error("WSAStartup failed: {}", error_string(err));
    }
    _socket_initialized = true;
  }
#endif
  init(type);
}

Socket::~Socket() { cleanup(); }

bool Socket::is_valid() const {
#ifdef _MSC_VER
  return socket_ != INVALID_SOCKET;
#else
  return socket_ >= 0;
#endif
}

bool Socket::is_valid_fd(sock_type_t socket_fd) {
#ifdef _MSC_VER
  return socket_fd != INVALID_SOCKET;
#else
  return socket_fd >= 0;
#endif
}

std::optional<Socket::Info> Socket::get_ipv4_info() {
  struct sockaddr_storage addr;
  socklen_t addr_len = sizeof(addr);
  if (getsockname(socket_, reinterpret_cast<struct sockaddr *>(&addr), &addr_len) < 0) {
    logger_.error("getsockname() failed: {}", error_string());
    return {};
  }
  Info info;
  info.from_sockaddr(addr);
  return info;
}

bool Socket::set_receive_timeout(const std::chrono::duration<float> &timeout) {
  float seconds = timeout.count();
  if (seconds <= 0) {
    return true;
  }
  float intpart;
  float fractpart = modff(seconds, &intpart);
  const auto response_timeout_s = static_cast<time_t>(intpart);
  const auto response_timeout_us = static_cast<time_t>(fractpart * 1E6f);
  //// Alternatively we could do this:
  // int microseconds =
  // (int)(std::chrono::duration_cast<std::chrono::microseconds>(timeout).count()) % (int)1E6;
  // const time_t response_timeout_s = floor(seconds);
  // const time_t response_timeout_us = microseconds;

  struct timeval tv;
  tv.tv_sec = response_timeout_s;
  tv.tv_usec = response_timeout_us;
  int err =
      setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char *>(&tv), sizeof(tv));
  if (err < 0) {
    return false;
  }
  return true;
}

bool Socket::enable_reuse() {
#if !CONFIG_LWIP_SO_REUSE && defined(ESP_PLATFORM)
  fmt::print(fg(fmt::color::red), "CONFIG_LWIP_SO_REUSE not defined!\n");
  return false;
#else // CONFIG_LWIP_SO_REUSE || !defined(ESP_PLATFORM)
  int err = 0;
  int enabled = 1;
  err = setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&enabled),
                   sizeof(enabled));
  if (err < 0) {
    fmt::print(fg(fmt::color::red), "Couldn't set SO_REUSEADDR: {}\n", error_string());
    return false;
  }
#if !defined(ESP_PLATFORM)
#ifdef _MSC_VER
  // NOTE: according to stackoverflow, we have to set broadcast instead of reuseport
  err = setsockopt(socket_, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char *>(&enabled),
                   sizeof(enabled));
  if (err < 0) {
    fmt::print(fg(fmt::color::red), "Couldn't set SO_BROADCAST: {}\n", error_string());
    return false;
  }
#else
  err = setsockopt(socket_, SOL_SOCKET, SO_REUSEPORT, reinterpret_cast<const char *>(&enabled),
                   sizeof(enabled));
  if (err < 0) {
    fmt::print(fg(fmt::color::red), "Couldn't set SO_REUSEPORT: {}\n", error_string());
    return false;
  }
#endif // _MSC_VER
#endif // !defined(ESP_PLATFORM)
  return true;
#endif // !CONFIG_LWIP_SO_REUSE && defined(ESP_PLATFORM)
}

// Resolve a dotted-decimal IPv4 interface address to an in_addr. An empty string or "0.0.0.0"
// resolves to INADDR_ANY (let the OS pick the default multicast interface).
static bool resolve_interface_address(const std::string &interface_address, struct in_addr &out) {
  if (interface_address.empty() || interface_address == "0.0.0.0") {
#if defined(ESP_PLATFORM)
    out.s_addr = IPADDR_ANY;
#else
    out.s_addr = htonl(INADDR_ANY);
#endif
    return true;
  }
#ifdef _MSC_VER
  return inet_pton(AF_INET, interface_address.c_str(), &out) == 1;
#else
  return inet_aton(interface_address.c_str(), &out) == 1;
#endif
}

bool Socket::make_multicast(uint8_t time_to_live, uint8_t loopback_enabled,
                            const std::string &interface_address) {
  int err = 0;
  // Assign multicast TTL - separate from normal interface TTL
  err = setsockopt(socket_, IPPROTO_IP, IP_MULTICAST_TTL,
                   reinterpret_cast<const char *>(&time_to_live), sizeof(uint8_t));
  if (err < 0) {
    fmt::print(fg(fmt::color::red), "Couldn't set IP_MULTICAST_TTL: {}\n", error_string());
    return false;
  }
  // select whether multicast traffic should be received by this device, too
  err = setsockopt(socket_, IPPROTO_IP, IP_MULTICAST_LOOP,
                   reinterpret_cast<const char *>(&loopback_enabled), sizeof(uint8_t));
  if (err < 0) {
    fmt::print(fg(fmt::color::red), "Couldn't set IP_MULTICAST_LOOP: {}\n", error_string());
    return false;
  }
  // Pin the outgoing multicast interface (IP_MULTICAST_IF) when an interface address is given.
  // Binding the socket does NOT control which NIC outgoing multicast leaves on; without this the OS
  // uses its default multicast route (e.g. Wi-Fi on a multi-homed macOS host).
  if (!interface_address.empty() && interface_address != "0.0.0.0") {
    struct in_addr iaddr {};
    if (!resolve_interface_address(interface_address, iaddr)) {
      fmt::print(fg(fmt::color::red), "Invalid multicast interface address: {}\n",
                 interface_address);
      return false;
    }
    err = setsockopt(socket_, IPPROTO_IP, IP_MULTICAST_IF, reinterpret_cast<const char *>(&iaddr),
                     sizeof(struct in_addr));
    if (err < 0) {
      fmt::print(fg(fmt::color::red), "Couldn't set IP_MULTICAST_IF to {}: {}\n", interface_address,
                 error_string());
      return false;
    }
  }
  return true;
}

bool Socket::add_multicast_group(const std::string &multicast_group,
                                 const std::string &interface_address) {
  struct ip_mreq imreq;
  int err = 0;

  // Resolve the local interface on which to join the group (and use for egress). Empty/"0.0.0.0"
  // resolves to INADDR_ANY (OS default interface). Specifying it makes a multi-homed host join the
  // group on the desired NIC instead of whichever interface the OS would pick by default.
  struct in_addr iface_addr {};
  if (!resolve_interface_address(interface_address, iface_addr)) {
    fmt::print(fg(fmt::color::red), "Invalid multicast interface address: {}\n", interface_address);
    return false;
  }
  imreq.imr_interface = iface_addr;

  // Configure multicast address to listen to
#if defined(ESP_PLATFORM)
  err = inet_aton(multicast_group.c_str(), &imreq.imr_multiaddr.s_addr);
#else
#ifdef _MSC_VER
  err = inet_pton(AF_INET, multicast_group.c_str(), &imreq.imr_multiaddr);
#else
  err = inet_aton(multicast_group.c_str(), &imreq.imr_multiaddr);
#endif // _MSC_VER
#endif // defined(ESP_PLATFORM)

  if (err != 1 || !IN_MULTICAST(ntohl(imreq.imr_multiaddr.s_addr))) {
    // it's not actually a multicast address, so return false?
    fmt::print(fg(fmt::color::red), "Not a valid multicast address ({})\n", multicast_group);
    return false;
  }

  // Assign the IPv4 multicast egress interface (IP_MULTICAST_IF) to the same interface we join on.
  err = setsockopt(socket_, IPPROTO_IP, IP_MULTICAST_IF,
                   reinterpret_cast<const char *>(&iface_addr), sizeof(struct in_addr));
  if (err < 0) {
    fmt::print(fg(fmt::color::red), "Couldn't set IP_MULTICAST_IF: {}\n", error_string());
    return false;
  }

  err = setsockopt(socket_, IPPROTO_IP, IP_ADD_MEMBERSHIP, reinterpret_cast<const char *>(&imreq),
                   sizeof(struct ip_mreq));
  if (err < 0) {
    fmt::print(fg(fmt::color::red), "Couldn't set IP_ADD_MEMBERSHIP: {}\n", error_string());
    return false;
  }

  return true;
}

int Socket::select(const std::chrono::microseconds &timeout) {
  fd_set readfds;
  fd_set exceptfds;
  FD_ZERO(&readfds);
  FD_ZERO(&exceptfds);
  FD_SET(socket_, &readfds);
  FD_SET(socket_, &exceptfds);
  int nfds = socket_ + 1;
  // convert timeout to timeval
  struct timeval tv;
  tv.tv_sec = std::chrono::duration_cast<std::chrono::seconds>(timeout).count();
  tv.tv_usec = std::chrono::duration_cast<std::chrono::microseconds>(timeout).count() % 1000000;
  int retval = ::select(nfds, &readfds, nullptr, &exceptfds, &tv);
  if (retval < 0) {
    logger_.error("select failed: {}", error_string());
    return -1;
  }
  if (retval == 0) {
    logger_.debug("select timed out");
    return 0;
  }
  if (FD_ISSET(socket_, &readfds)) {
    logger_.debug("select read");
  }
  if (FD_ISSET(socket_, &exceptfds)) {
    logger_.debug("select except");
  }
  return retval;
}

bool Socket::init(Socket::Type type) {
  // actually make the socket
  socket_ = socket(address_family_, static_cast<int>(type), ip_protocol_);
  if (!is_valid()) {
    logger_.error("Cannot create socket: {}", error_string());
    return false;
  }
  if (!enable_reuse()) {
    logger_.error("Cannot enable reuse: {}", error_string());
    return false;
  }
  return true;
}

std::string Socket::error_string() const {
#ifdef _MSC_VER
  int err = WSAGetLastError();
  return error_string(err);
#else
  return error_string(errno);
#endif
}

std::string Socket::error_string(int err) const {
#ifdef _MSC_VER
  if (err == WSAEWOULDBLOCK) {
    return "WSAEWOULDBLOCK";
  } else if (err == WSAECONNRESET) {
    return "WSAECONNRESET";
  } else if (err == WSAECONNABORTED) {
    return "WSAECONNABORTED";
  } else if (err == WSAECONNREFUSED) {
    return "WSAECONNREFUSED";
  } else if (err == WSAETIMEDOUT) {
    return "WSAETIMEDOUT";
  } else if (err == WSAEINTR) {
    return "WSAEINTR";
  } else if (err == WSAENOTSOCK) {
    return "WSAENOTSOCK";
  } else if (err == WSANOTINITIALISED) {
    return "WSANOTINITIALISED";
  } else {
    return fmt::format("Unknown error: {0} ({0:#x})", static_cast<int>(err));
  }
#else
  return fmt::format("{} - '{}'", err, strerror(err));
#endif
}

void Socket::cleanup() {
  if (is_valid()) {
    auto socket_fd = socket_;
#ifdef _MSC_VER
    socket_ = INVALID_SOCKET;
    int status = shutdown(socket_fd, SD_BOTH);
    if (status != 0) {
      int err = WSAGetLastError();
      if (err != WSAENOTCONN && err != WSAENOTSOCK) {
        logger_.debug("shutdown failed before close: {}", error_string(err));
      }
    }
    if (closesocket(socket_fd) != 0) {
      logger_.debug("close failed: {}", error_string(WSAGetLastError()));
    }
#else
    socket_ = -1;
    int status = shutdown(socket_fd, SHUT_RDWR);
    if (status != 0) {
      int err = errno;
      if (err != ENOTCONN) {
        logger_.debug("shutdown failed before close: {}", error_string(err));
      }
    }
    if (close(socket_fd) != 0) {
      logger_.debug("close failed: {}", error_string(errno));
    }
#endif

    logger_.info("Closed socket");
  }
}

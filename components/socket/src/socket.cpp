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
  return (struct sockaddr_in *)&raw;
}

struct sockaddr_in6 *Socket::Info::ipv6_ptr() {
  return (struct sockaddr_in6 *)&raw;
}

void Socket::Info::update() {
  if (raw.ss_family == PF_INET) {
    address = inet_ntoa(((struct sockaddr_in *)&raw)->sin_addr);
    port = ((struct sockaddr_in *)&raw)->sin_port;
  } else if (raw.ss_family == PF_INET6) {
#if defined(ESP_PLATFORM)
    address = inet_ntoa(((struct sockaddr_in6 *)&raw)->sin6_addr);
#else
    char str[INET6_ADDRSTRLEN];
    inet_ntop(AF_INET6, &(((struct sockaddr_in6 *)&raw)->sin6_addr), str, INET6_ADDRSTRLEN);
    address = str;
#endif
    port = ((struct sockaddr_in6 *)&raw)->sin6_port;
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

Socket::Socket(int socket_fd, const Logger::Config &logger_config)
    : BaseComponent(logger_config) {
  socket_ = socket_fd;
}

Socket::Socket(Type type, const Logger::Config &logger_config)
    : BaseComponent(logger_config) {
  init(type);
}

Socket::~Socket() { cleanup(); }

bool Socket::is_valid() const { return socket_ >= 0; }

bool Socket::is_valid(int socket_fd) { return socket_fd >= 0; }

std::optional<Socket::Info> Socket::get_ipv4_info() {
  struct sockaddr_storage addr;
  socklen_t addr_len = sizeof(addr);
  if (getsockname(socket_, (struct sockaddr *)&addr, &addr_len) < 0) {
    logger_.error("getsockname() failed: {} - {}", errno, strerror(errno));
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
  float fractpart = modf(seconds, &intpart);
  const time_t response_timeout_s = (int)intpart;
  const time_t response_timeout_us = (int)(fractpart * 1E6);
  //// Alternatively we could do this:
  // int microseconds =
  // (int)(std::chrono::duration_cast<std::chrono::microseconds>(timeout).count()) % (int)1E6;
  // const time_t response_timeout_s = floor(seconds);
  // const time_t response_timeout_us = microseconds;

  struct timeval tv;
  tv.tv_sec = response_timeout_s;
  tv.tv_usec = response_timeout_us;
  int err = setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));
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
  err = setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));
  if (err < 0) {
    fmt::print(fg(fmt::color::red), "Couldn't set SO_REUSEADDR\n");
    return false;
  }
#if !defined(ESP_PLATFORM)
  err = setsockopt(socket_, SOL_SOCKET, SO_REUSEPORT, &enabled, sizeof(enabled));
  if (err < 0) {
    fmt::print(fg(fmt::color::red), "Couldn't set SO_REUSEPORT\n");
    return false;
  }
#endif // !defined(ESP_PLATFORM)
  return true;
#endif // !CONFIG_LWIP_SO_REUSE && defined(ESP_PLATFORM)
}

bool Socket::make_multicast(uint8_t time_to_live, uint8_t loopback_enabled) {
  int err = 0;
  // Assign multicast TTL - separate from normal interface TTL
  err = setsockopt(socket_, IPPROTO_IP, IP_MULTICAST_TTL, &time_to_live, sizeof(uint8_t));
  if (err < 0) {
    fmt::print(fg(fmt::color::red), "Couldn't set IP_MULTICAST_TTL\n");
    return false;
  }
  // select whether multicast traffic should be received by this device, too
  err = setsockopt(socket_, IPPROTO_IP, IP_MULTICAST_LOOP, &loopback_enabled, sizeof(uint8_t));
  if (err < 0) {
    fmt::print(fg(fmt::color::red), "Couldn't set IP_MULTICAST_LOOP\n");
    return false;
  }
  return true;
}

bool Socket::add_multicast_group(const std::string &multicast_group) {
  struct ip_mreq imreq;
  int err = 0;

  // Configure source interface
#if defined(ESP_PLATFORM)
  imreq.imr_interface.s_addr = IPADDR_ANY;
  // Configure multicast address to listen to
  err = inet_aton(multicast_group.c_str(), &imreq.imr_multiaddr.s_addr);
#else
  imreq.imr_interface.s_addr = htonl(INADDR_ANY);
  // Configure multicast address to listen to
  err = inet_aton(multicast_group.c_str(), &imreq.imr_multiaddr);
#endif

  if (err != 1 || !IN_MULTICAST(ntohl(imreq.imr_multiaddr.s_addr))) {
    // it's not actually a multicast address, so return false?
    fmt::print(fg(fmt::color::red), "Not a valid multicast address ({})\n", multicast_group);
    return false;
  }

  // Assign the IPv4 multicast source interface, via its IP
  // (only necessary if this socket is IPV4 only)
  struct in_addr iaddr;
  err = setsockopt(socket_, IPPROTO_IP, IP_MULTICAST_IF, &iaddr, sizeof(struct in_addr));
  if (err < 0) {
    fmt::print(fg(fmt::color::red), "Couldn't set IP_MULTICAST_IF: {} - '{}'\n", errno,
               strerror(errno));
    return false;
  }

  err = setsockopt(socket_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &imreq, sizeof(struct ip_mreq));
  if (err < 0) {
    fmt::print(fg(fmt::color::red), "Couldn't set IP_ADD_MEMBERSHIP: {} - '{}'\n", errno,
               strerror(errno));
    return false;
  }

  return true;
}

int Socket::select(const std::chrono::microseconds &timeout) {
  fd_set readfds;
  fd_set writefds;
  fd_set exceptfds;
  FD_ZERO(&readfds);
  FD_ZERO(&writefds);
  FD_ZERO(&exceptfds);
  FD_SET(socket_, &readfds);
  FD_SET(socket_, &writefds);
  FD_SET(socket_, &exceptfds);
  int nfds = socket_ + 1;
  // convert timeout to timeval
  struct timeval tv;
  tv.tv_sec = std::chrono::duration_cast<std::chrono::seconds>(timeout).count();
  tv.tv_usec = std::chrono::duration_cast<std::chrono::microseconds>(timeout).count() % 1000000;
  int retval = ::select(nfds, &readfds, &writefds, &exceptfds, &tv);
  if (retval < 0) {
    logger_.error("select failed: {} - '{}'", errno, strerror(errno));
    return -1;
  }
  if (retval == 0) {
    logger_.warn("select timed out");
    return 0;
  }
  if (FD_ISSET(socket_, &readfds)) {
    logger_.debug("select read");
  }
  if (FD_ISSET(socket_, &writefds)) {
    logger_.debug("select write");
  }
  if (FD_ISSET(socket_, &exceptfds)) {
    logger_.debug("select except");
  }
  return retval;
}

bool Socket::init(Socket::Type type) {
  // actually make the socket
  socket_ = socket(address_family_, (int)type, ip_protocol_);
  if (!is_valid()) {
    logger_.error("Cannot create socket: {} - '{}'", errno, strerror(errno));
    return false;
  }
  if (!enable_reuse()) {
    logger_.error("Cannot enable reuse: {} - '{}'", errno, strerror(errno));
    return false;
  }
  return true;
}

void Socket::cleanup() {
  if (is_valid()) {
    shutdown(socket_, 0);
    close(socket_);
    socket_ = -1;
    logger_.info("Closed socket");
  }
}
#include "tcp_socket.hpp"

using namespace espp;

TcpSocket::TcpSocket(const TcpSocket::Config &config)
    : Socket(Type::STREAM, Logger::Config{.tag = "TcpSocket", .level = config.log_level}) {
  set_keepalive();
}

TcpSocket::~TcpSocket() {
  // we have to explicitly call cleanup here so that the server accept /
  // read will return and the task can stop.
  cleanup();
}

void TcpSocket::reinit() {
  if (is_valid()) {
    // cleanup our socket
    cleanup();
  }
  init(Type::STREAM);
}

void TcpSocket::close() { ::close(socket_); }

bool TcpSocket::is_connected() const { return connected_; }

bool TcpSocket::connect(const TcpSocket::ConnectConfig &connect_config) {
  if (!is_valid()) {
    logger_.error("Socket invalid, cannot connect");
    return false;
  }
  Socket::Info server_info;
  server_info.init_ipv4(connect_config.ip_address, connect_config.port);
  auto server_address = server_info.ipv4_ptr();
  logger_.info("Client connecting to {}", server_info);
  // connect
  int error = ::connect(socket_, (struct sockaddr *)server_address, sizeof(*server_address));
  if (error != 0) {
    logger_.error("Could not connect to the server: {}", error_string());
    return false;
  }
  connected_ = true;
  return true;
}

const Socket::Info &TcpSocket::get_remote_info() const { return remote_info_; }

bool TcpSocket::transmit(const std::vector<uint8_t> &data, const TransmitConfig &transmit_config) {
  return transmit(std::string_view{(const char *)data.data(), data.size()}, transmit_config);
}

bool TcpSocket::transmit(const std::vector<char> &data, const TransmitConfig &transmit_config) {
  return transmit(std::string_view{(const char *)data.data(), data.size()}, transmit_config);
}

bool TcpSocket::transmit(std::string_view data, const TransmitConfig &transmit_config) {
  if (!is_valid()) {
    logger_.error("Socket invalid, cannot send");
    return false;
  }
  // set the receive timeout
  if (!set_receive_timeout(transmit_config.response_timeout)) {
    logger_.error("Could not set receive timeout to {}: {}",
                  transmit_config.response_timeout.count(), error_string());
    return false;
  }
  // write
  logger_.info("Client sending {} bytes", data.size());
  int num_bytes_sent = write(socket_, data.data(), data.size());
  if (num_bytes_sent < 0) {
    logger_.error("Error occurred during sending: {}", error_string());
    // update our connection state here since remote end was likely closed...
    connected_ = false;
    return false;
  }
  logger_.debug("Client sent {} bytes", num_bytes_sent);
  // we don't need to wait for a response and the socket is good;
  if (!transmit_config.wait_for_response) {
    return true;
  }
  if (transmit_config.response_size == 0) {
    logger_.warn("Response requested, but response_size=0, not waiting for response!");
    // NOTE: we did send successfully, so we return true and warn about
    // misconfiguration
    return true;
  }
  std::vector<uint8_t> received_data;
  logger_.info("Client waiting for response");
  // read
  if (!receive(received_data, transmit_config.response_size)) {
    logger_.warn("Client could not get response, remote socket might have closed!");
    // TODO: should we upate our connected_ variable here?
    return false;
  }
  logger_.info("Client got {} bytes of response", received_data.size());
  if (transmit_config.on_response_callback) {
    logger_.debug("Client calling response callback");
    transmit_config.on_response_callback(received_data);
  }
  return true;
}

bool TcpSocket::receive(std::vector<uint8_t> &data, size_t max_num_bytes) {
  // make some space for received data - put it on the heap so that our
  // stack usage doesn't change depending on max_num_bytes
  std::unique_ptr<uint8_t[]> receive_buffer(new uint8_t[max_num_bytes]());
  int num_bytes_received = receive(receive_buffer.get(), max_num_bytes);
  if (num_bytes_received > 0) {
    logger_.info("Received {} bytes", num_bytes_received);
    uint8_t *data_ptr = (uint8_t *)receive_buffer.get();
    data.assign(data_ptr, data_ptr + num_bytes_received);
    return true;
  }
  return false;
}

size_t TcpSocket::receive(uint8_t *data, size_t max_num_bytes) {
  if (!is_valid()) {
    logger_.error("Socket invalid, cannot receive.");
    return 0;
  }
  if (!is_connected()) {
    logger_.error("Socket not connected, cannot receive.");
    return 0;
  }
  logger_.info("Receiving up to {} bytes", max_num_bytes);
  // now actually read data from the socket
  int num_bytes_received = ::recv(socket_, (char *)data, max_num_bytes, 0);
  // if we didn't receive anything return false and don't do anything else
  if (num_bytes_received < 0) {
    // if we got an error, log it and return 0
    logger_.debug("Receive failed: {}", error_string());
    return 0;
  } else if (num_bytes_received == 0) {
    logger_.warn("Remote socket closed!");
    // update our connection state here since remote end was closed...
    connected_ = false;
  } else {
    logger_.debug("Received {} bytes", num_bytes_received);
  }
  return num_bytes_received;
}

bool TcpSocket::bind(int port) {
  if (!is_valid()) {
    logger_.error("Socket invalid, cannot bind.");
    return false;
  }
  struct sockaddr_in server_addr;
  // configure the server socket accordingly - assume IPV4 and bind to the
  // any address "0.0.0.0"
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  server_addr.sin_family = address_family_;
  server_addr.sin_port = htons(port);
  auto err = ::bind(socket_, (struct sockaddr *)&server_addr, sizeof(server_addr));
  if (err < 0) {
    logger_.error("Unable to bind: {}", error_string());
    return false;
  }
  return true;
}

bool TcpSocket::listen(int max_pending_connections) {
  if (!is_valid()) {
    logger_.error("Socket invalid, cannot listen.");
    return false;
  }
  auto err = ::listen(socket_, max_pending_connections);
  if (err < 0) {
    logger_.error("Unable to listen: {}", error_string());
    return false;
  }
  return true;
}

std::unique_ptr<TcpSocket> TcpSocket::accept() {
  if (!is_valid()) {
    logger_.error("Socket invalid, cannot accept incoming connections.");
    return nullptr;
  }
  Socket::Info connected_client_info;
  auto sender_address = connected_client_info.ipv4_ptr();
  socklen_t socklen = sizeof(*sender_address);
  // accept connection
  auto accepted_socket = ::accept(socket_, (struct sockaddr *)sender_address, &socklen);
  if (accepted_socket < 0) {
    logger_.info("Could not accept connection: {}", error_string());
    return nullptr;
  }
  connected_client_info.update();
  logger_.info("Server accepted connection with {}", connected_client_info);
  // NOTE: have to use new here because we can't use make_unique with a
  //       protected or private constructor
  return std::unique_ptr<TcpSocket>(new TcpSocket(accepted_socket, connected_client_info));
}

TcpSocket::TcpSocket(sock_type_t socket_fd, const Socket::Info &remote_info)
    : Socket(socket_fd, Logger::Config{.tag = "TcpSocket", .level = Logger::Verbosity::WARN})
    , remote_info_(remote_info) {
  connected_ = true;
  set_keepalive();
}

bool TcpSocket::set_keepalive(const std::chrono::seconds &idle_time,
                              const std::chrono::seconds &interval, int max_probes) {
  if (!is_valid()) {
    logger_.error("Socket invalid, cannot set keepalive.");
    return false;
  }
  int optval = 1;
  // enable keepalive
  auto err = setsockopt(socket_, SOL_SOCKET, SO_KEEPALIVE, (const char *)&optval, sizeof(optval));
  if (err < 0) {
    logger_.error("Unable to set keepalive: {}", error_string());
    return false;
  }

#if defined(__APPLE__)
  // TODO: figure out how to set keepidle on macos
#else
  // set the idle time
  optval = idle_time.count();
  err = setsockopt(socket_, IPPROTO_TCP, TCP_KEEPIDLE, (const char *)&optval, sizeof(optval));
  if (err < 0) {
    logger_.error("Unable to set keepalive idle time: {}", error_string());
    return false;
  }
#endif

  // set the interval
  optval = interval.count();
  err = setsockopt(socket_, IPPROTO_TCP, TCP_KEEPINTVL, (const char *)&optval, sizeof(optval));
  if (err < 0) {
    logger_.error("Unable to set keepalive interval: {}", error_string());
    return false;
  }
  // set the max probes
  optval = max_probes;
  err = setsockopt(socket_, IPPROTO_TCP, TCP_KEEPCNT, (const char *)&optval, sizeof(optval));
  if (err < 0) {
    logger_.error("Unable to set keepalive max probes: {}", error_string());
    return false;
  }
  return true;
}

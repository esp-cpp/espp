#include "udp_socket.hpp"

using namespace espp;

UdpSocket::UdpSocket(const UdpSocket::Config &config)
    : Socket(Type::DGRAM, Logger::Config{.tag = "UdpSocket", .level = config.log_level}) {}

UdpSocket::~UdpSocket() {
  // we have to explicitly call cleanup here so that the server recvfrom
  // will return and the task can stop.
  cleanup();
}

bool UdpSocket::send(const std::vector<uint8_t> &data, const UdpSocket::SendConfig &send_config) {
  return send(std::string_view{(const char *)data.data(), data.size()}, send_config);
}

bool UdpSocket::send(std::string_view data, const UdpSocket::SendConfig &send_config) {
  if (!is_valid()) {
    logger_.error("Socket invalid, cannot send");
    return false;
  }
  if (send_config.is_multicast_endpoint) {
    // configure it for multicast
    if (!make_multicast()) {
      logger_.error("Cannot make multicast: {}", error_string());
      return false;
    }
  }
  if (send_config.wait_for_response) {
    // set the receive timeout
    if (!set_receive_timeout(send_config.response_timeout)) {
      logger_.error("Could not set receive timeout to {}: {}", send_config.response_timeout.count(),
                    error_string());
      return false;
    }
  }
  // sendto
  Socket::Info server_info;
  server_info.init_ipv4(send_config.ip_address, send_config.port);
  auto server_address = server_info.ipv4_ptr();
  logger_.info("Client sending {} bytes to {}:{}", data.size(), send_config.ip_address,
               send_config.port);
  int num_bytes_sent = sendto(socket_, data.data(), data.size(), 0,
                              (struct sockaddr *)server_address, sizeof(*server_address));
  if (num_bytes_sent < 0) {
    logger_.error("Error occurred during sending: {}", error_string());
    return false;
  }
  logger_.debug("Client sent {} bytes", num_bytes_sent);
  // we don't need to wait for a response and the socket is good;
  if (!send_config.wait_for_response) {
    return true;
  }
  if (send_config.response_size == 0) {
    logger_.warn("Response requested, but response_size=0, not waiting for response!");
    // NOTE: we did send successfully, so we return true and warn about
    // misconfiguration
    return true;
  }
  std::vector<uint8_t> received_data;
  logger_.info("Client waiting for response");
  if (!receive(send_config.response_size, received_data, server_info)) {
    logger_.warn("Client could not get response");
    return false;
  }
  logger_.info("Client got {} bytes of response", received_data.size());
  if (send_config.on_response_callback) {
    logger_.debug("Client calling response callback");
    send_config.on_response_callback(received_data);
  }
  return true;
}

bool UdpSocket::receive(size_t max_num_bytes, std::vector<uint8_t> &data,
                        Socket::Info &remote_info) {
  if (!is_valid()) {
    logger_.error("Socket invalid, cannot receive.");
    return false;
  }
  // recvfrom
  auto remote_address = remote_info.ipv4_ptr();
  socklen_t socklen = sizeof(*remote_address);
  // put it on the heap so that our stack usage doesn't change depending on
  // max_num_bytes
  std::unique_ptr<uint8_t[]> receive_buffer(new uint8_t[max_num_bytes]());
  // now actually receive
  logger_.info("Receiving up to {} bytes", max_num_bytes);
  int num_bytes_received = recvfrom(socket_, (char *)receive_buffer.get(), max_num_bytes, 0,
                                    (struct sockaddr *)remote_address, &socklen);
  // if we didn't receive anything return false and don't do anything else
  if (num_bytes_received < 0) {
    logger_.info("Receive failed: {}", error_string());
    return false;
  }
  // we received data, so call the callback function if one was provided.
  uint8_t *data_ptr = (uint8_t *)receive_buffer.get();
  data.assign(data_ptr, data_ptr + num_bytes_received);
  remote_info.update();
  logger_.debug("Received {} bytes from {}", num_bytes_received, remote_info);
  return true;
}

bool UdpSocket::start_receiving(Task::BaseConfig &task_config,
                                const UdpSocket::ReceiveConfig &receive_config) {
  if (task_ && task_->is_started()) {
    logger_.error("Server is alrady receiving");
    return false;
  }
  if (!is_valid()) {
    logger_.error("Socket invalid, cannot start receiving.");
    return false;
  }
  server_receive_callback_ = receive_config.on_receive_callback;
  // bind
  struct sockaddr_in server_addr;
  // configure the server socket accordingly - assume IPV4 and bind to the
  // any address "0.0.0.0"
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  server_addr.sin_family = address_family_;
  server_addr.sin_port = htons(receive_config.port);
  int err = bind(socket_, (struct sockaddr *)&server_addr, sizeof(server_addr));
  if (err < 0) {
    logger_.error("Unable to bind: {}", error_string());
    return false;
  }
  if (receive_config.is_multicast_endpoint) {
    // enable multicast
    if (!make_multicast()) {
      logger_.error("Unable to make bound socket multicast: {}", error_string());
      return false;
    }
    // add multicast group
    if (!add_multicast_group(receive_config.multicast_group)) {
      logger_.error("Unable to add multicast group to bound socket: {}", error_string());
      return false;
    }
  }
  // set the callback function
  using namespace std::placeholders;
  // start the thread
  task_ = Task::make_unique({
      .callback =
          std::bind(&UdpSocket::server_task_function, this, receive_config.buffer_size, _1, _2, _3),
      .task_config = task_config,
  });
  task_->start();
  return true;
}

bool UdpSocket::server_task_function(size_t buffer_size, std::mutex &m, std::condition_variable &cv,
                                     bool &task_notified) {
  // receive data
  std::vector<uint8_t> received_data;
  Socket::Info sender_info;
  if (!receive(buffer_size, received_data, sender_info)) {
    // if we failed to receive, then likely we should delay a little bit
    using namespace std::chrono_literals;
    std::unique_lock<std::mutex> lk(m);
    auto stop_requested = cv.wait_for(lk, 1ms, [&task_notified] { return task_notified; });
    task_notified = false;
    if (stop_requested) {
      return true;
    }
    return false;
  }
  if (!server_receive_callback_) {
    logger_.error("Server receive callback is invalid");
    return false;
  }
  // callback
  auto maybe_response = server_receive_callback_(received_data, sender_info);
  // send if callback returned data
  if (!maybe_response.has_value()) {
    return false;
  }
  auto response = maybe_response.value();
  // sendto
  logger_.info("Server responding to {} with message of length {}", sender_info, response.size());
  auto sender_address = sender_info.ipv4_ptr();
  int num_bytes_sent = sendto(socket_, (const char *)response.data(), response.size(), 0,
                              (struct sockaddr *)sender_address, sizeof(*sender_address));
  if (num_bytes_sent < 0) {
    logger_.error("Error occurred responding: {}", error_string());
  }
  logger_.info("Server responded with {} bytes", num_bytes_sent);
  // don't want to stop the task
  return false;
}

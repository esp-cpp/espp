#include "udp_socket.hpp"

#include <thread>

using namespace espp;

namespace {
#ifdef _MSC_VER
int last_socket_error() { return WSAGetLastError(); }
#else
int last_socket_error() { return errno; }
#endif

bool is_transient_send_error(int err) {
#ifdef _MSC_VER
  return err == WSAEWOULDBLOCK || err == WSAENOBUFS;
#else
  return err == EAGAIN || err == EWOULDBLOCK || err == ENOBUFS || err == ENOMEM;
#endif
}

constexpr int transient_send_retry_count = 5;
constexpr auto transient_send_retry_delay = std::chrono::milliseconds(2);

int transient_send_retry_limit(int err) {
#if defined(ESP_PLATFORM)
  if (err == ENOBUFS || err == ENOMEM) {
    return 0;
  }
#endif
  return transient_send_retry_count;
}
} // namespace

UdpSocket::UdpSocket(const UdpSocket::Config &config)
    : Socket(Type::DGRAM, Logger::Config{.tag = "UdpSocket", .level = config.log_level}) {}

UdpSocket::~UdpSocket() { stop_receiving(); }

void UdpSocket::stop_receiving() {
  // Close the socket first so any blocking recvfrom returns and the task can stop.
  cleanup();
  if (task_ && task_->is_started()) {
    task_->stop();
  }
}

bool UdpSocket::send(const std::vector<uint8_t> &data, const UdpSocket::SendConfig &send_config) {
  return send(std::span<const uint8_t>{data.data(), data.size()}, send_config);
}

bool UdpSocket::send(std::string_view data, const UdpSocket::SendConfig &send_config) {
  return send(std::span<const uint8_t>{reinterpret_cast<const uint8_t *>(data.data()), data.size()},
              send_config);
}

bool UdpSocket::send(std::span<const uint8_t> data, const UdpSocket::SendConfig &send_config) {
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
  int num_bytes_sent = -1;
  for (int attempt = 0; attempt <= transient_send_retry_count; attempt++) {
    num_bytes_sent =
        sendto(socket_, reinterpret_cast<const char *>(data.data()), data.size(), 0,
               reinterpret_cast<struct sockaddr *>(server_address), sizeof(*server_address));
    if (num_bytes_sent >= 0) {
      break;
    }
    int err = last_socket_error();
    int retry_limit = transient_send_retry_limit(err);
    if (!is_transient_send_error(err)) {
      logger_.error("Error occurred during sending {} bytes to {}:{}: {}", data.size(),
                    send_config.ip_address, send_config.port, error_string(err));
      return false;
    }
    if (attempt >= retry_limit) {
#if defined(ESP_PLATFORM)
      if (err == ENOBUFS || err == ENOMEM) {
        logger_.warn("Dropping UDP send of {} bytes to {}:{} due to TX backpressure: {}",
                     data.size(), send_config.ip_address, send_config.port, error_string(err));
        return false;
      }
#endif
      logger_.error("Error occurred during sending {} bytes to {}:{}: {}", data.size(),
                    send_config.ip_address, send_config.port, error_string(err));
      return false;
    }
    logger_.warn("Transient send failure sending {} bytes to {}:{} (attempt {}/{}): {}",
                 data.size(), send_config.ip_address, send_config.port, attempt + 1,
                 retry_limit + 1, error_string(err));
    std::this_thread::sleep_for(transient_send_retry_delay);
  }
  if (num_bytes_sent < 0) {
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
  int num_bytes_received =
      recvfrom(socket_, reinterpret_cast<char *>(receive_buffer.get()), max_num_bytes, 0,
               reinterpret_cast<struct sockaddr *>(remote_address), &socklen);
  // if we didn't receive anything return false and don't do anything else
  if (num_bytes_received < 0) {
    logger_.info("Receive failed: {}", error_string());
    return false;
  }
  // we received data, so call the callback function if one was provided.
  uint8_t *data_ptr = receive_buffer.get();
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
  struct sockaddr_in server_addr {};
  // configure the server socket accordingly - assume IPV4 and bind to the
  // any address "0.0.0.0"
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  server_addr.sin_family = address_family_;
  server_addr.sin_port = htons(receive_config.port);
  int err = bind(socket_, reinterpret_cast<struct sockaddr *>(&server_addr), sizeof(server_addr));
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
  int num_bytes_sent =
      sendto(socket_, reinterpret_cast<const char *>(response.data()), response.size(), 0,
             reinterpret_cast<struct sockaddr *>(sender_address), sizeof(*sender_address));
  if (num_bytes_sent < 0) {
    logger_.error("Error occurred responding: {}", error_string());
  }
  logger_.info("Server responded with {} bytes", num_bytes_sent);
  // don't want to stop the task
  return false;
}

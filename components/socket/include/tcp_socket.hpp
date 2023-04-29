#pragma once

#include <optional>
#include <string_view>
#include <vector>

#include "logger.hpp"
#include "socket.hpp"
#include "task.hpp"

namespace espp {
namespace detail {
/**
 * @brief Config struct for sending data to a remote TCP socket.
 * @note This is only used when waiting for a response from the remote.
 * @note This must be outside the TcpSocket class because of a gcc bug that
 *       still has not been fixed. See
 *       https://stackoverflow.com/questions/53408962/try-to-understand-compiler-error-message-default-member-initializer-required-be
 */
struct TcpTransmitConfig {
  bool wait_for_response{false}; /**< Whether to wait for a response from the remote or not. */
  size_t response_size{
      0}; /**< If waiting for a response, this is the maximum size response we will receive. */
  Socket::response_callback_fn on_response_callback{
      nullptr}; /**< If waiting for a response, this is an optional handler which is provided the
                   response data. */
  std::chrono::duration<float> response_timeout{
      0.5f}; /**< If waiting for a response, this is the maximum timeout to wait. */
};
} // namespace detail

/**
 *   @brief Class for managing sending and receiving data using TCP/IP. Can be
 *          used to create client or server sockets.
 *
 * \section tcp_ex1 TCP Client Example
 * \snippet socket_example.cpp TCP Client example
 * \section tcp_ex2 TCP Server Example
 * \snippet socket_example.cpp TCP Server example
 *
 * \section tcp_ex3 TCP Client Response Example
 * \snippet socket_example.cpp TCP Client Response example
 * \section tcp_ex4 TCP Server Response Example
 * \snippet socket_example.cpp TCP Server Response example
 *
 */
class TcpSocket : public Socket {
public:
  /**
   * @brief Config struct for the TCP socket.
   */
  struct Config {
    Logger::Verbosity log_level{
        Logger::Verbosity::WARN}; /**< Verbosity level for the TCP socket logger. */
  };

  /**
   * @brief Config struct for connecting to a remote TCP server.
   */
  struct ConnectConfig {
    std::string ip_address; /**< Address to send data to. */
    size_t port;            /**< Port number to send data to.*/
  };

  /**
   * @brief Initialize the socket and associated resources.
   * @note Enables keepalive on the socket.
   * @param config Config for the socket.
   */
  TcpSocket(const Config &config)
      : Socket(Type::STREAM, Logger::Config{.tag = "TcpSocket", .level = config.log_level}) {
    set_keepalive();
  }

  /**
   * @brief Tear down any resources associted with the socket.
   */
  ~TcpSocket() {
    // we have to explicitly call cleanup here so that the server accept /
    // read will return and the task can stop.
    cleanup();
  }

  /**
   * @brief Reinitialize the socket, cleaning it up if first it is already
   *        initalized.
   */
  void reinit() {
    if (is_valid()) {
      // cleanup our socket
      cleanup();
    }
    init(Type::STREAM);
  }

  /**
   * @brief Close the socket.
   */
  void close() { ::close(socket_); }

  /**
   * @brief Check if the socket is connected to a remote endpoint.
   * @return true if the socket is connected to a remote endpoint.
   */
  bool is_connected() const { return connected_; }

  /**
   * @brief Open a connection to the remote TCP server.
   * @param connect_config ConnectConfig struct describing the server endpoint.
   * @return true if the client successfully connected to the server.
   */
  bool connect(const ConnectConfig &connect_config) {
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
      logger_.error("Could not connect to the server: {} - '{}'", errno, strerror(errno));
      return false;
    }
    connected_ = true;
    return true;
  }

  /**
   * @brief Get the remote endpoint info.
   * @return The remote endpoint info.
   */
  const Socket::Info &get_remote_info() const { return remote_info_; }

  /**
   * @brief Send data to the endpoint already connected to by TcpSocket::connect.
   *        Can be configured to block waiting for a response from the remote.
   *
   *        If response is requested, a callback can be provided in
   *        send_config which will be provided the response data for
   *        processing.
   * @param data vector of bytes to send to the remote endpoint.
   * @param transmit_config detail::TcpTransmitConfig struct indicating whether to wait for a
   *        response.
   * @return true if the data was sent, false otherwise.
   */
  bool transmit(const std::vector<uint8_t> &data,
                const detail::TcpTransmitConfig &transmit_config = {}) {
    return transmit(std::string_view{(const char *)data.data(), data.size()}, transmit_config);
  }

  /**
   * @brief Send data to the endpoint already connected to by TcpSocket::connect.
   *        Can be configured to block waiting for a response from the remote.
   *
   *        If response is requested, a callback can be provided in
   *        send_config which will be provided the response data for
   *        processing.
   * @param data vector of bytes to send to the remote endpoint.
   * @param transmit_config detail::TcpTransmitConfig struct indicating whether to wait for a
   *        response.
   * @return true if the data was sent, false otherwise.
   */
  bool transmit(const std::vector<char> &data,
                const detail::TcpTransmitConfig &transmit_config = {}) {
    return transmit(std::string_view{(const char *)data.data(), data.size()}, transmit_config);
  }

  /**
   * @brief Send data to the endpoint already connected to by TcpSocket::connect.
   *        Can be configured to block waiting for a response from the remote.
   *
   *        If response is requested, a callback can be provided in
   *        send_config which will be provided the response data for
   *        processing.
   * @param data string view of bytes to send to the remote endpoint.
   * @param transmit_config detail::TcpTransmitConfig struct indicating whether to wait for a
   *        response.
   * @return true if the data was sent, false otherwise.
   */
  bool transmit(std::string_view data, const detail::TcpTransmitConfig &transmit_config = {}) {
    if (!is_valid()) {
      logger_.error("Socket invalid, cannot send");
      return false;
    }
    // set the receive timeout
    if (!set_receive_timeout(transmit_config.response_timeout)) {
      logger_.error("Could not set receive timeout to {}: {} - '{}'",
                    transmit_config.response_timeout.count(), errno, strerror(errno));
      return false;
    }
    // write
    logger_.info("Client sending {} bytes", data.size());
    int num_bytes_sent = write(socket_, data.data(), data.size());
    if (num_bytes_sent < 0) {
      logger_.error("Error occurred during sending: {} - '{}'", errno, strerror(errno));
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

  /**
   * @brief Call read on the socket, assuming it has already been configured
   *        appropriately.
   *
   * @param data Vector of bytes of received data.
   * @param max_num_bytes Maximum number of bytes to receive.
   * @return true if successfully received, false otherwise.
   */
  bool receive(std::vector<uint8_t> &data, size_t max_num_bytes) {
    // make some space for received data - put it on the heap so that our
    // stack usage doesn't change depending on max_num_bytes
    std::unique_ptr<uint8_t[]> receive_buffer(new uint8_t[max_num_bytes]());
    int num_bytes_received = receive(receive_buffer.get(), max_num_bytes);
    if (num_bytes_received > 0) {
      logger_.info("Received {} bytes", num_bytes_received);
      data.assign(receive_buffer.get(), receive_buffer.get() + num_bytes_received);
      return true;
    }
    return false;
  }

  /**
   * @brief Call read on the socket, assuming it has already been configured
   *        appropriately.
   * @note This function will block until max_num_bytes are received or the
   *       receive timeout is reached.
   * @note The data pointed to by data must be at least max_num_bytes in size.
   * @param data Pointer to buffer to receive data.
   * @param max_num_bytes Maximum number of bytes to receive.
   * @return Number of bytes received.
   */
  size_t receive(uint8_t *data, size_t max_num_bytes) {
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
    int num_bytes_received = ::recv(socket_, data, max_num_bytes, 0);
    // if we didn't receive anything return false and don't do anything else
    if (num_bytes_received < 0) {
      // if we got an error, log it and return false
      logger_.debug("Receive failed: {} - '{}'", errno, strerror(errno));
    } else if (num_bytes_received == 0) {
      logger_.warn("Remote socket closed!");
      // update our connection state here since remote end was closed...
      connected_ = false;
    } else {
      logger_.debug("Received {} bytes", num_bytes_received);
    }
    return num_bytes_received;
  }

  /**
   * @brief Bind the socket as a server on \p port.
   * @param port The port to which to bind the socket.
   * @return true if the socket was bound.
   */
  bool bind(int port) {
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
      logger_.error("Unable to bind: {} - '{}'", errno, strerror(errno));
      return false;
    }
    return true;
  }

  /**
   * @brief Listen for incoming client connections.
   * @note Must be called after bind and before accept.
   * @see bind
   * @see accept
   * @param max_pending_connections Max number of allowed pending connections.
   * @return True if socket was able to start listening.
   */
  bool listen(int max_pending_connections) {
    if (!is_valid()) {
      logger_.error("Socket invalid, cannot listen.");
      return false;
    }
    auto err = ::listen(socket_, max_pending_connections);
    if (err < 0) {
      logger_.error("Unable to listen: {} - '{}'", errno, strerror(errno));
      return false;
    }
    return true;
  }

  /**
   * @brief Accept an incoming connection.
   * @note Blocks until a connection is accepted.
   * @note Must be called after listen.
   * @note This function will block until a connection is accepted.
   * @return A unique pointer to a TcpClientSession if a connection was
   *         accepted, nullptr otherwise.
   */
  std::unique_ptr<TcpSocket> accept() {
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
      logger_.error("Could not accept connection: {} - '{}'", errno, strerror(errno));
      return nullptr;
    }
    connected_client_info.update();
    logger_.info("Server accepted connection with {}", connected_client_info);
    // NOTE: have to use new here because we can't use make_unique with a
    //       protected or private constructor
    return std::unique_ptr<TcpSocket>(new TcpSocket(accepted_socket, connected_client_info));
  }

protected:
  /**
   * @brief Construct a new TcpSocket object
   * @note This sets connected_ to true, under the assumption that the socket
   *       file descriptor is valid and already connected to a remote endpoint.
   * @note This constructor is primarily used by the accept() method to create
   *       a new TcpSocket object for the accepted connection.
   * @param socket_fd The socket file descriptor for the connection.
   * @param remote_info The remote endpoint info.
   */
  TcpSocket(int socket_fd, const Socket::Info &remote_info)
      : Socket(socket_fd, Logger::Config{.tag = "TcpSocket", .level = Logger::Verbosity::WARN}),
        remote_info_(remote_info) {
    connected_ = true;
    set_keepalive();
  }

  bool set_keepalive(std::chrono::seconds idle_time = std::chrono::seconds{60},
                     std::chrono::seconds interval = std::chrono::seconds{10}, int max_probes = 5) {
    if (!is_valid()) {
      logger_.error("Socket invalid, cannot set keepalive.");
      return false;
    }
    int optval = 1;
    // enable keepalive
    auto err = setsockopt(socket_, SOL_SOCKET, SO_KEEPALIVE, &optval, sizeof(optval));
    if (err < 0) {
      logger_.error("Unable to set keepalive: {} - '{}'", errno, strerror(errno));
      return false;
    }
    // set the idle time
    optval = idle_time.count();
    err = setsockopt(socket_, IPPROTO_TCP, TCP_KEEPIDLE, &optval, sizeof(optval));
    if (err < 0) {
      logger_.error("Unable to set keepalive idle time: {} - '{}'", errno, strerror(errno));
      return false;
    }
    // set the interval
    optval = interval.count();
    err = setsockopt(socket_, IPPROTO_TCP, TCP_KEEPINTVL, &optval, sizeof(optval));
    if (err < 0) {
      logger_.error("Unable to set keepalive interval: {} - '{}'", errno, strerror(errno));
      return false;
    }
    // set the max probes
    optval = max_probes;
    err = setsockopt(socket_, IPPROTO_TCP, TCP_KEEPCNT, &optval, sizeof(optval));
    if (err < 0) {
      logger_.error("Unable to set keepalive max probes: {} - '{}'", errno, strerror(errno));
      return false;
    }
    return true;
  }

  bool connected_{false};
  Socket::Info remote_info_;
};
} // namespace espp

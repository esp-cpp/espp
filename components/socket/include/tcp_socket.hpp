#pragma once

#include "socket_msvc.hpp"

#ifndef _MSC_VER
#include <netinet/tcp.h>
#endif // _MSC_VER

#include <optional>
#include <string_view>
#include <vector>

#include "logger.hpp"
#include "socket.hpp"
#include "task.hpp"

namespace espp {
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
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; /**< Verbosity level for the TCP socket logger. */
  };

  /**
   * @brief Config struct for connecting to a remote TCP server.
   */
  struct ConnectConfig {
    std::string ip_address; /**< Address to send data to. */
    size_t port;            /**< Port number to send data to.*/
  };

  /**
   * @brief Config struct for sending data to a remote TCP socket.
   * @note This is only used when waiting for a response from the remote.
   */
  struct TransmitConfig {
    bool wait_for_response = false; /**< Whether to wait for a response from the remote or not. */
    size_t response_size =
        0; /**< If waiting for a response, this is the maximum size response we will receive. */
    espp::Socket::response_callback_fn on_response_callback = nullptr; /**< If waiting for a
                   response, this is an optional handler which is provided the response data. */
    std::chrono::duration<float> response_timeout = std::chrono::duration<float>(
        0.5f); /**< If waiting for a response, this is the maximum timeout to wait. */

    static TransmitConfig Default() { return {}; }
  };

  /**
   * @brief Initialize the socket and associated resources.
   * @note Enables keepalive on the socket.
   * @param config Config for the socket.
   */
  explicit TcpSocket(const espp::TcpSocket::Config &config);

  /**
   * @brief Tear down any resources associted with the socket.
   */
  ~TcpSocket();

  /**
   * @brief Reinitialize the socket, cleaning it up if first it is already
   *        initalized.
   */
  void reinit();

  /**
   * @brief Close the socket.
   */
  void close();

  /**
   * @brief Check if the socket is connected to a remote endpoint.
   * @return true if the socket is connected to a remote endpoint.
   */
  bool is_connected() const;

  /**
   * @brief Open a connection to the remote TCP server.
   * @param connect_config ConnectConfig struct describing the server endpoint.
   * @return true if the client successfully connected to the server.
   */
  bool connect(const espp::TcpSocket::ConnectConfig &connect_config);

  /**
   * @brief Get the remote endpoint info.
   * @return The remote endpoint info.
   */
  const espp::Socket::Info &get_remote_info() const;

  /**
   * @brief Send data to the endpoint already connected to by TcpSocket::connect.
   *        Can be configured to block waiting for a response from the remote.
   *
   *        If response is requested, a callback can be provided in
   *        send_config which will be provided the response data for
   *        processing.
   * @param data vector of bytes to send to the remote endpoint.
   * @param transmit_config TransmitConfig struct indicating whether to wait for a
   *        response.
   * @return true if the data was sent, false otherwise.
   */
  bool transmit(const std::vector<uint8_t> &data,
                const espp::TcpSocket::TransmitConfig &transmit_config =
                    espp::TcpSocket::TransmitConfig::Default());

  /**
   * @brief Send data to the endpoint already connected to by TcpSocket::connect.
   *        Can be configured to block waiting for a response from the remote.
   *
   *        If response is requested, a callback can be provided in
   *        send_config which will be provided the response data for
   *        processing.
   * @param data vector of bytes to send to the remote endpoint.
   * @param transmit_config TransmitConfig struct indicating whether to wait for a
   *        response.
   * @return true if the data was sent, false otherwise.
   */
  bool transmit(const std::vector<char> &data,
                const espp::TcpSocket::TransmitConfig &transmit_config =
                    espp::TcpSocket::TransmitConfig::Default());

  /**
   * @brief Send data to the endpoint already connected to by TcpSocket::connect.
   *        Can be configured to block waiting for a response from the remote.
   *
   *        If response is requested, a callback can be provided in
   *        send_config which will be provided the response data for
   *        processing.
   * @param data string view of bytes to send to the remote endpoint.
   * @param transmit_config TransmitConfig struct indicating whether to wait for a
   *        response.
   * @return true if the data was sent, false otherwise.
   */
  bool transmit(std::string_view data, const espp::TcpSocket::TransmitConfig &transmit_config =
                                           espp::TcpSocket::TransmitConfig::Default());

  /**
   * @brief Call read on the socket, assuming it has already been configured
   *        appropriately.
   *
   * @param data Vector of bytes of received data.
   * @param max_num_bytes Maximum number of bytes to receive.
   * @return true if successfully received, false otherwise.
   */
  bool receive(std::vector<uint8_t> &data, size_t max_num_bytes);

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
  size_t receive(uint8_t *data, size_t max_num_bytes);

  /**
   * @brief Bind the socket as a server on \p port.
   * @param port The port to which to bind the socket.
   * @return true if the socket was bound.
   */
  bool bind(int port);

  /**
   * @brief Listen for incoming client connections.
   * @note Must be called after bind and before accept.
   * @see bind
   * @see accept
   * @param max_pending_connections Max number of allowed pending connections.
   * @return True if socket was able to start listening.
   */
  bool listen(int max_pending_connections);

  /**
   * @brief Accept an incoming connection.
   * @note Blocks until a connection is accepted.
   * @note Must be called after listen.
   * @note This function will block until a connection is accepted.
   * @return A unique pointer to a TcpClientSession if a connection was
   *         accepted, nullptr otherwise.
   */
  std::unique_ptr<espp::TcpSocket> accept();

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
  explicit TcpSocket(sock_type_t socket_fd, const espp::Socket::Info &remote_info);

  bool set_keepalive(const std::chrono::seconds &idle_time = std::chrono::seconds{60},
                     const std::chrono::seconds &interval = std::chrono::seconds{10},
                     int max_probes = 5);

  bool connected_{false};
  espp::Socket::Info remote_info_{};
};
} // namespace espp

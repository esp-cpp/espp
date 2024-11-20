#pragma once

#include "socket_msvc.hpp"

#include <optional>
#include <string_view>
#include <vector>

#include "logger.hpp"
#include "socket.hpp"
#include "task.hpp"

// TODO: should this class _contain_ a socket or just create sockets within each
//       call?

namespace espp {
/**
 *   @brief Class for managing sending and receiving data using UDP/IP. Can be
 *          used to create client or server sockets.
 *
 *   See
 *   https://github.com/espressif/esp-idf/tree/master/examples/protocols/sockets/udp_multicast
 *   for more information on udp multicast sockets.
 *
 * \section udp_ex1 UDP Client Example
 * \snippet socket_example.cpp UDP Client example
 * \section udp_ex2 UDP Server Example
 * \snippet socket_example.cpp UDP Server example
 *
 * \section udp_ex3 UDP Client Response Example
 * \snippet socket_example.cpp UDP Client Response example
 * \section udp_ex4 UDP Server Response Example
 * \snippet socket_example.cpp UDP Server Response example
 *
 * \section udp_ex5 UDP Multicast Client Example
 * \snippet socket_example.cpp UDP Multicast Client example
 * \section udp_ex6 UDP Multicast Server Example
 * \snippet socket_example.cpp UDP Multicast Server example
 *
 */
class UdpSocket : public Socket {
public:
  struct ReceiveConfig {
    size_t port;                       /**< Port number to bind to / receive from. */
    size_t buffer_size;                /**< Max size of data we can receive at one time. */
    bool is_multicast_endpoint{false}; /**< Whether this should be a multicast endpoint. */
    std::string multicast_group{
        ""}; /**< If this is a multicast endpoint, this is the group it belongs to. */
    espp::Socket::receive_callback_fn on_receive_callback{
        nullptr}; /**< Function containing business logic to handle data received. */
  };

  struct SendConfig {
    std::string ip_address;            /**< Address to send data to. */
    size_t port;                       /**< Port number to send data to.*/
    bool is_multicast_endpoint{false}; /**< Whether this should be a multicast endpoint. */
    bool wait_for_response{false}; /**< Whether to wait for a response from the remote or not. */
    size_t response_size{
        0}; /**< If waiting for a response, this is the maximum size response we will receive. */
    espp::Socket::response_callback_fn on_response_callback{
        nullptr}; /**< If waiting for a response, this is an optional handler which is provided the
                     response data. */
    std::chrono::duration<float> response_timeout = std::chrono::duration<float>(
        0.5f); /**< If waiting for a response, this is the maximum timeout to wait. */
  };

  struct Config {
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; /**< Verbosity level for the UDP socket logger. */
  };

  /**
   * @brief Initialize the socket and associated resources.
   * @param config Config for the socket.
   */
  explicit UdpSocket(const Config &config);

  /**
   * @brief Tear down any resources associted with the socket.
   */
  ~UdpSocket();

  /**
   * @brief Send data to the endpoint specified by the send_config.
   *        Can be configured to multicast (within send_config) and can be
   *        configured to block waiting for a response from the remote.
   *
   *        @note in the case of multicast, it will block only until the first
   *              response.
   *
   *        If response is requested, a callback can be provided in
   *        send_config which will be provided the response data for
   *        processing.
   * @param data vector of bytes to send to the remote endpoint.
   * @param send_config SendConfig struct indicating where to send and whether
   *        to wait for a response.
   * @return true if the data was sent, false otherwise.
   */
  bool send(const std::vector<uint8_t> &data, const SendConfig &send_config);

  /**
   * @brief Send data to the endpoint specified by the send_config.
   *        Can be configured to multicast (within send_config) and can be
   *        configured to block waiting for a response from the remote.
   *
   *        @note in the case of multicast, it will block only until the first
   *              response.
   *
   *        If response is requested, a callback can be provided in
   *        send_config which will be provided the response data for
   *        processing.
   * @param data String view of bytes to send to the remote endpoint.
   * @param send_config SendConfig struct indicating where to send and whether
   *        to wait for a response.
   * @return true if the data was sent, false otherwise.
   */
  bool send(std::string_view data, const SendConfig &send_config);

  /**
   * @brief Call recvfrom on the socket, assuming it has already been
   *        configured appropriately.
   *
   * @param max_num_bytes Maximum number of bytes to receive.
   * @param data Vector of bytes of received data.
   * @param remote_info Socket::Info containing the sender's information. This
   *        will be populated with the information about the sender.
   * @return true if successfully received, false otherwise.
   */
  bool receive(size_t max_num_bytes, std::vector<uint8_t> &data, Socket::Info &remote_info);

  /**
   * @brief Configure a server socket and start a thread to continuously
   *        receive and handle data coming in on that socket.
   *
   * @param task_config Task::BaseConfig struct for configuring the receive task.
   * @param receive_config ReceiveConfig struct with socket and callback info.
   * @return true if the socket was created and task was started, false otherwise.
   */
  bool start_receiving(Task::BaseConfig &task_config, const ReceiveConfig &receive_config);

protected:
  /**
   * @brief Function run in the task_ when start_receiving is called.
   *        Continuously receive data on the socket, pass the received data to
   *        the registered callback function (registered in start_receiving in
   *        the ReceiveConfig struct), and optionally respond to the sender if
   *        the registered callback returns data.
   * @param buffer_size number of bytes of receive buffer allowed.
   * @param m std::mutex provided from the task for use with the
   *          condition_variable (cv)
   * @param cv std::condition_variable from the task for allowing
   *           interruptible wait / delay.
   * @param task_notified bool from the task to indicate whether the task has
   *        been notified - used with the condition variable to ignore spurious
   *        wakeups.
   * @return Return true if the task should stop; false if it should continue.
   */
  bool server_task_function(size_t buffer_size, std::mutex &m, std::condition_variable &cv,
                            bool &task_notified);

  std::unique_ptr<Task> task_;
  receive_callback_fn server_receive_callback_;
};
} // namespace espp

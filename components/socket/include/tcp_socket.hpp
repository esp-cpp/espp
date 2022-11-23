#pragma once

#include <optional>
#include <vector>
#include <string_view>

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

    struct Config {
      Logger::Verbosity log_level{Logger::Verbosity::WARN}; /**< Verbosity level for the TCP socket logger. */
    };

    struct ConnectConfig {
      std::string ip_address; /**< Address to send data to. */
      size_t port; /**< Port number to send data to.*/
    };

    struct ReceiveConfig {
      size_t port; /**< Port number to bind to / receive from. */
      size_t max_pending_connections{5}; /**< Maximum length to which the queue of pending connections may grow. */
      size_t buffer_size; /**< Max size of data we can receive at one time. */
      receive_callback_fn on_receive_callback{nullptr}; /**< Function containing business logic to handle data received. */
    };

    struct TransmitConfig {
      bool wait_for_response{false}; /**< Whether to wait for a response from the remote or not. */
      size_t response_size{0}; /**< If waiting for a response, this is the maximum size response we will receive. */
      response_callback_fn on_response_callback{nullptr}; /**< If waiting for a response, this is an optional handler which is provided the response data. */
      std::chrono::duration<float> response_timeout{0.5f}; /**< If waiting for a response, this is the maximum timeout to wait. */
    };

    /**
      * @brief Initialize the socket and associated resources.
      * @param config Config for the socket.
      */
    TcpSocket(const Config& config) : Socket(Type::STREAM, Logger::Config{.tag="TcpSocket", .level=config.log_level}) { }

    /**
     * @brief Tear down any resources associted with the socket.
     */
    ~TcpSocket() {
      // we have to explicitly call cleanup here so that the server accept /
      // read will return and the task can stop.
      cleanup();
    }

    /**
     * @brief Open a connection to the remote TCP server.
     * @param connect_config ConnectConfig struct describing the server endpoint.
     * @return true if the client successfully connected to the server.
     */
    bool connect(const ConnectConfig& connect_config) {
      if (!is_valid()) {
        logger_.error("Socket invalid, cannot connect");
        return false;
      }
      Socket::Info server_info;
      server_info.init_ipv4(connect_config.ip_address, connect_config.port);
      auto server_address = server_info.ipv4_ptr();
      logger_.info("Client connecting to {}", server_info.to_string());
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
     * @brief Is this TCP client socket connected to a server?
     * @note This will be set to TRUE if the connect() call succeeded, but will
     *       reset to FALSE if the transmit call fails.
     * @return True if the socket has successfully called connect() and can call
     *         transmit().
     */
    bool is_connected() {
      return connected_;
    }

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
    bool transmit(const std::vector<uint8_t>& data, const TransmitConfig& transmit_config) {
      return transmit(std::string_view{(const char*)data.data(), data.size()}, transmit_config);
    }

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
    bool transmit(std::string_view data, const TransmitConfig& transmit_config) {
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
      if (!receive(socket_, transmit_config.response_size, received_data)) {
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
     * @param socket_fd Socket file descriptor.
     * @param max_num_bytes Maximum number of bytes to receive.
     * @param data Vector of bytes of received data.
     * @return true if successfully received, false otherwise.
     */
    bool receive(int socket_fd, size_t max_num_bytes, std::vector<uint8_t>& data) {
      if (!is_valid(socket_fd)) {
        logger_.error("Socket invalid, cannot receive.");
        return false;
      }
      // make some space for received data - put it on the heap so that our
      // stack usage doesn't change depending on max_num_bytes
      std::unique_ptr<uint8_t[]> receive_buffer(new uint8_t[max_num_bytes]());
      logger_.info("Receiving up to {} bytes", max_num_bytes);
      // now actually read data from the socket
      int num_bytes_received = read(socket_fd, receive_buffer.get(), max_num_bytes);
      // if we didn't receive anything return false and don't do anything else
      if (num_bytes_received < 0) {
        logger_.error("Receive failed: {} - '{}'", errno, strerror(errno));
        return false;
      }
      // we received data, so call the callback function if one was provided.
      data.assign(receive_buffer.get(), receive_buffer.get() + num_bytes_received);
      logger_.debug("Received {} bytes", num_bytes_received);
      return true;
    }

    /**
     * @brief Configure a server socket and start a thread to continuously
     *        receive and handle data coming in on that socket.
     *
     * @param task_config Task::Config struct for configuring the receive task.
     * @param receive_config ReceiveConfig struct with socket and callback info.
     * @return true if the socket was created and task was started, false otherwise.
     */
    bool start_receiving(Task::Config& task_config, const ReceiveConfig& receive_config) {
      // TODO: allow this to start multiple threads up to some max number of
      // connected clients. In this way we would have a thread that calls
      // accept, which in turn creates a new thread for each client that
      // accepts.
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
        logger_.error("Unable to bind: {} - '{}'", errno, strerror(errno));
        return false;
      }
      // listen
      err = listen(socket_, receive_config.max_pending_connections);
      if (err < 0) {
        logger_.error("Unable to listen: {} - '{}'", errno, strerror(errno));
        return false;
      }
      // set the callback function
      using namespace std::placeholders;
      task_config.callback = std::bind(&TcpSocket::server_task_function, this,
                                       receive_config.buffer_size, _1, _2);
      // start the thread
      task_ = Task::make_unique(task_config);
      task_->start();
      return true;
    }

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
     */
    void server_task_function(size_t buffer_size, std::mutex& m, std::condition_variable& cv) {
      Socket::Info sender_info;
      auto sender_address = sender_info.ipv4_ptr();
      socklen_t socklen = sizeof(*sender_address);
      // accept connection
      // int flags = 0; // TODO: could set to SOCK_NONBLOCK or SOCK_CLOEXEC
      int accepted_socket = accept(socket_, (struct sockaddr*)sender_address, &socklen);
      if (accepted_socket < 0) {
        logger_.error("Could not accept connection: {} - '{}'", errno, strerror(errno));
        // if we failed to accept that means there are no connections available
        // so we should delay a little bit
        using namespace std::chrono_literals;
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 1ms);
        return;
      }
      sender_info.update();
      logger_.info("Server accepted connection with {}", sender_info.to_string());
      while (true) {
        // receive data
        std::vector<uint8_t> received_data;
        if (!receive(accepted_socket, buffer_size, received_data)) {
          logger_.error("Could not receive data: {} - '{}'", errno, strerror(errno));
          break;
        }
        if (!server_receive_callback_) {
          logger_.error("Server receive callback is invalid");
          break;
        }
        // callback
        auto maybe_response = server_receive_callback_(received_data, sender_info);
        // send if callback returned data
        if (!maybe_response.has_value()) {
          continue;
        }
        auto response = maybe_response.value();
        // write
        logger_.info("Server responding to {} with message of length {}", sender_info.to_string(), response.size());
        int num_bytes_sent = write(accepted_socket, response.data(), response.size());
        if (num_bytes_sent < 0) {
          logger_.error("Error occurred responding: {} - '{}'", errno, strerror(errno));
          break;
        }
        logger_.info("Server responded with {} bytes", num_bytes_sent);
      }
      // if we've gotten here, we are no longer receiving data from the client,
      // so close the accepted socket and accept other connections.
      close(accepted_socket);
    }

    std::atomic<bool> connected_{false};
    std::unique_ptr<Task> task_;
    receive_callback_fn server_receive_callback_;
  };
}

#pragma once

#include <atomic>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#if defined(ESP_PLATFORM)
#include <esp_random.h>
#else
#include <random>
#endif

#include "base_component.hpp"
#include "task.hpp"
#include "tcp_socket.hpp"

#include "ftp_client_session.hpp"

namespace espp {
/// \brief A class that implements a FTP server.
class FtpServer : public BaseComponent {
public:
  /// \brief A class that implements a FTP server.
  /// \note The IP Address is not currently used to select the right
  ///       interface, but is instead passed to the FtpClientSession so that
  ///       it can be used in the PASV command.
  /// \param ip_address The IP address to listen on.
  /// \param port The port to listen on.
  /// \param root The root directory of the FTP server.
  FtpServer(std::string_view ip_address, uint16_t port, const std::filesystem::path &root)
      : BaseComponent("FtpServer")
      , ip_address_(ip_address)
      , port_(port)
      , server_({.log_level = Logger::Verbosity::WARN})
      , root_(root) {}

  /// \brief Destroy the FTP server.
  ~FtpServer() { stop(); }

  /// \brief Start the FTP server.
  /// Bind to the port and start accepting connections.
  /// \return True if the server was started, false otherwise.
  bool start() {
    if (accept_task_ && accept_task_->is_started()) {
      logger_.error("Server was already started");
      return false;
    }

    if (!server_.bind(port_)) {
      logger_.error("Failed to bind to port {}", port_);
      return false;
    }

    int max_pending_connections = 5;
    if (!server_.listen(max_pending_connections)) {
      logger_.error("Failed to listen on port {}", port_);
      return false;
    }

    using namespace std::placeholders;
    accept_task_ = std::make_unique<Task>(Task::Config{
        .callback = std::bind(&FtpServer::accept_task_function, this, _1, _2, _3),
        .task_config =
            {
                .name = "FtpServer::accept_task",
                .stack_size_bytes = 1024 * 4,
            },
        .log_level = Logger::Verbosity::WARN,
    });
    accept_task_->start();
    return true;
  }

  /// \brief Stop the FTP server.
  void stop() {
    clear_clients();
    stop_accepting();
  }

protected:
  /// \brief Stop accepting new connections.
  void stop_accepting() {
    if (accept_task_ && accept_task_->is_started()) {
      accept_task_->stop();
    }
  }

  void clear_clients() {
    std::lock_guard<std::mutex> lk(clients_mutex_);
    clients_.clear();
  }

  bool accept_task_function(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
    auto client_ptr = server_.accept();
    if (!client_ptr) {
      logger_.error("Could not accept connection");
      // if we failed to accept that means there are no connections available
      // so we should delay a little bit
      using namespace std::chrono_literals;
      std::unique_lock<std::mutex> lk(m);
      cv.wait_for(lk, 1ms, [&task_notified] { return task_notified; });
      task_notified = false;
      // don't want to stop the task
      return false;
    }

    // we have a new client, so make a new client id
    int client_id = generate_client_id();

    logger_.info("Accepted connection from {}, id {}", client_ptr->get_remote_info(), client_id);

    // create a new client session
    auto client_session_ptr =
        std::make_unique<FtpClientSession>(client_id, ip_address_, std::move(client_ptr), root_);

    // add the client session to the map of clients
    std::lock_guard<std::mutex> lk(clients_mutex_);
    // clean up any clients that have disconnected
    for (auto it = clients_.begin(); it != clients_.end();) {
      if (!it->second->is_connected() || !it->second->is_alive()) {
        logger_.info("Client {} disconnected, removing", it->first);
        it = clients_.erase(it);
      } else {
        ++it;
      }
    }
    // add the new client
    clients_[client_id] = std::move(client_session_ptr);

    // don't want to stop the task
    return false;
  }

  int generate_client_id() {
#if defined(ESP_PLATFORM)
    return esp_random();
#else
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(0, std::numeric_limits<int>::max());
    return dis(gen);
#endif
  }

  void remove_client_task(int client_id) {
    std::lock_guard<std::mutex> lk(clients_mutex_);
    clients_.erase(client_id);
  }

  std::string ip_address_;
  uint16_t port_;
  TcpSocket server_;
  std::unique_ptr<Task> accept_task_;

  std::filesystem::path root_;

  std::mutex clients_mutex_;
  std::unordered_map<int, std::unique_ptr<FtpClientSession>> clients_;
};
} // namespace espp

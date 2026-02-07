#pragma once

#include <functional>
#include <memory>
#include <string>

#include "base_component.hpp"
#include "udp_socket.hpp"

namespace espp {
/**
 * @brief Simple DNS server for captive portal support.
 *
 * This component implements a minimal DNS server that responds to all DNS queries
 * with a configured IP address. This is useful for captive portals where you want
 * all DNS requests to resolve to the ESP32's IP address.
 *
 * The server listens on UDP port 53 (standard DNS port) and responds to A record
 * queries with the configured IP address.
 *
 * \section dns_server_ex1 DNS Server Example
 * \snippet dns_server_example.cpp dns server example
 */
class DnsServer : public BaseComponent {
public:
  /**
   * @brief Configuration for the DNS server
   */
  struct Config {
    std::string ip_address; /**< IP address to respond with for all DNS queries */
    espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN; /**< Log verbosity */
  };

  /**
   * @brief Construct a new DNS Server
   * @param config Configuration for the DNS server
   */
  explicit DnsServer(const Config &config);

  /**
   * @brief Destroy the DNS Server
   */
  ~DnsServer();

  /**
   * @brief Start the DNS server
   * @param ec Error code set if start fails
   * @return true if server started successfully, false otherwise
   */
  bool start(std::error_code &ec);

  /**
   * @brief Stop the DNS server
   */
  void stop();

  /**
   * @brief Check if the server is running
   * @return true if running, false otherwise
   */
  bool is_running() const;

protected:
  /**
   * @brief Parse DNS query and generate response
   * @param query The DNS query packet
   * @param query_len Length of the query
   * @param response Buffer to write the response to
   * @param response_len Length of the response buffer
   * @return Number of bytes written to response buffer
   */
  size_t process_dns_query(const uint8_t *query, size_t query_len, uint8_t *response,
                           size_t response_len);

  std::string ip_address_;
  std::unique_ptr<UdpSocket> socket_;
  bool running_{false};
};
} // namespace espp

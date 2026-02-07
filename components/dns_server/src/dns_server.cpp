#include "dns_server.hpp"

#include <arpa/inet.h>
#include <cstring>

using namespace espp;

DnsServer::DnsServer(const Config &config)
    : BaseComponent("DnsServer", config.log_level)
    , ip_address_(config.ip_address) {
  logger_.info("DNS Server initialized, will respond with IP: {}", ip_address_);
}

DnsServer::~DnsServer() { stop(); }

bool DnsServer::start(std::error_code &ec) {
  if (running_) {
    logger_.warn("DNS server already running");
    ec = std::make_error_code(std::errc::device_or_resource_busy);
    return false;
  }

  // Validate IP address before starting
  in_addr addr;
  if (inet_pton(AF_INET, ip_address_.c_str(), &addr) != 1) {
    logger_.error("Invalid IP address: {}", ip_address_);
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }

  logger_.info("Starting DNS server on port 53");

  // Create UDP socket for DNS
  UdpSocket::Config socket_config{.log_level = get_log_level()};
  socket_ = std::make_unique<UdpSocket>(socket_config);

  // Configure task for receiving
  Task::BaseConfig task_config{
      .name = "dns_server", .stack_size_bytes = 4096, .priority = 5, .core_id = 0};

  // Configure receive settings
  UdpSocket::ReceiveConfig receive_config{
      .port = 53,         // DNS port
      .buffer_size = 512, // Standard DNS packet size
      .is_multicast_endpoint = false,
      .on_receive_callback = [this](auto &data,
                                    auto &source) -> std::optional<std::vector<uint8_t>> {
        logger_.debug("Received DNS query from {}:{}, {} bytes", source.address, source.port,
                      data.size());

        // Process DNS query and generate response
        std::vector<uint8_t> response(512);
        size_t response_len =
            process_dns_query(data.data(), data.size(), response.data(), response.size());

        if (response_len > 0) {
          response.resize(response_len);
          logger_.debug("Sending DNS response, {} bytes", response_len);
          return response; // Return response to be sent back
        }

        return std::nullopt; // No response
      }};

  if (!socket_->start_receiving(task_config, receive_config)) {
    logger_.error("Failed to start DNS server");
    socket_.reset();
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }

  running_ = true;
  logger_.info("DNS server started successfully");
  ec.clear();
  return true;
}

void DnsServer::stop() {
  if (!running_) {
    return;
  }

  logger_.info("Stopping DNS server");
  running_ = false;
  socket_.reset();
  logger_.info("DNS server stopped");
}

bool DnsServer::is_running() const { return running_; }

size_t DnsServer::process_dns_query(const uint8_t *query, size_t query_len, uint8_t *response,
                                    size_t response_len) {
  if (query_len < 12) {
    logger_.error("Invalid DNS packet size (too small)");
    return 0;
  }

  if (response_len < 512) {
    logger_.error("Response buffer too small");
    return 0;
  }

  // DNS header structure
  struct __attribute__((packed)) DnsHeader {
    uint16_t id;
    uint16_t flags;
    uint16_t qdcount;
    uint16_t ancount;
    uint16_t nscount;
    uint16_t arcount;
  };

  const DnsHeader *query_header = reinterpret_cast<const DnsHeader *>(query);
  DnsHeader *response_header = reinterpret_cast<DnsHeader *>(response);

  // Bounds check: ensure we don't copy more than response buffer size
  if (query_len > response_len) {
    logger_.error("Query too large for response buffer ({} > {})", query_len, response_len);
    return 0;
  }

  // Copy query to response
  std::memcpy(response, query, query_len);

  // Set response flags: standard query response, no error
  uint16_t flags = ntohs(query_header->flags);
  flags |= 0x8000; // Set response bit
  flags &= ~0x0F;  // Clear error code
  response_header->flags = htons(flags);

  // Set answer count to 1
  response_header->ancount = htons(1);
  response_header->nscount = 0;
  response_header->arcount = 0;

  // Find end of question section
  size_t pos = 12; // After header

  // Skip domain name in question
  while (pos < query_len && query[pos] != 0) {
    uint8_t len = query[pos];
    if (len > 63) { // Check for compression or invalid length
      logger_.error("Invalid domain name format");
      return 0;
    }
    pos += len + 1;
  }
  pos++; // Skip null terminator

  // Skip QTYPE and QCLASS (4 bytes)
  pos += 4;

  if (pos > query_len) {
    logger_.error("Malformed DNS query");
    return 0;
  }

  // Build answer section
  size_t answer_start = pos;

  // Calculate required space for answer section (16 bytes total)
  const size_t answer_size = 16;
  if (answer_start + answer_size > response_len) {
    logger_.error("Not enough space for DNS answer ({} + {} > {})", answer_start, answer_size,
                  response_len);
    return 0;
  }

  // Name pointer (points back to question name)
  response[answer_start++] = 0xC0;
  response[answer_start++] = 0x0C;

  // Type: A record
  response[answer_start++] = 0x00;
  response[answer_start++] = 0x01;

  // Class: IN
  response[answer_start++] = 0x00;
  response[answer_start++] = 0x01;

  // TTL: 60 seconds
  response[answer_start++] = 0x00;
  response[answer_start++] = 0x00;
  response[answer_start++] = 0x00;
  response[answer_start++] = 0x3C;

  // Data length: 4 bytes (IPv4 address)
  response[answer_start++] = 0x00;
  response[answer_start++] = 0x04;

  // Parse IP address (validated in constructor, but check again for safety)
  in_addr addr;
  if (inet_pton(AF_INET, ip_address_.c_str(), &addr) != 1) {
    logger_.error("Failed to parse IP address: {}", ip_address_);
    return 0;
  }

  // Copy IP address bytes
  std::memcpy(&response[answer_start], &addr.s_addr, 4);
  answer_start += 4;

  logger_.debug("DNS response prepared, {} bytes", answer_start);
  return answer_start;
}

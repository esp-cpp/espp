#pragma once

#include "socket_msvc.hpp"

#ifdef _MSC_VER
typedef unsigned int sock_type_t;
#else
/* Assume that any non-Windows platform uses POSIX-style sockets instead. */
#include <arpa/inet.h>
#include <netdb.h> /* Needed for getaddrinfo() and freeaddrinfo() */
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h> /* Needed for close() */
typedef int sock_type_t;
#endif

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <math.h>

#include "base_component.hpp"
#include "format.hpp"

namespace espp {
/**
 *   @brief Class for a generic socket with some helper functions for
 *          configuring the socket.
 */
class Socket : public BaseComponent {
public:
  enum class Type : int {
    RAW = SOCK_RAW,      /**< Only IP headers, no TCP or UDP headers as well. */
    DGRAM = SOCK_DGRAM,  /**< UDP/IP socket - datagram. */
    STREAM = SOCK_STREAM /**< TCP/IP socket - stream. */
  };

  /**
   *  @brief Storage for socket information (address, port) with convenience
   *         functions to convert to/from POSIX structures.
   */
  struct Info {
  protected:
    struct sockaddr_storage raw; /**< Raw sockaddr structure, allows this
                                    structure to contain all necessary socket
                                    information. */
  public:
    std::string address; /**< IP address of the endpoint as a string. */
    size_t port;         /**< Port of the endpoint as an integer. */

    /**
     * @brief Initialize the struct as an ipv4 address/port combo.
     * @param addr IPv4 address string
     * @param prt port number
     */
    void init_ipv4(const std::string &addr, size_t prt);

    /**
     * @brief Gives access to IPv4 sockaddr structure (sockaddr_in) for use
     *        with low level socket calls like sendto / recvfrom.
     * @return *sockaddr_in pointer to ipv4 data structure
     */
    struct sockaddr_in *ipv4_ptr();

    /**
     * @brief Gives access to IPv6 sockaddr structure (sockaddr_in6) for use
     *        with low level socket calls like sendto / recvfrom.
     * @return *sockaddr_in6 pointer to ipv6 data structure
     */
    struct sockaddr_in6 *ipv6_ptr();

    /**
     * @brief Will update address and port based on the curent data in raw.
     */
    void update();

    /**
     * @brief Fill this Info from the provided sockaddr struct.
     * @param &source_address sockaddr info filled out by recvfrom.
     */
    void from_sockaddr(const struct sockaddr_storage &source_address);

    /**
     * @brief Fill this Info from the provided sockaddr struct.
     * @param &source_address sockaddr info filled out by recvfrom.
     */
    void from_sockaddr(const struct sockaddr_in &source_address);

    /**
     * @brief Fill this Info from the provided sockaddr struct.
     * @param &source_address sockaddr info filled out by recvfrom.
     */
    void from_sockaddr(const struct sockaddr_in6 &source_address);
  };

  /**
   * @brief Callback function to be called when receiving data from a client.
   * @param data Byte array of data received from client
   * @param sender_info Sender information (address, port)
   * @return std::optional<std::vector<uint8_t>> optional data to return to sender.
   */
  typedef std::function<std::optional<std::vector<uint8_t>>(std::vector<uint8_t> &data,
                                                            const Info &sender_info)>
      receive_callback_fn;

  /**
   * @brief Callback function to be called with data returned after transmitting data to a server.
   * @param data The data that the server responded with
   */
  typedef std::function<void(std::vector<uint8_t> &data)> response_callback_fn;

  /**
   * @brief Construct the socket, setting its internal socket file descriptor.
   * @note This constructor does not check the validity of the socket file
   *       descriptor.
   * @param socket_fd Socket file descriptor.
   * @param logger_config configuration for the logger associated with the
   *        socket.
   */
  explicit Socket(sock_type_t socket_fd, const espp::Logger::Config &logger_config);

  /**
   * @brief Initialize the socket (calling init()).
   * @param type The Socket::Type of the socket to make.
   * @param logger_config configuration for the logger associated with the
   *        socket.
   */
  explicit Socket(Type type, const espp::Logger::Config &logger_config);

  /**
   * @brief Tear down any resources associted with the socket.
   */
  ~Socket();

  /**
   * @brief Is the socket valid.
   * @return true if the socket file descriptor is >= 0.
   */
  bool is_valid() const;

  /**
   * @brief Is the socket valid.
   * @param socket_fd Socket file descriptor.
   * @return true if the socket file descriptor is >= 0.
   */
  static bool is_valid_fd(sock_type_t socket_fd);

  /**
   * @brief Get the Socket::Info for the socket.
   * @details This will call getsockname() on the socket to get the
   *          sockaddr_storage structure, and then fill out the Socket::Info
   *          structure.
   * @return Socket::Info for the socket.
   */
  std::optional<Info> get_ipv4_info();

  /**
   * @brief Set the receive timeout on the provided socket.
   * @param timeout requested timeout, must be > 0.
   * @return true if SO_RECVTIMEO was successfully set.
   */
  bool set_receive_timeout(const std::chrono::duration<float> &timeout);

  /**
   * @brief Allow others to use this address/port combination after we're done
   *        with it.
   * @return true if SO_REUSEADDR and SO_REUSEPORT were successfully set.
   */
  bool enable_reuse();

  /**
   * @brief Configure the socket to be multicast (if time_to_live > 0).
   *        Sets the IP_MULTICAST_TTL (number of multicast hops allowed) and
   *        optionally configures whether this node should receive its own
   *        multicast packets (IP_MULTICAST_LOOP).
   * @param time_to_live number of multicast hops allowed (TTL).
   * @param loopback_enabled Whether to receive our own multicast packets.
   * @return true if IP_MULTICAST_TTL and IP_MULTICAST_LOOP were set.
   */
  bool make_multicast(uint8_t time_to_live = 1, uint8_t loopback_enabled = true);

  /**
   * @brief If this is a server socket, add it to the provided the multicast
   *        group.
   *
   *         @note Multicast groups must be Class D addresses (224.0.0.0 to
   *                239.255.255.255)
   *
   *        See https://en.wikipedia.org/wiki/Multicast_address for more
   *        information.
   * @param multicast_group multicast group to join.
   * @return true if IP_ADD_MEMBERSHIP was successfully set.
   */
  bool add_multicast_group(const std::string &multicast_group);

  /**
   * @brief Select on the socket for read events.
   * @param timeout how long to wait for an event.
   * @return number of events that occurred.
   */
  int select(const std::chrono::microseconds &timeout);

protected:
  /**
   * @brief Create the TCP socket and enable reuse.
   * @return true if the socket was initialized properly, false otherwise
   */
  bool init(Type type);

  /**
   * @brief Get the string representation of the last error that occurred.
   * @return string representation of the last error that occurred.
   */
  std::string error_string() const;

  /**
   * @brief Get the string representation of the last error that occurred.
   * @param error error code to get the string representation of.
   * @return string representation of the last error that occurred.
   */
  std::string error_string(int error) const;

  /**
   *  @brief If the socket was created, we shut it down and close it here.
   */
  void cleanup();

  static constexpr int address_family_{AF_INET};
  static constexpr int ip_protocol_{IPPROTO_IP};

  sock_type_t socket_;
};
} // namespace espp

#include "socket_formatters.hpp"

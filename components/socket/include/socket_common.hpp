#pragma once

#include <string>
#include <vector>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <math.h>

#include "format.hpp"

namespace espp {
  class Socket {
  public:
    struct Info {
    protected:
      struct sockaddr_storage raw; /**< Raw sockaddr structure, allows this
                                      structure to contain all necessary socket
                                      information. */
    public:
      std::string address; /**< IP address of the endpoint as a string. */
      size_t port; /**< Port of the endpoint as an integer. */

      /**
       * @brief Initialize the struct as an ipv4 address/port combo.
       * @param addr IPv4 address string
       * @param prt port number
       */
      void init_ipv4(const std::string& addr, size_t prt) {
        address = addr;
        port = prt;
        auto server_address = ipv4_ptr();
        server_address->sin_family = AF_INET;
        server_address->sin_addr.s_addr = inet_addr(address.c_str());
        server_address->sin_port = htons(port);
      }

      /**
       * @brief Gives access to IPv4 sockaddr structure (sockaddr_in) for use
       *        with low level socket calls like sendto / recvfrom.
       * @return *sockaddr_in pointer to ipv4 data structure
       */
      struct sockaddr_in* ipv4_ptr() {
        return (struct sockaddr_in*)&raw;
      }

      /**
       * @brief Gives access to IPv6 sockaddr structure (sockaddr_in6) for use
       *        with low level socket calls like sendto / recvfrom.
       * @return *sockaddr_in6 pointer to ipv6 data structure
       */
      struct sockaddr_in6* ipv6_ptr() {
        return (struct sockaddr_in6*)&raw;
      }

      /**
       * @brief Will update address and port based on the curent data in raw.
       */
      void update() {
        if (raw.ss_family == PF_INET) {
          address = inet_ntoa(((struct sockaddr_in *)&raw)->sin_addr);
          port = ((struct sockaddr_in *)&raw)->sin_port;
        } else if (raw.ss_family == PF_INET6) {
          address = inet_ntoa(((struct sockaddr_in6 *)&raw)->sin6_addr);
          port = ((struct sockaddr_in6 *)&raw)->sin6_port;
        }
      }

      /**
       * @brief Fill this Info from the provided sockaddr struct.
       * @param &source_address sockaddr info filled out by recvfrom.
       */
      void from_sockaddr(const struct sockaddr_storage &source_address) {
        memcpy(&raw, &source_address, sizeof(source_address));
        update();
      }

      /**
       * @brief Fill this Info from the provided sockaddr struct.
       * @param &source_address sockaddr info filled out by recvfrom.
       */
      void from_sockaddr(const struct sockaddr_in &source_address) {
        memcpy(&raw, &source_address, sizeof(source_address));
        address = inet_ntoa(source_address.sin_addr);
        port = source_address.sin_port;
      }

      /**
       * @brief Fill this Info from the provided sockaddr struct.
       * @param &source_address sockaddr info filled out by recvfrom.
       */
      void from_sockaddr(const struct sockaddr_in6 &source_address) {
        address = inet_ntoa(source_address.sin6_addr);
        port = source_address.sin6_port;
        memcpy(&raw, &source_address, sizeof(source_address));
      }

      /**
       * @brief Format the address and port into a string "{address}:{port}".
       * @return std::string contining the formatted info.
       */
      std::string to_string() const { return fmt::format("{}:{}", address, port); }
    };

    static bool is_valid(int socket_fd) {
      return socket_fd >= 0;
    }

    /**
     * @brief Set the receive timeout on the provided socket.
     * @param socket_fd socket file descriptor.
     * @param timeout requested timeout, must be > 0.
     * @return true if SO_RECVTIMEO was successfully set.
     */
    static bool set_receive_timeout(int socket_fd, const std::chrono::duration<float>& timeout) {
      float seconds = timeout.count();
      if (seconds <= 0) {
        return true;
      }
      float intpart;
      float fractpart = modf(seconds, &intpart);
      const time_t response_timeout_s = (int)intpart;
      const time_t response_timeout_us = (int)(fractpart * 1E6);
      //// Alternatively we could do this:
      // int microseconds = (int)(std::chrono::duration_cast<std::chrono::microseconds>(timeout).count()) % (int)1E6;
      // const time_t response_timeout_s = floor(seconds);
      // const time_t response_timeout_us = microseconds;

      struct timeval tv;
      tv.tv_sec = response_timeout_s;
      tv.tv_usec = response_timeout_us;
      int err = setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof (tv));
      if (err < 0) {
        return false;
      }
      return true;
    }

    /**
     * @brief Allow others to use this address/port combination after we're done
     *        with it.
     * @param socket_fd socket file descriptor
     * @return true if SO_REUSEADDR and SO_REUSEPORT were successfully set.
     */
    static bool enable_reuse(int socket_fd) {
#if !CONFIG_LWIP_SO_REUSE && defined(ESP_PLATFORM)
      fmt::print(fg(fmt::color::red), "CONFIG_LWIP_SO_REUSE not defined!\n");
      return false;
#endif
      int err = 0;
      int enabled = 1;
      err = setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));
      if (err < 0) {
        fmt::print(fg(fmt::color::red), "Couldn't set SO_REUSEADDR\n");
        return false;
      }
#if !defined(ESP_PLATFORM)
      err = setsockopt(socket_fd, SOL_SOCKET, SO_REUSEPORT, &enabled, sizeof(enabled));
      if (err < 0) {
        fmt::print(fg(fmt::color::red), "Couldn't set SO_REUSEPORT\n");
        return false;
      }
#endif
      return true;
    }

    /**
     * @brief Configure the socket to be multicast (if time_to_live > 0).
     *        Sets the IP_MULTICAST_TTL (number of multicast hops allowed) and
     *        optionally configures whether this node should receive its own
     *        multicast packets (IP_MULTICAST_LOOP).
     * @param socket_fd socket file descriptor.
     * @param time_to_live number of multicast hops allowed (TTL).
     * @param loopback_enabled Whether to receive our own multicast packets.
     * @return true if IP_MULTICAST_TTL and IP_MULTICAST_LOOP were set.
     */
    static bool make_multicast(int socket_fd, uint8_t time_to_live=1, uint8_t loopback_enabled=true) {
      int err = 0;
      // Assign multicast TTL - separate from normal interface TTL
      err = setsockopt(socket_fd, IPPROTO_IP, IP_MULTICAST_TTL,
                       &time_to_live, sizeof(uint8_t));
      if (err < 0) {
        fmt::print(fg(fmt::color::red), "Couldn't set IP_MULTICAST_TTL\n");
        return false;
      }
      // select whether multicast traffic should be received by this device, too
      err = setsockopt(socket_fd, IPPROTO_IP, IP_MULTICAST_LOOP,
                       &loopback_enabled, sizeof(uint8_t));
      if (err < 0) {
        fmt::print(fg(fmt::color::red), "Couldn't set IP_MULTICAST_LOOP\n");
        return false;
      }
      return true;
    }

    /**
     * @brief If this is a server socket, add it to the provided the multicast
     *        group.
     *
     *          NOTE: multicast groups must be Class D addresses (224.0.0.0 to
     *                239.255.255.255)
     *
     *        See https://en.wikipedia.org/wiki/Multicast_address for more
     *        information.
     * @param socket_fd socket file descriptor
     * @param multicast_group multicast group to join.
     * @return true if IP_ADD_MEMBERSHIP was successfully set.
     */
    static bool add_multicast_group(int socket_fd, const std::string& multicast_group) {
      struct ip_mreq imreq;
      int err = 0;

      // Configure source interface
      imreq.imr_interface.s_addr = IPADDR_ANY;
      // Configure multicast address to listen to
      err = inet_aton(multicast_group.c_str(), &imreq.imr_multiaddr.s_addr);

      if (err != 1 || !IN_MULTICAST(ntohl(imreq.imr_multiaddr.s_addr))) {
        // it's not actually a multicast address, so return false?
        fmt::print(fg(fmt::color::red), "Not a valid multicast address ({})\n", multicast_group);
        return false;
      }

      // Assign the IPv4 multicast source interface, via its IP
      // (only necessary if this socket is IPV4 only)
      struct in_addr iaddr;
      err = setsockopt(socket_fd, IPPROTO_IP, IP_MULTICAST_IF, &iaddr,
                       sizeof(struct in_addr));
      if (err < 0) {
        fmt::print(fg(fmt::color::red), "Couldn't set IP_MULTICAST_IF: {} - '{}'\n", errno, strerror(errno));
        return false;
      }

      err = setsockopt(socket_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                       &imreq, sizeof(struct ip_mreq));
      if (err < 0) {
        fmt::print(fg(fmt::color::red), "Couldn't set IP_ADD_MEMBERSHIP: {} - '{}'\n", errno, strerror(errno));
        return false;
      }

      return true;
    }
  };
}

#pragma once

#include <sdkconfig.h>

#include <iperf.h>

#include "format.hpp"

// for libfmt formatting of iperf_cfg_t
template <> struct fmt::formatter<iperf_cfg_t> : fmt::formatter<std::string> {
  template <typename FormatContext> auto format(const iperf_cfg_t &cfg, FormatContext &ctx) const {
    if (cfg.flag & IPERF_FLAG_CLIENT) {
      return fmt::format_to(ctx.out(),
                            "iperf_cfg_t client {{"
                            "  host: {},"
                            "  dest port: {},"
                            "  source port: {},"
                            "  protocol: {},"
                            "  duration: {},"
                            "  bandwidth: {},"
                            "  buffer_size: {}"
                            " }}",
                            cfg.destination_ip4, cfg.dport, cfg.sport,
                            cfg.flag & IPERF_FLAG_UDP ? "UDP" : "TCP", cfg.time, cfg.bw_lim,
                            cfg.len_send_buf);
    } else {
      return fmt::format_to(ctx.out(),
                            "iperf_cfg_t server {{"
                            "  dest port: {},"
                            "  source port: {},"
                            "  protocol: {},"
                            "  duration: {},"
                            "  bandwidth: {},"
                            "  buffer_size: {}"
                            " }}",
                            cfg.dport, cfg.sport, cfg.flag & IPERF_FLAG_UDP ? "UDP" : "TCP",
                            cfg.time, cfg.bw_lim, cfg.len_send_buf);
    }
  }
};

#if CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)

#include <functional>
#include <memory>
#include <vector>

#include "cli.hpp"
#include "format.hpp"

namespace espp {
/// @brief A CLI menu for running iperf tests.
/// @details
///
/// \section iperf_menu_ex1 Example
/// \snippet iperf_menu_example.cpp iperf menu example
class IperfMenu {
public:
  /// @brief Construct a new IperfMenu object.
  explicit IperfMenu() {
    // set the default configs
    cfg_.type = IPERF_IP_TYPE_IPV4;  // Default to IPv4
    cfg_.flag = IPERF_FLAG_TCP;      // Default to TCP
    cfg_.sport = IPERF_DEFAULT_PORT; // Default source port
    cfg_.dport = IPERF_DEFAULT_PORT; // Default destination port
    cfg_.len_send_buf = 0;
    cfg_.interval = IPERF_DEFAULT_INTERVAL;  // Default interval
    cfg_.time = IPERF_DEFAULT_TIME;          // Default time
    cfg_.bw_lim = IPERF_DEFAULT_NO_BW_LIMIT; // No bandwidth limit by default
    cfg_.format = KBITS_PER_SEC;             // Default output format is kibits/sec
  }

  /// @brief Get the CLI menu for the Iperf functionality.
  /// @param name The name of the menu.
  /// @param description The description of the menu.
  /// @return A unique pointer to the Iperf menu that you can use to add to a
  ///         CLI.
  std::unique_ptr<cli::Menu> get(std::string_view name = "iperf",
                                 std::string_view description = "Iperf menu") {
    auto menu = std::make_unique<cli::Menu>(std::string(name), std::string(description));

    // iperf options are:
    // - "client": Run as a client, connecting to a <host>
    // - "server": Run as a server
    // - "udp": Run in UDP mode, default is TCP
    // - "port": Set the port to use, default is 5000
    // - "length": Set the length of the buffer to use, default is IPERF_DEFAULT_TCP_TX_LEN for TCP,
    // or IPERF_DEFAULT_IPV4_UDP_TX_LEN / IPERF_DEFAULT_IPV6_UDP_TX_LEN for UDP
    // - "interval": Set the interval for reporting results, default is 1 second
    // (IPERF_DEFAULT_INTERVAL)
    // - "time": Set the time to run the test, default is 10 seconds (IPERF_DEFAULT_TIME)
    // - "bandwidth": Set the bandwidth limit to use for UDP tests, or <= 0 for no limit
    // (IPERF_DEFAULT_NO_BW_LIMIT)
    // - "format": Set the output format to <format>, 'k' = kibts/sec, 'm' = mbits/sec
    //
    // If IPV6 is enabled, the following options are also available:
    // - "ipv6": Run the test using IPv6
    //
    // Some non standard options:
    // - "abort": Abort the current iperf test, if running

    // Since we're making our own CLI for this, we'll have a few high-level menu options:
    // - client (udp, tcp)
    // - server (udp, tcp)
    // - abort
    //
    // The client / server will have various overloads for different parameters,
    // and the abort will just stop the current test if running.

    //////////////////
    //// Configuration
    //////////////////
    menu->Insert(
        "udp", [this](std::ostream &out) { get_udp(out); }, "Get the current UDP mode setting.");
    menu->Insert(
        "udp", {"udp"}, [this](std::ostream &out, bool udp) { set_udp(out, udp); },
        "Set the UDP mode for iperf tests.");
    menu->Insert(
        "tcp", [this](std::ostream &out) { get_udp(out); }, "Get the current TCP mode setting. ");
    menu->Insert(
        "tcp", {"tcp"}, [this](std::ostream &out, bool tcp) { set_udp(out, !tcp); },
        "Set the TCP mode for iperf tests. This will disable UDP mode.");
#if IPERF_IPV6_ENABLED || defined(_DOXYGEN_)
    menu->Insert(
        "ipv6", [this](std::ostream &out) { get_ipv6(out); }, "Get the current IPv6 setting.");
    menu->Insert(
        "ipv6", {"ipv6"}, [this](std::ostream &out, bool ipv6) { set_ipv6(out, ipv6); },
        "Set the IPv6 mode for iperf tests.");
#endif // IPERF_IPV6_ENABLED
    menu->Insert(
        "interval", [this](std::ostream &out) { get_interval(out); },
        "Get the current reporting interval setting.");
    menu->Insert(
        "interval", {"interval"},
        [this](std::ostream &out, int interval) { set_interval(out, interval); },
        "Set the reporting interval for iperf tests.");
    menu->Insert(
        "time", [this](std::ostream &out) { get_time(out); },
        "Get the current time (duration) setting.");
    menu->Insert(
        "time", {"time"}, [this](std::ostream &out, int time) { set_time(out, time); },
        "Set the time for iperf tests.");
    menu->Insert(
        "format", [this](std::ostream &out) { get_format(out); },
        "Get the current formatting setting (kibits/sec or mbits/sec).");
    menu->Insert(
        "format", {"format"}, [this](std::ostream &out, char format) { set_format(out, format); },
        "Set the output format for iperf tests.");
    menu->Insert(
        "length", [this](std::ostream &out) { get_length(out); },
        "Get the current buffer length setting.");
    menu->Insert(
        "length", {"length"}, [this](std::ostream &out, int length) { set_length(out, length); },
        "Set the buffer length for iperf tests.");

    //////////////////
    //// Abort
    //////////////////
    menu->Insert(
        "abort", {}, [this](std::ostream &out) { abort(out); },
        "Abort the current iperf test if running. This will stop the test and print a message.");

    //////////////////
    //// Client
    //////////////////
    menu->Insert(
        "client", {"host"},
        [this](std::ostream &out, const std::string &host) { iperf_client(out, host); },
        "Run an iperf client, connecting to the specified host, using the default port.");
    menu->Insert(
        "client", {"host", "port"},
        [this](std::ostream &out, const std::string &host, int port) {
          iperf_client(out, host, port);
        },
        "Run an iperf client, connecting to the specified host and port.");
    menu->Insert(
        "client", {"host", "port", "bandwidth"},
        [this](std::ostream &out, const std::string &host, int port, int length) {
          iperf_client(out, host, port, length);
        },
        "Run an iperf client, connecting to the specified host and port. Uses the "
        "passed length as the buffer length for data transmission.");

    //////////////////
    //// Server
    //////////////////
    menu->Insert(
        "server", [this](std::ostream &out) { iperf_server(out); },
        "Run an iperf server, listening on the default port.");
    menu->Insert(
        "server", {"port"}, [this](std::ostream &out, int port) { iperf_server(out, port); },
        "Run an iperf server, listening on the specified port.");

    return menu;
  }

  /// @brief Abort the current iperf test if running.
  /// @param out The output stream to write the result to.
  void abort(std::ostream &out) {
    iperf_stop();
    out << "Iperf test aborted.\n";
  }

#if IPERF_IPV6_ENABLED || defined(_DOXYGEN_)
  /// @brief Get the current IPv6 setting.
  /// @param out The output stream to write the result to.
  /// @note This function is only available if IPERF_IPV6_ENABLED is defined.
  void get_ipv6(std::ostream &out) const {
    out << (use_ipv6_ ? "Using IPv6 for iperf tests.\n" : "Using IPv4 for iperf tests.\n");
  }

  /// @brief Set the IPv6 mode for iperf tests.
  /// @param out The output stream to write the result to.
  /// @param ipv6 True to use IPv6 mode, false for IPv4 mode.
  /// @note This function is only available if IPERF_IPV6_ENABLED is defined.
  void set_ipv6(std::ostream &out, bool ipv6) {
    use_ipv6_ = ipv6;
    cfg_.type = ipv6 ? IPERF_IP_TYPE_IPV6 : IPERF_IP_TYPE_IPV4;
    out << (use_ipv6_ ? "Switched to IPv6 for iperf tests.\n"
                      : "Switched to IPv4 for iperf tests.\n");
  }
#endif // IPERF_IPV6_ENABLED

  /// @brief Get the current UDP mode setting.
  /// @param out The output stream to write the result to.
  void get_udp(std::ostream &out) const {
    out << (use_udp_ ? "Using UDP mode for iperf tests.\n" : "Using TCP mode for iperf tests.\n");
  }

  /// @brief Set the UDP mode for iperf tests.
  /// @param out The output stream to write the result to.
  /// @param udp True to use UDP mode, false for TCP mode.
  void set_udp(std::ostream &out, bool udp) {
    use_udp_ = udp;
    if (use_udp_) {
      cfg_.flag |= IPERF_FLAG_UDP;
      cfg_.flag &= ~IPERF_FLAG_TCP; // Ensure TCP flag is cleared
    } else {
      cfg_.flag |= IPERF_FLAG_TCP;
      cfg_.flag &= ~IPERF_FLAG_UDP; // Ensure UDP flag is cleared
    }
    out << (use_udp_ ? "Switched to UDP mode for iperf tests.\n"
                     : "Switched to TCP mode for iperf tests.\n");
  }

  /// @brief Get the current reporting interval setting.
  /// @param out The output stream to write the result to.
  void get_interval(std::ostream &out) const {
    out << "Current interval for iperf tests: " << interval_ << " seconds.\n";
  }

  /// @brief Set the reporting interval for iperf tests.
  /// @param out The output stream to write the result to.
  /// @param interval The interval in seconds to set.
  void set_interval(std::ostream &out, int interval) {
    if (interval <= 0) {
      out << "Invalid interval specified. Using default interval of " << IPERF_DEFAULT_INTERVAL
          << " seconds.\n";
      interval = IPERF_DEFAULT_INTERVAL;
    }
    interval_ = interval;
    cfg_.interval = interval;
    out << "Set the interval for iperf tests to " << interval_ << " seconds.\n";
  }

  /// @brief Get the current time (duration) setting.
  /// @param out The output stream to write the result to.
  void get_time(std::ostream &out) const {
    out << "Current time for iperf tests: " << time_ << " seconds.\n";
  }

  /// @brief Set the time for iperf tests.
  /// @param out The output stream to write the result to.
  /// @param time The time in seconds to set.
  void set_time(std::ostream &out, int time) {
    if (time <= 0) {
      out << "Invalid time specified. Using default time of " << IPERF_DEFAULT_TIME
          << " seconds.\n";
      time = IPERF_DEFAULT_TIME;
    }
    time_ = time;
    cfg_.time = time;
    out << "Set the time for iperf tests to " << time_ << " seconds.\n";
  }

  /// @brief Get the current formatting setting (kibits/sec or mbits/sec)
  /// @param out The output stream to write the result to.
  void get_format(std::ostream &out) const {
    out << "Current output format for iperf tests: " << format_ << ".\n";
  }

  /// @brief Set the output format for iperf tests.
  /// @param out The output stream to write the result to.
  /// @param format The format character to set ('k' for kibits/sec, 'm' for mbits/sec).
  void set_format(std::ostream &out, char format) {
    switch (format) {
    case 'k':
      format_ = 'k';
      cfg_.format = KBITS_PER_SEC; // kibits/sec
      break;
    case 'm':
      format_ = 'm';
      cfg_.format = MBITS_PER_SEC; // mbits/sec
      break;
    default:
      out << "Invalid format specified. Using kibits/sec by default.\n";
      format_ = 'k';
      cfg_.format = KBITS_PER_SEC; // Default to kibits/sec
      break;
    }
    out << "Set the output format for iperf tests to " << format_ << ".\n";
  }

  /// @brief Get the current buffer length setting.
  /// @param out The output stream to write the result to.
  void get_length(std::ostream &out) const {
    out << "Current buffer length for iperf tests: " << length_ << " bytes.\n";
  }

  /// @brief Set the buffer length for iperf tests.
  /// @param out The output stream to write the result to.
  /// @param length The length in bytes to set.
  void set_length(std::ostream &out, int length) {
    length_ = length;
    cfg_.len_send_buf = length;
    out << "Set the buffer length for iperf tests to " << length_ << " bytes.\n";
  }

  /// @brief Get the current bandwidth limit setting.
  void iperf_client(std::ostream &out, const std::string &host, int port = 0,
                    int bandwidth = IPERF_DEFAULT_NO_BW_LIMIT) {
    // set client flag
    cfg_.flag |= IPERF_FLAG_CLIENT;
    // ensure server flag is cleared
    cfg_.flag &= ~IPERF_FLAG_SERVER;

    // set the destination host
    host_ = host;
#if IPERF_IPV6_ENABLED
    if (use_ipv6_) {
      cfg_.destination_ip6 = (char *)host_.c_str(); // Set the IPv6 host
    } else {
      cfg_.destination_ip4 = esp_ip4addr_aton(host_.c_str()); // Set the IPv4 host
    }
#else
    cfg_.destination_ip4 = esp_ip4addr_aton(host_.c_str());
#endif

    // set the port
    if (port == 0) {
      cfg_.sport = IPERF_DEFAULT_PORT; // Default port
      cfg_.dport = IPERF_DEFAULT_PORT; // Default port
    } else {
      // NOTE: This is specifically for the client
      cfg_.sport = IPERF_DEFAULT_PORT; // Default source port
      cfg_.dport = port;               // Use specified destination port
    }

    // set the bandwidth limit for UDP tests
    cfg_.bw_lim = bandwidth;

    // print the configuration
    out << fmt::format("Using iperf config: {}\n", cfg_);

    // now start the client
    iperf_start(&cfg_);
  }

  void iperf_server(std::ostream &out, int port = 0) {
    // Set the server flag
    cfg_.flag |= IPERF_FLAG_SERVER;
    // ensure the client flag is cleared
    cfg_.flag &= IPERF_FLAG_CLIENT;

    // set the port
    if (port == 0) {
      cfg_.sport = IPERF_DEFAULT_PORT; // Default port
      cfg_.dport = IPERF_DEFAULT_PORT; // Default port
    } else {
      // NOTE: This is specifically for the server
      cfg_.sport = port;               // Use specified destination port
      cfg_.dport = IPERF_DEFAULT_PORT; // Default source port
    }

    // print the configuration
    out << fmt::format("Using iperf config: {}\n", cfg_);

    // now start the server
    iperf_start(&cfg_);
  }

protected:
  iperf_cfg_t cfg_{};

  std::string host_{""};
  bool use_ipv6_{false};
  bool use_udp_{false};
  int interval_{IPERF_DEFAULT_INTERVAL};
  int time_{IPERF_DEFAULT_TIME};
  char format_{'k'};
  int length_{0};
};
} // namespace espp

#endif // CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)

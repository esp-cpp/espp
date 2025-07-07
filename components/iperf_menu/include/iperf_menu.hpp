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
  explicit IperfMenu() {}

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
    // - "port": Set the port to use, default is 5201 (IPERF_DEFAULT_PORT)
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
    //// Abort
    //////////////////
    menu->Insert(
        "abort", {}, [this](std::ostream &out) { abort(out); },
        "Abort the current iperf test if running. This will stop the test and print a message.");

    //////////////////
    //// Client UDP
    //////////////////
    menu->Insert(
        "client_udp", {"host"},
        [this](std::ostream &out, const std::string &host) {
          iperf_client(out, host, true, false);
        },
        "Run an iperf client in UDP mode, connecting to the specified host.");
    menu->Insert(
        "client_udp", {"host", "port"},
        [this](std::ostream &out, const std::string &host, int port) {
          iperf_client(out, host, true, false, port);
        },
        "Run an iperf client in UDP mode, connecting to the specified host and port.");
    menu->Insert(
        "client_udp", {"host", "port", "length"},
        [this](std::ostream &out, const std::string &host, int port, int length) {
          iperf_client(out, host, true, false, port, length);
        },
        "Run an iperf client in UDP mode, connecting to the specified host and port with a "
        "specific length.");
    menu->Insert(
        "client_udp", {"host", "port", "length", "interval"},
        [this](std::ostream &out, const std::string &host, int port, int length, int interval) {
          iperf_client(out, host, true, false, port, length, interval);
        },
        "Run an iperf client in UDP mode, connecting to the specified host and port with a "
        "specific length and interval.");
    menu->Insert(
        "client_udp", {"host", "port", "length", "interval", "time"},
        [this](std::ostream &out, const std::string &host, int port, int length, int interval,
               int time) { iperf_client(out, host, true, false, port, length, interval, time); },
        "Run an iperf client in UDP mode, connecting to the specified host and port with a "
        "specific length, interval, and time.");
    menu->Insert(
        "client_udp", {"host", "port", "length", "interval", "time", "bandwidth"},
        [this](std::ostream &out, const std::string &host, int port, int length, int interval,
               int time, int bandwidth) {
          iperf_client(out, host, true, false, port, length, interval, time, bandwidth);
        },
        "Run an iperf client in UDP mode, connecting to the specified host and port with a "
        "specific length, interval, time, and bandwidth.");
    menu->Insert(
        "client_udp", {"host", "port", "length", "interval", "time", "bandwidth", "format"},
        [this](std::ostream &out, const std::string &host, int port, int length, int interval,
               int time, int bandwidth, char format) {
          iperf_client(out, host, true, false, port, length, interval, time, bandwidth, format);
        },
        "Run an iperf client in UDP mode, connecting to the specified host and port with a "
        "specific length, interval, time, bandwidth, and output format.");

    //////////////////
    //// Client TCP
    //////////////////
    menu->Insert(
        "client_tcp", {"host"},
        [this](std::ostream &out, const std::string &host) {
          iperf_client(out, host, false, false);
        },
        "Run an iperf client in TCP mode, connecting to the specified host.");
    menu->Insert(
        "client_tcp", {"host", "port"},
        [this](std::ostream &out, const std::string &host, int port) {
          iperf_client(out, host, false, false, port);
        },
        "Run an iperf client in TCP mode, connecting to the specified host and port.");
    menu->Insert(
        "client_tcp", {"host", "port", "length"},
        [this](std::ostream &out, const std::string &host, int port, int length) {
          iperf_client(out, host, false, false, port, length);
        },
        "Run an iperf client in TCP mode, connecting to the specified host and port with a "
        "specific length.");
    menu->Insert(
        "client_tcp", {"host", "port", "length", "interval"},
        [this](std::ostream &out, const std::string &host, int port, int length, int interval) {
          iperf_client(out, host, false, false, port, length, interval);
        },
        "Run an iperf client in TCP mode, connecting to the specified host and port with a "
        "specific length and interval.");
    menu->Insert(
        "client_tcp", {"host", "port", "length", "interval", "time"},
        [this](std::ostream &out, const std::string &host, int port, int length, int interval,
               int time) { iperf_client(out, host, false, false, port, length, interval, time); },
        "Run an iperf client in TCP mode, connecting to the specified host and port with a "
        "specific length, interval, and time.");
    menu->Insert(
        "client_tcp", {"host", "port", "length", "interval", "time", "bandwidth"},
        [this](std::ostream &out, const std::string &host, int port, int length, int interval,
               int time, int bandwidth) {
          iperf_client(out, host, false, false, port, length, interval, time, bandwidth);
        },
        "Run an iperf client in TCP mode, connecting to the specified host and port with a "
        "specific length, interval, time, and bandwidth.");
    menu->Insert(
        "client_tcp", {"host", "port", "length", "interval", "time", "bandwidth", "format"},
        [this](std::ostream &out, const std::string &host, int port, int length, int interval,
               int time, int bandwidth, char format) {
          iperf_client(out, host, false, false, port, length, interval, time, bandwidth, format);
        },
        "Run an iperf client in TCP mode, connecting to the specified host and port with a "
        "specific length, interval, time, bandwidth, and output format.");

    //////////////////
    //// Server UDP
    //////////////////
    menu->Insert(
        "server_udp", {"port"},
        [this](std::ostream &out, int port) { iperf_server(out, true, false, port); },
        "Run an iperf server in UDP mode, listening on the specified port.");
    menu->Insert(
        "server_udp", {"port", "length"},
        [this](std::ostream &out, int port, int length) {
          iperf_server(out, true, false, port, length);
        },
        "Run an iperf server in UDP mode, listening on the specified port with a specific length.");
    menu->Insert(
        "server_udp", {"port", "length", "interval"},
        [this](std::ostream &out, int port, int length, int interval) {
          iperf_server(out, true, false, port, length, interval);
        },
        "Run an iperf server in UDP mode, listening on the specified port with a specific length "
        "and interval.");
    menu->Insert(
        "server_udp", {"port", "length", "interval", "time"},
        [this](std::ostream &out, int port, int length, int interval, int time) {
          iperf_server(out, true, false, port, length, interval, time);
        },
        "Run an iperf server in UDP mode, listening on the specified port with a specific length, "
        "interval, and time.");
    menu->Insert(
        "server_udp", {"port", "length", "interval", "time", "bandwidth"},
        [this](std::ostream &out, int port, int length, int interval, int time, int bandwidth) {
          iperf_server(out, true, false, port, length, interval, time, bandwidth);
        },
        "Run an iperf server in UDP mode, listening on the specified port with a specific length, "
        "interval, time, and bandwidth.");
    menu->Insert(
        "server_udp", {"port", "length", "interval", "time", "bandwidth", "format"},
        [this](std::ostream &out, int port, int length, int interval, int time, int bandwidth,
               char format) {
          iperf_server(out, true, false, port, length, interval, time, format);
        },
        "Run an iperf server in UDP mode, listening on the specified port with a specific length, "
        "interval, time, bandwidth, and output format.");

    //////////////////
    //// Server TCP
    //////////////////
    menu->Insert(
        "server_tcp", {"port"},
        [this](std::ostream &out, int port) { iperf_server(out, false, false, port); },
        "Run an iperf server in TCP mode, listening on the specified port.");
    menu->Insert(
        "server_tcp", {"port", "length"},
        [this](std::ostream &out, int port, int length) {
          iperf_server(out, false, false, port, length);
        },
        "Run an iperf server in TCP mode, listening on the specified port with a specific length.");
    menu->Insert(
        "server_tcp", {"port", "length", "interval"},
        [this](std::ostream &out, int port, int length, int interval) {
          iperf_server(out, false, false, port, length, interval);
        },
        "Run an iperf server in TCP mode, listening on the specified port with a specific length "
        "and interval.");
    menu->Insert(
        "server_tcp", {"port", "length", "interval", "time"},
        [this](std::ostream &out, int port, int length, int interval, int time) {
          iperf_server(out, false, false, port, length, interval, time);
        },
        "Run an iperf server in TCP mode, listening on the specified port with a specific length, "
        "interval, and time.");
    menu->Insert(
        "server_tcp", {"port", "length", "interval", "time", "bandwidth"},
        [this](std::ostream &out, int port, int length, int interval, int time, int bandwidth) {
          iperf_server(out, false, false, port, length, interval, time);
        },
        "Run an iperf server in TCP mode, listening on the specified port with a specific length, "
        "interval, time, and bandwidth.");
    menu->Insert(
        "server_tcp", {"port", "length", "interval", "time", "bandwidth", "format"},
        [this](std::ostream &out, int port, int length, int interval, int time, int bandwidth,
               char format) {
          iperf_server(out, false, false, port, length, interval, time, format);
        },
        "Run an iperf server in TCP mode, listening on the specified port with a specific length, "
        "interval, time, bandwidth, and output format.");

//////////////////
//// IPV6
//////////////////
#if IPERF_IPV6_ENABLED
    menu->Insert(
        "client_udp_ipv6", {"host", "port", "length", "interval", "time", "bandwidth", "format"},
        [this](std::ostream &out, const std::string &host, int port, int length, int interval,
               int time, int bandwidth, char format) {
          iperf_client(out, host, true, true, port, length, interval, time, bandwidth, format);
        },
        "Run an iperf client in UDP mode using IPv6, connecting to the specified host and port "
        "with a specific length, interval, time, bandwidth, and output format.");
    menu->Insert(
        "client_tcp_ipv6", {"host", "port", "length", "interval", "time", "bandwidth", "format"},
        [this](std::ostream &out, const std::string &host, int port, int length, int interval,
               int time, int bandwidth, char format) {
          iperf_client(out, host, false, true, port, length, interval, time, bandwidth, format);
        },
        "Run an iperf client in TCP mode using IPv6, connecting to the specified host and port "
        "with a specific length, interval, time, bandwidth, and output format.");
    menu->Insert(
        "server_udp_ipv6", {"port", "length", "interval", "time", "bandwidth", "format"},
        [this](std::ostream &out, int port, int length, int interval, int time, int bandwidth,
               char format) {
          iperf_server(out, true, true, port, length, interval, time, format);
        },
        "Run an iperf server in UDP mode using IPv6, listening on the specified port with a "
        "specific length, interval, time, bandwidth, and output format.");
    menu->Insert(
        "server_tcp_ipv6", {"port", "length", "interval", "time", "bandwidth", "format"},
        [this](std::ostream &out, int port, int length, int interval, int time, int bandwidth,
               char format) {
          iperf_server(out, false, true, port, length, interval, time, format);
        },
        "Run an iperf server in TCP mode using IPv6, listening on the specified port with a "
        "specific length, interval, time, bandwidth, and output format.");
#endif // IPERF_IPV6_ENABLED

    return menu;
  }

  void abort(std::ostream &out) {
    iperf_stop();
    out << "Iperf test aborted.\n";
  }

  void iperf_client(std::ostream &out, const std::string &host, bool udp = false, bool ipv6 = false,
                    int port = 0, int length = 0, // 0 means use default length
                    int interval = IPERF_DEFAULT_INTERVAL, int time = IPERF_DEFAULT_TIME,
                    int bandwidth = IPERF_DEFAULT_NO_BW_LIMIT, char format = 'k') {
    iperf_cfg_t cfg{};

    // set ip type
    cfg.type = IPERF_IP_TYPE_IPV4; // Default to IPv4
#if IPERF_IPV6_ENABLED
    if (ipv6) {
      cfg.type = IPERF_IP_TYPE_IPV6; // Use IPv6 if specified
    }
#endif

    // set client flag
    cfg.flag |= IPERF_FLAG_CLIENT;

    // set the desitination host
#if IPERF_IPV6_ENABLED
    if (ipv6) {
      cfg.destination_ip6 = (char *)host.c_str(); // Set the IPv6 host
    } else {
      cfg.destination_ip4 = esp_ip4addr_aton(host.c_str()); // Set the IPv4 host
    }
#endif
#if IPERF_IPV4_ENABLED
    if (!ipv6) {
      cfg.destination_ip4 = esp_ip4addr_aton(host.c_str());
    }
#endif

    // set the tcp/udp flag
    if (udp) {
      cfg.flag |= IPERF_FLAG_UDP; // Use UDP if specified
    } else {
      cfg.flag |= IPERF_FLAG_TCP; // Use TCP by default
    }

    // set the length
    cfg.len_send_buf = length;

    // set the port
    if (port == 0) {
      cfg.sport = IPERF_DEFAULT_PORT; // Default port
      cfg.dport = IPERF_DEFAULT_PORT; // Default port
    } else {
      // NOTE: This is specifically for the client
      cfg.sport = IPERF_DEFAULT_PORT; // Default source port
      cfg.dport = port;               // Use specified destination port
    }

    // set the interval
    cfg.interval = interval;

    // set the time
    cfg.time = time;

    // set the bandwidth limit for UDP tests
    cfg.bw_lim = bandwidth;

    // set the output format
    switch (format) {
    case 'k':
      cfg.format = KBITS_PER_SEC; // kibits/sec
      break;
    case 'm':
      cfg.format = MBITS_PER_SEC; // mbits/sec
      break;
    default:
      out << "Invalid format specified. Using kibits/sec by default.\n";
      cfg.format = KBITS_PER_SEC; // Default to kibits/sec
      break;
    }

    // print the configuration
    out << fmt::format("Using iperf config: {}\n", cfg);

    // now start the client
    iperf_start(&cfg);
  }

  void iperf_server(std::ostream &out, bool udp = false, bool ipv6 = false, int port = 0,
                    int length = 0, int interval = IPERF_DEFAULT_INTERVAL,
                    int time = IPERF_DEFAULT_TIME, char format = 'k') {
    iperf_cfg_t cfg{};

    // set the ip type
    cfg.type = IPERF_IP_TYPE_IPV4; // Default to IPv4
#if IPERF_IPV6_ENABLED
    if (ipv6) {
      cfg.type = IPERF_IP_TYPE_IPV6; // Use IPv6 if specified
    }
#endif

    // Set the server flag
    cfg.flag |= IPERF_FLAG_SERVER;

    // set the tcp/udp flag
    if (udp) {
      cfg.flag |= IPERF_FLAG_UDP; // Use UDP if specified
    } else {
      cfg.flag |= IPERF_FLAG_TCP; // Use TCP by default
    }

    // set the length
    cfg.len_send_buf = length;

    // set the port
    if (port == 0) {
      cfg.sport = IPERF_DEFAULT_PORT; // Default port
      cfg.dport = IPERF_DEFAULT_PORT; // Default port
    } else {
      // NOTE: This is specifically for the server
      cfg.sport = port;               // Use specified destination port
      cfg.dport = IPERF_DEFAULT_PORT; // Default source port
    }

    //  set the interval
    cfg.interval = interval;

    // set the time
    cfg.time = time;

    // set the bandwidth limit for UDP tests
    cfg.bw_lim = IPERF_DEFAULT_NO_BW_LIMIT; // No bandwidth limit by default

    // set the output format
    switch (format) {
    case 'k':
      cfg.format = KBITS_PER_SEC; // kibits/sec
      break;
    case 'm':
      cfg.format = MBITS_PER_SEC; // mbits/sec
      break;
    default:
      out << "Invalid format specified. Using kibits/sec by default.\n";
      cfg.format = KBITS_PER_SEC; // Default to kibits/sec
      break;
    }

    // print the configuration
    out << fmt::format("Using iperf config: {}\n", cfg);

    // now start the server
    iperf_start(&cfg);
  }

protected:
};
} // namespace espp

#endif // CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)

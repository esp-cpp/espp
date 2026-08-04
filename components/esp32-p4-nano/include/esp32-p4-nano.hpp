#pragma once

#include <array>
#include <functional>
#include <memory>

#include <sdkconfig.h>

#include <esp_netif.h>

#include "base_component.hpp"
#include "ethernet.hpp"

namespace espp {
/// @brief Board Support Package (BSP) for the Waveshare ESP32-P4-NANO board.
///
/// This class provides a singleton interface to the board's Ethernet peripheral:
/// - 10/100 Ethernet via the ESP32-P4 internal EMAC and an IP101GRI RMII PHY.
///
/// The Ethernet bring-up is delegated to the reusable espp::Ethernet component;
/// this BSP just supplies the board-specific RMII pin mapping.
///
/// RMII pin mapping (ESP32-P4 routable EMAC pins):
/// | Signal   | GPIO |
/// |----------|------|
/// | REF_CLK  |   50 | ← 50 MHz (25 MHz crystal x2)
/// | TX_EN    |   49 |
/// | TXD0     |   34 |
/// | TXD1     |   35 |
/// | CRS_DV   |   28 |
/// | RXD0     |   29 |
/// | RXD1     |   30 |
/// | MDC      |   31 |
/// | MDIO     |   52 |
/// | PHY_RST  |   51 |
///
/// The class is a singleton and can be accessed via get().
///
/// \section esp32_p4_nano_example Example
/// \snippet esp32_p4_nano_example.cpp esp32 p4 nano example
class Esp32P4Nano : public BaseComponent {
public:
  /// Callback invoked (SERVER mode only) each time the DHCP server assigns an
  /// IP address to a connected client.
  using client_ip_callback_t = std::function<void(esp_ip4_addr_t ip, std::array<uint8_t, 6> mac)>;

  /// Callback invoked when the Ethernet link state changes or the IP is lost.
  /// \note Runs in the ESP-IDF event-loop task context — return quickly, do not block.
  using EthernetLinkCallback = std::function<void()>;

  /// Callback invoked when the interface obtains an IPv4 address.
  /// \note Runs in the ESP-IDF event-loop task context — return quickly, do not block.
  using EthernetIpCallback = std::function<void(esp_ip4_addr_t ip)>;

  /// DHCP operating mode for the Ethernet interface.
  enum class DhcpMode {
    CLIENT, ///< DHCP client — acquire an IP from an upstream server (default).
    SERVER, ///< DHCP server — assign IPs to hosts connected to this interface.
  };

  /// Static IP configuration used when operating as a DHCP server.
  /// Leave \c ip_info zero-initialised to use the built-in defaults
  /// (192.168.4.1 / 255.255.255.0 / gw 192.168.4.1).
  struct ServerConfig {
    esp_netif_ip_info_t ip_info{};                    ///< zero-initialised → 192.168.4.1/24
    client_ip_callback_t on_client_assigned{nullptr}; ///< Called for each assigned client IP.
  };

  /// Configuration for the Ethernet interface.
  struct EthernetConfig {
    DhcpMode mode{DhcpMode::CLIENT};            ///< DHCP operating mode.
    ServerConfig server_config{};               ///< Only used when mode == SERVER.
    EthernetLinkCallback on_link_up{nullptr};   ///< Physical link came up.
    EthernetLinkCallback on_link_down{nullptr}; ///< Physical link went down.
    EthernetIpCallback on_got_ip{nullptr};      ///< Interface obtained an IPv4 address.
    EthernetLinkCallback on_lost_ip{nullptr};   ///< Interface lost its IPv4 address.
  };

  /// @brief Access the singleton instance.
  static Esp32P4Nano &get() {
    static Esp32P4Nano instance;
    return instance;
  }

  Esp32P4Nano(const Esp32P4Nano &) = delete;
  Esp32P4Nano &operator=(const Esp32P4Nano &) = delete;
  Esp32P4Nano(Esp32P4Nano &&) = delete;
  Esp32P4Nano &operator=(Esp32P4Nano &&) = delete;

  /// Initialize the Ethernet interface (EMAC + IP101GRI RMII PHY).
  /// \param config Ethernet configuration (DHCP mode, callbacks). All fields
  ///        have defaults, so \c EthernetConfig{} gives a plain DHCP client.
  /// \return True if Ethernet was successfully initialized and started.
  bool initialize_ethernet(const EthernetConfig &config);

  /// Initialize Ethernet with default configuration (DHCP client).
  /// \return True if Ethernet was successfully initialized and started.
  bool initialize_ethernet();

  /// \return True if the interface is connected with a valid IP.
  bool is_ethernet_connected() const { return ethernet_ && ethernet_->is_connected(); }

  /// \return The most recently acquired IPv4 address (0 if none).
  esp_ip4_addr_t ethernet_ip() const { return ethernet_ ? ethernet_->ip() : esp_ip4_addr_t{}; }

protected:
  Esp32P4Nano();

  // RMII pin mapping for the ESP32-P4-NANO (IP101GRI PHY).
  static constexpr int eth_mdc_io = 31;
  static constexpr int eth_mdio_io = 52;
  static constexpr int eth_ref_clk_io = 50;
  static constexpr int eth_phy_reset_gpio = 51;
  static constexpr int eth_phy_addr = 1;
  static constexpr int eth_tx_en_io = 49;
  static constexpr int eth_txd0_io = 34;
  static constexpr int eth_txd1_io = 35;
  static constexpr int eth_crs_dv_io = 28;
  static constexpr int eth_rxd0_io = 29;
  static constexpr int eth_rxd1_io = 30;

  // The board's RMII Ethernet is driven by the reusable espp::Ethernet component.
  std::unique_ptr<espp::Ethernet> ethernet_;
}; // class Esp32P4Nano
} // namespace espp

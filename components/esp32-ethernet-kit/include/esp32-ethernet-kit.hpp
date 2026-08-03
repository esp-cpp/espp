#pragma once

#include <array>
#include <atomic>
#include <functional>

#include <sdkconfig.h>

#include <esp_eth.h>
#include <esp_netif.h>

#include "base_component.hpp"

namespace espp {
/// @brief Board Support Package (BSP) for the Espressif ESP32-Ethernet-Kit A V1.2.
///
/// This class provides a singleton interface to the board's Ethernet peripheral:
/// - 10/100 Ethernet via the internal ESP32 EMAC and an IP101GRI RMII PHY
///
/// RMII pin mapping (fixed in the ESP32 IO_MUX; cannot be changed):
/// | Signal   | GPIO |
/// |----------|------|
/// | REF_CLK  |    0 | ← external 50 MHz oscillator (EMAC_CLK_EXT_IN)
/// | TX_EN    |   21 |
/// | TXD0     |   19 |
/// | TXD1     |   22 |
/// | CRS_DV   |   27 |
/// | RXD0     |   25 |
/// | RXD1     |   26 |
/// | MDC      |   23 |
/// | MDIO     |   18 |
/// | PHY_RST  |    5 |
///
/// The class is a singleton and can be accessed via get().
///
/// \section esp32_ethernet_kit_example Example
/// \subsection esp32_ethernet_kit_get_instance Get Instance
/// \snippet esp32_ethernet_kit_example.cpp esp32 ethernet kit get instance
/// \subsection esp32_ethernet_kit_dhcp_server DHCP Server
/// \snippet esp32_ethernet_kit_example.cpp esp32 ethernet kit dhcp server
/// \subsection esp32_ethernet_kit_dhcp_client DHCP Client
/// \snippet esp32_ethernet_kit_example.cpp esp32 ethernet kit dhcp client
class Esp32EthernetKit : public BaseComponent {
public:
  /// Callback invoked (SERVER mode only) each time the DHCP server assigns an
  /// IP address to a connected client.
  using client_ip_callback_t = std::function<void(esp_ip4_addr_t ip, std::array<uint8_t, 6> mac)>;

  /// Callback invoked when the Ethernet link state changes (comes up or goes
  /// down) or when the IP address is lost.
  /// \note Runs in the ESP-IDF event-loop task context — return quickly, do not block.
  using EthernetLinkCallback = std::function<void()>;

  /// Callback invoked when the interface obtains an IPv4 address.
  /// \param ip The assigned IPv4 address.
  /// \note Runs in the ESP-IDF event-loop task context — return quickly, do not block.
  using EthernetIpCallback = std::function<void(esp_ip4_addr_t ip)>;

  /// DHCP operating mode for the Ethernet interface
  enum class DhcpMode {
    CLIENT, ///< DHCP client — acquire an IP from an upstream server (default)
    SERVER, ///< DHCP server — assign IPs to hosts connected to this interface
  };

  /// Static IP configuration used when operating as a DHCP server.
  /// Leave \c ip_info zero-initialised to use the built-in defaults
  /// (192.168.4.1 / 255.255.255.0 / gw 192.168.4.1).
  struct ServerConfig {
    esp_netif_ip_info_t ip_info{
        .ip = 0, .netmask = 0, .gw = 0}; ///< IP / netmask / gateway; zero → 192.168.4.1/24
    client_ip_callback_t on_client_assigned{
        nullptr}; ///< Called each time a client is assigned an IP
  };

  /// Configuration for the Ethernet interface.
  struct EthernetConfig {
    /// DHCP operating mode (CLIENT or SERVER).
    DhcpMode mode{DhcpMode::CLIENT};

    /// Static IP / DHCP server settings — only used when mode == SERVER.
    ServerConfig server_config{};

    /// Called when the physical link comes up (cable connected + negotiated).
    EthernetLinkCallback on_link_up{nullptr};

    /// Called when the physical link goes down (cable disconnected).
    EthernetLinkCallback on_link_down{nullptr};

    /// Called when the interface is assigned an IPv4 address.
    /// CLIENT mode: fired by the DHCP lease.
    /// SERVER mode: fired immediately when the link comes up (static IP).
    EthernetIpCallback on_got_ip{nullptr};

    /// Called when the interface loses its IPv4 address
    /// (DHCP lease loss in CLIENT mode, or link-down in SERVER mode).
    EthernetLinkCallback on_lost_ip{nullptr};
  };

  /// @brief Access the singleton instance
  /// @return Reference to the singleton instance
  static Esp32EthernetKit &get() {
    static Esp32EthernetKit instance;
    return instance;
  }

  Esp32EthernetKit(const Esp32EthernetKit &) = delete;
  Esp32EthernetKit &operator=(const Esp32EthernetKit &) = delete;
  Esp32EthernetKit(Esp32EthernetKit &&) = delete;
  Esp32EthernetKit &operator=(Esp32EthernetKit &&) = delete;

  /////////////////////////////////////////////////////////////////////////////
  // Ethernet (EMAC + IP101GRI RMII PHY)
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the Ethernet interface (EMAC + IP101GRI RMII PHY).
  /// \param config  Ethernet configuration (DHCP mode, callbacks).
  ///                All fields have defaults so \c EthernetConfig{} gives a
  ///                plain DHCP-client interface with no callbacks.
  /// \return True if Ethernet was successfully initialized and started.
  /// \note Requires the ESP-IDF default event loop. The BSP creates it if needed.
  bool initialize_ethernet(const EthernetConfig &config);

  /// Initialize Ethernet with default configuration (DHCP client mode).
  /// \return True if Ethernet was successfully initialized and started.
  bool initialize_ethernet();

  /// Check whether the interface has a usable IP address
  /// (DHCP lease granted in CLIENT mode, or link is up in SERVER mode).
  /// \return True if the interface is connected with a valid IP.
  bool is_ethernet_connected() const { return ethernet_connected_; }

  /// Get the most recently acquired IPv4 address (0 if none).
  /// \return The IPv4 address.
  esp_ip4_addr_t ethernet_ip() const { return ethernet_ip_; }

protected:
  Esp32EthernetKit();

  /////////////////////////////////////////////////////////////////////////////
  // RMII / EMAC pin constants
  /////////////////////////////////////////////////////////////////////////////
  // The ESP32 RMII data-plane signals are fixed via IO_MUX and cannot be
  // reassigned (TX_EN=21, TXD0=19, TXD1=22, CRS_DV=27, RXD0=25, RXD1=26).
  // They are listed here for documentation only; the EMAC driver selects them
  // automatically when EMAC_DATA_INTERFACE_RMII is chosen.

  // RMII REF_CLK input: external 50 MHz oscillator on V1.2 drives GPIO0 (EMAC_CLK_IN_GPIO)
  static constexpr int rmii_clk_gpio = 0; // EMAC_CLK_IN_GPIO — fixed by ESP32 IO_MUX

  // SMI (management interface) — routable via GPIO matrix
  static constexpr int eth_mdc_io = 23;
  static constexpr int eth_mdio_io = 18;

  // IP101GRI PHY
  static constexpr int eth_phy_reset_gpio = 5; ///< Active-low; set to -1 to disable
  static constexpr int eth_phy_addr = 1;

  /////////////////////////////////////////////////////////////////////////////
  // Member variables
  /////////////////////////////////////////////////////////////////////////////

  // Ethernet
  DhcpMode dhcp_mode_{DhcpMode::CLIENT};
  esp_netif_ip_info_t server_ip_info_{}; ///< Resolved static IP (server mode only)
  client_ip_callback_t client_ip_callback_{nullptr};
  EthernetLinkCallback on_link_up_{};
  EthernetLinkCallback on_link_down_{};
  EthernetIpCallback on_got_ip_{};
  EthernetLinkCallback on_lost_ip_{};
  std::atomic<bool> ethernet_initialized_{false};
  std::atomic<bool> ethernet_connected_{false};
  esp_ip4_addr_t ethernet_ip_{};
  esp_eth_handle_t eth_handle_{nullptr};
  esp_eth_netif_glue_handle_t eth_glue_{nullptr}; // esp_eth_netif_glue_handle_t
  esp_netif_t *eth_netif_{nullptr};

  static void ethernet_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                                     void *event_data);
  static void ethernet_got_ip_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                                      void *event_data);
  static void ethernet_lost_ip_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                                       void *event_data);
  static void ethernet_client_ip_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                                         void *event_data);
}; // class Esp32EthernetKit
} // namespace espp

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
  /// Callback invoked when the Ethernet link comes up and an IP is assigned
  using ethernet_link_callback_t = std::function<void(esp_ip4_addr_t ip)>;

  /// Callback invoked (SERVER mode only) each time the DHCP server assigns an
  /// IP address to a connected client.
  /// \param ip  IPv4 address that was assigned.
  /// \param mac MAC address of the client.
  using client_ip_callback_t =
      std::function<void(esp_ip4_addr_t ip, std::array<uint8_t, 6> mac)>;

  /// DHCP operating mode for the Ethernet interface
  enum class DhcpMode {
    CLIENT, ///< DHCP client — acquire an IP from an upstream server (default)
    SERVER, ///< DHCP server — assign IPs to hosts connected to this interface
  };

  /// Static IP configuration used when operating as a DHCP server.
  /// Leave \c ip_info zero-initialised to use the built-in defaults
  /// (192.168.4.1 / 255.255.255.0 / gw 192.168.4.1).
  struct ServerConfig {
    esp_netif_ip_info_t ip_info;           ///< IP / netmask / gateway; zero → 192.168.4.1/24
    client_ip_callback_t on_client_assigned; ///< Called each time a client is assigned an IP
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

  /// Initialize the Ethernet interface (EMAC + IP101GRI RMII PHY)
  /// \param on_link_up  Optional callback invoked once a usable IP is available.
  ///                    CLIENT mode: called when DHCP assigns an IP.
  ///                    SERVER mode: called immediately when the link comes up
  ///                                 (the static IP is already known).
  /// \param mode        DHCP operating mode (CLIENT or SERVER, default CLIENT).
  /// \param server_config  Static IP configuration for SERVER mode.
  ///                       Ignored in CLIENT mode.
  /// \return True if Ethernet was successfully initialized and started.
  /// \note Requires the ESP-IDF default event loop. The BSP creates it if needed.
  bool initialize_ethernet(const ethernet_link_callback_t &on_link_up = nullptr,
                           DhcpMode mode = DhcpMode::CLIENT,
                           const ServerConfig &server_config = ServerConfig{});

  /// Check whether the Ethernet link is up (cable connected + negotiated)
  /// \return True if the link is up
  bool is_ethernet_connected() const { return ethernet_connected_; }

  /// Get the most recently acquired IPv4 address (0 if none)
  /// \return The IPv4 address
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
  static constexpr int eth_mdc_io  = 23;
  static constexpr int eth_mdio_io = 18;

  // IP101GRI PHY
  static constexpr int eth_phy_reset_gpio = 5; ///< Active-low; set to -1 to disable
  static constexpr int eth_phy_addr       = 1;

  /////////////////////////////////////////////////////////////////////////////
  // Member variables
  /////////////////////////////////////////////////////////////////////////////

  // Ethernet
  DhcpMode dhcp_mode_{DhcpMode::CLIENT};
  esp_netif_ip_info_t server_ip_info_{}; ///< Resolved static IP (server mode only)
  client_ip_callback_t client_ip_callback_{nullptr}; ///< Per-client lease callback (server mode)
  std::atomic<bool> ethernet_initialized_{false};
  std::atomic<bool> ethernet_connected_{false};
  esp_ip4_addr_t ethernet_ip_{};
  ethernet_link_callback_t ethernet_link_callback_{nullptr};
  esp_eth_handle_t eth_handle_{nullptr};
  void *eth_glue_{nullptr}; // esp_eth_netif_glue_handle_t
  esp_netif_t *eth_netif_{nullptr};

  static void ethernet_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                                     void *event_data);
  static void ethernet_got_ip_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                                      void *event_data);
  static void ethernet_client_ip_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                                         void *event_data);
}; // class Esp32EthernetKit
} // namespace espp

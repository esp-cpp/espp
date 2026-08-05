#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <variant>

#include <sdkconfig.h>

#include <driver/spi_master.h>
#include <esp_eth.h>
#include <esp_netif.h>

#include "base_component.hpp"

namespace espp {
/// @brief Cross-interface Ethernet wrapper around the ESP-IDF esp_eth APIs.
///
/// @details One class drives both **RMII** (internal EMAC, on SoCs with
///          `SOC_EMAC_SUPPORTED` - esp32 / esp32-p4) and **SPI** (external
///          MAC+PHY chips such as the WIZnet W5500) interfaces, plus a
///          pre-built-driver escape hatch for any other chip. It owns all of
///          the common boilerplate - netif, event loop, glue, attach, DHCP
///          client/server, static IP, hostname, MAC assignment, link/IP event
///          dispatch and, unlike the inline BSP implementations it replaces, a
///          symmetric teardown (@ref deinitialize + destructor).
///
///          The interface-specific part is a single tagged @ref Config member:
///          @ref RmiiConfig, @ref SpiConfig, or @ref DriverConfig.
///
/// @note The concrete SPI chip drivers (W5500, DM9051, ENC28J60) are ESP-IDF
///       managed components pulled in only when the matching
///       `CONFIG_ESPP_ETHERNET_*` option is enabled. The RMII path uses the
///       generic 802.3 PHY driver in `esp_eth` core (no extra dependency),
///       which supports common PHYs (IP101, LAN87xx, DP83848, RTL8201, KSZ8041).
///
/// \section ethernet_ex1 RMII Example
/// \snippet ethernet_example.cpp ethernet rmii example
/// \section ethernet_ex2 SPI (W5500) Example
/// \snippet ethernet_example.cpp ethernet spi example
class Ethernet : public BaseComponent {
public:
  using MacAddress = std::array<uint8_t, 6>;

  /// DHCP operating mode.
  enum class DhcpMode {
    CLIENT, ///< Acquire an IP from an upstream DHCP server (or use a static IP if ip_info is set).
    SERVER, ///< Run a DHCP server on this interface and assign IPs to connected hosts.
  };

  /// PHY model hint for the RMII path. All are handled by the generic 802.3
  /// driver in v1; the field is advisory (kept for a future specific-driver
  /// option) and does not change behavior today.
  enum class PhyModel { GENERIC, IP101, LAN87XX, DP83848, RTL8201, KSZ80XX };

  /// Supported built-in SPI ethernet chips.
  enum class SpiChip { W5500, DM9051, ENC28J60 };

  /// Callback for link up/down and IP-lost. Runs in the esp event-loop task
  /// context - keep it short and non-blocking.
  using LinkCallback = std::function<void()>;
  /// Callback for IP acquisition. Runs in the esp event-loop task context.
  using IpCallback = std::function<void(esp_ip4_addr_t ip)>;
  /// Callback (SERVER mode) invoked for each DHCP lease the server assigns.
  using ClientIpCallback = std::function<void(esp_ip4_addr_t ip, MacAddress mac)>;

  /// RMII / internal-EMAC interface configuration (esp32 / esp32-p4 only).
  struct RmiiConfig {
    int mdc_gpio{-1};                ///< SMI management clock (MDC) GPIO (required).
    int mdio_gpio{-1};               ///< SMI management data (MDIO) GPIO (required).
    int phy_addr{-1};                ///< PHY SMI address; -1 auto-detects.
    int phy_reset_gpio{-1};          ///< Active-low PHY reset GPIO; -1 = none.
    PhyModel phy{PhyModel::GENERIC}; ///< PHY model hint (advisory; see PhyModel).
    bool clock_ext_in{true}; ///< true: external RMII 50 MHz ref-clock in; false: internal out.
    int clock_gpio{0};       ///< RMII REF_CLK GPIO (fixed set on esp32; routable on esp32-p4).
    /// RMII data-plane pins. Only used on SoCs with routable EMAC pins
    /// (esp32-p4); ignored on esp32 where they are fixed via IO_MUX.
    struct DataPins {
      int tx_en, txd0, txd1, crs_dv, rxd0, rxd1;
    };
    std::optional<DataPins> data_pins{};
  };

  /// SPI interface configuration (external MAC+PHY chip on a caller-owned,
  /// already-initialized SPI bus). Works on any SoC.
  struct SpiConfig {
    spi_host_device_t host{
        SPI2_HOST};               ///< SPI host whose bus the caller already initialized (required).
    int cs_gpio{-1};              ///< Chip-select GPIO (required).
    int int_gpio{-1};             ///< Interrupt GPIO; -1 to poll instead.
    int reset_gpio{-1};           ///< Chip reset GPIO; -1 = none.
    int clock_speed_hz{20000000}; ///< SPI clock (W5500 max ~33 MHz; conservative default).
    int phy_addr{1};              ///< PHY address (W5500 = 1).
    SpiChip chip{SpiChip::W5500}; ///< Which built-in SPI ethernet chip.
  };

  /// Pre-built-driver escape hatch: the caller creates the MAC+PHY for any chip
  /// and hands them over. The component takes ownership (they are freed by
  /// esp_eth_driver_uninstall on teardown).
  struct DriverConfig {
    esp_eth_mac_t *mac{nullptr}; ///< Caller-created MAC (ownership transfers on initialize()).
    esp_eth_phy_t *phy{nullptr}; ///< Caller-created PHY (ownership transfers on initialize()).
    bool needs_isr_service{
        false}; ///< true if the driver uses a GPIO INT line (installs the ISR service).
    bool needs_mac_assignment{
        false}; ///< true if the chip has no factory MAC (assign from config/eFuse).
  };

  /// Full configuration.
  struct Config {
    /// The interface: RMII (internal EMAC), SPI (external chip), or a pre-built driver.
    std::variant<RmiiConfig, SpiConfig, DriverConfig> interface;
    DhcpMode mode{DhcpMode::CLIENT}; ///< DHCP client or server.
    std::optional<MacAddress>
        mac_address{};      ///< Explicit MAC; else eFuse (ESP_MAC_ETH) for chips needing one.
    std::string hostname{}; ///< netif hostname (empty = ESP-IDF default).
    /// CLIENT: if ip.addr != 0, use this as a static IP (DHCP client is stopped); else DHCP.
    /// SERVER: the interface/gateway IP for the DHCP server (0 -> 192.168.4.1/24).
    esp_netif_ip_info_t ip_info{};
    LinkCallback on_link_up{};             ///< Physical link came up.
    LinkCallback on_link_down{};           ///< Physical link went down.
    IpCallback on_got_ip{};                ///< Interface obtained an IPv4 address.
    LinkCallback on_lost_ip{};             ///< Interface lost its IPv4 address.
    ClientIpCallback on_client_assigned{}; ///< SERVER mode: a client was assigned an IP.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN};
  };

  /// @brief Construct (does not start the interface; call initialize()).
  /// @param config The configuration.
  explicit Ethernet(const Config &config);

  /// @brief Destroy - deinitializes the interface if still running.
  ~Ethernet();

  Ethernet(const Ethernet &) = delete;
  Ethernet &operator=(const Ethernet &) = delete;

  /// @brief Bring up the interface (create driver, attach netif, register
  ///        events, apply IP/hostname/MAC, start). Idempotent.
  /// @param ec Set to a specific error on failure; cleared on success.
  /// @return true on success (interface started).
  bool initialize(std::error_code &ec);

  /// @brief Convenience overload that logs on failure.
  /// @return true on success.
  bool initialize();

  /// @brief Stop and fully tear down the interface (reverse-order unwind).
  ///        Idempotent. Does not undo the process-global netif/event-loop init.
  /// @param ec Set on failure; cleared on success.
  /// @return true on success.
  bool deinitialize(std::error_code &ec);

  /// @brief Convenience overload that logs on failure.
  void deinitialize();

  /// @return true if the interface has been initialized/started.
  bool is_initialized() const { return initialized_.load(); }
  /// @return true if the link is up AND an IP address is held.
  bool is_connected() const { return connected_.load(); }
  /// @return true if the physical link is up.
  bool link_up() const { return link_up_.load(); }

  /// @return The current IPv4 address (0 if none).
  esp_ip4_addr_t ip() const;
  /// @return The current IPv4 address as a dotted quad ("0.0.0.0" if none).
  std::string get_ip_address() const;
  /// @return The interface MAC as "aa:bb:cc:dd:ee:ff" (empty if not initialized).
  std::string get_mac_address() const;
  /// @return {speed_mbps, full_duplex} if the link is up, else nullopt.
  std::optional<std::pair<int, bool>> link_speed_duplex() const;

  /// @return The raw esp_eth driver handle (nullptr if not initialized).
  esp_eth_handle_t native_handle() const { return eth_handle_; }
  /// @return The raw esp_netif handle (nullptr if not initialized).
  esp_netif_t *netif() const { return eth_netif_; }

protected:
  /// Create the interface-specific MAC + PHY per config_.interface.
  bool create_mac_phy(esp_eth_mac_t **mac, esp_eth_phy_t **phy, bool &needs_isr,
                      bool &needs_mac_assign, std::error_code &ec);

  static void eth_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data);
  static void got_ip_handler(void *arg, esp_event_base_t base, int32_t id, void *data);
  static void lost_ip_handler(void *arg, esp_event_base_t base, int32_t id, void *data);
  static void client_ip_handler(void *arg, esp_event_base_t base, int32_t id, void *data);

  Config config_;
  std::atomic<bool> initialized_{false};
  std::atomic<bool> link_up_{false};
  std::atomic<bool> connected_{false};
  std::atomic<uint32_t> ip_addr_{0}; ///< IPv4 in network byte order.

  esp_eth_handle_t eth_handle_{nullptr};
  esp_eth_netif_glue_handle_t eth_glue_{nullptr};
  esp_netif_t *eth_netif_{nullptr};
  bool handlers_registered_{false};
  bool client_ip_handler_registered_{false};
  esp_netif_ip_info_t server_ip_info_{}; ///< Resolved static/server IP (SERVER mode).
};
} // namespace espp

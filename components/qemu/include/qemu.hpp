#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <sdkconfig.h>

#include <esp_lcd_panel_ops.h>
#include <esp_lcd_types.h>
#include <esp_netif_ip_addr.h>
#include <soc/soc_caps.h>

#include "base_component.hpp"
#include "display.hpp"
#include "ethernet.hpp"
#include "sdcard.hpp"

namespace espp {
/// Board-support package for Espressif's QEMU emulator.
///
/// QEMU is not a board, but it has a few virtual peripherals that no real board
/// has, and an application that targets them runs with no hardware at all: on
/// a developer's machine, in CI, or in a container. This BSP wraps the ones the
/// emulator offers, with the same shape as the real-board BSPs:
///
/// - the virtual RGB panel, as an LVGL display (`initialize_display()`): what
///   LVGL draws appears in the QEMU window (`-display sdl` / `idf.py qemu
///   --graphics`) and can be captured with the QEMU monitor's `screendump`;
/// - the OpenCores Ethernet MAC (`initialize_ethernet()`): a real lwIP
///   interface on QEMU's user-mode NAT (`-nic user,model=open_eth`, the
///   default of `idf.py qemu`) or on a tap device;
/// - the SD/MMC host (`initialize_sdcard()`, ESP32 only): a FAT volume on an
///   image file passed with `-drive file=sd.img,if=sd,format=raw`.
///
/// Everything else (touch, I2C, I2S, Wi-Fi, BLE, USB) is not emulated. The
/// component builds for the three targets the emulator supports (esp32,
/// esp32c3, esp32s3); see the component README for the QEMU command lines.
///
/// \section qemu_ex1 QEMU Example
/// \snippet qemu_example.cpp qemu example
class Qemu : public BaseComponent {
public:
  /// The pixel type the virtual panel is driven with (RGB565)
  using Pixel = lv_color16_t;

  /// Configuration for the virtual RGB panel + LVGL display
  struct DisplayConfig {
    size_t width{320};  ///< Panel (QEMU window) width in pixels
    size_t height{240}; ///< Panel (QEMU window) height in pixels
    /// Size of each of LVGL's two draw buffers, in pixels; 0 = a tenth of the
    /// panel. The panel has its own frame buffer, in QEMU-side memory, that
    /// draw_bitmap() copies into; LVGL renders into internal-RAM draw buffers.
    size_t pixel_buffer_size{0};
    bool double_buffered{true}; ///< Two draw buffers (render while the other is flushed)
  };

  /// Whether the Ethernet interface runs a DHCP client or a DHCP server
  enum class DhcpMode {
    CLIENT, ///< Ask QEMU's user-mode network (or the tap's network) for an address
    SERVER, ///< Serve addresses (tap networking with a host interface bridged in)
  };

  /// Configuration for the OpenCores Ethernet interface
  struct EthernetConfig {
    DhcpMode mode{DhcpMode::CLIENT}; ///< DHCP client (default) or server
    /// CLIENT: if `ip.addr` is non-zero, a static address instead of DHCP.
    /// SERVER: the interface / gateway address (zero = 192.168.4.1/24).
    esp_netif_ip_info_t ip_info{};
    std::string hostname{};                          ///< netif hostname (empty = ESP-IDF default)
    std::function<void()> on_link_up{};              ///< Link came up (immediate in QEMU)
    std::function<void()> on_link_down{};            ///< Link went down
    Ethernet::IpCallback on_got_ip{};                ///< An IPv4 address was obtained
    std::function<void()> on_lost_ip{};              ///< The IPv4 address was lost
    Ethernet::ClientIpCallback on_client_assigned{}; ///< SERVER: a lease was handed out
  };

#if SOC_SDMMC_HOST_SUPPORTED
  /// Configuration for the SD card (the image passed to QEMU with `-drive
  /// file=...,if=sd,format=raw`)
  struct SdCardConfig {
    std::string mount_point{"/sdcard"}; ///< VFS path the FAT volume is mounted at
    uint8_t bus_width{4};               ///< 1 or 4 data lines
    bool format_if_mount_failed{true};  ///< Format a blank image on first use
  };
#endif

  /// Access the singleton
  /// \return The one Qemu instance
  static Qemu &get() {
    static Qemu instance;
    return instance;
  }

  Qemu(const Qemu &) = delete;
  Qemu &operator=(const Qemu &) = delete;

  /////////////////////////////////////////////////////////////////////////////
  // Display (virtual RGB panel)
  /////////////////////////////////////////////////////////////////////////////

  /// Create the virtual RGB panel and the LVGL display on it
  /// \param config Panel size and draw-buffer configuration
  /// \return true on success (false outside QEMU: the panel's registers do
  ///         not exist on real hardware, and the driver refuses them)
  bool initialize_display(const DisplayConfig &config);

  /// Create the virtual RGB panel and the LVGL display with the default
  /// configuration (320 x 240)
  /// \return true on success
  bool initialize_display();

  /// \return The LVGL display, or null before initialize_display()
  std::shared_ptr<Display<Pixel>> display() const { return display_; }

  /// \return The esp_lcd panel handle, or null before initialize_display()
  esp_lcd_panel_handle_t lcd_panel() const { return panel_; }

  /// \return Panel width in pixels (0 before initialize_display())
  size_t display_width() const { return display_width_; }

  /// \return Panel height in pixels (0 before initialize_display())
  size_t display_height() const { return display_height_; }

  /// Write pixels straight to the panel, bypassing LVGL
  /// \param xs,ys Top-left corner (inclusive)
  /// \param xe,ye Bottom-right corner (inclusive)
  /// \param data (xe - xs + 1) * (ye - ys + 1) pixels, row-major
  void write_lcd_lines(int xs, int ys, int xe, int ye, const Pixel *data);

  /////////////////////////////////////////////////////////////////////////////
  // Ethernet (OpenCores MAC)
  /////////////////////////////////////////////////////////////////////////////

  /// Bring up the OpenCores Ethernet interface
  /// \note Needs CONFIG_ETH_USE_OPENETH (ESP-IDF: Component config ->
  ///       Ethernet -> Support OpenCores Ethernet MAC); fails otherwise.
  /// \param config DHCP mode, addresses and callbacks
  /// \return true once the interface is started
  bool initialize_ethernet(const EthernetConfig &config);

  /// Bring up the OpenCores Ethernet interface as a DHCP client
  /// \return true once the interface is started
  bool initialize_ethernet();

  /// \return true while the link is up
  bool is_ethernet_connected() const { return ethernet_ && ethernet_->is_connected(); }

  /// \return The interface's IPv4 address (zero until obtained)
  esp_ip4_addr_t ethernet_ip() const { return ethernet_ ? ethernet_->ip() : esp_ip4_addr_t{}; }

  /// \return The Ethernet component, or null before initialize_ethernet()
  std::shared_ptr<Ethernet> ethernet() const { return ethernet_; }

#if SOC_SDMMC_HOST_SUPPORTED
  /////////////////////////////////////////////////////////////////////////////
  // SD card (SD/MMC host; emulated on the ESP32 machine)
  /////////////////////////////////////////////////////////////////////////////

  /// Bring up the SD/MMC host and mount the card image as a FAT volume
  /// \param config Mount point, bus width, formatting
  /// \return true once mounted
  bool initialize_sdcard(const SdCardConfig &config);

  /// Mount the card image at /sdcard with the default configuration
  /// \return true once mounted
  bool initialize_sdcard();

  /// \return The SdCard component, or null before initialize_sdcard()
  std::shared_ptr<SdCard> sdcard() const { return sdcard_; }
#endif

protected:
  Qemu();

  void flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);

  // display
  esp_lcd_panel_handle_t panel_{nullptr};
  std::shared_ptr<Display<Pixel>> display_;
  size_t display_width_{0};
  size_t display_height_{0};
  std::mutex panel_mutex_;

  // ethernet
  std::shared_ptr<Ethernet> ethernet_;

#if SOC_SDMMC_HOST_SUPPORTED
  // sd card
  std::shared_ptr<SdCard> sdcard_;
#endif
}; // class Qemu
} // namespace espp

#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <vector>

#include <sdkconfig.h>

#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <driver/sdmmc_host.h>
#include <esp_netif.h>
#include <sd_pwr_ctrl.h>

#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>

#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>

#include "base_component.hpp"
#include "display.hpp"
#include "display_drivers.hpp"
#include "ek79007.hpp"
#include "es8311.hpp"
#include "ethernet.hpp"
#include "gt911.hpp"
#include "i2c.hpp"
#include "ili9881.hpp"
#include "interrupt.hpp"
#include "jd9365.hpp"
#include "task.hpp"
#include "touchpad_input.hpp"

namespace espp {
/// @brief Board Support Package (BSP) for the Waveshare ESP32-P4-Module-DEV-KIT board.
///
/// The dev kit is the Waveshare ESP32-P4-Module (ESP32-P4 + 16 MB flash + 32 MB
/// PSRAM + an on-module ESP32-C6 for WiFi 6 / BT 5) on a carrier board with
/// RJ45 Ethernet (PoE-module option), a 2-lane MIPI-DSI display FFC, a 2-lane
/// MIPI-CSI camera FFC, a TF-card slot, an ES8311 codec with on-board mic and
/// speaker header, USB 2.0 OTG HS (Type-C + Type-A host ports), an RTC
/// backup-battery connector, and a GPIO pin header.
///
/// This class provides a singleton interface to the board's peripherals:
/// - 10/100 Ethernet via the ESP32-P4 internal EMAC and an IP101GRI RMII PHY.
/// - MIPI-DSI display (JD9365 10.1" by default, or ILI9881C 10.1" / EK79007 7",
///   selected via Kconfig) with a GT911 capacitive-touch controller.
/// - MIPI-CSI camera (esp_video / V4L2 capture pipeline; OV5647 by default).
/// - microSD / TF card over 4-bit SDMMC.
/// - ES8311 audio codec (+ NS4150B amplifier) for speaker output and microphone
///   input over I2S.
///
/// The ES8311 codec, GT911 touch, and camera SCCB share a single internal I2C
/// bus (\ref internal_i2c()). The Ethernet bring-up is delegated to the reusable
/// espp::Ethernet component; this BSP supplies the board-specific pin mappings.
///
/// \note WiFi / Bluetooth are provided by the ESP32-C6 on the ESP32-P4-Module,
///       connected to the P4 over SDIO (esp_hosted default pins). This BSP does
///       not manage the C6; use the esp_wifi_remote + esp_hosted components
///       directly.
///
/// \note Timekeeping uses the ESP32-P4's internal RTC (the on-board RTC
///       connector is a backup-battery input for it); use the standard
///       C/ESP-IDF time APIs rather than a dedicated RTC driver.
///
/// RMII pin mapping (ESP32-P4 routable EMAC pins). REF_CLK carries a 50 MHz
/// reference clock (25 MHz crystal &times;2):
///
/// <table>
/// <tr><th>Signal</th><th>GPIO</th></tr>
/// <tr><td>REF_CLK</td><td>50</td></tr>
/// <tr><td>TX_EN</td><td>49</td></tr>
/// <tr><td>TXD0</td><td>34</td></tr>
/// <tr><td>TXD1</td><td>35</td></tr>
/// <tr><td>CRS_DV</td><td>28</td></tr>
/// <tr><td>RXD0</td><td>29</td></tr>
/// <tr><td>RXD1</td><td>30</td></tr>
/// <tr><td>MDC</td><td>31</td></tr>
/// <tr><td>MDIO</td><td>52</td></tr>
/// <tr><td>PHY_RST</td><td>51</td></tr>
/// </table>
///
/// The class is a singleton and can be accessed via get().
///
/// \section esp32_p4_module_dev_kit_example Example
/// \snippet esp32_p4_module_dev_kit_example.cpp esp32 p4 module dev kit example
class Esp32P4ModuleDevKit : public BaseComponent {
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
  static Esp32P4ModuleDevKit &get() {
    static Esp32P4ModuleDevKit instance;
    return instance;
  }

  Esp32P4ModuleDevKit(const Esp32P4ModuleDevKit &) = delete;
  Esp32P4ModuleDevKit &operator=(const Esp32P4ModuleDevKit &) = delete;
  Esp32P4ModuleDevKit(Esp32P4ModuleDevKit &&) = delete;
  Esp32P4ModuleDevKit &operator=(Esp32P4ModuleDevKit &&) = delete;

  /// Alias for the pixel type used by the display
  using Pixel = lv_color16_t;

  /// Alias for the low-level display driver interface
  using DisplayDriver = espp::display_drivers::Controller;

  /// Alias for the GT911 touch controller
  using TouchDriver = espp::Gt911;

  /// Alias for the touchpad data
  using TouchpadData = espp::TouchpadData;

  /// Alias for the touch callback when touch events are received
  using touch_callback_t = std::function<void(const TouchpadData &)>;

  /// Enum for the display controller type (selected via Kconfig)
  enum class DisplayController { UNKNOWN, EK79007, ILI9881C, JD9365 };

  /// Default touch INT GPIO used by initialize_touch(). GPIO_NUM_NC means the
  /// GT911 is polled; if interrupt-driven touch is enabled via Kconfig this is
  /// the configured GPIO (CONFIG_ESP32_P4_MODULE_DEV_KIT_TOUCH_INTERRUPT_GPIO).
#if CONFIG_ESP32_P4_MODULE_DEV_KIT_TOUCH_INTERRUPT
  static constexpr gpio_num_t touch_interrupt_default =
      static_cast<gpio_num_t>(CONFIG_ESP32_P4_MODULE_DEV_KIT_TOUCH_INTERRUPT_GPIO);
#else
  static constexpr gpio_num_t touch_interrupt_default = GPIO_NUM_NC;
#endif

  /// Get a reference to the internal I2C bus
  /// \return A reference to the internal I2C bus
  /// \note Shared by the ES8311 audio codec, the GT911 touch controller, and
  ///       (on this pinout) the camera SCCB
  I2c &internal_i2c() { return internal_i2c_; }

  /// Get a reference to the interrupts
  /// \return A reference to the interrupts
  espp::Interrupt &interrupts() { return interrupts_; }

  /// Get the display controller type for the configured panel
  /// \return The display controller type
  DisplayController get_display_controller() const { return display_controller_; }

  /// Get a string name for the configured display controller
  /// \return String name of the controller
  const char *get_display_controller_name() const {
    switch (display_controller_) {
    case DisplayController::EK79007:
      return "EK79007";
    case DisplayController::ILI9881C:
      return "ILI9881C";
    case DisplayController::JD9365:
      return "JD9365";
    default:
      return "Unknown";
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  // Display & Touchpad
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the LCD (MIPI-DSI + configured panel driver)
  /// \return true if the LCD was successfully initialized, false otherwise
  bool initialize_lcd();

  /// Initialize the LVGL display
  /// \param pixel_buffer_size The size of the pixel buffer, in pixels. If 0, a
  ///        default based on the configured panel width is used.
  /// \return true if the display was successfully initialized, false otherwise
  bool initialize_display(size_t pixel_buffer_size = 0);

  /// Initialize the GT911 multi-touch controller
  /// \param callback The touchpad callback
  /// \param interrupt_pin GPIO wired to the GT911 touch INT pin. If GPIO_NUM_NC
  ///        (the default), the GT911 is polled in a task. If a valid GPIO is
  ///        provided, touch is read from a GPIO interrupt on that pin instead.
  /// \return true if the touchpad was successfully initialized, false otherwise
  /// \note On the ESP32-P4-Module-DEV-KIT the GT911 reset and INT pins are not routed to
  ///       the ESP32-P4, so touch is polled by default.
  bool initialize_touch(const touch_callback_t &callback = nullptr,
                        gpio_num_t interrupt_pin = touch_interrupt_default);

  /// Get the number of bytes per pixel for the display
  /// \return The number of bytes per pixel
  size_t bytes_per_pixel() const { return sizeof(Pixel); }

  /// Get the touchpad input
  /// \return A shared pointer to the touchpad input
  std::shared_ptr<TouchpadInput> touchpad_input() const { return touchpad_input_; }

  /// Get the most recent touchpad data
  /// \return The touchpad data
  TouchpadData touchpad_data() const {
    std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
    return touchpad_data_;
  }

  /// Get the touchpad data for LVGL integration
  /// \param num_touch_points The number of touch points
  /// \param x The x coordinate
  /// \param y The y coordinate
  /// \param btn_state The button state (0 = released, 1 = pressed)
  void touchpad_read(uint8_t *num_touch_points, uint16_t *x, uint16_t *y, uint8_t *btn_state);

  /// Convert touchpad data from raw reading to display coordinates
  /// \param data The touchpad data to convert
  /// \return The converted touchpad data
  TouchpadData touchpad_convert(const TouchpadData &data) const;

  /// Set the display brightness
  /// \param brightness The brightness as a percentage (0-100)
  /// \note The ESP32-P4-Module-DEV-KIT has no backlight GPIO. On the 10.1" JD9365 panel
  ///       the backlight is driven by an on-board I2C controller (addr 0x45)
  ///       and this call writes it; on other panels the value is stored but
  ///       not applied to hardware (see the source for details).
  void brightness(float brightness);

  /// Get the display brightness
  /// \return The brightness as a percentage (0-100)
  float brightness() const;

  /// Get the display width in pixels (of the configured panel)
  /// \return The display width in pixels
  size_t display_width() const { return display_width_; }

  /// Get the display height in pixels (of the configured panel)
  /// \return The display height in pixels
  size_t display_height() const { return display_height_; }

  /// Get the display width in pixels, according to the current orientation
  size_t rotated_display_width() const;

  /// Get the display height in pixels, according to the current orientation
  size_t rotated_display_height() const;

  /// Get a shared pointer to the low-level display driver
  /// \return A shared pointer to the display driver
  const std::shared_ptr<DisplayDriver> &display_driver() const { return display_driver_; }

  /// Write lines to the LCD
  /// \note This method queues the panel transfer asynchronously.
  void write_lcd_lines(int xs, int ys, int xe, int ye, const uint8_t *data, uint32_t user_data);

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

  /////////////////////////////////////////////////////////////////////////////
  // Audio System (ES8311 + NS4150B)
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the audio system (ES8311 codec)
  /// \param sample_rate The audio sample rate in Hz (default 48kHz). Must be
  ///        one of the rates supported by the ES8311 clock coefficient table:
  ///        8000, 11025, 16000, 22050, 24000, 32000, 44100, or 48000 Hz.
  /// \param task_config The task configuration for the audio task
  /// \return true if the audio system was successfully initialized (or was
  ///         already initialized), false on failure or unsupported sample rate
  bool initialize_audio(uint32_t sample_rate = 48000,
                        const espp::Task::BaseConfig &task_config = {.name = "p4mdk_audio",
                                                                     .stack_size_bytes = 8192,
                                                                     .priority = 20,
                                                                     .core_id = 0});

  /// Enable or disable the speaker amplifier (NS4150B PA on GPIO53)
  /// \param enable True to enable the amplifier, false to disable
  void set_speaker_enabled(bool enable);

  /// Set the audio volume
  /// \param volume The volume as a percentage (0-100)
  void volume(float volume);

  /// Get the audio volume
  /// \return The volume as a percentage (0-100)
  float volume() const;

  /// Mute or unmute the audio
  /// \param mute True to mute, false to unmute
  void mute(bool mute);

  /// Check if audio is muted
  /// \return True if muted, false otherwise
  bool is_muted() const;

  /// Get the audio sample rate
  /// \return The audio sample rate, in Hz
  uint32_t audio_sample_rate() const;

  /// Set the audio sample rate
  /// \param sample_rate The audio sample rate, in Hz
  void audio_sample_rate(uint32_t sample_rate);

  /// Get the audio buffer size, in bytes
  /// \return The audio buffer size, in bytes
  size_t audio_buffer_size() const;

  /// Play audio data
  /// \param data The audio data to play (16-bit signed mono samples)
  /// \param num_bytes The number of bytes to play
  /// \return The number of bytes actually queued (may be less than \p
  ///         num_bytes if the internal stream buffer is full)
  /// \note This function is non-blocking and queues the data for the audio
  ///       task to play; to stream data larger than the internal buffer,
  ///       call it repeatedly, advancing by the returned number of bytes
  /// \note Must be called from task context, not from an ISR.
  size_t play_audio(const uint8_t *data, uint32_t num_bytes);

  /// Play audio data
  /// \param data The audio data to play (16-bit signed mono samples)
  /// \return The number of bytes actually queued (may be less than the data
  ///         size if the internal stream buffer is full)
  /// \note This function is non-blocking and queues the data for the audio
  ///       task to play; to stream data larger than the internal buffer,
  ///       call it repeatedly, advancing by the returned number of bytes
  /// \note Must be called from task context, not from an ISR.
  size_t play_audio(std::span<const uint8_t> data);

  /// Drop any queued (not yet played) audio so a subsequent play_audio() starts
  /// immediately instead of waiting behind previously queued sound. Useful for
  /// UI sounds where a new event should restart the sound for maximum
  /// responsiveness.
  void clear_audio();

  /////////////////////////////////////////////////////////////////////////////
  // Microphone
  /////////////////////////////////////////////////////////////////////////////

  /// Alias for the microphone callback, called with recorded audio data
  using microphone_callback_t = std::function<void(const uint8_t *data, size_t num_bytes)>;

  /// Initialize the microphone (the onboard analog microphone through the
  /// ES8311 codec's ADC) and start delivering audio data to the provided
  /// callback
  /// \param callback The callback to call with recorded audio data (16-bit
  ///        signed mono samples at audio_sample_rate())
  /// \param task_config The configuration for the microphone task
  /// \return true if the microphone was successfully initialized (or was
  ///         already initialized; the existing callback is kept), false
  ///         otherwise
  /// \note The audio subsystem must be initialized first (the ES8311 is a
  ///       full-duplex codec on a single I2S bus, so the microphone records
  ///       at the speaker's sample rate)
  /// \note The callback runs in the microphone task's context, so the task's
  ///       stack must be large enough for whatever the callback does with
  ///       the audio data
  bool initialize_microphone(const microphone_callback_t &callback,
                             const espp::Task::BaseConfig &task_config = {.name = "microphone",
                                                                          .stack_size_bytes = 4096,
                                                                          .priority = 10,
                                                                          .core_id = 1});

  /// Set the microphone volume
  /// \param volume The volume as a percentage (0 - 100), mapped onto the
  ///        ES8311 analog microphone gain range (0 dB - +42 dB)
  void microphone_volume(float volume);

  /// Get the microphone volume
  /// \return The microphone volume as a percentage (0 - 100)
  float microphone_volume() const;

  /////////////////////////////////////////////////////////////////////////////
  // Camera (MIPI-CSI, OV5647)
  /////////////////////////////////////////////////////////////////////////////

  /// Alias for the camera frame callback. Called from the camera task with each
  /// captured frame: \p data is the pixel buffer (RGB565, \p width x \p height),
  /// valid only for the duration of the callback, and \p length is its size in
  /// bytes. Copy the data if it needs to outlive the call.
  using camera_frame_callback_t =
      std::function<void(const uint8_t *data, int width, int height, size_t length)>;

  /// Initialize the on-board MIPI-CSI camera and start streaming frames.
  ///
  /// Brings up the ESP32-P4 camera pipeline (MIPI-CSI receiver + ISP + sensor)
  /// through esp_video (V4L2) and starts a task that delivers each captured
  /// RGB565 frame to \p callback. The camera sensor's SCCB shares the internal
  /// I2C bus (SDA=7/SCL=8), so no second I2C master is created on those pins.
  ///
  /// \param callback Function called from the camera task with each RGB565
  ///        frame (see camera_frame_callback_t). Keep it quick and non-blocking.
  /// \param task_config The configuration for the camera task
  /// \return true if the camera was successfully initialized and streaming (or
  ///         was already initialized; the existing callback and stream are
  ///         kept — call stop_camera() first to re-initialize)
  /// \note The camera reset / power-down lines are not routed to the ESP32-P4 on
  ///       this board (RPi-style CSI connector); the sensor free-runs (esp_video
  ///       handles CSI/ISP/LDO). Unlike the M5Stack Tab5 there is no IO expander
  ///       to pulse the camera reset. The callback runs in the camera task's
  ///       context.
  bool initialize_camera(const camera_frame_callback_t &callback,
                         const espp::Task::BaseConfig &task_config = {.name = "p4mdk_camera",
                                                                      .stack_size_bytes = 6144,
                                                                      .priority = 5,
                                                                      .core_id = 0});

  /// Stop the camera stream and release the camera pipeline.
  /// \note Safe to call from the camera frame callback itself: the capture
  ///       task cannot join itself, so in that context the stop is deferred -
  ///       the task tears the pipeline down and exits right after the callback
  ///       returns, and the exited task object is cleaned up by the next
  ///       stop_camera() or initialize_camera() call from another context.
  void stop_camera();

  /// Get the width of the captured camera frames, in pixels
  /// \return The camera frame width (0 if the camera is not initialized)
  uint16_t camera_width() const;

  /// Get the height of the captured camera frames, in pixels
  /// \return The camera frame height (0 if the camera is not initialized)
  uint16_t camera_height() const;

  /////////////////////////////////////////////////////////////////////////////
  // uSD / TF Card (4-bit SDMMC)
  /////////////////////////////////////////////////////////////////////////////

  /// Mount point for the uSD card filesystem.
  static constexpr char mount_point[] = "/sdcard";

  /// Configuration for the uSD card.
  struct SdCardConfig {
    bool format_if_mount_failed = false;    ///< Format the card if the mount fails.
    int max_files = 5;                      ///< Maximum number of open files.
    size_t allocation_unit_size = 2 * 1024; ///< FAT allocation unit size in bytes.
  };

  /// Initialize the microSD / TF card (4-bit SDMMC, powered by the on-chip LDO).
  /// \param config Configuration for the uSD card.
  /// \return True if the card was successfully mounted at \c mount_point.
  bool initialize_sdcard(const SdCardConfig &config);

  /// \return True if the SD card is present and mounted.
  bool is_sd_card_available() const { return sd_card_initialized_; }

  /// \return The SDMMC card handle, or nullptr if not initialized.
  sdmmc_card_t *sdcard() const { return sdcard_; }

  /// Get total/free space of the mounted card.
  /// \param size_mb Optional out: total size in MB.
  /// \param free_mb Optional out: free space in MB.
  /// \return True if the info was retrieved.
  bool get_sd_card_info(uint32_t *size_mb, uint32_t *free_mb) const;

protected:
  Esp32P4ModuleDevKit();

  bool audio_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);
  bool microphone_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);
  bool update_touch();

  /////////////////////////////////////////////////////////////////////////////
  // Display geometry / per-panel parameters
  /////////////////////////////////////////////////////////////////////////////
  // Per-panel parameters (geometry, DPI clock, and the DPI video timing porches).
  // The active panel is selected via Kconfig. On the ESP32-P4-Module-DEV-KIT both panels
  // are reset over DSI (no reset GPIO) and neither has a backlight GPIO (the
  // backlight is driven by an on-board I2C controller), so backlight_io and
  // reset_io are GPIO_NUM_NC for both.
  struct PanelParams {
    size_t width;
    size_t height;
    int dpi_clock_freq_mhz;
    int lane_bitrate_mbps;
    gpio_num_t backlight_io;
    gpio_num_t reset_io;
    int hsync_pulse_width, hsync_back_porch, hsync_front_porch;
    int vsync_pulse_width, vsync_back_porch, vsync_front_porch;
  };
  // EK79007 7" 1024x600 (reset over DSI, no backlight GPIO)
  static constexpr PanelParams EK79007_PARAMS{1024, 600, 52,  900, GPIO_NUM_NC, GPIO_NUM_NC,
                                              10,   160, 160, 1,   23,          12};
  // ILI9881C 10.1" 800x1280 (hsync: pulse=40, back=140, front=40; reset over DSI)
  static constexpr PanelParams ILI9881C_PARAMS{800, 1280, 80, 1500, GPIO_NUM_NC, GPIO_NUM_NC,
                                               40,  140,  40, 4,    16,          16};
  // JD9365 10.1" 800x1280 (the panel Waveshare sells for this board; reset over
  // DSI, no backlight GPIO). Timing matches Waveshare's
  // JD9365_800_1280_PANEL_60HZ_DPI_CONFIG vendor timing.
  static constexpr PanelParams JD9365_PARAMS{800, 1280, 80, 1500, GPIO_NUM_NC, GPIO_NUM_NC,
                                             20,  20,   40, 4,    10,          30};

#if CONFIG_ESP32_P4_MODULE_DEV_KIT_DISPLAY_EK79007
  static constexpr DisplayController default_controller_ = DisplayController::EK79007;
#elif CONFIG_ESP32_P4_MODULE_DEV_KIT_DISPLAY_ILI9881C
  static constexpr DisplayController default_controller_ = DisplayController::ILI9881C;
#else
  static constexpr DisplayController default_controller_ = DisplayController::JD9365;
#endif

  // Runtime display geometry, set from the configured panel.
  PanelParams panel_params_{default_controller_ == DisplayController::ILI9881C  ? ILI9881C_PARAMS
                            : default_controller_ == DisplayController::EK79007 ? EK79007_PARAMS
                                                                                : JD9365_PARAMS};
  size_t display_width_{panel_params_.width};
  size_t display_height_{panel_params_.height};

  /// Apply the parameters (geometry/timing) for the given controller.
  void apply_panel_params(DisplayController controller);

  // MIPI-DSI common parameters. The ESP32-P4-Module-DEV-KIT wires 2 DSI data lanes and
  // powers the DSI PHY from the on-chip LDO channel 3 (VDD_MIPI_DPHY) at 2500 mV.
  static constexpr int mipi_dsi_lanes = 2;
  // DSI HS lane bit rate (Mbps/lane). 900 Mbps matches Espressif's official
  // panel bus configs; using the wrong rate mis-packs the pixel bits on the link.
  static constexpr int mipi_dsi_phy_ldo_channel = 3; // on-chip LDO_VO3 -> VDD_MIPI_DPHY
  static constexpr int mipi_dsi_phy_ldo_voltage_mv = 2500;

  static constexpr bool invert_colors = false;
  static constexpr auto rotation = espp::DisplayRotation::LANDSCAPE;
  static constexpr bool swap_color_order = false;
  // Panel is used in its native orientation; no display mirror/swap.
  static constexpr bool mirror_x = false;
  static constexpr bool mirror_y = false;
  static constexpr bool swap_xy = false;
  // touch -> display coordinate conversion. May need tuning per panel.
  static constexpr bool touch_swap_xy = false;
  static constexpr bool touch_invert_x = false;
  static constexpr bool touch_invert_y = false;

  // Touch (GT911) - interrupt/reset are NOT connected on this board
  static constexpr uint8_t gt911_default_address = 0x5D;
  static constexpr uint8_t gt911_backup_address = 0x14;

  // RMII pin mapping for the ESP32-P4-Module-DEV-KIT (IP101GRI PHY).
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

  /////////////////////////////////////////////////////////////////////////////
  // Internal I2C bus (ES8311 codec 0x18; shared with display touch / camera SCCB)
  /////////////////////////////////////////////////////////////////////////////
  static constexpr auto internal_i2c_port = I2C_NUM_0;
  static constexpr auto internal_i2c_clock_speed = 400 * 1000;
  static constexpr gpio_num_t internal_i2c_sda = GPIO_NUM_7;
  static constexpr gpio_num_t internal_i2c_scl = GPIO_NUM_8;

  /////////////////////////////////////////////////////////////////////////////
  // Audio (ES8311 + NS4150B), I2S peripheral
  /////////////////////////////////////////////////////////////////////////////
  static constexpr uint8_t es8311_i2c_address = 0x18;
  static constexpr auto audio_i2s_port = I2S_NUM_0;
  static constexpr gpio_num_t audio_mclk_io = GPIO_NUM_13;      // MCLK
  static constexpr gpio_num_t audio_sclk_io = GPIO_NUM_12;      // BCLK
  static constexpr gpio_num_t audio_lrck_io = GPIO_NUM_10;      // WS/LRCK
  static constexpr gpio_num_t audio_dout_io = GPIO_NUM_9;       // P4 -> codec DSDIN
  static constexpr gpio_num_t audio_din_io = GPIO_NUM_11;       // codec ASDOUT -> P4
  static constexpr gpio_num_t audio_pa_enable_io = GPIO_NUM_53; // NS4150B enable

  // Audio buffer sizing: one UPDATE_FREQUENCY period's worth of 16-bit frames.
  // The TX (speaker) path is configured mono (I2S_SLOT_MODE_MONO in
  // initialize_audio()) while the RX (microphone) path captures both slots as
  // stereo (see initialize_microphone()), so the channel count of the path
  // being sized is passed explicitly rather than hard-coded.
  static constexpr int TX_NUM_CHANNELS = 1;       // TX slot mode is mono
  static constexpr int RX_NUM_CHANNELS = 2;       // RX captures both (L,R) slots
  static constexpr int NUM_BYTES_PER_CHANNEL = 2; // 16-bit samples
  static constexpr int UPDATE_FREQUENCY = 60;
  static constexpr int calc_audio_buffer_size(int sample_rate, int num_channels) {
    // NOTE: divide the rate by the update frequency FIRST so the result is
    // always a whole number of frames. Multiplying first yields a partial
    // frame for rates that are not a multiple of the update frequency (e.g.
    // 8 kHz stereo -> 533 bytes), and reading/writing partial samples shifts
    // the I2S sample framing on every transfer - heard as loud static.
    return (sample_rate / UPDATE_FREQUENCY) * num_channels * NUM_BYTES_PER_CHANNEL;
  }

  // Internal I2C bus (shared by the ES8311 codec)
  I2c internal_i2c_{{.port = internal_i2c_port,
                     .sda_io_num = internal_i2c_sda,
                     .scl_io_num = internal_i2c_scl,
                     .sda_pullup_en = GPIO_PULLUP_ENABLE,
                     .scl_pullup_en = GPIO_PULLUP_ENABLE,
                     .clk_speed = internal_i2c_clock_speed}};

  // Audio
  std::atomic<bool> audio_initialized_{false};
  std::atomic<float> volume_{50.0f};
  std::atomic<bool> mute_{false};
  std::shared_ptr<I2c::Device<uint8_t>> es8311_i2c_device_;
  std::unique_ptr<espp::Task> audio_task_{nullptr};
  i2s_chan_handle_t audio_tx_handle{nullptr};
  i2s_std_config_t audio_std_cfg{};
  i2s_event_callbacks_t audio_tx_callbacks_{};
  std::vector<uint8_t> audio_tx_buffer;
  StreamBufferHandle_t audio_tx_stream{nullptr};
  std::atomic<bool> has_sound{false};

  // Microphone (ES8311 ADC, full duplex with the speaker)
  std::atomic<bool> microphone_initialized_{false};
  microphone_callback_t microphone_callback_{nullptr};
  std::unique_ptr<espp::Task> microphone_task_{nullptr};
  i2s_chan_handle_t audio_rx_handle{nullptr};
  // Whether i2s_channel_init_std_mode() has been applied to audio_rx_handle.
  // Std-mode init is one-shot per channel (there is no deinit short of
  // deleting the channel, and full-duplex channels must be created together),
  // so a failed initialize_microphone() keeps this set and a retry
  // reconfigures the existing mode instead of re-initializing it.
  bool audio_rx_std_initialized_{false};
  std::vector<uint8_t> audio_rx_buffer;
  // microphone volume (percent), mapped onto the ES8311 analog gain range
  std::atomic<float> mic_volume_{70.0f};

  // microSD (4-bit SDMMC, slot 0, fixed IO-MUX pins). Powered via on-chip LDO_VO4.
  static constexpr int sd_ldo_channel = 4;
  static constexpr gpio_num_t sd_clk_io = GPIO_NUM_43;
  static constexpr gpio_num_t sd_cmd_io = GPIO_NUM_44;
  static constexpr gpio_num_t sd_d0_io = GPIO_NUM_39;
  static constexpr gpio_num_t sd_d1_io = GPIO_NUM_40;
  static constexpr gpio_num_t sd_d2_io = GPIO_NUM_41;
  static constexpr gpio_num_t sd_d3_io = GPIO_NUM_42;

  std::atomic<bool> sd_card_initialized_{false};
  sdmmc_card_t *sdcard_{nullptr};
  // SD power-control driver (on-chip LDO). Owned by this class: created in
  // initialize_sdcard() and deleted there (sd_pwr_ctrl_del_on_chip_ldo) if the
  // mount fails; a successful mount keeps it alive for the life of the card.
  sd_pwr_ctrl_handle_t sd_pwr_ctrl_handle_{nullptr};

  /////////////////////////////////////////////////////////////////////////////
  // Interrupts (used by the optional interrupt-driven touch path)
  /////////////////////////////////////////////////////////////////////////////
  espp::Interrupt interrupts_{
      {.interrupts = {},
       .task_config = {.name = "p4mdk interrupts",
                       .stack_size_bytes = CONFIG_ESP32_P4_MODULE_DEV_KIT_INTERRUPT_STACK_SIZE,
                       .priority = CONFIG_ESP32_P4_MODULE_DEV_KIT_INTERRUPT_PRIORITY,
                       .core_id = CONFIG_ESP32_P4_MODULE_DEV_KIT_INTERRUPT_CORE_ID}}};

  /////////////////////////////////////////////////////////////////////////////
  // Touch (GT911) - shares internal_i2c_
  /////////////////////////////////////////////////////////////////////////////
  std::shared_ptr<I2c::Device<uint8_t>> touch_i2c_device_;
  std::shared_ptr<TouchDriver> touch_driver_;
  std::shared_ptr<TouchpadInput> touchpad_input_;
  mutable std::recursive_mutex touchpad_data_mutex_;
  TouchpadData touchpad_data_;
  touch_callback_t touch_callback_{nullptr};
  std::unique_ptr<espp::Task> touch_task_{nullptr};

  /////////////////////////////////////////////////////////////////////////////
  // Display state (MIPI-DSI). NOTE: there is no backlight GPIO / espp::Led on
  // this board; the backlight is driven by an on-board I2C controller. On the
  // 10.1" JD9365 panel brightness() writes that controller (addr 0x45, reg
  // 0x96); on other panels the stored brightness is best-effort only (see
  // src/video.cpp).
  /////////////////////////////////////////////////////////////////////////////
  // On-board I2C backlight controller (10.1" JD9365 panel)
  static constexpr uint8_t backlight_i2c_address = 0x45;
  std::shared_ptr<I2c::Device<uint8_t>> backlight_i2c_device_;
  std::atomic<float> brightness_{100.0f};
  std::shared_ptr<Display<Pixel>> display_;
  // Raw LVGL display handle cached for notify_lvgl_flush_ready(): on the
  // JD9365/DMA2D path that callback runs from the DMA2D ISR, so it must not
  // touch the display_ shared_ptr; it only needs this pointer to call
  // lv_display_flush_ready(). Set once in initialize_display().
  std::atomic<lv_display_t *> lvgl_display_{nullptr};
  std::shared_ptr<DisplayDriver> display_driver_{static_cast<DisplayDriver *>(nullptr)};
  struct LcdHandles {
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus{nullptr};
    esp_lcd_panel_io_handle_t io{nullptr};
    esp_lcd_panel_handle_t panel{nullptr};
  } lcd_handles_{};
  // Initialized from the Kconfig selection so get_display_controller() /
  // get_display_controller_name() report the configured panel even before
  // initialize_lcd() runs apply_panel_params() (which re-affirms it).
  DisplayController display_controller_{default_controller_};

  /////////////////////////////////////////////////////////////////////////////
  // Camera (MIPI-CSI via esp_video / V4L2). Sensor SCCB shares internal_i2c_.
  /////////////////////////////////////////////////////////////////////////////
  bool camera_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);
  // Tear down the capture pipeline (STREAMOFF, munmap, close, esp_video_deinit)
  // and reset the camera state. Idempotent; does NOT touch camera_task_, so it
  // is safe to call from the camera task itself on a fatal capture error
  // (Task::stop() there would self-join).
  void teardown_camera_pipeline();
  std::atomic<bool> camera_initialized_{false};
  camera_frame_callback_t camera_callback_{nullptr};
  std::unique_ptr<espp::Task> camera_task_{nullptr};
  // Set by stop_camera() when called from the camera task itself (i.e. from
  // the frame callback, which cannot join its own task); the task callback
  // observes it after the frame callback returns, tears down the pipeline,
  // and exits the task.
  std::atomic<bool> camera_stop_requested_{false};
  int camera_fd_{-1};               // MIPI-CSI capture device (/dev/video0)
  bool camera_video_inited_{false}; // esp_video_init() succeeded (needs deinit)
  // atomic: read by camera_width()/camera_height() from other tasks (e.g. the
  // example's status task) while the camera task can concurrently reset them
  // in teardown_camera_pipeline() on a fatal capture error
  std::atomic<uint16_t> camera_width_{0};
  std::atomic<uint16_t> camera_height_{0};
  static constexpr int CAMERA_BUFFER_COUNT = 2;
  void *camera_buffers_[CAMERA_BUFFER_COUNT]{nullptr, nullptr};
  size_t camera_buffer_sizes_[CAMERA_BUFFER_COUNT]{0, 0};
  int camera_buffer_count_{0}; // buffers VIDIOC_REQBUFS actually allocated

  void flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);
  static bool notify_lvgl_flush_ready(esp_lcd_panel_handle_t panel,
                                      esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx);

  // DSI command helpers (used by the panel drivers)
  void dsi_write_command(uint8_t cmd, std::span<const uint8_t> params, uint32_t flags);
  void dsi_read_command(uint8_t cmd, std::span<uint8_t> data, uint32_t flags);
}; // class Esp32P4ModuleDevKit
} // namespace espp

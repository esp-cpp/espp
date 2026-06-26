#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <sdkconfig.h>

#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <driver/sdmmc_host.h>

#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>

#if CONFIG_ESP_P4_EV_BOARD_ETHERNET
#include <esp_eth.h>
#include <esp_netif.h>
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>

#include "base_component.hpp"
#include "display.hpp"
#include "display_drivers.hpp"
#include "ek79007.hpp"
#include "es8311.hpp"
#include "gt911.hpp"
#include "i2c.hpp"
#include "ili9881.hpp"
#include "interrupt.hpp"
#include "led.hpp"
#include "task.hpp"
#include "touchpad_input.hpp"

namespace espp {
/// @brief Board Support Package (BSP) for the Espressif ESP32-P4 Function EV
///        Board used with the ESP32-P4-HMI-Subboard.
///
/// This class provides a singleton interface to the board's peripherals:
/// - MIPI-DSI display (Kconfig-selectable EK79007 1024x600 or ILI9881C 800x1280)
///   with a GT911 capacitive multi-touch controller
/// - ES8311 audio codec (+ NS4150B speaker amplifier) over I2S
/// - 10/100 Ethernet (EMAC + IP101 RMII PHY)
/// - microSD card (4-bit SDMMC)
/// - MIPI-CSI camera (SC2336/OV5647) — pins wired, capture pipeline is a stub
///
/// \note The BOOT button cannot be used simultaneously with the ethernet PHY,
///       since the BOOT button is connected to the PHY's RMII_TXD1 pin. If you
///       need to use the BOOT button, you must disable the ethernet PHY.
///
/// All on-board control lines are direct ESP32-P4 GPIOs (this board has no I/O
/// expander). The display, touch, and audio codec share a single I2C bus
/// (SDA=GPIO7, SCL=GPIO8). The touch interrupt is not routed to the ESP32-P4 on
/// this board, so touch is polled.
///
/// The class is a singleton and can be accessed using the get() method.
///
/// \section esp32_p4_function_ev_board_example Example
/// \snippet esp32_p4_function_ev_board_example.cpp esp32 p4 function ev board example
class Esp32P4FunctionEvBoard : public BaseComponent {
public:
  /// Alias for the button callback function
  using button_callback_t = espp::Interrupt::event_callback_fn;

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
  enum class DisplayController { UNKNOWN, EK79007, ILI9881C };

  /// Mount point for the uSD card
  static constexpr char mount_point[] = "/sdcard";

  /// Default touch INT GPIO used by initialize_touch(). GPIO_NUM_NC means the
  /// GT911 is polled; if interrupt-driven touch is enabled via Kconfig this is
  /// the configured GPIO (CONFIG_ESP_P4_EV_BOARD_TOUCH_INTERRUPT_GPIO).
#if CONFIG_ESP_P4_EV_BOARD_TOUCH_INTERRUPT
  static constexpr gpio_num_t touch_interrupt_default =
      static_cast<gpio_num_t>(CONFIG_ESP_P4_EV_BOARD_TOUCH_INTERRUPT_GPIO);
#else
  static constexpr gpio_num_t touch_interrupt_default = GPIO_NUM_NC;
#endif

  /// @brief Access the singleton instance
  /// @return Reference to the singleton instance
  static Esp32P4FunctionEvBoard &get() {
    static Esp32P4FunctionEvBoard instance;
    return instance;
  }

  Esp32P4FunctionEvBoard(const Esp32P4FunctionEvBoard &) = delete;
  Esp32P4FunctionEvBoard &operator=(const Esp32P4FunctionEvBoard &) = delete;
  Esp32P4FunctionEvBoard(Esp32P4FunctionEvBoard &&) = delete;
  Esp32P4FunctionEvBoard &operator=(Esp32P4FunctionEvBoard &&) = delete;

  /// Get a reference to the internal I2C bus
  /// \return A reference to the internal I2C bus
  /// \note Shared by the GT911 touch, ES8311 codec, and camera SCCB
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
  ///        default based on the detected panel width is used.
  /// \return true if the display was successfully initialized, false otherwise
  bool initialize_display(size_t pixel_buffer_size = 0);

  /// Initialize the GT911 multi-touch controller
  /// \param callback The touchpad callback
  /// \param interrupt_pin GPIO wired to the GT911 touch INT pin. If GPIO_NUM_NC
  ///        (the default, unless interrupt-driven touch is enabled via Kconfig),
  ///        the GT911 is polled in a task. If a valid GPIO is provided, touch is
  ///        read from a GPIO interrupt on that pin instead of polling.
  /// \return true if the touchpad was successfully initialized, false otherwise
  /// \note The HMI subboard does not route the GT911 INT pin to the ESP32-P4 by
  ///       default (hence polling); the LCD expansion header exposes it, so wiring
  ///       it to a free GPIO enables the interrupt-driven path.
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
  TouchpadData touchpad_data() const { return touchpad_data_; }

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
  void brightness(float brightness);

  /// Get the display brightness
  /// \return The brightness as a percentage (0-100)
  float brightness() const;

  /// Get the display width in pixels (of the detected/active panel)
  /// \return The display width in pixels
  /// \note Valid after initialize_lcd() has detected the panel.
  size_t display_width() const { return display_width_; }

  /// Get the display height in pixels (of the detected/active panel)
  /// \return The display height in pixels
  /// \note Valid after initialize_lcd() has detected the panel.
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

  /////////////////////////////////////////////////////////////////////////////
  // Audio System (ES8311 + NS4150B)
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the audio system (ES8311 codec)
  /// \param sample_rate The audio sample rate in Hz (default 48kHz)
  /// \param task_config The task configuration for the audio task
  /// \return true if the audio system was successfully initialized
  bool initialize_audio(uint32_t sample_rate = 48000,
                        const espp::Task::BaseConfig &task_config = {
                            .name = "p4_ev_audio",
                            .stack_size_bytes = CONFIG_ESP_P4_EV_BOARD_AUDIO_TASK_STACK_SIZE,
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
  /// \param data The audio data to play
  /// \param num_bytes The number of bytes to play
  void play_audio(const uint8_t *data, uint32_t num_bytes);

  /// Play audio data
  /// \param data The audio data to play
  void play_audio(std::span<const uint8_t> data);

  /////////////////////////////////////////////////////////////////////////////
  // uSD Card (4-bit SDMMC)
  /////////////////////////////////////////////////////////////////////////////

  /// Configuration for the uSD card
  struct SdCardConfig {
    bool format_if_mount_failed = false;    ///< Format the uSD card if mount failed
    int max_files = 5;                      ///< The maximum number of files to open at once
    size_t allocation_unit_size = 2 * 1024; ///< The allocation unit size in bytes
  };

  /// Initialize microSD / uSD card
  /// \param config Configuration for the uSD card
  /// \return True if uSD card was successfully initialized
  bool initialize_sdcard(const SdCardConfig &config);

  /// Check if SD card is present and mounted
  /// \return True if SD card is available
  bool is_sd_card_available() const { return sd_card_initialized_; }

  /// Get the uSD card handle
  /// \return A pointer to the uSD card, or nullptr if not initialized
  sdmmc_card_t *sdcard() const { return sdcard_; }

  /// Get SD card info
  /// \param size_mb Pointer to store size in MB
  /// \param free_mb Pointer to store free space in MB
  /// \return True if info retrieved successfully
  bool get_sd_card_info(uint32_t *size_mb, uint32_t *free_mb) const;

#if CONFIG_ESP_P4_EV_BOARD_ETHERNET || defined(_DOXYGEN_)
  /////////////////////////////////////////////////////////////////////////////
  // Ethernet (EMAC + IP101 RMII PHY)
  /////////////////////////////////////////////////////////////////////////////

  /// Callback invoked when the Ethernet link goes up (with the assigned IP)
  using ethernet_link_callback_t = std::function<void(esp_ip4_addr_t ip)>;

  /// Initialize the Ethernet interface (EMAC + IP101 RMII PHY, DHCP client)
  /// \param on_link_up Optional callback invoked when an IP address is acquired
  /// \return True if Ethernet was successfully initialized and started
  /// \note Requires the ESP-IDF default event loop. The BSP creates it if needed.
  bool initialize_ethernet(const ethernet_link_callback_t &on_link_up = nullptr);

  /// Check whether the Ethernet link is up (cable connected + negotiated)
  /// \return True if the link is up
  bool is_ethernet_connected() const { return ethernet_connected_; }

  /// Get the most recently acquired IPv4 address (0 if none)
  /// \return The IPv4 address
  esp_ip4_addr_t ethernet_ip() const { return ethernet_ip_; }
#endif // CONFIG_ESP_P4_EV_BOARD_ETHERNET || defined(_DOXYGEN_)

  /////////////////////////////////////////////////////////////////////////////
  // Camera (MIPI-CSI) — pins wired, capture pipeline is a stub
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the MIPI-CSI camera (SC2336/OV5647).
  /// \return True if successful
  /// \note Not yet implemented — the camera pins/SCCB are documented in this
  ///       BSP but the esp_video capture pipeline is not wired up. This always
  ///       returns false for now.
  bool initialize_camera();

  /////////////////////////////////////////////////////////////////////////////
  // Button (BOOT)
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the BOOT button
  /// \param callback The callback function to call when pressed/released
  /// \return True if the button was successfully initialized
  bool initialize_button(const button_callback_t &callback = nullptr);

  /// Get the button state
  /// \return True if pressed, false otherwise
  bool button_state() const;

protected:
  Esp32P4FunctionEvBoard();

  bool update_touch();
  bool audio_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);

  /////////////////////////////////////////////////////////////////////////////
  // Display geometry / per-panel parameters
  /////////////////////////////////////////////////////////////////////////////
  // Per-panel parameters (geometry, DPI clock, backlight + reset GPIOs, and the
  // DPI video timing porches). The active panel is detected at runtime (see
  // detect_display_controller()), falling back to the Kconfig-selected default.
  struct PanelParams {
    size_t width;
    size_t height;
    int dpi_clock_freq_mhz;
    gpio_num_t backlight_io;
    gpio_num_t reset_io;
    int hsync_pulse_width, hsync_back_porch, hsync_front_porch;
    int vsync_pulse_width, vsync_back_porch, vsync_front_porch;
  };
  // EK79007 7" 1024x600
  static constexpr PanelParams EK79007_PARAMS{1024, 600, 52, GPIO_NUM_26, GPIO_NUM_27, 10,
                                              160,  160, 1,  23,          12};
  // ILI9881C 10.1" 800x1280 (hsync: pulse=40, back=140, front=40)
  static constexpr PanelParams ILI9881C_PARAMS{800, 1280, 80, GPIO_NUM_23, GPIO_NUM_NC, 40,
                                               140, 40,   4,  16,          16};

#if CONFIG_ESP_P4_EV_BOARD_DISPLAY_ILI9881C
  static constexpr DisplayController default_controller_ = DisplayController::ILI9881C;
#else
  static constexpr DisplayController default_controller_ = DisplayController::EK79007;
#endif

  // Runtime display geometry, set from the detected/configured panel.
  PanelParams panel_params_{default_controller_ == DisplayController::ILI9881C ? ILI9881C_PARAMS
                                                                               : EK79007_PARAMS};
  size_t display_width_{panel_params_.width};
  size_t display_height_{panel_params_.height};

  /// Apply the parameters (geometry/timing/GPIOs) for the given controller.
  /// \note The panel is selected via Kconfig; the ESP32-P4-HMI-Subboard does not
  ///       provide a way to safely auto-probe it (the EK79007 does not answer
  ///       DSI reads), matching Espressif's own esp-bsp which is Kconfig-driven.
  void apply_panel_params(DisplayController controller);

  // MIPI-DSI common parameters
  static constexpr int mipi_dsi_lanes = 2;
  // DSI HS lane bit rate. 900 Mbps matches Espressif's official
  // EK79007_PANEL_BUS_DSI_2CH_CONFIG; using the wrong rate (e.g. 1000) mis-packs
  // the pixel bits on the link, which shows up as shifted/oversaturated colors
  // (notably on mid-tones / alpha-blended content) while white/black look fine.
  static constexpr int mipi_dsi_lane_bitrate_mbps = 900;
  static constexpr int mipi_dsi_phy_ldo_channel = 3; // on-chip LDO_VO3 -> VDD_MIPI_DPHY
  static constexpr int mipi_dsi_phy_ldo_voltage_mv = 2500;

  static constexpr bool backlight_value = true;
  static constexpr bool invert_colors = false;
  static constexpr auto rotation = espp::DisplayRotation::LANDSCAPE;
  static constexpr bool swap_color_order = false;
  // Panel is used in its native orientation; no display mirror/swap.
  static constexpr bool mirror_x = false;
  static constexpr bool mirror_y = false;
  static constexpr bool swap_xy = false;
  // touch -> display coordinate conversion. The esp-bsp GT911 config on this
  // board mirrors x and y, so invert both axes here. May need tuning per panel.
  static constexpr bool touch_swap_xy = false;
  static constexpr bool touch_invert_x = true;
  static constexpr bool touch_invert_y = true;

  /////////////////////////////////////////////////////////////////////////////
  // Internal I2C bus (GT911 touch 0x5D/0x14, ES8311 codec 0x18, camera SCCB)
  /////////////////////////////////////////////////////////////////////////////
  static constexpr auto internal_i2c_port = I2C_NUM_0;
  static constexpr auto internal_i2c_clock_speed = 400 * 1000;
  static constexpr gpio_num_t internal_i2c_sda = GPIO_NUM_7;
  static constexpr gpio_num_t internal_i2c_scl = GPIO_NUM_8;

  // Touch (GT911) — interrupt/reset are NOT connected on this board
  static constexpr uint8_t gt911_default_address = 0x5D;
  static constexpr uint8_t gt911_backup_address = 0x14;

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

  /////////////////////////////////////////////////////////////////////////////
  // microSD (4-bit SDMMC, slot 0, fixed IO-MUX pins). Powered via on-chip LDO_VO4.
  /////////////////////////////////////////////////////////////////////////////
  static constexpr int sd_ldo_channel = 4;
  static constexpr gpio_num_t sd_clk_io = GPIO_NUM_43;
  static constexpr gpio_num_t sd_cmd_io = GPIO_NUM_44;
  static constexpr gpio_num_t sd_d0_io = GPIO_NUM_39;
  static constexpr gpio_num_t sd_d1_io = GPIO_NUM_40;
  static constexpr gpio_num_t sd_d2_io = GPIO_NUM_41;
  static constexpr gpio_num_t sd_d3_io = GPIO_NUM_42;

  /////////////////////////////////////////////////////////////////////////////
  // Camera (MIPI-CSI) — SCCB shares the internal I2C bus; reset/xclk not connected
  /////////////////////////////////////////////////////////////////////////////
  static constexpr gpio_num_t camera_reset_io = GPIO_NUM_NC;
  static constexpr gpio_num_t camera_xclk_io = GPIO_NUM_NC;

  // USB OTG (documented; USB stack not initialized by the BSP)
  static constexpr gpio_num_t usb_dp_io = GPIO_NUM_20;
  static constexpr gpio_num_t usb_dn_io = GPIO_NUM_19;

  // BOOT button (strapping pin). NOTE: GPIO35 is also Ethernet RMII TXD1, so the
  // button cannot be used as a runtime input while Ethernet is enabled (claiming
  // it as a GPIO would take down Ethernet TX). initialize_button() refuses to run
  // while Ethernet is up.
  static constexpr gpio_num_t button_io = GPIO_NUM_35;

  // Audio buffer sizing
  static constexpr int NUM_CHANNELS = 2;
  static constexpr int NUM_BYTES_PER_CHANNEL = 2;
  static constexpr int UPDATE_FREQUENCY = 60;
  static constexpr int calc_audio_buffer_size(int sample_rate) {
    return sample_rate * NUM_CHANNELS * NUM_BYTES_PER_CHANNEL / UPDATE_FREQUENCY;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Member variables
  /////////////////////////////////////////////////////////////////////////////
  I2c internal_i2c_{{.port = internal_i2c_port,
                     .sda_io_num = internal_i2c_sda,
                     .scl_io_num = internal_i2c_scl,
                     .sda_pullup_en = GPIO_PULLUP_ENABLE,
                     .scl_pullup_en = GPIO_PULLUP_ENABLE,
                     .clk_speed = internal_i2c_clock_speed}};

  // Interrupts (button)
  espp::Interrupt::PinConfig button_interrupt_pin_{
      .gpio_num = button_io,
      .callback =
          [this](const auto &event) {
            if (button_callback_) {
              button_callback_(event);
            }
          },
      .active_level = espp::Interrupt::ActiveLevel::LOW,
      .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
      .pullup_enabled = true};

  espp::Interrupt interrupts_{
      {.interrupts = {},
       .task_config = {.name = "p4-ev interrupts",
                       .stack_size_bytes = CONFIG_ESP_P4_EV_BOARD_INTERRUPT_STACK_SIZE,
                       .priority = CONFIG_ESP_P4_EV_BOARD_INTERRUPT_PRIORITY,
                       .core_id = CONFIG_ESP_P4_EV_BOARD_INTERRUPT_CORE_ID}}};

  // Touch
  std::shared_ptr<I2c::Device<uint8_t>> touch_i2c_device_;
  std::shared_ptr<TouchDriver> touch_driver_;
  std::shared_ptr<TouchpadInput> touchpad_input_;
  std::recursive_mutex touchpad_data_mutex_;
  TouchpadData touchpad_data_;
  touch_callback_t touch_callback_{nullptr};
  std::unique_ptr<espp::Task> touch_task_{nullptr};

  // Button
  button_callback_t button_callback_{nullptr};

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

  // uSD card
  std::atomic<bool> sd_card_initialized_{false};
  sdmmc_card_t *sdcard_{nullptr};
  void *sd_pwr_ctrl_handle_{nullptr};

#if CONFIG_ESP_P4_EV_BOARD_ETHERNET
  // Ethernet
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
#endif

  // Display state
  std::shared_ptr<Display<Pixel>> display_;
  std::shared_ptr<DisplayDriver> display_driver_{static_cast<DisplayDriver *>(nullptr)};
  std::shared_ptr<Led> backlight_;
  std::vector<Led::ChannelConfig> backlight_channel_configs_;
  struct LcdHandles {
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus{nullptr};
    esp_lcd_panel_io_handle_t io{nullptr};
    esp_lcd_panel_handle_t panel{nullptr};
  } lcd_handles_{};
  DisplayController display_controller_{DisplayController::UNKNOWN};

  void flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);
  static bool notify_lvgl_flush_ready(esp_lcd_panel_handle_t panel,
                                      esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx);

  // DSI command helpers (used by the panel drivers)
  void dsi_write_command(uint8_t cmd, std::span<const uint8_t> params, uint32_t flags);
  void dsi_read_command(uint8_t cmd, std::span<uint8_t> data, uint32_t flags);
}; // class Esp32P4FunctionEvBoard
} // namespace espp

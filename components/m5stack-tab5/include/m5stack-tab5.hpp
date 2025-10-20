#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <driver/sdmmc_host.h>
#include <driver/spi_master.h>
#include <driver/uart.h>
#include <hal/spi_types.h>

#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_interface.h>
#include <esp_lcd_panel_io.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/stream_buffer.h>
#include <freertos/task.h>

#include "base_component.hpp"
#include "bmi270.hpp"
#include "display.hpp"
#include "es7210.hpp"
#include "es8388.hpp"
#include "gt911.hpp"
#include "i2c.hpp"
#include "ili9881.hpp"
#include "ina226.hpp"
#include "interrupt.hpp"
#include "led.hpp"
#include "pi4ioe5v.hpp"
#include "touchpad_input.hpp"

namespace espp {
/// The M5StackTab5 class provides an interface to the M5Stack Tab5 development board.
///
/// The class provides access to the following features:
/// - 5" 720p MIPI-DSI Display with GT911 multi-touch
/// - Dual audio codecs (ES8388 + ES7210 AEC)
/// - BMI270 6-axis IMU sensor
/// - SC2356 2MP camera via MIPI-CSI
/// - ESP32-C6 wireless module (Wi-Fi 6, Thread, ZigBee)
/// - USB-A Host and USB-C OTG ports
/// - RS-485 industrial interface
/// - Grove and M5-Bus expansion headers
/// - microSD card slot
/// - NP-F550 removable battery with power management
/// - Real-time clock (RX8130CE)
/// - Multiple buttons and interrupts
///
/// The class is a singleton and can be accessed using the get() method.
///
/// \section m5stack_tab5_example Example
/// \snippet m5stack_tab5_example.cpp m5stack tab5 example
class M5StackTab5 : public BaseComponent {
public:
  /// Alias for the button callback function
  using button_callback_t = espp::Interrupt::event_callback_fn;

  /// Alias for the pixel type used by the Tab5 display
  using Pixel = lv_color16_t;

  /// Alias for the display driver used by the Tab5
  using DisplayDriver = espp::Ili9881;

  /// Alias for the GT911 touch controller used by the Tab5
  using TouchDriver = espp::Gt911;

  /// Alias for the touchpad data used by the Tab5 touchpad
  using TouchpadData = espp::TouchpadData;

  /// Alias the IMU used by the Tab5
  using Imu = espp::Bmi270<espp::bmi270::Interface::I2C>;

  /// Alias for the touch callback when touch events are received
  using touch_callback_t = std::function<void(const TouchpadData &)>;

  /// Camera data callback function
  using camera_callback_t = std::function<void(const uint8_t *data, size_t length)>;

  /// Mount point for the uSD card on the TDeck.
  static constexpr char mount_point[] = "/sdcard";

  /// Battery status structure
  struct BatteryStatus {
    float voltage_v;      ///< Battery voltage in volts
    float current_ma;     ///< Battery current in milliamps
    float power_mw;       ///< Battery power in milliwatts
    float charge_percent; ///< Estimated charge percentage (0-100)
    bool is_charging;     ///< True if battery is charging
    bool is_present;      ///< True if battery is present
  };

  /// Expansion port configuration
  enum class ExpansionPort {
    GROVE,   ///< Grove connector
    M5_BUS,  ///< M5-Bus connector
    STAMP,   ///< STAMP expansion pads
    GPIO_EXT ///< GPIO extension header
  };

  /// @brief Access the singleton instance of the M5StackTab5 class
  /// @return Reference to the singleton instance of the M5StackTab5 class
  static M5StackTab5 &get() {
    static M5StackTab5 instance;
    return instance;
  }

  M5StackTab5(const M5StackTab5 &) = delete;
  M5StackTab5 &operator=(const M5StackTab5 &) = delete;
  M5StackTab5(M5StackTab5 &&) = delete;
  M5StackTab5 &operator=(M5StackTab5 &&) = delete;

  /// Get a reference to the internal I2C bus
  /// \return A reference to the internal I2C bus
  /// \note The internal I2C bus is used for touchscreen, audio codecs, IMU, RTC, and power
  /// monitoring
  I2c &internal_i2c() { return internal_i2c_; }

  /// Get a reference to the interrupts
  /// \return A reference to the interrupts
  espp::Interrupt &interrupts() { return interrupts_; }

  /////////////////////////////////////////////////////////////////////////////
  // Display & Touchpad
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the LCD (low level display driver, MIPI-DSI + ST7703)
  /// \return true if the LCD was successfully initialized, false otherwise
  bool initialize_lcd();

  /// Initialize the LVGL display
  /// \param pixel_buffer_size The size of the pixel buffer
  /// \return true if the display was successfully initialized, false otherwise
  bool initialize_display(size_t pixel_buffer_size = 1280 * 720 / 10);

  /// Initialize the GT911 multi-touch controller
  /// \param callback The touchpad callback
  /// \return true if the touchpad was successfully initialized, false otherwise
  bool initialize_touch(const touch_callback_t &callback = nullptr);

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
  /// \param btn_state The button state (0 = button released, 1 = button pressed)
  void touchpad_read(uint8_t *num_touch_points, uint16_t *x, uint16_t *y, uint8_t *btn_state);

  /// Convert touchpad data from raw reading to display coordinates
  /// \param data The touchpad data to convert
  /// \return The converted touchpad data
  /// \note Uses the touch_invert_x and touch_invert_y settings to determine
  ///       if the x and y coordinates should be inverted
  TouchpadData touchpad_convert(const TouchpadData &data) const;

  /// Set the display brightness
  /// \param brightness The brightness as a percentage (0-100)
  void brightness(float brightness);

  /// Get the display brightness
  /// \return The brightness as a percentage (0-100)
  float brightness() const;

  /// Enable/disable the LCD backlight (routes through IO expander if mapped)
  void set_backlight_enabled(bool enable);

  /// Query backlight enable state if readable
  /// \return true if enabled, false if disabled; std::nullopt if unknown
  std::optional<bool> is_backlight_enabled() const;

  /// Get the display width in pixels
  /// \return The display width in pixels
  static constexpr size_t display_width() { return display_width_; }

  /// Get the display height in pixels
  /// \return The display height in pixels
  static constexpr size_t display_height() { return display_height_; }

  /// Get the display width in pixels, according to the current orientation
  /// \return The display width in pixels, according to the current orientation
  size_t rotated_display_width() const;

  /// Get the display height in pixels, according to the current orientation
  /// \return The display height in pixels, according to the current orientation
  size_t rotated_display_height() const;

  /////////////////////////////////////////////////////////////////////////////
  // Audio System
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the dual audio system (ES8388 codec + ES7210 AEC)
  /// \param sample_rate The audio sample rate (default 48kHz)
  /// \param task_config The task configuration for the audio task
  /// \return true if the audio system was successfully initialized, false otherwise
  bool initialize_audio(uint32_t sample_rate = 48000,
                        const espp::Task::BaseConfig &task_config = {
                            .name = "tab5_audio",
                            .stack_size_bytes = CONFIG_M5STACK_TAB5_AUDIO_TASK_STACK_SIZE,
                            .priority = 20,
                            .core_id = 1});

  /// Enable or disable the audio system
  /// \param enable True to enable, false to disable
  void enable_audio(bool enable);

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

  /// Play audio data
  /// \param data The audio data to play
  /// \param num_bytes The number of bytes to play
  void play_audio(const uint8_t *data, uint32_t num_bytes);

  /// Play audio data
  /// \param data The audio data to play
  void play_audio(const std::vector<uint8_t> &data);

  /// Start recording audio
  /// \param callback Function to call with recorded audio data
  /// \return True if recording started successfully
  bool start_audio_recording(std::function<void(const uint8_t *data, size_t length)> callback);

  /// Stop recording audio
  void stop_audio_recording();

  /// Test audio output with a simple tone
  /// \param frequency_hz The frequency of the test tone in Hz
  /// \param duration_ms The duration of the test tone in milliseconds
  void test_audio_output(uint16_t frequency_hz = 1000, uint16_t duration_ms = 1000);

  /////////////////////////////////////////////////////////////////////////////
  // Camera System
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the SC2356 2MP camera
  /// \param callback Function to call with camera frame data
  /// \return True if camera was successfully initialized
  bool initialize_camera(const camera_callback_t &callback = nullptr);

  /// Start camera capture
  /// \param width Frame width (max 1600)
  /// \param height Frame height (max 1200)
  /// \return True if capture started successfully
  bool start_camera_capture(uint16_t width = 1600, uint16_t height = 1200);

  /// Stop camera capture
  void stop_camera_capture();

  /// Take a single photo
  /// \param callback Function to call with photo data
  /// \return True if photo capture initiated successfully
  bool take_photo(const camera_callback_t &callback);

  /////////////////////////////////////////////////////////////////////////////
  // IMU & Sensors
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the BMI270 6-axis IMU
  /// \param orientation_filter Optional orientation filter function
  /// \return True if IMU was successfully initialized
  bool initialize_imu(const Imu::filter_fn &orientation_filter = nullptr);

  /// Get the IMU instance
  /// \return Shared pointer to the IMU
  std::shared_ptr<Imu> imu() const { return imu_; }

  /////////////////////////////////////////////////////////////////////////////
  // Power Management & Battery
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize battery monitoring (INA226)
  /// \return True if battery monitoring was successfully initialized
  bool initialize_battery_monitoring();

  /// Get the current battery status
  /// \return Battery status structure
  BatteryStatus get_battery_status();

  /// Enable or disable battery charging
  /// \param enable True to enable charging, false to disable
  void enable_battery_charging(bool enable);

  /// Set the system power mode
  /// \param low_power True for low power mode, false for normal mode
  void set_power_mode(bool low_power);

  /////////////////////////////////////////////////////////////////////////////
  // Real-Time Clock
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the RX8130CE real-time clock
  /// \return True if RTC was successfully initialized
  bool initialize_rtc();

  /// Set the RTC time
  /// \param unix_timestamp Unix timestamp to set
  /// \return True if time was set successfully
  bool set_rtc_time(uint64_t unix_timestamp);

  /// Get the RTC time
  /// \return Unix timestamp, or 0 if RTC not initialized
  uint64_t get_rtc_time();

  /// Enable RTC wake-up interrupt
  /// \param seconds_from_now Seconds from now to wake up
  /// \param callback Function to call on wake-up
  /// \return True if wake-up was set successfully
  bool set_rtc_wakeup(uint32_t seconds_from_now, std::function<void()> callback = nullptr);

  /////////////////////////////////////////////////////////////////////////////
  // Buttons & GPIO
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the button
  /// \param callback The callback function to call when pressed
  /// \return True if button was successfully initialized
  bool initialize_button(const button_callback_t &callback = nullptr);

  /// Get the button state
  /// \return True if pressed, false otherwise
  bool button_state() const;

  /////////////////////////////////////////////////////////////////////////////
  // IO Expanders (PI4IOE5V6408)
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the on-board IO expanders at addresses 0x43 and 0x44
  /// Configures required directions and safe default output states.
  bool initialize_io_expanders();

  /// Control the LCD reset (active-low) routed via IO expander (0x43 P4)
  /// \param assert_reset=true drives reset low; false releases reset high.
  /// \return true on success
  bool lcd_reset(bool assert_reset);

  /// Control the GT911 touch reset (active-low) via IO expander (0x43 P5)
  /// \param assert_reset=true drives reset low; false releases reset high.
  /// \return true on success
  bool touch_reset(bool assert_reset);

  /// Enable/disable the speaker amplifier (NS4150B SPK_EN on 0x43 P1)
  /// \param enable True to enable speaker, false to disable
  /// \return true on success
  bool set_speaker_enabled(bool enable);

  /// Enable/disable battery charging (IP2326 CHG_EN on 0x44 P7)
  /// \param enable True to enable charging, false to disable
  /// \return true on success
  bool set_charging_enabled(bool enable);

  /// Read battery charging status (IP2326 CHG_STAT on 0x44 P6)
  /// Returns true if charging is indicated asserted.
  bool charging_status();

  /// Generic helpers to control IO expander pins (0x43/0x44)
  /// These perform read-modify-write on the output latch.
  /// \param address 7-bit expander I2C address (e.g. 0x43 or 0x44)
  /// \param bit     Bit index 0..7
  /// \param level   Desired output level
  /// \return true on success
  bool set_io_expander_output(uint8_t address, uint8_t bit, bool level);

  /// Read a single output bit from the expander output register
  /// \param address 7-bit expander I2C address (e.g. 0x43 or 0x44)
  /// \param bit Bit index 0..7
  /// \return std::optional<bool> containing the output state, or std::nullopt on error
  std::optional<bool> get_io_expander_output(uint8_t address, uint8_t bit);

  /// Read a single input bit from the expander input register
  /// \param address 7-bit expander I2C address (e.g. 0x43 or 0x44)
  /// \param bit Bit index 0..7
  /// \return std::optional<bool> containing the input state, or std::nullopt on error
  std::optional<bool> get_io_expander_input(uint8_t address, uint8_t bit);

  /////////////////////////////////////////////////////////////////////////////
  // Expansion & Communication
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the RS-485 interface
  /// \param baud_rate The baud rate for RS-485 communication
  /// \param enable_termination True to enable 120Ω termination
  /// \return True if RS-485 was successfully initialized
  bool initialize_rs485(uint32_t baud_rate = 115200, bool enable_termination = false);

  /// Send data via RS-485
  /// \param data The data to send
  /// \param length The length of data to send
  /// \return Number of bytes sent, or -1 on error
  int rs485_send(const uint8_t *data, size_t length);

  /// Receive data via RS-485
  /// \param buffer Buffer to store received data
  /// \param max_length Maximum length to receive
  /// \param timeout_ms Timeout in milliseconds
  /// \return Number of bytes received, or -1 on error
  int rs485_receive(uint8_t *buffer, size_t max_length, uint32_t timeout_ms = 1000);

  /////////////////////////////////////////////////////////////////////////////
  // uSD Card
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
  bool is_sd_card_available() const;

  /// Get the uSD card
  /// \return A pointer to the uSD card
  /// \note The uSD card is only available if it was successfully initialized
  ///       and the mount point is valid
  sdmmc_card_t *sdcard() const { return sdcard_; }

  /// Get SD card info
  /// \param size_mb Pointer to store size in MB
  /// \param free_mb Pointer to store free space in MB
  /// \return True if info retrieved successfully
  bool get_sd_card_info(uint32_t *size_mb, uint32_t *free_mb) const;

  /// Initialize USB host functionality
  /// \return True if USB host was successfully initialized
  bool initialize_usb_host();

  /// Initialize USB device (OTG) functionality
  /// \return True if USB device was successfully initialized
  bool initialize_usb_device();

  /////////////////////////////////////////////////////////////////////////////
  // ESP32-C6 Wireless Module
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the ESP32-C6 wireless module
  /// \return True if wireless module was successfully initialized
  bool initialize_wireless();

  /// Send command to ESP32-C6 module
  /// \param command The command to send
  /// \param response Buffer to store response
  /// \param max_response_len Maximum response length
  /// \param timeout_ms Timeout in milliseconds
  /// \return Length of response, or -1 on error
  int send_wireless_command(const char *command, char *response, size_t max_response_len,
                            uint32_t timeout_ms = 5000);

protected:
  M5StackTab5();

  bool update_touch();
  void update_battery_status();
  bool audio_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);

  // Hardware pin definitions based on Tab5 specifications

  // ESP32-P4 Main Controller pins
  static constexpr size_t display_width_ = 720;
  static constexpr size_t display_height_ = 1280;

  // Internal I2C (GT911 touch, ES8388/ES7210 audio, BMI270 IMU, RX8130CE RTC, INA226 power,
  // PI4IOE5V6408 IO expanders)
  static constexpr auto internal_i2c_port = I2C_NUM_0;
  static constexpr auto internal_i2c_clock_speed = 1000 * 1000;
  static constexpr gpio_num_t internal_i2c_sda = GPIO_NUM_31; // Int SDA
  static constexpr gpio_num_t internal_i2c_scl = GPIO_NUM_32; // Int SCL

  // IO expander bit mapping (can be adjusted if hardware changes)
  static constexpr uint8_t IO43_BIT_SPK_EN = 1;   // P1
  static constexpr uint8_t IO43_BIT_LCD_RST = 4;  // P4
  static constexpr uint8_t IO43_BIT_TP_RST = 5;   // P5
  static constexpr uint8_t IO44_BIT_CHG_EN = 7;   // P7
  static constexpr uint8_t IO44_BIT_CHG_STAT = 6; // P6

  // IOX pins (0x43 PI4IO)
  static constexpr int HP_DET_PIN = (1 << 7);  // HP_DETECT (via PI4IOE5V6408 P7)
  static constexpr int CAM_RST_PIN = (1 << 6); // CAM_RST (via PI4IOE5V6408 P6)
  static constexpr int TP_RST_PIN = (1 << 5);  // TP_RST (via PI4IOE5V6408 P5)
  static constexpr int LCD_RST_PIN = (1 << 4); // LCD_RST (via PI4IOE5V6408 P4)
  // NOTE: pin 3 is not used in Tab5 design
  static constexpr int EXT_5V_EN_PIN = (1 << 2); // EXT_5V_EN (via PI4IOE5V6408 P2)
  static constexpr int SPK_EN_PIN = (1 << 1);    // SPK_EN (via PI4IOE5V6408 P1)
  // NOTE: pin 0 is not used in Tab5 design

  static constexpr uint8_t IOX_0x43_OUTPUTS =
      CAM_RST_PIN | TP_RST_PIN | LCD_RST_PIN | EXT_5V_EN_PIN | SPK_EN_PIN;
  static constexpr uint8_t IOX_0x43_INPUTS = HP_DET_PIN;
  // 0 = input, 1 = output
  static constexpr uint8_t IOX_0x43_DIRECTION_MASK = IOX_0x43_OUTPUTS;
  static constexpr uint8_t IOX_0x43_HIGH_Z_MASK = 0x00;                 // No high-Z outputs
  static constexpr uint8_t IOX_0x43_DEFAULT_OUTPUTS = IOX_0x43_OUTPUTS; // All outputs high to start
  static constexpr uint8_t IOX_0x43_PULL_UPS =
      CAM_RST_PIN | TP_RST_PIN | LCD_RST_PIN | EXT_5V_EN_PIN | SPK_EN_PIN;
  static constexpr uint8_t IOX_0x43_PULL_DOWNS = 0;

  // IOX pins (0x44 PI4IO)
  static constexpr int CHG_EN_PIN = (1 << 7);       // CHG_EN (via PI4IOE5V6408 P7)
  static constexpr int CHG_STAT_PIN = (1 << 6);     // CHG_STAT (via PI4IOE5V6408 P6)
  static constexpr int N_CHG_QC_EN_PIN = (1 << 5);  // N_CHG_QC_EN (via PI4IOE5V6408 P5)
  static constexpr int PWROFF_PLUSE_PIN = (1 << 4); // PWROFF_PLUSE (via PI4IOE5V6408 P4)
  static constexpr int USB_5V_EN_PIN = (1 << 3);    // USB_5V_EN (via PI4IOE5V6408 P5)
  // NOTE: pin 2 is not used in Tab5 design
  // NOTE: pin 1 is not used in Tab5 design
  static constexpr int WLAN_PWR_EN_PIN = (1 << 0); // WLAN_PWR_EN (via PI4IOE5V6408 P0)

  static constexpr uint8_t IOX_0x44_OUTPUTS =
      CHG_EN_PIN | N_CHG_QC_EN_PIN | PWROFF_PLUSE_PIN | USB_5V_EN_PIN | WLAN_PWR_EN_PIN;
  static constexpr uint8_t IOX_0x44_INPUTS = CHG_STAT_PIN;
  // 0 = input, 1 = output
  static constexpr uint8_t IOX_0x44_DIRECTION_MASK = IOX_0x44_OUTPUTS;
  static constexpr uint8_t IOX_0x44_HIGH_Z_MASK = (1 << 2) | (1 << 1); // P2, P1 are high-Z
  static constexpr uint8_t IOX_0x44_DEFAULT_OUTPUTS =
      WLAN_PWR_EN_PIN | USB_5V_EN_PIN; // Default outputs
  static constexpr uint8_t IOX_0x44_PULL_UPS =
      USB_5V_EN_PIN | WLAN_PWR_EN_PIN | PWROFF_PLUSE_PIN | N_CHG_QC_EN_PIN | CHG_EN_PIN;
  static constexpr uint8_t IOX_0x44_PULL_DOWNS = CHG_STAT_PIN;

  // button
  static constexpr gpio_num_t button_io = GPIO_NUM_35; // BOOT button

  // Display & Touch
  static constexpr gpio_num_t lcd_backlight_io = GPIO_NUM_22;   // LEDA
  static constexpr gpio_num_t touch_interrupt_io = GPIO_NUM_23; // TP_INT
  static constexpr bool backlight_value = true;
  static constexpr bool invert_colors = false;
  static constexpr auto rotation = espp::DisplayRotation::LANDSCAPE;
  static constexpr bool mirror_x = false;
  static constexpr bool mirror_y = false;
  static constexpr bool swap_xy = false;
  static constexpr bool swap_color_order = false;
  // touch
  static constexpr bool touch_swap_xy = false;
  static constexpr bool touch_invert_x = false;
  static constexpr bool touch_invert_y = false;

  // Audio
  static constexpr gpio_num_t audio_cdata_io = GPIO_NUM_31;  // CDATA (shared with I2C)
  static constexpr gpio_num_t audio_cclk_io = GPIO_NUM_32;   // CCLK (shared with I2C)
  static constexpr gpio_num_t audio_mclk_io = GPIO_NUM_30;   // MCLK (shared ES8388/ES7210)
  static constexpr gpio_num_t audio_sclk_io = GPIO_NUM_27;   // SCLK (shared ES8388/ES7210)
  static constexpr gpio_num_t audio_lrck_io = GPIO_NUM_29;   // LRCK (shared ES8388/ES7210)
  static constexpr gpio_num_t audio_dsdin_io = GPIO_NUM_26;  // ES8388 DSDIN
  static constexpr gpio_num_t audio_asdout_io = GPIO_NUM_28; // ES7210 ASDOUT

  // Camera
  static constexpr gpio_num_t camera_scl_io = GPIO_NUM_32;  // CAM_SCL (shared with I2C)
  static constexpr gpio_num_t camera_sda_io = GPIO_NUM_31;  // CAM_SDA (shared with I2C)
  static constexpr gpio_num_t camera_mclk_io = GPIO_NUM_36; // CAM_MCLK
  static constexpr gpio_num_t camera_reset_io = GPIO_NUM_6; // CAM_RST (via PI4IOE5V6408 P6)

  // ESP32-C6 Communication (SDIO)
  static constexpr gpio_num_t c6_sdio_d0_io = GPIO_NUM_11;  // SDIO2_D0
  static constexpr gpio_num_t c6_sdio_d1_io = GPIO_NUM_10;  // SDIO2_D1
  static constexpr gpio_num_t c6_sdio_d2_io = GPIO_NUM_9;   // SDIO2_D2
  static constexpr gpio_num_t c6_sdio_d3_io = GPIO_NUM_8;   // SDIO2_D3
  static constexpr gpio_num_t c6_sdio_cmd_io = GPIO_NUM_13; // SDIO2_CMD
  static constexpr gpio_num_t c6_sdio_clk_io = GPIO_NUM_12; // SDIO2_CK
  static constexpr gpio_num_t c6_reset_io = GPIO_NUM_15;    // C6 RESET
  static constexpr gpio_num_t c6_io2_io = GPIO_NUM_14;      // C6 IO2

  // microSD (SPI)
  static constexpr gpio_num_t sd_miso_io = GPIO_NUM_39; // MISO/DAT0
  static constexpr gpio_num_t sd_cs_io = GPIO_NUM_42;   // CS/DAT3
  static constexpr gpio_num_t sd_sck_io = GPIO_NUM_43;  // SCK/CLK
  static constexpr gpio_num_t sd_mosi_io = GPIO_NUM_44; // MOSI/CMD

  // microSD (SDIO / SDMMC)
  static constexpr gpio_num_t sd_dat0_io = GPIO_NUM_39; // MISO/DAT0
  static constexpr gpio_num_t sd_dat1_io = GPIO_NUM_40; // MISO/DAT0
  static constexpr gpio_num_t sd_dat2_io = GPIO_NUM_41; // MISO/DAT0
  static constexpr gpio_num_t sd_dat3_io = GPIO_NUM_42; // CS/DAT3
  static constexpr gpio_num_t sd_clk_io = GPIO_NUM_43;  // SCK/CLK
  static constexpr gpio_num_t sd_cmd_io = GPIO_NUM_44;  // MOSI/CMD

  // RS-485
  static constexpr gpio_num_t rs485_rx_io = GPIO_NUM_21;  // RX
  static constexpr gpio_num_t rs485_tx_io = GPIO_NUM_20;  // TX
  static constexpr gpio_num_t rs485_dir_io = GPIO_NUM_34; // DIR

  // M5-Bus expansion pins
  static constexpr gpio_num_t m5bus_mosi_io = GPIO_NUM_18; // MOSI
  static constexpr gpio_num_t m5bus_miso_io = GPIO_NUM_19; // MISO
  static constexpr gpio_num_t m5bus_sck_io = GPIO_NUM_5;   // SCK
  static constexpr gpio_num_t m5bus_rxd0_io = GPIO_NUM_38; // RXD0
  static constexpr gpio_num_t m5bus_txd0_io = GPIO_NUM_37; // TXD0
  static constexpr gpio_num_t m5bus_pc_rx_io = GPIO_NUM_7; // PC_RX
  static constexpr gpio_num_t m5bus_pc_tx_io = GPIO_NUM_6; // PC_TX

  // Grove connector
  static constexpr gpio_num_t grove_gpio1_io = GPIO_NUM_53; // Yellow
  static constexpr gpio_num_t grove_gpio2_io = GPIO_NUM_54; // White

  // Additional GPIO
  static constexpr gpio_num_t gpio_16_io = GPIO_NUM_16;
  static constexpr gpio_num_t gpio_17_io = GPIO_NUM_17;
  static constexpr gpio_num_t gpio_45_io = GPIO_NUM_45;
  static constexpr gpio_num_t gpio_52_io = GPIO_NUM_52;

  // Audio configuration
  static constexpr int NUM_CHANNELS = 2;
  static constexpr int NUM_BYTES_PER_CHANNEL = 2;
  static constexpr int UPDATE_FREQUENCY = 60;

  static constexpr int calc_audio_buffer_size(int sample_rate) {
    return sample_rate * NUM_CHANNELS * NUM_BYTES_PER_CHANNEL / UPDATE_FREQUENCY;
  }

  // Member variables
  I2c internal_i2c_{{.port = internal_i2c_port,
                     .sda_io_num = internal_i2c_sda,
                     .scl_io_num = internal_i2c_scl,
                     .sda_pullup_en = GPIO_PULLUP_ENABLE,
                     .scl_pullup_en = GPIO_PULLUP_ENABLE,
                     .timeout_ms = 200, // needs to be long enough for writing imu config file (8k)
                     .clk_speed = 400'000}};

  // Interrupt configurations
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

  espp::Interrupt::PinConfig touch_interrupt_pin_{.gpio_num = touch_interrupt_io,
                                                  .callback =
                                                      [this](const auto &event) {
                                                        if (update_touch()) {
                                                          if (touch_callback_) {
                                                            touch_callback_(touchpad_data());
                                                          }
                                                        }
                                                      },
                                                  .active_level = espp::Interrupt::ActiveLevel::LOW,
                                                  .interrupt_type =
                                                      espp::Interrupt::Type::FALLING_EDGE,
                                                  .pullup_enabled = true};

  espp::Interrupt interrupts_{
      {.interrupts = {},
       .task_config = {.name = "tab5 interrupts",
                       .stack_size_bytes = CONFIG_M5STACK_TAB5_INTERRUPT_STACK_SIZE}}};

  // Component instances
  std::shared_ptr<TouchDriver> touch_driver_;
  std::shared_ptr<TouchpadInput> touchpad_input_;
  std::recursive_mutex touchpad_data_mutex_;
  TouchpadData touchpad_data_;
  touch_callback_t touch_callback_{nullptr};

  std::shared_ptr<Imu> imu_;

  // Button callbacks
  button_callback_t button_callback_{nullptr};

  // Audio system
  std::atomic<bool> audio_initialized_{false};
  std::atomic<float> volume_{50.0f};
  std::atomic<bool> mute_{false};
  std::unique_ptr<espp::Task> audio_task_{nullptr};
  i2s_chan_handle_t audio_tx_handle{nullptr};
  i2s_chan_handle_t audio_rx_handle{nullptr};
  i2s_std_config_t audio_std_cfg{};
  i2s_event_callbacks_t audio_tx_callbacks_{};
  i2s_event_callbacks_t audio_rx_callbacks_{};
  std::vector<uint8_t> audio_tx_buffer;
  std::vector<uint8_t> audio_rx_buffer;
  StreamBufferHandle_t audio_tx_stream;
  StreamBufferHandle_t audio_rx_stream;
  std::atomic<bool> has_sound{false};
  std::atomic<bool> recording_{false};
  std::function<void(const uint8_t *data, size_t length)> audio_rx_callback_{nullptr};

  // Camera system
  std::atomic<bool> camera_initialized_{false};
  camera_callback_t camera_callback_{nullptr};

  // Power management
  std::atomic<bool> battery_monitoring_initialized_{false};
  BatteryStatus battery_status_;
  std::mutex battery_mutex_;
  std::shared_ptr<espp::Ina226> ina226_;
  // IO expanders on the internal I2C (addresses 0x43 and 0x44 per Tab5 design)
  std::shared_ptr<espp::Pi4ioe5v> ioexp_0x43_;
  std::shared_ptr<espp::Pi4ioe5v> ioexp_0x44_;

  // Communication interfaces
  std::atomic<bool> rs485_initialized_{false};
  std::atomic<bool> sd_card_initialized_{false};
  std::atomic<bool> usb_host_initialized_{false};
  std::atomic<bool> usb_device_initialized_{false};
  std::atomic<bool> wireless_initialized_{false};

  // uSD Card
  sdmmc_card_t *sdcard_{nullptr};

  // RTC
  std::atomic<bool> rtc_initialized_{false};
  std::function<void()> rtc_wakeup_callback_{nullptr};

  // Display state
  std::shared_ptr<Display<Pixel>> display_;
  std::shared_ptr<Led> backlight_;
  std::vector<Led::ChannelConfig> backlight_channel_configs_;
  struct LcdHandles {
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus{nullptr}; // dsi bus handle
    esp_lcd_panel_io_handle_t io{nullptr};          // io handle
    esp_lcd_panel_handle_t panel{nullptr};          // color handle
  } lcd_handles_{};

  // original function pointer for the panel del, init
  esp_err_t (*original_panel_del_)(esp_lcd_panel_t *panel){nullptr};
  esp_err_t (*original_panel_init_)(esp_lcd_panel_t *panel){nullptr};

  void flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);
  static bool notify_lvgl_flush_ready(esp_lcd_panel_handle_t panel,
                                      esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx);

  // DSI write helpers
  void dsi_write_command(uint8_t cmd, std::span<const uint8_t> params, uint32_t flags);
  void dsi_read_command(uint8_t cmd, std::span<uint8_t> data, uint32_t flags);
}; // class M5StackTab5
} // namespace espp

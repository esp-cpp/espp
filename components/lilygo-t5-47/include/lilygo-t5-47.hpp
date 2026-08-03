#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>

#include "base_component.hpp"
#include "bm8563.hpp"
#include "bq27220.hpp"
#include "gt911.hpp"
#include "i2c.hpp"
#include "interrupt.hpp"
#include "pca9535.hpp"
#include "spi.hpp"
#include "sx126x.hpp"
#include "task.hpp"
#include "touchpad_input.hpp"

#include "lvgl.h"

extern "C" {
#include "epdiy.h"
}

namespace espp {
/// The LilyGoT547 class provides an interface to the LilyGo T5 4.7" ESP32-S3
/// e-paper development board.
///
/// The board drives an ED047TC1 960x540 16-level-grayscale e-paper panel over a
/// parallel bus (the ESP32-S3 LCD_CAM peripheral). This BSP wraps Espressif's
/// esp-idf-friendly build of the epdiy library, which owns the parallel timing,
/// waveforms, grayscale rendering and partial-update handling for this exact
/// board.
///
/// The class provides access to the following features:
/// - 4.7" 960x540 e-paper display (ED047TC1) via epdiy
///
/// The class is a singleton and can be accessed using the get() method.
///
/// \section lilygo_t5_47_example Example
/// \snippet lilygo_t5_47_example.cpp lilygo t5 47 example
class LilyGoT547 : public BaseComponent {
public:
  /// Native panel width in pixels
  static constexpr int panel_width = 960;
  /// Native panel height in pixels
  static constexpr int panel_height = 540;

  /// @brief Access the singleton instance of the LilyGoT547 class
  /// @return Reference to the singleton instance
  static LilyGoT547 &get() {
    static LilyGoT547 instance;
    return instance;
  }

  LilyGoT547(const LilyGoT547 &) = delete;
  LilyGoT547 &operator=(const LilyGoT547 &) = delete;
  LilyGoT547(LilyGoT547 &&) = delete;
  LilyGoT547 &operator=(LilyGoT547 &&) = delete;

  /// Initialize the e-paper display: bring up epdiy for this board (parallel
  /// bus + waveforms) and allocate the high-level grayscale framebuffer.
  /// \return true if the display was successfully initialized
  bool initialize_display();

  /// Power the display's high-voltage rails on. Required before an update; the
  /// panel draws significant current, so keep it off when idle.
  void power_on();

  /// Power the display's high-voltage rails off.
  void power_off();

  /// Clear the whole panel to white (a full refresh; removes ghosting).
  void clear();

  /// Get the epdiy high-level framebuffer (4 bits per pixel, 2 pixels per byte,
  /// panel_width x panel_height). Draw into it with epdiy's drawing functions,
  /// then call update().
  /// \return Pointer to the framebuffer, or nullptr if not initialized
  uint8_t *framebuffer();

  /// Push the current framebuffer contents to the panel.
  /// \param mode The epdiy draw mode (default MODE_GC16 for full 16-gray)
  /// \note The high-voltage rails must be powered (power_on()) for the update to
  ///       take effect.
  void update(EpdDrawMode mode = MODE_GC16);

  /// Get the current display width (respecting the configured rotation)
  /// \return Width in pixels
  int width() const { return epd_rotated_display_width(); }

  /// Get the current display height (respecting the configured rotation)
  /// \return Height in pixels
  int height() const { return epd_rotated_display_height(); }

  /// Get the ambient temperature (degrees C) used for the e-paper waveform.
  /// \return Temperature in degrees Celsius
  int temperature();

  /////////////////////////////////////////////////////////////////////////////
  // I2C
  /////////////////////////////////////////////////////////////////////////////

  /// Get the internal I2C bus. This is the bus epdiy uses to drive the e-paper
  /// power ICs (PCA9555 + TPS65185); this BSP creates it and hands the handle to
  /// epdiy so the same bus is shared with the board's other I2C peripherals
  /// (touch, RTC, battery gauge, the qwiic connector). It is created by
  /// initialize_display().
  /// \return Pointer to the espp::I2c, or nullptr if initialize_display() has
  ///         not been called
  espp::I2c *internal_i2c() { return internal_i2c_.get(); }

  /// Get the I2C bus exposed on the qwiic / STEMMA-QT connector. On this board
  /// the qwiic connector is wired to the internal I2C bus (SDA=39/SCL=40), so
  /// this returns the same bus as internal_i2c(). Add external devices to it
  /// with internal_i2c()->add_device<...>(...).
  /// \return Pointer to the espp::I2c, or nullptr if initialize_display() has
  ///         not been called
  espp::I2c *qwiic_i2c() { return internal_i2c_.get(); }

  /////////////////////////////////////////////////////////////////////////////
  // LVGL
  /////////////////////////////////////////////////////////////////////////////

  /// Set up LVGL with an 8-bit grayscale (L8) display whose flush writes into
  /// the epdiy framebuffer. Each LVGL refresh cycle's dirty areas are batched
  /// into a single panel update (see set_lvgl_update_mode() / full_refresh()),
  /// so many small UI changes become one e-paper refresh instead of one per
  /// change.
  /// \param buffer_lines Height, in lines, of the LVGL partial draw buffer
  ///        (double-buffered, allocated in PSRAM). Larger means fewer flushes
  ///        per refresh at the cost of more memory.
  /// \return true on success. initialize_display() must have been called first.
  bool initialize_lvgl(int buffer_lines = 60);

  /// Get the LVGL display created by initialize_lvgl().
  /// \return The LVGL display, or nullptr if LVGL has not been initialized
  lv_display_t *lvgl_display() const { return lvgl_display_; }

  /// Set the epdiy draw mode used for LVGL-driven panel updates. Default is
  /// MODE_GC16 (full 16-level grayscale, highest quality but slowest). Use a
  /// faster mono mode (e.g. MODE_DU) for snappier black-and-white updates.
  /// \param mode The epdiy draw mode
  void set_lvgl_update_mode(EpdDrawMode mode) { lvgl_update_mode_ = mode; }

  /// Force a full grayscale (MODE_GC16) refresh of the whole panel. Use this to
  /// clear ghosting that builds up after repeated partial updates.
  void full_refresh();

  /// Set the display rotation. Rotates the e-paper (via epdiy) and resizes the
  /// LVGL display to match, so an LVGL UI re-lays-out for the new dimensions.
  /// Touch coordinates are transformed to match the rotation automatically.
  /// \param rotation The epdiy rotation (EPD_ROT_LANDSCAPE / EPD_ROT_PORTRAIT /
  ///        EPD_ROT_INVERTED_LANDSCAPE / EPD_ROT_INVERTED_PORTRAIT)
  void set_rotation(EpdRotation rotation);

  /// Cycle to the next display rotation.
  void rotate();

  /// Get the current display rotation.
  /// \return The current EpdRotation
  EpdRotation rotation() const { return rotation_; }

  /////////////////////////////////////////////////////////////////////////////
  // Buttons
  /////////////////////////////////////////////////////////////////////////////

  /// Alias for the button callback (an interrupt event handler)
  using button_callback_t = espp::Interrupt::event_callback_fn;

  /// Initialize the BOOT button (GPIO0). The callback fires on press and
  /// release.
  /// \param callback Called with each button interrupt event
  /// \return true on success
  /// \note This wires only the BOOT button (GPIO0). The board's other buttons
  ///       are elsewhere: the "IO48"-labelled button is on the PCA9535 expander
  ///       (see io48_button_pressed()), and the PWR button is handled by the
  ///       board's power-management IC (not exposed here).
  bool initialize_button(const button_callback_t &callback = nullptr);

  /// Get the BOOT button state
  /// \return true if pressed, false otherwise
  bool button_state() const;

  /// Get the interrupts manager (created by initialize_button()).
  /// \return Pointer to the espp::Interrupt, or nullptr if no button was
  ///         initialized
  espp::Interrupt *interrupts() { return interrupts_.get(); }

  /////////////////////////////////////////////////////////////////////////////
  // Touch (GT911)
  /////////////////////////////////////////////////////////////////////////////

  /// Alias for the touch callback
  using touch_callback_t = std::function<void(const espp::TouchpadData &)>;

  /// Initialize the GT911 capacitive touch controller on the internal I2C bus,
  /// and register an LVGL input device for it.
  /// \param callback Called (from the touch interrupt) with each new touch state
  /// \return true on success. initialize_display() must have been called first
  ///         (it creates the shared I2C bus).
  /// \note The GT911's reported orientation may not match the panel's; if touch
  ///       is mirrored/rotated on hardware, adjust touch_swap_xy / touch_invert_x
  ///       / touch_invert_y.
  bool initialize_touch(const touch_callback_t &callback = nullptr);

  /// Get the LVGL touchpad input device created by initialize_touch().
  /// \return The touchpad input, or nullptr if touch was not initialized
  std::shared_ptr<espp::TouchpadInput> touchpad_input() const { return touchpad_input_; }

  /// Get the latest touchpad data (thread-safe copy).
  /// \return The most recent espp::TouchpadData
  espp::TouchpadData touchpad_data() const;

  /// Read the latest touchpad data (signature used by espp::TouchpadInput).
  void touchpad_read(uint8_t *num_touch_points, uint16_t *x, uint16_t *y, uint8_t *btn_state);

  /// Get the state of the capacitive home button (the touch key below the
  /// display). The GT911 reports it as a key press, separate from finger touch
  /// points, so it is available even when num_touch_points is 0.
  /// \return true if the home button is currently pressed
  /// \note Requires initialize_touch(). Whether the key is active depends on the
  ///       GT911 configuration flashed on the board; flagged for hardware
  ///       verification.
  bool home_button_pressed() const;

  /////////////////////////////////////////////////////////////////////////////
  // RTC (PCF8563)
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the PCF8563 real-time clock on the internal I2C bus.
  /// \return true on success. initialize_display() must have been called first
  ///         (it creates the shared I2C bus).
  /// \note The PCF8563 is register-compatible with the BM8563, so this uses the
  ///       espp::Bm8563 driver.
  bool initialize_rtc();

  /// Get the RTC driver (created by initialize_rtc()).
  /// \return Pointer to the espp::Bm8563, or nullptr if the RTC was not
  ///         initialized
  espp::Bm8563 *rtc() { return rtc_.get(); }

  /////////////////////////////////////////////////////////////////////////////
  // Battery (BQ27220 fuel gauge)
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the BQ27220 battery fuel gauge on the internal I2C bus.
  /// \return true on success. initialize_display() must have been called first
  ///         (it creates the shared I2C bus).
  bool initialize_battery();

  /// Get the battery fuel gauge (created by initialize_battery()). Query it for
  /// voltage, current, state-of-charge, temperature, etc.
  /// \return Pointer to the espp::Bq27220, or nullptr if not initialized
  espp::Bq27220 *battery() { return battery_.get(); }

  /////////////////////////////////////////////////////////////////////////////
  // I/O expander (PCA9535)
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize a driver for the on-board PCA9535 I/O expander on the internal
  /// I2C bus.
  /// \return true on success. initialize_display() must have been called first.
  /// \warning This is the SAME physical expander (address 0x20) that epdiy drives
  ///          for the e-paper power ICs. epdiy owns port 1's high bits (output
  ///          enable, mode, power-up, VCOM, wakeup, power-good, INT). The driver
  ///          is created with auto_init=false so it does not reconfigure the
  ///          chip; only read inputs / drive port-0 pins epdiy does not use, and
  ///          always read-modify-write. Reconfiguring epdiy's bits will break the
  ///          display's power sequencing.
  bool initialize_io_expander();

  /// Get the PCA9535 I/O expander driver (created by initialize_io_expander()).
  /// \return Pointer to the espp::Pca9535, or nullptr if not initialized
  /// \note See initialize_io_expander()'s warning about epdiy co-ownership.
  espp::Pca9535 *io_expander() { return io_expander_.get(); }

  /// Read the "IO48"-labelled button, which is wired to the PCA9535 expander
  /// (pin PC12 = port 1, bit 2 - a pin epdiy does not use). This is not the PWR
  /// button (that is handled by the board's power-management IC).
  /// \return true if the button is currently pressed
  /// \note Requires initialize_io_expander().
  bool io48_button_pressed();

  /////////////////////////////////////////////////////////////////////////////
  // Frontlight
  /////////////////////////////////////////////////////////////////////////////

  /// Turn the e-paper frontlight on or off (BL_EN / GPIO11). The pin is
  /// configured as an output on first use.
  /// \param on true to enable the frontlight
  /// \note This is a simple on/off enable (active-high, flagged for hardware
  ///       verification). For dimming, drive GPIO11 with LEDC PWM instead.
  void set_frontlight(bool on);

  /// Get the current frontlight state.
  /// \return true if the frontlight is on
  bool frontlight_on() const { return frontlight_on_; }

  /////////////////////////////////////////////////////////////////////////////
  // Power
  /////////////////////////////////////////////////////////////////////////////

  /// Power the board off by putting the BQ25896 PMIC into ship mode (disconnect
  /// the battery / BATFET_DIS). The board then draws negligible current and is
  /// turned back on by pressing the PWR button.
  /// \return true if the ship-mode command was written successfully
  /// \note This only powers the board off when running on battery - if USB power
  ///       is connected the board stays powered. initialize_display() must have
  ///       been called first (it creates the shared I2C bus the PMIC is on).
  bool shutdown();

  /////////////////////////////////////////////////////////////////////////////
  // microSD Card
  /////////////////////////////////////////////////////////////////////////////

  /// The filesystem mount point for the microSD card
  static constexpr char mount_point[] = "/sdcard";

  /// Configuration for the microSD card
  struct SdCardConfig {
    bool format_if_mount_failed = false;    ///< Format the card if mounting fails
    int max_files = 5;                      ///< Maximum number of open files at once
    size_t allocation_unit_size = 2 * 1024; ///< Allocation unit size in bytes
  };

  /// Initialize the microSD card (SPI). Mounts a FAT filesystem at
  /// mount_point ("/sdcard").
  /// \param config The microSD card configuration
  /// \return true if the card was mounted successfully
  /// \note The card is on a dedicated SPI bus (it does not conflict with the
  ///       e-paper's parallel bus), so this can be called independently of
  ///       initialize_display().
  bool initialize_sdcard(const SdCardConfig &config);

  /// Get the mounted microSD card.
  /// \return Pointer to the sdmmc_card_t, or nullptr if not initialized
  sdmmc_card_t *sdcard() const { return sdcard_; }

  /////////////////////////////////////////////////////////////////////////////
  // LoRa Radio (SX1262)
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the LoRa radio (SX1262).
  /// \param radio_config The radio (modem) configuration to apply
  /// \return True if the radio was initialized properly
  /// \note The radio shares the SPI bus with the microSD card.
  /// \note The radio's DIO1 interrupt is automatically serviced via the shared
  ///       interrupt manager, so received packets / transmit completion trigger
  ///       the driver's callbacks (see espp::Sx126x::set_receive_callback etc.).
  bool initialize_lora(const espp::Sx126x::RadioConfig &radio_config = {});

  /// Get the LoRa radio.
  /// \return A shared pointer to the LoRa radio driver, or nullptr if
  ///         initialize_lora() has not succeeded
  std::shared_ptr<espp::Sx126x> lora() const { return lora_; }

  /// Get the GPIO pin for the LoRa radio chip select.
  static constexpr auto lora_cs_gpio() { return lora_cs_io; }
  /// Get the GPIO pin for the LoRa radio DIO1 (interrupt) line.
  static constexpr auto lora_dio1_gpio() { return lora_dio1_io; }
  /// Get the GPIO pin for the LoRa radio BUSY line.
  static constexpr auto lora_busy_gpio() { return lora_busy_io; }
  /// Get the GPIO pin for the LoRa radio reset line.
  static constexpr auto lora_reset_gpio() { return lora_reset_io; }

private:
  LilyGoT547();

  /// Lazily initialize the shared SPI bus used by the microSD card (and the
  /// board's other SPI peripherals). \return true if the bus is initialized.
  bool init_spi_bus();

  /// Lazily create the shared espp::Interrupt manager (used by both the buttons
  /// and the touch controller). epdiy's epd_init() must have installed the GPIO
  /// ISR service first. \return true if the manager exists.
  bool ensure_interrupts();

  /// Poll the GT911 for new touch data and cache it. \return true if new data.
  bool update_gt911();

  static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);
  void lvgl_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);

  std::atomic<bool> initialized_{false};
  std::atomic<bool> powered_on_{false};
  EpdiyHighlevelState hl_{};
  EpdRotation rotation_{EPD_ROT_LANDSCAPE};

  // Internal I2C bus (shared with epdiy's e-paper power ICs). Pins from the T5
  // 4.7" ePaper S3 PRO factory firmware; this is the same bus/pins epdiy's
  // lilygo_board_s3 board definition uses.
  static constexpr gpio_num_t internal_i2c_sda = GPIO_NUM_39;
  static constexpr gpio_num_t internal_i2c_scl = GPIO_NUM_40;
  std::unique_ptr<espp::I2c> internal_i2c_{nullptr};

  // LVGL
  lv_display_t *lvgl_display_{nullptr};
  uint8_t *lvgl_buffer0_{nullptr};
  uint8_t *lvgl_buffer1_{nullptr};
  EpdDrawMode lvgl_update_mode_{MODE_GC16};
  lv_area_t lvgl_dirty_{}; // accumulated dirty box for the current refresh cycle
  bool lvgl_dirty_valid_{false};

  // Buttons
  static constexpr gpio_num_t button_io = GPIO_NUM_0; // BOOT button
  std::atomic<bool> button_initialized_{false};
  button_callback_t button_callback_{nullptr};
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
  // Created lazily in initialize_button(): the espp::Interrupt constructor
  // installs the GPIO ISR service, but epdiy's epd_init() (in
  // initialize_display) must install it first, so this cannot be constructed
  // eagerly with the singleton.
  std::unique_ptr<espp::Interrupt> interrupts_{nullptr};

  // Touch (GT911) on the internal I2C bus. Pins from the T5 4.7" ePaper S3 PRO
  // factory firmware. RST is currently unused (the controller comes up at its
  // default address); it is kept here for a future explicit-reset sequence.
  static constexpr gpio_num_t touch_interrupt_io = GPIO_NUM_3;
  static constexpr gpio_num_t touch_reset_io = GPIO_NUM_9;
  // Orientation of the GT911 relative to the panel (landscape 960x540). The
  // controller reports in the portrait frame, so swap X/Y; an invert may also be
  // needed depending on which corner is the origin (hardware-verify).
  static constexpr bool touch_swap_xy = true;
  static constexpr bool touch_invert_x = false;
  static constexpr bool touch_invert_y = true;
  std::atomic<bool> touch_initialized_{false};
  std::shared_ptr<espp::I2c::Device<std::uint8_t>> touch_i2c_device_;
  std::unique_ptr<espp::Gt911> gt911_{nullptr};
  std::shared_ptr<espp::TouchpadInput> touchpad_input_{nullptr};
  mutable std::recursive_mutex touchpad_data_mutex_;
  espp::TouchpadData touchpad_data_{};
  touch_callback_t touch_callback_{nullptr};
  // The GT911's config on this board is query-mode: it does not reliably signal
  // via an edge on INT, so we poll it from a task (matching the vendor firmware,
  // which uses LOW_LEVEL_QUERY + isPressed()).
  std::unique_ptr<espp::Task> touch_task_{nullptr};

  // RTC (PCF8563) on the internal I2C bus. IRQ (GPIO2) is not wired here yet.
  static constexpr gpio_num_t rtc_interrupt_io = GPIO_NUM_2;
  std::atomic<bool> rtc_initialized_{false};
  std::shared_ptr<espp::I2c::Device<std::uint8_t>> rtc_i2c_device_;
  std::unique_ptr<espp::Bm8563> rtc_{nullptr};

  // Battery (BQ27220 fuel gauge) on the internal I2C bus.
  std::atomic<bool> battery_initialized_{false};
  std::shared_ptr<espp::I2c::Device<std::uint8_t>> battery_i2c_device_;
  std::unique_ptr<espp::Bq27220> battery_{nullptr};

  // I/O expander (PCA9535) on the internal I2C bus — the SAME chip epdiy drives
  // (see initialize_io_expander()). The IO48 button is pin PC12 = port 1, bit 2.
  static constexpr uint8_t io48_button_port1_mask = 0x04; // PC12
  std::atomic<bool> io_expander_initialized_{false};
  std::atomic<bool> io48_button_configured_{false};
  std::shared_ptr<espp::I2c::Device<std::uint8_t>> io_expander_i2c_device_;
  std::unique_ptr<espp::Pca9535> io_expander_{nullptr};

  // Frontlight (BL_EN). Configured as an output on first use.
  static constexpr gpio_num_t frontlight_io = GPIO_NUM_11;
  std::atomic<bool> frontlight_initialized_{false};
  std::atomic<bool> frontlight_on_{false};

  // BQ25896 PMIC (battery charger / power path) on the internal I2C bus. Used to
  // enter ship mode (power off). Device handle created lazily by shutdown().
  static constexpr uint8_t pmic_address = 0x6B;
  std::shared_ptr<espp::I2c::Device<std::uint8_t>> pmic_i2c_device_;

  // microSD card (SPI). Pins from the T5 4.7" ePaper S3 PRO factory firmware.
  static constexpr gpio_num_t spi_mosi_io = GPIO_NUM_13;
  static constexpr gpio_num_t spi_miso_io = GPIO_NUM_21;
  static constexpr gpio_num_t spi_sclk_io = GPIO_NUM_14;
  static constexpr gpio_num_t sdcard_cs = GPIO_NUM_12;
  static constexpr auto spi_num = SPI2_HOST;
  static constexpr int SPI_MAX_TRANSFER_BYTES = 4092;
  static constexpr int spi_queue_size = 6;
  std::unique_ptr<espp::Spi> spi_{nullptr};
  sdmmc_card_t *sdcard_{nullptr};

  // LoRa radio (SX1262) on the shared SPI bus. Pins from the T5 4.7" ePaper S3
  // PRO factory firmware.
  static constexpr gpio_num_t lora_cs_io = GPIO_NUM_46;
  static constexpr gpio_num_t lora_dio1_io = GPIO_NUM_10;
  static constexpr gpio_num_t lora_reset_io = GPIO_NUM_1;
  static constexpr gpio_num_t lora_busy_io = GPIO_NUM_47;
  static constexpr int lora_spi_clock_speed = 8 * 1000 * 1000;
  std::shared_ptr<espp::Spi::Device> lora_spi_device_;
  std::shared_ptr<espp::Sx126x> lora_;
  espp::Interrupt::PinConfig lora_dio1_interrupt_pin_{
      .gpio_num = lora_dio1_io,
      .callback =
          [this](const auto &) {
            if (lora_) {
              std::error_code ec;
              lora_->handle_dio1_interrupt(ec);
            }
          },
      .active_level = espp::Interrupt::ActiveLevel::HIGH,
      .interrupt_type = espp::Interrupt::Type::RISING_EDGE};
};
} // namespace espp

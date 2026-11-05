#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <esp_err.h>
#include <sdmmc_cmd.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <hal/spi_ll.h>
#include <hal/spi_types.h>

#include <freertos/FreeRTOS.h>

#include "audio.hpp"
#include "base_component.hpp"
#include "bmi270.hpp"
#include "fast_math.hpp"
#include "gps.hpp"
#include "i2c.hpp"
#include "interrupt.hpp"
#include "led.hpp"
#include "neopixel.hpp"
#include "oneshot_adc.hpp"
#include "sdcard.hpp"
#include "spi.hpp"
#include "st7789.hpp"
#include "sx126x.hpp"
#include "task.hpp"

namespace espp {
/// The M5StackCardputer class provides an interface to the M5Stack Cardputer
/// (K132) and Cardputer ADV, ESP32-S3 (StampS3) based card-sized computers
/// with a 56-key QWERTY keyboard. The variant is detected automatically at
/// runtime (see variant()); both share the same 56-key layout and the same
/// API.
///
/// The class provides access to the following features:
/// - Display (1.14" 240x135 IPS TFT, ST7789V2)
/// - Keyboard (original: 56-key matrix scanned through a 74HC138
///   demultiplexer; ADV: TCA8418 I2C keyboard controller)
/// - Audio output (mono speaker; original: NS4168 I2S amplifier; ADV: ES8311
///   codec + NS4150B amplifier, initialized automatically)
/// - Microphone (original: SPM1423 PDM; ADV: analog MEMS mic via the ES8311
///   codec)
/// - micro-SD (uSD) card (SPI mode)
/// - RGB LED (WS2812, on the StampS3 module)
/// - Battery voltage measurement
/// - G0 (BOOT) button
/// - IR transmitter and Grove port pin definitions
/// - Internal I2C bus accessor (ADV only; also hosts a BMI270 IMU at 0x68
///   which can be used with the espp bmi270 component)
/// - LoRa radio + GNSS receiver on the LoRa+GPS Cap (U201 / U214) expansion
///   module for the Cardputer ADV (SX1262 + ATGM336H)
///
/// The class is a singleton and can be accessed using the get() method.
///
/// \note On the original Cardputer the speaker and microphone cannot be used
///       at the same time: GPIO 43 doubles as the speaker's I2S word-select
///       and the PDM microphone's (MHz-range) clock - two different signals
///       on one physical pin - so initializing one while the other is active
///       will fail. On the Cardputer ADV both go through the ES8311 codec in
///       full duplex on a single I2S bus (shared bit/word clocks, separate
///       data pins), so the speaker and microphone can be used
///       simultaneously, at one sample rate.
/// \note The audio runs on espp::Audio: on the original a playback instance
///       (NS4168, no codec) or a recording instance (PDM microphone, on its
///       own I2S port); on the ADV one full-duplex instance with the ES8311
///       (clocked from BCLK; see CardputerAdvCodec in src/).
///
/// \section m5stack_cardputer_example Example
/// \snippet m5stack_cardputer_example.cpp m5stack-cardputer example
class M5StackCardputer : public BaseComponent {
public:
  /// Alias for the pixel type used by the display
  using Pixel = lv_color16_t;

  /// Alias for the display driver
  using DisplayDriver = espp::St7789;

  /// Alias for the button callback function
  using button_callback_t = std::function<void(const espp::Interrupt::Event &)>;

  /// Number of rows in the keyboard matrix
  static constexpr size_t KEYBOARD_ROWS = 4;
  /// Number of columns in the keyboard matrix
  static constexpr size_t KEYBOARD_COLS = 14;

  /// Special (non-printable) keys, produced by the Fn layer of the keyboard
  enum class SpecialKey : uint8_t {
    NONE = 0, ///< Not a special key
    ESC,      ///< Escape (fn + `)
    F1,       ///< F1 (fn + 1)
    F2,       ///< F2 (fn + 2)
    F3,       ///< F3 (fn + 3)
    F4,       ///< F4 (fn + 4)
    F5,       ///< F5 (fn + 5)
    F6,       ///< F6 (fn + 6)
    F7,       ///< F7 (fn + 7)
    F8,       ///< F8 (fn + 8)
    F9,       ///< F9 (fn + 9)
    F10,      ///< F10 (fn + 0)
    F11,      ///< F11 (fn + -)
    F12,      ///< F12 (fn + =)
    DELETE,   ///< Delete (fn + backspace)
    UP,       ///< Up arrow (fn + ;)
    DOWN,     ///< Down arrow (fn + .)
    LEFT,     ///< Left arrow (fn + ,)
    RIGHT,    ///< Right arrow (fn + /)
  };

  /// The state of the keyboard modifier keys
  struct Modifiers {
    bool fn{false};    ///< Fn key
    bool shift{false}; ///< Left shift key
    bool ctrl{false};  ///< Left ctrl key
    bool opt{false};   ///< Opt key
    bool alt{false};   ///< Left alt key
  };

  /// A single key state change reported by the keyboard scanner
  struct KeyEvent {
    uint8_t row;         ///< Row of the key in the matrix (0 = top / esc row)
    uint8_t col;         ///< Column of the key in the matrix (0 = leftmost)
    bool pressed;        ///< True if the key is now pressed, false if released
    char value;          ///< The character for the key, with the shift layer
                         ///  applied (0 if the key has no character, e.g. a
                         ///  modifier or an fn-layer special key). Backspace,
                         ///  tab, enter, and space are reported as '\b', '\t',
                         ///  '\n', and ' '.
    SpecialKey special;  ///< The special key (fn layer) if fn was held and the
                         ///  key has one, SpecialKey::NONE otherwise
    Modifiers modifiers; ///< The modifier state when the event was generated
  };

  /// Alias for the keypress callback function. Called once for each key that
  /// changes state during a keyboard scan.
  using keypress_callback_t = std::function<void(const KeyEvent &)>;

  /// Alias for the microphone data callback. Called with 16-bit signed mono
  /// samples from the microphone.
  using microphone_callback_t = std::function<void(const uint8_t *data, size_t num_bytes)>;

  /// Maximum number of bytes that can be transferred in a single SPI
  /// transaction to the display. 32k on the ESP32-S3.
  static constexpr size_t SPI_MAX_TRANSFER_BYTES = SPI_LL_DMA_MAX_BIT_LEN / 8;

  /// Mount point for the uSD card.
  static constexpr char mount_point[] = "/sdcard";

  /// @brief Access the singleton instance of the M5StackCardputer class
  /// @return Reference to the singleton instance of the M5StackCardputer class
  static M5StackCardputer &get() {
    static M5StackCardputer instance;
    return instance;
  }

  M5StackCardputer(const M5StackCardputer &) = delete;
  M5StackCardputer &operator=(const M5StackCardputer &) = delete;
  M5StackCardputer(M5StackCardputer &&) = delete;
  M5StackCardputer &operator=(M5StackCardputer &&) = delete;

  /// Get a reference to the interrupts
  /// \return A reference to the interrupts
  espp::Interrupt &interrupts();

  /////////////////////////////////////////////////////////////////////////////
  // Variant (original Cardputer vs Cardputer ADV)
  /////////////////////////////////////////////////////////////////////////////

  /// The hardware variant of the board
  enum class Variant : uint8_t {
    ORIGINAL, ///< Original Cardputer (K132): 74HC138 matrix keyboard, NS4168
              ///  I2S amplifier, SPM1423 PDM microphone
    ADV,      ///< Cardputer ADV: TCA8418 I2C keyboard controller, ES8311 codec
              ///  (speaker + microphone), BMI270 IMU on the internal I2C bus
  };

  /// Get the hardware variant of the board.
  /// \return The hardware variant of the board
  /// \note The first call detects the variant by probing for the ADV's
  ///       TCA8418 keyboard controller on the internal I2C bus (GPIO 8/9);
  ///       on the original those pins are then returned to plain GPIO for
  ///       the 74HC138 matrix.
  Variant variant();

  /// Get the name of a variant
  /// \param variant The variant to get the name of
  /// \return The name of the variant
  static const char *variant_name(Variant variant) {
    return variant == Variant::ADV ? "Cardputer ADV" : "Cardputer";
  }

  /// Get a pointer to the internal I2C bus (ADV only)
  /// \return A pointer to the internal I2C bus, or nullptr on the original
  ///         Cardputer (which has no internal I2C bus)
  /// \note On the ADV the internal bus hosts the TCA8418 keyboard controller
  ///       (0x34), the ES8311 codec (0x18), and a BMI270 IMU (0x68)
  I2c *internal_i2c();

  /////////////////////////////////////////////////////////////////////////////
  // Keyboard
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the keyboard
  /// \param callback The callback function to call when a key changes state
  /// \param poll_interval The interval at which to scan the keyboard matrix
  /// \param task_config The configuration for the keyboard scanner task. The
  ///        defaults come from the M5STACK_CARDPUTER_KEYBOARD_TASK_* Kconfig
  ///        options.
  /// \return true if the keyboard was successfully initialized, false
  ///         otherwise
  /// \note The keyboard is a 4x14 matrix (a 74HC138-scanned GPIO matrix on
  ///       the original, a TCA8418 I2C controller on the ADV); a scanner task
  ///       owned by this class polls it at the given interval and calls the
  ///       callback once per key state change.
  /// \note The callback runs in the scanner task's context, so the task's
  ///       stack must be large enough for whatever the callback calls into
  ///       (e.g. LVGL).
  bool
  initialize_keyboard(const keypress_callback_t &callback = nullptr,
                      std::chrono::milliseconds poll_interval = std::chrono::milliseconds(10),
                      const espp::Task::BaseConfig &task_config = {
                          .name = "keyboard",
                          .stack_size_bytes = CONFIG_M5STACK_CARDPUTER_KEYBOARD_TASK_STACK_SIZE,
                          .priority = CONFIG_M5STACK_CARDPUTER_KEYBOARD_TASK_PRIORITY,
                          .core_id = CONFIG_M5STACK_CARDPUTER_KEYBOARD_TASK_CORE_ID});

  /// Get whether a key is currently pressed
  /// \param row The row of the key in the matrix
  /// \param col The column of the key in the matrix
  /// \return true if the key is currently pressed, false otherwise
  bool is_key_pressed(uint8_t row, uint8_t col) const;

  /// Get the current state of the modifier keys
  /// \return The current state of the modifier keys
  Modifiers modifiers() const;

  /// Get the raw state of the keyboard matrix
  /// \return One entry per row; in each entry bit N is set if the key in
  ///         column N is currently pressed
  std::array<uint16_t, KEYBOARD_ROWS> keyboard_state() const;

  /// Get the character for a key, given a modifier state
  /// \param row The row of the key in the matrix
  /// \param col The column of the key in the matrix
  /// \param modifiers The modifier state to apply
  /// \return The character for the key (with the shift layer applied), or 0
  ///         if the key has no character
  static char key_value(uint8_t row, uint8_t col, const Modifiers &modifiers);

  /// Get the special key (fn layer) for a key
  /// \param row The row of the key in the matrix
  /// \param col The column of the key in the matrix
  /// \return The special key for the key, or SpecialKey::NONE if it has none
  static SpecialKey special_key(uint8_t row, uint8_t col);

  /// Get the name of a special key
  /// \param key The special key to get the name of
  /// \return The name of the special key
  static const char *special_key_name(SpecialKey key);

  /////////////////////////////////////////////////////////////////////////////
  // Button (G0 / BOOT)
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the G0 (BOOT) button
  /// \param callback The callback function to call when the button changes
  ///        state
  /// \return true if the button was successfully initialized, false otherwise
  bool initialize_button(const button_callback_t &callback = nullptr);

  /// Get the state of the G0 (BOOT) button
  /// \return true if the button is pressed, false otherwise
  bool button_state() const;

  /////////////////////////////////////////////////////////////////////////////
  // Display
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the LCD (low level display driver)
  /// \return true if the LCD was successfully initialized, false otherwise
  bool initialize_lcd();

  /// Initialize the display (lvgl display driver)
  /// \param pixel_buffer_size The size of the pixel buffer
  /// \return true if the display was successfully initialized, false otherwise
  bool initialize_display(size_t pixel_buffer_size);

  /// Get the width of the LCD in pixels
  /// \return The width of the LCD in pixels
  static constexpr size_t lcd_width() { return lcd_width_; }

  /// Get the height of the LCD in pixels
  /// \return The height of the LCD in pixels
  static constexpr size_t lcd_height() { return lcd_height_; }

  /// Get the GPIO pin for the LCD data/command signal
  /// \return The GPIO pin for the LCD data/command signal
  static constexpr auto get_lcd_dc_gpio() { return lcd_dc_io; }

  /// Get a shared pointer to the display
  /// \return A shared pointer to the display
  std::shared_ptr<Display<Pixel>> display() const;

  /// Get a shared pointer to the low-level display driver
  /// \return A shared pointer to the display driver
  const std::shared_ptr<DisplayDriver> &display_driver() const { return display_driver_; }

  /// Set the brightness of the backlight
  /// \param brightness The brightness of the backlight as a percentage (0 - 100)
  /// \note This function will only work after initialize_lcd() has been called
  void brightness(float brightness);

  /// Get the brightness of the backlight
  /// \return The brightness of the backlight as a percentage (0 - 100)
  /// \note This function will only work after initialize_lcd() has been called
  float brightness() const;

  /// Get the VRAM 0 pointer (DMA memory used by LVGL)
  /// \return The VRAM 0 pointer
  /// \note This is the memory used by LVGL for rendering
  /// \note This is null unless initialize_display() has been called
  Pixel *vram0() const;

  /// Get the VRAM 1 pointer (DMA memory used by LVGL)
  /// \return The VRAM 1 pointer
  /// \note This is the memory used by LVGL for rendering
  /// \note This is null unless initialize_display() has been called
  Pixel *vram1() const;

  /// Write a frame to the LCD
  /// \param x The x coordinate
  /// \param y The y coordinate
  /// \param width The width of the frame, in pixels
  /// \param height The height of the frame, in pixels
  /// \param data The data to write
  /// \note This method queues the data to be written to the LCD, only blocking
  ///      if there is an ongoing SPI transaction
  void write_lcd_frame(const uint16_t x, const uint16_t y, const uint16_t width,
                       const uint16_t height, uint8_t *data);

  /// Write lines to the LCD
  /// \param xs The x start coordinate
  /// \param ys The y start coordinate
  /// \param xe The x end coordinate
  /// \param ye The y end coordinate
  /// \param data The data to write
  /// \param user_data User data to pass to the SPI transaction callback
  /// \note This method queues the panel transfer asynchronously and may return
  ///       before the write has completed.
  void write_lcd_lines(int xs, int ys, int xe, int ye, const uint8_t *data, uint32_t user_data);

  /////////////////////////////////////////////////////////////////////////////
  // Audio (speaker + microphone)
  /////////////////////////////////////////////////////////////////////////////

  /// How initialize_audio() sets the audio system up
  struct AudioConfig {
    /// The audio sample rate, in Hz (playback and recording share it)
    uint32_t sample_rate{44100};
    /// Set up playback: the speaker (original: NS4168 amplifier; ADV: ES8311
    /// DAC + NS4150B amplifier) and the playback task. Without it
    /// play_audio() queues nothing.
    bool playback_enabled{true};
    /// Set up recording: the microphone (original: SPM1423 PDM microphone;
    /// ADV: the ES8311's ADC) and the microphone task. Without it
    /// start_audio_recording() fails.
    /// \note On the original Cardputer playback and recording cannot both be
    ///       enabled: GPIO 43 is both the speaker's word select and the PDM
    ///       microphone's clock (see the class notes). On the ADV they run
    ///       full duplex.
    /// \note Each I2S channel has a DMA ring in internal DMA-capable RAM, and
    ///       each task's stack is in internal RAM too, so a direction that is
    ///       not needed is better left off.
    bool recording_enabled{false};
    /// Size of the playback stream play_audio() queues into, in bytes (0:
    /// about 67 ms of audio). Larger lets one play_audio() call queue a whole
    /// UI sound.
    size_t playback_buffer_size{0};
    /// Heap capabilities for the playback stream (e.g. MALLOC_CAP_SPIRAM for
    /// a large one)
    uint32_t playback_buffer_caps{MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT};
    /// The playback task's configuration
    espp::Task::BaseConfig playback_task_config{
        .name = "audio out",
        .stack_size_bytes = CONFIG_M5STACK_CARDPUTER_AUDIO_TASK_STACK_SIZE,
        .priority = CONFIG_M5STACK_CARDPUTER_AUDIO_TASK_PRIORITY,
        .core_id = CONFIG_M5STACK_CARDPUTER_AUDIO_TASK_CORE_ID};
    /// The microphone task's configuration (the recording callback runs on
    /// it, so size its stack for what the callback does)
    espp::Task::BaseConfig recording_task_config{
        .name = "audio in",
        .stack_size_bytes = CONFIG_M5STACK_CARDPUTER_MICROPHONE_TASK_STACK_SIZE,
        .priority = CONFIG_M5STACK_CARDPUTER_MICROPHONE_TASK_PRIORITY,
        .core_id = CONFIG_M5STACK_CARDPUTER_MICROPHONE_TASK_CORE_ID};
  };

  /// Initialize the audio system with the default AudioConfig: playback at
  /// 44.1 kHz
  /// \return true if the audio system was successfully initialized, false
  ///         otherwise
  bool initialize_audio();

  /// Initialize the audio system
  /// \param config The sample rate, which directions to set up, the playback
  ///        buffer and the task configurations
  /// \return true if the audio system was successfully initialized, false
  ///         otherwise (also when neither playback nor recording is enabled,
  ///         or, on the original Cardputer, both are)
  /// \note A failure part way through undoes what was set up before it, so
  ///       the call can be retried.
  bool initialize_audio(const AudioConfig &config);

  /// Stop the audio system and free what initialize_audio() set up.
  /// Idempotent.
  /// \note The audio methods below may be called from any task meanwhile,
  ///       but initialize_audio() and deinitialize_audio() must not run
  ///       concurrently with each other, nor from an audio callback.
  void deinitialize_audio();

  /// Whether the audio system is initialized
  /// \return True if initialize_audio() succeeded (until deinitialize_audio())
  bool is_audio_initialized() const {
    auto audio = audio_.load();
    auto microphone = microphone_audio_.load();
    return (audio && audio->is_initialized()) || (microphone && microphone->is_initialized());
  }

  /// The playback audio component, for anything the BSP methods below do not
  /// expose
  /// \return The audio component, or nullptr before initialize_audio() (or
  ///         without playback)
  /// \note On the ADV it is the same object as microphone_audio().
  /// \note deinitialize_audio() releases the hardware even while the returned
  ///       pointer is held: the object is then uninitialized (its methods
  ///       fail / return 0), and a later initialize_audio() makes a new one.
  std::shared_ptr<espp::Audio> audio() const { return audio_.load(); }

  /// The recording audio component, for anything the BSP methods below do
  /// not expose
  /// \return The audio component, or nullptr before initialize_audio() (or
  ///         without recording)
  /// \note On the original Cardputer it is a separate, recording-only
  ///       instance on the microphone's I2S port (PDM); on the ADV the same
  ///       object as audio().
  std::shared_ptr<espp::Audio> microphone_audio() const { return microphone_audio_.load(); }

  /// Set the audio volume
  /// \param volume The volume as a percentage (0 - 100)
  /// \note Neither amplifier has a volume control, so the volume is applied
  ///       in software when the samples are written to the I2S peripheral.
  /// \note Before initialize_audio() (or without playback) the value is
  ///       stored and applied when playback starts.
  void volume(float volume);

  /// Get the audio volume
  /// \return The volume as a percentage (0 - 100)
  float volume() const;

  /// Mute or unmute the audio
  /// \param mute true to mute the audio, false to unmute it
  /// \note Before initialize_audio() (or without playback) the value is
  ///       stored and applied when playback starts.
  void mute(bool mute);

  /// Check if the audio is muted
  /// \return true if the audio is muted, false otherwise
  bool is_muted() const;

  /// Get the audio sample rate
  /// \return The audio sample rate, in Hz (0 before initialize_audio())
  uint32_t audio_sample_rate() const;

  /// Set the audio sample rate (the I2S clocks). The audio queued for
  /// playback is dropped, since it was for the old rate.
  /// \param sample_rate The audio sample rate, in Hz
  /// \return True on success (false before initialize_audio())
  bool audio_sample_rate(uint32_t sample_rate);

  /// Get the audio buffer size
  /// \return The size of one playback block, in bytes (1/60 s of audio)
  size_t audio_buffer_size() const;

  /// Play audio data
  /// \param data The audio data to play (16-bit signed mono samples)
  /// \return The number of bytes actually queued (may be less than the data
  ///         size if the internal stream buffer is full)
  /// \note This function is non-blocking and queues the data for the audio
  ///       task to play; to stream data larger than the internal buffer,
  ///       call it repeatedly, advancing by the returned number of bytes
  ///       (or make the buffer larger: AudioConfig::playback_buffer_size)
  /// \note Must be called from task context, not from an ISR.
  size_t play_audio(std::span<const uint8_t> data);

  /// Play audio data
  /// \param data The audio data to play (16-bit signed mono samples)
  /// \param num_bytes The number of bytes to play
  /// \return The number of bytes actually queued (may be less than \p
  ///         num_bytes if the internal stream buffer is full)
  /// \note This function is non-blocking and queues the data for the audio
  ///       task to play; to stream data larger than the internal buffer,
  ///       call it repeatedly, advancing by the returned number of bytes
  ///       (or make the buffer larger: AudioConfig::playback_buffer_size)
  /// \note Must be called from task context, not from an ISR.
  size_t play_audio(const uint8_t *data, size_t num_bytes);

  /// Play the audio data
  /// \param data The audio data to play (16-bit signed mono samples)
  /// \return The number of bytes actually queued
  [[deprecated("use play_audio(std::span<const uint8_t>) instead")]] size_t
  play_audio(const std::vector<uint8_t> &data);

  /// Drop any queued (not yet played) audio so a subsequent play_audio() starts
  /// immediately instead of waiting behind previously queued sound. Useful for
  /// UI sounds where a new event should restart the sound for maximum
  /// responsiveness.
  void clear_audio();

  /// Start recording audio
  /// \param callback Function to call with recorded audio data: 16-bit
  ///        signed mono samples at audio_sample_rate()
  /// \return True if recording started successfully (false, among other
  ///         reasons, when initialize_audio() did not enable recording)
  /// \note The callback runs in the microphone task's context, so keep it
  ///       quick and non-blocking and size that task's stack accordingly
  ///       (AudioConfig::recording_task_config).
  bool start_audio_recording(std::function<void(const uint8_t *data, size_t length)> callback);

  /// Stop recording audio. Once it returns the callback is not called again
  /// (unless it is called from the callback itself).
  void stop_audio_recording();

  /// Set the microphone volume
  /// \param volume The volume as a percentage (0 - 100); 75 is unity (0 dB,
  ///        the default)
  /// \note On the ADV this adjusts the ES8311 codec's digital ADC volume
  ///       (values above 75 amplify, up to +32 dB at 100); on the original
  ///       the samples are scaled in software in the microphone task (values
  ///       above 75 amplify with saturation).
  /// \note Before initialize_audio() (or without recording) the value is
  ///       stored and applied when recording starts.
  void microphone_volume(float volume);

  /// Get the microphone volume
  /// \return The microphone volume as a percentage (0 - 100); 75 is unity
  ///         (0 dB)
  float microphone_volume() const;

  /// Initialize the sound subsystem (playback)
  /// \param default_audio_rate The sample rate for the audio, in Hz
  /// \param task_config The configuration for the playback task
  /// \return true if the sound subsystem was successfully initialized, false
  ///         otherwise
  /// \note On the original Cardputer this fails while the microphone is set
  ///       up. On the ADV, if recording is already set up, the audio system
  ///       is set up again with playback added, at the current sample rate
  ///       (\p default_audio_rate is then ignored, with a warning).
  [[deprecated("use initialize_audio(const AudioConfig &) instead")]] bool
  initialize_sound(uint32_t default_audio_rate = 44100,
                   const espp::Task::BaseConfig &task_config = {
                       .name = "audio",
                       .stack_size_bytes = CONFIG_M5STACK_CARDPUTER_AUDIO_TASK_STACK_SIZE,
                       .priority = CONFIG_M5STACK_CARDPUTER_AUDIO_TASK_PRIORITY,
                       .core_id = CONFIG_M5STACK_CARDPUTER_AUDIO_TASK_CORE_ID});

  /// Initialize the microphone and start delivering audio data to the
  /// provided callback
  /// \param callback The callback to call with recorded audio data (16-bit
  ///        signed mono samples)
  /// \param sample_rate The sample rate for the microphone, in Hz
  /// \param task_config The configuration for the microphone task
  /// \return true if the microphone was successfully initialized, false
  ///         otherwise
  /// \note On the original Cardputer this fails while the sound subsystem is
  ///       set up. On the ADV, if playback is already set up, the audio
  ///       system is set up again with recording added, at the current sample
  ///       rate (\p sample_rate is then ignored, with a warning) - check
  ///       microphone_sample_rate() for the actual rate. Like
  ///       initialize_audio() and deinitialize_audio() it must not run
  ///       concurrently with them, nor from an audio callback.
  [[deprecated("enable AudioConfig::recording_enabled and use start_audio_recording() "
               "instead")]] bool
  initialize_microphone(const microphone_callback_t &callback, uint32_t sample_rate = 16000,
                        const espp::Task::BaseConfig &task_config = {
                            .name = "microphone",
                            .stack_size_bytes = CONFIG_M5STACK_CARDPUTER_MICROPHONE_TASK_STACK_SIZE,
                            .priority = CONFIG_M5STACK_CARDPUTER_MICROPHONE_TASK_PRIORITY,
                            .core_id = CONFIG_M5STACK_CARDPUTER_MICROPHONE_TASK_CORE_ID});

  /// Get the microphone sample rate
  /// \return The microphone sample rate, in Hz (0 without recording)
  uint32_t microphone_sample_rate() const;

  /////////////////////////////////////////////////////////////////////////////
  // uSD Card
  /////////////////////////////////////////////////////////////////////////////

  /// Configuration for the uSD card
  struct SdCardConfig {
    bool format_if_mount_failed = false;    ///< Format the uSD card if mount failed
    int max_files = 5;                      ///< The maximum number of files to open at once
    size_t allocation_unit_size = 2 * 1024; ///< The allocation unit size in bytes
  };

  /// Initialize the uSD card (SPI mode)
  /// \param config The configuration for the uSD card
  /// \return True if the uSD card was initialized properly.
  bool initialize_sdcard(const SdCardConfig &config);

  /// Get the uSD card
  /// \return A pointer to the uSD card
  /// \note nullptr until initialize_sdcard() succeeded. The card stays valid while
  ///       its volume is unmounted (sdcard_component()->unmount()), e.g. to hand it
  ///       to a USB host through espp::UsbDevice's MSC function.
  sdmmc_card_t *sdcard() const { return sdcard_ ? sdcard_->card() : nullptr; }

  /// Get the SD card component: mount() / unmount() (e.g. to hand the card to a
  /// USB host with the usb_device MSC function), format(), card_info(),
  /// volume_info(), ...
  /// \return The component, or nullptr until initialize_sdcard() succeeded
  espp::SdCard *sdcard_component() const { return sdcard_.get(); }

  /////////////////////////////////////////////////////////////////////////////
  // RGB LED
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the RGB LED (WS2812 on the StampS3 module)
  /// \return true if the LED was successfully initialized, false otherwise
  bool initialize_led();

  /// Set the color of the LED
  /// \param hsv The color of the LED in HSV format
  /// \return true if the color was successfully set, false otherwise
  bool led(const Hsv &hsv);

  /// Set the color of the LED
  /// \param rgb The color of the LED in RGB format
  /// \return true if the color was successfully set, false otherwise
  bool led(const Rgb &rgb);

  /////////////////////////////////////////////////////////////////////////////
  // Battery
  /////////////////////////////////////////////////////////////////////////////

  /// Get the battery voltage
  /// \return The battery voltage, in volts
  /// \note The battery voltage is measured through a 2:1 divider on GPIO 10
  float battery_voltage();

  /// Get the battery state of charge
  /// \return The battery state of charge as a percentage (0 - 100)
  /// \note This is estimated from the battery voltage using a typical 1S
  ///       lithium-ion discharge curve, so it is only an approximation - the
  ///       voltage sags under load (e.g. with the backlight at full
  ///       brightness or the speaker playing) which will lower the estimate.
  float battery_soc();

  /////////////////////////////////////////////////////////////////////////////
  // IMU (Cardputer ADV only)
  /////////////////////////////////////////////////////////////////////////////

  /// Alias for the IMU (BMI270) on the Cardputer ADV's internal I2C bus
  using Imu = espp::Bmi270<espp::bmi270::Interface::I2C>;

  /// Initialize the IMU (BMI270; Cardputer ADV only)
  /// \param orientation_filter Optional filter function for orientation
  ///        (e.g. a kalman or madgwick filter); called by Imu::update()
  /// \param imu_config The IMU configuration
  /// \return true if the IMU was successfully initialized, false otherwise
  /// \note The original Cardputer has no IMU, so this fails (with an error
  ///       log) unless the board is a Cardputer ADV.
  bool initialize_imu(
      const Imu::filter_fn &orientation_filter = nullptr,
      const Imu::ImuConfig &imu_config = {
          .accelerometer_range = Imu::AccelerometerRange::RANGE_4G,
          .accelerometer_odr = Imu::AccelerometerODR::ODR_100_HZ,
          .accelerometer_bandwidth = Imu::AccelerometerBandwidth::NORMAL_AVG4,
          .gyroscope_range = Imu::GyroscopeRange::RANGE_1000DPS,
          .gyroscope_odr = Imu::GyroscopeODR::ODR_100_HZ,
          .gyroscope_bandwidth = Imu::GyroscopeBandwidth::NORMAL_MODE,
          .gyroscope_performance_mode = Imu::GyroscopePerformanceMode::PERFORMANCE_OPTIMIZED});

  /// Get a shared pointer to the IMU
  /// \return A shared pointer to the IMU, or nullptr if it has not been
  ///         (successfully) initialized
  std::shared_ptr<Imu> imu() const { return imu_; }

  /////////////////////////////////////////////////////////////////////////////
  // LoRa + GPS Cap (U201 / U214 expansion module for the Cardputer ADV)
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the LoRa radio (SX1262) on the LoRa+GPS Cap
  /// \param radio_config The radio (modem) configuration to apply
  /// \return True if the radio was initialized properly
  /// \note The Cap mounts on the Cardputer ADV's rear expansion connector;
  ///       the radio shares the uSD card's SPI bus with its own chip select,
  ///       so the radio and uSD card can be used together.
  /// \note On the U214 Cap the radio's RF switch is controlled through a
  ///       PI4IOE5V6408 I2C IO expander, which this method configures to
  ///       connect the antenna. If no expander is found (e.g. the older U201
  ///       Cap), initialization proceeds without it.
  /// \note The radio's DIO1 interrupt is automatically serviced via the
  ///       board's interrupt handler, so packets are delivered through the
  ///       callbacks registered on the returned driver (see
  ///       espp::Sx126x::set_receive_callback and friends).
  bool initialize_lora(const Sx126x::RadioConfig &radio_config = {});

  /// Get the LoRa radio
  /// \return A shared pointer to the LoRa radio driver
  /// \note The radio is only available if initialize_lora() succeeded
  std::shared_ptr<Sx126x> lora() const { return lora_; }

  /// Initialize the GNSS receiver (ATGM336H) on the LoRa+GPS Cap
  /// \param fix_cb Optional callback invoked on each fix update
  /// \param baud_rate The baud rate of the GPS UART. The Cap ships
  ///        configured for 115200 baud.
  /// \return True if the GPS was initialized properly
  bool initialize_gps(const Gps::fix_callback_fn &fix_cb = nullptr, uint32_t baud_rate = 115200);

  /// Get the GPS
  /// \return A shared pointer to the GPS driver
  /// \note The GPS is only available if initialize_gps() succeeded
  std::shared_ptr<Gps> gps() const { return gps_; }

  /// Get the GPIO pin for the LoRa radio chip select
  /// \return The GPIO pin for the LoRa radio chip select
  static constexpr auto lora_cs_gpio() { return lora_cs_io; }

  /// Get the GPIO pin for the LoRa radio DIO1 (interrupt) line
  /// \return The GPIO pin for the LoRa radio DIO1 line
  static constexpr auto lora_dio1_gpio() { return lora_dio1_io; }

  /// Get the GPIO pin for the LoRa radio BUSY line
  /// \return The GPIO pin for the LoRa radio BUSY line
  static constexpr auto lora_busy_gpio() { return lora_busy_io; }

  /// Get the GPIO pin for the LoRa radio reset line
  /// \return The GPIO pin for the LoRa radio reset line
  static constexpr auto lora_reset_gpio() { return lora_reset_io; }

  /// Get the GPIO pin for the GPS UART TX (ESP32 -> GPS)
  /// \return The GPIO pin for the GPS UART TX
  static constexpr auto gps_tx_gpio() { return gps_tx_io; }

  /// Get the GPIO pin for the GPS UART RX (GPS -> ESP32)
  /// \return The GPIO pin for the GPS UART RX
  static constexpr auto gps_rx_gpio() { return gps_rx_io; }

  /////////////////////////////////////////////////////////////////////////////
  // Misc. pins (IR transmitter, Grove port)
  /////////////////////////////////////////////////////////////////////////////

  /// Get the GPIO pin for the IR transmitter
  /// \return The GPIO pin for the IR transmitter
  static constexpr gpio_num_t ir_tx_gpio() { return ir_tx_io; }

  /// Get the GPIO pin for the Grove port SCL / G1 signal
  /// \return The GPIO pin for the Grove port SCL / G1 signal
  static constexpr gpio_num_t grove_scl_gpio() { return grove_scl_io; }

  /// Get the GPIO pin for the Grove port SDA / G2 signal
  /// \return The GPIO pin for the Grove port SDA / G2 signal
  static constexpr gpio_num_t grove_sda_gpio() { return grove_sda_io; }

protected:
  M5StackCardputer();
  void lcd_wait_lines();
  bool keyboard_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);
  void detect_variant();
  bool initialize_keyboard_matrix();
  bool ensure_expansion_spi_bus();
  bool enable_cap_expander_outputs(uint8_t mask);
  bool initialize_keyboard_tca8418();
  void scan_keyboard_matrix();
  void process_tca8418_events();
  void emit_key_event(uint8_t row, uint8_t col, bool pressed, const Modifiers &modifiers);
  bool es8311_write(uint8_t reg, uint8_t value);

  // Entry in the keyboard key map: the normal / shifted characters (0 if the
  // position is a modifier or has no character) and the fn-layer special key.
  struct KeyMapEntry {
    char value;
    char shifted;
    SpecialKey special;
  };

  // LCD (1.14" 240x135 IPS, ST7789V2). The panel is a 240x135 window into the
  // controller's 320x240 (landscape) RAM, centered: x offset (320-240)/2=40,
  // y offset (240-135)/2=52.
  static constexpr size_t lcd_width_ = 240;
  static constexpr size_t lcd_height_ = 135;
  static constexpr size_t lcd_bytes_per_pixel = 2;
  static constexpr size_t frame_buffer_size = (((lcd_width_)*lcd_bytes_per_pixel) * lcd_height_);
  static constexpr int lcd_clock_speed = 40 * 1000 * 1000;
  static constexpr auto lcd_spi_num = SPI2_HOST;
  static constexpr gpio_num_t lcd_cs_io = GPIO_NUM_37;
  static constexpr gpio_num_t lcd_mosi_io = GPIO_NUM_35;
  static constexpr gpio_num_t lcd_sclk_io = GPIO_NUM_36;
  static constexpr gpio_num_t lcd_dc_io = GPIO_NUM_34;
  static constexpr gpio_num_t lcd_reset_io = GPIO_NUM_33;
  static constexpr gpio_num_t backlight_io = GPIO_NUM_38;
  static constexpr int lcd_offset_x = 40;
  static constexpr int lcd_offset_y = 52;
  static constexpr bool backlight_value = true;
  static constexpr bool reset_value = false;
  static constexpr bool invert_colors = true;
  static constexpr bool swap_color_order = false;
  static constexpr auto rotation = espp::DisplayRotation::LANDSCAPE;
  static constexpr bool swap_xy = false;
  static constexpr bool mirror_x = false;
  static constexpr bool mirror_y = false;

  // Keyboard matrix (original Cardputer). The three output pins are the
  // address lines of a 74HC138 3-to-8 demultiplexer whose (active low)
  // outputs select one of 8 scan lines; the 7 input pins (with pullups,
  // active low) each sense one key on the selected line. 8 scan lines x 7
  // inputs = 56 keys = 4 rows x 14 columns.
  static constexpr gpio_num_t keyboard_a0_io = GPIO_NUM_8;
  static constexpr gpio_num_t keyboard_a1_io = GPIO_NUM_9;
  static constexpr gpio_num_t keyboard_a2_io = GPIO_NUM_11;
  static constexpr std::array<gpio_num_t, 7> keyboard_input_ios = {
      GPIO_NUM_13, GPIO_NUM_15, GPIO_NUM_3, GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7};

  // Internal I2C bus (Cardputer ADV only; the original repurposes GPIO 8/9
  // as the 74HC138 address lines above). Hosts the TCA8418 keyboard
  // controller, the ES8311 codec, and a BMI270 IMU.
  static constexpr auto internal_i2c_port = I2C_NUM_0;
  static constexpr int internal_i2c_clock_speed = 400 * 1000;
  static constexpr gpio_num_t internal_i2c_sda = GPIO_NUM_8;
  static constexpr gpio_num_t internal_i2c_scl = GPIO_NUM_9;

  // TCA8418 keyboard controller (Cardputer ADV). Configured as a 7-row x
  // 8-column keypad; key events are read from its FIFO. Key number k (bits
  // 6:0 of an event; bit 7 = pressed) maps to the logical 4x14 grid as:
  //   n = k - 1; col = (n / 10) * 2 + ((n % 10) > 3 ? 1 : 0); row = (n % 10) % 4
  static constexpr uint8_t tca8418_address = 0x34;
  static constexpr gpio_num_t tca8418_int_io = GPIO_NUM_11;
  static constexpr uint8_t TCA8418_REG_CFG = 0x01;
  static constexpr uint8_t TCA8418_REG_INT_STAT = 0x02;
  static constexpr uint8_t TCA8418_REG_KEY_LCK_EC = 0x03;
  static constexpr uint8_t TCA8418_REG_KEY_EVENT_A = 0x04;
  static constexpr uint8_t TCA8418_REG_KP_GPIO_1 = 0x1D;
  static constexpr uint8_t TCA8418_REG_KP_GPIO_2 = 0x1E;
  static constexpr uint8_t TCA8418_REG_KP_GPIO_3 = 0x1F;

  // ES8311 audio codec (Cardputer ADV): DAC to the NS4150B speaker
  // amplifier, ADC from the analog MEMS microphone. Clocked from BCLK (no
  // MCLK pin).
  static constexpr uint8_t es8311_address = 0x18;

  // Positions of the modifier keys in the matrix (row, col)
  static constexpr uint8_t fn_row = 2, fn_col = 0;
  static constexpr uint8_t shift_row = 2, shift_col = 1;
  static constexpr uint8_t ctrl_row = 3, ctrl_col = 0;
  static constexpr uint8_t opt_row = 3, opt_col = 1;
  static constexpr uint8_t alt_row = 3, alt_col = 2;

  // The 4x14 key map. Row 0 is the top (esc / number) row; column 0 is the
  // leftmost key. Backspace, tab, enter, and space are the control characters
  // '\b', '\t', '\n', and ' '; modifiers have no character.
  static constexpr KeyMapEntry key_map_[KEYBOARD_ROWS][KEYBOARD_COLS] = {
      // row 0: ` 1 2 3 4 5 6 7 8 9 0 - = backspace
      {{'`', '~', SpecialKey::ESC},
       {'1', '!', SpecialKey::F1},
       {'2', '@', SpecialKey::F2},
       {'3', '#', SpecialKey::F3},
       {'4', '$', SpecialKey::F4},
       {'5', '%', SpecialKey::F5},
       {'6', '^', SpecialKey::F6},
       {'7', '&', SpecialKey::F7},
       {'8', '*', SpecialKey::F8},
       {'9', '(', SpecialKey::F9},
       {'0', ')', SpecialKey::F10},
       {'-', '_', SpecialKey::F11},
       {'=', '+', SpecialKey::F12},
       {'\b', '\b', SpecialKey::DELETE}},
      // row 1: tab q w e r t y u i o p [ ] backslash
      {{'\t', '\t', SpecialKey::NONE},
       {'q', 'Q', SpecialKey::NONE},
       {'w', 'W', SpecialKey::NONE},
       {'e', 'E', SpecialKey::NONE},
       {'r', 'R', SpecialKey::NONE},
       {'t', 'T', SpecialKey::NONE},
       {'y', 'Y', SpecialKey::NONE},
       {'u', 'U', SpecialKey::NONE},
       {'i', 'I', SpecialKey::NONE},
       {'o', 'O', SpecialKey::NONE},
       {'p', 'P', SpecialKey::NONE},
       {'[', '{', SpecialKey::NONE},
       {']', '}', SpecialKey::NONE},
       {'\\', '|', SpecialKey::NONE}},
      // row 2: fn shift a s d f g h j k l ; ' enter
      {{0, 0, SpecialKey::NONE},
       {0, 0, SpecialKey::NONE},
       {'a', 'A', SpecialKey::NONE},
       {'s', 'S', SpecialKey::NONE},
       {'d', 'D', SpecialKey::NONE},
       {'f', 'F', SpecialKey::NONE},
       {'g', 'G', SpecialKey::NONE},
       {'h', 'H', SpecialKey::NONE},
       {'j', 'J', SpecialKey::NONE},
       {'k', 'K', SpecialKey::NONE},
       {'l', 'L', SpecialKey::NONE},
       {';', ':', SpecialKey::UP},
       {'\'', '"', SpecialKey::NONE},
       {'\n', '\n', SpecialKey::NONE}},
      // row 3: ctrl opt alt z x c v b n m , . / space
      {{0, 0, SpecialKey::NONE},
       {0, 0, SpecialKey::NONE},
       {0, 0, SpecialKey::NONE},
       {'z', 'Z', SpecialKey::NONE},
       {'x', 'X', SpecialKey::NONE},
       {'c', 'C', SpecialKey::NONE},
       {'v', 'V', SpecialKey::NONE},
       {'b', 'B', SpecialKey::NONE},
       {'n', 'N', SpecialKey::NONE},
       {'m', 'M', SpecialKey::NONE},
       {',', '<', SpecialKey::LEFT},
       {'.', '>', SpecialKey::DOWN},
       {'/', '?', SpecialKey::RIGHT},
       {' ', ' ', SpecialKey::NONE}},
  };

  // button (G0 / BOOT)
  static constexpr gpio_num_t button_io = GPIO_NUM_0; // active low

  // Audio out (NS4168 mono I2S amplifier). GPIO 43 (WS) is shared with the
  // PDM microphone clock, so speaker and microphone are mutually exclusive.
  static constexpr auto i2s_port = I2S_NUM_1;
  static constexpr gpio_num_t i2s_bck_io = GPIO_NUM_41;
  static constexpr gpio_num_t i2s_ws_io = GPIO_NUM_43;
  static constexpr gpio_num_t i2s_do_io = GPIO_NUM_42;

  // Microphone. Original: SPM1423 PDM (clk = GPIO 43, shared with the
  // speaker WS). ADV: standard I2S from the ES8311 ADC (ASDOUT = GPIO 46,
  // sharing the speaker's BCK/WS).
  static constexpr auto mic_i2s_port = I2S_NUM_0;
  static constexpr gpio_num_t mic_clk_io = GPIO_NUM_43; // shared with speaker WS
  static constexpr gpio_num_t mic_data_io = GPIO_NUM_46;

  // uSD card (SPI mode, dedicated bus)
  static constexpr auto sdcard_spi_num = SPI3_HOST;
  static constexpr gpio_num_t sdcard_sclk = GPIO_NUM_40;
  static constexpr gpio_num_t sdcard_mosi = GPIO_NUM_14;
  static constexpr gpio_num_t sdcard_miso = GPIO_NUM_39;
  static constexpr gpio_num_t sdcard_cs = GPIO_NUM_12;

  // LoRa + GPS Cap (U201 / U214) on the rear expansion connector. The
  // SX1262 shares the uSD card's SPI bus with its own chip select.
  static constexpr gpio_num_t lora_cs_io = GPIO_NUM_5;
  static constexpr gpio_num_t lora_dio1_io = GPIO_NUM_4;
  static constexpr gpio_num_t lora_busy_io = GPIO_NUM_6;
  static constexpr gpio_num_t lora_reset_io = GPIO_NUM_3;
  static constexpr int lora_spi_clock_speed = 8 * 1000 * 1000;
  static constexpr gpio_num_t gps_tx_io = GPIO_NUM_13; // ESP32 -> GPS
  static constexpr gpio_num_t gps_rx_io = GPIO_NUM_15; // GPS -> ESP32
  static constexpr auto gps_uart_port = UART_NUM_1;
  // PI4IOE5V6408 IO expander on the U214 Cap (internal I2C bus):
  // P0 = LoRa RF switch (high = antenna connected), P1 = GPS LNA enable
  static constexpr uint8_t cap_expander_address = 0x43;
  static constexpr uint8_t cap_expander_lora_rf_switch_mask = 0x01;
  static constexpr uint8_t cap_expander_gps_lna_mask = 0x02;

  // RGB LED (WS2812 on the StampS3 module)
  static constexpr gpio_num_t rgb_led_io = GPIO_NUM_21;

  // IR transmitter
  static constexpr gpio_num_t ir_tx_io = GPIO_NUM_44;

  // Grove port (HY2.0-4P): G1 / G2 (+5V, GND)
  static constexpr gpio_num_t grove_scl_io = GPIO_NUM_1;
  static constexpr gpio_num_t grove_sda_io = GPIO_NUM_2;

  // Battery voltage measurement (2:1 divider into ADC1 on GPIO 10)
  static constexpr float BATTERY_VOLTAGE_SCALE = 2.0f / 1000.0f; // divider ratio, mV -> V

  // Approximate resting discharge curve for a 1S lithium-ion cell, used to
  // estimate the state of charge from the battery voltage. Entries are
  // {voltage (V), state of charge (%)}, in ascending voltage order
  static constexpr std::array<std::pair<float, float>, 10> BATTERY_SOC_CURVE{{
      {3.30f, 0.0f},
      {3.40f, 2.0f},
      {3.50f, 7.0f},
      {3.60f, 17.0f},
      {3.70f, 32.0f},
      {3.80f, 49.0f},
      {3.90f, 66.0f},
      {4.00f, 81.0f},
      {4.10f, 93.0f},
      {4.20f, 100.0f},
  }};
  espp::AdcConfig battery_channel_{.unit = ADC_UNIT_1,
                                   .channel = ADC_CHANNEL_9, // GPIO 10 on ESP32-S3
                                   .attenuation = ADC_ATTEN_DB_12};
  espp::OneshotAdc adc_{{.unit = battery_channel_.unit,
                         .channels = {battery_channel_},
                         .log_level = espp::Logger::Verbosity::WARN}};

  // sdcard
  std::unique_ptr<espp::SdCard> sdcard_;
  // whether the (shared) uSD / LoRa Cap SPI bus has been initialized
  bool expansion_spi_bus_initialized_{false};

  // LoRa + GPS Cap
  spi_device_handle_t lora_spi_handle_{nullptr};
  std::shared_ptr<Sx126x> lora_{nullptr};
  std::shared_ptr<Gps> gps_{nullptr};
  espp::Interrupt::PinConfig lora_dio1_interrupt_pin_{
      .gpio_num = lora_dio1_io,
      .callback =
          [this](const auto &event) {
            if (lora_) {
              std::error_code ec;
              lora_->handle_dio1_interrupt(ec);
            }
          },
      .active_level = espp::Interrupt::ActiveLevel::HIGH,
      .interrupt_type = espp::Interrupt::Type::RISING_EDGE};

  // RGB LED
  std::shared_ptr<Neopixel> rgb_led_{nullptr};

  // IMU (Cardputer ADV only)
  std::shared_ptr<Imu> imu_{nullptr};

  // Interrupts
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

  // we'll only add the interrupt pin if the initialize method is called
  espp::Interrupt interrupts_{
      {.interrupts = {},
       .task_config = {.name = "m5stack-cardputer interrupts",
                       .stack_size_bytes = CONFIG_M5STACK_CARDPUTER_INTERRUPT_STACK_SIZE,
                       .priority = CONFIG_M5STACK_CARDPUTER_INTERRUPT_PRIORITY,
                       .core_id = CONFIG_M5STACK_CARDPUTER_INTERRUPT_CORE_ID}}};

  // button
  std::atomic<bool> button_initialized_{false};
  button_callback_t button_callback_{nullptr};

  // variant / internal I2C. The once_flag makes the lazy detection (see
  // variant() / internal_i2c()) thread-safe.
  std::once_flag variant_detect_once_;
  Variant variant_{Variant::ORIGINAL};
  std::unique_ptr<I2c> internal_i2c_{nullptr};

  // keyboard
  std::atomic<bool> keyboard_initialized_{false};
  keypress_callback_t keypress_callback_{nullptr};
  std::chrono::milliseconds keyboard_poll_interval_{10};
  std::unique_ptr<espp::Task> keyboard_task_{nullptr};
  mutable std::mutex keyboard_state_mutex_;
  // one bit per column (bit set = pressed), one entry per row
  std::array<uint16_t, KEYBOARD_ROWS> keyboard_state_{};

  // microphone volume (percent, 75 = unity / 0 dB)
  std::atomic<float> mic_volume_{75.0f};

  // display
  std::shared_ptr<Display<Pixel>> display_;
  std::shared_ptr<DisplayDriver> display_driver_{static_cast<DisplayDriver *>(nullptr)};
  std::vector<Led::ChannelConfig> backlight_channel_configs_{};
  std::shared_ptr<Led> backlight_{};
  static constexpr int spi_queue_size = 6;
  std::unique_ptr<Spi> lcd_spi_;
  std::unique_ptr<SpiPanelIo> lcd_;

  // audio. Atomic: the forwarders may run on any task while
  // initialize_audio() / deinitialize_audio() publish or unpublish them. On
  // the ADV both hold the same full-duplex instance; on the original at most
  // one is set (playback or the PDM microphone).
  std::atomic<std::shared_ptr<espp::Audio>> audio_{};
  std::atomic<std::shared_ptr<espp::Audio>> microphone_audio_{};
  std::shared_ptr<espp::AudioCodec> adv_codec_{};
  // the last initialize_audio() configuration (for the deprecated
  // initialize_sound() / initialize_microphone())
  AudioConfig audio_config_{};
  std::atomic<bool> mute_{false};
  std::atomic<float> volume_{50.0f};
  // the original's PDM microphone has no gain: start_audio_recording() scales
  // into this buffer (on the microphone task) before calling the callback
  std::vector<int16_t> mic_scaled_;
  // the last start_audio_recording() callback, so the deprecated
  // initialize_sound() can resume a recording after it sets the ADV's audio
  // system up again with playback added
  microphone_callback_t recording_callback_{};
}; // class M5StackCardputer
} // namespace espp

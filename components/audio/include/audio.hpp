#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <system_error>
#include <vector>

#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include <freertos/task.h>

#include "audio_codec.hpp"
#include "base_component.hpp"
#include "i2s_tx_feed.hpp"
#include "task.hpp"

namespace espp {
/// @brief I2S audio playback and recording, shared by the espp BSPs.
///
/// @details One instance drives one I2S port in standard (Philips) mode with
///          16-bit PCM: a TX channel for playback, an RX channel for
///          recording, or both, full duplex on the same clocks. It owns all of
///          the machinery that used to be copied into each BSP:
///          - the I2S channels and their DMA rings, sized so the ring
///            outlasts the playback task's bounded wait (i2s_tx_feed.hpp),
///          - the playback stream buffer and task, which always writes whole
///            blocks so the TX ring never runs dry,
///          - the recording task, which drains the RX ring continuously and
///            hands the samples to a callback while recording,
///          - volume / mute (in the codec, or in software when it has none)
///            and microphone gain,
///          - sample rate changes, and
///          - a complete unwind on failure, deinitialize() and the destructor.
///
///          Chip-specific configuration goes through espp::AudioCodec (see the
///          ES8388 / ES7210 adapters in this component). Board-specific
///          parts - amplifier enable pins, IO expanders, buttons - stay in the
///          BSP; @ref Config::on_playback_block lets a BSP follow playback
///          activity (the M5Stack Tab5 pauses its charger with it).
///
///          A board whose microphones are on a separate I2S port uses two
///          instances: one for playback and one for recording.
///
/// \section audio_ex1 Example
/// \snippet audio_example.cpp audio example
class Audio : public BaseComponent {
public:
  /// Function called with recorded audio: 16-bit signed PCM, interleaved
  /// when HardwareConfig::recording_channels is 2. Runs on the recording task; it may
  /// call any method except start_recording() (which fails from it: it waits
  /// for the callback to return), initialize() and deinitialize().
  using recording_callback_t = std::function<void(const uint8_t *data, size_t length)>;

  /// Function called by the playback task once per block with whether the
  /// block carried audio (false: silence). Keep it short and non-blocking;
  /// it must not call initialize() or deinitialize().
  using playback_block_callback_t = std::function<void(bool active)>;

  /// How many blocks per second the playback task writes: a block is
  /// sample_rate / UPDATE_FREQUENCY frames, rounded down to whole frames (so
  /// at most 1 / UPDATE_FREQUENCY seconds of audio).
  static constexpr uint32_t UPDATE_FREQUENCY = 60;

  /// How the recording (RX) channel receives.
  enum class RecordingFormat {
    STANDARD, ///< Standard (Philips) I2S: two slots per frame. In full duplex
              ///< it runs on the playback clocks.
    TDM,      ///< TDM: HardwareConfig::recording_tdm_slots slots per frame (e.g. an
              ///< ES7210 with four microphones). Recording only; needs
              ///< SOC_I2S_SUPPORTS_TDM.
    PDM,      ///< A PDM microphone: Pins::ws is its clock and Pins::din its
              ///< data; mono. Recording only; needs SOC_I2S_SUPPORTS_PDM_RX.
  };

  /// I2S pins. Unused pins are GPIO_NUM_NC.
  struct Pins {
    gpio_num_t mclk{GPIO_NUM_NC}; ///< Master clock (NC for codecs that derive it from BCLK).
    gpio_num_t bclk{GPIO_NUM_NC}; ///< Bit clock.
    gpio_num_t ws{GPIO_NUM_NC};   ///< Word select (LRCK).
    gpio_num_t dout{GPIO_NUM_NC}; ///< Data out (to the DAC / amplifier), used for playback.
    gpio_num_t din{GPIO_NUM_NC};  ///< Data in (from the ADC / microphones), used for recording.
  };

  /// What an application chooses: the rate, which directions run, buffering
  /// and the tasks. The BSPs take this (as their AudioConfig) and fill in the
  /// HardwareConfig for their board.
  struct AppConfig {
    /// Sample rate, in Hz (at least UPDATE_FREQUENCY).
    uint32_t sample_rate{48000};
    /// The recording rate, in Hz, for an instance that only records (0:
    /// sample_rate). An instance that plays and records runs both on one
    /// clock, so it must be 0 or sample_rate there; a playback-only instance
    /// ignores it. Lets a board whose microphones have their own I2S port
    /// (e.g. the T-Deck) record at a rate other than it plays.
    uint32_t recording_sample_rate{0};
    /// Set up playback (the TX channel, the output codec and the playback
    /// task).
    bool playback_enabled{true};
    /// Set up recording (the RX channel, the input codec and the recording
    /// task).
    /// \note Each I2S channel has a DMA ring in internal DMA-capable RAM
    ///       (about 11.5 KB of stereo frames up to 48 kHz; half that for an
    ///       instance that only records, times the slots / 2 for TDM) and
    ///       each task a stack in internal RAM, so a direction that is not
    ///       needed is better left off.
    bool recording_enabled{false};
    /// Size of the playback stream that play() queues into, in bytes (0: four
    /// blocks, about 67 ms). Larger lets one play() queue a whole UI sound;
    /// at least two blocks.
    size_t playback_buffer_size{0};
    /// Heap capabilities for the playback stream (e.g. MALLOC_CAP_SPIRAM for
    /// a large one).
    uint32_t playback_buffer_caps{MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT};
    /// The playback task's configuration (unpinned by default; a BSP can set
    /// core_id for its board).
    espp::Task::BaseConfig playback_task_config{
        .name = "audio out", .stack_size_bytes = 4096, .priority = 20};
    /// The recording task's configuration (its stack also runs the
    /// recording callback; unpinned by default).
    espp::Task::BaseConfig recording_task_config{
        .name = "audio in", .stack_size_bytes = 4096, .priority = 20};
  };

  /// How the board is wired: the I2S port and pins, the slot layout and the
  /// codecs. A BSP fills this in for its board.
  struct HardwareConfig {
    int port{I2S_NUM_0}; ///< I2S port (I2S_NUM_0, I2S_NUM_1, ...).
    Pins pins{};         ///< I2S pins.
    /// Playback channels: 2 (interleaved stereo) or 1 (mono).
    uint8_t playback_channels{2};
    /// Recording channels: 2 (interleaved stereo) or 1 (mono); other values
    /// are rejected. They are taken from recording_slots (by default mono is
    /// the left slot).
    /// \note Mono is received as stereo slots and the callback gets the left
    ///       slot only: an RX channel in mono slot mode still delivers both
    ///       slots (measured on the ESP32-P4, recording only and full duplex:
    ///       twice the samples, in a DMA ring sized for one). So mono costs a
    ///       stereo RX DMA ring.
    uint8_t recording_channels{2};
    /// How the RX channel receives (STANDARD, TDM or PDM). TDM and PDM are
    /// for an instance that only records (playback_enabled = false), e.g. a
    /// board whose microphones have their own I2S port.
    RecordingFormat recording_format{RecordingFormat::STANDARD};
    /// TDM: slots per frame, 2 - 16.
    uint8_t recording_tdm_slots{4};
    /// The slots the callback receives, in order: the first
    /// recording_channels (1 or 2) of them (STANDARD: of 2, TDM: of
    /// recording_tdm_slots; PDM has one). E.g. {0, 2} keeps a four-slot
    /// ES7210's MIC1 and MIC3 as stereo, and {1} with one channel the right
    /// slot as mono.
    std::array<uint8_t, 2> recording_slots{0, 1};
    /// MCLK as a multiple of the sample rate.
    i2s_mclk_multiple_t mclk_multiple{I2S_MCLK_MULTIPLE_256};
    /// The codec for playback (nullptr: none, e.g. an I2S amplifier; volume
    /// and mute are then applied in software).
    std::shared_ptr<AudioCodec> output_codec{};
    /// The codec for recording (nullptr: none, e.g. an I2S microphone). May
    /// be the same object as output_codec.
    std::shared_ptr<AudioCodec> input_codec{};
  };

  /// Configuration
  struct Config {
    AppConfig app{};                ///< What the application chooses.
    HardwareConfig hardware{};      ///< How the board is wired.
    float volume{50.0f};            ///< Initial output volume, 0 - 100.
    bool muted{false};              ///< Initial mute state.
    float microphone_volume{70.0f}; ///< Initial microphone gain, 0 - 100.
    /// Called by the playback task once per block (optional).
    playback_block_callback_t on_playback_block{};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Log verbosity.
  };

  /// @brief Construct (does not touch the hardware; call initialize()).
  /// @param config The configuration.
  explicit Audio(const Config &config);

  /// @brief Destroy - deinitializes if still initialized.
  ~Audio();

  Audio(const Audio &) = delete;
  Audio &operator=(const Audio &) = delete;

  /// @brief Set up the I2S channels, codecs and tasks. Idempotent.
  /// @param ec Set to the error on failure; cleared on success.
  /// @return true on success. A failure part way through undoes what was
  ///         set up before it, so the call can be retried.
  bool initialize(std::error_code &ec);

  /// @brief Convenience overload that logs on failure.
  /// @return true on success.
  bool initialize();

  /// @brief Stop the tasks and codecs and free the I2S channels and buffers.
  ///        Idempotent.
  void deinitialize();

  /// @return true once initialize() has succeeded (until deinitialize()).
  bool is_initialized() const { return initialized_.load(); }

  /// @return Whether playback is set up (AppConfig::playback_enabled).
  bool is_playback_enabled() const { return config_.app.playback_enabled; }

  /// @return Whether recording is set up (AppConfig::recording_enabled).
  bool is_recording_enabled() const { return config_.app.recording_enabled; }

  /// @brief Queue audio for playback.
  /// @param data 16-bit signed PCM, interleaved when
  ///        HardwareConfig::playback_channels is 2.
  /// @return The number of bytes queued: whole frames only, and less than the
  ///         data when the stream buffer is full. To stream more than the
  ///         buffer holds, call it again from the returned offset.
  /// @note Non-blocking; call it from a task, not an ISR. Returns 0 when
  ///       playback is not initialized.
  size_t play(std::span<const uint8_t> data);

  /// @brief Queue audio for playback.
  /// @param data 16-bit signed PCM, interleaved when
  ///        HardwareConfig::playback_channels is 2.
  /// @param num_bytes The number of bytes.
  /// @return The number of bytes queued (see play(std::span)).
  size_t play(const uint8_t *data, size_t num_bytes);

  /// @brief Drop the audio queued for playback.
  void clear();

  /// @brief Set the output volume.
  /// @param volume The volume, 0 - 100 (clamped).
  /// @note Before initialize() (or without playback) the value is stored and
  ///       applied when playback starts.
  void volume(float volume);

  /// @return The output volume, 0 - 100.
  float volume() const { return volume_.load(); }

  /// @brief Mute or unmute the output.
  /// @param mute true to mute.
  /// @note Before initialize() (or without playback) the value is stored and
  ///       applied when playback starts.
  void mute(bool mute);

  /// @return Whether the output is muted.
  bool is_muted() const { return muted_.load(); }

  /// @brief Change the sample rate: the I2S clocks, the codecs and, since
  ///        the queued audio was for the old rate, the playback stream (it is
  ///        cleared).
  /// @param sample_rate The new sample rate, in Hz (at least UPDATE_FREQUENCY).
  /// @return true on success (false when not initialized, a codec does not
  ///         support the rate - the previous rate is then restored - the rate
  ///         is below
  ///         UPDATE_FREQUENCY, or the playback DMA ring, sized for the rate
  ///         passed to initialize(), is too short for it: initialize() at the
  ///         highest rate you will use).
  /// @note The block size (buffer_size()) stays the one for the rate passed
  ///       to initialize().
  bool sample_rate(uint32_t sample_rate);

  /// @return The sample rate, in Hz.
  uint32_t sample_rate() const { return sample_rate_.load(); }

  /// @return The size of one playback block, in bytes (0 before
  ///         initialize()): sample_rate / UPDATE_FREQUENCY frames,
  ///         rounded down (at most 1 / UPDATE_FREQUENCY seconds).
  size_t buffer_size() const { return block_bytes_; }

  /// @brief Start delivering recorded audio to @p callback.
  /// @param callback Called on the recording task with each buffer of
  ///        samples (one block's worth).
  /// @return false when recording is not initialized, the callback is empty
  ///         or a recording is already in progress (stop it first).
  /// @note Not from the recording callback (it would replace the function
  ///       that is running).
  bool start_recording(const recording_callback_t &callback);

  /// @brief Stop delivering recorded audio. The recording task keeps draining
  ///        the RX ring (which in full duplex keeps playback clean).
  void stop_recording();

  /// @return Whether a recording is in progress.
  bool is_recording() const { return recording_.load(); }

  /// @brief Set the microphone gain.
  /// @param volume The gain, 0 - 100 (clamped), mapped onto the input codec's
  ///        range.
  /// @note Before initialize() (or without recording) the value is stored and
  ///       applied when recording starts.
  void microphone_volume(float volume);

  /// @return The microphone gain, 0 - 100.
  float microphone_volume() const { return microphone_volume_.load(); }

  /// @return The I2S TX channel (nullptr without playback).
  i2s_chan_handle_t tx_handle() const { return tx_handle_; }

  /// @return The I2S RX channel (nullptr without recording).
  i2s_chan_handle_t rx_handle() const { return rx_handle_; }

protected:
  bool playback_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);
  bool recording_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);
  void teardown(std::unique_lock<std::mutex> &lock);
  void apply_software_volume(int16_t *samples, size_t count) const;
  bool software_volume() const;
  bool same_codec() const;
  bool tx_ring_outlasts_wait(uint32_t sample_rate) const;
  bool apply_sample_rate(uint32_t sample_rate);

  Config config_;
  std::atomic<bool> initialized_{false};
  std::atomic<uint32_t> sample_rate_;
  std::atomic<float> volume_;
  std::atomic<bool> muted_;
  std::atomic<float> microphone_volume_;

  // One mutex for everything that changes the setup: initialize(),
  // deinitialize(), sample_rate(), the codec setters and play() / clear()
  // (against the stream's creation and deletion). On this platform each
  // std::mutex allocates its FreeRTOS semaphore (about 116 B of internal
  // RAM), so there is only one. The tasks never take it, and teardown
  // releases it while it joins them, so a callback that calls a method
  // cannot deadlock against teardown.
  std::mutex mutex_;
  // set while teardown has released mutex_ to join the tasks: initialize()
  // and the other teardown entry points back off meanwhile
  bool tearing_down_{false};

  i2s_chan_handle_t tx_handle_{nullptr};
  i2s_chan_handle_t rx_handle_{nullptr};
  bool tx_enabled_{false};
  bool rx_enabled_{false};
  // the clock configuration of the channel that drives the clocks (TX, or RX
  // when recording only), for sample_rate(uint32_t)
  i2s_std_clk_config_t clk_cfg_{};
  // slots per received frame (2 for STANDARD, recording_tdm_slots for TDM, 1
  // for PDM)
  uint8_t rx_slots_{2};
  bool output_started_{false};
  bool input_started_{false};

  size_t block_bytes_{0};
  uint32_t dma_frame_num_{0};
  std::vector<int16_t> tx_buffer_;
  std::vector<int16_t> rx_buffer_;
  StreamBufferHandle_t tx_stream_{nullptr};
  // the playback stream's memory (AppConfig::playback_buffer_caps) and structure
  uint8_t *tx_stream_storage_{nullptr};
  StaticStreamBuffer_t tx_stream_struct_{};
  espp::i2s_tx_feed::Feeder tx_feed_;
  std::unique_ptr<espp::Task> playback_task_;
  std::unique_ptr<espp::Task> recording_task_;

  // the recording callback, published by start_recording() before it sets
  // recording_; in_callback_ (set by the recording task around the call)
  // lets stop_recording() wait for a call in flight without a mutex
  std::atomic<bool> recording_{false};
  std::atomic<bool> in_callback_{false};
  std::atomic<TaskHandle_t> recording_task_handle_{nullptr};
  recording_callback_t recording_callback_;
};
} // namespace espp

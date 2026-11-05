#pragma once

#include <cstdint>

namespace espp {
/// @brief Interface between espp::Audio and an audio codec chip.
///
/// @details espp::Audio owns the I2S channels, the playback and recording
///          tasks and the stream buffer; a codec only has to configure its
///          chip. espp::Audio calls these methods from initialize(),
///          deinitialize() and its setters - never per sample or per block -
///          so the virtual dispatch costs nothing measurable.
///
///          A chip that handles both directions (e.g. an ES8311) is passed as
///          both the output and the input codec of espp::Audio::Config; it is
///          then started once, with Direction::BOTH.
///
///          The I2S clocks (MCLK, BCLK, WS) are running when start() is
///          called, since most codecs need MCLK to accept their register
///          writes.
class AudioCodec {
public:
  /// Which direction(s) a codec is started for.
  enum class Direction : uint8_t {
    PLAYBACK = 1,  ///< DAC: I2S TX to the speaker / headphones
    RECORDING = 2, ///< ADC: microphones to I2S RX
    BOTH = 3,      ///< DAC and ADC
  };

  virtual ~AudioCodec() = default;

  /// @return A short name for logs (e.g. "ES8388").
  virtual const char *name() const = 0;

  /// @brief Configure the chip and start it.
  /// @param direction The direction(s) to start.
  /// @param sample_rate The sample rate the I2S clocks run at, in Hz.
  /// @return true on success. On failure espp::Audio still calls stop(),
  ///         since a chip may be partly configured.
  virtual bool start(Direction direction, uint32_t sample_rate) = 0;

  /// @return The MCLK / sample rate ratio the chip's configuration assumes
  ///         (espp::Audio::Config::mclk_multiple must match), or 0 if it
  ///         follows any.
  virtual uint32_t mclk_multiple() const { return 0; }

  /// @brief Stop the chip (and power down what start() powered up).
  virtual void stop() = 0;

  /// @brief Follow a change of the I2S sample rate (the clocks are already
  ///        running at the new rate).
  /// @param sample_rate The new sample rate, in Hz.
  /// @return true on success.
  virtual bool set_sample_rate(uint32_t sample_rate) { return true; }

  /// @return true if the chip has a hardware output volume and mute; if not,
  ///         espp::Audio scales the samples in software instead.
  virtual bool has_volume() const { return false; }

  /// @brief Set the output volume.
  /// @param percent The volume, 0 - 100.
  /// @return true on success.
  virtual bool set_volume(float percent) { return false; }

  /// @brief Mute or unmute the output.
  /// @param mute true to mute.
  /// @return true on success.
  virtual bool set_mute(bool mute) { return false; }

  /// @return true if the chip has a hardware input (microphone) gain.
  virtual bool has_gain() const { return false; }

  /// @brief Set the input (microphone) gain.
  /// @param percent The gain, 0 - 100, mapped onto the chip's gain range.
  /// @return true on success.
  virtual bool set_gain(float percent) { return false; }
};

/// @return Whether @p direction includes playback.
constexpr bool includes_playback(AudioCodec::Direction direction) {
  return static_cast<uint8_t>(direction) & static_cast<uint8_t>(AudioCodec::Direction::PLAYBACK);
}

/// @return Whether @p direction includes recording.
constexpr bool includes_recording(AudioCodec::Direction direction) {
  return static_cast<uint8_t>(direction) & static_cast<uint8_t>(AudioCodec::Direction::RECORDING);
}
} // namespace espp

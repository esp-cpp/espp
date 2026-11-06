#pragma once

#include "audio_codec.hpp"
#include "es8311.hpp"

namespace espp {
/// @brief espp::AudioCodec for the ES8311 mono codec (DAC and ADC), on the
///        codec component's ES8311 driver.
///
/// @details Configures the ES8311 as an I2S slave for 16-bit Philips data at
///          the I2S sample rate (its clock coefficients follow
///          set_sample_rate() too), drives its hardware volume and mute and
///          maps a 0 - 100 % microphone gain onto its analog gain steps
///          (0 dB - 42 dB). It handles both directions, so pass the same
///          instance as espp::Audio::HardwareConfig::output_codec and input_codec;
///          its ADC drives the left slot (use recording_channels = 1).
///
/// @note The codec component's ES8311 driver keeps its register access
///       functions in globals, so there can be only one ES8311 per firmware.
///       This class sets them when it is constructed.
/// @note It logs through ESP_LOGx (tag "Es8311Codec") rather than an
///       espp::Logger, whose mutex would cost internal RAM for the few
///       messages an init failure prints.
class Es8311Codec : public AudioCodec {
public:
  /// Configuration
  struct Config {
    /// Register write, e.g. espp::make_i2c_addressed_write(device).
    write_fn write;
    /// Register read, e.g. espp::make_i2c_addressed_read_register(device).
    read_register_fn read;
  };

  /// @brief Construct and wire up the driver's register access (no I2C
  ///        traffic yet).
  /// @param config The configuration.
  explicit Es8311Codec(const Config &config);

  /// @brief Clear the driver's register access functions, which hold the
  ///        I2C device (e.g. a shared_ptr captured by
  ///        espp::make_i2c_addressed_write()), unless a newer instance has
  ///        set them since.
  ~Es8311Codec() override;

  const char *name() const override { return "ES8311"; }
  /// @return 256: the driver selects its clock coefficients for MCLK = 256 x
  ///         the rate.
  uint32_t mclk_multiple() const override { return 256; }
  bool start(Direction direction, uint32_t sample_rate) override;
  void stop() override;
  bool set_sample_rate(uint32_t sample_rate) override;
  bool has_volume() const override { return true; }
  bool set_volume(float percent) override;
  bool set_mute(bool mute) override;
  bool has_gain() const override { return true; }
  bool set_gain(float percent) override;

protected:
  // the driver mode start() started, for stop()
  audio_hal_codec_mode_t mode_{AUDIO_HAL_CODEC_MODE_DECODE};
  // the last set_volume(), which set_mute(false) restores
  float volume_{0.0f};
  // the instance whose functions the driver holds
  static Es8311Codec *owner_;
};
} // namespace espp

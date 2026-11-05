#pragma once

#include "audio_codec.hpp"
#include "es8388.hpp"

namespace espp {
/// @brief espp::AudioCodec for the ES8388's DAC (playback), on the codec
///        component's ES8388 driver.
///
/// @details Configures the ES8388 as an I2S slave for 16-bit Philips data,
///          with both DAC outputs enabled, and drives its hardware volume and
///          mute. Its ADC is not used (boards with an ES8388 record through
///          an ES7210, see espp::Es7210Codec), so start() supports
///          Direction::PLAYBACK only.
///
/// @note The codec component's ES8388 driver keeps its register access
///       functions in globals, so there can be only one ES8388 per firmware.
///       This class sets them when it is constructed.
/// @note It logs through ESP_LOGx (tag "Es8388Codec") rather than an
///       espp::Logger, whose mutex would cost internal RAM for the few
///       messages an init failure prints.
class Es8388Codec : public AudioCodec {
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
  explicit Es8388Codec(const Config &config);

  /// @brief Clear the driver's register access functions, which hold the
  ///        I2C device (e.g. a shared_ptr captured by
  ///        espp::make_i2c_addressed_write()), unless a newer instance has
  ///        set them since.
  ~Es8388Codec() override;

  const char *name() const override { return "ES8388"; }
  bool start(Direction direction, uint32_t sample_rate) override;
  void stop() override;
  bool has_volume() const override { return true; }
  bool set_volume(float percent) override;
  bool set_mute(bool mute) override;

protected:
  // the instance whose functions the driver holds
  static Es8388Codec *owner_;
};
} // namespace espp

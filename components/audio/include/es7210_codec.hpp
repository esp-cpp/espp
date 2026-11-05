#pragma once

#include <optional>

#include "audio_codec.hpp"
#include "es7210.hpp"

namespace espp {
/// @brief espp::AudioCodec for the ES7210 four-channel microphone ADC
///        (recording), on the codec component's ES7210 driver.
///
/// @details Configures the ES7210 as an I2S slave for 16-bit Philips data at
///          the I2S sample rate (its coefficient table follows
///          set_sample_rate() too), enables its digital high-pass filter and
///          maps a 0 - 100 % gain onto its analog gain steps (0 dB - 37.5 dB).
///          start() supports Direction::RECORDING only.
///
/// @note The codec component's ES7210 driver keeps its register access
///       functions in globals, so there can be only one ES7210 per firmware.
///       This class sets them when it is constructed.
/// @note It logs through ESP_LOGx (tag "Es7210Codec") rather than an
///       espp::Logger, whose mutex would cost internal RAM for the few
///       messages an init failure prints.
class Es7210Codec : public AudioCodec {
public:
  /// Configuration
  struct Config {
    /// Register write, e.g. espp::make_i2c_addressed_write(device).
    write_fn write;
    /// Register read, e.g. espp::make_i2c_addressed_read_register(device).
    read_register_fn read;
    /// The ES7210's 7-bit I2C address (0x40 - 0x43, set by its AD1 / AD0
    /// pins), passed to `write` for the adapter's own register writes.
    /// \note The codec component's driver passes 0x40 for its writes, so a
    ///       write function that addresses by its argument (rather than a
    ///       device, like espp::make_i2c_addressed_write()) only reaches an
    ///       ES7210 at 0x40.
    uint8_t address{0x40};
    /// The microphones to enable (es7210_mic_select()); unset keeps the
    /// driver's selection.
    std::optional<es7210_input_mics_t> microphones{};
    /// Enable the digital high-pass filter. The microphones' DC offset is
    /// otherwise amplified by the analog gain and rails the ADC: recordings
    /// come out as a near-constant level.
    bool high_pass_filter{true};
  };

  /// @brief Construct and wire up the driver's register access (no I2C
  ///        traffic yet).
  /// @param config The configuration.
  explicit Es7210Codec(const Config &config);

  /// @brief Clear the driver's register access functions, which hold the
  ///        I2C device (e.g. a shared_ptr captured by
  ///        espp::make_i2c_addressed_write()), unless a newer instance has
  ///        set them since.
  ~Es7210Codec() override;

  const char *name() const override { return "ES7210"; }
  bool start(Direction direction, uint32_t sample_rate) override;
  void stop() override;
  bool set_sample_rate(uint32_t sample_rate) override;
  bool has_gain() const override { return true; }
  bool set_gain(float percent) override;

protected:
  // what start() needs after construction (the driver holds its own copy of
  // the register access functions)
  write_fn write_;
  uint8_t address_;
  std::optional<es7210_input_mics_t> microphones_;
  bool high_pass_filter_;
  // the instance whose functions the driver holds
  static Es7210Codec *owner_;
};
} // namespace espp

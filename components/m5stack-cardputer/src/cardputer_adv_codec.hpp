#pragma once

#include <cstdint>
#include <functional>
#include <initializer_list>
#include <utility>

#include "audio_codec.hpp"

namespace espp {
/// espp::AudioCodec for the Cardputer ADV's ES8311, which has no MCLK pin: it
/// is clocked from BCLK, so it does not use the codec component's ES8311
/// driver (which assumes MCLK = 256 x the rate). It writes the minimal DAC /
/// ADC bring-up sequences derived from M5Unified's Cardputer ADV setup.
///
/// The DAC volume stays at 0 dB (the volume is applied in software by
/// espp::Audio, as before); set_gain() drives the ADC's digital volume
/// (register 0x17: 0 % = -95.5 dB, 75 % = 0 dB, 100 % = +32 dB).
class CardputerAdvCodec : public AudioCodec {
public:
  /// Writes one register: (register, value), returns true on success.
  using write_fn = std::function<bool(uint8_t reg, uint8_t value)>;

  /// @brief Construct (no I2C traffic yet).
  /// @param write The register write.
  explicit CardputerAdvCodec(write_fn write)
      : write_(std::move(write)) {}

  const char *name() const override { return "ES8311 (BCLK)"; }
  bool start(Direction direction, uint32_t sample_rate) override;
  void stop() override;
  // the codec's clock follows BCLK: nothing to do on a rate change
  bool set_sample_rate(uint32_t) override { return true; }
  bool has_gain() const override { return true; }
  bool set_gain(float percent) override;

protected:
  bool write_all(std::initializer_list<std::pair<uint8_t, uint8_t>> registers);

  write_fn write_;
  float gain_{75.0f};
  Direction direction_{Direction::PLAYBACK};
};
} // namespace espp

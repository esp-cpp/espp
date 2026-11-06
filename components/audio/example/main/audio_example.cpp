#include <chrono>
#include <cmath>
#include <numbers>
#include <thread>
#include <vector>

#include <driver/gpio.h>

#include "audio.hpp"
#include "logger.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Audio Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example");

#if CONFIG_EXAMPLE_POWER_ENABLE_GPIO >= 0
  const auto power_pin = static_cast<gpio_num_t>(CONFIG_EXAMPLE_POWER_ENABLE_GPIO);
  gpio_set_direction(power_pin, GPIO_MODE_OUTPUT);
  gpio_set_level(power_pin, 1);
#endif

  //! [audio example]
  // Playback only, to an I2S amplifier with no codec (e.g. a MAX98357A):
  // volume and mute are applied in software.
  espp::Audio audio({
      .app = {.sample_rate = 44100, .playback_enabled = true, .recording_enabled = false},
      .hardware = {.pins = {.bclk = static_cast<gpio_num_t>(CONFIG_EXAMPLE_I2S_BCLK_GPIO),
                            .ws = static_cast<gpio_num_t>(CONFIG_EXAMPLE_I2S_WS_GPIO),
                            .dout = static_cast<gpio_num_t>(CONFIG_EXAMPLE_I2S_DOUT_GPIO)}},
      .volume = 30.0f,
      .log_level = espp::Logger::Verbosity::INFO,
  });
  if (!audio.initialize()) {
    logger.error("Could not initialize the audio");
    return;
  }

  // One second of a 440 Hz tone, 16-bit interleaved stereo
  const uint32_t rate = audio.sample_rate();
  std::vector<int16_t> tone(rate * 2);
  for (size_t i = 0; i < rate; i++) {
    const float t = static_cast<float>(i) / rate;
    const auto sample =
        static_cast<int16_t>(std::sin(2.0f * std::numbers::pi_v<float> * 440.0f * t) * 8000.0f);
    tone[2 * i] = sample;
    tone[2 * i + 1] = sample;
  }
  std::span<const uint8_t> bytes(reinterpret_cast<const uint8_t *>(tone.data()),
                                 tone.size() * sizeof(int16_t));

  // play() queues what fits and returns how much that was: stream the tone in
  // from the returned offset
  auto play_all = [&](std::span<const uint8_t> data) {
    while (!data.empty()) {
      const size_t queued = audio.play(data);
      data = data.subspan(queued);
      if (queued == 0) {
        std::this_thread::sleep_for(5ms);
      }
    }
  };

  for (float volume : {30.0f, 60.0f, 90.0f}) {
    logger.info("Playing the tone at {:.0f} % volume", volume);
    audio.volume(volume);
    play_all(bytes);
    std::this_thread::sleep_for(500ms);
  }
  logger.info("Playing the tone muted");
  audio.mute(true);
  play_all(bytes);
  // play_all() returns once the tone is queued (the stream and the DMA ring
  // still hold about 130 ms): let that play out muted before unmuting
  std::this_thread::sleep_for(250ms);
  audio.mute(false);

  // the audio is torn down when `audio` goes out of scope; deinitialize() does
  // it explicitly
  std::this_thread::sleep_for(1s);
  audio.deinitialize();
  //! [audio example]

  logger.info("Example complete");
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

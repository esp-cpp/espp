#include <chrono>
#include <vector>

#include <driver/gpio.h>

#include "color.hpp"
#include "logger.hpp"
#include "rmt.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  // create a logger
  espp::Logger logger({.tag = "rmt example", .level = espp::Logger::Verbosity::INFO});
  {
    // configure the power pin (TinyS3)
    static constexpr int led_power_pin = 17;
    gpio_config_t power_pin_config = {
        .pin_bit_mask = (1ULL << led_power_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&power_pin_config);
    // turn on the power
    gpio_set_level((gpio_num_t)led_power_pin, 1);

    //! [rmt encoder example]
    //
    // The RmtEncoder provides a way to encode data into the RMT peripheral.
    // This code is a custom encoder that encodes WS2812B data. It uses two
    // encoders, a bytes encoder and a copy encoder. The bytes encoder encodes
    // the RGB data into the RMT peripheral and the copy encoder encodes the
    // reset code. The reset code is a special code that is sent after the RGB
    // data to reset the WS2812B LEDs. The reset code is a 50us low pulse
    // followed by a 50us high pulse. The reset code is sent after the RGB data
    // to ensure that the WS2812B LEDs latch the RGB data. The reset code is
    // sent after the RGB data because the WS2812B LEDs latch the RGB data on
    // the rising edge of the reset code.
    //
    // This code is copied from the led_stip example in the esp-idf
    // (https://github.com/espressif/esp-idf/tree/master/examples/peripherals/rmt/led_strip/main)
    int led_encoder_state = 0;
    static constexpr int WS2812_FREQ_HZ = 10000000;
    static constexpr int MICROS_PER_SEC = 1000000;
    auto led_encoder = std::make_unique<espp::RmtEncoder>(espp::RmtEncoder::Config{
        // NOTE: since we're using the 10MHz RMT clock, we can use the pre-defined
        //       ws2812_10mhz_bytes_encoder_config
        .bytes_encoder_config = espp::RmtEncoder::ws2812_10mhz_bytes_encoder_config,
        .encode = [&led_encoder_state](auto channel, auto *copy_encoder, auto *bytes_encoder,
                                       const void *data, size_t data_size,
                                       rmt_encode_state_t *ret_state) -> size_t {
          // divide by 2 since we have both duration0 and duration1 in the reset code
          static uint16_t reset_ticks =
              WS2812_FREQ_HZ / MICROS_PER_SEC * 50 / 2; // reset code duration defaults to 50us
          static rmt_symbol_word_t led_reset_code = (rmt_symbol_word_t){
              .duration0 = reset_ticks,
              .level0 = 0,
              .duration1 = reset_ticks,
              .level1 = 0,
          };
          rmt_encode_state_t session_state = RMT_ENCODING_RESET;
          int state = RMT_ENCODING_RESET;
          size_t encoded_symbols = 0;
          switch (led_encoder_state) {
          case 0: // send RGB data
            encoded_symbols +=
                bytes_encoder->encode(bytes_encoder, channel, data, data_size, &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
              led_encoder_state = 1; // switch to next state when current encoding session finished
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
              state |= RMT_ENCODING_MEM_FULL;
              goto out; // yield if there's no free space for encoding artifacts
            }
            // fall-through
          case 1: // send reset code
            encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_reset_code,
                                                    sizeof(led_reset_code), &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
              led_encoder_state = RMT_ENCODING_RESET; // back to the initial encoding session
              state |= RMT_ENCODING_COMPLETE;
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
              state |= RMT_ENCODING_MEM_FULL;
              goto out; // yield if there's no free space for encoding artifacts
            }
          }
        out:
          *ret_state = static_cast<rmt_encode_state_t>(state);
          return encoded_symbols;
        },
        .del = [](auto *base_encoder) -> esp_err_t {
          // we don't have any extra resources to free, so just return ESP_OK
          return ESP_OK;
        },
        .reset = [&led_encoder_state](auto *base_encoder) -> esp_err_t {
          // all we have is some extra state to reset
          led_encoder_state = 0;
          return ESP_OK;
        },
    });
    //! [rmt encoder example]

    //! [rmt example]
    // create the rmt object
    espp::Rmt rmt(espp::Rmt::Config{
        .gpio_num = 18, // WS2812B data pin on the TinyS3
        .resolution_hz = WS2812_FREQ_HZ,
        .log_level = espp::Logger::Verbosity::INFO,
    });

    // tell the RMT object to use the led_encoder (espp::RmtEncoder) that's
    // defined above
    rmt.set_encoder(std::move(led_encoder));

    // create a task to cycle through rainbow colors and send them to the
    // WS2812B LED using the RMT peripheral
    auto task_fn = [&rmt](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      float t = std::chrono::duration<float>(now - start).count();
      // rotate through rainbow colors in hsv based on time, hue is 0-360
      float hue = (cos(t) * 0.5f + 0.5f) * 360.0f;
      espp::Hsv hsv(hue, 1.0f, 1.0f);
      espp::Rgb rgb = hsv.rgb();
      uint8_t green = std::clamp(int(rgb.g * 255), 0, 255);
      uint8_t blue = std::clamp(int(rgb.b * 255), 0, 255);
      uint8_t red = std::clamp(int(rgb.r * 255), 0, 255);
      // NOTE: we only have one LED so we only need to send one set of RGB data
      uint8_t data[3] = {green, blue, red};
      // now we can send the data to the WS2812B LED
      rmt.transmit(data, sizeof(data));
      fmt::print("hsv->rgb->uint: {} -> {} -> {} {} {}\n", hsv, rgb, green, blue, red);
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 50ms);
      }
      // don't want to stop the task
      return false;
    };

    auto task = espp::Task({.callback = task_fn,
                            .task_config =
                                {
                                    .name = "Rmt Task",
                                    .stack_size_bytes = 5 * 1024,
                                },
                            .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [rmt example]

    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  fmt::print("Rmt example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

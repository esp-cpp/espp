#include <chrono>
#include <vector>

#include <driver/gpio.h>

#include "led_strip.hpp"
#include "logger.hpp"
#include "rmt.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

// The Neopixel BFF has an array of 5x5 SK6805-2427 LEDs which are compatible with
// WS2812 LEDs. The data line is connected to GPIO8.
static constexpr auto NEO_BFF_IO = (GPIO_NUM_8); // QtPy ESP32s3 A3
static constexpr int NEO_BFF_NUM_LEDS = 5 * 5;
static constexpr int SK6805_FREQ_HZ = 10000000; // 10MHz
static constexpr int MICROSECONDS_PER_SECOND = 1000000;
static constexpr int SK6805_RESET_US = 80;

extern "C" void app_main(void) {
  // create a logger
  espp::Logger logger({.tag = "Led Strip example", .level = espp::Logger::Verbosity::INFO});
  {
    int led_encoder_state = 0;
    auto led_encoder =
        std::make_unique<espp::RmtEncoder>(espp::RmtEncoder::Config{
            // NOTE: we're using the 10MHz clock so we could use the pre-defined
            //      SK6805 encoder config but we're using a custom encoder here to
            //      demonstrate how to use the RmtEncoder interface. The values we
            //      use here are based on the SK6805 encoder, which gets its values
            //      from the datasheet for the SK6805
            //      (https://cdn-shop.adafruit.com/product-files/3484/3484_Datasheet.pdf)
            .bytes_encoder_config = {.bit0 =
                                         {
                                             .duration0 = static_cast<uint16_t>(
                                                 SK6805_FREQ_HZ / MICROSECONDS_PER_SECOND * 0.3),
                                             .level0 = 1,
                                             .duration1 = static_cast<uint16_t>(
                                                 SK6805_FREQ_HZ / MICROSECONDS_PER_SECOND * 0.9),
                                             .level1 = 0,
                                         },
                                     .bit1 =
                                         {
                                             .duration0 = static_cast<uint16_t>(
                                                 SK6805_FREQ_HZ / MICROSECONDS_PER_SECOND * 0.6),
                                             .level0 = 1,
                                             .duration1 =
                                                 static_cast<uint16_t>(
                                                     SK6805_FREQ_HZ / MICROSECONDS_PER_SECOND *
                                                     0.6),
                                             .level1 = 0,
                                         },
                                     .flags =
                                         {
                                             .msb_first = 1, // SK6805 transfer bit order: G7 G6 ...
                                                             // G0 R7 R6 ... R0 B7 B6 ... B0
                                         }},
            .encode = [&led_encoder_state](auto channel, auto *copy_encoder, auto *bytes_encoder,
                                           const void *data, size_t data_size,
                                           rmt_encode_state_t *ret_state) -> size_t {
              // divide the transmit resolution (10MHz) by 1,000,000 to get the
              // number of ticks per microsecond
              // we divide by two since we have both duration0 and duration1 in the
              // reset code
              static uint16_t reset_ticks =
                  SK6805_FREQ_HZ / MICROSECONDS_PER_SECOND * SK6805_RESET_US / 2;
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
                  led_encoder_state =
                      1; // switch to next state when current encoding session finished
                }
                if (session_state & RMT_ENCODING_MEM_FULL) {
                  state |= RMT_ENCODING_MEM_FULL;
                  // skip the fallthrough to leave the switch and finish the
                  // function
                  break;
                }
                // intentional fall-through
              case 1: // send reset code
                encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_reset_code,
                                                        sizeof(led_reset_code), &session_state);
                if (session_state & RMT_ENCODING_COMPLETE) {
                  led_encoder_state = RMT_ENCODING_RESET; // back to the initial encoding session
                  state |= RMT_ENCODING_COMPLETE;
                }
                if (session_state & RMT_ENCODING_MEM_FULL) {
                  state |= RMT_ENCODING_MEM_FULL;
                  // skip the fallthrough to leave the switch and finish the
                  // function
                  break;
                }
              }
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

    //! [led strip ex1]
    // create the rmt object
    espp::Rmt rmt(espp::Rmt::Config{
        .gpio_num = NEO_BFF_IO,
        .resolution_hz = SK6805_FREQ_HZ,
        .log_level = espp::Logger::Verbosity::INFO,
    });

    // tell the RMT object to use the led_encoder (espp::RmtEncoder) that's
    // defined above
    rmt.set_encoder(std::move(led_encoder));

    // create the write function we'll use
    auto neopixel_write = [&rmt](const uint8_t *data, size_t len) {
      if (len == 0) {
        return;
      }
      // send the data to the RMT object
      rmt.transmit(data, len);
    };

    // now create the LedStrip object
    espp::LedStrip led_strip(espp::LedStrip::Config{
        .num_leds = NEO_BFF_NUM_LEDS,
        .write = neopixel_write,
        .send_brightness = false,
        .byte_order = espp::LedStrip::ByteOrder::GRB,
        .start_frame = {},
        .end_frame = {},
        .log_level = espp::Logger::Verbosity::INFO,
    });

    // Set all pixels
    led_strip.set_all(espp::Rgb(0, 0, 0));
    // And show it
    led_strip.show();

    std::this_thread::sleep_for(1s);

    // Use a task to rotate the LED through the rainbow using HSV
    auto task_fn = [&led_strip](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      float t = std::chrono::duration<float>(now - start).count();
      // shift the LEDs right one
      led_strip.shift_right();
      // rotate through rainbow colors in hsv based on time, hue is 0-360
      float hue = (cos(t) * 0.5f + 0.5f) * 360.0f;
      espp::Hsv hsv(hue, 1.0f, 1.0f);
      // full brightness (1.0, default) is _really_ bright, so tone it down
      led_strip.set_pixel(0, hsv, 0.05f);
      // show the new colors
      led_strip.show();
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
                                    .name = "LedStrip Task",
                                    .stack_size_bytes = 5 * 1024,
                                },
                            .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [led strip ex1]

    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  logger.info("example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

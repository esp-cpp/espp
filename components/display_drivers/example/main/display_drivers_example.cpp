#include <chrono>
#include <memory>
#include <vector>

#include "hal/spi_types.h"
#include "spi_host_cxx.hpp"

#include "display.hpp"

#if CONFIG_HARDWARE_WROVER_KIT
#include "ili9341.hpp"
#elif CONFIG_HARDWARE_TTGO
#include "st7789.hpp"
#elif CONFIG_HARDWARE_BOX
#include "st7789.hpp"
#else
#error "Misconfigured hardware!"
#endif

// for the example
#include "gui.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  size_t num_seconds_to_run = 10;

  /**
   *   NOTE: for a reference of what display controllers exist on the different
   *   espressif development boards, see:
   *   https://github.com/espressif/esp-bsp/blob/master/LCD.md
   */

#if CONFIG_HARDWARE_WROVER_KIT
  fmt::print("Starting display_drivers example for ESP WROVER KIT\n");
  {
    //! [ili9341 example]
    size_t pixel_buffer_size = 16384;
    // create the spi host for the ILI9331 on ESP_WROVER_DEV_KIT
    idf::SPIMaster master(idf::SPINum(2),
                          idf::MOSI(23),
                          // NOTE: this is actually the reset pin, but we don't
                          // need MISO since it's a screen, so it's OK
                          idf::MISO(18),
                          idf::SCLK(19),
                          idf::SPI_DMAConfig::AUTO(),
                          idf::SPITransferSize(pixel_buffer_size * sizeof(lv_color_t)));
    // create the spi device
    auto lcd = master.create_dev(idf::CS(22), idf::Frequency::MHz(20));
    // create the lcd_write function
    espp::Display::write_fn lcd_write = [&lcd](auto data, auto length, auto user_data) {
      if (length == 0) {
        // oddly the esp-idf-cxx spi driver asserts if we try to send 0 data...
        return;
      }
      // NOTE: we could simply provide user_data as context to the function
      // NOTE: if we don't call get() to block for the transaction, then the
      // transaction will go out scope and fail.
      lcd->transfer(data, data+length, nullptr,
                    [](void* ud) {
                      uint32_t flags = (uint32_t)ud;
                      if (flags & (uint32_t)espp::Display::Signal::FLUSH) {
                        lv_disp_t * disp = _lv_refr_get_disp_refreshing();
                        lv_disp_flush_ready(disp->driver);
                      }
                    },
                    (void*)user_data).get();
    };
    // initialize the controller
    espp::Ili9341::initialize({
        .lcd_write = lcd_write,
        .reset_pin = (gpio_num_t)18,
        .data_command_pin = (gpio_num_t)21,
        .backlight_pin = (gpio_num_t)5,
        .invert_colors = false
      });
    // initialize the display / lvgl
    auto display = std::make_shared<espp::Display>(espp::Display::Config{
        .width = 320,
        .height = 240,
        .pixel_buffer_size = pixel_buffer_size,
        .flush_callback = espp::Ili9341::flush,
        .rotation = espp::Display::Rotation::LANDSCAPE,
        .software_rotation_enabled = true
      });
    // initialize the gui
    Gui gui({
        .display = display
      });
    size_t iterations = 0;
    while (true) {
      auto label = fmt::format("Iterations: {}", iterations);
      gui.set_label(label);
      gui.set_meter(iterations % 100);
      iterations++;
      std::this_thread::sleep_for(100ms);
    }
    //! [ili9341 example]
    // and sleep
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }
#endif

#if CONFIG_HARDWARE_TTGO
  fmt::print("Starting display_drivers example for TTGO T-Display\n");
  {
    //! [st7789 example]
    size_t pixel_buffer_size = 12800;
    // create the spi host for the ST7789 on TTGO T-Display
    idf::SPIMaster master(idf::SPINum(2),
                          idf::MOSI(19),
                          // NOTE: this is actually the reset pin, but we don't
                          // need MISO since it's a screen, so it's OK
                          idf::MISO(23),
                          idf::SCLK(18),
                          idf::SPI_DMAConfig::AUTO(),
                          idf::SPITransferSize(pixel_buffer_size * sizeof(lv_color_t)));
    // create the spi device
    auto lcd = master.create_dev(idf::CS(5), idf::Frequency::MHz(60));
    // create the lcd_write function
    espp::Display::write_fn lcd_write = [&lcd](auto data, auto length, auto user_data) {
      if (length == 0) {
        // oddly the esp-idf-cxx spi driver asserts if we try to send 0 data...
        return;
      }
      // NOTE: we could simply provide user_data as context to the function
      // NOTE: if we don't call get() to block for the transaction, then the
      // transaction will go out scope and fail.
      lcd->transfer(data, data+length, nullptr,
                    [](void* ud) {
                      uint32_t flags = (uint32_t)ud;
                      if (flags & (uint32_t)espp::Display::Signal::FLUSH) {
                        lv_disp_t * disp = _lv_refr_get_disp_refreshing();
                        lv_disp_flush_ready(disp->driver);
                      }
                    },
                    (void*)user_data).get();
    };
    // initialize the controller
    espp::St7789::initialize({
        .lcd_write = lcd_write,
        .reset_pin = (gpio_num_t)23,
        .data_command_pin = (gpio_num_t)16,
        .backlight_pin = (gpio_num_t)4,
        .invert_colors = false,
        .offset_x = 40,
        .offset_y = 53,
      });
    // initialize the display / lvgl
    auto display = std::make_shared<espp::Display>(espp::Display::Config{
        .width = 240,
        .height = 135,
        .pixel_buffer_size = pixel_buffer_size,
        .flush_callback = espp::St7789::flush,
        .rotation = espp::Display::Rotation::PORTRAIT,
        .software_rotation_enabled = true
      });
    // initialize the gui
    Gui gui({
        .display = display
      });
    size_t iterations = 0;
    while (true) {
      auto label = fmt::format("Iterations: {}", iterations);
      gui.set_label(label);
      gui.set_meter(iterations % 100);
      iterations++;
      std::this_thread::sleep_for(100ms);
    }
    //! [st7789 example]
    // and sleep
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }
#endif

#if CONFIG_HARDWARE_BOX
  fmt::print("Starting display_drivers example for ESP32s3 BOX\n");
  // NOTE: see esp-box/components/bsp/src/boards/esp32_s3_box.c
  {
    //! [st7789 esp-box example]
    size_t pixel_buffer_size = 6*1024;
    // create the spi host for the ST7789 on ESP BOX
    idf::SPIMaster master(idf::SPINum(SPI2_HOST),
                          idf::MOSI(6),
                          idf::MISO(18),
                          idf::SCLK(7),
                          idf::SPI_DMAConfig::AUTO(),
                          idf::SPITransferSize(pixel_buffer_size * sizeof(lv_color_t)));
    // create the spi device
    auto lcd = master.create_dev(idf::CS(5), idf::Frequency::MHz(40));
    // create the lcd_write function
    espp::Display::write_fn lcd_write = [&lcd](auto data, auto length, auto user_data) {
      if (length == 0) {
        // oddly the esp-idf-cxx spi driver asserts if we try to send 0 data...
        return;
      }
      // NOTE: we could simply provide user_data as context to the function
      // NOTE: if we don't call get() to block for the transaction, then the
      // transaction will go out scope and fail.
      lcd->transfer(data, data+length, nullptr,
                    [](void* ud) {
                      uint32_t flags = (uint32_t)ud;
                      if (flags & (uint32_t)espp::Display::Signal::FLUSH) {
                        lv_disp_t * disp = _lv_refr_get_disp_refreshing();
                        lv_disp_flush_ready(disp->driver);
                      }
                    },
                    (void*)user_data).get();
    };
    // initialize the controller, for the ESP32s3 box we need to:
    // * mirror x
    // * mirror y
    espp::St7789::initialize({
        .lcd_write = lcd_write,
        .reset_pin = (gpio_num_t)48,
        .data_command_pin = (gpio_num_t)4,
        .backlight_pin = (gpio_num_t)45,
        .backlight_on_value = true,
        .invert_colors = true,
        .mirror_x = true,
        .mirror_y = true,
      });
    // initialize the display / lvgl
    auto display = std::make_shared<espp::Display>(espp::Display::Config{
        .width = 320,
        .height = 240,
        .pixel_buffer_size = pixel_buffer_size,
        .flush_callback = espp::St7789::flush,
        .rotation = espp::Display::Rotation::LANDSCAPE,
        .software_rotation_enabled = true
      });
    // initialize the gui
    Gui gui({
        .display = display
      });
    size_t iterations = 0;
    while (true) {
      auto label = fmt::format("Iterations: {}", iterations);
      gui.set_label(label);
      gui.set_meter(iterations % 100);
      iterations++;
      std::this_thread::sleep_for(100ms);
    }
    /*
    */
    //! [st7789 esp-box example]
    // and sleep
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }
#endif

  fmt::print("Display Driver example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

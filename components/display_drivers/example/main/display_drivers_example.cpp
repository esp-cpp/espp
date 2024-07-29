#include <chrono>
#include <memory>
#include <vector>

#include "driver/spi_master.h"
#include "hal/spi_types.h"

#include "display.hpp"

#if CONFIG_HARDWARE_WROVER_KIT
#include "ili9341.hpp"
static constexpr int DC_PIN_NUM = 21;
using Display = espp::Display<lv_color16_t>;
#elif CONFIG_HARDWARE_TTGO
#include "st7789.hpp"
static constexpr int DC_PIN_NUM = 16;
using Display = espp::Display<lv_color16_t>;
#elif CONFIG_HARDWARE_BOX
#include "st7789.hpp"
static constexpr int DC_PIN_NUM = 4;
using Display = espp::Display<lv_color16_t>;
#elif CONFIG_SMARTKNOB_HA
#include "gc9a01.hpp"
static constexpr int DC_PIN_NUM = 16;
using Display = espp::Display<lv_color16_t>;
#else
#error "Misconfigured hardware!"
#endif

// for the example
#include "gui.hpp"

using namespace std::chrono_literals;

static spi_device_handle_t spi;
static const int spi_queue_size = 7;
static size_t num_queued_trans = 0;

// the user flag for the callbacks does two things:
// 1. Provides the GPIO level for the data/command pin, and
// 2. Sets some bits for other signaling (such as LVGL FLUSH)
static constexpr int FLUSH_BIT = (1 << (int)espp::display_drivers::Flags::FLUSH_BIT);
static constexpr int DC_LEVEL_BIT = (1 << (int)espp::display_drivers::Flags::DC_LEVEL_BIT);

//! [pre_transfer_callback example]
// This function is called (in irq context!) just before a transmission starts.
// It will set the D/C line to the value indicated in the user field
// (DC_LEVEL_BIT).
static void IRAM_ATTR lcd_spi_pre_transfer_callback(spi_transaction_t *t) {
  uint32_t user_flags = (uint32_t)(t->user);
  bool dc_level = user_flags & DC_LEVEL_BIT;
  gpio_set_level((gpio_num_t)DC_PIN_NUM, dc_level);
}
//! [pre_transfer_callback example]

//! [post_transfer_callback example]
// This function is called (in irq context!) just after a transmission ends. It
// will indicate to lvgl that the next flush is ready to be done if the
// FLUSH_BIT is set.
static void IRAM_ATTR lcd_spi_post_transfer_callback(spi_transaction_t *t) {
  uint16_t user_flags = (uint32_t)(t->user);
  bool should_flush = user_flags & FLUSH_BIT;
  if (should_flush) {
    lv_display_t *disp = _lv_refr_get_disp_refreshing();
    lv_display_flush_ready(disp);
  }
}
//! [post_transfer_callback example]

//! [polling_transmit example]
extern "C" void IRAM_ATTR lcd_write(const uint8_t *data, size_t length, uint32_t user_data) {
  if (length == 0) {
    return;
  }
  static spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length = length * 8;
  t.tx_buffer = data;
  t.user = (void *)user_data;
  spi_device_polling_transmit(spi, &t);
}
//! [polling_transmit example]

//! [queued_transmit example]
static void lcd_wait_lines() {
  spi_transaction_t *rtrans;
  esp_err_t ret;
  // Wait for all transactions to be done and get back the results.
  while (num_queued_trans) {
    // fmt::print("Waiting for {} lines\n", num_queued_trans);
    ret = spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
    if (ret != ESP_OK) {
      fmt::print("Could not get trans result: {} '{}'\n", ret, esp_err_to_name(ret));
    }
    num_queued_trans--;
    // We could inspect rtrans now if we received any info back. The LCD is treated as write-only,
    // though.
  }
}

void IRAM_ATTR lcd_send_lines(int xs, int ys, int xe, int ye, const uint8_t *data,
                              uint32_t user_data) {
  // if we haven't waited by now, wait here...
  lcd_wait_lines();
  esp_err_t ret;
  // Transaction descriptors. Declared static so they're not allocated on the stack; we need this
  // memory even when this function is finished because the SPI driver needs access to it even while
  // we're already calculating the next line.
  static spi_transaction_t trans[6];
  // In theory, it's better to initialize trans and data only once and hang on to the initialized
  // variables. We allocate them on the stack, so we need to re-init them each call.
  for (int i = 0; i < 6; i++) {
    memset(&trans[i], 0, sizeof(spi_transaction_t));
    if ((i & 1) == 0) {
      // Even transfers are commands
      trans[i].length = 8;
      trans[i].user = (void *)0;
    } else {
      // Odd transfers are data
      trans[i].length = 8 * 4;
      trans[i].user = (void *)DC_LEVEL_BIT;
    }
    trans[i].flags = SPI_TRANS_USE_TXDATA;
  }
  size_t length = (xe - xs + 1) * (ye - ys + 1) * 2;
#if CONFIG_HARDWARE_WROVER_KIT
  trans[0].tx_data[0] = (uint8_t)espp::Ili9341::Command::caset;
#elif CONFIG_HARDWARE_TTGO || CONFIG_HARDWARE_BOX
  trans[0].tx_data[0] = (uint8_t)espp::St7789::Command::caset;
#elif CONFIG_SMARTKNOB_HA
  trans[0].tx_data[0] = (uint8_t)espp::Gc9a01::Command::caset;
#endif
  trans[1].tx_data[0] = (xs) >> 8;
  trans[1].tx_data[1] = (xs)&0xff;
  trans[1].tx_data[2] = (xe) >> 8;
  trans[1].tx_data[3] = (xe)&0xff;
#if CONFIG_HARDWARE_WROVER_KIT
  trans[2].tx_data[0] = (uint8_t)espp::Ili9341::Command::raset;
#elif CONFIG_HARDWARE_TTGO || CONFIG_HARDWARE_BOX
  trans[2].tx_data[0] = (uint8_t)espp::St7789::Command::raset;
#elif CONFIG_SMARTKNOB_HA
  trans[2].tx_data[0] = (uint8_t)espp::Gc9a01::Command::raset;
#endif
  trans[3].tx_data[0] = (ys) >> 8;
  trans[3].tx_data[1] = (ys)&0xff;
  trans[3].tx_data[2] = (ye) >> 8;
  trans[3].tx_data[3] = (ye)&0xff;
#if CONFIG_HARDWARE_WROVER_KIT
  trans[4].tx_data[0] = (uint8_t)espp::Ili9341::Command::ramwr;
#elif CONFIG_HARDWARE_TTGO || CONFIG_HARDWARE_BOX
  trans[4].tx_data[0] = (uint8_t)espp::St7789::Command::ramwr;
#elif CONFIG_SMARTKNOB_HA
  trans[4].tx_data[0] = (uint8_t)espp::Gc9a01::Command::ramwr;
#endif
  trans[5].tx_buffer = data;
  trans[5].length = length * 8;
  // undo SPI_TRANS_USE_TXDATA flag
  trans[5].flags = 0;
  // we need to keep the dc bit set, but also add our flags
  trans[5].user = (void *)(DC_LEVEL_BIT | user_data);
  // Queue all transactions.
  for (int i = 0; i < 6; i++) {
    ret = spi_device_queue_trans(spi, &trans[i], portMAX_DELAY);
    if (ret != ESP_OK) {
      fmt::print("Couldn't queue trans: {} '{}'\n", ret, esp_err_to_name(ret));
    } else {
      num_queued_trans++;
    }
  }
  // When we are here, the SPI driver is busy (in the background) getting the
  // transactions sent. That happens mostly using DMA, so the CPU doesn't have
  // much to do here. We're not going to wait for the transaction to finish
  // because we may as well spend the time calculating the next line. When that
  // is done, we can call send_line_finish, which will wait for the transfers
  // to be done and check their status.
}
//! [queued_transmit example]

extern "C" void app_main(void) {
  size_t num_seconds_to_run = 10;

  /**
   *   NOTE: for a reference of what display controllers exist on the different
   *   espressif development boards, see:
   *   https://github.com/espressif/esp-bsp/blob/master/LCD.md
   */

#if CONFIG_HARDWARE_WROVER_KIT
  //! [wrover_kit_config example]
  static constexpr std::string_view dev_kit = "ESP-WROVER-DevKit";
  int clock_speed = 20 * 1000 * 1000;
  auto spi_num = SPI2_HOST;
  gpio_num_t mosi = GPIO_NUM_23;
  gpio_num_t sclk = GPIO_NUM_19;
  gpio_num_t spics = GPIO_NUM_22;
  gpio_num_t reset = GPIO_NUM_18;
  gpio_num_t dc_pin = (gpio_num_t)DC_PIN_NUM;
  gpio_num_t backlight = GPIO_NUM_5;
  size_t width = 320;
  size_t height = 240;
  size_t pixel_buffer_size = 16384;
  bool backlight_on_value = false;
  bool reset_value = false;
  bool invert_colors = false;
  int offset_x = 0;
  int offset_y = 0;
  bool mirror_x = false;
  bool mirror_y = false;
  using DisplayDriver = espp::Ili9341;
  auto rotation = espp::DisplayRotation::LANDSCAPE;
  //! [wrover_kit_config example]
#elif CONFIG_HARDWARE_TTGO
  //! [ttgo_config example]
  static constexpr std::string_view dev_kit = "TTGO T-Display";
  int clock_speed = 60 * 1000 * 1000;
  auto spi_num = SPI2_HOST;
  gpio_num_t mosi = GPIO_NUM_19;
  gpio_num_t sclk = GPIO_NUM_18;
  gpio_num_t spics = GPIO_NUM_5;
  gpio_num_t reset = GPIO_NUM_23;
  gpio_num_t dc_pin = (gpio_num_t)DC_PIN_NUM;
  gpio_num_t backlight = GPIO_NUM_4;
  size_t width = 240;
  size_t height = 135;
  size_t pixel_buffer_size = 12800;
  bool backlight_on_value = false;
  bool reset_value = false;
  bool invert_colors = false;
  int offset_x = 40;
  int offset_y = 53;
  bool mirror_x = false;
  bool mirror_y = false;
  using DisplayDriver = espp::St7789;
  auto rotation = espp::DisplayRotation::PORTRAIT;
  //! [ttgo_config example]
#elif CONFIG_HARDWARE_BOX
  //! [box_config example]
  static constexpr std::string_view dev_kit = "ESP32-S3-BOX";
  int clock_speed = 60 * 1000 * 1000;
  auto spi_num = SPI2_HOST;
  gpio_num_t mosi = GPIO_NUM_6;
  gpio_num_t sclk = GPIO_NUM_7;
  gpio_num_t spics = GPIO_NUM_5;
  gpio_num_t reset = GPIO_NUM_48;
  gpio_num_t dc_pin = (gpio_num_t)DC_PIN_NUM;
  gpio_num_t backlight = GPIO_NUM_45;
  size_t width = 320;
  size_t height = 240;
  size_t pixel_buffer_size = width * 50;
  bool backlight_on_value = true;
  bool reset_value = false;
  bool invert_colors = true;
  int offset_x = 0;
  int offset_y = 0;
  bool mirror_x = true;
  bool mirror_y = true;
  using DisplayDriver = espp::St7789;
  auto rotation = espp::DisplayRotation::LANDSCAPE;
  //! [box_config example]
#elif CONFIG_SMARTKNOB_HA
  //! [smartknob_config example]
  static constexpr std::string_view dev_kit = "Smartknob-HA";
  int clock_speed = 80 * 1000 * 1000;
  auto spi_num = SPI2_HOST;
  gpio_num_t mosi = GPIO_NUM_6;
  gpio_num_t sclk = GPIO_NUM_5;
  gpio_num_t spics = GPIO_NUM_15;
  gpio_num_t reset = GPIO_NUM_4;
  gpio_num_t dc_pin = (gpio_num_t)DC_PIN_NUM;
  gpio_num_t backlight = GPIO_NUM_7;
  size_t width = 240;
  size_t height = 240;
  size_t pixel_buffer_size = width * 50;
  bool backlight_on_value = true;
  bool reset_value = false;
  bool invert_colors = true;
  int offset_x = 0;
  int offset_y = 0;
  bool mirror_x = true;
  bool mirror_y = true;
  using DisplayDriver = espp::Gc9a01;
  auto rotation = espp::DisplayRotation::LANDSCAPE;
  //! [smartknob_config example]
#endif

  fmt::print("Starting display_drivers example for {}\n", dev_kit);
  {
    //! [display_drivers example]
    // create the spi host
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.mosi_io_num = mosi;
    buscfg.miso_io_num = -1;
    buscfg.sclk_io_num = sclk;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = (int)(pixel_buffer_size * sizeof(lv_color_t));
    // create the spi device
    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.mode = 0;
    devcfg.clock_speed_hz = clock_speed;
    devcfg.input_delay_ns = 0;
    devcfg.spics_io_num = spics;
    devcfg.queue_size = spi_queue_size;
    devcfg.pre_cb = lcd_spi_pre_transfer_callback;
    devcfg.post_cb = lcd_spi_post_transfer_callback;
    esp_err_t ret;
    // Initialize the SPI bus
    ret = spi_bus_initialize(spi_num, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    // Attach the LCD to the SPI bus
    ret = spi_bus_add_device(spi_num, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    // initialize the controller
    DisplayDriver::initialize(espp::display_drivers::Config{
        .lcd_write = lcd_write,
        .lcd_send_lines = lcd_send_lines,
        .reset_pin = reset,
        .data_command_pin = dc_pin,
        .reset_value = reset_value,
        .invert_colors = invert_colors,
        .offset_x = offset_x,
        .offset_y = offset_y,
        .mirror_x = mirror_x,
        .mirror_y = mirror_y,
    });
    // initialize the display / lvgl
    auto display = std::make_shared<Display>(
        Display::AllocatingConfig{.width = width,
                                  .height = height,
                                  .pixel_buffer_size = pixel_buffer_size,
                                  .flush_callback = DisplayDriver::flush,
                                  .rotation_callback = DisplayDriver::rotate,
                                  .backlight_pin = backlight,
                                  .backlight_on_value = backlight_on_value,
                                  .rotation = rotation});

    // initialize the gui
    Gui gui({});
    size_t iterations = 0;
    while (true) {
      auto label = fmt::format("Iterations: {}", iterations);
      gui.set_label(label);
      gui.set_meter(iterations % 100);
      iterations++;
      std::this_thread::sleep_for(100ms);
    }
    //! [display_drivers example]
    // and sleep
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  fmt::print("Display Driver example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

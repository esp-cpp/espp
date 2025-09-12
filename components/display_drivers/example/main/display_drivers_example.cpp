#include <chrono>
#include <memory>
#include <vector>

#include <driver/spi_master.h>
#include <hal/spi_ll.h>
#include <hal/spi_types.h>

#include "display.hpp"

// default, most displays use 16-bit coordinates
#define DISPLAY_COORDINATES_16BIT 1
#define DISPLAY_COORDINATES_8BIT 0
#define DISPLAY_IS_OLED 0 // most displays are not OLEDs

#if CONFIG_HARDWARE_WROVER_KIT
#include "ili9341.hpp"
static constexpr int DC_PIN_NUM = 21;
using Display = espp::Display<lv_color16_t>;
using DisplayDriver = espp::Ili9341;
#elif CONFIG_HARDWARE_TTGO
#include "st7789.hpp"
static constexpr int DC_PIN_NUM = 16;
using Display = espp::Display<lv_color16_t>;
using DisplayDriver = espp::St7789;
#elif CONFIG_HARDWARE_BOX
#include "st7789.hpp"
static constexpr int DC_PIN_NUM = 4;
using Display = espp::Display<lv_color16_t>;
using DisplayDriver = espp::St7789;
#elif CONFIG_SMARTKNOB_HA
#include "gc9a01.hpp"
static constexpr int DC_PIN_NUM = 16;
using Display = espp::Display<lv_color16_t>;
using DisplayDriver = espp::Gc9a01;
#elif CONFIG_T_ENCODER_PRO
#include "sh8601.hpp"
#define DISPLAY_IS_OLED 1 // T-Encoder Pro uses an OLED display
using Display = espp::Display<lv_color16_t>;
using DisplayDriver = espp::Sh8601;
#elif CONFIG_HARDWARE_BYTE90
#include "ssd1351.hpp"
#define DISPLAY_COORDINATES_8BIT 1 // ssd1351 only supports 8-bit coordinates
#define DISPLAY_COORDINATES_16BIT 0
#define DISPLAY_IS_OLED 1 // Byte90 uses an OLED display
static constexpr int DC_PIN_NUM = 43;
using Display = espp::Display<lv_color16_t>;
using DisplayDriver = espp::Ssd1351;
#else
#error "Misconfigured hardware!"
#endif

// for the example
#include "gui.hpp"

using namespace std::chrono_literals;

static spi_device_handle_t spi;
static const int spi_queue_size = 7;
static size_t num_queued_trans = 0;
static auto spi_num = SPI2_HOST;

// the user flag for the callbacks does two things:
// 1. Provides the GPIO level for the data/command pin, and
// 2. Sets some bits for other signaling (such as LVGL FLUSH)
static constexpr int FLUSH_BIT = (1 << (int)espp::display_drivers::Flags::FLUSH_BIT);
static constexpr int DC_LEVEL_BIT = (1 << (int)espp::display_drivers::Flags::DC_LEVEL_BIT);

//! [pre_transfer_callback example]
// This function is called (in irq context!) just before a transmission starts.
// It will set the D/C line to the value indicated in the user field
// (DC_LEVEL_BIT).
// Except for the T-Encoder Pro, which does not have a D/C line.
#ifndef CONFIG_T_ENCODER_PRO
// cppcheck-suppress constParameterCallback
static void IRAM_ATTR lcd_spi_pre_transfer_callback(spi_transaction_t *t) {
  uint32_t user_flags = (uint32_t)(t->user);
  bool dc_level = user_flags & DC_LEVEL_BIT;
  gpio_set_level((gpio_num_t)DC_PIN_NUM, dc_level);
}
#endif
//! [pre_transfer_callback example]

//! [post_transfer_callback example]
// This function is called (in irq context!) just after a transmission ends. It
// will indicate to lvgl that the next flush is ready to be done if the
// FLUSH_BIT is set.
//
// cppcheck-suppress constParameterCallback
static void IRAM_ATTR lcd_spi_post_transfer_callback(spi_transaction_t *t) {
  uint16_t user_flags = (uint32_t)(t->user);
  bool should_flush = user_flags & FLUSH_BIT;
  if (should_flush) {
    lv_display_t *disp = lv_display_get_default();
    lv_display_flush_ready(disp);
  }
}
//! [post_transfer_callback example]

//! [polling_transmit example]
#ifdef CONFIG_DISPLAY_QUAD_SPI
extern "C" void IRAM_ATTR write_command(uint8_t command, std::span<const uint8_t> parameters,
                                        uint32_t user_data) {
  static spi_transaction_t t = {};

  t.cmd = static_cast<uint8_t>(DisplayDriver::TransferMode::SINGLE_LINE);
  t.addr = static_cast<uint32_t>(command) << 8;
  t.flags = SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
  t.length = parameters.size() * 8;
  t.user = reinterpret_cast<void *>(user_data);

  if (!parameters.empty() && parameters.size() <= 4) {
    memcpy(t.tx_data, parameters.data(), parameters.size());
    t.flags |= SPI_TRANS_USE_TXDATA;
  } else if (!parameters.empty()) {
    t.tx_buffer = parameters.data();
  }

  auto ret = spi_device_acquire_bus(spi, portMAX_DELAY);
  if (ret != ESP_OK) {
    fmt::print("Failed to acquire bus: {}\n", esp_err_to_name(ret));
    return;
  }

  ret = spi_device_polling_transmit(spi, &t);
  if (ret != ESP_OK) {
    fmt::print("Failed to send command: {}\n", esp_err_to_name(ret));
  }
  spi_device_release_bus(spi);
}
#else
extern "C" void IRAM_ATTR write_command(uint8_t command, std::span<const uint8_t> parameters,
                                        uint32_t user_data) {
  static spi_transaction_t t = {};
  t.length = 8;
  t.tx_buffer = &command;
  t.user = reinterpret_cast<void *>(user_data);
  if (!parameters.empty()) {
    spi_device_polling_transmit(spi, &t);
    t.length = parameters.size() * 8;
    t.tx_buffer = parameters.data();
    t.user = reinterpret_cast<void *>(
        user_data | (1 << static_cast<int>(espp::display_drivers::Flags::DC_LEVEL_BIT)));
  }
  spi_device_polling_transmit(spi, &t);
}
#endif
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

#ifdef CONFIG_DISPLAY_QUAD_SPI
void IRAM_ATTR lcd_send_lines(const int xStart, const int yStart, const int xEnd, const int yEnd,
                              const uint8_t *data, const uint32_t user_data) {
  if (data == nullptr) {
    return;
  }

  static bool initialized = false;

  // Transaction descriptors. Declared static so they're not allocated on the stack; we need this
  // memory even when this function is finished because the SPI driver needs access to it even while
  // we're already calculating the next line.
  static std::array<spi_transaction_t, spi_queue_size> transactions = {};

  static size_t max_transfer_size = 0;

  // The first two transactions are for setting the column and page addresses
  constexpr size_t pixel_setup_trans_size = 2;

  // Initialize the above SPI transactions, this only has to be done once
  if (!initialized) {
    transactions[0].cmd = static_cast<uint8_t>(DisplayDriver::TransferMode::SINGLE_LINE);
    transactions[0].addr = static_cast<uint8_t>(DisplayDriver::Command::caset) << 8;

    transactions[0].length = 4 * 8;
    transactions[0].flags =
        SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR | SPI_TRANS_USE_TXDATA;

    transactions[1].cmd = static_cast<uint8_t>(DisplayDriver::TransferMode::SINGLE_LINE);
    transactions[1].addr = static_cast<uint8_t>(DisplayDriver::Command::paset) << 8;
    transactions[1].length = 4 * 8;
    transactions[1].flags =
        SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR | SPI_TRANS_USE_TXDATA;

    transactions[2].cmd = static_cast<uint8_t>(DisplayDriver::TransferMode::MULTI_LINE);
    transactions[2].addr = static_cast<uint8_t>(DisplayDriver::Command::ramwr) << 8;

    spi_bus_get_max_transaction_len(spi_num, &max_transfer_size);
    initialized = true;
  }

  const size_t length = (xEnd - xStart + 1) * (yEnd - yStart + 1) * sizeof(lv_color_t);
  if (length == 0) {
    return;
  }

  // Wait for previous flush transactions to finish
  lcd_wait_lines();

  // Bitwise operations are to split the coordinates into two 8-bit values
  transactions[0].tx_data[0] = (xStart) >> 8;
  transactions[0].tx_data[1] = (xStart)&0xff;
  transactions[0].tx_data[2] = (xEnd) >> 8;
  transactions[0].tx_data[3] = (xEnd)&0xff;

  transactions[1].tx_data[0] = (yStart) >> 8;
  transactions[1].tx_data[1] = (yStart)&0xff;
  transactions[1].tx_data[2] = (yEnd) >> 8;
  transactions[1].tx_data[3] = (yEnd)&0xff;

  size_t remaining = length;
  size_t index =
      pixel_setup_trans_size; // Start at 3 because the first 3 transactions are required for setup
  while (remaining && index < transactions.size()) {
    const size_t transfer_size = std::min(remaining, max_transfer_size);
    // Move the data pointer to the max_transfer_size times the amount of transactions already
    // created
    transactions[index].tx_buffer = data + max_transfer_size * (index - pixel_setup_trans_size);
    transactions[index].length = transfer_size * 8; // Length is in bits
    if (index == pixel_setup_trans_size) {
      transactions[index].flags = SPI_TRANS_MODE_QIO | SPI_TRANS_CS_KEEP_ACTIVE;
    } else {
      // Only the first transaction should transfer the command and address
      transactions[index].flags = SPI_TRANS_MODE_QIO | SPI_TRANS_CS_KEEP_ACTIVE |
                                  SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR |
                                  SPI_TRANS_VARIABLE_DUMMY;
    }

    remaining -= transfer_size;
    index++;
  }

  // Set the flush bit on the last transaction, index - 1 as index is already incremented
  transactions[index - 1].user = reinterpret_cast<void *>(user_data);
  // Have the final pixel transaction stop asserting the CS line
  transactions[index - 1].flags &= ~SPI_TRANS_CS_KEEP_ACTIVE;

  // Acquire the SPI bus, required for the SPI_TRANS_CS_KEEP_ACTIVE flag
  auto ret = spi_device_acquire_bus(spi, portMAX_DELAY);
  if (ret != ESP_OK) {
    fmt::print("Couldn't acquire bus: {}", esp_err_to_name(ret));
    return;
  }

  // Queue all used transactions
  for (int i = 0; i < index; i++) {
    esp_err_t ret = spi_device_queue_trans(spi, &transactions[i], portMAX_DELAY);
    if (ret != ESP_OK) {
      fmt::print("Couldn't queue transaction: {}", esp_err_to_name(ret));
    } else {
      num_queued_trans++;
    }
  }
  spi_device_release_bus(spi);
  // When we are here, the SPI driver is busy (in the background) getting the
  // transactions sent. That happens mostly using DMA, so the CPU doesn't have
  // much to do here. We're not going to wait for the transaction to finish
  // because we may as well spend the time calculating the next line. When that
  // is done, we can call send_line_finish, which will wait for the transfers
  // to be done and check their status.
}
#else
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
#if DISPLAY_COORDINATES_8BIT
      trans[i].length = 8 * 2; // byte90 only has 2 byte per pixel address (1 byte for each axis)
#else // other displays support 16-bit coordinates
      trans[i].length = 8 * 4;
#endif
      trans[i].user = (void *)DC_LEVEL_BIT;
    }
    trans[i].flags = SPI_TRANS_USE_TXDATA;
  }

#ifdef CONFIG_HARDWARE_BYTE90
  lv_display_t *disp = lv_disp_get_default();
  auto rotation = lv_disp_get_rotation(disp);
  if (rotation == lv_display_rotation_t::LV_DISPLAY_ROTATION_90 ||
      rotation == lv_display_rotation_t::LV_DISPLAY_ROTATION_270) {
    // swap x and y coordinates for 90/270 degree rotation
    std::swap(xs, ys);
    std::swap(xe, ye);
  }
#endif

  size_t length = (xe - xs + 1) * (ye - ys + 1) * 2;
  trans[0].tx_data[0] = (uint8_t)DisplayDriver::Command::caset;
#if DISPLAY_COORDINATES_8BIT
  trans[1].tx_data[0] = (xs)&0xff;
  trans[1].tx_data[1] = (xe)&0xff;
#else // other displays support 16-bit coordinates
  trans[1].tx_data[0] = (xs) >> 8;
  trans[1].tx_data[1] = (xs)&0xff;
  trans[1].tx_data[2] = (xe) >> 8;
  trans[1].tx_data[3] = (xe)&0xff;
#endif
  trans[2].tx_data[0] = (uint8_t)DisplayDriver::Command::raset;
#if DISPLAY_COORDINATES_8BIT
  trans[3].tx_data[0] = (ys)&0xff;
  trans[3].tx_data[1] = (ye)&0xff;
#else // other displays support 16-bit coordinates
  trans[3].tx_data[0] = (ys) >> 8;
  trans[3].tx_data[1] = (ys)&0xff;
  trans[3].tx_data[2] = (ye) >> 8;
  trans[3].tx_data[3] = (ye)&0xff;
#endif
  trans[4].tx_data[0] = (uint8_t)DisplayDriver::Command::ramwr;
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
#endif

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
  auto rotation = espp::DisplayRotation::LANDSCAPE;
  //! [wrover_kit_config example]
#elif CONFIG_HARDWARE_TTGO
  //! [ttgo_config example]
  static constexpr std::string_view dev_kit = "TTGO T-Display";
  int clock_speed = 60 * 1000 * 1000;
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
  auto rotation = espp::DisplayRotation::PORTRAIT;
  //! [ttgo_config example]
#elif CONFIG_HARDWARE_BOX
  //! [box_config example]
  static constexpr std::string_view dev_kit = "ESP32-S3-BOX";
  int clock_speed = 60 * 1000 * 1000;
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
  auto rotation = espp::DisplayRotation::LANDSCAPE;
  //! [box_config example]
#elif CONFIG_SMARTKNOB_HA
  //! [smartknob_config example]
  static constexpr std::string_view dev_kit = "Smartknob-HA";
  int clock_speed = 80 * 1000 * 1000;
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
  auto rotation = espp::DisplayRotation::LANDSCAPE;
  //! [smartknob_config example]
#elif CONFIG_T_ENCODER_PRO
  //! [t_encoder_pro_config example]
  static constexpr std::string_view dev_kit = "Lilygo T-Encoder-Pro";
  int clock_speed = 80 * 1000 * 1000;
  constexpr gpio_num_t mosi = GPIO_NUM_11;
  constexpr gpio_num_t miso = GPIO_NUM_13;
  constexpr gpio_num_t data2 = GPIO_NUM_7;
  constexpr gpio_num_t data3 = GPIO_NUM_14;
  constexpr gpio_num_t sclk = GPIO_NUM_12;
  constexpr gpio_num_t spics = GPIO_NUM_10;
  constexpr gpio_num_t reset = GPIO_NUM_4;
  constexpr gpio_num_t enable = GPIO_NUM_3;
  constexpr size_t width = 390;
  constexpr size_t height = 390;
  constexpr size_t pixel_buffer_size = width * height / 10;

  bool reset_value = false;
  bool invert_colors = false;
  int offset_x = 0;
  int offset_y = 0;
  bool mirror_x = false;
  bool mirror_y = false;
  auto rotation = espp::DisplayRotation::LANDSCAPE;
  //! [t_encoder_pro_config example]
#elif CONFIG_HARDWARE_BYTE90
  //! [byte90_config example]
  static constexpr std::string_view dev_kit = "ALXV Labs Byte90";
  int clock_speed = 20 * 1000 * 1000;
  constexpr gpio_num_t mosi = GPIO_NUM_9;
  constexpr gpio_num_t miso = GPIO_NUM_NC;
  constexpr gpio_num_t sclk = GPIO_NUM_7;
  constexpr gpio_num_t spics = GPIO_NUM_44;
  constexpr gpio_num_t reset = GPIO_NUM_1;
  constexpr gpio_num_t dc_pin = (gpio_num_t)DC_PIN_NUM;
  constexpr size_t width = 128;
  constexpr size_t height = 128;
  constexpr size_t pixel_buffer_size = width * height * sizeof(uint16_t);

  bool reset_value = false;
  bool invert_colors = false;
  int offset_x = 0;
  int offset_y = 0;
  bool mirror_x = false;
  bool mirror_y = true;
  auto rotation = espp::DisplayRotation::LANDSCAPE;
  //! [byte90_config example]
#endif

  /// Maximum number of bytes that can be transferred in a single SPI
  /// transaction to the Display. 2MB on ESP32, 1MB on ESP32-S2, 32k on the
  /// ESP32-S3.
  static constexpr size_t SPI_MAX_TRANSFER_BYTES = SPI_LL_DMA_MAX_BIT_LEN / 8;

  fmt::print("Starting display_drivers example for {}\n", dev_kit);
  {
#ifdef CONFIG_T_ENCODER_PRO
    // T-Encoder-Pro display has an enable pin
    constexpr gpio_config_t o_conf{.pin_bit_mask = (1ULL << enable),
                                   .mode = GPIO_MODE_OUTPUT,
                                   .pull_up_en = GPIO_PULLUP_DISABLE,
                                   .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                   .intr_type = GPIO_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&o_conf));

    gpio_set_level(enable, 1);
#endif
    //! [display_drivers example]
    // create the spi host
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.mosi_io_num = mosi;
#ifdef CONFIG_DISPLAY_QUAD_SPI
    buscfg.miso_io_num = miso;
    buscfg.data2_io_num = data2;
    buscfg.data3_io_num = data3;
#else
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
#endif
    buscfg.sclk_io_num = sclk;
    buscfg.max_transfer_sz = SPI_MAX_TRANSFER_BYTES;
    // create the spi device
    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.mode = 0;
    devcfg.clock_speed_hz = clock_speed;
    devcfg.input_delay_ns = 0;
    devcfg.spics_io_num = spics;
    devcfg.queue_size = spi_queue_size;
#ifdef CONFIG_DISPLAY_QUAD_SPI
    devcfg.flags = SPI_DEVICE_HALFDUPLEX;
    devcfg.command_bits = 8;
    devcfg.address_bits = 24;
#endif
#ifndef CONFIG_T_ENCODER_PRO
    devcfg.pre_cb = lcd_spi_pre_transfer_callback;
#endif
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
        .write_command = write_command,
        .lcd_send_lines = lcd_send_lines,
        .reset_pin = reset,
#ifndef CONFIG_T_ENCODER_PRO
        .data_command_pin = dc_pin,
#endif
        .reset_value = reset_value,
        .invert_colors = invert_colors,
        .offset_x = offset_x,
        .offset_y = offset_y,
        .mirror_x = mirror_x,
        .mirror_y = mirror_y,
    });
    // initialize the display / lvgl
    auto display = std::make_shared<Display>(
        Display::LvglConfig{.width = width,
                            .height = height,
                            .flush_callback = DisplayDriver::flush,
                            .rotation_callback = DisplayDriver::rotate,
                            .rotation = rotation},
#if DISPLAY_IS_OLED
        Display::OledConfig{.set_brightness_callback = DisplayDriver::set_brightness,
                            .get_brightness_callback = DisplayDriver::get_brightness},
#else
        Display::LcdConfig{.backlight_pin = backlight, .backlight_on_value = backlight_on_value},
#endif
        Display::DynamicMemoryConfig{.pixel_buffer_size = pixel_buffer_size,
                                     .double_buffered = true});

    display->set_brightness(1.0f);

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

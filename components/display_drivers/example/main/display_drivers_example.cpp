#include <array>
#include <chrono>
#include <cstring>
#include <memory>
#include <vector>

#include <hal/spi_ll.h>

#include "display.hpp"
#include "spi.hpp"

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
#undef DISPLAY_IS_OLED
#define DISPLAY_IS_OLED 1 // T-Encoder Pro uses an OLED display
using Display = espp::Display<lv_color16_t>;
using DisplayDriver = espp::Sh8601;
#elif CONFIG_HARDWARE_BYTE90
#include "ssd1351.hpp"
#undef DISPLAY_IS_OLED
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

static std::unique_ptr<espp::Spi> spi_bus;
static constexpr auto spi_num = SPI2_HOST;
#ifdef CONFIG_DISPLAY_QUAD_SPI
static std::shared_ptr<espp::Spi::Device> spi_device;
static constexpr int spi_queue_size = 7;
static size_t num_queued_trans = 0;
#else
static std::unique_ptr<espp::SpiPanelIo> panel_io;
static constexpr int spi_queue_size = 6;
#endif

// the user flag for the callbacks does two things:
// 1. Provides the GPIO level for the data/command pin, and
// 2. Sets some bits for other signaling (such as LVGL FLUSH)
static constexpr int FLUSH_BIT = (1 << (int)espp::display_drivers::Flags::FLUSH_BIT);
static constexpr int DC_LEVEL_BIT = (1 << (int)espp::display_drivers::Flags::DC_LEVEL_BIT);

static void IRAM_ATTR lcd_spi_flush_ready(uint32_t) {
  lv_display_t *disp = lv_display_get_default();
  lv_display_flush_ready(disp);
}

#ifdef CONFIG_DISPLAY_QUAD_SPI
#ifndef CONFIG_T_ENCODER_PRO
// cppcheck-suppress constParameterCallback
static void IRAM_ATTR lcd_spi_pre_transfer_callback(spi_transaction_t *t) {
  auto user_flags = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(t->user));
  bool dc_level = (user_flags & DC_LEVEL_BIT) != 0;
  gpio_set_level((gpio_num_t)DC_PIN_NUM, dc_level);
}
#endif

// cppcheck-suppress constParameterCallback
static void IRAM_ATTR lcd_spi_post_transfer_callback(spi_transaction_t *t) {
  auto user_flags = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(t->user));
  if ((user_flags & FLUSH_BIT) != 0) {
    lcd_spi_flush_ready(user_flags);
  }
}

extern "C" void IRAM_ATTR write_command(uint8_t command, std::span<const uint8_t> parameters,
                                        uint32_t user_data) {
  static spi_transaction_t t = {};
  if (!spi_device) {
    return;
  }
  std::memset(&t, 0, sizeof(t));

  t.cmd = static_cast<uint8_t>(DisplayDriver::TransferMode::SINGLE_LINE);
  t.addr = static_cast<uint32_t>(command) << 8;
  t.flags = SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
  t.length = parameters.size() * 8;
  t.user = reinterpret_cast<void *>(static_cast<uintptr_t>(user_data));

  if (!parameters.empty() && parameters.size() <= 4) {
    std::memcpy(t.tx_data, parameters.data(), parameters.size());
    t.flags |= SPI_TRANS_USE_TXDATA;
  } else if (!parameters.empty()) {
    t.tx_buffer = parameters.data();
  }

  std::error_code ec;
  auto lock = spi_device->acquire_bus(portMAX_DELAY, ec);
  if (ec || !lock) {
    fmt::print("Failed to acquire bus: {}\n", ec.message());
    return;
  }

  if (!spi_device->polling_transmit(t, ec)) {
    fmt::print("Failed to send command: {}\n", ec.message());
  }
}

static void lcd_wait_lines() {
  if (!spi_device) {
    return;
  }
  spi_transaction_t *rtrans = nullptr;
  while (num_queued_trans) {
    std::error_code ec;
    if (!spi_device->get_transaction_result(&rtrans, portMAX_DELAY, ec)) {
      fmt::print("Could not get trans result: {}\n", ec.message());
      return;
    }
    num_queued_trans--;
  }
}

void IRAM_ATTR lcd_send_lines(const int xStart, const int yStart, const int xEnd, const int yEnd,
                              const uint8_t *data, const uint32_t user_data) {
  if (!spi_device || data == nullptr) {
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
    transactions[index].user = nullptr;

    remaining -= transfer_size;
    index++;
  }

  // Set the flush bit on the last transaction, index - 1 as index is already incremented
  transactions[index - 1].user = reinterpret_cast<void *>(static_cast<uintptr_t>(user_data));
  // Have the final pixel transaction stop asserting the CS line
  transactions[index - 1].flags &= ~SPI_TRANS_CS_KEEP_ACTIVE;

  // Acquire the SPI bus, required for the SPI_TRANS_CS_KEEP_ACTIVE flag
  std::error_code ec;
  auto lock = spi_device->acquire_bus(portMAX_DELAY, ec);
  if (ec || !lock) {
    fmt::print("Couldn't acquire bus: {}\n", ec.message());
    return;
  }
  // Queue all used transactions
  for (int i = 0; i < index; i++) {
    if (!spi_device->queue_transaction(transactions[i], portMAX_DELAY, ec)) {
      fmt::print("Couldn't queue transaction: {}\n", ec.message());
    } else {
      num_queued_trans++;
    }
  }
}
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
    spi_bus = std::make_unique<espp::Spi>(espp::Spi::Config{
        .host = spi_num,
        .sclk_io_num = sclk,
        .mosi_io_num = mosi,
#ifdef CONFIG_DISPLAY_QUAD_SPI
        .miso_io_num = miso,
        .quadwp_io_num = data2,
        .quadhd_io_num = data3,
#else
        .miso_io_num = GPIO_NUM_NC,
#endif
        .max_transfer_sz = SPI_MAX_TRANSFER_BYTES,
    });
    if (!spi_bus || !spi_bus->initialized()) {
      fmt::print("Failed to initialize SPI bus\n");
      return;
    }

#ifdef CONFIG_DISPLAY_QUAD_SPI
    std::error_code ec;
    spi_device = spi_bus->add_device(
        espp::Spi::DeviceConfig{
            .command_bits = 8,
            .address_bits = 24,
            .mode = 0,
            .clock_speed_hz = clock_speed,
            .input_delay_ns = 0,
            .cs_io_num = spics,
            .queue_size = spi_queue_size,
            .flags = SPI_DEVICE_HALFDUPLEX,
#ifndef CONFIG_T_ENCODER_PRO
            .pre_cb = lcd_spi_pre_transfer_callback,
#endif
            .post_cb = lcd_spi_post_transfer_callback,
        },
        ec);
    if (ec || !spi_device) {
      fmt::print("Failed to initialize SPI device: {}\n", ec.message());
      return;
    }
#else
    panel_io = std::make_unique<espp::SpiPanelIo>(espp::SpiPanelIo::Config{
        .spi = spi_bus.get(),
        .device_config =
            {
                .mode = 0,
                .clock_speed_hz = clock_speed,
                .input_delay_ns = 0,
                .cs_io_num = spics,
                .queue_size = spi_queue_size,
#ifdef CONFIG_HARDWARE_BYTE90
                .flags = SPI_DEVICE_NO_DUMMY | SPI_DEVICE_3WIRE,
#endif
            },
        .data_command_io = dc_pin,
        .data_command_bit_mask = DC_LEVEL_BIT,
        .post_transaction_callback_bit_mask = FLUSH_BIT,
        .post_transaction_callback = lcd_spi_flush_ready,
    });
    if (!panel_io || !panel_io->initialized()) {
      fmt::print("Failed to initialize SPI panel I/O\n");
      return;
    }
#endif

    auto display_driver = std::make_shared<DisplayDriver>(espp::display_drivers::Config{
        .panel_io =
#ifdef CONFIG_DISPLAY_QUAD_SPI
            nullptr,
        .write_command = write_command,
#else
            panel_io.get(),
        .write_command = nullptr,
#endif
        .read_command = nullptr,
#ifdef CONFIG_DISPLAY_QUAD_SPI
        .lcd_send_lines = lcd_send_lines,
#else
        .lcd_send_lines = nullptr,
#endif
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
    display_driver->initialize();
    // initialize the display / lvgl
    auto display = std::make_shared<Display>(
        Display::LvglConfig{
            .width = width,
            .height = height,
            .flush_callback =
                [display_driver](lv_display_t *disp, const lv_area_t *area, uint8_t *color_map) {
                  display_driver->flush(disp, area, color_map);
                },
            .rotation_callback =
                [display_driver](const espp::DisplayRotation &rotation) {
                  display_driver->set_rotation(rotation);
                },
            .rotation = rotation},
#if DISPLAY_IS_OLED
        Display::OledConfig{
            .set_brightness_callback =
                [display_driver](float brightness) { display_driver->set_brightness(brightness); },
            .get_brightness_callback =
                [display_driver]() { return display_driver->get_brightness(); }},
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

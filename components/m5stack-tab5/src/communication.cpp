#include "m5stack-tab5.hpp"

#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>

namespace espp {

bool M5StackTab5::initialize_rs485(uint32_t baud_rate, bool enable_termination) {
  logger_.info("Initializing RS-485 interface at {} baud", baud_rate);

  // Configure UART for RS-485
  uart_config_t uart_config = {
      .baud_rate = static_cast<int>(baud_rate),
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 122,
      .source_clk = UART_SCLK_DEFAULT,
  };

  // Install UART driver
  const auto uart_num = UART_NUM_1; // Use UART1 for RS-485
  esp_err_t err = uart_driver_install(uart_num, 1024, 1024, 0, nullptr, 0);
  if (err != ESP_OK) {
    logger_.error("Failed to install UART driver: {}", esp_err_to_name(err));
    return false;
  }

  err = uart_param_config(uart_num, &uart_config);
  if (err != ESP_OK) {
    logger_.error("Failed to configure UART: {}", esp_err_to_name(err));
    return false;
  }

  // Set UART pins
  err = uart_set_pin(uart_num, rs485_tx_io, rs485_rx_io, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (err != ESP_OK) {
    logger_.error("Failed to set UART pins: {}", esp_err_to_name(err));
    return false;
  }

  // Configure direction control pin
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << rs485_dir_io);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);

  // Set to receive mode initially
  gpio_set_level(rs485_dir_io, 0);

  // TODO: Configure 120Ω termination via SIT3088 if enable_termination is true
  if (enable_termination) {
    logger_.info("RS-485 120Ω termination enabled");
  }

  rs485_initialized_ = true;
  logger_.info("RS-485 interface initialized successfully");
  return true;
}

int M5StackTab5::rs485_send(const uint8_t *data, size_t length) {
  if (!rs485_initialized_ || !data || length == 0) {
    return -1;
  }

  const auto uart_num = UART_NUM_1;

  // Set to transmit mode
  gpio_set_level(rs485_dir_io, 1);

  // Small delay to ensure direction switch
  vTaskDelay(pdMS_TO_TICKS(1));

  // Send data
  int sent = uart_write_bytes(uart_num, data, length);

  // Wait for transmission to complete
  uart_wait_tx_done(uart_num, pdMS_TO_TICKS(100));

  // Set back to receive mode
  gpio_set_level(rs485_dir_io, 0);

  logger_.debug("RS-485 sent {} bytes", sent);
  return sent;
}

int M5StackTab5::rs485_receive(uint8_t *buffer, size_t max_length, uint32_t timeout_ms) {
  if (!rs485_initialized_ || !buffer || max_length == 0) {
    return -1;
  }

  const auto uart_num = UART_NUM_1;

  // Ensure we're in receive mode
  gpio_set_level(rs485_dir_io, 0);

  // Read data with timeout
  int received = uart_read_bytes(uart_num, buffer, max_length, pdMS_TO_TICKS(timeout_ms));

  logger_.debug("RS-485 received {} bytes", received);
  return received;
}

bool M5StackTab5::initialize_sd_card() {
  logger_.info("Initializing microSD card");

  // TODO: Implement SD card initialization
  // This can use either SPI mode or SDIO mode depending on requirements
  // SPI mode uses: MISO, CS, SCK, MOSI pins
  // SDIO mode uses: DAT0-3, CLK, CMD pins

  esp_err_t ret;

  // Mount configuration
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
      .format_if_mount_failed = false, .max_files = 5, .allocation_unit_size = 16 * 1024};

  sdmmc_card_t *card;
  const char mount_point[] = "/sdcard";

  logger_.info("Initializing SD card using SDIO peripheral");

  // Configure SDIO host
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.flags = SDMMC_HOST_FLAG_4BIT; // Use 4-bit mode

  // Configure slot
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  slot_config.width = 4; // 4-bit mode
  slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

  // Mount filesystem
  ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      logger_.error("Failed to mount filesystem. If you want the card to be formatted, set "
                    "format_if_mount_failed = true.");
    } else {
      logger_.error("Failed to initialize the card ({}). Make sure SD card lines have pull-up "
                    "resistors in place.",
                    esp_err_to_name(ret));
    }
    return false;
  }

  // Print card info
  sdmmc_card_print_info(stdout, card);

  sd_card_initialized_ = true;
  logger_.info("SD card initialized successfully");
  return true;
}

bool M5StackTab5::is_sd_card_available() const { return sd_card_initialized_; }

bool M5StackTab5::get_sd_card_info(uint32_t *size_mb, uint32_t *free_mb) const {
  if (!sd_card_initialized_) {
    return false;
  }

  // TODO: Get actual SD card size and free space
  // This would involve reading filesystem statistics

  if (size_mb)
    *size_mb = 1024; // Placeholder: 1GB
  if (free_mb)
    *free_mb = 512; // Placeholder: 512MB free

  return true;
}

bool M5StackTab5::initialize_usb_host() {
  logger_.info("Initializing USB host functionality");

  // TODO: Implement USB host initialization
  // This would involve:
  // 1. USB host stack initialization
  // 2. Device enumeration setup
  // 3. Class driver registration (HID, MSC, etc.)
  // 4. Power management for USB-A port

  usb_host_initialized_ = true;
  logger_.info("USB host initialization placeholder completed");
  return true;
}

bool M5StackTab5::initialize_usb_device() {
  logger_.info("Initializing USB device (OTG) functionality");

  // TODO: Implement USB device initialization
  // This would involve:
  // 1. USB device stack initialization
  // 2. Device descriptor configuration
  // 3. Endpoint setup
  // 4. USB-C OTG detection and role switching

  usb_device_initialized_ = true;
  logger_.info("USB device initialization placeholder completed");
  return true;
}

bool M5StackTab5::initialize_wireless() {
  logger_.info("Initializing ESP32-C6 wireless module");

  // TODO: Implement ESP32-C6 communication
  // This would involve:
  // 1. SDIO communication setup with ESP32-C6
  // 2. Firmware loading and initialization
  // 3. Wi-Fi 6 stack initialization
  // 4. Thread/ZigBee protocol stack setup
  // 5. AT command interface or direct API

  wireless_initialized_ = true;
  logger_.info("Wireless module initialization placeholder completed");
  return true;
}

int M5StackTab5::send_wireless_command(const char *command, char *response, size_t max_response_len,
                                       uint32_t timeout_ms) {
  if (!wireless_initialized_ || !command) {
    return -1;
  }

  // TODO: Send command to ESP32-C6 via SDIO interface
  // This would involve:
  // 1. Format command packet
  // 2. Send via SDIO
  // 3. Wait for response with timeout
  // 4. Parse response packet

  logger_.debug("Sending wireless command: {}", command);

  // Placeholder response
  if (response && max_response_len > 0) {
    snprintf(response, max_response_len, "OK");
    return strlen(response);
  }

  return 0;
}

} // namespace espp

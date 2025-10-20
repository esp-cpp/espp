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

/////////////////////////////////////////////////////////////////////////////
// uSD Card
/////////////////////////////////////////////////////////////////////////////

bool M5StackTab5::initialize_sdcard(const M5StackTab5::SdCardConfig &config) {
  if (sdcard_) {
    logger_.error("SD card already initialized!");
    return false;
  }

  logger_.info("Initializing SD card");

  esp_err_t ret;
  // Options for mounting the filesystem. If format_if_mount_failed is set to
  // true, SD card will be partitioned and formatted in case when mounting
  // fails.
  esp_vfs_fat_sdmmc_mount_config_t mount_config;
  memset(&mount_config, 0, sizeof(mount_config));
  mount_config.format_if_mount_failed = config.format_if_mount_failed;
  mount_config.max_files = config.max_files;
  mount_config.allocation_unit_size = config.allocation_unit_size;

  // Use settings defined above to initialize SD card and mount FAT filesystem.
  // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
  // Please check its source code and implement error recovery when developing
  // production applications.
  logger_.debug("Using SDMMC peripheral");

  // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
  // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.max_freq_khz = SDMMC_FREQ_HIGHSPEED; // 40MHz

  // This initializes the slot without card detect (CD) and write protect (WP) signals.
  // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  slot_config.clk = sd_clk_io;
  slot_config.cmd = sd_cmd_io;
  slot_config.d0 = sd_dat0_io;
  slot_config.d1 = sd_dat1_io;
  slot_config.d2 = sd_dat2_io;
  slot_config.d3 = sd_dat3_io;

  logger_.debug("Mounting filesystem");
  ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &sdcard_);

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      logger_.error("Failed to mount filesystem. ");
      return false;
    } else {
      logger_.error("Failed to initialize the card ({}). "
                    "Make sure SD card lines have pull-up resistors in place.",
                    esp_err_to_name(ret));
      return false;
    }
    return false;
  }

  logger_.info("Filesystem mounted");

  // Card has been initialized, print its properties
  sdmmc_card_print_info(stdout, sdcard_);

  sd_card_initialized_ = true;

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

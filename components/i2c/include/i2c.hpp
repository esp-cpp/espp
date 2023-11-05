#pragma once

#include <driver/i2c.h>

#include "logger.hpp"

namespace espp {
/// I2C driver
class I2c {
public:
  /// Configuration for I2C
  struct Config {
    i2c_port_t port = I2C_NUM_0;                       ///< I2C port
    gpio_num_t sda_io_num = GPIO_NUM_NC;               ///< SDA pin
    gpio_num_t scl_io_num = GPIO_NUM_NC;               ///< SCL pin
    gpio_pullup_t sda_pullup_en = GPIO_PULLUP_DISABLE; ///< SDA pullup
    gpio_pullup_t scl_pullup_en = GPIO_PULLUP_DISABLE; ///< SCL pullup
    uint32_t timeout_ms = 10;                          ///< I2C timeout in milliseconds
    uint32_t clk_speed = 400000;                       ///< I2C clock speed in hertz
    bool auto_init = true; ///< Automatically initialize I2C on construction
    espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN; ///< Verbosity of logger
  };

  /// Construct I2C driver
  /// \param config Configuration for I2C
  explicit I2c(const Config &config)
      : config_(config), logger_({.tag = "I2C", .level = config.log_level}) {
    if (config.auto_init) {
      std::error_code ec;
      init(ec);
      if (ec) {
        logger_.error("auto init failed");
      }
    }
  }

  /// Initialize I2C driver
  void init(std::error_code &ec) {
    if (initialized_) {
      logger_.warn("already initialized");
      ec = std::make_error_code(std::errc::protocol_error);
      return;
    }

    i2c_config_t i2c_cfg;
    memset(&i2c_cfg, 0, sizeof(i2c_cfg));
    i2c_cfg.sda_io_num = config_.sda_io_num;
    i2c_cfg.scl_io_num = config_.scl_io_num;
    i2c_cfg.mode = I2C_MODE_MASTER;
    i2c_cfg.sda_pullup_en = config_.sda_pullup_en;
    i2c_cfg.scl_pullup_en = config_.scl_pullup_en;
    i2c_cfg.master.clk_speed = config_.clk_speed;
    auto err = i2c_param_config(config_.port, &i2c_cfg);
    if (err != ESP_OK) {
      logger_.error("config i2c failed {}", esp_err_to_name(err));
      ec = std::make_error_code(std::errc::io_error);
      return;
    }
    err = i2c_driver_install(config_.port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
      logger_.error("install i2c driver failed {}", esp_err_to_name(err));
      ec = std::make_error_code(std::errc::io_error);
      return;
    }

    initialized_ = true;
  }

  /// Deinitialize I2C driver
  void deinit(std::error_code &ec) {
    if (!initialized_) {
      logger_.warn("not initialized");
      ec = std::make_error_code(std::errc::protocol_error);
      return;
    }

    auto err = i2c_driver_delete(config_.port);
    if (err != ESP_OK) {
      logger_.error("delete i2c driver failed");
      ec = std::make_error_code(std::errc::io_error);
      return;
    }

    initialized_ = false;
  }

  /// Write data to I2C device
  /// \param dev_addr I2C device address
  /// \param data Data to write
  /// \param data_len Length of data to write
  /// \return True if successful
  bool write(uint8_t dev_addr, uint8_t *data, size_t data_len) {
    if (!initialized_) {
      logger_.error("not initialized");
      return false;
    }

    auto err = i2c_master_write_to_device(config_.port, dev_addr, data, data_len,
                                          config_.timeout_ms / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
      logger_.error("write {:#04x} error: '{}'", dev_addr, esp_err_to_name(err));
      return false;
    }

    return true;
  }

  /// Write to and read data from I2C device
  /// \param dev_addr I2C device address
  /// \param write_data Data to write
  /// \param write_size Length of data to write
  /// \param read_data Data to read
  /// \param read_size Length of data to read
  /// \return True if successful
  bool write_read(uint8_t dev_addr, uint8_t *write_data, size_t write_size, uint8_t *read_data,
                  size_t read_size) {
    if (!initialized_) {
      logger_.error("not initialized");
      return false;
    }

    auto err =
        i2c_master_write_read_device(config_.port, dev_addr, write_data, write_size, read_data,
                                     read_size, config_.timeout_ms / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
      logger_.error("write read {:#04x} error: '{}'", dev_addr, esp_err_to_name(err));
      return false;
    }

    return true;
  }

  /// Read data from I2C device at register
  /// \param dev_addr I2C device address
  /// \param reg_addr Register address
  /// \param data Data to read
  /// \param data_len Length of data to read
  /// \return True if successful
  bool read_at_register(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t data_len) {
    if (!initialized_) {
      logger_.error("not initialized");
      return false;
    }

    auto err = i2c_master_write_read_device(config_.port, dev_addr, &reg_addr, 1, data, data_len,
                                            config_.timeout_ms / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
      logger_.error("read {:#04x} at register {} error: '{}'", dev_addr, reg_addr,
                    esp_err_to_name(err));
      return false;
    }

    return true;
  }

  /// Read data from I2C device
  /// \param dev_addr I2C device address
  /// \param data Data to read
  /// \param data_len Length of data to read
  /// \return True if successful
  bool read(uint8_t dev_addr, uint8_t *data, size_t data_len) {
    if (!initialized_) {
      logger_.error("not initialized");
      return false;
    }

    auto err = i2c_master_read_from_device(config_.port, dev_addr, data, data_len,
                                           config_.timeout_ms / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
      logger_.error("read {:#04x} error: '{}'", dev_addr, esp_err_to_name(err));
      return false;
    }

    return true;
  }

protected:
  Config config_;
  bool initialized_ = false;

  espp::Logger logger_;
};
} // namespace espp

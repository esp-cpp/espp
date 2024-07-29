#pragma once

#include <mutex>

#include <driver/i2c_master.h>
#include <driver/i2c_types.h>

#include "logger.hpp"

namespace espp {
/// @brief I2C Master driver
/// @details
/// This class is a wrapper around the ESP-IDF I2C Master driver.
///
/// \section Example
/// \snippet i2c_example.cpp i2c master example
class I2cMaster {
public:
  /// Configuration for I2C Master
  struct Config {
    i2c_port_t port = I2C_NUM_0;         ///< I2C port
    gpio_num_t sda_io_num = GPIO_NUM_NC; ///< SDA pin
    gpio_num_t scl_io_num = GPIO_NUM_NC; ///< SCL pin
    bool internal_pullup_en = false;     ///< Internal pullup enable
    uint32_t timeout_ms = 10;            ///< I2C timeout in milliseconds
    uint32_t clk_speed = 400 * 1000;     ///< I2C clock speed in hertz
    bool auto_init = true;               ///< Automatically initialize I2C on construction
    espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN; ///< Verbosity of logger
  };

  /// Construct I2C Master driver
  /// \param config Configuration for I2C Master
  explicit I2cMaster(const Config &config)
      : config_(config)
      , logger_({.tag = "I2C Master", .level = config.log_level}) {
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

    std::lock_guard<std::mutex> lock(mutex_);
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = config_.port,
        .scl_io_num = config_.scl_io_num,
        .sda_io_num = config_.sda_io_num,
        .glitch_ignore_cnt = 7, // TODO: add this to config?
        .flags.enable_internal_pullup = config_.internal_pullup_en,
    };

    auto err = i2c_new_master_bus(&i2c_mst_config, &bus_handle_);
    if (err != ESP_OK) {
      logger_.error("new i2c master bus failed {}", esp_err_to_name(err));
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

    std::lock_guard<std::mutex> lock(mutex_);
    auto err = i2c_del_master_bus(bus_handle_);
    if (err != ESP_OK) {
      logger_.error("delete i2c master bus failed {}", esp_err_to_name(err));
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

    std::lock_guard<std::mutex> lock(mutex_);
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

    std::lock_guard<std::mutex> lock(mutex_);
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

    std::lock_guard<std::mutex> lock(mutex_);
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

    std::lock_guard<std::mutex> lock(mutex_);
    auto err = i2c_master_read_from_device(config_.port, dev_addr, data, data_len,
                                           config_.timeout_ms / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
      logger_.error("read {:#04x} error: '{}'", dev_addr, esp_err_to_name(err));
      return false;
    }

    return true;
  }

  /// Probe I2C device
  /// \param dev_addr I2C device address
  /// \param timeout_ms Timeout in milliseconds
  /// \return True if successful
  /// \details
  /// This function sends a start condition, writes the device address, and then
  /// sends a stop condition. If the device acknowledges the address, then it is
  /// present on the bus.
  bool probe_device(uint8_t dev_addr, int timeout_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto err = i2c_master_probe(bus_handle_, dev_addr, timeout_ms);
    return err == ESP_OK;
  }

protected:
  Config config_;
  bool initialized_ = false;
  i2c_master_bus_handle_t bus_handle_;
  std::mutex mutex_;
  espp::Logger logger_;
};
} // namespace espp

#pragma once

#include <sdkconfig.h>

// Only include this header if the legacy API is selected
#if defined(CONFIG_ESPP_I2C_USE_LEGACY_API) || defined(_DOXYGEN_)

#include <mutex>
#include <vector>

#include <driver/i2c.h>

#include "base_component.hpp"
#include "run_on_core.hpp"

#include "i2c_format_helpers.hpp"

#if CONFIG_ESPP_I2C_LEGACY_API_DISABLE_DEPRECATION_WARNINGS
#define ESPP_I2C_LEGACY_API_DEPRECATED_ATTR
#else
#define ESPP_I2C_LEGACY_API_DEPRECATED_ATTR                                                        \
  [[deprecated(                                                                                    \
      "Use the new I2C API instead or set kconfig "                                                \
      "CONFIG_ESPP_I2C_LEGACY_API_DISABLE_DEPRECATION_WARNINGS to disable this warning")]]
#endif

namespace espp {
/// @brief I2C driver
/// @details
/// This class is a wrapper around the ESP-IDF I2C driver.
///
/// \section i2c_ex1 Example
/// \snippet i2c_example.cpp i2c example
class ESPP_I2C_LEGACY_API_DEPRECATED_ATTR I2c : public espp::BaseComponent {
public:
  /// Configuration for I2C
  struct Config {
    int isr_core_id = -1;        ///< The core to install the I2C interrupt on. If -1, then the I2C
                                 ///  interrupt is installed on the core that this constructor is
                                 ///  called on. If 0 or 1, then the I2C interrupt is installed on
                                 ///  the specified core.
    i2c_port_t port = I2C_NUM_0; ///< I2C port
    gpio_num_t sda_io_num = GPIO_NUM_NC;               ///< SDA pin
    gpio_num_t scl_io_num = GPIO_NUM_NC;               ///< SCL pin
    gpio_pullup_t sda_pullup_en = GPIO_PULLUP_DISABLE; ///< SDA pullup
    gpio_pullup_t scl_pullup_en = GPIO_PULLUP_DISABLE; ///< SCL pullup
    uint32_t timeout_ms = 10;                          ///< I2C timeout in milliseconds
    uint32_t clk_speed = 400 * 1000;                   ///< I2C clock speed in hertz
    bool auto_init = true; ///< Automatically initialize I2C on construction
    espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN; ///< Verbosity of logger
  };

  /// Construct I2C driver
  /// \param config Configuration for I2C
  explicit I2c(const Config &config)
      : BaseComponent("I2C", config.log_level)
      , config_(config) {
    if (config.auto_init) {
      std::error_code ec;
      init(ec);
      if (ec) {
        logger_.error("auto init failed");
      }
    }
  }

  /// Destructor
  ~I2c() {
    std::error_code ec;
    deinit(ec);
    if (ec) {
      logger_.error("deinit failed");
    }
  }

  /// Initialize I2C driver
  void init(std::error_code &ec) {
    if (initialized_) {
      logger_.warn("already initialized");
      ec = std::make_error_code(std::errc::protocol_error);
      return;
    }

    logger_.debug("Initializing I2C with config: {}", config_);

    std::unique_lock lock(mutex_);
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
    // Make copy of variables for clearer code / easier capture
    auto i2c_port = config_.port;
    auto install_fn = [i2c_port]() -> esp_err_t {
      return i2c_driver_install(i2c_port, I2C_MODE_MASTER, 0, 0, 0);
    };
    err = espp::task::run_on_core(install_fn, config_.isr_core_id);
    if (err != ESP_OK) {
      logger_.error("install i2c driver failed {}", esp_err_to_name(err));
      ec = std::make_error_code(std::errc::io_error);
      return;
    }

    logger_.info("I2C initialized on port {}", config_.port);

    initialized_ = true;
  }

  /// Deinitialize I2C driver
  void deinit(std::error_code &ec) {
    if (!initialized_) {
      logger_.warn("not initialized");
      // dont make this an error
      ec.clear();
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    auto err = i2c_driver_delete(config_.port);
    if (err != ESP_OK) {
      logger_.error("delete i2c driver failed");
      ec = std::make_error_code(std::errc::io_error);
      return;
    }

    ec.clear();
    logger_.info("I2C deinitialized on port {}", config_.port);
    initialized_ = false;
  }

  /// Write data to I2C device
  /// \param dev_addr I2C device address
  /// \param data Data to write
  /// \param data_len Length of data to write
  /// \return True if successful
  bool write(const uint8_t dev_addr, const uint8_t *data, const size_t data_len) {
    if (!initialized_) {
      logger_.error("not initialized");
      return false;
    }

    logger_.debug("write {} bytes to address {:#02x}", data_len, dev_addr);
    std::lock_guard<std::mutex> lock(mutex_);
    auto err = i2c_master_write_to_device(config_.port, dev_addr, data, data_len,
                                          config_.timeout_ms / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
      logger_.error("write {:#04x} error: '{}'", dev_addr, esp_err_to_name(err));
      return false;
    }

    return true;
  }

  /// Write data to I2C device
  /// \param dev_addr I2C device address
  /// \param data Data to write
  /// \return True if successful
  bool write_vector(const uint8_t dev_addr, const std::vector<uint8_t> &data) {
    return write(dev_addr, data.data(), data.size());
  }

  /// Write to and read data from I2C device
  /// \param dev_addr I2C device address
  /// \param write_data Data to write
  /// \param write_size Length of data to write
  /// \param read_data Data to read
  /// \param read_size Length of data to read
  /// \return True if successful
  bool write_read(const uint8_t dev_addr, const uint8_t *write_data, const size_t write_size,
                  uint8_t *read_data, size_t read_size) {
    if (!initialized_) {
      logger_.error("not initialized");
      return false;
    }

    logger_.debug("write {} bytes and read {} bytes from address {:#02x}", write_size, read_size,
                  dev_addr);
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

  /// Write data to and read data from I2C device
  /// \param dev_addr I2C device address
  /// \param write_data Data to write
  /// \param read_data Data to read
  /// \return True if successful
  bool write_read_vector(const uint8_t dev_addr, const std::vector<uint8_t> &write_data,
                         std::vector<uint8_t> &read_data) {
    return write_read(dev_addr, write_data.data(), write_data.size(), read_data.data(),
                      read_data.size());
  }

  /// Read data from I2C device at register
  /// \param dev_addr I2C device address
  /// \param reg_addr Register address
  /// \param data Data to read
  /// \param data_len Length of data to read
  /// \return True if successful
  bool read_at_register(const uint8_t dev_addr, const uint8_t reg_addr, uint8_t *data,
                        size_t data_len) {
    if (!initialized_) {
      logger_.error("not initialized");
      return false;
    }

    logger_.debug("read {} bytes from address {:#02x} at register {}", data_len, dev_addr,
                  reg_addr);
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

  /// Read data from I2C device at register
  /// \param dev_addr I2C device address
  /// \param reg_addr Register address
  /// \param data Data to read
  /// \return True if successful
  bool read_at_register_vector(const uint8_t dev_addr, const uint8_t reg_addr,
                               std::vector<uint8_t> &data) {
    return read_at_register(dev_addr, reg_addr, data.data(), data.size());
  }

  /// Read data from I2C device
  /// \param dev_addr I2C device address
  /// \param data Data to read
  /// \param data_len Length of data to read
  /// \return True if successful
  bool read(const uint8_t dev_addr, uint8_t *data, size_t data_len) {
    if (!initialized_) {
      logger_.error("not initialized");
      return false;
    }

    logger_.debug("read {} bytes from address {:#02x}", data_len, dev_addr);
    std::lock_guard<std::mutex> lock(mutex_);
    auto err = i2c_master_read_from_device(config_.port, dev_addr, data, data_len,
                                           config_.timeout_ms / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
      logger_.error("read {:#04x} error: '{}'", dev_addr, esp_err_to_name(err));
      return false;
    }

    return true;
  }

  /// Read data from I2C device
  /// \param dev_addr I2C device address
  /// \param data Data to read
  /// \return True if successful
  bool read_vector(const uint8_t dev_addr, std::vector<uint8_t> &data) {
    return read(dev_addr, data.data(), data.size());
  }

  /// Probe I2C device
  /// \param dev_addr I2C device address
  /// \return True if successful
  /// \details
  /// This function sends a start condition, writes the device address, and then
  /// sends a stop condition. If the device acknowledges the address, then it is
  /// present on the bus.
  bool probe_device(const uint8_t dev_addr) {
    bool success = false;
    logger_.debug("probe device {:#02x}", dev_addr);
    std::lock_guard<std::mutex> lock(mutex_);
    auto cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    if (i2c_master_cmd_begin(config_.port, cmd, config_.timeout_ms / portTICK_PERIOD_MS) ==
        ESP_OK) {
      success = true;
    }
    i2c_cmd_link_delete(cmd);
    return success;
  }

protected:
  Config config_;
  bool initialized_ = false;
  std::mutex mutex_;
};
} // namespace espp

// for printing the I2C::Config using fmt
template <> struct fmt::formatter<espp::I2c::Config> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const espp::I2c::Config &c, FormatContext &ctx) const {
    // print the clock speed in khz
    auto clk_speed_khz = c.clk_speed / 1000;
    // if it's MHz, print it as such
    if (clk_speed_khz >= 1000) {
      clk_speed_khz = c.clk_speed / 1000000;
      return fmt::format_to(
          ctx.out(),
          "I2c::Config{{I2C port: {}, SDA: {}, SCL: {}, SDA pullup: {}, SCL pullup: {}, "
          "timeout: {}ms, clock speed: {}MHz}}",
          c.port, c.sda_io_num, c.scl_io_num, c.sda_pullup_en, c.scl_pullup_en, c.timeout_ms,
          clk_speed_khz);
    }
    return fmt::format_to(
        ctx.out(),
        "I2c::Config{{I2C port: {}, SDA: {}, SCL: {}, SDA pullup: {}, SCL pullup: {}, "
        "timeout: {}ms, clock speed: {}kHz}}",
        c.port, c.sda_io_num, c.scl_io_num, c.sda_pullup_en, c.scl_pullup_en, c.timeout_ms,
        clk_speed_khz);
  }
};

#else
#error                                                                                             \
    "i2c.hpp included but CONFIG_ESPP_I2C_USE_LEGACY_API is not set. Please select the correct I2C API in KConfig."
#endif

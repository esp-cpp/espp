#pragma once

#include <sdkconfig.h>

#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <system_error>
#include <vector>

#include "base_component.hpp"
#include "i2c_format_helpers.hpp"
#include "run_on_core.hpp"

#if defined(CONFIG_ESPP_I2C_USE_LEGACY_API)

#include <driver/i2c.h>

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
  template <typename RegisterType = uint8_t> class Device : public espp::BaseComponent {
  public:
    struct Config {
      uint16_t device_address = 0;
      int timeout_ms = 10;
      int addr_bit_len = 7;
      uint32_t scl_speed_hz = 400000;
      bool auto_init = true;
      Logger::Verbosity log_level = Logger::Verbosity::WARN;
    };

    explicit Device(I2c &bus, const Config &config)
        : BaseComponent("I2C Device", config.log_level)
        , bus_(&bus)
        , config_(config) {
      if (config.auto_init) {
        std::error_code ec;
        if (!init(ec)) {
          logger_.error("auto init failed");
        }
      }
    }

    ~Device() {
      std::error_code ec;
      deinit(ec);
      if (ec) {
        logger_.error("deinit failed");
      }
    }

    bool init(std::error_code &ec) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (initialized_) {
        ec = std::make_error_code(std::errc::already_connected);
        return false;
      }
      if (!bus_ || !bus_->initialized()) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
      }
      initialized_ = true;
      ec.clear();
      return true;
    }

    bool deinit(std::error_code &ec) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      initialized_ = false;
      ec.clear();
      return true;
    }

    int get_timeout_ms() const { return config_.timeout_ms; }
    void set_timeout_ms(int timeout_ms) { config_.timeout_ms = timeout_ms; }

    uint16_t get_device_address() const { return config_.device_address; }
    void set_device_address(uint16_t device_address, int addr_bit_len = 7) {
      config_.device_address = device_address;
      config_.addr_bit_len = addr_bit_len;
    }

    bool probe(std::error_code &ec) { return probe(config_.timeout_ms, ec); }

    bool probe(int32_t timeout_ms, std::error_code &ec) {
      (void)timeout_ms;
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!initialized_ || !bus_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
      }
      bool found = bus_->probe_device(config_.device_address);
      if (found) {
        ec.clear();
      } else {
        ec = std::make_error_code(std::errc::io_error);
      }
      return found;
    }

    bool write(const uint8_t *data, size_t len, std::error_code &ec) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!initialized_ || !bus_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
      }
      bool ok = bus_->write(config_.device_address, data, len);
      if (ok) {
        ec.clear();
      } else {
        ec = std::make_error_code(std::errc::io_error);
      }
      return ok;
    }

    bool read(uint8_t *data, size_t len, std::error_code &ec) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!initialized_ || !bus_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
      }
      bool ok = bus_->read(config_.device_address, data, len);
      if (ok) {
        ec.clear();
      } else {
        ec = std::make_error_code(std::errc::io_error);
      }
      return ok;
    }

    bool write_read(const uint8_t *wdata, size_t wlen, uint8_t *rdata, size_t rlen,
                    std::error_code &ec) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!initialized_ || !bus_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
      }
      bool ok = bus_->write_read(config_.device_address, wdata, wlen, rdata, rlen);
      if (ok) {
        ec.clear();
      } else {
        ec = std::make_error_code(std::errc::io_error);
      }
      return ok;
    }

    bool write(const std::vector<uint8_t> &data, std::error_code &ec) {
      return write(data.data(), data.size(), ec);
    }

    bool read(std::vector<uint8_t> &data, std::error_code &ec) {
      return read(data.data(), data.size(), ec);
    }

    bool write_read(const std::vector<uint8_t> &wdata, std::vector<uint8_t> &rdata,
                    std::error_code &ec) {
      return write_read(wdata.data(), wdata.size(), rdata.data(), rdata.size(), ec);
    }

    bool write_register(RegisterType reg, const uint8_t *data, size_t len, std::error_code &ec) {
      std::vector<uint8_t> buffer(sizeof(RegisterType) + len);
      std::memcpy(buffer.data(), &reg, sizeof(RegisterType));
      if (len > 0 && data) {
        std::memcpy(buffer.data() + sizeof(RegisterType), data, len);
      }
      return write(buffer.data(), buffer.size(), ec);
    }

    bool write_register(RegisterType reg, const std::vector<uint8_t> &data, std::error_code &ec) {
      return write_register(reg, data.data(), data.size(), ec);
    }

    bool read_register(RegisterType reg, uint8_t *data, size_t len, std::error_code &ec) {
      return write_read(reinterpret_cast<const uint8_t *>(&reg), sizeof(RegisterType), data, len,
                        ec);
    }

    bool read_register(RegisterType reg, std::vector<uint8_t> &data, std::error_code &ec) {
      return read_register(reg, data.data(), data.size(), ec);
    }

    const Config &config() const { return config_; }
    bool initialized() const { return initialized_; }

  protected:
    I2c *bus_ = nullptr;
    Config config_;
    bool initialized_ = false;
    mutable std::recursive_mutex mutex_;
  };

  template <typename RegisterType = uint8_t>
  using DeviceConfig = typename Device<RegisterType>::Config;

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

  const Config &config() const { return config_; }
  bool initialized() const { return initialized_; }

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

  template <typename RegisterType = uint8_t>
  std::shared_ptr<Device<RegisterType>> add_device(const DeviceConfig<RegisterType> &dev_config,
                                                   std::error_code &ec) {
    if (!initialized_) {
      logger_.error("not initialized");
      ec = std::make_error_code(std::errc::not_connected);
      return nullptr;
    }
    auto device = std::make_shared<Device<RegisterType>>(*this, dev_config);
    if (dev_config.auto_init && !device->initialized()) {
      ec = std::make_error_code(std::errc::io_error);
      return nullptr;
    }
    ec.clear();
    return device;
  }

protected:
  Config config_;
  bool initialized_ = false;
  std::mutex mutex_;
};
} // namespace espp

#elif defined(CONFIG_ESPP_I2C_USE_NEW_API) || defined(_DOXYGEN_)

#include "i2c_master.hpp"

namespace espp {
/// @brief I2C master bus wrapper with backwards-compatible helpers.
/// @details
/// On ESP-IDF v6.x this class is backed by the new master bus/device API, but it
/// keeps the familiar address-based helper methods for existing code. New code
/// can create explicit per-device handles with add_device().
///
/// \section i2c_ex1 Example
/// \snippet i2c_example.cpp i2c example
/// \section i2c_ex2 Add a device
/// \snippet i2c_example.cpp i2c device creation example
class I2c : public BaseComponent {
public:
  /// Configuration for the I2C master bus.
  struct Config {
    int isr_core_id = -1;        ///< Kept for compatibility; ignored by the new ESP-IDF API.
    i2c_port_t port = I2C_NUM_0; ///< I2C port
    gpio_num_t sda_io_num = GPIO_NUM_NC;               ///< SDA pin
    gpio_num_t scl_io_num = GPIO_NUM_NC;               ///< SCL pin
    gpio_pullup_t sda_pullup_en = GPIO_PULLUP_DISABLE; ///< SDA pullup
    gpio_pullup_t scl_pullup_en = GPIO_PULLUP_DISABLE; ///< SCL pullup
    uint32_t timeout_ms = 10;                          ///< Default timeout in milliseconds
    uint32_t clk_speed = 400 * 1000;                   ///< I2C clock speed in hertz
    bool auto_init = true; ///< Automatically initialize I2C on construction
    Logger::Verbosity log_level = Logger::Verbosity::WARN; ///< Verbosity of logger
  };

  template <typename RegisterType = uint8_t> using Device = I2cMasterDevice<RegisterType>;
  template <typename RegisterType = uint8_t>
  using DeviceConfig = typename Device<RegisterType>::Config;

  /// Construct I2C driver
  /// \param config Configuration for I2C
  explicit I2c(const Config &config)
      : BaseComponent("I2C", config.log_level)
      , config_(config)
      , master_bus_(make_master_bus_config(config)) {
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

  /// Set the logger verbosity for the bus and cached compatibility devices.
  /// \param log_level The desired logger verbosity.
  void set_log_level(Logger::Verbosity log_level) {
    BaseComponent::set_log_level(log_level);
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    master_bus_.set_log_level(log_level);
    for (auto &[address, device] : compatibility_devices_) {
      (void)address;
      if (device) {
        device->set_log_level(get_compatibility_device_log_level(log_level));
      }
    }
  }

  /// Get the active configuration.
  /// \return The active configuration.
  const Config &config() const { return config_; }

  /// Initialize the I2C bus.
  /// \param ec Error code populated on failure.
  void init(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (initialized_) {
      logger_.warn("already initialized");
      ec = std::make_error_code(std::errc::protocol_error);
      return;
    }

    if (config_.isr_core_id != -1) {
      logger_.warn("isr_core_id is ignored by the ESP-IDF v6 I2C master bus API");
    }
    if (config_.sda_pullup_en != config_.scl_pullup_en) {
      logger_.warn("ESP-IDF v6 uses a single internal-pullup flag; enabling internal pullups "
                   "when either SDA or SCL requests them");
    }

    master_bus_.init(ec);
    if (ec) {
      return;
    }
    master_bus_.set_log_level(get_log_level());
    initialized_ = true;
    ec.clear();
  }

  /// Deinitialize the I2C bus.
  /// \param ec Error code populated on failure.
  void deinit(std::error_code &ec) {
    std::map<uint16_t, std::shared_ptr<Device<uint8_t>>> compatibility_devices;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!initialized_) {
        ec.clear();
        return;
      }
      compatibility_devices.swap(compatibility_devices_);
    }

    bool device_deinit_failed = false;
    for (auto &[address, device] : compatibility_devices) {
      if (!device) {
        continue;
      }
      std::error_code device_ec;
      device->deinit(device_ec);
      if (device_ec) {
        logger_.error("could not deinitialize cached I2C device at address {:#04x}: {}", address,
                      device_ec.message());
        device_deinit_failed = true;
      }
    }

    std::error_code bus_ec;
    master_bus_.deinit(bus_ec);

    std::lock_guard<std::recursive_mutex> lock(mutex_);
    initialized_ = bus_ec ? true : false;
    if (bus_ec) {
      ec = bus_ec;
      return;
    }
    if (device_deinit_failed) {
      ec = std::make_error_code(std::errc::io_error);
      return;
    }
    ec.clear();
  }

  /// Check whether the bus is initialized.
  /// \return True if the bus is initialized.
  bool initialized() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return initialized_;
  }

  /// Add an explicit I2C device to the bus.
  /// \tparam RegisterType Register address type for the device.
  /// \param dev_config Device configuration.
  /// \param ec Error code populated on failure.
  /// \return Shared pointer to the created device, or nullptr on failure.
  template <typename RegisterType = uint8_t>
  std::shared_ptr<Device<RegisterType>> add_device(const DeviceConfig<RegisterType> &dev_config,
                                                   std::error_code &ec) {
    return master_bus_.add_device<RegisterType>(dev_config, ec);
  }

  /// Write data to an addressed I2C device.
  bool write(const uint8_t dev_addr, const uint8_t *data, const size_t data_len) {
    std::error_code ec;
    auto device = get_compatibility_device(dev_addr, ec);
    if (!device) {
      return false;
    }
    return device->write(data, data_len, ec);
  }

  /// Write data to an addressed I2C device.
  bool write_vector(const uint8_t dev_addr, const std::vector<uint8_t> &data) {
    return write(dev_addr, data.data(), data.size());
  }

  /// Write to and then read from an addressed I2C device.
  bool write_read(const uint8_t dev_addr, const uint8_t *write_data, const size_t write_size,
                  uint8_t *read_data, size_t read_size) {
    std::error_code ec;
    auto device = get_compatibility_device(dev_addr, ec);
    if (!device) {
      return false;
    }
    return device->write_read(write_data, write_size, read_data, read_size, ec);
  }

  /// Write to and then read from an addressed I2C device.
  bool write_read_vector(const uint8_t dev_addr, const std::vector<uint8_t> &write_data,
                         std::vector<uint8_t> &read_data) {
    return write_read(dev_addr, write_data.data(), write_data.size(), read_data.data(),
                      read_data.size());
  }

  /// Read from a register on an addressed I2C device.
  bool read_at_register(const uint8_t dev_addr, const uint8_t reg_addr, uint8_t *data,
                        size_t data_len) {
    std::error_code ec;
    auto device = get_compatibility_device(dev_addr, ec);
    if (!device) {
      return false;
    }
    return device->read_register(reg_addr, data, data_len, ec);
  }

  /// Read from a register on an addressed I2C device.
  bool read_at_register_vector(const uint8_t dev_addr, const uint8_t reg_addr,
                               std::vector<uint8_t> &data) {
    return read_at_register(dev_addr, reg_addr, data.data(), data.size());
  }

  /// Read data from an addressed I2C device.
  bool read(const uint8_t dev_addr, uint8_t *data, size_t data_len) {
    std::error_code ec;
    auto device = get_compatibility_device(dev_addr, ec);
    if (!device) {
      return false;
    }
    return device->read(data, data_len, ec);
  }

  /// Read data from an addressed I2C device.
  bool read_vector(const uint8_t dev_addr, std::vector<uint8_t> &data) {
    return read(dev_addr, data.data(), data.size());
  }

  /// Probe an addressed I2C device.
  bool probe_device(const uint8_t dev_addr) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_) {
      logger_.error("not initialized");
      return false;
    }
    std::error_code ec;
    auto previous_log_level = master_bus_.get_log_level();
    master_bus_.set_log_level(Logger::Verbosity::ERROR);
    bool found = master_bus_.probe(dev_addr, static_cast<int32_t>(config_.timeout_ms), ec);
    master_bus_.set_log_level(previous_log_level);
    return found;
  }

protected:
  static Logger::Verbosity get_compatibility_device_log_level(Logger::Verbosity log_level) {
    return log_level == Logger::Verbosity::DEBUG ? Logger::Verbosity::DEBUG
                                                 : Logger::Verbosity::WARN;
  }

  static I2cMasterBus::Config make_master_bus_config(const Config &config) {
    I2cMasterBus::Config master_bus_config{
        .port = config.port,
        .sda_io_num = config.sda_io_num,
        .scl_io_num = config.scl_io_num,
        .clk_speed = config.clk_speed,
        .enable_internal_pullup = config.sda_pullup_en == GPIO_PULLUP_ENABLE ||
                                  config.scl_pullup_en == GPIO_PULLUP_ENABLE,
        .log_level = config.log_level,
    };
    return master_bus_config;
  }

  DeviceConfig<uint8_t> make_compatibility_device_config(uint16_t device_address) const {
    return DeviceConfig<uint8_t>{
        .device_address = device_address,
        .timeout_ms = static_cast<int>(config_.timeout_ms),
        .scl_speed_hz = config_.clk_speed,
        .log_level = get_compatibility_device_log_level(get_log_level()),
    };
  }

  std::shared_ptr<Device<uint8_t>> get_compatibility_device(uint16_t device_address,
                                                            std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_) {
      logger_.error("not initialized");
      ec = std::make_error_code(std::errc::not_connected);
      return nullptr;
    }

    auto it = compatibility_devices_.find(device_address);
    if (it != compatibility_devices_.end() && it->second) {
      ec.clear();
      return it->second;
    }

    auto device =
        master_bus_.add_device<uint8_t>(make_compatibility_device_config(device_address), ec);
    if (!device) {
      logger_.error("could not create compatibility I2C device at address {:#04x}: {}",
                    device_address, ec.message());
      return nullptr;
    }
    device->set_log_level(get_compatibility_device_log_level(get_log_level()));
    compatibility_devices_[device_address] = device;
    ec.clear();
    return device;
  }

  Config config_;
  mutable std::recursive_mutex mutex_;
  I2cMasterBus master_bus_;
  bool initialized_ = false;
  std::map<uint16_t, std::shared_ptr<Device<uint8_t>>> compatibility_devices_;
};

} // namespace espp

#else
#error                                                                                             \
    "i2c.hpp included but neither CONFIG_ESPP_I2C_USE_LEGACY_API nor CONFIG_ESPP_I2C_USE_NEW_API is set. Please select the correct I2C API in KConfig."
#endif

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

namespace espp {

template <typename RegisterType = uint8_t>
auto make_i2c_addressed_probe(
    const std::shared_ptr<espp::I2c::template Device<RegisterType>> &device) {
  return [device](uint8_t) -> bool {
    if (!device) {
      return false;
    }
    std::error_code ec;
    return device->probe(ec);
  };
}

template <typename RegisterType = uint8_t>
auto make_i2c_addressed_write(
    const std::shared_ptr<espp::I2c::template Device<RegisterType>> &device) {
  return [device](uint8_t, const uint8_t *data, size_t len) -> bool {
    if (!device) {
      return false;
    }
    std::error_code ec;
    return device->write(data, len, ec);
  };
}

template <typename RegisterType = uint8_t>
auto make_i2c_addressed_read(
    const std::shared_ptr<espp::I2c::template Device<RegisterType>> &device) {
  return [device](uint8_t, uint8_t *data, size_t len) -> bool {
    if (!device) {
      return false;
    }
    std::error_code ec;
    return device->read(data, len, ec);
  };
}

template <typename RegisterType = uint8_t>
auto make_i2c_addressed_write_then_read(
    const std::shared_ptr<espp::I2c::template Device<RegisterType>> &device) {
  return [device](uint8_t, const uint8_t *write_data, size_t write_len, uint8_t *read_data,
                  size_t read_len) -> bool {
    if (!device) {
      return false;
    }
    std::error_code ec;
    return device->write_read(write_data, write_len, read_data, read_len, ec);
  };
}

template <typename RegisterType = uint8_t>
auto make_i2c_addressed_read_register(
    const std::shared_ptr<espp::I2c::template Device<RegisterType>> &device) {
  return [device](uint8_t, RegisterType reg, uint8_t *data, size_t len) -> bool {
    if (!device) {
      return false;
    }
    std::error_code ec;
    return device->read_register(reg, data, len, ec);
  };
}

} // namespace espp

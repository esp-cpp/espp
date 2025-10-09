#pragma once

#include <sdkconfig.h>

// Only include this header if the new API is selected
#if defined(CONFIG_ESPP_I2C_USE_NEW_API) || defined(_DOXYGEN_)

#include <functional>
#include <memory>
#include <mutex>
#include <system_error>
#include <vector>

#include "base_component.hpp"

extern "C" {
#include <driver/i2c_master.h>
}

#include "i2c_format_helpers.hpp"

namespace espp {

/// @brief I2C Master Device (C++ wrapper for ESP-IDF new I2C master API)
/// @details
/// This class is a wrapper around the ESP-IDF I2C master device API.
/// It provides thread-safe, modern C++ access to I2C master device operations.
///
/// \section i2c_master_device_ex1 Create a device
/// \snippet i2c_example.cpp i2c master device creation example
/// \section i2c_master_device_ex2 Read from a device
/// \snippet i2c_example.cpp i2c master device read example
///
/// Usage:
///   - Construct with a bus handle and config
///   - Use read, write, write_read, and register-based access methods
///   - Use probe() to check for device presence
///   - All methods are thread-safe
///
/// \note This class is intended for use with the new ESP-IDF I2C master API (>=5.4.0)
template <typename RegisterType = uint8_t> class I2cMasterDevice : public BaseComponent {
public:
  /// @brief Configuration for I2C Master Device
  struct Config {
    uint16_t device_address = 0;                          ///< I2C device address (7-bit or 10-bit)
    int timeout_ms = 10;                                  ///< Timeout for I2C operations (ms)
    i2c_addr_bit_len_t addr_bit_len = I2C_ADDR_BIT_LEN_7; ///< Address bit length
    uint32_t scl_speed_hz = 400000;                       ///< I2C clock speed in hertz
    bool auto_init = true; ///< Automatically initialize on construction
    Logger::Verbosity log_level = Logger::Verbosity::WARN; ///< Logger verbosity
  };

  /// @brief Construct I2C Master Device
  /// @param bus_handle I2C master bus handle
  /// @param config Configuration for the device
  explicit I2cMasterDevice(i2c_master_bus_handle_t bus_handle, const Config &config);

  /// @brief Destructor
  ~I2cMasterDevice();

  /// @brief Initialize the device
  /// @param ec Error code output
  /// @return True if successful
  bool init(std::error_code &ec);

  /// @brief Deinitialize the device
  /// @param ec Error code output
  /// @return True if successful
  bool deinit(std::error_code &ec);

  /// @brief Get the configured timeout (ms)
  int get_timeout_ms() const { return config_.timeout_ms; }
  /// @brief Set the timeout (ms)
  void set_timeout_ms(int timeout_ms) { config_.timeout_ms = timeout_ms; }

  /// @brief Get the device address
  /// @return Device address (7-bit or 10-bit)
  uint16_t get_device_address() const { return config_.device_address; }
  /// @brief Set the device address
  /// @param device_address Device address (7-bit or 10-bit)
  /// @param addr_bit_len Address bit length (default is 7-bit)
  void set_device_address(uint16_t device_address,
                          i2c_addr_bit_len_t addr_bit_len = I2C_ADDR_BIT_LEN_7);

  /// @brief Probe for the device on the bus
  /// @param ec Error code output
  /// @return True if device responds
  bool probe(std::error_code &ec);
  /// @brief Probe for the device with a custom timeout
  /// @param timeout_ms Timeout in ms
  /// @param ec Error code output
  /// @return True if device responds
  bool probe(int32_t timeout_ms, std::error_code &ec);

  /// @brief Write data to the device
  /// @param data Pointer to data
  /// @param len Length of data
  /// @param ec Error code output
  /// @return True if successful
  bool write(const uint8_t *data, size_t len, std::error_code &ec);
  /// @brief Read data from the device
  /// @param data Pointer to buffer
  /// @param len Length to read
  /// @param ec Error code output
  /// @return True if successful
  bool read(uint8_t *data, size_t len, std::error_code &ec);
  /// @brief Write then read from the device
  /// @param wdata Write data pointer
  /// @param wlen Write data length
  /// @param rdata Read buffer pointer
  /// @param rlen Read buffer length
  /// @param ec Error code output
  /// @return True if successful
  bool write_read(const uint8_t *wdata, size_t wlen, uint8_t *rdata, size_t rlen,
                  std::error_code &ec);

  /// @brief Write data to the device (vector overload)
  /// @param data Data vector
  /// @param ec Error code output
  /// @return True if successful
  bool write(const std::vector<uint8_t> &data, std::error_code &ec);
  /// @brief Read data from the device (vector overload)
  /// @param data Data vector (output)
  /// @param ec Error code output
  /// @return True if successful
  bool read(std::vector<uint8_t> &data, std::error_code &ec);
  /// @brief Write then read from the device (vector overload)
  /// @param wdata Write data vector
  /// @param rdata Read data vector (output)
  /// @param ec Error code output
  /// @return True if successful
  bool write_read(const std::vector<uint8_t> &wdata, std::vector<uint8_t> &rdata,
                  std::error_code &ec);

  /// @brief Write to a register
  /// @param reg Register address
  /// @param data Data pointer
  /// @param len Data length
  /// @param ec Error code output
  /// @return True if successful
  bool write_register(RegisterType reg, const uint8_t *data, size_t len, std::error_code &ec);
  /// @brief Write to a register (vector overload)
  /// @param reg Register address
  /// @param data Data vector
  /// @param ec Error code output
  /// @return True if successful
  bool write_register(RegisterType reg, const std::vector<uint8_t> &data, std::error_code &ec);
  /// @brief Read from a register
  /// @param reg Register address
  /// @param data Data pointer (output)
  /// @param len Data length
  /// @param ec Error code output
  /// @return True if successful
  bool read_register(RegisterType reg, uint8_t *data, size_t len, std::error_code &ec);
  /// @brief Read from a register (vector overload)
  /// @param reg Register address
  /// @param data Data vector (output)
  /// @param ec Error code output
  /// @return True if successful
  bool read_register(RegisterType reg, std::vector<uint8_t> &data, std::error_code &ec);

  /// @brief Expose config for CLI menu
  /// @return Reference to config
  const Config &config() const;

protected:
  Config config_;
  i2c_master_dev_handle_t dev_handle_ = nullptr;
  i2c_master_bus_handle_t bus_handle_ = nullptr;
  bool initialized_ = false;
  std::recursive_mutex mutex_;
};

/// @brief I2C Master Bus (C++ wrapper for ESP-IDF new I2C master API)
/// @details
/// This class is a wrapper around the ESP-IDF I2C master bus API.
/// It provides thread-safe, modern C++ access to I2C master bus operations and device creation.
///
/// \section i2c_master_bus_ex1 Construct
/// \snippet i2c_example.cpp i2c master bus creation example
/// \section i2c_master_bus_ex2 Create a device
/// \snippet i2c_example.cpp i2c master device creation example
/// \section i2c_master_bus_ex3 Probe for device(s)
/// \snippet i2c_example.cpp i2c master bus probe example
///
/// Usage:
///   - Construct with a config
///   - Use init() and deinit() to manage the bus
///   - Use add_device() to create I2cMasterDevice objects
///   - Use probe() to check for device presence
///   - All methods are thread-safe
///
/// \note This class is intended for use with the new ESP-IDF I2C master API (>=5.4.0)
class I2cMasterBus : public BaseComponent {
public:
  /// @brief Configuration for I2C Master Bus
  struct Config {
    int port = 0;                                          ///< I2C port number
    int sda_io_num = -1;                                   ///< SDA pin
    int scl_io_num = -1;                                   ///< SCL pin
    uint32_t clk_speed = 400000;                           ///< I2C clock speed in hertz
    bool enable_internal_pullup = true;                    ///< Enable internal pullups
    int intr_priority = 0;                                 ///< Interrupt priority
    Logger::Verbosity log_level = Logger::Verbosity::WARN; ///< Logger verbosity
  };

  /// @brief Construct I2C Master Bus
  /// @param config Configuration for the bus
  explicit I2cMasterBus(const Config &config);

  /// @brief Destructor
  ~I2cMasterBus();

  /// @brief Initialize the bus
  /// @param ec Error code output
  /// @return True if successful
  bool init(std::error_code &ec);

  /// @brief Deinitialize the bus
  /// @param ec Error code output
  /// @return True if successful
  bool deinit(std::error_code &ec);

  /// @brief Add a device to the bus
  /// @tparam RegisterType Register address type
  /// @param dev_config Device configuration
  /// @param ec Error code output
  /// @return Shared pointer to the created device
  template <typename RegisterType>
  std::shared_ptr<I2cMasterDevice<RegisterType>>
  add_device(const typename I2cMasterDevice<RegisterType>::Config &dev_config,
             std::error_code &ec) {
    logger_.info("Adding I2C device at address 0x{:02x} on bus {}", dev_config.device_address,
                 config_.port);
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_ || !bus_handle_) {
      logger_.error("Bus not initialized or handle is null");
      ec = std::make_error_code(std::errc::not_connected);
      return nullptr;
    }
    auto device = std::make_shared<I2cMasterDevice<RegisterType>>(bus_handle_, dev_config);
    logger_.info("I2C device added at address 0x{:02x} on bus {}", dev_config.device_address,
                 config_.port);
    return device;
  }

  /// @brief Probe for a device on the bus
  /// @param device_address I2C device address
  /// @param ec Error code output
  /// @return True if device responds
  bool probe(uint16_t device_address, std::error_code &ec);
  /// @brief Probe for a device with a custom timeout
  /// @param device_address I2C device address
  /// @param timeout_ms Timeout in ms
  /// @param ec Error code output
  /// @return True if device responds
  bool probe(uint16_t device_address, int32_t timeout_ms, std::error_code &ec);

protected:
  Config config_;
  i2c_master_bus_handle_t bus_handle_ = nullptr;
  bool initialized_ = false;
  std::recursive_mutex mutex_;
};

// Explicit instantiation for uint8_t and uint16_t register types
extern template class I2cMasterDevice<uint8_t>;
extern template class I2cMasterDevice<uint16_t>;

} // namespace espp

#else
#error                                                                                             \
    "i2c_master.hpp included but CONFIG_ESPP_I2C_USE_NEW_API is not set. Please select the correct I2C API in KConfig."
#endif

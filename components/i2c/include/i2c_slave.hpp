#pragma once

#include <sdkconfig.h>

// Only include this header if the new API is selected
#if defined(CONFIG_ESPP_I2C_USE_NEW_API) || defined(_DOXYGEN_)

#include <functional>
#include <memory>
#include <mutex>
#include <system_error>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "base_component.hpp"

extern "C" {
#include <driver/i2c_slave.h>
}

#include "i2c_format_helpers.hpp"

namespace espp {

/// @brief I2C Slave Device (C++ wrapper for ESP-IDF new I2C slave API)
/// @details
/// This class is a wrapper around the ESP-IDF I2C slave device API.
/// It provides thread-safe, modern C++ access to I2C slave device operations.
///
/// @note There is no example for this yet as this code is untested.
///
/// Usage:
///   - Construct with a config
///   - Use read, write, and callback registration methods
///   - All methods are thread-safe
///
/// \note This class is intended for use with the new ESP-IDF I2C slave API (>=5.4.0)
class I2cSlaveDevice : public BaseComponent {
public:
  using RequestCallback =
      std::function<void(const uint8_t *data, size_t len)>; ///< Callback for data requests
  using ReceiveCallback =
      std::function<void(const uint8_t *data, size_t len)>; ///< Callback for data received

  /// @brief Callbacks for I2C slave events
  struct Callbacks {
    RequestCallback on_request = nullptr; ///< Callback for data request from master
    ReceiveCallback on_receive = nullptr; ///< Callback for data received from master
  };

  /// @brief Configuration for I2C Slave Device
  struct Config {
    int port = 0;                                          ///< I2C port number
    int sda_io_num = -1;                                   ///< SDA pin
    int scl_io_num = -1;                                   ///< SCL pin
    uint16_t slave_address = 0;                            ///< I2C slave address
    i2c_addr_bit_len_t addr_bit_len = I2C_ADDR_BIT_LEN_7;  ///< Address bit length
    uint32_t clk_speed = 100000;                           ///< I2C clock speed in hertz
    bool enable_internal_pullup = true;                    ///< Enable internal pullups
    int intr_priority = 0;                                 ///< Interrupt priority
    Logger::Verbosity log_level = Logger::Verbosity::WARN; ///< Logger verbosity
  };

  /// @brief Construct I2C Slave Device
  /// @param config Configuration for the device
  explicit I2cSlaveDevice(const Config &config);

  /// @brief Destructor
  ~I2cSlaveDevice();

  /// @brief Initialize the device
  /// @param ec Error code output
  /// @return True if successful
  bool init(std::error_code &ec);

  /// @brief Deinitialize the device
  /// @param ec Error code output
  /// @return True if successful
  bool deinit(std::error_code &ec);

  /// @brief Write data to the master
  /// @param data Pointer to data
  /// @param len Length of data
  /// @param ec Error code output
  /// @return True if successful
  bool write(const uint8_t *data, size_t len, std::error_code &ec);
  /// @brief Read data from the master
  /// @param data Pointer to buffer
  /// @param len Length to read
  /// @param ec Error code output
  /// @return True if successful
  bool read(uint8_t *data, size_t len, std::error_code &ec);

  /// @brief Register callbacks for slave events
  /// @param callbacks Callbacks to register
  /// @param ec Error code output
  /// @return True if successful
  bool register_callbacks(const Callbacks &callbacks, std::error_code &ec);

  /// @brief Expose config for CLI menu
  /// @return Reference to config
  const Config &config() const;

protected:
  Config config_;
  i2c_slave_dev_handle_t dev_handle_ = nullptr;
  bool initialized_ = false;
  std::recursive_mutex mutex_;
  Callbacks callbacks_{};
  // FreeRTOS queue for request/receive events
  QueueHandle_t event_queue_ = nullptr;
  // TODO: create a espp::Task which waits on a queue to be notified from ISR
  // for request/receive events
};

} // namespace espp

#else
#error                                                                                             \
    "i2c_slave.hpp included but CONFIG_ESPP_I2C_USE_NEW_API is not set. Please select the correct I2C API in KConfig."
#endif

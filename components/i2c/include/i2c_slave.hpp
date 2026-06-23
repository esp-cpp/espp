#pragma once

#include <sdkconfig.h>

// Only include this header if the new API is selected
#if defined(CONFIG_ESPP_I2C_USE_NEW_API) || defined(_DOXYGEN_)

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <system_error>
#include <vector>

#include <esp_idf_version.h>
#ifndef ESP_IDF_VERSION_VAL
#define ESP_IDF_VERSION_VAL(major, minor, patch) (((major) << 16) | ((minor) << 8) | (patch))
#endif
#ifndef ESP_IDF_VERSION
#define ESP_IDF_VERSION ESP_IDF_VERSION_VAL(0, 0, 0)
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/message_buffer.h>
#include <freertos/queue.h>

#include "base_component.hpp"
#include "task.hpp"

extern "C" {
#include <driver/i2c_slave.h>
}

#include "i2c_format_helpers.hpp"

namespace espp {

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0) || CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2
#define ESPP_I2C_SLAVE_V2_API 1
#else
#define ESPP_I2C_SLAVE_V2_API 0
#endif

/// @brief I2C slave device wrapper for ESP-IDF's callback-driven slave API.
/// @details
/// ESP-IDF's slave driver is event/callback based: master writes arrive through
/// receive callbacks, and master reads can trigger request callbacks when the
/// slave transmit FIFO needs more data. This wrapper provides:
///
/// - queued `read()` access to complete master-write transactions
/// - blocking `write()` access for staging data back to the master
/// - task-context request / receive callbacks so user code does not have to run
///   inside the ISR callback context
///
/// When built against ESP-IDF v5.5's default slave driver, the receive callback
/// API does not expose the actual transaction length. In that configuration this
/// wrapper buffers the requested receive length and zero-fills any trailing bytes
/// the master did not write so `read()` and `on_receive` still behave consistently.
///
/// @note No dedicated example exists yet.
///
/// Usage:
///   - Construct with a config, then call `init()`
///   - Call `read()` to receive the next complete master-write transaction
///   - Call `write()` to stage response bytes for the next master read
///   - Register callbacks if you want task-context notifications
///
/// \note This class is intended for use with the new ESP-IDF I2C slave API (>=5.4.0)
class I2cSlaveDevice : public BaseComponent {
public:
  using RequestCallback = std::function<void()>; ///< Callback for master read requests.
  using ReceiveCallback =
      std::function<void(const uint8_t *data, size_t len)>; ///< Callback for master writes.

  /// @brief Callbacks for I2C slave events
  struct Callbacks {
    RequestCallback on_request = nullptr; ///< Callback for data request from master
    ReceiveCallback on_receive = nullptr; ///< Callback for data received from master
  };

  /// @brief Configuration for I2C Slave Device
  struct Config {
    int port = 0;                                         ///< I2C port number
    int sda_io_num = -1;                                  ///< SDA pin
    int scl_io_num = -1;                                  ///< SCL pin
    uint16_t slave_address = 0;                           ///< I2C slave address
    i2c_addr_bit_len_t addr_bit_len = I2C_ADDR_BIT_LEN_7; ///< Address bit length
    int timeout_ms = 10;                                  ///< Read/write timeout in milliseconds
    uint32_t receive_buffer_depth = 256;                  ///< Max bytes per received transaction
    uint32_t send_buffer_depth = 256;                     ///< Depth of driver TX buffer
    size_t event_queue_depth = 8;       ///< Number of queued request/receive events
    bool enable_internal_pullup = true; ///< Enable internal pullups
    int intr_priority = 0;              ///< Interrupt priority
    espp::Task::BaseConfig task_config = {.name = "I2C Slave Task"};
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
  /// @param len Maximum transaction length to read
  /// @param received_len Actual number of bytes received for the transaction
  /// @param ec Error code output
  /// @return True if a complete master-write transaction was received
  bool read(uint8_t *data, size_t len, size_t &received_len, std::error_code &ec);
  /// @brief Read data from the master
  /// @param data Pointer to buffer
  /// @param len Maximum transaction length to read
  /// @param ec Error code output
  /// @return True if a complete master-write transaction was received
  bool read(uint8_t *data, size_t len, std::error_code &ec);

  /// @brief Register callbacks for slave events
  /// @param callbacks Callbacks to register
  /// @param ec Error code output
  /// @return True if successful
  bool register_callbacks(const Callbacks &callbacks, std::error_code &ec);

  /// @brief Expose config for CLI menu
  /// @return Reference to config
  const Config &config() const { return config_; }

protected:
  enum class EventType : uint8_t { STOP, REQUEST, RECEIVE };

  struct Event {
    EventType type;
  };

#if ESPP_I2C_SLAVE_V2_API
  static bool IRAM_ATTR request_callback_trampoline(i2c_slave_dev_handle_t i2c_slave,
                                                    const i2c_slave_request_event_data_t *evt_data,
                                                    void *user_data);
#endif
  static bool IRAM_ATTR receive_callback_trampoline(i2c_slave_dev_handle_t i2c_slave,
                                                    const i2c_slave_rx_done_event_data_t *evt_data,
                                                    void *user_data);

  static size_t message_buffer_capacity(size_t max_message_size, size_t max_messages);
  static TickType_t timeout_ticks(int timeout_ms);

  bool event_task_callback(std::mutex &m, std::condition_variable &cv, bool &notified);
  void log_pending_overflows();

  Config config_;
  i2c_slave_dev_handle_t dev_handle_ = nullptr;
  bool initialized_ = false;
  std::recursive_mutex mutex_;
  Callbacks callbacks_{};
  QueueHandle_t event_queue_ = nullptr;
  MessageBufferHandle_t read_buffer_ = nullptr;
  MessageBufferHandle_t callback_buffer_ = nullptr;
  std::unique_ptr<espp::Task> event_task_;
  std::vector<uint8_t> legacy_receive_buffer_;
  std::vector<uint8_t> callback_dispatch_buffer_;
  std::atomic<size_t> legacy_receive_length_{0};
  std::atomic<bool> legacy_receive_armed_{false};
  std::atomic<bool> read_buffer_overflowed_{false};
  std::atomic<bool> callback_buffer_overflowed_{false};
  std::atomic<bool> event_queue_overflowed_{false};
};

} // namespace espp

#else
#error                                                                                             \
    "i2c_slave.hpp included but CONFIG_ESPP_I2C_USE_NEW_API is not set. Please select the correct I2C API in KConfig."
#endif

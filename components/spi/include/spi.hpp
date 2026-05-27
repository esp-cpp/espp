#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <system_error>
#include <vector>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>

#include "base_component.hpp"

namespace espp {
/// @brief SPI master wrapper and helpers.
/// @details
/// `Spi` owns a SPI bus and can create one or more attached `Device` instances.
/// Those devices support blocking reads/writes, queued transactions, and bus
/// acquisition.
///
/// \section spi_example Example
/// \snippet spi_example.cpp spi example
class Spi : public BaseComponent {
public:
  /// @brief Bus-level configuration for the SPI master.
  struct Config {
    int isr_core_id = -1;                         ///< Core on which to initialize the SPI bus.
    spi_host_device_t host = SPI2_HOST;           ///< SPI host controller to use.
    gpio_num_t sclk_io_num = GPIO_NUM_NC;         ///< Clock pin.
    gpio_num_t mosi_io_num = GPIO_NUM_NC;         ///< MOSI pin.
    gpio_num_t miso_io_num = GPIO_NUM_NC;         ///< MISO pin.
    gpio_num_t quadwp_io_num = GPIO_NUM_NC;       ///< Quad WP pin.
    gpio_num_t quadhd_io_num = GPIO_NUM_NC;       ///< Quad HD pin.
    int max_transfer_sz = 0;                      ///< Maximum transfer size in bytes.
    uint32_t bus_flags = 0;                       ///< ESP-IDF bus configuration flags.
    int intr_flags = 0;                           ///< Interrupt allocation flags.
    spi_dma_chan_t dma_channel = SPI_DMA_CH_AUTO; ///< DMA channel selection.
    bool auto_init = true;                        ///< Automatically initialize on construction.
    Logger::Verbosity log_level = Logger::Verbosity::WARN; ///< Logger verbosity.
  };

  /// @brief Per-device configuration for a device attached to the SPI bus.
  struct DeviceConfig {
    int command_bits = 0;                 ///< Number of command bits sent before payload data.
    int address_bits = 0;                 ///< Number of address bits sent before payload data.
    int dummy_bits = 0;                   ///< Number of dummy bits inserted before payload data.
    uint8_t mode = 0;                     ///< SPI mode.
    int clock_speed_hz = 1 * 1000 * 1000; ///< Device clock speed.
    int input_delay_ns = 0;               ///< Input delay in nanoseconds.
    gpio_num_t cs_io_num = GPIO_NUM_NC;   ///< Chip-select pin.
    int queue_size = 1;                   ///< Queue depth for asynchronous transactions.
    uint32_t flags = 0;                   ///< ESP-IDF device flags.
    uint16_t cs_ena_pretrans = 0;         ///< CS lead time in SPI bit-cycles.
    uint16_t cs_ena_posttrans = 0;        ///< CS hold time in SPI bit-cycles.
    transaction_cb_t pre_cb = nullptr;    ///< Optional pre-transfer callback.
    transaction_cb_t post_cb = nullptr;   ///< Optional post-transfer callback.
  };

  /// @brief Per-transaction overrides for read/write/transfer helpers.
  struct TransactionConfig {
    uint16_t command = 0;      ///< Command value for this transaction.
    uint64_t address = 0;      ///< Address value for this transaction.
    uint32_t flags = 0;        ///< ESP-IDF transaction flags.
    size_t rx_length_bits = 0; ///< Optional receive length override, in bits. Must not exceed the
                               ///< RX buffer capacity, or the TX bit length when both TX and RX are
                               ///< used in the same transaction.
  };

  class Device;

  /// @brief RAII helper for holding an acquired SPI device bus lock.
  class BusLock {
  public:
    /// @brief Construct an empty bus lock.
    BusLock();

    /// @brief Construct a bus lock from an acquired ESP-IDF SPI handle.
    /// @param handle Acquired device handle.
    explicit BusLock(spi_device_handle_t handle);

    BusLock(const BusLock &) = delete;
    BusLock &operator=(const BusLock &) = delete;

    /// @brief Move-construct the bus lock.
    /// @param other Lock to move from.
    BusLock(BusLock &&other) noexcept;

    /// @brief Move-assign the bus lock.
    /// @param other Lock to move from.
    /// @return Reference to this lock.
    BusLock &operator=(BusLock &&other) noexcept;

    /// @brief Release the held bus lock on destruction.
    ~BusLock();

    /// @brief Release the held SPI bus lock, if any.
    void release();

    /// @brief Check whether this object currently owns a bus lock.
    /// @return True if a bus lock is held.
    explicit operator bool() const;

  private:
    spi_device_handle_t handle_{nullptr};
  };

  /// @brief Construct the SPI bus wrapper.
  /// @param config SPI bus configuration.
  explicit Spi(const Config &config);

  /// @brief Deinitialize the SPI bus and remove any remaining devices.
  ~Spi();

  Spi(const Spi &) = delete;
  Spi &operator=(const Spi &) = delete;
  Spi(Spi &&) = delete;
  Spi &operator=(Spi &&) = delete;

  /// @brief Convert a `Spi::Config` into an ESP-IDF `spi_bus_config_t`.
  /// @param config High-level SPI bus configuration.
  /// @return Equivalent ESP-IDF bus configuration.
  static spi_bus_config_t make_bus_config(const Config &config);

  /// @brief Convert a `Spi::DeviceConfig` into an ESP-IDF device config.
  /// @param config High-level SPI device configuration.
  /// @return Equivalent ESP-IDF device interface configuration.
  static spi_device_interface_config_t make_device_config(const DeviceConfig &config);

  /// @brief Initialize the SPI bus.
  /// @param ec Error code populated on failure.
  void init(std::error_code &ec);

  /// @brief Deinitialize the SPI bus.
  /// @param ec Error code populated on failure.
  void deinit(std::error_code &ec);

  /// @brief Check whether the SPI bus is initialized.
  /// @return True if the SPI bus is initialized.
  bool initialized() const;

  /// @brief Get the configured SPI host.
  /// @return Configured SPI host controller.
  spi_host_device_t host() const;

  /// @brief Add a device to the SPI bus.
  /// @param config Device configuration.
  /// @param ec Error code populated on failure.
  /// @return Shared pointer to the registered device, or nullptr on failure.
  std::shared_ptr<Device> add_device(const DeviceConfig &config, std::error_code &ec);

private:
  void prune_expired_devices_locked();

  friend class Device;

  Config config_;
  mutable std::recursive_mutex mutex_;
  bool initialized_{false};
  std::vector<std::weak_ptr<Device>> devices_;
};

/// @brief Attached SPI device wrapper.
class Spi::Device : public BaseComponent {
public:
  /// @brief Construct an unattached device wrapper.
  /// @param spi Owning SPI bus wrapper.
  /// @param config Device configuration.
  explicit Device(Spi &spi, const DeviceConfig &config);

  /// @brief Remove the device from the SPI bus on destruction.
  ~Device();

  Device(const Device &) = delete;
  Device &operator=(const Device &) = delete;
  Device(Device &&) = delete;
  Device &operator=(Device &&) = delete;

  /// @brief Check whether the device is attached to the SPI bus.
  /// @return True if the device has a valid ESP-IDF handle.
  bool initialized() const;

  /// @brief Get the underlying ESP-IDF device handle.
  /// @return SPI device handle, or nullptr if unattached.
  spi_device_handle_t handle() const;

  /// @brief Get the configuration used to create this device.
  /// @return Device configuration.
  const DeviceConfig &config() const;

  /// @brief Remove this device from the SPI bus.
  /// @param ec Error code populated on failure.
  void remove_device(std::error_code &ec);

  /// @brief Submit a blocking SPI transaction.
  /// @param transaction Transaction to execute.
  /// @param ec Error code populated on failure.
  /// @return True on success.
  bool transmit(spi_transaction_t &transaction, std::error_code &ec);

  /// @brief Submit a polling SPI transaction.
  /// @param transaction Transaction to execute.
  /// @param ec Error code populated on failure.
  /// @return True on success.
  bool polling_transmit(spi_transaction_t &transaction, std::error_code &ec);

  /// @brief Queue an asynchronous SPI transaction.
  /// @param transaction Transaction to queue.
  /// @param timeout Queue timeout in FreeRTOS ticks.
  /// @param ec Error code populated on failure.
  /// @return True on success.
  bool queue_transaction(spi_transaction_t &transaction, TickType_t timeout, std::error_code &ec);

  /// @brief Wait for a queued SPI transaction to complete.
  /// @param transaction Filled with the completed transaction pointer.
  /// @param timeout Wait timeout in FreeRTOS ticks.
  /// @param ec Error code populated on failure.
  /// @return True on success.
  bool get_transaction_result(spi_transaction_t **transaction, TickType_t timeout,
                              std::error_code &ec);

  /// @brief Acquire exclusive access to the SPI bus for this device.
  /// @param timeout Acquire timeout in FreeRTOS ticks.
  /// @param ec Error code populated on failure.
  /// @return RAII bus lock object.
  BusLock acquire_bus(TickType_t timeout, std::error_code &ec);

  /// @brief Write a transmit buffer using the configured per-transaction overrides.
  /// @param data Data to transmit.
  /// @param config Transaction overrides.
  /// @param ec Error code populated on failure.
  /// @return True on success.
  bool write(std::span<const uint8_t> data, const TransactionConfig &config, std::error_code &ec);

  /// @brief Read data using the configured per-transaction overrides.
  /// @param data Receive buffer to fill.
  /// @param config Transaction overrides.
  /// @param ec Error code populated on failure.
  /// @return True on success.
  bool read(std::span<uint8_t> data, const TransactionConfig &config, std::error_code &ec);

  /// @brief Perform a combined transmit/receive transaction.
  /// @param tx_data Transmit buffer.
  /// @param rx_data Receive buffer.
  /// @param config Transaction overrides.
  /// @param ec Error code populated on failure.
  /// @return True on success.
  bool transfer(std::span<const uint8_t> tx_data, std::span<uint8_t> rx_data,
                const TransactionConfig &config, std::error_code &ec);

private:
  friend class Spi;

  Spi &spi_;
  DeviceConfig config_;
  spi_device_handle_t handle_{nullptr};
  mutable std::recursive_mutex mutex_;
};
} // namespace espp

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <vector>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>

#include "base_component.hpp"
#include "display_drivers.hpp"
#include "spi.hpp"

namespace espp {
/// @brief LCD-style command/data helper built on top of `Spi`.
/// @details
/// `queue_command()` transmits with D/C low. `queue_data()` and
/// `queue_pixels()` always transmit with D/C high, so callers should treat them
/// as payload helpers rather than manually setting the D/C bit for normal panel
/// data transactions.
class SpiPanelIo : public display_drivers::PanelIo, public BaseComponent {
public:
  /// @brief IRQ-safe callback invoked after matching queued transactions finish.
  using post_transaction_callback_t = void (*)(uint32_t user_flags);

  /// @brief Configuration for `SpiPanelIo`.
  struct Config {
    Spi *spi = nullptr;                       ///< Bus on which to register the display device.
    Spi::DeviceConfig device_config{};        ///< SPI device configuration.
    gpio_num_t data_command_io = GPIO_NUM_NC; ///< D/C GPIO pin.
    uint32_t data_command_bit_mask =
        1u << static_cast<uint32_t>(display_drivers::Flags::DC_LEVEL_BIT); ///< User bit that
                                                                           ///< selects data mode.
    uint32_t post_transaction_callback_bit_mask = 0; ///< User bit mask that triggers the callback.
    post_transaction_callback_t post_transaction_callback =
        nullptr; ///< Optional IRQ-safe callback.
    uint32_t pixel_data_flags =
        SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL; ///< Flags used for pixel payload transfers.
    uint32_t timeout_ms = 10;              ///< Queue/result timeout in milliseconds.
    Logger::Verbosity log_level = Logger::Verbosity::WARN; ///< Logger verbosity.
  };

  /// @brief Construct a panel-I/O helper for SPI command/data displays.
  /// @param config Panel transport configuration.
  explicit SpiPanelIo(const Config &config);

  /// @brief Check whether the helper has a registered SPI device.
  /// @return True if initialization succeeded.
  bool initialized() const override;

  /// @brief Get the underlying SPI device wrapper.
  /// @return Shared pointer to the attached SPI device.
  std::shared_ptr<Spi::Device> device() const;

  /// @brief Wait for all queued transactions to complete.
  void wait() override;

  /// @brief Send a command byte and optional parameter payload.
  /// @param command Command byte sent with D/C low.
  /// @param parameters Optional payload bytes sent with D/C high.
  /// @param user_flags Additional user-defined flags passed to callbacks.
  void write_command(uint8_t command, std::span<const uint8_t> parameters,
                     uint32_t user_flags = 0) override;

  /// @brief Queue a command transaction with D/C low.
  /// @param command Command byte to transmit.
  /// @param user_flags Additional user-defined flags passed to callbacks.
  void queue_command(uint8_t command, uint32_t user_flags = 0) override;

  /// @brief Queue a non-pixel data payload with D/C high.
  /// @param data Data bytes to transmit.
  /// @param user_flags Additional user-defined flags passed to callbacks.
  void queue_data(std::span<const uint8_t> data, uint32_t user_flags = 0) override;

  /// @brief Queue a pixel payload with D/C high.
  /// @param data Pointer to the pixel buffer.
  /// @param size Pixel payload size in bytes.
  /// @param user_flags Additional user-defined flags passed to callbacks.
  /// @param transaction_flags Optional SPI transaction flags overriding the default pixel flags.
  void queue_pixels(const uint8_t *data, size_t size, uint32_t user_flags = 0,
                    uint32_t transaction_flags = 0) override;

private:
  struct TransactionContext {
    SpiPanelIo *helper{nullptr};
    uint32_t user_flags{0};
  };

  static void pre_transfer_callback(spi_transaction_t *transaction);
  static void post_transfer_callback(spi_transaction_t *transaction);

  TickType_t timeout_ticks() const;
  size_t prepare_transaction(uint32_t user_flags);
  void queue_command_locked(uint8_t command, uint32_t user_flags);
  void queue_data_locked(std::span<const uint8_t> data, uint32_t user_flags);
  void queue_pixels_locked(const uint8_t *data, size_t size, uint32_t user_flags,
                           uint32_t transaction_flags);
  void queue_transaction_locked(size_t index);
  void wait_locked();

  Config config_;
  std::shared_ptr<Spi::Device> device_{};
  std::mutex mutex_;
  std::vector<spi_transaction_t> transactions_{};
  std::vector<TransactionContext> contexts_{};
  int queued_transactions_{0};
};

/// @brief Backward-compatible alias for the older SPI display transport name.
using SpiCommandData = SpiPanelIo;
} // namespace espp

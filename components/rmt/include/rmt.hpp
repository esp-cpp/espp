#pragma once

#include <memory>

#include <driver/rmt_tx.h>

#include "logger.hpp"
#include "rmt_encoder.hpp"

namespace espp {
/// \brief Class wrapping the RMT peripheral on the ESP32
/// \details The RMT (Remote Control Transceiver) peripheral is used to
///  generate precise timing pulses on a GPIO pin. It can be used to drive a
///  WS2812B or similar LED strip which uses a 1-wire protocol such as the
///  WS2812B. The RMT peripheral is also used by the ESP32 to drive the IR
///  transmitter.
///
///  \sa
///  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html
///
///  \section rmt_ex1 Example 1: Transmitting data
///  \snippet rmt_example.cpp rmt example
class Rmt {
public:
  /// \brief Configuration for the RMT class
  struct Config {
    int gpio_num;                                       ///< GPIO pin to use for the RMT peripheral
    rmt_clock_source_t clock_src = RMT_CLK_SRC_DEFAULT; ///< Clock source for the RMT peripheral
    bool dma_enabled = false; ///< Whether to use DMA for the RMT peripheral
    int block_size =
        64; ///< Memory block size (e.g. 64 * 4 = 256 bytes) for the RMT peripheral. Note: this has
            ///< different meaning depending on whether DMA is configured or not. Suggested size
            ///< without DMA is >= 64, with DMA is >= 1024.
    size_t resolution_hz = 10000000; ///< Resolution of the RMT peripheral
    int transaction_queue_depth =
        1; ///< Depth of the RMT transaction queue (number of transactions that can be queued)
    Logger::Verbosity log_level = Logger::Verbosity::WARN; ///< Log level for this class
  };

  /// \brief Constructor
  /// \param config Configuration for this class
  Rmt(const Config &config) : logger_({.tag = "RMT", .level = config.log_level}) { init(config); }

  /// \brief Destructor
  /// \details This function disables the RMT peripheral and frees the
  /// RMT channel.
  ~Rmt() {
    esp_err_t err = rmt_disable(channel_);
    if (err != ESP_OK) {
      logger_.error("Failed to disable RMT peripheral: '{}' ({})", esp_err_to_name(err), err);
    }
    err = rmt_del_channel(channel_);
    if (err != ESP_OK) {
      logger_.error("Failed to delete RMT: '{}' ({})", esp_err_to_name(err), err);
    }
  }

  void set_encoder(std::unique_ptr<RmtEncoder> encoder) {
    // NOTE: this will delete the existing encoder if there is one
    encoder_ = std::move(encoder);
  }

  /// \brief Transmit a buffer of data using the RMT peripheral
  /// \param data Pointer to the data to transmit
  /// \param length Length of the data to transmit
  /// \return True if the data was successfully transmitted, false otherwise
  /// \note This function blocks until the data has been transmitted.
  bool transmit(const uint8_t *data, size_t length) {
    if (!encoder_) {
      logger_.error("No encoder set");
      return false;
    }
    rmt_transmit_config_t tx_config;
    memset(&tx_config, 0, sizeof(tx_config));
    tx_config.loop_count = 0; // no transfer loop
    esp_err_t err = rmt_transmit(channel_, encoder_->handle(), data, length, &tx_config);
    if (err != ESP_OK) {
      logger_.error("Failed to transmit: '{}' ({})", esp_err_to_name(err), err);
      return false;
    }
    return true;
  }

protected:
  /// \brief Initialize the RMT peripheral
  /// \param config Configuration for this class
  /// \return True if the RMT peripheral was successfully initialized, false otherwise
  /// \note This function is protected because it is called by the
  /// constructor. It should not be called again.
  bool init(const Config &config) {
    esp_err_t err;
    rmt_tx_channel_config_t tx_channel_config;
    memset(&tx_channel_config, 0, sizeof(tx_channel_config));
    tx_channel_config.clk_src = config.clock_src;
    tx_channel_config.gpio_num = static_cast<gpio_num_t>(config.gpio_num);
    tx_channel_config.flags.invert_out = false;
    tx_channel_config.flags.with_dma = config.dma_enabled;
    tx_channel_config.mem_block_symbols = config.block_size;
    tx_channel_config.resolution_hz = config.resolution_hz;
    tx_channel_config.trans_queue_depth = config.transaction_queue_depth;
    err = rmt_new_tx_channel(&tx_channel_config, &channel_);
    if (err != ESP_OK) {
      logger_.error("Failed to initialize RMT peripheral: '{}' ({})", esp_err_to_name(err), err);
      return false;
    }

    err = rmt_enable(channel_);
    if (err != ESP_OK) {
      logger_.error("Failed to enable RMT peripheral: '{}' ({})", esp_err_to_name(err), err);
      return false;
    }

    return true;
  }

  rmt_channel_handle_t channel_; ///< RMT channel handle

  std::unique_ptr<RmtEncoder> encoder_;

  Logger logger_; ///< Logger for this class
};
} // namespace espp

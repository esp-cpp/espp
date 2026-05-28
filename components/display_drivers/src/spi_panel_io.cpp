#include "spi_panel_io.hpp"

#include <algorithm>
#include <cstring>

namespace espp {
SpiPanelIo::SpiPanelIo(const Config &config)
    : BaseComponent("SPI Panel IO", config.log_level)
    , config_(config) {
  if (config_.device_config.queue_size < 1) {
    logger_.error("SPI panel device queue_size must be at least 1");
    return;
  }
  if (config_.data_command_io != GPIO_NUM_NC && config_.data_command_bit_mask == 0) {
    logger_.error("SPI panel D/C bit mask must be non-zero when a D/C GPIO is configured");
    return;
  }

  auto queue_depth = static_cast<size_t>(std::max(2, config_.device_config.queue_size));
  transactions_.resize(queue_depth);
  contexts_.resize(queue_depth);
  if (!config_.spi) {
    logger_.error("missing SPI bus");
    return;
  }
  if (config_.data_command_io != GPIO_NUM_NC) {
    gpio_set_direction(config_.data_command_io, GPIO_MODE_OUTPUT);
    gpio_set_level(config_.data_command_io, 0);
  }
  auto device_config = config_.device_config;
  device_config.queue_size = static_cast<int>(queue_depth);
  device_config.pre_cb = &SpiPanelIo::pre_transfer_callback;
  device_config.post_cb = &SpiPanelIo::post_transfer_callback;
  std::error_code ec;
  device_ = config_.spi->add_device(device_config, ec);
  if (ec || !device_) {
    logger_.error("could not initialize SPI command/data device");
  }
}

bool SpiPanelIo::initialized() const { return static_cast<bool>(device_); }

std::shared_ptr<Spi::Device> SpiPanelIo::device() const { return device_; }

void SpiPanelIo::wait() {
  std::lock_guard<std::mutex> lock(mutex_);
  wait_locked();
}

void SpiPanelIo::write_command(uint8_t command, std::span<const uint8_t> parameters,
                               uint32_t user_flags) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!device_) {
    logger_.error("device not initialized");
    return;
  }
  wait_locked();

  queue_command_locked(command, user_flags);

  if (parameters.empty()) {
    wait_locked();
    return;
  }

  queue_data_locked(parameters, user_flags | config_.data_command_bit_mask);
  wait_locked();
}

void SpiPanelIo::queue_command(uint8_t command, uint32_t user_flags) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!device_) {
    logger_.error("device not initialized");
    return;
  }
  queue_command_locked(command, user_flags);
}

void SpiPanelIo::queue_data(std::span<const uint8_t> data, uint32_t user_flags) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!device_) {
    logger_.error("device not initialized");
    return;
  }
  if (data.empty()) {
    return;
  }
  queue_data_locked(data, user_flags | config_.data_command_bit_mask);
}

void SpiPanelIo::queue_pixels(const uint8_t *data, size_t size, uint32_t user_flags,
                              uint32_t transaction_flags) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!device_) {
    logger_.error("device not initialized");
    return;
  }
  if (!data || size == 0) {
    logger_.error("cannot queue null or empty pixel data");
    return;
  }
  queue_pixels_locked(data, size, config_.data_command_bit_mask | user_flags, transaction_flags);
}

void SpiPanelIo::pre_transfer_callback(spi_transaction_t *transaction) {
  auto *context = static_cast<TransactionContext *>(transaction->user);
  if (!context || !context->helper) {
    return;
  }
  if (context->helper->config_.data_command_io == GPIO_NUM_NC) {
    return;
  }
  bool dc_level = (context->user_flags & context->helper->config_.data_command_bit_mask) != 0;
  gpio_set_level(context->helper->config_.data_command_io, dc_level);
}

void SpiPanelIo::post_transfer_callback(spi_transaction_t *transaction) {
  auto *context = static_cast<TransactionContext *>(transaction->user);
  if (!context || !context->helper) {
    return;
  }
  if (!context->helper->config_.post_transaction_callback) {
    return;
  }
  if (context->helper->config_.post_transaction_callback_bit_mask != 0 &&
      (context->user_flags & context->helper->config_.post_transaction_callback_bit_mask) == 0) {
    return;
  }
  context->helper->config_.post_transaction_callback(context->user_flags);
}

TickType_t SpiPanelIo::timeout_ticks() const { return pdMS_TO_TICKS(config_.timeout_ms); }

size_t SpiPanelIo::prepare_transaction(uint32_t user_flags) {
  if (queued_transactions_ >= static_cast<int>(transactions_.size())) {
    wait_locked();
  }
  auto index = static_cast<size_t>(queued_transactions_);
  std::memset(&transactions_[index], 0, sizeof(transactions_[index]));
  contexts_[index] = {.helper = this, .user_flags = user_flags};
  transactions_[index].user = &contexts_[index];
  return index;
}

void SpiPanelIo::queue_command_locked(uint8_t command, uint32_t user_flags) {
  auto index = prepare_transaction(user_flags);
  transactions_[index].length = 8;
  transactions_[index].flags = SPI_TRANS_USE_TXDATA;
  transactions_[index].tx_data[0] = command;
  queue_transaction_locked(index);
}

void SpiPanelIo::queue_data_locked(std::span<const uint8_t> data, uint32_t user_flags) {
  auto index = prepare_transaction(user_flags);
  transactions_[index].length = data.size() * 8;
  if (data.size() <= sizeof(transactions_[index].tx_data)) {
    std::memcpy(transactions_[index].tx_data, data.data(), data.size());
    transactions_[index].flags = SPI_TRANS_USE_TXDATA;
  } else {
    transactions_[index].tx_buffer = data.data();
  }
  queue_transaction_locked(index);
}

void SpiPanelIo::queue_pixels_locked(const uint8_t *data, size_t size, uint32_t user_flags,
                                     uint32_t transaction_flags) {
  auto index = prepare_transaction(user_flags);
  transactions_[index].length = size * 8;
  transactions_[index].flags =
      transaction_flags != 0 ? transaction_flags : config_.pixel_data_flags;
  transactions_[index].tx_buffer = data;
  queue_transaction_locked(index);
}

void SpiPanelIo::queue_transaction_locked(size_t index) {
  std::error_code ec;
  if (device_->queue_transaction(transactions_[index], timeout_ticks(), ec)) {
    queued_transactions_++;
    return;
  }
  logger_.error("could not queue SPI command/data transaction");
}

void SpiPanelIo::wait_locked() {
  spi_transaction_t *completed = nullptr;
  while (queued_transactions_ > 0) {
    std::error_code ec;
    if (!device_->get_transaction_result(&completed, timeout_ticks(), ec)) {
      logger_.error("could not get queued SPI transaction result");
      continue;
    }
    (void)completed;
    queued_transactions_--;
  }
}
} // namespace espp

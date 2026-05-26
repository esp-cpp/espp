#include "spi.hpp"

#include <algorithm>
#include <cstring>
#include <utility>

#include "run_on_core.hpp"

namespace espp {
Spi::BusLock::BusLock() = default;

Spi::BusLock::BusLock(spi_device_handle_t handle)
    : handle_(handle) {}

Spi::BusLock::BusLock(BusLock &&other) noexcept
    : handle_(other.handle_) {
  other.handle_ = nullptr;
}

Spi::BusLock &Spi::BusLock::operator=(BusLock &&other) noexcept {
  if (this == &other) {
    return *this;
  }
  release();
  handle_ = other.handle_;
  other.handle_ = nullptr;
  return *this;
}

Spi::BusLock::~BusLock() { release(); }

void Spi::BusLock::release() {
  if (handle_) {
    spi_device_release_bus(handle_);
    handle_ = nullptr;
  }
}

Spi::BusLock::operator bool() const { return handle_ != nullptr; }

Spi::Spi(const Config &config)
    : BaseComponent("SPI", config.log_level)
    , config_(config) {
  if (config.auto_init) {
    std::error_code ec;
    init(ec);
    if (ec) {
      logger_.error("auto init failed");
    }
  }
}

Spi::~Spi() {
  std::error_code ec;
  deinit(ec);
  if (ec) {
    logger_.error("deinit failed");
  }
}

spi_bus_config_t Spi::make_bus_config(const Config &config) {
  spi_bus_config_t bus_config{};
  bus_config.mosi_io_num = config.mosi_io_num;
  bus_config.miso_io_num = config.miso_io_num;
  bus_config.sclk_io_num = config.sclk_io_num;
  bus_config.quadwp_io_num = config.quadwp_io_num;
  bus_config.quadhd_io_num = config.quadhd_io_num;
  bus_config.max_transfer_sz = config.max_transfer_sz;
  bus_config.flags = config.bus_flags;
  bus_config.intr_flags = config.intr_flags;
  return bus_config;
}

spi_device_interface_config_t Spi::make_device_config(const DeviceConfig &config) {
  spi_device_interface_config_t device_config{};
  device_config.command_bits = config.command_bits;
  device_config.address_bits = config.address_bits;
  device_config.dummy_bits = config.dummy_bits;
  device_config.mode = config.mode;
  device_config.clock_speed_hz = config.clock_speed_hz;
  device_config.input_delay_ns = config.input_delay_ns;
  device_config.spics_io_num = config.cs_io_num;
  device_config.queue_size = config.queue_size;
  device_config.flags = config.flags;
  device_config.cs_ena_pretrans = config.cs_ena_pretrans;
  device_config.cs_ena_posttrans = config.cs_ena_posttrans;
  device_config.pre_cb = config.pre_cb;
  device_config.post_cb = config.post_cb;
  return device_config;
}

void Spi::init(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (initialized_) {
    logger_.warn("already initialized");
    ec = std::make_error_code(std::errc::protocol_error);
    return;
  }

  auto bus_config = make_bus_config(config_);
  auto install_fn = [this, &bus_config]() -> esp_err_t {
    return spi_bus_initialize(config_.host, &bus_config, config_.dma_channel);
  };
  auto err = espp::task::run_on_core(install_fn, config_.isr_core_id);
  if (err != ESP_OK) {
    logger_.error("could not initialize SPI bus: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return;
  }
  initialized_ = true;
  ec.clear();
}

void Spi::deinit(std::error_code &ec) {
  std::vector<std::shared_ptr<Device>> devices;
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_) {
      ec.clear();
      return;
    }
    for (auto &weak_device : devices_) {
      if (auto device = weak_device.lock()) {
        devices.push_back(device);
      }
    }
    devices_.clear();
  }

  for (auto &device : devices) {
    std::error_code device_ec;
    device->remove_device(device_ec);
    if (device_ec) {
      logger_.error("could not remove SPI device before bus free");
    }
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto err = spi_bus_free(config_.host);
  if (err != ESP_OK) {
    logger_.error("could not free SPI bus: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return;
  }
  initialized_ = false;
  ec.clear();
}

bool Spi::initialized() const { return initialized_; }

spi_host_device_t Spi::host() const { return config_.host; }

std::shared_ptr<Spi::Device> Spi::add_device(const DeviceConfig &config, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!initialized_) {
    logger_.error("bus not initialized");
    ec = std::make_error_code(std::errc::not_connected);
    return nullptr;
  }
  auto device = std::make_shared<Device>(*this, config);
  auto device_config = make_device_config(config);
  auto err = spi_bus_add_device(this->config_.host, &device_config, &device->handle_);
  if (err != ESP_OK) {
    logger_.error("could not add SPI device: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return nullptr;
  }
  devices_.push_back(device);
  ec.clear();
  return device;
}

Spi::Device::Device(Spi &spi, const DeviceConfig &config)
    : BaseComponent("SPI Device", spi.get_log_level())
    , spi_(spi)
    , config_(config) {}

Spi::Device::~Device() {
  std::error_code ec;
  remove_device(ec);
  if (ec) {
    logger_.error("device removal failed");
  }
}

bool Spi::Device::initialized() const { return handle_ != nullptr; }

spi_device_handle_t Spi::Device::handle() const { return handle_; }

const Spi::DeviceConfig &Spi::Device::config() const { return config_; }

void Spi::Device::remove_device(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!handle_) {
    ec.clear();
    return;
  }
  auto err = spi_bus_remove_device(handle_);
  if (err != ESP_OK) {
    logger_.error("could not remove SPI device: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return;
  }
  handle_ = nullptr;
  ec.clear();
}

bool Spi::Device::transmit(spi_transaction_t &transaction, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!handle_) {
    logger_.error("device not initialized");
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  auto err = spi_device_transmit(handle_, &transaction);
  if (err != ESP_OK) {
    logger_.error("could not transmit SPI transaction: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  ec.clear();
  return true;
}

bool Spi::Device::polling_transmit(spi_transaction_t &transaction, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!handle_) {
    logger_.error("device not initialized");
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  auto err = spi_device_polling_transmit(handle_, &transaction);
  if (err != ESP_OK) {
    logger_.error("could not polling-transmit SPI transaction: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  ec.clear();
  return true;
}

bool Spi::Device::queue_transaction(spi_transaction_t &transaction, TickType_t timeout,
                                    std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!handle_) {
    logger_.error("device not initialized");
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  auto err = spi_device_queue_trans(handle_, &transaction, timeout);
  if (err != ESP_OK) {
    logger_.error("could not queue SPI transaction: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  ec.clear();
  return true;
}

bool Spi::Device::get_transaction_result(spi_transaction_t **transaction, TickType_t timeout,
                                         std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!handle_) {
    logger_.error("device not initialized");
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  auto err = spi_device_get_trans_result(handle_, transaction, timeout);
  if (err != ESP_OK) {
    logger_.error("could not get SPI transaction result: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  ec.clear();
  return true;
}

Spi::BusLock Spi::Device::acquire_bus(TickType_t timeout, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!handle_) {
    logger_.error("device not initialized");
    ec = std::make_error_code(std::errc::not_connected);
    return {};
  }
  auto err = spi_device_acquire_bus(handle_, timeout);
  if (err != ESP_OK) {
    logger_.error("could not acquire SPI bus: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return {};
  }
  ec.clear();
  return BusLock(handle_);
}

bool Spi::Device::write(std::span<const uint8_t> data, const TransactionConfig &config,
                        std::error_code &ec) {
  return transfer(data, {}, config, ec);
}

bool Spi::Device::read(std::span<uint8_t> data, const TransactionConfig &config,
                       std::error_code &ec) {
  return transfer({}, data, config, ec);
}

bool Spi::Device::transfer(std::span<const uint8_t> tx_data, std::span<uint8_t> rx_data,
                           const TransactionConfig &config, std::error_code &ec) {
  spi_transaction_t transaction{};
  transaction.cmd = config.command;
  transaction.addr = config.address;
  transaction.flags = config.flags;

  if (!tx_data.empty()) {
    transaction.length = tx_data.size() * 8;
    if (tx_data.size() <= sizeof(transaction.tx_data)) {
      std::memcpy(transaction.tx_data, tx_data.data(), tx_data.size());
      transaction.flags |= SPI_TRANS_USE_TXDATA;
    } else {
      transaction.tx_buffer = tx_data.data();
    }
  } else if (!rx_data.empty()) {
    transaction.length = rx_data.size() * 8;
  }

  if (!rx_data.empty()) {
    transaction.rxlength = config.rx_length_bits ? config.rx_length_bits : rx_data.size() * 8;
    if (rx_data.size() <= sizeof(transaction.rx_data)) {
      transaction.flags |= SPI_TRANS_USE_RXDATA;
    } else {
      transaction.rx_buffer = rx_data.data();
    }
  }

  if (!transmit(transaction, ec)) {
    return false;
  }

  if (!rx_data.empty() && rx_data.size() <= sizeof(transaction.rx_data)) {
    std::memcpy(rx_data.data(), transaction.rx_data, rx_data.size());
  }
  ec.clear();
  return true;
}

SpiPanelIo::SpiPanelIo(const Config &config)
    : BaseComponent("SPI Panel IO", config.log_level)
    , config_(config) {
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
    }
    (void)completed;
    queued_transactions_--;
  }
}
} // namespace espp

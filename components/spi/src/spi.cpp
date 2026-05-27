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
    prune_expired_devices_locked();
    for (auto &weak_device : devices_) {
      if (auto device = weak_device.lock()) {
        devices.push_back(device);
      }
    }
    initialized_ = false;
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
  ec.clear();
}

bool Spi::initialized() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return initialized_;
}

spi_host_device_t Spi::host() const { return config_.host; }

std::shared_ptr<Spi::Device> Spi::add_device(const DeviceConfig &config, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!initialized_) {
    logger_.error("bus not initialized");
    ec = std::make_error_code(std::errc::not_connected);
    return nullptr;
  }
  prune_expired_devices_locked();
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

void Spi::prune_expired_devices_locked() {
  std::erase_if(devices_, [](const auto &device) { return device.expired(); });
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

bool Spi::Device::initialized() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return handle_ != nullptr;
}

spi_device_handle_t Spi::Device::handle() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return handle_;
}

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
  if (tx_data.empty() && rx_data.empty()) {
    ec.clear();
    return true;
  }

  const size_t tx_length_bits = tx_data.size() * 8;
  const size_t rx_buffer_bits = rx_data.size() * 8;
  const size_t rx_length_bits = config.rx_length_bits ? config.rx_length_bits : rx_buffer_bits;

  if (!rx_data.empty() && rx_length_bits > rx_buffer_bits) {
    logger_.error("requested RX length exceeds RX buffer capacity");
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  if (!tx_data.empty() && !rx_data.empty() && rx_length_bits > tx_length_bits) {
    logger_.error("requested RX length exceeds TX clock length");
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }

  spi_transaction_t transaction{};
  transaction.cmd = config.command;
  transaction.addr = config.address;
  transaction.flags = config.flags;

  if (!tx_data.empty()) {
    transaction.length = tx_length_bits;
    if (tx_data.size() <= sizeof(transaction.tx_data)) {
      std::memcpy(transaction.tx_data, tx_data.data(), tx_data.size());
      transaction.flags |= SPI_TRANS_USE_TXDATA;
    } else {
      transaction.tx_buffer = tx_data.data();
    }
  } else if (!rx_data.empty()) {
    transaction.length = rx_length_bits;
  }

  if (!rx_data.empty()) {
    transaction.rxlength = rx_length_bits;
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
} // namespace espp

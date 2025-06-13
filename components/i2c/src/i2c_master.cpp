#include <sdkconfig.h>

// Only compile this file if the new API is selected
#if defined(CONFIG_ESPP_I2C_USE_NEW_API)

#include "i2c_master.hpp"

namespace espp {

I2cMasterBus::I2cMasterBus(const Config &config)
    : BaseComponent("I2cMasterBus", config.log_level)
    , config_(config) {
  logger_.info("I2cMasterBus constructed with port={}, sda_io_num={}, scl_io_num={}, clk_speed={}",
               config.port, config.sda_io_num, config.scl_io_num, config.clk_speed);
}

I2cMasterBus::~I2cMasterBus() {
  std::error_code ec;
  deinit(ec);
  logger_.info("I2cMasterBus destroyed");
}

bool I2cMasterBus::init(std::error_code &ec) {
  logger_.info("Initializing I2C bus on port {}", config_.port);
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (initialized_) {
    logger_.warn("I2C bus already initialized");
    ec = std::make_error_code(std::errc::already_connected);
    return false;
  }
  i2c_master_bus_config_t bus_cfg = {};
  bus_cfg.i2c_port = config_.port;
  bus_cfg.sda_io_num = static_cast<gpio_num_t>(config_.sda_io_num);
  bus_cfg.scl_io_num = static_cast<gpio_num_t>(config_.scl_io_num);
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.intr_priority = config_.intr_priority;
  bus_cfg.trans_queue_depth = 0; // Force synchronous / blocking mode.
  bus_cfg.flags.enable_internal_pullup = config_.enable_internal_pullup;
  bus_cfg.flags.allow_pd = 0;
  bus_cfg.glitch_ignore_cnt = 7;
  logger_.debug(
      "Bus config: port={}, sda={}, scl={}, clk={}, intr_prio={}, queue_depth={}, pullup={}",
      bus_cfg.i2c_port, bus_cfg.sda_io_num, bus_cfg.scl_io_num, config_.clk_speed,
      bus_cfg.intr_priority, bus_cfg.trans_queue_depth, (bool)bus_cfg.flags.enable_internal_pullup);
  esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus_handle_);
  if (err != ESP_OK) {
    logger_.error("Failed to initialize I2C bus: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  initialized_ = true;
  logger_.info("I2C bus initialized on port {}", config_.port);
  ec.clear();
  return true;
}

bool I2cMasterBus::deinit(std::error_code &ec) {
  logger_.info("Deinitializing I2C bus on port {}", config_.port);
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!initialized_) {
    logger_.warn("I2C bus not initialized");
    return true;
  }
  if (bus_handle_) {
    esp_err_t err = i2c_del_master_bus(bus_handle_);
    if (err != ESP_OK) {
      logger_.error("Failed to delete I2C bus: {}", esp_err_to_name(err));
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    bus_handle_ = nullptr;
  }
  initialized_ = false;
  logger_.info("I2C bus deinitialized on port {}", config_.port);
  ec.clear();
  return true;
}

bool I2cMasterBus::probe(uint16_t device_address, std::error_code &ec) {
  // use default of 10ms timeout
  return probe(device_address, 10, ec);
}

bool I2cMasterBus::probe(uint16_t device_address, int32_t timeout_ms, std::error_code &ec) {
  logger_.info("Probing for device at address 0x{:02x} on bus {}", device_address, config_.port);
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!initialized_ || !bus_handle_) {
    logger_.error("Bus not initialized or handle is null");
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  esp_err_t err = i2c_master_probe(bus_handle_, device_address, timeout_ms);
  if (err == ESP_OK) {
    logger_.info("Device 0x{:02x} found on bus {}", device_address, config_.port);
    ec.clear();
    return true;
  } else if (err == ESP_ERR_TIMEOUT) {
    logger_.error("Probe timeout for device 0x{:02x} on bus {}: {}", device_address, config_.port,
                  esp_err_to_name(err));
    ec = std::make_error_code(std::errc::timed_out);
    return false;
  }
  logger_.warn("Device 0x{:02x} not found on bus {}: {}", device_address, config_.port,
               esp_err_to_name(err));
  ec = std::make_error_code(std::errc::io_error);
  return false;
}

template <typename RegisterType>
I2cMasterDevice<RegisterType>::I2cMasterDevice(i2c_master_bus_handle_t bus_handle,
                                               const Config &config)
    : BaseComponent("I2cMasterDevice", config.log_level)
    , config_(config) {
  bus_handle_ = bus_handle;
  logger_.info("I2cMasterDevice constructed for address 0x{:02x}, speed {}", config.device_address,
               config.scl_speed_hz);
  if (config_.auto_init) {
    std::error_code ec;
    if (!init(ec)) {
      logger_.error("Failed to initialize I2C device at address 0x{:02x}: {}",
                    config_.device_address, ec.message());
    }
  }
}

template <typename RegisterType> I2cMasterDevice<RegisterType>::~I2cMasterDevice() {
  std::error_code ec;
  deinit(ec);
  logger_.info("I2cMasterDevice destroyed for address 0x{:02x}", config_.device_address);
}

template <typename RegisterType> bool I2cMasterDevice<RegisterType>::init(std::error_code &ec) {
  logger_.info("Initializing I2C device at address 0x{:02x}", config_.device_address);
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (initialized_) {
    logger_.warn("Device already initialized");
    ec = std::make_error_code(std::errc::already_connected);
    return false;
  }
  i2c_device_config_t dev_cfg = {};
  dev_cfg.dev_addr_length = config_.addr_bit_len;
  dev_cfg.device_address = config_.device_address;
  dev_cfg.scl_speed_hz = config_.scl_speed_hz;
  dev_cfg.scl_wait_us = 0;
  dev_cfg.flags.disable_ack_check = 0;
  logger_.debug("Device config: address=0x{:02x}, speed={}, addr_len={}, handle={:p}",
                dev_cfg.device_address, dev_cfg.scl_speed_hz, (int)dev_cfg.dev_addr_length,
                fmt::ptr(bus_handle_));
  esp_err_t err = i2c_master_bus_add_device(bus_handle_, &dev_cfg, &dev_handle_);
  if (err != ESP_OK) {
    logger_.error("Failed to add I2C device: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  initialized_ = true;
  logger_.info("I2C device initialized at address 0x{:02x}", config_.device_address);
  ec.clear();
  return true;
}

template <typename RegisterType> bool I2cMasterDevice<RegisterType>::deinit(std::error_code &ec) {
  logger_.info("Deinitializing I2C device at address 0x{:02x}", config_.device_address);
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!initialized_) {
    logger_.warn("Device not initialized");
    return true;
  }
  if (dev_handle_) {
    esp_err_t err = i2c_master_bus_rm_device(dev_handle_);
    if (err != ESP_OK) {
      logger_.error("Failed to remove I2C device: {}", esp_err_to_name(err));
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    dev_handle_ = nullptr;
  }
  initialized_ = false;
  logger_.info("I2C device deinitialized at address 0x{:02x}", config_.device_address);
  ec.clear();
  return true;
}

template <typename RegisterType>
void I2cMasterDevice<RegisterType>::set_device_address(uint16_t device_address,
                                                       i2c_addr_bit_len_t addr_bit_len) {
  // This function is not used in the new API, but kept for compatibility
  // with the old API. The device address is set in the I2cMasterDevice constructor.
  // If needed, this can be implemented to update the device address dynamically.
  // For now, we just log the address and bit length.
  logger_.info("Setting device address to 0x{:02x} with bit length {}", device_address,
               static_cast<int>(addr_bit_len));
}

template <typename RegisterType>
bool I2cMasterDevice<RegisterType>::write(const uint8_t *data, size_t len, std::error_code &ec) {
  logger_.debug("Writing {} bytes to device 0x{:02x}", len, config_.device_address);
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!initialized_ || !dev_handle_) {
    logger_.error("Device not initialized or handle is null");
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  esp_err_t err = i2c_master_transmit(dev_handle_, data, len, config_.timeout_ms);
  if (err != ESP_OK) {
    logger_.error("Write failed: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  logger_.info("Write successful to device 0x{:02x}", config_.device_address);
  ec.clear();
  return true;
}

template <typename RegisterType>
bool I2cMasterDevice<RegisterType>::read(uint8_t *data, size_t len, std::error_code &ec) {
  logger_.debug("Reading {} bytes from device 0x{:02x}", len, config_.device_address);
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!initialized_ || !dev_handle_) {
    logger_.error("Device not initialized or handle is null");
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  esp_err_t err = i2c_master_receive(dev_handle_, data, len, config_.timeout_ms);
  if (err != ESP_OK) {
    logger_.error("Read failed: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  logger_.info("Read successful from device 0x{:02x}", config_.device_address);
  logger_.debug("Read data: {} bytes", len);
  ec.clear();
  return true;
}

template <typename RegisterType>
bool I2cMasterDevice<RegisterType>::write_read(const uint8_t *wdata, size_t wlen, uint8_t *rdata,
                                               size_t rlen, std::error_code &ec) {
  logger_.debug("WriteRead: writing {} bytes, reading {} bytes to/from device 0x{:02x}", wlen, rlen,
                config_.device_address);
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!initialized_ || !dev_handle_) {
    logger_.error("Device not initialized or handle is null");
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  esp_err_t err =
      i2c_master_transmit_receive(dev_handle_, wdata, wlen, rdata, rlen, config_.timeout_ms);
  if (err != ESP_OK) {
    logger_.error("WriteRead failed: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  logger_.info("WriteRead successful to device 0x{:02x}", config_.device_address);
  logger_.debug("WriteRead read data: {} bytes", rlen);
  ec.clear();
  return true;
}

template <typename RegisterType>
bool I2cMasterDevice<RegisterType>::write(const std::vector<uint8_t> &data, std::error_code &ec) {
  return write(data.data(), data.size(), ec);
}

template <typename RegisterType>
bool I2cMasterDevice<RegisterType>::read(std::vector<uint8_t> &data, std::error_code &ec) {
  return read(data.data(), data.size(), ec);
}

template <typename RegisterType>
bool I2cMasterDevice<RegisterType>::write_read(const std::vector<uint8_t> &wdata,
                                               std::vector<uint8_t> &rdata, std::error_code &ec) {
  return write_read(wdata.data(), wdata.size(), rdata.data(), rdata.size(), ec);
}

template <typename RegisterType>
bool I2cMasterDevice<RegisterType>::write_register(RegisterType reg, const uint8_t *data,
                                                   size_t len, std::error_code &ec) {
  std::vector<uint8_t> buffer(sizeof(RegisterType) + len);
  std::memcpy(buffer.data(), &reg, sizeof(RegisterType));
  if (len > 0 && data)
    std::memcpy(buffer.data() + sizeof(RegisterType), data, len);
  return write(buffer.data(), buffer.size(), ec);
}

template <typename RegisterType>
bool I2cMasterDevice<RegisterType>::write_register(RegisterType reg,
                                                   const std::vector<uint8_t> &data,
                                                   std::error_code &ec) {
  return write_register(reg, data.data(), data.size(), ec);
}

template <typename RegisterType>
bool I2cMasterDevice<RegisterType>::read_register(RegisterType reg, uint8_t *data, size_t len,
                                                  std::error_code &ec) {
  // Write register, then read data
  return write_read(reinterpret_cast<const uint8_t *>(&reg), sizeof(RegisterType), data, len, ec);
}

template <typename RegisterType>
bool I2cMasterDevice<RegisterType>::read_register(RegisterType reg, std::vector<uint8_t> &data,
                                                  std::error_code &ec) {
  return read_register(reg, data.data(), data.size(), ec);
}

template <typename RegisterType> bool I2cMasterDevice<RegisterType>::probe(std::error_code &ec) {
  logger_.info("Probing for device at address 0x{:02x}", config_.device_address);
  if (!bus_handle_) {
    logger_.error("Bus handle is null");
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  esp_err_t err = i2c_master_probe(bus_handle_, config_.device_address, config_.timeout_ms);
  if (err == ESP_OK) {
    logger_.info("Device 0x{:02x} found", config_.device_address);
    ec.clear();
    return true;
  }
  logger_.warn("Device 0x{:02x} not found: {}", config_.device_address, esp_err_to_name(err));
  ec = std::make_error_code(std::errc::io_error);
  return false;
}

template <typename RegisterType>
bool I2cMasterDevice<RegisterType>::probe(int32_t timeout_ms, std::error_code &ec) {
  // NOTE: This uses the bus_handle_ provided at construction. If the bus is deleted, this will
  // fail.
  if (!bus_handle_) {
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  esp_err_t err = i2c_master_probe(bus_handle_, config_.device_address, timeout_ms);
  if (err == ESP_OK) {
    ec.clear();
    return true;
  }
  ec = std::make_error_code(std::errc::io_error);
  return false;
}

// Explicit instantiations
template class I2cMasterDevice<uint8_t>;
template class I2cMasterDevice<uint16_t>;

} // namespace espp

#endif // CONFIG_ESPP_I2C_USE_NEW_API

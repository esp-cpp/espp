#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

#include "base_component.hpp"

namespace espp {
/// Base class for all peripherals
/// This class provides a common interface for all peripherals
///
/// It provides a way to probe the peripheral, write data to the peripheral,
/// read data from the peripheral, and write then read data from the
/// peripheral.
///
/// The peripheral is protected by a mutex to ensure that only one
/// operation can be performed at a time.
template <std::integral RegisterAddressType = std::uint8_t>
class BasePeripheral : public BaseComponent {
public:
  /// Function to probe the peripheral
  /// \param address The address to probe
  /// \return True if the peripheral is found at the given address
  typedef std::function<bool(uint8_t)> probe_fn;

  /// Function to write data to the peripheral
  /// \param address The address of the peripheral to write to
  /// \param data The data to write
  /// \param length The length of the data to write
  /// \return True if the write was successful
  typedef std::function<bool(uint8_t, const uint8_t *, size_t)> write_fn;

  /// Function to read data from the peripheral
  /// \param address The address of the peripheral to read from
  /// \param data The buffer to read into
  /// \param length The length of the buffer
  /// \return True if the read was successful
  typedef std::function<bool(uint8_t, uint8_t *, size_t)> read_fn;

  /// Function to read data at a specific address from the peripheral
  /// \param address The address of the peripheral to read from
  /// \param reg_addr The address of the register to read from
  /// \param data The buffer to read into
  /// \param length The length of the buffer
  /// \return True if the read was successful
  typedef std::function<bool(uint8_t, RegisterAddressType, uint8_t *, size_t)> read_register_fn;

  /// Function to write then read data from the peripheral
  /// \param address The address of the peripheral to write to
  /// \param write_data The data to write
  /// \param write_length The length of the data to write
  /// \param read_data The buffer to read into
  /// \param read_length The length of the buffer
  /// \return True if the write then read was successful
  typedef std::function<bool(uint8_t, const uint8_t *, size_t, uint8_t *, size_t)>
      write_then_read_fn;

  /// Configuration for the peripheral
  struct Config {
    uint8_t address{0};      ///< The address of the peripheral
    probe_fn probe{nullptr}; ///< Function to probe the peripheral
    write_fn write{nullptr}; ///< Function to write data to the peripheral
    read_fn read{nullptr};   ///< Function to read data from the peripheral
    read_register_fn read_register{
        nullptr}; ///< Function to read data at a specific address from the peripheral
    write_then_read_fn write_then_read{
        nullptr}; ///< Function to write then read data from the peripheral
  };

protected:
  /// Constructor
  /// \param config The configuration for the peripheral
  /// \param name The name of the peripheral
  /// \param verbosity The verbosity level for the peripheral
  BasePeripheral(const Config &config, std::string_view name,
                 espp::Logger::Verbosity verbosity = espp::Logger::Verbosity::WARN)
      : BaseComponent(name, verbosity)
      , base_config_(config) {}

  /// Get the configuration for the peripheral
  /// \return The configuration for the peripheral
  const Config &config() const { return base_config_; }

  /// Write data to the peripheral
  /// \param data The data to write
  /// \param length The length of the data to write
  /// \param ec The error code to set if there is an error
  void write_many(const uint8_t *data, size_t length, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (base_config_.write) {
      if (!base_config_.write(base_config_.address, data, length)) {
        ec = std::make_error_code(std::errc::io_error);
      } else {
        ec.clear();
      }
    } else {
      ec = std::make_error_code(std::errc::operation_not_supported);
    }
  }

  /// Write a uint8_t to the peripheral
  /// \param data The data to write
  /// \param ec The error code to set if there is an error
  void write_u8(uint8_t data, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (base_config_.write) {
      if (!base_config_.write(base_config_.address, &data, 1)) {
        ec = std::make_error_code(std::errc::io_error);
      } else {
        ec.clear();
      }
    } else {
      ec = std::make_error_code(std::errc::operation_not_supported);
    }
  }

  /// Write a uint16_t to the peripheral
  /// \param data The data to write
  /// \param ec The error code to set if there is an error
  void write_u16(uint16_t data, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (base_config_.write) {
      uint8_t buffer[2];
      buffer[0] = (data >> 8) & 0xff;
      buffer[1] = data & 0xff;
      if (!base_config_.write(base_config_.address, buffer, 2)) {
        ec = std::make_error_code(std::errc::io_error);
      } else {
        ec.clear();
      }
    } else {
      ec = std::make_error_code(std::errc::operation_not_supported);
    }
  }

  /// Read data from the peripheral
  /// \param data The buffer to read into
  /// \param length The length of the buffer
  /// \param ec The error code to set if there is an error
  void read_many(uint8_t *data, size_t length, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (base_config_.read) {
      if (!base_config_.read(base_config_.address, data, length)) {
        ec = std::make_error_code(std::errc::io_error);
      } else {
        ec.clear();
      }
    } else {
      ec = std::make_error_code(std::errc::operation_not_supported);
    }
  }

  /// Read a uint8_t from the peripheral
  /// \param ec The error code to set if there is an error
  /// \return The data read from the peripheral
  uint8_t read_u8(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data = 0;
    if (base_config_.read) {
      if (!base_config_.read(base_config_.address, &data, 1)) {
        ec = std::make_error_code(std::errc::io_error);
        return 0;
      } else {
        ec.clear();
      }
    } else {
      ec = std::make_error_code(std::errc::operation_not_supported);
    }
    return data;
  }

  /// Read a uint16_t from the peripheral
  /// \param ec The error code to set if there is an error
  /// \return The data read from the peripheral
  uint16_t read_u16(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint16_t data = 0;
    if (base_config_.read) {
      uint8_t buffer[2];
      if (!base_config_.read(base_config_.address, buffer, 2)) {
        ec = std::make_error_code(std::errc::io_error);
        return 0;
      } else {
        data = (buffer[0] << 8) | buffer[1];
        ec.clear();
      }
    } else {
      ec = std::make_error_code(std::errc::operation_not_supported);
    }
    return data;
  }

  /// Write then read data from the peripheral
  /// \param write_data The data to write
  /// \param write_length The length of the data to write
  /// \param read_data The buffer to read into
  /// \param read_length The length of the buffer
  /// \param ec The error code to set if there is an error
  void write_then_read(const uint8_t *write_data, size_t write_length, uint8_t *read_data,
                       size_t read_length, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (base_config_.write_then_read) {
      if (!base_config_.write_then_read(base_config_.address, write_data, write_length, read_data,
                                        read_length)) {
        ec = std::make_error_code(std::errc::io_error);
      } else {
        ec.clear();
      }
    } else if (base_config_.write && base_config_.read) {
      // write the data
      if (!base_config_.write(base_config_.address, write_data, write_length)) {
        ec = std::make_error_code(std::errc::io_error);
      } else {
        // read the data
        if (!base_config_.read(base_config_.address, read_data, read_length)) {
          ec = std::make_error_code(std::errc::io_error);
        } else {
          ec.clear();
        }
      }
    } else {
      ec = std::make_error_code(std::errc::operation_not_supported);
    }
  }

  /// Write a uint8_t to the peripheral
  /// \param reg_addr The address of the register to write to
  /// \param data The data to write
  /// \param ec The error code to set if there is an error
  void write_u8_to_register(RegisterAddressType reg_addr, uint8_t data, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (base_config_.write) {
      // use the size of the register address to determine how many bytes the
      // register address is
      static constexpr size_t reg_addr_size = sizeof(RegisterAddressType);
      uint8_t buffer[reg_addr_size + 1];
      for (size_t i = 0; i < reg_addr_size; i++) {
        buffer[i] = (reg_addr >> (i * 8)) & 0xff;
      }
      buffer[reg_addr_size] = data;
      if (!base_config_.write(base_config_.address, buffer, reg_addr_size + 1)) {
        ec = std::make_error_code(std::errc::io_error);
      } else {
        ec.clear();
      }
    } else {
      ec = std::make_error_code(std::errc::operation_not_supported);
    }
  }

  /// Write a uint16_t to the peripheral
  /// \param reg_addr The address of the register to write to
  /// \param data The data to write
  /// \param ec The error code to set if there is an error
  void write_u16_to_register(RegisterAddressType reg_addr, uint16_t data, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (base_config_.write) {
      // use the size of the register address to determine how many bytes the
      // register address is
      static constexpr size_t reg_addr_size = sizeof(RegisterAddressType);
      uint8_t buffer[reg_addr_size + 2];
      for (size_t i = 0; i < reg_addr_size; i++) {
        buffer[i] = (reg_addr >> (i * 8)) & 0xff;
      }
      buffer[reg_addr_size] = (data >> 8) & 0xff;
      buffer[reg_addr_size + 1] = data & 0xff;
      if (!base_config_.write(base_config_.address, buffer, reg_addr_size + 2)) {
        ec = std::make_error_code(std::errc::io_error);
      } else {
        ec.clear();
      }
    } else {
      ec = std::make_error_code(std::errc::operation_not_supported);
    }
  }

  /// Write many bytes to a register on the peripheral
  /// \param reg_addr The address of the register to write to
  /// \param data The data to write
  /// \param length The length of the data to write
  /// \param ec The error code to set if there is an error
  void write_many_to_register(RegisterAddressType reg_addr, const uint8_t *data, size_t length,
                              std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (base_config_.write) {
      // use the size of the register address to determine how many bytes the
      // register address is
      static constexpr size_t reg_addr_size = sizeof(RegisterAddressType);
      uint8_t buffer[length + reg_addr_size];
      for (size_t i = 0; i < reg_addr_size; i++) {
        buffer[i] = (reg_addr >> (i * 8)) & 0xff;
      }
      std::copy(data, data + length, buffer + reg_addr_size);
      if (!base_config_.write(base_config_.address, buffer, length + reg_addr_size)) {
        ec = std::make_error_code(std::errc::io_error);
      } else {
        ec.clear();
      }
    } else {
      ec = std::make_error_code(std::errc::operation_not_supported);
    }
  }

  /// Read a uint8_t from a register on the peripheral
  /// \param register_address The address of the register to read from
  /// \param ec The error code to set if there is an error
  /// \return The data read from the peripheral
  uint8_t read_u8_from_register(uint8_t register_address, std::error_code &ec) {
    uint8_t data = 0;
    if (base_config_.read_register) {
      if (!base_config_.read_register(base_config_.address, register_address, &data, 1)) {
        ec = std::make_error_code(std::errc::io_error);
        return 0;
      } else {
        ec.clear();
      }
    } else {
      write_then_read(&register_address, 1, &data, 1, ec);
      if (ec) {
        return 0;
      }
    }
    return data;
  }

  /// Read a uint16_t from a register on the peripheral
  /// \param register_address The address of the register to read from
  /// \param ec The error code to set if there is an error
  /// \return The data read from the peripheral
  uint16_t read_u16_from_register(uint8_t register_address, std::error_code &ec) {
    uint8_t data[2];
    if (base_config_.read_register) {
      if (!base_config_.read_register(base_config_.address, register_address, data, 2)) {
        ec = std::make_error_code(std::errc::io_error);
        return 0;
      } else {
        ec.clear();
      }
    } else {
      write_then_read(&register_address, 1, (uint8_t *)&data, 2, ec);
      if (ec) {
        return 0;
      }
    }
    return (data[0] << 8) | data[1];
  }

  /// Read many bytes from a register on the peripheral
  /// \param register_address The address of the register to read from
  /// \param data The buffer to read into
  /// \param length The length of the buffer
  /// \param ec The error code to set if there is an error
  void read_many_from_register(uint8_t register_address, uint8_t *data, size_t length,
                               std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (base_config_.read_register) {
      if (!base_config_.read_register(base_config_.address, register_address, data, length)) {
        ec = std::make_error_code(std::errc::io_error);
      } else {
        ec.clear();
      }
    } else {
      write_then_read(&register_address, 1, data, length, ec);
    }
  }

  /// Set bits in a register on the peripheral
  /// \param register_address The address of the register to modify
  /// \param mask The mask to set
  /// \param ec The error code to set if there is an error
  /// \note This function reads the register, sets the bits, and then writes the register
  ///       back to the peripheral
  void set_bits_in_register(uint8_t register_address, uint8_t mask, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data = read_u8_from_register(register_address, ec);
    if (ec) {
      return;
    }
    data |= mask;
    write_u8_to_register(register_address, data, ec);
  }

  /// Clear bits in a register on the peripheral
  /// \param register_address The address of the register to modify
  /// \param mask The mask to clear
  /// \param ec The error code to set if there is an error
  /// \note This function reads the register, clears the bits, and then writes the register
  ///       back to the peripheral
  void clear_bits_in_register(uint8_t register_address, uint8_t mask, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data = read_u8_from_register(register_address, ec);
    if (ec) {
      return;
    }
    data &= ~mask;
    write_u8_to_register(register_address, data, ec);
  }

  Config base_config_;              ///< The configuration for the peripheral
  std::recursive_mutex base_mutex_; ///< The mutex to protect access to the peripheral
};
} // namespace espp

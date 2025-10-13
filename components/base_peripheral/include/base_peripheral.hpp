#pragma once

#include <bitset>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

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
///
/// @tparam RegisterAddressType The type of the register address. This is usually
///                            uint8_t or uint16_t.
///                            Default is uint8_t.
/// @tparam UseAddress Whether the peripheral uses an address. If true, the
///                    peripheral will use the address provided in the config
///                    for all operations. If false, the peripheral will not use
///                    an address.
///                    Default is true.
/// @tparam BigEndianRegisterAddress Whether the register address is in big
///                                 endian format. If true, the register address
///                                 will be converted to big endian format before
///                                 being sent to the peripheral.
///                                 Default is true. This only applies to
///                                 register addresses larger than u8.
/// @tparam BigEndianData Whether the data is in big endian format. If true,
///                        the data will be converted to big endian format before
///                        being sent to the peripheral, and will be converted from
///                        big endian format after being read from the peripheral.
///                        Default is true (big-endian), which matches the previously hardcoded
///                        behavior. This only applies to u16 functions.
template <std::integral RegisterAddressType = std::uint8_t, bool UseAddress = true,
          bool BigEndianRegisterAddress = true, bool BigEndianData = true>
class BasePeripheral : public BaseComponent {
public:
  /// Function to probe the peripheral
  /// \param address The address to probe
  /// \return True if the peripheral is found at the given address
  typedef std::function<bool(uint8_t)> probe_fn;

  // Functions for writing/reading data to/from the peripheral that take an address
  // as the first parameter
  typedef std::function<bool(uint8_t, const uint8_t *, size_t)> write_to_address_fn;
  typedef std::function<bool(uint8_t, uint8_t *, size_t)> read_from_address_fn;
  typedef std::function<bool(uint8_t, RegisterAddressType, uint8_t *, size_t)>
      read_register_from_address_fn;
  typedef std::function<bool(uint8_t, const uint8_t *, size_t, uint8_t *, size_t)>
      write_then_read_from_address_fn;

  // Functions for writing/reading data to/from the peripheral that do not take an address
  typedef std::function<bool(const uint8_t *, size_t)> write_no_address_fn;
  typedef std::function<bool(uint8_t *, size_t)> read_no_address_fn;
  typedef std::function<bool(RegisterAddressType, uint8_t *, size_t)> read_register_no_address_fn;
  typedef std::function<bool(const uint8_t *, size_t, uint8_t *, size_t)>
      write_then_read_no_address_fn;

  // Simplify the function types based on whether the peripheral uses an address
  using write_fn = std::conditional_t<UseAddress, write_to_address_fn, write_no_address_fn>;
  using read_fn = std::conditional_t<UseAddress, read_from_address_fn, read_no_address_fn>;
  using read_register_fn =
      std::conditional_t<UseAddress, read_register_from_address_fn, read_register_no_address_fn>;
  using write_then_read_fn = std::conditional_t<UseAddress, write_then_read_from_address_fn,
                                                write_then_read_no_address_fn>;

  /// Configuration for the peripheral
  struct Config {
    uint8_t address{
        0}; ///< The address of the peripheral. Note that this is only used if UseAddress is true
    probe_fn probe{nullptr}; ///< Function to probe the peripheral. Note that this is only used if
                             ///< UseAddress is true
    write_fn write{nullptr}; ///< Function to write data to the peripheral
    read_fn read{nullptr};   ///< Function to read data from the peripheral
    read_register_fn read_register{
        nullptr}; ///< Function to read data at a specific address from the peripheral
    write_then_read_fn write_then_read{
        nullptr}; ///< Function to write then read data from the peripheral
    std::chrono::milliseconds separate_write_then_read_delay{
        0}; ///< The delay between the write and read operations in write_then_read in
            ///< milliseconds if the write_then_read function is not set to a custom function
            ///< and the write and read functions are separate
  };

  /// Probe the peripheral
  /// \param ec The error code to set if there is an error
  /// \return True if the peripheral is found
  /// \note This function is thread safe
  /// \note If the probe function is not set, this function will return false
  ///      and set the error code to operation_not_supported
  /// \note This function is only available if UseAddress is true
  bool probe(std::error_code &ec) const requires(UseAddress) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (base_config_.probe) {
      return base_config_.probe(base_config_.address);
    } else {
      ec = std::make_error_code(std::errc::operation_not_supported);
    }
    return false;
  }

  /// Set the address of the peripheral
  /// \param address The address of the peripheral
  /// \note This function is thread safe
  /// \note This function is only available if UseAddress is true
  void set_address(uint8_t address) requires(UseAddress) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    base_config_.address = address;
  }

  /// Set the probe function
  /// \param probe The probe function
  /// \note This function is thread safe
  /// \note This should rarely be used, as the probe function is usually set in
  ///      the constructor. If you need to change the probe function, consider
  ///      using the set_config function instead.
  /// \note This function is only available if UseAddress is true
  void set_probe(const probe_fn &probe) requires(UseAddress) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    base_config_.probe = probe;
  }

  /// Set the write function
  /// \param write The write function
  /// \note This function is thread safe
  /// \note This should rarely be used, as the write function is usually set in
  ///       the constructor. If you need to change the write function, consider
  ///       using the set_config function instead.
  void set_write(const write_fn &write) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    base_config_.write = write;
  }

  /// Set the read function
  /// \param read The read function
  /// \note This function is thread safe
  /// \note This should rarely be used, as the read function is usually set in
  ///      the constructor. If you need to change the read function, consider
  ///      using the set_config function instead.
  void set_read(const read_fn &read) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    base_config_.read = read;
  }

  /// Set the read register function
  /// \param read_register The read register function
  /// \note This function is thread safe
  /// \note This should rarely be used, as the read register function is usually
  ///      set in the constructor. If you need to change the read register
  ///      function, consider using the set_config function instead.
  void set_read_register(const read_register_fn &read_register) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    base_config_.read_register = read_register;
  }

  /// Set the write then read function
  /// \param write_then_read The write then read function
  /// \note This function is thread safe
  /// \note This should rarely be used, as the write then read function is
  ///      usually set in the constructor. If you need to change the write then
  void set_write_then_read(const write_then_read_fn &write_then_read) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    base_config_.write_then_read = write_then_read;
  }

  /// Set the delay between the write and read operations in write_then_read
  /// \param delay The delay between the write and read operations in
  ///             write_then_read
  /// \note This function is thread safe
  /// \note This should rarely be used, as the delay is usually set in the
  ///      constructor. If you need to change the delay, consider using the
  ///      set_config function instead.
  /// \note This delay is only used if the write_then_read function is not set to
  ///      a custom function and the write and read functions are separate
  ///      functions.
  void set_separate_write_then_read_delay(const std::chrono::milliseconds &delay) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    base_config_.separate_write_then_read_delay = delay;
  }

  /// Set the configuration for the peripheral
  /// \param config The configuration for the peripheral
  /// \note This function is thread safe
  /// \note The configuration should normally be set in the constructor, but
  ///       this function can be used to change the configuration after the
  ///       peripheral has been created - for instance if the peripheral could
  ///       be found on different communications buses.
  void set_config(const Config &config) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    base_config_ = config;
  }

  /// Set the configuration for the peripheral
  /// \param config The configuration for the peripheral
  /// \note This function is thread safe
  /// \note The configuration should normally be set in the constructor, but
  ///       this function can be used to change the configuration after the
  ///       peripheral has been created - for instance if the peripheral could
  ///       be found on different communications buses.
  void set_config(Config &&config) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    base_config_ = std::move(config);
  }

  /// Get the configuration for the peripheral
  /// \return The configuration for the peripheral
  const Config &config() const { return base_config_; }

  /// Get the address of the peripheral
  /// \return The address of the peripheral
  uint8_t address() const { return base_config_.address; }

protected:
  /// Constructor
  /// \param config The configuration for the peripheral
  /// \param name The name of the peripheral
  /// \param verbosity The verbosity level for the peripheral
  BasePeripheral(const Config &config, std::string_view name,
                 espp::Logger::Verbosity verbosity = espp::Logger::Verbosity::WARN)
      : BaseComponent(name, verbosity)
      , base_config_(config) {}

  /// Write data to the peripheral
  /// \param data The data to write
  /// \param length The length of the data to write
  /// \param ec The error code to set if there is an error
  void write(const uint8_t *data, size_t length, std::error_code &ec) requires(UseAddress) {
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

  /// Write data to the peripheral
  /// \param data The data to write
  /// \param length The length of the data to write
  /// \param ec The error code to set if there is an error
  void write(const uint8_t *data, size_t length, std::error_code &ec) requires(!UseAddress) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (base_config_.write) {
      if (!base_config_.write(data, length)) {
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
  void read(uint8_t *data, size_t length, std::error_code &ec) const requires(UseAddress) {
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

  /// Read data from the peripheral
  /// \param data The buffer to read into
  /// \param length The length of the buffer
  /// \param ec The error code to set if there is an error
  void read(uint8_t *data, size_t length, std::error_code &ec) const requires(!UseAddress) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (base_config_.read) {
      if (!base_config_.read(data, length)) {
        ec = std::make_error_code(std::errc::io_error);
      } else {
        ec.clear();
      }
    } else {
      ec = std::make_error_code(std::errc::operation_not_supported);
    }
  }

  /// Read data at a specific address from the peripheral
  /// \param reg_addr The address of the register to read from
  /// \param data The buffer to read into
  /// \param length The length of the buffer
  /// \param ec The error code to set if there is an error
  void read_register(RegisterAddressType reg_addr, uint8_t *data, size_t length,
                     std::error_code &ec) const requires(UseAddress) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (base_config_.read_register) {
      if (!base_config_.read_register(base_config_.address, reg_addr, data, length)) {
        ec = std::make_error_code(std::errc::io_error);
      } else {
        ec.clear();
      }
    } else {
      // use the size of the register address to determine how many bytes the
      // register address is
      static constexpr size_t reg_addr_size = sizeof(RegisterAddressType);
      uint8_t buffer[reg_addr_size];
      put_register_bytes(reg_addr, buffer);

      // now we need to determine if we can write_then_read, or if we need to
      // write then read separately
      if (base_config_.write_then_read) {
        // we can use write_then_read
        if (!base_config_.write_then_read(base_config_.address, buffer, reg_addr_size, data,
                                          length)) {
          ec = std::make_error_code(std::errc::io_error);
        } else {
          ec.clear();
        }
      } else if (base_config_.write && base_config_.read) {
        if (!base_config_.write(base_config_.address, buffer, reg_addr_size)) {
          ec = std::make_error_code(std::errc::io_error);
          return;
        }
        if (base_config_.separate_write_then_read_delay.count() > 0) {
          std::this_thread::sleep_for(base_config_.separate_write_then_read_delay);
        }
        if (!base_config_.read(base_config_.address, data, length)) {
          ec = std::make_error_code(std::errc::io_error);
          return;
        }
      } else {
        // we can't do anything
        ec = std::make_error_code(std::errc::operation_not_supported);
        return;
      }
    }
  }

  /// Read data at a specific address from the peripheral
  /// \param reg_addr The address of the register to read from
  /// \param data The buffer to read into
  /// \param length The length of the buffer
  /// \param ec The error code to set if there is an error
  void read_register(RegisterAddressType reg_addr, uint8_t *data, size_t length,
                     std::error_code &ec) const requires(!UseAddress) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (base_config_.read_register) {
      if (!base_config_.read_register(reg_addr, data, length)) {
        ec = std::make_error_code(std::errc::io_error);
      } else {
        ec.clear();
      }
    } else {
      // use the size of the register address to determine how many bytes the
      // register address is
      static constexpr size_t reg_addr_size = sizeof(RegisterAddressType);
      uint8_t buffer[reg_addr_size];
      put_register_bytes(reg_addr, buffer);

      // now we need to determine if we can write_then_read, or if we need to
      // write then read separately
      if (base_config_.write_then_read) {
        // we can use write_then_read
        if (!base_config_.write_then_read(buffer, reg_addr_size, data, length)) {
          ec = std::make_error_code(std::errc::io_error);
        } else {
          ec.clear();
        }
      } else if (base_config_.write && base_config_.read) {
        if (!base_config_.write(buffer, reg_addr_size)) {
          ec = std::make_error_code(std::errc::io_error);
          return;
        }
        if (base_config_.separate_write_then_read_delay.count() > 0) {
          std::this_thread::sleep_for(base_config_.separate_write_then_read_delay);
        }
        if (!base_config_.read(data, length)) {
          ec = std::make_error_code(std::errc::io_error);
          return;
        }
      } else {
        // we can't do anything
        ec = std::make_error_code(std::errc::operation_not_supported);
        return;
      }
    }
  }

  /// Write then read data from the peripheral
  /// \param write_data The data to write
  /// \param write_length The length of the data to write
  /// \param read_data The buffer to read into
  /// \param read_length The length of the buffer
  /// \param ec The error code to set if there is an error
  void write_then_read(const uint8_t *write_data, size_t write_length, uint8_t *read_data,
                       size_t read_length, std::error_code &ec) requires(UseAddress) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (base_config_.write_then_read) {
      logger_.debug("write_then_read write: {}, read: {} bytes", write_length, read_length);
      if (!base_config_.write_then_read(base_config_.address, write_data, write_length, read_data,
                                        read_length)) {
        ec = std::make_error_code(std::errc::io_error);
      } else {
        ec.clear();
      }
    } else if (base_config_.write && base_config_.read) {
      logger_.debug("write {} bytes then read {} bytes", write_length, read_length);
      write(write_data, write_length, ec);
      if (ec) {
        return;
      }
      if (base_config_.separate_write_then_read_delay.count() > 0) {
        std::this_thread::sleep_for(base_config_.separate_write_then_read_delay);
      }
      read(read_data, read_length, ec);
    } else {
      ec = std::make_error_code(std::errc::operation_not_supported);
    }
  }

  /// Write then read data from the peripheral
  /// \param write_data The data to write
  /// \param write_length The length of the data to write
  /// \param read_data The buffer to read into
  /// \param read_length The length of the buffer
  /// \param ec The error code to set if there is an error
  void write_then_read(const uint8_t *write_data, size_t write_length, uint8_t *read_data,
                       size_t read_length, std::error_code &ec) requires(!UseAddress) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (base_config_.write_then_read) {
      logger_.debug("write_then_read write: {}, read: {} bytes", write_length, read_length);
      if (!base_config_.write_then_read(write_data, write_length, read_data, read_length)) {
        ec = std::make_error_code(std::errc::io_error);
      } else {
        ec.clear();
      }
    } else if (base_config_.write && base_config_.read) {
      logger_.debug("write {} bytes then read {} bytes", write_length, read_length);
      write(write_data, write_length, ec);
      if (ec) {
        return;
      }
      if (base_config_.separate_write_then_read_delay.count() > 0) {
        std::this_thread::sleep_for(base_config_.separate_write_then_read_delay);
      }
      read(read_data, read_length, ec);
    } else {
      ec = std::make_error_code(std::errc::operation_not_supported);
    }
  }

  /// Write data to the peripheral
  /// \param data The data to write
  /// \param length The length of the data to write
  /// \param ec The error code to set if there is an error
  void write_many(const uint8_t *data, size_t length, std::error_code &ec) {
    logger_.debug("write {} bytes", length);
    write(data, length, ec);
  }

  /// Write data to the peripheral
  /// \param data The data to write
  /// \param ec The error code to set if there is an error
  void write_many(const std::vector<uint8_t> &data, std::error_code &ec) {
    logger_.debug("write {} bytes", data.size());
    write(data.data(), data.size(), ec);
  }

  /// Write a uint8_t to the peripheral
  /// \param data The data to write
  /// \param ec The error code to set if there is an error
  void write_u8(uint8_t data, std::error_code &ec) {
    logger_.debug("write u8");
    write(&data, 1, ec);
  }

  /// Write a uint16_t to the peripheral
  /// \param data The data to write
  /// \param ec The error code to set if there is an error
  void write_u16(uint16_t data, std::error_code &ec) {
    logger_.debug("write u16");
    uint8_t buffer[2];
    if constexpr (BigEndianData) {
      buffer[0] = (data >> 8) & 0xff;
      buffer[1] = data & 0xff;
    } else {
      buffer[0] = data & 0xff;
      buffer[1] = (data >> 8) & 0xff;
    }
    write(buffer, 2, ec);
  }

  /// Read data from the peripheral
  /// \param data The buffer to read into
  /// \param length The length of the buffer
  /// \param ec The error code to set if there is an error
  void read_many(uint8_t *data, size_t length, std::error_code &ec) const {
    logger_.debug("read {} bytes", length);
    read(data, length, ec);
  }

  /// Read many bytes from the peripheral
  /// \param data The buffer to read into
  /// \param ec The error code to set if there is an error
  /// \note You should ensure that the buffer is the correct size before calling
  ///       this function since the buffer size is what is used to know how many
  ///       bytes to read
  void read_many(std::vector<uint8_t> &data, std::error_code &ec) const {
    logger_.debug("read {} bytes", data.size());
    read(data.data(), data.size(), ec);
  }

  /// Read a uint8_t from the peripheral
  /// \param ec The error code to set if there is an error
  /// \return The data read from the peripheral
  uint8_t read_u8(std::error_code &ec) const {
    logger_.debug("read u8");
    uint8_t data = 0;
    read(&data, 1, ec);
    if (ec) {
      return 0;
    }
    return data;
  }

  /// Read a uint16_t from the peripheral
  /// \param ec The error code to set if there is an error
  /// \return The data read from the peripheral
  uint16_t read_u16(std::error_code &ec) const {
    logger_.debug("read u16");
    uint16_t data = 0;
    uint8_t buffer[2];
    read(buffer, 2, ec);
    if (!ec) {
      if constexpr (BigEndianData) {
        data = (buffer[0] << 8) | buffer[1];
      } else {
        data = (buffer[1] << 8) | buffer[0];
      }
    }
    return data;
  }

  /// Write a uint8_t to the peripheral
  /// \param reg_addr The address of the register to write to
  /// \param data The data to write
  /// \param ec The error code to set if there is an error
  void write_u8_to_register(RegisterAddressType reg_addr, uint8_t data, std::error_code &ec) {
    logger_.debug("write u8 to register 0x{:x}", reg_addr);
    // use the size of the register address to determine how many bytes the
    // register address is
    static constexpr size_t reg_addr_size = sizeof(RegisterAddressType);
    uint8_t buffer[reg_addr_size + 1];
    put_register_bytes(reg_addr, buffer);
    buffer[reg_addr_size] = data;
    write(buffer, reg_addr_size + 1, ec);
  }

  /// Write a uint16_t to the peripheral
  /// \param reg_addr The address of the register to write to
  /// \param data The data to write
  /// \param ec The error code to set if there is an error
  void write_u16_to_register(RegisterAddressType reg_addr, uint16_t data, std::error_code &ec) {
    logger_.debug("write u16 to register 0x{:x}", reg_addr);
    // use the size of the register address to determine how many bytes the
    // register address is
    static constexpr size_t reg_addr_size = sizeof(RegisterAddressType);
    uint8_t buffer[reg_addr_size + 2];
    put_register_bytes(reg_addr, buffer);
    if constexpr (BigEndianData) {
      buffer[reg_addr_size] = (data >> 8) & 0xff;
      buffer[reg_addr_size + 1] = data & 0xff;
    } else {
      buffer[reg_addr_size] = data & 0xff;
      buffer[reg_addr_size + 1] = (data >> 8) & 0xff;
    }
    write(buffer, reg_addr_size + 2, ec);
  }

  /// Write many bytes to a register on the peripheral
  /// \param reg_addr The address of the register to write to
  /// \param data The data to write
  /// \param length The length of the data to write
  /// \param ec The error code to set if there is an error
  void write_many_to_register(RegisterAddressType reg_addr, const uint8_t *data, size_t length,
                              std::error_code &ec) {
    logger_.debug("write {} bytes to register 0x{:x}", length, reg_addr);
    // use the size of the register address to determine how many bytes the
    // register address is
    static constexpr size_t reg_addr_size = sizeof(RegisterAddressType);
    uint8_t buffer[length + reg_addr_size];
    put_register_bytes(reg_addr, buffer);
    std::copy(data, data + length, buffer + reg_addr_size);
    write(buffer, length + reg_addr_size, ec);
  }

  /// Write many bytes to a register on the peripheral
  /// \param reg_addr The address of the register to write to
  /// \param data The data to write
  /// \param ec The error code to set if there is an error
  void write_many_to_register(RegisterAddressType reg_addr, const std::vector<uint8_t> &data,
                              std::error_code &ec) {
    logger_.debug("write {} bytes to register 0x{:x}", data.size(), reg_addr);
    write_many_to_register(reg_addr, data.data(), data.size(), ec);
  }

  /// Read a uint8_t from a register on the peripheral
  /// \param register_address The address of the register to read from
  /// \param ec The error code to set if there is an error
  /// \return The data read from the peripheral
  uint8_t read_u8_from_register(RegisterAddressType register_address, std::error_code &ec) const {
    logger_.debug("read u8 from register 0x{:x}", register_address);
    uint8_t data = 0;
    read_register(register_address, &data, 1, ec);
    if (ec) {
      return 0;
    }
    return data;
  }

  /// Read a uint16_t from a register on the peripheral
  /// \param register_address The address of the register to read from
  /// \param ec The error code to set if there is an error
  /// \return The data read from the peripheral
  uint16_t read_u16_from_register(RegisterAddressType register_address, std::error_code &ec) const {
    logger_.debug("read u16 from register 0x{:x}", register_address);
    uint8_t data[2];
    read_register(register_address, data, 2, ec);
    if (ec) {
      return 0;
    }
    if constexpr (BigEndianData) {
      return (data[0] << 8) | data[1];
    } else {
      return (data[1] << 8) | data[0];
    }
  }

  /// Read many bytes from a register on the peripheral
  /// \param register_address The address of the register to read from
  /// \param data The buffer to read into
  /// \param length The length of the buffer
  /// \param ec The error code to set if there is an error
  void read_many_from_register(RegisterAddressType register_address, uint8_t *data, size_t length,
                               std::error_code &ec) const {
    logger_.debug("read_many_from_register {} bytes from register 0x{:x}", length,
                  register_address);
    read_register(register_address, data, length, ec);
  }

  /// Read many bytes from a register on the peripheral
  /// \param register_address The address of the register to read from
  /// \param data The buffer to read into
  /// \param ec The error code to set if there is an error
  void read_many_from_register(RegisterAddressType register_address, std::vector<uint8_t> &data,
                               std::error_code &ec) const {
    logger_.debug("read_many_from_register {} bytes from register 0x{:x}", data.size(),
                  register_address);
    read_many_from_register(register_address, data.data(), data.size(), ec);
  }

  /// Set bits in a register on the peripheral
  /// \param register_address The address of the register to modify
  /// \param mask The mask to set
  /// \param ec The error code to set if there is an error
  /// \note This function reads the register, sets the bits, and then writes the register
  ///       back to the peripheral
  void set_bits_in_register(RegisterAddressType register_address, uint8_t mask,
                            std::error_code &ec) {
    logger_.debug("set_bits_in_register 0x{:x} with mask 0x{:x}", register_address, mask);
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data = read_u8_from_register(register_address, ec);
    if (ec) {
      return;
    }
    data |= mask;
    write_u8_to_register(register_address, data, ec);
  }

  /// Set bits in a register on the peripheral
  /// \param register_address The address of the register to modify
  /// \param bits The bits to set
  /// \param ec The error code to set if there is an error
  /// \note This function reads the register, sets the bits, and then writes the
  ///       register back to the peripheral
  void set_bits_in_register(RegisterAddressType register_address, std::bitset<8> bits,
                            std::error_code &ec) {
    logger_.debug("set_bits_in_register 0x{:x} with bits {}", register_address, bits);
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data = read_u8_from_register(register_address, ec);
    if (ec) {
      return;
    }
    data |= bits.to_ulong();
    write_u8_to_register(register_address, data, ec);
  }

  /// Set bits in a register on the peripheral by mask
  /// \param register_address The address of the register to modify
  /// \param mask The mask to use when setting the bits. All bits not in the
  ///             mask will be unmodified, while all bits within the mask will
  ///             be set to the value of the corresponding bit in the value
  /// \param value The value to set. Bits in the value should correspond to the
  ///             bits in the mask
  /// \param ec The error code to set if there is an error
  void set_bits_in_register_by_mask(RegisterAddressType register_address, uint8_t mask,
                                    uint8_t value, std::error_code &ec) {
    logger_.debug("set_bits_in_register_by_mask 0x{:x} with mask 0x{:x} and value 0x{:x}",
                  register_address, mask, value);
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data = read_u8_from_register(register_address, ec);
    if (ec) {
      return;
    }
    data &= ~mask;
    data |= value & mask;
    write_u8_to_register(register_address, data, ec);
  }

  /// Clear bits in a register on the peripheral
  /// \param register_address The address of the register to modify
  /// \param mask The mask to clear
  /// \param ec The error code to set if there is an error
  /// \note This function reads the register, clears the bits, and then writes the register
  ///       back to the peripheral
  void clear_bits_in_register(RegisterAddressType register_address, uint8_t mask,
                              std::error_code &ec) {
    logger_.debug("clear_bits_in_register 0x{:x} with mask 0x{:x}", register_address, mask);
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data = read_u8_from_register(register_address, ec);
    if (ec) {
      return;
    }
    data &= ~mask;
    write_u8_to_register(register_address, data, ec);
  }

  /// Clear bits in a register on the peripheral
  /// \param register_address The address of the register to modify
  /// \param bits The bits to clear
  /// \param ec The error code to set if there is an error
  /// \note This function reads the register, clears the bits, and then writes the
  ///       register back to the peripheral
  void clear_bits_in_register(RegisterAddressType register_address, std::bitset<8> bits,
                              std::error_code &ec) {
    logger_.debug("clear_bits_in_register 0x{:x} with bits {}", register_address, bits);
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data = read_u8_from_register(register_address, ec);
    if (ec) {
      return;
    }
    data &= ~bits.to_ulong();
    write_u8_to_register(register_address, data, ec);
  }

  /// Toggle bits in a register on the peripheral
  /// \param register_address The address of the register to modify
  /// \param mask The mask to toggle
  /// \param ec The error code to set if there is an error
  /// \note This function reads the register, toggles the bits, and then writes the register
  ///       back to the peripheral
  void toggle_bits_in_register(RegisterAddressType register_address, uint8_t mask,
                               std::error_code &ec) {
    logger_.debug("toggle_bits_in_register 0x{:x} with mask 0x{:x}", register_address, mask);
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data = read_u8_from_register(register_address, ec);
    if (ec) {
      return;
    }
    data ^= mask;
    write_u8_to_register(register_address, data, ec);
  }

  /// Toggle bits in a register on the peripheral
  /// \param register_address The address of the register to modify
  /// \param bits The bits to toggle
  /// \param ec The error code to set if there is an error
  /// \note This function reads the register, toggles the bits, and then writes the
  ///       register back to the peripheral
  void toggle_bits_in_register(RegisterAddressType register_address, std::bitset<8> bits,
                               std::error_code &ec) {
    logger_.debug("toggle_bits_in_register 0x{:x} with bits {}", register_address, bits);
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data = read_u8_from_register(register_address, ec);
    if (ec) {
      return;
    }
    data ^= bits.to_ulong();
    write_u8_to_register(register_address, data, ec);
  }

  // Set the bytes of the register address in the buffer
  void put_register_bytes(RegisterAddressType register_address, uint8_t *data) const {
    if constexpr (std::is_same_v<RegisterAddressType, uint8_t>) {
      data[0] = register_address;
    } else {
      // use the size of the register address to determine how many bytes the
      // register address is
      static constexpr size_t reg_addr_size = sizeof(RegisterAddressType);
      if constexpr (BigEndianRegisterAddress) {
        // encode the address as big endian (MSB first)
        for (size_t i = 0; i < reg_addr_size; i++) {
          int shift = (reg_addr_size - i - 1) * 8;
          data[i] = (register_address >> shift) & 0xff;
        }
      } else {
        // encode the address as little endian (LSB first)
        for (size_t i = 0; i < reg_addr_size; i++) {
          int shift = i * 8;
          data[i] = (register_address >> shift) & 0xff;
        }
      }
    }
  }

  Config base_config_;                      ///< The configuration for the peripheral
  mutable std::recursive_mutex base_mutex_; ///< The mutex to protect access to the peripheral
};
} // namespace espp

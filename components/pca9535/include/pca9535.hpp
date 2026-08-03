#pragma once

#include <functional>

#include "base_peripheral.hpp"

namespace espp {
/**
 * Class for communicating with and controlling a PCA9535 / PCA9555 16-bit I2C
 * GPIO expander. The PCA9535 and PCA9555 share an identical register map; the
 * PCA9535 additionally provides an open-drain interrupt output, while the
 * PCA9555 is otherwise identical. A single driver therefore covers both parts.
 *
 * The device exposes two 8-bit ports (Port 0 and Port 1), each of which can be
 * independently configured for input or output, and each of which supports
 * input polarity inversion.
 *
 * \section pca9535_ex1 PCA9535 Example
 * \snippet pca9535_example.cpp pca9535 example
 */
class Pca9535 : public BasePeripheral<> {
public:
  static constexpr uint8_t DEFAULT_ADDRESS =
      0x20; ///< Default I2C address (A2..A0 = 000). Valid range is 0x20 - 0x27.

  /**
   * The two GPIO ports the PCA9535 / PCA9555 has.
   */
  enum class Port {
    PORT0, ///< Port 0
    PORT1  ///< Port 1
  };

  /**
   * @brief Configuration information for the Pca9535.
   */
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS; ///< I2C address of this device.
    uint8_t port_0_direction_mask =
        0xFF; ///< Direction mask (1 = input, 0 = output) for port 0. Default all inputs.
    uint8_t port_0_polarity_mask =
        0x00; ///< Polarity inversion mask (1 = inverted) for port 0 inputs.
    uint8_t port_1_direction_mask =
        0xFF; ///< Direction mask (1 = input, 0 = output) for port 1. Default all inputs.
    uint8_t port_1_polarity_mask =
        0x00;                       ///< Polarity inversion mask (1 = inverted) for port 1 inputs.
    BasePeripheral::write_fn write; ///< Function to write to the device.
    BasePeripheral::read_register_fn
        read_register;     ///< Function to read bytes at a register address from the device.
    bool auto_init = true; ///< True if the device should be initialized on
                           ///< construction.
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log verbosity for the component.
  };

  /**
   * @brief Construct the Pca9535 and configure it.
   * @param config Config structure for configuring the PCA9535 / PCA9555
   */
  explicit Pca9535(const Config &config)
      : BasePeripheral({.address = config.device_address,
                        .write = config.write,
                        .read_register = config.read_register},
                       "Pca9535", config.log_level)
      , port_0_direction_mask_(config.port_0_direction_mask)
      , port_0_polarity_mask_(config.port_0_polarity_mask)
      , port_1_direction_mask_(config.port_1_direction_mask)
      , port_1_polarity_mask_(config.port_1_polarity_mask) {
    if (config.auto_init) {
      std::error_code ec;
      init(ec);
    }
  }

  /**
   * @brief Initialize the device.
   * @details Writes the direction (Config) registers and the polarity
   *          inversion registers for both ports from the configured masks.
   * @param ec Error code to set if there is an error.
   */
  void initialize(std::error_code &ec) {
    init(ec);
    if (ec) {
      logger_.error("Failed to initialize: {}", ec.message());
    }
  }

  /**
   * @brief Read the pin values on the provided port.
   * @param port The Port for which to read the pins.
   * @param ec Error code to set if there is an error.
   * @return The pin values as an 8 bit mask.
   */
  uint8_t get_pins(Port port, std::error_code &ec) {
    auto addr = port == Port::PORT0 ? Registers::InputPort0 : Registers::InputPort1;
    auto val = read_u8_from_register((uint8_t)addr, ec);
    if (ec) {
      logger_.error("Failed to read pins: {}", ec.message());
      return 0;
    }
    return val;
  }

  /**
   * @brief Read the pin values on both Port 0 and Port 1.
   * @param ec Error code to set if an error occurs.
   * @return The pin values as a 16 bit mask (Port 0 low byte, Port 1 high byte).
   */
  uint16_t get_pins(std::error_code &ec) {
    uint8_t buf[2] = {0, 0};
    read_many_from_register((uint8_t)Registers::InputPort0, buf, 2, ec);
    if (ec) {
      logger_.error("Failed to read pins: {}", ec.message());
      return 0;
    }
    uint16_t port0 = buf[0];
    uint16_t port1 = buf[1];
    return port0 | (port1 << 8);
  }

  /**
   * @brief Set the pin values on the provided port.
   * @param port The Port for which to set the pin outputs.
   * @param output The pin values as an 8 bit mask to set.
   * @param ec Error code to set if there is an error.
   */
  void set_pins(Port port, uint8_t output, std::error_code &ec) {
    auto addr = port == Port::PORT0 ? Registers::OutputPort0 : Registers::OutputPort1;
    write_u8_to_register((uint8_t)addr, output, ec);
  }

  /**
   * @brief Read back the output register for the provided port.
   * @param port The Port for which to read the output register.
   * @param ec Error code to set if there is an error.
   * @return The output register value as an 8 bit mask.
   */
  uint8_t get_output(Port port, std::error_code &ec) {
    auto addr = port == Port::PORT0 ? Registers::OutputPort0 : Registers::OutputPort1;
    auto val = read_u8_from_register((uint8_t)addr, ec);
    if (ec) {
      logger_.error("Failed to read output: {}", ec.message());
      return 0;
    }
    return val;
  }

  /**
   * @brief Set the i/o direction for the pins according to mask.
   * @note For the PCA9535 / PCA9555 the direction (Config) register uses the
   *       convention 1 = input, 0 = output. This differs from some other
   *       expanders, so take care when porting masks between drivers.
   * @param port The port associated with the provided pin mask.
   * @param mask The mask indicating direction (1 = input, 0 = output).
   * @param ec Error code to set if there is an error.
   */
  void set_direction(Port port, uint8_t mask, std::error_code &ec) {
    logger_.debug("Setting direction for {} to {}", (uint8_t)port, mask);
    auto addr = port == Port::PORT0 ? Registers::ConfigPort0 : Registers::ConfigPort1;
    write_u8_to_register((uint8_t)addr, mask, ec);
  }

  /// @brief Read the direction (configuration) register for a port.
  /// @param port The Port whose direction register to read.
  /// @param ec The error code, set on failure.
  /// @return The direction mask (1 = input, 0 = output). Useful for a
  ///         read-modify-write when only some pins should change direction.
  uint8_t get_direction(Port port, std::error_code &ec) {
    auto addr = port == Port::PORT0 ? Registers::ConfigPort0 : Registers::ConfigPort1;
    return read_u8_from_register((uint8_t)addr, ec);
  }

  /**
   * @brief Set the input polarity inversion for the pins according to mask.
   * @param port The port associated with the provided polarity mask.
   * @param mask Polarity mask for the pins, 1 -> invert the input pin value.
   * @param ec Error code to set if there is an error.
   */
  void set_polarity_inversion(Port port, uint8_t mask, std::error_code &ec) {
    logger_.debug("Setting polarity inversion for {} to {}", (uint8_t)port, mask);
    auto addr = port == Port::PORT0 ? Registers::PolarityPort0 : Registers::PolarityPort1;
    write_u8_to_register((uint8_t)addr, mask, ec);
  }

protected:
  /**
   * @brief Register map for the PCA9535 / PCA9555.
   */
  enum class Registers : uint8_t {
    InputPort0 = 0x00,    ///< Input port 0 - reflects the incoming logic levels of the pins
    InputPort1 = 0x01,    ///< Input port 1 - reflects the incoming logic levels of the pins
    OutputPort0 = 0x02,   ///< Output port 0 - the outgoing logic levels of the pins defined as
                          ///< outputs
    OutputPort1 = 0x03,   ///< Output port 1 - the outgoing logic levels of the pins defined as
                          ///< outputs
    PolarityPort0 = 0x04, ///< Polarity inversion (port 0) - 1 = input value inverted
    PolarityPort1 = 0x05, ///< Polarity inversion (port 1) - 1 = input value inverted
    ConfigPort0 = 0x06,   ///< Configuration / direction (port 0) - 1 = input, 0 = output
    ConfigPort1 = 0x07,   ///< Configuration / direction (port 1) - 1 = input, 0 = output
  };

  void init(std::error_code &ec) {
    set_direction(Port::PORT0, port_0_direction_mask_, ec);
    if (ec)
      return;
    set_direction(Port::PORT1, port_1_direction_mask_, ec);
    if (ec)
      return;
    set_polarity_inversion(Port::PORT0, port_0_polarity_mask_, ec);
    if (ec)
      return;
    set_polarity_inversion(Port::PORT1, port_1_polarity_mask_, ec);
  }

  uint8_t port_0_direction_mask_;
  uint8_t port_0_polarity_mask_;
  uint8_t port_1_direction_mask_;
  uint8_t port_1_polarity_mask_;
};
} // namespace espp

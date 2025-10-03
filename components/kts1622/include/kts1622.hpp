#pragma once

#include <functional>

#include "base_peripheral.hpp"

namespace espp {
/**
 * Class for communicating with and controlling a KTS1622 GPIO expander
 * including interrupt configuration. It supports 16 GPIO pins, 8 on each port,
 * and can support optional input debounce timing (only P0_1-P0_7 and P1_0-P1_7,
 * with P0_0 as clock input) and interrupt memory with trigger/mask/clear/status
 * features. It supports up to 1MHz Fast-mode Plus I2C and can operate [1.65,
 * 5.5]V on the I2C bus and I/O pins (with separate power pins for each).
 *
 * \section kts1622_ex1 KTS1622 Example
 * \snippet kts1622_example.cpp kts1622 example
 */
class Kts1622 : public BasePeripheral<> {
public:
  static constexpr uint8_t DEFAULT_ADDRESS = 0x20; ///< Lower 2 bits are configurable via the ADDR
                                                   ///< pin (GND, VCC, SCL, SDA -> 00, 01, 02, 03)

  /**
   * The two GPIO ports the Kts1622 has.
   */
  enum class Port {
    PORT0, ///< Port 0
    PORT1  ///< Port 1
  };

  enum class InterruptType {
    LEVEL = 0,   ///< Interrupt on level
    RISING = 1,  ///< Interrupt on rising edge
    FALLING = 2, ///< Interrupt on falling edge
    CHANGE = 3,  ///< Interrupt on change (any edge)
  };

  /**
   * The output drive mode configuration.
   */
  enum class OutputDriveMode : int {
    PUSH_PULL = 0,  ///< In this mode it needs no pull-up resistor. This is the default mode.
    OPEN_DRAIN = 1, ///< In this mode it needs a pull-up reistor.
  };

  /**
   * The output drive mode configuration.
   */
  enum class OutputDriveStrength : uint8_t {
    F_0_25 = 0, ///< 0.25x drive capability of the I/O pins
    F_0_5 = 1,  ///< 0.5x drive capability of the I/O pins
    F_0_75 = 2, ///< 0.75x drive capability of the I/O pins
    F_1 = 3,    ///< 1x drive capability of the I/O pins
  };

  /**
   * The Pull Resistor configuration.
   */
  enum class PullResistor : uint8_t {
    NO_PULL = 0,   ///< No pull resistor enabled
    PULL_UP = 1,   ///< Pull-up resistor enabled
    PULL_DOWN = 2, ///< Pull-down resistor enabled
  };

  /**
   * @brief Configuration information for the Kts1622.
   */
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS; ///< I2C address to use to talk to this KTS1622B.
    uint8_t port_0_direction_mask = 0xFF;     ///< Direction mask (1 = input) for port 0.
    uint8_t port_0_interrupt_mask = 0xFF; ///< Interrupt mask (1 = disable interrupt) for port 0.
    uint8_t port_1_direction_mask = 0xFF; ///< Direction mask (1 = input) for port 1.
    uint8_t port_1_interrupt_mask = 0xFF; ///< Interrupt mask (1 = disable interrupt) for port 1.
    std::optional<uint8_t> port_0_initial_output =
        std::nullopt; ///< Initial output value for port 0 (only for pins set as
                      ///< outputs). If not set, outputs will be low.
    std::optional<uint8_t> port_1_initial_output =
        std::nullopt; ///< Initial output value for port 1 (only for pins set as
                      ///< outputs). If not set, outputs will be low.
    OutputDriveMode output_drive_mode =
        OutputDriveMode::PUSH_PULL; ///< Output drive mode for the ports.
    BasePeripheral::write_fn write; ///< Function to write to the device.
    BasePeripheral::write_then_read_fn
        write_then_read;   ///< Function to write then read from the device.
    bool auto_init = true; ///< Automatically initialize the device.
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log verbosity for the component.
  };

  /**
   * @brief Construct the Kts1622. Will call initialize() if auto_init is true.
   * @param config Config structure for configuring the KTS1622
   */
  explicit Kts1622(const Config &config)
      : BasePeripheral({.address = config.device_address,
                        .write = config.write,
                        .write_then_read = config.write_then_read},
                       "Kts1622", config.log_level)
      , config_(config) {
    if (config.auto_init) {
      std::error_code ec;
      if (!initialize(ec)) {
        logger_.error("Failed to initialize KTS1622: {}", ec.message());
      }
    }
  }

  /**
   * @brief Initialize the component class.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool initialize(std::error_code &ec) { return init(config_, ec); }

  // TODO: provide configuration of output drive strength (registers 0x40-0x43),
  //      where each port pin can be configured to have a drive strength of
  //      0.25x, 0.5x, 0.75x, or 1x (default) the normal drive strength.

  // TODO: provide configuration of individual pin output drive mode (registers
  //     0x44-0x45), where each port pin can be configured to have a push-pull

  /**
   * @brief Read the pin values on the provided port.
   * @param port The Port for which to read the pins
   * @param ec Error code to set if an error occurs.
   * @return The pin values as an 8 bit mask.
   */
  uint8_t get_pins(Port port, std::error_code &ec) {
    auto addr = port == Port::PORT0 ? Registers::INPORT0 : Registers::INPORT1;
    return read_u8_from_register((uint8_t)addr, ec);
  }

  /**
   * @brief Read the pin values on both Port 0 and Port 1.
   * @param ec Error code to set if an error occurs.
   * @return The pin values as a 16 bit mask (P0_0 lsb, P1_7 msb).
   */
  uint16_t get_pins(std::error_code &ec) {
    uint16_t p0 = read_u8_from_register((uint8_t)Registers::INPORT0, ec);
    if (ec)
      return 0;
    uint16_t p1 = read_u8_from_register((uint8_t)Registers::INPORT1, ec);
    if (ec)
      return 0;
    return (p1 << 8) | p0;
  }

  /**
   * @brief Write the pin values on the provided port.
   * @param port The Port for which to write the pins
   * @param value The pin values to apply.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   * @note This will overwrite any previous pin values on the port for all
   *       output pins on the port.
   */
  bool output(Port port, uint8_t value, std::error_code &ec) {
    auto addr = port == Port::PORT0 ? Registers::OUTPORT0 : Registers::OUTPORT1;
    write_u8_to_register((uint8_t)addr, value, ec);
    return !ec;
  }

  /**
   * @brief Write the pin values on both Port 0 and Port 1.
   * @param p0 The pin values to apply to Port 0.
   * @param p1 The pin values to apply to Port 1.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   * @note This will overwrite any previous pin values on the port for all
   *       output pins on the ports.
   */
  bool output(uint8_t p0, uint8_t p1, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (!output(Port::PORT0, p0, ec))
      return false;
    return output(Port::PORT1, p1, ec);
  }

  /**
   * @brief Write the pin values on both Port 0 and Port 1.
   * @param value The pin values to apply as a 16 bit value (P0_0 lsb, P1_7 msb).
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   * @note This will overwrite any previous pin values on the port for all
   *       output pins on the ports.
   */
  bool output(uint16_t value, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (!output(Port::PORT0, value & 0xFF, ec))
      return false;
    return output(Port::PORT1, value >> 8, ec);
  }

  /**
   * @brief Clear the pin values on the provided port according to the provided mask.
   * @details Reads the current pin values and clears any bits set in the mask.
   * @param port The Port for which to clear the pin outputs.
   * @param mask The pin values as an 8 bit mask to clear.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool clear_pins(Port port, uint8_t mask, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    auto addr = port == Port::PORT0 ? Registers::OUTPORT0 : Registers::OUTPORT1;
    auto data = read_u8_from_register((uint8_t)addr, ec);
    if (ec)
      return false;
    data &= ~mask;
    write_u8_to_register((uint8_t)addr, data, ec);
    return !ec;
  }

  /**
   * @brief Clear the pin values for Port 0 and Port 1 according to the provided masks.
   * @details Reads the current pin values and clears any bits set in the masks.
   * @param p0 The pin values as an 8 bit mask for Port 0 to clear.
   * @param p1 The pin values as an 8 bit mask for Port 1 to clear.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool clear_pins(uint8_t p0, uint8_t p1, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (!clear_pins(Port::PORT0, p0, ec))
      return false;
    return clear_pins(Port::PORT1, p1, ec);
  }

  /**
   * @brief Clear the pin values for Port 0 and Port 1 according to the provided mask.
   * @details Reads the current pin values and clears any bits set in the mask.
   * @param mask The pin values as a 16 bit mask (P0_0 lsb, P1_7 msb) to clear.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool clear_pins(uint16_t mask, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (!clear_pins(Port::PORT0, mask & 0xFF, ec))
      return false;
    return clear_pins(Port::PORT1, mask >> 8, ec);
  }

  /**
   * @brief Set the pin values on the provided port according to the provided mask.
   * @brief Reads the current pin values and sets any bits set in the mask.
   * @param port The Port for which to set the pin outputs.
   * @param mask The pin values as an 8 bit mask to set.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool set_pins(Port port, uint8_t mask, std::error_code &ec) {
    auto addr = port == Port::PORT0 ? Registers::OUTPORT0 : Registers::OUTPORT1;
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    auto data = read_u8_from_register((uint8_t)addr, ec);
    if (ec)
      return false;
    data |= mask;
    write_u8_to_register((uint8_t)addr, data, ec);
    return !ec;
  }

  /**
   * @brief Set the pin values for Port 0 and Port 1 according to the provided masks.
   * @details Reads the current pin values and sets any bits set in the masks.
   * @param p0 The pin values for Port 0 as an 8 bit mask to set.
   * @param p1 The pin values for Port 1 as an 8 bit mask to set.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool set_pins(uint8_t p0, uint8_t p1, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (!set_pins(Port::PORT0, p0, ec))
      return false;
    return set_pins(Port::PORT1, p1, ec);
  }

  /**
   * @brief Set the pin values for Port 0 and Port 1 according to the provided mask.
   * @details Reads the current pin values and sets any bits set in the mask.
   * @param mask The pin values as a 16 bit mask (P0_0 lsb, P1_7 msb) to set.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool set_pins(uint16_t mask, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (!set_pins(Port::PORT0, mask & 0xFF, ec))
      return false;
    return set_pins(Port::PORT1, mask >> 8, ec);
  }

  /**
   * @brief Read the output pin values on the provided port.
   * @param port The Port for which to read the pins
   * @param ec Error code to set if an error occurs.
   * @return The pin values as an 8 bit mask.
   */
  uint8_t get_output(Port port, std::error_code &ec) {
    auto addr = port == Port::PORT0 ? Registers::OUTPORT0 : Registers::OUTPORT1;
    return read_u8_from_register((uint8_t)addr, ec);
  }

  /**
   * @brief Read the output pin values on both Port 0 and Port 1.
   * @param ec Error code to set if an error occurs.
   * @return The pin values as a 16 bit mask (P0_0 lsb, P1_7 msb).
   */
  uint16_t get_output(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint16_t p0 = read_u8_from_register((uint8_t)Registers::OUTPORT0, ec);
    if (ec)
      return 0;
    uint16_t p1 = read_u8_from_register((uint8_t)Registers::OUTPORT1, ec);
    if (ec)
      return 0;
    return (p1 << 8) | p0;
  }

  /**
   * @brief Set the output drive mode for the provided pins on the provided port.
   * @param port The port to set the output drive mode for.
   * @param mode The output drive mode to set for the port.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool set_port_output_drive_mode(Port port, OutputDriveMode mode, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // get the current value from the register
    uint8_t data = read_u8_from_register((uint8_t)Registers::OUT_CFG, ec);
    if (ec)
      return false;
    // if it's PORT0, we write to bit 0 of the register, if it's PORT1, we write to bit 1
    uint8_t mask = port == Port::PORT0 ? 0x01 : 0x02;
    // if it's OPEN_DRAIN, we write 1 to the bit, if it's PUSH_PULL, we write 0
    data = mode == OutputDriveMode::OPEN_DRAIN ? data | mask : data & ~mask;
    write_u8_to_register((uint8_t)Registers::OUT_CFG, data, ec);
    return !ec;
  }

  /**
   * @brief Configure the provided pins to interrupt.
   * @param port The port associated with the provided pin mask.
   * @param mask The pin mask to configure for interrupt (0=interrupt).
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   * @note You should also call configure_interrupt to configure the interrupt type for the
   *       pins.
   * @see configure_interrupt
   */
  bool enable_interrupt(Port port, uint8_t mask, std::error_code &ec) {
    logger_.info("Enabling interrupt (0=interrupt enabled) for Port {} pins {:#08b}", port, mask);
    auto addr = port == Port::PORT0 ? Registers::INT_MASK0 : Registers::INT_MASK1;
    write_u8_to_register((uint8_t)addr, mask, ec);
    return !ec;
  }

  /**
   * @brief Configure the provided pins to interrupt on change.
   * @param p0 The pin mask for Port 0 to configure for interrupt (0=interrupt).
   * @param p1 The pin mask for Port 1 to configure for interrupt (0=interrupt).
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   * @note You should also call configure_interrupt to configure the interrupt type for the
   *       pins.
   * @see configure_interrupt
   */
  bool enable_interrupt(uint8_t p0, uint8_t p1, std::error_code &ec) {
    logger_.info("Enabling interrupt (0=interrupt enabled) p0:{:#08b}, p1:{:#08b}", p0, p1);
    auto addr = Registers::INT_MASK0;
    const uint8_t data[] = {p0, p1};
    write_many_to_register((uint8_t)addr, data, 2, ec);
    return !ec;
  }

  /**
   * @brief Configure the provided pins to interrupt on change.
   * @param mask The pin mask to configure for interrupt (0=interrupt).
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   * @note You should also call configure_interrupt to configure the interrupt type for the
   *       pins.
   * @see configure_interrupt
   */
  bool enable_interrupt(uint16_t mask, std::error_code &ec) {
    return enable_interrupt((uint8_t)(mask & 0xFF), (uint8_t)(mask >> 8), ec);
  }

  /**
   * @brief Configure the provided pins to interrupt on the provided type.
   * @param port The port associated with the provided pin mask.
   * @param mask The pin mask to configure for interrupt.
   * @param type The type of interrupt to configure.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   * @note You should also call enable_interrupt to enable the interrupt for the
   *       pins.
   * @note This will overwrite any previous interrupt configuration on the port for all
   *       pins on the port.
   * @see enable_interrupt
   */
  bool configure_interrupt(Port port, uint8_t mask, InterruptType type, std::error_code &ec) {
    logger_.info("Configuring interrupt for Port {} pins {:#08b} to {}", port, mask, type);
    // The configuration takes 2 bits per pin, so we need to potentially write 2
    // bytes for a single 8 bit mask (8 pins). Therefore we'll build the value
    // by taking the mask and expanding each 1 bit into two bits.
    uint16_t data = 0;
    for (int i = 0; i < 8; i++) {
      if (mask & (1 << i)) {
        data |= (uint16_t)type << (i * 2);
      }
    }
    // now write those two bytes to the appropriate INT_EDGE register depending
    // on the port
    auto addr = port == Port::PORT0 ? Registers::INT_EDGE0A : Registers::INT_EDGE1A;
    write_u16_to_register((uint8_t)addr, data, ec);
    return !ec;
  }

  /**
   * @brief Configure the provided pins to interrupt on the provided type.
   * @param p0 The pin mask for Port 0 to configure for interrupt.
   * @param p1 The pin mask for Port 1 to configure for interrupt.
   * @param type The type of interrupt to configure.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   * @note You should also call enable_interrupt to enable the interrupt for the
   *       pins.
   * @note This will overwrite any previous interrupt configuration on the port for all
   *       pins on the port.
   * @see enable_interrupt
   */
  bool configure_interrupt(uint8_t p0, uint8_t p1, InterruptType type, std::error_code &ec) {
    logger_.info("Configuring interrupt p0:{:#08b}, p1:{:#08b} to {}", p0, p1, type);
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    configure_interrupt(Port::PORT0, p0, type, ec);
    if (ec)
      return false;
    configure_interrupt(Port::PORT1, p1, type, ec);
    return !ec;
  }

  /**
   * @brief Clear the interrupt for the provided pins.
   * @param port The port associated with the provided pin mask.
   * @param mask The pin mask to clear the interrupt for.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool clear_interrupt(Port port, uint8_t mask, std::error_code &ec) {
    logger_.info("Clearing interrupt for Port {} pins {:#08b}", port, mask);
    auto addr = port == Port::PORT0 ? Registers::INT_CLEAR0 : Registers::INT_CLEAR1;
    write_u8_to_register((uint8_t)addr, mask, ec);
    return !ec;
  }

  /**
   * @brief Clear the interrupt for the provided pins.
   * @param p0 The pin mask for Port 0 to clear the interrupt for.
   * @param p1 The pin mask for Port 1 to clear the interrupt for.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool clear_interrupt(uint8_t p0, uint8_t p1, std::error_code &ec) {
    logger_.info("Clearing interrupt p0:{:#08b}, p1:{:#08b}", p0, p1);
    auto addr = Registers::INT_CLEAR0;
    const uint8_t data[] = {p0, p1};
    write_many_to_register((uint8_t)addr, data, 2, ec);
    return !ec;
  }

  /**
   * @brief Clear the interrupt for the provided pins.
   * @param mask The pin mask to clear the interrupt for.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool clear_interrupt(uint16_t mask, std::error_code &ec) {
    clear_interrupt((uint8_t)(mask & 0xFF), (uint8_t)(mask >> 8), ec);
    return !ec;
  }

  /**
   * @brief Clear all interrupts.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool clear_interrupts(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    write_u8_to_register((uint8_t)Registers::INT_CLEAR0, 0xFF, ec);
    if (ec)
      return false;
    write_u8_to_register((uint8_t)Registers::INT_CLEAR1, 0xFF, ec);
    return !ec;
  }

  /**
   * @brief Set the i/o direction for the pins according to mask.
   * @param port The port associated with the provided pin mask.
   * @param mask The mask indicating direction (1 = input, 0 = output)
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool set_direction(Port port, uint8_t mask, std::error_code &ec) {
    logger_.info("Setting direction (1=input) for Port {} to {:#08b}", port, mask);
    auto addr = port == Port::PORT0 ? Registers::CONFIG0 : Registers::CONFIG1;
    write_u8_to_register((uint8_t)addr, mask, ec);
    return !ec;
  }

  /**
   * @brief Set the i/o direction for the pins on Port 0 and Port 1.
   * @param p0 The mask for Port 0 indicating direction (1 = input, 0 = output)
   * @param p1 The mask for Port 1 indicating direction (1 = input, 0 = output)
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool set_direction(uint8_t p0, uint8_t p1, std::error_code &ec) {
    logger_.info("Setting direction (1=input) p0:{:#08b}, p1:{:#08b}", p0, p1);
    auto addr = Registers::CONFIG0;
    const uint8_t data[] = {p0, p1};
    write_many_to_register((uint8_t)addr, data, 2, ec);
    return !ec;
  }

  /**
   * @brief Set the i/o direction for the pins according to mask.
   * @param mask The mask indicating direction (1 = input, 0 = output)
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool set_direction(uint16_t mask, std::error_code &ec) {
    return set_direction((uint8_t)(mask & 0xFF), (uint8_t)(mask >> 8), ec);
  }

  /**
   * @brief Set the i/o direction for the pins according to mask.
   * @param mask The mask indicating pin position
   * @param direction The direction indicating direction (1 = input, 0 = output)
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool set_direction(uint16_t mask, bool direction, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    auto addr = Registers::CONFIG0;
    uint8_t data[] = {0, 0};
    read_many_from_register((uint8_t)addr, data, 2, ec);
    if (ec)
      return false;
    if (direction) {
      // To Input port
      data[0] |= (uint8_t)mask;
      data[1] |= (uint8_t)(mask >> 8);
    } else {
      // To Output port
      data[0] &= (uint8_t)~mask;
      data[1] &= (uint8_t)(~mask >> 8);
    }
    write_many_to_register((uint8_t)addr, data, 2, ec);
    return !ec;
  }

  /**
   * @brief Set polarity inversion for the pins according to mask.
   * @param port The port associated with the provided pin mask.
   * @param mask The mask indicating polarity inversion (1 = inverted, 0 = normal)
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   * @note This will overwrite any previous polarity inversion on the port for all
   *       pins on the port.
   * @note This will only affect pins configured as inputs.
   */
  bool set_polarity_inversion(Port port, uint8_t mask, std::error_code &ec) {
    logger_.info("Setting polarity inversion (1=inverted) for Port {} to {:#08b}", port, mask);
    auto addr = port == Port::PORT0 ? Registers::POLARITY0 : Registers::POLARITY1;
    write_u8_to_register((uint8_t)addr, mask, ec);
    return !ec;
  }

  /**
   * @brief Set polarity inversion for the pins on Port 0 and Port 1.
   * @param p0 The mask for Port 0 indicating polarity inversion (1 = inverted, 0 = normal)
   * @param p1 The mask for Port 1 indicating polarity inversion (1 = inverted, 0 = normal)
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   * @note This will overwrite any previous polarity inversion on the port for all
   *       pins on the port.
   * @note This will only affect pins configured as inputs.
   */
  bool set_polarity_inversion(uint8_t p0, uint8_t p1, std::error_code &ec) {
    logger_.info("Setting polarity inversion (1=inverted) p0:{:#08b}, p1:{:#08b}", p0, p1);
    auto addr = Registers::POLARITY0;
    const uint8_t data[] = {p0, p1};
    write_many_to_register((uint8_t)addr, data, 2, ec);
    return !ec;
  }

  /**
   * @brief Set polarity inversion for the pins according to mask.
   * @param mask The mask indicating polarity inversion (1 = inverted, 0 = normal)
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   * @note This will overwrite any previous polarity inversion on the port for all
   *       pins on the port.
   * @note This will only affect pins configured as inputs.
   */
  bool set_polarity_inversion(uint16_t mask, std::error_code &ec) {
    return set_polarity_inversion((uint8_t)(mask & 0xFF), (uint8_t)(mask >> 8), ec);
  }

  /**
   * @brief Set the input latch for the input pins according to latch.
   * @param port The port associated with the provided pin mask.
   * @param latch The value to set the latch register to. A value of 0 for a bit
   *        indicates that the corresponding input is not latched, meaning that
   *        a pin configured as an interrupt input will clear the interrupt if
   *        it returns to its original state before the interrupt is read.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool set_input_latch(Port port, uint8_t latch, std::error_code &ec) {
    logger_.info("Setting input latch (1=latched) for Port {} to {:#08b}", port, latch);
    auto addr = port == Port::PORT0 ? Registers::INPUT_LATCH0 : Registers::INPUT_LATCH1;
    write_u8_to_register((uint8_t)addr, latch, ec);
    return !ec;
  }

  /**
   * @brief Set the input latch for the input pins on Port 0 and Port 1.
   * @param p0 The value to set the latch register to for Port 0. A value of 0 for a bit
   *        indicates that the corresponding input is not latched, meaning that
   *        a pin configured as an interrupt input will clear the interrupt if
   *        it returns to its original state before the interrupt is read.
   * @param p1 The value to set the latch register to for Port 1. A value of 0 for a bit
   *        indicates that the corresponding input is not latched, meaning that
   *        a pin configured as an interrupt input will clear the interrupt if
   *        it returns to its original state before the interrupt is read.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   * @note This will overwrite any previous latch values on the port for all
   *       input pins on the port.
   * @see set_input_latch(Port, uint8_t, std::error_code&)
   * @see enable_interrupt(Port, uint8_t, std::error_code&)
   */
  bool set_input_latch(uint8_t p0, uint8_t p1, std::error_code &ec) {
    logger_.info("Setting input latch (1=latched) p0:{:#08b}, p1:{:#08b}", p0, p1);
    auto addr = Registers::INPUT_LATCH0;
    const uint8_t data[] = {p0, p1};
    write_many_to_register((uint8_t)addr, data, 2, ec);
    return !ec;
  }

  /**
   * @brief Set the input latch for the input pins according to latch.
   * @param mask The value to set the latch register to. A value of 0 for a bit
   *        indicates that the corresponding input is not latched, meaning that
   *        a pin configured as an interrupt input will clear the interrupt if
   *        it returns to its original state before the interrupt is read.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   * @note This will overwrite any previous latch values on the port for all
   *       input pins on the port.
   * @see set_input_latch(Port, uint8_t, std::error_code&)
   * @see enable_interrupt(Port, uint8_t, std::error_code&)
   */
  bool set_input_latch(uint16_t mask, std::error_code &ec) {
    return set_input_latch((uint8_t)(mask & 0xFF), (uint8_t)(mask >> 8), ec);
  }

  /**
   * @brief Set the pull resistor for the provided pins.
   * @param port The port associated with the provided pin mask.
   * @param pin_mask The pin mask to configure for pull resistor.
   * @param pull The pull resistor to configure.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool set_pull_resistor_for_pin(Port port, uint8_t pin_mask, PullResistor pull,
                                 std::error_code &ec) {
    // if pull == PullResistor::NO_PULL, then clear the bits in the appropriate
    // PULL_UP_DOWN_EN register
    auto addr = port == Port::PORT0 ? Registers::PULL_UP_DOWN_EN0 : Registers::PULL_UP_DOWN_EN1;
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    auto data = read_u8_from_register((uint8_t)addr, ec);
    if (ec)
      return false;
    if (pull == PullResistor::NO_PULL) {
      logger_.info("Disabling pull-up/down for Port {} pin {:#08b}", port, pin_mask);
      data &= ~pin_mask;
    } else {
      logger_.info("Enabling pull-up/down for Port {} pin {:#08b}", port, pin_mask);
      data |= pin_mask;
    }
    write_u8_to_register((uint8_t)addr, data, ec);
    if (ec)
      return false;
    if (pull == PullResistor::NO_PULL) {
      return true;
    }
    // if pull != PullResistor::NO_PULL, then set the bits in the appropriate
    // PULL_UP_DOWN_SEL register. 1 (default) is pull-up, 0 is pull-down
    addr = port == Port::PORT0 ? Registers::PULL_UP_DOWN_SEL0 : Registers::PULL_UP_DOWN_SEL1;
    data = read_u8_from_register((uint8_t)addr, ec);
    if (ec)
      return false;
    if (pull == PullResistor::PULL_UP) {
      logger_.info("Setting pull-up for Port {} pin {:#08b}", port, pin_mask);
      data |= pin_mask;
    } else {
      logger_.info("Setting pull-down for Port {} pin {:#08b}", port, pin_mask);
      data &= ~pin_mask;
    }
    write_u8_to_register((uint8_t)addr, data, ec);
    return !ec;
  }

protected:
  /**
   * @brief Register map for the MT23X17
   */
  enum class Registers : uint8_t {
    INPORT0 = 0x00,   ///< Read input values for Port 0
    INPORT1 = 0x01,   ///< Read input values for Port 1
    OUTPORT0 = 0x02,  ///< Write output values for Port 0
    OUTPORT1 = 0x03,  ///< Write output values for Port 1
    POLARITY0 = 0x04, ///< Polarity inversion for Port 0 (1=inverted, 0=normal, default=0)
    POLARITY1 = 0x05, ///< Polarity inversion for Port 1 (1=inverted, 0=normal, default=0)
    CONFIG0 = 0x06,   ///< Configuration for Port 0 (1=push-pull, 0=open-drain, default=0)
    CONFIG1 = 0x07,   ///< Configuration for Port 1 (1=push-pull, 0=open-drain, default=0)

    // registers 0x08-0x3F are reserved

    OUT_STR0A = 0x40, ///< Output strength for Port 0A (1=high, 0=low, default=0)
    OUT_STR0B = 0x41, ///< Output strength for Port 0B (1=high, 0=low, default=0)
    OUT_STR1A = 0x42, ///< Output strength for Port 1A (1=high, 0=low, default=0)
    OUT_STR1B = 0x43, ///< Output strength for Port 1B (1=high, 0=low, default=0)

    INPUT_LATCH0 = 0x44, ///< Input latch for Port 0 (1=latched, 0=transparent, default=0)
    INPUT_LATCH1 = 0x45, ///< Input latch for Port 1 (1=latched, 0=transparent, default=0)

    PULL_UP_DOWN_EN0 = 0x46,  ///< Pull-up/down enable for Port 0 (1=down, 0=up, default=0)
    PULL_UP_DOWN_EN1 = 0x47,  ///< Pull-up/down enable for Port 1 (1=down, 0=up, default=0)
    PULL_UP_DOWN_SEL0 = 0x48, ///< Pull-up/down select for Port 0 (1=down, 0=up, default=0)
    PULL_UP_DOWN_SEL1 = 0x49, ///< Pull-up/down select for Port 1 (1=down, 0=up, default=0)

    INT_MASK0 = 0x4A,   ///< Interrupt mask for Port 0 (1=disable, 0=enable, default=0)
    INT_MASK1 = 0x4B,   ///< Interrupt mask for Port 1 (1=disable, 0=enable, default=0)
    INT_STATUS0 = 0x4C, ///< Interrupt status for Port 0 (1=triggered, 0=not triggered, default=0)
    INT_STATUS1 = 0x4D, ///< Interrupt status for Port 1 (1=triggered, 0=not triggered, default=0)

    // registers 0x4E is reserved

    OUT_CFG =
        0x4F, ///< Output configuration for Port 0 and Port 1 (0=push-pull, 1=open-drain, default=0)

    INT_EDGE0A = 0x50, ///< Interrupt edge for Port 0A (1=rising, 0=falling, default=0)
    INT_EDGE0B = 0x51, ///< Interrupt edge for Port 0B (1=rising, 0=falling, default=0)
    INT_EDGE1A = 0x52, ///< Interrupt edge for Port 1A (1=rising, 0=falling, default=0)
    INT_EDGE1B = 0x53, ///< Interrupt edge for Port 1B (1=rising, 0=falling, default=0)
    INT_CLEAR0 = 0x54, ///< Clear interrupt for Port 0 (write 1 to clear)
    INT_CLEAR1 = 0x55, ///< Clear interrupt for Port 1 (write 1 to clear)

    INPUT_STAT0 = 0x56, ///< Input status for Port 0 (1=high, 0=low, default=0)
    INPUT_STAT1 = 0x57, ///< Input status for Port 1 (1=high, 0=low, default=0)

    OUT_CFG0 = 0x58, ///< Output configuration for Port 0 (0=push-pull, 1=open-drain, default=0)
    OUT_CFG1 = 0x59, ///< Output configuration for Port 1 (0=push-pull, 1=open-drain, default=0)

    DEBOUNCE_EN0 = 0x5A,   ///< Debounce enable for Port 0 (1=enable, 0=disable, default=0)
    DEBOUNCE_EN1 = 0x5B,   ///< Debounce enable for Port 1 (1=enable, 0=disable, default=0)
    DEBOUNCE_COUNT = 0x5C, ///< Debounce count (default=0)
  };

  bool init(const Config &config, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // set the output drive mode for both ports before setting the output values
    // or direction
    if (!set_port_output_drive_mode(Port::PORT0, config.output_drive_mode, ec))
      return false;
    if (!set_port_output_drive_mode(Port::PORT1, config.output_drive_mode, ec))
      return false;

    // set the intial output value before setting the direction, to make sure
    // there are no glitches on the output pins
    if (config.port_0_initial_output.has_value()) {
      auto initial_value = config.port_0_initial_output.value();
      logger_.info("Setting initial output value for Port 0 to {:#08b}", initial_value);
      if (!output(Port::PORT0, initial_value, ec))
        return false;
    }
    if (config.port_1_initial_output.has_value()) {
      auto initial_value = config.port_1_initial_output.value();
      logger_.info("Setting initial output value for Port 1 to {:#08b}", initial_value);
      if (!output(Port::PORT1, initial_value, ec))
        return false;
    }

    // now set the direction
    if (!set_direction(Port::PORT0, config.port_0_direction_mask, ec))
      return false;
    if (!set_direction(Port::PORT1, config.port_1_direction_mask, ec))
      return false;

    // and finally enable any interrupts
    if (!enable_interrupt(Port::PORT0, config.port_0_interrupt_mask, ec))
      return false;
    if (!enable_interrupt(Port::PORT1, config.port_1_interrupt_mask, ec))
      return false;

    return true;
  }

  Config config_;
};
} // namespace espp

// for printing of enums using libfmt
template <> struct fmt::formatter<espp::Kts1622::Port> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::Kts1622::Port &p, FormatContext &ctx) const {
    switch (p) {
    case espp::Kts1622::Port::PORT0:
      return fmt::format_to(ctx.out(), "Port 0");
    case espp::Kts1622::Port::PORT1:
      return fmt::format_to(ctx.out(), "Port 1");
    default:
      return fmt::format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::Kts1622::InterruptType> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::Kts1622::InterruptType &t, FormatContext &ctx) const {
    switch (t) {
    case espp::Kts1622::InterruptType::LEVEL:
      return fmt::format_to(ctx.out(), "Level");
    case espp::Kts1622::InterruptType::RISING:
      return fmt::format_to(ctx.out(), "Rising");
    case espp::Kts1622::InterruptType::FALLING:
      return fmt::format_to(ctx.out(), "Falling");
    case espp::Kts1622::InterruptType::CHANGE:
      return fmt::format_to(ctx.out(), "Change");
    default:
      return fmt::format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::Kts1622::OutputDriveMode> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::Kts1622::OutputDriveMode &m, FormatContext &ctx) const {
    switch (m) {
    case espp::Kts1622::OutputDriveMode::PUSH_PULL:
      return fmt::format_to(ctx.out(), "Push Pull");
    case espp::Kts1622::OutputDriveMode::OPEN_DRAIN:
      return fmt::format_to(ctx.out(), "Open Drain");
    default:
      return fmt::format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::Kts1622::OutputDriveStrength> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::Kts1622::OutputDriveStrength &s, FormatContext &ctx) const {
    switch (s) {
    case espp::Kts1622::OutputDriveStrength::F_0_25:
      return fmt::format_to(ctx.out(), "0.25x");
    case espp::Kts1622::OutputDriveStrength::F_0_5:
      return fmt::format_to(ctx.out(), "0.5x");
    case espp::Kts1622::OutputDriveStrength::F_0_75:
      return fmt::format_to(ctx.out(), "0.75x");
    case espp::Kts1622::OutputDriveStrength::F_1:
      return fmt::format_to(ctx.out(), "1x");
    default:
      return fmt::format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::Kts1622::PullResistor> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::Kts1622::PullResistor &p, FormatContext &ctx) const {
    switch (p) {
    case espp::Kts1622::PullResistor::PULL_UP:
      return fmt::format_to(ctx.out(), "Pull Up");
    case espp::Kts1622::PullResistor::PULL_DOWN:
      return fmt::format_to(ctx.out(), "Pull Down");
    case espp::Kts1622::PullResistor::NO_PULL:
      return fmt::format_to(ctx.out(), "No Pull");
    default:
      return fmt::format_to(ctx.out(), "Unknown");
    }
  }
};

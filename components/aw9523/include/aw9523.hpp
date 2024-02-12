#pragma once

#include <functional>

#include "base_peripheral.hpp"

namespace espp {
/**
 * Class for communicating with and controlling a AW9523 GPIO expander
 * including interrupt configuration and LED drive capability. Datasheet
 * hosted by adafruit.com here:
 * https://cdn-shop.adafruit.com/product-files/4886/AW9523+English+Datasheet.pdf
 *
 * \section aw9523_ex1 AW9523 Example
 * \snippet aw9523_example.cpp aw9523 example
 */
class Aw9523 : public BasePeripheral {
public:
  static constexpr uint8_t DEFAULT_ADDRESS = 0x58; ///< Lower 2 bits are AD1, AD0 pins on the chip.

  /**
   * The two GPIO ports the Aw9523 has.
   */
  enum class Port {
    PORT0, ///< Port 0
    PORT1  ///< Port 1
  };

  /**
   * The output drive mode configuration for PORT 0 pins.
   */
  enum class OutputDriveModeP0 : int {
    OPEN_DRAIN = 0, ///< In this mode it needs a pull-up reistor. This is the default mode.
    PUSH_PULL = 1,  ///< In this mode it needs no pull-up resistor.
  };

  /**
   * The max current allowed when driving LEDs.
   */
  enum class MaxLedCurrent : int {
    IMAX = 0,    ///< Full drive current (37mA), default
    IMAX_75 = 1, ///< 75% drive current
    IMAX_50 = 2, ///< 50% drive current
    IMAX_25 = 3, ///< 25% drive current
  };

  /**
   * @brief Configuration information for the Aw9523.
   */
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS; ///< I2C address to use to talk to this AW9523B.
    uint8_t port_0_direction_mask = 0x00;     ///< Direction mask (1 = input) for port 0.
    uint8_t port_0_interrupt_mask = 0x00; ///< Interrupt mask (1 = disable interrupt) for port 0.
    uint8_t port_1_direction_mask = 0x00; ///< Direction mask (1 = input) for port 1.
    uint8_t port_1_interrupt_mask = 0x00; ///< Interrupt mask (1 = disable interrupt) for port 1.
    OutputDriveModeP0 output_drive_mode_p0 =
        OutputDriveModeP0::OPEN_DRAIN;                   ///< Output drive mode for Port 0
    MaxLedCurrent max_led_current = MaxLedCurrent::IMAX; ///< Max current allowed on each LED.
    BasePeripheral::write_fn write;                      ///< Function to write to the device.
    BasePeripheral::write_then_read_fn
        write_then_read;   ///< Function to write then read from the device.
    bool auto_init = true; ///< Automatically initialize the device.
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log verbosity for the component.
  };

  /**
   * @brief Construct the Aw9523. Will call initialize() if auto_init is true.
   * @param config Config structure for configuring the AW9523
   */
  explicit Aw9523(const Config &config)
      : BasePeripheral({.address = config.device_address,
                        .write = config.write,
                        .write_then_read = config.write_then_read},
                       "Aw9523", config.log_level)
      , config_(config) {
    if (config.auto_init) {
      std::error_code ec;
      initialize(ec);
      if (ec) {
        logger_.error("Failed to initialize AW9523: {}", ec.message());
      }
    }
  }

  /**
   * @brief Initialize the component class.
   * @param ec Error code to set if an error occurs.
   */
  void initialize(std::error_code &ec) { init(config_, ec); }

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
   * @note This will overwrite any previous pin values on the port for all
   *       output pins on the port.
   */
  void output(Port port, uint8_t value, std::error_code &ec) {
    auto addr = port == Port::PORT0 ? Registers::OUTPORT0 : Registers::OUTPORT1;
    write_u8_to_register((uint8_t)addr, value, ec);
  }

  /**
   * @brief Write the pin values on both Port 0 and Port 1.
   * @param p0 The pin values to apply to Port 0.
   * @param p1 The pin values to apply to Port 1.
   * @param ec Error code to set if an error occurs.
   * @note This will overwrite any previous pin values on the port for all
   *       output pins on the ports.
   */
  void output(uint8_t p0, uint8_t p1, std::error_code &ec) {
    output(Port::PORT0, p0, ec);
    if (ec)
      return;
    output(Port::PORT1, p1, ec);
  }

  /**
   * @brief Write the pin values on both Port 0 and Port 1.
   * @param value The pin values to apply as a 16 bit value (P0_0 lsb, P1_7 msb).
   * @param ec Error code to set if an error occurs.
   * @note This will overwrite any previous pin values on the port for all
   *       output pins on the ports.
   */
  void output(uint16_t value, std::error_code &ec) {
    output(Port::PORT0, value & 0xFF, ec);
    if (ec)
      return;
    output(Port::PORT1, value >> 8, ec);
  }

  /**
   * @brief Clear the pin values on the provided port according to the provided mask.
   * @details Reads the current pin values and clears any bits set in the mask.
   * @param port The Port for which to clear the pin outputs.
   * @param mask The pin values as an 8 bit mask to clear.
   * @param ec Error code to set if an error occurs.
   */
  void clear_pins(Port port, uint8_t mask, std::error_code &ec) {
    auto addr = port == Port::PORT0 ? Registers::OUTPORT0 : Registers::OUTPORT1;
    auto data = read_u8_from_register((uint8_t)addr, ec);
    if (ec)
      return;
    data &= ~mask;
    write_u8_to_register((uint8_t)addr, data, ec);
  }

  /**
   * @brief Clear the pin values for Port 0 and Port 1 according to the provided masks.
   * @details Reads the current pin values and clears any bits set in the masks.
   * @param p0 The pin values as an 8 bit mask for Port 0 to clear.
   * @param p1 The pin values as an 8 bit mask for Port 1 to clear.
   * @param ec Error code to set if an error occurs.
   */
  void clear_pins(uint8_t p0, uint8_t p1, std::error_code &ec) {
    clear_pins(Port::PORT0, p0, ec);
    if (ec)
      return;
    clear_pins(Port::PORT1, p1, ec);
  }

  /**
   * @brief Clear the pin values for Port 0 and Port 1 according to the provided mask.
   * @details Reads the current pin values and clears any bits set in the mask.
   * @param mask The pin values as a 16 bit mask (P0_0 lsb, P1_7 msb) to clear.
   * @param ec Error code to set if an error occurs.
   */
  void clear_pins(uint16_t mask, std::error_code &ec) {
    clear_pins(Port::PORT0, mask & 0xFF, ec);
    if (ec)
      return;
    clear_pins(Port::PORT1, mask >> 8, ec);
  }

  /**
   * @brief Set the pin values on the provided port according to the provided mask.
   * @brief Reads the current pin values and sets any bits set in the mask.
   * @param port The Port for which to set the pin outputs.
   * @param mask The pin values as an 8 bit mask to set.
   * @param ec Error code to set if an error occurs.
   */
  void set_pins(Port port, uint8_t mask, std::error_code &ec) {
    auto addr = port == Port::PORT0 ? Registers::OUTPORT0 : Registers::OUTPORT1;
    auto data = read_u8_from_register((uint8_t)addr, ec);
    if (ec)
      return;
    data |= mask;
    write_u8_to_register((uint8_t)addr, data, ec);
  }

  /**
   * @brief Set the pin values for Port 0 and Port 1 according to the provided masks.
   * @details Reads the current pin values and sets any bits set in the masks.
   * @param p0 The pin values for Port 0 as an 8 bit mask to set.
   * @param p1 The pin values for Port 1 as an 8 bit mask to set.
   * @param ec Error code to set if an error occurs.
   */
  void set_pins(uint8_t p0, uint8_t p1, std::error_code &ec) {
    set_pins(Port::PORT0, p0, ec);
    if (ec)
      return;
    set_pins(Port::PORT1, p1, ec);
  }

  /**
   * @brief Set the pin values for Port 0 and Port 1 according to the provided mask.
   * @details Reads the current pin values and sets any bits set in the mask.
   * @param mask The pin values as a 16 bit mask (P0_0 lsb, P1_7 msb) to set.
   * @param ec Error code to set if an error occurs.
   */
  void set_pins(uint16_t mask, std::error_code &ec) {
    set_pins(Port::PORT0, mask & 0xFF, ec);
    if (ec)
      return;
    set_pins(Port::PORT1, mask >> 8, ec);
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
    uint16_t p0 = read_u8_from_register((uint8_t)Registers::OUTPORT0, ec);
    if (ec)
      return 0;
    uint16_t p1 = read_u8_from_register((uint8_t)Registers::OUTPORT1, ec);
    if (ec)
      return 0;
    return (p1 << 8) | p0;
  }

  /**
   * @brief Configure the provided pins to interrupt on change.
   * @param port The port associated with the provided pin mask.
   * @param mask The pin mask to configure for interrupt (0=interrupt).
   * @param ec Error code to set if an error occurs.
   */
  void set_interrupt(Port port, uint8_t mask, std::error_code &ec) {
    logger_.debug("Setting interrupt on change for Port {} pins {}", (uint8_t)port, mask);
    auto addr = port == Port::PORT0 ? Registers::INTPORT0 : Registers::INTPORT1;
    write_u8_to_register((uint8_t)addr, mask, ec);
  }

  /**
   * @brief Configure the provided pins to interrupt on change.
   * @param p0 The pin mask for Port 0 to configure for interrupt (0=interrupt).
   * @param p1 The pin mask for Port 1 to configure for interrupt (0=interrupt).
   * @param ec Error code to set if an error occurs.
   */
  void set_interrupt(uint8_t p0, uint8_t p1, std::error_code &ec) {
    logger_.debug("Setting interrupt on change p0:{}, p1:{}", p0, p1);
    auto addr = Registers::INTPORT0;
    const uint8_t data[] = {p0, p1};
    write_many_to_register((uint8_t)addr, data, 2, ec);
  }

  /**
   * @brief Set the i/o direction for the pins according to mask.
   * @param port The port associated with the provided pin mask.
   * @param mask The mask indicating direction (1 = input, 0 = output)
   * @param ec Error code to set if an error occurs.
   */
  void set_direction(Port port, uint8_t mask, std::error_code &ec) {
    logger_.debug("Setting direction for Port {} to {}", (uint8_t)port, mask);
    auto addr = port == Port::PORT0 ? Registers::DIRPORT0 : Registers::DIRPORT1;
    write_u8_to_register((uint8_t)addr, mask, ec);
  }

  /**
   * @brief Set the i/o direction for the pins on Port 0 and Port 1.
   * @param p0 The mask for Port 0 indicating direction (1 = input, 0 = output)
   * @param p1 The mask for Port 1 indicating direction (1 = input, 0 = output)
   * @param ec Error code to set if an error occurs.
   */
  void set_direction(uint8_t p0, uint8_t p1, std::error_code &ec) {
    logger_.debug("Setting direction  p0:{}, p1:{}", p0, p1);
    auto addr = Registers::DIRPORT0;
    const uint8_t data[] = {p0, p1};
    write_many_to_register((uint8_t)addr, data, 2, ec);
  }

  /**
   * @brief Enable/disable the LED function on the associated port pins.
   * @param port The port associated with the provided pin mask.
   * @param mask The mask indicating LED function (1 = GPIO, 0 = LED)
   * @param ec Error code to set if an error occurs.
   */
  void configure_led(Port port, uint8_t mask, std::error_code &ec) {
    logger_.debug("Configuring LED function for Port {} to {}", (uint8_t)port, mask);
    auto addr = port == Port::PORT0 ? Registers::LEDMODE0 : Registers::LEDMODE1;
    write_u8_to_register((uint8_t)addr, mask, ec);
  }

  /**
   * @brief Enable/disable the LED function on the associated port pins.
   * @param p0 The mask for Port 0 indicating LED function (1 = GPIO, 0 = LED)
   * @param p1 The mask for Port 1 indicating LED function (1 = GPIO, 0 = LED)
   * @param ec Error code to set if an error occurs.
   */
  void configure_led(uint8_t p0, uint8_t p1, std::error_code &ec) {
    logger_.debug("Configuring LED function p0:{}, p1:{}", p0, p1);
    auto addr = Registers::LEDMODE0;
    const uint8_t data[] = {p0, p1};
    write_many_to_register((uint8_t)addr, data, 2, ec);
  }

  /**
   * @brief Enable/disable the LED function on the associated port pins.
   * @param mask The bit mask for Port 0 and Port 1 [(Port 1 << 8) | (Port 0)]
   *        indicating LED function (1 = GPIO, 0 = LED)
   * @param ec Error code to set if an error occurs.
   */
  void configure_led(uint16_t mask, std::error_code &ec) {
    uint8_t p0 = mask & 0xFF;
    uint8_t p1 = (mask >> 8) & 0xFF;
    logger_.debug("Configuring LED function p0:{}, p1:{}", p0, p1);
    auto addr = Registers::LEDMODE0;
    const uint8_t data[] = {p0, p1};
    write_many_to_register((uint8_t)addr, data, 2, ec);
  }

  /**
   * @brief Set the pin's LED to new brightness value.
   * @param pin The pin bit corresponding to the pin (P0_0 lsb, P1_7 msb).
   * @param brightness The brightness value [0,255] to set the LED to.
   * @param ec Error code to set if an error occurs.
   */
  void led(uint16_t pin, uint8_t brightness, std::error_code &ec) {
    logger_.debug("Setting LED brightness for pin {} to {}", pin, brightness);
    uint8_t addr = 0;
    switch (pin) {
    case 1: // P0_0
      addr = (uint8_t)Registers::DIM0_0;
      break;
    case 2: // P0_1
      addr = (uint8_t)Registers::DIM0_1;
      break;
    case 4: // P0_2
      addr = (uint8_t)Registers::DIM0_2;
      break;
    case 8: // P0_3
      addr = (uint8_t)Registers::DIM0_3;
      break;
    case 16: // P0_4
      addr = (uint8_t)Registers::DIM0_4;
      break;
    case 32: // P0_5
      addr = (uint8_t)Registers::DIM0_5;
      break;
    case 64: // P0_6
      addr = (uint8_t)Registers::DIM0_6;
      break;
    case 128: // P0_7
      addr = (uint8_t)Registers::DIM0_7;
      break;
    case 256: // P1_0
      addr = (uint8_t)Registers::DIM1_0;
      break;
    case 512: // P1_1
      addr = (uint8_t)Registers::DIM1_1;
      break;
    case 1024: // P1_2
      addr = (uint8_t)Registers::DIM1_2;
      break;
    case 2048: // P1_3
      addr = (uint8_t)Registers::DIM1_3;
      break;
    case 4096: // P1_4
      addr = (uint8_t)Registers::DIM1_4;
      break;
    case 8192: // P1_5
      addr = (uint8_t)Registers::DIM1_5;
      break;
    case 16384: // P1_6
      addr = (uint8_t)Registers::DIM1_6;
      break;
    case 32768: // P1_7
      addr = (uint8_t)Registers::DIM1_7;
      break;
    default:
      return; // bad mask, don't do anything!
      break;
    }
    write_u8_to_register(addr, brightness, ec);
  }

  /**
   * @brief Configure the global control register.
   * @param output_drive_mode_p0 Output drive mode for Port 0.
   * @param max_led_current Maximum LED current for each LED pin.
   * @param ec Error code to set if an error occurs.
   */
  void configure_global_control(OutputDriveModeP0 output_drive_mode_p0,
                                MaxLedCurrent max_led_current, std::error_code &ec) {
    logger_.debug("Configuring output drive for port 0 to {}", (uint8_t)output_drive_mode_p0);
    logger_.debug("Configuring max led current to {}", (uint8_t)max_led_current);
    auto addr = Registers::GLOBALCTRL;
    // all other bits must be set to 0 or system error may occur (per
    // datasheet).
    uint8_t data = 0;
    data |= ((uint8_t)output_drive_mode_p0) << (int)ControlBit::GPOMD;
    data |= ((uint8_t)max_led_current) << (int)ControlBit::ISEL;
    write_u8_to_register((uint8_t)addr, data, ec);
  }

protected:
  /**
   * @brief Register map for the MT23X17
   */
  enum class Registers : uint8_t {
    INPORT0 = 0x00,    ///< Read input values for Port 0
    INPORT1 = 0x01,    ///< Read input values for Port 1
    OUTPORT0 = 0x02,   ///< Write output values for Port 0
    OUTPORT1 = 0x03,   ///< Write output values for Port 1
    DIRPORT0 = 0x04,   ///< Write direction values for Port 0 (1=input, 0=output, default=0)
    DIRPORT1 = 0x05,   ///< Write direction values for Port 1 (1=input, 0=output, default=0)
    INTPORT0 = 0x06,   ///< Interrupt enable for Port 0 (1=disable, 0=enable, default=0)
    INTPORT1 = 0x07,   ///< Interrupt enable for Port 1 (1=disable, 0=enable, default=0)
    ID = 0x10,         ///< ID register, read only (value = 0x23)
    GLOBALCTRL = 0x11, ///< Global control for P0 output drive mode and LED current select
    LEDMODE0 = 0x12,   ///< Configure Port 0 as LED or GPIO (1=GPIO, 0=LED, default=0xff)
    LEDMODE1 = 0x13,   ///< Configure Port 1 as LED or GPIO (1=GPIO, 0=LED, default=0xff)
    DIM1_0 = 0x20,     ///< Register for controlling P1_0 in LED mode (default=0)
    DIM1_1 = 0x21,     ///< Register for controlling P1_1 in LED mode (default=0)
    DIM1_2 = 0x22,     ///< Register for controlling P1_2 in LED mode (default=0)
    DIM1_3 = 0x23,     ///< Register for controlling P1_3 in LED mode (default=0)
    DIM0_0 = 0x24,     ///< Register for controlling P0_0 in LED mode (default=0)
    DIM0_1 = 0x25,     ///< Register for controlling P0_1 in LED mode (default=0)
    DIM0_2 = 0x26,     ///< Register for controlling P0_2 in LED mode (default=0)
    DIM0_3 = 0x27,     ///< Register for controlling P0_3 in LED mode (default=0)
    DIM0_4 = 0x28,     ///< Register for controlling P0_4 in LED mode (default=0)
    DIM0_5 = 0x29,     ///< Register for controlling P0_5 in LED mode (default=0)
    DIM0_6 = 0x2A,     ///< Register for controlling P0_6 in LED mode (default=0)
    DIM0_7 = 0x2B,     ///< Register for controlling P0_7 in LED mode (default=0)
    DIM1_4 = 0x2C,     ///< Register for controlling P1_4 in LED mode (default=0)
    DIM1_5 = 0x2D,     ///< Register for controlling P1_5 in LED mode (default=0)
    DIM1_6 = 0x2E,     ///< Register for controlling P1_6 in LED mode (default=0)
    DIM1_7 = 0x2F,     ///< Register for controlling P1_7 in LED mode (default=0)
    SOFTRESET = 0x7F,  ///< Write 0x00 to this register to trigger a software reset.
  };

  enum class ControlBit : int {
    GPOMD = 4, ///< [bit 4] Set P0 output drive mode (1=push-pull, 0=open-drain, default=0)
    ISEL = 0,  ///< [bits [1:0]] Set max drive current of LED (b11=imax/4, b10=imax/2, b01=imax*3/4,
               ///< b00=imax, default=b00)
  };

  void init(const Config &config, std::error_code &ec) {
    configure_global_control(config.output_drive_mode_p0, config.max_led_current, ec);
    if (ec)
      return;
    set_direction(Port::PORT0, config.port_0_direction_mask, ec);
    if (ec)
      return;
    set_direction(Port::PORT1, config.port_1_direction_mask, ec);
    if (ec)
      return;
    set_interrupt(Port::PORT0, config.port_0_interrupt_mask, ec);
    if (ec)
      return;
    set_interrupt(Port::PORT1, config.port_1_interrupt_mask, ec);
  }

  Config config_;
};
} // namespace espp

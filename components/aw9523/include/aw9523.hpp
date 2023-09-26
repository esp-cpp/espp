#pragma once

#include <functional>

#include "logger.hpp"

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
class Aw9523 {
public:
  static constexpr uint8_t DEFAULT_ADDRESS = 0x58; ///< Lower 2 bits are AD1, AD0 pins on the chip.

  /**
   * @brief Function to write bytes to the device.
   * @param dev_addr Address of the device to write to.
   * @param data Pointer to array of bytes to write.
   * @param data_len Number of data bytes to write.
   */
  typedef std::function<void(uint8_t dev_addr, uint8_t *data, size_t data_len)> write_fn;

  /**
   * @brief Function to read bytes from the device.
   * @param dev_addr Address of the device to write to.
   * @param reg_addr Register address to read from.
   * @param data Pointer to array of bytes to read into.
   * @param data_len Number of data bytes to read.
   */
  typedef std::function<void(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t data_len)>
      read_fn;

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
    PUSH_PUSH = 1,  ///< In this mode it needs no pull-up resistor.
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
        OutputDriveModeP0::OPEN_DRAIN;                    ///< Output drive mode for Port 0
    MaxLedCurrent max_led_current = MaxLedCurrent::IMAX;  ///< Max current allowed on each LED.
    write_fn write;                                       ///< Function to write to the device.
    read_fn read;                                         ///< Function to read from the device.
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log verbosity for the component.
  };

  /**
   * @brief Construct the Aw9523. Initialization called separately. 
   * @param config Config structure for configuring the AW9523
   */
  Aw9523(const Config &config)
      : config_(config), address_(config.device_address), write_(config.write), read_(config.read),
        logger_({.tag = "Aw9523", .level = config.log_level}) {
  }

   /**
   * @brief Initialize the component class. 
   */
  void initialize(void) {
    init(config_);
  }

  /**
   * @brief Read the pin values on the provided port.
   * @param port The Port for which to read the pins
   * @return The pin values as an 8 bit mask.
   */
  uint8_t get_pins(Port port) {
    auto addr = port == Port::PORT0 ? Registers::INPORT0 : Registers::INPORT1;
    return read_one_((uint8_t)addr);
  }

  /**
   * @brief Read the pin values on both Port 0 and Port 1.
   * @return The pin values as a 16 bit mask (P0_0 lsb, P1_7 msb).
   */
  uint16_t get_pins() {
    return (read_one_((uint8_t)Registers::INPORT1) << 8) | read_one_((uint8_t)Registers::INPORT0);
    // TODO: this should work as well, but doesn't seem to (only the first
    //       byte read seems to be correct...)
    // return read_two_((uint8_t)Registers::INPORT0);
  }

  /**
   * @brief Write the pin values on the provided port.
   * @param port The Port for which to write the pins
   * @param value The pin values to apply.
   * @note This will overwrite any previous pin values on the port for all
   *       output pins on the port.
   */
  void output(Port port, uint8_t value) {
    auto addr = port == Port::PORT0 ? Registers::OUTPORT0 : Registers::OUTPORT1;
    write_one_((uint8_t)addr, value);
  }

  /**
   * @brief Write the pin values on both Port 0 and Port 1.
   * @param p0 The pin values to apply to Port 0.
   * @param p1 The pin values to apply to Port 1.
   * @note This will overwrite any previous pin values on the port for all
   *       output pins on the ports.
   */
  void output(uint8_t p0, uint8_t p1) {
    output(Port::PORT0, p0);
    output(Port::PORT1, p1);
  }

  /**
   * @brief Write the pin values on both Port 0 and Port 1.
   * @param value The pin values to apply as a 16 bit value (P0_0 lsb, P1_7 msb).
   * @note This will overwrite any previous pin values on the port for all
   *       output pins on the ports.
   */
  void output(uint16_t value) {
    output(Port::PORT0, value & 0xFF);
    output(Port::PORT1, value >> 8);
  }

  /**
   * @brief Clear the pin values on the provided port according to the provided mask.
   * @details Reads the current pin values and clears any bits set in the mask.
   * @param port The Port for which to clear the pin outputs.
   * @param mask The pin values as an 8 bit mask to clear.
   */
  void clear_pins(Port port, uint8_t mask) {
    auto addr = port == Port::PORT0 ? Registers::OUTPORT0 : Registers::OUTPORT1;
    auto data = read_one_((uint8_t)addr);
    data &= ~mask;
    write_one_((uint8_t)addr, data);
  }

  /**
   * @brief Clear the pin values for Port 0 and Port 1 according to the provided masks.
   * @details Reads the current pin values and clears any bits set in the masks.
   * @param p0 The pin values as an 8 bit mask for Port 0 to clear.
   * @param p1 The pin values as an 8 bit mask for Port 1 to clear.
   */
  void clear_pins(uint8_t p0, uint8_t p1) {
    clear_pins(Port::PORT0, p0);
    clear_pins(Port::PORT1, p1);
  }

  /**
   * @brief Clear the pin values for Port 0 and Port 1 according to the provided mask.
   * @details Reads the current pin values and clears any bits set in the mask.
   * @param mask The pin values as a 16 bit mask (P0_0 lsb, P1_7 msb) to clear.
   */
  void clear_pins(uint16_t mask) {
    clear_pins(Port::PORT0, mask & 0xFF);
    clear_pins(Port::PORT1, mask >> 8);
  }

  /**
   * @brief Set the pin values on the provided port according to the provided mask.
   * @brief Reads the current pin values and sets any bits set in the mask.
   * @param port The Port for which to set the pin outputs.
   * @param mask The pin values as an 8 bit mask to set.
   */
  void set_pins(Port port, uint8_t mask) {
    auto addr = port == Port::PORT0 ? Registers::OUTPORT0 : Registers::OUTPORT1;
    auto data = read_one_((uint8_t)addr);
    data |= mask;
    write_one_((uint8_t)addr, data);
  }

  /**
   * @brief Set the pin values for Port 0 and Port 1 according to the provided masks.
   * @details Reads the current pin values and sets any bits set in the masks.
   * @param p0 The pin values for Port 0 as an 8 bit mask to set.
   * @param p1 The pin values for Port 1 as an 8 bit mask to set.
   */
  void set_pins(uint8_t p0, uint8_t p1) {
    set_pins(Port::PORT0, p0);
    set_pins(Port::PORT1, p1);
  }

  /**
   * @brief Set the pin values for Port 0 and Port 1 according to the provided mask.
   * @details Reads the current pin values and sets any bits set in the mask.
   * @param mask The pin values as a 16 bit mask (P0_0 lsb, P1_7 msb) to set.
   */
  void set_pins(uint16_t mask) {
    set_pins(Port::PORT0, mask & 0xFF);
    set_pins(Port::PORT1, mask >> 8);
  }

    /**
   * @brief Read the output pin values on the provided port.
   * @param port The Port for which to read the pins
   * @return The pin values as an 8 bit mask.
   */
  uint8_t get_output(Port port) {
    auto addr = port == Port::PORT0 ? Registers::OUTPORT0 : Registers::OUTPORT1;
    return read_one_((uint8_t)addr);
  }

  /**
   * @brief Read the output pin values on both Port 0 and Port 1.
   * @return The pin values as a 16 bit mask (P0_0 lsb, P1_7 msb).
   */
  uint16_t get_output() {
    return (read_one_((uint8_t)Registers::OUTPORT1) << 8) | read_one_((uint8_t)Registers::OUTPORT0);
    // TODO: this should work as well, but doesn't seem to (only the first
    //       byte read seems to be correct...)
    // return read_two_((uint8_t)Registers::OUTPORT0);
  }

  /**
   * @brief Configure the provided pins to interrupt on change.
   * @param port The port associated with the provided pin mask.
   * @param mask The pin mask to configure for interrupt (0=interrupt).
   */
  void set_interrupt(Port port, uint8_t mask) {
    logger_.debug("Setting interrupt on change for Port {} pins {}", (uint8_t)port, mask);
    auto addr = port == Port::PORT0 ? Registers::INTPORT0 : Registers::INTPORT1;
    write_one_((uint8_t)addr, mask);
  }

  /**
   * @brief Configure the provided pins to interrupt on change.
   * @param p0 The pin mask for Port 0 to configure for interrupt (0=interrupt).
   * @param p1 The pin mask for Port 1 to configure for interrupt (0=interrupt).
   */
  void set_interrupt(uint8_t p0, uint8_t p1) {
    logger_.debug("Setting interrupt on change p0:{}, p1:{}", p0, p1);
    auto addr = Registers::INTPORT0;
    uint8_t data[] = {p0, p1};
    write_many_((uint8_t)addr, data, 2);
  }

  /**
   * @brief Set the i/o direction for the pins according to mask.
   * @param port The port associated with the provided pin mask.
   * @param mask The mask indicating direction (1 = input, 0 = output)
   */
  void set_direction(Port port, uint8_t mask) {
    logger_.debug("Setting direction for Port {} to {}", (uint8_t)port, mask);
    auto addr = port == Port::PORT0 ? Registers::DIRPORT0 : Registers::DIRPORT1;
    write_one_((uint8_t)addr, mask);
  }

  /**
   * @brief Set the i/o direction for the pins on Port 0 and Port 1.
   * @param p0 The mask for Port 0 indicating direction (1 = input, 0 = output)
   * @param p1 The mask for Port 1 indicating direction (1 = input, 0 = output)
   */
  void set_direction(uint8_t p0, uint8_t p1) {
    logger_.debug("Setting direction  p0:{}, p1:{}", p0, p1);
    auto addr = Registers::DIRPORT0;
    uint8_t data[] = {p0, p1};
    write_many_((uint8_t)addr, data, 2);
  }

  /**
   * @brief Enable/disable the LED function on the associated port pins.
   * @param port The port associated with the provided pin mask.
   * @param mask The mask indicating LED function (1 = GPIO, 0 = LED)
   */
  void configure_led(Port port, uint8_t mask) {
    logger_.debug("Configuring LED function for Port {} to {}", (uint8_t)port, mask);
    auto addr = port == Port::PORT0 ? Registers::LEDMODE0 : Registers::LEDMODE1;
    write_one_((uint8_t)addr, mask);
  }

  /**
   * @brief Enable/disable the LED function on the associated port pins.
   * @param p0 The mask for Port 0 indicating LED function (1 = GPIO, 0 = LED)
   * @param p1 The mask for Port 1 indicating LED function (1 = GPIO, 0 = LED)
   */
  void configure_led(uint8_t p0, uint8_t p1) {
    logger_.debug("Configuring LED function p0:{}, p1:{}", p0, p1);
    auto addr = Registers::LEDMODE0;
    uint8_t data[] = {p0, p1};
    write_many_((uint8_t)addr, data, 2);
  }

  /**
   * @brief Enable/disable the LED function on the associated port pins.
   * @param mask The bit mask for Port 0 and Port 1 [(Port 1 << 8) | (Port 0)]
   *        indicating LED function (1 = GPIO, 0 = LED)
   */
  void configure_led(uint16_t mask) {
    uint8_t p0 = mask & 0xFF;
    uint8_t p1 = (mask >> 8) & 0xFF;
    logger_.debug("Configuring LED function p0:{}, p1:{}", p0, p1);
    auto addr = Registers::LEDMODE0;
    uint8_t data[] = {p0, p1};
    write_many_((uint8_t)addr, data, 2);
  }

  /**
   * @brief Set the pin's LED to new brightness value.
   * @param pin The pin bit corresponding to the pin (P0_0 lsb, P1_7 msb).
   * @param brightness The brightness value [0,255] to set the LED to.
   */
  void led(uint16_t pin, uint8_t brightness) {
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
    write_one_(addr, brightness);
  }

  /**
   * @brief Configure the global control register.
   * @param output_drive_mode_p0 Output drive mode for Port 0.
   * @param max_led_current Maximum LED current for each LED pin.
   */
  void configure_global_control(OutputDriveModeP0 output_drive_mode_p0,
                                MaxLedCurrent max_led_current) {
    logger_.debug("Configuring output drive for port 0 to {}", (uint8_t)output_drive_mode_p0);
    logger_.debug("Configuring max led current to {}", (uint8_t)max_led_current);
    auto addr = Registers::GLOBALCTRL;
    // all other bits must be set to 0 or system error may occur (per
    // datasheet).
    uint8_t data = 0;
    data |= ((uint8_t)output_drive_mode_p0) << (int)ControlBit::GPOMD;
    data |= ((uint8_t)max_led_current) << (int)ControlBit::ISEL;
    write_one_((uint8_t)addr, data);
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

  

  void init(const Config &config) {
    configure_global_control(config.output_drive_mode_p0, config.max_led_current);
    set_direction(Port::PORT0, config.port_0_direction_mask);
    set_direction(Port::PORT1, config.port_1_direction_mask);
    set_interrupt(Port::PORT0, config.port_0_interrupt_mask);
    set_interrupt(Port::PORT1, config.port_1_interrupt_mask);
  }

  uint8_t read_one_(uint8_t reg_addr) {
    uint8_t data;
    read_(address_, reg_addr, &data, 1);
    return data;
  }

  uint16_t read_two_(uint8_t reg_addr) {
    uint8_t data[2];
    read_(address_, reg_addr, data, 2);
    return (data[1] << 8) | data[0];
  }

  void write_one_(uint8_t reg_addr, uint8_t data) { write_many_(reg_addr, &data, 1); }

  void write_many_(uint8_t reg_addr, uint8_t *write_data, size_t write_data_len) {
    uint8_t total_len = 1 + write_data_len;
    uint8_t data[total_len];
    data[0] = reg_addr;
    memcpy(&data[1], write_data, write_data_len);
    write_(address_, data, total_len);
  }

  Config config_;
  uint8_t address_;
  write_fn write_;
  read_fn read_;
  Logger logger_;
};
} // namespace espp

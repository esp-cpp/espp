#pragma once

#include <functional>

#include "logger.hpp"

namespace espp {
  /**
   * Class for communicating with and controlling a MCP23X17 (23017, 23S17) GPIO
   * expander including interrupt configuration.
   */
  class Mcp23x17 {
  public:
    static constexpr uint8_t ADDRESS = 0b0100000; ///< Lower 3 bits are A2, A2, A0 pins on the chip.

    /**
     * @brief Function to write a byte to a register for Mcp23x17.
     * @param reg_addr Register address to write to.
     * @param data Data to be written.
     */
    typedef std::function<void(uint8_t reg_addr, uint8_t data)> write_fn;

    /**
     * @brief Function to read a byte from a register for Mcp23x17.
     * @param reg_addr Register address to read from.
     * @return Byte read from register.
     */
    typedef std::function<uint8_t(uint8_t reg_addr)> read_fn;

    /**
     * The two GPIO ports the MCP23x17 has.
     */
    enum class Port {
      A, ///< Port A
      B  ///< Port B
    };

    /**
     * @brief Configuration information for the Mcp23x17.
     */
    struct Config {
      uint8_t port_a_direction_mask = 0x00; ///< Direction mask (1 = input) for port a.
      uint8_t port_a_interrupt_mask = 0x00; ///< Interrupt mask (1 = interrupt) for port a.
      uint8_t port_b_direction_mask = 0x00; ///< Direction mask (1 = input) for port b.
      uint8_t port_b_interrupt_mask = 0x00; ///< Interrupt mask (1 = interrupt) for port b.
      write_fn write; ///< Function to write to the device.
      read_fn read; ///< Function to read from the device.
      Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log verbosity for the component.
    };

    /**
     * @brief Construct the Mcp23x17 and configure it.
     * @param config Config structure for configuring the MCP23X17
     */
    Mcp23x17(const Config& config)

      : write_(config.write),
        read_(config.read),
        logger_({.tag = "Mcp23x17", .level = config.log_level}) {
      init(config);
    }

    /**
     * @brief Read the pin values on the provided port.
     * @param port The Port for which to read the pins
     * @return The pin values as an 8 bit mask.
     */
    uint8_t get_pins(Port port) {
      auto addr = port == Port::A ? Registers::GPIOA : Registers::GPIOB;
      return read_((uint8_t)addr);
    }

    /**
     * @brief Set the pin values on the provided port.
     * @param port The Port for which to set the pin outputs.
     * @param output The pin values as an 8 bit mask to set.
     */
    void set_pins(Port port, uint8_t output) {
      auto addr = port == Port::A ? Registers::GPIOA : Registers::GPIOB;
      write_((uint8_t)addr, output);
    }

    /**
     * @brief Get the pin state at the time of interrupt for the provided port.
     * @param port The port for which to get the pin states at time of capture.
     * @return An 8 bit mask of pin values for the provided port.
     */
    uint8_t get_interrupt_capture(Port port) {
      auto addr = port == Port::A ? Registers::INTCAPA : Registers::INTCAPB;
      return read_((uint8_t)addr);
    }

    /**
     * @brief Configure the provided pins to interrupt on change.
     * @param port The port associated with the provided pin mask.
     * @param mask The pin mask to configure for interrupt (1=interrupt).
     */
    void set_interrupt_on_change(Port port, uint8_t mask) {
      logger_.debug("Setting interrupt on change for {} pins {}", (uint8_t)port, mask);
      auto addr = port == Port::A ? Registers::GPINTENA : Registers::GPINTENB;
      write_((uint8_t)addr, mask);
    }

    /**
     * @brief Configure the provided pins to interrupt on value.
     * @param port The port associated with the provided pin mask.
     * @param pin_mask The pin mask to configure for interrupt (1=interrupt).
     * @param val_mask The value mask for the pin level to trigger interrupt.
     */
    void set_interrupt_on_value(Port port, uint8_t pin_mask, uint8_t val_mask) {
      logger_.debug("Setting interrupt on value for {} pins {} to {}", (uint8_t)port, pin_mask, val_mask);
      // set the pin to enable interrupt
      auto addr = port == Port::A ? Registers::GPINTENA : Registers::GPINTENB;
      write_((uint8_t)addr, pin_mask);

      // set the pin to interrupt on comparison to defval register
      addr = port == Port::A ? Registers::INTCONA : Registers::INTCONB;
      write_((uint8_t)addr, pin_mask);

      // set the defval register to be the value to compare against
      addr = port == Port::A ? Registers::DEFVALA : Registers::DEFVALB;
      write_((uint8_t)addr, val_mask);
    }

    /**
     * @brief Set the i/o direction for the pins according to mask.
     * @param port The port associated with the provided pin mask.
     * @param mask The mask indicating direction (1 = input)
     */
    void set_direction(Port port, uint8_t mask) {
      logger_.debug("Setting direction for {} to {}", (uint8_t)port, mask);
      auto addr = port == Port::A ? Registers::IODIRA : Registers::IODIRB;
      write_((uint8_t)addr, mask);
    }

    /**
     * @brief Set the input polarity for the pins according to mask.
     * @param port The port associated with the provided polarity mask.
     * @param mask Polarity mask for the pins, 1 -> invert the pin value.
     */
    void set_input_polarity(Port port, uint8_t mask) {
      logger_.debug("Setting input polarity for {} to {}", (uint8_t)port, mask);
      auto addr = port == Port::A ? Registers::IOPOLA : Registers::IOPOLB;
      write_((uint8_t)addr, mask);
    }

    /**
     * @brief Set the internal pull up (100 Kohm) for the port's pins.
     * @param port The port associated with the provided pull up mask.
     * @param mask Mask indicating which pins should be pulled up (1).
     */
    void set_pull_up(Port port, uint8_t mask) {
      logger_.debug("Setting pull-up for {} to {}", (uint8_t)port, mask);
      auto addr = port == Port::A ? Registers::GPPUA : Registers::GPPUB;
      write_((uint8_t)addr, mask);
    }

    /**
     * @brief Configure the interrupt mirroring for the MCP23x17.
     * @param mirror True if the interrupt pins should be internally connected,
     *        false otherwise.
     */
    void set_interrupt_mirror(bool mirror) {
      logger_.debug("Setting interrupt mirror: {}", mirror);
      auto addr = (uint8_t)Registers::IOCON;
      auto config = read_(addr);
      logger_.debug("Read config: {}", config);
      if (mirror) {
        config &= (1 << (int)ConfigBit::MIRROR);
      } else {
        config ^= (1 << (int)ConfigBit::MIRROR);
      }
      // now write it back
      logger_.debug("Writing new config: {}", config);
      write_(addr, config);
    }

    /**
     * @brief Set the polarity of the interrupt pins.
     * @param active_high True if it should be active-high, false for
     *        active-low (default).
     */
    void set_interrupt_polarity(bool active_high) {
      logger_.debug("Setting interrupt polarity: {}", active_high);
      auto addr = (uint8_t)Registers::IOCON;
      auto config = read_(addr);
      logger_.debug("Read config: {}", config);
      if (active_high) {
        config &= (1 << (int)ConfigBit::INTPOL);
      } else {
        config ^= (1 << (int)ConfigBit::INTPOL);
      }
      // now write it back
      logger_.debug("Writing new config: {}", config);
      write_(addr, config);
    }

  protected:
    /**
     * @brief Register map for the MT23X17
     */
    enum class Registers : uint8_t {
      IODIRA   = 0x00, ///< Direction (port A) 1 = input, 0 = output
      IODIRB   = 0x01, ///< Direction (port B) 1 = input, 0 = output
      IOPOLA   = 0x02, ///< Input polarity (port A) 1 = invert pin value
      IOPOLB   = 0x03, ///< Input polarity (port B) 1 = invert pin value
      GPINTENA = 0x04, ///< Interrupt on change control (port A) 1 enables GPIO input for interrupt on change event
      GPINTENB = 0x05, ///< Interrupt on change control (port B) 1 enables GPIO input for interrupt on change event
      DEFVALA  = 0x06, ///< Default compare register for interrupt on change (port A) - if enabled via GPINTEN and INTCON to compare against DEFVAL, an opposite value on the associated pin will cause an interrupt to occur.
      DEFVALB  = 0x07, ///< Default compare register for interrupt on change (port B) - if enabled via GPINTEN and INTCON to compare against DEFVAL, an opposite value on the associated pin will cause an interrupt to occur.
      INTCONA  = 0x08, ///< How the pin value is compared for interrupt on chage (port A) - 1 = compare against DEFVAL, 0 = compare against previous.
      INTCONB  = 0x09, ///< How the pin value is compared for interrupt on chage (port B) - 1 = compare against DEFVAL, 0 = compare against previous.
      IOCON    = 0x0A, ///< Bit mask of control flags for configuring the device. BANK | MIRROR | SEQOP | DISSLW | HAEN | ODR | INTPOL | ---
      IOCON_0  = 0x0B, ///< Same as IOCON
      GPPUA    = 0x0C, ///< GPIO Pull Up (port A) - 1 = internal pull up with 100kOhm
      GPPUB    = 0x0D, ///< GPIO Pull Up (port B) - 1 = internal pull up with 100kOhm
      INTFA    = 0x0E, ///< Interrupt flag (port A) Indicates which pin caused the interrupt
      INTFB    = 0x0F, ///< Interrupt flag (port B) Indicates which pin caused the interrupt
      INTCAPA  = 0x10, ///< Interrupt capture (port A) - state of the pins when the interrupt occurred; remains until interrupt cleared via reading INTCAP or GPIO
      INTCAPB  = 0x11, ///< Interrupt capture (port B) - state of the pins when the interrupt occurred; remains until interrupt cleared via reading INTCAP or GPIO
      GPIOA    = 0x12, ///< GPIO register (port A) - value of the port
      GPIOB    = 0x13, ///< GPIO register (port B) - value of the port
      OLATA    = 0x14, ///< Output latch register (port A)
      OLATB    = 0x15, ///< Output latch register (port B)
    };

    enum class ConfigBit : int {
      INTPOL = 1, ///< Polarity of the input pin (0 (default) = active low, 1 = active high)
      ODR    = 2, ///< Configures INT pin as open-drain output (1 = open drain, 0 = active driver)
      HAEN   = 3, ///< Hardware address enable (MCP23S17 only) (1 = enabled)
      DISSLW = 4, ///< Slew rate control for SDA output (1 = disabled)
      SEQOP  = 5, ///< 0 (default) = address pointer increments
      MIRROR = 6, ///< 0 (default) = INT pins are not connected; 1 = INT pins are internally connected.
      BANK   = 7, ///< 0 (default) = registers are in same bank / sequential addresses; 1 = registers associated with each port are separated into different banks
    };

    void init(const Config& config) {
      set_direction(Port::A, config.port_a_direction_mask);
      set_direction(Port::B, config.port_b_direction_mask);
      set_interrupt_on_change(Port::A, config.port_a_interrupt_mask);
      set_interrupt_on_change(Port::B, config.port_b_interrupt_mask);
    }

    write_fn write_;
    read_fn read_;
    Logger logger_;
  };
}

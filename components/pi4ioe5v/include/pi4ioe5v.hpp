#pragma once

#include <cstdint>
#include <functional>

#include "base_peripheral.hpp"

namespace espp {
/**
 * @brief PI4IOE5V I2C GPIO Expander driver.
 *
 * Class for communicating with and controlling a PI4IOE5V GPIO expander.
 * The PI4IOE5V is an 8-bit I2C GPIO expander with interrupt capability
 * and configurable pull-up/pull-down resistors.
 * It operates at [1.65V, 5.5V] and supports I2C Fast-mode (400kHz) and
 * Fast-mode Plus (1MHz).
 *
 * The register layout used here matches the PI4IOE5V6408 datasheet.
 *
 * Datasheet for the PI4IOE5V can be found here:
 * https://www.diodes.com/datasheet/download/PI4IOE5V6408.pdf
 *
 * @section pi4ioe5v_example Example
 * @snippet pi4ioe5v_example.cpp pi4ioe5v_example
 */
class Pi4ioe5v : public BasePeripheral<> {
public:
  static constexpr uint8_t ADDRESS_LOW = 0x43;            ///< Address with addr pin low
  static constexpr uint8_t ADDRESS_HIGH = 0x44;           ///< Address with addr pin high
  static constexpr uint8_t DEFAULT_ADDRESS = ADDRESS_LOW; ///< Default I2C address

  /**
   * The Pull Resistor configuration.
   */
  enum class PullResistor : uint8_t {
    NO_PULL = 0,   ///< No pull resistor enabled
    PULL_UP = 1,   ///< Pull-up resistor enabled
    PULL_DOWN = 2, ///< Pull-down resistor enabled
  };

  /**
   * @brief Configuration information for the Pi4ioe5v.
   */
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS; ///< I2C address to use to talk to this PI4IOE5V.
    uint8_t direction_mask = 0x00;            ///< Direction mask (0 = input, 1 = output).
    uint8_t interrupt_mask =
        0x00; ///< Interrupt mask (0 = enabled interrupt, 1 = disable interrupt).
    uint8_t input_defaults = 0x00; ///< Default input values for input interrupts. A change from
                                   ///< this level will trigger an interrupt.
    std::optional<uint8_t> initial_output =
        std::nullopt; ///< Initial output value (only for pins set as
                      ///< outputs). If not set, outputs will be low or high-z depending on
                      ///< high_z_mask.
    std::optional<uint8_t> high_z_mask =
        std::nullopt; ///< Mask of pins to set as high impedance outputs. Overrides initial_output
                      ///< for these pins. All pins default to high-z on power-up.
    std::optional<uint8_t> pull_up_mask =
        std::nullopt; ///< Mask of pins to enable pull-up resistors on.
    std::optional<uint8_t> pull_down_mask =
        std::nullopt;               ///< Mask of pins to enable pull-down resistors on.
    BasePeripheral::write_fn write; ///< Function to write to the device.
    BasePeripheral::write_then_read_fn
        write_then_read;   ///< Function to write then read from the device.
    bool auto_init = true; ///< Automatically initialize the device.
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log verbosity for the component.
  };

  /**
   * @brief Construct the Pi4ioe5v. Will call initialize() if auto_init is true.
   * @param config Configuration structure for configuring the PI4IOE5V
   */
  explicit Pi4ioe5v(const Config &config)
      : BasePeripheral({.address = config.device_address,
                        .write = config.write,
                        .write_then_read = config.write_then_read},
                       "Pi4ioe5v", config.log_level)
      , config_(config) {
    if (config.auto_init) {
      std::error_code ec;
      if (!initialize(ec)) {
        logger_.error("Failed to initialize PI4IOE5V: {}", ec.message());
      }
    }
  }

  /**
   * @brief Initialize the component class.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool initialize(std::error_code &ec) { return init(config_, ec); }

  /**
   * @brief Read the pin values.
   * @param ec Error code to set if an error occurs.
   * @return The pin values as an 8 bit mask.
   */
  uint8_t get_pins(std::error_code &ec) {
    return read_u8_from_register((uint8_t)Registers::INPUT, ec);
  }

  /**
   * @brief Write the pin values.
   * @param value The pin values to apply.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   * @note This will overwrite any previous pin values for all output pins.
   * @note Only affects pins configured as outputs.
   * @note Pins configured as high-z will remain high-z.
   * @note All pins default to high-z on power-up.
   */
  bool output(uint8_t value, std::error_code &ec) {
    write_u8_to_register((uint8_t)Registers::OUTPUT, value, ec);
    return !ec;
  }

  /**
   * @brief Clear the pin values according to the provided mask.
   * @details Reads the current pin values and clears any bits set in the mask.
   * @param mask The pin values as an 8 bit mask to clear.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool clear_pins(uint8_t mask, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    auto data = read_u8_from_register((uint8_t)Registers::OUTPUT, ec);
    if (ec)
      return false;
    data &= ~mask;
    write_u8_to_register((uint8_t)Registers::OUTPUT, data, ec);
    return !ec;
  }

  /**
   * @brief Set the pin values according to the provided value.
   * @details Reads the current pin values and sets any bits set in the mask.
   * @param value The pin values as an 8 bit mask to set.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   * @note Only affects pins configured as outputs.
   */
  bool set_pins(uint8_t value, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    auto data = read_u8_from_register((uint8_t)Registers::OUTPUT, ec);
    if (ec)
      return false;
    data |= value;
    write_u8_to_register((uint8_t)Registers::OUTPUT, data, ec);
    return !ec;
  }

  /**
   * @brief Read the output pin values.
   * @param ec Error code to set if an error occurs.
   * @return The pin values as an 8 bit mask.
   */
  uint8_t get_output(std::error_code &ec) {
    return read_u8_from_register((uint8_t)Registers::OUTPUT, ec);
  }

  /**
   * @brief Read the input pin values.
   * @param ec Error code to set if an error occurs.
   * @return The pin values as an 8 bit mask.
   */
  uint8_t get_input(std::error_code &ec) {
    return read_u8_from_register((uint8_t)Registers::INPUT, ec);
  }

  /**
   * @brief Read the i/o direction for the pins.
   * @param ec Error code to set if an error occurs.
   * @return The direction as an 8 bit mask (1 = output, 0 = input).
   */
  uint8_t get_direction(std::error_code &ec) {
    return read_u8_from_register((uint8_t)Registers::DIRECTION, ec);
  }

  /**
   * @brief Set the i/o direction for the pins according to mask.
   * @param mask The mask indicating direction (1 = output, 0 = input)
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool set_direction(uint8_t mask, std::error_code &ec) {
    logger_.info("Setting direction (1=output) to {:#10b}", mask);
    write_u8_to_register((uint8_t)Registers::DIRECTION, mask, ec);
    return !ec;
  }

  /**
   * @brief Set the pull resistor for the provided pins.
   * @param pin_mask The pin mask to configure for pull resistor.
   * @param pull The pull resistor to configure.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool set_pull_resistor_for_pin(uint8_t pin_mask, PullResistor pull, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    // Configure pull enable
    auto enable_data = read_u8_from_register((uint8_t)Registers::PULL_ENABLE, ec);
    if (ec)
      return false;

    if (pull == PullResistor::NO_PULL) {
      logger_.info("Disabling pull-up/down for pin {:#10b}", pin_mask);
      enable_data &= ~pin_mask;
    } else {
      logger_.info("Enabling pull-up/down for pin {:#10b}", pin_mask);
      enable_data |= pin_mask;
    }

    write_u8_to_register((uint8_t)Registers::PULL_ENABLE, enable_data, ec);
    if (ec)
      return false;

    if (pull == PullResistor::NO_PULL) {
      return true;
    }

    // Configure pull select
    auto select_data = read_u8_from_register((uint8_t)Registers::PULL_SELECT, ec);
    if (ec)
      return false;

    if (pull == PullResistor::PULL_UP) {
      logger_.info("Setting pull-up for pin {:#10b}", pin_mask);
      select_data |= pin_mask;
    } else {
      logger_.info("Setting pull-down for pin {:#10b}", pin_mask);
      select_data &= ~pin_mask;
    }

    write_u8_to_register((uint8_t)Registers::PULL_SELECT, select_data, ec);
    return !ec;
  }

  /**
   * @brief Configure interrupt mask for the pins.
   * @param mask The pin mask to configure for interrupt (1 = disable interrupt, 0 = enable).
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool set_interrupt_mask(uint8_t mask, std::error_code &ec) {
    logger_.info("Setting interrupt mask (1=disabled) to {:#10b}", mask);
    write_u8_to_register((uint8_t)Registers::INT_MASK, mask, ec);
    return !ec;
  }

  /**
   * @brief Read interrupt status.
   * @param ec Error code to set if an error occurs.
   * @return The interrupt status as an 8 bit mask (1 = interrupt triggered).
   */
  uint8_t get_interrupt_status(std::error_code &ec) {
    logger_.info("Reading interrupt status");
    return read_u8_from_register((uint8_t)Registers::INT_STATUS, ec);
  }

  /**
   * @brief Set input default values for input interrupts.
   * @param value The default value for the pins. An opposite value on input
   *        pins will trigger an interrupt.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool set_input_default(uint8_t value, std::error_code &ec) {
    logger_.info("Setting input default to {:#10b}", value);
    write_u8_to_register((uint8_t)Registers::INPUT_DEFAULT, value, ec);
    return !ec;
  }

  /**
   * @brief Set output high impedance control.
   * @param mask The mask indicating which outputs should be high impedance (1 = high-Z).
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   */
  bool set_output_high_impedance(uint8_t mask, std::error_code &ec) {
    logger_.info("Setting output high impedance to {:#10b}", mask);
    write_u8_to_register((uint8_t)Registers::OUTPUT_HIGH_IM, mask, ec);
    return !ec;
  }

  /**
   * @brief Perform a software reset of the device.
   * @param ec Error code to set if an error occurs.
   * @return true if successful, false if an error occurred.
   * @note This will reset all registers to their default values.
   * @note This will cause the reset interrupt bit to be set in the CHIP_ID_CTRL
   *       register, which can be cleared by reading the register.
   */
  bool reset(std::error_code &ec) {
    logger_.info("Performing software reset");
    // Software reset is bit 0 of the CHIP_ID_CTRL register
    write_u8_to_register((uint8_t)Registers::CHIP_ID_CTRL, 0x01, ec);
    return !ec;
  }

  /**
   * @brief Read the manufacturer ID.
   * @param ec Error code to set if an error occurs.
   * @return The manufacturer ID.
   * @note The manufacturer ID is contained in bits 5,6,7 of the CHIP_ID_CTRL
   *       register. The expected value is 0b101 (Diodes Incorporated).
   * @note The manufacturer ID will be shifted to the least significant bits in
   *       the return value.
   * @note This read will also clear the reset interrupt bit if it was set.
   */
  uint8_t get_manufacturer_id(std::error_code &ec) {
    // read the chip_id register and return bits 5,6,7
    auto chip_id = read_u8_from_register((uint8_t)Registers::CHIP_ID_CTRL, ec);
    if (ec)
      return 0;
    return (chip_id >> 5) & 0x07;
  }

  /**
   * @brief Read the firmware revision.
   * @param ec Error code to set if an error occurs.
   * @return The firmware revision.
   * @note The firmware revision is contained in bits 2,3,4 of the CHIP_ID_CTRL
   *       register.
   * @note The firmware revision will be shifted to the least significant bits in
   *       the return value.
   * @note This read will also clear the reset interrupt bit if it was set.
   */
  uint8_t get_firmware_revision(std::error_code &ec) {
    // read the chip_id register and return bits 2,3,4
    auto chip_id = read_u8_from_register((uint8_t)Registers::CHIP_ID_CTRL, ec);
    if (ec)
      return 0;
    return (chip_id >> 2) & 0x07;
  }

  /**
   * @brief Read the chip ID.
   * @param ec Error code to set if an error occurs.
   * @return The chip ID.
   * @note This read will also clear the reset interrupt bit if it was set.
   */
  uint8_t get_chip_id(std::error_code &ec) {
    return read_u8_from_register((uint8_t)Registers::CHIP_ID_CTRL, ec);
  }

protected:
  /**
   * @brief Register map for the PI4IOE5V
   */
  enum class Registers : uint8_t {
    CHIP_ID_CTRL = 0x01,   ///< Chip ID and control
    DIRECTION = 0x03,      ///< Direction (0=input, 1=output), default is input
    OUTPUT = 0x05,         ///< Output latch (read/write)
    OUTPUT_HIGH_IM = 0x07, ///< Output high-impedance control (1 = high-Z)
    INPUT_DEFAULT = 0x09,  ///< Input default state (used if input and no pull)
    PULL_ENABLE = 0x0B,    ///< Pull enable (1=enabled, 0=disabled)
    PULL_SELECT = 0x0D,    ///< Pull select (1=pull-up, 0=pull-down)
    INPUT = 0x0F,          ///< Input values (read-only)
    INT_MASK = 0x11,       ///< Interrupt mask (1=disabled, 0=enabled)
    INT_STATUS = 0x13,     ///< Interrupt status (read-only)
  };

  /**
   * @brief Apply initial configuration to the device.
   * @param config Driver configuration
   * @param ec Error code set on failure
   * @return true on success, false on error
   */
  bool init(const Config &config, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    logger_.info("Initializing PI4IOE5V with address {:#04x}", config.device_address);

    // perform a reset to ensure we start from a known state
    if (!reset(ec))
      return false;

    // read chip id and log it
    auto chip_id = get_chip_id(ec);
    if (ec)
      return false;
    logger_.info("PI4IOE5V Chip ID: {:#04x}", chip_id);

    // Set the initial output value before setting the direction, to make sure
    // there are no glitches on the output pins
    if (config.initial_output.has_value()) {
      auto initial_value = config.initial_output.value();
      logger_.info("Setting initial output value to {:#10b}", initial_value);
      if (!output(initial_value, ec))
        return false;
    }

    // Set high impedance outputs
    if (config.high_z_mask.has_value()) {
      if (!set_output_high_impedance(config.high_z_mask.value(), ec))
        return false;
    }

    // Set pull-up resistors
    if (config.pull_up_mask.has_value()) {
      if (!set_pull_resistor_for_pin(config.pull_up_mask.value(), PullResistor::PULL_UP, ec))
        return false;
    }

    // Set pull-down resistors
    if (config.pull_down_mask.has_value()) {
      if (!set_pull_resistor_for_pin(config.pull_down_mask.value(), PullResistor::PULL_DOWN, ec))
        return false;
    }

    // Set direction
    if (!set_direction(config.direction_mask, ec))
      return false;

    // Set input defaults
    if (!set_input_default(config.input_defaults, ec))
      return false;

    // Set interrupt mask
    if (!set_interrupt_mask(config.interrupt_mask, ec))
      return false;

    return true;
  }

  Config config_;
};
} // namespace espp

// for printing of enums using libfmt
template <> struct fmt::formatter<espp::Pi4ioe5v::PullResistor> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::Pi4ioe5v::PullResistor &p, FormatContext &ctx) const {
    switch (p) {
    case espp::Pi4ioe5v::PullResistor::PULL_UP:
      return fmt::format_to(ctx.out(), "Pull Up");
    case espp::Pi4ioe5v::PullResistor::PULL_DOWN:
      return fmt::format_to(ctx.out(), "Pull Down");
    case espp::Pi4ioe5v::PullResistor::NO_PULL:
      return fmt::format_to(ctx.out(), "No Pull");
    default:
      return fmt::format_to(ctx.out(), "Unknown");
    }
  }
};

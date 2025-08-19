#pragma once

#include <bitset>
#include <cstdint>

#include "base_peripheral.hpp"

namespace espp {
/**
 * @brief PI4IOE5V I2C GPIO Expander family driver.
 *
 * Provides APIs to configure direction, set/clear/read GPIOs, invert input
 * polarity, and enable/configure pulls for PI4IOE5Vxxxx devices. Derives from
 * `espp::BasePeripheral<uint8_t, true>` (8-bit register addresses, addressed
 * device).
 *
 * The register layout used here matches common PI4IOE5V variants with two 8-bit
 * ports (total 16 GPIOs). If your variant differs (e.g., pull register
 * addresses), adjust the `Reg` map accordingly.
 *
 * @section pi4ioe5v_example Example
 * @snippet pi4ioe5v_example.cpp pi4ioe5v_example
 * This component is typically bound to an `espp::I2c` instance using
 * std::bind for the IO hooks. See the README for a short example.
 */
class Pi4ioe5v : public BasePeripheral<uint8_t, true> {
public:
  /// Default I2C address when address pins are tied low (variant dependent)
  static constexpr uint8_t DEFAULT_ADDRESS = 0x40;

  /// Register map (typical 8-GPIO variant: single 8-bit port)
  enum class Reg : uint8_t {
    INPUT = 0x00,        ///< Input values (read-only)
    OUTPUT = 0x01,       ///< Output latch (read/write)
    POLARITY_INV = 0x02, ///< Input polarity invert (1=invert)
    DIRECTION = 0x03,    ///< Direction (1=input, 0=output)
    PULL_ENABLE = 0x46,  ///< Pull enable (1=enabled) [variant-specific]
    PULL_SELECT = 0x47,  ///< Pull select (1=pull-up, 0=pull-down) [variant-specific]
  };

  /// Configuration structure for the PI4IOE5V driver
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS; ///< I2C device address
    /// Optional initial configuration for single 8-bit port (1=input, 0=output)
    uint8_t direction_mask = 0xFF;
    uint8_t output_state = 0x00;    ///< Initial output latch state
    uint8_t polarity_invert = 0x00; ///< Input invert mask (1=invert)
    uint8_t pull_enable = 0x00;     ///< Pull enable mask (1=enable)
    uint8_t pull_up_select = 0x00;  ///< Pull select mask (1=up, 0=down)
    // IO hooks
    BasePeripheral::probe_fn probe{nullptr};                     ///< Optional bus probe
    BasePeripheral::write_fn write{nullptr};                     ///< Write function
    BasePeripheral::read_register_fn read_register{nullptr};     ///< Read register function
    BasePeripheral::write_then_read_fn write_then_read{nullptr}; ///< Write-then-read function
    // Behavior
    bool auto_init = true;                                ///< Initialize in ctor
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log verbosity
  };

  /**
   * @brief Construct the PI4IOE5V expander.
   * @param cfg Driver configuration, including IO hooks and initial state.
   */
  explicit Pi4ioe5v(const Config &cfg)
      : BasePeripheral({.address = cfg.device_address,
                        .probe = cfg.probe,
                        .write = cfg.write,
                        .read_register = cfg.read_register,
                        .write_then_read = cfg.write_then_read},
                       "Pi4ioe5v", cfg.log_level)
      , config_(cfg) {
    if (cfg.auto_init) {
      std::error_code ec;
      if (!initialize(ec)) {
        logger_.error("Pi4ioe5v init failed: {}", ec.message());
      }
    }
  }

  /**
   * @brief Initialize the expander with the provided configuration.
   * @param ec Error code set on failure
   * @return true on success, false on error
   */
  bool initialize(std::error_code &ec) { return init(config_, ec); }

  /// Set pin directions for the 8-bit port (1=input, 0=output)
  /// @param mask_inputs Bit mask of inputs (bits 0..7)
  /// @param ec Error code set on failure
  void set_direction(uint8_t mask_inputs, std::error_code &ec) {
    std::scoped_lock lock{base_mutex_};
    write_u8_to_register((uint8_t)Reg::DIRECTION, mask_inputs, ec);
  }

  /// Write the output latch for the 8-bit port
  /// @param value Latch values (bits 0..7)
  /// @param ec Error code set on failure
  void write_outputs(uint8_t value, std::error_code &ec) {
    std::scoped_lock lock{base_mutex_};
    write_u8_to_register((uint8_t)Reg::OUTPUT, value, ec);
  }

  /// Read current input pin states for the 8-bit port
  /// @param ec Error code set on failure
  /// @return Bit mask of inputs (bits 0..7)
  uint8_t read_inputs(std::error_code &ec) {
    std::scoped_lock lock{base_mutex_};
    return read_u8_from_register((uint8_t)Reg::INPUT, ec);
  }

  /// Read current output latch for the 8-bit port
  /// @param ec Error code set on failure
  /// @return Bit mask of outputs (bits 0..7)
  uint8_t read_outputs(std::error_code &ec) {
    std::scoped_lock lock{base_mutex_};
    return read_u8_from_register((uint8_t)Reg::OUTPUT, ec);
  }

  /// Configure input polarity inversion (1=invert)
  /// @param mask Bit mask to invert (bits 0..7)
  /// @param ec Error code set on failure
  void set_polarity_invert(uint8_t mask, std::error_code &ec) {
    std::scoped_lock lock{base_mutex_};
    write_u8_to_register((uint8_t)Reg::POLARITY_INV, mask, ec);
  }

  /// Configure internal pull resistors
  /// @param enable_mask Pull enable mask (1=enable)
  /// @param up_select_mask Pull select mask (1=up, 0=down)
  /// @param ec Error code set on failure
  void configure_pull(uint8_t enable_mask, uint8_t up_select_mask, std::error_code &ec) {
    std::scoped_lock lock{base_mutex_};
    write_u8_to_register((uint8_t)Reg::PULL_ENABLE, enable_mask, ec);
    if (ec)
      return;
    write_u8_to_register((uint8_t)Reg::PULL_SELECT, up_select_mask, ec);
  }

protected:
  /**
   * @brief Apply initial configuration to the device.
   * @param cfg Driver configuration
   * @param ec Error code set on failure
   * @return true on success, false on error
   */
  bool init(const Config &cfg, std::error_code &ec) {
    std::scoped_lock lock{base_mutex_};
    set_direction(cfg.direction_mask, ec);
    if (ec)
      return false;
    set_polarity_invert(cfg.polarity_invert, ec);
    if (ec)
      return false;
    configure_pull(cfg.pull_enable, cfg.pull_up_select, ec);
    if (ec)
      return false;
    write_outputs(cfg.output_state, ec);
    if (ec)
      return false;
    return true;
  }

  /// Stored configuration
  Config config_{};
};
} // namespace espp

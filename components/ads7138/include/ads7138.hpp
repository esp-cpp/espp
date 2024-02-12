#pragma once

#include <algorithm>
#include <chrono>
#include <functional>
#include <mutex>
#include <numeric>
#include <thread>
#include <unordered_map>

#include "base_peripheral.hpp"

namespace espp {
/**
 * @brief Class for reading values from the ADS7138 family of ADC chips.
 * @details The ADS7138 is a 16-bit, 8-channel ADC with 8 digital I/O pins. It
 *          supports a variety of sampling modes, including autonomous
 *          sampling, manual sampling, and auto sequence sampling. It also
 *          supports oversampling ratios of 2, 4, 8, 16, 32, 64, and 128. It
 *          additionally allows the user to configure the analog or digital
 *          inputs to trigger an alert when the value goes above or below a
 *          threshold (enter or leave a region of voltage).
 * @see https://www.ti.com/lit/ds/symlink/ads7138.pdf
 *
 * @section ads7138_ex1 ADS7138 Example
 * @snippet ads7138_example.cpp ads7138 example
 */
class Ads7138 : public BasePeripheral<> {
public:
  static constexpr uint8_t DEFAULT_ADDRESS =
      (0x10); ///< Default I2C address of the device (when both R1 and R2 are DNP) (see data sheet
              ///< Table 2, p. 16)

  /// @brief Possible oversampling ratios, see data sheet Table 15 (p. 34)
  enum class OversamplingRatio : uint8_t {
    NONE = 0,   ///< No oversampling
    OSR_2 = 1,  ///< 2x oversampling
    OSR_4 = 2,  ///< 4x oversampling
    OSR_8 = 3,  ///< 8x oversampling
    OSR_16 = 4, ///< 16x oversampling
    OSR_32 = 5, ///< 32x oversampling
    OSR_64 = 6, ///< 64x oversampling
    OSR_128 = 7 ///< 128x oversampling
  };

  /// @brief Possible channel numbers
  /// @details The ADS7138 has 8 channels, see data sheet Table 1 (p. 4)
  /// @note The channel numbers are 0-indexed.
  /// @note The channel may be configured as digital input, digital output, or
  ///       analog input.
  enum class Channel : uint8_t {
    CH0 = 0, ///< Channel 0
    CH1 = 1, ///< Channel 1
    CH2 = 2, ///< Channel 2
    CH3 = 3, ///< Channel 3
    CH4 = 4, ///< Channel 4
    CH5 = 5, ///< Channel 5
    CH6 = 6, ///< Channel 6
    CH7 = 7, ///< Channel 7
  };

  /// @brief Possible modes for analog input conversion
  ///
  /// The ADS7128 device has the following sampling modes:
  /// * Manual Mode: Allows the external host processor to directly request
  ///   and control when the data are sampled. The host provides I2C frames to
  ///   control conversions and the captured data are returned overthe I2C bus
  ///   after each conversion.
  /// * Auto-Sequence Mode: The host can configure the device to scan through
  ///   the enabled analog input channels. The host must provide continuous
  ///   clocks (SCL) to the device to scan through the channels and to read
  ///   the data from the device. The mux automatically switches through the
  ///   predetermined channel sequence, and the data conversion results are
  ///   sent through the data bus.
  /// * Autonomous Mode: After receiving the first start of conversion pulse
  ///   from the host, the ADS7128 device then generates the subsequent start
  ///   of conversion signals autonomously. The device features an internal
  ///   oscillator to generate the start of ADC conversion pulses without the
  ///   host controlling the conversions. Output data are not returned over
  ///   the digital bus; only a signal on the ALERT is generated when an input
  ///   signal crosses the programmable thresholds.
  enum class Mode : uint8_t {
    MANUAL = 0, ///< Manual mode (9th falling edge of SCL (ACK) triggers conversion) and the MUX is
                ///< controlled by register write to MANUAL_CHID field of the CHANNEL_SEL register.
    AUTO_SEQ = 1,   ///< Auto sequence mode (9th falling edge of SCL (ACK) triggers conversion) and
                    ///< the MUX is incremented after each conversion.
    AUTONOMOUS = 2, ///< Autonomous mode (conversion is controlled by the ADC internally) and the
                    ///< MUX is incremented after each conversion.
  };

  /// @brief Event for triggering alerts on analog inputs.
  enum class AnalogEvent {
    OUTSIDE = 0b00, ///< Trigger when ADC value goes outside the low/high thresholds
    INSIDE = 0b01,  ///< Trigger when ADC value goes inside the low/high thresholds
  };

  /// @brief Event for triggering alerts on digital inputs.
  enum class DigitalEvent {
    HIGH = 0b00, ///< Trigger on logic 1
    LOW = 0b01,  ///< Trigger on logic 0
  };

  /// @brief Output mode for digital output channels and ALERT pin
  enum class OutputMode : uint8_t {
    OPEN_DRAIN = 0, ///< Open drain output mode
    PUSH_PULL = 1,  ///< Push-pull output mode
  };

  /// @brief Enum for the data format that can be read from the ADC
  enum class DataFormat : uint8_t {
    RAW = 0,      ///< Raw data format, 12 bit ADC data
    AVERAGED = 1, ///< Averaged data format, 16 bit ADC data
  };

  /// @brief Enum for the different configurations of bits that can be
  ///        appended to the data when reading from the ADC
  enum class Append : uint8_t {
    NONE = 0,       ///< No append
    CHANNEL_ID = 1, ///< Append Channel ID
    STATUS = 2,     ///< Append status flags
  };

  /// @brief Alert logic for ALERT pin
  enum class AlertLogic : uint8_t {
    ACTIVE_LOW = 0,  ///< ALERT pin is active low
    ACTIVE_HIGH = 1, ///< ALERT pin is active high
    PULSED_LOW = 2,  ///< ALERT pin is pulsed low per alert flag
    PULSED_HIGH = 3, ///< ALERT pin is pulsed high per alert flag
  };

  /**
   * @brief Configuration structure
   */
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS; ///< I2C address of the device.
    float avdd_volts = 3.3f; ///< AVDD voltage in Volts. Used for calculating analog input voltage.
    Mode mode = Mode::AUTONOMOUS;              ///< Mode for analog input conversion.
    std::vector<Channel> analog_inputs = {};   ///< List of analog input channels to sample.
    std::vector<Channel> digital_inputs = {};  ///< List of digital input channels to sample.
    std::vector<Channel> digital_outputs = {}; ///< List of digital output channels to sample.
    std::unordered_map<Channel, OutputMode> digital_output_modes =
        {}; ///< Optional output mode for digital
            ///< output channels. If not specified,
            ///< the default value is open-drain.
    std::unordered_map<Channel, bool> digital_output_values =
        {}; ///< Optional initial values for digital
            ///< output channels. If not specified,
            ///< the default value is false in open-drain
            ///< mode.
    OversamplingRatio oversampling_ratio = OversamplingRatio::NONE; ///< Oversampling ratio to use.
    bool statistics_enabled = true; ///< Enable statistics collection (min, max, recent)
    BasePeripheral::write_fn write; ///< Function to write to the ADC
    BasePeripheral::read_fn read;   ///< Function to read from the ADC
    bool auto_init = true;          ///< Automatically initialize the ADC on construction. If false,
                                    ///< initialize() must be called before any other functions.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Verbosity for the logger.
  };

  /**
   * @brief Construct Ads7138
   * @param config Configuration structure.
   */
  explicit Ads7138(const Config &config)
      : BasePeripheral(
            {.address = config.device_address, .write = config.write, .read = config.read},
            "Ads7138", config.log_level)
      , config_(config)
      , mode_(config.mode)
      , avdd_mv_(config.avdd_volts * 1000.0f) // Convert to mV
      , data_format_(config.oversampling_ratio == OversamplingRatio::NONE ? DataFormat::RAW
                                                                          : DataFormat::AVERAGED)
      , statistics_enabled_(config.statistics_enabled)
      , analog_inputs_(config.analog_inputs)
      , digital_inputs_(config.digital_inputs)
      , digital_outputs_(config.digital_outputs)
      , oversampling_ratio_(config.oversampling_ratio) {
    // initialize the ADC
    if (config.auto_init) {
      std::error_code ec;
      initialize(ec);
      if (ec) {
        logger_.error("Error initializing ADC: {}", ec.message());
      }
    }
  }

  /**
   * @brief Initialize the ADC
   *        This function uses the configuration structure passed to the
   *        constructor to configure the ADC.
   * @param ec Error code to set if an error occurs.
   * @note This function must be called before any other functions as it
   *       configures the ADC pins and sets the mode.
   */
  void initialize(std::error_code &ec) { init(config_, ec); }

  /**
   * @brief Communicate with the ADC to get the analog value for the channel
   *        and return it.
   * @param channel Which channel of the ADC to read
   * @param ec Error code to set if an error occurs.
   * @return The voltage (in mV) read from the channel.
   * @note The channel must have been configured as an analog input.
   * @note If the ADC is in autonomous mode, this function will simply read
   *       the value from the ADC's buffer. If the ADC is in manual mode, this
   *       function will trigger a conversion and then read the value from the
   *       ADC's buffer (blocking until conversion is complete).
   * @note This function will return 0 and log an error if the channel is not
   *       configured as an analog input.
   */
  float get_mv(Channel channel, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (mode_ == Mode::MANUAL) {
      // If auto conversion is not enabled, we need to trigger a conversion
      // and wait for it to complete.
      trigger_conversion(channel, ec);
      if (ec)
        return 0;
    }
    if (mode_ != Mode::AUTONOMOUS) {
      // If auto conversion is not enabled, we need to wait for the conversion
      // to complete.
      wait_for_conversion(ec);
      if (ec)
        return 0;
    }
    auto raw = read_recent(channel, ec);
    if (ec)
      return 0;
    return raw_to_mv(raw);
  }

  /**
   * @brief Communicate with the ADC to get the analog value for all channels
   *        and return them.
   * @param ec Error code to set if an error occurs.
   * @return A vector of the voltages (in mV) read from each channel.
   * @note The channels must have been configured as analog inputs.
   * @note The vector will be in the order of the channels configured in the
   *       constructor.
   * @note The vector will be the same length as the number of channels
   *       configured in the constructor.
   * @note If the ADC is in autonomous mode, this function will simply read
   *       the values from the ADC's buffer. If the ADC is in manual mode, this
   *       function will trigger a conversion and then read the values from the
   *       ADC's buffer (blocking until conversion is complete).
   */
  std::vector<float> get_all_mv(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    // TODO: handle the non-autonomous case
    auto raw_values = read_recent_all(ec);
    if (ec)
      return {};
    std::vector<float> values(analog_inputs_.size());
    // convert the raw values (uint16_t) to mv (float)
    for (int i = 0; i < raw_values.size(); i++) {
      values[i] = raw_to_mv(raw_values[i]);
    }
    return values;
  }

  /**
   * @brief Communicate with the ADC to get the analog value for all channels
   *        and return them.
   * @param ec Error code to set if an error occurs.
   * @return An unordered map of the voltages (in mV) read from each channel.
   * @note These are the channels that were configured as analog inputs.
   * @note If the ADC is in autonomous mode, this function will simply read
   *       the values from the ADC's buffer. If the ADC is in manual mode, this
   *       function will trigger a conversion and then read the values from the
   *       ADC's buffer (blocking until conversion is complete).
   */
  std::unordered_map<Channel, float> get_all_mv_map(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::unordered_map<Channel, float> values;
    // TODO: handle the non-autonomous case
    auto raw_values = read_mapped_recent_all(ec);
    if (ec)
      return {};
    // convert the raw values (uint16_t) to mv (float)
    for (auto &[channel, raw] : raw_values) {
      values[channel] = raw_to_mv(raw);
    }
    return values;
  }

  /// @brief Configure the ALERT pin
  /// @param output_mode Output mode ALERT pin
  /// @param alert_logic Alert logic for ALERT pin
  /// @param ec Error code to set if an error occurs.
  void configure_alert(OutputMode output_mode, AlertLogic alert_logic, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    // set the ALERT_PIN_CFG register
    if (output_mode == OutputMode::OPEN_DRAIN) {
      set_bits_(Register::ALERT_PIN_CFG, ALERT_DRIVE, ec);
    } else {
      clear_bits_(Register::ALERT_PIN_CFG, ALERT_DRIVE, ec);
    }
    if (ec)
      return;
    clear_bits_(Register::ALERT_PIN_CFG, ALERT_LOGIC, ec);
    if (ec)
      return;
    set_bits_(Register::ALERT_PIN_CFG, static_cast<uint8_t>(alert_logic), ec);
    if (ec)
      return;
    // ensure we have enabled threshold comparison
    enable_threshold_comparison(true, ec);
  }

  /// @brief Configure the analog channel to generate an alert when the
  ///       voltage crosses a low or high threshold.
  /// @param channel Analog channel to configure
  /// @param high_threshold_mv High threshold (in mV) to generate an alert
  /// @param low_threshold_mv Low threshold (in mV) to generate an alert
  /// @param event Event type which will generate an alert
  /// @param event_count Number of events to generate an alert (1-15). Checks n+1 consecutive
  /// samples above or below threshold before setting event flag.
  /// @param ec Error code to set if an error occurs.
  /// @note The channel must have been configured as an analog input.
  void set_analog_alert(Channel channel, float high_threshold_mv, float low_threshold_mv,
                        AnalogEvent event, int event_count, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    // if it's a digital output channel, we can't set thresholds
    if (!is_analog_input(channel)) {
      logger_.error("Channel {} is configured as a digital output, cannot set alert", channel);
      ec = std::make_error_code(std::errc::invalid_argument);
      return;
    }
    // alert flags for this channel assert the ALERT pin
    set_bits_(Register::ALERT_CH_SEL, 1 << (int)channel, ec);
    if (ec)
      return;
    // set the event region and configure the event
    set_event_region(static_cast<uint8_t>(channel), event, ec);
    if (ec)
      return;
    configure_event(channel, high_threshold_mv, low_threshold_mv, event_count, ec);
    if (ec)
      return;
    // ensure we have enabled threshold comparison
    enable_threshold_comparison(true, ec);
  }

  /// @brief Configure the digital input channel to generate an alert when the
  ///        input goes high or low.
  /// @param channel Digital input channel to configure
  /// @param event Event type which will generate an alert
  /// @param ec Error code to set if an error occurs.
  void set_digital_alert(Channel channel, DigitalEvent event, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    // if it's not a digital input channel we can't set a digital alert
    if (!is_digital_input(channel)) {
      logger_.error("Channel {} is not configured as a digital input, cannot set alert", channel);
      ec = std::make_error_code(std::errc::invalid_argument);
      return;
    }
    // alert flags for this channel assert the ALERT pin
    set_bits_(Register::ALERT_CH_SEL, 1 << (int)channel, ec);
    if (ec)
      return;
    // set the event region
    set_event_region(static_cast<uint8_t>(channel), event, ec);
    if (ec)
      return;
    // ensure we have enabled threshold comparison
    enable_threshold_comparison(true, ec);
  }

  /// @brief Get all the event data registers
  /// @param[inout] event_flags Event flag register
  /// @param[inout] event_high_flags Event high flag register
  /// @param[inout] event_low_flags Event low flag register
  /// @param[inout] ec Error code to set if an error occurs.
  /// @note The event flags are cleared after reading.
  void get_event_data(uint8_t *event_flags, uint8_t *event_high_flags, uint8_t *event_low_flags,
                      std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    // read the event data registers
    uint8_t read_val[3] = {0};
    read_val[0] = read_one_(Register::EVENT_FLAG, ec);
    if (ec)
      return;
    read_val[1] = read_one_(Register::EVENT_HIGH_FLAG, ec);
    if (ec)
      return;
    read_val[2] = read_one_(Register::EVENT_LOW_FLAG, ec);
    if (ec)
      return;
    *event_flags = read_val[0];
    *event_high_flags = read_val[1];
    *event_low_flags = read_val[2];
    // now clear the data
    clear_event_high_flag(*event_high_flags, ec);
    if (ec)
      return;
    clear_event_low_flag(*event_low_flags, ec);
  }

  /// @brief Get the event flag register
  /// @param ec Error code to set if an error occurs.
  /// @return The event flag register
  /// @note The event flags are NOT cleared after reading.
  uint8_t get_event_flags(std::error_code &ec) { return read_one_(Register::EVENT_FLAG, ec); }

  /// @brief Get the event high flag register
  /// @param ec Error code to set if an error occurs.
  /// @return The event high flag register
  /// @note The event high flags are NOT cleared after reading.
  uint8_t get_event_high_flag(std::error_code &ec) {
    return read_one_(Register::EVENT_HIGH_FLAG, ec);
  }

  /// @brief Get the event low flag register
  /// @param ec Error code to set if an error occurs.
  /// @return The event low flag register
  /// @note The event low flags are NOT cleared after reading.
  uint8_t get_event_low_flag(std::error_code &ec) {
    return read_one_(Register::EVENT_LOW_FLAG, ec);
  }

  /// @brief Clear the event flag register
  /// @param flags Flags to clear
  /// @param ec Error code to set if an error occurs.
  void clear_event_high_flag(uint8_t flags, std::error_code &ec) {
    set_bits_(Register::EVENT_HIGH_FLAG, flags, ec);
  }

  /// @brief Clear the event flag register
  /// @param flags Flags to clear
  /// @param ec Error code to set if an error occurs.
  void clear_event_low_flag(uint8_t flags, std::error_code &ec) {
    set_bits_(Register::EVENT_LOW_FLAG, flags, ec);
  }

  /// @brief Configure the digital output mode for the given channel.
  /// @param channel Channel to configure
  /// @param output_mode Output mode for the channel
  /// @param ec Error code to set if an error occurs.
  /// @note The channel must have been configured as a digital output.
  void set_digital_output_mode(Channel channel, OutputMode output_mode, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!is_digital_output(channel)) {
      logger_.error("Channel {} is not configured as a digital output", channel);
      ec = std::make_error_code(std::errc::invalid_argument);
      return;
    }
    if (output_mode == OutputMode::OPEN_DRAIN) {
      // 0 in the register = open-drain
      clear_bits_(Register::GPO_DRIVE_CFG, 1 << (int)channel, ec);
    } else {
      // 1 in the register = push-pull
      set_bits_(Register::GPO_DRIVE_CFG, 1 << (int)channel, ec);
    }
  }

  /**
   * @brief Set the digital output value for the given channel.
   * @param channel Which channel to set the digital output value for.
   * @param value The value to set the digital output to.
   * @param ec Error code to set if an error occurs.
   * @note The channel must have been configured as a digital output.
   */
  void set_digital_output_value(Channel channel, bool value, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!is_digital_output(channel)) {
      logger_.error("Channel {} is not configured as a digital output", channel);
      ec = std::make_error_code(std::errc::invalid_argument);
      return;
    }
    if (value) {
      set_bits_(Register::GPO_VALUE, 1 << (int)channel, ec);
    } else {
      clear_bits_(Register::GPO_VALUE, 1 << (int)channel, ec);
    }
  }

  /**
   * @brief Get the digital input value for the given channel.
   * @param channel Which channel to get the digital input value for.
   * @param ec Error code to set if an error occurs.
   * @return The value of the digital input.
   * @note The channel must have been configured as a digital input.
   */
  bool get_digital_input_value(Channel channel, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!is_digital_input(channel)) {
      logger_.error("Channel {} is not configured as a digital input", channel);
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    auto value = read_one_(Register::GPI_VALUE, ec);
    if (ec)
      return false;
    return (value & (1 << (int)channel)) != 0;
  }

  /**
   * @brief Get the digital input values for all channels.
   * @param ec Error code to set if an error occurs.
   * @return The values of the digital inputs.
   * @note The returned value is a bitfield, with each bit corresponding to a
   *       channel. The LSB corresponds to channel 0, the MSB to channel 7.
   * @note Only channels configured as digital inputs are returned.
   */
  uint8_t get_digital_input_values(std::error_code &ec) {
    return read_one_(Register::GPI_VALUE, ec);
  }

  /// @brief Perform a software reset of the device
  /// @param ec Error code to set if an error occurs.
  /// @note This will reset all registers to their default values (converting
  ///       all channels to analog inputs and disabling all events).
  /// @note If the write is successful, the function will wait for the reset
  ///       to complete before returning
  void reset(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    // reset the device
    write_one_(Register::GENERAL_CFG, SW_RST, ec);
    if (ec)
      return;
    // wait for the reset to complete
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

protected:
  /// Opcodes for the ADS7138, see data sheet Table 9 (p. 26)
  /// @{
  /// @brief Opcode for the ADS7138, see data sheet Table 9 (p. 26)
  /// @details The opcode is the first byte sent to the device before the
  ///          register address and data bytes.
  static constexpr uint8_t OP_GENERAL = 0b00000000;  ///< General call - used for software reset,
                                                     ///< programming address, and setting i2c speed
  static constexpr uint8_t OP_READ_ONE = 0b00010000; ///< Read one register
  static constexpr uint8_t OP_WRITE_ONE = 0b00001000;   ///< Write one register
  static constexpr uint8_t OP_SET_BITS = 0b00011000;    ///< Set bits in a register
  static constexpr uint8_t OP_CLR_BITS = 0b00100000;    ///< Clear bits in a register
  static constexpr uint8_t OP_READ_BLOCK = 0b00110000;  ///< Read a block of registers
  static constexpr uint8_t OP_WRITE_BLOCK = 0b00101000; ///< Write a block of registers
  /// @}

  /// @{
  /// @brief Bit masks for the bits in the SYSTEM_STATUS register, see data
  ///       sheet Table 12 (p. 32)
  static constexpr int SEQ_STATUS = (1 << 6); ///< Sequence status bit (0 = idle, 1 = busy)
  static constexpr int I2C_SPEED = (1 << 5);  ///< I2C speed bit (0 = 100 kHz, 1 = 400 kHz)
  static constexpr int OSR_DONE =
      (1 << 3); ///< OSR done bit (0 = not done, 1 = done), R/W (clear by writing 1)
  static constexpr int CRC_ERR_FUSE =
      (1 << 2); ///< CRC error fuse bit (0 = no error, 1 = error) (power up configuration check)
  static constexpr int CRC_ERR_IN =
      (1 << 1); ///< CRC error input bit (0 = no error, 1 = error) (input data check)
  static constexpr int BOR =
      (1 << 0); ///< Brown-out reset bit (0 = no reset, 1 = brown out condition detected)
  /// @}

  /// @{
  /// @brief Bit masks for the bits in the GENERAL_CFG register, see data
  ///        sheet Table 13 (p. 33)
  static constexpr int CRC_EN = (1 << 6);   ///< CRC enable bit (0 = disable, 1 = enable)
  static constexpr int STATS_EN = (1 << 5); ///< Statistics enable bit (0 = disable, 1 = enable)
  static constexpr int DWC_EN =
      (1 << 4); ///< Digital window comparator enable bit (0 = disable, 1 = enable)
  static constexpr int CNVST = (1 << 3); ///< Control start conversion bit (0 = disable, 1 = enable)
  static constexpr int CH_RST = (1 << 2); ///< Channel reset (force all channels tobe analog inputs)
                                          ///< bit (0 = disable, 1 = enable)
  static constexpr int CAL = (1 << 1);    ///< Calibration bit (0 = disable, 1 = start calibration)
  static constexpr int SW_RST = (1 << 0); ///< Software reset (resets all registers to default
                                          ///< values) bit (0 = normal operation, 1 = reset)
  /// @}

  /// @{
  /// @brief Bit masks for the bits in the DATA_CFG register, see data sheet
  ///       Table 14 (p. 33)
  static constexpr int FIX_PAT = (1 << 7); ///< Fixed pattern bit (0 = disable, 1 = enable)
  static constexpr int APPEND =
      (1 << 5) |
      (1 << 4); ///< Append 4-bit channel ID or status flags to output data. 00 = no append, 01 =
                ///< append channel ID, 10 = append status flags, 11 = reserved
  static constexpr int APPEND_NONE = 0;          ///< Raw data format (no append)
  static constexpr int APPEND_CHID = (1 << 4);   ///< Append 4-bit channel ID to output data
  static constexpr int APPEND_STATUS = (1 << 5); ///< Append status flags to output data
  /// @}

  /// @{
  /// @brief Bit masks for the bits in the OSR_CFG register, see data sheet
  ///      Table 15 (p. 34)
  static constexpr int OSR =
      (1 << 2) | (1 << 1) |
      (1 << 0); ///< Oversampling ratio bits (0 = no oversampling, 1 = 2x oversampling, 2 = 4x
                ///< oversampling, 3 = 8x oversampling, 4 = 16x oversampling, 5 = 32x oversampling,
                ///< 6 = 64x oversampling, 7 = 128x oversampling)
  /// @}

  /// @{
  /// @brief Bit masks for the bits in the OPMODE_CFG register, see data sheet
  ///      Table 16 (p. 34)
  static constexpr int CONV_ON_ERROR = (1 << 7); ///< Control continuation of autonomous modes if
                                                 ///< CRC error is detected (0 = continue, 1 = stop)
  static constexpr int CONV_MODE =
      (1 << 5); ///< Conversion mode bits (0 = manual, 1 = autonomous) (note: the actual conv_mode
                ///< bits are bits 6 and 5, but bit 6 is reserved and must be set to 0)
  static constexpr int OSC_SEL =
      (1 << 4); ///< Oscillator selection bit (0 = high-speed oscillator, 1 = low-power oscillator)
  static constexpr int CLK_DIV =
      (1 << 3) | (1 << 2) | (1 << 1) |
      (1 << 0); ///< Sampling speed control in autonomous monitoring mode.
  /// @}

  /// @{
  /// @brief Bit masks for the bits in the PIN_CFG register, see data sheet
  ///     Table 17 (p. 35)
  /// @note 0 = analog input, 1 = digital input
  /// @}

  /// @{
  /// @brief Bit masks for the bits in the GPIO_CFG register, see data sheet
  ///    Table 18 (p. 35)
  /// @note 0 = digital input, 1 = digital output
  /// @}

  /// @{
  /// @brief Bit masks for the bits in the GPO_DRIVE_CFG register, see data
  ///      sheet Table 19 (p. 35)
  /// @note 0 = open drain, 1 = push-pull
  /// @}

  /// @{
  /// @brief Bit masks for the bits in the GPO_VALUE register, see data sheet
  ///     Table 20 (p. 36)
  /// @note 0 = low, 1 = high
  /// @}

  /// @{
  /// @brief Bit masks for the bits in the GPI_VALUE register, see data sheet
  ///     Table 21 (p. 36)
  /// @note 0 = low, 1 = high
  /// @}

  /// @{
  /// @brief Bit masks for the bits in the SEQUENCE_CFG register, see data
  ///     sheet Table 22 (p. 36)
  static constexpr int SEQ_START =
      (1 << 4); ///< Sequence start bit (0 = no action, 1 = start sequencing in ascending order for
                ///< channels enabled in the AUTO_SEQ_CH_SEL register)
  static constexpr int SEQ_MODE =
      (1 << 0); ///< Sequence mode bits (0 = manual, 1 = autonomous) (note: the actual sequence mode
                ///< bits are bits 1 and 0, but bit 1 is reserved and must be set to 0).
  /// @}

  /// @{
  /// @brief Bit masks for the bits in the CHANNEL_SEL register, see data sheet
  ///    Table 23 (p. 37)
  static constexpr int CH_SEL =
      (1 << 3) | (1 << 2) | (1 << 1) |
      (1 << 0); ///< Channel selection bits (0 = AIN0, 1 = AIN1, 2 = AIN2, 3 = AIN3, 4 = AIN4, 5 =
                ///< AIN5, 6 = AIN6, 7 = AIN7, 8 = reserved, 9 = reserved, 10 = reserved, 11 =
                ///< reserved, 12 = reserved, 13 = reserved, 14 = reserved, 15 = reserved)
  /// @}

  /// @{
  /// @brief Bit masks for the bits in the AUTO_SEQ_CH_SEL register, see data
  ///    sheet Table 24 (p. 37)
  /// @note 0 = disable, 1 = enable for each channel according to the bit
  ///       position
  /// @}

  /// @{
  /// @brief Bit masks for the bits in the ALERT_PIN_CFG register, see data
  ///   sheet Table 27 (p. 38)
  static constexpr int ALERT_DRIVE =
      (1 << 2); ///< Alert pin drive configuration bit (0 = open drain, 1 = push-pull)
  static constexpr int ALERT_LOGIC =
      (1 << 1) | (1 << 0); ///< Alert pin logic configuration bits (0 = active low, 1 = active high,
                           ///< 2 = pulsed low, 3 = pulsed high)
  /// @}

  /// @brief Register addresses for the ADS7138, see data sheet Table 10 (p. 29)
  enum class Register : uint8_t {
    SYSTEM_STATUS = 0x00, ///< System status register
    GENERAL_CFG = 0x01,   ///< General configuration register
    DATA_CFG = 0x02,      ///< Data configuration register
    OSR_CFG = 0x03,       ///< Oversampling ratio configuration register
    OPMODE_CFG = 0x04,    ///< Operating mode configuration register
    PIN_CFG = 0x05,       ///< Pin configuration register

    GPIO_CFG = 0x07,      ///< GPIO configuration register
    GPO_DRIVE_CFG = 0x09, ///< GPO drive configuration register
    GPO_VALUE = 0x0B,     ///< GPO value register
    GPI_VALUE = 0x0D,     ///< GPI value register

    ZCD_BLANKING_CFG = 0x0F, ///< Zero-crossing detection blanking configuration register (NOTE:
                             ///< only available on ADS7128)

    SEQUENCE_CFG = 0x10,    ///< Sequence configuration register
    CHANNEL_SEL = 0x11,     ///< Channel selection register
    AUTO_SEQ_CH_SEL = 0x12, ///< Auto sequence channel selection register

    ALERT_CH_SEL = 0x14,  ///< Alert channel selection register
    ALERT_MAP = 0x16,     ///< Alert map register
    ALERT_PIN_CFG = 0x17, ///< Alert pin configuration register

    EVENT_FLAG = 0x18,      ///< Event flag register
    EVENT_HIGH_FLAG = 0x1A, ///< Event high flag register
    EVENT_LOW_FLAG = 0x1C,  ///< Event low flag register
    EVENT_RGN = 0x1E,       ///< Event region register

    HYSTERESIS_CH0 = 0x20,  ///< Hysteresis channel 0 register
    HIGH_TH_CH0 = 0x21,     ///< High threshold channel 0 register
    EVENT_COUNT_CH0 = 0x22, ///< Event count channel 0 register
    LOW_TH_CH0 = 0x23,      ///< Low threshold channel 0 register
    HYSTERESIS_CH1 = 0x24,  ///< Hysteresis channel 1 register
    HIGH_TH_CH1 = 0x25,     ///< High threshold channel 1 register
    EVENT_COUNT_CH1 = 0x26, ///< Event count channel 1 register
    LOW_TH_CH1 = 0x27,      ///< Low threshold channel 1 register
    HYSTERESIS_CH2 = 0x28,  ///< Hysteresis channel 2 register
    HIGH_TH_CH2 = 0x29,     ///< High threshold channel 2 register
    EVENT_COUNT_CH2 = 0x2A, ///< Event count channel 2 register
    LOW_TH_CH2 = 0x2B,      ///< Low threshold channel 2 register
    HYSTERESIS_CH3 = 0x2C,  ///< Hysteresis channel 3 register
    HIGH_TH_CH3 = 0x2D,     ///< High threshold channel 3 register
    EVENT_COUNT_CH3 = 0x2E, ///< Event count channel 3 register
    LOW_TH_CH3 = 0x2F,      ///< Low threshold channel 3 register
    HYSTERESIS_CH4 = 0x30,  ///< Hysteresis channel 4 register
    HIGH_TH_CH4 = 0x31,     ///< High threshold channel 4 register
    EVENT_COUNT_CH4 = 0x32, ///< Event count channel 4 register
    LOW_TH_CH4 = 0x33,      ///< Low threshold channel 4 register
    HYSTERESIS_CH5 = 0x34,  ///< Hysteresis channel 5 register
    HIGH_TH_CH5 = 0x35,     ///< High threshold channel 5 register
    EVENT_COUNT_CH5 = 0x36, ///< Event count channel 5 register
    LOW_TH_CH5 = 0x37,      ///< Low threshold channel 5 register
    HYSTERESIS_CH6 = 0x38,  ///< Hysteresis channel 6 register
    HIGH_TH_CH6 = 0x39,     ///< High threshold channel 6 register
    EVENT_COUNT_CH6 = 0x3A, ///< Event count channel 6 register
    LOW_TH_CH6 = 0x3B,      ///< Low threshold channel 6 register
    HYSTERESIS_CH7 = 0x3C,  ///< Hysteresis channel 7 register
    HIGH_TH_CH7 = 0x3D,     ///< High threshold channel 7 register
    EVENT_COUNT_CH7 = 0x3E, ///< Event count channel 7 register
    LOW_TH_CH7 = 0x3F,      ///< Low threshold channel 7 register

    MAX_CH0_LSB = 0x60,    ///< Maximum channel 0 LSB register
    MAX_CH0_MSB = 0x61,    ///< Maximum channel 0 MSB register
    MAX_CH1_LSB = 0x62,    ///< Maximum channel 1 LSB register
    MAX_CH1_MSB = 0x63,    ///< Maximum channel 1 MSB register
    MAX_CH2_LSB = 0x64,    ///< Maximum channel 2 LSB register
    MAX_CH2_MSB = 0x65,    ///< Maximum channel 2 MSB register
    MAX_CH3_LSB = 0x66,    ///< Maximum channel 3 LSB register
    MAX_CH3_MSB = 0x67,    ///< Maximum channel 3 MSB register
    MAX_CH4_LSB = 0x68,    ///< Maximum channel 4 LSB register
    MAX_CH4_MSB = 0x69,    ///< Maximum channel 4 MSB register
    MAX_CH5_LSB = 0x6A,    ///< Maximum channel 5 LSB register
    MAX_CH5_MSB = 0x6B,    ///< Maximum channel 5 MSB register
    MAX_CH6_LSB = 0x6C,    ///< Maximum channel 6 LSB register
    MAX_CH6_MSB = 0x6D,    ///< Maximum channel 6 MSB register
    MAX_CH7_LSB = 0x6E,    ///< Maximum channel 7 LSB register
    MAX_CH7_MSB = 0x6F,    ///< Maximum channel 7 MSB register
    MIN_CH0_LSB = 0x80,    ///< Minimum channel 0 LSB register
    MIN_CH0_MSB = 0x81,    ///< Minimum channel 0 MSB register
    MIN_CH1_LSB = 0x82,    ///< Minimum channel 1 LSB register
    MIN_CH1_MSB = 0x83,    ///< Minimum channel 1 MSB register
    MIN_CH2_LSB = 0x84,    ///< Minimum channel 2 LSB register
    MIN_CH2_MSB = 0x85,    ///< Minimum channel 2 MSB register
    MIN_CH3_LSB = 0x86,    ///< Minimum channel 3 LSB register
    MIN_CH3_MSB = 0x87,    ///< Minimum channel 3 MSB register
    MIN_CH4_LSB = 0x88,    ///< Minimum channel 4 LSB register
    MIN_CH4_MSB = 0x89,    ///< Minimum channel 4 MSB register
    MIN_CH5_LSB = 0x8A,    ///< Minimum channel 5 LSB register
    MIN_CH5_MSB = 0x8B,    ///< Minimum channel 5 MSB register
    MIN_CH6_LSB = 0x8C,    ///< Minimum channel 6 LSB register
    MIN_CH6_MSB = 0x8D,    ///< Minimum channel 6 MSB register
    MIN_CH7_LSB = 0x8E,    ///< Minimum channel 7 LSB register
    MIN_CH7_MSB = 0x8F,    ///< Minimum channel 7 MSB register
    RECENT_CH0_LSB = 0xA0, ///< Recent channel 0 LSB register
    RECENT_CH0_MSB = 0xA1, ///< Recent channel 0 MSB register
    RECENT_CH1_LSB = 0xA2, ///< Recent channel 1 LSB register
    RECENT_CH1_MSB = 0xA3, ///< Recent channel 1 MSB register
    RECENT_CH2_LSB = 0xA4, ///< Recent channel 2 LSB register
    RECENT_CH2_MSB = 0xA5, ///< Recent channel 2 MSB register
    RECENT_CH3_LSB = 0xA6, ///< Recent channel 3 LSB register
    RECENT_CH3_MSB = 0xA7, ///< Recent channel 3 MSB register
    RECENT_CH4_LSB = 0xA8, ///< Recent channel 4 LSB register
    RECENT_CH4_MSB = 0xA9, ///< Recent channel 4 MSB register
    RECENT_CH5_LSB = 0xAA, ///< Recent channel 5 LSB register
    RECENT_CH5_MSB = 0xAB, ///< Recent channel 5 MSB register
    RECENT_CH6_LSB = 0xAC, ///< Recent channel 6 LSB register
    RECENT_CH6_MSB = 0xAD, ///< Recent channel 6 MSB register
    RECENT_CH7_LSB = 0xAE, ///< Recent channel 7 LSB register
    RECENT_CH7_MSB = 0xAF, ///< Recent channel 7 MSB register

    RMS_CFG = 0xC0, ///< RMS configuration register (NOTE: only available on AD7128)
    RMS_LSB = 0xC1, ///< RMS LSB register (NOTE: only available on AD7128)
    RMS_MSB = 0xC2, ///< RMS MSB register (NOTE: only available on AD7128)

    GPO0_TRIG_EVENT_SEL = 0xC3, ///< GPO0 trigger event selection register
    GPO1_TRIG_EVENT_SEL = 0xC5, ///< GPO1 trigger event selection register
    GPO2_TRIG_EVENT_SEL = 0xC7, ///< GPO2 trigger event selection register
    GPO3_TRIG_EVENT_SEL = 0xC9, ///< GPO3 trigger event selection register
    GPO4_TRIG_EVENT_SEL = 0xCB, ///< GPO4 trigger event selection register
    GPO5_TRIG_EVENT_SEL = 0xCD, ///< GPO5 trigger event selection register
    GPO6_TRIG_EVENT_SEL = 0xCF, ///< GPO6 trigger event selection register
    GPO7_TRIG_EVENT_SEL = 0xD1, ///< GPO7 trigger event selection register

    GPO_TRIGGER_CFG = 0xE9,   ///< GPO trigger configuration register
    GPO_VALUE_TRIGGER = 0xEB, ///< GPO value trigger register
  };

  /// @brief Mode for scanning of analog inputs
  enum class SequenceMode : uint8_t {
    MANUAL = 0, ///< Manual mode
    AUTO = 1,   ///< Auto mode
  };

  enum class EventRegion {
    OUTSIDE_OR_HIGH = 0, ///< Trigger when ADC value goes outside the low/high thresholds or
                         ///< digital input is high
    INSIDE_OR_LOW = 1,   ///< Trigger when ADC value goes inside the low/high thresholds or digital
                         ///< input is low
  };

  void init(const Config &config, std::error_code &ec) {
    // Set the data format
    set_data_format(data_format_, Append::CHANNEL_ID, ec);
    if (ec)
      return;

    // Set the oversampling ratio
    set_oversampling_ratio(config.oversampling_ratio, ec);
    if (ec)
      return;

    // Set the statistics mode
    set_statistics_enabled(config.statistics_enabled, ec);
    if (ec)
      return;

    // set pin configuration (analog vs digital)
    set_pin_configuration(ec);
    if (ec)
      return;

    // set the digital output outputmode (push-pull vs open-drain). NOTE: this
    // will work because the digital output channels have already been saved
    // (though they have not been configured yet)
    for (auto &[channel, output_mode] : config.digital_output_modes) {
      set_digital_output_mode(channel, output_mode, ec);
      if (ec)
        return;
    }

    // set the digital output values. NOTE: this will work because the digital
    // output channels have already been saved (though they have not been
    // configured yet)
    for (auto &[channel, value] : config.digital_output_values) {
      set_digital_output_value(channel, value, ec);
      if (ec)
        return;
    }

    // Set the digital input/output channels
    set_digital_io_direction(ec);
    if (ec)
      return;

    // Set the analog input channels
    set_analog_inputs(ec);
    if (ec)
      return;

    // set the operational mode
    set_operational_mode(ec);
  }

  void set_data_format(DataFormat format, Append append, std::error_code &ec) {
    clear_bits_(Register::DATA_CFG, APPEND, ec);
    if (ec)
      return;
    if (append == Append::CHANNEL_ID) {
      logger_.debug("Appending channel ID");
      set_bits_(Register::DATA_CFG, APPEND_CHID, ec);
    } else if (append == Append::STATUS) {
      logger_.debug("Appending status");
      set_bits_(Register::DATA_CFG, APPEND_STATUS, ec);
    }
    if (ec)
      return;
    num_data_bytes_ = 2;
    if (format == DataFormat::AVERAGED && append != Append::NONE) {
      num_data_bytes_ = 3;
    }
    data_format_ = format;
    logger_.info("Data format set to {}", data_format_);
    logger_.info("Number of data bytes set to {}", num_data_bytes_);
  }

  void set_oversampling_ratio(OversamplingRatio ratio, std::error_code &ec) {
    uint8_t data = static_cast<uint8_t>(ratio);
    write_one_(Register::OSR_CFG, data, ec);
  }

  void set_statistics_enabled(bool enabled, std::error_code &ec) {
    if (enabled) {
      set_bits_(Register::GENERAL_CFG, STATS_EN, ec);
    } else {
      clear_bits_(Register::GENERAL_CFG, STATS_EN, ec);
    }
    if (ec)
      return;
    statistics_enabled_ = enabled;
    logger_.info("Statistics enabled set to {}", statistics_enabled_);
  }

  static uint8_t bit_pred(uint8_t value, Channel channel) {
    return value | (1 << static_cast<uint8_t>(channel));
  };

  void set_pin_configuration(std::error_code &ec) {
    logger_.info("Setting digital mode for outputs {} and inputs {}", digital_outputs_,
                 digital_inputs_);
    uint8_t data = 0;
    data = std::accumulate(digital_inputs_.begin(), digital_inputs_.end(), data, bit_pred);
    data = std::accumulate(digital_outputs_.begin(), digital_outputs_.end(), data, bit_pred);
    // don't have to do anything for analog inputs since they are the default
    // state (0)
    write_one_(Register::PIN_CFG, data, ec);
  }

  void set_digital_io_direction(std::error_code &ec) {
    logger_.info("Setting digital output for pins {}", digital_outputs_);
    // default direction is input (0)
    uint8_t data = std::accumulate(digital_outputs_.begin(), digital_outputs_.end(), 0, bit_pred);
    write_one_(Register::GPIO_CFG, data, ec);
  }

  void set_analog_inputs(std::error_code &ec) {
    logger_.info("Setting analog inputs for pins {}", analog_inputs_);
    if (mode_ == Mode::AUTONOMOUS) {
      logger_.info("Setting analog inputs for autonomous mode");
      // configure the analog inputs for autonomous conversion sequence
      uint8_t data = std::accumulate(analog_inputs_.begin(), analog_inputs_.end(), 0, bit_pred);
      write_one_(Register::AUTO_SEQ_CH_SEL, data, ec);
    }
  }

  /// @brief Sets the operational mode of the device
  /// @details The operational mode is set by the CONV_MODE bits in the
  ///         OPMODE_CFG register. The mode_ member variable is used to
  ///         determine the value to write to the register.
  ///         The SEQ_MODE bits in the SEQUENCE_CFG register are also set
  ///         based on the mode_ member variable.
  /// @sa The Mode enum for more information on the different modes.
  /// @sa The mode_ member variable.
  /// @sa Table 7 (page 22) of the datasheet for more information on the
  ///     functional modes.
  void set_operational_mode(std::error_code &ec) {
    // set the CONV_MODE bits in OPMODE_CFG based on mode_
    set_auto_conversion(mode_ == Mode::AUTONOMOUS, ec);
    if (ec)
      return;
    // set the SEQ_MODE bits in SEQUENCE_CFG based on mode_
    if (mode_ == Mode::MANUAL) {
      set_sequence_mode(SequenceMode::MANUAL, ec);
    } else {
      set_sequence_mode(SequenceMode::AUTO, ec);
      if (ec)
        return;
      // if we're in autonomous mode, start the auto conversion sequence
      start_auto_conversion(ec);
    }
  }

  void set_auto_conversion(bool enable, std::error_code &ec) {
    // set conversion mode
    if (enable) {
      logger_.info("Setting auto conversion mode");
      // 1 = auto conversion mode
      set_bits_(Register::OPMODE_CFG, CONV_MODE, ec);
    } else {
      logger_.info("Setting manual conversion mode");
      // 0 = manual conversion mode
      clear_bits_(Register::OPMODE_CFG, CONV_MODE, ec);
    }
  }

  void set_sequence_mode(SequenceMode mode, std::error_code &ec) {
    if (mode == SequenceMode::AUTO) {
      logger_.info("Setting auto sequence mode");
      set_bits_(Register::SEQUENCE_CFG, SEQ_MODE, ec);
    } else {
      logger_.info("Setting manual sequence mode");
      clear_bits_(Register::SEQUENCE_CFG, SEQ_MODE, ec);
    }
  }

  void start_auto_conversion(std::error_code &ec) {
    if (mode_ == Mode::MANUAL) {
      logger_.error("Auto sequence mode is not enabled, not starting");
      ec = std::make_error_code(std::errc::protocol_error);
      return;
    }
    logger_.info("Starting auto conversion sequence");
    // start the auto conversion sequence
    set_bits_(Register::SEQUENCE_CFG, SEQ_START, ec);
  }

  void stop_auto_conversion(std::error_code &ec) {
    if (mode_ == Mode::MANUAL) {
      logger_.error("Auto sequence mode is not enabled, not stopping");
      ec = std::make_error_code(std::errc::protocol_error);
      return;
    }
    logger_.info("Stopping auto conversion sequence");
    // start the auto conversion sequence
    clear_bits_(Register::SEQUENCE_CFG, SEQ_START, ec);
  }

  uint16_t read_recent(Channel ch, std::error_code &ec) {
    if (!is_analog_input(ch)) {
      logger_.error("Channel {} is not configured as an analog input", ch);
      ec = std::make_error_code(std::errc::invalid_argument);
      return 0;
    }
    if (!statistics_enabled_) {
      logger_.error("Statistics are not enabled, cannot read recent value");
      ec = std::make_error_code(std::errc::protocol_error);
      return 0;
    }
    logger_.info("Reading recent value for channel {}", ch);
    int channel = static_cast<int>(ch);
    // read both the LSB AND MSB registers and combine them
    return read_two_(
        static_cast<Register>(static_cast<uint8_t>(Register::RECENT_CH0_LSB) + channel * 2), ec);
  }

  std::vector<uint16_t> read_recent_all(std::error_code &ec) {
    if (!statistics_enabled_) {
      logger_.error("Statistics are not enabled, cannot read recent value");
      ec = std::make_error_code(std::errc::protocol_error);
      return {};
    }
    logger_.info("Reading recent values for all channels");
    std::vector<uint16_t> values(analog_inputs_.size());
    uint8_t raw_values[16];
    read_block_(Register::RECENT_CH0_LSB, raw_values, 16, ec);
    if (ec)
      return {};
    int analog_index = 0;
    // only pull out the ones that were configured as analog inputs
    for (int i = 0; i < 8; i++) {
      if (is_analog_input(static_cast<Channel>(i))) {
        // read both the LSB AND MSB registers and combine them (lsb is first)
        uint8_t lsb = raw_values[i * 2];
        uint8_t msb = raw_values[i * 2 + 1];
        values[analog_index++] = (msb << 8) | lsb;
      }
    }
    return values;
  }

  std::unordered_map<Channel, uint16_t> read_mapped_recent_all(std::error_code &ec) {
    if (!statistics_enabled_) {
      logger_.error("Statistics are not enabled, cannot read recent value");
      ec = std::make_error_code(std::errc::protocol_error);
      return {};
    }
    logger_.info("Reading recent mapped values for all channels");
    std::unordered_map<Channel, uint16_t> values;
    uint8_t raw_values[16];
    read_block_(Register::RECENT_CH0_LSB, raw_values, 16, ec);
    if (ec)
      return {};
    // only pull out the ones that were configured as analog inputs
    for (int i = 0; i < 8; i++) {
      Channel ch = static_cast<Channel>(i);
      if (is_analog_input(ch)) {
        // read both the LSB AND MSB registers and combine them (lsb is first)
        uint8_t lsb = raw_values[i * 2];
        uint8_t msb = raw_values[i * 2 + 1];
        values[ch] = (msb << 8) | lsb;
      }
    }
    return values;
  }

  uint16_t read_max(Channel ch, std::error_code &ec) {
    if (!is_analog_input(ch)) {
      logger_.error("Channel {} is not configured as an analog input", ch);
      ec = std::make_error_code(std::errc::invalid_argument);
      return 0;
    }
    if (!statistics_enabled_) {
      logger_.error("Statistics are not enabled, cannot read max value");
      ec = std::make_error_code(std::errc::protocol_error);
      return 0;
    }
    logger_.info("Reading max value for channel {}", ch);
    int channel = static_cast<int>(ch);
    // read both the LSB AND MSB registers and combine them
    return read_two_(
        static_cast<Register>(static_cast<uint8_t>(Register::MAX_CH0_LSB) + channel * 2), ec);
  }

  std::vector<uint16_t> read_max_all(std::error_code &ec) {
    if (!statistics_enabled_) {
      logger_.error("Statistics are not enabled, cannot read max value");
      ec = std::make_error_code(std::errc::protocol_error);
      return {};
    }
    logger_.info("Reading max values for all channels");
    std::vector<uint16_t> values(analog_inputs_.size());
    uint8_t raw_values[16];
    int analog_index = 0;
    read_block_(Register::MAX_CH0_LSB, raw_values, 16, ec);
    if (ec)
      return {};
    // only pull out the ones that were configured as analog inputs
    for (int i = 0; i < 8; i++) {
      if (is_analog_input(static_cast<Channel>(i))) {
        // read both the LSB AND MSB registers and combine them (lsb first)
        uint8_t lsb = raw_values[i * 2];
        uint8_t msb = raw_values[i * 2 + 1];
        values[analog_index++] = (msb << 8) | lsb;
      }
    }
    return values;
  }

  uint16_t read_min(Channel ch, std::error_code &ec) {
    if (!is_analog_input(ch)) {
      logger_.error("Channel {} is not configured as an analog input", ch);
      ec = std::make_error_code(std::errc::invalid_argument);
      return 0;
    }
    if (!statistics_enabled_) {
      logger_.error("Statistics are not enabled, cannot read min value");
      ec = std::make_error_code(std::errc::protocol_error);
      return 0;
    }
    logger_.info("Reading min value for channel {}", ch);
    int channel = static_cast<int>(ch);
    // read both the LSB AND MSB registers and combine them
    return read_two_(
        static_cast<Register>(static_cast<uint8_t>(Register::MIN_CH0_LSB) + channel * 2), ec);
  }

  std::vector<uint16_t> read_min_all(std::error_code &ec) {
    if (!statistics_enabled_) {
      logger_.error("Statistics are not enabled, cannot read min value");
      ec = std::make_error_code(std::errc::protocol_error);
      return {};
    }
    logger_.info("Reading min values for all channels");
    std::vector<uint16_t> values(analog_inputs_.size());
    uint8_t raw_values[16];
    int analog_index = 0;
    read_block_(Register::MIN_CH0_LSB, raw_values, 16, ec);
    if (ec)
      return {};
    // only pull out the ones that were configured as analog inputs
    for (int i = 0; i < 8; i++) {
      if (is_analog_input(static_cast<Channel>(i))) {
        // read both the LSB AND MSB registers and combine them (lsb first)
        uint8_t lsb = raw_values[i * 2];
        uint8_t msb = raw_values[i * 2 + 1];
        values[analog_index++] = (msb << 8) | lsb;
      }
    }
    return values;
  }

  void trigger_conversion(Channel ch, std::error_code &ec) {
    if (!is_analog_input(ch)) {
      logger_.error("Channel {} is not configured as an analog input", ch);
      ec = std::make_error_code(std::errc::invalid_argument);
      return;
    }
    logger_.info("Triggering conversion for channel {}", ch);
    // set the channel to sample
    select_channel(ch, ec);
    if (ec)
      return;
    // start the conversion
    start_conversion(ec);
  }

  /**
   * @brief Convert a raw ADC value to a voltage in mV.
   * @param raw Raw ADC value.
   * @return Voltage in mV.
   */
  float raw_to_mv(uint16_t raw) const {
    // we have a 16-bit ADC, so we can represent 2^16 = 65536 values
    // we were configured with avdd_mv_ as the reference voltage, so we
    // can represent avdd_mv_ volts with 65536 values
    // therefore, each value represents avdd_mv_ / 65536 volts.
    // multiply by 1000 to get mV
    return static_cast<float>(raw) * avdd_mv_ / 65536.0;
  }

  /**
   * @brief Convert a voltage in mV to a raw ADC value.
   * @param mv Voltage in mV.
   * @return Raw ADC value.
   */
  uint16_t mv_to_raw(float mv) const {
    // we have a 16-bit ADC, so we can represent 2^16 = 65536 values
    // we were configured with avdd_mv_ as the reference voltage, so we
    // can represent avdd_mv_ volts with 65536 values
    // therefore, each value represents avdd_mv_ / 65536 volts
    // divide by 1000 to get volts
    return static_cast<uint16_t>(mv / avdd_mv_ * 65536);
  }

  bool conversion_complete(std::error_code &ec) {
    if (data_format_ == DataFormat::RAW) {
      // TODO: figure out how to know when the conversion is complete in RAW
      // mode
      logger_.error("Unsupported data format: RAW for conversion complete, returning true");
      ec = std::make_error_code(std::errc::protocol_error);
      return true;
    } else if (data_format_ == DataFormat::AVERAGED) {
      // if we have enabled the OSR (averaging), then we need to wait (for
      // t_conv * OSR_CFG[2:0]) for the averaging to complete. We'll know it's
      // done when the OSR_DONE bit is set in the SYSTEM_STATUS register
      uint8_t status = read_one_(Register::SYSTEM_STATUS, ec);
      if (ec)
        return false;
      return (status & OSR_DONE) == OSR_DONE;
    } else {
      logger_.error("Unknown data format");
      ec = std::make_error_code(std::errc::protocol_error);
      return false;
    }
  }

  void wait_for_conversion(std::error_code &ec) {
    logger_.info("Waiting for conversion to complete");
    while (!conversion_complete(ec) && !ec) {
      // wait for the conversion to complete
    }
  }

  void select_channel(Channel channel, std::error_code &ec) {
    // only in manual mode
    if (mode_ != Mode::MANUAL) {
      logger_.error("Cannot select channel in non-manual mode");
      ec = std::make_error_code(std::errc::protocol_error);
      return;
    }
    logger_.info("Selecting channel {}", channel);
    // set the channel to sample
    write_one_(Register::CHANNEL_SEL, static_cast<uint8_t>(channel), ec);
  }

  void start_conversion(std::error_code &ec) {
    // we either stretch the I2C clock or we use the CNVST bit in the
    // GENERAL_CFG register to start the conversion. NOTE: clock stretching on
    // the I2C bus is supported on the ESP32 (for I2C master) via the i2c
    // timeout feature, up to a maximum of 13ms. For simplicity, we'll just
    // use the CNVST bit.
    logger_.info("Starting conversion");
    set_bits_(Register::GENERAL_CFG, CNVST, ec);
    // TODO: look into using the I2C timeout feature to stretch the clock?
  }

  void enable_threshold_comparison(bool enable, std::error_code &ec) {
    // configure the digital window comparator functionality
    if (enable) {
      set_bits_(Register::GENERAL_CFG, DWC_EN, ec);
    } else {
      clear_bits_(Register::GENERAL_CFG, DWC_EN, ec);
    }
    if (ec)
      return;
    logger_.info("Threshold comparison {}abled", enable ? "en" : "dis");
  }

  void set_analog_high_threshold(int channel, float mv, std::error_code &ec) {
    logger_.info("Setting high threshold for channel {} to {} mV", channel, mv);
    uint16_t threshold = mv_to_raw(mv);
    // write the 8 MSBs of the 12 bit value to the high threshold register
    write_one_(static_cast<Register>(static_cast<uint8_t>(Register::HIGH_TH_CH0) + channel * 4),
               threshold >> 4, ec);
    if (ec)
      return;
    // clear the 4 LSBs of the 12 bit value in the HYSTERESIS_CHx register
    clear_bits_(static_cast<Register>(static_cast<uint8_t>(Register::HYSTERESIS_CH0) + channel * 4),
                0xF0, ec);
    if (ec)
      return;
    // write the 4 LSBs of the 12 bit value to the HYSTERESIS_CHx register (as bits 7-4)
    set_bits_(static_cast<Register>(static_cast<uint8_t>(Register::HYSTERESIS_CH0) + channel * 4),
              (threshold & 0x0F) << 4, ec);
  }

  void set_analog_low_threshold(int channel, float mv, std::error_code &ec) {
    logger_.info("Setting low threshold for channel {} to {} mV", channel, mv);
    uint16_t threshold = mv_to_raw(mv);
    // write the 8 MSBs of the 12 bit value to the low threshold register
    write_one_(static_cast<Register>(static_cast<uint8_t>(Register::LOW_TH_CH0) + channel * 4),
               threshold >> 4, ec);
    if (ec)
      return;
    // clear the 4 LSBs of the 12 bit value in the EVENT_COUNT_CHx register (bits 7-4)
    clear_bits_(
        static_cast<Register>(static_cast<uint8_t>(Register::EVENT_COUNT_CH0) + channel * 4), 0xF0,
        ec);
    if (ec)
      return;
    // write the 4 LSBs of the 12 bit value to the EVENT_COUNT_CHx register (as bits 7-4)
    set_bits_(static_cast<Register>(static_cast<uint8_t>(Register::EVENT_COUNT_CH0) + channel * 4),
              (threshold & 0x0F) << 4, ec);
  }

  void set_event_region(int channel, EventRegion region, std::error_code &ec) {
    if (channel < 0 || channel > 7) {
      logger_.error("Invalid channel number: {}", channel);
      ec = std::make_error_code(std::errc::invalid_argument);
      return;
    }
    logger_.info("Setting event region for channel {} to {}", channel,
                 static_cast<uint8_t>(region));
    if (region == EventRegion::INSIDE_OR_LOW) {
      set_bits_(Register::EVENT_RGN, 1 << channel, ec);
    } else if (region == EventRegion::OUTSIDE_OR_HIGH) {
      clear_bits_(Register::EVENT_RGN, 1 << channel, ec);
    } else {
      logger_.error("Invalid event region: {}", static_cast<uint8_t>(region));
      ec = std::make_error_code(std::errc::invalid_argument);
      return;
    }
  }

  void set_event_region(int channel, AnalogEvent event, std::error_code &ec) {
    if (event == AnalogEvent::INSIDE) {
      set_event_region(channel, EventRegion::INSIDE_OR_LOW, ec);
    } else if (event == AnalogEvent::OUTSIDE) {
      set_event_region(channel, EventRegion::OUTSIDE_OR_HIGH, ec);
    } else {
      logger_.error("Invalid analog event: {}", static_cast<uint8_t>(event));
      ec = std::make_error_code(std::errc::invalid_argument);
    }
  }

  void set_event_region(int channel, DigitalEvent event, std::error_code &ec) {
    if (event == DigitalEvent::HIGH) {
      set_event_region(channel, EventRegion::OUTSIDE_OR_HIGH, ec);
    } else if (event == DigitalEvent::LOW) {
      set_event_region(channel, EventRegion::INSIDE_OR_LOW, ec);
    } else {
      logger_.error("Invalid digital event: {}", static_cast<uint8_t>(event));
      ec = std::make_error_code(std::errc::invalid_argument);
    }
  }

  /// @brief Configure an alert events for an analog channel.
  /// @param channel Channel to configure.
  /// @param low_threshold_mv Low threshold in mV.
  /// @param high_threshold_mv High threshold in mV.
  /// @param event_count Number of events to trigger an alert.
  /// @param ec Error code.
  /// @note The event count is the number of events (event_count + 1) that must occur before an
  /// alert is triggered.
  void configure_event(Channel ch, float low_threshold_mv, float high_threshold_mv, int event_count,
                       std::error_code &ec) {
    // ensure the channel is configured as an analog input
    if (!is_analog_input(ch)) {
      logger_.error("Channel {} is not configured as an analog input", ch);
      ec = std::make_error_code(std::errc::invalid_argument);
      return;
    }
    // ensure event count is between 0 and 15
    if (event_count < 0 || event_count > 15) {
      logger_.error("Invalid event count: {}, valid values: 0-15", event_count);
      ec = std::make_error_code(std::errc::invalid_argument);
      return;
    }
    logger_.info("Configuring event for channel {} with low threshold {} mV, high threshold {} mV, "
                 "and event count {}",
                 ch, low_threshold_mv, high_threshold_mv, event_count);
    int channel = static_cast<int>(ch);
    // convert the thresholds to raw values
    uint16_t low_threshold = mv_to_raw(low_threshold_mv);
    uint16_t high_threshold = mv_to_raw(high_threshold_mv);
    uint8_t data[] = {
        // hysteresis register contains hyseresis and 4 lsb high threshold
        static_cast<uint8_t>((high_threshold & 0x0F)
                             << 4), // NOTE: we're not setting the hysteresis
                                    // high threshold register contains 8 msb of high threshold
        static_cast<uint8_t>(high_threshold >> 4),
        // event count register contains 4 lsb (bits  7-4) of low threshold and event
        // count (bits 3-0)
        static_cast<uint8_t>((low_threshold & 0x0F) << 4 | event_count),
        // low threshold register contains 8 msb of low threshold
        static_cast<uint8_t>(low_threshold >> 4)};
    // write the data to the registers
    write_block_(
        static_cast<Register>(static_cast<uint8_t>(Register::HYSTERESIS_CH0) + channel * 4), data,
        sizeof(data), ec);
  }

  bool is_digital_input(Channel channel) {
    return std::find(digital_inputs_.begin(), digital_inputs_.end(), channel) !=
           digital_inputs_.end();
  }

  bool is_digital_output(Channel channel) {
    return std::find(digital_outputs_.begin(), digital_outputs_.end(), channel) !=
           digital_outputs_.end();
  }

  bool is_analog_input(Channel channel) {
    return std::find(analog_inputs_.begin(), analog_inputs_.end(), channel) != analog_inputs_.end();
  }

  // NOTE: this chip has specific read and write operation commands that are
  // used, so we don't use the subclass's read_* and write_* methods directly

  uint8_t read_one_(Register reg, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data = 0;
    uint8_t read_one_command[] = {OP_READ_ONE, (uint8_t)reg};
    write_then_read(read_one_command, sizeof(read_one_command), &data, 1, ec);
    if (ec) {
      return 0;
    }
    return data;
  }

  uint16_t read_two_(Register reg, std::error_code &ec) {
    uint8_t data[2];
    read_block_(reg, data, 2, ec);
    if (ec) {
      return 0;
    }
    // NOTE: registers are little endian (LSB first, then MSB) so if we want
    // to read the value of a 16 bit register we need to read the LSB first,
    // then the MSB and combine them into a 16 bit value
    return (data[1] << 8) | data[0];
  }

  void read_block_(Register reg, uint8_t *data, uint8_t len, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t read_block_command[] = {OP_READ_BLOCK, (uint8_t)reg};
    write_then_read(read_block_command, sizeof(read_block_command), data, len, ec);
  }

  void set_bits_(Register reg, uint8_t bit, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data[] = {OP_SET_BITS, (uint8_t)reg, bit};
    write_many(data, sizeof(data), ec);
  }

  void clear_bits_(Register reg, uint8_t bit, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data[] = {OP_CLR_BITS, (uint8_t)reg, bit};
    write_many(data, sizeof(data), ec);
  }

  void write_one_(Register reg, uint8_t value, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t data[] = {OP_WRITE_ONE, (uint8_t)reg, value};
    write_many(data, sizeof(data), ec);
  }

  void write_two_(Register reg, uint16_t value, std::error_code &ec) {
    write_block_(reg, (uint8_t *)&value, 2, ec);
  }

  void write_block_(Register reg, const uint8_t *data, uint8_t len, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t total_len = len + 2;
    uint8_t data_with_header[total_len];
    data_with_header[0] = OP_WRITE_BLOCK;
    data_with_header[1] = (uint8_t)reg;
    memcpy(data_with_header + 2, data, len);
    write_many(data_with_header, total_len, ec);
  }

  Config config_;

  Mode mode_;
  float avdd_mv_;
  DataFormat data_format_;
  bool statistics_enabled_{false};
  int num_data_bytes_{2}; // number of bytes for each conversion read, depends on setting of
                          // averaging mode and status/channelid append
  std::vector<Channel> analog_inputs_;
  std::vector<Channel> digital_inputs_;
  std::vector<Channel> digital_outputs_;
  OversamplingRatio oversampling_ratio_;
  std::recursive_mutex mutex_; ///< mutex for thread safety
};
} // namespace espp

#include "format.hpp"

// for allowing easy serialization/printing of the
// espp::Ads7138::OverSamplingRatio enum
template <> struct fmt::formatter<espp::Ads7138::OversamplingRatio> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(espp::Ads7138::OversamplingRatio const &ratio, FormatContext &ctx) {
    switch (ratio) {
    case espp::Ads7138::OversamplingRatio::NONE:
      return fmt::format_to(ctx.out(), "NONE");
    case espp::Ads7138::OversamplingRatio::OSR_2:
      return fmt::format_to(ctx.out(), "OSR_2 (2x)");
    case espp::Ads7138::OversamplingRatio::OSR_4:
      return fmt::format_to(ctx.out(), "OSR_4 (4x)");
    case espp::Ads7138::OversamplingRatio::OSR_8:
      return fmt::format_to(ctx.out(), "OSR_8 (8x)");
    case espp::Ads7138::OversamplingRatio::OSR_16:
      return fmt::format_to(ctx.out(), "OSR_16 (16x)");
    case espp::Ads7138::OversamplingRatio::OSR_32:
      return fmt::format_to(ctx.out(), "OSR_32 (32x)");
    case espp::Ads7138::OversamplingRatio::OSR_64:
      return fmt::format_to(ctx.out(), "OSR_64 (64x)");
    case espp::Ads7138::OversamplingRatio::OSR_128:
      return fmt::format_to(ctx.out(), "OSR_128 (128x)");
    default:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    }
  }
};

// for allowing easy serialization/printing of the
// espp::Ads7138::Mode enum
template <> struct fmt::formatter<espp::Ads7138::Mode> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(espp::Ads7138::Mode const &mode, FormatContext &ctx) {
    switch (mode) {
    case espp::Ads7138::Mode::MANUAL:
      return fmt::format_to(ctx.out(), "MANUAL");
    case espp::Ads7138::Mode::AUTO_SEQ:
      return fmt::format_to(ctx.out(), "AUTO_SEQ");
    case espp::Ads7138::Mode::AUTONOMOUS:
      return fmt::format_to(ctx.out(), "AUTONOMOUS");
    default:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    }
  }
};

// for allowing easy serialization/printing of the
// espp::Ads7138::DataFormat enum
template <> struct fmt::formatter<espp::Ads7138::DataFormat> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(espp::Ads7138::DataFormat const &format, FormatContext &ctx) {
    switch (format) {
    case espp::Ads7138::DataFormat::RAW:
      return fmt::format_to(ctx.out(), "RAW");
    case espp::Ads7138::DataFormat::AVERAGED:
      return fmt::format_to(ctx.out(), "AVERAGED");
    default:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    }
  }
};

// for allowing easy serialization/printing of the
// espp::Ads7138::Append enum
template <> struct fmt::formatter<espp::Ads7138::Append> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(espp::Ads7138::Append const &format, FormatContext &ctx) {
    switch (format) {
    case espp::Ads7138::Append::NONE:
      return fmt::format_to(ctx.out(), "NONE");
    case espp::Ads7138::Append::STATUS:
      return fmt::format_to(ctx.out(), "STATUS");
    case espp::Ads7138::Append::CHANNEL_ID:
      return fmt::format_to(ctx.out(), "CHANNEL_ID");
    default:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    }
  }
};

// for allowing easy serialization/printing of the
// espp::Ads7138::Channel enum
template <> struct fmt::formatter<espp::Ads7138::Channel> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(espp::Ads7138::Channel const &ch, FormatContext &ctx) {
    switch (ch) {
    case espp::Ads7138::Channel::CH0:
      return fmt::format_to(ctx.out(), "CH0");
    case espp::Ads7138::Channel::CH1:
      return fmt::format_to(ctx.out(), "CH1");
    case espp::Ads7138::Channel::CH2:
      return fmt::format_to(ctx.out(), "CH2");
    case espp::Ads7138::Channel::CH3:
      return fmt::format_to(ctx.out(), "CH3");
    case espp::Ads7138::Channel::CH4:
      return fmt::format_to(ctx.out(), "CH4");
    case espp::Ads7138::Channel::CH5:
      return fmt::format_to(ctx.out(), "CH5");
    case espp::Ads7138::Channel::CH6:
      return fmt::format_to(ctx.out(), "CH6");
    case espp::Ads7138::Channel::CH7:
      return fmt::format_to(ctx.out(), "CH7");
    default:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    }
  }
};

// for allowing easy serialization/printing of a
// std::vector<espp::Ads7138::Channel> object
template <> struct fmt::formatter<std::vector<espp::Ads7138::Channel>> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(std::vector<espp::Ads7138::Channel> const &channels, FormatContext &ctx) {
    std::string result = "{";
    for (auto const &ch : channels) {
      result += fmt::format("{}, ", ch);
    }
    if (result.size() > 1)
      result.resize(result.size() - 2); // remove trailing ", "
    result += "}";
    return fmt::format_to(ctx.out(), "{}", result);
  }
};

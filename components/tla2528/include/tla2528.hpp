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
 * @brief Class for reading values from the TLA2528 family of ADC chips.
 * @details The TLA2528 is a 16-bit, 8-channel ADC with 8 digital I/O pins. It
 *          supports a variety of sampling modes, including manual sampling, and
 *          auto sequence sampling. It also supports oversampling ratios of 2,
 *          4, 8, 16, 32, 64, and 128. It additionally allows the user to
 *          configure the analog or digital inputs to trigger an alert when the
 *          value goes above or below a threshold (enter or leave a region of
 *          voltage).
 * @see https://www.ti.com/lit/ds/symlink/tla2528.pdf
 *
 * \section tla2528_ex1 TLA2528 Example
 * \snippet tla2528_example.cpp tla2528 example
 */
class Tla2528 : public BasePeripheral<> {
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
  /// @details The TLA2528 has 8 channels, see data sheet Table 1 (p. 4)
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
  enum class Mode : uint8_t {
    MANUAL = 0, ///< Manual mode (9th falling edge of SCL (ACK) triggers conversion) and the MUX is
                ///< controlled by register write to MANUAL_CHID field of the CHANNEL_SEL register.
    AUTO_SEQ = 1, ///< Auto sequence mode (9th falling edge of SCL (ACK) triggers conversion) and
                  ///< the MUX is incremented after each conversion.
  };

  /// @brief Output mode for digital output channels and ALERT pin
  enum class OutputMode : uint8_t {
    OPEN_DRAIN = 0, ///< Open drain output mode
    PUSH_PULL = 1,  ///< Push-pull output mode
  };

  /// \brief Enum for the data format that can be read from the ADC
  enum class DataFormat : uint8_t {
    RAW = 0,      ///< Raw data format, 12 bit ADC data
    AVERAGED = 1, ///< Averaged data format, 16 bit ADC data
  };

  /// \brief Enum for the different configurations of bits that can be
  ///        appended to the data when reading from the ADC
  enum class Append : uint8_t {
    NONE = 0,       ///< No append
    CHANNEL_ID = 1, ///< Append Channel ID
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
    float avdd_volts = 3.3f;  ///< AVDD voltage in Volts. Used for calculating analog input voltage.
    Mode mode = Mode::MANUAL; ///< Mode for analog input conversion.
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
    Append append = Append::NONE;   ///< What data to append to samples when reading analog inputs.
    BasePeripheral::write_fn write; ///< Function to write to the ADC
    BasePeripheral::read_fn read;   ///< Function to read from the ADC
    bool auto_init = true;          ///< Automatically initialize the ADC on construction. If false,
                                    ///< initialize() must be called before any other functions.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Verbosity for the logger.
  };

  /**
   * @brief Construct Tla2528
   * @param config Configuration structure.
   */
  explicit Tla2528(const Config &config)
      : BasePeripheral(
            {.address = config.device_address, .write = config.write, .read = config.read},
            "Tla2528", config.log_level)
      , config_(config)
      , mode_(config.mode)
      , avdd_mv_(config.avdd_volts * 1000.0f)
      , data_format_(config.oversampling_ratio == OversamplingRatio::NONE ? DataFormat::RAW
                                                                          : DataFormat::AVERAGED)
      , append_(config.append)
      , analog_inputs_(config.analog_inputs)
      , digital_inputs_(config.digital_inputs)
      , digital_outputs_(config.digital_outputs)
      , oversampling_ratio_(config.oversampling_ratio) {
    num_bytes_per_sample_ = 2;
    if (data_format_ == DataFormat::AVERAGED && append_ == Append::CHANNEL_ID) {
      num_bytes_per_sample_ = 3;
    }
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
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // we need to trigger a conversion and then read the result
    trigger_conversion(channel, ec);
    if (ec) {
      return 0.0f;
    }
    uint8_t data[num_bytes_per_sample_];
    read_many(data, num_bytes_per_sample_, ec);
    if (ec) {
      return 0.0f;
    }
    uint16_t raw = parse_frame(data);
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
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // TODO: handle the non-autonomous case
    auto raw_values = read_all(ec);
    if (ec) {
      return {};
    }
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
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    std::unordered_map<Channel, float> values;
    // TODO: handle the non-autonomous case
    auto raw_values = read_all_map(ec);
    if (ec) {
      return {};
    }
    // convert the raw values (uint16_t) to mv (float)
    for (auto &[channel, raw] : raw_values) {
      values[channel] = raw_to_mv(raw);
    }
    return values;
  }

  /// @brief Configure the digital output mode for the given channel.
  /// @param channel Channel to configure
  /// @param output_mode Output mode for the channel
  /// @param ec Error code to set if an error occurs.
  /// @note The channel must have been configured as a digital output.
  void set_digital_output_mode(Channel channel, OutputMode output_mode, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
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
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
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
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    if (!is_digital_input(channel)) {
      logger_.error("Channel {} is not configured as a digital input", channel);
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    uint8_t value = read_one_(Register::GPI_VALUE, ec);
    if (ec) {
      return false;
    }
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
  void reset(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // reset the device
    write_one_(Register::GENERAL_CFG, SW_RST, ec);
    if (ec) {
      return;
    }
    // wait for the reset to complete
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

protected:
  /// Opcodes for the TLA2528, see data sheet Table 9 (p. 26)
  /// @{
  /// @brief Opcode for the TLA2528, see data sheet Table 9 (p. 26)
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
  static constexpr int BOR =
      (1 << 0); ///< Brown-out reset bit (0 = no reset, 1 = brown out condition detected)
  /// @}

  /// @{
  /// @brief Bit masks for the bits in the GENERAL_CFG register, see data
  ///        sheet Table 13 (p. 33)
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
  static constexpr int APPEND = (1 << 4);  ///< Append 4-bit channel ID or status flags to output
                                           ///< data. 00 = no append, 01 = append channel ID
  static constexpr int APPEND_NONE = 0;    ///< Raw data format (no append)
  static constexpr int APPEND_CHID = (1 << 4); ///< Append 4-bit channel ID to output data
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

  /// @brief Register addresses for the TLA2528, see data sheet Table 10 (p. 29)
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

    SEQUENCE_CFG = 0x10,    ///< Sequence configuration register
    CHANNEL_SEL = 0x11,     ///< Channel selection register
    AUTO_SEQ_CH_SEL = 0x12, ///< Auto sequence channel selection register
  };

  /// @brief Mode for scanning of analog inputs
  enum class SequenceMode : uint8_t {
    MANUAL = 0, ///< Manual mode
    AUTO = 1,   ///< Auto mode
  };

  void init(const Config &config, std::error_code &ec) {
    // Set the data format
    set_data_format(data_format_, append_, ec);
    if (ec)
      return;

    // Set the oversampling ratio
    set_oversampling_ratio(config.oversampling_ratio, ec);
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
    data_format_ = format;
    logger_.info("Data format set to {}", data_format_);
    append_ = append;
    logger_.info("Append set to {}", append_);
    clear_bits_(Register::DATA_CFG, APPEND, ec);
    if (ec)
      return;
    if (append_ == Append::CHANNEL_ID) {
      logger_.debug("Appending channel ID");
      set_bits_(Register::DATA_CFG, APPEND_CHID, ec);
    }
  }

  void set_oversampling_ratio(OversamplingRatio ratio, std::error_code &ec) {
    uint8_t data = static_cast<uint8_t>(ratio);
    write_one_(Register::OSR_CFG, data, ec);
  }

  static uint8_t bit_pred(uint8_t value, const Channel channel) {
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
    if (mode_ == Mode::AUTO_SEQ) {
      logger_.info("Setting analog inputs for autonomous mode");
      // configure the analog inputs for autonomous conversion sequence
      uint8_t data = std::accumulate(analog_inputs_.begin(), analog_inputs_.end(), 0, bit_pred);
      write_one_(Register::AUTO_SEQ_CH_SEL, data, ec);
    }
  }

  /// \brief Sets the operational mode of the device
  /// \param ec Error code to set if an error occurs.
  /// \details Set the operational mode based on the value of the mode_
  ///         variable. The SEQ_MODE bits in the SEQUENCE_CFG register are also
  ///         set based on the mode_ member variable.
  /// \sa The Mode enum for more information on the different modes.
  /// \sa The mode_ member variable.
  /// \sa Table 7 (page 22) of the datasheet for more information on the
  ///     functional modes.
  void set_operational_mode(std::error_code &ec) {
    // set the SEQ_MODE bits in SEQUENCE_CFG based on mode_
    if (mode_ == Mode::MANUAL) {
      set_sequence_mode(SequenceMode::MANUAL, ec);
    } else {
      set_sequence_mode(SequenceMode::AUTO, ec);
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

  std::vector<uint16_t> read_all(std::error_code &ec) {
    if (mode_ == Mode::MANUAL) {
      logger_.error("cannot read_all when in MANUAL mode!");
      ec = std::make_error_code(std::errc::protocol_error);
      return {};
    }
    logger_.info("Reading recent values for all channels");
    size_t num_inputs = analog_inputs_.size();
    std::vector<uint16_t> values(num_inputs);
    size_t num_bytes = num_inputs * num_bytes_per_sample_;
    uint8_t raw_values[num_bytes];
    // start the auto conversion sequence
    start_auto_conversion(ec);
    if (ec) {
      return {};
    }
    read_many(raw_values, num_bytes, ec);
    if (ec) {
      return {};
    }
    // stop the auto conversion sequence
    stop_auto_conversion(ec);
    if (ec) {
      return {};
    }
    int analog_index = 0;
    // only pull out the ones that were configured as analog inputs
    for (int i = 0; i < num_bytes; i += num_bytes_per_sample_) {
      values[analog_index++] = parse_frame(&raw_values[i]);
    }
    return values;
  }

  uint16_t parse_frame(const uint8_t *frame_ptr) {
    uint8_t msb = frame_ptr[0];
    uint8_t lsb = frame_ptr[1];
    uint8_t channel_id = (append_ == Append::CHANNEL_ID) ? (lsb & 0x0F) : 0;
    uint16_t value = 0;
    if (num_bytes_per_sample_ == 3) {
      // we are averaging and we have channel id
      channel_id = frame_ptr[2] >> 4;
    }
    if (data_format_ == DataFormat::RAW) {
      // if it's raw, then it's only 12 bit instead of 16 bit
      value = (msb << 4) | (lsb >> 4);
    } else {
      value = (msb << 8) | lsb;
    }
    logger_.debug("Got channel id: {}", channel_id);
    logger_.debug("         value: {}", value);
    return value;
  }

  std::unordered_map<Channel, uint16_t> read_all_map(std::error_code &ec) {
    auto raw_values = read_all(ec);
    if (ec) {
      return {};
    }
    // convert the vector (in order of analog_inputs) into a map
    std::unordered_map<Channel, uint16_t> map_values;
    for (int i = 0; i < analog_inputs_.size(); i++) {
      auto ch = analog_inputs_[i];
      map_values[ch] = raw_values[i];
    }
    return map_values;
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
  }

  /**
   * @brief Convert a raw ADC value to a voltage in mV.
   * @param raw Raw ADC value.
   * @return Voltage in mV.
   */
  float raw_to_mv(uint16_t raw) const {
    if (data_format_ == DataFormat::AVERAGED) {
      // we have a 16-bit ADC, so we can represent 2^16 = 65536 values
      // we were configured with avdd_mv_ as the reference voltage, so we
      // can represent avdd_mv_ volts with 65536 values
      // therefore, each value represents avdd_mv_ / 65536 volts.
      // multiply by 1000 to get mV
      return static_cast<float>(raw) * avdd_mv_ / 65536.0;
    } else {
      // we have a 12-bit ADC, so we can represent 2^12 = 4096 values
      // we were configured with avdd_mv_ as the reference voltage, so we
      // can represent avdd_mv_ volts with 4096 values
      // therefore, each value represents avdd_mv_ / 4096 volts.
      // multiply by 1000 to get mV
      return static_cast<float>(raw) * avdd_mv_ / 4096.0;
    }
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
      ec = std::make_error_code(std::errc::not_supported);
      return true;
    } else if (data_format_ == DataFormat::AVERAGED) {
      // if we have enabled the OSR (averaging), then we need to wait (for
      // t_conv * OSR_CFG[2:0]) for the averaging to complete. We'll know it's
      // done when the OSR_DONE bit is set in the SYSTEM_STATUS register
      uint8_t status = read_one_(Register::SYSTEM_STATUS, ec);
      if (ec) {
        return false;
      }
      return (status & OSR_DONE) == OSR_DONE;
    } else {
      logger_.error("Unknown data format");
      ec = std::make_error_code(std::errc::not_supported);
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
  Append append_;
  size_t num_bytes_per_sample_{2};
  std::vector<Channel> analog_inputs_;
  std::vector<Channel> digital_inputs_;
  std::vector<Channel> digital_outputs_;
  OversamplingRatio oversampling_ratio_;
};
} // namespace espp

#include "format.hpp"

// for allowing easy serialization/printing of the
// espp::Tla2528::OverSamplingRatio enum
template <> struct fmt::formatter<espp::Tla2528::OversamplingRatio> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(espp::Tla2528::OversamplingRatio const &ratio, FormatContext &ctx) {
    switch (ratio) {
    case espp::Tla2528::OversamplingRatio::NONE:
      return fmt::format_to(ctx.out(), "NONE");
    case espp::Tla2528::OversamplingRatio::OSR_2:
      return fmt::format_to(ctx.out(), "OSR_2 (2x)");
    case espp::Tla2528::OversamplingRatio::OSR_4:
      return fmt::format_to(ctx.out(), "OSR_4 (4x)");
    case espp::Tla2528::OversamplingRatio::OSR_8:
      return fmt::format_to(ctx.out(), "OSR_8 (8x)");
    case espp::Tla2528::OversamplingRatio::OSR_16:
      return fmt::format_to(ctx.out(), "OSR_16 (16x)");
    case espp::Tla2528::OversamplingRatio::OSR_32:
      return fmt::format_to(ctx.out(), "OSR_32 (32x)");
    case espp::Tla2528::OversamplingRatio::OSR_64:
      return fmt::format_to(ctx.out(), "OSR_64 (64x)");
    case espp::Tla2528::OversamplingRatio::OSR_128:
      return fmt::format_to(ctx.out(), "OSR_128 (128x)");
    default:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    }
  }
};

// for allowing easy serialization/printing of the
// espp::Tla2528::Mode enum
template <> struct fmt::formatter<espp::Tla2528::Mode> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(espp::Tla2528::Mode const &mode, FormatContext &ctx) {
    switch (mode) {
    case espp::Tla2528::Mode::MANUAL:
      return fmt::format_to(ctx.out(), "MANUAL");
    case espp::Tla2528::Mode::AUTO_SEQ:
      return fmt::format_to(ctx.out(), "AUTO_SEQ");
    default:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    }
  }
};

// for allowing easy serialization/printing of the
// espp::Tla2528::DataFormat enum
template <> struct fmt::formatter<espp::Tla2528::DataFormat> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(espp::Tla2528::DataFormat const &format, FormatContext &ctx) {
    switch (format) {
    case espp::Tla2528::DataFormat::RAW:
      return fmt::format_to(ctx.out(), "RAW");
    case espp::Tla2528::DataFormat::AVERAGED:
      return fmt::format_to(ctx.out(), "AVERAGED");
    default:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    }
  }
};

// for allowing easy serialization/printing of the
// espp::Tla2528::Append enum
template <> struct fmt::formatter<espp::Tla2528::Append> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(espp::Tla2528::Append const &format, FormatContext &ctx) {
    switch (format) {
    case espp::Tla2528::Append::NONE:
      return fmt::format_to(ctx.out(), "NONE");
    case espp::Tla2528::Append::CHANNEL_ID:
      return fmt::format_to(ctx.out(), "CHANNEL_ID");
    default:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    }
  }
};

// for allowing easy serialization/printing of the
// espp::Tla2528::Channel enum
template <> struct fmt::formatter<espp::Tla2528::Channel> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(espp::Tla2528::Channel const &ch, FormatContext &ctx) {
    switch (ch) {
    case espp::Tla2528::Channel::CH0:
      return fmt::format_to(ctx.out(), "CH0");
    case espp::Tla2528::Channel::CH1:
      return fmt::format_to(ctx.out(), "CH1");
    case espp::Tla2528::Channel::CH2:
      return fmt::format_to(ctx.out(), "CH2");
    case espp::Tla2528::Channel::CH3:
      return fmt::format_to(ctx.out(), "CH3");
    case espp::Tla2528::Channel::CH4:
      return fmt::format_to(ctx.out(), "CH4");
    case espp::Tla2528::Channel::CH5:
      return fmt::format_to(ctx.out(), "CH5");
    case espp::Tla2528::Channel::CH6:
      return fmt::format_to(ctx.out(), "CH6");
    case espp::Tla2528::Channel::CH7:
      return fmt::format_to(ctx.out(), "CH7");
    default:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    }
  }
};

// for allowing easy serialization/printing of a
// std::vector<espp::Tla2528::Channel> object
template <> struct fmt::formatter<std::vector<espp::Tla2528::Channel>> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(std::vector<espp::Tla2528::Channel> const &channels, FormatContext &ctx) {
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

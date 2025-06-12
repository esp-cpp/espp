#pragma once

#include <functional>

#include "base_peripheral.hpp"

namespace espp {
/**
 * @brief Class for controlling the Texas Instruments DRV2605 Haptic Motor
 *        Driver. Drives ECM (eccentric rotating mass) and LRA (linear
 *        resonant actuator) types of haptic motors. The datasheet for the
 *        DRV2605 can be found here:
 *        https://www.ti.com/lit/ds/symlink/drv2605.pdf?ts=1678892742599.
 *
 * \section drv2605_ex1 DRV2605 Example
 * \snippet drv2605_example.cpp drv2605 example
 */
class Drv2605 : public BasePeripheral<> {
public:
  static constexpr uint8_t DEFAULT_ADDRESS =
      (0x5A); ///< Default I2C address of the DRV2605. NOTE: this cannot be changed, as the DRV2605
              ///< does not support changing its I2C address.

  /**
   * @brief Convert LRA frequency to drive time.
   * @details The drive time is half the period of the LRA frequency, in ms.
   *          The LRA frequency is in Hz, so we need to convert it to ms.
   *          The formula is:
   *            - drive_time_ms = 1000 / (2 * lra_freq).
   *            - drive_time = (drive_time_ms - 0.5) / 0.1
   * @param lra_freq The frequency of the LRA in Hz.
   * @return The drive time in bits (0-31).
   */
  static uint8_t lra_freq_to_drive_time(float lra_freq) {
    // drive time is half the period of the LRA frequency, in ms
    // lra_freq is in Hz, so we need to convert it to ms
    // drive_time = 1000 / (2 * lra_freq)
    float drive_time_ms = 1000.0f / (2.0f * lra_freq);
    // convert to value for drive_time bits
    return static_cast<uint8_t>((drive_time_ms - 0.5f) / 0.1f) & 0x1F; // 0.1 ms resolution, 5 bits
  }

  /**
   * @brief The mode of the vibration.
   */
  enum class Mode : uint8_t {
    INTTRIG,     ///< Internal Trigger (call star() to start playback)
    EXTTRIGEDGE, ///< External edge trigger (rising edge on IN pin starts playback)
    EXTTRIGLVL,  ///< External level trigger (playback follows state of IN pin)
    PWMANALOG,   ///< PWM/Analog input
    AUDIOVIBE,   ///< Audio-to-vibe mode
    REALTIME,    ///< Real-time playback (RTP). Drives the actuator based on the value of the RTPIN
                 ///< register.
    DIAGNOS,     ///< Diagnostics
    AUTOCAL,     ///< Auto-calibration. User must set all required input parameters then set the
             ///< GO/START bit to start. Calibration is complete when the GO/START bit self-clears.
  };

  /**
   * @brief The waveforms supported by the DRV2605. It has 123 different
   *        waveforms, with waveform ID 0 being a special _END_ identifier
   *        used when playing multiple waveforms in sequence.
   * @note All of the waveform names have not been enumerated here. For a
   *       complete list, please see https://learn.adafruit.com/assets/72593
   */
  enum class Waveform : uint8_t {
    END = 0, ///< Signals this is the end of the waveforms to play
    STRONG_CLICK = 1,
    SHARP_CLICK = 4,
    SOFT_BUMP = 7,
    DOUBLE_CLICK = 10,
    TRIPLE_CLICK = 12,
    SOFT_FUZZ = 13,
    STRONG_BUZZ = 14,
    ALERT_750MS = 15,
    ALERT_1000MS = 16, // omg there are 123 of theese i'm not typing them out right now...
    BUZZ1 = 47,
    BUZZ2 = 48,
    BUZZ3 = 49,
    BUZZ4 = 50,
    BUZZ5 = 51,
    PULSING_STRONG_1 = 52,
    PULSING_STRONG_2 = 53,
    TRANSITION_CLICK_1 = 58,
    TRANSITION_HUM_1 = 64,
    MAX = 124, ///< Values >= to this do not correspond to valid waveforms
  };

  /**
   *  @brief The type of vibration motor connected to the Drv2605
   */
  enum class MotorType {
    ERM, ///< Eccentric Rotating Mass (more common, therefore default)
    LRA  ///< Linear Resonant Actuator
  };

  /**
   * @brief The library of waveforms to use.
   * @note The DRV2605 has 7 different libraries of waveforms. The first
   *       library is empty, and the next 5 are ERM (eccentric rotating mass)
   *       libraries. The last library is an LRA (linear resonant actuator)
   *       library.
   */
  enum class Library {
    EMPTY = 0,
    ERM_0 = 1,
    ERM_1 = 2,
    ERM_2 = 3,
    ERM_3 = 4,
    ERM_4 = 5,
    LRA = 6,
  };

  /**
   * @brief Calibration Routine Settings structure for the DRV2605.
   */
  struct BaseCalibrationSettings {
    uint8_t fb_brake_factor : 3 = 2; /**< Feedback brake factor, 0-7. */
    uint8_t loop_gain : 2 = 2;       /**< Loop gain, 0-3. */
    uint8_t rated_voltage : 8;       /**< Rated voltage, 0-255. */
    uint8_t overdrive_clamp : 8 = 0; /**< Overdrive clamp, 0-255. */
    uint8_t auto_cal_time : 2 = 3;   /**< Auto calibration time, 0-3. */
    uint8_t drive_time : 5; /**< Drive time, 0-31. For LRA it should be used as an initial guess for
                               the half-period. E.g. if the resonance freq is 200 Hz, then drive
                               time should be 2.5ms. For ERM it controls the rate for back-EMF
                               sampling. Higher drive times imply lower back-EMF sampling freq which
                               cause the feedback to react at slower rate. */
  };

  /**< Calibration Routine Settings structure for ERM motors. */
  using ErmCalibrationSettings = BaseCalibrationSettings;

  /**< Calibration Routine Settings structure for LRA motors. */
  struct LraCalibrationSettings : BaseCalibrationSettings {
    uint8_t sample_time : 2 = 3;   /**< Sample time, 0-3. */
    uint8_t blanking_time : 2 = 1; /**< Blanking time, 0-3. */
    uint8_t idiss_time : 2 = 1;    /**< IDISS time, 0-3. */
  };

  /**
   * @brief Structure to hold the calibrated data for the DRV2605.
   */
  struct CalibratedData {
    uint8_t bemf_gain : 2;  /**< Back-EMF gain, 0-3. */
    uint8_t a_cal_comp : 8; /**< Auto calibration compensation result, 0-255. */
    uint8_t a_cal_bemf : 8; /**< Auto calibration back-EMF result, 0-255. */
  };

  /**
   * @brief Configuration structure for the DRV2605
   */
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS; /**< I2C address of the device. */
    BasePeripheral::write_fn
        write; /**< Function for writing a byte to a register on the Drv2605. */
    BasePeripheral::read_register_fn
        read_register;                    /**< Function for reading a register from the Drv2605. */
    MotorType motor_type{MotorType::ERM}; /**< MotorType that this driver is driving. */
    bool auto_init{true}; /**< If true, the driver will initialize the DRV2605 on construction. */
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; /**< Log verbosity for the Drv2605.  */
  };

  /**
   * @brief Construct and initialize the DRV2605.
   */
  explicit Drv2605(const Config &config)
      : BasePeripheral({.address = config.device_address,
                        .write = config.write,
                        .read_register = config.read_register},
                       "Drv2605", config.log_level)
      , motor_type_(config.motor_type) {
    if (config.auto_init) {
      std::error_code ec;
      initialize(ec);
      if (ec) {
        logger_.error("Failed to initialize: {}", ec.message());
      }
    }
  }

  /**
   * @brief Initialize the DRV2605.
   * @param ec Error code to set if there is an error.
   * @return true if the initialization was successful, false if there was an
   *         error.
   */
  bool initialize(std::error_code &ec) { return init(ec); }

  /**
   * @brief Reset the DRV2605.
   * @param ec Error code to set if there is an error.
   * @return true if the reset was successful, false if there was an error.
   */
  bool reset(std::error_code &ec) {
    // reset is bit 7 of the MODE register, so we want to set that bit
    logger_.info("Resetting DRV2605");
    static constexpr uint8_t RESET_BIT_MASK = 0x80; // bit 7
    set_bits_in_register((uint8_t)Register::MODE, RESET_BIT_MASK, ec);
    return !ec; // return true if no error
  }

  /**
   * @brief Set the standby mode of the DRV2605.
   * @param standby_enabled If true, the DRV2605 will enter standby mode.
   * @param ec Error code to set if there is an error.
   * @return true if the standby mode was set successfully, false if there was
   *         an error.
   */
  bool set_standby(bool standby_enabled, std::error_code &ec) {
    // standby is bit 6 of the MODE register, so we want to set or clear that
    // bit
    logger_.info("{} standby mode", standby_enabled ? "Enabling" : "Disabling");
    static constexpr uint8_t STANDBY_BIT_MASK = 0x40; // bit 6
    if (standby_enabled) {
      set_bits_in_register((uint8_t)Register::MODE, STANDBY_BIT_MASK, ec);
    } else {
      clear_bits_in_register((uint8_t)Register::MODE, STANDBY_BIT_MASK, ec);
    }
    return !ec; // return true if no error
  }

  /**
   * @brief Set the mode of the vibration.
   * @param mode Mode to set to.
   * @param ec Error code to set if there is an error.
   * @return true if the mode was set successfully, false if there was an error.
   */
  bool set_mode(Mode mode, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    logger_.info("Setting mode {}", (uint8_t)mode);
    // mode is bits 0-2 of the MODE register, so we want to set those bits
    static constexpr uint8_t MODE_MASK = 0x07; // bits 0-2
    set_bits_in_register_by_mask((uint8_t)Register::MODE, MODE_MASK, (uint8_t)mode, ec);
    if (!ec) {
      mode_ = mode; // update the current mode
    } else {
      logger_.error("Failed to set mode: {}", ec.message());
    }
    return !ec; // return true if no error
  }

  /**
   * @brief Start playing the configured waveform / sequence.
   * @param ec Error code to set if there is an error.
   * @return true if the start was successful, false if there was an error.
   */
  bool start(std::error_code &ec) {
    logger_.info("Starting");
    write_u8_to_register((uint8_t)Register::START, 1, ec);
    return !ec; // return true if no error
  }

  /**
   * @brief Stop playing the waveform / sequence.
   * @param ec Error code to set if there is an error.
   * @return true if the stop was successful, false if there was an error.
   */
  bool stop(std::error_code &ec) {
    logger_.info("Stopping");
    write_u8_to_register((uint8_t)Register::START, 0, ec);
    return !ec; // return true if no error
  }

  /**
   * @brief Set the RTP (Real-Time Playback) data format.
   * @details This method sets the data format for the RTP mode. The DRV2605
   *          supports both signed and unsigned data formats for the RTP mode.
   *          The default data format is signed, but you can change it to
   *          unsigned by calling this method with `use_unsigned` set to true.
   * @note This is only valid when the mode is set to Mode::REALTIME.
   * @param use_unsigned If true, the data format will be set to unsigned,
   *                     otherwise it will be set to signed.
   * @param ec Error code to set if there is an error.
   * @return true if the data format was set successfully, false if there was
   *         an error.
   */
  bool set_rtp_data_format_unsigned(bool use_unsigned, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // set / clear the DATA_FORMAT_RTP bit (BIT 3) in register 0x1D (CONTROL3)
    logger_.info("Setting RTP data format to {}", use_unsigned ? "unsigned" : "signed");
    static constexpr uint8_t DATA_FORMAT_RTP_BIT_MASK = 0x08; // bit 3
    if (use_unsigned) {
      set_bits_in_register((uint8_t)Register::CONTROL3, DATA_FORMAT_RTP_BIT_MASK, ec); // set bit 3
    } else {
      clear_bits_in_register((uint8_t)Register::CONTROL3, DATA_FORMAT_RTP_BIT_MASK,
                             ec); // clear bit 3
    }
    return !ec; // return true if no error
  }

  /**
   * @brief Set the RTP (Real-Time Playback) mode PWM value as unsigned.
   * @note This is only valid when the mode is set to Mode::REALTIME.
   * @note This is only valid when the RTP data format is set to unsigned
   *       (requires set_rtp_data_format_unsigned(true)).
   * @param pwm_value The PWM value to set (0-255).
   * @param ec Error code to set if there is an error.
   * @return true if the PWM value was set successfully, false if there was an
   *         error.
   */
  bool set_rtp_pwm_unsigned(uint8_t pwm_value, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // we log a warning if mode_ is not Mode::REALTIME
    if (mode_ != Mode::REALTIME) {
      logger_.warn("Setting RTP PWM value while not in REALTIME mode. Current mode: {}", mode_);
    } else {
      logger_.info("Setting RTP PWM value to {}", pwm_value);
    }
    write_u8_to_register((uint8_t)Register::RTPIN, pwm_value, ec);
    return !ec; // return true if no error
  }

  /**
   * @brief Set the RTP (Real-Time Playback) mode PWM value.
   * @note This is only valid when the mode is set to Mode::REALTIME.
   * @note This is only valid when the RTP data format is set to signed
   *       (default).
   * @param pwm_value The PWM value to set (-128 to 127).
   * @param ec Error code to set if there is an error.
   * @return true if the PWM value was set successfully, false if there was an
   *         error.
   */
  bool set_rtp_pwm_signed(int8_t pwm_value, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // we log a warning if mode_ is not Mode::REALTIME
    if (mode_ != Mode::REALTIME) {
      logger_.warn("Setting RTP PWM value while not in REALTIME mode. Current mode: {}", mode_);
    } else {
      logger_.info("Setting RTP PWM value to {}", pwm_value);
    }
    write_u8_to_register((uint8_t)Register::RTPIN, static_cast<uint8_t>(pwm_value), ec);
    return !ec; // return true if no error
  }

  /**
   * @brief Set the waveform at slot.
   * @note When calling start() to play the configured waveform slots, the
   *       driver will always start playing from slot 0 and will continue
   *       until it reaches a slot that has been configured with the
   *       Waveform::END.
   * @param slot The slot (0-8) to set.
   * @param w The Waveform to play in slot.
   * @param ec Error code to set if there is an error.
   * @return true if the waveform was set successfully, false if there was an
   *         error.
   */
  bool set_waveform(uint8_t slot, Waveform w, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    logger_.info("Setting waveform {}", (uint8_t)w);
    write_u8_to_register((uint8_t)Register::WAVESEQ1 + slot, (uint8_t)w, ec);
    return !ec; // return true if no error
  }

  /**
   * @brief Select the waveform library to use.
   * @param lib Library to use, 0=Empty, 1-5 are ERM, 6 is LRA
   * @param ec Error code to set if there is an error.
   * @return true if the library was selected successfully, false if there was an
   *         error.
   */
  bool select_library(Library lib, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    logger_.info("Selecting library {}", lib);
    if (motor_type_ == MotorType::LRA && lib != Library::LRA) {
      logger_.warn("LRA motor selected, but library {} is not an LRA library", lib);
    }
    write_u8_to_register((uint8_t)Register::LIBRARY, (uint8_t)lib, ec);
    return !ec; // return true if no error
  }

  /**
   * @brief Calibrate the DRV2605 for an ERM motor.
   * @details This method performs the auto-calibration routine for the DRV2605.
   *          It will put the DRV2605 into auto-calibration
   *          mode, and then start the calibration process. The calibration
   *          data will be stored in the `cal_data_out` parameter.
   * @param cal_conf The calibration settings to use.
   * @param cal_data_out The structure to store the calibration data.
   * @param ec Error code to set if there is an error.
   * @return true if the calibration was successful, false if there was an
   *         error.
   */
  bool calibrate(const ErmCalibrationSettings &cal_conf, CalibratedData &cal_data_out,
                 std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    // if the motor type is not ERM, log a warning
    if (motor_type_ != MotorType::ERM) {
      logger_.warn("Calibrating ERM motor while the motor type is set to {}", motor_type_);
    }

    // save the mode here
    Mode previous_mode = mode_;

    logger_.info("Calibrating ERM motor with settings: {}", cal_conf);
    // 1. put into auto-calibration mode
    if (!set_mode(Mode::AUTOCAL, ec))
      return false;

    // 2. populate the data needed for the auto-calibration engine
    if (!set_calibration_config(&cal_conf, ec)) {
      logger_.error("Failed to set calibration configuration: {}", ec.message());
      return false;
    }

    // 3. set the GO/START bit to start the calibration
    write_u8_to_register((uint8_t)Register::START, 1, ec);
    if (ec) {
      logger_.error("Failed to start calibration: {}", ec.message());
      return false;
    }

    // 4. wait for the GO/START bit to self-clear
    logger_.info("Waiting for calibration to complete...");
    // Note: The DRV2605 will automatically clear the GO/START bit when the
    //       calibration is complete, so we can just wait for it to self-clear.
    while (true) {
      // sleep for a short time to avoid busy-waiting
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      uint8_t start_reg = read_u8_from_register((uint8_t)Register::START, ec);
      if (ec) {
        logger_.error("Failed to read START register: {}", ec.message());
      } else if (start_reg == 0) {
        logger_.info("Calibration completed successfully");
        break; // calibration completed
      }
    }

    // 5. read the value of the DIAG result bit t oensure that it completed
    //    without faults.
    uint8_t diag_result = read_u8_from_register((uint8_t)Register::STATUS, ec);
    if (ec) {
      logger_.error("Failed to read STATUS register: {}", ec.message());
      return false;
    }
    // DIAG_RESULT is bit 3 of the STATUS register
    static constexpr uint8_t DIAG_RESULT_BIT_MASK = 0x08; // bit 3
    if ((diag_result & DIAG_RESULT_BIT_MASK) == 0) {
      logger_.error("Calibration failed with DIAG_RESULT = 0");
      ec = std::make_error_code(std::errc::operation_not_permitted);
      return false;
    }
    logger_.info("Calibration completed successfully with DIAG_RESULT = 1");

    // 6. get the data from the calibration output registers:
    //  - BEMF_GAIN[1:0]
    //  - A_CAL_COMP[7:0]
    //  - A_CAL_BEMF[7:0]
    if (!read_calibration_data(cal_data_out, ec)) {
      logger_.error("Failed to read calibration data: {}", ec.message());
      return false;
    }
    logger_.info("Calibration data read successfully: {}", cal_data_out);

    // now set the mode back
    if (!set_mode(previous_mode, ec)) {
      logger_.error("Failed to set mode back to {}: {}", previous_mode, ec.message());
      return false;
    }

    return true; // return true if no error
  }

  /**
   * @brief Calibrate the DRV2605 for an LRA motor.
   * @details This method performs the auto-calibration routine for the DRV2605.
   *          It will put the DRV2605 into auto-calibration
   *          mode, and then start the calibration process. The calibration
   *          data will be stored in the `cal_data_out` parameter.
   * @param cal_conf The calibration settings to use.
   * @param cal_data_out The structure to store the calibration data.
   * @param ec Error code to set if there is an error.
   * @return true if the calibration was successful, false if there was an
   *         error.
   */
  bool calibrate(const LraCalibrationSettings &cal_conf, CalibratedData &cal_data_out,
                 std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    // if the motor type is not LRA, log a warning
    if (motor_type_ != MotorType::LRA) {
      logger_.warn("Calibrating LRA motor while the motor type is set to {}", motor_type_);
    }

    // save the mode here
    Mode previous_mode = mode_;

    logger_.info("Calibrating LRA motor");
    // 1. put into auto-calibration mode
    if (!set_mode(Mode::AUTOCAL, ec))
      return false;

    // 2. populate the data needed for the auto-calibration engine
    if (!set_calibration_config(static_cast<const BaseCalibrationSettings *>(&cal_conf), ec)) {
      logger_.error("Failed to set calibration configuration: {}", ec.message());
      return false;
    }
    // also set the LRA-specific settings:
    // set SAMPLE_TIME[1:0], which is bits 4-5 of CONTROL2 register
    static constexpr uint8_t SAMPLE_TIME_MASK = 0x30; // bits 4-5
    set_bits_in_register_by_mask((uint8_t)Register::CONTROL2, SAMPLE_TIME_MASK,
                                 cal_conf.sample_time << 4, ec);
    if (ec) {
      logger_.error("Failed to set SAMPLE_TIME: {}", ec.message());
      return false;
    }
    // set BLANKING_TIME[1:0], which is bits 2-3 of CONTROL2 register
    static constexpr uint8_t BLANKING_TIME_MASK = 0x0C; // bits 2-3
    set_bits_in_register_by_mask((uint8_t)Register::CONTROL2, BLANKING_TIME_MASK,
                                 cal_conf.blanking_time << 2, ec);
    if (ec) {
      logger_.error("Failed to set BLANKING_TIME: {}", ec.message());
      return false;
    }
    // set IDISS_TIME[1:0], which is bits 0-1 of CONTROL2 register
    static constexpr uint8_t IDISS_TIME_MASK = 0x03; // bits 0-1
    set_bits_in_register_by_mask((uint8_t)Register::CONTROL2, IDISS_TIME_MASK, cal_conf.idiss_time,
                                 ec);
    if (ec) {
      logger_.error("Failed to set IDISS_TIME: {}", ec.message());
      return false;
    }

    // 3. set the GO/START bit to start the calibration
    write_u8_to_register((uint8_t)Register::START, 1, ec);
    if (ec) {
      logger_.error("Failed to start calibration: {}", ec.message());
      return false;
    }

    // 4. wait for the GO/START bit to self-clear
    logger_.info("Waiting for calibration to complete...");
    // Note: The DRV2605 will automatically clear the GO/START bit when the
    //       calibration is complete, so we can just wait for it to self-clear.
    while (true) {
      // sleep for a short time to avoid busy-waiting
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      uint8_t start_reg = read_u8_from_register((uint8_t)Register::START, ec);
      if (ec) {
        logger_.error("Failed to read START register: {}", ec.message());
      } else if (start_reg == 0) {
        logger_.info("Calibration completed successfully");
        break; // calibration completed
      }
    }

    // 5. read the value of the DIAG result bit t oensure that it completed
    //    without faults.
    uint8_t diag_result = read_u8_from_register((uint8_t)Register::STATUS, ec);
    if (ec) {
      logger_.error("Failed to read STATUS register: {}", ec.message());
      return false;
    }
    // DIAG_RESULT is bit 3 of the STATUS register
    static constexpr uint8_t DIAG_RESULT_BIT_MASK = 0x08; // bit 3
    if ((diag_result & DIAG_RESULT_BIT_MASK) == 0) {
      logger_.error("Calibration failed with DIAG_RESULT = 0");
      ec = std::make_error_code(std::errc::operation_not_permitted);
      return false;
    }
    logger_.info("Calibration completed successfully with DIAG_RESULT = 1");

    // 6. get the data from the calibration output registers:
    //  - BEMF_GAIN[1:0]
    //  - A_CAL_COMP[7:0]
    //  - A_CAL_BEMF[7:0]
    if (!read_calibration_data(cal_data_out, ec)) {
      logger_.error("Failed to read calibration data: {}", ec.message());
      return false;
    }
    logger_.info("Calibration data read successfully: {}", cal_data_out);

    // now set the mode back
    if (!set_mode(previous_mode, ec)) {
      logger_.error("Failed to set mode back to {}: {}", previous_mode, ec.message());
      return false;
    }

    return true; // return true if no error
  }

  /**
   * @brief Read the calibration data from the DRV2605.
   * @details This method reads the calibration data from the DRV2605. It is
   *          used to read the BEMF gain, A_CAL_COMP, and A_CAL_BEMF values.
   * @param cal_data_out The structure to store the calibration data.
   * @param ec Error code to set if there is an error.
   * @return true if the calibration data was read successfully, false if there
   *         was an error.
   */
  bool read_calibration_data(CalibratedData &cal_data_out, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    logger_.info("Reading calibration data");

    // read BEMF_GAIN[1:0], which is bits 0-1 of the FEEDBACK register
    uint8_t bemf_gain = read_u8_from_register((uint8_t)Register::FEEDBACK, ec);
    if (ec)
      return false;

    // read A_CAL_COMP[7:0], which is bits 0-7 of the AUTOCALCOMP register
    uint8_t a_cal_comp = read_u8_from_register((uint8_t)Register::AUTOCALCOMP, ec);
    if (ec)
      return false;

    // read A_CAL_BEMF[7:0], which is bits 0-7 of the AUTOCALEMP register
    uint8_t bemf = read_u8_from_register((uint8_t)Register::AUTOCALEMP, ec);
    if (ec)
      return false;

    cal_data_out.bemf_gain = bemf_gain & 0x03; // mask to get only the first 2 bits
    cal_data_out.a_cal_comp = a_cal_comp;
    cal_data_out.a_cal_bemf = bemf;
    return true; // return true if no error
  }

  /**
   * @brief Set the calibration data for the DRV2605.
   * @details This method sets the calibration data for the DRV2605. It is
   *          used to set the BEMF gain, A_CAL_COMP, and A_CAL_BEMF values.
   * @param cal_data The calibration data to set.
   * @param ec Error code to set if there is an error.
   * @return true if the calibration data was set successfully, false if there
   *         was an error.
   */
  bool set_calibration_data(const CalibratedData &cal_data, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    logger_.info("Setting calibration data: {}", cal_data);

    // set BEMF_GAIN[1:0], which is bits 0-1 of the FEEDBACK register
    static constexpr uint8_t BEMF_GAIN_MASK = 0x03; // bits 0-1
    set_bits_in_register_by_mask((uint8_t)Register::FEEDBACK, BEMF_GAIN_MASK, cal_data.bemf_gain,
                                 ec);
    if (ec)
      return false;

    // set A_CAL_COMP[7:0], which is bits 0-7 of the AUTOCALCOMP register
    write_u8_to_register((uint8_t)Register::AUTOCALCOMP, cal_data.a_cal_comp, ec);
    if (ec)
      return false;

    // set A_CAL_BEMF[7:0], which is bits 0-7 of the AUTOCALEMP register
    write_u8_to_register((uint8_t)Register::AUTOCALEMP, cal_data.a_cal_bemf, ec);
    if (ec)
      return false;

    return true; // return true if no error
  }

protected:
  bool init(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    logger_.info("Initializing motor");

    // disable standby mode
    if (!set_standby(false, ec))
      return false;

    // set default mode to be INTTRIG
    if (!set_mode(Mode::INTTRIG, ec))
      return false;

    // set a simple effect as the default waveform sequence
    if (!set_waveform(0, Waveform::STRONG_CLICK, ec))
      return false;
    if (!set_waveform(1, Waveform::END, ec))
      return false;

    // set the motor type based on the config
    if (!set_motor_type(motor_type_, ec))
      return false;

    return true; // initialization successful
  }

  bool set_motor_type(MotorType motor_type, std::error_code &ec) {
    logger_.info("Setting motor type {}", motor_type);
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    motor_type_ = motor_type;

    static constexpr uint8_t MOTOR_TYPE_BIT_MASK = 0x80; // bit 7
    // set the motor type in the FEEDBACK register, 0 = ERM, 1 = LRA
    if (motor_type == MotorType::ERM) {
      clear_bits_in_register((uint8_t)Register::FEEDBACK, MOTOR_TYPE_BIT_MASK, ec);
    } else if (motor_type == MotorType::LRA) {
      set_bits_in_register((uint8_t)Register::FEEDBACK, MOTOR_TYPE_BIT_MASK, ec);
    } else {
      logger_.error("Invalid motor type: {}", static_cast<uint8_t>(motor_type));
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    return !ec; // return true if no error
  }

  bool set_calibration_config(const BaseCalibrationSettings *cal_conf, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    logger_.info("Setting calibration configuration: {}", *cal_conf);

    // set FB_BRAKE_FACTOR[2:0], which is bits 4-6 of the FEEDBACK register
    static constexpr uint8_t FB_BRAKE_FACTOR_MASK = 0x70; // bits 4-6
    set_bits_in_register_by_mask((uint8_t)Register::FEEDBACK, FB_BRAKE_FACTOR_MASK,
                                 cal_conf->fb_brake_factor << 4, ec);
    if (ec)
      return false;

    // set LOOP_GAIN[1:0], which is bits 2-3 of the FEEDBACK register
    static constexpr uint8_t LOOP_GAIN_MASK = 0x0C; // bits 2-3
    set_bits_in_register_by_mask((uint8_t)Register::FEEDBACK, LOOP_GAIN_MASK,
                                 cal_conf->loop_gain << 2, ec);
    if (ec)
      return false;

    // set RATED_VOLTAGE[7:0], which is bits 0-7 of the RATEDV register
    write_u8_to_register((uint8_t)Register::RATEDV, cal_conf->rated_voltage, ec);
    if (ec)
      return false;

    // set OVERDRIVE_CLAMP[7:0], which is bits 0-7 of the CLAMPV register
    write_u8_to_register((uint8_t)Register::CLAMPV, cal_conf->overdrive_clamp, ec);
    if (ec)
      return false;

    // set AUTO_CAL_TIME[1:0], which is bits 4-5 of CONTROL4 register
    static constexpr uint8_t AUTO_CAL_TIME_MASK = 0x30; // bits 4-5
    set_bits_in_register_by_mask((uint8_t)Register::CONTROL4, AUTO_CAL_TIME_MASK,
                                 cal_conf->auto_cal_time << 4, ec);
    if (ec)
      return false;

    // set DRIVE_TIME[4:0], which is bits 0-4 of CONTROL1 register
    static constexpr uint8_t DRIVE_TIME_MASK = 0x1F; // bits 0-4
    set_bits_in_register_by_mask((uint8_t)Register::CONTROL1, DRIVE_TIME_MASK, cal_conf->drive_time,
                                 ec);

    return !ec; // return true if no error
  }

  enum class Register : uint8_t {
    STATUS = 0x00,      ///< Status
    MODE = 0x01,        ///< Mode
    RTPIN = 0x02,       ///< Real-Time playback input
    LIBRARY = 0x03,     ///< Waveform library selection
    WAVESEQ1 = 0x04,    ///< Waveform sequence 1
    WAVESEQ2 = 0x05,    ///< Waveform sequence 2
    WAVESEQ3 = 0x06,    ///< Waveform sequence 3
    WAVESEQ4 = 0x07,    ///< Waveform sequence 4
    WAVESEQ5 = 0x08,    ///< Waveform sequence 5
    WAVESEQ6 = 0x09,    ///< Waveform sequence 6
    WAVESEQ7 = 0x0A,    ///< Waveform sequence 7
    WAVESEQ8 = 0x0B,    ///< Waveform sequence 8
    START = 0x0C,       ///< Start/Stop playback control. ALso known as the GO register
    OVERDRIVE = 0x0D,   ///< Overdrive time offset
    SUSTAINPOS = 0x0E,  ///< Sustain time offset (positive)
    SUSTAINNEG = 0x0F,  ///< Sustain time offset (negative)
    BREAK = 0x10,       ///< Break time offset
    AUDIOCTRL = 0x11,   ///< Audio to vibe control
    AUDIOMIN = 0x12,    ///< Audio to vibe min input level
    AUDIOMAX = 0x12,    ///< Audio to vibe max input level
    AUDIOOUTMIN = 0x14, ///< Audio to vibe min output drive
    AUDIOOUTMAX = 0x15, ///< Audio to vibe max output drive
    RATEDV = 0x16,      ///< Rated voltage
    CLAMPV = 0x17,      ///< Overdrive clamp
    AUTOCALCOMP = 0x18, ///< Auto calibration compensation result
    AUTOCALEMP = 0x19,  ///< Auto calibration back-EMF result
    FEEDBACK = 0x1A,    ///< Feedback control
    CONTROL1 = 0x1B,    ///< Control1
    CONTROL2 = 0x1C,    ///< Control2
    CONTROL3 = 0x1D,    ///< Control3
    CONTROL4 = 0x1E,    ///< Control4
    VBAT = 0x21,        ///< Vbat voltage monitor
    LRARSON = 0x22,     ///< LRA resonance-period
  };

  std::atomic<MotorType> motor_type_;
  std::atomic<Mode> mode_{Mode::INTTRIG};
};
} // namespace espp

// for easy printing of the enums with the libfmt library:
template <> struct fmt::formatter<espp::Drv2605::Mode> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext> auto format(espp::Drv2605::Mode m, FormatContext &ctx) const {
    switch (m) {
    case espp::Drv2605::Mode::INTTRIG:
      return fmt::format_to(ctx.out(), "INTTRIG");
    case espp::Drv2605::Mode::EXTTRIGEDGE:
      return fmt::format_to(ctx.out(), "EXTTRIGEDGE");
    case espp::Drv2605::Mode::EXTTRIGLVL:
      return fmt::format_to(ctx.out(), "EXTTRIGLVL");
    case espp::Drv2605::Mode::PWMANALOG:
      return fmt::format_to(ctx.out(), "PWMANALOG");
    case espp::Drv2605::Mode::AUDIOVIBE:
      return fmt::format_to(ctx.out(), "AUDIOVIBE");
    case espp::Drv2605::Mode::REALTIME:
      return fmt::format_to(ctx.out(), "REALTIME");
    case espp::Drv2605::Mode::DIAGNOS:
      return fmt::format_to(ctx.out(), "DIAGNOS");
    case espp::Drv2605::Mode::AUTOCAL:
      return fmt::format_to(ctx.out(), "AUTOCAL");
    default:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    }
  }
};

template <> struct fmt::formatter<espp::Drv2605::Waveform> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext>
  auto format(espp::Drv2605::Waveform w, FormatContext &ctx) const {
    switch (w) {
    case espp::Drv2605::Waveform::END:
      return fmt::format_to(ctx.out(), "END");
    case espp::Drv2605::Waveform::STRONG_CLICK:
      return fmt::format_to(ctx.out(), "STRONG_CLICK");
    case espp::Drv2605::Waveform::SHARP_CLICK:
      return fmt::format_to(ctx.out(), "SHARP_CLICK");
    case espp::Drv2605::Waveform::SOFT_BUMP:
      return fmt::format_to(ctx.out(), "SOFT_BUMP");
    case espp::Drv2605::Waveform::DOUBLE_CLICK:
      return fmt::format_to(ctx.out(), "DOUBLE_CLICK");
    case espp::Drv2605::Waveform::TRIPLE_CLICK:
      return fmt::format_to(ctx.out(), "TRIPLE_CLICK");
    case espp::Drv2605::Waveform::SOFT_FUZZ:
      return fmt::format_to(ctx.out(), "SOFT_FUZZ");
    case espp::Drv2605::Waveform::STRONG_BUZZ:
      return fmt::format_to(ctx.out(), "STRONG_BUZZ");
    case espp::Drv2605::Waveform::ALERT_750MS:
      return fmt::format_to(ctx.out(), "ALERT_750MS");
    case espp::Drv2605::Waveform::ALERT_1000MS:
      return fmt::format_to(ctx.out(), "ALERT_1000MS");
    case espp::Drv2605::Waveform::BUZZ1:
      return fmt::format_to(ctx.out(), "BUZZ1");
    case espp::Drv2605::Waveform::BUZZ2:
      return fmt::format_to(ctx.out(), "BUZZ2");
    case espp::Drv2605::Waveform::BUZZ3:
      return fmt::format_to(ctx.out(), "BUZZ3");
    case espp::Drv2605::Waveform::BUZZ4:
      return fmt::format_to(ctx.out(), "BUZZ4");
    case espp::Drv2605::Waveform::BUZZ5:
      return fmt::format_to(ctx.out(), "BUZZ5");
    case espp::Drv2605::Waveform::PULSING_STRONG_1:
      return fmt::format_to(ctx.out(), "PULSING_STRONG_1");
    case espp::Drv2605::Waveform::PULSING_STRONG_2:
      return fmt::format_to(ctx.out(), "PULSING_STRONG_2");
    case espp::Drv2605::Waveform::TRANSITION_CLICK_1:
      return fmt::format_to(ctx.out(), "TRANSITION_CLICK_1");
    case espp::Drv2605::Waveform::TRANSITION_HUM_1:
      return fmt::format_to(ctx.out(), "TRANSITION_HUM_1");
    default:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    }
  }
};

template <> struct fmt::formatter<espp::Drv2605::Library> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext>
  auto format(espp::Drv2605::Library l, FormatContext &ctx) const {
    switch (l) {
    case espp::Drv2605::Library::EMPTY:
      return fmt::format_to(ctx.out(), "EMPTY");
    case espp::Drv2605::Library::ERM_0:
      return fmt::format_to(ctx.out(), "ERM_0");
    case espp::Drv2605::Library::ERM_1:
      return fmt::format_to(ctx.out(), "ERM_1");
    case espp::Drv2605::Library::ERM_2:
      return fmt::format_to(ctx.out(), "ERM_2");
    case espp::Drv2605::Library::ERM_3:
      return fmt::format_to(ctx.out(), "ERM_3");
    case espp::Drv2605::Library::ERM_4:
      return fmt::format_to(ctx.out(), "ERM_4");
    case espp::Drv2605::Library::LRA:
      return fmt::format_to(ctx.out(), "LRA");
    default:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    }
  }
};

template <> struct fmt::formatter<espp::Drv2605::MotorType> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext>
  auto format(espp::Drv2605::MotorType mt, FormatContext &ctx) const {
    switch (mt) {
    case espp::Drv2605::MotorType::ERM:
      return fmt::format_to(ctx.out(), "ERM");
    case espp::Drv2605::MotorType::LRA:
      return fmt::format_to(ctx.out(), "LRA");
    default:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    }
  }
};

template <> struct fmt::formatter<espp::Drv2605::BaseCalibrationSettings> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const espp::Drv2605::BaseCalibrationSettings &cal, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(),
                          "FB_BRAKE_FACTOR: {}, LOOP_GAIN: {}, RATED_VOLTAGE: {}, OVERDRIVE_CLAMP: "
                          "{}, AUTO_CAL_TIME: {}, DRIVE_TIME: {}",
                          cal.fb_brake_factor, cal.loop_gain, cal.rated_voltage,
                          cal.overdrive_clamp, cal.auto_cal_time, cal.drive_time);
  }
};

template <> struct fmt::formatter<espp::Drv2605::LraCalibrationSettings> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const espp::Drv2605::LraCalibrationSettings &cal, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "{}, SAMPLE_TIME: {}, BLANKING_TIME: {}, IDISS_TIME: {}",
                          static_cast<const espp::Drv2605::BaseCalibrationSettings &>(cal),
                          cal.sample_time, cal.blanking_time, cal.idiss_time);
  }
};

template <> struct fmt::formatter<espp::Drv2605::CalibratedData> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const espp::Drv2605::CalibratedData &cal_data, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "BEMF_GAIN: {}, A_CAL_COMP: {}, A_CAL_BEMF: {}",
                          cal_data.bemf_gain, cal_data.a_cal_comp, cal_data.a_cal_bemf);
  }
};

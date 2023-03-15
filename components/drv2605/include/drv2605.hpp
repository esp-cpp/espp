#pragma once

#include <functional>

#include "logger.hpp"

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
  class Drv2605 {
  public:
    static constexpr uint8_t DEFAULT_ADDRESS = (0x5A);

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
    typedef std::function<void(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t data_len)> read_fn;

    /**
     * @brief The mode of the vibration.
     */
    enum class Mode : uint8_t {
      INTTRIG,     ///< Internal Trigger (call star() to start playback)
      EXTTRIGEDGE, ///< External edge trigger (rising edge on IN pin starts playback)
      EXTTRIGLVL,  ///< External level trigger (playback follows state of IN pin)
      PWMANALOG,   ///< PWM/Analog input
      AUDIOVIBE,   ///< Audio-to-vibe mode
      REALTIME,    ///< Real-time playback (RTP)
      DIAGNOS,     ///< Diagnostics
      AUTOCAL,     ///< Auto-calibration
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
     * @brief Configuration structure for the DRV2605
     */
    struct Config {
      uint8_t device_address = DEFAULT_ADDRESS; /**< I2C address of the device. */
      write_fn write;  /**< Function for writing a byte to a register on the Drv2605. */
      read_fn read; /**< Function for reading a byte from a register on the Drv2605. */
      MotorType motor_type{MotorType::ERM}; /**< MotorType that this driver is driving. */
      espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN};  /**< Log verbosity for the Drv2605.  */
    };

    /**
     * @brief Construct and initialize the DRV2605.
     */
    Drv2605(const Config& config)
      : address_(config.device_address),
        write_(config.write),
        read_(config.read),
        logger_({.tag="Drv2605", .level = config.log_level}){
      init(config);
    }

    /**
     * @brief Start playing the configured waveform / sequence.
     */
    void start() {
      logger_.info("Starting");
      write_one_((uint8_t)Register::START, 1);
    }

    /**
     * @brief Stop playing the waveform / sequence.
     */
    void stop() {
      logger_.info("Stopping");
      write_one_((uint8_t)Register::START, 0);
    }

    /**
     * @brief Set the mode of the vibration.
     * @param mode Mode to set to.
     */
    void set_mode(Mode mode) {
      logger_.info("Setting mode {}", (uint8_t)mode);
      write_one_((uint8_t)Register::MODE, (uint8_t)mode);
    }

    /**
     * @brief Set the waveform slot at \p slot to \w.
     * @note When calling start() to play the configured waveform slots, the
     *       driver will always start playing from slot 0 and will continue
     *       until it reaches a slot that has been configured with the
     *       Waveform::END.
     * @param slot The slot (0-8) to set.
     * @param w The Waveform to play in slot \p slot.
     */
    void set_waveform(uint8_t slot, Waveform w) {
      logger_.info("Setting waveform {}", (uint8_t)w);
      write_one_((uint8_t)Register::WAVESEQ1 + slot, (uint8_t)w);
    }

    /**
     * @brief Select the waveform library to use.
     * @param lib Library to use, 0=Empty, 1-5 are ERM, 6 is LRA
     */
    void select_library(uint8_t lib) {
      logger_.info("Selecting library {}", lib);
      write_one_((uint8_t)Register::LIBRARY, lib);
    }

  protected:
    void init (const Config& config) {
      logger_.info("Initializing motor");
      write_one_((uint8_t)Register::MODE, 0);  // out of standby
      write_one_((uint8_t)Register::RTPIN, 0); // no real-time playback
      set_waveform(0, Waveform::STRONG_CLICK);  // Strong Click
      set_waveform(1, Waveform::END);           // end sequence
      write_one_((uint8_t)Register::OVERDRIVE, 0);  // no overdrive
      write_one_((uint8_t)Register::SUSTAINPOS, 0);
      write_one_((uint8_t)Register::SUSTAINNEG, 0);
      write_one_((uint8_t)Register::BREAK, 0);
      write_one_((uint8_t)Register::AUDIOMAX, 0x64);
      // set the motor type based on the config
      set_motor_type(config.motor_type);
      // turn on ERM OPEN LOOP
      auto current_control3 = read_one_((uint8_t)Register::CONTROL3);
      write_one_((uint8_t)Register::CONTROL3, current_control3 | 0x20);
    }

    void set_motor_type(MotorType motor_type) {
      logger_.info("Setting motor type {}", motor_type == MotorType::ERM ? "ERM" : "LRA");
      auto current_feedback = read_one_((uint8_t)Register::FEEDBACK);
      uint8_t motor_config = (motor_type == MotorType::ERM) ? 0x7F : 0x80;
      write_one_((uint8_t)Register::FEEDBACK, current_feedback | motor_config);
    }

    enum class Register : uint8_t {
      STATUS = 0x00,   ///< Status
      MODE = 0x01,     ///< Mode
      RTPIN = 0x02,    ///< Real-Time playback input
      LIBRARY = 0x03,  ///< Waveform library selection
      WAVESEQ1 = 0x04, ///< Waveform sequence 1
      WAVESEQ2 = 0x05, ///< Waveform sequence 2
      WAVESEQ3 = 0x06, ///< Waveform sequence 3
      WAVESEQ4 = 0x07, ///< Waveform sequence 4
      WAVESEQ5 = 0x08, ///< Waveform sequence 5
      WAVESEQ6 = 0x09, ///< Waveform sequence 6
      WAVESEQ7 = 0x0A, ///< Waveform sequence 7
      WAVESEQ8 = 0x0B, ///< Waveform sequence 8
      START = 0x0C,    ///< Start/Stop playback control
      OVERDRIVE = 0x0D,///< Overdrive time offset
      SUSTAINPOS= 0x0E,///< Sustain time offset (positive)
      SUSTAINNEG= 0x0F,///< Sustain time offset (negative)
      BREAK = 0x10,    ///< Break time offset
      AUDIOCTRL = 0x11,///< Audio to vibe control
      AUDIOMIN = 0x12, ///< Audio to vibe min input level
      AUDIOMAX = 0x12, ///< Audio to vibe max input level
      AUDIOOUTMIN=0x14,///< Audio to vibe min output drive
      AUDIOOUTMAX=0x15,///< Audio to vibe max output drive
      RATEDV = 0x16,   ///< Rated voltage
      CLAMPV = 0x17,   ///< Overdrive clamp
      AUTOCALCOMP=0x18,///< Auto calibration compensation result
      AUTOCALEMP =0x19,///< Auto calibration back-EMF result
      FEEDBACK = 0x1A, ///< Feedback control
      CONTROL1 = 0x1B, ///< Control1
      CONTROL2 = 0x1C, ///< Control2
      CONTROL3 = 0x1D, ///< Control3
      CONTROL4 = 0x1E, ///< Control4
      VBAT = 0x21,     ///< Vbat voltage monitor
      LRARSON = 0x22,  ///< LRA resonance-period
    };

    uint8_t read_one_(uint8_t reg_addr) {
      uint8_t data;
      read_(address_, reg_addr, &data, 1);
      return data;
    }

    void write_one_(uint8_t reg_addr, uint8_t data) {
      write_many_(reg_addr, &data, 1);
    }

    void write_many_(uint8_t reg_addr, uint8_t *write_data, size_t write_data_len) {
      uint8_t total_len = 1+write_data_len;
      uint8_t data[total_len];
      data[0] = reg_addr;
      memcpy(&data[1], write_data, write_data_len);
      write_(address_, data, total_len);
    }

    uint8_t address_;
    write_fn write_;
    read_fn read_;
    espp::Logger logger_;
  };
}

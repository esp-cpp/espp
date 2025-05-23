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
  static constexpr uint8_t DEFAULT_ADDRESS = (0x5A);

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
   */
  void initialize(std::error_code &ec) { init(ec); }

  /**
   * @brief Start playing the configured waveform / sequence.
   * @param ec Error code to set if there is an error.
   */
  void start(std::error_code &ec) {
    logger_.info("Starting");
    write_u8_to_register((uint8_t)Register::START, 1, ec);
  }

  /**
   * @brief Stop playing the waveform / sequence.
   * @param ec Error code to set if there is an error.
   */
  void stop(std::error_code &ec) {
    logger_.info("Stopping");
    write_u8_to_register((uint8_t)Register::START, 0, ec);
  }

  /**
   * @brief Set the mode of the vibration.
   * @param mode Mode to set to.
   * @param ec Error code to set if there is an error.
   */
  void set_mode(Mode mode, std::error_code &ec) {
    logger_.info("Setting mode {}", (uint8_t)mode);
    write_u8_to_register((uint8_t)Register::MODE, (uint8_t)mode, ec);
  }

  /**
   * @brief Set the waveform slot at \p slot to \w.
   * @note When calling start() to play the configured waveform slots, the
   *       driver will always start playing from slot 0 and will continue
   *       until it reaches a slot that has been configured with the
   *       Waveform::END.
   * @param slot The slot (0-8) to set.
   * @param w The Waveform to play in slot \p slot.
   * @param ec Error code to set if there is an error.
   */
  void set_waveform(uint8_t slot, Waveform w, std::error_code &ec) {
    logger_.info("Setting waveform {}", (uint8_t)w);
    write_u8_to_register((uint8_t)Register::WAVESEQ1 + slot, (uint8_t)w, ec);
  }

  /**
   * @brief Select the waveform library to use.
   * @param lib Library to use, 0=Empty, 1-5 are ERM, 6 is LRA
   * @param ec Error code to set if there is an error.
   */
  void select_library(Library lib, std::error_code &ec) {
    logger_.info("Selecting library {}", lib);
    if (motor_type_ == MotorType::LRA && lib != Library::LRA) {
      logger_.warn("LRA motor selected, but library {} is not an LRA library", lib);
    }
    write_u8_to_register((uint8_t)Register::LIBRARY, (uint8_t)lib, ec);
  }

protected:
  void init(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    logger_.info("Initializing motor");
    write_u8_to_register((uint8_t)Register::MODE, 0, ec); // out of standby
    if (ec)
      return;
    write_u8_to_register((uint8_t)Register::RTPIN, 0, ec); // no real-time playback
    if (ec)
      return;
    set_waveform(0, Waveform::STRONG_CLICK, ec); // Strong Click
    if (ec)
      return;
    set_waveform(1, Waveform::END, ec); // end sequence
    if (ec)
      return;
    write_u8_to_register((uint8_t)Register::OVERDRIVE, 0, ec); // no overdrive
    if (ec)
      return;
    write_u8_to_register((uint8_t)Register::SUSTAINPOS, 0, ec);
    if (ec)
      return;
    write_u8_to_register((uint8_t)Register::SUSTAINNEG, 0, ec);
    if (ec)
      return;
    write_u8_to_register((uint8_t)Register::BREAK, 0, ec);
    if (ec)
      return;
    write_u8_to_register((uint8_t)Register::AUDIOMAX, 0x64, ec);
    if (ec)
      return;
    // set the motor type based on the config
    set_motor_type(motor_type_, ec);
    if (ec)
      return;
    // turn on ERM OPEN LOOP
    auto current_control3 = read_u8_from_register((uint8_t)Register::CONTROL3, ec);
    if (ec)
      return;
    write_u8_to_register((uint8_t)Register::CONTROL3, current_control3 | 0x20, ec);
  }

  void set_motor_type(MotorType motor_type, std::error_code &ec) {
    logger_.info("Setting motor type {}", motor_type == MotorType::ERM ? "ERM" : "LRA");
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    motor_type_ = motor_type;
    auto current_feedback = read_u8_from_register((uint8_t)Register::FEEDBACK, ec);
    if (ec)
      return;
    uint8_t motor_config = (motor_type_ == MotorType::ERM) ? 0x7F : 0x80;
    write_u8_to_register((uint8_t)Register::FEEDBACK, current_feedback | motor_config, ec);
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
    START = 0x0C,       ///< Start/Stop playback control
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

  MotorType motor_type_;
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

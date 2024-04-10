#pragma once

#include <atomic>
#include <cmath>
#include <functional>

#include "base_peripheral.hpp"
#include "task.hpp"

namespace espp {
/// @brief Enum class for the interface type of the MT6701.
enum class Mt6701Interface : uint8_t {
  I2C = 0, ///< Inter-Integrated Circuit (I2C)
  SSI = 1, ///< Synchronous Serial Interface (SSI), which can be SPI or SSI
};
/**
 * @brief Class for position and velocity measurement using a MT6701 magnetic
 *        encoder. This class starts its own measurement task at the specified
 *        frequency which reads the current angle, updates the accumulator, and
 *        filters / updates the velocity measurement. The Mt6701 supports I2C,
 *        SSI, ABZ, UVW, Analog/PWM, and Push-Button interfaces.
 *
 * @note This implementation currently only supports I2C and SSI interfaces.
 *
 * @note There is an implicit assumption in this class regarding the maximum
 *       velocity it can measure (above which there will be aliasing). The
 *       fastest velocity it can measure will be (0.5f / update_period * 60.0f)
 *       RPM which is half a rotation in one update period.
 *
 * @note The assumption above also affects the reliability of the accumulator,
 *       since it is based on accumulating position differences every update
 *       period.
 *
 * \section mt6701_ex1 Mt6701 I2C Example
 * \snippet mt6701_example.cpp mt6701 i2c example
 * \section mt6701_ex2 Mt6701 SSI / SPI Example
 * \snippet mt6701_example.cpp mt6701 ssi example
 */
template <Mt6701Interface Interface = Mt6701Interface::I2C>
class Mt6701 : public BasePeripheral<uint8_t, Interface == Mt6701Interface::I2C> {
  // Since the BasePeripheral is a dependent base class (e.g. its template
  // parameters depend on our template parameters), we need to use the `using`
  // keyword to bring in the functions / members we want to use, otherwise we
  // have to either use `this->` or explicitly scope each call, which clutters
  // the code / is annoying. This is needed because of the two phases of name
  // lookups for templates.
  using BasePeripheral<uint8_t, Interface == Mt6701Interface::I2C>::set_address;
  using BasePeripheral<uint8_t, Interface == Mt6701Interface::I2C>::set_write;
  using BasePeripheral<uint8_t, Interface == Mt6701Interface::I2C>::set_read;
  using BasePeripheral<uint8_t, Interface == Mt6701Interface::I2C>::read_u8_from_register;
  using BasePeripheral<uint8_t, Interface == Mt6701Interface::I2C>::read;
  using BasePeripheral<uint8_t, Interface == Mt6701Interface::I2C>::logger_;

public:
  static constexpr uint8_t DEFAULT_ADDRESS =
      (0b0000110); ///< I2C address of the MT6701. It can be programmed to be 0b1000110 as well.
                   ///< Only used if Interface == Mt6701Interface::I2C.

  /**
   * @brief Filter the input raw velocity and return it.
   * @param raw Most recent raw velocity measured.
   * @return Filtered velocity.
   */
  typedef std::function<float(float raw)> velocity_filter_fn;

  /**
   * @brief Enum class for the magnetic field strength of the MT6701.
   */
  enum class MagneticFieldStrength : uint8_t {
    NORMAL = 0,     ///< The magnetic field is normal.
    TOO_STRONG = 1, ///< The magnetic field is too strong to measure the angle.
    TOO_WEAK = 2,   ///< The magnetic field is too weak to measure the angle.
  };

  /**
   * @brief Enum class for the tracking status of the MT6701.
   */
  enum class TrackingStatus : uint8_t {
    NORMAL = 0, ///< Normal tracking status.
    LOST = 1,   ///< Tracking has been lost.
  };

  static constexpr int COUNTS_PER_REVOLUTION =
      16384; ///< Int number of counts per revolution for the magnetic encoder.
  static constexpr float COUNTS_PER_REVOLUTION_F =
      16384.0f; ///< Float number of counts per revolution for the magnetic encoder.
  static constexpr float COUNTS_TO_RADIANS =
      2.0f * M_PI /
      COUNTS_PER_REVOLUTION_F; ///< Conversion factor to convert from count value to radians.
  static constexpr float COUNTS_TO_DEGREES =
      360.0f /
      COUNTS_PER_REVOLUTION_F; ///< Conversion factor to convert from count value to degrees.
  static constexpr float SECONDS_PER_MINUTE =
      60.0f; ///< Conversion factor to convert from seconds to minutes.

  /**
   * @brief Configuration information for the Mt6701.
   */
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS; ///< I2C address of the device. Only used if
                                              ///< Interface == Mt6701Interface::I2C.
    BasePeripheral<uint8_t, Interface == Mt6701Interface::I2C>::write_fn write{
        nullptr}; ///< Function to write to the device.
    BasePeripheral<uint8_t, Interface == Mt6701Interface::I2C>::read_fn read{
        nullptr};                                ///< Function to read data from the device.
    velocity_filter_fn velocity_filter{nullptr}; ///< Function to filter the veolcity. @note Will be
                                                 ///< called once every update_period seconds.
    std::chrono::duration<float> update_period{
        .01f}; ///< Update period (1/sample rate) in seconds. This determines the periodicity of the
               ///< task which will read the position, update the accumulator, and update/filter
               ///< velocity.
    bool auto_init{true}; ///< Whether to automatically initialize the accumulator to the current
                          ///< position on startup.
    Logger::Verbosity log_level{Logger::Verbosity::WARN};
  };

  /**
   * @brief Construct the Mt6701 and start the update task.
   */
  explicit Mt6701(const Config &config)
      : BasePeripheral<uint8_t, Interface == Mt6701Interface::I2C>({}, "Mt6701", config.log_level)
      , velocity_filter_(config.velocity_filter)
      , update_period_(config.update_period) {
    if constexpr (Interface == Mt6701Interface::I2C) {
      set_address(config.device_address);
      set_write(config.write);
      set_read(config.read);
    } else {
      set_read(config.read);
    }
    if (config.auto_init) {
      std::error_code ec;
      initialize(ec);
    }
  }

  /**
   * @brief Initialize the accumulator to the current position and start the
   *        update task.
   * @param ec Error code to set if there is an error.
   */
  void initialize(std::error_code &ec) {
    logger_.info("Initializing. Fastest measurable velocity will be {:.3f} RPM",
                 // half a rotation in one update period is the fastest we can
                 // measure
                 0.5f / update_period_.count() * SECONDS_PER_MINUTE);
    init(ec);
    if (ec) {
      logger_.error("Error initializing: {}", ec.message());
    }
  }

  /**
   * @brief Return whether the sensor has found absolute 0 yet.
   * @note The MT6701 (using I2C/SPI) does not need to search for absolute 0
   *       and will always know it on startup. Therefore this function always
   *       returns false.
   * @return True because the magnetic sensor (using I2C/SPI) does not need to
   *         sarch for 0.
   */
  bool needs_zero_search() const { return false; }

  /**
   * @brief Get the most recently updated raw count value from the encoder.
   * @note This value always represents the angle of the encoder modulo one
   *       rotation, meaning it only represents the range 0 to 360 degrees. It
   *       is not recommended to use this function, but is provided for edge use
   *       cases.
   * @return Raw count value in the range [0, 16384] (0 to 360 degrees).
   */
  int get_count() const { return count_.load(); }

  /**
   * @brief Return the accumulated count that the encoder has generated since it
   *        was initialized.
   * @note This value is a raw counter value that can be +/-, meaning
   *       COUNTS_PER_REVOLUTION can be used to convert it to revolutions.
   * @return Raw accumulator value.
   */
  int get_accumulator() const { return accumulator_.load(); }

  /**
   * @brief Return the mechanical / shaft angle of the encoder, in radians,
   *        within the range [0, 2pi].
   * @return Angle in radians of the encoder within the range [0, 2pi].
   */
  float get_mechanical_radians() const { return (float)get_count() * COUNTS_TO_RADIANS; }

  /**
   * @brief Return the mechanical / shaft angle of the encoder, in degrees,
   *        within the range [0, 360].
   * @return Angle in degrees of the encoder within the range [0, 360].
   */
  float get_mechanical_degrees() const { return (float)get_count() * COUNTS_TO_DEGREES; }

  /**
   * @brief Return the accumulated position of the encoder, in radians.
   * @note This can be any value, it is not restricted to [0, 2pi].
   * @return Position in radians of the encoder.
   */
  float get_radians() const { return (float)get_accumulator() * COUNTS_TO_RADIANS; }

  /**
   * @brief Return the accumulated position of the encoder, in degrees.
   * @note This can be any value, it is not restricted to [0, 360].
   * @return Position in degrees of the encoder.
   */
  float get_degrees() const { return (float)get_accumulator() * COUNTS_TO_DEGREES; }

  /**
   * @brief Return the filtered velocity of the encoder, in RPM.
   * @return Filtered velocity (revolutions / minute, RPM).
   */
  float get_rpm() const { return velocity_rpm_.load(); }

  /**
   * @brief Return the magnetic field strength of the encoder.
   * @return Magnetic field strength of the encoder.
   * @note This function is only available when using SSI communications.
   */
  MagneticFieldStrength get_magnetic_field_strength() const
      requires(Interface == Mt6701Interface::SSI) {
    return magnetic_field_strength_.load();
  }

  /**
   * @brief Return the tracking status of the encoder.
   * @return Tracking status of the encoder.
   * @note This function is only available when using SSI communications.
   */
  TrackingStatus get_tracking_status() const requires(Interface == Mt6701Interface::SSI) {
    return tracking_status_.load();
  }

  /**
   * @brief Return whether the push button is currently pressed.
   * @return True if the push button is pressed, false otherwise.
   * @note This function is only available when using SSI communications.
   */
  bool get_push_button() const requires(Interface == Mt6701Interface::SSI) {
    return push_button_.load();
  }

protected:
  struct MagneticFieldStatus {
    uint8_t magnetic_field_strength : 2;
    uint8_t push_button : 1;
    uint8_t tracking_status : 1;
  } __attribute__((packed));

  void read(std::error_code &ec) requires(Interface == Mt6701Interface::I2C) {
    logger_.info("read");
    // read the angle count registers
    uint8_t angle_h = read_u8_from_register((uint8_t)Registers::ANGLE_H, ec);
    if (ec) {
      return;
    }
    uint8_t angle_l = read_u8_from_register((uint8_t)Registers::ANGLE_L, ec) >> 2;
    if (ec) {
      return;
    }
    count_.store((angle_h << 6) | angle_l);
  }

  void read(std::error_code &ec) requires(Interface == Mt6701Interface::SSI) {
    logger_.info("read");
    // read the angle count as 24 bits (3 bytes) from the serial stream
    uint8_t buffer[3] = {0};
    read(&buffer[0], 3, ec);
    if (ec) {
      return;
    }
    // the first 14 bits is the angle data, followed by 4 bit status, and 6 bit
    // crc
    uint16_t angle_h = buffer[0];
    uint8_t angle_l = buffer[1] >> 2;
    // status is the lower 2 bits of the second byte and the upper 2 bits of
    // the third byte
    uint8_t status = ((buffer[1] & 0b11) << 2) | (buffer[2] >> 6);
    // crc is the lower 6 bits of the third byte
    uint8_t crc = buffer[2] & 0b111111;
    logger_.debug("Angle: {}, Status: {}, CRC: {}", (angle_h << 8) | angle_l, status, crc);
    // update the count
    count_.store((angle_h << 6) | angle_l);
    // update the magnetic field strength, tracking status, and push button.
    // strength is the lower two bits [0:1], push button is the third bit [2],
    // and tracking status is the fourth bit [3]
    magnetic_field_strength_ = (MagneticFieldStrength)(status & 0b11);
    push_button_ = (bool)((status >> 2) & 0b1);
    tracking_status_ = (TrackingStatus)((status >> 3) & 0b1);
  }

  void update(std::error_code &ec) {
    logger_.info("update");
    // measure update timing
    static auto prev_time = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(now - prev_time).count();
    prev_time = now;
    float seconds = elapsed ? elapsed : update_period_.count();
    // store the previous count
    int prev_count = count_;
    // read the latest data from the encoder and update the state
    read(ec);
    if (ec) {
      return;
    }
    // compute diff
    int diff = count_ - prev_count;
    // check for zero crossing
    if (diff > COUNTS_PER_REVOLUTION / 2) {
      // we crossed zero going clockwise (1 -> 359)
      diff -= COUNTS_PER_REVOLUTION;
    } else if (diff < -COUNTS_PER_REVOLUTION / 2) {
      // we crossed zero going counter-clockwise (359 -> 1)
      diff += COUNTS_PER_REVOLUTION;
    }
    // update accumulator
    accumulator_ += diff;
    logger_.debug("CDA: {}, {}, {}", count_, diff, accumulator_);
    // update velocity (filtering it)
    float raw_velocity = (float)(diff) / COUNTS_PER_REVOLUTION_F / seconds * SECONDS_PER_MINUTE;
    velocity_rpm_ = velocity_filter_ ? velocity_filter_(raw_velocity) : raw_velocity;
    static float max_velocity = 0.5f / update_period_.count() * SECONDS_PER_MINUTE;
    if (raw_velocity >= max_velocity) {
      logger_.warn("Velocity nearing measurement limit ({:.3f} RPM), consider decreasing your "
                   "update period!",
                   max_velocity);
    }
  }

  bool update_task(std::mutex &m, std::condition_variable &cv) {
    auto start = std::chrono::high_resolution_clock::now();
    std::error_code ec;
    update(ec);
    if (ec) {
      logger_.error("Error updating: {}", ec.message());
    }
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait_until(lk, start + update_period_);
    }
    // don't want to stop the task
    return false;
  }

  void init(std::error_code &ec) {
    // initialize the accumulator to have the current angle
    read(ec);
    if (ec) {
      return;
    }
    accumulator_ = count_.load();
    // start the task
    using namespace std::placeholders;
    task_ = Task::make_unique(
        {.name = "Mt6701", .callback = std::bind(&Mt6701::update_task, this, _1, _2)});
    task_->start();
  }

  /**
   * @brief Register map for the MT6701.
   *
   * @note The MT6701 contains a push-button output (pin 5) with configuration
   *       (mentioned on page 25) via PUSH_THRD register, PUSH_DIFF_DLY
   *       register, and PUSH_TIME_OUT register. However, the register addresses
   *       for these configurations (and their bitfields) are not provided in
   *       the datasheet and must be provided by the manufacturer.
   *
   * @note The push button can only be read from the MT6701 when using SSI
   *       communications, and is returned as part of the magnetic field status
   *       truth table (page 26).
   */
  enum class Registers : uint8_t {
    ANGLE_H = 0x03,     ///< Angle[13:6]
    ANGLE_L = 0x04,     ///< Angle[5:0] (bit 2-7)
    MUX_1 = 0x25,       ///< UVW MUX (bit 7)
    MUX_2 = 0x29,       ///< ABZ MUX (bit 6), DIR (bit 1)
    RES_1 = 0x30,       ///< UVW_RES (bit 4-7), ABZ_RES[9:8] (bit 0-1)
    RES_2 = 0x31,       ///< ABZ_RES[7:0]
    CONFIG_1 = 0x32,    ///< HYST[2] (bit 7), Z_PULSE_WIDTH (bit 4-6), ZERO[11:8] (bit 0-3)
    CONFIG_2 = 0x33,    ///< ZERO[7:0]
    CONFIG_3 = 0x34,    ///< HYST[1:0] (bit 6-7)
    CONFIG_4 = 0x38,    ///< PWM_FREQ (bit 7), PWM_POL (bit 6), OUT_MODE (bit 5)
    A_SS_HIGH = 0x3E,   ///< A_STOP[11:8] (bit 4-7), A_START[11:8] (bit 0-3)
    A_START_LOW = 0x3F, ///< A_START[7:0]
    A_STOP_LOW = 0x40,  ///< A_STOP[7:0]
  };

  velocity_filter_fn velocity_filter_{nullptr};
  std::chrono::duration<float> update_period_;
  std::atomic<int> count_{0};
  std::atomic<int> accumulator_{0};
  std::atomic<float> velocity_rpm_{0};
  std::atomic<MagneticFieldStrength> magnetic_field_strength_{MagneticFieldStrength::NORMAL};
  std::atomic<TrackingStatus> tracking_status_{TrackingStatus::NORMAL};
  std::atomic<bool> push_button_{false};
  std::unique_ptr<Task> task_;
};
} // namespace espp

// for easy printing of MagneticFieldStatus using libfmt
template <> struct fmt::formatter<espp::Mt6701<espp::Mt6701Interface::SSI>::MagneticFieldStrength> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::Mt6701<espp::Mt6701Interface::SSI>::MagneticFieldStrength const &mfs,
              FormatContext &ctx) {
    return fmt::format_to(
        ctx.out(), "{}",
        mfs == espp::Mt6701<espp::Mt6701Interface::SSI>::MagneticFieldStrength::NORMAL ? "NORMAL"
        : mfs == espp::Mt6701<espp::Mt6701Interface::SSI>::MagneticFieldStrength::TOO_STRONG
            ? "TOO_STRONG"
            : "TOO_WEAK");
  }
};
template <> struct fmt::formatter<espp::Mt6701<espp::Mt6701Interface::SSI>::TrackingStatus> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::Mt6701<espp::Mt6701Interface::SSI>::TrackingStatus const &ts,
              FormatContext &ctx) {
    return fmt::format_to(
        ctx.out(), "{}",
        ts == espp::Mt6701<espp::Mt6701Interface::SSI>::TrackingStatus::NORMAL ? "NORMAL" : "LOST");
  }
};
template <> struct fmt::formatter<espp::Mt6701<espp::Mt6701Interface::I2C>::MagneticFieldStrength> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::Mt6701<espp::Mt6701Interface::I2C>::MagneticFieldStrength const &mfs,
              FormatContext &ctx) {
    return fmt::format_to(
        ctx.out(), "{}",
        mfs == espp::Mt6701<espp::Mt6701Interface::I2C>::MagneticFieldStrength::NORMAL ? "NORMAL"
        : mfs == espp::Mt6701<espp::Mt6701Interface::I2C>::MagneticFieldStrength::TOO_STRONG
            ? "TOO_STRONG"
            : "TOO_WEAK");
  }
};
template <> struct fmt::formatter<espp::Mt6701<espp::Mt6701Interface::I2C>::TrackingStatus> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::Mt6701<espp::Mt6701Interface::I2C>::TrackingStatus const &ts,
              FormatContext &ctx) {
    return fmt::format_to(
        ctx.out(), "{}",
        ts == espp::Mt6701<espp::Mt6701Interface::I2C>::TrackingStatus::NORMAL ? "NORMAL" : "LOST");
  }
};

#pragma once

#include <atomic>
#include <cmath>
#include <functional>

#include "base_peripheral.hpp"
#include "task.hpp"

namespace espp {
/**
 * @brief Class for position and velocity measurement using a AS5600 magnetic
 *        encoder. This class starts its own measurement task at the specified
 *        frequency which reads the current angle, updates the accumulator,
 *        and filters / updates the velocity measurement. The datasheet for
 *        the AS5600 can be found here:
 *        https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf/649ee61c-8f9a-20df-9e10-43173a3eb323
 *
 * @note There is an implicit assumption in this class regarding the maximum
 *       velocity it can measure (above which there will be aliasing). The
 *       fastest velocity it can measure will be (0.5f * update_period * 60.0f)
 *       which is half a rotation in one update period.
 *
 * @note The assumption above also affects the reliability of the accumulator,
 *       since it is based on accumulating position differences every update
 *       period.
 *
 * \section as5600_ex1 As5600 Example
 * \snippet as5600_example.cpp as5600 example
 */
class As5600 : public BasePeripheral<> {
public:
  static constexpr uint8_t DEFAULT_ADDRESS = (0b0110110); ///< I2C address of the AS5600

  /**
   * @brief Filter the input raw velocity and return it.
   * @param raw Most recent raw velocity measured.
   * @return Filtered velocity.
   */
  typedef std::function<float(float raw)> velocity_filter_fn;

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
   * @brief Configuration information for the As5600.
   */
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS; ///< I2C address for this device.
    BasePeripheral::write_then_read_fn
        write_then_read;                         ///< Function to write then read from the device.
    velocity_filter_fn velocity_filter{nullptr}; ///< Function to filter the veolcity. @note Will be
                                                 ///< called once every update_period seconds.
    std::chrono::duration<float> update_period{
        .01f}; ///< Update period (1/sample rate) in seconds. This determines the periodicity of the
               ///< task which will read the position, update the accumulator, and update/filter
               ///< velocity.
    bool auto_init{true}; ///< Whether to automatically initialize the accumulator to the current
                          ///< position.
    Logger::Verbosity log_level{Logger::Verbosity::WARN};
  };

  /**
   * @brief Construct the As5600 and start the update task.
   */
  explicit As5600(const Config &config)
      : BasePeripheral(
            {.address = config.device_address, .write_then_read = config.write_then_read}, "As5600",
            config.log_level)
      , velocity_filter_(config.velocity_filter)
      , update_period_(config.update_period) {
    logger_.info("Initializing. Fastest measurable velocity will be {:.3f} RPM",
                 // half a rotation in one update period is the fastest we can
                 // measure
                 0.5f / update_period_.count() * SECONDS_PER_MINUTE);
    if (config.auto_init) {
      std::error_code ec;
      initialize(ec);
    }
  }

  /**
   * @brief Initialize the sensor.
   * @param ec Error code to set if there is an error.
   */
  void initialize(std::error_code &ec) { init(ec); }

  /**
   * @brief Return whether the sensor has found absolute 0 yet.
   * @note The AS5600 (using I2C/SPI) does not need to search for absolute 0
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
   * @note This can be any value, it is not restricted to [-2pi, 2pi].
   * @return Position in radians of the encoder.
   */
  float get_radians() const { return (float)get_accumulator() * COUNTS_TO_RADIANS; }

  /**
   * @brief Return the accumulated position of the encoder, in degrees.
   * @note This can be any value, it is not restricted to [-360, 360].
   * @return Position in degrees of the encoder.
   */
  float get_degrees() const { return (float)get_accumulator() * COUNTS_TO_DEGREES; }

  /**
   * @brief Return the filtered velocity of the encoder, in RPM.
   * @return Filtered velocity (revolutions / minute, RPM).
   */
  float get_rpm() const { return velocity_rpm_.load(); }

protected:
  int read_count(std::error_code &ec) {
    logger_.info("read_count");
    // read the angle count registers
    uint8_t angle_h = read_u8_from_register((uint8_t)Registers::ANGLE_H, ec);
    if (ec) {
      return 0;
    }
    uint8_t angle_l = read_u8_from_register((uint8_t)Registers::ANGLE_L, ec) >> 2;
    if (ec) {
      return 0;
    }
    return (int)((angle_h << 6) | angle_l);
  }

  void update(std::error_code &ec) {
    logger_.info("update");
    // update raw count
    auto count = read_count(ec);
    if (ec) {
      return;
    }
    count_.store(count);
    static int prev_count = count_;
    // compute diff
    int diff = count_ - prev_count;
    // update prev_count
    prev_count = count_;
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
    static auto prev_time = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(now - prev_time).count();
    prev_time = now;
    float seconds = elapsed ? elapsed : update_period_.count();
    float raw_velocity = (float)(diff) / COUNTS_PER_REVOLUTION_F / seconds * SECONDS_PER_MINUTE;
    velocity_rpm_ = velocity_filter_ ? velocity_filter_(raw_velocity) : raw_velocity;
    static float max_velocity = 0.5f / update_period_.count() * SECONDS_PER_MINUTE;
    if (raw_velocity >= max_velocity) {
      logger_.warn("Velocity nearing measurement limit ({:.3f} RPM), consider decreasing your "
                   "update period!",
                   max_velocity);
    }
  }

  bool update_task(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
    auto start = std::chrono::high_resolution_clock::now();
    std::error_code ec;
    update(ec);
    if (ec) {
      logger_.error("Error updating: {}", ec.message());
    }
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait_until(lk, start + update_period_, [&task_notified] { return task_notified; });
      task_notified = false;
    }
    // don't want the task to stop
    return false;
  }

  void init(std::error_code &ec) {
    // initialize the accumulator to have the current angle
    auto count = read_count(ec);
    if (ec) {
      return;
    }
    accumulator_ = count;
    // start the task
    using namespace std::placeholders;
    task_ = Task::make_unique(
        {.name = "As5600", .callback = std::bind(&As5600::update_task, this, _1, _2, _3)});
    task_->start();
  }

  /**
   * @brief Register map for the AS5600.
   *
   * @note The AS5600 contains a push-button output (pin 5) with configuration
   *       (mentioned on page 25) via PUSH_THRD register, PUSH_DIFF_DLY
   *       register, and PUSH_TIME_OUT register. However, the register addresses
   *       for these configurations (and their bitfields) are not provided in
   *       the datasheet and must be provided by the manufacturer.
   *
   * @note The push button can only be read from the AS5600 when using SSI
   *       communications, and is returned as part of the magnetic field status
   *       truth table (page 24).
   */
  enum class Registers : uint8_t {
    // configuration registers:
    ZMCO = 0x00,   ///< ZMCO[1:0] (bits 1-0)
    ZPOS_H = 0x01, ///< ZPOS[11:8] (bits 3-0)
    ZPOS_L = 0x02, ///< ZPOS[7:0]
    MPOS_H = 0x03, ///< MPOS[11:8] (bits 3-0)
    MPOS_L = 0x04, ///< MPOS[7:0]
    MANG_H = 0x05, ///< MANG[11:8] (bits 3-0)
    MANG_L = 0x06, ///< MANG[7:0]
    CONF_0 = 0x07, ///< Watchdog (bit 5), Fast Filter Threshold[2:0] (bits 4-2), Slow Filter[1:0]
                   ///< (bits 1-0)
    CONF_1 = 0x08, ///< PWM Freq[1:0] (bits 7-6), Output Stage[1:0] (bits 5-4), HYST[1:0] (bits
                   ///< 3-2), Power Mode[1:0] (bits 1-0)
    // output registers:
    RANG_H = 0x0C,  ///< Raw angle [11:8] (bits 3-0)
    RANG_L = 0x0D,  ///< Raw angle [7:0]
    ANGLE_H = 0x0E, ///< Angle [11:8] (bits 3-0)
    ANGLE_L = 0x0F, ///< Angle [7:0]
    // status registers:
    STATUS = 0x0B, ///< Magnet Detected (bit 5), Magnet too Weak (bit 4), Magnet Too Strong (bit 3)
    AGC = 0x1A,    ///< AGC (automatic gain control)
    MAGN_H = 0x1B, ///< Magnitude[11:8] (bits 3-0)
    MAGN_L = 0x1C, ///< Magnitude[7:0]
    // burn commands:
    BURN = 0xFF, ///< Burn_Angle = 0x80, Burn_Setting = 0x40
  };

  static constexpr int MAGNET_HIGH = (1 << 3);     ///< For use with the STATUS register
  static constexpr int MAGNET_LOW = (1 << 4);      ///< For use with the STATUS register
  static constexpr int MAGNET_DETECTED = (1 << 5); ///< For use with the STATUS register

  velocity_filter_fn velocity_filter_{nullptr};
  std::chrono::duration<float> update_period_;
  std::atomic<int> count_{0};
  std::atomic<int> accumulator_{0};
  std::atomic<float> velocity_rpm_{0};
  std::unique_ptr<Task> task_;
};
} // namespace espp

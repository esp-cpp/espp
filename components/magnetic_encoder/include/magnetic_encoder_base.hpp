#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <system_error>
#include <type_traits>

#include "base_peripheral.hpp"
#include "high_resolution_timer.hpp"
#include "timer.hpp"

namespace espp {
/**
 * @brief CRTP base class for magnetic angle encoders (e.g. As5600, Mt6701).
 *
 * This class holds all of the machinery shared by the magnetic encoders: the
 * periodic update loop (raw-count accumulation + velocity estimation), the
 * position / velocity accessors, and the periodic driver that calls update() at
 * the configured rate. The concrete encoder supplies exactly one thing - a
 * `read(std::error_code&)` that refreshes `count_` from the sensor (and any
 * device-specific state) - which the base invokes through static (CRTP)
 * dispatch, so there is no virtual-call overhead even when the update loop runs
 * at very high frequency (e.g. 1-2 kHz).
 *
 * The periodic driver is selected at compile time via @p UseHighResTimer:
 * - `true`  (default in the encoders' Kconfig): an esp_timer-backed
 *   espp::HighResolutionTimer, which has microsecond resolution and is the
 *   right choice for sub-millisecond update periods.
 * - `false`: an espp::Timer, which schedules against an absolute wake-up time
 *   (the k-th callback targets `start + k * period`) so it is periodic and in
 *   phase - critical for stable velocity / accumulator state - but is limited
 *   to the FreeRTOS tick resolution.
 *
 * @warning You should not call update() yourself if you have configured the
 *          encoder to run its own timer (run_task = true) or if you have called
 *          start().
 *
 * @note There is an implicit assumption regarding the maximum velocity that can
 *       be measured (above which there will be aliasing). The fastest velocity
 *       that can be measured is `0.5 / update_period * 60` RPM, i.e. half a
 *       rotation in one update period. This also bounds the reliability of the
 *       accumulator, since it accumulates position differences every update.
 *
 * @tparam Derived The concrete encoder class (CRTP). Must provide
 *         `void read(std::error_code&)` which updates `count_`.
 * @tparam UseHighResTimer If true, drive updates with HighResolutionTimer;
 *         if false, drive with espp::Timer.
 * @tparam CountsPerRevolution Number of raw counts per mechanical revolution
 *         (e.g. 4096 for the 12-bit As5600, 16384 for the 14-bit Mt6701).
 * @tparam MinDiff Minimum count difference required to update the velocity
 *         estimate; smaller differences are treated as zero velocity to reject
 *         noise / jitter.
 * @tparam RegisterAddressType Register address type for BasePeripheral.
 * @tparam UseAddress Whether the peripheral is addressed (I2C) or not (SSI).
 */
template <typename Derived, bool UseHighResTimer, int CountsPerRevolution, int MinDiff = 2,
          std::integral RegisterAddressType = std::uint8_t, bool UseAddress = true>
class MagneticEncoderBase : public BasePeripheral<RegisterAddressType, UseAddress> {
public:
  /// @brief The periodic driver type selected by @p UseHighResTimer.
  using TimerType = std::conditional_t<UseHighResTimer, espp::HighResolutionTimer, espp::Timer>;

  /**
   * @brief Filter the input raw velocity and return it.
   * @param raw Most recent raw velocity measured.
   * @return Filtered velocity.
   */
  typedef std::function<float(float raw)> velocity_filter_fn;

  static constexpr int COUNTS_PER_REVOLUTION =
      CountsPerRevolution; ///< Int number of counts per revolution for the magnetic encoder.
  static constexpr float COUNTS_PER_REVOLUTION_F =
      (float)CountsPerRevolution; ///< Float number of counts per revolution.
  static constexpr float COUNTS_TO_RADIANS =
      2.0f * (float)(M_PI) /
      COUNTS_PER_REVOLUTION_F; ///< Conversion factor to convert from count value to radians.
  static constexpr float COUNTS_TO_DEGREES =
      360.0f /
      COUNTS_PER_REVOLUTION_F; ///< Conversion factor to convert from count value to degrees.
  static constexpr float SECONDS_PER_MINUTE =
      60.0f; ///< Conversion factor to convert from seconds to minutes.
  static constexpr int MIN_DIFF = MinDiff; ///< Minimum difference for velocity calculation.

  /**
   * @brief Return whether the sensor needs to search for absolute 0 on startup.
   * @note Magnetic angle encoders (using I2C / SPI) always know their absolute
   *       angle on startup, so this always returns false.
   * @return False.
   */
  bool needs_zero_search() const { return false; }

  /**
   * @brief Get the most recently updated raw count value from the encoder.
   * @note This value always represents the angle of the encoder modulo one
   *       rotation, meaning it only represents the range 0 to 360 degrees.
   * @return Raw count value in the range [0, COUNTS_PER_REVOLUTION).
   */
  int get_count() const { return count_.load(); }

  /**
   * @brief Return the accumulated count generated since initialization.
   * @note This value is a raw counter value that can be +/-; divide by
   *       COUNTS_PER_REVOLUTION to convert it to revolutions. It is stored as a
   *       64-bit value so it does not overflow during long, continuous rotation.
   * @return Raw accumulator value.
   */
  int64_t get_accumulator() const { return accumulator_.load(); }

  /**
   * @brief Reset the accumulator to zero.
   */
  void reset_accumulator() { accumulator_ = 0; }

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

  /**
   * @brief Initialize the accumulator to the current position and start the
   *        update timer.
   * @param ec Error code to set if there is an error.
   */
  void initialize(std::error_code &ec) { initialize(true, ec); }

  /**
   * @brief Initialize the accumulator to the current position and start the
   *        update timer, if desired.
   * @param run_task Whether to start the update timer.
   * @param ec Error code to set if there is an error.
   * @note If you do not start the timer, you must call update() manually.
   */
  void initialize(bool run_task, std::error_code &ec) {
    logger_.info("Initializing. Fastest measurable velocity will be {:.3f} RPM",
                 // half a rotation in one update period is the fastest we can measure
                 0.5f / update_period_.count() * SECONDS_PER_MINUTE);
    init(run_task, ec);
    if (ec) {
      logger_.error("Error initializing: {}", ec.message());
    }
  }

  /**
   * @brief Update the state of the encoder by reading the latest data from the
   *        encoder and updating the associated state.
   * @param ec Error code to set if there is an error.
   * @note You should not call this function if you have started the encoder's
   *       update timer (e.g. run_task = true, or you called start()).
   */
  void update(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // sample the timestamp and previous count before the read; both are only
    // committed once the read succeeds
    uint64_t now_us = esp_timer_get_time();
    int prev_count = count_;
    // refresh count_ (and any device-specific state) via CRTP static dispatch
    static_cast<Derived *>(this)->read(ec);
    if (ec) {
      // leave prev_time_us_ untouched so the next successful sample measures its
      // movement over the time since the last good read, not since this failed
      // attempt (which would inflate the velocity / aliasing check)
      return;
    }
    // measure update timing
    auto dt = now_us - prev_time_us_;
    float seconds = dt / 1e6f;
    prev_time_us_ = now_us;
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
    logger_.debug_rate_limited("CDA: {}, {}, {}", count_.load(), diff, accumulator_.load());
    // update velocity (filtering it)
    float raw_velocity =
        (dt > 0 && std::abs(diff) > MIN_DIFF)
            ? (float)(diff) / COUNTS_PER_REVOLUTION_F / seconds * SECONDS_PER_MINUTE
            : 0.0f;
    velocity_rpm_ = velocity_filter_ ? velocity_filter_(raw_velocity) : raw_velocity;
    if (dt > 0) {
      float max_velocity = 0.5f / seconds * SECONDS_PER_MINUTE;
      // compare magnitude so the limit is caught for both rotation directions
      if (std::abs(raw_velocity) >= max_velocity) {
        logger_.warn_rate_limited(
            "Velocity nearing measurement limit ({:.3f} RPM), consider decreasing your "
            "update period!",
            max_velocity);
      }
    }
  }

  /**
   * @brief Start the update timer.
   * @note This will start the timer that calls update() at the update_period.
   * @note This is only useful if you previously stopped the timer or if you
   *       initialized with run_task = false.
   * @return True if the timer was started successfully, false otherwise.
   */
  bool start() {
    logger_.info("Starting update timer with period of {:.3f} seconds", update_period_.count());
    prev_time_us_ = esp_timer_get_time();
    if (!timer_) {
      return false;
    }
    if constexpr (UseHighResTimer) {
      uint64_t period_us =
          std::chrono::duration_cast<std::chrono::microseconds>(update_period_).count();
      return timer_->periodic(period_us);
    } else {
      timer_->set_period(update_period_);
      return timer_->start();
    }
  }

  /**
   * @brief Stop the update timer.
   * @note This will stop the timer that calls update() at the update_period.
   * @note After stopping, you can manually call update() or restart with start().
   */
  void stop() {
    logger_.info("Stopping update timer");
    if (timer_) {
      timer_->stop();
    }
  }

protected:
  using Base = BasePeripheral<RegisterAddressType, UseAddress>;
  using Base::base_mutex_;
  using Base::logger_;

  /**
   * @brief Construct the base, forwarding an empty peripheral config.
   * @param name Name used for the peripheral and its update timer.
   * @param velocity_filter Optional velocity filter, called once per update.
   * @param update_period Update period (1 / sample rate) in seconds.
   * @param log_level Log verbosity.
   * @note The concrete encoder must set the transport (write / read /
   *       write_then_read / address) in its own constructor body, then call
   *       initialize(). The update timer is created here but never auto-starts;
   *       start() (via initialize()) is authoritative.
   */
  MagneticEncoderBase(std::string_view name, const velocity_filter_fn &velocity_filter,
                      const std::chrono::duration<float> &update_period,
                      espp::Logger::Verbosity log_level)
      : Base({}, name, log_level)
      , velocity_filter_(velocity_filter)
      , update_period_(update_period) {
    // Construct the timer here in the constructor body (after update_period_ is
    // set) so the espp::Timer branch can read the period. auto_start is false so
    // that the timer does not fire until start() is called.
    if constexpr (UseHighResTimer) {
      // Use the same bool-returning lambda as the espp::Timer branch below: a
      // bool-returning callable converts to std::function<void()> (the current
      // HighResolutionTimer callback type, which discards the result) AND to
      // std::function<bool()>, so this stays valid even if the HRT callback
      // signature is ever aligned with espp::Timer's.
      timer_ = std::make_unique<espp::HighResolutionTimer>(espp::HighResolutionTimer::Config{
          .name = std::string(name), .callback = [this]() { return update_task(); }});
    } else {
      timer_ = std::make_unique<espp::Timer>(
          espp::Timer::Config{.name = name,
                              .period = update_period_,
                              .callback = [this]() { return update_task(); },
                              .auto_start = false});
    }
  }

  /**
   * @brief The periodic callback: run one update().
   * @return Always false (never cancels the timer).
   * @note For the HighResolutionTimer branch the bool return is ignored; for the
   *       espp::Timer branch, returning false keeps the timer running.
   */
  bool update_task() {
    std::error_code ec;
    update(ec);
    if (ec) {
      logger_.error("Error updating: {}", ec.message());
    }
    // don't want to stop the timer
    return false;
  }

  /**
   * @brief Seed the accumulator from the current angle and (optionally) start.
   * @param run_task Whether to start the update timer.
   * @param ec Error code to set if there is an error.
   */
  void init(bool run_task, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // initialize the accumulator to have the current angle
    static_cast<Derived *>(this)->read(ec);
    if (ec) {
      return;
    }
    accumulator_ = count_.load();
    // seed the timestamp so the first update() (in particular a manual update()
    // when run_task is false) measures dt from now, not from boot. start() also
    // refreshes it for timer mode.
    prev_time_us_ = esp_timer_get_time();
    if (!run_task) {
      logger_.info(
          "Not starting timer, run_task is false. Manually call update() to update the state.");
      return;
    }
    if (!start()) {
      logger_.error("Error starting update timer");
      ec = make_error_code(std::errc::operation_not_permitted);
    }
  }

  velocity_filter_fn velocity_filter_{nullptr};
  uint64_t prev_time_us_{0};
  std::chrono::duration<float> update_period_;
  std::atomic<int> count_{0};
  std::atomic<int64_t> accumulator_{0};
  std::atomic<float> velocity_rpm_{0};
  std::unique_ptr<TimerType> timer_;
};
} // namespace espp

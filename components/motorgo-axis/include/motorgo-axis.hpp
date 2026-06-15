#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>

#include <driver/gpio.h>

#include "base_component.hpp"
#include "bldc_driver.hpp"
#include "gaussian.hpp"
#include "i2c.hpp"
#include "led.hpp"
#include "lsm6dso.hpp"
#include "mt6701.hpp"
#include "spi.hpp"
#include "task.hpp"

namespace espp {
/// MotorGoAxis is a lightweight board-support component for the MotorGo Axis
/// hardware.
///
/// It exposes the documented pin mapping for the board's:
/// - two 6-PWM BLDC motor outputs
/// - two encoder chip-selects on a shared SSI/SPI bus
/// - one onboard LSM6DS3TR IMU on the hidden I2C bus
/// - Qwiic and hidden I2C buses
/// - user (green) and status (blue) LEDs
///
/// The class is a singleton and can be accessed with get().
///
/// \section motorgo_axis_ex1 MotorGo Axis Example
/// \snippet motorgo_axis_example.cpp motorgo-axis example
class MotorGoAxis : public BaseComponent {
public:
  /// Alias for the SSI-based magnetic encoder helper used on each motor
  /// channel.
  using Encoder = espp::Mt6701<espp::Mt6701Interface::SSI>;
  /// Alias for the 6-PWM BLDC motor driver helper used on each motor channel.
  using MotorDriver = espp::BldcDriver;
  /// Alias for the onboard hidden-bus IMU helper.
  using Imu = espp::Lsm6dso<espp::lsm6dso::Interface::I2C>;

  /// Pin mapping for one BLDC motor channel.
  struct MotorPins {
    gpio_num_t u_high{GPIO_NUM_NC}; ///< Phase U high-side PWM pin.
    gpio_num_t u_low{GPIO_NUM_NC};  ///< Phase U low-side PWM pin.
    gpio_num_t v_high{GPIO_NUM_NC}; ///< Phase V high-side PWM pin.
    gpio_num_t v_low{GPIO_NUM_NC};  ///< Phase V low-side PWM pin.
    gpio_num_t w_high{GPIO_NUM_NC}; ///< Phase W high-side PWM pin.
    gpio_num_t w_low{GPIO_NUM_NC};  ///< Phase W low-side PWM pin.
  };

  /// Shared encoder SSI/SPI bus pins.
  struct EncoderBusPins {
    gpio_num_t sclk{GPIO_NUM_NC}; ///< Shared encoder clock pin.
    gpio_num_t miso{GPIO_NUM_NC}; ///< Shared encoder data pin.
  };

  /// I2C pin mapping.
  struct I2cPins {
    gpio_num_t sda{GPIO_NUM_NC}; ///< I2C SDA pin.
    gpio_num_t scl{GPIO_NUM_NC}; ///< I2C SCL pin.
  };

  /// User-visible LED pin mapping.
  struct LedPins {
    gpio_num_t user{GPIO_NUM_NC};   ///< User LED pin.
    gpio_num_t status{GPIO_NUM_NC}; ///< Status LED pin.
  };

  /// Access the singleton board instance.
  /// \return Reference to the singleton MotorGoAxis object.
  static MotorGoAxis &get() {
    static MotorGoAxis instance;
    return instance;
  }

  /// Deleted copy constructor.
  MotorGoAxis(const MotorGoAxis &) = delete;
  /// Deleted copy assignment operator.
  MotorGoAxis &operator=(const MotorGoAxis &) = delete;
  /// Deleted move constructor.
  MotorGoAxis(MotorGoAxis &&) = delete;
  /// Deleted move assignment operator.
  MotorGoAxis &operator=(MotorGoAxis &&) = delete;

  /// Get the external Qwiic I2C bus.
  /// \return Reference to the initialized external I2C bus.
  I2c &get_external_i2c();

  /// Get the external Qwiic I2C bus.
  /// \return Reference to the initialized external I2C bus.
  I2c &qwiic_i2c();

  /// Get the internal hidden I2C bus.
  /// \return Reference to the initialized internal I2C bus.
  I2c &hidden_i2c();

  /// Get the documented motor pin mappings for both channels.
  /// \return Array containing the six PWM pins for each motor channel.
  std::array<MotorPins, 2> motor_pins() const;

  /// Get one motor channel pin mapping.
  /// \param index Zero-based motor index in the range [0, num_motor_channels()).
  /// \return The motor pin mapping for the requested channel, or a default
  ///         `MotorPins{}` with `GPIO_NUM_NC` entries if the index is invalid.
  MotorPins motor_pins(size_t index) const;

  /// Get the shared encoder bus pin mapping.
  /// \return The shared SCLK and MISO pins used for the encoder bus.
  EncoderBusPins encoder_bus_pins() const;

  /// Get the two encoder chip-select pins.
  /// \return Array containing the chip-select pin for each encoder channel.
  std::array<gpio_num_t, 2> encoder_cs_pins() const;

  /// Get one encoder chip-select pin.
  /// \param index Zero-based encoder index in the range [0,
  ///              num_motor_channels()).
  /// \return The requested encoder chip-select pin, or `GPIO_NUM_NC` if the
  ///         index is invalid.
  gpio_num_t encoder_cs_pin(size_t index) const;

  /// Get the external Qwiic I2C pin mapping.
  /// \return The external Qwiic SDA/SCL pin mapping.
  I2cPins qwiic_pins() const;

  /// Get the hidden internal I2C pin mapping.
  /// \return The internal SDA/SCL pin mapping.
  I2cPins hidden_i2c_pins() const;

  /// Get the user/status LED pins.
  /// \return The documented user and status LED GPIOs.
  LedPins led_pins() const;

  /// Initialize the two BLDC motor driver helpers.
  /// \param power_supply_voltage Maximum motor supply voltage in volts.
  /// \param limit_voltage Optional motor voltage limit in volts. Values less
  ///                      than zero mean "no extra limit" and will be clamped
  ///                      to the supply voltage by `espp::BldcDriver`.
  /// \param dead_zone_ns Dead-time to apply to both sides of each phase PWM.
  /// \return True if both motor driver helpers were initialized successfully;
  ///         false if the configuration is invalid or any channel could not be
  ///         configured.
  bool initialize_motors(float power_supply_voltage = driver_default_power_supply_voltage(),
                         float limit_voltage = driver_default_voltage_limit(),
                         uint64_t dead_zone_ns = default_motor_dead_zone_ns());

  /// Get one motor driver helper.
  /// \param index Zero-based motor index in the range [0, num_motor_channels()).
  /// \return Shared pointer to the requested motor driver helper, or `nullptr`
  ///         if the index is invalid or that motor has not been initialized yet.
  std::shared_ptr<MotorDriver> motor_driver(size_t index) const;

  /// Check whether one motor driver helper is currently enabled.
  /// \param index Zero-based motor index in the range [0, num_motor_channels()).
  /// \return True if the requested motor driver exists and is enabled.
  bool motor_driver_enabled(size_t index) const;

  /// Enable one motor driver helper.
  /// \param index Zero-based motor index in the range [0, num_motor_channels()).
  /// \return True if the driver exists and was enabled.
  bool enable_motor_driver(size_t index);

  /// Disable one motor driver helper.
  /// \param index Zero-based motor index in the range [0, num_motor_channels()).
  void disable_motor_driver(size_t index);

  /// Enable all initialized motor driver helpers.
  void enable_all_motor_drivers();

  /// Disable all initialized motor driver helpers.
  void disable_all_motor_drivers();

  /// Initialize the shared encoder bus and create the two MT6701 SSI helpers.
  /// \param run_tasks If true, each encoder starts its own update task after
  ///                  initialization. If false, callers must invoke
  ///                  `Encoder::update()` manually.
  /// \return True if the shared bus and both encoder helpers were initialized
  ///         successfully; false otherwise.
  bool initialize_encoders(bool run_tasks = true);

  /// Get one encoder helper.
  /// \param index Zero-based encoder index in the range [0,
  ///              num_motor_channels()).
  /// \return Shared pointer to the requested encoder helper, or `nullptr` if the
  ///         index is invalid or that encoder has not been initialized yet.
  std::shared_ptr<Encoder> encoder(size_t index);

  /// Reset one encoder's accumulator.
  /// \param index Zero-based encoder index in the range [0,
  ///              num_motor_channels()).
  /// \details If the encoder has not been initialized, the request is ignored
  ///          after logging an error.
  void reset_encoder_accumulator(size_t index);

  /// Initialize the onboard IMU on the hidden I2C bus.
  /// \param orientation_filter Optional orientation filter callback used by the
  ///                           IMU driver when update() is called.
  /// \param imu_config Basic accelerometer / gyroscope configuration to apply
  ///                   during initialization.
  /// \return True if the hidden-bus I2C device and IMU helper were initialized
  ///         successfully; false otherwise.
  bool initialize_imu(const Imu::filter_fn &orientation_filter = nullptr,
                      const Imu::ImuConfig &imu_config = {
                          .accel_range = Imu::AccelRange::RANGE_2G,
                          .accel_odr = Imu::AccelODR::ODR_104_HZ,
                          .gyro_range = Imu::GyroRange::DPS_1000,
                          .gyro_odr = Imu::GyroODR::ODR_104_HZ,
                      });

  /// Get the onboard IMU helper.
  /// \return Shared pointer to the initialized IMU helper, or `nullptr` if
  ///         initialize_imu() has not been called yet.
  std::shared_ptr<Imu> imu() const;

  /// Initialize the user/status LED helpers.
  /// \param breathing_period Default breathing period in seconds for
  ///                         start_led_breathing().
  /// \return True if the indicator LED PWM helper and breathing task were
  ///         created successfully; false if they were already initialized or the
  ///         LEDs could not be driven after setup.
  bool initialize_leds(float breathing_period = 3.5f);

  /// Start breathing both indicator LEDs.
  /// \details This uses the shared Gaussian waveform returned by gaussian().
  void start_led_breathing();

  /// Stop breathing and turn both indicator LEDs off.
  void stop_led_breathing();

  /// Set the user LED brightness in the range [0, 1].
  /// \param brightness Desired brightness, clamped to [0.0, 1.0].
  /// \return True if the user LED duty cycle was updated; false if the LEDs have
  ///         not been initialized or the breathing task is currently running.
  bool set_user_led_brightness(float brightness);

  /// Get the user LED brightness in the range [0, 1].
  /// \return Current user LED brightness normalized to [0.0, 1.0], or 0.0f if
  ///         the LEDs have not been initialized or the duty cycle could not be
  ///         read.
  float get_user_led_brightness();

  /// Set the status LED brightness in the range [0, 1].
  /// \param brightness Desired brightness, clamped to [0.0, 1.0].
  /// \return True if the status LED duty cycle was updated; false if the LEDs
  ///         have not been initialized or the breathing task is currently
  ///         running.
  bool set_status_led_brightness(float brightness);

  /// Get the status LED brightness in the range [0, 1].
  /// \return Current status LED brightness normalized to [0.0, 1.0], or 0.0f if
  ///         the LEDs have not been initialized or the duty cycle could not be
  ///         read.
  float get_status_led_brightness();

  /// Set the LED breathing period in seconds.
  /// \param period Breathing period in seconds. Must be greater than zero.
  /// \return True if the period was accepted; false if the value is not
  ///         positive.
  bool set_led_breathing_period(float period);

  /// Get the LED breathing period in seconds.
  /// \return The configured LED breathing period in seconds.
  float get_led_breathing_period();

  /// Get the LED helper used for the indicator LEDs.
  /// \return Shared pointer to the indicator LED helper, or `nullptr` if
  ///         initialize_leds() has not been called yet.
  std::shared_ptr<espp::Led> leds();

  /// Get the Gaussian waveform used for LED breathing.
  /// \return Reference to the Gaussian waveform object used by the LED
  ///         breathing task.
  espp::Gaussian &gaussian();

  /// Get the number of supported motor channels.
  /// \return The number of motor channels exposed by the board.
  static constexpr size_t num_motor_channels() { return motor_pin_map_.size(); }
  /// Get the default motor supply voltage used for the board definition.
  /// \return Default power supply voltage in volts.
  static constexpr float driver_default_power_supply_voltage() {
    return driver_default_power_supply_voltage_;
  }
  /// Get the default motor voltage limit used for the board definition.
  /// \return Default motor voltage limit in volts.
  static constexpr float driver_default_voltage_limit() { return driver_default_voltage_limit_; }
  /// Get the default BLDC driver dead-time.
  /// \return Default dead-time in nanoseconds.
  static constexpr uint64_t default_motor_dead_zone_ns() { return default_motor_dead_zone_ns_; }
  /// Get the encoder bus clock speed.
  /// \return Default encoder SSI/SPI clock speed in Hz.
  static constexpr size_t encoder_spi_clock_speed_hz() { return encoder_spi_clock_speed_hz_; }
  /// Get the user LED GPIO.
  /// \return User LED pin.
  static constexpr gpio_num_t user_led_pin() { return user_led_pin_; }
  /// Get the status LED GPIO.
  /// \return Status LED pin.
  static constexpr gpio_num_t status_led_pin() { return status_led_pin_; }

protected:
  MotorGoAxis();

  bool initialize_encoder_spi();
  bool read_encoder(size_t index, uint8_t *data, size_t size);
  float led_breathe(float phase_offset = 0.0f);
  bool led_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);

  static constexpr std::array<MotorPins, 2> motor_pin_map_{{
      {.u_high = GPIO_NUM_3,
       .u_low = GPIO_NUM_17,
       .v_high = GPIO_NUM_8,
       .v_low = GPIO_NUM_16,
       .w_high = GPIO_NUM_18,
       .w_low = GPIO_NUM_15},
      {.u_high = GPIO_NUM_41,
       .u_low = GPIO_NUM_11,
       .v_high = GPIO_NUM_42,
       .v_low = GPIO_NUM_10,
       .w_high = GPIO_NUM_12,
       .w_low = GPIO_NUM_9},
  }};
  static constexpr std::array<gpio_num_t, 2> encoder_cs_pin_map_{
      GPIO_NUM_6,
      GPIO_NUM_7,
  };
  static constexpr EncoderBusPins encoder_bus_pin_map_{.sclk = GPIO_NUM_5, .miso = GPIO_NUM_4};
  static constexpr I2cPins qwiic_pin_map_{
      .sda = GPIO_NUM_1,
      .scl = GPIO_NUM_2,
  };
  static constexpr I2cPins hidden_i2c_pin_map_{
      .sda = GPIO_NUM_13,
      .scl = GPIO_NUM_14,
  };
  static constexpr gpio_num_t user_led_pin_ = GPIO_NUM_43;
  static constexpr gpio_num_t status_led_pin_ = GPIO_NUM_44;
  static constexpr spi_host_device_t encoder_spi_host_ = SPI2_HOST;
  static constexpr size_t encoder_spi_clock_speed_hz_ = 10 * 1000 * 1000;
  static constexpr float driver_default_power_supply_voltage_ = 17.0f;
  static constexpr float driver_default_voltage_limit_ = 17.0f;
  static constexpr uint64_t default_motor_dead_zone_ns_ = 100;
  static constexpr size_t indicator_led_pwm_frequency_hz_ = 5 * 1000;
  static constexpr float encoder_update_period_seconds_ = 0.001f;

  I2c qwiic_i2c_{{.port = I2C_NUM_0,
                  .sda_io_num = qwiic_pin_map_.sda,
                  .scl_io_num = qwiic_pin_map_.scl,
                  .sda_pullup_en = GPIO_PULLUP_ENABLE,
                  .scl_pullup_en = GPIO_PULLUP_ENABLE}};
  I2c hidden_i2c_{{.port = I2C_NUM_1,
                   .sda_io_num = hidden_i2c_pin_map_.sda,
                   .scl_io_num = hidden_i2c_pin_map_.scl,
                   .sda_pullup_en = GPIO_PULLUP_ENABLE,
                   .scl_pullup_en = GPIO_PULLUP_ENABLE}};

  std::unique_ptr<Spi> encoder_spi_;
  std::array<std::shared_ptr<Spi::Device>, 2> encoder_spi_devices_{};
  std::array<std::shared_ptr<Encoder>, 2> encoders_{};
  std::shared_ptr<I2c::Device<uint8_t>> imu_i2c_device_;
  std::shared_ptr<Imu> imu_;

  std::array<espp::Led::ChannelConfig, 2> led_channels_{{
      {static_cast<size_t>(user_led_pin_), LEDC_CHANNEL_4, LEDC_TIMER_1},
      {static_cast<size_t>(status_led_pin_), LEDC_CHANNEL_5, LEDC_TIMER_1},
  }};

  std::array<std::shared_ptr<MotorDriver>, 2> motor_drivers_{};
  std::shared_ptr<espp::Led> indicator_leds_;
  std::unique_ptr<espp::Task> led_task_;
  std::atomic<float> breathing_period_{3.5f};
  std::chrono::high_resolution_clock::time_point breathing_start_{};
  espp::Gaussian gaussian_{{.gamma = 0.1f, .alpha = 1.0f, .beta = 0.5f}};
}; // class MotorGoAxis
} // namespace espp

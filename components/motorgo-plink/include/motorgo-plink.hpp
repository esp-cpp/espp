#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <vector>

#include <driver/gpio.h>

#include "base_component.hpp"
#include "bdc_driver.hpp"
#include "gaussian.hpp"
#include "i2c.hpp"
#include "led.hpp"
#include "lsm6dso.hpp"
#include "mt6701.hpp"
#include "spi.hpp"
#include "task.hpp"

namespace espp {
/// MotorGoPlink is a lightweight board-support component for the MotorGo Plink
/// hardware.
///
/// It exposes the documented pin mapping for the board's:
/// - four dual-PWM DC motor outputs
/// - four encoder chip-selects on a shared SSI/SPI bus
/// - four RC-servo signal pins
/// - one onboard LSM6DS33 IMU on the hidden I2C bus
/// - Qwiic and hidden I2C buses
/// - user and status LEDs
///
/// The class is a singleton and can be accessed with get().
///
/// \section motorgo_plink_ex1 MotorGo Plink Example
/// \snippet motorgo_plink_example.cpp motorgo-plink example
class MotorGoPlink : public BaseComponent {
public:
  /// Alias for the SSI-based magnetic encoder helper used on each motor
  /// channel.
  using Encoder = espp::Mt6701<espp::Mt6701Interface::SSI>;
  /// Alias for the brushed DC motor driver helper used on each motor channel.
  using MotorDriver = espp::BdcDriver;
  /// Alias for the onboard hidden-bus IMU helper.
  using Imu = espp::Lsm6dso<espp::lsm6dso::Interface::I2C>;

  /// Pin mapping for one DC motor channel.
  struct MotorPins {
    gpio_num_t pwm_a{GPIO_NUM_NC}; ///< First PWM pin for the motor channel.
    gpio_num_t pwm_b{GPIO_NUM_NC}; ///< Second PWM pin for the motor channel.
  };

  /// Shared encoder SSI/SPI bus pins.
  struct EncoderBusPins {
    gpio_num_t sclk{GPIO_NUM_NC}; ///< Shared encoder clock pin.
    gpio_num_t miso{GPIO_NUM_NC}; ///< Shared encoder data pin.
    gpio_num_t mosi{GPIO_NUM_NC}; ///< Dummy MOSI pin used to satisfy the SPI host.
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
  /// \return Reference to the singleton MotorGoPlink object.
  static MotorGoPlink &get() {
    static MotorGoPlink instance;
    return instance;
  }

  /// Deleted copy constructor.
  MotorGoPlink(const MotorGoPlink &) = delete;
  /// Deleted copy assignment operator.
  MotorGoPlink &operator=(const MotorGoPlink &) = delete;
  /// Deleted move constructor.
  MotorGoPlink(MotorGoPlink &&) = delete;
  /// Deleted move assignment operator.
  MotorGoPlink &operator=(MotorGoPlink &&) = delete;

  /// Get the external Qwiic I2C bus.
  /// \return Reference to the initialized external I2C bus.
  I2c &get_external_i2c();

  /// Get the external Qwiic I2C bus.
  /// \return Reference to the initialized external I2C bus.
  I2c &qwiic_i2c();

  /// Get the internal hidden I2C bus.
  /// \return Reference to the initialized internal I2C bus.
  I2c &hidden_i2c();

  /// Get the documented motor pin mappings for all four channels.
  /// \return Array containing the two PWM pins for each motor
  ///         channel.
  std::array<MotorPins, 4> motor_pins() const;

  /// Get one motor channel pin mapping.
  /// \param index Zero-based motor index in the range [0, num_motor_channels()).
  /// \return The motor pin mapping for the requested channel, or a default
  ///         `MotorPins{}` with `GPIO_NUM_NC` entries if the index is invalid.
  MotorPins motor_pins(size_t index) const;

  /// Get the shared encoder bus pin mapping.
  /// \return The shared SCLK, MISO, and dummy MOSI pins used for the encoder
  ///         bus.
  EncoderBusPins encoder_bus_pins() const;

  /// Get the four encoder chip-select pins.
  /// \return Array containing the chip-select pin for each encoder channel.
  std::array<gpio_num_t, 4> encoder_cs_pins() const;

  /// Get one encoder chip-select pin.
  /// \param index Zero-based encoder index in the range [0,
  ///              num_motor_channels()).
  /// \return The requested encoder chip-select pin, or `GPIO_NUM_NC` if the
  ///         index is invalid.
  gpio_num_t encoder_cs_pin(size_t index) const;

  /// Get the four servo signal pins.
  /// \return Array containing the four documented RC-servo signal GPIOs.
  std::array<gpio_num_t, 4> servo_pins() const;

  /// Get one servo signal pin.
  /// \param index Zero-based servo index in the range [0, num_servo_channels()).
  /// \return The requested servo signal pin, or `GPIO_NUM_NC` if the index is
  ///         invalid.
  gpio_num_t servo_pin(size_t index) const;

  /// Get the external Qwiic I2C pin mapping.
  /// \return The external Qwiic SDA/SCL pin mapping.
  I2cPins qwiic_pins() const;

  /// Get the hidden internal I2C pin mapping.
  /// \return The internal SDA/SCL pin mapping.
  I2cPins hidden_i2c_pins() const;

  /// Get the user/status LED pins.
  /// \return The documented user and status LED GPIOs.
  LedPins led_pins() const;

  /// Initialize the four MCPWM-backed motor driver helpers.
  /// \param pwm_frequency_hz PWM carrier frequency for all four motor channels.
  /// \return True if the motor driver helpers were initialized; false if the PWM
  ///         frequency is invalid or any motor channel could not be configured.
  bool initialize_motors(size_t pwm_frequency_hz = motor_default_pwm_frequency_hz());

  /// Set a normalized motor speed in the range [-1, 1].
  /// \param index Zero-based motor index in the range [0, num_motor_channels()).
  /// \param speed Normalized motor command. Values are clamped to [-1.0, 1.0],
  ///              with positive values driving `pwm_a`, negative values driving
  ///              `pwm_b`, and zero disabling both PWM outputs for that motor.
  /// \return True if the motor command was applied; false if the motor PWM
  ///         subsystem has not been initialized or the index is invalid.
  bool set_motor_speed(size_t index, float speed);

  /// Get the last commanded normalized motor speed.
  /// \param index Zero-based motor index in the range [0, num_motor_channels()).
  /// \return The last commanded normalized speed for that channel, or 0.0f if
  ///         the index is invalid.
  float motor_speed(size_t index) const;

  /// Stop one motor channel.
  /// \param index Zero-based motor index in the range [0, num_motor_channels()).
  void stop_motor(size_t index);

  /// Stop all motor channels.
  /// \details If the motor PWM subsystem has been initialized, each motor is
  ///          actively commanded to zero speed. Otherwise the cached command
  ///          values are simply reset to zero.
  void stop_all_motors();

  /// Get one motor driver helper.
  /// \param index Zero-based motor index in the range [0, num_motor_channels()).
  /// \return Shared pointer to the requested motor driver helper, or `nullptr`
  ///         if the index is invalid or that motor has not been initialized yet.
  std::shared_ptr<MotorDriver> motor_driver(size_t index) const;

  /// Initialize the shared encoder bus and create the four MT6701 SSI helpers.
  /// \param run_tasks If true, each encoder starts its own update task after
  ///                  initialization. If false, callers must invoke
  ///                  `Encoder::update()` manually.
  /// \return True if the shared bus and all four encoder helpers were
  ///         initialized successfully; false otherwise.
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
  /// Get the number of exposed servo signal pins.
  /// \return The number of servo channels exposed by the board.
  static constexpr size_t num_servo_channels() { return servo_pin_map_.size(); }
  /// Get the default PWM frequency used for motor outputs.
  /// \return Default motor PWM frequency in Hz.
  static constexpr size_t motor_default_pwm_frequency_hz() {
    return motor_default_pwm_frequency_hz_;
  }
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
  MotorGoPlink();

  bool initialize_encoder_spi();
  bool read_encoder(size_t index, uint8_t *data, size_t size);
  float led_breathe(float phase_offset = 0.0f);
  bool led_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);

  static constexpr std::array<MotorPins, 4> motor_pin_map_{{
      {.pwm_a = GPIO_NUM_16, .pwm_b = GPIO_NUM_15},
      {.pwm_a = GPIO_NUM_17, .pwm_b = GPIO_NUM_18},
      {.pwm_a = GPIO_NUM_8, .pwm_b = GPIO_NUM_3},
      {.pwm_a = GPIO_NUM_11, .pwm_b = GPIO_NUM_12},
  }};
  static constexpr std::array<gpio_num_t, 4> encoder_cs_pin_map_{
      GPIO_NUM_4,
      GPIO_NUM_5,
      GPIO_NUM_10,
      GPIO_NUM_9,
  };
  static constexpr std::array<gpio_num_t, 4> servo_pin_map_{
      GPIO_NUM_38,
      GPIO_NUM_40,
      GPIO_NUM_39,
      GPIO_NUM_37,
  };
  static constexpr EncoderBusPins encoder_bus_pin_map_{
      .sclk = GPIO_NUM_7,
      .miso = GPIO_NUM_6,
      .mosi = GPIO_NUM_45,
  };
  static constexpr I2cPins qwiic_pin_map_{
      .sda = GPIO_NUM_1,
      .scl = GPIO_NUM_2,
  };
  static constexpr I2cPins hidden_i2c_pin_map_{
      .sda = GPIO_NUM_13,
      .scl = GPIO_NUM_14,
  };
  static constexpr gpio_num_t user_led_pin_ = GPIO_NUM_42;
  static constexpr gpio_num_t status_led_pin_ = GPIO_NUM_41;
  static constexpr spi_host_device_t encoder_spi_host_ = SPI2_HOST;
  static constexpr size_t encoder_spi_clock_speed_hz_ = 10 * 1000 * 1000;
  static constexpr size_t motor_default_pwm_frequency_hz_ = 20 * 1000;
  static constexpr size_t indicator_led_pwm_frequency_hz_ = 5 * 1000;
  static constexpr float encoder_update_period_seconds_ = 0.001f;
  static constexpr std::array<int, 4> motor_driver_group_ids_{0, 0, 0, 1};

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
  std::array<std::shared_ptr<Spi::Device>, 4> encoder_spi_devices_{};
  std::array<std::shared_ptr<Encoder>, 4> encoders_{};
  std::shared_ptr<I2c::Device<uint8_t>> imu_i2c_device_;
  std::shared_ptr<Imu> imu_;

  std::array<espp::Led::ChannelConfig, 2> led_channels_{{
      {static_cast<size_t>(user_led_pin_), LEDC_CHANNEL_4, LEDC_TIMER_1},
      {static_cast<size_t>(status_led_pin_), LEDC_CHANNEL_5, LEDC_TIMER_1},
  }};

  std::array<std::shared_ptr<MotorDriver>, 4> motor_drivers_{};
  std::shared_ptr<espp::Led> indicator_leds_;
  std::array<float, 4> motor_speeds_{0.0f, 0.0f, 0.0f, 0.0f};
  std::unique_ptr<espp::Task> led_task_;
  std::atomic<float> breathing_period_{3.5f};
  std::chrono::high_resolution_clock::time_point breathing_start_{};
  espp::Gaussian gaussian_{{.gamma = 0.1f, .alpha = 1.0f, .beta = 0.5f}};
}; // class MotorGoPlink
} // namespace espp

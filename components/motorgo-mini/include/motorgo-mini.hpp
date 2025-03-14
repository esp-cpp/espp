#pragma once

#include <string>
#include <vector>

#include <driver/spi_master.h>

#include "base_component.hpp"
#include "bldc_driver.hpp"
#include "bldc_motor.hpp"
#include "gaussian.hpp"
#include "i2c.hpp"
#include "interrupt.hpp"
#include "led.hpp"
#include "mt6701.hpp"
#include "oneshot_adc.hpp"
#include "simple_lowpass_filter.hpp"

namespace espp {
/// This class acts as a board support component for the MotorGo-Mini board.
/// It provides a high-level interface to the board's functionality.
/// The MotorGo-Mini board is a motor driver board that can drive two DC motors.
/// More information about the board can be found at:
/// https://github.com/Every-Flavor-Robotics/motorgo-mini-hardware
///
/// High level overview of the board:
/// - ESP32s3 module
/// - Two LEDs (yellow and red)
/// - Two BLDC motor drivers (TMC6300)
/// - Two magnetic encoders (connected via two JST-SH 5 pin connectors to EncoderGo boards)
/// (SPI2_HOST)
/// - Two Qwiic connectors for I2C communication (I2C0)
/// - Current sense resistors for each motor on the U and W phases
///   NOTE: on the MotorGo-Mini v1.3, the M2 motor sense is swapped in the
///         schematic, with U and W swapped
///
/// The class is a singleton and can be accessed using the get() method.
///
/// \section motorgo_ex1 MotorGo-Mini Example
/// \snippet motorgo_mini_example.cpp motorgo-mini example
class MotorGoMini : public BaseComponent {
public:
  /// Alias for the button callback function
  using button_callback_t = espp::Interrupt::event_callback_fn;

  /// Alias for the encoder type
  using Encoder = espp::Mt6701<espp::Mt6701Interface::SSI>;

  /// Alias for the BLDC motor type
  using BldcMotor = espp::BldcMotor<espp::BldcDriver, Encoder>;

  /// Alias for the velocity filter type
  using VelocityFilter = espp::SimpleLowpassFilter;

  /// Alias for the angle filter type
  using AngleFilter = espp::SimpleLowpassFilter;

  /// @brief Access the singleton instance of the MotorGoMini class
  /// @return Reference to the singleton instance of the MotorGoMini class
  static MotorGoMini &get() {
    static MotorGoMini instance;
    return instance;
  }

  MotorGoMini(const MotorGoMini &) = delete;
  MotorGoMini &operator=(const MotorGoMini &) = delete;
  MotorGoMini(MotorGoMini &&) = delete;
  MotorGoMini &operator=(MotorGoMini &&) = delete;

  /// Get a reference to the external I2C bus
  /// \return A reference to the external I2C bus
  I2c &get_external_i2c();

  /// Get a reference to the interrupts
  /// \return A reference to the interrupts
  espp::Interrupt &interrupts();

  /////////////////////////////////////////////////////////////////////////////
  // Button
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the button
  /// \param callback The callback function to call when the button is pressed
  /// \return true if the button was successfully initialized, false otherwise
  bool initialize_button(const button_callback_t &callback = nullptr);

  /// Get the button state
  /// \return The button state (true = button pressed, false = button released)
  bool button_state() const;

  /////////////////////////////////////////////////////////////////////////////
  // LEDs
  /////////////////////////////////////////////////////////////////////////////

  /// Get a reference to the yellow LED channel (channel 0)
  /// \return A reference to the yellow LED channel (channel 0)
  espp::Led::ChannelConfig &yellow_led();

  /// Get a reference to the yellow LED channel (channel 0)
  /// \return A reference to the yellow LED channel (channel 0)
  espp::Led::ChannelConfig &led_channel0();

  /// Set the duty cycle of the yellow LED
  /// \param duty The duty cycle of the yellow LED (0.0 - 100.0)
  /// \note The duty cycle is a percentage of the maximum duty cycle
  ///      (which is 100.0)
  ///      0.0 is off, 100.0 is fully on
  /// \note For this function to have an effect, the LED timer must NOT be running
  ///      (i.e. stop_breathing() must be called)
  void set_yellow_led_duty(float duty);

  /// Get a reference to the red LED channel (channel 1)
  /// \return A reference to the red LED channel (channel 1)
  espp::Led::ChannelConfig &red_led();

  /// Get a reference to the red LED channel (channel 1)
  /// \return A reference to the red LED channel (channel 1)
  espp::Led::ChannelConfig &led_channel1();

  /// Set the duty cycle of the red LED
  /// \param duty The duty cycle of the red LED (0.0 - 100.0)
  /// \note The duty cycle is a percentage of the maximum duty cycle
  ///      (which is 100.0)
  ///      0.0 is off, 100.0 is fully on
  /// \note For this function to have an effect, the LED timer must NOT be running
  ///      (i.e. stop_breathing() must be called)
  void set_red_led_duty(float duty);

  /// Get a reference to the LED object which controls the LEDs
  /// \return A reference to the Led object
  espp::Led &led();

  /// \brief Get a reference to the Gaussian object which is used for breathing
  ///        the LEDs.
  /// \return A reference to the Gaussian object
  espp::Gaussian &gaussian();

  /// Start breathing the LEDs
  /// \details This function starts the LED timer which will periodically update
  ///          the LED duty cycle using the gaussian to create a breathing
  ///          efect.
  void start_breathing();

  /// Stop breathing the LEDs
  /// \details This function stops the LED timer which will stop updating the
  ///         LED duty cycle. It will also set the LED duty cycle to 0,
  ///         effectively turning off the LEDs.
  void stop_breathing();

  /////////////////////////////////////////////////////////////////////////////
  // ADCs
  /////////////////////////////////////////////////////////////////////////////

  /// Get a reference to the ADC_UNIT_1 OneshotAdc object
  /// \return A reference to the ADC_UNIT_1 OneshotAdc object
  espp::OneshotAdc &adc1();

  /// Get a reference to the ADC_UNIT_2 OneshotAdc object
  /// \return A reference to the ADC_UNIT_2 OneshotAdc object
  espp::OneshotAdc &adc2();

  /////////////////////////////////////////////////////////////////////////////
  // Motors
  /////////////////////////////////////////////////////////////////////////////

  /// Driver Configuration for the MotorGo-Mini Motor Driver(s)
  struct DriverConfig {
    float power_supply_voltage; ///< The power supply voltage in volts
    float limit_voltage;        ///< The limit voltage in volts
  };

  /// Default configuration for the MotorGo-Mini's BLDC motor on channel 1
  const BldcMotor::Config default_motor1_config{
      .num_pole_pairs = 7,
      .phase_resistance = 4.0f,
      .kv_rating = 320,
      .current_limit = 1.0f,
      .foc_type = espp::detail::FocType::SPACE_VECTOR_PWM,
      .driver = motor1_driver_, // NOTE: user cannot override this
      .sensor = encoder1_,      // NOTE: user cannot override this
      .velocity_pid_config =
          {
              .kp = 0.020f,
              .ki = 0.700f,
              .kd = 0.000f,
              .integrator_min = -1.0f, // same scale as output_min (so same scale as current)
              .integrator_max = 1.0f,  // same scale as output_max (so same scale as current)
              .output_min = -1.0, // velocity pid works on current (if we have phase resistance)
              .output_max = 1.0,  // velocity pid works on current (if we have phase resistance)
          },
      .angle_pid_config =
          {
              .kp = 5.000f,
              .ki = 1.000f,
              .kd = 0.000f,
              .integrator_min = -10.0f, // same scale as output_min (so same scale as velocity)
              .integrator_max = 10.0f,  // same scale as output_max (so same scale as velocity)
              .output_min = -20.0,      // angle pid works on velocity (rad/s)
              .output_max = 20.0,       // angle pid works on velocity (rad/s)
          },
      .velocity_filter = [this](float v) { return motor1_velocity_filter_(v); },
      .angle_filter = [this](float a) { return motor1_angle_filter_(a); },
  };

  /// Default configuration for the MotorGo-Mini's BLDC motor on channel 2
  const BldcMotor::Config default_motor2_config{
      .num_pole_pairs = 7,
      .phase_resistance = 4.0f,
      .kv_rating = 320,
      .current_limit = 1.0f,
      .foc_type = espp::detail::FocType::SPACE_VECTOR_PWM,
      .driver = motor2_driver_, // NOTE: user cannot override this
      .sensor = encoder2_,      // NOTE: user cannot override this
      .velocity_pid_config =
          {
              .kp = 0.020f,
              .ki = 0.700f,
              .kd = 0.000f,
              .integrator_min = -1.0f, // same scale as output_min (so same scale as current)
              .integrator_max = 1.0f,  // same scale as output_max (so same scale as current)
              .output_min = -1.0, // velocity pid works on current (if we have phase resistance)
              .output_max = 1.0,  // velocity pid works on current (if we have phase resistance)
          },
      .angle_pid_config =
          {
              .kp = 5.000f,
              .ki = 1.000f,
              .kd = 0.000f,
              .integrator_min = -10.0f, // same scale as output_min (so same scale as velocity)
              .integrator_max = 10.0f,  // same scale as output_max (so same scale as velocity)
              .output_min = -20.0,      // angle pid works on velocity (rad/s)
              .output_max = 20.0,       // angle pid works on velocity (rad/s)
          },
      .velocity_filter = [this](float v) { return motor2_velocity_filter_(v); },
      .angle_filter = [this](float a) { return motor2_angle_filter_(a); },
  };

  /// Initialize the MotorGo-Mini's components for motor channel 1
  /// \details This function initializes the encoder, driver, and motor for
  ///          motor channel 1. This consists of initializing encoder1,
  ///          motor_driver, and motor1.
  /// \param motor_config The motor configuration
  /// \param driver_config The driver configuration
  void init_motor_channel_1(const BldcMotor::Config &motor_config,
                            const DriverConfig &driver_config = {.power_supply_voltage = 5.0f,
                                                                 .limit_voltage = 5.0f});

  /// Initialize the MotorGo-Mini's components for motor channel 2
  /// \details This function initializes the encoder, driver and motor for motor
  ///          channel 2. This consists of initializing encoder2, motor2_driver,
  ///          and motor2.
  /// \param motor_config The motor configuration
  /// \param driver_config The driver configuration
  void init_motor_channel_2(const BldcMotor::Config &motor_config,
                            const DriverConfig &driver_config = {.power_supply_voltage = 5.0f,
                                                                 .limit_voltage = 5.0f});

  /// Get a shared pointer to the motor 1 driver
  /// \return A shared pointer to the motor 1 driver
  std::shared_ptr<espp::BldcDriver> motor1_driver();

  /// Get a shared pointer to the motor 2 driver
  /// \return A shared pointer to the motor 2 driver
  std::shared_ptr<espp::BldcDriver> motor2_driver();

  /// Get a shared pointer to the motor 1
  /// \return A shared pointer to the motor 1
  std::shared_ptr<BldcMotor> motor1();

  /// Get a shared pointer to the motor 2
  /// \return A shared pointer to the motor 2
  std::shared_ptr<BldcMotor> motor2();

  /// Get a reference to the motor 1 velocity filter
  /// \return A reference to the motor 1 velocity filter
  VelocityFilter &motor1_velocity_filter();

  /// Get a reference to the motor 1 angle filter
  /// \return A reference to the motor 1 angle filter
  AngleFilter &motor1_angle_filter();

  /// Get a reference to the motor 2 velocity filter
  /// \return A reference to the motor 2 velocity filter
  VelocityFilter &motor2_velocity_filter();

  /// Get a reference to the motor 2 angle filter
  /// \return A reference to the motor 2 angle filter
  AngleFilter &motor2_angle_filter();

  /////////////////////////////////////////////////////////////////////////////
  // Encoders
  /////////////////////////////////////////////////////////////////////////////

  /// Get a shared pointer to the encoder 1
  /// \return A shared pointer to the encoder 1
  std::shared_ptr<Encoder> encoder1();

  /// Get a shared pointer to the encoder 2
  /// \return A shared pointer to the encoder 2
  std::shared_ptr<Encoder> encoder2();

  /// Reset the encoder 1 accumulator
  /// \details This function resets the encoder 1 accumulator to 0.
  ///          This will reset the encoder's position to be within the range
  ///          of 0 to 2*pi.
  void reset_encoder1_accumulator();

  /// Reset the encoder 2 accumulator
  /// \details This function resets the encoder 2 accumulator to 0.
  ///          This will reset the encoder's position to be within the range
  ///          of 0 to 2*pi.
  void reset_encoder2_accumulator();

  /////////////////////////////////////////////////////////////////////////////
  // Motor Current Sense
  /////////////////////////////////////////////////////////////////////////////

  /// Get the current sense value for motor 1 phase U
  /// \return The current sense value for motor 1 phase U in amps
  float motor1_current_u_amps();

  /// Get the current sense value for motor 1 phase W
  /// \return The current sense value for motor 1 phase W in amps
  float motor1_current_w_amps();

  /// Get the current sense value for motor 2 phase U
  /// \return The current sense value for motor 2 phase U in amps
  float motor2_current_u_amps();

  /// Get the current sense value for motor 2 phase W
  /// \return The current sense value for motor 2 phase W in amps
  float motor2_current_w_amps();

protected:
  static constexpr auto I2C_PORT = I2C_NUM_0;
  static constexpr auto I2C_SDA_PIN = GPIO_NUM_2;
  static constexpr auto I2C_SCL_PIN = GPIO_NUM_1;

  static constexpr uint64_t core_update_period_us = 1000; // 1 ms
  static constexpr auto ENCODER_SPI_HOST = SPI2_HOST;
  static constexpr auto ENCODER_SPI_CLK_SPEED = 10 * 1000 * 1000; // max is 10 MHz
  static constexpr auto ENCODER_SPI_MISO_PIN = GPIO_NUM_35;
  static constexpr auto ENCODER_SPI_SCLK_PIN = GPIO_NUM_36;
  static constexpr auto ENCODER_1_CS_PIN = GPIO_NUM_37;
  static constexpr auto ENCODER_2_CS_PIN = GPIO_NUM_48;

  static constexpr auto MOTOR_1_A_H = GPIO_NUM_18;
  static constexpr auto MOTOR_1_A_L = GPIO_NUM_15;
  static constexpr auto MOTOR_1_B_H = GPIO_NUM_17;
  static constexpr auto MOTOR_1_B_L = GPIO_NUM_5;
  static constexpr auto MOTOR_1_C_H = GPIO_NUM_16;
  static constexpr auto MOTOR_1_C_L = GPIO_NUM_6;

  static constexpr auto MOTOR_2_A_H = GPIO_NUM_9;
  static constexpr auto MOTOR_2_A_L = GPIO_NUM_13;
  static constexpr auto MOTOR_2_B_H = GPIO_NUM_10;
  static constexpr auto MOTOR_2_B_L = GPIO_NUM_21;
  static constexpr auto MOTOR_2_C_H = GPIO_NUM_11;
  static constexpr auto MOTOR_2_C_L = GPIO_NUM_14;

  static constexpr auto BUTTON_GPIO = GPIO_NUM_0;
  static constexpr auto YELLOW_LED_GPIO = GPIO_NUM_38;
  static constexpr auto RED_LED_GPIO = GPIO_NUM_47;

  // TODO: figure this out and update it :)
  static constexpr float CURRENT_SENSE_MV_TO_A = 1.0f;

  /// Constructor
  MotorGoMini();

  void always_init();
  void init_spi();

  float breathe(float breathing_period, uint64_t start_us, bool restart = false);

  bool read_encoder(const auto &encoder_handle, uint8_t *data, size_t size);

  /// I2C bus for external communication
  I2c external_i2c_{{.port = I2C_PORT,
                     .sda_io_num = I2C_SDA_PIN,
                     .scl_io_num = I2C_SCL_PIN,
                     .sda_pullup_en = GPIO_PULLUP_ENABLE,
                     .scl_pullup_en = GPIO_PULLUP_ENABLE}};

  /// SPI bus for communication with the Encoders
  spi_bus_config_t encoder_spi_bus_config_;

  // SPI handles for the encoders
  spi_device_interface_config_t encoder1_config;
  spi_device_handle_t encoder1_handle_;
  spi_device_interface_config_t encoder2_config;
  spi_device_handle_t encoder2_handle_;

  // Encoders
  Encoder::Config encoder1_config_{.read = [this](uint8_t *data, size_t size) -> bool {
                                     return read_encoder(encoder1_handle_, data, size);
                                   },
                                   .update_period =
                                       std::chrono::duration<float>(core_update_period_us / 1e6f),
                                   .log_level = get_log_level()};
  Encoder::Config encoder2_config_{.read = [this](uint8_t *data, size_t size) -> bool {
                                     return read_encoder(encoder2_handle_, data, size);
                                   },
                                   .update_period =
                                       std::chrono::duration<float>(core_update_period_us / 1e6f),
                                   .log_level = get_log_level()};

  // NOTE: use explicit type of nullptr to force allocation of control block, so
  // that it can be shared even if it's nullptr;
  std::shared_ptr<Encoder> encoder1_{(Encoder *)(nullptr)};
  // NOTE: use explicit type of nullptr to force allocation of control block, so
  // that it can be shared even if it's nullptr;
  std::shared_ptr<Encoder> encoder2_{(Encoder *)(nullptr)};

  // Drivers
  espp::BldcDriver::Config motor1_driver_config_{
      .gpio_a_h = MOTOR_1_A_H,
      .gpio_a_l = MOTOR_1_A_L,
      .gpio_b_h = MOTOR_1_B_H,
      .gpio_b_l = MOTOR_1_B_L,
      .gpio_c_h = MOTOR_1_C_H,
      .gpio_c_l = MOTOR_1_C_L,
      .gpio_enable = -1,            // pulled up, not connected
      .gpio_fault = -1,             // not connected
      .power_supply_voltage = 5.0f, // NOTE: can be replaced by user
      .limit_voltage = 5.0f,        // NOTE: can be replaced by user
      .log_level = get_log_level()};
  // NOTE: use explicit type of nullptr to force allocation of control block, so
  // that it can be shared even if it's nullptr;
  std::shared_ptr<espp::BldcDriver> motor1_driver_{(espp::BldcDriver *)(nullptr)};

  espp::BldcDriver::Config motor2_driver_config_{
      .gpio_a_h = MOTOR_2_A_H,
      .gpio_a_l = MOTOR_2_A_L,
      .gpio_b_h = MOTOR_2_B_H,
      .gpio_b_l = MOTOR_2_B_L,
      .gpio_c_h = MOTOR_2_C_H,
      .gpio_c_l = MOTOR_2_C_L,
      .gpio_enable = -1,            // pulled up, not connected
      .gpio_fault = -1,             // not connected
      .power_supply_voltage = 5.0f, // NOTE: can be replaced by user
      .limit_voltage = 5.0f,        // NOTE: can be replaced by user
      .log_level = get_log_level()};
  // NOTE: use explicit type of nullptr to force allocation of control block, so
  // that it can be shared even if it's nullptr;
  std::shared_ptr<espp::BldcDriver> motor2_driver_{(espp::BldcDriver *)(nullptr)};

  // Filters
  VelocityFilter motor1_velocity_filter_{{.time_constant = 0.005f}};
  AngleFilter motor1_angle_filter_{{.time_constant = 0.001f}};
  VelocityFilter motor2_velocity_filter_{{.time_constant = 0.005f}};
  AngleFilter motor2_angle_filter_{{.time_constant = 0.001f}};

  // Motors
  // NOTE: use explicit type of nullptr to force allocation of control block, so
  // that it can be shared even if it's nullptr;
  std::shared_ptr<BldcMotor> motor1_{(BldcMotor *)(nullptr)};
  // NOTE: use explicit type of nullptr to force allocation of control block, so
  // that it can be shared even if it's nullptr;
  std::shared_ptr<BldcMotor> motor2_{(BldcMotor *)(nullptr)};

  // current sense Motor 1 phase U
  espp::AdcConfig current_sense_m1_u_ = {
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_6, // GPIO7
      .attenuation = ADC_ATTEN_DB_12,
  };
  // current sense Motor 1 phase W
  espp::AdcConfig current_sense_m1_w_ = {
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_3, // GPIO4
      .attenuation = ADC_ATTEN_DB_12,
  };
  // current sense Motor 2 phase U - note; the schmatic has the U and W swapped
  espp::AdcConfig current_sense_m2_u_ = {
      .unit = ADC_UNIT_2,
      .channel = ADC_CHANNEL_1, // GPIO12
      .attenuation = ADC_ATTEN_DB_12,
  };
  // current sense Motor 2 phase W - note; the schmatic has the U and W swapped
  espp::AdcConfig current_sense_m2_w_ = {
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_7, // GPIO8
      .attenuation = ADC_ATTEN_DB_12,
  };
  espp::OneshotAdc adc_1{{
      .unit = ADC_UNIT_1,
      .channels = {current_sense_m1_u_, current_sense_m1_w_, current_sense_m2_w_},
  }};
  espp::OneshotAdc adc_2{{
      .unit = ADC_UNIT_2,
      .channels = {current_sense_m2_u_},
  }};

  // Interrupts
  espp::Interrupt::PinConfig button_interrupt_pin_{
      .gpio_num = BUTTON_GPIO,
      .callback =
          [this](const auto &event) {
            if (button_callback_) {
              button_callback_(event);
            }
          },
      .active_level = espp::Interrupt::ActiveLevel::LOW,
      .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
      .pullup_enabled = true};

  // we'll only add each interrupt pin if the initialize method is called
  espp::Interrupt interrupts_{
      {.interrupts = {},
       .task_config = {.name = "motorgo mini interrupts",
                       .stack_size_bytes = CONFIG_MOTORGO_MINI_INTERRUPT_STACK_SIZE}}};

  // button
  std::atomic<bool> button_initialized_{false};
  button_callback_t button_callback_{nullptr};

  // led
  std::vector<espp::Led::ChannelConfig> led_channels_{
      // shown on the board as io38
      {.gpio = (int)YELLOW_LED_GPIO, .channel = LEDC_CHANNEL_0, .timer = LEDC_TIMER_2},
      // shown on the board as io8 >.<
      {.gpio = (int)RED_LED_GPIO, .channel = LEDC_CHANNEL_1, .timer = LEDC_TIMER_2},
  };
  espp::Led led_{espp::Led::Config{
      .timer = LEDC_TIMER_2,
      .frequency_hz = 5000,
      .channels = led_channels_,
      .duty_resolution = LEDC_TIMER_10_BIT,
      .clock_config = LEDC_USE_RC_FAST_CLK, // to support light sleep
  }};
  espp::Gaussian gaussian_{{.gamma = 0.1f, .alpha = 1.0f, .beta = 0.5f}};
  uint64_t io38_breathe_start_us = 0;
  uint64_t io8_breathe_start_us = 0;
  espp::HighResolutionTimer led_timer_{
      {.name = "MotorGo Mini LED Timer",
       .callback = [this]() -> bool {
         led_.set_duty(led_channels_[0].channel, 100.0f * breathe(1.0f, io38_breathe_start_us));
         led_.set_duty(led_channels_[1].channel, 100.0f * breathe(1.0f, io8_breathe_start_us));
         return true;
       },
       .log_level = espp::Logger::Verbosity::WARN}};
};
} // namespace espp

#pragma once

#include <string>
#include <vector>

#include <driver/spi_master.h>

#include "base_component.hpp"
#include "bldc_driver.hpp"
#include "bldc_motor.hpp"
#include "button.hpp"
#include "gaussian.hpp"
#include "i2c.hpp"
#include "led.hpp"
#include "mt6701.hpp"
#include "oneshot_adc.hpp"

namespace espp {
/// This class acts as a board support component for the MotorGo-Mini board.
/// It provides a high-level interface to the board's functionality.
/// The MotorGo-Mini board is a motor driver board that can drive two DC motors.
/// More information about the board can be found at:
/// https://github.com/Every-Flavor-Robotics/motorgo-mini-hardware
///
/// High level overview of the board:
/// - ESP32s3 module
/// - Two BLDC motor drivers (TMC6300)
/// - Two magnetic encoders (connected via two JST-SH 5 pin connectors to EncoderGo boards)
/// (SPI2_HOST)
/// - Two Qwiic connectors for I2C communication (I2C0)
/// - Current sense resistors for each motor on the U and W phases
///   NOTE: on the MotorGo-Mini v1.3, the M2 motor sense is swapped in the
///         schematic, with U and W swapped
///
/// \section motorgo_ex1 MotorGo-Mini Example
/// \snippet motorgo_mini_example.cpp motorgo-mini example
class MotorGoMini : public BaseComponent {
public:
  using Encoder = espp::Mt6701<espp::Mt6701Interface::SSI>;
  using BldcMotor = espp::BldcMotor<espp::BldcDriver, Encoder>;

  explicit MotorGoMini(espp::Logger::Verbosity verbosity = espp::Logger::Verbosity::WARN)
      : BaseComponent("MotorGo-Mini", verbosity) {
    init();
  }

  // I2C
  I2c &get_external_i2c() { return external_i2c_; }

  // Button
  espp::Button &button() { return button_; }

  // LEDs on the board
  espp::Led::ChannelConfig &led_channel0() { return led_channels_[0]; }
  espp::Led::ChannelConfig &led_channel1() { return led_channels_[1]; }
  espp::Led &led() { return led_; }
  espp::Gaussian &gaussian() { return gaussian_; }
  void start_breathing() {
    io38_breathe_start_us = esp_timer_get_time();
    io8_breathe_start_us = esp_timer_get_time();
    static constexpr uint64_t led_timer_period_us = 30 * 1000; // 30 ms
    led_timer_.periodic(led_timer_period_us);
  }
  void stop_breathing() { led_timer_.stop(); }

  // Encoders
  Encoder &encoder1() { return encoder1_; }
  Encoder &encoder2() { return encoder2_; }

  // Motors
  espp::BldcDriver &motor1_driver() { return motor1_driver_; }
  espp::BldcDriver &motor2_driver() { return motor2_driver_; }
  BldcMotor &motor1() { return motor1_; }
  BldcMotor &motor2() { return motor2_; }

  // Current sense
  espp::OneshotAdc &adc1() { return adc_1; }
  espp::OneshotAdc &adc2() { return adc_2; }

protected:
  static constexpr uint64_t core_update_period_us = 1000; // 1 ms
  static constexpr auto ENCODER_SPI_HOST = SPI2_HOST;
  static constexpr auto ENCODER_SPI_CLK_SPEED = 10 * 1000 * 1000; // 10 MHz
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

  void init() {
    start_breathing();
    init_spi();
    init_encoders();
    init_motors();
  }

  void init_spi() {
    // Initialize the SPI bus for the encoders
    memset(&encoder_spi_bus_config_, 0, sizeof(encoder_spi_bus_config_));
    encoder_spi_bus_config_.mosi_io_num = -1;
    encoder_spi_bus_config_.miso_io_num = ENCODER_SPI_MISO_PIN;
    encoder_spi_bus_config_.sclk_io_num = ENCODER_SPI_SCLK_PIN;
    encoder_spi_bus_config_.quadwp_io_num = -1;
    encoder_spi_bus_config_.quadhd_io_num = -1;
    encoder_spi_bus_config_.max_transfer_sz = 100;
    auto err = spi_bus_initialize(ENCODER_SPI_HOST, &encoder_spi_bus_config_, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
      logger_.error("Failed to initialize SPI bus for encoders: {}", esp_err_to_name(err));
      return;
    }

    // Initialize the encoder 1
    memset(&encoder1_config, 0, sizeof(encoder1_config));
    encoder1_config.mode = 0;
    encoder1_config.clock_speed_hz = ENCODER_SPI_CLK_SPEED;
    encoder1_config.queue_size = 7;
    encoder1_config.spics_io_num = ENCODER_1_CS_PIN;
    err = spi_bus_add_device(ENCODER_SPI_HOST, &encoder1_config, &encoder1_handle_);
    if (err != ESP_OK) {
      logger_.error("Failed to initialize Encoder 1: {}", esp_err_to_name(err));
      return;
    }

    // Initialize the encoder 2
    memset(&encoder2_config, 0, sizeof(encoder2_config));
    encoder2_config.mode = 0;
    encoder2_config.clock_speed_hz = ENCODER_SPI_CLK_SPEED;
    encoder2_config.queue_size = 7;
    encoder2_config.spics_io_num = ENCODER_2_CS_PIN;
    err = spi_bus_add_device(ENCODER_SPI_HOST, &encoder2_config, &encoder2_handle_);
    if (err != ESP_OK) {
      logger_.error("Failed to initialize Encoder 2: {}", esp_err_to_name(err));
      return;
    }
  }

  void init_encoders() {
    bool run_task = false;
    std::error_code ec;
    encoder1_.initialize(run_task, ec);
    if (ec) {
      logger_.error("Could not initialize encoder1: {}", ec.message());
    }
    ec.clear();
    encoder2_.initialize(run_task, ec);
    if (ec) {
      logger_.error("Could not initialize encoder2: {}", ec.message());
    }
    ec.clear();
  }

  void init_motors() {
    motor1_.initialize();
    motor2_.initialize();
  }

  float breathe(float breathing_period, uint64_t start_us, bool restart = false) {
    auto now_us = esp_timer_get_time();
    if (restart) {
      start_us = now_us;
    }
    auto elapsed_us = now_us - start_us;
    float elapsed = elapsed_us / 1e6f;
    float t = std::fmod(elapsed, breathing_period) / breathing_period;
    return gaussian_(t);
  }

  bool IRAM_ATTR read_encoder(const auto &encoder_handle, uint8_t *data, size_t size) {
    static constexpr uint8_t SPIBUS_READ = 0x80;
    spi_transaction_t t = {
        .flags = 0,
        .cmd = 0,
        .addr = SPIBUS_READ,
        .length = size * 8,
        .rxlength = size * 8,
        .user = nullptr,
        .tx_buffer = nullptr,
        .rx_buffer = data,
    };
    if (size <= 4) {
      t.flags = SPI_TRANS_USE_RXDATA;
      t.rx_buffer = nullptr;
    }
    esp_err_t err = spi_device_polling_transmit(encoder_handle, &t);
    if (err != ESP_OK) {
      return false;
    }
    if (size <= 4) {
      // copy the data from the rx_data field
      for (size_t i = 0; i < size; i++) {
        data[i] = t.rx_data[i];
      }
    }
    return true;
  }

  /// I2C bus for external communication
  I2c external_i2c_{{.port = I2C_NUM_0,
                     .sda_io_num = GPIO_NUM_2,
                     .scl_io_num = GPIO_NUM_1,
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
  Encoder encoder1_{
      {.read = [this](uint8_t *data, size_t size) -> bool {
         return read_encoder(encoder1_handle_, data, size);
       },
       .velocity_filter = nullptr,
       .update_period = std::chrono::duration<float>(core_update_period_us / 1e6f),
       .auto_init = false, // we have to initialize the SPI first before we can use the encoder
       .run_task = false,  // we will manually call update
       .log_level = get_log_level()}};
  Encoder encoder2_{
      {.read = [this](uint8_t *data, size_t size) -> bool {
         return read_encoder(encoder2_handle_, data, size);
       },
       .velocity_filter = nullptr,
       .update_period = std::chrono::duration<float>(core_update_period_us / 1e6f),
       .auto_init = false, // we have to initialize the SPI first before we can use the encoder
       .run_task = false,  // we will manually call update
       .log_level = get_log_level()}};

  // Drivers
  espp::BldcDriver motor1_driver_{{.gpio_a_h = MOTOR_1_A_H,
                                   .gpio_a_l = MOTOR_1_A_L,
                                   .gpio_b_h = MOTOR_1_B_H,
                                   .gpio_b_l = MOTOR_1_B_L,
                                   .gpio_c_h = MOTOR_1_C_H,
                                   .gpio_c_l = MOTOR_1_C_L,
                                   .gpio_enable = -1, // pulled up, not connected
                                   .gpio_fault = -1,  // not connected
                                   .power_supply_voltage = 5.0f,
                                   .limit_voltage = 5.0f,
                                   .log_level = get_log_level()}};
  espp::BldcDriver motor2_driver_{{.gpio_a_h = MOTOR_2_A_H,
                                   .gpio_a_l = MOTOR_2_A_L,
                                   .gpio_b_h = MOTOR_2_B_H,
                                   .gpio_b_l = MOTOR_2_B_L,
                                   .gpio_c_h = MOTOR_2_C_H,
                                   .gpio_c_l = MOTOR_2_C_L,
                                   .gpio_enable = -1, // pulled up, not connected
                                   .gpio_fault = -1,  // not connected
                                   .power_supply_voltage = 5.0f,
                                   .limit_voltage = 5.0f,
                                   .log_level = get_log_level()}};

  // Motors
  BldcMotor motor1_{{
      .num_pole_pairs = 7,
      .phase_resistance = 5.0f,
      .kv_rating = 320,
      .current_limit = 1.0f,
      .foc_type = espp::detail::FocType::SPACE_VECTOR_PWM,
      .driver = std::shared_ptr<espp::BldcDriver>(&motor1_driver_),
      .sensor = std::shared_ptr<Encoder>(&encoder1_),
      .velocity_pid_config =
          {
              .kp = 0.010f,
              .ki = 1.000f,
              .kd = 0.000f,
              .integrator_min = -1.0f, // same scale as output_min (so same scale as current)
              .integrator_max = 1.0f,  // same scale as output_max (so same scale as current)
              .output_min = -1.0, // velocity pid works on current (if we have phase resistance)
              .output_max = 1.0,  // velocity pid works on current (if we have phase resistance)
          },
      .angle_pid_config =
          {
              .kp = 7.000f,
              .ki = 0.300f,
              .kd = 0.010f,
              .integrator_min = -10.0f, // same scale as output_min (so same scale as velocity)
              .integrator_max = 10.0f,  // same scale as output_max (so same scale as velocity)
              .output_min = -20.0,      // angle pid works on velocity (rad/s)
              .output_max = 20.0,       // angle pid works on velocity (rad/s)
          },
      .auto_init = false, // we have to initialize the SPI first before we can use the encoder
      .log_level = get_log_level(),
  }};
  BldcMotor motor2_{{
      .num_pole_pairs = 7,
      .phase_resistance = 5.0f,
      .kv_rating = 320,
      .current_limit = 1.0f,
      .foc_type = espp::detail::FocType::SPACE_VECTOR_PWM,
      .driver = std::shared_ptr<espp::BldcDriver>(&motor2_driver_),
      .sensor = std::shared_ptr<Encoder>(&encoder2_),
      .velocity_pid_config =
          {
              .kp = 0.010f,
              .ki = 1.000f,
              .kd = 0.000f,
              .integrator_min = -1.0f, // same scale as output_min (so same scale as current)
              .integrator_max = 1.0f,  // same scale as output_max (so same scale as current)
              .output_min = -1.0, // velocity pid works on current (if we have phase resistance)
              .output_max = 1.0,  // velocity pid works on current (if we have phase resistance)
          },
      .angle_pid_config =
          {
              .kp = 7.000f,
              .ki = 0.300f,
              .kd = 0.010f,
              .integrator_min = -10.0f, // same scale as output_min (so same scale as velocity)
              .integrator_max = 10.0f,  // same scale as output_max (so same scale as velocity)
              .output_min = -20.0,      // angle pid works on velocity (rad/s)
              .output_max = 20.0,       // angle pid works on velocity (rad/s)
          },
      .auto_init = false, // we have to initialize the SPI first before we can use the encoder
      .log_level = get_log_level(),
  }};

  // current sense Motor 1 phase U
  espp::AdcConfig current_sense_m1_u_ = {
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_6,
      .attenuation = ADC_ATTEN_DB_12,
  };
  // current sense Motor 1 phase W
  espp::AdcConfig current_sense_m1_w_ = {
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_3,
      .attenuation = ADC_ATTEN_DB_12,
  };
  // current sense Motor 2 phase U - note; the schmatic has the U and W swapped
  espp::AdcConfig current_sense_m2_u_ = {
      .unit = ADC_UNIT_2,
      .channel = ADC_CHANNEL_1,
      .attenuation = ADC_ATTEN_DB_12,
  };
  // current sense Motor 2 phase W - note; the schmatic has the U and W swapped
  espp::AdcConfig current_sense_m2_w_ = {
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_7,
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

  // button
  espp::Button button_{{
      .name = "MotorGo Mini Button",
      .gpio_num = GPIO_NUM_0,
      .active_level = espp::Interrupt::ActiveLevel::LOW,
      .pullup_enabled = false,
      .pulldown_enabled = false,
      .log_level = espp::Logger::Verbosity::WARN,
  }};

  // led
  std::vector<espp::Led::ChannelConfig> led_channels_{
      // shown on the board as io38
      {.gpio = 38, .channel = LEDC_CHANNEL_0, .timer = LEDC_TIMER_2},
      // shown on the board as io8 >.<
      {.gpio = 47, .channel = LEDC_CHANNEL_1, .timer = LEDC_TIMER_2},
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

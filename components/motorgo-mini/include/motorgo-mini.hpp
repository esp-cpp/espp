#pragma once

#include <string>
#include <vector>

#include <driver/spi_master.h>

#include "base_component.hpp"
#include "bldc_driver.hpp"
#include "bldc_motor.hpp"
#include "i2c.hpp"
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
class MotorGoMini : public BaseComponent {
public:
  using Encoder = espp::Mt6701<espp::Mt6701Interface::SSI>;
  using BldcMotor = espp::BldcMotor<espp::BldcDriver, Encoder>;

  MotorGoMini()
      : BaseComponent("MotorGo-Mini") {
    init();
  }

  I2c &get_external_i2c() { return external_i2c_; }

  Encoder &get_encoder1() { return encoder1_; }
  Encoder &get_encoder2() { return encoder2_; }

protected:
  static constexpr auto ENCODER_1_CS_PIN = GPIO_NUM_37;
  static constexpr auto ENCODER_2_CS_PIN = GPIO_NUM_48;
  static constexpr auto ENCODER_SPI_HOST = SPI2_HOST;

  void init() {
    init_encoders();
    init_motors();
  }

  void init_encoders() {
    // Initialize the SPI bus for the encoders
    auto err = spi_bus_initialize(ENCODER_SPI_HOST, &encoder_spi_bus_config_, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
      logger_.error("Failed to initialize SPI bus for encoders: {}", esp_err_to_name(err));
      return;
    }

    spi_device_interface_config_t encoder_config;
    memset(&encoder_config, 0, sizeof(encoder_config));
    encoder_config.command_bits = 0;
    encoder_config.address_bits = 0;
    encoder_config.mode = 0;
    encoder_config.clock_speed_hz = 10000000; // 10 MHz

    // Initialize the encoder 1
    encoder_config.spics_io_num = ENCODER_1_CS_PIN;
    err = spi_bus_add_device(ENCODER_SPI_HOST, &encoder_config, &encoder1_handle_);
    if (err != ESP_OK) {
      logger_.error("Failed to initialize Encoder 1: {}", esp_err_to_name(err));
      return;
    }

    // Initialize the encoder 2
    encoder_config.spics_io_num = ENCODER_2_CS_PIN;
    err = spi_bus_add_device(ENCODER_SPI_HOST, &encoder_config, &encoder2_handle_);
    if (err != ESP_OK) {
      logger_.error("Failed to initialize Encoder 2: {}", esp_err_to_name(err));
      return;
    }
  }

  void init_motors() {}

  /// I2C bus for external communication
  I2c external_i2c_{{.port = I2C_NUM_0,
                     .sda_io_num = GPIO_NUM_2,
                     .scl_io_num = GPIO_NUM_1,
                     .sda_pullup_en = GPIO_PULLUP_ENABLE,
                     .scl_pullup_en = GPIO_PULLUP_ENABLE}};

  /// SPI bus for communication with the Encoders
  spi_bus_config_t encoder_spi_bus_config_ = {
      .mosi_io_num = -1,
      .miso_io_num = GPIO_NUM_35,
      .sclk_io_num = GPIO_NUM_36,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 32,
  };
  spi_device_handle_t encoder1_handle_;
  spi_device_handle_t encoder2_handle_;

  Encoder encoder1_{{.read = [&](uint8_t *data, size_t size) -> bool { return false; },
                     .velocity_filter = nullptr,
                     .update_period = std::chrono::duration<float>(0.01f),
                     .log_level = espp::Logger::Verbosity::WARN}};
  Encoder encoder2_{{.read = [&](uint8_t *data, size_t size) -> bool { return false; },
                     .velocity_filter = nullptr,
                     .update_period = std::chrono::duration<float>(0.01f),
                     .log_level = espp::Logger::Verbosity::WARN}};

  /// Driver for motor 1
  espp::BldcDriver motor1_driver_{{
      .gpio_a_h = 18,
      .gpio_a_l = 15,
      .gpio_b_h = 17,
      .gpio_b_l = 5,
      .gpio_c_h = 16,
      .gpio_c_l = 6,
      .gpio_enable = -1, // pulled up, not connected
      .gpio_fault = -1,  // not connected
      .power_supply_voltage = 5.0f,
      .limit_voltage = 5.0f,
  }};
  /// Driver for motor 2
  espp::BldcDriver motor2_driver_{{
      .gpio_a_h = 9,
      .gpio_a_l = 13,
      .gpio_b_h = 10,
      .gpio_b_l = 21,
      .gpio_c_h = 11,
      .gpio_c_l = 14,
      .gpio_enable = -1, // pulled up, not connected
      .gpio_fault = -1,  // not connected
      .power_supply_voltage = 5.0f,
      .limit_voltage = 5.0f,
  }};
  // current sense m1 U
  espp::AdcConfig current_sense_m1_u_ = {
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_6,
      .attenuation = ADC_ATTEN_DB_11,
  };
  // current sense m1 W
  espp::AdcConfig current_sense_m1_w_ = {
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_3,
      .attenuation = ADC_ATTEN_DB_11,
  };
  // current sense m2 U - note; the schmatic has the U and W swapped
  espp::AdcConfig current_sense_m2_u_ = {
      .unit = ADC_UNIT_2,
      .channel = ADC_CHANNEL_1,
      .attenuation = ADC_ATTEN_DB_11,
  };
  // current sense m2 W - note; the schmatic has the U and W swapped
  espp::AdcConfig current_sense_m2_w_ = {
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_7,
      .attenuation = ADC_ATTEN_DB_11,
  };
  espp::OneshotAdc adc_1{{
      .unit = ADC_UNIT_1,
      .channels = {current_sense_m1_u_, current_sense_m1_w_, current_sense_m2_w_},
  }};
  espp::OneshotAdc adc_2{{
      .unit = ADC_UNIT_2,
      .channels = {current_sense_m2_u_},
  }};
};
} // namespace espp

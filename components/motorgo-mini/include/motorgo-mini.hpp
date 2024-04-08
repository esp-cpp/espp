#pragma once

#include <string>
#include <vector>

#include "base_component.hpp"
#include "i2c.hpp"
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
/// - Two magnetic encoders (connected via two JST-SH 5 pin connectors to EncoderGo boards) (I2C0)
/// - Two Qwiic connectors for I2C communication (I2C1)
/// - Current sense resistors for each motor on the U and W phases
///   NOTE: on the MotorGo-Mini v1.3, the M2 motor sense is swapped in the
///         schematic, with U and W swapped
class MotorGoMini : public BaseComponent {
protected:
  /// I2C bus for communication with the Encoders
  I2C encoder_i2c_{{.sda = GPIO_NUM_35,
                    .scl = GPIO_NUM_36,
                    .port = I2C_NUM_0,
                    .mode = I2C_MODE_MASTER,
                    .sda_pullup_en = GPIO_PULLUP_ENABLE,
                    .scl_pullup_en = GPIO_PULLUP_ENABLE,
                    .clk_speed = 400000}};
  /// I2C bus for external communication
  I2C external_i2c_{{.sda = GPIO_NUM_2,
                     .scl = GPIO_NUM_1,
                     .port = I2C_NUM_1,
                     .mode = I2C_MODE_MASTER,
                     .sda_pullup_en = GPIO_PULLUP_ENABLE,
                     .scl_pullup_en = GPIO_PULLUP_ENABLE,
                     .clk_speed = 400000}};
  /// Driver for motor 1
  espp::BldcDriver motor1_{{
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
  espp::BldcDriver motor2_{{
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
      .channel = ADC1_CHANNEL_6,
      .attenuation = ADC_ATTEN_DB_11,
  };
  // current sense m1 W
  espp::AdcConfig current_sense_m1_w_ = {
      .unit = ADC_UNIT_1,
      .channel = ADC1_CHANNEL_3,
      .attenuation = ADC_ATTEN_DB_11,
  };
  // current sense m2 U - note; the schmatic has the U and W swapped
  espp::AdcConfig current_sense_m2_u_ = {
      .unit = ADC_UNIT_2,
      .channel = ADC2_CHANNEL_1,
      .attenuation = ADC_ATTEN_DB_11,
  };
  // current sense m2 W - note; the schmatic has the U and W swapped
  espp::AdcConfig current_sense_m2_w_ = {
      .unit = ADC_UNIT_1,
      .channel = ADC1_CHANNEL_7,
      .attenuation = ADC_ATTEN_DB_11,
  };
  espp::OneshotAdc adc_1({
      .unit = ADC_UNIT_1,
      .channels = {current_sense_m1_u_, current_sense_m1_w_, current_sense_m2_w_},
  });
  espp::OneshotAdc adc_2({
      .unit = ADC_UNIT_2,
      .channels = {current_sense_m2_u_},
  });
};
} // namespace espp

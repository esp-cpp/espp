#include <chrono>
#include <vector>

#include "driver/i2c.h"


#include "bldc_driver.hpp"
#include "bldc_motor.hpp"
#include "butterworth_filter.hpp"
#include "mt6701.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

#define I2C_NUM         (I2C_NUM_1)
#define I2C_SCL_IO      (GPIO_NUM_40)
#define I2C_SDA_IO      (GPIO_NUM_41)
#define I2C_FREQ_HZ     (400 * 1000)
#define I2C_TIMEOUT_MS  (10)

extern "C" void app_main(void) {
  constexpr int num_seconds_to_run = 60;
  {
    fmt::print("Running BLDC Motor (FOC) example for {} seconds!\n", num_seconds_to_run);
    //! [bldc motor example]
    // make the I2C that we'll use to communicate with the mt6701 (magnetic encoder)
    i2c_config_t i2c_cfg;
    fmt::print("initializing i2c driver...\n");
    memset(&i2c_cfg, 0, sizeof(i2c_cfg));
    i2c_cfg.sda_io_num = I2C_SDA_IO;
    i2c_cfg.scl_io_num = I2C_SCL_IO;
    i2c_cfg.mode = I2C_MODE_MASTER;
    i2c_cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_cfg.master.clk_speed = I2C_FREQ_HZ;
    auto err = i2c_param_config(I2C_NUM, &i2c_cfg);
    if (err != ESP_OK) printf("config i2c failed\n");
    err = i2c_driver_install(I2C_NUM, I2C_MODE_MASTER,  0, 0, 0);
    if (err != ESP_OK) printf("install i2c driver failed\n");
    // make some lambda functions we'll use to read/write to the mt6701
    auto mt6701_write = [](uint8_t reg_addr, uint8_t value) {
      uint8_t data[] = {reg_addr, value};
      i2c_master_write_to_device(I2C_NUM,
                                 espp::Mt6701::ADDRESS,
                                 data,
                                 2,
                                 I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    };

    auto mt6701_read = [](uint8_t reg_addr) -> uint8_t{
      uint8_t data;
      i2c_master_write_read_device(I2C_NUM,
                                   espp::Mt6701::ADDRESS,
                                   &reg_addr,
                                   1,
                                   &data,
                                   1,
                                   I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
      return data;
    };
    // make the velocity filter
    static constexpr float core_update_period = 0.01f; // seconds
    static constexpr float filter_cutoff_hz = 4.0f;
    espp::ButterworthFilter<2, espp::BiquadFilterDf2> filter({
        .normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * core_update_period
      });
    auto filter_fn = [&filter](float raw) -> float {
      return filter.update(raw);
    };
    // now make the mt6701 which decodes the data
    std::shared_ptr<espp::Mt6701> mt6701 = std::make_shared<espp::Mt6701>(espp::Mt6701::Config{
        .write = mt6701_write,
        .read = mt6701_read,
        .velocity_filter = filter_fn,
        .update_period = std::chrono::duration<float>(core_update_period),
        .log_level = espp::Logger::Verbosity::WARN
      });
    // now make the bldc driver
    std::shared_ptr<espp::BldcDriver> driver = std::make_shared<espp::BldcDriver>(espp::BldcDriver::Config{
        .gpio_a_h = 9,
        .gpio_a_l = 43,
        .gpio_b_h = 44,
        .gpio_b_l = 14,
        .gpio_c_h = 38,
        .gpio_c_l = 39,
        .gpio_enable = 42, // connected to the VIO/~Stdby pin of TMC6300-BOB
        .power_supply_voltage = 5.0f,
        .limit_voltage = 2.5f,
        .log_level = espp::Logger::Verbosity::WARN
      });
    // now make the bldc motor
    using BldcMotor = espp::BldcMotor<espp::BldcDriver, espp::Mt6701>;
    auto motor = BldcMotor(BldcMotor::Config{
        // measured by setting it into ANGLE_OPENLOOP and then counting how many
        // spots you feel when rotating it.
        .num_pole_pairs = 7,
        .phase_resistance = 5.0f, // not sure if this is right (guess?)
        .kv_rating = 320.0f, // not sure if this is right (from website but what are units)
        .driver = driver,
        .sensor = mt6701,
        .log_level = espp::Logger::Verbosity::WARN
      });
    motor.set_motion_control_type(espp::MotionControlType::VELOCITY_OPENLOOP);
    std::atomic<float> target_velocity = 0.0f;
    // motor.set_motion_control_type(espp::MotionControlType::ANGLE_OPENLOOP);
    auto motor_task_fn = [&motor, &target_velocity](std::mutex& m, std::condition_variable& cv) {
      static constexpr float max_velocity = 250.0f * espp::RPM_TO_RADS;
      static constexpr float velocity_delta = 50.0f * espp::RPM_TO_RADS * core_update_period;
      static auto delay = std::chrono::duration<float>(core_update_period);
      static bool incrementing = true;
      auto start = std::chrono::high_resolution_clock::now();
      // update target velocity
      if (incrementing) {
        target_velocity += velocity_delta;
        if (target_velocity > max_velocity) {
          incrementing = false;
          target_velocity -= velocity_delta;
        }
      } else {
        target_velocity -= velocity_delta;
        if (target_velocity < -max_velocity) {
          incrementing = true;
          target_velocity += velocity_delta;
        }
      }
      // command the motor
      motor.move(target_velocity);
      // motor.move(100); // target angle
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_until(lk, start + delay);
      }
    };
    auto motor_task = espp::Task({
        .name = "Motor Task",
        .callback = motor_task_fn,
        .stack_size_bytes = 5*1024,
        .priority = 20,
        .core_id = 1,
        .log_level = espp::Logger::Verbosity::WARN
      });
    motor_task.start();

    // and finally, make the task to periodically poll the mt6701 and print the
    // state. NOTE: the Mt6701 runs its own task to maintain state, so we're
    // just polling the current state.
    auto task_fn = [&mt6701, &target_velocity](std::mutex& m, std::condition_variable& cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto seconds = std::chrono::duration<float>(now-start).count();
      auto count = mt6701->get_count();
      auto radians = mt6701->get_radians();
      auto degrees = mt6701->get_degrees();
      auto rpm = mt6701->get_rpm();
      fmt::print("{:.3f}, {}, {:.3f}, {:.3f}, {:.3f}, {:.3f}\n",
                 seconds,
                 count,
                 radians,
                 degrees,
                 target_velocity.load() / espp::RPM_TO_RADS,
                 rpm
                 );
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 50ms);
      }
    };
    auto task = espp::Task({
        .name = "Encoder Task",
        .callback = task_fn,
        .stack_size_bytes = 5*1024,
        .log_level = espp::Logger::Verbosity::WARN
      });
    fmt::print("%time(s), count, radians, degrees, target velocity (rpm), actual speed (rpm)\n");
    task.start();
    //! [bldc_motor example]
    static auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    auto seconds = std::chrono::duration<float>(now-start).count();
    while (seconds < num_seconds_to_run) {
      now = std::chrono::high_resolution_clock::now();
      seconds = std::chrono::duration<float>(now-start).count();
      std::this_thread::sleep_for(100ms);
    }
  }
  // now clean up the i2c driver
  i2c_driver_delete(I2C_NUM);

  fmt::print("BLDC Motor (FOC) example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

#include "motorgo-mini.hpp"

using namespace espp;

MotorGoMini::MotorGoMini()
    : BaseComponent("MotorGo-Mini") {
  always_init();
}

I2c &MotorGoMini::get_external_i2c() { return external_i2c_; }

espp::Interrupt &MotorGoMini::interrupts() { return interrupts_; }

espp::Led::ChannelConfig &MotorGoMini::yellow_led() { return led_channels_[0]; }

espp::Led::ChannelConfig &MotorGoMini::led_channel0() { return led_channels_[0]; }

void MotorGoMini::set_yellow_led_duty(float duty) { led_.set_duty(led_channels_[0].channel, duty); }

espp::Led::ChannelConfig &MotorGoMini::red_led() { return led_channels_[1]; }

espp::Led::ChannelConfig &MotorGoMini::led_channel1() { return led_channels_[1]; }

void MotorGoMini::set_red_led_duty(float duty) { led_.set_duty(led_channels_[1].channel, duty); }

espp::Led &MotorGoMini::led() { return led_; }

espp::Gaussian &MotorGoMini::gaussian() { return gaussian_; }

void MotorGoMini::start_breathing() {
  io38_breathe_start_us = esp_timer_get_time();
  io8_breathe_start_us = esp_timer_get_time();
  static constexpr uint64_t led_timer_period_us = 30 * 1000; // 30 ms
  led_timer_.periodic(led_timer_period_us);
}

void MotorGoMini::stop_breathing() {
  led_timer_.stop();
  led_.set_duty(led_channels_[0].channel, 0.0f);
  led_.set_duty(led_channels_[1].channel, 0.0f);
}

void MotorGoMini::init_motor_channel_1(const MotorGoMini::BldcMotor::Config &motor_config,
                                       const MotorGoMini::DriverConfig &driver_config) {
  bool run_task = true;
  std::error_code ec;
  // make the encoder
  encoder1_ = std::make_shared<Encoder>(encoder1_config_);
  // initialize the encoder
  encoder1_->initialize(run_task, ec);
  if (ec) {
    logger_.error("Could not initialize encoder1: {}", ec.message());
    return;
  }

  // copy the config data for the driver
  motor1_driver_config_.power_supply_voltage = driver_config.power_supply_voltage;
  motor1_driver_config_.limit_voltage = driver_config.limit_voltage;
  // make the driver
  motor1_driver_ = std::make_shared<BldcDriver>(motor1_driver_config_);

  // now copy the relevant configs into the motor config
  auto motor1_config = motor_config;
  motor1_config.driver = motor1_driver_;
  motor1_config.sensor = encoder1_;
  // now make the motor
  motor1_ = std::make_shared<BldcMotor>(motor1_config);
  motor1_->initialize();
}

void MotorGoMini::init_motor_channel_2(const MotorGoMini::BldcMotor::Config &motor_config,
                                       const MotorGoMini::DriverConfig &driver_config) {
  bool run_task = true;
  std::error_code ec;
  // make the encoder
  encoder2_ = std::make_shared<Encoder>(encoder2_config_);
  // initialize the encoder
  encoder2_->initialize(run_task, ec);
  if (ec) {
    logger_.error("Could not initialize encoder2: {}", ec.message());
    return;
  }

  // copy the config data for the driver
  motor2_driver_config_.power_supply_voltage = driver_config.power_supply_voltage;
  motor2_driver_config_.limit_voltage = driver_config.limit_voltage;
  // make the driver
  motor2_driver_ = std::make_shared<BldcDriver>(motor2_driver_config_);

  // now copy the relevant configs into the motor config
  auto motor2_config = motor_config;
  motor2_config.driver = motor2_driver_;
  motor2_config.sensor = encoder2_;
  // now make the motor
  motor2_ = std::make_shared<BldcMotor>(motor2_config);
  motor2_->initialize();
}

std::shared_ptr<MotorGoMini::Encoder> MotorGoMini::encoder1() { return encoder1_; }

std::shared_ptr<MotorGoMini::Encoder> MotorGoMini::encoder2() { return encoder2_; }

void MotorGoMini::reset_encoder1_accumulator() { encoder1_->reset_accumulator(); }

void MotorGoMini::reset_encoder2_accumulator() { encoder2_->reset_accumulator(); }

std::shared_ptr<espp::BldcDriver> MotorGoMini::motor1_driver() { return motor1_driver_; }

std::shared_ptr<espp::BldcDriver> MotorGoMini::motor2_driver() { return motor2_driver_; }

std::shared_ptr<MotorGoMini::BldcMotor> MotorGoMini::motor1() { return motor1_; }

std::shared_ptr<MotorGoMini::BldcMotor> MotorGoMini::motor2() { return motor2_; }

MotorGoMini::VelocityFilter &MotorGoMini::motor1_velocity_filter() {
  return motor1_velocity_filter_;
}

MotorGoMini::AngleFilter &MotorGoMini::motor1_angle_filter() { return motor1_angle_filter_; }

MotorGoMini::VelocityFilter &MotorGoMini::motor2_velocity_filter() {
  return motor2_velocity_filter_;
}

MotorGoMini::AngleFilter &MotorGoMini::motor2_angle_filter() { return motor2_angle_filter_; }

espp::OneshotAdc &MotorGoMini::adc1() { return adc_1; }

espp::OneshotAdc &MotorGoMini::adc2() { return adc_2; }

float MotorGoMini::motor1_current_u_amps() {
  return adc_1.read_mv(current_sense_m1_u_).value() * CURRENT_SENSE_MV_TO_A;
}

float MotorGoMini::motor1_current_w_amps() {
  return adc_1.read_mv(current_sense_m1_w_).value() * CURRENT_SENSE_MV_TO_A;
}

float MotorGoMini::motor2_current_u_amps() {
  return adc_2.read_mv(current_sense_m2_u_).value() * CURRENT_SENSE_MV_TO_A;
}

float MotorGoMini::motor2_current_w_amps() {
  return adc_1.read_mv(current_sense_m2_w_).value() * CURRENT_SENSE_MV_TO_A;
}

/////////////////////////////////////////////////////////////////////////////
// Symmetric (index-based) API
/////////////////////////////////////////////////////////////////////////////

bool MotorGoMini::init_encoder(size_t index, bool run_tasks) {
  std::error_code ec;
  if (index == 0) {
    if (!encoder1_) {
      encoder1_ = std::make_shared<Encoder>(encoder1_config_);
      encoder1_->initialize(run_tasks, ec);
    }
  } else if (index == 1) {
    if (!encoder2_) {
      encoder2_ = std::make_shared<Encoder>(encoder2_config_);
      encoder2_->initialize(run_tasks, ec);
    }
  } else {
    logger_.error("Invalid encoder index: {}", index);
    return false;
  }
  if (ec) {
    logger_.error("Could not initialize encoder {}: {}", index + 1, ec.message());
    return false;
  }
  return true;
}

bool MotorGoMini::init_driver(size_t index, float power_supply_voltage, float limit_voltage,
                              uint64_t dead_zone_ns) {
  if (index == 0) {
    if (!motor1_driver_) {
      motor1_driver_config_.power_supply_voltage = power_supply_voltage;
      motor1_driver_config_.limit_voltage = limit_voltage;
      motor1_driver_config_.dead_zone_ns = dead_zone_ns;
      motor1_driver_ = std::make_shared<BldcDriver>(motor1_driver_config_);
    }
  } else if (index == 1) {
    if (!motor2_driver_) {
      motor2_driver_config_.power_supply_voltage = power_supply_voltage;
      motor2_driver_config_.limit_voltage = limit_voltage;
      motor2_driver_config_.dead_zone_ns = dead_zone_ns;
      motor2_driver_ = std::make_shared<BldcDriver>(motor2_driver_config_);
    }
  } else {
    logger_.error("Invalid motor index: {}", index);
    return false;
  }
  return true;
}

bool MotorGoMini::initialize_encoders(bool run_tasks) {
  bool ok = true;
  ok &= init_encoder(0, run_tasks);
  ok &= init_encoder(1, run_tasks);
  return ok;
}

bool MotorGoMini::initialize_motors(float power_supply_voltage, float limit_voltage,
                                    uint64_t dead_zone_ns) {
  bool ok = true;
  ok &= init_driver(0, power_supply_voltage, limit_voltage, dead_zone_ns);
  ok &= init_driver(1, power_supply_voltage, limit_voltage, dead_zone_ns);
  return ok;
}

std::shared_ptr<MotorGoMini::MotorDriver> MotorGoMini::motor_driver(size_t index) {
  if (index == 0)
    return motor1_driver_;
  if (index == 1)
    return motor2_driver_;
  logger_.error("Invalid motor index: {}", index);
  return nullptr;
}

std::shared_ptr<MotorGoMini::Encoder> MotorGoMini::encoder(size_t index) {
  if (index == 0)
    return encoder1_;
  if (index == 1)
    return encoder2_;
  logger_.error("Invalid encoder index: {}", index);
  return nullptr;
}

void MotorGoMini::reset_encoder_accumulator(size_t index) {
  auto enc = encoder(index);
  if (!enc) {
    logger_.error("Encoder {} not initialized", index + 1);
    return;
  }
  enc->reset_accumulator();
}

bool MotorGoMini::motor_driver_enabled(size_t index) {
  auto drv = motor_driver(index);
  return drv && drv->is_enabled();
}

bool MotorGoMini::enable_motor_driver(size_t index) {
  auto drv = motor_driver(index);
  if (!drv) {
    logger_.error("Motor driver {} not initialized", index + 1);
    return false;
  }
  drv->enable();
  return drv->is_enabled();
}

void MotorGoMini::disable_motor_driver(size_t index) {
  auto drv = motor_driver(index);
  if (!drv) {
    logger_.error("Motor driver {} not initialized", index + 1);
    return;
  }
  drv->disable();
}

void MotorGoMini::enable_all_motor_drivers() {
  for (size_t i = 0; i < num_motor_channels(); i++) {
    auto drv = motor_driver(i);
    if (drv) {
      drv->enable();
    }
  }
}

void MotorGoMini::disable_all_motor_drivers() {
  for (size_t i = 0; i < num_motor_channels(); i++) {
    auto drv = motor_driver(i);
    if (drv) {
      drv->disable();
    }
  }
}

MotorGoMini::BldcMotor::Config MotorGoMini::default_motor_config(size_t index) const {
  if (index == 0)
    return default_motor1_config;
  if (index == 1)
    return default_motor2_config;
  logger_.error("Invalid motor index: {}", index);
  return default_motor1_config;
}

std::shared_ptr<MotorGoMini::BldcMotor>
MotorGoMini::initialize_motor(size_t index, const MotorGoMini::BldcMotor::Config &config) {
  if (index >= num_motor_channels()) {
    logger_.error("Invalid motor index: {}", index);
    return nullptr;
  }
  // make sure the encoder and driver for this channel exist; initialize the
  // board defaults if the user has not done so already.
  if (!encoder(index)) {
    init_encoder(index, true);
  }
  if (!motor_driver(index)) {
    init_driver(index, driver_default_power_supply_voltage(), driver_default_voltage_limit(),
                default_motor_dead_zone_ns());
  }
  auto enc = encoder(index);
  auto drv = motor_driver(index);
  if (!enc || !drv) {
    logger_.error("Could not initialize driver / encoder for motor {}", index + 1);
    return nullptr;
  }
  // the board owns the driver + encoder, so override those fields regardless of
  // what the caller passed.
  auto motor_config = config;
  motor_config.driver = drv;
  motor_config.sensor = enc;
  auto m = std::make_shared<BldcMotor>(motor_config);
  if (index == 0) {
    motor1_ = m;
  } else {
    motor2_ = m;
  }
  return m;
}

std::shared_ptr<MotorGoMini::BldcMotor> MotorGoMini::motor(size_t index) {
  if (index == 0)
    return motor1_;
  if (index == 1)
    return motor2_;
  logger_.error("Invalid motor index: {}", index);
  return nullptr;
}

void MotorGoMini::always_init() {
  start_breathing();
  init_spi();
}

void MotorGoMini::init_spi() {
  encoder_spi_ = std::make_unique<Spi>(Spi::Config{
      .host = ENCODER_SPI_HOST,
      .sclk_io_num = ENCODER_SPI_SCLK_PIN,
      .mosi_io_num = GPIO_NUM_NC,
      .miso_io_num = ENCODER_SPI_MISO_PIN,
      .max_transfer_sz = 100,
      .log_level = get_log_level(),
  });
  std::error_code ec;
  encoder1_spi_device_ = encoder_spi_->add_device(
      Spi::DeviceConfig{
          .mode = 0,
          .clock_speed_hz = ENCODER_SPI_CLK_SPEED,
          .cs_io_num = ENCODER_1_CS_PIN,
          .queue_size = 1,
      },
      ec);
  if (ec || !encoder1_spi_device_) {
    logger_.error("Failed to initialize Encoder 1 SPI device: {}", ec.message());
    return;
  }
  encoder2_spi_device_ = encoder_spi_->add_device(
      Spi::DeviceConfig{
          .mode = 0,
          .clock_speed_hz = ENCODER_SPI_CLK_SPEED,
          .cs_io_num = ENCODER_2_CS_PIN,
          .queue_size = 1,
      },
      ec);
  if (ec || !encoder2_spi_device_) {
    logger_.error("Failed to initialize Encoder 2 SPI device: {}", ec.message());
    return;
  }
}

float MotorGoMini::breathe(float breathing_period, uint64_t start_us, bool restart) {
  auto now_us = esp_timer_get_time();
  if (restart) {
    start_us = now_us;
  }
  auto elapsed_us = now_us - start_us;
  float elapsed = elapsed_us / 1e6f;
  float t = std::fmod(elapsed, breathing_period) / breathing_period;
  return gaussian_(t);
}

bool IRAM_ATTR MotorGoMini::read_encoder(const std::shared_ptr<Spi::Device> &encoder_device,
                                         uint8_t *data, size_t size) {
  if (!encoder_device) {
    return false;
  }
  std::error_code ec;
  return encoder_device->read(std::span<uint8_t>(data, size), {}, ec);
}

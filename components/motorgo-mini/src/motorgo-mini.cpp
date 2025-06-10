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

void MotorGoMini::always_init() {
  start_breathing();
  init_spi();
}

void MotorGoMini::init_spi() {
  // Initialize the SPI bus for the encoders
  memset(&encoder_spi_bus_config_, 0, sizeof(encoder_spi_bus_config_));
  encoder_spi_bus_config_.mosi_io_num = -1;
  encoder_spi_bus_config_.miso_io_num = ENCODER_SPI_MISO_PIN;
  encoder_spi_bus_config_.sclk_io_num = ENCODER_SPI_SCLK_PIN;
  encoder_spi_bus_config_.quadwp_io_num = -1;
  encoder_spi_bus_config_.quadhd_io_num = -1;
  encoder_spi_bus_config_.max_transfer_sz = 100;
  // encoder_spi_bus_config_.isr_cpu_id = 0; // set to the same core as the esp-timer task (which
  // runs the encoders)
  auto err = spi_bus_initialize(ENCODER_SPI_HOST, &encoder_spi_bus_config_, SPI_DMA_CH_AUTO);
  if (err != ESP_OK) {
    logger_.error("Failed to initialize SPI bus for encoders: {}", esp_err_to_name(err));
    return;
  }

  // Initialize the encoder 1
  memset(&encoder1_config, 0, sizeof(encoder1_config));
  encoder1_config.mode = 0;
  encoder1_config.clock_speed_hz = ENCODER_SPI_CLK_SPEED;
  encoder1_config.queue_size = 1;
  encoder1_config.spics_io_num = ENCODER_1_CS_PIN;
  // encoder1_config.cs_ena_pretrans = 2;
  // encoder1_config.input_delay_ns = 30;
  err = spi_bus_add_device(ENCODER_SPI_HOST, &encoder1_config, &encoder1_handle_);
  if (err != ESP_OK) {
    logger_.error("Failed to initialize Encoder 1: {}", esp_err_to_name(err));
    return;
  }

  // Initialize the encoder 2
  memset(&encoder2_config, 0, sizeof(encoder2_config));
  encoder2_config.mode = 0;
  encoder2_config.clock_speed_hz = ENCODER_SPI_CLK_SPEED;
  encoder2_config.queue_size = 1;
  encoder2_config.spics_io_num = ENCODER_2_CS_PIN;
  // encoder2_config.cs_ena_pretrans = 2;
  // encoder2_config.input_delay_ns = 30;
  err = spi_bus_add_device(ENCODER_SPI_HOST, &encoder2_config, &encoder2_handle_);
  if (err != ESP_OK) {
    logger_.error("Failed to initialize Encoder 2: {}", esp_err_to_name(err));
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

bool IRAM_ATTR MotorGoMini::read_encoder(const auto &encoder_handle, uint8_t *data, size_t size) {
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
  esp_err_t err = spi_device_transmit(encoder_handle, &t);
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

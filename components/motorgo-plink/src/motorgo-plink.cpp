#include "motorgo-plink.hpp"

#include <algorithm>
#include <cmath>

using namespace espp;

MotorGoPlink::MotorGoPlink()
    : BaseComponent("MotorGoPlink") {}

I2c &MotorGoPlink::get_external_i2c() { return qwiic_i2c_; }

I2c &MotorGoPlink::qwiic_i2c() { return qwiic_i2c_; }

I2c &MotorGoPlink::hidden_i2c() { return hidden_i2c_; }

std::array<MotorGoPlink::MotorPins, 4> MotorGoPlink::motor_pins() const { return motor_pin_map_; }

MotorGoPlink::MotorPins MotorGoPlink::motor_pins(size_t index) const {
  if (index >= motor_pin_map_.size()) {
    logger_.error("Invalid motor index: {}", index);
    return {};
  }
  return motor_pin_map_[index];
}

MotorGoPlink::EncoderBusPins MotorGoPlink::encoder_bus_pins() const { return encoder_bus_pin_map_; }

std::array<gpio_num_t, 4> MotorGoPlink::encoder_cs_pins() const { return encoder_cs_pin_map_; }

gpio_num_t MotorGoPlink::encoder_cs_pin(size_t index) const {
  if (index >= encoder_cs_pin_map_.size()) {
    logger_.error("Invalid encoder index: {}", index);
    return GPIO_NUM_NC;
  }
  return encoder_cs_pin_map_[index];
}

std::array<gpio_num_t, 4> MotorGoPlink::servo_pins() const { return servo_pin_map_; }

gpio_num_t MotorGoPlink::servo_pin(size_t index) const {
  if (index >= servo_pin_map_.size()) {
    logger_.error("Invalid servo index: {}", index);
    return GPIO_NUM_NC;
  }
  return servo_pin_map_[index];
}

MotorGoPlink::I2cPins MotorGoPlink::qwiic_pins() const { return qwiic_pin_map_; }

MotorGoPlink::I2cPins MotorGoPlink::hidden_i2c_pins() const { return hidden_i2c_pin_map_; }

MotorGoPlink::LedPins MotorGoPlink::led_pins() const {
  return {.user = user_led_pin_, .status = status_led_pin_};
}

bool MotorGoPlink::initialize_motors(size_t pwm_frequency_hz) {
  if (std::any_of(motor_drivers_.begin(), motor_drivers_.end(),
                  [](const auto &driver) { return static_cast<bool>(driver); })) {
    logger_.error("Motor drivers already initialized");
    return false;
  }
  if (pwm_frequency_hz == 0) {
    logger_.error("Motor PWM frequency must be greater than zero");
    return false;
  }
  for (size_t i = 0; i < motor_drivers_.size(); i++) {
    auto pins = motor_pin_map_[i];
    auto driver = std::make_shared<MotorDriver>(MotorDriver::Config{
        .gpio_a = pins.pwm_a,
        .gpio_b = pins.pwm_b,
        .group_id = motor_driver_group_ids_[i],
        .pwm_frequency_hz = pwm_frequency_hz,
        .log_level = get_log_level(),
    });
    if (!driver->initialized()) {
      logger_.error("Failed to initialize motor driver {}", i + 1);
      stop_all_motors();
      motor_drivers_ = {};
      motor_speeds_.fill(0.0f);
      return false;
    }
    motor_drivers_[i] = driver;
  }
  stop_all_motors();
  return true;
}

bool MotorGoPlink::set_motor_speed(size_t index, float speed) {
  if (index >= motor_pin_map_.size()) {
    logger_.error("Invalid motor index: {}", index);
    return false;
  }
  if (!motor_drivers_[index]) {
    logger_.error("Motor driver {} not initialized", index + 1);
    return false;
  }
  speed = std::clamp(speed, -1.0f, 1.0f);
  bool ok = motor_drivers_[index]->set_speed(speed);
  if (ok) {
    motor_speeds_[index] = speed;
  }
  return ok;
}

float MotorGoPlink::motor_speed(size_t index) const {
  if (index >= motor_speeds_.size()) {
    logger_.error("Invalid motor index: {}", index);
    return 0.0f;
  }
  return motor_speeds_[index];
}

void MotorGoPlink::stop_motor(size_t index) { set_motor_speed(index, 0.0f); }

void MotorGoPlink::stop_all_motors() {
  for (size_t i = 0; i < motor_drivers_.size(); i++) {
    if (motor_drivers_[i]) {
      motor_drivers_[i]->stop();
    }
    motor_speeds_[i] = 0.0f;
  }
}

std::shared_ptr<MotorGoPlink::MotorDriver> MotorGoPlink::motor_driver(size_t index) const {
  if (index >= motor_drivers_.size()) {
    logger_.error("Invalid motor index: {}", index);
    return nullptr;
  }
  return motor_drivers_[index];
}

bool MotorGoPlink::initialize_encoders(bool run_tasks) {
  if (!initialize_encoder_spi()) {
    return false;
  }
  for (size_t i = 0; i < encoders_.size(); i++) {
    if (encoders_[i]) {
      continue;
    }
    Encoder::Config config{
        .read = [this, i](uint8_t *data, size_t size) -> bool {
          return read_encoder(i, data, size);
        },
        .update_period = std::chrono::duration<float>(encoder_update_period_seconds_),
        .auto_init = false,
        .run_task = false,
        .log_level = get_log_level(),
    };
    auto encoder = std::make_shared<Encoder>(config);
    std::error_code ec;
    encoder->initialize(run_tasks, ec);
    if (ec) {
      logger_.error("Failed to initialize encoder {}: {}", i + 1, ec.message());
      return false;
    }
    encoders_[i] = encoder;
  }
  return true;
}

std::shared_ptr<MotorGoPlink::Encoder> MotorGoPlink::encoder(size_t index) {
  if (index >= encoders_.size()) {
    logger_.error("Invalid encoder index: {}", index);
    return nullptr;
  }
  return encoders_[index];
}

void MotorGoPlink::reset_encoder_accumulator(size_t index) {
  auto encoder_ptr = encoder(index);
  if (!encoder_ptr) {
    logger_.error("Encoder {} not initialized", index + 1);
    return;
  }
  encoder_ptr->reset_accumulator();
}

bool MotorGoPlink::initialize_imu(const Imu::filter_fn &orientation_filter,
                                  const Imu::ImuConfig &imu_config) {
  if (imu_) {
    logger_.warn("IMU already initialized, not initializing again!");
    return false;
  }

  std::error_code ec;
  imu_i2c_device_ = hidden_i2c_.add_device<uint8_t>(
      {
          .device_address = Imu::DEFAULT_I2C_ADDRESS,
          .timeout_ms = static_cast<int>(hidden_i2c_.config().timeout_ms),
          .scl_speed_hz = hidden_i2c_.config().clk_speed,
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!imu_i2c_device_) {
    logger_.error("Could not initialize IMU I2C device: {}", ec.message());
    return false;
  }

  Imu::Config config{
      .device_address = Imu::DEFAULT_I2C_ADDRESS,
      .write = espp::make_i2c_addressed_write(imu_i2c_device_),
      .read = espp::make_i2c_addressed_read(imu_i2c_device_),
      .imu_config = imu_config,
      .orientation_filter = orientation_filter,
      .auto_init = false,
      .log_level = get_log_level(),
  };

  logger_.info("Initializing hidden-bus IMU");
  imu_ = std::make_shared<Imu>(config);
  if (!imu_ || !imu_->init(ec)) {
    logger_.error("Failed to initialize hidden-bus IMU: {}", ec.message());
    imu_.reset();
    imu_i2c_device_.reset();
    return false;
  }
  return true;
}

std::shared_ptr<MotorGoPlink::Imu> MotorGoPlink::imu() const { return imu_; }

bool MotorGoPlink::initialize_leds(float breathing_period) {
  if (indicator_leds_) {
    logger_.error("Indicator LEDs already initialized");
    return false;
  }
  indicator_leds_ = std::make_shared<espp::Led>(espp::Led::Config{
      .timer = LEDC_TIMER_1,
      .frequency_hz = indicator_led_pwm_frequency_hz_,
      .channels = std::vector<espp::Led::ChannelConfig>(led_channels_.begin(), led_channels_.end()),
      .duty_resolution = LEDC_TIMER_10_BIT,
      .log_level = get_log_level(),
  });
  using namespace std::placeholders;
  led_task_ = espp::Task::make_unique(
      {.callback = std::bind(&MotorGoPlink::led_task_callback, this, _1, _2, _3),
       .task_config = {.name = "motorgo_plink_led"}});
  if (!led_task_) {
    logger_.error("Failed to create indicator LED task");
    indicator_leds_.reset();
    return false;
  }
  if (!set_led_breathing_period(breathing_period)) {
    led_task_.reset();
    indicator_leds_.reset();
    return false;
  }
  if (!indicator_leds_->set_duty(led_channels_[0].channel, 0.0f) ||
      !indicator_leds_->set_duty(led_channels_[1].channel, 0.0f)) {
    logger_.error("Failed to initialize indicator LED channels");
    led_task_.reset();
    indicator_leds_.reset();
    return false;
  }
  return true;
}

void MotorGoPlink::start_led_breathing() {
  if (!indicator_leds_ || !led_task_) {
    logger_.error("Indicator LEDs not initialized");
    return;
  }
  breathing_start_ = std::chrono::high_resolution_clock::now();
  led_task_->start();
}

void MotorGoPlink::stop_led_breathing() {
  if (!indicator_leds_ || !led_task_) {
    logger_.error("Indicator LEDs not initialized");
    return;
  }
  led_task_->stop();
  indicator_leds_->set_duty(led_channels_[0].channel, 0.0f);
  indicator_leds_->set_duty(led_channels_[1].channel, 0.0f);
}

bool MotorGoPlink::set_user_led_brightness(float brightness) {
  if (!indicator_leds_ || !led_task_) {
    logger_.error("Indicator LEDs not initialized");
    return false;
  }
  if (led_task_->is_running()) {
    logger_.error("Cannot set user LED brightness while breathing");
    return false;
  }
  brightness = std::clamp(brightness, 0.0f, 1.0f);
  return indicator_leds_->set_duty(led_channels_[0].channel, 100.0f * brightness);
}

float MotorGoPlink::get_user_led_brightness() {
  if (!indicator_leds_) {
    logger_.error("Indicator LEDs not initialized");
    return 0.0f;
  }
  auto maybe_duty = indicator_leds_->get_duty(led_channels_[0].channel);
  if (!maybe_duty.has_value()) {
    logger_.error("Failed to get user LED duty");
    return 0.0f;
  }
  return maybe_duty.value() / 100.0f;
}

bool MotorGoPlink::set_status_led_brightness(float brightness) {
  if (!indicator_leds_ || !led_task_) {
    logger_.error("Indicator LEDs not initialized");
    return false;
  }
  if (led_task_->is_running()) {
    logger_.error("Cannot set status LED brightness while breathing");
    return false;
  }
  brightness = std::clamp(brightness, 0.0f, 1.0f);
  return indicator_leds_->set_duty(led_channels_[1].channel, 100.0f * brightness);
}

float MotorGoPlink::get_status_led_brightness() {
  if (!indicator_leds_) {
    logger_.error("Indicator LEDs not initialized");
    return 0.0f;
  }
  auto maybe_duty = indicator_leds_->get_duty(led_channels_[1].channel);
  if (!maybe_duty.has_value()) {
    logger_.error("Failed to get status LED duty");
    return 0.0f;
  }
  return maybe_duty.value() / 100.0f;
}

bool MotorGoPlink::set_led_breathing_period(float period) {
  if (period <= 0.0f) {
    logger_.error("Invalid LED breathing period: {}", period);
    return false;
  }
  breathing_period_ = period;
  return true;
}

float MotorGoPlink::get_led_breathing_period() { return breathing_period_; }

std::shared_ptr<espp::Led> MotorGoPlink::leds() { return indicator_leds_; }

espp::Gaussian &MotorGoPlink::gaussian() { return gaussian_; }

bool MotorGoPlink::initialize_encoder_spi() {
  if (encoder_spi_) {
    return true;
  }
  encoder_spi_ = std::make_unique<Spi>(Spi::Config{
      .host = encoder_spi_host_,
      .sclk_io_num = encoder_bus_pin_map_.sclk,
      .mosi_io_num = encoder_bus_pin_map_.mosi,
      .miso_io_num = encoder_bus_pin_map_.miso,
      .max_transfer_sz = 32,
      .log_level = get_log_level(),
  });

  std::error_code ec;
  for (size_t i = 0; i < encoder_spi_devices_.size(); i++) {
    encoder_spi_devices_[i] = encoder_spi_->add_device(
        Spi::DeviceConfig{
            .mode = 0,
            .clock_speed_hz = encoder_spi_clock_speed_hz_,
            .cs_io_num = encoder_cs_pin_map_[i],
            .queue_size = 1,
        },
        ec);
    if (ec || !encoder_spi_devices_[i]) {
      logger_.error("Failed to initialize encoder {} SPI device: {}", i + 1, ec.message());
      return false;
    }
  }
  return true;
}

bool IRAM_ATTR MotorGoPlink::read_encoder(size_t index, uint8_t *data, size_t size) {
  if (index >= encoder_spi_devices_.size() || !encoder_spi_devices_[index]) {
    return false;
  }
  std::error_code ec;
  return encoder_spi_devices_[index]->read(std::span<uint8_t>(data, size), {}, ec);
}

float MotorGoPlink::led_breathe(float phase_offset) {
  auto now = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration<float>(now - breathing_start_).count();
  float t = std::fmod(elapsed, breathing_period_) / breathing_period_;
  t = std::fmod(t + phase_offset, 1.0f);
  return gaussian_(t);
}

bool MotorGoPlink::led_task_callback(std::mutex &m, std::condition_variable &cv,
                                     bool &task_notified) {
  using namespace std::chrono_literals;
  float user_brightness = led_breathe(0.0f);
  float status_brightness = led_breathe(0.5f);
  indicator_leds_->set_duty(led_channels_[0].channel, 100.0f * user_brightness);
  indicator_leds_->set_duty(led_channels_[1].channel, 100.0f * status_brightness);
  std::unique_lock<std::mutex> lk(m);
  cv.wait_for(lk, 10ms, [&task_notified] { return task_notified; });
  task_notified = false;
  return false;
}

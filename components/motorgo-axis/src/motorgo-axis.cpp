#include "motorgo-axis.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <span>
#include <vector>

using namespace espp;

MotorGoAxis::MotorGoAxis()
    : BaseComponent("MotorGoAxis") {}

I2c &MotorGoAxis::get_external_i2c() { return qwiic_i2c_; }

I2c &MotorGoAxis::qwiic_i2c() { return qwiic_i2c_; }

I2c &MotorGoAxis::hidden_i2c() { return hidden_i2c_; }

std::array<MotorGoAxis::MotorPins, 2> MotorGoAxis::motor_pins() const { return motor_pin_map_; }

MotorGoAxis::MotorPins MotorGoAxis::motor_pins(size_t index) const {
  if (index >= motor_pin_map_.size()) {
    logger_.error("Invalid motor index: {}", index);
    return {};
  }
  return motor_pin_map_[index];
}

MotorGoAxis::EncoderBusPins MotorGoAxis::encoder_bus_pins() const { return encoder_bus_pin_map_; }

std::array<gpio_num_t, 2> MotorGoAxis::encoder_cs_pins() const { return encoder_cs_pin_map_; }

gpio_num_t MotorGoAxis::encoder_cs_pin(size_t index) const {
  if (index >= encoder_cs_pin_map_.size()) {
    logger_.error("Invalid encoder index: {}", index);
    return GPIO_NUM_NC;
  }
  return encoder_cs_pin_map_[index];
}

MotorGoAxis::I2cPins MotorGoAxis::qwiic_pins() const { return qwiic_pin_map_; }

MotorGoAxis::I2cPins MotorGoAxis::hidden_i2c_pins() const { return hidden_i2c_pin_map_; }

MotorGoAxis::LedPins MotorGoAxis::led_pins() const {
  return {.user = user_led_pin_, .status = status_led_pin_};
}

bool MotorGoAxis::initialize_motors(float power_supply_voltage, float limit_voltage,
                                    uint64_t dead_zone_ns) {
  if (std::any_of(motor_drivers_.begin(), motor_drivers_.end(),
                  [](const auto &driver) { return static_cast<bool>(driver); })) {
    logger_.error("Motor drivers already initialized");
    return false;
  }
  if (power_supply_voltage <= 0.0f) {
    logger_.error("Motor power supply voltage must be greater than zero");
    return false;
  }
  for (size_t i = 0; i < motor_drivers_.size(); i++) {
    auto pins = motor_pin_map_[i];
    auto driver = std::make_shared<MotorDriver>(MotorDriver::Config{
        .gpio_a_h = pins.u_high,
        .gpio_a_l = pins.u_low,
        .gpio_b_h = pins.v_high,
        .gpio_b_l = pins.v_low,
        .gpio_c_h = pins.w_high,
        .gpio_c_l = pins.w_low,
        .gpio_enable = -1,
        .gpio_fault = -1,
        .power_supply_voltage = power_supply_voltage,
        .limit_voltage = limit_voltage,
        .dead_zone_ns = dead_zone_ns,
        .log_level = get_log_level(),
    });
    motor_drivers_[i] = driver;
  }
  disable_all_motor_drivers();
  return true;
}

std::shared_ptr<MotorGoAxis::MotorDriver> MotorGoAxis::motor_driver(size_t index) const {
  if (index >= motor_drivers_.size()) {
    logger_.error("Invalid motor index: {}", index);
    return nullptr;
  }
  return motor_drivers_[index];
}

bool MotorGoAxis::motor_driver_enabled(size_t index) const {
  auto driver = motor_driver(index);
  return driver && driver->is_enabled();
}

bool MotorGoAxis::enable_motor_driver(size_t index) {
  auto driver = motor_driver(index);
  if (!driver) {
    logger_.error("Motor driver {} not initialized", index + 1);
    return false;
  }
  driver->enable();
  return driver->is_enabled();
}

void MotorGoAxis::disable_motor_driver(size_t index) {
  auto driver = motor_driver(index);
  if (!driver) {
    logger_.error("Motor driver {} not initialized", index + 1);
    return;
  }
  driver->disable();
}

void MotorGoAxis::enable_all_motor_drivers() {
  for (auto &driver : motor_drivers_) {
    if (driver) {
      driver->enable();
    }
  }
}

void MotorGoAxis::disable_all_motor_drivers() {
  for (auto &driver : motor_drivers_) {
    if (driver) {
      driver->disable();
    }
  }
}

bool MotorGoAxis::initialize_encoders(bool run_tasks) {
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

std::shared_ptr<MotorGoAxis::Encoder> MotorGoAxis::encoder(size_t index) {
  if (index >= encoders_.size()) {
    logger_.error("Invalid encoder index: {}", index);
    return nullptr;
  }
  return encoders_[index];
}

void MotorGoAxis::reset_encoder_accumulator(size_t index) {
  auto encoder_ptr = encoder(index);
  if (!encoder_ptr) {
    logger_.error("Encoder {} not initialized", index + 1);
    return;
  }
  encoder_ptr->reset_accumulator();
}

MotorGoAxis::BldcMotor::Config MotorGoAxis::default_motor_config(size_t index) const {
  if (index >= num_motor_channels()) {
    logger_.error("Invalid motor index: {}", index);
  }
  // Sensible defaults for the board's gimbal motors. The driver and sensor are
  // filled in by initialize_motor(). These mirror the espp::MotorGoMini default
  // motor configuration so the same code drives either board.
  return BldcMotor::Config{
      .num_pole_pairs = 7,
      .phase_resistance = 4.0f,
      .kv_rating = 320,
      .current_limit = 1.0f,
      .foc_type = espp::detail::FocType::SPACE_VECTOR_PWM,
      .driver = nullptr,          // filled in by initialize_motor()
      .sensor = nullptr,          // filled in by initialize_motor()
      .run_sensor_update = false, // the board runs the encoder update task
      .velocity_pid_config =
          {
              .kp = 0.010f,
              .ki = 0.100f,
              .kd = 0.000f,
              .integrator_min = -1.0f,
              .integrator_max = 1.0f,
              .output_min = -1.0,
              .output_max = 1.0,
          },
      .angle_pid_config =
          {
              .kp = 5.000f,
              .ki = 1.000f,
              .kd = 0.000f,
              .integrator_min = -10.0f,
              .integrator_max = 10.0f,
              .output_min = -20.0,
              .output_max = 20.0,
          },
      .log_level = get_log_level(),
  };
}

std::shared_ptr<MotorGoAxis::BldcMotor>
MotorGoAxis::initialize_motor(size_t index, const MotorGoAxis::BldcMotor::Config &config) {
  if (index >= num_motor_channels()) {
    logger_.error("Invalid motor index: {}", index);
    return nullptr;
  }
  // make sure the encoder and driver for this channel exist; initialize the
  // board defaults if the user has not done so already.
  if (!encoder(index)) {
    initialize_encoders();
  }
  if (!motor_driver(index)) {
    initialize_motors();
  }
  auto enc = encoder(index);
  auto drv = motor_driver(index);
  if (!enc || !drv) {
    logger_.error("Could not initialize driver / encoder for motor {}", index + 1);
    return nullptr;
  }
  // the board owns the driver + encoder (and the encoder runs its own update
  // task), so override those fields regardless of what the caller passed.
  auto motor_config = config;
  motor_config.driver = drv;
  motor_config.sensor = enc;
  motor_config.run_sensor_update = false;
  motors_[index] = std::make_shared<BldcMotor>(motor_config);
  return motors_[index];
}

std::shared_ptr<MotorGoAxis::BldcMotor> MotorGoAxis::motor(size_t index) {
  if (index >= motors_.size()) {
    logger_.error("Invalid motor index: {}", index);
    return nullptr;
  }
  return motors_[index];
}

bool MotorGoAxis::initialize_imu(const Imu::filter_fn &orientation_filter,
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

std::shared_ptr<MotorGoAxis::Imu> MotorGoAxis::imu() const { return imu_; }

bool MotorGoAxis::initialize_leds(float breathing_period) {
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
      {.callback = std::bind(&MotorGoAxis::led_task_callback, this, _1, _2, _3),
       .task_config = {.name = "motorgo_axis_led"}});
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

void MotorGoAxis::start_led_breathing() {
  if (!indicator_leds_ || !led_task_) {
    logger_.error("Indicator LEDs not initialized");
    return;
  }
  breathing_start_ = std::chrono::high_resolution_clock::now();
  led_task_->start();
}

void MotorGoAxis::stop_led_breathing() {
  if (!indicator_leds_ || !led_task_) {
    logger_.error("Indicator LEDs not initialized");
    return;
  }
  led_task_->stop();
  indicator_leds_->set_duty(led_channels_[0].channel, 0.0f);
  indicator_leds_->set_duty(led_channels_[1].channel, 0.0f);
}

bool MotorGoAxis::set_user_led_brightness(float brightness) {
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

float MotorGoAxis::get_user_led_brightness() {
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

bool MotorGoAxis::set_status_led_brightness(float brightness) {
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

float MotorGoAxis::get_status_led_brightness() {
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

bool MotorGoAxis::set_led_breathing_period(float period) {
  if (period <= 0.0f) {
    logger_.error("Invalid LED breathing period: {}", period);
    return false;
  }
  breathing_period_ = period;
  return true;
}

float MotorGoAxis::get_led_breathing_period() { return breathing_period_; }

std::shared_ptr<espp::Led> MotorGoAxis::leds() { return indicator_leds_; }

espp::Gaussian &MotorGoAxis::gaussian() { return gaussian_; }

bool MotorGoAxis::initialize_encoder_spi() {
  if (encoder_spi_) {
    return true;
  }
  encoder_spi_ = std::make_unique<Spi>(Spi::Config{
      .host = encoder_spi_host_,
      .sclk_io_num = encoder_bus_pin_map_.sclk,
      .mosi_io_num = GPIO_NUM_NC,
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
      encoder_spi_devices_ = {};
      encoder_spi_.reset();
      return false;
    }
  }
  return true;
}

bool IRAM_ATTR MotorGoAxis::read_encoder(size_t index, uint8_t *data, size_t size) {
  if (index >= encoder_spi_devices_.size() || !encoder_spi_devices_[index]) {
    return false;
  }
  std::error_code ec;
  return encoder_spi_devices_[index]->read(std::span<uint8_t>(data, size), {}, ec);
}

float MotorGoAxis::led_breathe(float phase_offset) {
  auto now = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration<float>(now - breathing_start_).count();
  float t = std::fmod(elapsed, breathing_period_) / breathing_period_;
  t = std::fmod(t + phase_offset, 1.0f);
  return gaussian_(t);
}

bool MotorGoAxis::led_task_callback(std::mutex &m, std::condition_variable &cv,
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

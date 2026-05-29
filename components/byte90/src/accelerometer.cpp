#include "byte90.hpp"

using namespace espp;

/////////////////////////////
// Accelerometer Functions //
/////////////////////////////

bool Byte90::initialize_accelerometer(const Byte90::accel_callback_t &callback) {
  // check if the accelerometer is already initialized
  if (accelerometer_) {
    logger_.warn("Accelerometer already initialized");
    return true; // already initialized
  }

  logger_.info("Initializing accelerometer");

  // store the callback
  accel_callback_ = callback;

  std::error_code ec;
  accelerometer_i2c_device_ = internal_i2c_.add_device<uint8_t>(
      {
          .device_address = Accelerometer::DEFAULT_ADDRESS,
          .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
          .scl_speed_hz = internal_i2c_.config().clk_speed,
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!accelerometer_i2c_device_) {
    logger_.error("Could not initialize accelerometer I2C device: {}", ec.message());
    return false;
  }
  // create the accelerometer
  accelerometer_ = std::make_shared<Accelerometer>(Accelerometer::Config{
      .device_address = Accelerometer::DEFAULT_ADDRESS,
      .range = Accelerometer::RANGE_2G,
      .data_rate = Accelerometer::RATE_100_HZ,
      .write = espp::make_i2c_addressed_write(accelerometer_i2c_device_),
      .read = espp::make_i2c_addressed_read(accelerometer_i2c_device_),
      .log_level = espp::Logger::Verbosity::WARN,
  });

  // Disable measurement mode initially, so we can configure interrupts and such
  accelerometer_->set_measurement_mode(false, ec);

  // add the accelerometer interrupt pin config to the interrupt manager
  interrupts_.add_interrupt(accelerometer_interrupt_pin_);

  // Configure the ADXL345
  accelerometer_->set_fifo_mode(espp::Adxl345::FifoMode::STREAM,
                                ec); // discard old data when FIFO is full
  bool active_high = false;
  accelerometer_->set_interrupt_polarity(active_high,
                                         ec); // set interrupt polarity to active low (default)
  // configure the interrupt to trigger on watermark on INT1 pin
  accelerometer_->set_interrupt_mapping(espp::Adxl345::InterruptType::WATERMARK,
                                        espp::Adxl345::InterruptPin::INT1, ec);
  // enable the watermark interrupt
  accelerometer_->configure_interrupt(espp::Adxl345::InterruptType::WATERMARK, true, ec);

  // enable the accelerometer
  accelerometer_->set_measurement_mode(true, ec);

  return !ec;
}

# BMI270 6-Axis IMU Driver

[![Badge](https://components.espressif.com/components/espp/bmi270/badge.svg)](https://components.espressif.com/components/espp/bmi270)

The `Bmi270` component provides a comprehensive driver for the Bosch Sensortec BMI270 6-Axis Inertial Measurement Unit (IMU).

## Features

The BMI270 is a highly integrated, low power inertial measurement unit (IMU) that combines precise acceleration and angular rate measurement with intelligent on-chip motion-triggered interrupt features.

### Key Specifications
- **16-bit digital, triaxial accelerometer**: ±2g/±4g/±8g/±16g range
- **16-bit digital, triaxial gyroscope**: ±125°/s to ±2000°/s range  
- **Output data rates**: 0.78Hz to 1.6kHz (accelerometer), 25Hz to 6.4kHz (gyroscope)
- **Low power consumption**: typ. 685µA in full operation, 3.5µA in sleep mode
- **Built-in temperature sensor**: 0.5°C resolution, ±2°C accuracy
- **2KB on-chip FIFO buffer** for accelerometer, gyroscope, and auxiliary sensor data
- **Advanced power management** with multiple power modes
- **Hardware synchronization** of accelerometer and gyroscope (< 1µs)

### Advanced Features
The BMI270 includes sophisticated motion detection capabilities:
- **Motion Detection**: Any motion, no motion, significant motion
- **Step Counter/Detector**: Wrist-worn step counting with activity recognition
- **Activity Recognition**: Still, walk, run classification
- **Wrist Gestures**: Wrist wear wakeup, gesture detection
- **Orientation Detection**: Portrait/landscape, face up/down
- **Tap Detection**: Single and double tap recognition

### Supported Interfaces
- **I2C**: Up to 1MHz (Fast Mode Plus)
- **SPI**: Up to 10MHz (4-wire and 3-wire modes)

## Usage

### Basic IMU Reading

```cpp
#include "bmi270.hpp"
#include "i2c.hpp"

// Create I2C instance
espp::I2c i2c({
    .port = I2C_NUM_0,
    .sda_io_num = GPIO_NUM_21,
    .scl_io_num = GPIO_NUM_22,
    .clk_speed = 400000
});

// Configure BMI270
espp::Bmi270<>::Config config{
    .device_address = espp::Bmi270<>::DEFAULT_ADDRESS,
    .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, 
                       std::placeholders::_2, std::placeholders::_3),
    .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1, 
                      std::placeholders::_2, std::placeholders::_3),
    .imu_config = {
        .accelerometer_range = espp::Bmi270<>::AccelerometerRange::RANGE_4G,
        .accelerometer_odr = espp::Bmi270<>::AccelerometerODR::ODR_100_HZ,
        .gyroscope_range = espp::Bmi270<>::GyroscopeRange::RANGE_1000DPS,
        .gyroscope_odr = espp::Bmi270<>::GyroscopeODR::ODR_100_HZ,
    }
};

// Create BMI270 instance
espp::Bmi270<> imu(config);

// Read sensor data
std::error_code ec;
if (imu.update(0.01f, ec)) {  // 10ms update interval
    auto accel = imu.get_accelerometer();  // in g
    auto gyro = imu.get_gyroscope();       // in °/s  
    auto temp = imu.get_temperature();     // in °C
    
    fmt::print("Accel: {:.2f}, {:.2f}, {:.2f} g\n", accel.x, accel.y, accel.z);
    fmt::print("Gyro: {:.2f}, {:.2f}, {:.2f} °/s\n", gyro.x, gyro.y, gyro.z);
    fmt::print("Temp: {:.1f} °C\n", temp);
}
```

### With Orientation Filter

```cpp
#include "kalman_filter.hpp"

// Create Kalman filter for orientation estimation
static espp::KalmanFilter<2> kf;
kf.set_process_noise(0.1f);
kf.set_measurement_noise(0.001f);

auto kalman_filter_fn = [](float dt, const auto &accel, const auto &gyro) {
    // Calculate angles from accelerometer
    float accel_roll = atan2(accel.y, accel.z);
    float accel_pitch = atan2(-accel.x, sqrt(accel.y * accel.y + accel.z * accel.z));
    
    // Predict with gyroscope
    kf.predict({espp::deg_to_rad(gyro.x), espp::deg_to_rad(gyro.y)}, dt);
    
    // Update with accelerometer
    kf.update({accel_roll, accel_pitch});
    
    auto [roll, pitch] = kf.get_state();
    return espp::Bmi270<>::Value{.roll = roll, .pitch = pitch, .yaw = 0.0f};
};

// Configure BMI270 with orientation filter
config.orientation_filter = kalman_filter_fn;
espp::Bmi270<> imu(config);

// Get orientation data
if (imu.update(0.01f, ec)) {
    auto orientation = imu.get_orientation();  // in radians
    auto gravity = imu.get_gravity_vector();
    
    fmt::print("Orientation: roll={:.2f}°, pitch={:.2f}°\n", 
               espp::rad_to_deg(orientation.roll), 
               espp::rad_to_deg(orientation.pitch));
}
```

### Advanced Features Configuration

```cpp
// Enable advanced motion features
espp::Bmi270<>::InterruptConfig int_config{
    .pin = espp::Bmi270<>::InterruptPin::INT1,
    .enable_step_detector = true,
    .enable_any_motion = true,
    .enable_significant_motion = true
};

imu.configure_interrupts(int_config, ec);
imu.enable_advanced_features(true, ec);
```

## Configuration Options

### Accelerometer Configuration
- **Range**: ±2g, ±4g, ±8g, ±16g
- **Output Data Rate**: 0.78Hz to 1600Hz
- **Bandwidth**: Various filtering options (OSR4, OSR2, Normal, CIC, etc.)

### Gyroscope Configuration  
- **Range**: ±125°/s, ±250°/s, ±500°/s, ±1000°/s, ±2000°/s
- **Output Data Rate**: 25Hz to 6400Hz
- **Bandwidth**: OSR4, OSR2, Normal, CIC modes
- **Performance Mode**: Power optimized or performance optimized (lower noise)

### Power Management
- **Suspend Mode**: Lowest power consumption
- **Normal Mode**: Full operation
- **Low Power Mode**: Reduced power with limited functionality
- **Fast Startup Mode**: Quick wake-up from suspend

### FIFO Configuration
- **Modes**: Bypass, FIFO, Stream
- **Data Selection**: Accelerometer, gyroscope, temperature
- **Watermark Levels**: Configurable interrupt thresholds

## Hardware Connections

### I2C Interface
| BMI270 Pin | ESP32 Pin | Description |
|------------|-----------|-------------|
| VDD        | 3.3V      | Power supply |
| VDDIO      | 3.3V      | I/O power supply |
| GND        | GND       | Ground |
| SCL        | GPIO22    | I2C clock |
| SDA        | GPIO21    | I2C data |
| SDO        | GND/3.3V  | I2C address select (0x68/0x69) |
| INT1       | GPIO (optional) | Interrupt 1 output |
| INT2       | GPIO (optional) | Interrupt 2 output |

### SPI Interface
| BMI270 Pin | ESP32 Pin | Description |
|------------|-----------|-------------|
| VDD        | 3.3V      | Power supply |
| VDDIO      | 3.3V      | I/O power supply |
| GND        | GND       | Ground |
| SCK        | GPIO18    | SPI clock |
| SDI        | GPIO23    | SPI MOSI |
| SDO        | GPIO19    | SPI MISO |
| CSB        | GPIO5     | SPI chip select |

## Applications

The BMI270 is ideal for:
- **Wearables**: Fitness trackers, smartwatches, hearables
- **Smart Clothing**: Activity monitoring garments
- **AR/VR**: Head-mounted displays and controllers  
- **Gaming**: Motion controllers and input devices
- **IoT Devices**: Context-aware applications
- **Robotics**: Orientation and motion sensing
- **Industrial**: Vibration monitoring, tilt sensing

## Example

The [example](./example) demonstrates comprehensive BMI270 usage including:
- Basic accelerometer and gyroscope reading
- Temperature measurement
- Orientation calculation using Kalman and Madgwick filters
- Data logging for analysis
- Interrupt configuration

## Performance Notes

- **Update Rate**: Up to 1600Hz for accelerometer, 6400Hz for gyroscope
- **Noise Performance**: < 7 mdps/√Hz (gyroscope in performance mode)
- **Startup Time**: 2ms for both accelerometer and gyroscope
- **Current Consumption**: 685µA typical in normal mode
- **Operating Temperature**: -40°C to +85°C

## References

- [BMI270 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf)
- [BMI270 GitHub Repository](https://github.com/BoschSensortec/BMI270-Sensor-API)
- [Application Notes](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi270/)

## License

This component is provided under the same license as the ESP-CPP project. 
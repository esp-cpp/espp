#pragma once

#include <atomic>
#include <functional>

#include "base_peripheral.hpp"

namespace espp {
/// @brief Driver for the GT911 touch controller
///
/// \section gt911_ex1 Example
/// \snippet gt911_example.cpp gt911 example
class Gt911 : public BasePeripheral<std::uint16_t> {
public:
  /// Default address for the GT911 chip, if the interrupt pin is low on power on
  static constexpr uint8_t DEFAULT_ADDRESS_1 = 0x5D;
  /// Alternate address for the GT911 chip, if the interrupt pin is high on power on
  static constexpr uint8_t DEFAULT_ADDRESS_2 = 0x14;

  /// @brief Configuration for the GT911 driver
  struct Config {
    BasePeripheral::write_fn write;      ///< Function for writing to the GT911 chip
    BasePeripheral::read_fn read;        ///< Function for reading from the GT911 chip
    uint8_t address = DEFAULT_ADDRESS_1; ///< Which address to use for this chip?
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; ///< Log verbosity for the input driver.
  };

  /// @brief Constructor for the GT911 driver
  /// @param config The configuration for the driver
  explicit Gt911(const Config &config)
      : BasePeripheral({.address = config.address, .write = config.write, .read = config.read},
                       "Gt911", config.log_level) {}

  /// @brief Update the state of the GT911 driver
  /// @param ec Error code to set if an error occurs
  /// @return True if the GT911 has new data, false otherwise
  bool update(std::error_code &ec) {
    bool new_data = false;
    static constexpr size_t DATA_LEN = CONTACT_SIZE * MAX_CONTACTS;
    static uint8_t data[DATA_LEN];
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    read_many_from_register((uint16_t)Registers::POINT_INFO, data, 1, ec);
    if (ec)
      return false;

    // the below is copied from
    // https://github.com/espressif/esp-bsp/blob/master/components/lcd_touch/esp_lcd_touch_gt911/esp_lcd_touch_gt911.c
    if ((data[0] & 0x80) == 0) {
      // no data available
    } else if ((data[0] & 0x10) == 0x10) {
      // only the keys were pressed, read them
      uint8_t key_max = MAX_KEYS;
      read_many_from_register((uint16_t)Registers::KEY, data, key_max, ec);
      if (ec)
        return false;
      // set the button state
      home_button_pressed_ = data[0];
      logger_.debug("Home button is {}", home_button_pressed_ ? "pressed" : "released");
      new_data = true;
    } else if ((data[0] & 0x80) == 0x80) {
      // clear the home button state
      home_button_pressed_ = false;

      // touch data is available
      num_touch_points_ = data[0] & 0x0f;
      logger_.debug("Got {} touch points", num_touch_points_);
      if (num_touch_points_ > 0) {
        read_many_from_register((uint16_t)Registers::POINTS, data, CONTACT_SIZE * num_touch_points_,
                                ec);
        if (ec)
          return false;
        // convert the data pointer to a GTPoint*
        const GTPoint *point = (GTPoint *)&data[0];
        x_ = point->x;
        y_ = point->y;
        logger_.debug("Touch at ({}, {})", x_, y_);
      }
      new_data = true;
    }

    // send the clear command
    write_u8_to_register((uint16_t)Registers::POINT_INFO, 0x00, ec); // sync signal

    return new_data;
  }

  /// @brief Get the number of touch points
  /// @return The number of touch points as of the last update
  /// @note This is a cached value from the last update() call
  uint8_t get_num_touch_points() const { return num_touch_points_; }

  /// @brief Get the touch point data
  /// @param num_touch_points The number of touch points as of the last update
  /// @param x The x coordinate of the touch point
  /// @param y The y coordinate of the touch point
  /// @note This is a cached value from the last update() call
  void get_touch_point(uint8_t *num_touch_points, uint16_t *x, uint16_t *y) const {
    *num_touch_points = get_num_touch_points();
    if (*num_touch_points != 0) {
      *x = x_;
      *y = y_;
    }
  }

  /// @brief Get the home button state
  /// @return True if the home button is pressed, false otherwise
  /// @note This is a cached value from the last update() call
  bool get_home_button_state() const { return home_button_pressed_; }

protected:
  static constexpr int CONTACT_SIZE = 8;
  static constexpr int MAX_CONTACTS = 5;
  static constexpr int MAX_KEYS = 4;
  static constexpr int CONFIG_MAX_LEN = 240;
  static constexpr int CONFIG_911_LEN = 186;
  static constexpr int CONFIG_967_LEN = 228;

  enum class Registers : uint16_t {
    COMMAND = 0x8040,
    CONFIG = 0x8047,
    SWITCH_1 = 0x804D,
    SWITCH_2 = 0x804E,
    REFRESH_RATE = 0x8056,
    KEY = 0x8093,
    DATA = 0x8140,
    POINT_INFO = 0x814E,
    POINTS = 0x814F,
    POINT_1 = 0x814F,
    POINT_2 = 0x8157,
    POINT_3 = 0x815F,
    POINT_4 = 0x8167,
    POINT_5 = 0x816F,
  };

#pragma pack(push, 1)

  // From Goodix library
  struct GTInfo {
    // 0x8140-0x814A
    char productId[4];
    uint16_t fwId;
    uint16_t xResolution;
    uint16_t yResolution;
    uint8_t vendorId;
  };

  struct GTPoint {
    // 0x814F-0x8156, ... 0x8176 (5 points)
    uint8_t trackId;
    uint16_t x;
    uint16_t y;
    uint16_t area;
    uint8_t reserved;
  };

  struct GTLevelConfig {
    uint8_t touch; // Threshold of touch grow out of nothing
    uint8_t leave; // Threshold of touch decrease to nothing
  };

  struct GTStylusConfig {
    uint8_t txGain;
    uint8_t rxGain;
    uint8_t dumpShift;
    GTLevelConfig level;
    uint8_t control; // Pen mode escape time out period (Unit: Sec)
  };

  struct GTFreqHoppingConfig {
    uint16_t hoppingBitFreq;
    uint8_t hoppingFactor;
  };

  struct GTKeyConfig {
    // Key position: 0-255 valid
    // 0 means no touch, it means independent touch key when 4 of the keys are 8 multiples
    uint8_t pos1;
    uint8_t pos2;
    uint8_t pos3;
    uint8_t pos4;
    uint8_t area;
    GTLevelConfig level;
    uint8_t sens12;
    uint8_t sens34;
    uint8_t restrain;
  };

  struct GTConfig {
    // start at 0x8047
    uint8_t configVersion;
    uint16_t xResolution;
    uint16_t yResolution;
    // 0x804C
    uint8_t touchNumber; // 3:0 Touch No.: 1~10

    // 7:6 Reserved, 5:4 Stretch rank, 3 X2Y, 2 Sito
    // 1:0 INT trig method: 00-rising, 01-falling, 02-low level, 03-high level enquiry
    uint8_t moduleSwitch1;
    uint8_t moduleSwitch2; // bit0 TouchKey
    uint8_t shakeCount;    // 3:0 Finger shake count
    // 0x8050
    // 7:6 First filter, 5:0 Normal filter (filtering value of original coordinate window,
    // coefficiency is 1)
    uint8_t filter;
    uint8_t largeTouch;
    uint8_t noiseReduction;
    GTLevelConfig screenLevel;

    uint8_t lowPowerControl; // Time to low power consumption (0~15s)
    uint8_t refreshRate;     // Coordinate report rate (Cycle: 5+N ms)
    uint8_t xThreshold;      // res
    uint8_t yThreshold;      // res
    uint8_t xSpeedLimit;     // res
    uint8_t ySpeedLimit;     // res
    uint8_t vSpace;          // 4bit top/bottom  (coefficient 32)
    uint8_t hSpace;          // 4bit left/right
    // 0x805D-0x8061
    uint8_t stretchRate; // Level of weak stretch (Strtch X/16 Pitch)
    uint8_t stretchR0;   // Interval 1 coefficient
    uint8_t stretchR1;   // Interval 2 coefficient
    uint8_t stretchR2;   // Interval 3 coefficient
    uint8_t stretchRM;   // All intervals base number

    uint8_t drvGroupANum;
    uint8_t drvGroupBNum;
    uint8_t sensorNum;
    uint8_t freqAFactor;
    uint8_t freqBFactor;
    // 0x8067
    uint16_t pannelBitFreq;    // Baseband of Driver group A\B (1526HZ<baseband<14600Hz)
    uint16_t pannelSensorTime; // res
    uint8_t pannelTxGain;
    uint8_t pannelRxGain;
    uint8_t pannelDumpShift;
    uint8_t drvFrameControl;
    // 0x806F - 0x8071
    uint8_t NC_2[3];
    GTStylusConfig stylusConfig;
    // 0x8078-0x8079
    uint8_t NC_3[2];
    uint8_t freqHoppingStart; // Frequency hopping start frequency (Unit: 2KHz, 50 means 100KHz )
    uint8_t freqHoppingEnd;   // Frequency hopping stop frequency (Unit: 2KHz, 150 means 300KHz )
    uint8_t noiseDetectTims;
    uint8_t hoppingFlag;
    uint8_t hoppingThreshold;

    uint8_t noiseThreshold;
    uint8_t NC_4[2];
    // 0x8082
    GTFreqHoppingConfig hoppingSegments[5];
    // 0x8091
    uint8_t NC_5[2];
    GTKeyConfig keys;
  };

#pragma pack(pop)

  std::atomic<bool> home_button_pressed_{false};
  std::atomic<uint8_t> num_touch_points_;
  std::atomic<uint16_t> x_;
  std::atomic<uint16_t> y_;
};
} // namespace espp

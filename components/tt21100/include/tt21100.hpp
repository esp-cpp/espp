#pragma once

#include <algorithm>
#include <functional>

#include "base_peripheral.hpp"
#include "touch.hpp"

namespace espp {
/// @brief Driver for the Tt21100 touch controller
///
/// \section tt21100_ex1 Example
/// \snippet tt21100_example.cpp tt21100 example
class Tt21100 : public BasePeripheral<>, public ITouchDevice {
public:
  /// @brief The default i2c address
  static constexpr uint8_t DEFAULT_ADDRESS = (0x24);

  /// @brief Configuration for the Tt21100 driver
  struct Config {
    BasePeripheral::write_fn write = nullptr; ///< Function for writing to the i2c device (unused)
    BasePeripheral::read_fn read;             ///< Function for reading from the i2c device
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Log level
  };

  /// @brief Constructor
  /// @param config The configuration for the driver
  explicit Tt21100(const Config &config)
      : BasePeripheral({.address = DEFAULT_ADDRESS, .read = config.read}, "Tt21100",
                       config.log_level) {
    std::error_code ec;
    init(ec);
    if (ec) {
      logger_.error("Failed to initialize: {}", ec.message());
    }
  }

  /// @brief Read the touch data
  /// @param ec The error code to set if an error occurs
  /// @return True if there is new touch data
  bool update(std::error_code &ec) {
    static uint16_t data_len;
    static uint8_t data[256];
    TouchState state{};

    // NOTE: this chip is weird, and even though we're reading a u16, we can't
    //       use the read_u16 since that function assumes the data is in little
    //       endian format, but this chip sends the data in big endian format meaning
    //       the bytes are swapped

    {
      std::lock_guard<std::recursive_mutex> lock(base_mutex_);

      read_many(reinterpret_cast<uint8_t *>(&data_len), 2, ec);
      if (ec) {
        logger_.error("Failed to read data length: {}", ec.message());
        return false;
      }

      logger_.debug("Data length: {}", data_len);

      if (data_len >= 0xff) {
        logger_.error("Invalid data length");
        ec = std::make_error_code(std::errc::io_error);
        return false;
      }

      read_many(data, data_len, ec);
      if (ec) {
        logger_.error("Failed to read data");
        return false;
      }
    }

    bool new_data = false;
    switch (data_len) {
    case 2:
      // no available data
      break;
    case 7:
    case 17:
    case 27: {
      // touch event
      const auto report_data = reinterpret_cast<const TouchReport *>(data);
      const auto *touch_records = reinterpret_cast<const TouchRecord *>(data + sizeof(TouchReport));
      state = touch_state();
      state.num_touch_points =
          std::min<uint8_t>(report_data->record_num, TouchState::MAX_TOUCH_POINTS);
      for (size_t i = 0; i < state.num_touch_points; i++) {
        state.points[i] = {.x = touch_records[i].x, .y = touch_records[i].y};
      }
      if (state.num_touch_points > 0) {
        logger_.debug("Touch event: #={}, [0]=({}, {})", state.num_touch_points, state.points[0].x,
                      state.points[0].y);
      }
      new_data = true;
      break;
    }
    case 14: {
      // button event
      const auto button_data = reinterpret_cast<const ButtonRecord *>(data);
      state = touch_state();
      state.btn_state = button_data->btn_val;
      auto btn_signal = button_data->btn_signal[0];
      logger_.debug("Button event({}): {}, {}", static_cast<int>(button_data->length),
                    state.btn_state, btn_signal);
      new_data = true;
      break;
    }
    default:
      break;
    }
    if (new_data) {
      std::lock_guard<std::recursive_mutex> lock(base_mutex_);
      touch_state_ = state;
    }
    return new_data;
  }

  /// @brief Get the cached touch state.
  /// @return The cached touch state as of the last update() call.
  TouchState touch_state() const override {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    return touch_state_;
  }

  /// @brief Whether the controller exposes a home button.
  /// @return True.
  bool has_home_button() const override { return true; }

  /// @brief Get the number of touch points
  /// @note This is the number of touch points that were present when the last
  ///       update() was called
  /// @return The number of touch points
  uint8_t get_num_touch_points() const { return touch_state().num_touch_points; }

  /// @brief Get the touch point data
  /// @note This is the touch point data that was present when the last
  ///       update() was called
  /// @param num_touch_points The number of touch points
  /// @param x The x position of the touch point
  /// @param y The y position of the touch point
  void get_touch_point(uint8_t *num_touch_points, uint16_t *x, uint16_t *y) const {
    auto state = touch_state();
    auto point = state.primary_point();
    *num_touch_points = state.num_touch_points;
    *x = point.x;
    *y = point.y;
  }

  /// @brief Get the state of the home button
  /// @note This is the state of the home button when the last update() was
  ///       called
  /// @return True if the home button is pressed
  uint8_t get_home_button_state() const { return touch_state().btn_state; }

protected:
  void init(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    logger_.debug("Initializing...");
    uint16_t reg_val = 0;
    static constexpr int max_tries = 10;
    int num_tries = 0;
    do {
      using namespace std::chrono_literals;
      read_many(reinterpret_cast<uint8_t *>(&reg_val), 2, ec);
      if (ec) {
        logger_.error("Failed to read...");
        return;
      }
      logger_.debug("reg_val: {:#04x}", reg_val);
      std::this_thread::sleep_for(20ms);
    } while (0x0002 != reg_val && ++num_tries < max_tries);
    if (num_tries >= max_tries) {
      logger_.warn("Reached max tries trying to read...");
    }
  }

  enum class Registers : uint8_t {
    TP_NUM = 0x01,
    X_POS = 0x02,
    Y_POS = 0x03,
  };

#pragma pack(push, 1)

  struct TouchRecord {
    uint8_t : 5;
    uint8_t touch_type : 3;
    uint8_t tip : 1;
    uint8_t event_id : 2;
    uint8_t touch_id : 5;
    uint16_t x;
    uint16_t y;
    uint8_t pressure;
    uint16_t major_axis_length;
    uint8_t orientation;
  };

  struct TouchReport {
    uint16_t data_len;
    uint8_t report_id;
    uint16_t time_stamp;
    uint8_t : 2;
    uint8_t large_object : 1;
    uint8_t record_num : 5;
    uint8_t report_counter : 2;
    uint8_t : 3;
    uint8_t noise_efect : 3;
    TouchRecord touch_record[0];
  };

  struct ButtonRecord {
    uint16_t length;     /*!< Always 14(0x000E) */
    uint8_t report_id;   /*!< Always 03h */
    uint16_t time_stamp; /*!< Number in units of 100 us */
    uint8_t btn_val;     /*!< Only use bit[0..3] */
    uint16_t btn_signal[4];
  };

#pragma pack(pop)

  TouchState touch_state_;
};
} // namespace espp

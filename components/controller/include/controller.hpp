#pragma once

#include <algorithm>
#include <mutex>
#include <numeric>

#include "driver/dedic_gpio.h"
#include "driver/gpio.h"

#include "base_component.hpp"
#include "joystick.hpp"

namespace espp {
/**
 * @brief Class for managing controller input.
 *
 * The controller can be configured to either use a digital d-pad or an analog
 * 2-axis joystick with select button.
 *
 * Digital configuration can support ABXY, start, select, and 4 digital
 * directional inputs.
 *
 * Anaolg Joystick Configuration can support ABXY, start, select, two axis
 * (analog) joystick, and joystick select button. It will also convert the
 * joystick analog values into digital d-pad buttons.
 *
 * \section controller_ex1 Digital Controller Example
 * \snippet controller_example.cpp digital controller example
 * \section controller_ex2 Analog Controller Example
 * \snippet controller_example.cpp analog controller example
 * \section controller_ex3 I2C Analog Controller Example
 * \snippet controller_example.cpp i2c analog controller example
 */
class Controller : public BaseComponent {
public:
  /**
   * @brief The buttons that the controller supports.
   */
  enum class Button : int {
    A = 0,
    B,
    X,
    Y,
    SELECT,
    START,
    UP,
    DOWN,
    LEFT,
    RIGHT,
    JOYSTICK_SELECT,
    LAST_UNUSED
  };

  /**
   * @brief Packed bit structure containing the state (pressed=1) of each
   *        button.
   */
  struct State {
    uint32_t a : 1;               ///< State of the A button
    uint32_t b : 1;               ///< State of the B button
    uint32_t x : 1;               ///< State of the X button
    uint32_t y : 1;               ///< State of the Y button
    uint32_t select : 1;          ///< State of the SELECT button
    uint32_t start : 1;           ///< State of the START button
    uint32_t up : 1;              ///< State of the UP button
    uint32_t down : 1;            ///< State of the DOWN button
    uint32_t left : 1;            ///< State of the LEFT button
    uint32_t right : 1;           ///< State of the RIGHT button
    uint32_t joystick_select : 1; ///< State of the Joystick Select button
  };

  /**
   * @brief Configuration for the controller to use d-pad only, no joystick.
   */
  struct DigitalConfig {
    bool active_low{true}; ///< Whether the buttons are active-low (default) or not.
    int gpio_a{-1};        ///< GPIO for the A button
    int gpio_b{-1};        ///< GPIO for the B button
    int gpio_x{-1};        ///< GPIO for the X button
    int gpio_y{-1};        ///< GPIO for the Y button
    int gpio_start{-1};    ///< GPIO for the START button
    int gpio_select{-1};   ///< GPIO for the SELECT button
    int gpio_up{-1};       ///< GPIO for the UP button
    int gpio_down{-1};     ///< GPIO for the DOWN button
    int gpio_left{-1};     ///< GPIO for the LEFT button
    int gpio_right{-1};    ///< GPIO for the RIGHT button
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Verbosity for the logger.
  };

  /**
   * @brief Configuration for the controller to be used with a joystick (which
   *        has a center-press / select button), no d-pad.
   * @note In this configuration, the controller will create and manage a
   *       joystick component to read the analog axes of the joystick and
   *       convert them into digital up/down/left/right signals as a virtual
   *       d-pad.
   */
  struct AnalogJoystickConfig {
    bool active_low{true};        ///< Whether the buttons are active-low (default) or not.
    int gpio_a{-1};               ///< GPIO for the A button
    int gpio_b{-1};               ///< GPIO for the B button
    int gpio_x{-1};               ///< GPIO for the X button
    int gpio_y{-1};               ///< GPIO for the Y button
    int gpio_start{-1};           ///< GPIO for the START button
    int gpio_select{-1};          ///< GPIO for the SELECT button
    int gpio_joystick_select{-1}; ///< GPIO for the JOYSTICK SELECT button
    espp::Joystick::Config joystick_config; ///< Configuration for the analog joystick which will be
                                            ///< used to read digital direction values.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Verbosity for the logger.
  };

  /**
   * @brief Configuration for the controller to be used with both a joystick (which
   *        has a center-press / select button), and a d-pad.
   * @note In this configuration, the controller will NOT create / manage a
   *       joystick component. The configured d-pad provides the digital
   *       directions so the analog joystick values should be retrieved from a
   *       separately managed joystick component.
   */
  struct DualConfig {
    bool active_low{true};        ///< Whether the buttons are active-low (default) or not.
    int gpio_a{-1};               ///< GPIO for the A button
    int gpio_b{-1};               ///< GPIO for the B button
    int gpio_x{-1};               ///< GPIO for the X button
    int gpio_y{-1};               ///< GPIO for the Y button
    int gpio_start{-1};           ///< GPIO for the START button
    int gpio_select{-1};          ///< GPIO for the SELECT button
    int gpio_up{-1};              ///< GPIO for the UP button
    int gpio_down{-1};            ///< GPIO for the DOWN button
    int gpio_left{-1};            ///< GPIO for the LEFT button
    int gpio_right{-1};           ///< GPIO for the RIGHT button
    int gpio_joystick_select{-1}; ///< GPIO for the JOYSTICK SELECT button
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Verbosity for the logger.
  };

  /**
   * @brief Create a Digital controller.
   */
  explicit Controller(const DigitalConfig &config)
      : BaseComponent("Digital Controller", config.log_level) {
    gpio_.assign((int)Button::LAST_UNUSED, -1);
    input_state_.assign((int)Button::LAST_UNUSED, false);
    gpio_[(int)Button::A] = config.gpio_a;
    gpio_[(int)Button::B] = config.gpio_b;
    gpio_[(int)Button::X] = config.gpio_x;
    gpio_[(int)Button::Y] = config.gpio_y;
    gpio_[(int)Button::START] = config.gpio_start;
    gpio_[(int)Button::SELECT] = config.gpio_select;
    gpio_[(int)Button::UP] = config.gpio_up;
    gpio_[(int)Button::DOWN] = config.gpio_down;
    gpio_[(int)Button::LEFT] = config.gpio_left;
    gpio_[(int)Button::RIGHT] = config.gpio_right;
    init_gpio(config.active_low);
  }

  /**
   * @brief Create an analog joystick controller.
   */
  explicit Controller(const AnalogJoystickConfig &config)
      : BaseComponent("Analog Joystick Controller", config.log_level)
      , joystick_(std::make_unique<espp::Joystick>(config.joystick_config)) {
    gpio_.assign((int)Button::LAST_UNUSED, -1);
    input_state_.assign((int)Button::LAST_UNUSED, false);
    gpio_[(int)Button::A] = config.gpio_a;
    gpio_[(int)Button::B] = config.gpio_b;
    gpio_[(int)Button::X] = config.gpio_x;
    gpio_[(int)Button::Y] = config.gpio_y;
    gpio_[(int)Button::START] = config.gpio_start;
    gpio_[(int)Button::SELECT] = config.gpio_select;
    gpio_[(int)Button::JOYSTICK_SELECT] = config.gpio_joystick_select;
    init_gpio(config.active_low);
  }

  /**
   * @brief Create a dual d-pad + analog joystick controller.
   */
  explicit Controller(const DualConfig &config)
      : BaseComponent("Dual Digital Controller", config.log_level) {
    gpio_.assign((int)Button::LAST_UNUSED, -1);
    input_state_.assign((int)Button::LAST_UNUSED, false);
    gpio_[(int)Button::A] = config.gpio_a;
    gpio_[(int)Button::B] = config.gpio_b;
    gpio_[(int)Button::X] = config.gpio_x;
    gpio_[(int)Button::Y] = config.gpio_y;
    gpio_[(int)Button::START] = config.gpio_start;
    gpio_[(int)Button::SELECT] = config.gpio_select;
    gpio_[(int)Button::UP] = config.gpio_up;
    gpio_[(int)Button::DOWN] = config.gpio_down;
    gpio_[(int)Button::LEFT] = config.gpio_left;
    gpio_[(int)Button::RIGHT] = config.gpio_right;
    gpio_[(int)Button::JOYSTICK_SELECT] = config.gpio_joystick_select;
    init_gpio(config.active_low);
  }

  /**
   * @brief Destroys the controller and deletes the associated dedicated GPIO
   *        bundle.
   */
  ~Controller() { dedic_gpio_del_bundle(gpio_bundle_); }

  /**
   * @brief Get the most recent state structure for the controller.
   * @return State structure for the inputs - updated when update() was last
   *         called.
   */
  State get_state() {
    logger_.debug("Returning state structure");
    std::scoped_lock<std::mutex> lk(state_mutex_);
    return State{
        .a = input_state_[(int)Button::A],
        .b = input_state_[(int)Button::B],
        .x = input_state_[(int)Button::X],
        .y = input_state_[(int)Button::Y],
        .select = input_state_[(int)Button::SELECT],
        .start = input_state_[(int)Button::START],
        .up = input_state_[(int)Button::UP],
        .down = input_state_[(int)Button::DOWN],
        .left = input_state_[(int)Button::LEFT],
        .right = input_state_[(int)Button::RIGHT],
        .joystick_select = input_state_[(int)Button::JOYSTICK_SELECT],
    };
  }

  /**
   * @brief Return true if the \p input was pressed, false otherwise.
   * @param input The Button of interest.
   * @return True if \p input was pressed last time update() was called.
   */
  bool is_pressed(const Button input) {
    std::scoped_lock<std::mutex> lk(state_mutex_);
    return input_state_[(int)input];
  }

  /**
   * @brief Read the current button values and update the internal input state
   *        accordingly.
   */
  void update() {
    logger_.debug("Reading gpio bundle");
    // read the updated state for configured gpios (all at once)
    uint32_t pin_state = dedic_gpio_bundle_read_in(gpio_bundle_);
    // when setting up the dedic gpio, we removed the gpio that were not
    // configured (-1) in our vector, but the returned bitmask simply orders
    // them (low bit is low member in originally provided vector) so we need to
    // track the actual bit corresponding to the pin in the pin_state.
    // and pull out the state into the vector accordingly
    logger_.debug("Parsing bundle state from pin state 0x{:04X}", pin_state);
    {
      std::scoped_lock<std::mutex> lk(state_mutex_);
      int bit = 0;
      for (int i = 0; i < gpio_.size(); i++) {
        auto gpio = gpio_[i];
        if (gpio != -1) {
          input_state_[i] = is_bit_set(pin_state, bit);
          // this pin is used, increment the bit index
          bit++;
        } else {
          input_state_[i] = false;
        }
      }
    }
    // now update the joystick if we have it
    if (joystick_) {
      logger_.debug("Updating joystick");
      joystick_->update();
      // now update the d-pad state if the joystick values are high enough
      float x = joystick_->x();
      float y = joystick_->y();
      logger_.debug("Got joystick x,y: ({},{})", x, y);
      std::scoped_lock<std::mutex> lk(state_mutex_);
      if (x > 0.5f) {
        input_state_[(int)Button::RIGHT] = true;
      } else if (x < -0.5f) {
        input_state_[(int)Button::LEFT] = true;
      }
      if (y > 0.5f) {
        input_state_[(int)Button::UP] = true;
      } else if (y < -0.5f) {
        input_state_[(int)Button::DOWN] = true;
      }
    }
  }

protected:
  bool is_bit_set(uint32_t data, int bit) { return (data & (1 << bit)) != 0; }

  void init_gpio(bool active_low) {
    // select only the gpios that are used (not -1)
    std::vector<int> actual_gpios;
    std::copy_if(gpio_.begin(), gpio_.end(), std::back_inserter(actual_gpios),
                 [](int gpio) { return gpio != -1; });

    uint64_t pin_mask =
        std::accumulate(actual_gpios.begin(), actual_gpios.end(), 0,
                        [](uint64_t mask, int gpio) { return mask | (1ULL << gpio); });
    gpio_config_t io_config = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = active_low ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = active_low ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_config));

    // Create gpio_bundle_, input only
    dedic_gpio_bundle_config_t gpio_bundle_config = {
        .gpio_array = actual_gpios.data(),
        .array_size = actual_gpios.size(),
        .flags =
            {
                .in_en = 1,
                .in_invert = (unsigned int)(active_low ? 1 : 0),
                .out_en = 0, // we _could_ enable input & output but we don't want to
                .out_invert = 0,
            },
    };
    ESP_ERROR_CHECK(dedic_gpio_new_bundle(&gpio_bundle_config, &gpio_bundle_));
  }

  std::mutex state_mutex_;
  std::vector<int> gpio_;
  std::vector<bool> input_state_;
  dedic_gpio_bundle_handle_t gpio_bundle_{NULL};
  std::unique_ptr<espp::Joystick> joystick_;
};
} // namespace espp

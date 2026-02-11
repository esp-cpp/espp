#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>

#include "base_component.hpp"

namespace espp {
/**
 * @brief Expressive Eyes Animation Component
 *
 * Renders animated expressive eyes using simple blob shapes. Eyes can blink, look around,
 * change expression, and display various emotions.
 *
 * Features:
 * - Smooth eye movement and blinking
 * - Multiple expressions (happy, sad, angry, surprised, etc.)
 * - Optional pupils with physics-based movement
 * - Eyebrows and cheeks for enhanced expressions
 * - Customizable colors and sizes
 * - Frame-based animation system
 *
 * \section eyes_ex1 Expressive Eyes Example
 * \snippet expressive_eyes_example.cpp expressive eyes example
 */
class ExpressiveEyes : public BaseComponent {
public:
  /**
   * @brief Pupil configuration
   */
  struct Pupil {
    bool enabled{true};     ///< Whether to draw pupils
    float size{0.3f};       ///< Pupil size relative to eye (0.0-1.0)
    uint16_t color{0x0000}; ///< Pupil color (black)
    float x{0.0f};          ///< Pupil X position (-1.0 to 1.0)
    float y{0.0f};          ///< Pupil Y position (-1.0 to 1.0)
  };

  /**
   * @brief Eyebrow configuration
   */
  struct Eyebrow {
    bool enabled{false};    ///< Whether to draw eyebrows
    float angle{0.0f};      ///< Eyebrow angle in degrees
    float height{0.0f};     ///< Vertical offset relative to eye (-1.0 to 1.0)
    float thickness{0.1f};  ///< Eyebrow thickness relative to eye
    float width{1.0f};      ///< Eyebrow width relative to eye
    uint16_t color{0x0000}; ///< Eyebrow color (black)
  };

  /**
   * @brief Cheek configuration (for shaping eye bottom)
   */
  struct Cheek {
    bool enabled{false}; ///< Whether to draw cheeks
    float size{0.3f};    ///< Cheek size relative to eye
  };

  /**
   * @brief Complete expression state
   */
  struct ExpressionState {
    float eye_width_scale{1.0f};  ///< Eye width multiplier
    float eye_height_scale{1.0f}; ///< Eye height multiplier
    float eye_rotation{0.0f};     ///< Eye rotation in radians
    float top_curve{0.5f};        ///< Top eyelid curve (0=flat, 1=round)
    float bottom_curve{0.5f};     ///< Bottom eyelid curve (0=flat, 1=round)
    float eye_offset_y{0.0f};     ///< Vertical offset for eye position
    float cheek_offset_y{0.0f};   ///< Vertical offset for cheek position
    Pupil pupil;                  ///< Pupil configuration
    Eyebrow eyebrow;              ///< Eyebrow configuration
    Cheek cheek;                  ///< Cheek configuration
  };

  /**
   * @brief Single eye render data
   */
  struct EyeState {
    int x;                      ///< X coordinate of eye center
    int y;                      ///< Y coordinate of eye center
    int width;                  ///< Current eye width
    int height;                 ///< Current eye height
    ExpressionState expression; ///< Expression state for this eye
  };

  /**
   * @brief Draw callback function
   * @param left_eye Left eye render data
   * @param right_eye Right eye render data
   */
  typedef std::function<void(const EyeState &left_eye, const EyeState &right_eye)> draw_callback;

  /**
   * @brief Eye expression presets
   */
  enum class Expression {
    NEUTRAL,    ///< Normal open eyes
    HAPPY,      ///< Squinted happy eyes with raised cheeks
    SAD,        ///< Droopy sad eyes with angled eyebrows
    ANGRY,      ///< Angled angry eyes with furrowed eyebrows
    SURPRISED,  ///< Wide open eyes with raised eyebrows
    SLEEPY,     ///< Droopy half-closed eyes with low eyebrows
    BORED,      ///< Half-closed eyes with neutral expression
    WINK_LEFT,  ///< Left eye closed
    WINK_RIGHT, ///< Right eye closed
  };

  /**
   * @brief Configuration for expressive eyes
   */
  struct Config {
    int screen_width{320};                                ///< Screen width in pixels
    int screen_height{240};                               ///< Screen height in pixels
    int eye_spacing{100};                                 ///< Distance between eye centers
    int eye_width{60};                                    ///< Base eye width
    int eye_height{80};                                   ///< Base eye height
    float blink_duration{0.12f};                          ///< Blink duration in seconds
    float blink_interval{4.0f};                           ///< Average time between blinks
    bool enable_auto_blink{true};                         ///< Automatic random blinking
    bool enable_pupil_physics{true};                      ///< Smooth pupil movement
    draw_callback on_draw{nullptr};                       ///< Drawing callback
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log verbosity
  };

  /**
   * @brief Construct expressive eyes
   * @param config Configuration structure
   */
  explicit ExpressiveEyes(const Config &config);

  /**
   * @brief Update animation (call this every frame)
   * @param dt Delta time since last update in seconds
   */
  void update(float dt);

  /**
   * @brief Set target look direction
   * @param x Horizontal look direction (-1.0 to 1.0, 0=center)
   * @param y Vertical look direction (-1.0 to 1.0, 0=center)
   */
  void look_at(float x, float y);

  /**
   * @brief Set expression
   * @param expr Expression to display
   */
  void set_expression(Expression expr);

  /**
   * @brief Trigger a blink
   */
  void blink();

  /**
   * @brief Get current expression
   * @return Current expression
   */
  Expression get_expression() const { return current_expression_; }

  /**
   * @brief Get preset expression state
   * @param expr Expression preset
   * @return Expression state configuration
   */
  static ExpressionState get_preset_expression(Expression expr);

protected:
  void init(const Config &config);
  void update_blink(float dt);
  void update_pupils(float dt);
  void update_expression(float dt);
  void draw_eyes();
  void blend_expression_states(ExpressionState &result, const ExpressionState &from,
                               const ExpressionState &to, float t);

  float lerp(float a, float b, float t) { return a + (b - a) * t; }

  Config config_;
  Expression source_expression_{Expression::NEUTRAL}; ///< Source expression for blending
  Expression current_expression_{Expression::NEUTRAL};
  Expression target_expression_{Expression::NEUTRAL};
  ExpressionState current_state_;
  ExpressionState target_state_;
  float expression_blend_{0.0f};

  // Eye state
  float blink_state_{0.0f}; // 0=open, 1=closed
  float blink_timer_{0.0f};
  float next_blink_time_{4.0f};
  bool is_blinking_{false};

  // Look direction
  float target_look_x_{0.0f};
  float target_look_y_{0.0f};
  float current_look_x_{0.0f};
  float current_look_y_{0.0f};

  // Pupil physics
  float pupil_velocity_x_{0.0f};
  float pupil_velocity_y_{0.0f};

  // Redraw throttling
  float redraw_timer_{0.0f};
};
} // namespace espp

// for libfmt formatting of ExpressiveEyes::Expression
template <> struct fmt::formatter<espp::ExpressiveEyes::Expression> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::ExpressiveEyes::Expression &expr, FormatContext &ctx) const {
    std::string name;
    switch (expr) {
    case espp::ExpressiveEyes::Expression::NEUTRAL:
      name = "NEUTRAL";
      break;
    case espp::ExpressiveEyes::Expression::HAPPY:
      name = "HAPPY";
      break;
    case espp::ExpressiveEyes::Expression::SAD:
      name = "SAD";
      break;
    case espp::ExpressiveEyes::Expression::ANGRY:
      name = "ANGRY";
      break;
    case espp::ExpressiveEyes::Expression::SURPRISED:
      name = "SURPRISED";
      break;
    case espp::ExpressiveEyes::Expression::SLEEPY:
      name = "SLEEPY";
      break;
    case espp::ExpressiveEyes::Expression::BORED:
      name = "BORED";
      break;
    case espp::ExpressiveEyes::Expression::WINK_LEFT:
      name = "WINK_LEFT";
      break;
    case espp::ExpressiveEyes::Expression::WINK_RIGHT:
      name = "WINK_RIGHT";
      break;
    default:
      name = "UNKNOWN";
      break;
    }
    return fmt::format_to(ctx.out(), "{}", name);
  }
};

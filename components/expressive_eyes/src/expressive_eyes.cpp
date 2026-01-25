#include "expressive_eyes.hpp"

#include <esp_heap_caps.h>
#include <random>

using namespace espp;

ExpressiveEyes::ExpressiveEyes(const Config &config)
    : BaseComponent("ExpressiveEyes", config.log_level) {
  init(config);
}

void ExpressiveEyes::init(const Config &config) {
  config_ = config;

  // Initialize with neutral expression
  current_state_ = get_preset_expression(Expression::NEUTRAL);
  target_state_ = current_state_;

  // Randomize initial blink time
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dist(2.0f, config_.blink_interval * 1.5f);
  next_blink_time_ = dist(gen);

  logger_.info("Initialized: {}x{}, eye_size={}x{}", config_.screen_width, config_.screen_height,
               config_.eye_width, config_.eye_height);
}

void ExpressiveEyes::update(float dt) {
  update_blink(dt);
  update_pupils(dt);
  update_expression(dt);

  // Limit redraw rate to avoid watchdog
  static constexpr float MIN_REDRAW_INTERVAL = 0.033f; // ~30 FPS max
  redraw_timer_ += dt;

  if (redraw_timer_ >= MIN_REDRAW_INTERVAL) {
    redraw_timer_ = 0.0f;
    draw_eyes();
  }
}

void ExpressiveEyes::update_blink(float dt) {
  if (config_.enable_auto_blink) {
    blink_timer_ += dt;

    if (!is_blinking_ && blink_timer_ >= next_blink_time_) {
      blink();
    }
  }

  if (is_blinking_) {
    blink_state_ += dt / config_.blink_duration;

    if (blink_state_ >= 2.0f) {
      // Blink complete
      blink_state_ = 0.0f;
      is_blinking_ = false;
      blink_timer_ = 0.0f;

      // Random next blink time
      static std::random_device rd;
      static std::mt19937 gen(rd());
      std::uniform_real_distribution<float> dist(config_.blink_interval * 0.5f,
                                                 config_.blink_interval * 1.5f);
      next_blink_time_ = dist(gen);
    }
  }
}

void ExpressiveEyes::update_pupils(float dt) {
  if (!current_state_.pupil.enabled)
    return;

  if (config_.enable_pupil_physics) {
    // Spring physics for smooth pupil movement with critical damping to prevent overshoot
    const float spring_strength = 40.0f; // Much stronger for faster response
    const float damping = 15.0f;         // Higher damping to prevent bounce

    float dx = target_look_x_ - current_look_x_;
    float dy = target_look_y_ - current_look_y_;

    // Stop motion if very close to target
    float distance = std::sqrt(dx * dx + dy * dy);
    if (distance < 0.005f) { // Tighter threshold
      current_look_x_ = target_look_x_;
      current_look_y_ = target_look_y_;
      pupil_velocity_x_ = 0.0f;
      pupil_velocity_y_ = 0.0f;
    } else {
      pupil_velocity_x_ += dx * spring_strength * dt - pupil_velocity_x_ * damping * dt;
      pupil_velocity_y_ += dy * spring_strength * dt - pupil_velocity_y_ * damping * dt;

      current_look_x_ += pupil_velocity_x_ * dt;
      current_look_y_ += pupil_velocity_y_ * dt;

      // Clamp to valid range
      current_look_x_ = clamp(current_look_x_, -1.0f, 1.0f);
      current_look_y_ = clamp(current_look_y_, -1.0f, 1.0f);
    }
  } else {
    // Direct movement
    current_look_x_ = target_look_x_;
    current_look_y_ = target_look_y_;
  }

  // Update pupil position in state
  current_state_.pupil.x = current_look_x_;
  current_state_.pupil.y = current_look_y_;
}

void ExpressiveEyes::update_expression(float dt) {
  if (current_expression_ != target_expression_) {
    // Start new blend - save current blended state as source
    source_expression_ = current_expression_;
    target_state_ = get_preset_expression(target_expression_);
    expression_blend_ = 0.0f;
    current_expression_ = target_expression_; // Update tracking immediately
  }

  // Blend towards target
  if (expression_blend_ < 1.0f) {
    expression_blend_ += dt * 8.0f; // Blend over ~0.125 seconds (fast, smooth transitions)
    if (expression_blend_ > 1.0f) {
      expression_blend_ = 1.0f;
    }

    ExpressionState from = get_preset_expression(source_expression_);
    // Save current pupil offset before blending
    float current_pupil_x = current_state_.pupil.x;
    float current_pupil_y = current_state_.pupil.y;
    blend_expression_states(current_state_, from, target_state_, expression_blend_);
    // Restore pupil offset (controlled by update_pupils, not expression blend)
    current_state_.pupil.x = current_pupil_x;
    current_state_.pupil.y = current_pupil_y;
  } else {
    // Blend complete, use target state but preserve pupil offset
    float current_pupil_x = current_state_.pupil.x;
    float current_pupil_y = current_state_.pupil.y;
    current_state_ = target_state_;
    current_state_.pupil.x = current_pupil_x;
    current_state_.pupil.y = current_pupil_y;
  }
}

void ExpressiveEyes::blend_expression_states(ExpressionState &result, const ExpressionState &from,
                                             const ExpressionState &to, float t) {
  result.eye_width_scale = lerp(from.eye_width_scale, to.eye_width_scale, t);
  result.eye_height_scale = lerp(from.eye_height_scale, to.eye_height_scale, t);
  result.eye_rotation = lerp(from.eye_rotation, to.eye_rotation, t);
  result.top_curve = lerp(from.top_curve, to.top_curve, t);
  result.bottom_curve = lerp(from.bottom_curve, to.bottom_curve, t);
  result.eye_offset_y = lerp(from.eye_offset_y, to.eye_offset_y, t);

  // Eyebrow blending - animate sliding in/out
  if (from.eyebrow.enabled && to.eyebrow.enabled) {
    // Both enabled - blend normally
    result.eyebrow.enabled = true;
    result.eyebrow.angle = lerp(from.eyebrow.angle, to.eyebrow.angle, t);
    result.eyebrow.height = lerp(from.eyebrow.height, to.eyebrow.height, t);
    result.eyebrow.thickness = lerp(from.eyebrow.thickness, to.eyebrow.thickness, t);
    result.eyebrow.width = lerp(from.eyebrow.width, to.eyebrow.width, t);
  } else if (!from.eyebrow.enabled && to.eyebrow.enabled) {
    // Fading in - slide down from above
    result.eyebrow.enabled = true;
    result.eyebrow.angle = to.eyebrow.angle;
    result.eyebrow.height = lerp(-1.2f, to.eyebrow.height, t); // Start above screen
    result.eyebrow.thickness = to.eyebrow.thickness;
    result.eyebrow.width = to.eyebrow.width;
  } else if (from.eyebrow.enabled && !to.eyebrow.enabled) {
    // Fading out - slide up above screen
    result.eyebrow.enabled = (t < 1.0f);
    result.eyebrow.angle = from.eyebrow.angle;
    result.eyebrow.height = lerp(from.eyebrow.height, -1.2f, t); // Slide up
    result.eyebrow.thickness = from.eyebrow.thickness;
    result.eyebrow.width = from.eyebrow.width;
  } else {
    // Both disabled
    result.eyebrow.enabled = false;
  }

  // Cheek blending - animate sliding in/out
  if (from.cheek.enabled && to.cheek.enabled) {
    // Both enabled - blend normally
    result.cheek.enabled = true;
    result.cheek.size = lerp(from.cheek.size, to.cheek.size, t);
    result.cheek_offset_y = lerp(from.cheek_offset_y, to.cheek_offset_y, t);
  } else if (!from.cheek.enabled && to.cheek.enabled) {
    // Fading in - slide up from below
    result.cheek.enabled = true;
    result.cheek.size = to.cheek.size;
    result.cheek_offset_y = lerp(1.2f, to.cheek_offset_y, t); // Start below screen
  } else if (from.cheek.enabled && !to.cheek.enabled) {
    // Fading out - slide down below screen
    result.cheek.enabled = (t < 1.0f);
    result.cheek.size = from.cheek.size;
    result.cheek_offset_y = lerp(from.cheek_offset_y, 1.2f, t); // Slide down
  } else {
    // Both disabled
    result.cheek.enabled = false;
  }

  // Keep pupil settings from target but use current look position
  result.pupil = to.pupil;
  result.pupil.x = current_look_x_;
  result.pupil.y = current_look_y_;
}

void ExpressiveEyes::draw_eyes() {
  if (!config_.on_draw)
    return;

  int center_x = config_.screen_width / 2;
  int center_y = config_.screen_height / 2;

  int left_x = center_x - config_.eye_spacing / 2;
  int right_x = center_x + config_.eye_spacing / 2;

  // Apply vertical offset from expression
  int eye_y = center_y + static_cast<int>(current_state_.eye_offset_y * config_.screen_height);

  // Apply eye position offset based on look direction
  // Scale the offset to be smaller than pupil movement (about 15% of eye width/height)
  int eye_offset_x = static_cast<int>(current_look_x_ * config_.eye_width * 0.15f);
  int eye_offset_y = static_cast<int>(current_look_y_ * config_.eye_height * 0.15f);

  left_x += eye_offset_x;
  right_x += eye_offset_x;
  eye_y += eye_offset_y;

  // Calculate blink effect (0=open, 1=closed)
  float blink_amount = 0.0f;
  if (is_blinking_) {
    if (blink_state_ <= 1.0f) {
      // Closing (0 -> 1)
      blink_amount = blink_state_;
    } else {
      // Opening (1 -> 0)
      blink_amount = 2.0f - blink_state_;
    }
  }

  // Compute final eye dimensions
  int eye_width = static_cast<int>(config_.eye_width * current_state_.eye_width_scale);
  int base_eye_height = static_cast<int>(config_.eye_height * current_state_.eye_height_scale);

  // Handle wink expressions - apply different blink amounts to each eye
  float left_blink = blink_amount;
  float right_blink = blink_amount;

  if (current_expression_ == Expression::WINK_LEFT) {
    left_blink = 0.9f;  // Almost closed
    right_blink = 0.0f; // Open
  } else if (current_expression_ == Expression::WINK_RIGHT) {
    left_blink = 0.0f;  // Open
    right_blink = 0.9f; // Almost closed
  }

  int left_eye_height = static_cast<int>(base_eye_height * (1.0f - left_blink));
  int right_eye_height = static_cast<int>(base_eye_height * (1.0f - right_blink));

  // Create state for each eye
  EyeState left_eye;
  left_eye.x = left_x;
  left_eye.y = eye_y;
  left_eye.width = eye_width;
  left_eye.height = left_eye_height;
  left_eye.expression = current_state_;

  EyeState right_eye;
  right_eye.x = right_x;
  right_eye.y = eye_y;
  right_eye.width = eye_width;
  right_eye.height = right_eye_height;
  right_eye.expression = current_state_;
  right_eye.expression.eye_rotation = -current_state_.eye_rotation; // Mirror rotation

  // Draw both eyes in single call
  config_.on_draw(left_eye, right_eye);
}

void ExpressiveEyes::look_at(float x, float y) {
  target_look_x_ = clamp(x, -1.0f, 1.0f);
  target_look_y_ = clamp(y, -1.0f, 1.0f);
}

void ExpressiveEyes::set_expression(Expression expr) {
  target_expression_ = expr;
  logger_.debug("Expression changed to: {}", static_cast<int>(expr));
}

void ExpressiveEyes::blink() {
  if (!is_blinking_) {
    is_blinking_ = true;
    blink_state_ = 0.0f;
    logger_.debug("Blinking");
  }
}

ExpressiveEyes::ExpressionState ExpressiveEyes::get_preset_expression(Expression expr) {
  ExpressionState state;

  // Default pupil - keep size constant across all expressions
  state.pupil.enabled = true;
  state.pupil.size = 0.3f; // Constant size
  state.pupil.color = 0x0000;

  switch (expr) {
  case Expression::NEUTRAL:
    state.eye_width_scale = 1.0f;
    state.eye_height_scale = 1.0f;
    state.eye_rotation = 0.0f;
    state.top_curve = 0.5f;
    state.bottom_curve = 0.5f;
    break;

  case Expression::HAPPY:
    state.eye_width_scale = 1.0f;  // Same size as normal
    state.eye_height_scale = 1.0f; // Same size as normal
    state.eye_rotation = 0.0f;
    state.top_curve = 0.5f;
    state.bottom_curve = 0.5f;
    state.eye_offset_y = -0.08f;  // Move eyes up slightly on face
    state.cheek_offset_y = 0.18f; // Move cheeks much lower (positive is down)
    // No eyebrows for happy (hidden by default)
    state.cheek.enabled = true; // Shape bottom with cheeks
    state.cheek.size = 0.8f;    // Wider, more prominent cheeks
    break;

  case Expression::SAD:
    state.eye_width_scale = 0.9f;
    state.eye_height_scale = 0.85f; // Slightly drooped
    state.eye_rotation = -0.25f;    // Downward curve
    state.top_curve = 0.3f;         // Flatter top
    state.bottom_curve = 0.7f;      // Rounder bottom
    state.eyebrow.enabled = true;
    state.eyebrow.angle = -0.436f; // Angled UP toward center (sad brows) - ~25 degrees in radians
    state.eyebrow.height = -0.35f;
    state.eyebrow.thickness = 0.1f;
    state.eyebrow.width = 1.0f;
    state.eyebrow.color = 0x0000;
    state.cheek.enabled = true; // Shape bottom with cheeks
    state.cheek.size = 0.6f;
    state.cheek_offset_y = 0.15f; // Position below eye
    break;

  case Expression::ANGRY:
    state.eye_width_scale = 0.85f; // Narrower
    state.eye_height_scale = 0.75f;
    state.eye_rotation = 0.4f; // Sharp inward angle
    state.top_curve = 0.2f;    // Angular top
    state.bottom_curve = 0.3f;
    state.eyebrow.enabled = true;
    state.eyebrow.angle =
        0.524f; // Angled DOWN toward center (angry furrowed brows) - ~30 degrees in radians
    state.eyebrow.height = -0.15f;   // Close to eyes
    state.eyebrow.thickness = 0.12f; // Thicker
    state.eyebrow.width = 0.9f;
    state.eyebrow.color = 0x0000;
    // Don't override pupil size
    break;

  case Expression::SURPRISED:
    state.eye_width_scale = 1.3f; // Wide open
    state.eye_height_scale = 1.4f;
    state.eye_rotation = 0.0f;
    state.top_curve = 0.7f;
    state.bottom_curve = 0.7f;
    state.eyebrow.enabled = false; // No eyebrows for surprised
    // Don't override pupil size
    break;

  case Expression::SLEEPY:
    state.eye_width_scale = 1.0f;
    state.eye_height_scale = 0.35f; // Eyes mostly closed, droopy
    state.eye_rotation = -0.1f;     // Slight downward droop
    state.top_curve = 0.3f;
    state.bottom_curve = 0.3f;
    state.pupil.enabled = true; // Keep pupils visible
    state.eyebrow.enabled = true;
    state.eyebrow.angle = 0.0f;      // Straight across
    state.eyebrow.height = -0.15f;   // Lower, almost touching eyes
    state.eyebrow.thickness = 0.18f; // Very thick, heavy looking
    state.eyebrow.width = 1.0f;
    state.eyebrow.color = 0x0000;
    break;

  case Expression::BORED:
    state.eye_width_scale = 1.0f;
    state.eye_height_scale = 0.5f; // Half-closed
    state.eye_rotation = 0.0f;
    state.top_curve = 0.3f;
    state.bottom_curve = 0.3f;
    state.pupil.enabled = true;
    // No eyebrows for bored - just half-closed neutral eyes
    break;

  case Expression::WINK_LEFT:
    // Base on neutral, blink applied per-eye in draw_eyes()
    state.eye_width_scale = 1.0f;
    state.eye_height_scale = 1.0f;
    state.eye_rotation = 0.0f;
    state.top_curve = 0.5f;
    state.bottom_curve = 0.5f;
    break;

  case Expression::WINK_RIGHT:
    // Base on neutral, blink applied per-eye in draw_eyes()
    state.eye_width_scale = 1.0f;
    state.eye_height_scale = 1.0f;
    state.eye_rotation = 0.0f;
    state.top_curve = 0.5f;
    state.bottom_curve = 0.5f;
    break;
  }

  return state;
}

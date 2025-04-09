#include "joystick.hpp"

using namespace espp;

espp::Joystick::Joystick(const espp::Joystick::Config &config)
    : BaseComponent("Joystick", config.log_level)
    , x_mapper_(config.x_calibration)
    , y_mapper_(config.y_calibration)
    , type_(config.type)
    , center_deadzone_radius_(config.center_deadzone_radius)
    , range_deadzone_(config.range_deadzone)
    , get_values_(config.get_values) {}

void espp::Joystick::set_type(espp::Joystick::Type type, float radius, float range_deadzone) {
  type_ = type;
  if (type_ == Type::CIRCULAR) {
    x_mapper_.set_center_deadband(0);
    y_mapper_.set_center_deadband(0);
    x_mapper_.set_range_deadband(0);
    y_mapper_.set_range_deadband(0);
  }
  set_center_deadzone_radius(radius);
  set_range_deadzone(range_deadzone);
}

espp::Joystick::Type espp::Joystick::type() const { return type_; }

void espp::Joystick::set_center_deadzone_radius(float radius) {
  center_deadzone_radius_ = std::clamp<float>(radius, 0, 1);
}

float espp::Joystick::center_deadzone_radius() const { return center_deadzone_radius_; }

void espp::Joystick::set_range_deadzone(float range_deadzone) {
  range_deadzone_ = std::clamp<float>(range_deadzone, 0, 1);
}

float espp::Joystick::range_deadzone() const { return range_deadzone_; }

void espp::Joystick::set_calibration(const espp::FloatRangeMapper::Config &x_calibration,
                                     const espp::FloatRangeMapper::Config &y_calibration,
                                     float center_deadzone_radius, float range_deadzone) {
  x_mapper_.configure(x_calibration);
  y_mapper_.configure(y_calibration);
  if (type_ == Type::CIRCULAR) {
    x_mapper_.set_center_deadband(0);
    y_mapper_.set_center_deadband(0);
    x_mapper_.set_range_deadband(0);
    y_mapper_.set_range_deadband(0);
  }
  set_center_deadzone_radius(center_deadzone_radius);
  set_range_deadzone(range_deadzone);
}

void espp::Joystick::update() {
  if (!get_values_) {
    logger_.error("No function provided with which to get values!");
    return;
  }
  float _x, _y;
  logger_.info("Getting x,y values");
  bool success = get_values_(&_x, &_y);
  if (!success) {
    logger_.error("Could not get values!");
    return;
  }
  logger_.debug("Got x,y values: ({}, {})", _x, _y);
  recalculate(_x, _y);
}

void espp::Joystick::update(float raw_x, float raw_y) { recalculate(raw_x, raw_y); }

float espp::Joystick::x() const { return position_.x(); }

float espp::Joystick::y() const { return position_.y(); }

const espp::Vector2f &espp::Joystick::position() const { return position_; }

const espp::Vector2f &espp::Joystick::raw() const { return raw_; }

void espp::Joystick::recalculate(float raw_x, float raw_y) {
  raw_.x(raw_x);
  raw_.y(raw_y);
  position_.x(x_mapper_.map(raw_x));
  position_.y(y_mapper_.map(raw_y));
  if (type_ == Type::CIRCULAR) {
    auto magnitude = position_.magnitude();
    if (magnitude < center_deadzone_radius_) {
      position_.x(0);
      position_.y(0);
    } else if (magnitude >= 1.0f - range_deadzone_) {
      position_ = position_.normalized();
    } else {
      const float magnitude_range = 1.0f - center_deadzone_radius_ - range_deadzone_;
      const float new_magnitude = (magnitude - center_deadzone_radius_) / magnitude_range;
      position_ = position_.normalized() * new_magnitude;
    }
  }
}

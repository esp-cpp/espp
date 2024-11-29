#include "esp-box.hpp"

using namespace espp;

EspBox::EspBox()
    : BaseComponent("EspBox") {
  detect();
}

void EspBox::detect() {
  // probe the internal i2c bus for the gt911 and the tt21100 if we find the
  // gt911, we will use it as the touch controller, and detect that the box is
  // BOX3. If we find the tt21100, we will use it as the touch controller, and
  // detect that the box is BOX. If we find neither, something is wrong.
  bool found_gt911 = internal_i2c_.probe_device(espp::Gt911::DEFAULT_ADDRESS_1) ||
                     internal_i2c_.probe_device(espp::Gt911::DEFAULT_ADDRESS_2);

  bool found_tt21100 = internal_i2c_.probe_device(espp::Tt21100::DEFAULT_ADDRESS);

  if (found_gt911) {
    box_type_ = BoxType::BOX3;
  } else if (found_tt21100) {
    box_type_ = BoxType::BOX;
  } else {
    logger_.warn("Could not detect box type, are you sure you're running on a box?");
    box_type_ = BoxType::UNKNOWN;
  }
  logger_.info("Detected box type: {}", box_type_);
  // now configure the pins based on the box type
  switch (box_type_) {
  case BoxType::BOX3:
    backlight_io = box3::backlight_io;
    reset_value = box3::reset_value;
    i2s_ws_io = box3::i2s_ws_io;
    touch_invert_x = box3::touch_invert_x;
    touch_interrupt_level = box3::touch_interrupt_level;
    touch_interrupt_type = box3::touch_interrupt_type;
    touch_interrupt_pullup_enabled = box3::touch_interrupt_pullup_enabled;
    break;
  case BoxType::BOX:
    backlight_io = box::backlight_io;
    reset_value = box::reset_value;
    i2s_ws_io = box::i2s_ws_io;
    touch_invert_x = box::touch_invert_x;
    touch_interrupt_level = box::touch_interrupt_level;
    touch_interrupt_type = box::touch_interrupt_type;
    touch_interrupt_pullup_enabled = box::touch_interrupt_pullup_enabled;
    break;
  default:
    break;
  }
  // now actually set the touch_interrupt_pin members:
  touch_interrupt_pin_.active_level = touch_interrupt_level;
  touch_interrupt_pin_.interrupt_type = touch_interrupt_type;
  touch_interrupt_pin_.pullup_enabled = touch_interrupt_pullup_enabled;
}

#include "vmu-pro.hpp"

using namespace espp;

////////////////////////
// Button Functions   //
////////////////////////

bool VmuPro::initialize_buttons(const VmuPro::button_callback_t &callback) {
  if (buttons_initialized_) {
    logger_.warn("Buttons already initialized, not initializing again!");
    return false;
  }

  logger_.info("Initializing buttons");

  // save the callback
  button_callback_ = callback;

  // configure the buttons
  for (const auto &pin_config : button_interrupt_pins_) {
    interrupts_.add_interrupt(pin_config);
  }
  buttons_initialized_ = true;
  return true;
}

bool VmuPro::button_state(VmuPro::Button button) const {
  if (!buttons_initialized_) {
    return false;
  }
  auto index = static_cast<size_t>(button);
  if (index >= button_interrupt_pins_.size()) {
    return false;
  }
  return interrupts_.is_active(button_interrupt_pins_[index]);
}

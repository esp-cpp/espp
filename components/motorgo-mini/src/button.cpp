#include "motorgo-mini.hpp"

using namespace espp;

////////////////////////
// Button Functions   //
////////////////////////

bool MotorGoMini::initialize_button(const MotorGoMini::button_callback_t &callback) {
  logger_.info("Initializing button");

  // save the callback
  button_callback_ = callback;

  // configure the button
  if (!button_initialized_) {
    interrupts_.add_interrupt(button_interrupt_pin_);
  }
  button_initialized_ = true;
  return true;
}

bool MotorGoMini::button_state() const {
  if (!button_initialized_) {
    return false;
  }
  return interrupts_.is_active(button_interrupt_pin_);
}

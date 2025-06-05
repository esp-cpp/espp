#include "byte90.hpp"

using namespace espp;

///////////////////////
// Button Functions  //
///////////////////////

bool Byte90::initialize_button(const Byte90::button_callback_t &callback) {
  logger_.info("Initializing button");

  // save the callback
  button_callback_ = callback;

  if (!button_initialized_) {
    // configure the button
    interrupts_.add_interrupt(button_interrupt_pin_);
  }

  button_initialized_ = true;
  return true;
}

bool Byte90::button_state() const {
  if (!button_initialized_) {
    return false;
  }
  return interrupts_.is_active(button_interrupt_pin_);
}

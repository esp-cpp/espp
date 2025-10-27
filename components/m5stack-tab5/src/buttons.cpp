#include "m5stack-tab5.hpp"

namespace espp {

bool M5StackTab5::initialize_button(const button_callback_t &callback) {
  logger_.info("Initializing button");

  button_callback_ = callback;

  // Add interrupt to the interrupt manager
  interrupts_.add_interrupt(button_interrupt_pin_);

  logger_.info("Button initialized successfully");
  return true;
}

bool M5StackTab5::button_state() const {
  // Read the current state of the reset button
  return interrupts_.is_active(button_interrupt_pin_);
}

} // namespace espp

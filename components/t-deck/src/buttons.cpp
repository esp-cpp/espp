#include "t-deck.hpp"

using namespace espp;

////////////////////////
// Button Functions   //
////////////////////////

bool TDeck::initialize_boot_button(const TDeck::button_callback_t &callback) {
  logger_.info("Initializing boot button");

  // save the callback
  boot_button_callback_ = callback;

  // configure the button
  if (!boot_button_initialized_) {
    interrupts_.add_interrupt(boot_button_interrupt_pin_);
  }
  boot_button_initialized_ = true;
  return true;
}

bool TDeck::boot_button_state() const {
  if (!boot_button_initialized_) {
    return false;
  }
  return interrupts_.is_active(boot_button_interrupt_pin_);
}

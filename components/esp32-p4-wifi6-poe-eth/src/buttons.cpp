#include "esp32-p4-wifi6-poe-eth.hpp"

using namespace espp;

////////////////////////
// Button Functions   //
////////////////////////

bool Esp32P4Wifi6PoeEth::initialize_boot_button(
    const Esp32P4Wifi6PoeEth::button_callback_t &callback) {
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

bool Esp32P4Wifi6PoeEth::boot_button_state() const {
  if (!boot_button_initialized_) {
    return false;
  }
  return interrupts_.is_active(boot_button_interrupt_pin_);
}

#include "lilygo-t5-47.hpp"

namespace espp {

namespace {
// epdiy's epd_init() installs the GPIO ISR service (and aborts if it is already
// installed), so it must be the one to install it. espp::Interrupt would
// otherwise try to install it again on construction; this subclass lets us set
// its (protected) "already installed" flag so it skips the install and just
// adds handlers to epdiy's service.
struct IsrMarker : public espp::Interrupt {
  static void mark_isr_installed() { ISR_SERVICE_INSTALLED = true; }
};
} // namespace

bool LilyGoT547::ensure_interrupts() {
  if (!initialized_) {
    logger_.error("initialize_display() must be called first: epdiy owns the GPIO ISR service");
    return false;
  }
  if (interrupts_) {
    return true;
  }
  // epd_init() already installed the GPIO ISR service; tell espp::Interrupt so
  // it does not try to install it again.
  IsrMarker::mark_isr_installed();
  interrupts_ = std::make_unique<espp::Interrupt>(espp::Interrupt::Config{
      .interrupts = {},
      .event_queue_size = 20,
      .task_config = {
          .name = "lilygo-t5-47 interrupts", .stack_size_bytes = 6 * 1024, .priority = 10}});
  return interrupts_ != nullptr;
}

bool LilyGoT547::initialize_button(const button_callback_t &callback) {
  if (!ensure_interrupts()) {
    return false;
  }
  logger_.info("Initializing BOOT button (GPIO{})", static_cast<int>(button_io));
  button_callback_ = callback;
  if (!button_initialized_) {
    interrupts_->add_interrupt(button_interrupt_pin_);
    button_initialized_ = true;
  }
  return true;
}

bool LilyGoT547::button_state() const {
  if (!button_initialized_ || !interrupts_) {
    return false;
  }
  return interrupts_->is_active(button_interrupt_pin_);
}

} // namespace espp

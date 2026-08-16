#include "lilygo-t5-47.hpp"

namespace espp {

void LilyGoT547::set_frontlight(bool on) {
  if (!frontlight_initialized_) {
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << static_cast<uint32_t>(frontlight_io);
    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&cfg);
    frontlight_initialized_ = true;
  }
  // BL_EN is assumed active-high (flagged for hardware verification).
  gpio_set_level(frontlight_io, on ? 1 : 0);
  frontlight_on_ = on;
  logger_.debug("Frontlight {}", on ? "on" : "off");
}

} // namespace espp

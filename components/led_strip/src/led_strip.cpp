#include "led_strip.hpp"

const std::vector<uint8_t> espp::LedStrip::APA102_START_FRAME{0x00, 0x00, 0x00, 0x00};
const std::vector<uint8_t> espp::LedStrip::APA102_END_FRAME{0xFF, 0xFF, 0xFF, 0xFF};

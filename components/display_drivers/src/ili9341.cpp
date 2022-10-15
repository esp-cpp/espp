#include "ili9341.hpp"

espp::Display::write_fn espp::Ili9341::lcd_write_;
gpio_num_t espp::Ili9341::reset_pin_;
gpio_num_t espp::Ili9341::dc_pin_;
gpio_num_t espp::Ili9341::backlight_pin_;

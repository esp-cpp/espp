#include "st7789.hpp"

espp::Display::write_fn espp::St7789::lcd_write_;
gpio_num_t espp::St7789::reset_pin_;
gpio_num_t espp::St7789::dc_pin_;
gpio_num_t espp::St7789::backlight_pin_;
int espp::St7789::offset_x_;
int espp::St7789::offset_y_;

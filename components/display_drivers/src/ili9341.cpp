#include "ili9341.hpp"

espp::display_drivers::write_fn espp::Ili9341::lcd_write_;
espp::display_drivers::send_lines_fn espp::Ili9341::lcd_send_lines_;
gpio_num_t espp::Ili9341::reset_pin_;
gpio_num_t espp::Ili9341::dc_pin_;
gpio_num_t espp::Ili9341::backlight_pin_;
int espp::Ili9341::offset_x_;
int espp::Ili9341::offset_y_;

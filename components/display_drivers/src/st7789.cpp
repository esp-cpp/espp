#include "st7789.hpp"

espp::display_drivers::write_fn espp::St7789::lcd_write_;
espp::display_drivers::send_lines_fn espp::St7789::lcd_send_lines_;
gpio_num_t espp::St7789::reset_pin_;
gpio_num_t espp::St7789::dc_pin_;
int espp::St7789::offset_x_;
int espp::St7789::offset_y_;

#include "gc9a01.hpp"

espp::display_drivers::write_fn espp::Gc9a01::lcd_write_;
espp::display_drivers::send_lines_fn espp::Gc9a01::lcd_send_lines_;
gpio_num_t espp::Gc9a01::reset_pin_;
gpio_num_t espp::Gc9a01::dc_pin_;
int espp::Gc9a01::offset_x_;
int espp::Gc9a01::offset_y_;

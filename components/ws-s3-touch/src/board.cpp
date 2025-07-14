#include "ws-s3-touch.hpp"

using namespace espp;

WsS3Touch::WsS3Touch()
    : BaseComponent("WsS3Touch") {
  enable();
}

bool WsS3Touch::enable() {
  logger_.info("Enabling S3 Touch");
  // set sys_enable_io to output, sys_sense_io to input, and enable output of
  // sys_enable_io
  gpio_set_direction(sys_enable_io, GPIO_MODE_OUTPUT);
  gpio_set_direction(sys_sense_io, GPIO_MODE_INPUT);
  gpio_set_level(sys_enable_io, 1);
  return true;
}

bool WsS3Touch::disable() {
  logger_.info("Disabling S3 Touch");
  // set sys_enable_io to low, sys_sense_io remains input
  gpio_set_level(sys_enable_io, 0);
  return true;
}

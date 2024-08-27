#include <chrono>
#include <deque>
#include <stdlib.h>
#include <vector>

#include "esp32-timer-cam.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "ESP32 TimerCam Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [esp timer cam example]
  espp::EspTimerCam &timer_cam = espp::EspTimerCam::get();

  // initialize the LED
  static constexpr float led_breathing_period = 3.5f;
  if (!timer_cam.initialize_led(led_breathing_period)) {
    logger.error("Failed to initialize LED!");
    return;
  }

  // initialize the rtc
  if (!timer_cam.initialize_rtc()) {
    logger.error("Failed to initialize RTC!");
    return;
  }

  // start the LED breathing
  timer_cam.start_led_breathing();

  // loop forever
  while (true) {
    // print out the battery voltage
    logger.info("Battery voltage: {:.02f} V", timer_cam.get_battery_voltage());
    std::this_thread::sleep_for(100ms);
    // go up a line to overwrite the previous message
    logger.move_up();
    logger.clear_line();
  }
  //! [esp timer cam example]
}

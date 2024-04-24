#include "gfps.hpp"

static espp::Logger logger({.tag = "GFPS OS", .level = espp::gfps::LOG_LEVEL});

static std::chrono::system_clock::time_point s_start_time;

std::vector<espp::HighResolutionTimer *> s_timers_to_delete;

void nearby_platform_Cleanup() {
  logger.debug("cleaning up {} timers", s_timers_to_delete.size());
  for (auto t : s_timers_to_delete) {
    delete t;
  }
  s_timers_to_delete.clear();
}

/////////////////PLATFORM///////////////////////

// Gets current time in ms.
unsigned int nearby_platform_GetCurrentTimeMs() {
  auto now = std::chrono::system_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - s_start_time);
  return elapsed.count();
}

// Starts a timer. Returns an opaque timer handle or null on error.
//
// callback - Function to call when timer matures.
// delay_ms - Number of milliseconds to run the timer.
void *nearby_platform_StartTimer(void (*callback)(), unsigned int delay_ms) {
  logger.debug("starting timer with delay {} ms", delay_ms);
  auto *timer = new espp::HighResolutionTimer(espp::HighResolutionTimer::Config{
      .name = "gfps timer",
      .callback = callback,
      .log_level = espp::gfps::LOG_LEVEL,
  });
  if (timer == nullptr) {
    logger.error("failed to create timer");
    return nullptr;
  }
  uint64_t delay_us = uint64_t(delay_ms) * uint64_t(1000);
  if (!timer->oneshot(delay_us)) {
    logger.error("failed to start timer");
    delete timer;
    return nullptr;
  }
  return timer;
}

// Cancels a timer
//
// timer - Timer handle returned by StartTimer.
nearby_platform_status nearby_platform_CancelTimer(void *timer) {
  // clean up any timers that should be deleted
  nearby_platform_Cleanup();

  // add this timer to the list of timers to delete
  auto *t = (espp::HighResolutionTimer *)timer;
  if (!t)
    return kNearbyStatusError;
  logger.debug("canceling timer");
  // NOTE: this function is actually called from within the timer's callback, so
  //       we can't cancel/delete it here. Instead, we'll add it to a list of
  //       timers to cancel/delete later.
  s_timers_to_delete.push_back(t);
  return kNearbyStatusOK;
}

// Initializes OS module
nearby_platform_status nearby_platform_OsInit() {
  s_start_time = std::chrono::system_clock::now();
  return kNearbyStatusOK;
}

// Starts ringing
//
// `command` - the requested ringing state as a bitmap:
// Bit 1 (0x01): ring right
// Bit 2 (0x02): ring left
// Bit 3 (0x04): ring case
// Alternatively, `command` hold a special value of 0xFF to ring all
// components that can ring.
// `timeout` - the timeout in deciseconds. The timeout overrides the one already
// in effect if any component of the device is already ringing.
nearby_platform_status nearby_platform_Ring(uint8_t command, uint16_t timeout) {
  // TODO: implement
  return kNearbyStatusOK;
}

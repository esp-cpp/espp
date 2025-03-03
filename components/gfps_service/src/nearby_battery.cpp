#include "gfps.hpp"

// Gets battery and charging info
//
// battery_info - Battery status structure.
nearby_platform_status nearby_platform_GetBatteryInfo(nearby_platform_BatteryInfo *) {
  // TODO: Implement
  return kNearbyStatusOK;
}

// Initializes battery module
//
// battery_interface - Battery status callback events.
nearby_platform_status nearby_platform_BatteryInit(nearby_platform_BatteryInterface *) {
  // TODO: Implement
  return kNearbyStatusOK;
}

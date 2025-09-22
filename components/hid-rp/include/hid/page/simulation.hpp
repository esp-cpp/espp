#ifndef __HID_PAGE_SIMULATION_HPP_
#define __HID_PAGE_SIMULATION_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class simulation : std::uint8_t;
template <> constexpr inline auto get_info<simulation>() {
  return info(
      0x0002, 0x00d0, "Simulation Controls",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Flight Simulation Device";
        case 0x0002:
          return "Automobile Simulation Device";
        case 0x0003:
          return "Tank Simulation Device";
        case 0x0004:
          return "Spaceship Simulation Device";
        case 0x0005:
          return "Submarine Simulation Device";
        case 0x0006:
          return "Sailing Simulation Device";
        case 0x0007:
          return "Motorcycle Simulation Device";
        case 0x0008:
          return "Sports Simulation Device";
        case 0x0009:
          return "Airplane Simulation Device";
        case 0x000a:
          return "Helicopter Simulation Device";
        case 0x000b:
          return "Magic Carpet Simulation Device";
        case 0x000c:
          return "Bicycle Simulation Device";
        case 0x0020:
          return "Flight Control Stick";
        case 0x0021:
          return "Flight Stick";
        case 0x0022:
          return "Cyclic Control";
        case 0x0023:
          return "Cyclic Trim";
        case 0x0024:
          return "Flight Yoke";
        case 0x0025:
          return "Track Control";
        case 0x00b0:
          return "Aileron";
        case 0x00b1:
          return "Aileron Trim";
        case 0x00b2:
          return "Anti-Torque Control";
        case 0x00b3:
          return "Autopilot Enable";
        case 0x00b4:
          return "Chaff Release";
        case 0x00b5:
          return "Collective Control";
        case 0x00b6:
          return "Dive Brake";
        case 0x00b7:
          return "Electronic Countermeasures";
        case 0x00b8:
          return "Elevator";
        case 0x00b9:
          return "Elevator Trim";
        case 0x00ba:
          return "Rudder";
        case 0x00bb:
          return "Throttle";
        case 0x00bc:
          return "Flight Communications";
        case 0x00bd:
          return "Flare Release";
        case 0x00be:
          return "Landing Gear";
        case 0x00bf:
          return "Toe Brake";
        case 0x00c0:
          return "Trigger";
        case 0x00c1:
          return "Weapons Arm";
        case 0x00c2:
          return "Weapons Select";
        case 0x00c3:
          return "Wing Flaps";
        case 0x00c4:
          return "Accelerator";
        case 0x00c5:
          return "Brake";
        case 0x00c6:
          return "Clutch";
        case 0x00c7:
          return "Shifter";
        case 0x00c8:
          return "Steering";
        case 0x00c9:
          return "Turret Direction";
        case 0x00ca:
          return "Barrel Elevation";
        case 0x00cb:
          return "Dive Plane";
        case 0x00cc:
          return "Ballast";
        case 0x00cd:
          return "Bicycle Crank";
        case 0x00ce:
          return "Handle Bars";
        case 0x00cf:
          return "Front Brake";
        case 0x00d0:
          return "Rear Brake";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class simulation : std::uint8_t {
  FLIGHT_SIMULATION_DEVICE = 0x0001,
  AUTOMOBILE_SIMULATION_DEVICE = 0x0002,
  TANK_SIMULATION_DEVICE = 0x0003,
  SPACESHIP_SIMULATION_DEVICE = 0x0004,
  SUBMARINE_SIMULATION_DEVICE = 0x0005,
  SAILING_SIMULATION_DEVICE = 0x0006,
  MOTORCYCLE_SIMULATION_DEVICE = 0x0007,
  SPORTS_SIMULATION_DEVICE = 0x0008,
  AIRPLANE_SIMULATION_DEVICE = 0x0009,
  HELICOPTER_SIMULATION_DEVICE = 0x000a,
  MAGIC_CARPET_SIMULATION_DEVICE = 0x000b,
  BICYCLE_SIMULATION_DEVICE = 0x000c,
  FLIGHT_CONTROL_STICK = 0x0020,
  FLIGHT_STICK = 0x0021,
  CYCLIC_CONTROL = 0x0022,
  CYCLIC_TRIM = 0x0023,
  FLIGHT_YOKE = 0x0024,
  TRACK_CONTROL = 0x0025,
  AILERON = 0x00b0,
  AILERON_TRIM = 0x00b1,
  ANTI_TORQUE_CONTROL = 0x00b2,
  AUTOPILOT_ENABLE = 0x00b3,
  CHAFF_RELEASE = 0x00b4,
  COLLECTIVE_CONTROL = 0x00b5,
  DIVE_BRAKE = 0x00b6,
  ELECTRONIC_COUNTERMEASURES = 0x00b7,
  ELEVATOR = 0x00b8,
  ELEVATOR_TRIM = 0x00b9,
  RUDDER = 0x00ba,
  THROTTLE = 0x00bb,
  FLIGHT_COMMUNICATIONS = 0x00bc,
  FLARE_RELEASE = 0x00bd,
  LANDING_GEAR = 0x00be,
  TOE_BRAKE = 0x00bf,
  TRIGGER = 0x00c0,
  WEAPONS_ARM = 0x00c1,
  WEAPONS_SELECT = 0x00c2,
  WING_FLAPS = 0x00c3,
  ACCELERATOR = 0x00c4,
  BRAKE = 0x00c5,
  CLUTCH = 0x00c6,
  SHIFTER = 0x00c7,
  STEERING = 0x00c8,
  TURRET_DIRECTION = 0x00c9,
  BARREL_ELEVATION = 0x00ca,
  DIVE_PLANE = 0x00cb,
  BALLAST = 0x00cc,
  BICYCLE_CRANK = 0x00cd,
  HANDLE_BARS = 0x00ce,
  FRONT_BRAKE = 0x00cf,
  REAR_BRAKE = 0x00d0,
};
} // namespace hid::page

#endif // __HID_PAGE_SIMULATION_HPP_

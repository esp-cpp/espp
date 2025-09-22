#ifndef __HID_PAGE_ALL_HPP_
#define __HID_PAGE_ALL_HPP_

#include "arcade.hpp"
#include "auxiliary_display.hpp"
#include "barcode_scanner.hpp"
#include "battery_system.hpp"
#include "braille_display.hpp"
#include "button.hpp"
#include "camera.hpp"
#include "consumer.hpp"
#include "digitizer.hpp"
#include "eye_and_head_trackers.hpp"
#include "fido.hpp"
#include "game.hpp"
#include "generic_desktop.hpp"
#include "generic_device.hpp"
#include "haptics.hpp"
#include "keyboard_keypad.hpp"
#include "leds.hpp"
#include "lighting_and_illumination.hpp"
#include "magnetic_stripe_reader.hpp"
#include "medical_instruments.hpp"
#include "monitor.hpp"
#include "monitor_enumerated.hpp"
#include "ordinal.hpp"
#include "physical_input_device.hpp"
#include "power.hpp"
#include "scale.hpp"
#include "sensor.hpp"
#include "simulation.hpp"
#include "soc.hpp"
#include "sport.hpp"
#include "telephony.hpp"
#include "unicode.hpp"
#include "vesa_virtual.hpp"
#include "vr.hpp"

namespace hid::page {
constexpr inline auto get_page_info(page_id_t page_id) {
  switch (page_id) {
  case get_info<sensor>().page_id:
    return get_info<sensor>();
  case get_info<keyboard_keypad>().page_id:
    return get_info<keyboard_keypad>();
  case get_info<braille_display>().page_id:
    return get_info<braille_display>();
  case get_info<arcade>().page_id:
    return get_info<arcade>();
  case get_info<auxiliary_display>().page_id:
    return get_info<auxiliary_display>();
  case get_info<haptics>().page_id:
    return get_info<haptics>();
  case get_info<digitizer>().page_id:
    return get_info<digitizer>();
  case get_info<ordinal>().page_id:
    return get_info<ordinal>();
  case get_info<barcode_scanner>().page_id:
    return get_info<barcode_scanner>();
  case get_info<magnetic_stripe_reader>().page_id:
    return get_info<magnetic_stripe_reader>();
  case get_info<telephony>().page_id:
    return get_info<telephony>();
  case get_info<camera>().page_id:
    return get_info<camera>();
  case get_info<physical_input_device>().page_id:
    return get_info<physical_input_device>();
  case get_info<vr>().page_id:
    return get_info<vr>();
  case get_info<soc>().page_id:
    return get_info<soc>();
  case get_info<generic_desktop>().page_id:
    return get_info<generic_desktop>();
  case get_info<scale>().page_id:
    return get_info<scale>();
  case get_info<sport>().page_id:
    return get_info<sport>();
  case get_info<lighting_and_illumination>().page_id:
    return get_info<lighting_and_illumination>();
  case get_info<power>().page_id:
    return get_info<power>();
  case get_info<eye_and_head_trackers>().page_id:
    return get_info<eye_and_head_trackers>();
  case get_info<monitor_enumerated>().page_id:
    return get_info<monitor_enumerated>();
  case get_info<simulation>().page_id:
    return get_info<simulation>();
  case get_info<game>().page_id:
    return get_info<game>();
  case get_info<consumer>().page_id:
    return get_info<consumer>();
  case get_info<unicode>().page_id:
    return get_info<unicode>();
  case get_info<leds>().page_id:
    return get_info<leds>();
  case get_info<monitor>().page_id:
    return get_info<monitor>();
  case get_info<medical_instruments>().page_id:
    return get_info<medical_instruments>();
  case get_info<fido>().page_id:
    return get_info<fido>();
  case get_info<button>().page_id:
    return get_info<button>();
  case get_info<vesa_virtual>().page_id:
    return get_info<vesa_virtual>();
  case get_info<generic_device>().page_id:
    return get_info<generic_device>();
  case get_info<battery_system>().page_id:
    return get_info<battery_system>();
  default:
    return get_info<hid::nullusage_t>();
  }
}
} // namespace hid::page

#endif // __HID_PAGE_ALL_HPP_

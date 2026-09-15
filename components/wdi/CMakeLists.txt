# Wheelchair Digital Interface (WDI) component.
#
# The protocol core (include/detail/wdi_protocol.hpp) is header-only and ESP-free
# (host-testable — see test/). Registering "include" alone makes both
# `#include "wdi.hpp"` and `#include "detail/wdi_protocol.hpp"` resolve for
# consumers (detail/ lives inside include/, as in the ota / odrive_native
# components).
#
# base_component is a public REQUIRES because the transport role classes
# (wdi_ble.hpp / wdi_usb.hpp and the host-side wrappers) derive from
# espp::BaseComponent; the protocol core and the WdiDevice / WdiHost cores are
# dependency-free. The transport-specific roles pull their own dependencies
# (usb_device for the USB device role; the USB Host HID + BLE stacks for the
# host / BLE roles) in their own translation units / examples.
idf_component_register(
  INCLUDE_DIRS "include"
  REQUIRES base_component
)

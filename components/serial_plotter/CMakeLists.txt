# The serial_plotter component ships:
#  - web/serial_plotter.html: the browser data plotter (Web Serial + WebUSB),
#    auto-hosted by the docs "web apps" pipeline and usable offline over file://.
#  - include/telemetry_service.hpp: espp::Telemetry, a binary telemetry emitter
#    (dispatcher module 3 over stream_frame) that the web app plots over WebUSB.
# The header only depends on base_component + stream_frame; an app pairs it with
# a Dispatcher and a transport (e.g. usb_device) — see the example.
idf_component_register(
  INCLUDE_DIRS "include"
  REQUIRES base_component stream_frame
)

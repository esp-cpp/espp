# serial_plotter currently ships only a self-contained browser webapp
# (web/serial_plotter.html), auto-hosted by the docs "web apps" pipeline and
# usable offline over file://. There is no firmware source yet: a telemetry
# service (headers + example) built on stream_frame / dispatcher is a planned
# follow-up. This registration keeps the directory a valid ESP-IDF component so
# repo tooling and the component registry stay consistent; it gains INCLUDE_DIRS
# and REQUIRES when the telemetry service lands.
idf_component_register()

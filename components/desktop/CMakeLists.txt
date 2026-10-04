# Header-only component (desktop.hpp, desktop_service.hpp, console_capture.hpp).
#
# REQUIRES are public since the headers include theirs:
#   base_component, task          — espp::Desktop is a BaseComponent running an espp::Task
#   format                        — fmt-style text helpers (Widget::set_text(fmt, ...))
#   stream_frame, dispatcher      — the framed service (header-only, ESP-free codec)
#   vfs, esp_driver_uart          — ConsoleCapture's write-only VFS device + the UART
#                                   console it tees to
idf_component_register(
  INCLUDE_DIRS "include"
  REQUIRES base_component task format stream_frame dispatcher vfs esp_driver_uart
)

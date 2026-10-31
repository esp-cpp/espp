# Header-only component (system_info.hpp, system_control.hpp, system_service.hpp).
#
# REQUIRES are public since the headers include theirs:
#   esp_system      — esp_system.h (esp_restart, esp_reset_reason, heap figures)
#   esp_app_format  — esp_app_desc.h (the embedded application description)
#   app_update      — esp_ota_ops.h (running / boot partition, OTA state)
#   esp_hw_support  — esp_chip_info.h, esp_mac.h, esp_clk_tree.h
#   esp_timer       — esp_timer.h (uptime)
#   spi_flash       — esp_flash.h (flash size)
#   esp_psram       — esp_psram.h (PSRAM size; empty component without PSRAM)
#   soc / esp_rom   — the always-on register + ROM USB persist calls behind
#                     SystemControl::reboot_to_bootloader()
#   format          — format.hpp (SystemInfo::to_string)
#   stream_frame, dispatcher — the framed service (header-only, ESP-free codec)
idf_component_register(
  INCLUDE_DIRS "include"
  REQUIRES base_component esp_system esp_app_format app_update esp_hw_support esp_timer spi_flash esp_psram soc esp_rom format stream_frame dispatcher
)

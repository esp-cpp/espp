# Display Component

[![Badge](https://components.espressif.com/components/espp/display/badge.svg)](https://components.espressif.com/components/espp/display)

The `display` component is responsible for managing the memory associated the
display buffer / video RAM (vram). It ensures that the LVGL subsystem is
configured appropriately.

The display buffers it allocates (`DynamicMemoryConfig`) are aligned to
`LV_DRAW_BUF_ALIGN`, and their size is rounded up to a multiple of it. LVGL
refuses display buffers that are not aligned to it, so buffers you provide
yourself (`StaticMemoryConfig`) must be aligned too; the display logs an error
if they are not. A raised `LV_DRAW_BUF_ALIGN` is needed, for example, by LVGL's
PPA draw unit on the ESP32-P4 (`LV_USE_PPA`), which requires it to equal the
data cache line size (`CONFIG_CACHE_L2_CACHE_LINE_SIZE`, 128 by default).

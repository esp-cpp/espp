#include "esp32-p4-function-ev-board.hpp"

#include <driver/gpio.h>

// Peripheral subsystems are split into separate translation units:
//   video.cpp     - MIPI-DSI display + LVGL
//   touchpad.cpp  - GT911 touch (polled)
//   audio.cpp     - ES8311 codec + I2S
//   sdcard.cpp    - 4-bit SDMMC
//   ethernet.cpp  - EMAC + IP101 RMII

namespace espp {

Esp32P4FunctionEvBoard::Esp32P4FunctionEvBoard()
    : BaseComponent("Esp32P4FunctionEvBoard") {
  // The display panel is selected at compile time via Kconfig.
#if CONFIG_ESP_P4_EV_BOARD_DISPLAY_ILI9881C
  display_controller_ = DisplayController::ILI9881C;
#else
  display_controller_ = DisplayController::EK79007;
#endif
}

bool Esp32P4FunctionEvBoard::initialize_button(const button_callback_t &callback) {
  // GPIO35 (button_io) is shared with Ethernet RMII TXD1 on this board. Claiming
  // it as a GPIO input would take down Ethernet TX, so refuse rather than break a
  // live network: Ethernet (system connectivity) outranks the UI button. Disable
  // Ethernet if you need the BOOT button.
  if (ethernet_initialized_) {
    logger_.error("BOOT button shares GPIO{} with Ethernet RMII TXD1; refusing to "
                  "initialize it while Ethernet is up (it would kill Ethernet TX).",
                  static_cast<int>(button_io));
    return false;
  }

  logger_.info("Initializing BOOT button on GPIO{}", static_cast<int>(button_io));
  button_callback_ = callback;
  interrupts_.add_interrupt(button_interrupt_pin_);

  return true;
}

bool Esp32P4FunctionEvBoard::button_state() const {
  // BOOT button is active-low
  return gpio_get_level(button_io) == 0;
}

bool Esp32P4FunctionEvBoard::initialize_camera() {
  // The MIPI-CSI camera (SC2336/OV5647, SCCB on the internal I2C bus, RST/XCLK
  // not connected) is wired in this BSP's pin map, but the esp_video capture
  // pipeline is not implemented yet. See the README for the camera status.
  logger_.warn("Camera (MIPI-CSI) is not yet implemented in this BSP");
  return false;
}

} // namespace espp

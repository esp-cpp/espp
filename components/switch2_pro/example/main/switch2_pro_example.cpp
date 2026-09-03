#include <chrono>
#include <thread>

#include "driver/gpio.h"
#include "nvs_flash.h"

#include "switch2_pro.hpp"

#include "logger.hpp"

using namespace std::chrono_literals;

// The BOOT button doubles as the A button for boards without dedicated buttons.
// It reads low when pressed. GPIO0 on Xtensa (S3/S2/classic); GPIO9 on the
// RISC-V chips (C6/C61/C3/C2/H2 devkits).
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
static constexpr gpio_num_t kBootButtonGpio = GPIO_NUM_0;
#else
static constexpr gpio_num_t kBootButtonGpio = GPIO_NUM_9;
#endif

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "switch2_pro example", .level = espp::Logger::Verbosity::INFO});

  // Bond persistence (LTK + console address) is stored in NVS so the controller
  // reconnects after a reboot without re-pairing.
  esp_err_t nvs_err = nvs_flash_init();
  if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }

  //! [switch2_pro example]
  // Bring up the emulated Switch 2 Pro Controller. init() verifies the pairing
  // crypto against a known-answer vector, builds the custom Nintendo GATT
  // services, configures security so the console (not BLE SMP) drives pairing,
  // and starts advertising with Nintendo manufacturer data.
  //
  // INFO shows the high-level protocol flow (connect, pairing steps, subscribes,
  // encryption, input-stream enable). Use DEBUG to also dump every command/
  // response byte — but note that flood can saturate the serial link during the
  // rapid init sequence.
  // Defaults: continuous per-interval streaming (like a real controller,
  // verified stable on C6-class chips) and an all-zero IMU motion block.
  // Wake-on-boot is disabled so waking is user-initiated, like a real
  // controller: while bonded but disconnected, pressing BOOT broadcasts the
  // wake advertisement (see the loop below) instead of the driver nudging the
  // console automatically every few seconds.
  espp::Switch2Pro controller({
      .device_name = "Pro Controller",
      .log_level = espp::Logger::Verbosity::INFO,
      .wake_console_on_boot = false,
  });

  if (!controller.init()) {
    logger.error("failed to initialize Switch2Pro");
    return;
  }
  logger.info("advertising — on the Switch 2, open Controllers > Pair, and watch "
              "the log for the connect interval, the 0x15 pairing exchange, and "
              "either 'pairing finalised' or a disconnect reason");

  // The BOOT button (GPIO0) is wired as the A button for easy testing.
  gpio_config_t btn_cfg = {};
  btn_cfg.pin_bit_mask = 1ULL << kBootButtonGpio;
  btn_cfg.mode = GPIO_MODE_INPUT;
  btn_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
  btn_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  btn_cfg.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&btn_cfg);

  // Feed input state to the driver. set_input_report() just stores the latest
  // report; the driver's streaming task paces the actual BLE notifications (one
  // per connection interval, like a real controller) and only streams once the
  // console has subscribed. So it is safe to call every loop — keep one report
  // and mutate it. Replace the BOOT read below with your real button/stick source.
  //
  // Press BOOT and watch A register on the Switch 2's "Test Input Devices" screen.
  espp::switch2::Pro2InputReport report;
  report.set_power(/*battery_level=*/9, /*charging=*/false, /*external_power=*/false); // full
  int tick = 0;
  // The Switch 2 shows "press L + R on the controller you want to use" while
  // selecting; auto-hold L+R for ~1 s each time streaming (re)starts to satisfy
  // that selection prompt so the console activates this controller.
  constexpr int kLrHoldTicks = 66; // ~1 s at the 15 ms cadence below
  bool prev_streaming = false;
  bool prev_pressed = false;
  int lr_ticks = 0;
  while (true) {
    const bool pressed = gpio_get_level(kBootButtonGpio) == 0; // BOOT reads low when pressed
    const bool press_edge = pressed && !prev_pressed;
    prev_pressed = pressed;

    // BOOT while bonded-but-disconnected = wake the console (a real controller
    // wakes the console on a button press). wake_console() no-ops unless there
    // is a stored bond and no active connection, so the edge check is enough.
    if (press_edge && !controller.is_connected()) {
      if (controller.wake_console())
        logger.info("BOOT pressed while disconnected -> sent wake advertisement");
    }

    // Rising edge of streaming: (re)arm the L+R auto-press.
    const bool streaming = controller.is_input_streaming();
    if (streaming && !prev_streaming)
      lr_ticks = kLrHoldTicks;
    prev_streaming = streaming;
    const bool lr_auto = lr_ticks > 0;
    if (lr_ticks > 0)
      --lr_ticks;

    report.set_a(pressed); // BOOT doubles as A while connected
    report.set_l(lr_auto);
    report.set_r(lr_auto);
    report.set_left_stick(0.f, 0.f);  // centered
    report.set_right_stick(0.f, 0.f); // centered
    controller.set_input_report(report);

    if (++tick % 66 == 0) // ~1 s at the 15 ms cadence below
      logger.info("connected={} streaming={} A(boot)={} L+R(auto)={}", controller.is_connected(),
                  streaming, pressed, lr_auto);
    std::this_thread::sleep_for(15ms); // ~66 Hz, matching the real controller
  }
  //! [switch2_pro example]
}

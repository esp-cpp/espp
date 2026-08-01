#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <memory>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "gui.hpp"
#include "lilygo-t5-47.hpp"
#include "logger.hpp"

using namespace std::chrono_literals;

// Render a received LoRa payload as a printable string (non-printable bytes -
// e.g. a stray frame from another transmitter on the same frequency - are shown
// as '.').
static std::string printable(const std::vector<uint8_t> &data) {
  std::string s(data.size(), '.');
  std::transform(data.begin(), data.end(), s.begin(),
                 [](uint8_t b) { return (b >= 0x20 && b < 0x7f) ? static_cast<char>(b) : '.'; });
  return s;
}

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "LilyGo T5 4.7 Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting LilyGo T5 4.7\" e-paper LVGL example");

  //! [lilygo t5 47 example]
  auto &board = espp::LilyGoT547::get();
  if (!board.initialize_display()) {
    logger.error("Failed to initialize the e-paper display!");
    return;
  }
  if (!board.initialize_lvgl()) {
    logger.error("Failed to initialize LVGL!");
    return;
  }
  logger.info("Display is {}x{}", board.width(), board.height());

  // Keep the panel's high-voltage rails powered on for the demo so each update
  // doesn't pay the power-up latency, and start from a clean (ghost-free) panel.
  board.power_on();
  board.clear();

  // Build the interactive GUI. The Gui owns the LVGL update task (which flushes
  // to the panel), so all panel access from here on goes through it.
  Gui gui({.board = board, .log_level = espp::Logger::Verbosity::WARN});

  // The BOOT button triggers a full grayscale refresh (handy to clear ghosting).
  board.initialize_button([&logger, &board, &gui](const auto &) {
    if (board.button_state()) {
      logger.info("BOOT button pressed -> full refresh");
      gui.request_full_refresh();
    }
  });

  // Touch drives the on-screen widgets (via the registered LVGL input device);
  // the callback also feeds the "Touch" and "Home button" rows. The GT911
  // reports the capacitive home button as t.btn_state.
  if (board.initialize_touch([&gui, &logger](const espp::TouchpadData &t) {
        logger.info("touch event: points={} ({}, {}) home={}", t.num_touch_points, t.x, t.y,
                    t.btn_state);
        gui.set_touch(t.num_touch_points, t.x, t.y);
        gui.set_home_button(t.btn_state);
      })) {
    logger.info("Touch initialized");
  } else {
    logger.warn("Touch not initialized");
  }

  bool have_rtc = board.initialize_rtc();
  bool have_battery = board.initialize_battery();
  bool have_expander = board.initialize_io_expander();
  logger.info("RTC {}, battery {}, I/O expander {}", have_rtc ? "ok" : "n/a",
              have_battery ? "ok" : "n/a", have_expander ? "ok" : "n/a");

  // Initialize the LoRa radio (SX1262) BEFORE the microSD card. Both share the
  // SPI bus; bringing the radio up first means it initializes on a pristine bus
  // (an absent/failed microSD mount touches the same SPI host). Received packets
  // are logged and shown in the stats panel; the "LoRa TX" button transmits a
  // test message. Set the frequency for your region via the RadioConfig (default
  // is US 906.875 MHz).
  espp::Sx126x::RadioConfig lora_config{};
  lora_config.sync_word = 0x12; // private link
  std::shared_ptr<espp::Sx126x> radio;
  if (board.initialize_lora(lora_config)) {
    radio = board.lora();
    radio->set_receive_callback([&gui, &logger](const espp::Sx126x::RxPacket &packet) {
      char buf[64];
      snprintf(buf, sizeof(buf), "RX %ddBm: %s", static_cast<int>(packet.status.rssi),
               printable(packet.data).c_str());
      logger.info("LoRa {}", buf);
      gui.set_lora(buf);
    });
    std::error_code ec;
    if (radio->start_receive(ec)) {
      logger.info("LoRa radio listening");
      gui.set_lora("listening");
    } else {
      logger.error("LoRa start_receive failed: {}", ec.message());
    }
  } else {
    logger.warn("LoRa radio not initialized");
    gui.set_lora("n/a");
  }

  // Mount the microSD card (SPI) and list its root directory (optional).
  if (board.initialize_sdcard({})) {
    if (DIR *dir = opendir(espp::LilyGoT547::mount_point)) {
      logger.info("microSD card contents:");
      while (dirent *entry = readdir(dir)) {
        logger.info("  {} {}", entry->d_type == DT_DIR ? "[dir] " : "[file]", entry->d_name);
      }
      closedir(dir);
    }
  } else {
    logger.warn("No microSD card mounted (this is fine if none is inserted)");
  }

  // Scan the internal I2C bus and log every device that answers. Expected:
  // 0x20 (PCA9535 expander), 0x51 (PCF8563 RTC), 0x55 (BQ27220 gauge), and
  // 0x5D (GT911 touch). If 0x5D/0x14 is missing, the GT911 isn't coming out of
  // reset (check the touch RST/INT pins).
  logger.info("Scanning internal I2C bus...");
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    if (board.internal_i2c()->probe_device(addr)) {
      logger.info("  I2C device found at 0x{:02X}", addr);
    }
  }

  // Render everything cleanly once.
  gui.request_full_refresh();
  //! [lilygo t5 47 example]

  // Poll the peripherals and feed the GUI. The clock ticks every second, the
  // battery updates every 5 s, and the IO48 button is polled every second. A
  // periodic full refresh clears the ghosting that partial updates leave behind.
  static const char *const weekdays[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
  auto now = std::chrono::steady_clock::now();
  auto last_clock = now - 1s;
  auto last_status = now - 1s;
  auto last_battery = now - 5s;
  auto last_full = now;
  bool logged_rtc = false, logged_pwr = false;
  char buf[48];
  while (true) {
    now = std::chrono::steady_clock::now();

    if (have_rtc && now - last_clock >= 1s) {
      last_clock = now;
      std::error_code ec;
      auto dt = board.rtc()->get_date_time(ec);
      if (!ec) {
        snprintf(buf, sizeof(buf), "%02d:%02d:%02d", dt.time.hour, dt.time.minute, dt.time.second);
        gui.set_time(buf);
        const char *wd = dt.date.weekday < 7 ? weekdays[dt.date.weekday] : "";
        snprintf(buf, sizeof(buf), "%04d-%02d-%02d %s", dt.date.year, dt.date.month, dt.date.day,
                 wd);
        gui.set_date(buf);
      } else if (!logged_rtc) {
        logger.error("RTC read failed: {}", ec.message());
      }
      logged_rtc = true;
    }

    // Poll the IO48 button independently of the RTC path.
    if (have_expander && now - last_status >= 1s) {
      last_status = now;
      bool pressed = board.io48_button_pressed();
      gui.set_io48_button(pressed);
      if (!logged_pwr) {
        logger.info("IO48 button first read: {}", pressed ? "pressed" : "released");
        logged_pwr = true;
      }
    }

    if (have_battery && now - last_battery >= 5s) {
      last_battery = now;
      std::error_code ec;
      auto mv = board.battery()->get_voltage_mv(ec);
      auto soc = board.battery()->get_state_of_charge(ec);
      auto ma = board.battery()->get_current_ma(ec);
      if (!ec) {
        snprintf(buf, sizeof(buf), "Battery: %.2f V  %d%%  %d mA", mv / 1000.0f, soc, ma);
        gui.set_battery(buf);
      } else {
        gui.set_battery("Battery: read error");
      }
    }

    // Service a LoRa send requested from the "LoRa TX" button. transmit() blocks
    // for the packet's time-on-air, so it is done here (not in the UI task).
    if (radio && gui.take_lora_send_request()) {
      static int tx_count = 0;
      char msg[32];
      snprintf(msg, sizeof(msg), "T5 hello %d", ++tx_count);
      std::span<const uint8_t> payload{reinterpret_cast<const uint8_t *>(msg), std::strlen(msg)};
      std::error_code ec;
      if (radio->transmit(payload, 3s, ec)) {
        logger.info("LoRa TX: {}", msg);
        gui.set_lora(std::string("TX ") + msg);
      } else {
        logger.error("LoRa transmit failed: {}", ec.message());
        gui.set_lora("TX failed");
      }
      radio->start_receive(ec); // resume listening after the transmit
    }

    // Clear accumulated ghosting periodically. The default DU (mono) mode ghosts
    // faster than GC16, so refresh a bit more often.
    if (now - last_full >= 30s) {
      last_full = now;
      gui.request_full_refresh();
    }

    std::this_thread::sleep_for(200ms);
  }
}

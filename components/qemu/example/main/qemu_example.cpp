#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include <esp_heap_caps.h>
#include <esp_timer.h>

#include "qemu.hpp"

#include "gui.hpp"
#include "logger.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "QEMU Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [qemu example]
  auto &qemu = espp::Qemu::get();

  // The virtual RGB panel: appears as the QEMU window with --graphics /
  // -display sdl, and renders headless otherwise (screendump works either
  // way). Nothing to do with real hardware: initialize_display() fails there.
  logger.info("Initializing display...");
  if (!qemu.initialize_display({.width = 320, .height = 240})) {
    logger.error("Failed to initialize the display (not running in QEMU?)");
    return;
  }

  // the UI: status lines + a sweeping arc, updated from here and from the
  // Ethernet callbacks
  Gui gui({.log_level = espp::Logger::Verbosity::WARN});

  // The OpenCores Ethernet MAC on QEMU's user-mode network: a DHCP client
  // gets 10.0.2.15 from QEMU, with 10.0.2.2 as the gateway (the host) and
  // 10.0.2.3 as the DNS server.
  logger.info("Initializing Ethernet...");
  bool eth_ok = qemu.initialize_ethernet({
      .mode = espp::Qemu::DhcpMode::CLIENT,
      .on_link_up = [&]() { gui.set_ethernet_text("Ethernet: link up, waiting for DHCP"); },
      .on_link_down = [&]() { gui.set_ethernet_text("Ethernet: link down"); },
      .on_got_ip =
          [&](esp_ip4_addr_t ip) {
            std::string text = "Ethernet: " + std::to_string(esp_ip4_addr1(&ip)) + "." +
                               std::to_string(esp_ip4_addr2(&ip)) + "." +
                               std::to_string(esp_ip4_addr3(&ip)) + "." +
                               std::to_string(esp_ip4_addr4(&ip));
            logger.info("{}", text);
            gui.set_ethernet_text(text);
          },
  });
  if (!eth_ok) {
    logger.error("Failed to initialize Ethernet");
    gui.set_ethernet_text("Ethernet: failed (CONFIG_ETH_USE_OPENETH?)");
  }

#if SOC_SDMMC_HOST_SUPPORTED
  // The SD card image attached with -drive file=sd.img,if=sd,format=raw
  // (ESP32 machine only): mount it, write a file, read it back.
  logger.info("Initializing SD card...");
  if (qemu.initialize_sdcard({.mount_point = "/sdcard"})) {
    const char *path = "/sdcard/hello.txt";
    std::string status = "SD card: mounted";
    if (FILE *f = fopen(path, "w"); f != nullptr) {
      fprintf(f, "hello from espp in QEMU, boot %lld us\n", esp_timer_get_time());
      fclose(f);
      char line[80] = {};
      if (FILE *r = fopen(path, "r"); r != nullptr) {
        if (fgets(line, sizeof(line), r) != nullptr) {
          logger.info("Read back from {}: {}", path, line);
          status = "SD card: hello.txt written and read back";
        }
        fclose(r);
      }
    } else {
      status = "SD card: mounted, but could not write hello.txt";
    }
    gui.set_sdcard_text(status);
  } else {
    logger.warn("No SD card (attach an image with -drive file=sd.img,if=sd,format=raw)");
    gui.set_sdcard_text("SD card: none (see README)");
  }
#else
  gui.set_sdcard_text("SD card: not emulated on this target");
#endif

  // Keep the uptime line and a heap figure moving, so a log capture or a
  // screenshot taken at any time shows a live system.
  int seconds = 0;
  while (true) {
    std::this_thread::sleep_for(1s);
    seconds++;
    gui.set_uptime_text("Uptime: " + std::to_string(seconds) + " s");
    if (seconds % 10 == 0) {
      logger.info("Uptime {} s, free heap {} bytes, Ethernet {}", seconds,
                  heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
                  qemu.is_ethernet_connected() ? "up" : "down");
    }
  }
  //! [qemu example]
}

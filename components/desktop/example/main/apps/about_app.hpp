#pragma once

// About: a static window of SystemInfo labels (chip, firmware, partitions).

#include "desktop.hpp"
#include "system_info.hpp"

inline void register_about_app(espp::Desktop &desktop) {
  desktop.register_app({
      .name = "About",
      .icon = "svg:info",
      .description = "What this device is",
      .launch =
          [](espp::Desktop &d, espp::Desktop::AppId app) {
            using D = espp::Desktop;
            const auto s = espp::SystemInfo::collect();
            auto win = d.create_window({.title = "About this device", .app = app});
            win.label(fmt::format("{} rev {}.{} ({} core{})", s.chip_model, s.chip_revision / 100,
                                  s.chip_revision % 100, s.cores, s.cores == 1 ? "" : "s"),
                      0, D::kLabelBold);
            auto grid = win.group("Firmware");
            auto line = [&](std::string_view key, std::string value) {
              auto row = win.row(grid.id());
              win.label(key, row.id(), D::kLabelBold).set_size(110, 0);
              win.label(value, row.id(), D::kLabelMonospace);
            };
            line("Project", fmt::format("{} {}", s.project_name, s.app_version));
            line("Built", fmt::format("{} {}", s.build_date, s.build_time));
            line("ESP-IDF", s.idf_version);
            line("Partition", fmt::format("{} (boot {})", s.running_partition, s.boot_partition));
            line("Reset", fmt::format("reason {}", s.reset_reason));
            auto hw = win.group("Hardware");
            auto hwline = [&](std::string_view key, std::string value) {
              auto row = win.row(hw.id());
              win.label(key, row.id(), D::kLabelBold).set_size(110, 0);
              win.label(value, row.id(), D::kLabelMonospace);
            };
            hwline("MAC", fmt::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}", s.mac[0],
                                      s.mac[1], s.mac[2], s.mac[3], s.mac[4], s.mac[5]));
            hwline("Flash", fmt::format("{} KiB", s.flash_size / 1024));
            hwline("PSRAM", s.psram_size ? fmt::format("{} KiB", s.psram_size / 1024) : "none");
            hwline("CPU", fmt::format("{} MHz", s.cpu_mhz));
            win.separator();
            win.label("espp desktop \xE2\x80\x94 the browser draws, the firmware decides.", 0,
                      D::kLabelWrap);
          },
  });
}

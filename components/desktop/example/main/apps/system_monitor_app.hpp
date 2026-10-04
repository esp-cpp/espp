#pragma once

// System Monitor: uptime / heap gauges refreshed by a 1 s window timer.

#include <chrono>

#include "esp_heap_caps.h"

#include "desktop.hpp"
#include "heap_monitor.hpp"
#include "system_info.hpp"

inline void register_system_monitor_app(espp::Desktop &desktop) {
  desktop.register_app({
      .name = "System Monitor",
      .icon = "\xF0\x9F\x93\x8A", // bar chart
      .description = "Uptime and heap usage, live",
      .launch =
          [](espp::Desktop &d, espp::Desktop::AppId app) {
            using namespace std::chrono_literals;
            using D = espp::Desktop;
            auto win = d.create_window({.title = "System Monitor", .app = app, .w = 360, .h = 0});
            auto uptime = win.label("", 0, D::kLabelMonospace);
            struct Gauge {
              const char *name;
              int caps;
              D::Widget label;
              D::Widget bar;
            };
            auto gauges = std::make_shared<std::vector<Gauge>>();
            for (const auto &[name, caps] :
                 {std::pair{"Internal", MALLOC_CAP_INTERNAL}, std::pair{"PSRAM", MALLOC_CAP_SPIRAM},
                  std::pair{"Default", MALLOC_CAP_DEFAULT}}) {
              const auto hi = espp::HeapMonitor::get_info(caps);
              if (hi.total_size == 0)
                continue; // e.g. no PSRAM
              auto grp = win.group(name);
              auto label = win.label("", grp.id(), D::kLabelMonospace);
              auto bar = win.progress(0, 0, 1000, grp.id());
              gauges->push_back({name, caps, label, bar});
            }
            auto refresh = [=]() mutable {
              const auto ms = espp::SystemInfo::uptime_ms();
              uptime.set_text("Uptime {}d {:02}:{:02}:{:02}   CPU {} MHz", ms / 86'400'000,
                              (ms / 3'600'000) % 24, (ms / 60'000) % 60, (ms / 1000) % 60,
                              espp::SystemInfo::cpu_mhz());
              for (auto &g : *gauges) {
                const auto hi = espp::HeapMonitor::get_info(g.caps);
                const size_t used = hi.total_size - hi.free_bytes;
                g.label.set_text("{} / {} KiB used, min free {} KiB, largest block {} KiB",
                                 used / 1024, hi.total_size / 1024, hi.min_free_bytes / 1024,
                                 hi.largest_free_block / 1024);
                g.bar.set_value(
                    static_cast<int32_t>(hi.total_size ? used * 1000 / hi.total_size : 0));
              }
            };
            refresh();
            win.add_timer(1s, refresh); // cancelled with the window
          },
  });
}

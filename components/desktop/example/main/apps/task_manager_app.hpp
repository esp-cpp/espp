#pragma once

// Task Manager: the FreeRTOS task table (espp::TaskMonitor) with a refresh
// period selector.

#include <chrono>
#include <memory>

#include "sdkconfig.h"

#include "desktop.hpp"
#include "task_monitor.hpp"

inline void register_task_manager_app(espp::Desktop &desktop) {
  desktop.register_app({
      .name = "Task Manager",
      .icon = "\xE2\x9A\x99\xEF\xB8\x8F", // gear
      .description = "FreeRTOS tasks: CPU, stack, priority, core",
      .launch =
          [](espp::Desktop &d, espp::Desktop::AppId app) {
            using namespace std::chrono_literals;
            using D = espp::Desktop;
            auto win = d.create_window({.title = "Task Manager", .app = app, .w = 460, .h = 360});
#if !(CONFIG_FREERTOS_USE_TRACE_FACILITY && CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS)
            win.label("Task statistics need CONFIG_FREERTOS_USE_TRACE_FACILITY and "
                      "CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS.",
                      0, D::kLabelWrap);
            return;
#endif
            auto bar = win.row();
            win.label("Refresh", bar.id());
            auto timer = std::make_shared<D::TimerId>(0);
            auto table = win.table({"Task", "CPU %", "Stack free", "Prio", "Core"}, {}, nullptr);
            auto count = win.label("", 0, D::kLabelMonospace);
            auto refresh = [=]() mutable {
              const auto infos = espp::TaskMonitor::get_latest_info_vector();
              std::vector<std::string> rows;
              rows.reserve(infos.size());
              for (const auto &t : infos)
                rows.push_back(fmt::format(
                    "{}\t{}\t{}\t{}\t{}", t.name, t.cpu_percent, t.high_water_mark, t.priority,
                    t.core_id < 0 ? std::string("any") : std::to_string(t.core_id)));
              table.set_items(rows);
              count.set_text("{} tasks", infos.size());
            };
            const std::vector<std::chrono::milliseconds> periods = {1s, 2s, 5s, 0s};
            auto select_period = [=, &d](int32_t index) mutable {
              if (*timer)
                d.cancel_timer(*timer);
              *timer = 0;
              if (index >= 0 && static_cast<size_t>(index) < periods.size() &&
                  periods[static_cast<size_t>(index)].count() > 0)
                *timer = win.add_timer(periods[static_cast<size_t>(index)], refresh);
            };
            win.select({"1 s", "2 s", "5 s", "paused"}, 0, select_period, bar.id());
            win.button("Refresh now", refresh, bar.id());
            refresh();
            select_period(0);
          },
  });
}

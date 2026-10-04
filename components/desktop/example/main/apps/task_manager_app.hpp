#pragma once

// Task Manager: the FreeRTOS task table (espp::TaskMonitor) with a refresh
// period selector, a name / core filter and browser-side column sorting
// (kTableSortable: click a column header, again for descending).

#include <algorithm>
#include <cctype>
#include <chrono>
#include <memory>
#include <string>
#include <string_view>

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
            // filters are applied on every refresh; the table sorts in the browser
            auto filters = win.row();
            win.label("Filter", filters.id());
            auto name_filter = std::make_shared<std::string>();
            auto core_filter =
                std::make_shared<int32_t>(0); // 0 all, 1 core 0, 2 core 1, 3 unpinned
            auto table = win.table({"Task", "CPU %", "Stack free", "Prio", "Core"}, {}, nullptr);
            table.set_flags(D::kTableSortable);
            auto count = win.label("", 0, D::kLabelMonospace);
            auto refresh = [=]() mutable {
              auto contains_ci = [](std::string_view hay, std::string_view needle) {
                if (needle.empty())
                  return true;
                auto lower = [](unsigned char c) { return std::tolower(c); };
                return std::search(hay.begin(), hay.end(), needle.begin(), needle.end(),
                                   [&](char a, char b) {
                                     return lower(static_cast<unsigned char>(a)) ==
                                            lower(static_cast<unsigned char>(b));
                                   }) != hay.end();
              };
              const auto infos = espp::TaskMonitor::get_latest_info_vector();
              std::vector<std::string> rows;
              rows.reserve(infos.size());
              for (const auto &t : infos) {
                const int32_t cf = *core_filter;
                const bool core_ok = cf == 0 || (cf == 1 && t.core_id == 0) ||
                                     (cf == 2 && t.core_id == 1) || (cf == 3 && t.core_id < 0);
                if (!core_ok || !contains_ci(t.name, *name_filter))
                  continue;
                rows.push_back(fmt::format(
                    "{}\t{}\t{}\t{}\t{}", t.name, t.cpu_percent, t.high_water_mark, t.priority,
                    t.core_id < 0 ? std::string("any") : std::to_string(t.core_id)));
              }
              table.set_items(rows);
              if (rows.size() == infos.size())
                count.set_text("{} tasks", infos.size());
              else
                count.set_text("{} of {} tasks", rows.size(), infos.size());
            };
            // the name filter applies on Enter and when the field loses focus
            auto name_box = win.textbox("", nullptr, filters.id(), "task name");
            name_box.on_event([=](const D::WidgetEvent &e) mutable {
              if (e.kind == D::WidgetEventKind::Submit || e.kind == D::WidgetEventKind::Text) {
                *name_filter = e.text;
                refresh();
              }
            });
            win.select(
                {"all cores", "core 0", "core 1", "unpinned"}, 0,
                [=](int32_t index) mutable {
                  *core_filter = index;
                  refresh();
                },
                filters.id());
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

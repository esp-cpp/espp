#pragma once

#include <chrono>
#include <iostream>

#include "sdkconfig.h"

#include "base_component.hpp"
#include "tabulate.hpp"
#include "task.hpp"

#include "esp_freertos_hooks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace espp {
/**
 *  @brief Class which monitors the currently running tasks in the system and
 *     periodically logs their states. See also <a
 *     href="https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html#_CPPv412vTaskGetInfo12TaskHandle_tP12TaskStatus_t10BaseType_t10eTaskState">FreeRTOS::vTaskGetInfo()</a>.
 *
 *  @note you must enable CONFIG_FREERTOS_USE_TRACE_FACILITY and
 *     CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS for this class to do anything.
 *     This means that you can always instantiate this class in your app_main,
 *     and then based on those two config settings it will either do nothing
 *     (default) or print out the stats for you to analyze. Finally, the
 *     monitoring period can be configured as well.
 *
 * \section task_monitor_ex1 Basic Task Monitor Example
 * \snippet monitor_example.cpp TaskMonitor example
 *
 * \section task_monitor_ex2 get_latest_info_vector() Example
 * \snippet monitor_example.cpp get_latest_info_vector example
 *
 * \section task_monitor_ex3 get_latest_info_*() Example
 * \snippet monitor_example.cpp get_latest_info example
 */
class TaskMonitor : public BaseComponent {
public:
  /**
   * Info structure for each task monitored.
   */
  struct TaskInfo {
    std::string name;         /**< Name of the task. */
    uint32_t cpu_percent;     /**< % CPU run time the task has used. */
    uint32_t high_water_mark; /**< Stack high water mark (bytes). */
    uint32_t priority;        /**< Current priority of the task. */
  };

  /**
   * Config structure for TaskMonitor object.
   */
  struct Config {
    std::chrono::duration<float> period; /**< Period (s) the TaskMonitor::task_callback runs at. */
    size_t task_stack_size_bytes{
        8 * 1024}; /**< Stack size (B) allocated to the TaskMonitor::task_callback.  */
  };

  explicit TaskMonitor(const Config &config)
      : BaseComponent("TaskMonitor")
      , period_(config.period) {
#if CONFIG_FREERTOS_USE_TRACE_FACILITY && CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
    using namespace std::placeholders;
    task_ = Task::make_unique({.name = "TaskMonitor Task",
                               .callback = std::bind(&TaskMonitor::task_callback, this, _1, _2),
                               .stack_size_bytes = config.task_stack_size_bytes});
    task_->start();
#else
    logger_.warn("Project was not built with "
                 "CONFIG_FREERTOS_USE_TRACE_FACILITY && CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS "
                 "cannot monitor task performance!");
#endif
  }

  /**
   * @brief Get information about all the tasks running.
   *        Will provide for each task the following information:
   *          * name
   *          * % CPU run time the task has used
   *          * stack high water mark (bytes)
   *          * current priority of the task
   * @return std::vector<TaskInfo> vector containing info for each task.
   */
  static std::vector<TaskInfo> get_latest_info_vector() {
    std::vector<TaskInfo> task_info;
#if CONFIG_FREERTOS_USE_TRACE_FACILITY && CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
    TaskStatus_t *pxTaskStatusArray;
    volatile UBaseType_t uxArraySize;
    uint32_t ulTotalRunTime, ulStatsAsPercentage;

    // Take a snapshot of the number of tasks in case it changes while this
    // function is executing.
    uxArraySize = uxTaskGetNumberOfTasks();

    // Allocate a TaskStatus_t structure for each task.  An array could be
    // allocated statically at compile time.
    pxTaskStatusArray =
        static_cast<TaskStatus_t *>(pvPortMalloc(uxArraySize * sizeof(TaskStatus_t)));

    if (pxTaskStatusArray != NULL) {
      // Generate raw status information about each task.
      uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);

      // For percentage calculations.
      ulTotalRunTime /= 100UL;

      // Avoid divide by zero errors.
      if (ulTotalRunTime > 0) {
        // For each populated position in the pxTaskStatusArray array,
        // format the raw data as human readable ASCII data
        for (size_t x = 0; x < uxArraySize; x++) {
          // Minimum amount of stack space that has remained for the task
          // since the task was created. The closer this value is to zero, the
          // closer the task has come to overflowing its stack.
          auto high_water_mark = pxTaskStatusArray[x].usStackHighWaterMark;

          // The priority at which the task was running (may be inherited).
          auto priority = pxTaskStatusArray[x].uxCurrentPriority;

          // What percentage of the total run time has the task used?
          // This will always be rounded down to the nearest integer.
          // ulTotalRunTimeDiv100 has already been divided by 100.
          ulStatsAsPercentage = pxTaskStatusArray[x].ulRunTimeCounter / ulTotalRunTime;

          if (ulStatsAsPercentage > 0UL) {
            task_info.push_back(
                {pxTaskStatusArray[x].pcTaskName, ulStatsAsPercentage, high_water_mark, priority});
          } else {
            // If the percentage is zero here then the task has
            // consumed less than 1% of the total run time.
            task_info.push_back({pxTaskStatusArray[x].pcTaskName, 0, high_water_mark, priority});
          }
        }
      }
      // The array is no longer needed, free the memory it consumes.
      vPortFree(pxTaskStatusArray);
    }
#endif
    return task_info;
  }

  /**
   * @brief Get information about all the tasks running.
   *        Will provide for each task the following information:
   *          * name
   *          * % CPU run time the task has used
   *          * stack high water mark (bytes)
   *          * current priority of the task
   *
   *        Where each entry is separated by ',' and each set of task data is
   *        separated by ';'.
   *
   *        @note There is no newline returned.
   *
   *        This is a static function, so it can be called without having to
   *        instantiate a TaskMonitor object.
   *
   * @return std::string containing sequence of entries, formatted:
   *
   *     name, cpu%, high_water_mark, priority;;
   *
   * @note This function calls TaskMonitor::get_latest_info_vector() and then
   *       formats the data into a single line string separated by , and ;.
   */
  static std::string get_latest_info_string() {
    std::string info = "";
    auto task_info = get_latest_info_vector();
    for (const auto &t : task_info) {
      if (t.cpu_percent > 0.0f) {
        info += fmt::format("{},{},{},{};", t.name, t.cpu_percent, t.high_water_mark, t.priority);
      } else {
        info += fmt::format("{},<1%,{},{};", t.name, 0, t.high_water_mark, t.priority);
      }
    }
    return info;
  }

  /**
   * @brief Print the latest task information in a nice table format.
   *        This is a static function, so it can be called without having to
   *        instantiate a TaskMonitor object.
   * @param os std::ostream to write the table to.
   * @return A tabulate::Table object which can be streamed to a std::ostream.
   * @note This function calls TaskMonitor::get_latest_info_vector() and then
   *       formats the data into a table using the tabulate library.
   */
  static auto get_latest_info_table() {
    using namespace tabulate;
    Table table;
    table.add_row({"Task Name", "CPU %", "High Water Mark", "Priority"});
    auto task_info = get_latest_info_vector();
    for (const auto &t : task_info) {
      std::string percent = t.cpu_percent > 0 ? fmt::format("{} %", t.cpu_percent) : "<1%";
      table.add_row(
          {t.name, percent, fmt::format("{} B", t.high_water_mark), fmt::format("{}", t.priority)});
    }
    return table;
  }

protected:
  bool task_callback(std::mutex &m, std::condition_variable &cv) {
    auto start = std::chrono::high_resolution_clock::now();
    // print out the monitor information
    fmt::print("[TM]{}\n", get_latest_info_string());
    // sleep until our period is up
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait_until(lk, start + period_);
    }
    // don't want the task to stop
    return false;
  }

  std::chrono::duration<float> period_;
  std::unique_ptr<Task> task_;
};
} // namespace espp

// for printing TaskMonitor::TaskInfo using libfmt
template <> struct fmt::formatter<espp::TaskMonitor::TaskInfo> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::TaskMonitor::TaskInfo &t, FormatContext &ctx) {
    return fmt::format_to(ctx.out(),
                          "TaskInfo(name={}, cpu_percent={}, high_water_mark={}, priority={})",
                          t.name, t.cpu_percent, t.high_water_mark, t.priority);
  }
};

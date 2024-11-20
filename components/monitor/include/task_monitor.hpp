#pragma once

#include <chrono>
#include <iostream>

#include "sdkconfig.h"

#include "base_component.hpp"
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
 *  @note If you wish to also get the core id of the task, you must enable
 *     CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID. If you do not enable this
 *     option, the core id will be -2.
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
    int core_id; /**< Core the task is running on. Will be 0,1, or -1 if task is not pinned to a
                    core. Only valid if CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID is set to y,
                    otherwise will be -2.*/
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
                               .callback = std::bind(&TaskMonitor::task_callback, this, _1, _2, _3),
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

          int core_id = -2; // -2 is the default value if core id is not available
#if CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID
          core_id = pxTaskStatusArray[x].xCoreID;
          if (core_id == tskNO_AFFINITY) {
            core_id = -1;
          }
#endif

          task_info.push_back({pxTaskStatusArray[x].pcTaskName, ulStatsAsPercentage,
                               high_water_mark, priority, core_id});
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
   *          * core the task is running on
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
   *     name, cpu%, high_water_mark, priority, core_id;;
   *
   * @note This function calls TaskMonitor::get_latest_info_vector() and then
   *       formats the data into a single line string separated by , and ;.
   */
  static std::string get_latest_info_string() {
    std::string info = "";
    auto task_info = get_latest_info_vector();
    // cppcheck-suppress knownEmptyContainer
    for (const auto &t : task_info) {
      if (t.cpu_percent > 0.0f) {
        info += fmt::format("{},{},{},{},{};", t.name, t.cpu_percent, t.high_water_mark, t.priority,
                            t.core_id);
      } else {
        info +=
            fmt::format("{},<1%,{},{},{};", t.name, 0, t.high_water_mark, t.priority, t.core_id);
      }
    }
    return info;
  }

  /**
   * @brief Print the latest task information in a nice table format.
   *        This is a static function, so it can be called without having to
   *        instantiate a TaskMonitor object.
   * @return A string containing the information in a table format.
   * @note This function calls TaskMonitor::get_latest_info_vector() and then
   *       formats the data into a table using the libfmt library.
   */
  static auto get_latest_info_table() {
    std::string output = "";
    auto task_info = get_latest_info_vector();
    // cppcheck-suppress knownConditionTrueFalse
    if (task_info.empty()) {
      return output;
    }
    static constexpr int task_name_header_min_width = 9; // length of "Task Name"
    static constexpr int task_name_max_width = CONFIG_FREERTOS_MAX_TASK_NAME_LEN;
    static constexpr int task_name_width =
        std::max(task_name_header_min_width, task_name_max_width);
#if CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID
    output = fmt::format("| {1: ^{0}.{0}s} | CPU % | High Water Mark | Priority | Core ID |\n"
                         "| {2:->{0}} | ----- | --------------- | -------- | ------- |\n",
                         task_name_width, "Task Name", "");
    for (const auto &t : task_info) {
      output += fmt::format("| {1: >{0}.{0}s} | {2: >3} % | {3: >13} B | {4: >8} | {5: >7} |\n",
                            task_name_width, t.name, t.cpu_percent, t.high_water_mark, t.priority,
                            t.core_id);
    }
#else
    output = fmt::format("| {1: ^{0}.{0}s} | CPU % | High Water Mark | Priority |\n"
                         "| {2:->{0}} | ----- | --------------- | -------- |\n",
                         task_name_width, "Task Name", "");
    for (const auto &t : task_info) {
      output += fmt::format("| {1: >{0}.{0}s} | {2: >3} % | {3: >13} B | {4: >8} |\n",
                            task_name_width, t.name, t.cpu_percent, t.high_water_mark, t.priority);
    }
#endif
    return output;
  }

protected:
  bool task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
    auto start = std::chrono::high_resolution_clock::now();
    // print out the monitor information
    fmt::print("[TM]{}\n", get_latest_info_string());
    // sleep until our period is up
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait_until(lk, start + period_, [&task_notified] { return task_notified; });
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
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::TaskMonitor::TaskInfo &t, FormatContext &ctx) const {
    return fmt::format_to(
#if CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID
        ctx.out(), "TaskInfo(name={}, cpu_percent={}, high_water_mark={}, priority={}, core_id={})",
        t.name, t.cpu_percent, t.high_water_mark, t.priority, t.core_id);
#else
        ctx.out(), "TaskInfo(name={}, cpu_percent={}, high_water_mark={}, priority={})", t.name,
        t.cpu_percent, t.high_water_mark, t.priority);
#endif
  }
};

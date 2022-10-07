#pragma once

#include <chrono>

#include "sdkconfig.h"

#include "logger.hpp"
#include "task.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"

namespace espp {
  /**
   *  @brief Class which monitors the currently running tasks in the system and
   *     periodically logs their states. See also <a
   *     href="https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html#_CPPv412vTaskGetInfo12TaskHandle_tP12TaskStatus_t10BaseType_t10eTaskState">FreeRTOS::vTaskGetInfo()</a>.
   *     NOTE: you must enable CONFIG_FREERTOS_USE_TRACE_FACILITY and
   *     CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS for this class to do anything.
   *     This means that you can always instantiate this class in your app_main,
   *     and then based on those two config settings it will either do nothing
   *     (default) or print out the stats for you to analyze. Finally, the
   *     monitoring period can be configured as well.
   *
   * \section task_monitor_ex1 Basic Task Monitor Example
   * \snippet monitor_example.cpp TaskMonitor example
   */
  class TaskMonitor {
  public:
    struct Config {
      std::chrono::duration<float> period;  /**< Period (s) the TaskMonitor::task_callback runs at. */
      size_t task_stack_size_bytes{8*1024}; /**< Stack size (B) allocated to the TaskMonitor::task_callback.  */
    };

    TaskMonitor(const Config& config) : period_(config.period), logger_({.tag = "TaskMonitor"}) {
#if CONFIG_FREERTOS_USE_TRACE_FACILITY && CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
      using namespace std::placeholders;
      task_ = Task::make_unique({
          .name = "TaskMonitor Task",
          .callback = std::bind(&TaskMonitor::task_callback, this, _1, _2),
          .stack_size_bytes = config.task_stack_size_bytes
        });
      task_->start();
#else
      logger_.warn("Project was not built with "
                   "CONFIG_FREERTOS_USE_TRACE_FACILITY && CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS"
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
     *
     *        Where each entry is separated by ',' and each set of task data is
     *        separated by ';'. NOTE: there is no newline returned.
     *
     * @return std::string containing sequence of entries, formatted:
     *
     *     name, cpu%, high_water_mark, priority;;
     */
    std::string get_latest_info() {
#if CONFIG_FREERTOS_USE_TRACE_FACILITY && CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
      // make this static so that we don't allocate on the stack each time we
      // call this function (and then deallocate later). NOTE(WARN): doing this
      // makes this function non-reentrant and not thread-safe.
      static char info_str[1024] = {0};

      TaskStatus_t *pxTaskStatusArray;
      volatile UBaseType_t uxArraySize;
      uint32_t ulTotalRunTime, ulStatsAsPercentage;

      char *pcWriteBuffer = &info_str[0];

      // Make sure the write buffer does not contain a string.
      *pcWriteBuffer = 0x00;

      // Take a snapshot of the number of tasks in case it changes while this
      // function is executing.
      uxArraySize = uxTaskGetNumberOfTasks();

      // Allocate a TaskStatus_t structure for each task.  An array could be
      // allocated statically at compile time.
      pxTaskStatusArray = static_cast<TaskStatus_t*>(pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) ));

      if( pxTaskStatusArray != NULL ) {
		// Generate raw status information about each task.
		uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalRunTime );

		// For percentage calculations.
		ulTotalRunTime /= 100UL;

		// Avoid divide by zero errors.
		if( ulTotalRunTime > 0 ) {
          // For each populated position in the pxTaskStatusArray array,
          // format the raw data as human readable ASCII data
          for(size_t x = 0; x < uxArraySize; x++ ) {
            // Minimum amount of stack space that has remained for the task
            // since the task was created. The closer this value is to zero, the
            // closer the task has come to overflowing its stack.
            auto high_water_mark = pxTaskStatusArray[ x ].usStackHighWaterMark;

            // The priority at which the task was running (may be inherited).
            auto priority = pxTaskStatusArray[ x ].uxCurrentPriority;

            // What percentage of the total run time has the task used?
            // This will always be rounded down to the nearest integer.
            // ulTotalRunTimeDiv100 has already been divided by 100.
            ulStatsAsPercentage = pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalRunTime;

            if( ulStatsAsPercentage > 0UL ) {
              sprintf( pcWriteBuffer, "%s,%lu%%,%ld,%d;;",
                       pxTaskStatusArray[ x ].pcTaskName,
                       ulStatsAsPercentage,
                       high_water_mark,
                       priority );
            } else {
              // If the percentage is zero here then the task has
              // consumed less than 1% of the total run time.
              sprintf( pcWriteBuffer, "%s,<1%%,%ld,%d;;",
                       pxTaskStatusArray[ x ].pcTaskName,
                       high_water_mark,
                       priority );
            }

            pcWriteBuffer += strlen( ( char * ) pcWriteBuffer );
          }
		}
		// The array is no longer needed, free the memory it consumes.
		vPortFree( pxTaskStatusArray );
      }
      return std::string{info_str};
#else
      logger_.error("Project was not built with "
                    "CONFIG_FREERTOS_USE_TRACE_FACILITY && CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS"
                    "cannot monitor task performance!");
      return "";
#endif
    }

  protected:
    void task_callback(std::mutex& m, std::condition_variable& cv) {
      auto start = std::chrono::high_resolution_clock::now();
      // print out the monitor information
      fmt::print("[TM]{}\n", get_latest_info());
      // sleep until our period is up
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_until(lk, start + period_);
      }
    }

    std::chrono::duration<float> period_;
    Logger logger_;
    std::unique_ptr<Task> task_;
  };
}

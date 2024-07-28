if (core_id < 0 || core_id == xPortGetCoreID()) {
  // If no core id specified or we are already executing on the desired core,
  // run the function directly
  return f();
} else {
  // Otherwise run the function on the desired core
  if (core_id > configNUM_CORES - 1) {
    // If the core id is larger than the number of cores, run on the last core
    core_id = configNUM_CORES - 1;
  }
  std::mutex mutex;
  std::unique_lock lock(mutex); // cppcheck-suppress localMutex
  std::condition_variable cv;   ///< Signal for when the task is done / function is run
  if constexpr (!std::is_void_v<decltype(f())>) {
    // the function returns something
    decltype(f()) ret_val;
    auto f_task = espp::Task::make_unique(espp::Task::Config{
        .name = "run_on_core_task",
        .callback = [&mutex, &cv, &f, &ret_val](auto &cb_m, auto &cb_cv) -> bool {
          // synchronize with the main thread - block here until the main thread
          // waits on the condition variable (cv), then run the function
          std::unique_lock lock(mutex);
          // run the function
          ret_val = f();
          // signal that the task is done
          cv.notify_all();
          return true; // stop the task
        },
        .stack_size_bytes = stack_size_bytes,
        .priority = priority,
        .core_id = core_id,
    });
    f_task->start();
    cv.wait(lock);
    return ret_val;
  } else {
    // the function returns void
    auto f_task = espp::Task::make_unique(espp::Task::Config{
        .name = "run_on_core_task",
        .callback = [&mutex, &cv, &f](auto &cb_m, auto &cb_cv) -> bool {
          // synchronize with the main thread - block here until the main thread
          // waits on the condition variable (cv), then run the function
          std::unique_lock lock(mutex);
          // run the function
          f();
          // signal that the task is done
          cv.notify_all();
          return true; // stop the task
        },
        .stack_size_bytes = stack_size_bytes,
        .priority = priority,
        .core_id = core_id,
    });
    f_task->start();
    cv.wait(lock);
  }
}

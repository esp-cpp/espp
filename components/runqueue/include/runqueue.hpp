#pragma once

#include <deque>
#include <functional>
#include <mutex>
#include <set>
#include <utility>

#include "base_component.hpp"
#include "task.hpp"

namespace espp {
/// This class implements a run queue for generic functions.
///
/// The functions can be added to the queue with a priority. The functions are
/// executed in the order of their priority, but the currently executing
/// function will run to completion before the next function is executed.
///
/// The RunQueue is implemented as a Task that runs the functions in the queue.
/// The Task will run the highest priority function in the queue (if any) and
/// will block indefinitely until either either a new function is added to the
/// queue or the RunQueue is stopped.
///
/// \note Function priorities are relative to each other and are only compared
///       to other tasks in the specific RunQueue object. Different RunQueue
///       objects may be active at the same time and the priorities of
///       functions in one RunQueue are not compared to the priorities of
///       functions in another. Similarly, the priorities of functions is not
///       taken into account in the FreeRTOS scheduler - all functions in a
///       given RunQueue will run with the same task priority as the
///       RunQueue's task.
///
/// Because the RunQueue is implemented as an espp::Task, you can customize
/// its task configuration, such as priority, stack size, core id, etc.
///
/// \note All functions within a RunQueue share the same task context, so you
///       should set the stack size for the RunQueue accordingly.
///
/// The RunQueue can be configured to start automatically or manually. If the
/// RunQueue is configured to start automatically, it will start when it is
/// constructed. If it is configured to start manually, it will not start until
/// the start() method is called.
///
/// The RunQueue is thread-safe and can be used from multiple threads.
///
/// \warning Care should be taken to ensure that no functions in the queue are
///          blocking on each other, and all functions in the queue must return
///          (they cannot be infinite loops).
///
/// \warning Functions take a long time to run may delay or prevent the
///          execution of other functions in the queue. For this reason it's
///          recommended to try to keep the functions as short-lived as
///          possible, and to minimize the priorities of any functions which
///          take longer to execute.
///
/// \section runq_ex0 RunQueue Example
/// \snippet runqueue_example.cpp runqueue example
/// \section runq_ex1 Multiple RunQueues Example
/// \snippet runqueue_example.cpp multiple runqueue example
class RunQueue : public espp::BaseComponent {
public:
  /// The type used to represent the priority of a function.
  using Priority = std::uint8_t;

  /// The minimum priority a function can have.
  static constexpr int MIN_PRIORITY = std::numeric_limits<Priority>::min();

  /// The maximum priority a function can have.
  static constexpr int MAX_PRIORITY = std::numeric_limits<Priority>::max();

  /// A function that takes no arguments and returns void.
  using Function = std::function<void(void)>;
  // typedef std::function<void(void)> Function;

  /// A pair of a priority and a function.
  struct PriorityFunction {
    Priority priority; ///< The priority of the function.
    Function function; ///< The function.
  };

  /// Less than operator for PriorityFunction.
  /// \param lhs The left hand side of the comparison.
  /// \param rhs The right hand side of the comparison.
  /// \return True if the right hand side has a greater priority.
  friend bool operator<(const PriorityFunction &lhs, const PriorityFunction &rhs) {
    return lhs.priority < rhs.priority;
  }

  /// Greater than operator for PriorityFunction.
  /// \param lhs The left hand side of the comparison.
  /// \param rhs The right hand side of the comparison.
  /// \return True if the left hand side has a greater priority.
  friend bool operator>(const PriorityFunction &lhs, const PriorityFunction &rhs) {
    return lhs.priority > rhs.priority;
  }

  /// Configuration struct for the RunQueue
  struct Config {
    bool auto_start = true;                  ///< Whether the RunQueue should start automatically.
    espp::Task::BaseConfig task_config = {}; ///< The configuration for the runner task.
    espp::Logger::Verbosity log_level =
        espp::Logger::Verbosity::WARN; ///< The log level for the RunQueue.
  };

  /// Construct a RunQueue.
  /// \param config The configuration for the RunQueue.
  explicit RunQueue(const Config &config);

  /// Destroy the RunQueue.
  ~RunQueue();

  /// Get the number of functions in the queue.
  /// \return The number of functions in the queue.
  std::size_t queue_size() const;

  /// Get whether the run queue is running.
  /// \return True if the run queue is running.
  bool is_running() const;

  /// Start the run queue.
  void start();

  /// Stop the run queue.
  void stop();

  /// Add a function to the queue.
  /// \param function The function to add.
  /// \param priority The priority of the function. Defaults to MIN_PRIORITY.
  void add_function(const Function &function, Priority priority = MIN_PRIORITY);

protected:
  /// Manage the run queue.
  /// \details This function is called by the runner task to manage the run
  ///          queue. It will run the highest priority function in the queue
  ///          (if any) and will return true if there are more functions to
  ///          run.
  /// \return True if there are more functions to run.
  bool manage_queue();

  bool task_fn(std::mutex &m, std::condition_variable &cv, bool &task_notified);

  std::unique_ptr<espp::Task> runner_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  bool queue_notified_ = false;
  std::multiset<PriorityFunction> priority_function_queue_;
}; // class RunQueue
} // namespace espp

#pragma once

// Registry of live python-owned espp objects whose teardown joins a thread
// (Task, Timer, TrajectoryPlanner, ThreadPool, ...). Two problems it solves:
//
//  1. Deleting such an object from python runs its destructor with the GIL
//     held. The destructor joins the object's task thread, which may at that
//     moment be waiting for the GIL to invoke its python callback: deadlock.
//     StopOnExitHolder (stop_on_exit_holder.h) is the pybind11 holder for these
//     classes; it releases the GIL around `delete`.
//  2. At interpreter shutdown the same join happens while the interpreter is
//     finalising, when a C++ thread can no longer re-acquire the GIL to run its
//     callback (it blocks for good, or the process crashes). The holder
//     registers every live object here, and the module's atexit hook calls
//     stop_all() -- which stops each object's threads with the GIL released --
//     while python threads can still run, so the later destructors find nothing
//     left to join.
//
// Standard library + pybind11 only (no espp headers), so module.cpp can
// register the atexit hook without pulling in the whole library.

#include <functional>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

#include <pybind11/pybind11.h>

namespace espp {
namespace py_detail {

class StopRegistry {
public:
  static StopRegistry &instance() {
    static StopRegistry registry;
    return registry;
  }

  /// Register a live object with the function that stops its threads.
  void add(const void *key, std::function<void()> stop) {
    std::lock_guard<std::mutex> lock(mutex_);
    entries_[key] = std::move(stop);
  }

  /// Forget an object (its holder is deleting it).
  void remove(const void *key) {
    std::lock_guard<std::mutex> lock(mutex_);
    entries_.erase(key);
  }

  /// Stop the threads of every registered object. Called from python's atexit
  /// (GIL held): the GIL is released around the stops so a callback thread
  /// waiting for it can finish its callback and exit.
  void stop_all() {
    std::vector<std::function<void()>> stops;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stops.reserve(entries_.size());
      for (auto &[key, stop] : entries_)
        stops.push_back(stop);
    }
    pybind11::gil_scoped_release release;
    for (auto &stop : stops)
      stop();
  }

  size_t size() {
    std::lock_guard<std::mutex> lock(mutex_);
    return entries_.size();
  }

private:
  std::mutex mutex_;
  std::unordered_map<const void *, std::function<void()>> entries_;
};

} // namespace py_detail
} // namespace espp

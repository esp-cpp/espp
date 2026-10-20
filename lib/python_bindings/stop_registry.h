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
#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

#include <pybind11/pybind11.h>

namespace espp {
namespace py_detail {

/// One registered object. Its mutex serialises "stop it" against "delete it", so
/// stop_all() can never run a stop function on an object another python thread
/// is deleting at the same moment: the holder takes the entry's mutex (with the
/// GIL released, see StopOnExitHolder::reset) around the delete and clears
/// `alive`; stop_all() takes it around each stop and skips dead entries.
struct StopEntry {
  std::mutex mutex;
  std::function<void()> stop;
  bool alive = true;
};

class StopRegistry {
public:
  static StopRegistry &instance() {
    static StopRegistry registry;
    return registry;
  }

  /// Register a live object with the function that stops its threads.
  std::shared_ptr<StopEntry> add(const void *key, std::function<void()> stop) {
    auto entry = std::make_shared<StopEntry>();
    entry->stop = std::move(stop);
    std::lock_guard<std::mutex> lock(mutex_);
    entries_[key] = entry;
    return entry;
  }

  /// Forget an object (its holder is about to delete it); returns its entry so
  /// the holder can serialise the delete against a concurrent stop_all().
  std::shared_ptr<StopEntry> take(const void *key) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = entries_.find(key);
    if (it == entries_.end())
      return nullptr;
    auto entry = std::move(it->second);
    entries_.erase(it);
    return entry;
  }

  /// Stop the threads of every registered object. Called from python's atexit
  /// (GIL held): the GIL is released around the stops so a callback thread
  /// waiting for it can finish its callback and exit. A stop function that
  /// throws is ignored (nothing sensible can be done with it at exit).
  void stop_all() {
    std::vector<std::shared_ptr<StopEntry>> entries;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      entries.reserve(entries_.size());
      for (auto &[key, entry] : entries_)
        entries.push_back(entry);
    }
    pybind11::gil_scoped_release release;
    for (auto &entry : entries) {
      std::lock_guard<std::mutex> lock(entry->mutex);
      if (!entry->alive)
        continue;
      try {
        entry->stop();
      } catch (...) {
      }
    }
  }

  size_t size() {
    std::lock_guard<std::mutex> lock(mutex_);
    return entries_.size();
  }

private:
  std::mutex mutex_;
  std::unordered_map<const void *, std::shared_ptr<StopEntry>> entries_;
};

} // namespace py_detail
} // namespace espp

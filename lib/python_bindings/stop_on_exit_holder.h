#pragma once

// StopOnExitHolder<T>: the pybind11 holder type for espp classes whose
// destructor joins a thread (see stop_registry.h for the two deadlocks it
// prevents). It owns the object like std::unique_ptr<T>, registers it with the
// StopRegistry while alive, and deletes it with the GIL released.
//
// The classes using it are listed in _STOP_ON_EXIT_CLASSES in
// lib/autogenerate_bindings.py, which rewrites their `py::class_<espp::X>` to
// `py::class_<espp::X, espp::py_detail::StopOnExitHolder<espp::X>>`; every such
// class needs a stop_for_exit() overload below naming the call that stops its
// threads (idempotent, safe on an object that never started).

#include <pybind11/pybind11.h>

#include "ftp_server.hpp"
#include "rtsp_server.hpp"
#include "task.hpp"
#include "thread_pool.hpp"
#include "timer.hpp"
#include "trajectory_planner.hpp"

#include "stop_registry.h"

namespace espp {
namespace py_detail {

inline void stop_for_exit(espp::Task &t) { t.stop(); }
inline void stop_for_exit(espp::Timer &t) { t.cancel(); }
inline void stop_for_exit(espp::TrajectoryPlanner &p) { p.stop_task(); }
inline void stop_for_exit(espp::ThreadPool &p) { p.stop(); }
inline void stop_for_exit(espp::FtpServer &s) { s.stop(); }
inline void stop_for_exit(espp::RtspServer &s) { s.stop(); }

template <typename T> class StopOnExitHolder {
public:
  StopOnExitHolder() = default;
  explicit StopOnExitHolder(T *p)
      : ptr_(p) {
    if (ptr_)
      StopRegistry::instance().add(ptr_, [ptr = ptr_]() { stop_for_exit(*ptr); });
  }
  StopOnExitHolder(const StopOnExitHolder &) = delete;
  StopOnExitHolder &operator=(const StopOnExitHolder &) = delete;
  StopOnExitHolder(StopOnExitHolder &&other) noexcept
      : ptr_(other.ptr_) {
    other.ptr_ = nullptr;
  }
  StopOnExitHolder &operator=(StopOnExitHolder &&other) noexcept {
    if (this != &other) {
      reset();
      ptr_ = other.ptr_;
      other.ptr_ = nullptr;
    }
    return *this;
  }
  ~StopOnExitHolder() { reset(); }

  T *get() const { return ptr_; }
  T *operator->() const { return ptr_; }
  T &operator*() const { return *ptr_; }
  explicit operator bool() const { return ptr_ != nullptr; }

  /// Give up ownership without deleting (pybind11 uses this for
  /// return_value_policy::take_ownership hand-overs).
  T *release() {
    T *p = ptr_;
    if (p)
      StopRegistry::instance().remove(p);
    ptr_ = nullptr;
    return p;
  }

  /// Delete the object. The destructor joins the object's threads; a thread
  /// that is waiting for the GIL to run its python callback can only finish if
  /// the GIL is released here (pybind11 deletes holders with it held).
  void reset() {
    T *p = ptr_;
    if (!p)
      return;
    ptr_ = nullptr;
    StopRegistry::instance().remove(p);
    if (PyGILState_Check()) {
      pybind11::gil_scoped_release release;
      delete p;
    } else {
      delete p;
    }
  }

private:
  T *ptr_ = nullptr;
};

} // namespace py_detail
} // namespace espp

PYBIND11_DECLARE_HOLDER_TYPE(T, espp::py_detail::StopOnExitHolder<T>)

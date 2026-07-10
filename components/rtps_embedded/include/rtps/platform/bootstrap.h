#ifndef RTPS_PLATFORM_BOOTSTRAP_H
#define RTPS_PLATFORM_BOOTSTRAP_H

#include <condition_variable>
#include <mutex>

namespace rtps {
namespace platform {
namespace bootstrap {

struct InitSemaphoreHandle {
  std::mutex mutex;
  std::condition_variable cv;
  bool signaled{false};
};

inline bool createInitSemaphore(InitSemaphoreHandle *sem) {
  if (sem == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(sem->mutex);
  sem->signaled = false;
  return true;
}

inline void signalInitSemaphore(InitSemaphoreHandle *sem) {
  if (sem != nullptr) {
    {
      std::lock_guard<std::mutex> lock(sem->mutex);
      sem->signaled = true;
    }
    sem->cv.notify_one();
  }
}

inline void waitInitSemaphore(InitSemaphoreHandle *sem) {
  if (sem != nullptr) {
    std::unique_lock<std::mutex> lock(sem->mutex);
    sem->cv.wait(lock, [sem]() { return sem->signaled; });
  }
}

inline void freeInitSemaphore(InitSemaphoreHandle *sem) {
  (void)sem;
}

} // namespace bootstrap
} // namespace platform
} // namespace rtps

#endif // RTPS_PLATFORM_BOOTSTRAP_H

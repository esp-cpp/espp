#ifndef RTPS_PLATFORM_THREADING_H
#define RTPS_PLATFORM_THREADING_H

#include "lwip/err.h"
#include "lwip/sys.h"

namespace rtps {
namespace platform {
namespace threading {

using ThreadHandle = sys_thread_t;
using SemaphoreHandle = sys_sem_t;
using ThreadFunction = lwip_thread_fn;

inline bool createSemaphore(SemaphoreHandle *sem, u8_t initial_count = 0) {
  return sem != nullptr && sys_sem_new(sem, initial_count) == ERR_OK;
}

inline bool isSemaphoreValid(SemaphoreHandle *sem) {
  return sem != nullptr && sys_sem_valid(sem) != 0;
}

inline void freeSemaphore(SemaphoreHandle *sem) {
  if (sem != nullptr) {
    sys_sem_free(sem);
  }
}

inline void signalSemaphore(SemaphoreHandle *sem) {
  if (sem != nullptr) {
    sys_sem_signal(sem);
  }
}

inline void waitSemaphore(SemaphoreHandle *sem) {
  if (sem != nullptr) {
    sys_sem_wait(sem);
  }
}

inline ThreadHandle startThread(const char *name, ThreadFunction function,
                                void *arg, int stacksize, int prio) {
  return sys_thread_new(name, function, arg, stacksize, prio);
}

inline void sleepMs(u32_t milliseconds) { sys_msleep(milliseconds); }

} // namespace threading
} // namespace platform
} // namespace rtps

#endif // RTPS_PLATFORM_THREADING_H

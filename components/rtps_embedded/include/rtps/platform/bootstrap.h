#ifndef RTPS_PLATFORM_BOOTSTRAP_H
#define RTPS_PLATFORM_BOOTSTRAP_H

#include "lwip/err.h"
#include "lwip/sys.h"

namespace rtps {
namespace platform {
namespace bootstrap {

using InitSemaphoreHandle = sys_sem_t;

inline bool createInitSemaphore(InitSemaphoreHandle *sem) {
  return sem != nullptr && sys_sem_new(sem, 0) == ERR_OK;
}

inline void signalInitSemaphore(InitSemaphoreHandle *sem) {
  if (sem != nullptr) {
    sys_sem_signal(sem);
  }
}

inline void waitInitSemaphore(InitSemaphoreHandle *sem) {
  if (sem != nullptr) {
    sys_sem_wait(sem);
  }
}

inline void freeInitSemaphore(InitSemaphoreHandle *sem) {
  if (sem != nullptr) {
    sys_sem_free(sem);
  }
}

} // namespace bootstrap
} // namespace platform
} // namespace rtps

#endif // RTPS_PLATFORM_BOOTSTRAP_H

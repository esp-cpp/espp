#include "rtps/platform/sync.h"

namespace rtps {
namespace platform {
namespace sync {

bool createRecursiveMutex(RecursiveMutexHandle *mutex) {
  if (mutex == nullptr) {
    return false;
  }
  *mutex = xSemaphoreCreateRecursiveMutex();
  return *mutex != nullptr;
}

bool lockRecursive(RecursiveMutexHandle mutex) {
  if (mutex == nullptr) {
    return false;
  }
  return xSemaphoreTakeRecursive(mutex, portMAX_DELAY) == pdTRUE;
}

bool unlockRecursive(RecursiveMutexHandle mutex) {
  if (mutex == nullptr) {
    return false;
  }
  return xSemaphoreGiveRecursive(mutex) == pdTRUE;
}

} // namespace sync
} // namespace platform
} // namespace rtps

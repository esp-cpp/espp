#include "rtps/platform/sync.h"

#include <new>

namespace rtps {
namespace platform {
namespace sync {

bool createRecursiveMutex(RecursiveMutexHandle *mutex) {
  if (mutex == nullptr) {
    return false;
  }

  *mutex = new (std::nothrow) std::recursive_mutex();
  return *mutex != nullptr;
}

bool lockRecursive(RecursiveMutexHandle mutex) {
  if (mutex == nullptr) {
    return false;
  }

  mutex->lock();
  return true;
}

bool unlockRecursive(RecursiveMutexHandle mutex) {
  if (mutex == nullptr) {
    return false;
  }

  mutex->unlock();
  return true;
}

} // namespace sync
} // namespace platform
} // namespace rtps

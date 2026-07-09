#include "rtps/utils/Lock.h"

#include <cassert>

namespace rtps {

bool createMutex(platform::sync::RecursiveMutexHandle *mutex) {
  if (mutex == nullptr) {
    assert(false && "Mutex pointer is null");
    return false;
  }

  if (platform::sync::createRecursiveMutex(mutex)) {
    return true;
  } else {
    assert(false && "Mutex creation failed");
    return false;
  }
}

} // namespace rtps

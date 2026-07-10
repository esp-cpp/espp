#ifndef RTPS_PLATFORM_SYNC_H
#define RTPS_PLATFORM_SYNC_H

#include <mutex>

namespace rtps {
namespace platform {
namespace sync {

using RecursiveMutexHandle = std::recursive_mutex *;

bool createRecursiveMutex(RecursiveMutexHandle *mutex);
bool lockRecursive(RecursiveMutexHandle mutex);
bool unlockRecursive(RecursiveMutexHandle mutex);

} // namespace sync
} // namespace platform
} // namespace rtps

#endif // RTPS_PLATFORM_SYNC_H

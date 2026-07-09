#ifndef RTPS_PLATFORM_SYNC_H
#define RTPS_PLATFORM_SYNC_H

#if defined(ESP_PLATFORM)
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#else
#include "FreeRTOS.h"
#include "semphr.h"
#endif

namespace rtps {
namespace platform {
namespace sync {

using RecursiveMutexHandle = SemaphoreHandle_t;

bool createRecursiveMutex(RecursiveMutexHandle *mutex);
bool lockRecursive(RecursiveMutexHandle mutex);
bool unlockRecursive(RecursiveMutexHandle mutex);

} // namespace sync
} // namespace platform
} // namespace rtps

#endif // RTPS_PLATFORM_SYNC_H

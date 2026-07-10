#ifndef RTPS_PLATFORM_TYPES_H
#define RTPS_PLATFORM_TYPES_H

#include <array>
#include <cstddef>
#include <cstdint>
#ifdef __cplusplus
#include <mutex>
#endif

namespace rtps {
namespace platform {

using Ip4Address = std::array<uint8_t, 4>;
using SemaphoreHandle = void *;
using ThreadHandle = void *;
#ifdef __cplusplus
using Mutex = std::recursive_mutex;
#endif

inline bool tryGetDefaultIp4AddressBytes(std::array<uint8_t, 4> &ip) {
  (void)ip;
  return false;
}

} // namespace platform
} // namespace rtps

#endif // RTPS_PLATFORM_TYPES_H

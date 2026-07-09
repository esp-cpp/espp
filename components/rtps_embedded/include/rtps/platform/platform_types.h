#ifndef RTPS_PLATFORM_TYPES_H
#define RTPS_PLATFORM_TYPES_H

#include <array>
#include <cstdint>
#ifdef __cplusplus
#include <mutex>
#endif

#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/sys.h"

namespace rtps {
namespace platform {

using Ip4Address = ip4_addr_t;
using SemaphoreHandle = sys_sem_t;
using ThreadHandle = sys_thread_t;
#ifdef __cplusplus
using Mutex = std::recursive_mutex;
#endif

inline bool tryGetDefaultIp4AddressBytes(std::array<uint8_t, 4> &ip) {
  if (netif_default == nullptr) {
    return false;
  }

  const ip4_addr_t *iface_ip = netif_ip4_addr(netif_default);
  if (iface_ip == nullptr) {
    return false;
  }

  ip[0] = ip4_addr1(iface_ip);
  ip[1] = ip4_addr2(iface_ip);
  ip[2] = ip4_addr3(iface_ip);
  ip[3] = ip4_addr4(iface_ip);
  return true;
}

} // namespace platform
} // namespace rtps

#endif // RTPS_PLATFORM_TYPES_H

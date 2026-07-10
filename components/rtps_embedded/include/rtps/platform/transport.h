#ifndef RTPS_PLATFORM_TRANSPORT_H
#define RTPS_PLATFORM_TRANSPORT_H

#include <array>
#include <cstddef>

#include "rtps/common/types.h"
#include "rtps/platform/platform_types.h"

namespace rtps {
struct PacketInfo;

namespace platform {
namespace transport {

using Ip4AddressBytes = std::array<uint8_t, 4>;

using ReceiveCallback = void (*)(void *arg, const uint8_t *data,
                                 std::size_t size, Ip4Port_t localPort,
                                 Ip4Port_t remotePort,
                                 const Ip4AddressBytes &remoteAddress);

class ITransport {
public:
  virtual ~ITransport() = default;

  virtual bool ensureReceivePort(Ip4Port_t receivePort) = 0;
  virtual bool joinMultiCastGroup(const Ip4AddressBytes &addr) const = 0;
  virtual void sendPacket(PacketInfo &info) = 0;
};

} // namespace transport
} // namespace platform
} // namespace rtps

#endif // RTPS_PLATFORM_TRANSPORT_H

#ifndef RTPS_PLATFORM_TRANSPORT_H
#define RTPS_PLATFORM_TRANSPORT_H

#include "rtps/common/types.h"
#include "rtps/platform/platform_types.h"

namespace rtps {
struct PacketInfo;
struct UdpConnection;

namespace platform {
namespace transport {

class ITransport {
public:
  virtual ~ITransport() = default;

  virtual const UdpConnection *createUdpConnection(Ip4Port_t receivePort) = 0;
  virtual bool joinMultiCastGroup(platform::Ip4Address addr) const = 0;
  virtual void sendPacket(PacketInfo &info) = 0;
};

} // namespace transport
} // namespace platform
} // namespace rtps

#endif // RTPS_PLATFORM_TRANSPORT_H

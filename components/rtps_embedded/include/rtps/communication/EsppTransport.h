/*
The MIT License
Copyright (c) 2019 Lehrstuhl Informatik 11 - RWTH Aachen University
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE

This file is part of embeddedRTPS.

Author: i11 - Embedded Software, RWTH Aachen University
*/

#ifndef RTPS_ESPPTRANSPORT_H
#define RTPS_ESPPTRANSPORT_H

#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "rtps/communication/UdpConnection.h"
#include "rtps/config.h"
#include "rtps/platform/transport.h"
#include "udp_socket.hpp"

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace rtps {

class EsppTransport : public platform::transport::ITransport {
public:
  using RxCallback =
      void (*)(void *arg, udp_pcb *pcb, pbuf *p, const ip_addr_t *addr,
               Ip4Port_t port);

  EsppTransport(RxCallback callback, void *args);
  ~EsppTransport() override = default;

  const rtps::UdpConnection *createUdpConnection(Ip4Port_t receivePort) override;
  bool joinMultiCastGroup(platform::Ip4Address addr) const override;
  void sendPacket(PacketInfo &info) override;

private:
  struct Channel {
    rtps::UdpConnection connection{};
    std::unique_ptr<espp::UdpSocket> socket{};
    bool in_use{false};
  };

  Channel *findChannel(Ip4Port_t port);
  const Channel *findChannel(Ip4Port_t port) const;
  Channel *createChannel(Ip4Port_t receivePort);
  bool startReceiver(Channel &channel, Ip4Port_t receivePort);
  void onReceive(Ip4Port_t receivePort, std::vector<uint8_t> &data,
                 const espp::Socket::Info &sender) const;

  static std::string ip4ToString(platform::Ip4Address addr);

  RxCallback m_rxCallback{nullptr};
  void *m_callbackArgs{nullptr};
  mutable std::recursive_mutex m_mutex;
  std::array<Channel, rtps::Config::MAX_NUM_UDP_CONNECTIONS> m_channels{};
  mutable std::vector<std::string> m_multicastGroups;
};

} // namespace rtps

#endif // RTPS_ESPPTRANSPORT_H

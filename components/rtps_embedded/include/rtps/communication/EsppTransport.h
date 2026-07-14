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

#include "rtps/common/types.h"
#include "rtps/config.h"
#include "udp_socket.hpp"
#include "rtps/communication/PacketInfo.h"

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace rtps {

class EsppTransport {
public:
  using RxCallback = ReceiveCallback;

  EsppTransport(RxCallback callback, void *args);
  ~EsppTransport() = default;

    bool ensureReceivePort(Ip4Port_t receivePort);
    bool joinMultiCastGroup(const Ip4AddressBytes &addr) const;
    void sendPacket(PacketInfo &info);

private:
  struct Channel {
    Ip4Port_t port{0};
    std::unique_ptr<espp::UdpSocket> socket{};
    bool in_use{false};
  };

  Channel *findChannel(Ip4Port_t port);
  const Channel *findChannel(Ip4Port_t port) const;
  Channel *createChannel(Ip4Port_t receivePort);
  bool startReceiver(Channel &channel, Ip4Port_t receivePort);
  void onReceive(Ip4Port_t receivePort, std::vector<uint8_t> &data,
                 const espp::Socket::Info &sender) const;

  static std::string ip4ToString(const Ip4AddressBytes &addr);

  RxCallback m_rxCallback{nullptr};
  void *m_callbackArgs{nullptr};
  mutable std::recursive_mutex m_mutex;
  std::array<Channel, Config::MAX_NUM_UDP_CONNECTIONS> m_channels{};
  mutable std::vector<std::string> m_multicastGroups;
};

} // namespace rtps

#endif // RTPS_ESPPTRANSPORT_H

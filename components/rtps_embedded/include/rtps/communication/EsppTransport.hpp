/*
The MIT License
Copyright (c) 2019 Lehrstuhl Informatik 11 - RWTH Aachen University
Modifications Copyright (c) 2026 ATDev
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

#include "base_component.hpp"
#include "rtps/common/types.hpp"
#include "rtps/communication/PacketInfo.hpp"
#include "rtps/config.hpp"
#include "socket_reactor.hpp"
#include "udp_socket.hpp"

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace rtps {

class EsppTransport : public espp::BaseComponent {
public:
  using RxCallback = ReceiveCallback;

  EsppTransport(RxCallback callback, void *args);
  ~EsppTransport() = default;

  /// Ensure a receive channel exists for the port. Unicast ports are bound
  /// with address/port reuse DISABLED so an in-use port fails loudly (the
  /// Domain then probes the next participant id); multicast ports keep reuse
  /// enabled so multiple processes on one host can share them.
  bool ensureReceivePort(Ip4Port_t receivePort, bool is_multicast);
  /// Tear down the receive channel for a port (used to unwind a partially
  /// successful unicast port probe).
  bool releaseReceivePort(Ip4Port_t receivePort);
  bool joinMultiCastGroup(const Ip4AddressBytes &addr) const;
  void sendPacket(PacketInfo &info);

private:
  struct Channel {
    Ip4Port_t port{0};
    std::unique_ptr<espp::UdpSocket> socket{};
    espp::SocketReactor::Id reactor_id{espp::SocketReactor::INVALID_ID};
    bool in_use{false};
  };

  Channel *findChannel(Ip4Port_t port);
  const Channel *findChannel(Ip4Port_t port) const;
  Channel *createChannel(Ip4Port_t receivePort, bool allow_reuse);
  bool startReceiver(Channel &channel, Ip4Port_t receivePort);
  void onReceive(Ip4Port_t receivePort, std::vector<uint8_t> &data,
                 const espp::Socket::Info &sender) const;

  static std::string ip4ToString(const Ip4AddressBytes &addr);

  RxCallback m_rxCallback{nullptr};
  void *m_callbackArgs{nullptr};
  mutable std::recursive_mutex m_mutex;
  std::array<Channel, Config::MAX_NUM_UDP_CONNECTIONS> m_channels{};
  /// One select() loop + a small shared worker pool multiplexes every receive
  /// socket (SocketReactor's one-shot arming preserves per-socket ordering,
  /// which RTPS requires per locator), replacing a dedicated blocking-recv
  /// task per channel. Declared after m_channels so it is destroyed FIRST
  /// (reverse member order): the reactor must stop before its sockets die.
  std::shared_ptr<espp::SocketReactor> m_reactor{};
  mutable std::vector<std::string> m_multicastGroups;
};

} // namespace rtps

#endif // RTPS_ESPPTRANSPORT_H

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

#include "rtps/communication/EsppTransport.h"

#include "lwip/ip_addr.h"
#include "lwip/ip4_addr.h"
#include "rtps/communication/PacketInfo.h"
#include "task.hpp"

#include <array>
#include <optional>
#include <vector>

using rtps::EsppTransport;

EsppTransport::EsppTransport(RxCallback callback, void *args)
    : m_rxCallback(callback), m_callbackArgs(args) {}

EsppTransport::Channel *EsppTransport::findChannel(Ip4Port_t port) {
  for (auto &channel : m_channels) {
    if (channel.in_use && channel.connection.port == port) {
      return &channel;
    }
  }
  return nullptr;
}

const EsppTransport::Channel *EsppTransport::findChannel(Ip4Port_t port) const {
  for (const auto &channel : m_channels) {
    if (channel.in_use && channel.connection.port == port) {
      return &channel;
    }
  }
  return nullptr;
}

std::string EsppTransport::ip4ToString(platform::Ip4Address addr) {
  std::array<char, 16> buffer{};
  const char *result = ip4addr_ntoa_r(&addr, buffer.data(),
                                      static_cast<int>(buffer.size()));
  if (result == nullptr) {
    return "0.0.0.0";
  }
  return std::string(result);
}

bool EsppTransport::startReceiver(Channel &channel, Ip4Port_t receivePort) {
  if (!channel.socket) {
    return false;
  }

  espp::Task::BaseConfig task_config;
  task_config.name = "rtps_rx_" + std::to_string(receivePort);
  task_config.stack_size_bytes = Config::THREAD_POOL_READER_STACKSIZE;
  task_config.priority = Config::THREAD_POOL_READER_PRIO;

  espp::UdpSocket::ReceiveConfig receive_config;
  receive_config.port = receivePort;
  receive_config.buffer_size = 4096;
  receive_config.on_receive_callback =
      [this, receivePort](std::vector<uint8_t> &data,
                          const espp::Socket::Info &sender)
      -> std::optional<std::vector<uint8_t>> {
    onReceive(receivePort, data, sender);
    return std::nullopt;
  };

  return channel.socket->start_receiving(task_config, receive_config);
}

EsppTransport::Channel *EsppTransport::createChannel(Ip4Port_t receivePort) {
  for (auto &channel : m_channels) {
    if (channel.in_use) {
      continue;
    }

    espp::UdpSocket::Config socket_config;
    socket_config.log_level = espp::Logger::Verbosity::WARN;
    channel.socket = std::make_unique<espp::UdpSocket>(socket_config);
    if (!channel.socket || !channel.socket->is_valid()) {
      channel.socket.reset();
      return nullptr;
    }

    channel.connection.port = receivePort;
    channel.in_use = true;

    if (!startReceiver(channel, receivePort)) {
      channel.socket.reset();
      channel.connection.port = 0;
      channel.in_use = false;
      return nullptr;
    }

    for (const auto &group : m_multicastGroups) {
      (void)channel.socket->add_multicast_group(group);
    }
    return &channel;
  }
  return nullptr;
}

void EsppTransport::onReceive(Ip4Port_t receivePort, std::vector<uint8_t> &data,
                              const espp::Socket::Info &sender) const {
  if (m_rxCallback == nullptr) {
    return;
  }

  pbuf *packet_buffer = pbuf_alloc(PBUF_TRANSPORT,
                                   static_cast<u16_t>(data.size()), PBUF_POOL);
  if (packet_buffer == nullptr) {
    return;
  }

  if (!data.empty() &&
      pbuf_take(packet_buffer, data.data(), static_cast<u16_t>(data.size())) !=
          ERR_OK) {
    pbuf_free(packet_buffer);
    return;
  }

  udp_pcb pcb{};
  pcb.local_port = receivePort;

  ip_addr_t source_addr{};
  if (!ipaddr_aton(sender.address.c_str(), &source_addr)) {
    ip_addr_set_any(IPADDR_TYPE_V4, &source_addr);
  }

  m_rxCallback(m_callbackArgs, &pcb, packet_buffer, &source_addr,
               static_cast<Ip4Port_t>(sender.port));
}

const rtps::UdpConnection *EsppTransport::createUdpConnection(Ip4Port_t receivePort) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);

  Channel *existing = findChannel(receivePort);
  if (existing != nullptr) {
    return &existing->connection;
  }

  Channel *created = createChannel(receivePort);
  return created != nullptr ? &created->connection : nullptr;
}

bool EsppTransport::joinMultiCastGroup(platform::Ip4Address addr) const {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);

  const std::string group = ip4ToString(addr);
  for (const auto &existing : m_multicastGroups) {
    if (existing == group) {
      return true;
    }
  }

  bool any_joined = false;
  for (auto &channel : m_channels) {
    if (!channel.in_use || !channel.socket) {
      continue;
    }
    any_joined = channel.socket->add_multicast_group(group) || any_joined;
  }

  m_multicastGroups.push_back(group);
  return any_joined || m_multicastGroups.size() == 1;
}

void EsppTransport::sendPacket(PacketInfo &info) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);

  Channel *channel = findChannel(info.srcPort);
  if (channel == nullptr) {
    channel = createChannel(info.srcPort);
  }

  if (channel == nullptr || !channel->socket || info.buffer.firstElement == nullptr) {
    return;
  }

  const size_t packet_size = info.buffer.firstElement->tot_len;
  std::vector<uint8_t> bytes(packet_size);
  u16_t copied = pbuf_copy_partial(info.buffer.firstElement, bytes.data(),
                                   static_cast<u16_t>(packet_size), 0);
  if (copied != static_cast<u16_t>(packet_size)) {
    return;
  }

  espp::UdpSocket::SendConfig send_config;
  send_config.ip_address = ip4ToString(info.destAddr);
  send_config.port = info.destPort;
  send_config.is_multicast_endpoint = ip4_addr_ismulticast(&info.destAddr) != 0;

  (void)channel->socket->send(bytes, send_config);
}

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

#include "rtps/communication/PacketInfo.h"
#include "task.hpp"

#include <array>
#include <cstdlib>
#include <optional>
#include <vector>

using rtps::EsppTransport;

namespace {

bool parseIp4Address(const std::string &address,
                     rtps::platform::transport::Ip4AddressBytes &out) {
  rtps::platform::transport::Ip4AddressBytes parsed{0, 0, 0, 0};
  const char *cursor = address.c_str();

  for (std::size_t i = 0; i < parsed.size(); ++i) {
    char *end = nullptr;
    const unsigned long value = std::strtoul(cursor, &end, 10);
    if (end == cursor || value > 255) {
      return false;
    }

    parsed[i] = static_cast<uint8_t>(value);
    if (i + 1 < parsed.size()) {
      if (*end != '.') {
        return false;
      }
      cursor = end + 1;
    } else if (*end != '\0') {
      return false;
    }
  }

  out = parsed;
  return true;
}

bool isMulticastAddress(const rtps::platform::transport::Ip4AddressBytes &addr) {
  return addr[0] >= 224 && addr[0] <= 239;
}

} // namespace

EsppTransport::EsppTransport(RxCallback callback, void *args)
    : m_rxCallback(callback), m_callbackArgs(args) {}

EsppTransport::Channel *EsppTransport::findChannel(Ip4Port_t port) {
  for (auto &channel : m_channels) {
    if (channel.in_use && channel.port == port) {
      return &channel;
    }
  }
  return nullptr;
}

const EsppTransport::Channel *EsppTransport::findChannel(Ip4Port_t port) const {
  for (const auto &channel : m_channels) {
    if (channel.in_use && channel.port == port) {
      return &channel;
    }
  }
  return nullptr;
}

std::string EsppTransport::ip4ToString(
    const platform::transport::Ip4AddressBytes &addr) {
  return std::to_string(addr[0]) + "." + std::to_string(addr[1]) + "." +
         std::to_string(addr[2]) + "." + std::to_string(addr[3]);
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

    channel.port = receivePort;
    channel.in_use = true;

    if (!startReceiver(channel, receivePort)) {
      channel.socket.reset();
      channel.port = 0;
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

  platform::transport::Ip4AddressBytes remoteAddress{0, 0, 0, 0};
  (void)parseIp4Address(sender.address, remoteAddress);

  m_rxCallback(m_callbackArgs, data.data(), data.size(), receivePort,
               static_cast<Ip4Port_t>(sender.port), remoteAddress);
}

bool EsppTransport::ensureReceivePort(Ip4Port_t receivePort) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);

  Channel *existing = findChannel(receivePort);
  if (existing != nullptr) {
    return true;
  }

  Channel *created = createChannel(receivePort);
  return created != nullptr;
}

bool EsppTransport::joinMultiCastGroup(
    const platform::transport::Ip4AddressBytes &addr) const {
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

  if (channel == nullptr || !channel->socket) {
    return;
  }

  if (info.payload.empty()) {
    return;
  }

  espp::UdpSocket::SendConfig send_config;
  const platform::transport::Ip4AddressBytes destination = info.destAddr;
  send_config.ip_address = ip4ToString(destination);
  send_config.port = info.destPort;
  send_config.is_multicast_endpoint = isMulticastAddress(info.destAddr);

  (void)channel->socket->send(info.payload, send_config);
}

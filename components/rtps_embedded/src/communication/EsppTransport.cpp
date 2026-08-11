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

#include "rtps/communication/EsppTransport.hpp"

#include "task.hpp"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <optional>
#include <vector>

using rtps::EsppTransport;

namespace {

bool parseIp4Address(const std::string &address, rtps::Ip4AddressBytes &out) {
  rtps::Ip4AddressBytes parsed{0, 0, 0, 0};
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

bool isMulticastAddress(const rtps::Ip4AddressBytes &addr) {
  return addr[0] >= 224 && addr[0] <= 239;
}

} // namespace

EsppTransport::EsppTransport(RxCallback callback, void *args)
    : espp::BaseComponent("RtpsTransport", espp::Logger::Verbosity::WARN)
    , m_rxCallback(callback)
    , m_callbackArgs(args) {}

EsppTransport::Channel *EsppTransport::findChannel(Ip4Port_t port) {
  auto it = std::find_if(m_channels.begin(), m_channels.end(), [port](const auto &channel) {
    return channel.in_use && channel.port == port;
  });
  return (it != m_channels.end()) ? &(*it) : nullptr;
}

const EsppTransport::Channel *EsppTransport::findChannel(Ip4Port_t port) const {
  auto it = std::find_if(m_channels.begin(), m_channels.end(), [port](const auto &channel) {
    return channel.in_use && channel.port == port;
  });
  return (it != m_channels.end()) ? &(*it) : nullptr;
}

std::string EsppTransport::ip4ToString(const Ip4AddressBytes &addr) {
  return std::to_string(addr[0]) + "." + std::to_string(addr[1]) + "." + std::to_string(addr[2]) +
         "." + std::to_string(addr[3]);
}

bool EsppTransport::startReceiver(Channel &channel, Ip4Port_t receivePort) {
  if (!channel.socket) {
    logger_.error("startReceiver called with null socket on port {}", receivePort);
    return false;
  }

  espp::Task::BaseConfig task_config;
  task_config.name = "rtps_rx_" + std::to_string(receivePort);
  // Reuse the reader-worker stack size for the UDP receive task. Both tasks
  // perform similar deserialization work, so the value is a reasonable bound.
  task_config.stack_size_bytes = Config::THREAD_POOL_READER_STACKSIZE;
  task_config.priority = Config::THREAD_POOL_READER_PRIO;

  espp::UdpSocket::ReceiveConfig receive_config;
  receive_config.port = receivePort;
  receive_config.buffer_size = 1024 * 8;
  receive_config.on_receive_callback =
      [this, receivePort](std::vector<uint8_t> &data,
                          const espp::Socket::Info &sender) -> std::optional<std::vector<uint8_t>> {
    onReceive(receivePort, data, sender);
    return std::nullopt;
  };

  const bool started = channel.socket->start_receiving(task_config, receive_config);
  if (!started) {
    logger_.error("Failed to start UDP receiver on port {}", receivePort);
  }
  return started;
}

EsppTransport::Channel *EsppTransport::createChannel(Ip4Port_t receivePort, bool allow_reuse) {
  for (auto &channel : m_channels) {
    if (channel.in_use) {
      continue;
    }

    espp::UdpSocket::Config socket_config;
    socket_config.log_level = espp::Logger::Verbosity::WARN;
    channel.socket = std::make_unique<espp::UdpSocket>(socket_config);
    if (!channel.socket->is_valid()) {
      logger_.error("Failed to create valid UDP socket for port {}", receivePort);
      channel.socket.reset();
      return nullptr;
    }

    if (!allow_reuse && !channel.socket->disable_reuse()) {
      logger_.error("Failed to disable port reuse for unicast port {}", receivePort);
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

  Ip4AddressBytes remoteAddress{0, 0, 0, 0};
  if (!parseIp4Address(sender.address, remoteAddress)) {
    logger_.warn("Could not parse sender IPv4 address '{}', using 0.0.0.0", sender.address);
  }
  logger_.debug("received {} bytes on port {}", static_cast<unsigned int>(data.size()),
                receivePort);
  m_rxCallback(m_callbackArgs, data.data(), data.size(), receivePort,
               static_cast<Ip4Port_t>(sender.port), remoteAddress);
}

bool EsppTransport::ensureReceivePort(Ip4Port_t receivePort, bool is_multicast) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);

  Channel *existing = findChannel(receivePort);
  if (existing != nullptr) {
    return true;
  }

  Channel *created = createChannel(receivePort, /*allow_reuse=*/is_multicast);
  return created != nullptr;
}

bool EsppTransport::releaseReceivePort(Ip4Port_t receivePort) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  Channel *channel = findChannel(receivePort);
  if (channel == nullptr) {
    return false;
  }
  if (channel->socket) {
    channel->socket->stop_receiving();
    channel->socket.reset();
  }
  channel->port = 0;
  channel->in_use = false;
  return true;
}

bool EsppTransport::joinMultiCastGroup(const Ip4AddressBytes &addr) const {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);

  const std::string group = ip4ToString(addr);
  const bool already_joined =
      std::any_of(m_multicastGroups.begin(), m_multicastGroups.end(),
                  [&](const auto &existing_group) { return existing_group == group; });
  if (already_joined) {
    return true;
  }

  bool any_joined = false;
  bool has_active_channels = false;
  for (auto &channel : m_channels) {
    if (!channel.in_use || !channel.socket) {
      continue;
    }
    has_active_channels = true;
    any_joined = channel.socket->add_multicast_group(group) || any_joined;
  }

  if (!has_active_channels) {
    // No active sockets yet: defer join until channels are created.
    m_multicastGroups.push_back(group);
    return true;
  }

  if (any_joined) {
    m_multicastGroups.push_back(group);
    return true;
  }

  logger_.warn("Failed to join multicast group {} on all active channels", group);
  return false;
}

void EsppTransport::sendPacket(PacketInfo &info) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);

  Channel *channel = findChannel(info.srcPort);
  if (channel == nullptr) {
    // Sending from one of our own unicast ports: apply unicast semantics
    // (no port sharing) if the channel was not already registered.
    channel = createChannel(info.srcPort, /*allow_reuse=*/false);
  }

  if (channel == nullptr || !channel->socket) {
    logger_.error("No UDP channel available for source port {}", info.srcPort);
    return;
  }

  if (info.payload.empty()) {
    return;
  }

  espp::UdpSocket::SendConfig send_config;
  const Ip4AddressBytes destination = info.destAddr;
  send_config.ip_address = ip4ToString(destination);
  send_config.port = info.destPort;
  send_config.is_multicast_endpoint = isMulticastAddress(info.destAddr);

  (void)channel->socket->send(info.payload, send_config);
}

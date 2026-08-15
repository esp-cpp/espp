/*
The MIT License
Copyright (c) 2026 ATDev
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

This file is part of the espp embeddedRTPS port.
*/

#include "rtps/communication/EsppTransport.hpp"

#include "task.hpp"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <optional>
#include <vector>

#ifdef RTPS_ENABLE_FRAGMENTATION
#if defined(_WIN32)
#include <winsock2.h>
#else
#include <sys/socket.h>
#endif
#endif

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
    , m_callbackArgs(args) {
  espp::ThreadPool::Config pool_config;
  pool_config.worker_count = 2;
  pool_config.worker_task_config = {
      .name = "rtps_worker",
      .stack_size_bytes = Config::THREAD_POOL_READER_STACKSIZE,
      .priority = Config::THREAD_POOL_READER_PRIO,
  };
  m_pool = std::make_shared<espp::ThreadPool>(pool_config);

  espp::SocketReactor::Config reactor_config;
  reactor_config.thread_pool = m_pool;
  reactor_config.loop_task_config = {
      .name = "rtps_reactor",
      .stack_size_bytes = Config::THREAD_POOL_READER_STACKSIZE,
      .priority = Config::THREAD_POOL_READER_PRIO,
  };
  reactor_config.log_level = espp::Logger::Verbosity::WARN;
  m_reactor = std::make_shared<espp::SocketReactor>(reactor_config);
}

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

  // Register the socket with the shared reactor instead of spawning a
  // dedicated blocking-recv task per port: add_udp_receiver() binds the socket
  // per the config and dispatches each datagram onto the reactor's worker
  // pool. One-shot arming guarantees at most one in-flight handler per socket,
  // preserving RTPS's per-locator ordering.
  espp::UdpSocket::ReceiveConfig receive_config;
  receive_config.port = receivePort;
#ifdef RTPS_ENABLE_FRAGMENTATION
  // With fragmentation enabled a peer (e.g. FastDDS/ROS 2) may send DATA_FRAG
  // fragments as large as a full UDP datagram (~64 KB), so the per-datagram read
  // buffer must accept any single datagram or large fragments get truncated on
  // receive. The ESP32 default build (fragmentation off) keeps the small 8 KB
  // buffer, paying nothing for this.
  receive_config.buffer_size = 64 * 1024;
#else
  receive_config.buffer_size = 1024 * 8;
#endif
  receive_config.on_receive_callback =
      [this, receivePort](std::vector<uint8_t> &data,
                          const espp::Socket::Info &sender) -> std::optional<std::vector<uint8_t>> {
    onReceive(receivePort, data, sender);
    return std::nullopt;
  };

  channel.reactor_id = m_reactor->add_udp_receiver(*channel.socket, receive_config);
  if (channel.reactor_id == espp::SocketReactor::INVALID_ID) {
    logger_.error("Failed to register UDP receiver on port {} with the reactor", receivePort);
    return false;
  }
  return true;
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

#ifdef RTPS_ENABLE_FRAGMENTATION
    // Enlarge the kernel receive buffer so a burst of DATA_FRAG datagrams for one
    // large (fragmented) sample is not dropped before the reactor drains it.
    // Best-effort: some stacks clamp SO_RCVBUF, so failure is ignored. Only
    // compiled when fragmentation is enabled (never on the ESP32 default build).
    {
      int rcvbuf = 4 * 1024 * 1024; // request 4 MB (kernel may clamp)
      ::setsockopt(channel.socket->native_handle(), SOL_SOCKET, SO_RCVBUF,
                   reinterpret_cast<const char *>(&rcvbuf), sizeof(rcvbuf));
    }
#endif

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

bool EsppTransport::submit(std::function<void()> job) {
  if (!m_pool || !m_pool->try_submit(std::move(job))) {
    logger_.warn("Transport worker pool rejected a job (stopped or queue full)");
    return false;
  }
  return true;
}

void EsppTransport::stop() {
  if (m_reactor) {
    m_reactor->stop();
  }
  if (m_pool) {
    m_pool->stop();
  }
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
  if (channel->reactor_id != espp::SocketReactor::INVALID_ID) {
    m_reactor->remove(channel->reactor_id);
    channel->reactor_id = espp::SocketReactor::INVALID_ID;
  }
  channel->socket.reset();
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

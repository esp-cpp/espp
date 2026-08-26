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

#ifndef RTPS_ESPPTRANSPORT_H
#define RTPS_ESPPTRANSPORT_H

#include "base_component.hpp"
#include "dscp.hpp"
#include "qos_band.hpp"
#include "rtps/common/types.hpp"
#include "rtps/communication/PacketInfo.hpp"
#include "rtps/config.hpp"
#include "socket_reactor.hpp"
#include "thread_pool.hpp"
#include "timer.hpp"
#include "udp_socket.hpp"

#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace rtps {

/// Per-channel scheduling options applied when a transport channel's socket is
/// registered on the reactor (see EsppTransport::ensureReceivePort).
struct ChannelOptions {
  /// Priority band the reactor dispatches this socket's datagrams at (see
  /// espp::QosBand / espp::SocketReactor). Normal = pre-band behavior.
  espp::QosBand band{espp::QosBand::Normal};
  /// Optional DSCP code point set on the socket (marks traffic SENT from this
  /// channel; see espp::Socket::set_dscp()). Best-effort: unsupported stacks
  /// simply ignore it.
  std::optional<espp::Dscp> dscp{};
};

class EsppTransport : public espp::BaseComponent {
public:
  using RxCallback = ReceiveCallback;

  EsppTransport(RxCallback callback, void *args);
  ~EsppTransport() = default;

  /// Ensure a receive channel exists for the port. Unicast ports are bound
  /// with address/port reuse DISABLED so an in-use port fails loudly (the
  /// Domain then probes the next participant id); multicast ports keep reuse
  /// enabled so multiple processes on one host can share them. The options
  /// only apply when the channel is newly created (an existing channel keeps
  /// its original band/dscp).
  bool ensureReceivePort(Ip4Port_t receivePort, bool is_multicast,
                         const ChannelOptions &options = {});
  /// Tear down the receive channel for a port (unwinds a partially successful
  /// unicast port probe; also releases a deleted endpoint's dedicated port).
  /// The channel slot is freed immediately; the underlying socket is RETIRED
  /// (kept alive for any in-flight reactor dispatch that references it) and
  /// destroyed - closing the fd and unbinding the port - as soon as the
  /// reactor confirms the registration is fully removed, usually immediately
  /// (see m_retiredSockets).
  bool releaseReceivePort(Ip4Port_t receivePort);

  /// Number of retired sockets still awaiting their removal-completion
  /// destruction (see m_retiredSockets). Normally 0 shortly after a release;
  /// exposed for tests/diagnostics.
  std::size_t retiredSocketCount() const;
  bool joinMultiCastGroup(const Ip4AddressBytes &addr) const;
  void sendPacket(PacketInfo &info);

  /// Submit asynchronous protocol work (e.g. a writer's progress()) onto the
  /// transport's shared worker pool - the same pool the reactor dispatches
  /// received datagrams on - at the given priority band (Normal preserves the
  /// pre-band FIFO behavior). Non-blocking; returns false (and logs) when the
  /// pool queue is full or stopped.
  bool submit(std::function<void()> job, espp::QosBand band = espp::QosBand::Normal);

  /// Submit work that MUST eventually run - e.g. a writer's progress(): the
  /// pool queue is bounded, and a silently rejected progress submission would
  /// strand unsent samples (a lone best-effort DATA has no heartbeat/acknack
  /// path to recover it). On rejection the poke is PARKED per producer \p key
  /// (the writer): each key holds one job + band and a count of owed runs, and
  /// a retry timer re-submits them until the pool accepts. This is lossless
  /// (nothing is dropped) and bounded by the number of producers, not by the
  /// number of pokes - key by the producer so repeated pokes coalesce into its
  /// count instead of growing an unbounded/dropping list. Each owed run maps
  /// one-to-one to a progress() call, preserving the one-poke/one-sample
  /// semantics. Jobs must remain safe to run until stop() (engine endpoints
  /// are pooled and outlive the transport's stop, per the submit() contract).
  void submitGuaranteed(const void *key, std::function<void()> job,
                        espp::QosBand band = espp::QosBand::Normal);

  /// Drop any PARKED guaranteed job for `key` so the retry timer cannot
  /// resubmit it after the producer has been deleted. Call this when an
  /// endpoint keyed here is being torn down individually (a single writer
  /// delete, not a full stop()): a parked progress() job would otherwise be
  /// resurrected against a reset/reused endpoint or an already-released port.
  /// Only removes not-yet-accepted work; a job already handed to the pool is
  /// made safe by the endpoint's own reset()/progress() init guard.
  void cancelGuaranteed(const void *key);

  /// Stop receive dispatch and the worker pool. Must be called before the
  /// objects referenced by in-flight/queued jobs (writers, participants) are
  /// destroyed; safe to call more than once.
  void stop();

private:
  struct Channel {
    Ip4Port_t port{0};
    std::unique_ptr<espp::UdpSocket> socket{};
    espp::SocketReactor::Id reactor_id{espp::SocketReactor::INVALID_ID};
    bool in_use{false};
  };

  Channel *findChannel(Ip4Port_t port);
  const Channel *findChannel(Ip4Port_t port) const;
  /// Destroy a retired socket once the reactor's removal has fully completed
  /// (invoked by the removal-completion callback; may run on the releasing
  /// caller, a pool worker, or the reactor loop). No-op if stop() already
  /// cleared it - the pointer is only used to find the entry.
  void destroyRetiredSocket(espp::UdpSocket *socket);
  Channel *createChannel(Ip4Port_t receivePort, bool allow_reuse, const ChannelOptions &options);
  bool startReceiver(Channel &channel, Ip4Port_t receivePort, const ChannelOptions &options);
  void onReceive(Ip4Port_t receivePort, std::vector<uint8_t> &data,
                 const espp::Socket::Info &sender) const;

  static std::string ip4ToString(const Ip4AddressBytes &addr);

  /// Park a rejected guaranteed poke under \p key (coalescing into its owed
  /// count) and (lazily) create the retry timer. Lossless and bounded by the
  /// producer count.
  void parkPendingJob(const void *key, std::function<void()> job, espp::QosBand band);

  RxCallback m_rxCallback{nullptr};
  void *m_callbackArgs{nullptr};
  mutable std::recursive_mutex m_mutex;
  std::array<Channel, Config::MAX_NUM_UDP_CONNECTIONS> m_channels{};
  /// Sockets released at runtime (releaseReceivePort) are RETIRED here, not
  /// destroyed inline: SocketReactor::remove() intentionally defers
  /// unregistration while a dispatch is in flight, and that dispatch's
  /// handler references the UdpSocket - destroying it immediately is a
  /// use-after-free (a handler was observed blocking forever on the freed
  /// object's internal mutex, wedging SocketReactor::stop()'s in-flight
  /// wait). Each retired socket is destroyed by the reactor's
  /// removal-completion callback (destroyRetiredSocket) as soon as the
  /// registration is fully gone and no handler can reference it - closing the
  /// fd and unbinding the port promptly, so endpoint churn does not
  /// accumulate fds (relevant to lwIP's small socket budget). stop() clears
  /// any stragglers whose completion never fired. Declared before
  /// m_pool/m_reactor so it is destroyed AFTER them (reverse member order).
  std::vector<std::unique_ptr<espp::UdpSocket>> m_retiredSockets{};
  /// Shared worker pool for received-datagram dispatch (via the reactor) and
  /// asynchronous writer work (submit()). Declared after m_channels and before
  /// m_reactor: destruction runs reactor -> pool -> channels.
  std::shared_ptr<espp::ThreadPool> m_pool{};
  /// One select() loop + a small shared worker pool multiplexes every receive
  /// socket (SocketReactor's one-shot arming preserves per-socket ordering,
  /// which RTPS requires per locator), replacing a dedicated blocking-recv
  /// task per channel. Declared after m_channels so it is destroyed FIRST
  /// (reverse member order): the reactor must stop before its sockets die.
  std::shared_ptr<espp::SocketReactor> m_reactor{};
  mutable std::vector<std::string> m_multicastGroups;

  /// Guaranteed-submission retry state (see submitGuaranteed()). The timer is
  /// created lazily on the first rejection and NEVER self-cancels (its callback
  /// always returns false; it is a cheap no-op once the pending map drains);
  /// only stop() cancels it - synchronously, BEFORE stopping the reactor/pool,
  /// so no retry callback runs during or after teardown. (A self-cancelling
  /// espp::Timer leaves running_ set while its task exits, so a later start()
  /// would no-op against a dead task and strand parked work - hence keep-alive.)
  struct PendingProgress {
    std::function<void()> job;
    espp::QosBand band{espp::QosBand::Normal};
    std::size_t count{0}; ///< owed progress() runs (one per rejected poke), never dropped
  };
  std::mutex m_pendingMutex;
  /// Coalesced per producer (writer pointer): repeated rejected pokes for the
  /// same writer accumulate in `count` rather than growing the map, so memory
  /// is bounded by the producer count regardless of publish volume.
  std::map<const void *, PendingProgress> m_pendingByKey;
  std::unique_ptr<espp::Timer> m_retryTimer;
  bool m_stopping{false};
};

} // namespace rtps

#endif // RTPS_ESPPTRANSPORT_H

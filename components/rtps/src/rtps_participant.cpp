#include "rtps_participant.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <condition_variable>
#include <cstdio>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rtps/common/Locator.hpp"
#include "rtps/entities/Domain.hpp"
#include "rtps/rpc/action_naming.hpp"
#include "rtps/rpc/action_types.hpp"
#include "rtps/rpc/cyclonedds_request_header.hpp"
#include "rtps/rpc/native_protocol.hpp"
#include "rtps/rpc/sample_identity.hpp"
#include "rtps/rpc/service_naming.hpp"
#include "thread_pool.hpp"

// Host-side interface auto-detection uses platform-specific enumeration APIs.
#if defined(ESP_PLATFORM)
// No enumeration: ESP targets must pass interface_address explicitly.
#elif defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>

// iphlpapi.h (GetAdaptersAddresses) must follow winsock2.h; the blank line keeps
// clang-format's alphabetical include sort from reordering it ahead of winsock2.
#include <iphlpapi.h>
#else
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#endif

namespace espp {

RtpsParticipant::RtpsParticipant(const Config &config)
    : BaseComponent("RtpsParticipant", config.log_level)
    , config_(config) {}

// NOTE: ~RtpsParticipant is defined at the END of this file, after the RPC
// context structs (ServiceServerContext, ActionServerContext, Native*Context,
// *Client::Impl) are complete - the destructor destroys vectors of
// unique_ptr/shared_ptr to those types, and libc++ requires the pointee to be
// complete at the point of destruction (libstdc++ is more lenient).

namespace {
// "255.255.128.0" or "17" -> prefix length; false for anything else (a mask
// whose set bits are not contiguous is not a netmask).
bool parse_netmask(const std::string &text, uint8_t &prefix_length) {
  unsigned a = 0, b = 0, c = 0, d = 0;
  char trailing = 0;
  // The %c sentinel only matches when something follows the quad, so exactly
  // four conversions means the whole string was consumed.
  if (std::sscanf(text.c_str(), "%u.%u.%u.%u%c", &a, &b, &c, &d, &trailing) == 4) {
    if (a > 255 || b > 255 || c > 255 || d > 255) {
      return false;
    }
    const uint32_t mask = (a << 24) | (b << 16) | (c << 8) | d;
    const uint8_t bits =
        rtps::prefixLengthFromNetmask({static_cast<uint8_t>(a), static_cast<uint8_t>(b),
                                       static_cast<uint8_t>(c), static_cast<uint8_t>(d)});
    const uint32_t expected = bits == 0 ? 0u : (0xFFFFFFFFu << (32u - bits));
    if (mask != expected) {
      return false;
    }
    prefix_length = bits;
    return true;
  }
  unsigned bits = 0;
  if (std::sscanf(text.c_str(), "%u%c", &bits, &trailing) == 1 && bits <= 32) {
    prefix_length = static_cast<uint8_t>(bits);
    return true;
  }
  return false;
}
} // namespace

bool RtpsParticipant::resolve_interface(std::array<uint8_t, 4> &ip_bytes,
                                        uint8_t &prefix_length) const {
  std::string addr = config_.interface_address;
  // Only the host enumerates interfaces, so this is unused on ESP targets.
  [[maybe_unused]] const bool auto_detect = addr.empty();
  // Prefix length of the interface that owns `addr`, when the platform can
  // tell (host only). Config::interface_netmask overrides it below.
  std::optional<uint8_t> detected_prefix{};
  // Skip loopback / link-local candidates during auto-detection. Unused on ESP
  // targets, which require an explicit interface_address.
  [[maybe_unused]] const auto usable = [](const std::string &ip) {
    return !ip.empty() && ip.rfind("127.", 0) != 0 && ip.rfind("169.254.", 0) != 0;
  };
#if defined(ESP_PLATFORM)
  // No interface enumeration on ESP targets: interface_address is required
  // (checked below) and the netmask comes from interface_netmask or the
  // /24 default.
#elif defined(_WIN32)
  {
    // Walk the up, non-loopback IPv4 adapters (GetAdaptersAddresses): pick the
    // first usable address when none is configured, and either way take the
    // prefix length of the adapter that owns the address.
    ULONG size = 15000;
    std::vector<uint8_t> buffer(size);
    auto *adapters = reinterpret_cast<IP_ADAPTER_ADDRESSES *>(buffer.data());
    const ULONG flags = GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER;
    ULONG ret = GetAdaptersAddresses(AF_INET, flags, nullptr, adapters, &size);
    if (ret == ERROR_BUFFER_OVERFLOW) {
      buffer.resize(size);
      adapters = reinterpret_cast<IP_ADAPTER_ADDRESSES *>(buffer.data());
      ret = GetAdaptersAddresses(AF_INET, flags, nullptr, adapters, &size);
    }
    if (ret == NO_ERROR) {
      for (const auto *a = adapters; a != nullptr && !detected_prefix; a = a->Next) {
        // Auto-detection skips down and loopback adapters (like the POSIX
        // path's usable()); an explicitly configured address may name any
        // adapter, loopback included, and still gets that adapter's prefix.
        if (auto_detect &&
            (a->OperStatus != IfOperStatusUp || a->IfType == IF_TYPE_SOFTWARE_LOOPBACK)) {
          continue;
        }
        for (auto *ua = a->FirstUnicastAddress; ua != nullptr; ua = ua->Next) {
          const auto *sin = reinterpret_cast<const sockaddr_in *>(ua->Address.lpSockaddr);
          if (sin == nullptr || sin->sin_family != AF_INET) {
            continue;
          }
          char buf[INET_ADDRSTRLEN] = {0};
          if (inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof(buf)) == nullptr) {
            continue;
          }
          if (auto_detect ? usable(buf) : addr == buf) {
            if (auto_detect) {
              logger_.info("Auto-detected interface address {}", buf);
            }
            addr = buf;
            detected_prefix = static_cast<uint8_t>(ua->OnLinkPrefixLength);
            break;
          }
        }
      }
    }
  }
#else
  {
    // Walk the IPv4 interfaces (getifaddrs): pick the first non-loopback,
    // non-link-local address when none is configured, and either way take the
    // netmask of the interface that owns the address.
    struct ifaddrs *ifaddr = nullptr;
    if (getifaddrs(&ifaddr) == 0) {
      for (struct ifaddrs *ifa = ifaddr; ifa != nullptr && !detected_prefix; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) {
          continue;
        }
        char buf[INET_ADDRSTRLEN] = {0};
        const auto *sin = reinterpret_cast<const struct sockaddr_in *>(ifa->ifa_addr);
        if (inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof(buf)) == nullptr) {
          continue;
        }
        if (auto_detect ? usable(buf) : addr == buf) {
          if (auto_detect) {
            logger_.info("Auto-detected interface address {}", buf);
          }
          addr = buf;
          if (ifa->ifa_netmask != nullptr && ifa->ifa_netmask->sa_family == AF_INET) {
            const auto *mask = reinterpret_cast<const struct sockaddr_in *>(ifa->ifa_netmask);
            const uint32_t m = ntohl(mask->sin_addr.s_addr);
            detected_prefix = rtps::prefixLengthFromNetmask(
                {static_cast<uint8_t>(m >> 24), static_cast<uint8_t>(m >> 16),
                 static_cast<uint8_t>(m >> 8), static_cast<uint8_t>(m)});
          }
          break;
        }
      }
      freeifaddrs(ifaddr);
    }
  }
#endif
  if (addr.empty()) {
    logger_.error("interface_address is empty and no usable IPv4 interface could be auto-detected; "
                  "set it (required on ESP targets, e.g. from the netif IP)");
    return false;
  }
  unsigned a = 0, b = 0, c = 0, d = 0;
  if (std::sscanf(addr.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) != 4 || a > 255 || b > 255 ||
      c > 255 || d > 255) {
    logger_.error("Invalid interface_address '{}'", addr);
    return false;
  }
  ip_bytes = {static_cast<uint8_t>(a), static_cast<uint8_t>(b), static_cast<uint8_t>(c),
              static_cast<uint8_t>(d)};

  // The network's prefix length: the configured netmask, else the interface's
  // own (host), else the historical /24 assumption.
  prefix_length = rtps::DEFAULT_SUBNET_PREFIX_LENGTH;
  if (!config_.interface_netmask.empty()) {
    if (!parse_netmask(config_.interface_netmask, prefix_length)) {
      logger_.error("Invalid interface_netmask '{}' (want e.g. \"255.255.128.0\" or \"17\")",
                    config_.interface_netmask);
      return false;
    }
  } else if (detected_prefix) {
    prefix_length = *detected_prefix;
  } else {
    logger_.debug("No netmask known for {}; assuming a /{} network (set interface_netmask)", addr,
                  prefix_length);
  }
  logger_.debug("Interface {}/{}", addr, prefix_length);
  return true;
}

namespace {
// The transport's tasks: the compile-time defaults, overridden by whatever
// Config::tasks sets.
rtps::EsppTransport::TaskConfig transport_config(const RtpsParticipant::Config &config) {
  rtps::EsppTransport::TaskConfig transport;
  if (config.tasks.receive_workers > 0) {
    transport.worker_count = config.tasks.receive_workers; // 0 keeps the transport's default
  }
  if (config.tasks.receive_worker) {
    transport.worker_task = *config.tasks.receive_worker;
  }
  if (config.tasks.reactor) {
    transport.reactor_task = *config.tasks.reactor;
  }
  if (config.tasks.job_retry) {
    transport.retry_task = *config.tasks.job_retry;
  }
  return transport;
}
} // namespace

bool RtpsParticipant::start() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (started_) {
    logger_.warn("Already started");
    return false;
  }

  std::array<uint8_t, 4> ip_bytes{};
  uint8_t prefix_length = rtps::DEFAULT_SUBNET_PREFIX_LENGTH;
  if (!resolve_interface(ip_bytes, prefix_length)) {
    return false;
  }

  // Channel/endpoint scheduling: metatraffic (SPDP/SEDP) dispatches at
  // metatraffic_band (High by default), user traffic at user_traffic_band;
  // banded endpoints may get dedicated ports, rationed by the configured cap.
  const rtps::DomainConfig engine_defaults{}; // the engine's own task defaults
  const rtps::DomainConfig domain_config{
      .metatraffic_band = config_.metatraffic_band,
      .user_traffic_band = config_.user_traffic_band,
      .enable_dedicated_endpoint_ports = config_.enable_dedicated_endpoint_ports,
      .max_prioritized_endpoint_ports = config_.max_prioritized_endpoint_ports,
      .local_prefix_length = prefix_length,
      .transport = transport_config(config_),
      .protocol_task = config_.tasks.protocol.value_or(engine_defaults.protocol_task),
  };
  domain_ = std::make_unique<rtps::Domain>(ip_bytes, domain_config);
  // Fresh liveness token for this run (a prior stop() left the old one flipped).
  live_ = std::make_shared<Liveness>();

  // Engine lifecycle: participants must be created before completeInit()
  // starts the discovery machinery; endpoints are added after.
  participant_ = domain_->createParticipant();
  if (participant_ == nullptr) {
    logger_.error("Could not create the RTPS participant");
    domain_.reset();
    return false;
  }

  if (config_.on_publisher_matched) {
    participant_->registerOnNewPublisherMatchedCallback(&publisher_matched_trampoline, this);
  }
  // Always hooked: besides the user's callback, a writer gaining a reader is
  // what a held service reply waits for (see subscriber_matched_trampoline).
  participant_->registerOnNewSubscriberMatchedCallback(&subscriber_matched_trampoline, this);

  if (!domain_->completeInit()) {
    logger_.error("RTPS domain init failed");
    participant_ = nullptr;
    domain_.reset();
    return false;
  }

#ifdef RTPS_WITH_RPC
  // The held-reply deadline timer, with the other tasks: never created while
  // traffic is served (see the member's comment). A failure is logged and
  // leaves the fallback in note_held_reply().
  {
    std::lock_guard<std::mutex> lock(held_replies_mutex_);
    (void)start_held_reply_timer_locked();
  }
#endif

  started_ = true;
  logger_.info("Started (interface {}.{}.{}.{})", ip_bytes[0], ip_bytes[1], ip_bytes[2],
               ip_bytes[3]);
  return true;
}

bool RtpsParticipant::add_writer(const WriterConfig &config) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!started_) {
    logger_.error("Cannot add writer '{}': not started", config.topic);
    return false;
  }
  if (writers_.count(config.topic) != 0) {
    logger_.error("Writer for topic '{}' already exists", config.topic);
    return false;
  }
  // A zero fragment size is invalid: the fragmented send paths treat it as
  // failure and would silently drop every large sample while publish() reports
  // success. Reject it up front rather than creating a broken writer.
  if (config.fragment_size == 0) {
    logger_.error("Writer '{}': fragment_size must be non-zero", config.topic);
    return false;
  }
  if (config.dscp.has_value() && static_cast<uint8_t>(config.dscp.value()) > 63) {
    logger_.error("Writer '{}': invalid DSCP {} (valid code points are 0..63)", config.topic,
                  static_cast<int>(config.dscp.value()));
    return false;
  }
  rtps::Writer *writer =
      domain_->createWriter(*participant_, config.topic.c_str(), config.type_name.c_str(),
                            config.reliability == Reliability::RELIABLE, /*enforceUnicast=*/false,
                            rtps::EndpointOptions{.band = config.band, .dscp = config.dscp});
  if (writer == nullptr) {
    // Name the exact failure: which limit bound, its configured size, and the
    // knob that raises it - so hitting a pool ceiling is a one-line config fix
    // instead of a debugging session (the builtin discovery endpoints consume
    // slots from these same pools, which makes the usable count non-obvious).
    if (config.topic.size() >= rtps::Config::MAX_TOPICNAME_LENGTH ||
        config.type_name.size() >= rtps::Config::MAX_TYPENAME_LENGTH) {
      // >= : the engine stores names in fixed arrays with a terminating NUL,
      // so a name of exactly MAX_*_LENGTH is rejected there too.
      logger_.error("Engine could not create writer '{}': topic/type name too long "
                    "(MAX_TOPICNAME_LENGTH={}, MAX_TYPENAME_LENGTH={})",
                    config.topic, static_cast<int>(rtps::Config::MAX_TOPICNAME_LENGTH),
                    static_cast<int>(rtps::Config::MAX_TYPENAME_LENGTH));
    } else if (config.reliability == Reliability::RELIABLE) {
      // Two limits can bind (whichever is hit first): the stateful pool and the
      // per-participant writer cap; the builtin discovery writers (1 SPDP + 2
      // SEDP) consume slots from both, hence the "usable" numbers.
      logger_.error(
          "Engine could not create writer '{}': RELIABLE writer capacity reached - stateful pool "
          "NUM_STATEFUL_WRITERS={} (2 reserved for SEDP -> {} usable) and/or per-participant cap "
          "NUM_WRITERS_PER_PARTICIPANT={} (3 builtin writers -> {} usable). Select a larger limits "
          "profile (CONFIG_RTPS_LIMITS_PROFILE_HOST or _HOST_LARGE; capacity-only, no wire "
          "change).",
          config.topic, static_cast<int>(rtps::Config::NUM_STATEFUL_WRITERS),
          static_cast<int>(rtps::Config::NUM_STATEFUL_WRITERS) - 2,
          static_cast<int>(rtps::Config::NUM_WRITERS_PER_PARTICIPANT),
          static_cast<int>(rtps::Config::NUM_WRITERS_PER_PARTICIPANT) - 3);
    } else {
      logger_.error(
          "Engine could not create writer '{}': BEST_EFFORT writer capacity reached - stateless "
          "pool NUM_STATELESS_WRITERS={} (1 reserved for SPDP -> {} usable) and/or per-participant "
          "cap NUM_WRITERS_PER_PARTICIPANT={} (3 builtin writers -> {} usable). Select a larger "
          "limits profile (CONFIG_RTPS_LIMITS_PROFILE_HOST or _HOST_LARGE; capacity-only, no wire "
          "change).",
          config.topic, static_cast<int>(rtps::Config::NUM_STATELESS_WRITERS),
          static_cast<int>(rtps::Config::NUM_STATELESS_WRITERS) - 1,
          static_cast<int>(rtps::Config::NUM_WRITERS_PER_PARTICIPANT),
          static_cast<int>(rtps::Config::NUM_WRITERS_PER_PARTICIPANT) - 3);
    }
    return false;
  }
  // Per-writer fragment size (only used when a sample exceeds a single DATA
  // submessage and fragmentation is compiled in; otherwise inert).
  writer->setFragmentSize(config.fragment_size);
  writers_[config.topic] = writer;
  logger_.info("Added {} writer: topic='{}' type='{}'",
               config.reliability == Reliability::RELIABLE ? "reliable" : "best-effort",
               config.topic, config.type_name);
  return true;
}

bool RtpsParticipant::add_reader(const ReaderConfig &config) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!started_) {
    logger_.error("Cannot add reader '{}': not started", config.topic);
    return false;
  }
  if (config.dscp.has_value() && static_cast<uint8_t>(config.dscp.value()) > 63) {
    logger_.error("Reader '{}': invalid DSCP {} (valid code points are 0..63)", config.topic,
                  static_cast<int>(config.dscp.value()));
    return false;
  }
  rtps::Reader *reader = domain_->createReader(
      *participant_, config.topic.c_str(), config.type_name.c_str(),
      config.reliability == Reliability::RELIABLE, /*mcastaddress=*/{0, 0, 0, 0},
      rtps::EndpointOptions{.band = config.band, .dscp = config.dscp});
  if (reader == nullptr) {
    // Same actionable diagnostics as add_writer(): name the bound limit, its
    // size, and the Kconfig knob (builtin discovery readers consume slots from
    // these pools: 1 stateless for SPDP, 2 stateful for SEDP).
    if (config.topic.size() >= rtps::Config::MAX_TOPICNAME_LENGTH ||
        config.type_name.size() >= rtps::Config::MAX_TYPENAME_LENGTH) {
      // >= : matches the engine's fixed-array + NUL bound (see add_writer()).
      logger_.error("Engine could not create reader '{}': topic/type name too long "
                    "(MAX_TOPICNAME_LENGTH={}, MAX_TYPENAME_LENGTH={})",
                    config.topic, static_cast<int>(rtps::Config::MAX_TOPICNAME_LENGTH),
                    static_cast<int>(rtps::Config::MAX_TYPENAME_LENGTH));
    } else if (config.reliability == Reliability::RELIABLE) {
      logger_.error(
          "Engine could not create reader '{}': RELIABLE reader capacity reached - stateful pool "
          "NUM_STATEFUL_READERS={} (2 reserved for SEDP -> {} usable) and/or per-participant cap "
          "NUM_READERS_PER_PARTICIPANT={} (3 builtin readers -> {} usable). Select a larger limits "
          "profile (CONFIG_RTPS_LIMITS_PROFILE_HOST or _HOST_LARGE; capacity-only, no wire "
          "change).",
          config.topic, static_cast<int>(rtps::Config::NUM_STATEFUL_READERS),
          static_cast<int>(rtps::Config::NUM_STATEFUL_READERS) - 2,
          static_cast<int>(rtps::Config::NUM_READERS_PER_PARTICIPANT),
          static_cast<int>(rtps::Config::NUM_READERS_PER_PARTICIPANT) - 3);
    } else {
      logger_.error(
          "Engine could not create reader '{}': BEST_EFFORT reader capacity reached - stateless "
          "pool NUM_STATELESS_READERS={} (1 reserved for SPDP -> {} usable) and/or per-participant "
          "cap NUM_READERS_PER_PARTICIPANT={} (3 builtin readers -> {} usable). Select a larger "
          "limits profile (CONFIG_RTPS_LIMITS_PROFILE_HOST or _HOST_LARGE; capacity-only, no wire "
          "change).",
          config.topic, static_cast<int>(rtps::Config::NUM_STATELESS_READERS),
          static_cast<int>(rtps::Config::NUM_STATELESS_READERS) - 1,
          static_cast<int>(rtps::Config::NUM_READERS_PER_PARTICIPANT),
          static_cast<int>(rtps::Config::NUM_READERS_PER_PARTICIPANT) - 3);
    }
    return false;
  }
  auto ctx = std::make_shared<ReaderContext>();
  ctx->self = this;
  ctx->on_sample = config.on_sample;
  ctx->topic = config.topic;
  ctx->reader = reader;
  // Banded reader without a dedicated port (ration exhausted or dedicated
  // ports disabled): fall back to deferred banded dispatch of on_sample (see
  // DeferredDispatch). Dedicated-port readers are already dispatched at their
  // band by the reactor, so they keep the inline path.
  if (config.band != espp::QosBand::Normal && !reader->m_attributes.hasDedicatedPort) {
    ctx->deferred.enabled = true;
    ctx->deferred.band = config.band;
    ctx->deferred.transport = &domain_->getTransport();
    logger_.info("Reader '{}' uses deferred banded dispatch (band {}, no dedicated port)",
                 config.topic, static_cast<int>(config.band));
  }
  if (config.on_sample) {
    if (reader->registerCallback(&reader_trampoline, ctx.get()) == 0) {
      logger_.error("Engine could not register the sample callback for '{}'", config.topic);
      return false;
    }
  }
  reader_contexts_.push_back(std::move(ctx));
  logger_.info("Added {} reader: topic='{}' type='{}'",
               config.reliability == Reliability::RELIABLE ? "reliable" : "best-effort",
               config.topic, config.type_name);
  return true;
}

bool RtpsParticipant::remove_writer(const std::string &topic) {
  std::lock_guard<std::mutex> lock(mutex_);
  // Reject once teardown has begun: after stop()'s phase-1.5 wait releases
  // mutex_, phase 4 runs domain_->stop() WITHOUT it, so holding mutex_ here no
  // longer serializes against the engine's own teardown - a deleteWriter()
  // racing domain_->stop() must not start. (domain_ is only reset in phase 5
  // under mutex_, so this check is what makes the dereference safe.) The
  // teardown reclaims every endpoint anyway, so a rejected removal during
  // stop leaks nothing.
  if (stopping_) {
    return false;
  }
  auto it = writers_.find(topic);
  if (it == writers_.end() || domain_ == nullptr || participant_ == nullptr) {
    return false;
  }
  // Delete the engine endpoint FIRST (announces the SEDP disposal and
  // releases any dedicated port); only then drop our handle. If deletion
  // fails the writer is still active in the engine - erasing the map entry
  // then would strand it with no handle to retry/publish through.
  if (!domain_->deleteWriter(*participant_, it->second)) {
    return false;
  }
  writers_.erase(it);
  return true;
}

std::size_t RtpsParticipant::matched_writers(const std::string &topic) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!started_ || stopping_) {
    return 0;
  }
  // Several readers may share a topic (add_reader() allows it, e.g. one
  // reliable and one best-effort): count the DISTINCT remote writers matched
  // to any of them, so a writer matched to two of our readers counts once and
  // a writer matched to the second reader only is not missed.
  std::vector<rtps::Guid_t> writers;
  const auto collect = [](const rtps::Reader *, const rtps::WriterProxy &proxy, void *arg) {
    auto *out = static_cast<std::vector<rtps::Guid_t> *>(arg);
    if (std::find(out->begin(), out->end(), proxy.remoteWriterGuid) == out->end()) {
      out->push_back(proxy.remoteWriterGuid);
    }
  };
  for (const auto &ctx : reader_contexts_) {
    if (ctx->topic == topic && ctx->reader != nullptr) {
      ctx->reader->dumpAllProxies(collect, &writers);
    }
  }
  return writers.size();
}

std::size_t RtpsParticipant::matched_readers(const std::string &topic) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!started_ || stopping_) {
    return 0;
  }
  const auto it = writers_.find(topic);
  if (it == writers_.end() || it->second == nullptr) {
    return 0;
  }
  return it->second->getProxiesCount();
}

bool RtpsParticipant::remove_reader(const std::string &topic) {
  // Supported from within the reader's OWN on_sample for INLINE (Normal-band /
  // non-deferred) readers: the engine's teardown drain excludes the caller's
  // own dispatch and the delivery trampoline holds the context alive across
  // the callback. NOT supported from within a DEFERRED reader's own callback:
  // the dispatcher's close() below waits for the in-flight delivery - i.e.
  // the caller - and would deadlock.
  //
  // Select the most-recent matching context and detach it from the list under
  // the lock (composites roll back most recent first). Detaching up front both
  // keeps a concurrent remover / stop() from processing it and lets us do the
  // engine deletion + quiesce WITHOUT holding mutex_: close() waits for the
  // in-flight delivery, and that user callback may itself call back into the
  // participant (e.g. publish()) and take mutex_ - holding it here would
  // deadlock. `target` keeps the context alive throughout.
  // Pin the engine for the unlocked phase below: stop() waits for active
  // engine ops before stopping/destroying the domain, so domain_/participant_
  // stay valid across the deletion + quiesce even if a stop() races in.
  if (!begin_engine_op()) {
    return false;
  }
  EngineOpGuard op_guard(*this);
  std::shared_ptr<ReaderContext> target;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = reader_contexts_.rbegin(); it != reader_contexts_.rend(); ++it) {
      if ((*it)->topic == topic && (*it)->reader != nullptr) {
        target = *it;
        reader_contexts_.erase(std::next(it).base());
        break;
      }
    }
  }
  if (!target) {
    return false;
  }
  // ENGINE deletion FIRST (clears the callback registration under the engine's
  // locks). On failure the reader is still live, so re-attach the (intact)
  // context for a retry rather than leaking it.
  if (!domain_->deleteReader(*participant_, target->reader)) {
    std::lock_guard<std::mutex> lock(mutex_);
    reader_contexts_.push_back(std::move(target));
    return false;
  }
  // Quiesce the dispatcher (waits for the in-flight delivery). The context is
  // freed when `target` goes out of scope here - after close() has drained.
  target->deferred.close();
  return true;
}

bool RtpsParticipant::publish(std::string_view topic, std::span<const uint8_t> cdr_payload) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!started_) {
    logger_.error("Cannot publish: not started");
    return false;
  }
  auto it = writers_.find(std::string(topic));
  if (it == writers_.end()) {
    logger_.error("No writer for topic '{}'", topic);
    return false;
  }
  // Reject rather than silently truncate.
#if defined(RTPS_ENABLE_FRAGMENTATION)
  // With fragmentation compiled in, the cap is the large-sample reassembly bound
  // (RTPS_MAX_SAMPLE_SIZE). Samples above one DATA submessage are split into
  // DATA_FRAG submessages by the engine writer.
  if (cdr_payload.size() > max_payload_size) {
    logger_.error(
        "Payload for topic '{}' is {} bytes, exceeds the {}-byte max sample size; dropped", topic,
        cdr_payload.size(), max_payload_size);
    return false;
  }
#else
  // Without fragmentation, the binding cap is the RTPS wire format: a single DATA
  // submessage must fit one UDP datagram and its 16-bit octetsToNextHeader. Keep
  // the facade constant in sync with the engine's actual computed bound.
  static_assert(RtpsParticipant::max_payload_size == rtps::MAX_UNFRAGMENTED_PAYLOAD);
  if (cdr_payload.size() > max_payload_size) {
    logger_.error("Payload for topic '{}' is {} bytes, exceeds the {}-byte RTPS limit; dropped "
                  "(large payloads need DATA_FRAG fragmentation, not enabled in this build)",
                  topic, cdr_payload.size(), max_payload_size);
    return false;
  }
#endif
  // KEEP_LAST overflow visibility: on a full static history ring the engine
  // OVERWRITES the oldest unsent sample and still accepts the new one, so a
  // saturated publisher would otherwise report nothing but success while
  // silently losing data. Detect the overwrite via the per-writer drop counter
  // delta across this call and surface it (rate-limited; the process-wide
  // total lives in rtps::Diagnostics::Writer::history_overwrite_drops).
  const uint32_t drops_before = it->second->historyDrops();
  const auto *change = it->second->newChange(rtps::ChangeKind_t::ALIVE, cdr_payload.data(),
                                             static_cast<rtps::DataSize_t>(cdr_payload.size()));
  if (change == nullptr) {
    logger_.warn("Writer history full for topic '{}'; sample dropped", topic);
    return false;
  }
  const uint32_t drops_now = it->second->historyDrops();
  if (drops_now != drops_before) {
    logger_.warn_rate_limited(
        "History overflow on topic '{}': publish outran the send path, oldest UNSENT sample "
        "overwritten (writer total: {}). This sample WAS queued (KEEP_LAST). Raise the history "
        "depth (RTPS_CFG_HISTORY_SIZE_STATELESS / _STATEFUL or menuconfig 'Custom capacity "
        "overrides'), enable RTPS_STORAGE_DYNAMIC, or pace the publisher.",
        topic, drops_now);
  }
  return true;
}

namespace {
// DeferredDispatch has no logger of its own (it is a small POD-ish helper
// embedded in several contexts), so drops are reported through this one.
espp::Logger s_deferred_logger({.tag = "RtpsDeferred", .level = espp::Logger::Verbosity::WARN});
} // namespace

void RtpsParticipant::DeferredDispatch::run_or_defer(std::function<void()> delivery,
                                                     std::shared_ptr<void> owner) {
  if (!enabled || transport == nullptr) {
    delivery();
    return;
  }
  bool do_arm = false;
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (closed) {
      return; // quiesced: the endpoint is being removed
    }
    if (queue.size() >= max_queued) {
      ++dropped;
      s_deferred_logger.warn(
          "Deferred delivery queue full ({}); dropping sample (total dropped {})", max_queued,
          dropped);
      return;
    }
    queue.push_back(std::move(delivery));
    if (!in_flight) {
      in_flight = true;
      do_arm = true;
    }
  }
  if (do_arm) {
    arm(std::move(owner));
  }
}

void RtpsParticipant::DeferredDispatch::arm(std::shared_ptr<void> owner) {
  // in_flight is already true (set by the caller under the mutex). The drain
  // job captures `owner` (the shared context embedding this dispatcher), so
  // queued work can never outlive the context. Guaranteed: when the pool is
  // full the transport parks the job (one per dispatcher, drain semantics)
  // and its retry task resubmits it, so a queued (possibly lone/last)
  // delivery is never stranded waiting for traffic that may not come.
  // Submitted under the dispatcher mutex so that close() - which marks the
  // dispatcher closed under it and then withdraws the parked job through
  // cancelGuaranteed() - either sees the arm refused here or cancels what it
  // parked: an arm slipping in between would park a job holding the removed
  // endpoint's context. The transport takes only its own pending-job mutex
  // under this one, never a dispatcher's.
  std::lock_guard<std::mutex> lock(mutex);
  if (closed) {
    in_flight = false;
    return;
  }
  transport->submitGuaranteedDrain(
      this, [this, owner]() { drain(owner); }, band);
}

void RtpsParticipant::DeferredDispatch::close() {
  {
    std::lock_guard<std::mutex> lock(mutex);
    closed = true;
    queue.clear();
  }
  // Withdraw a parked drain job (it holds the owner): after this the
  // transport's retry task cannot resubmit it. A job it already handed to the
  // pool runs drain(), which sees `closed` and returns. Either way no drain
  // is queued or parked for this dispatcher any more: say so in its state.
  if (transport != nullptr) {
    transport->cancelGuaranteed(this);
  }
  {
    std::lock_guard<std::mutex> lock(mutex);
    in_flight = false;
  }
  // Wait for any in-flight delivery to finish. A delivery runs a user handler
  // that may still be using endpoints the caller is about to delete (e.g. a
  // service reply writer via ServiceResponder::reply()); close() must not
  // return until it completes. The caller must NOT hold Participant::mutex_
  // here (the delivery may need it) - every remove_* path calls close()
  // outside that lock.
  std::unique_lock<std::mutex> lock(mutex);
  drain_done.wait(lock, [this]() { return !delivering; });
}

void RtpsParticipant::DeferredDispatch::drain(std::shared_ptr<void> owner) {
  // One delivery per job (mirrors the reactor's one-shot arming): pop the
  // oldest, run it OUTSIDE the lock, then re-arm while more are pending.
  std::function<void()> delivery;
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (closed || queue.empty()) {
      in_flight = false;
      return;
    }
    delivery = std::move(queue.front());
    queue.pop_front();
    // Mark a delivery as executing so close() can wait for it before the
    // caller deletes endpoints the delivery may still use (e.g. a service
    // reply writer). Runs OUTSIDE the lock below; close() waits on drain_done.
    delivering = true;
  }
  // Exception boundary, mirroring SocketReactor::dispatch(): this runs as a
  // plain pool job, and a throwing user callback would otherwise kill the
  // worker AND leave in_flight set, permanently wedging this dispatcher.
#if defined(__cpp_exceptions) && __cpp_exceptions
  try {
    delivery();
  } catch (const std::exception &e) {
    s_deferred_logger.error("Exception in deferred delivery: {}", e.what());
  } catch (...) {
    s_deferred_logger.error("Unknown exception in deferred delivery");
  }
#else
  // C++ exceptions are disabled (e.g. the ESP-IDF default), so a throwing
  // delivery would abort regardless; call it directly.
  delivery();
#endif
  bool rearm = false;
  {
    std::lock_guard<std::mutex> lock(mutex);
    delivering = false;
    drain_done.notify_all(); // wake close() if it is waiting for us
    if (closed || queue.empty()) {
      in_flight = false;
    } else {
      rearm = true;
    }
  }
  if (rearm) {
    // Same guaranteed-recovery path as the initial arm: a rejected re-arm
    // the transport parks the drain job and its retry task resubmits it.
    arm(std::move(owner));
  }
}

void RtpsParticipant::reader_trampoline(void *arg, const rtps::ReaderCacheChange &change) {
  auto *ctx = static_cast<ReaderContext *>(arg);
  if (ctx == nullptr || !ctx->on_sample) {
    return;
  }
  if (change.kind != rtps::ChangeKind_t::ALIVE) {
    // A dispose/unregister (PID_STATUS_INFO in the inline QoS) carries no
    // sample; user topics here are keyless, so there is no instance to retire
    // either. Discovery readers handle theirs in the SEDP/SPDP agents.
    return;
  }
  if (ctx->deferred.enabled) {
    // Banded shared-port reader: copy the payload now (`change` is only valid
    // during this callback) and deliver it from the pool at the reader's band.
    // shared_ptr payload (rather than a moved vector) keeps the closure
    // cheaply copyable inside std::function and avoids a GCC 15
    // -Wfree-nonheap-object false positive on moved-vector captures.
    auto sample = std::make_shared<std::vector<uint8_t>>(change.getDataSize());
    if (sample->empty() || !change.copyInto(sample->data(), change.getDataSize())) {
      return;
    }
    // Shared capture: the delivery (and the drain job) own the context, so a
    // concurrent remove/rollback cannot free it under queued work; close()
    // (run by every removal path) stops further deliveries.
    auto self = ctx->shared_from_this();
    ctx->deferred.run_or_defer(
        [self, sample]() {
          self->on_sample(std::span<const uint8_t>(sample->data(), sample->size()));
        },
        self);
    return;
  }
  // Serialize deliveries per reader: the engine may invoke this from a worker
  // thread while a previous delivery is still running. Hold a shared reference
  // for the delivery's duration: the callback may legally remove its OWN
  // reader (remove_reader() from on_sample), which detaches and releases the
  // registry's reference while this invocation is still on the stack - the
  // engine's teardown drain deliberately excludes the caller's own dispatch,
  // so without this hold the context would be destroyed under the callback.
  auto self = ctx->shared_from_this();
  std::lock_guard<std::mutex> lock(self->buffer_mutex);
  const auto size = change.getDataSize();
  self->buffer.resize(size);
  if (size == 0 || !change.copyInto(self->buffer.data(), size)) {
    return;
  }
  self->on_sample(std::span<const uint8_t>(self->buffer.data(), self->buffer.size()));
}

void RtpsParticipant::publisher_matched_trampoline(void *arg) {
  auto *self = static_cast<RtpsParticipant *>(arg);
  if (self != nullptr && self->config_.on_publisher_matched) {
    self->config_.on_publisher_matched();
  }
}

void RtpsParticipant::subscriber_matched_trampoline(void *arg) {
  auto *self = static_cast<RtpsParticipant *>(arg);
  if (self == nullptr) {
    return;
  }
  if (self->config_.on_subscriber_matched) {
    self->config_.on_subscriber_matched();
  }
#ifdef RTPS_WITH_RPC
  // A local writer gained a remote reader: a held service reply may be waiting
  // for exactly this match. Runs on the SEDP receive worker under the SEDP
  // mutex; the sweep's locks (participant liveness -> endpoint liveness -> held
  // table -> writer) are only ever taken after it, never before it.
  self->flush_held_replies();
#endif
}

#ifdef RTPS_WITH_RPC
// ===========================================================================
// Services (RMI) - request/reply correlation. Two wire forms, chosen per peer
// by its vendor id:
//   - related_sample_identity inline QoS on request and reply: rmw_fastrtps_cpp,
//     espp itself, DDS-RPC (see rpc/sample_identity.hpp);
//   - a 16-byte {guid, seq} header inside the payload of request and reply:
//     rmw_cyclonedds_cpp (see rpc/cyclonedds_request_header.hpp).
// A server answers each request in the form it arrived in; a client picks the
// form from the vendor of its matched request readers.
// ===========================================================================

namespace {
// Pack a SequenceNumber_t into a single key for the pending-request map.
uint64_t seq_key(const rtps::SequenceNumber_t &sn) {
  return (static_cast<uint64_t>(static_cast<uint32_t>(sn.high)) << 32) | sn.low;
}

// The promise behind call_future(), completed exactly once. Three parties may
// try: the reply, the abandonment (request_timeout or stop(), through the
// pending entry's on_abandon) and the send-failure fallback in call_future()
// itself - and the last two can meet: the native client inserts its entry
// before publishing, so a stop() in between abandons the entry (fulfilling
// the promise) and then the failed publish makes send() return nullopt. A
// second set_value would throw std::future_error; the flag makes it a no-op.
struct FutureReply {
  std::promise<std::optional<std::vector<uint8_t>>> promise;
  std::atomic<bool> done{false};
  void complete(std::optional<std::vector<uint8_t>> value) {
    if (!done.exchange(true)) {
      promise.set_value(std::move(value));
    }
  }
};
} // namespace

// Per-server bridge: engine request-reader callback -> user handler -> reply.
struct RtpsParticipant::ServiceServerContext
    : std::enable_shared_from_this<RtpsParticipant::ServiceServerContext> {
  RtpsParticipant *self{nullptr};
  std::string service;                         // for the responder's diagnostics
  service_deferred_handler_t handler{nullptr}; // sync handlers are wrapped as deferred
  rtps::Writer *reply_writer{nullptr};
  rtps::Reader *request_reader{nullptr};
  DeferredDispatch deferred; // banded request reader without a dedicated port
  // Endpoint-scoped liveness for RETAINED ServiceResponders: a responder may
  // legally outlive the handler invocation, and the participant-wide token
  // only covers stop() - an individual removal (e.g. an action rollback)
  // deletes/reuses this reply writer while the participant stays alive.
  // remove_service_server() flips this false (under its lock, which waits for
  // an in-flight reply()) BEFORE the writer is deleted.
  std::shared_ptr<Liveness> writer_live{std::make_shared<Liveness>()};

  // A reply in wire form, and the reader whose acknowledgement it waits for.
  // Replies are held here until the requester's reply reader is matched to
  // reply_writer AND has acknowledged it (or the participant's
  // service_reply_match_timeout passes); see ServiceResponder::reply() for why.
  struct HeldReply {
    std::vector<uint8_t> payload; // the Cyclone header, when one applies, already spliced in
    bool cyclonedds{false};
    rtps::rpc::SampleIdentity related{}; // inline-QoS form only
    rtps::Guid_t reply_reader{};         // the exact reader to wait for, when known
    bool reply_reader_known{false};
    rtps::GuidPrefix_t requester{}; // else: any reader of the requester's participant
    std::chrono::steady_clock::time_point deadline{};
    uint32_t token{0}; // names the entry while it is held (see reply())
  };
  static constexpr std::size_t MAX_HELD_REPLIES = 8;
  std::mutex held_mutex;
  std::vector<HeldReply> held;
  uint32_t held_token_seq{0}; // under held_mutex

  // Matched, and acknowledged where the reader is reliable (every reply
  // reader is): a VOLATILE Cyclone DDS reader skips everything the first
  // HEARTBEAT it processes announces, so the reply must not be written before
  // the reader has answered one (ReaderProxy::acknowledged).
  static bool reader_matched(rtps::Writer &writer, const HeldReply &reply) {
    return reply.reply_reader_known ? writer.hasAcknowledgedProxy(reply.reply_reader)
                                    : writer.hasAcknowledgedProxyOfParticipant(reply.requester);
  }

  // The engine write of a reply, in the requester's form. The caller holds the
  // liveness locks that keep `writer` valid. False when the writer's history
  // is full (more replies in flight than its depth): the client never hears
  // back, and the caller says so, as publish() does.
  static bool write(rtps::Writer &writer, const HeldReply &reply) {
    if (reply.cyclonedds) {
      return writer.newChange(rtps::ChangeKind_t::ALIVE, reply.payload.data(),
                              static_cast<rtps::DataSize_t>(reply.payload.size())) != nullptr;
    }
    return writer.newChangeWithRelatedSampleIdentity(
               rtps::ChangeKind_t::ALIVE, reply.payload.data(),
               static_cast<rtps::DataSize_t>(reply.payload.size()), reply.related) != nullptr;
  }
};

// Deferred-reply state: the reply writer + the identity to echo, so a response
// can be sent once, later, from any thread.
struct RtpsParticipant::ServiceResponder::State {
  rtps::Writer *reply_writer{nullptr};
  RtpsParticipant *self{nullptr}; // only dereferenced while `live` says alive
  std::string service;
  rtps::rpc::SampleIdentity related{}; // inline-QoS form (unused when cyclonedds)
  // Set when the request came from an rmw_cyclonedds_cpp client: the reply
  // echoes its in-payload {guid, seq} header as a plain DATA, with no inline QoS.
  bool cyclonedds{false};
  rtps::rpc::CycloneDdsRequestHeader cyclonedds_header{};
  std::atomic<bool> replied{false};
  // Held so a deferred reply that races participant shutdown no-ops instead of
  // writing through a freed engine writer (see RtpsParticipant::Liveness).
  std::shared_ptr<Liveness> live;
  // Endpoint-scoped: invalidated by remove_service_server() BEFORE the reply
  // writer is deleted, so a responder retained past an individual removal
  // (e.g. an action rollback) no-ops instead of writing through a
  // reset/reused writer slot while the participant is still alive.
  std::shared_ptr<Liveness> endpoint_live;
  // Whose match the reply waits for (see reply()): the exact reply reader when
  // the request named one, else any reader of the requester's participant.
  rtps::Guid_t reply_reader{};
  bool reply_reader_known{false};
  rtps::GuidPrefix_t requester{};
  std::weak_ptr<ServiceServerContext> server; // holds the reply while it waits
};

void RtpsParticipant::ServiceResponder::reply(std::span<const uint8_t> response) const {
  if (!state_ || state_->reply_writer == nullptr || !state_->live) {
    return;
  }
  // A reply must fit a single DATA submessage: with inline QoS a fragmented
  // reply loses the correlation (see MAX_UNFRAGMENTED_RPC_PAYLOAD), and the
  // Cyclone header counts against the same budget so both client kinds get the
  // same limit. Reject rather than send an uncorrelated reply, and say so: a
  // deferred handler has no other way to learn that its reply was dropped.
  const std::size_t wire_size =
      response.size() +
      (state_->cyclonedds ? rtps::rpc::CYCLONEDDS_REQUEST_HEADER_SIZE : std::size_t{0});
  const char *dropped = nullptr;
  if (response.empty()) {
    // Nothing to send: a 0-byte DATA is a deserialization error on the peer,
    // not a timeout, and nothing would say why. A handler that cannot answer
    // should let the client time out; a ROS 2 reply is never empty (an empty
    // message is one placeholder byte).
    dropped = "it is empty (a reply must be a CDR-encapsulated message)";
  } else if (wire_size > rtps::MAX_UNFRAGMENTED_RPC_PAYLOAD) {
    dropped = "it exceeds the single-DATA budget";
  } else if (state_->cyclonedds && !rtps::rpc::isCdrEncapsulation(response)) {
    dropped = "it does not start with a CDR encapsulation to splice the Cyclone header behind";
  }
  if (dropped != nullptr) {
    std::lock_guard<std::mutex> live_lock(state_->live->m);
    if (state_->live->alive && state_->self != nullptr) {
      state_->self->logger_.warn(
          "Service '{}': reply of {} bytes ({} on the wire) dropped, {} (limit {} bytes)",
          state_->service, response.size(), wire_size, dropped, rtps::MAX_UNFRAGMENTED_RPC_PAYLOAD);
    }
    return;
  }
  bool expected = false;
  if (!state_->replied.compare_exchange_strong(expected, true)) {
    return; // reply exactly once
  }
  // Hold BOTH liveness locks across the write: stop() flips the participant
  // token false and remove_service_server() flips the endpoint token false -
  // each under its own lock, before the writer is destroyed/deleted - so we
  // either complete the write against a still-valid writer or observe !alive
  // and drop. Lock order participant -> endpoint (removal only ever takes the
  // endpoint lock alone, so there is no reverse nesting).
  std::lock_guard<std::mutex> live_lock(state_->live->m);
  if (!state_->live->alive) {
    return;
  }
  std::unique_lock<std::mutex> endpoint_lock;
  if (state_->endpoint_live) {
    endpoint_lock = std::unique_lock<std::mutex>(state_->endpoint_live->m);
    if (!state_->endpoint_live->alive) {
      return;
    }
  }
  ServiceServerContext::HeldReply wire{};
  wire.cyclonedds = state_->cyclonedds;
  wire.related = state_->related;
  wire.reply_reader = state_->reply_reader;
  wire.reply_reader_known = state_->reply_reader_known;
  wire.requester = state_->requester;
  if (state_->cyclonedds) {
    wire.payload = rtps::rpc::prependCycloneDdsRequestHeader(state_->cyclonedds_header, response);
    if (wire.payload.empty()) {
      return; // cannot happen past the encapsulation check above; never send a bare DATA
    }
  } else {
    wire.payload.assign(response.begin(), response.end());
  }
  // Written at once when the requester's reply reader is matched to the reply
  // writer and has acknowledged it. Until then it is HELD: the client sent its
  // request as soon as its own side had matched, and this writer may learn of
  // the client's reply reader (its SEDP record) a little after the request was
  // served. A change written before that match is never sent to the reader -
  // the writer sends new changes to the readers it has - and the first
  // HEARTBEAT the reader then sees announces it as history, which a VOLATILE
  // reader skips: the call times out although it was served. A Cyclone DDS
  // reader applies that rule to the first heartbeat it PROCESSES, so under
  // loss the match alone is not enough: a reply written between the match and
  // the reader's answer to our heartbeat was skipped the same way (the QEMU
  // interop matrix's service call under 15 % loss, one call in 25). Hence the
  // acknowledgement. rmw_fastrtps / rmw_cyclonedds servers wait for the same
  // match before writing a response. The hold ends with the acknowledgement
  // (sweep_held_replies polls for it, the match flushes at once) or after
  // Config::service_reply_match_timeout.
  const auto timeout = state_->self->config_.service_reply_match_timeout;
  auto server = state_->server.lock();
  if (server == nullptr || timeout <= std::chrono::milliseconds{0} ||
      ServiceServerContext::reader_matched(*state_->reply_writer, wire)) {
    if (!ServiceServerContext::write(*state_->reply_writer, wire)) {
      state_->self->logger_.warn("Service '{}': reply dropped, the reply writer's history is full",
                                 state_->service);
    }
    return;
  }
  wire.deadline = std::chrono::steady_clock::now() + timeout;
  bool held = false;
  uint32_t token = 0;
  {
    std::lock_guard<std::mutex> held_lock(server->held_mutex);
    if (server->held.size() < ServiceServerContext::MAX_HELD_REPLIES) {
      token = ++server->held_token_seq;
      wire.token = token;
      server->held.push_back(wire);
      held = true;
    }
  }
  if (held && !state_->self->note_held_reply(server)) {
    // No deadline timer: take the reply back (unless the match flushed it
    // meanwhile) and write it now rather than hold it with nothing to end the
    // hold.
    std::lock_guard<std::mutex> held_lock(server->held_mutex);
    const auto it =
        std::find_if(server->held.begin(), server->held.end(),
                     [&](const ServiceServerContext::HeldReply &r) { return r.token == token; });
    if (it == server->held.end()) {
      return; // already written by the match
    }
    server->held.erase(it);
    held = false;
  }
  if (!held) {
    // Never lose a reply to the cap or to a missing timer: it goes out now, as
    // before the hold existed.
    state_->self->logger_.warn(
        "Service '{}': reply written without waiting for its reply reader ({})", state_->service,
        token == 0 ? "the held-reply table is full" : "no deadline timer could be created");
    if (!ServiceServerContext::write(*state_->reply_writer, wire)) {
      state_->self->logger_.warn("Service '{}': reply dropped, the reply writer's history is full",
                                 state_->service);
    }
  }
}

bool RtpsParticipant::note_held_reply(const std::shared_ptr<ServiceServerContext> &server) {
  std::lock_guard<std::mutex> lock(held_replies_mutex_);
  const bool listed =
      std::any_of(servers_with_held_replies_.begin(), servers_with_held_replies_.end(),
                  [&](const std::weak_ptr<ServiceServerContext> &w) {
                    return !w.owner_before(server) && !server.owner_before(w);
                  });
  if (!listed) {
    servers_with_held_replies_.push_back(server);
  }
  if (held_replies_timer_) {
    return true;
  }
  // The timer is normally created at start(); this is the fallback for a
  // start() that could not create it (Task::start() said why then). A task
  // created while traffic is served is the one whose creation fails - and on
  // ESP a failed pthread_create terminates the program outright (exceptions
  // are off) - so this path is not expected to run.
  if (!start_held_reply_timer_locked()) {
    std::erase_if(servers_with_held_replies_, [&](const std::weak_ptr<ServiceServerContext> &w) {
      return !w.owner_before(server) && !server.owner_before(w);
    });
    return false;
  }
  return true;
}

bool RtpsParticipant::start_held_reply_timer_locked() {
  // One deadline timer per participant, kept until stop() (see the member's
  // comment). The match itself flushes a held reply
  // (subscriber_matched_trampoline), so this only polls for the reader's
  // acknowledgement and enforces the deadline: a tenth of the default timeout
  // is plenty, and a tick with nothing held is a lock and an empty check.
  espp::Timer::Config timer_config{
      .name = "rtps_reply_hold",
      .period = std::chrono::milliseconds(100),
      .delay = std::chrono::milliseconds(100),
      .callback = [this]() -> bool {
        sweep_held_replies();
        return false; // never self-cancel (a cancelled espp::Timer cannot restart)
      },
      .auto_start = true,
      .log_level = espp::Logger::Verbosity::WARN,
  };
  if (config_.tasks.reply_hold) {
    // Timer::Config carries the task's stack, priority and core directly.
    timer_config.stack_size_bytes = config_.tasks.reply_hold->stack_size_bytes;
    timer_config.priority = config_.tasks.reply_hold->priority;
    timer_config.core_id = config_.tasks.reply_hold->core_id;
  }
  held_replies_timer_ = std::make_unique<espp::Timer>(timer_config);
  if (!held_replies_timer_->is_running()) {
    held_replies_timer_.reset();
    logger_.error("Held-reply deadline timer could not be started; replies are written without "
                  "waiting for their reply reader");
    return false;
  }
  return true;
}

void RtpsParticipant::flush_held_replies() {
  {
    std::lock_guard<std::mutex> lock(held_replies_mutex_);
    if (servers_with_held_replies_.empty()) {
      return; // the common case: nothing is waiting
    }
  }
  sweep_held_replies();
}

bool RtpsParticipant::sweep_held_replies() {
  std::vector<std::shared_ptr<ServiceServerContext>> servers;
  {
    std::lock_guard<std::mutex> lock(held_replies_mutex_);
    std::erase_if(servers_with_held_replies_,
                  [](const std::weak_ptr<ServiceServerContext> &w) { return w.expired(); });
    for (const auto &w : servers_with_held_replies_) {
      if (auto s = w.lock()) {
        servers.push_back(std::move(s));
      }
    }
  }
  const auto now = std::chrono::steady_clock::now();
  for (const auto &server : servers) {
    {
      // Same locks and order as ServiceResponder::reply(): the writer is valid
      // only while both liveness tokens say so.
      std::lock_guard<std::mutex> live_lock(live_->m);
      if (!live_->alive) {
        return false; // stopping: stop() forgets the held replies
      }
      std::lock_guard<std::mutex> endpoint_lock(server->writer_live->m);
      if (!server->writer_live->alive || server->reply_writer == nullptr) {
        continue; // being removed: remove_service_server() drops what it held
      }
      std::vector<ServiceServerContext::HeldReply> due;
      {
        std::lock_guard<std::mutex> held_lock(server->held_mutex);
        std::erase_if(server->held, [&](ServiceServerContext::HeldReply &reply) {
          const bool matched = ServiceServerContext::reader_matched(*server->reply_writer, reply);
          if (!matched && now < reply.deadline) {
            return false;
          }
          if (!matched) {
            logger_.warn("Service '{}': reply reader not matched within {} ms; writing the "
                         "reply anyway",
                         server->service, config_.service_reply_match_timeout.count());
          }
          due.push_back(std::move(reply));
          return true;
        });
      }
      for (const auto &reply : due) {
        if (!ServiceServerContext::write(*server->reply_writer, reply)) {
          logger_.warn("Service '{}': reply dropped, the reply writer's history is full",
                       server->service);
        }
      }
    }
    // Forget a server with nothing left - under BOTH locks, so a reply held
    // between the check and the erase is not stranded (reply() pushes under
    // held_mutex and lists the server under held_replies_mutex_ afterwards).
    std::lock_guard<std::mutex> lock(held_replies_mutex_);
    std::lock_guard<std::mutex> held_lock(server->held_mutex);
    if (server->held.empty()) {
      std::erase_if(servers_with_held_replies_, [&](const std::weak_ptr<ServiceServerContext> &w) {
        return !w.owner_before(server) && !server.owner_before(w);
      });
    }
  }
  std::lock_guard<std::mutex> lock(held_replies_mutex_);
  return !servers_with_held_replies_.empty();
}

// Client state: request writer + pending-request table. Against an inline-QoS
// server the key is the request's RTPS writerSeqNumber (echoed in the reply's
// related_sample_identity), matched on our own reply-reader GUID. Against an
// rmw_cyclonedds_cpp server the key is CYCLONEDDS_KEY_BIT | our header seq
// (echoed in the reply's in-payload header), matched on our client id.
struct RtpsParticipant::ServiceClient::Impl
    : std::enable_shared_from_this<RtpsParticipant::ServiceClient::Impl> {
  struct SyncSlot {
    std::mutex m;
    std::condition_variable cv;
    bool done{false};
    bool failed{false}; // abandoned (timed out, or the participant stopped)
    std::vector<uint8_t> reply;
  };
  struct Pending {
    reply_callback_t on_reply{nullptr};      // set for call_async
    std::shared_ptr<SyncSlot> sync{nullptr}; // set for call
    // When the request is given up on (ServiceConfig::request_timeout; max()
    // = never) and what to tell then (call_future's promise). A reply that
    // lands after that is a late reply, ignored like any unknown key.
    std::chrono::steady_clock::time_point deadline{std::chrono::steady_clock::time_point::max()};
    std::function<void()> on_abandon{nullptr};
  };

  RtpsParticipant *self{nullptr};
  rtps::Writer *request_writer{nullptr};
  rtps::Reader *reply_reader{nullptr}; ///< retained for composite (action) rollback
  rtps::Guid_t reply_reader_guid{};
  std::chrono::milliseconds request_timeout{0}; ///< ServiceConfig::request_timeout
  std::size_t max_pending{0};                   ///< ServiceConfig::max_pending_requests
  // The participant run this client was created in (RtpsParticipant::live_ at
  // that time). stop() flips it before the engine - and with it request_writer
  // / reply_reader - goes away, and a later start() mints a new token, so a
  // handle kept across a stop() can never dereference the stale endpoints.
  std::shared_ptr<Liveness> live;

  // Engine access from a client handle: the user may call server_count() /
  // call_async() / wait_for_server() from any thread at any time, including
  // while another thread stop()s the participant, or on a handle retained past
  // a stop(). Every engine touch (proxy walks, newChange) is pinned as an
  // engine operation so stop() waits at its phase 1.5, and then checked
  // against the run's liveness token: with the op pinned stop() cannot have
  // passed phase 2 (where the token flips), so a still-alive token means the
  // endpoints are this run's and valid.
  struct Pin {
    RtpsParticipant *participant{nullptr};
    Pin() = default;
    explicit Pin(RtpsParticipant *p)
        : participant(p) {}
    Pin(const Pin &) = delete;
    Pin &operator=(const Pin &) = delete;
    Pin(Pin &&other) noexcept
        : participant(other.participant) {
      other.participant = nullptr;
    }
    ~Pin() {
      if (participant != nullptr) {
        participant->end_engine_op();
      }
    }
    explicit operator bool() const { return participant != nullptr; }
  };
  Pin pin() const {
    if (self == nullptr || !live) {
      return {};
    }
    // The token first, and held while the op is registered: a handle may
    // outlive its participant (the user keeps the shared_ptr), and then `self`
    // dangles. stop() - which the destructor runs - flips the token under its
    // mutex (phase 2) before anything is torn down, so while the token is
    // held alive here the participant exists and `self` may be touched; the
    // registered op then keeps it from passing phase 1.5. Lock order
    // Liveness::m -> mutex_ (begin_engine_op): nothing takes them reversed.
    std::lock_guard<std::mutex> lock(live->m);
    if (!live->alive || !self->begin_engine_op()) {
      return {};
    }
    return Pin{self};
  }
  /// True while the participant run this client belongs to is up.
  bool running() const { return static_cast<bool>(pin()); }
  /// Diagnostics for a refused request (see send()): logged through the
  /// participant while it is alive (the token, as in pin(): a handle that
  /// outlived its participant refuses silently).
  void refuse(const char *what, const char *why) const {
    if (self == nullptr || !live) {
      return;
    }
    std::lock_guard<std::mutex> lock(live->m);
    if (live->alive) {
      self->logger_.warn("Service client: {} refused, {}", what, why);
    }
  }
  // Cyclone form: a packed RTPS sequence number never has bit 63 set (its high
  // word is a non-negative int32), so tagging the header seq with it keeps the
  // two key spaces of `pending` apart.
  static constexpr uint64_t CYCLONEDDS_KEY_BIT = uint64_t{1} << 63;
  uint64_t cyclonedds_client_id{0}; ///< the header's guid field, ours to choose
  /// The header's seq field, per request: an int64 like the field itself, kept
  /// positive (wraps to 1 after INT64_MAX, beyond any device's lifetime) so the
  /// pending-map key CYCLONEDDS_KEY_BIT | seq never sees a sign bit.
  int64_t cyclonedds_next_seq{0};
  std::mutex mutex;
  std::unordered_map<uint64_t, Pending> pending;
  DeferredDispatch deferred; // banded reply reader without a dedicated port

  // Giving up on requests. The table is bounded two ways: entries past their
  // deadline are dropped whenever the table is touched (a send, a count), and
  // stop() drops every entry before the engine goes down. An abandoned call()
  // wakes with `failed`, an abandoned call_future() resolves to nullopt
  // through on_abandon; a call_async() callback is simply never run. Always
  // finish the gone entries OUTSIDE `mutex`: on_abandon may run user code.
  static void finish_abandoned(std::vector<Pending> &gone) {
    for (auto &p : gone) {
      if (p.sync) {
        std::lock_guard<std::mutex> lock(p.sync->m);
        p.sync->failed = true;
        p.sync->done = true;
        p.sync->cv.notify_all();
      }
      if (p.on_abandon) {
        p.on_abandon();
      }
    }
    gone.clear();
  }
  void prune_expired_locked(std::vector<Pending> &gone) {
    const auto now = std::chrono::steady_clock::now();
    for (auto it = pending.begin(); it != pending.end();) {
      if (it->second.deadline <= now) {
        gone.push_back(std::move(it->second));
        it = pending.erase(it);
      } else {
        ++it;
      }
    }
  }
  void abandon_all() {
    std::vector<Pending> gone;
    {
      std::lock_guard<std::mutex> lock(mutex);
      gone.reserve(pending.size());
      std::transform(pending.begin(), pending.end(), std::back_inserter(gone),
                     [](auto &kv) { return std::move(kv.second); });
      pending.clear();
    }
    finish_abandoned(gone);
  }
  std::size_t pending_count() {
    std::vector<Pending> gone;
    std::size_t count = 0;
    {
      std::lock_guard<std::mutex> lock(mutex);
      prune_expired_locked(gone);
      count = pending.size();
    }
    finish_abandoned(gone);
    return count;
  }

  // What is on the other end of the request writer right now.
  struct PeerForm {
    std::size_t matched{0}; ///< request readers (servers) matched to this client
    bool cyclonedds{false}; ///< every one of them is an Eclipse Cyclone DDS participant
  };

  // The distinct participants behind the acknowledged, matched request readers.
  struct Prefixes {
    std::array<rtps::GuidPrefix_t, rtps::Config::NUM_READER_PROXIES_PER_WRITER> items{};
    std::size_t count{0};   ///< distinct prefixes stored
    std::size_t proxies{0}; ///< readers counted (acknowledged reliable, or best-effort)
    bool overflow{false};   ///< a distinct prefix did not fit: the vendor check is partial
  };
  static void collect_prefix(const rtps::Writer *, const rtps::ReaderProxy &proxy, void *arg) {
    auto *out = static_cast<Prefixes *>(arg);
    // A reliable reader counts once it has acknowledged us (see
    // ReaderProxy::acknowledged); until then the server may not have
    // discovered our writer yet, and its VOLATILE request reader would
    // drop a request sent now. A best-effort reader never acknowledges.
    if (proxy.is_reliable && !proxy.acknowledged) {
      return;
    }
    ++out->proxies;
    for (std::size_t i = 0; i < out->count; ++i) {
      if (out->items[i] == proxy.remoteReaderGuid.prefix) {
        return;
      }
    }
    if (out->count < out->items.size()) {
      out->items[out->count++] = proxy.remoteReaderGuid.prefix;
    } else {
      out->overflow = true;
    }
  }

  // Servers matched right now, both legs (see peer_form()), without the
  // per-participant vendor lookups: what wait_for_server() polls.
  std::size_t matched_servers() const {
    const Pin pinned = pin();
    if (!pinned) {
      return 0; // not started, stopping, or a handle from an earlier run
    }
    Prefixes prefixes;
    request_writer->dumpAllProxies(&collect_prefix, &prefixes);
    const std::size_t reply_writers = reply_reader != nullptr ? reply_reader->getProxiesCount() : 0;
    return std::min(prefixes.proxies, reply_writers);
  }

  // `cyclonedds` means the servers ignore inline QoS and expect the in-payload
  // header (vendor 01.10). Mixed vendors keep the inline-QoS form, which
  // rmw_fastrtps_cpp, espp and DDS-RPC servers speak. Decided per request so a
  // server that appears later is served in its form without re-creating the
  // client. Walks the writer's proxies (writer lock), then looks each distinct
  // participant up (participant lock): never nested, and never under `mutex`,
  // so a reply landing meanwhile does not wait for the walk.
  PeerForm peer_form() const {
    // Called under send()'s pin; the walk below needs the engine.
    Prefixes prefixes;
    request_writer->dumpAllProxies(&collect_prefix, &prefixes);
    // A server counts only when both legs are matched: its request reader
    // (a proxy on our request writer that has acknowledged us) and its reply
    // writer (a proxy on our reply reader), the two legs
    // rmw_service_server_is_available checks. The acknowledgement is what
    // closes the window rclcpp's wait_for_service leaves open: a Cyclone
    // server discovers our endpoints from espp's SEDP replay up to a heartbeat
    // period after we discovered its, and a request sent in between reached
    // a reader that did not know our writer yet and was dropped for good.
    const std::size_t reply_writers = reply_reader != nullptr ? reply_reader->getProxiesCount() : 0;
    PeerForm form{.matched = std::min(prefixes.proxies, reply_writers), .cyclonedds = false};
    if (form.matched == 0 || self == nullptr || self->participant_ == nullptr) {
      return form;
    }
    std::size_t cyclonedds = 0;
    for (std::size_t i = 0; i < prefixes.count; ++i) {
      const auto *remote = self->participant_->findRemoteParticipant(prefixes.items[i]);
      if (remote != nullptr && rtps::rpc::isCycloneDds(remote->m_vendorId)) {
        ++cyclonedds;
      }
    }
    // Only when every distinct participant was seen and is Cyclone; a set that
    // overflowed the array was only partially checked, so it keeps the inline
    // form, which every other server kind accepts.
    form.cyclonedds = !prefixes.overflow && (cyclonedds == prefixes.count);
    return form;
  }

  // Send a request in the matched server's form, register the pending entry
  // under the key its reply will carry, and return that key. nullopt on failure.
  // Inline-QoS form: our reply-reader GUID as related_sample_identity (with an
  // UNKNOWN sequence number, per rmw), keyed by the assigned writerSeqNumber.
  // Cyclone form: {client id, next seq} spliced into the payload, keyed by
  // CYCLONEDDS_KEY_BIT | seq.
  std::optional<uint64_t> send(std::span<const uint8_t> request, reply_callback_t on_reply,
                               std::shared_ptr<SyncSlot> sync,
                               std::function<void()> on_abandon = nullptr) {
    // Room in the pending table: drop what has expired first, then refuse at
    // the cap (a server that stopped answering must not take the client down
    // with it, see max_pending_requests). Done BEFORE the engine pin below:
    // an expired entry's on_abandon is user code and may stop() the
    // participant, and stop() waits for every registered engine operation -
    // held across this send, that wait would never end. The table needs only
    // its own mutex.
    bool full = false;
    {
      std::vector<Pending> gone;
      {
        std::lock_guard<std::mutex> lock(mutex);
        prune_expired_locked(gone);
        full = max_pending != 0 && pending.size() >= max_pending;
      }
      finish_abandoned(gone);
    }
    // The engine stays up for the rest of the send (see Pin): the proxy walk,
    // the vendor lookups and the newChange all dereference this run's endpoints.
    const Pin pinned = pin();
    if (!pinned) {
      refuse("request", "the participant is not running (stopped, or stopping)");
      return std::nullopt;
    }
    if (full) {
      refuse("request", "the pending table is full (max_pending_requests); no server answers");
      return std::nullopt;
    }
    Pending entry{std::move(on_reply), std::move(sync)};
    entry.on_abandon = std::move(on_abandon);
    if (request_timeout.count() > 0) {
      entry.deadline = std::chrono::steady_clock::now() + request_timeout;
    }
    // A request must fit a single DATA submessage: inline QoS does not survive
    // fragmentation, and the Cyclone header counts against the same budget (see
    // MAX_UNFRAGMENTED_RPC_PAYLOAD).
    if (request.size() > rtps::MAX_UNFRAGMENTED_RPC_PAYLOAD) {
      refuse("request", "it exceeds the single-DATA budget");
      return std::nullopt;
    }
    // The form is chosen from the servers matched now, outside `mutex`. With
    // none matched the request is refused: the reliable writer would replay it
    // to a server matched later, and a Cyclone server would read the first 16
    // body bytes of an inline-QoS request as its header (see
    // ServiceClient::call_async / wait_for_server).
    const PeerForm peer = peer_form();
    if (peer.matched == 0) {
      refuse("request", "no server is matched (use wait_for_server())");
      return std::nullopt;
    }
    if (peer.cyclonedds && (request.size() < rtps::rpc::CDR_ENCAPSULATION_SIZE ||
                            request.size() + rtps::rpc::CYCLONEDDS_REQUEST_HEADER_SIZE >
                                rtps::MAX_UNFRAGMENTED_RPC_PAYLOAD)) {
      refuse("request", "it is not CDR-encapsulated or leaves no room for the Cyclone header");
      return std::nullopt;
    }
    std::lock_guard<std::mutex> lock(mutex);
    // The lock covers only newChange + insert, so a reply can never look up the
    // key before it is registered (the send itself is async). The cap is
    // checked again here: the check above ran under an earlier hold of this
    // lock, and concurrent senders could all have seen room for one.
    if (max_pending != 0 && pending.size() >= max_pending) {
      refuse("request", "the pending table is full (max_pending_requests); no server answers");
      return std::nullopt;
    }
    if (peer.cyclonedds) {
      if (cyclonedds_next_seq == std::numeric_limits<int64_t>::max()) {
        cyclonedds_next_seq = 0;
      }
      const rtps::rpc::CycloneDdsRequestHeader header{cyclonedds_client_id, ++cyclonedds_next_seq};
      const auto payload = rtps::rpc::prependCycloneDdsRequestHeader(header, request);
      if (payload.empty()) {
        refuse("request", "it is not CDR-encapsulated");
        return std::nullopt;
      }
      const auto *change = request_writer->newChange(rtps::ChangeKind_t::ALIVE, payload.data(),
                                                     static_cast<rtps::DataSize_t>(payload.size()));
      if (change == nullptr) {
        refuse("request", "the request writer's history is full");
        return std::nullopt;
      }
      const uint64_t key = CYCLONEDDS_KEY_BIT | static_cast<uint64_t>(header.seq);
      pending[key] = std::move(entry);
      return key;
    }
    const rtps::rpc::SampleIdentity related{reply_reader_guid, rtps::SequenceNumber_t{-1, 0}};
    const auto *change = request_writer->newChangeWithRelatedSampleIdentity(
        rtps::ChangeKind_t::ALIVE, request.data(), static_cast<rtps::DataSize_t>(request.size()),
        related);
    if (change == nullptr) {
      refuse("request", "the request writer's history is full");
      return std::nullopt;
    }
    const uint64_t key = seq_key(change->sequenceNumber);
    pending[key] = std::move(entry);
    return key;
  }
};

void RtpsParticipant::service_request_trampoline(void *arg, const rtps::ReaderCacheChange &change) {
  auto *ctx = static_cast<ServiceServerContext *>(arg);
  if (ctx == nullptr || !ctx->handler || ctx->reply_writer == nullptr) {
    return;
  }
  if (change.kind != rtps::ChangeKind_t::ALIVE) {
    return; // a dispose/unregister carries no sample (see reader_trampoline)
  }
  // Copy the request payload (valid only during this callback). shared_ptr so
  // the deferred closure stays cheaply copyable inside std::function (and to
  // avoid a GCC 15 -Wfree-nonheap-object false positive on moved vectors).
  auto request = std::make_shared<std::vector<uint8_t>>(change.getDataSize());
  if (!request->empty() && !change.copyInto(request->data(), change.getDataSize())) {
    return;
  }

  // Build a responder correlated to this request, in the client's own form. A
  // sync handler replies immediately; a deferred one may hold the responder.
  auto state = std::make_shared<ServiceResponder::State>();
  state->reply_writer = ctx->reply_writer;
  state->self = ctx->self;
  state->service = ctx->service;
  state->live = ctx->self->live_;
  state->endpoint_live = ctx->writer_live;
  if (!change.hasRelatedSampleIdentity && rtps::rpc::isCycloneDds(change.sourceVendor)) {
    // rmw_cyclonedds_cpp client: the correlation is the {guid, seq} header
    // inside the payload. Keep it for the reply; the handler sees the bare
    // request, exactly as it would from any other client.
    rtps::rpc::CycloneDdsRequestHeader header{};
    if (!rtps::rpc::parseCycloneDdsRequestHeader(*request, header)) {
      return; // too short to carry the header: not a request we can answer
    }
    state->cyclonedds = true;
    state->cyclonedds_header = header;
    *request = rtps::rpc::stripCycloneDdsRequestHeader(*request);
  } else {
    // Inline-QoS client: echo {client reply-reader GUID (from the request's
    // related sample identity), request writerSeqNumber}.
    state->related.writer_guid = change.hasRelatedSampleIdentity
                                     ? change.relatedSampleIdentity.writer_guid
                                     : change.writerGuid;
    state->related.sequence_number = change.sn;
  }
  // Whose match the reply waits for (see reply()): the exact reply reader when
  // the request named one (rmw_fastrtps and this facade's client put the
  // client's reply reader in the related sample identity), else any reader of
  // the requester's participant (the DDS-RPC form names the request writer
  // there; a Cyclone request carries no GUID at all).
  state->requester = change.writerGuid.prefix;
  if (change.hasRelatedSampleIdentity) {
    const auto kind = change.relatedSampleIdentity.writer_guid.entityId.entityKind;
    if (kind == rtps::EntityKind_t::USER_DEFINED_READER_WITHOUT_KEY ||
        kind == rtps::EntityKind_t::USER_DEFINED_READER_WITH_KEY) {
      state->reply_reader = change.relatedSampleIdentity.writer_guid;
      state->reply_reader_known = true;
    }
  }
  state->server = ctx->weak_from_this();
  // Inline for the default path; banded shared-port servers run the handler
  // from the pool at their band instead (see DeferredDispatch). Shared
  // capture: queued work owns the context, so removal/rollback cannot free it
  // underneath (close() stops further deliveries).
  auto owner = ctx->shared_from_this();
  ctx->deferred.run_or_defer(
      [owner, request, responder = ServiceResponder(state)]() {
        owner->handler(std::span<const uint8_t>(request->data(), request->size()), responder);
      },
      owner);
}

void RtpsParticipant::service_reply_trampoline(void *arg, const rtps::ReaderCacheChange &change) {
  auto *impl = static_cast<ServiceClient::Impl *>(arg);
  if (impl == nullptr) {
    return;
  }
  if (change.kind != rtps::ChangeKind_t::ALIVE) {
    return; // a dispose/unregister carries no sample (see reader_trampoline)
  }
  // Only replies addressed to this client, in either form. The inline-QoS
  // check needs no payload; the Cyclone header is read after the copy below.
  uint64_t key = 0;
  bool cyclonedds = false;
  if (change.hasRelatedSampleIdentity) {
    if (!(change.relatedSampleIdentity.writer_guid == impl->reply_reader_guid)) {
      return;
    }
    key = seq_key(change.relatedSampleIdentity.sequence_number);
  } else if (rtps::rpc::isCycloneDds(change.sourceVendor)) {
    cyclonedds = true;
  } else {
    return; // no correlation we understand
  }

  // shared_ptr payload: see service_request_trampoline.
  auto reply = std::make_shared<std::vector<uint8_t>>(change.getDataSize());
  if (!reply->empty() && !change.copyInto(reply->data(), change.getDataSize())) {
    return;
  }
  if (cyclonedds) {
    rtps::rpc::CycloneDdsRequestHeader header{};
    if (!rtps::rpc::parseCycloneDdsRequestHeader(*reply, header) ||
        header.guid != impl->cyclonedds_client_id) {
      return; // malformed, or another client's reply
    }
    key = ServiceClient::Impl::CYCLONEDDS_KEY_BIT | static_cast<uint64_t>(header.seq);
    *reply = rtps::rpc::stripCycloneDdsRequestHeader(*reply);
  }

  ServiceClient::Impl::Pending pending;
  {
    std::lock_guard<std::mutex> lock(impl->mutex);
    auto it = impl->pending.find(key);
    if (it == impl->pending.end()) {
      return; // unknown/duplicate/late reply
    }
    pending = std::move(it->second);
    impl->pending.erase(it);
  }
  // Correlation (map lookup/erase) ran inline above; only the user-facing
  // delivery is deferred for banded shared-port clients (inline by default).
  // The delivery body is self-contained (pending + reply), but the drain job
  // still owns the Impl via the shared owner capture.
  auto owner = impl->shared_from_this();
  impl->deferred.run_or_defer(
      [pending = std::move(pending), reply]() {
        if (pending.sync) {
          std::lock_guard<std::mutex> lock(pending.sync->m);
          pending.sync->reply = std::move(*reply);
          pending.sync->done = true;
          pending.sync->cv.notify_one();
        } else if (pending.on_reply) {
          pending.on_reply(std::span<const uint8_t>(reply->data(), reply->size()));
        }
      },
      owner);
}

RtpsParticipant::ServiceClient::ServiceClient(std::shared_ptr<Impl> impl)
    : impl_(std::move(impl)) {}
RtpsParticipant::ServiceClient::~ServiceClient() = default;

std::size_t RtpsParticipant::ServiceClient::server_count() const {
  return impl_->matched_servers();
}

std::size_t RtpsParticipant::ServiceClient::pending_count() const { return impl_->pending_count(); }

// Polled, like rclpy's wait_for_service: a match is confirmed by an ACKNACK
// that lands on an engine worker with no hook out to the facade, and the poll
// is one walk of a handful of proxies under the writer's lock every 10 ms for
// the few seconds discovery takes, with no vendor lookups.
bool RtpsParticipant::ServiceClient::wait_for_server(std::chrono::milliseconds timeout) const {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (impl_->matched_servers() == 0) {
    // A participant that is stopping (or stopped under us) matches nothing
    // again this run: give up at once rather than wait out the timeout.
    if (!impl_->running() || std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return true;
}

bool RtpsParticipant::ServiceClient::call_async(std::span<const uint8_t> request,
                                                reply_callback_t on_reply) {
  return impl_->send(request, std::move(on_reply), nullptr).has_value();
}

bool RtpsParticipant::ServiceClient::call_async(std::span<const uint8_t> request,
                                                reply_callback_t on_reply,
                                                abandon_callback_t on_abandon) {
  // The pending entry owns on_abandon and runs it exactly once when the entry
  // is pruned or abandoned; a reply erases the entry first (the callbacks are
  // moved out under the client mutex), so the two never both run.
  return impl_->send(request, std::move(on_reply), nullptr, std::move(on_abandon)).has_value();
}

std::optional<std::vector<uint8_t>>
RtpsParticipant::ServiceClient::call(std::span<const uint8_t> request,
                                     std::chrono::milliseconds timeout) {
  auto slot = std::make_shared<Impl::SyncSlot>();
  auto key = impl_->send(request, nullptr, slot);
  if (!key.has_value()) {
    return std::nullopt;
  }
  std::unique_lock<std::mutex> lock(slot->m);
  if (!slot->cv.wait_for(lock, timeout, [&] { return slot->done; })) {
    // Timed out: drop the pending entry so a late reply is ignored cleanly.
    std::lock_guard<std::mutex> plock(impl_->mutex);
    impl_->pending.erase(*key);
    return std::nullopt;
  }
  if (slot->failed) {
    return std::nullopt; // abandoned: the participant stopped, or request_timeout
  }
  return std::move(slot->reply);
}

std::future<std::optional<std::vector<uint8_t>>>
RtpsParticipant::ServiceClient::call_future(std::span<const uint8_t> request) {
  // Promise fulfilled from the async reply callback (shared_ptr so it outlives
  // this call and is owned by the pending entry until the reply arrives), or
  // with nullopt when the request is abandoned (request_timeout, stop()) - a
  // promise destroyed unfulfilled would throw broken_promise at the waiter.
  auto reply = std::make_shared<FutureReply>(); // completed once, see FutureReply
  auto future = reply->promise.get_future();
  const auto key = impl_->send(
      request,
      [reply](std::span<const uint8_t> bytes) {
        reply->complete(std::vector<uint8_t>(bytes.begin(), bytes.end()));
      },
      nullptr, [reply]() { reply->complete(std::nullopt); });
  if (!key.has_value()) {
    reply->complete(std::nullopt);
  }
  return future;
}

bool RtpsParticipant::add_service_server(const ServiceConfig &config, service_handler_t handler) {
  // A synchronous handler is a deferred handler that replies immediately.
  return add_service_server_deferred(
      config, [h = std::move(handler)](std::span<const uint8_t> request, ServiceResponder resp) {
        resp.reply(h(request));
      });
}

bool RtpsParticipant::add_service_server_deferred(const ServiceConfig &config,
                                                  service_deferred_handler_t handler) {
  return add_service_server_deferred_internal(config, std::move(handler)) != nullptr;
}

// Internal variant returning the exact context created, so composite builders
// (actions) can roll back precisely what THIS invocation added.
bool RtpsParticipant::begin_engine_op() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopping_ || domain_ == nullptr || participant_ == nullptr) {
    // Teardown has begun (or never started): the engine will be (or already
    // is) destroyed; the caller must not touch it. During stop() the domain
    // teardown itself reclaims every endpoint, so skipping the individual
    // removal leaks nothing.
    return false;
  }
  ++active_engine_ops_;
  return true;
}

void RtpsParticipant::end_engine_op() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (--active_engine_ops_ == 0) {
    engine_ops_cv_.notify_all();
  }
}

void RtpsParticipant::rollback_delete_writer(rtps::Writer *writer) {
  if (writer == nullptr) {
    return;
  }
  if (domain_ == nullptr || participant_ == nullptr ||
      !domain_->deleteWriter(*participant_, writer)) {
    // Deletion failed (e.g. the SEDP dispose could not be sent): the endpoint
    // stays registered and keeps its dedicated port. Retain it so stop() can
    // retry rather than dropping the only handle and leaking an untracked
    // endpoint.
    logger_.warn("Rollback: writer deletion failed; retaining for cleanup at stop()");
    orphaned_writers_.push_back(writer);
  }
}

void RtpsParticipant::rollback_delete_reader(rtps::Reader *reader) {
  if (reader == nullptr) {
    return;
  }
  if (domain_ == nullptr || participant_ == nullptr ||
      !domain_->deleteReader(*participant_, reader)) {
    logger_.warn("Rollback: reader deletion failed; retaining for cleanup at stop()");
    orphaned_readers_.push_back(reader);
  }
}

std::shared_ptr<RtpsParticipant::ServiceServerContext>
RtpsParticipant::add_service_server_deferred_internal(const ServiceConfig &config,
                                                      service_deferred_handler_t handler) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!started_) {
    logger_.error("Cannot add service server '{}': not started", config.service);
    return nullptr;
  }
  const std::string req_topic = rtps::rpc::service_request_topic(config.service);
  const std::string rep_topic = rtps::rpc::service_reply_topic(config.service);
  const std::string req_type = rtps::rpc::service_request_type(config.type_name);
  const std::string rep_type = rtps::rpc::service_response_type(config.type_name);

  // The service's band/dscp apply to BOTH endpoints (request reader + reply
  // writer) - each banded endpoint may get a dedicated port (rationed).
  const rtps::EndpointOptions endpoint_options{.band = config.band, .dscp = config.dscp};
  rtps::Writer *reply_writer =
      domain_->createWriter(*participant_, rep_topic.c_str(), rep_type.c_str(), /*reliable=*/true,
                            /*enforceUnicast=*/false, endpoint_options);
  rtps::Reader *request_reader =
      domain_->createReader(*participant_, req_topic.c_str(), req_type.c_str(), /*reliable=*/true,
                            /*mcastaddress=*/{0, 0, 0, 0}, endpoint_options);
  if (reply_writer == nullptr || request_reader == nullptr) {
    // Transactional: a partial failure must not leave the successful endpoint
    // announced (and its dedicated-port ration slot consumed). A deletion that
    // itself fails is retained for retry rather than leaked.
    rollback_delete_writer(reply_writer);
    rollback_delete_reader(request_reader);
    logger_.error("Service server '{}': endpoint creation failed", config.service);
    return nullptr;
  }
  auto ctx = std::make_shared<ServiceServerContext>();
  ctx->self = this;
  ctx->service = config.service;
  ctx->handler = std::move(handler);
  ctx->reply_writer = reply_writer;
  ctx->request_reader = request_reader;
  if (config.band != espp::QosBand::Normal && !request_reader->m_attributes.hasDedicatedPort) {
    // Banded request reader on the shared port: run the handler deferred at
    // the service's band instead of inline on the receive worker.
    ctx->deferred.enabled = true;
    ctx->deferred.band = config.band;
    ctx->deferred.transport = &domain_->getTransport();
    logger_.info("Service server '{}' uses deferred banded dispatch (band {}, no dedicated port)",
                 config.service, static_cast<int>(config.band));
  }
  if (request_reader->registerCallback(&service_request_trampoline, ctx.get()) == 0) {
    rollback_delete_reader(request_reader);
    rollback_delete_writer(reply_writer);
    logger_.error("Service server '{}': could not register request callback", config.service);
    return nullptr;
  }
  service_servers_.push_back(ctx);
  logger_.info("Added service server: '{}' ({})", config.service, config.type_name);
  return ctx;
}

// ===========================================================================
// Actions (AMI) - 3 services (send_goal/cancel_goal/get_result) + 2 topics
// (feedback/status), composed over the service + pub/sub facade above.
// ===========================================================================

namespace {
namespace ract = rtps::rpc;

// The execute workers of an action server: a small espp::ThreadPool owned by
// the server (created with it, stopped before the domain is torn down).
// Accepted goals are submitted to it in acceptance order at the action's band;
// its stacks are allocated once, not per goal as the std::thread per goal
// used to (3 KB of internal RAM each on ESP, created and destroyed per goal).
std::unique_ptr<espp::ThreadPool> make_exec_pool(const RtpsParticipant::ActionConfig &config) {
  espp::ThreadPool::Config pool;
  pool.worker_count = config.execute_workers > 0 ? config.execute_workers : 1;
  pool.max_queue_size = 0; // bounded by max_goals_in_flight, applied before on_goal
  pool.worker_task_config = config.execute_task;
  if (pool.worker_task_config.name.empty()) {
    // "exec_<action>" with everything but letters and digits replaced by '_',
    // capped at 15 characters: FreeRTOS keeps configMAX_TASK_NAME_LEN (16 by
    // default) including the terminator, and '/' in a name reads badly in logs.
    std::string name = "exec_" + config.action;
    std::transform(name.begin() + 5, name.end(), name.begin() + 5, [](const char c) {
      return std::isalnum(static_cast<unsigned char>(c)) ? c : '_';
    });
    if (name.back() == '_' && name.size() > 5) {
      name.pop_back();
    }
    pool.worker_task_config.name = name.substr(0, 15);
  }
  pool.log_level = espp::Logger::Verbosity::WARN;
  return std::make_unique<espp::ThreadPool>(pool);
}

// Generate a unique 16-byte goal id: random_device bytes mixed with a process
// counter so uniqueness holds even if random_device is weak (e.g. on an MCU).
ract::GoalUuid generate_goal_id() {
  static std::atomic<uint32_t> counter{0};
  ract::GoalUuid id{};
  std::random_device rd;
  std::generate(id.begin(), id.end(), [&rd]() { return static_cast<uint8_t>(rd() & 0xFF); });
  const uint32_t c = counter.fetch_add(1);
  id[0] = static_cast<uint8_t>(c & 0xFF);
  id[1] = static_cast<uint8_t>((c >> 8) & 0xFF);
  return id;
}
} // namespace

// --- Action server -------------------------------------------------------

// One accepted goal's state + the terminal-status plumbing.
struct RtpsParticipant::ActionGoalHandle::State {
  ract::GoalUuid goal_id{};
  std::vector<uint8_t> goal;      // CDR goal payload
  RtpsParticipant *self{nullptr}; // for feedback/status publish (via publish())
  // weak (not shared) back-reference: the server's goals map owns the State, so
  // a shared_ptr here would form a cycle and leak both. Only needed to retire
  // the goal from that map; publishing uses the copied topics below.
  std::weak_ptr<ActionServerContext> server;
  std::string feedback_topic; // copied so feedback/status publishing needs no
  std::string status_topic;   // server lock and survives server teardown.
  std::mutex mutex;
  ract::GoalStatus status{ract::GoalStatus::ACCEPTED};
  // When the goal was accepted, on the wall clock: the stamp of the
  // SendGoal_Response, and what a CancelGoal request's stamp is compared with.
  std::chrono::system_clock::time_point accepted_at{};
  bool done{false};             // terminate() has run
  bool result_delivered{false}; // get_result answered -> goal may be retired
  std::chrono::steady_clock::time_point terminated_at{}; // for result_timeout
  std::vector<uint8_t> response;                         // the GetResult_Response, set on terminate
  ServiceResponder result_responder;                     // pending get_result (if any)
  std::atomic<bool> cancel_requested{false};
};

struct RtpsParticipant::ActionServerContext {
  RtpsParticipant *self{nullptr};
  std::string feedback_topic;
  std::string status_topic;
  std::vector<uint8_t> unknown_goal_response;  // ActionConfig::unknown_goal_response
  std::size_t max_goals_in_flight{0};          // ActionConfig::max_goals_in_flight
  std::chrono::milliseconds result_timeout{0}; // ActionConfig::result_timeout
  action_execute_callback_t execute{nullptr};
  std::mutex goals_mutex;
  std::map<ract::GoalUuid, std::shared_ptr<ActionGoalHandle::State>> goals;

  // Retire finished goals: past their retention (result_timeout, counted from
  // termination, whether or not the result was fetched), or - with no
  // retention - as soon as their result was delivered. Run whenever a goal
  // request comes in, so the map stays bounded without a timer. Call with
  // goals_mutex held; the per-goal mutex nests inside it (nowhere is the
  // reverse order taken).
  void prune_finished_locked() {
    const auto now = std::chrono::steady_clock::now();
    for (auto it = goals.begin(); it != goals.end();) {
      bool drop = false;
      {
        std::lock_guard<std::mutex> lock(it->second->mutex);
        if (it->second->done) {
          drop = result_timeout.count() == 0 ? it->second->result_delivered
                                             : now - it->second->terminated_at >= result_timeout;
        }
      }
      it = drop ? goals.erase(it) : std::next(it);
    }
  }
  // Goals being executed right now (accepted, not terminated). goals_mutex held.
  std::size_t active_locked() {
    std::size_t n = 0;
    for (auto &kv : goals) {
      std::lock_guard<std::mutex> lock(kv.second->mutex);
      n += kv.second->done ? 0 : 1;
    }
    return n;
  }
  // The execute workers (see make_exec_pool): stopped in stop() before the
  // domain is torn down. Goals run at `band`.
  std::unique_ptr<espp::ThreadPool> exec_pool;
  espp::QosBand band{espp::QosBand::Normal};
};

const RtpsParticipant::GoalId &RtpsParticipant::ActionGoalHandle::goal_id() const {
  return state_->goal_id;
}
std::span<const uint8_t> RtpsParticipant::ActionGoalHandle::goal() const {
  return {state_->goal.data(), state_->goal.size()};
}
bool RtpsParticipant::ActionGoalHandle::is_canceling() const {
  return state_->cancel_requested.load();
}

void RtpsParticipant::ActionGoalHandle::publish_feedback(std::span<const uint8_t> feedback) const {
  auto msg = ract::wrap_feedback(state_->goal_id, feedback);
  state_->self->publish(state_->feedback_topic, {msg.data(), msg.size()});
}

// Publish a single-goal GoalStatusArray (the common case; a full multi-goal list
// is not needed for correlation - clients match on the goal id).
static void publish_goal_status(RtpsParticipant *self, const std::string &status_topic,
                                const ract::GoalUuid &id, ract::GoalStatus status) {
  ract::GoalStatusEntry e;
  e.goal_id = id;
  e.status = status;
  std::array<ract::GoalStatusEntry, 1> arr{e};
  auto msg = ract::make_goal_status_array(arr);
  self->publish(status_topic, {msg.data(), msg.size()});
}

void RtpsParticipant::ActionGoalHandle::terminate(int status_value,
                                                  std::span<const uint8_t> result) const {
  const auto status = static_cast<ract::GoalStatus>(static_cast<int8_t>(status_value));
  const auto response = ract::wrap_get_result_response(status, result);
  terminate_with_response(static_cast<int8_t>(status_value), response);
}

void RtpsParticipant::ActionGoalHandle::terminate_with_response(
    int8_t status_value, std::span<const uint8_t> get_result_response) const {
  const auto status = static_cast<ract::GoalStatus>(status_value);
  ServiceResponder responder;
  bool retire = false;
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->done) {
      return;
    }
    state_->done = true;
    state_->status = status;
    state_->terminated_at = std::chrono::steady_clock::now();
    state_->response.assign(get_result_response.begin(), get_result_response.end());
    if (state_->result_responder.valid()) {
      responder = state_->result_responder; // get_result already waiting
      state_->result_delivered = true;
      retire = true;
    }
  }
  publish_goal_status(state_->self, state_->status_topic, state_->goal_id, status);
  if (responder.valid()) {
    responder.reply({state_->response.data(), state_->response.size()});
  }
  if (retire) {
    // Terminated with the client's get_result already waiting: it now has the
    // result, so drop the goal from the server map (a still-running worker keeps
    // the State alive via its own reference).
    // Without a retention time the goal leaves the server map now (a running
    // worker keeps the State alive via its own reference); with one it stays
    // known - for a cancel (ERROR_GOAL_TERMINATED) or a repeated get_result -
    // until prune_finished_locked() retires it.
    if (auto server = state_->server.lock()) {
      std::lock_guard<std::mutex> lock(server->goals_mutex);
      if (server->result_timeout.count() == 0) {
        server->goals.erase(state_->goal_id);
      }
    }
  }
}
void RtpsParticipant::ActionGoalHandle::succeed(std::span<const uint8_t> result) const {
  terminate(static_cast<int>(ract::GoalStatus::SUCCEEDED), result);
}
void RtpsParticipant::ActionGoalHandle::abort(std::span<const uint8_t> result) const {
  terminate(static_cast<int>(ract::GoalStatus::ABORTED), result);
}
void RtpsParticipant::ActionGoalHandle::canceled(std::span<const uint8_t> result) const {
  terminate(static_cast<int>(ract::GoalStatus::CANCELED), result);
}

bool RtpsParticipant::remove_service_server(const std::shared_ptr<ServiceServerContext> &server) {
  if (server == nullptr) {
    return false;
  }
  // Pin the engine: the deletions + deferred close() below run OUTSIDE mutex_
  // (close() waits for an in-flight handler that may take mutex_), and stop()
  // must not stop/destroy the domain while they are using it.
  if (!begin_engine_op()) {
    return false;
  }
  EngineOpGuard op_guard(*this);
  // Ordering (all confirmed before facade state is mutated; on failure the
  // context stays registered and live for retry, already-deleted endpoints
  // nulled):
  //   1. delete the request READER first, so no NEW request is dispatched;
  //   2. close() the deferred dispatcher, which WAITS for the in-flight
  //      request handler to finish - that handler may still call
  //      ServiceResponder::reply() through the reply writer, so it must be
  //      quiesced BEFORE the writer is deleted;
  //   3. delete the reply WRITER, now that no handler can reference it;
  //   4. drop the registry entry.
  if (server->request_reader != nullptr) {
    if (!domain_->deleteReader(*participant_, server->request_reader)) {
      return false;
    }
    server->request_reader = nullptr;
  }
  server->deferred.close();
  // Invalidate RETAINED responders before the reply writer dies: holding the
  // endpoint-liveness lock waits for an in-flight reply() to finish against
  // the still-live writer, and any later reply() observes !alive and no-ops
  // instead of writing through the deleted (possibly reused) writer slot.
  if (server->writer_live) {
    std::lock_guard<std::mutex> endpoint_lock(server->writer_live->m);
    server->writer_live->alive = false;
  }
  {
    // Replies still waiting for their reader's match go with the writer.
    std::lock_guard<std::mutex> held_lock(server->held_mutex);
    server->held.clear();
  }
  if (server->reply_writer != nullptr) {
    if (!domain_->deleteWriter(*participant_, server->reply_writer)) {
      return false;
    }
    server->reply_writer = nullptr;
  }
  {
    // Remove exactly THIS handle (pointer identity) - a concurrently added
    // server is untouched.
    std::lock_guard<std::mutex> lock(mutex_);
    std::erase(service_servers_, server);
  }
  return true;
}

bool RtpsParticipant::remove_service_client(const std::shared_ptr<ServiceClient> &client) {
  if (client == nullptr) {
    return false;
  }
  // Pin the engine for the unlocked deletion + quiesce (see
  // remove_service_server()).
  if (!begin_engine_op()) {
    return false;
  }
  EngineOpGuard op_guard(*this);
  // Same ordering as remove_service_server(): delete the reply READER first
  // (stops new reply deliveries), then close() the deferred dispatcher
  // (WAITS for the in-flight reply callback, which references this Impl),
  // then delete the request WRITER, then drop the registry entry. Facade
  // state is mutated only on confirmed engine deletion; partial progress is
  // nulled for retry.
  if (client->impl_->reply_reader != nullptr) {
    if (!domain_->deleteReader(*participant_, client->impl_->reply_reader)) {
      return false;
    }
    client->impl_->reply_reader = nullptr;
  }
  client->impl_->deferred.close();
  if (client->impl_->request_writer != nullptr) {
    if (!domain_->deleteWriter(*participant_, client->impl_->request_writer)) {
      return false;
    }
    client->impl_->request_writer = nullptr;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::erase(service_clients_, client);
  }
  return true;
}

bool RtpsParticipant::add_action_server(const ActionConfig &config, action_goal_callback_t on_goal,
                                        action_execute_callback_t execute,
                                        action_cancel_callback_t on_cancel) {
  if (!started_) {
    logger_.error("Cannot add action server '{}': not started", config.action);
    return false;
  }
  // Pin the ENTIRE composite transaction as one engine operation: the nested
  // adds release mutex_ between endpoints, so without the pin a concurrent
  // stop() could tear the engine down mid-build (or race the registry commit
  // below against teardown's container clearing). With the operation
  // registered, stop() waits at its phase 1.5 until this function returns.
  if (!begin_engine_op()) {
    logger_.error("Cannot add action server '{}': shutting down", config.action);
    return false;
  }
  EngineOpGuard op_guard(*this);
  auto ctx = std::make_shared<ActionServerContext>();
  ctx->self = this;
  ctx->feedback_topic = rtps::rpc::action_feedback_topic(config.action);
  ctx->status_topic = rtps::rpc::action_status_topic(config.action);
  ctx->unknown_goal_response = config.unknown_goal_response;
  ctx->max_goals_in_flight = config.max_goals_in_flight;
  ctx->result_timeout = config.result_timeout;
  ctx->execute = std::move(execute);
  ctx->band = config.band;
  ctx->exec_pool = make_exec_pool(config);
  if (!ctx->exec_pool->is_running()) {
    logger_.error("Action server '{}': the execute workers could not be started", config.action);
    return false;
  }

  // Feedback + status publishers (plain reliable topics). The action's
  // band/dscp are inherited by every underlying endpoint (see ActionConfig).
  // Track exactly what THIS invocation created: a failed add_writer() (e.g.
  // the topic already exists because the action was added twice) must NOT
  // cause the rollback to delete another instance's endpoints, and a
  // concurrent add by another thread must never be rolled back as collateral.
  const bool created_feedback =
      add_writer({.topic = ctx->feedback_topic,
                  .type_name = rtps::rpc::action_feedback_type(config.type_name),
                  .reliability = Reliability::RELIABLE,
                  .band = config.band,
                  .dscp = config.dscp});
  const bool created_status =
      created_feedback && add_writer({.topic = ctx->status_topic,
                                      .type_name = rtps::rpc::action_status_type(),
                                      .reliability = Reliability::RELIABLE,
                                      .band = config.band,
                                      .dscp = config.dscp});
  if (!created_feedback || !created_status) {
    if (created_feedback) {
      remove_writer(ctx->feedback_topic);
    }
    logger_.error("Action server '{}': feedback/status writer creation failed", config.action);
    return false;
  }

  // The exact service-server handles created by this invocation (for precise
  // rollback - see the internal add variant).
  std::vector<std::shared_ptr<ServiceServerContext>> created_servers;
  const auto add_sync_tracked = [this, &created_servers](const ServiceConfig &cfg,
                                                         service_handler_t h) -> bool {
    auto server = add_service_server_deferred_internal(
        cfg, [h = std::move(h)](std::span<const uint8_t> request, ServiceResponder resp) {
          resp.reply(h(request));
        });
    if (server != nullptr) {
      created_servers.push_back(std::move(server));
      return true;
    }
    return false;
  };
  const auto add_deferred_tracked = [this, &created_servers](const ServiceConfig &cfg,
                                                             service_deferred_handler_t h) -> bool {
    auto server = add_service_server_deferred_internal(cfg, std::move(h));
    if (server != nullptr) {
      created_servers.push_back(std::move(server));
      return true;
    }
    return false;
  };

  auto weak = std::weak_ptr<ActionServerContext>(ctx);

  // send_goal service: accept/reject, then spawn the execute thread.
  const ServiceConfig send_goal_cfg{rtps::rpc::action_send_goal_service(config.action),
                                    rtps::rpc::action_send_goal_type(config.type_name), config.band,
                                    config.dscp};
  bool ok = add_sync_tracked(
      send_goal_cfg, [this, weak, on_goal](std::span<const uint8_t> req) -> std::vector<uint8_t> {
        auto server = weak.lock();
        ract::GoalUuid id{};
        std::vector<uint8_t> goal;
        if (server == nullptr || !ract::unwrap_send_goal_request(req, id, goal)) {
          return ract::make_send_goal_response(false);
        }
        // Bounded: a goal beyond max_goals_in_flight (executing or waiting for
        // an execute worker) or a goal id the server already has is rejected before the
        // user's on_goal sees it. A duplicate would replace an in-flight goal
        // and orphan its pending get_result; rcl rejects them too.
        {
          std::lock_guard<std::mutex> lock(server->goals_mutex);
          server->prune_finished_locked();
          if (server->goals.count(id) != 0) {
            this->logger_.warn("Action server: goal rejected, its id is already in use");
            return ract::make_send_goal_response(false);
          }
          if (server->max_goals_in_flight != 0 &&
              server->active_locked() >= server->max_goals_in_flight) {
            this->logger_.warn("Action server: goal rejected, {} goals already in flight "
                               "(max_goals_in_flight)",
                               static_cast<int>(server->max_goals_in_flight));
            return ract::make_send_goal_response(false);
          }
        }
        const bool accept = !on_goal || on_goal(id, {goal.data(), goal.size()});
        if (!accept) {
          return ract::make_send_goal_response(false);
        }
        auto gstate = std::make_shared<ActionGoalHandle::State>();
        gstate->goal_id = id;
        gstate->goal = std::move(goal);
        gstate->self = this;
        gstate->server = server; // weak back-reference (see State)
        gstate->feedback_topic = server->feedback_topic;
        gstate->status_topic = server->status_topic;
        gstate->accepted_at = std::chrono::system_clock::now();
        {
          std::lock_guard<std::mutex> lock(server->goals_mutex);
          server->goals[id] = gstate;
        }
        if (server->execute) {
          // Runs on one of the server's execute workers (owned by the server,
          // stopped before the domain is torn down). The job captures a weak
          // server ref (locked only while running), so a goal waiting for a
          // worker forms no goals<->server cycle. The pool's queue is
          // unbounded (max_goals_in_flight bounds it above), so the only
          // rejection is a stopped pool: then the goal is not accepted after
          // all - forgotten again and rejected to the client - rather than
          // held without ever running.
          std::weak_ptr<ActionServerContext> weak_server = server;
          const bool queued =
              server->exec_pool &&
              server->exec_pool->try_submit(
                  [gstate, weak_server]() {
                    auto s = weak_server.lock();
                    if (!s) {
                      return;
                    }
                    // EXECUTING, unless the goal was cancelled while it
                    // waited: it is CANCELING then, and stays so until the
                    // execute callback terminates it. Under the goal's
                    // mutex: the cancel path sets the flag and publishes
                    // CANCELING under it too, so the two never cross.
                    {
                      std::lock_guard<std::mutex> lock(gstate->mutex);
                      if (!gstate->cancel_requested.load()) {
                        publish_goal_status(s->self, s->status_topic, gstate->goal_id,
                                            ract::GoalStatus::EXECUTING);
                      }
                    }
                    s->execute(ActionGoalHandle(gstate));
                  },
                  server->band);
          if (!queued) {
            logger_.warn("Action server: the execute workers are stopped; goal rejected");
            std::lock_guard<std::mutex> lock(server->goals_mutex);
            server->goals.erase(id);
            return ract::make_send_goal_response(false);
          }
        }
        // ACCEPTED now, EXECUTING once an execute worker picks the goal up (as
        // rclcpp_action reports it: the goal may wait for a worker).
        publish_goal_status(this, server->status_topic, id, ract::GoalStatus::ACCEPTED);
        // The acceptance stamp (builtin_interfaces/Time, wall clock), as rcl
        // reports it; a client may hand it back in a CancelGoal request.
        const auto since_epoch = gstate->accepted_at.time_since_epoch();
        const auto sec = std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
        const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - sec);
        return ract::make_send_goal_response(true, static_cast<int32_t>(sec.count()),
                                             static_cast<uint32_t>(nsec.count()));
      });

  // get_result service (DEFERRED): reply now if done, else hold the responder.
  const ServiceConfig get_result_cfg{rtps::rpc::action_get_result_service(config.action),
                                     rtps::rpc::action_get_result_type(config.type_name),
                                     config.band, config.dscp};
  ok = ok && add_deferred_tracked(get_result_cfg, [weak](std::span<const uint8_t> req,
                                                         ServiceResponder responder) {
         auto server = weak.lock();
         ract::GoalUuid id{};
         if (server == nullptr || !ract::parse_get_result_request(req, id)) {
           return;
         }
         std::shared_ptr<ActionGoalHandle::State> gstate;
         {
           std::lock_guard<std::mutex> lock(server->goals_mutex);
           server->prune_finished_locked();
           auto it = server->goals.find(id);
           if (it != server->goals.end()) {
             gstate = it->second;
           }
         }
         if (gstate == nullptr) {
           // Unknown goal: the configured well-formed response, else
           // the bare status (parses only for an empty Result).
           if (!server->unknown_goal_response.empty()) {
             responder.reply(
                 {server->unknown_goal_response.data(), server->unknown_goal_response.size()});
           } else {
             responder.reply(ract::wrap_get_result_response(ract::GoalStatus::UNKNOWN, {}));
           }
           return;
         }
         bool retire = false;
         {
           std::lock_guard<std::mutex> lock(gstate->mutex);
           if (gstate->done) {
             responder.reply({gstate->response.data(), gstate->response.size()});
             gstate->result_delivered = true;
             retire = true;
           } else {
             gstate->result_responder = responder; // fulfilled on terminate()
           }
         }
         if (retire) {
           // Client has its result: without a retention time, drop the goal
           // from the server map now (see ActionGoalHandle::terminate).
           std::lock_guard<std::mutex> glock(server->goals_mutex);
           if (server->result_timeout.count() == 0) {
             server->goals.erase(id);
           }
         }
       });

  // cancel_goal service (action_msgs/srv/CancelGoal), the selection rules of
  // rcl_action_server_process_cancel_request: a zero goal id and a zero stamp
  // address every active goal; a zero id with a stamp every active goal
  // accepted at or before the stamp; an id with a zero stamp that goal; an id
  // with a stamp that goal AND every active goal accepted at or before the
  // stamp. The named goal must be known (else ERROR_UNKNOWN_GOAL_ID, 2) and
  // still active (else ERROR_GOAL_TERMINATED, 3). The goals that can be
  // cancelled are marked canceling (the execute callback observes
  // is_canceling()) and reported CANCELING on the status topic; the reply
  // lists them, with ERROR_REJECTED (1) when on_cancel refused every
  // candidate (rclcpp's rule) and ERROR_NONE (0) otherwise, also for a stamp
  // that selects nothing. rclpy's client checks goals_canceling, rclcpp's
  // the code. Stamps are wall-clock (builtin_interfaces/Time) and compared
  // with the acceptance time the SendGoal_Response reported.
  const ServiceConfig cancel_cfg{rtps::rpc::action_cancel_goal_service(config.action),
                                 rtps::rpc::action_cancel_goal_type(), config.band, config.dscp};
  ok = ok &&
       add_sync_tracked(
           cancel_cfg, [weak, on_cancel](std::span<const uint8_t> req) -> std::vector<uint8_t> {
             auto server = weak.lock();
             ract::GoalUuid id{};
             int32_t sec = 0;
             uint32_t nsec = 0;
             if (server == nullptr || !ract::parse_cancel_goal_request(req, id, sec, nsec)) {
               return ract::make_cancel_goal_response(ract::CANCEL_ERROR_REJECTED, {});
             }
             const bool by_id =
                 !std::all_of(id.begin(), id.end(), [](uint8_t b) { return b == 0; });
             const bool by_stamp = sec != 0 || nsec != 0;
             const std::chrono::system_clock::time_point stamp{
                 std::chrono::duration_cast<std::chrono::system_clock::duration>(
                     std::chrono::seconds{sec} + std::chrono::nanoseconds{nsec})};
             std::vector<std::pair<ract::GoalUuid, std::shared_ptr<ActionGoalHandle::State>>>
                 candidates;
             {
               std::lock_guard<std::mutex> lock(server->goals_mutex);
               server->prune_finished_locked();
               if (by_id) {
                 auto it = server->goals.find(id);
                 if (it == server->goals.end()) {
                   return ract::make_cancel_goal_response(ract::CANCEL_ERROR_UNKNOWN_GOAL_ID, {});
                 }
                 bool done = false;
                 {
                   std::lock_guard<std::mutex> glock(it->second->mutex);
                   done = it->second->done;
                 }
                 if (done) {
                   return ract::make_cancel_goal_response(ract::CANCEL_ERROR_GOAL_TERMINATED, {});
                 }
                 candidates.emplace_back(it->first, it->second);
               }
               if (!by_id || by_stamp) {
                 for (auto &kv : server->goals) {
                   if (by_id && kv.first == id) {
                     continue; // already the first candidate
                   }
                   if (by_stamp && kv.second->accepted_at > stamp) {
                     continue; // accepted after the stamp
                   }
                   candidates.emplace_back(kv.first, kv.second);
                 }
               }
             }
             std::size_t refused = 0;
             std::vector<ract::GoalUuid> canceling;
             for (auto &[goal_id, gstate] : candidates) {
               bool done = false;
               {
                 std::lock_guard<std::mutex> lock(gstate->mutex);
                 done = gstate->done;
               }
               if (done) {
                 continue; // a finished goal selected by the stamp: nothing to cancel
               }
               if (on_cancel && !on_cancel(goal_id)) {
                 ++refused;
                 continue;
               }
               {
                 // The flag flips under the goal's mutex, where the execute
                 // job checks it and publishes EXECUTING in one step: a worker
                 // starting this goal either sees the flag and stays silent, or
                 // has already published EXECUTING, which CANCELING (published
                 // below, outside the lock) then follows. The publish itself
                 // needs no lock.
                 bool first_cancel = false;
                 {
                   std::lock_guard<std::mutex> glock(gstate->mutex);
                   first_cancel = !gstate->cancel_requested.exchange(true);
                 }
                 if (first_cancel) {
                   publish_goal_status(server->self, server->status_topic, goal_id,
                                       ract::GoalStatus::CANCELING);
                 }
               }
               canceling.push_back(goal_id);
             }
             const int8_t code = (canceling.empty() && refused > 0) ? ract::CANCEL_ERROR_REJECTED
                                                                    : ract::CANCEL_ERROR_NONE;
             return ract::make_cancel_goal_response(code, canceling);
           });

  if (!ok) {
    // Transactional: unwind EXACTLY the endpoints this invocation created -
    // the tracked service-server handles and the two topic writers (created
    // above by this call) - so nothing stays announced (or holds a ration
    // slot) for the action that failed to build, and nothing else is touched.
    // The goal service was announced before this failure, so a peer may already
    // have submitted an accepted goal that runs (or waits) on ctx->exec_pool.
    // Teardown order matters:
    //   1. remove the send_goal service FIRST (created_servers[0]) so no NEW
    //      goal is queued after the stop below;
    //   2. signal cooperative cancellation and STOP the workers (stop() waits
    //      for the running goal) - BEFORE the get_result service (and its
    //      reply writer) is deleted, because a deferred get_result handler may
    //      have handed a ServiceResponder to a worker, whose goal-terminating
    //      reply() must go through that writer while it is still alive
    //      (deleting it first would leave the responder replying through a
    //      reset/reused writer slot);
    //   3. only then remove the remaining services and the topic writers.
    // Not under mutex_, so workers can take it.
    if (!created_servers.empty()) {
      remove_service_server(created_servers.front()); // send_goal: stop new spawns
    }
    {
      std::lock_guard<std::mutex> lock(ctx->goals_mutex);
      for (auto &kv : ctx->goals) {
        kv.second->cancel_requested.store(true);
      }
    }
    if (ctx->exec_pool) {
      ctx->exec_pool->stop();
    }
    for (size_t i = 1; i < created_servers.size(); ++i) {
      remove_service_server(created_servers[i]);
    }
    remove_writer(ctx->status_topic);
    remove_writer(ctx->feedback_topic);
    logger_.error("Action server '{}': service endpoint creation failed", config.action);
    return false;
  }
  {
    // Commit under mutex_: the registry vectors are iterated/cleared by
    // stop()'s teardown and mutated by concurrent adds.
    std::lock_guard<std::mutex> lock(mutex_);
    action_servers_.push_back(std::move(ctx));
  }
  logger_.info("Added action server: '{}' ({})", config.action, config.type_name);
  return true;
}

// --- Action client -------------------------------------------------------

struct RtpsParticipant::ActionClient::Impl {
  struct Goal {
    feedback_callback_t on_feedback{nullptr};
    result_callback_t on_result{nullptr};
    result_response_callback_t on_response{nullptr}; // send_goal_raw: the raw response
  };
  RtpsParticipant *self{nullptr};
  std::shared_ptr<ServiceClient> send_goal_client;
  std::shared_ptr<ServiceClient> get_result_client;
  std::shared_ptr<ServiceClient> cancel_client;
  std::string action;
  std::mutex mutex;
  std::map<ract::GoalUuid, Goal> goals;
};

RtpsParticipant::ActionClient::ActionClient(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}
RtpsParticipant::ActionClient::~ActionClient() = default;

std::size_t RtpsParticipant::ActionClient::server_count() const {
  return std::min({impl_->send_goal_client->server_count(),
                   impl_->get_result_client->server_count(), impl_->cancel_client->server_count()});
}

bool RtpsParticipant::ActionClient::wait_for_server(std::chrono::milliseconds timeout) const {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (server_count() == 0) {
    // see ServiceClient::wait_for_server(): a stopping participant matches nothing
    if (!impl_->send_goal_client->impl_->running() ||
        std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return true;
}

std::optional<RtpsParticipant::GoalId> RtpsParticipant::ActionClient::send_goal(
    std::span<const uint8_t> goal, feedback_callback_t on_feedback, result_callback_t on_result) {
  return send_goal_impl(goal, std::move(on_feedback), std::move(on_result), nullptr);
}

std::optional<RtpsParticipant::GoalId>
RtpsParticipant::ActionClient::send_goal_raw(std::span<const uint8_t> goal,
                                             feedback_callback_t on_feedback,
                                             result_response_callback_t on_response) {
  return send_goal_impl(goal, std::move(on_feedback), nullptr, std::move(on_response));
}

std::optional<RtpsParticipant::GoalId> RtpsParticipant::ActionClient::send_goal_impl(
    std::span<const uint8_t> goal, feedback_callback_t on_feedback, result_callback_t on_result,
    result_response_callback_t on_response) {
  const ract::GoalUuid id = generate_goal_id();
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->goals[id] =
        Impl::Goal{std::move(on_feedback), std::move(on_result), std::move(on_response)};
  }
  auto *impl = impl_.get();
  // The goal ends in exactly one of: a rejection, the GetResult_Response, or an
  // abandoned request (see ServiceConfig::request_timeout / stop()). The last
  // two report ABORTED with no result rather than leaving the goal's callbacks
  // in the map forever.
  const auto finish = [impl, id](ract::GoalStatus status, std::span<const uint8_t> result,
                                 std::span<const uint8_t> response) {
    Impl::Goal g;
    {
      std::lock_guard<std::mutex> lock(impl->mutex);
      auto it = impl->goals.find(id);
      if (it == impl->goals.end()) {
        return;
      }
      g = std::move(it->second);
      impl->goals.erase(it);
    }
    if (g.on_response) {
      g.on_response(response);
    } else if (g.on_result) {
      g.on_result(static_cast<int8_t>(status), result);
    }
  };
  const auto abandoned = [finish]() { finish(ract::GoalStatus::ABORTED, {}, {}); };
  const auto queued = impl->send_goal_client->impl_->send(
      ract::wrap_send_goal_request(id, goal),
      [impl, id, finish, abandoned](std::span<const uint8_t> reply) {
        bool accepted = false;
        if (!ract::parse_send_goal_response(reply, accepted) || !accepted) {
          finish(ract::GoalStatus::ABORTED, {}, {});
          return;
        }
        // Accepted: request the result (completes when the goal finishes).
        const auto requested = impl->get_result_client->impl_->send(
            ract::make_get_result_request(id),
            [finish](std::span<const uint8_t> res) {
              ract::GoalStatus status{};
              std::vector<uint8_t> result;
              if (!ract::unwrap_get_result_response(res, status, result)) {
                finish(ract::GoalStatus::ABORTED, {}, {}); // too short to be a response
                return;
              }
              finish(status, {result.data(), result.size()}, res);
            },
            nullptr, abandoned);
        if (!requested.has_value()) {
          abandoned();
        }
      },
      nullptr, abandoned);
  if (!queued.has_value()) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->goals.erase(id);
    impl_->self->logger_.warn("Action client '{}': goal refused (see the service client's reason)",
                              impl_->action);
    return std::nullopt;
  }
  return id;
}

bool RtpsParticipant::ActionClient::cancel_goal(const GoalId &goal_id,
                                                cancel_callback_t on_response) {
  return impl_->cancel_client->call_async(
      ract::make_cancel_goal_request(goal_id),
      [on_response = std::move(on_response)](std::span<const uint8_t> reply) {
        if (!on_response) {
          return;
        }
        int8_t code = ract::CANCEL_ERROR_REJECTED;
        std::vector<ract::GoalUuid> canceling;
        if (!ract::parse_cancel_goal_response(reply, code, canceling)) {
          code = ract::CANCEL_ERROR_REJECTED; // unreadable: treat as refused
          canceling.clear();
        }
        on_response(code, canceling.size());
      });
}

std::shared_ptr<RtpsParticipant::ActionClient>
RtpsParticipant::add_action_client(const ActionConfig &config) {
  if (!started_) {
    logger_.error("Cannot add action client '{}': not started", config.action);
    return nullptr;
  }
  // Pin the ENTIRE composite transaction as one engine operation: the nested
  // adds release mutex_ between endpoints, so without the pin a concurrent
  // stop() could tear the engine down mid-build (or race the registry commit
  // below against teardown's container clearing). With the operation
  // registered, stop() waits at its phase 1.5 until this function returns.
  if (!begin_engine_op()) {
    logger_.error("Cannot add action client '{}': shutting down", config.action);
    return nullptr;
  }
  EngineOpGuard op_guard(*this);
  auto impl = std::make_unique<ActionClient::Impl>();
  impl->self = this;
  impl->action = config.action;
  // The action's band/dscp are inherited by every underlying endpoint. On a
  // later failure exactly the handles created HERE are removed (precise
  // rollback - never a concurrently added endpoint).
  // The get_result request waits for the whole goal: it must never be timed
  // out by the service client (a goal running longer than the request timeout
  // would be reported ABORTED while still executing); the server's result,
  // its result_timeout retirement, or stop() release it. send_goal and
  // cancel_goal are answered at once and keep the configured timeout.
  impl->send_goal_client =
      add_service_client({.service = rtps::rpc::action_send_goal_service(config.action),
                          .type_name = rtps::rpc::action_send_goal_type(config.type_name),
                          .band = config.band,
                          .dscp = config.dscp,
                          .request_timeout = config.request_timeout});
  impl->get_result_client =
      add_service_client({.service = rtps::rpc::action_get_result_service(config.action),
                          .type_name = rtps::rpc::action_get_result_type(config.type_name),
                          .band = config.band,
                          .dscp = config.dscp,
                          .request_timeout = std::chrono::milliseconds{0}});
  impl->cancel_client =
      add_service_client({.service = rtps::rpc::action_cancel_goal_service(config.action),
                          .type_name = rtps::rpc::action_cancel_goal_type(),
                          .band = config.band,
                          .dscp = config.dscp,
                          .request_timeout = config.request_timeout});
  if (!impl->send_goal_client || !impl->get_result_client || !impl->cancel_client) {
    // Transactional: unwind exactly the service clients that DID build.
    remove_service_client(impl->send_goal_client);
    remove_service_client(impl->get_result_client);
    remove_service_client(impl->cancel_client);
    logger_.error("Action client '{}': service client creation failed", config.action);
    return nullptr;
  }

  ActionClient::Impl *raw = impl.get();
  // Feedback subscriber routes by goal id to the goal's on_feedback.
  if (!add_reader({rtps::rpc::action_feedback_topic(config.action),
                   rtps::rpc::action_feedback_type(config.type_name), Reliability::RELIABLE,
                   [raw](std::span<const uint8_t> msg) {
                     ract::GoalUuid id{};
                     std::vector<uint8_t> fb;
                     if (!ract::unwrap_feedback(msg, id, fb)) {
                       return;
                     }
                     ActionClient::feedback_callback_t cb;
                     {
                       std::lock_guard<std::mutex> lock(raw->mutex);
                       auto it = raw->goals.find(id);
                       if (it != raw->goals.end()) {
                         cb = it->second.on_feedback;
                       }
                     }
                     if (cb) {
                       cb({fb.data(), fb.size()});
                     }
                   },
                   config.band, config.dscp})) {
    remove_service_client(impl->send_goal_client);
    remove_service_client(impl->get_result_client);
    remove_service_client(impl->cancel_client);
    logger_.error("Action client '{}': feedback reader creation failed", config.action);
    return nullptr;
  }

  auto client = std::shared_ptr<ActionClient>(new ActionClient(std::move(impl)));
  {
    std::lock_guard<std::mutex> lock(mutex_); // commit vs stop()/concurrent adds
    action_clients_.push_back(client);
  }
  logger_.info("Added action client: '{}' ({})", config.action, config.type_name);
  return client;
}

std::shared_ptr<RtpsParticipant::ServiceClient>
RtpsParticipant::add_service_client(const ServiceConfig &config) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!started_) {
    logger_.error("Cannot add service client '{}': not started", config.service);
    return nullptr;
  }
  const std::string req_topic = rtps::rpc::service_request_topic(config.service);
  const std::string rep_topic = rtps::rpc::service_reply_topic(config.service);
  const std::string req_type = rtps::rpc::service_request_type(config.type_name);
  const std::string rep_type = rtps::rpc::service_response_type(config.type_name);

  // The service's band/dscp apply to BOTH endpoints (reply reader + request
  // writer) - each banded endpoint may get a dedicated port (rationed).
  const rtps::EndpointOptions endpoint_options{.band = config.band, .dscp = config.dscp};
  rtps::Reader *reply_reader =
      domain_->createReader(*participant_, rep_topic.c_str(), rep_type.c_str(), /*reliable=*/true,
                            /*mcastaddress=*/{0, 0, 0, 0}, endpoint_options);
  rtps::Writer *request_writer =
      domain_->createWriter(*participant_, req_topic.c_str(), req_type.c_str(), /*reliable=*/true,
                            /*enforceUnicast=*/false, endpoint_options);
  if (reply_reader == nullptr || request_writer == nullptr) {
    // Transactional: see add_service_server_deferred(). A deletion that itself
    // fails is retained for retry rather than leaked.
    rollback_delete_reader(reply_reader);
    rollback_delete_writer(request_writer);
    logger_.error("Service client '{}': endpoint creation failed", config.service);
    return nullptr;
  }
  auto impl = std::make_shared<ServiceClient::Impl>();
  impl->self = this;
  impl->live = live_;
  impl->request_timeout = config.request_timeout;
  impl->max_pending = config.max_pending_requests;
  impl->request_writer = request_writer;
  impl->reply_reader = reply_reader;
  impl->reply_reader_guid = reply_reader->m_attributes.endpointGuid;
  impl->cyclonedds_client_id =
      rtps::rpc::cycloneDdsClientId(request_writer->m_attributes.endpointGuid);
  if (config.band != espp::QosBand::Normal && !reply_reader->m_attributes.hasDedicatedPort) {
    // Banded reply reader on the shared port: deliver replies deferred at the
    // service's band instead of inline on the receive worker.
    impl->deferred.enabled = true;
    impl->deferred.band = config.band;
    impl->deferred.transport = &domain_->getTransport();
    logger_.info("Service client '{}' uses deferred banded dispatch (band {}, no dedicated port)",
                 config.service, static_cast<int>(config.band));
  }
  if (reply_reader->registerCallback(&service_reply_trampoline, impl.get()) == 0) {
    rollback_delete_reader(reply_reader);
    rollback_delete_writer(request_writer);
    logger_.error("Service client '{}': could not register reply callback", config.service);
    return nullptr;
  }
  auto client = std::shared_ptr<ServiceClient>(new ServiceClient(std::move(impl)));
  service_clients_.push_back(client);
  logger_.info("Added service client: '{}' ({})", config.service, config.type_name);
  return client;
}

// ===========================================================================
// Native services (espp<->espp): lean request/reply over plain pub/sub with a
// 20-byte in-band correlation header. No engine wire support needed.
// ===========================================================================

struct RtpsParticipant::NativeServiceServerContext {
  RtpsParticipant *self{nullptr};
  std::string reply_topic;
  std::string request_topic; ///< retained for composite (native action) rollback
  service_handler_t handler{nullptr};
  // Partial-removal markers (see remove_native_service_server): flags, not
  // cleared strings - in-flight handlers still read the topic strings.
  bool request_removed{false};
  bool reply_removed{false};
};

struct RtpsParticipant::NativeServiceClient::Impl {
  struct SyncSlot {
    std::mutex m;
    std::condition_variable cv;
    bool done{false};
    bool failed{false}; // abandoned (see ServiceClient::Impl)
    std::vector<uint8_t> reply;
  };
  struct Pending {
    reply_callback_t on_reply{nullptr};
    std::shared_ptr<SyncSlot> sync{nullptr};
    std::chrono::steady_clock::time_point deadline{std::chrono::steady_clock::time_point::max()};
    std::function<void()> on_abandon{nullptr};
  };
  RtpsParticipant *self{nullptr};
  std::string request_topic;
  std::string reply_topic; ///< retained for composite (native action) rollback
  // Partial-removal markers (see remove_native_service_client).
  bool reply_removed{false};
  bool request_removed{false};
  std::array<uint8_t, 12> my_prefix{};
  std::atomic<uint32_t> next_id{1};
  std::mutex mutex;
  std::unordered_map<uint32_t, Pending> pending;
  std::chrono::milliseconds request_timeout{0}; ///< ServiceConfig::request_timeout
  std::size_t max_pending{0};                   ///< ServiceConfig::max_pending_requests
  // The run's liveness token and the engine pin, as for the ROS client (see
  // ServiceClient::Impl::pin): a handle kept past stop() or destruction
  // refuses instead of publishing through a dangling participant.
  std::shared_ptr<Liveness> live;
  using Pin = ServiceClient::Impl::Pin;
  Pin pin() const {
    if (self == nullptr || !live) {
      return {};
    }
    std::lock_guard<std::mutex> lock(live->m);
    if (!live->alive || !self->begin_engine_op()) {
      return {};
    }
    return Pin{self};
  }

  // The same bounded table as ServiceClient::Impl: expired entries are dropped
  // on every send / count, every entry at stop(); gone entries are finished
  // outside `mutex`.
  static void finish_abandoned(std::vector<Pending> &gone) {
    for (auto &p : gone) {
      if (p.sync) {
        std::lock_guard<std::mutex> lock(p.sync->m);
        p.sync->failed = true;
        p.sync->done = true;
        p.sync->cv.notify_all();
      }
      if (p.on_abandon) {
        p.on_abandon();
      }
    }
    gone.clear();
  }
  void prune_expired_locked(std::vector<Pending> &gone) {
    const auto now = std::chrono::steady_clock::now();
    for (auto it = pending.begin(); it != pending.end();) {
      if (it->second.deadline <= now) {
        gone.push_back(std::move(it->second));
        it = pending.erase(it);
      } else {
        ++it;
      }
    }
  }
  void abandon_all() {
    std::vector<Pending> gone;
    {
      std::lock_guard<std::mutex> lock(mutex);
      gone.reserve(pending.size());
      std::transform(pending.begin(), pending.end(), std::back_inserter(gone),
                     [](auto &kv) { return std::move(kv.second); });
      pending.clear();
    }
    finish_abandoned(gone);
  }
  std::size_t pending_count() {
    std::vector<Pending> gone;
    std::size_t count = 0;
    {
      std::lock_guard<std::mutex> lock(mutex);
      prune_expired_locked(gone);
      count = pending.size();
    }
    finish_abandoned(gone);
    return count;
  }

  std::optional<uint32_t> send(std::span<const uint8_t> request, reply_callback_t on_reply,
                               std::shared_ptr<SyncSlot> sync,
                               std::function<void()> on_abandon = nullptr) {
    // Expired entries first and OUTSIDE the engine pin, as in
    // ServiceClient::Impl::send(): their on_abandon may stop() the participant.
    bool full = false;
    {
      std::vector<Pending> gone;
      {
        std::lock_guard<std::mutex> lock(mutex);
        prune_expired_locked(gone);
        full = max_pending != 0 && pending.size() >= max_pending;
      }
      finish_abandoned(gone);
    }
    const Pin pinned = pin();
    if (!pinned) {
      return std::nullopt; // not running, or a handle from an earlier run: silent
    }
    if (full) {
      self->logger_.warn("Native service client '{}': request refused, the pending table is "
                         "full (max_pending_requests); no server answers",
                         request_topic);
      return std::nullopt;
    }
    rtps::rpc::NativeHeader h;
    h.client_prefix = my_prefix;
    h.op = rtps::rpc::NativeOp::REQUEST;
    const uint32_t id = next_id.fetch_add(1);
    h.request_id = id;
    auto frame = rtps::rpc::native_encode(h, request);
    {
      std::lock_guard<std::mutex> lock(mutex);
      // Re-checked under the lock of the insert (see ServiceClient::Impl::send).
      if (max_pending != 0 && pending.size() >= max_pending) {
        if (self != nullptr) {
          self->logger_.warn("Native service client '{}': request refused, the pending table is "
                             "full (max_pending_requests); no server answers",
                             request_topic);
        }
        return std::nullopt;
      }
      Pending entry{std::move(on_reply), std::move(sync)};
      entry.on_abandon = std::move(on_abandon);
      if (request_timeout.count() > 0) {
        entry.deadline = std::chrono::steady_clock::now() + request_timeout;
      }
      pending[id] = std::move(entry);
    }
    if (!self->publish(request_topic, {frame.data(), frame.size()})) {
      std::lock_guard<std::mutex> lock(mutex);
      pending.erase(id);
      return std::nullopt;
    }
    return id;
  }
};

RtpsParticipant::NativeServiceClient::NativeServiceClient(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}
RtpsParticipant::NativeServiceClient::~NativeServiceClient() = default;

bool RtpsParticipant::NativeServiceClient::call_async(std::span<const uint8_t> request,
                                                      reply_callback_t on_reply) {
  return impl_->send(request, std::move(on_reply), nullptr).has_value();
}

bool RtpsParticipant::NativeServiceClient::call_async(std::span<const uint8_t> request,
                                                      reply_callback_t on_reply,
                                                      abandon_callback_t on_abandon) {
  return impl_->send(request, std::move(on_reply), nullptr, std::move(on_abandon)).has_value();
}

std::size_t RtpsParticipant::NativeServiceClient::pending_count() const {
  return impl_->pending_count();
}

std::optional<std::vector<uint8_t>>
RtpsParticipant::NativeServiceClient::call(std::span<const uint8_t> request,
                                           std::chrono::milliseconds timeout) {
  auto slot = std::make_shared<Impl::SyncSlot>();
  auto id = impl_->send(request, nullptr, slot);
  if (!id.has_value()) {
    return std::nullopt;
  }
  std::unique_lock<std::mutex> lock(slot->m);
  if (!slot->cv.wait_for(lock, timeout, [&] { return slot->done; })) {
    std::lock_guard<std::mutex> plock(impl_->mutex);
    impl_->pending.erase(*id);
    return std::nullopt;
  }
  if (slot->failed) {
    return std::nullopt; // abandoned: the participant stopped, or request_timeout
  }
  return std::move(slot->reply);
}

std::future<std::optional<std::vector<uint8_t>>>
RtpsParticipant::NativeServiceClient::call_future(std::span<const uint8_t> request) {
  // See ServiceClient::call_future(): an abandoned request resolves nullopt,
  // and the completion is single-shot (FutureReply) - this client inserts its
  // pending entry before publishing, so a stop() in between abandons it and
  // the failed publish then makes send() return nullopt as well.
  auto reply = std::make_shared<FutureReply>();
  auto future = reply->promise.get_future();
  const auto id = impl_->send(
      request,
      [reply](std::span<const uint8_t> bytes) {
        reply->complete(std::vector<uint8_t>(bytes.begin(), bytes.end()));
      },
      nullptr, [reply]() { reply->complete(std::nullopt); });
  if (!id.has_value()) {
    reply->complete(std::nullopt);
  }
  return future;
}

bool RtpsParticipant::remove_native_service_server(
    const std::shared_ptr<NativeServiceServerContext> &server) {
  if (server == nullptr) {
    return false;
  }
  // Same invariant as remove_service_server(): the endpoints (a facade reader
  // whose callback captures this context, and a writer) are deleted FIRST via
  // remove_reader/remove_writer - which themselves only mutate facade state on
  // confirmed engine deletion - and the registry entry is dropped only after
  // both succeed. Removal flags record partial progress for retry.
  if (!server->request_removed) {
    if (!remove_reader(server->request_topic)) {
      return false;
    }
    server->request_removed = true;
  }
  if (!server->reply_removed) {
    if (!remove_writer(server->reply_topic)) {
      return false;
    }
    server->reply_removed = true;
  }
  {
    // Remove exactly THIS handle - a concurrently added server is untouched.
    std::lock_guard<std::mutex> lock(mutex_);
    std::erase(native_service_servers_, server);
  }
  return true;
}

bool RtpsParticipant::remove_native_service_client(
    const std::shared_ptr<NativeServiceClient> &client) {
  if (client == nullptr) {
    return false;
  }
  // Same invariant as remove_native_service_server(): the reply reader's
  // callback captures the Impl, so its engine deletion must be confirmed
  // before this handle is unregistered.
  if (!client->impl_->reply_removed) {
    if (!remove_reader(client->impl_->reply_topic)) {
      return false;
    }
    client->impl_->reply_removed = true;
  }
  if (!client->impl_->request_removed) {
    if (!remove_writer(client->impl_->request_topic)) {
      return false;
    }
    client->impl_->request_removed = true;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::erase(native_service_clients_, client);
  }
  return true;
}

bool RtpsParticipant::add_native_service_server(const ServiceConfig &config,
                                                service_handler_t handler) {
  return add_native_service_server_internal(config, std::move(handler)) != nullptr;
}

// Internal variant returning the exact context created (precise composite
// rollback - see add_service_server_deferred_internal).
std::shared_ptr<RtpsParticipant::NativeServiceServerContext>
RtpsParticipant::add_native_service_server_internal(const ServiceConfig &config,
                                                    service_handler_t handler) {
  if (!started_) {
    logger_.error("Cannot add native service server '{}': not started", config.service);
    return nullptr;
  }
  // Pin the ENTIRE composite transaction as one engine operation (same idiom
  // as the composite action builders): the nested add_writer/add_reader
  // release mutex_ between endpoints, so without the pin a concurrent stop()
  // could tear the engine down between the adds - and the registry commit
  // below could then append a stale server context after teardown cleared the
  // containers. With the operation registered, stop() waits at its phase 1.5
  // until this function returns; once stopping, the add is rejected here.
  if (!begin_engine_op()) {
    logger_.error("Cannot add native service server '{}': shutting down", config.service);
    return nullptr;
  }
  EngineOpGuard op_guard(*this);
  auto ctx = std::make_shared<NativeServiceServerContext>();
  ctx->self = this;
  ctx->reply_topic = rtps::rpc::native_reply_topic(config.service);
  ctx->request_topic = rtps::rpc::native_request_topic(config.service);
  ctx->handler = std::move(handler);
  const std::string &req_topic = ctx->request_topic;

  // The service's band/dscp apply to both native endpoints (request reader +
  // reply writer); the request reader inherits deferred banded dispatch from
  // add_reader() when it gets no dedicated port.
  if (!add_writer({.topic = ctx->reply_topic,
                   .type_name = config.type_name,
                   .reliability = Reliability::RELIABLE,
                   .band = config.band,
                   .dscp = config.dscp})) {
    logger_.error("Native service server '{}': reply writer failed", config.service);
    return nullptr;
  }
  NativeServiceServerContext *raw = ctx.get();
  if (!add_reader({req_topic, config.type_name, Reliability::RELIABLE,
                   [raw](std::span<const uint8_t> frame) {
                     rtps::rpc::NativeHeader h;
                     std::span<const uint8_t> payload;
                     if (!rtps::rpc::native_decode(frame, h, payload) ||
                         h.op != rtps::rpc::NativeOp::REQUEST) {
                       return;
                     }
                     std::vector<uint8_t> reply =
                         raw->handler ? raw->handler(payload) : std::vector<uint8_t>{};
                     // Echo {client_prefix, request_id} back as a REPLY.
                     rtps::rpc::NativeHeader rh;
                     rh.client_prefix = h.client_prefix;
                     rh.request_id = h.request_id;
                     rh.op = rtps::rpc::NativeOp::REPLY;
                     auto out = rtps::rpc::native_encode(rh, reply);
                     raw->self->publish(raw->reply_topic, {out.data(), out.size()});
                   },
                   config.band, config.dscp})) {
    // Transactional: don't leave the reply writer announced (nor its ration
    // slot consumed) when the pair could not be completed.
    remove_writer(ctx->reply_topic);
    logger_.error("Native service server '{}': request reader failed", config.service);
    return nullptr;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    native_service_servers_.push_back(ctx);
  }
  logger_.info("Added native service server: '{}'", config.service);
  return ctx;
}

std::shared_ptr<RtpsParticipant::NativeServiceClient>
RtpsParticipant::add_native_service_client(const ServiceConfig &config) {
  if (!started_) {
    logger_.error("Cannot add native service client '{}': not started", config.service);
    return nullptr;
  }
  // Pin the ENTIRE composite transaction as one engine operation (same idiom
  // as the composite action builders): the nested add_writer/add_reader
  // release mutex_ between endpoints, so without the pin a concurrent stop()
  // could tear the engine down between the adds and the registry commit below
  // could append a stale client into a cleared container. The pin also makes
  // the participant_ dereference below safe: begin_engine_op() validates the
  // pointer under mutex_, and stop() cannot reset it while the operation is
  // registered (it waits at phase 1.5).
  if (!begin_engine_op()) {
    logger_.error("Cannot add native service client '{}': shutting down", config.service);
    return nullptr;
  }
  EngineOpGuard op_guard(*this);
  auto impl = std::make_unique<NativeServiceClient::Impl>();
  impl->self = this;
  impl->live = live_;
  impl->request_timeout = config.request_timeout;
  impl->max_pending = config.max_pending_requests;
  impl->request_topic = rtps::rpc::native_request_topic(config.service);
  impl->reply_topic = rtps::rpc::native_reply_topic(config.service);
  impl->my_prefix = participant_->m_guidPrefix.id;
  const std::string &rep_topic = impl->reply_topic;

  if (!add_writer({.topic = impl->request_topic,
                   .type_name = config.type_name,
                   .reliability = Reliability::RELIABLE,
                   .band = config.band,
                   .dscp = config.dscp})) {
    logger_.error("Native service client '{}': request writer failed", config.service);
    return nullptr;
  }
  NativeServiceClient::Impl *raw = impl.get();
  if (!add_reader({rep_topic, config.type_name, Reliability::RELIABLE,
                   [raw](std::span<const uint8_t> frame) {
                     rtps::rpc::NativeHeader h;
                     std::span<const uint8_t> payload;
                     if (!rtps::rpc::native_decode(frame, h, payload) ||
                         h.op != rtps::rpc::NativeOp::REPLY || h.client_prefix != raw->my_prefix) {
                       return;
                     }
                     NativeServiceClient::Impl::Pending p;
                     {
                       std::lock_guard<std::mutex> lock(raw->mutex);
                       auto it = raw->pending.find(h.request_id);
                       if (it == raw->pending.end()) {
                         return;
                       }
                       p = std::move(it->second);
                       raw->pending.erase(it);
                     }
                     if (p.sync) {
                       std::lock_guard<std::mutex> lock(p.sync->m);
                       p.sync->reply.assign(payload.begin(), payload.end());
                       p.sync->done = true;
                       p.sync->cv.notify_one();
                     } else if (p.on_reply) {
                       p.on_reply(payload);
                     }
                   },
                   config.band, config.dscp})) {
    // Transactional: see add_native_service_server().
    remove_writer(impl->request_topic);
    logger_.error("Native service client '{}': reply reader failed", config.service);
    return nullptr;
  }
  auto client = std::shared_ptr<NativeServiceClient>(new NativeServiceClient(std::move(impl)));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    native_service_clients_.push_back(client);
  }
  logger_.info("Added native service client: '{}'", config.service);
  return client;
}

// ===========================================================================
// Native actions (espp<->espp): lean AMI - a native send_goal service + a
// feedback topic carrying the terminal result.
// ===========================================================================

struct RtpsParticipant::NativeGoalHandle::State {
  uint32_t goal_handle{0};
  std::vector<uint8_t> goal;
  RtpsParticipant *self{nullptr};
  std::string feedback_topic;
  std::atomic<bool> done{false};
  std::atomic<bool> cancel_requested{false};
};

struct RtpsParticipant::NativeActionServerContext {
  RtpsParticipant *self{nullptr};
  std::string feedback_topic;
  native_execute_callback_t execute{nullptr};
  std::atomic<uint32_t> next_handle{1};
  std::size_t max_goals_in_flight{0}; ///< running + waiting goals held at once; 0 = unlimited
  // The execute workers (see make_exec_pool): stopped in stop() before the
  // domain is torn down. Goals run at `band`.
  std::unique_ptr<espp::ThreadPool> exec_pool;
  espp::QosBand band{espp::QosBand::Normal};
  // Running goals keyed by handle, for routing cancel requests. weak so a
  // finished goal's State can expire; the executing worker owns the strong ref.
  std::mutex goals_mutex;
  std::map<uint32_t, std::weak_ptr<NativeGoalHandle::State>> goals;
};

uint32_t RtpsParticipant::NativeGoalHandle::goal_handle() const { return state_->goal_handle; }
std::span<const uint8_t> RtpsParticipant::NativeGoalHandle::goal() const {
  return {state_->goal.data(), state_->goal.size()};
}
bool RtpsParticipant::NativeGoalHandle::is_canceling() const {
  return state_->cancel_requested.load();
}
void RtpsParticipant::NativeGoalHandle::publish_feedback(std::span<const uint8_t> feedback) const {
  auto msg = rtps::rpc::native_make_feedback(state_->goal_handle,
                                             rtps::rpc::NativeGoalStatus::EXECUTING, feedback);
  state_->self->publish(state_->feedback_topic, {msg.data(), msg.size()});
}
void RtpsParticipant::NativeGoalHandle::terminate(uint8_t status,
                                                  std::span<const uint8_t> result) const {
  bool expected = false;
  if (!state_->done.compare_exchange_strong(expected, true)) {
    return;
  }
  auto msg = rtps::rpc::native_make_feedback(
      state_->goal_handle, static_cast<rtps::rpc::NativeGoalStatus>(status), result);
  state_->self->publish(state_->feedback_topic, {msg.data(), msg.size()});
}
void RtpsParticipant::NativeGoalHandle::succeed(std::span<const uint8_t> result) const {
  terminate(static_cast<uint8_t>(rtps::rpc::NativeGoalStatus::SUCCEEDED), result);
}
void RtpsParticipant::NativeGoalHandle::abort(std::span<const uint8_t> result) const {
  terminate(static_cast<uint8_t>(rtps::rpc::NativeGoalStatus::ABORTED), result);
}
void RtpsParticipant::NativeGoalHandle::canceled(std::span<const uint8_t> result) const {
  terminate(static_cast<uint8_t>(rtps::rpc::NativeGoalStatus::CANCELED), result);
}

bool RtpsParticipant::add_native_action_server(const ActionConfig &config,
                                               native_goal_callback_t on_goal,
                                               native_execute_callback_t execute,
                                               native_cancel_callback_t on_cancel) {
  if (!started_) {
    logger_.error("Cannot add native action server '{}': not started", config.action);
    return false;
  }
  // Pin the ENTIRE composite transaction as one engine operation: the nested
  // adds release mutex_ between endpoints, so without the pin a concurrent
  // stop() could tear the engine down mid-build (or race the registry commit
  // below against teardown's container clearing). With the operation
  // registered, stop() waits at its phase 1.5 until this function returns.
  if (!begin_engine_op()) {
    logger_.error("Cannot add native action server '{}': shutting down", config.action);
    return false;
  }
  EngineOpGuard op_guard(*this);
  auto ctx = std::make_shared<NativeActionServerContext>();
  ctx->self = this;
  ctx->feedback_topic = rtps::rpc::native_feedback_topic(config.action);
  ctx->execute = std::move(execute);
  ctx->band = config.band;
  ctx->max_goals_in_flight = config.max_goals_in_flight;
  ctx->exec_pool = make_exec_pool(config);
  if (!ctx->exec_pool->is_running()) {
    logger_.error("Native action server '{}': the execute workers could not be started",
                  config.action);
    return false;
  }

  // The action's band/dscp are inherited by all ~3 native endpoints.
  if (!add_writer({.topic = ctx->feedback_topic,
                   .type_name = config.type_name,
                   .reliability = Reliability::RELIABLE,
                   .band = config.band,
                   .dscp = config.dscp})) {
    logger_.error("Native action server '{}': feedback writer failed", config.action);
    return false;
  }
  auto weak = std::weak_ptr<NativeActionServerContext>(ctx);
  // The send_goal native service: accept -> spawn execute -> reply goal_handle.
  // Handles created by THIS invocation, for precise rollback.
  const auto goal_server = add_native_service_server_internal(
      {rtps::rpc::native_goal_service(config.action), config.type_name, config.band, config.dscp},
      [this, weak, on_goal](std::span<const uint8_t> goal) -> std::vector<uint8_t> {
        auto server = weak.lock();
        if (server == nullptr) {
          return rtps::rpc::native_make_goal_reply(false, 0);
        }
        // Bounded like the ROS 2 server: a goal beyond max_goals_in_flight
        // (running or waiting for an execute worker - `goals` holds a goal
        // until its execute job finishes) is rejected before on_goal.
        if (server->max_goals_in_flight > 0) {
          std::lock_guard<std::mutex> lock(server->goals_mutex);
          if (server->goals.size() >= server->max_goals_in_flight) {
            logger_.warn("Native action server: {} goals in flight (max_goals_in_flight); "
                         "goal rejected",
                         server->goals.size());
            return rtps::rpc::native_make_goal_reply(false, 0);
          }
        }
        if (on_goal && !on_goal(goal)) {
          return rtps::rpc::native_make_goal_reply(false, 0);
        }
        const uint32_t handle = server->next_handle.fetch_add(1);
        auto gstate = std::make_shared<NativeGoalHandle::State>();
        gstate->goal_handle = handle;
        gstate->goal.assign(goal.begin(), goal.end());
        gstate->self = this;
        gstate->feedback_topic = server->feedback_topic;
        if (server->execute) {
          {
            std::lock_guard<std::mutex> lock(server->goals_mutex);
            server->goals[handle] = gstate; // weak; for cancel routing
          }
          // Runs on one of the server's execute workers (owned by the server,
          // stopped before teardown); the job holds the goal state while it
          // waits for a worker.
          std::weak_ptr<NativeActionServerContext> weak_server = server;
          const bool queued =
              server->exec_pool && server->exec_pool->try_submit(
                                       [gstate, weak_server, handle]() {
                                         auto s = weak_server.lock();
                                         if (!s) {
                                           return;
                                         }
                                         s->execute(NativeGoalHandle(gstate));
                                         // Goal finished: drop it from the
                                         // cancel-routing map.
                                         std::lock_guard<std::mutex> lock(s->goals_mutex);
                                         s->goals.erase(handle);
                                       },
                                       server->band);
          if (!queued) {
            // A stopped pool (the only rejection: the queue is unbounded):
            // the goal is not accepted after all.
            logger_.warn("Native action server: the execute workers are stopped; goal rejected");
            std::lock_guard<std::mutex> lock(server->goals_mutex);
            server->goals.erase(handle);
            return rtps::rpc::native_make_goal_reply(false, 0); // handle 0: rejected, like above
          }
        }
        return rtps::rpc::native_make_goal_reply(true, handle);
      });
  if (goal_server == nullptr) {
    remove_writer(ctx->feedback_topic);
    logger_.error("Native action server '{}': goal service failed", config.action);
    return false;
  }
  // The cancel native service: mark a running goal canceling (the execute
  // callback observes is_canceling()); on_cancel, if set, gates acceptance.
  const auto cancel_server = add_native_service_server_internal(
      {rtps::rpc::native_cancel_service(config.action), config.type_name, config.band, config.dscp},
      [weak, on_cancel](std::span<const uint8_t> req) -> std::vector<uint8_t> {
        auto server = weak.lock();
        uint32_t handle = 0;
        if (server == nullptr || !rtps::rpc::native_parse_cancel_request(req, handle)) {
          return rtps::rpc::native_make_cancel_reply(false);
        }
        std::shared_ptr<NativeGoalHandle::State> gstate;
        {
          std::lock_guard<std::mutex> lock(server->goals_mutex);
          auto it = server->goals.find(handle);
          if (it != server->goals.end()) {
            gstate = it->second.lock();
          }
        }
        if (!gstate) {
          return rtps::rpc::native_make_cancel_reply(false); // unknown/finished goal
        }
        const bool accept = !on_cancel || on_cancel(handle);
        if (accept) {
          gstate->cancel_requested.store(true);
        }
        return rtps::rpc::native_make_cancel_reply(accept);
      });
  if (cancel_server == nullptr) {
    // Transactional: unwind EXACTLY what this invocation created - the goal
    // service handle and the feedback writer. The goal service was announced
    // before this failure, so a peer may already have submitted an accepted
    // goal to ctx->exec_pool: remove the endpoints, then stop the workers
    // (waiting for a running goal) before ctx is destroyed. Matches the
    // native-server teardown in stop(), which likewise stops without a cancel
    // signal (native goals are weak refs).
    remove_native_service_server(goal_server);
    remove_writer(ctx->feedback_topic);
    if (ctx->exec_pool) {
      ctx->exec_pool->stop();
    }
    logger_.error("Native action server '{}': cancel service failed", config.action);
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_); // commit vs stop()/concurrent adds
    native_action_servers_.push_back(std::move(ctx));
  }
  logger_.info("Added native action server: '{}'", config.action);
  return true;
}

struct RtpsParticipant::NativeActionClient::Impl {
  struct Goal {
    feedback_callback_t on_feedback{nullptr};
    result_callback_t on_result{nullptr};
  };
  struct BufferedMsg {
    rtps::rpc::NativeGoalStatus status{};
    std::vector<uint8_t> payload;
  };
  RtpsParticipant *self{nullptr};
  std::shared_ptr<NativeServiceClient> goal_client;
  std::shared_ptr<NativeServiceClient> cancel_client;
  std::mutex mutex;
  std::map<uint32_t, Goal> goals;
  // Feedback/result can arrive before send_goal's reply installs the goal (the
  // server starts executing immediately, so a fast native action may publish
  // before the reply lands). Buffer such early messages by handle and replay
  // them on registration. Bounded so a stray/unknown handle cannot grow it
  // without limit.
  std::map<uint32_t, std::vector<BufferedMsg>> pending_early;
  size_t pending_early_count{0};
  static constexpr size_t kMaxPendingEarly = 64;

  // Route a parsed feedback/result message to the registered goal's callbacks;
  // terminal status delivers the result and retires the goal. No-op for an
  // unknown handle. Callbacks run outside the lock.
  static void deliver(Impl *impl, uint32_t handle, rtps::rpc::NativeGoalStatus status,
                      std::span<const uint8_t> payload) {
    const bool terminal = status == rtps::rpc::NativeGoalStatus::SUCCEEDED ||
                          status == rtps::rpc::NativeGoalStatus::ABORTED ||
                          status == rtps::rpc::NativeGoalStatus::CANCELED;
    Goal g;
    {
      std::lock_guard<std::mutex> lock(impl->mutex);
      auto it = impl->goals.find(handle);
      if (it == impl->goals.end()) {
        return;
      }
      g = it->second;
      if (terminal) {
        impl->goals.erase(it);
      }
    }
    if (terminal) {
      if (g.on_result) {
        g.on_result(static_cast<uint8_t>(status), {payload.data(), payload.size()});
      }
    } else if (g.on_feedback) {
      g.on_feedback({payload.data(), payload.size()});
    }
  }
};

RtpsParticipant::NativeActionClient::NativeActionClient(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}
RtpsParticipant::NativeActionClient::~NativeActionClient() = default;

bool RtpsParticipant::NativeActionClient::send_goal(std::span<const uint8_t> goal,
                                                    feedback_callback_t on_feedback,
                                                    result_callback_t on_result,
                                                    accepted_callback_t on_accepted) {
  auto *impl = impl_.get();
  return impl->goal_client->call_async(
      goal, [impl, on_feedback, on_result, on_accepted](std::span<const uint8_t> reply) {
        bool accepted = false;
        uint32_t handle = 0;
        if (!rtps::rpc::native_parse_goal_reply(reply, accepted, handle) || !accepted) {
          if (on_result) {
            on_result(static_cast<uint8_t>(rtps::rpc::NativeGoalStatus::ABORTED), {});
          }
          return;
        }
        std::vector<Impl::BufferedMsg> replay;
        {
          std::lock_guard<std::mutex> lock(impl->mutex);
          impl->goals[handle] = Impl::Goal{on_feedback, on_result};
          // Drain any feedback/result that arrived before this registration.
          auto it = impl->pending_early.find(handle);
          if (it != impl->pending_early.end()) {
            replay = std::move(it->second);
            impl->pending_early_count -= replay.size();
            impl->pending_early.erase(it);
          }
        }
        if (on_accepted) {
          on_accepted(handle); // hand the caller the goal_handle for cancel_goal()
        }
        for (auto &m : replay) {
          Impl::deliver(impl, handle, m.status, m.payload);
        }
      });
}

bool RtpsParticipant::NativeActionClient::cancel_goal(uint32_t goal_handle) {
  if (!impl_->cancel_client) {
    return false;
  }
  return impl_->cancel_client->call_async(rtps::rpc::native_make_cancel_request(goal_handle),
                                          [](std::span<const uint8_t>) {});
}

std::shared_ptr<RtpsParticipant::NativeActionClient>
RtpsParticipant::add_native_action_client(const ActionConfig &config) {
  if (!started_) {
    logger_.error("Cannot add native action client '{}': not started", config.action);
    return nullptr;
  }
  // Pin the ENTIRE composite transaction as one engine operation: the nested
  // adds release mutex_ between endpoints, so without the pin a concurrent
  // stop() could tear the engine down mid-build (or race the registry commit
  // below against teardown's container clearing). With the operation
  // registered, stop() waits at its phase 1.5 until this function returns.
  if (!begin_engine_op()) {
    logger_.error("Cannot add native action client '{}': shutting down", config.action);
    return nullptr;
  }
  EngineOpGuard op_guard(*this);
  auto impl = std::make_unique<NativeActionClient::Impl>();
  impl->self = this;
  // The action's band/dscp are inherited by all native client endpoints. On a
  // later failure exactly the handles created HERE are removed.
  impl->goal_client = add_native_service_client(
      {rtps::rpc::native_goal_service(config.action), config.type_name, config.band, config.dscp});
  impl->cancel_client = add_native_service_client({rtps::rpc::native_cancel_service(config.action),
                                                   config.type_name, config.band, config.dscp});
  if (!impl->goal_client || !impl->cancel_client) {
    // Transactional: unwind exactly the native service client that DID build.
    remove_native_service_client(impl->goal_client);
    remove_native_service_client(impl->cancel_client);
    logger_.error("Native action client '{}': goal/cancel client failed", config.action);
    return nullptr;
  }
  NativeActionClient::Impl *raw = impl.get();
  // Feedback subscriber: route feedback/result by goal_handle; terminal status
  // (>= SUCCEEDED) delivers the result and retires the goal.
  if (!add_reader(
          {rtps::rpc::native_feedback_topic(config.action), config.type_name, Reliability::RELIABLE,
           [raw](std::span<const uint8_t> msg) {
             uint32_t handle = 0;
             rtps::rpc::NativeGoalStatus status{};
             std::vector<uint8_t> payload;
             if (!rtps::rpc::native_parse_feedback(msg, handle, status, payload)) {
               return;
             }
             {
               std::lock_guard<std::mutex> lock(raw->mutex);
               if (raw->goals.find(handle) == raw->goals.end()) {
                 // Goal not registered yet (its send_goal reply is still in
                 // flight): buffer this early message, bounded, for replay when
                 // send_goal installs the goal. See Impl::pending_early.
                 if (raw->pending_early_count < NativeActionClient::Impl::kMaxPendingEarly) {
                   raw->pending_early[handle].push_back({status, std::move(payload)});
                   ++raw->pending_early_count;
                 }
                 return;
               }
             }
             NativeActionClient::Impl::deliver(raw, handle, status, payload);
           },
           config.band, config.dscp})) {
    remove_native_service_client(impl->goal_client);
    remove_native_service_client(impl->cancel_client);
    logger_.error("Native action client '{}': feedback reader failed", config.action);
    return nullptr;
  }
  auto client = std::shared_ptr<NativeActionClient>(new NativeActionClient(std::move(impl)));
  {
    std::lock_guard<std::mutex> lock(mutex_); // commit vs stop()/concurrent adds
    native_action_clients_.push_back(client);
  }
  logger_.info("Added native action client: '{}'", config.action);
  return client;
}
#endif // RTPS_WITH_RPC

// stop() and ~RtpsParticipant are defined here (end of file) so every RPC
// context type the member containers point to is complete when their
// unique_ptr/shared_ptr elements are destroyed - see the note by the constructor.
void RtpsParticipant::stop() {
  // Phase 1: flip started_ under mutex_ so no further publish()/add_*()/reply
  // proceeds past its started_ check, and stopping_ so no NEW engine operation
  // (an unlocked removal/quiesce sequence) can begin.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!started_) {
      return;
    }
    started_ = false;
    stopping_ = true;
  }
  // Phase 1.5: wait for in-flight engine operations to finish. Removals
  // deliberately dereference domain_/participant_ OUTSIDE mutex_ (their
  // deferred close() waits for user callbacks that may take mutex_), so the
  // engine must stay alive until they complete. cv.wait releases mutex_ while
  // waiting, so those callbacks can still acquire it and the operations can
  // finish; begin_engine_op() rejects new operations now that stopping_ is set.
  {
    std::unique_lock<std::mutex> lock(mutex_);
    engine_ops_cv_.wait(lock, [this] { return active_engine_ops_ == 0; });
  }
  // Phase 2: invalidate deferred RPC replies. A service responder held by user
  // code checks live_->alive under this lock before writing through its engine
  // reply writer; flipping it here (before the domain and its writers are
  // destroyed) turns any racing reply into a safe no-op, and holding the lock
  // first waits for an in-flight reply to finish.
  if (live_) {
    std::lock_guard<std::mutex> lock(live_->m);
    live_->alive = false;
  }
#ifdef RTPS_WITH_RPC
  // Held service replies die with the participant: stop their sweep timer
  // (outside held_replies_mutex_, which its callback takes) and forget them.
  {
    std::unique_ptr<espp::Timer> timer;
    {
      std::lock_guard<std::mutex> lock(held_replies_mutex_);
      timer = std::move(held_replies_timer_);
      servers_with_held_replies_.clear();
    }
    timer.reset();
  }
  // Phase 2.5: give up on every pending request. A blocked call() wakes with
  // nullopt, a call_future() resolves to nullopt (not broken_promise), and no
  // reply callback runs after this. The registries are stable (no add past
  // phase 1); mutex_ only orders the snapshot against a concurrent remove, and
  // is NOT held across abandon_all(): that takes the client's own mutex, which
  // a sending thread holds while it logs a refusal through the liveness lock,
  // under which pin() takes mutex_ - the three in a cycle were a potential
  // deadlock (TSan: lock-order inversion). The snapshot's shared_ptrs keep the
  // clients alive for the walk.
  std::vector<std::shared_ptr<ServiceClient>> service_clients;
  std::vector<std::shared_ptr<NativeServiceClient>> native_service_clients;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    service_clients = service_clients_;
    native_service_clients = native_service_clients_;
  }
  for (const auto &cli : service_clients) {
    if (cli && cli->impl_) {
      cli->impl_->abandon_all();
    }
  }
  for (const auto &cli : native_service_clients) {
    if (cli && cli->impl_) {
      cli->impl_->abandon_all();
    }
  }
#endif // RTPS_WITH_RPC
  // Phase 3: retry any endpoint deletions that a creation-time rollback could
  // not complete (the SEDP dispose failed then, so the endpoint stayed
  // registered with its dedicated port). Do it while the domain is still live so
  // a successful retry releases the port and disposes cleanly; whatever still
  // fails is torn down by domain_->stop() below regardless.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (domain_ != nullptr && participant_ != nullptr) {
      for (auto *writer : orphaned_writers_) {
        domain_->deleteWriter(*participant_, writer);
      }
      for (auto *reader : orphaned_readers_) {
        domain_->deleteReader(*participant_, reader);
      }
    }
    orphaned_writers_.clear();
    orphaned_readers_.clear();
  }
  // Phase 4: stop the engine (no more reader/service callbacks fire), then join
  // every owned action-execute worker so none touches this participant after
  // the domain and its writers are gone. Done WITHOUT mutex_ held: a worker's
  // final publish()/reply must be able to take mutex_/live_ and run to
  // completion (as a no-op) so the thread can exit and be joined. The RPC
  // container vectors are stable here - after phase 1 no add_*() can mutate them.
  if (domain_) {
    domain_->stop();
  }
#ifdef RTPS_WITH_RPC
  for (auto &ctx : action_servers_) {
    if (!ctx) {
      continue;
    }
    // Ask cooperative execute callbacks (those that poll is_canceling()) to wind
    // down, then join. A callback that ignores the signal blocks stop() until it
    // returns - execute callbacks must be finite / cancel-aware.
    {
      std::lock_guard<std::mutex> lock(ctx->goals_mutex);
      for (auto &kv : ctx->goals) {
        kv.second->cancel_requested.store(true);
      }
    }
    if (ctx->exec_pool) {
      ctx->exec_pool->stop();
    }
  }
  for (const auto &ctx : native_action_servers_) {
    if (ctx && ctx->exec_pool) {
      ctx->exec_pool->stop();
    }
  }
#endif // RTPS_WITH_RPC
  // Phase 5: tear the domain down and drop bookkeeping under mutex_. The engine
  // owns the endpoint objects, so release our references before the domain (and
  // with it every writer/reader and their callback registrations) goes away.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Quiesce every deferred dispatcher BEFORE releasing the context
    // references: close() cancels each retry timer synchronously, so no timer
    // callback can hold (and later drop, on its own thread) the last context
    // reference - context destruction always happens here.
    for (const auto &ctx : reader_contexts_) {
      ctx->deferred.close();
    }
#ifdef RTPS_WITH_RPC
    for (const auto &srv : service_servers_) {
      if (srv) {
        srv->deferred.close();
      }
    }
    for (const auto &cli : service_clients_) {
      if (cli && cli->impl_) {
        cli->impl_->deferred.close();
      }
    }
#endif // RTPS_WITH_RPC
    writers_.clear();
    participant_ = nullptr;
    domain_.reset();
    reader_contexts_.clear();
#ifdef RTPS_WITH_RPC
    // RPC endpoints are owned by the (now-reset) domain; drop our bookkeeping.
    // Execute workers were joined above, so nothing here races teardown.
    action_clients_.clear();
    action_servers_.clear();
    service_clients_.clear();
    service_servers_.clear();
    native_action_clients_.clear();
    native_action_servers_.clear();
    native_service_clients_.clear();
    native_service_servers_.clear();
#endif                 // RTPS_WITH_RPC
    stopping_ = false; // teardown complete; a future start() may proceed
  }
  logger_.info("Stopped");
}

RtpsParticipant::~RtpsParticipant() { stop(); }

} // namespace espp

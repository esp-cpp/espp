#include "rtps_participant.hpp"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdio>
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
  const rtps::DomainConfig domain_config{
      .metatraffic_band = config_.metatraffic_band,
      .user_traffic_band = config_.user_traffic_band,
      .enable_dedicated_endpoint_ports = config_.enable_dedicated_endpoint_ports,
      .max_prioritized_endpoint_ports = config_.max_prioritized_endpoint_ports,
      .local_prefix_length = prefix_length,
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
      needs_arm = false;
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
  // queued work can never outlive the context.
  if (transport->submit([this, owner]() { drain(owner); }, band)) {
    return;
  }
  // Pool full/stopped: a queued (possibly lone/last) delivery must never be
  // stranded waiting for traffic that may not come - flag the failed arm and
  // let the retry timer recover it.
  std::lock_guard<std::mutex> lock(mutex);
  in_flight = false;
  if (closed) {
    return;
  }
  needs_arm = true;
  ensure_retry_timer_locked(owner);
}

void RtpsParticipant::DeferredDispatch::ensure_retry_timer_locked(
    const std::shared_ptr<void> &owner) {
  if (retry_timer) {
    return; // already running; it re-arms from needs_arm on its next tick
  }
  // The timer runs until close() cancels it synchronously - it NEVER
  // self-cancels. A self-cancelling espp::Timer (callback returning true)
  // stops its underlying task but leaves Timer::running_ set, so a later
  // start() would CAS-fail ("already running") and no-op against a dead task,
  // stranding the parked work. Keeping it alive (callback always returns
  // false) sidesteps that entirely; the per-tick cost is a mutex + empty
  // check, paid only after an actual pool rejection and only until close().
  //
  // Weak owner capture: the context owns this dispatcher (and thus the timer),
  // so a strong capture would be a cycle; the callback promotes it per tick.
  std::weak_ptr<void> weak_owner = owner;
  retry_timer = std::make_unique<espp::Timer>(espp::Timer::Config{
      .name = "rtps_defer_arm",
      .period = std::chrono::milliseconds(20),
      .delay = std::chrono::milliseconds(20),
      .callback = [this, weak_owner]() -> bool {
        auto strong = weak_owner.lock();
        if (!strong) {
          return false; // owner gone (close() cancels first); nothing to do
        }
        {
          std::lock_guard<std::mutex> lock(mutex);
          if (closed || !needs_arm || in_flight) {
            return false; // nothing to recover this tick; keep the timer alive
          }
          if (queue.empty()) {
            needs_arm = false;
            return false;
          }
          in_flight = true;
          needs_arm = false;
        }
        // Reached only when a drain is owed and now in_flight: try to arm it.
        if (!transport->submit([this, strong]() { drain(strong); }, band)) {
          std::lock_guard<std::mutex> lock(mutex);
          in_flight = false;
          if (!closed) {
            needs_arm = true; // still saturated; the next tick retries
          }
        }
        return false; // never self-cancel (see above)
      },
      .auto_start = true,
      .log_level = espp::Logger::Verbosity::WARN,
  });
}

void RtpsParticipant::DeferredDispatch::close() {
  {
    std::lock_guard<std::mutex> lock(mutex);
    closed = true;
    needs_arm = false;
    queue.clear();
  }
  // Cancel the retry timer synchronously and OUTSIDE the lock: cancel() joins
  // the timer task, whose callback takes `mutex` - holding it here would
  // deadlock. After cancel() returns no timer callback is running or will run.
  if (retry_timer) {
    retry_timer->cancel();
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
    // flags needs_arm and the retry timer picks it up.
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

  // A reply in wire form, with whose match it waits for. Replies are held here
  // until the requester's reply reader is matched to reply_writer (or the
  // participant's service_reply_match_timeout passes); see
  // ServiceResponder::reply() for why.
  struct HeldReply {
    std::vector<uint8_t> payload; // the Cyclone header, when one applies, already spliced in
    bool cyclonedds{false};
    rtps::rpc::SampleIdentity related{}; // inline-QoS form only
    rtps::Guid_t reply_reader{};         // the exact reader to wait for, when known
    bool reply_reader_known{false};
    rtps::GuidPrefix_t requester{}; // else: any reader of the requester's participant
    std::chrono::steady_clock::time_point deadline{};
  };
  static constexpr std::size_t MAX_HELD_REPLIES = 8;
  std::mutex held_mutex;
  std::vector<HeldReply> held;

  static bool reader_matched(rtps::Writer &writer, const HeldReply &reply) {
    return reply.reply_reader_known ? writer.hasProxy(reply.reply_reader)
                                    : writer.hasProxyOfParticipant(reply.requester);
  }

  // The engine write of a reply, in the requester's form. The caller holds the
  // liveness locks that keep `writer` valid.
  static void write(rtps::Writer &writer, const HeldReply &reply) {
    if (reply.cyclonedds) {
      writer.newChange(rtps::ChangeKind_t::ALIVE, reply.payload.data(),
                       static_cast<rtps::DataSize_t>(reply.payload.size()));
      return;
    }
    writer.newChangeWithRelatedSampleIdentity(rtps::ChangeKind_t::ALIVE, reply.payload.data(),
                                              static_cast<rtps::DataSize_t>(reply.payload.size()),
                                              reply.related);
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
  if (wire_size > rtps::MAX_UNFRAGMENTED_RPC_PAYLOAD) {
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
  // writer. Until then it is HELD: the client sent its request as soon as its
  // own side had matched, and this writer may learn of the client's reply
  // reader (its SEDP record) a little after the request was served. A change
  // written before that match is never sent to the reader - the writer sends
  // new changes to the readers it has - and the first HEARTBEAT the reader
  // then sees announces it as history, which a VOLATILE reader skips: the call
  // times out although it was served. rmw_fastrtps / rmw_cyclonedds servers
  // wait for the same match before writing a response. The hold ends with the
  // match or after Config::service_reply_match_timeout (see sweep_held_replies).
  const auto timeout = state_->self->config_.service_reply_match_timeout;
  auto server = state_->server.lock();
  if (server == nullptr || timeout <= std::chrono::milliseconds{0} ||
      ServiceServerContext::reader_matched(*state_->reply_writer, wire)) {
    ServiceServerContext::write(*state_->reply_writer, wire);
    return;
  }
  wire.deadline = std::chrono::steady_clock::now() + timeout;
  bool held = false;
  {
    std::lock_guard<std::mutex> held_lock(server->held_mutex);
    if (server->held.size() < ServiceServerContext::MAX_HELD_REPLIES) {
      server->held.push_back(std::move(wire));
      held = true;
    }
  }
  if (!held) {
    // Never lose a reply to the cap: it goes out now, as before the hold existed.
    state_->self->logger_.warn("Service '{}': {} replies already waiting for their reply "
                               "reader; writing this one unmatched",
                               state_->service, ServiceServerContext::MAX_HELD_REPLIES);
    ServiceServerContext::write(*state_->reply_writer, wire);
    return;
  }
  state_->self->note_held_reply(server);
}

void RtpsParticipant::note_held_reply(const std::shared_ptr<ServiceServerContext> &server) {
  std::unique_ptr<espp::Timer> finished; // a self-cancelled timer, destroyed outside the lock
  {
    std::lock_guard<std::mutex> lock(held_replies_mutex_);
    const bool listed =
        std::any_of(servers_with_held_replies_.begin(), servers_with_held_replies_.end(),
                    [&](const std::weak_ptr<ServiceServerContext> &w) {
                      return !w.owner_before(server) && !server.owner_before(w);
                    });
    if (!listed) {
      servers_with_held_replies_.push_back(server);
    }
    if (held_replies_timer_ && !held_replies_timer_done_) {
      return; // running: it sweeps until nothing is held
    }
    // The match itself flushes a held reply (subscriber_matched_trampoline);
    // this timer only enforces the deadline and backs the match up, so it runs
    // at a tenth of the default timeout and cancels itself once nothing is
    // held. A cancelled espp::Timer cannot be restarted (see
    // DeferredDispatch::ensure_retry_timer_locked), so the next hold replaces
    // it - the old object is destroyed after the lock is released, as its task
    // may still be returning from the callback that cancelled it.
    finished = std::move(held_replies_timer_);
    held_replies_timer_done_ = false;
    held_replies_timer_ = std::make_unique<espp::Timer>(espp::Timer::Config{
        .name = "rtps_reply_hold",
        .period = std::chrono::milliseconds(100),
        .delay = std::chrono::milliseconds(100),
        .callback = [this]() -> bool {
          if (sweep_held_replies()) {
            return false; // something is still held
          }
          std::lock_guard<std::mutex> lock(held_replies_mutex_);
          if (!servers_with_held_replies_.empty()) {
            return false; // a reply was held while the sweep ran
          }
          held_replies_timer_done_ = true; // the next hold replaces this timer
          return true;
        },
        .auto_start = true,
        .log_level = espp::Logger::Verbosity::WARN,
    });
  }
  finished.reset();
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
        ServiceServerContext::write(*server->reply_writer, reply);
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
    std::vector<uint8_t> reply;
  };
  struct Pending {
    reply_callback_t on_reply{nullptr};      // set for call_async
    std::shared_ptr<SyncSlot> sync{nullptr}; // set for call
  };

  RtpsParticipant *self{nullptr};
  rtps::Writer *request_writer{nullptr};
  rtps::Reader *reply_reader{nullptr}; ///< retained for composite (action) rollback
  rtps::Guid_t reply_reader_guid{};
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
                               std::shared_ptr<SyncSlot> sync) {
    // A request must fit a single DATA submessage: inline QoS does not survive
    // fragmentation, and the Cyclone header counts against the same budget (see
    // MAX_UNFRAGMENTED_RPC_PAYLOAD).
    if (request.size() > rtps::MAX_UNFRAGMENTED_RPC_PAYLOAD) {
      return std::nullopt;
    }
    // The form is chosen from the servers matched now, outside `mutex`. With
    // none matched the request is refused: the reliable writer would replay it
    // to a server matched later, and a Cyclone server would read the first 16
    // body bytes of an inline-QoS request as its header (see
    // ServiceClient::call_async / wait_for_server).
    const PeerForm peer = peer_form();
    if (peer.matched == 0) {
      return std::nullopt;
    }
    if (peer.cyclonedds && (request.size() < rtps::rpc::CDR_ENCAPSULATION_SIZE ||
                            request.size() + rtps::rpc::CYCLONEDDS_REQUEST_HEADER_SIZE >
                                rtps::MAX_UNFRAGMENTED_RPC_PAYLOAD)) {
      return std::nullopt;
    }
    std::lock_guard<std::mutex> lock(mutex);
    // The lock covers only newChange + insert, so a reply can never look up the
    // key before it is registered (the send itself is async).
    if (peer.cyclonedds) {
      if (cyclonedds_next_seq == std::numeric_limits<int64_t>::max()) {
        cyclonedds_next_seq = 0;
      }
      const rtps::rpc::CycloneDdsRequestHeader header{cyclonedds_client_id, ++cyclonedds_next_seq};
      const auto payload = rtps::rpc::prependCycloneDdsRequestHeader(header, request);
      if (payload.empty()) {
        return std::nullopt; // not a CDR-encapsulated request
      }
      const auto *change = request_writer->newChange(rtps::ChangeKind_t::ALIVE, payload.data(),
                                                     static_cast<rtps::DataSize_t>(payload.size()));
      if (change == nullptr) {
        return std::nullopt;
      }
      const uint64_t key = CYCLONEDDS_KEY_BIT | static_cast<uint64_t>(header.seq);
      pending[key] = Pending{std::move(on_reply), std::move(sync)};
      return key;
    }
    const rtps::rpc::SampleIdentity related{reply_reader_guid, rtps::SequenceNumber_t{-1, 0}};
    const auto *change = request_writer->newChangeWithRelatedSampleIdentity(
        rtps::ChangeKind_t::ALIVE, request.data(), static_cast<rtps::DataSize_t>(request.size()),
        related);
    if (change == nullptr) {
      return std::nullopt;
    }
    const uint64_t key = seq_key(change->sequenceNumber);
    pending[key] = Pending{std::move(on_reply), std::move(sync)};
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

// Polled, like rclpy's wait_for_service: a match is confirmed by an ACKNACK
// that lands on an engine worker with no hook out to the facade, and the poll
// is one walk of a handful of proxies under the writer's lock every 10 ms for
// the few seconds discovery takes, with no vendor lookups.
bool RtpsParticipant::ServiceClient::wait_for_server(std::chrono::milliseconds timeout) const {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (impl_->matched_servers() == 0) {
    if (std::chrono::steady_clock::now() >= deadline) {
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
  return std::move(slot->reply);
}

std::future<std::optional<std::vector<uint8_t>>>
RtpsParticipant::ServiceClient::call_future(std::span<const uint8_t> request) {
  // Promise fulfilled from the async reply callback (shared_ptr so it outlives
  // this call and is owned by the pending entry until the reply arrives).
  auto promise = std::make_shared<std::promise<std::optional<std::vector<uint8_t>>>>();
  auto future = promise->get_future();
  const bool queued = call_async(request, [promise](std::span<const uint8_t> reply) {
    promise->set_value(std::vector<uint8_t>(reply.begin(), reply.end()));
  });
  if (!queued) {
    promise->set_value(std::nullopt);
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

// An owned action-execute worker: the std::thread plus a flag it sets true just
// before returning. Threads are stored (not detached) so shutdown can join them
// before the participant is torn down; finished ones are reaped when new goals
// arrive so the list stays bounded over a long-lived server. See
// reap_and_store / the join loop in stop().
struct ActionExecThread {
  std::thread thread;
  std::shared_ptr<std::atomic<bool>> finished;
};

// Append a freshly-spawned worker after reaping any that have finished (joining
// them releases their captured goal state). Bounds the vector without blocking.
void reap_and_store(std::mutex &m, std::vector<ActionExecThread> &threads, std::thread th,
                    std::shared_ptr<std::atomic<bool>> finished) {
  std::lock_guard<std::mutex> lock(m);
  for (auto it = threads.begin(); it != threads.end();) {
    if (it->finished->load()) {
      if (it->thread.joinable()) {
        it->thread.join();
      }
      it = threads.erase(it);
    } else {
      ++it;
    }
  }
  threads.push_back(ActionExecThread{std::move(th), std::move(finished)});
}

// Join every execute worker and clear the list. A joinable std::thread's
// destructor calls std::terminate, so any worker an accepted goal spawned MUST
// be joined before its owning context is destroyed - during shutdown AND when a
// partially-created action server is rolled back (its goal service was already
// announced, so a peer could have submitted a goal). Callers should first
// remove the endpoints / signal cooperative cancellation so this cannot block
// on a long-running execute callback.
void join_exec_threads(std::mutex &m, std::vector<ActionExecThread> &threads) {
  std::lock_guard<std::mutex> lock(m);
  for (auto &t : threads) {
    if (t.thread.joinable()) {
      t.thread.join();
    }
  }
  threads.clear();
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
  bool done{false};                  // terminate() has run
  bool result_delivered{false};      // get_result answered -> goal may be retired
  std::vector<uint8_t> result;       // set on terminate
  ServiceResponder result_responder; // pending get_result (if any)
  std::atomic<bool> cancel_requested{false};
};

struct RtpsParticipant::ActionServerContext {
  RtpsParticipant *self{nullptr};
  std::string feedback_topic;
  std::string status_topic;
  action_execute_callback_t execute{nullptr};
  std::mutex goals_mutex;
  std::map<ract::GoalUuid, std::shared_ptr<ActionGoalHandle::State>> goals;
  // Owned execute workers (not detached): joined in stop() before the domain is
  // torn down, reaped as they finish. See ActionExecThread / reap_and_store.
  std::mutex threads_mutex;
  std::vector<ActionExecThread> exec_threads;
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
  ServiceResponder responder;
  bool retire = false;
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->done) {
      return;
    }
    state_->done = true;
    state_->status = status;
    state_->result.assign(result.begin(), result.end());
    if (state_->result_responder.valid()) {
      responder = state_->result_responder; // get_result already waiting
      state_->result_delivered = true;
      retire = true;
    }
  }
  publish_goal_status(state_->self, state_->status_topic, state_->goal_id, status);
  if (responder.valid()) {
    responder.reply(
        ract::wrap_get_result_response(status, {state_->result.data(), state_->result.size()}));
  }
  if (retire) {
    // Terminated with the client's get_result already waiting: it now has the
    // result, so drop the goal from the server map (a still-running worker keeps
    // the State alive via its own reference).
    if (auto server = state_->server.lock()) {
      std::lock_guard<std::mutex> lock(server->goals_mutex);
      server->goals.erase(state_->goal_id);
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
  ctx->execute = std::move(execute);

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
        {
          std::lock_guard<std::mutex> lock(server->goals_mutex);
          server->goals[id] = gstate;
        }
        publish_goal_status(this, server->status_topic, id, ract::GoalStatus::EXECUTING);
        if (server->execute) {
          // Own the worker (not detached) so stop() joins it before the domain
          // is torn down. It captures a weak server ref (locked only while
          // running), so a finished worker forms no goals<->server cycle.
          auto finished = std::make_shared<std::atomic<bool>>(false);
          std::weak_ptr<ActionServerContext> weak_server = server;
          std::thread worker([gstate, weak_server, finished]() {
            if (auto s = weak_server.lock()) {
              s->execute(ActionGoalHandle(gstate));
            }
            finished->store(true);
          });
          reap_and_store(server->threads_mutex, server->exec_threads, std::move(worker), finished);
        }
        return ract::make_send_goal_response(true);
      });

  // get_result service (DEFERRED): reply now if done, else hold the responder.
  const ServiceConfig get_result_cfg{rtps::rpc::action_get_result_service(config.action),
                                     rtps::rpc::action_get_result_type(config.type_name),
                                     config.band, config.dscp};
  ok = ok && add_deferred_tracked(
                 get_result_cfg, [weak](std::span<const uint8_t> req, ServiceResponder responder) {
                   auto server = weak.lock();
                   ract::GoalUuid id{};
                   if (server == nullptr || !ract::parse_get_result_request(req, id)) {
                     return;
                   }
                   std::shared_ptr<ActionGoalHandle::State> gstate;
                   {
                     std::lock_guard<std::mutex> lock(server->goals_mutex);
                     auto it = server->goals.find(id);
                     if (it != server->goals.end()) {
                       gstate = it->second;
                     }
                   }
                   if (gstate == nullptr) {
                     responder.reply(ract::wrap_get_result_response(ract::GoalStatus::UNKNOWN, {}));
                     return;
                   }
                   bool retire = false;
                   {
                     std::lock_guard<std::mutex> lock(gstate->mutex);
                     if (gstate->done) {
                       responder.reply(ract::wrap_get_result_response(
                           gstate->status, {gstate->result.data(), gstate->result.size()}));
                       gstate->result_delivered = true;
                       retire = true;
                     } else {
                       gstate->result_responder = responder; // fulfilled on terminate()
                     }
                   }
                   if (retire) {
                     // Client has its result: drop the goal from the server map.
                     std::lock_guard<std::mutex> glock(server->goals_mutex);
                     server->goals.erase(id);
                   }
                 });

  // cancel_goal service: mark the goal canceling; the execute callback observes
  // is_canceling(). Minimal CancelGoal_Response (return_code=0, empty list).
  const ServiceConfig cancel_cfg{rtps::rpc::action_cancel_goal_service(config.action),
                                 rtps::rpc::action_cancel_goal_type(), config.band, config.dscp};
  ok = ok &&
       add_sync_tracked(cancel_cfg,
                        [weak, on_cancel](std::span<const uint8_t> req) -> std::vector<uint8_t> {
                          auto server = weak.lock();
                          // CancelGoal_Request: goal_info{ goal_id: UUID(16), stamp }.
                          if (server != nullptr && req.size() >= 4 + 16) {
                            ract::GoalUuid id{};
                            std::memcpy(id.data(), req.data() + 4, 16);
                            std::shared_ptr<ActionGoalHandle::State> gstate;
                            {
                              std::lock_guard<std::mutex> lock(server->goals_mutex);
                              auto it = server->goals.find(id);
                              if (it != server->goals.end()) {
                                gstate = it->second;
                              }
                            }
                            if (gstate && (!on_cancel || on_cancel(id))) {
                              gstate->cancel_requested.store(true);
                            }
                          }
                          // CancelGoal_Response: return_code:int8 + pad(3) + goals[]=0.
                          std::vector<uint8_t> resp{0x00, 0x01, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0};
                          return resp;
                        });

  if (!ok) {
    // Transactional: unwind EXACTLY the endpoints this invocation created -
    // the tracked service-server handles and the two topic writers (created
    // above by this call) - so nothing stays announced (or holds a ration
    // slot) for the action that failed to build, and nothing else is touched.
    // The goal service was announced before this failure, so a peer may already
    // have submitted an accepted goal and spawned a joinable execute worker in
    // ctx->exec_threads. Teardown order matters:
    //   1. remove the send_goal service FIRST (created_servers[0]) so no NEW
    //      goal can spawn another worker after the join below;
    //   2. signal cooperative cancellation and JOIN the workers - BEFORE the
    //      get_result service (and its reply writer) is deleted, because a
    //      deferred get_result handler may have handed a ServiceResponder to a
    //      worker, whose goal-terminating reply() must go through that writer
    //      while it is still alive (deleting it first would leave the
    //      responder replying through a reset/reused writer slot);
    //   3. only then remove the remaining services and the topic writers.
    // A joinable std::thread destructor would std::terminate, so the join must
    // also precede ctx destruction. Not under mutex_, so workers can take it.
    if (!created_servers.empty()) {
      remove_service_server(created_servers.front()); // send_goal: stop new spawns
    }
    {
      std::lock_guard<std::mutex> lock(ctx->goals_mutex);
      for (auto &kv : ctx->goals) {
        kv.second->cancel_requested.store(true);
      }
    }
    join_exec_threads(ctx->threads_mutex, ctx->exec_threads);
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
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return true;
}

std::optional<RtpsParticipant::GoalId> RtpsParticipant::ActionClient::send_goal(
    std::span<const uint8_t> goal, feedback_callback_t on_feedback, result_callback_t on_result) {
  const ract::GoalUuid id = generate_goal_id();
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->goals[id] = Impl::Goal{std::move(on_feedback), std::move(on_result)};
  }
  auto *impl = impl_.get();
  const bool queued = impl->send_goal_client->call_async(
      ract::wrap_send_goal_request(id, goal), [impl, id](std::span<const uint8_t> reply) {
        bool accepted = false;
        if (!ract::parse_send_goal_response(reply, accepted) || !accepted) {
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
          if (g.on_result) {
            g.on_result(static_cast<int8_t>(ract::GoalStatus::ABORTED), {});
          }
          return;
        }
        // Accepted: request the result (completes when the goal finishes).
        impl->get_result_client->call_async(
            ract::make_get_result_request(id), [impl, id](std::span<const uint8_t> res) {
              ract::GoalStatus status{};
              std::vector<uint8_t> result;
              ract::unwrap_get_result_response(res, status, result);
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
              if (g.on_result) {
                g.on_result(static_cast<int8_t>(status), {result.data(), result.size()});
              }
            });
      });
  if (!queued) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->goals.erase(id);
    return std::nullopt;
  }
  return id;
}

bool RtpsParticipant::ActionClient::cancel_goal(const GoalId &goal_id) {
  // CancelGoal_Request: goal_info{ goal_id: UUID(16), stamp{sec,nsec} }.
  std::vector<uint8_t> req{0x00, 0x01, 0x00, 0x00};
  req.insert(req.end(), goal_id.begin(), goal_id.end());
  for (int i = 0; i < 8; ++i) {
    req.push_back(0); // stamp
  }
  return impl_->cancel_client->call_async(req, [](std::span<const uint8_t>) {});
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
  impl->send_goal_client = add_service_client({rtps::rpc::action_send_goal_service(config.action),
                                               rtps::rpc::action_send_goal_type(config.type_name),
                                               config.band, config.dscp});
  impl->get_result_client = add_service_client({rtps::rpc::action_get_result_service(config.action),
                                                rtps::rpc::action_get_result_type(config.type_name),
                                                config.band, config.dscp});
  impl->cancel_client =
      add_service_client({rtps::rpc::action_cancel_goal_service(config.action),
                          rtps::rpc::action_cancel_goal_type(), config.band, config.dscp});
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
    std::vector<uint8_t> reply;
  };
  struct Pending {
    reply_callback_t on_reply{nullptr};
    std::shared_ptr<SyncSlot> sync{nullptr};
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

  std::optional<uint32_t> send(std::span<const uint8_t> request, reply_callback_t on_reply,
                               std::shared_ptr<SyncSlot> sync) {
    rtps::rpc::NativeHeader h;
    h.client_prefix = my_prefix;
    h.op = rtps::rpc::NativeOp::REQUEST;
    const uint32_t id = next_id.fetch_add(1);
    h.request_id = id;
    auto frame = rtps::rpc::native_encode(h, request);
    {
      std::lock_guard<std::mutex> lock(mutex);
      pending[id] = Pending{std::move(on_reply), std::move(sync)};
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
  return std::move(slot->reply);
}

std::future<std::optional<std::vector<uint8_t>>>
RtpsParticipant::NativeServiceClient::call_future(std::span<const uint8_t> request) {
  auto promise = std::make_shared<std::promise<std::optional<std::vector<uint8_t>>>>();
  auto future = promise->get_future();
  const bool queued = call_async(request, [promise](std::span<const uint8_t> reply) {
    promise->set_value(std::vector<uint8_t>(reply.begin(), reply.end()));
  });
  if (!queued) {
    promise->set_value(std::nullopt);
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
  // Owned execute workers (not detached): joined in stop() before the domain is
  // torn down, reaped as they finish. See ActionExecThread / reap_and_store.
  std::mutex threads_mutex;
  std::vector<ActionExecThread> exec_threads;
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
        if (server == nullptr || (on_goal && !on_goal(goal))) {
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
          // Own the worker (not detached) so stop() joins it before teardown.
          auto finished = std::make_shared<std::atomic<bool>>(false);
          std::weak_ptr<NativeActionServerContext> weak_server = server;
          std::thread worker([gstate, weak_server, finished, handle]() {
            if (auto s = weak_server.lock()) {
              s->execute(NativeGoalHandle(gstate));
              // Goal finished: drop it from the cancel-routing map.
              std::lock_guard<std::mutex> lock(s->goals_mutex);
              s->goals.erase(handle);
            }
            finished->store(true);
          });
          reap_and_store(server->threads_mutex, server->exec_threads, std::move(worker), finished);
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
    // goal and spawned a joinable execute worker: remove the endpoints, then
    // JOIN before ctx is destroyed (a joinable std::thread destructor would
    // call std::terminate). Matches the native-server teardown in stop(), which
    // likewise joins without a cancel signal (native goals are weak refs).
    remove_native_service_server(goal_server);
    remove_writer(ctx->feedback_topic);
    join_exec_threads(ctx->threads_mutex, ctx->exec_threads);
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
#endif
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
    join_exec_threads(ctx->threads_mutex, ctx->exec_threads);
  }
  for (auto &ctx : native_action_servers_) {
    if (ctx) {
      join_exec_threads(ctx->threads_mutex, ctx->exec_threads);
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

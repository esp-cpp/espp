#include "rtps_participant.hpp"

#include <cstdio>

#include "rtps/entities/Domain.hpp"

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

RtpsParticipant::~RtpsParticipant() { stop(); }

bool RtpsParticipant::resolve_interface_address(std::array<uint8_t, 4> &ip_bytes) const {
  std::string addr = config_.interface_address;
  // Skip loopback / link-local candidates during auto-detection. Unused on ESP
  // targets, which require an explicit interface_address.
  [[maybe_unused]] const auto usable = [](const std::string &ip) {
    return !ip.empty() && ip.rfind("127.", 0) != 0 && ip.rfind("169.254.", 0) != 0;
  };
#if defined(ESP_PLATFORM)
  if (addr.empty()) {
    logger_.error("interface_address must be set on ESP targets (e.g. from the netif IP)");
    return false;
  }
#elif defined(_WIN32)
  if (addr.empty()) {
    // Auto-detect: first up, non-loopback IPv4 adapter (GetAdaptersAddresses).
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
      for (auto *a = adapters; a != nullptr && addr.empty(); a = a->Next) {
        if (a->OperStatus != IfOperStatusUp || a->IfType == IF_TYPE_SOFTWARE_LOOPBACK) {
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
          if (usable(buf)) {
            addr = buf;
            break;
          }
        }
      }
    }
    if (addr.empty()) {
      logger_.error("Could not auto-detect a usable IPv4 interface; set interface_address");
      return false;
    }
    logger_.info("Auto-detected interface address {}", addr);
  }
#else
  if (addr.empty()) {
    // Auto-detect: first non-loopback, non-link-local IPv4 interface.
    struct ifaddrs *ifaddr = nullptr;
    if (getifaddrs(&ifaddr) == 0) {
      for (struct ifaddrs *ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) {
          continue;
        }
        char buf[INET_ADDRSTRLEN] = {0};
        const auto *sin = reinterpret_cast<const struct sockaddr_in *>(ifa->ifa_addr);
        if (inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof(buf)) == nullptr) {
          continue;
        }
        if (usable(buf)) {
          addr = buf;
          break;
        }
      }
      freeifaddrs(ifaddr);
    }
    if (addr.empty()) {
      logger_.error("Could not auto-detect a usable IPv4 interface; set interface_address");
      return false;
    }
    logger_.info("Auto-detected interface address {}", addr);
  }
#endif
  unsigned a = 0, b = 0, c = 0, d = 0;
  if (std::sscanf(addr.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) != 4 || a > 255 || b > 255 ||
      c > 255 || d > 255) {
    logger_.error("Invalid interface_address '{}'", addr);
    return false;
  }
  ip_bytes = {static_cast<uint8_t>(a), static_cast<uint8_t>(b), static_cast<uint8_t>(c),
              static_cast<uint8_t>(d)};
  return true;
}

bool RtpsParticipant::start() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (started_) {
    logger_.warn("Already started");
    return false;
  }

  std::array<uint8_t, 4> ip_bytes{};
  if (!resolve_interface_address(ip_bytes)) {
    return false;
  }

  domain_ = std::make_unique<rtps::Domain>(ip_bytes);

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
  if (config_.on_subscriber_matched) {
    participant_->registerOnNewSubscriberMatchedCallback(&subscriber_matched_trampoline, this);
  }

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

void RtpsParticipant::stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!started_) {
    return;
  }
  started_ = false;
  domain_->stop();
  // The engine owns the endpoint objects; drop our references before the
  // domain (and with it every writer/reader and their callback registrations)
  // goes away.
  writers_.clear();
  participant_ = nullptr;
  domain_.reset();
  reader_contexts_.clear();
  logger_.info("Stopped");
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
  rtps::Writer *writer =
      domain_->createWriter(*participant_, config.topic.c_str(), config.type_name.c_str(),
                            config.reliability == Reliability::RELIABLE);
  if (writer == nullptr) {
    logger_.error("Engine could not create writer '{}' (pool exhausted or name too long)",
                  config.topic);
    return false;
  }
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
  rtps::Reader *reader =
      domain_->createReader(*participant_, config.topic.c_str(), config.type_name.c_str(),
                            config.reliability == Reliability::RELIABLE);
  if (reader == nullptr) {
    logger_.error("Engine could not create reader '{}' (pool exhausted or name too long)",
                  config.topic);
    return false;
  }
  auto ctx = std::make_unique<ReaderContext>();
  ctx->self = this;
  ctx->on_sample = config.on_sample;
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
  const auto *change = it->second->newChange(rtps::ChangeKind_t::ALIVE, cdr_payload.data(),
                                             static_cast<rtps::DataSize_t>(cdr_payload.size()));
  if (change == nullptr) {
    logger_.warn("Writer history full for topic '{}'; sample dropped", topic);
    return false;
  }
  return true;
}

void RtpsParticipant::reader_trampoline(void *arg, const rtps::ReaderCacheChange &change) {
  auto *ctx = static_cast<ReaderContext *>(arg);
  if (ctx == nullptr || !ctx->on_sample) {
    return;
  }
  // Serialize deliveries per reader: the engine may invoke this from a worker
  // thread while a previous delivery is still running.
  std::lock_guard<std::mutex> lock(ctx->buffer_mutex);
  const auto size = change.getDataSize();
  ctx->buffer.resize(size);
  if (size == 0 || !change.copyInto(ctx->buffer.data(), size)) {
    return;
  }
  ctx->on_sample(std::span<const uint8_t>(ctx->buffer.data(), ctx->buffer.size()));
}

void RtpsParticipant::publisher_matched_trampoline(void *arg) {
  auto *self = static_cast<RtpsParticipant *>(arg);
  if (self != nullptr && self->config_.on_publisher_matched) {
    self->config_.on_publisher_matched();
  }
}

void RtpsParticipant::subscriber_matched_trampoline(void *arg) {
  auto *self = static_cast<RtpsParticipant *>(arg);
  if (self != nullptr && self->config_.on_subscriber_matched) {
    self->config_.on_subscriber_matched();
  }
}

} // namespace espp

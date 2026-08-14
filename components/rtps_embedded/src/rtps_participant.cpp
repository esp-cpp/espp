#include "rtps_participant.hpp"

#include <condition_variable>
#include <cstdio>
#include <limits>
#include <unordered_map>

#include "rtps/entities/Domain.hpp"
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
  // A zero fragment size is invalid: the fragmented send paths treat it as
  // failure and would silently drop every large sample while publish() reports
  // success. Reject it up front rather than creating a broken writer.
  if (config.fragment_size == 0) {
    logger_.error("Writer '{}': fragment_size must be non-zero", config.topic);
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

// ===========================================================================
// Services (RMI) - request/reply with related_sample_identity correlation.
// ===========================================================================

namespace {
// Pack a SequenceNumber_t into a single key for the pending-request map.
uint64_t seq_key(const rtps::SequenceNumber_t &sn) {
  return (static_cast<uint64_t>(static_cast<uint32_t>(sn.high)) << 32) | sn.low;
}
} // namespace

// Per-server bridge: engine request-reader callback -> user handler -> reply.
struct RtpsParticipant::ServiceServerContext {
  RtpsParticipant *self{nullptr};
  service_handler_t handler{nullptr};
  rtps::Writer *reply_writer{nullptr};
  rtps::Reader *request_reader{nullptr};
};

// Client state: request writer + pending-request table keyed by the request's
// RTPS writerSeqNumber (which the server echoes in the reply's
// related_sample_identity), matched on our own reply-reader GUID.
struct RtpsParticipant::ServiceClient::Impl {
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
  rtps::Guid_t reply_reader_guid{};
  std::mutex mutex;
  std::unordered_map<uint64_t, Pending> pending;

  // Send a request carrying our reply-reader GUID as related_sample_identity
  // (with an UNKNOWN sequence number, per rmw), register the pending entry keyed
  // by the assigned writerSeqNumber, and return that key. nullopt on failure.
  std::optional<uint64_t> send(std::span<const uint8_t> request, reply_callback_t on_reply,
                               std::shared_ptr<SyncSlot> sync) {
    std::lock_guard<std::mutex> lock(mutex);
    // Hold the lock across newChange + insert so a reply can never look up the
    // key before it is registered (the send itself is async).
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
  // Copy the request payload (valid only during this callback).
  std::vector<uint8_t> request(change.getDataSize());
  if (!request.empty() && !change.copyInto(request.data(), change.getDataSize())) {
    return;
  }
  std::vector<uint8_t> reply = ctx->handler(request);

  // Correlate: echo {client reply-reader GUID (from the request's related
  // sample identity), request writerSeqNumber} so the client can match it.
  rtps::rpc::SampleIdentity related;
  related.writer_guid = change.hasRelatedSampleIdentity ? change.relatedSampleIdentity.writer_guid
                                                        : change.writerGuid;
  related.sequence_number = change.sn;
  ctx->reply_writer->newChangeWithRelatedSampleIdentity(rtps::ChangeKind_t::ALIVE, reply.data(),
                                                        static_cast<rtps::DataSize_t>(reply.size()),
                                                        related);
}

void RtpsParticipant::service_reply_trampoline(void *arg, const rtps::ReaderCacheChange &change) {
  auto *impl = static_cast<ServiceClient::Impl *>(arg);
  if (impl == nullptr || !change.hasRelatedSampleIdentity) {
    return;
  }
  // Only replies addressed to this client (our reply-reader GUID).
  if (!(change.relatedSampleIdentity.writer_guid == impl->reply_reader_guid)) {
    return;
  }
  const uint64_t key = seq_key(change.relatedSampleIdentity.sequence_number);

  std::vector<uint8_t> reply(change.getDataSize());
  if (!reply.empty() && !change.copyInto(reply.data(), change.getDataSize())) {
    return;
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
  if (pending.sync) {
    std::lock_guard<std::mutex> lock(pending.sync->m);
    pending.sync->reply = std::move(reply);
    pending.sync->done = true;
    pending.sync->cv.notify_one();
  } else if (pending.on_reply) {
    pending.on_reply(reply);
  }
}

RtpsParticipant::ServiceClient::ServiceClient(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}
RtpsParticipant::ServiceClient::~ServiceClient() = default;

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
  std::lock_guard<std::mutex> lock(mutex_);
  if (!started_) {
    logger_.error("Cannot add service server '{}': not started", config.service);
    return false;
  }
  const std::string req_topic = rtps::rpc::service_request_topic(config.service);
  const std::string rep_topic = rtps::rpc::service_reply_topic(config.service);
  const std::string req_type = rtps::rpc::service_request_type(config.type_name);
  const std::string rep_type = rtps::rpc::service_response_type(config.type_name);

  rtps::Writer *reply_writer =
      domain_->createWriter(*participant_, rep_topic.c_str(), rep_type.c_str(), /*reliable=*/true);
  rtps::Reader *request_reader =
      domain_->createReader(*participant_, req_topic.c_str(), req_type.c_str(), /*reliable=*/true);
  if (reply_writer == nullptr || request_reader == nullptr) {
    logger_.error("Service server '{}': endpoint creation failed", config.service);
    return false;
  }
  auto ctx = std::make_unique<ServiceServerContext>();
  ctx->self = this;
  ctx->handler = std::move(handler);
  ctx->reply_writer = reply_writer;
  ctx->request_reader = request_reader;
  if (request_reader->registerCallback(&service_request_trampoline, ctx.get()) == 0) {
    logger_.error("Service server '{}': could not register request callback", config.service);
    return false;
  }
  service_servers_.push_back(std::move(ctx));
  logger_.info("Added service server: '{}' ({})", config.service, config.type_name);
  return true;
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

  rtps::Reader *reply_reader =
      domain_->createReader(*participant_, rep_topic.c_str(), rep_type.c_str(), /*reliable=*/true);
  rtps::Writer *request_writer =
      domain_->createWriter(*participant_, req_topic.c_str(), req_type.c_str(), /*reliable=*/true);
  if (reply_reader == nullptr || request_writer == nullptr) {
    logger_.error("Service client '{}': endpoint creation failed", config.service);
    return nullptr;
  }
  auto impl = std::make_unique<ServiceClient::Impl>();
  impl->self = this;
  impl->request_writer = request_writer;
  impl->reply_reader_guid = reply_reader->m_attributes.endpointGuid;
  if (reply_reader->registerCallback(&service_reply_trampoline, impl.get()) == 0) {
    logger_.error("Service client '{}': could not register reply callback", config.service);
    return nullptr;
  }
  auto client = std::shared_ptr<ServiceClient>(new ServiceClient(std::move(impl)));
  service_clients_.push_back(client);
  logger_.info("Added service client: '{}' ({})", config.service, config.type_name);
  return client;
}

} // namespace espp

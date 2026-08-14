#include "rtps_participant.hpp"

#include <condition_variable>
#include <cstdio>
#include <limits>
#include <map>
#include <random>
#include <thread>
#include <unordered_map>

#include "rtps/entities/Domain.hpp"
#include "rtps/rpc/action_naming.hpp"
#include "rtps/rpc/action_types.hpp"
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
  // RPC endpoints are owned by the (now-reset) domain; drop our bookkeeping.
  // NOTE: an action server's detached execute threads are not joined here (v1);
  // goals are expected to finish before stop().
  action_clients_.clear();
  action_servers_.clear();
  service_clients_.clear();
  service_servers_.clear();
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
  service_deferred_handler_t handler{nullptr}; // sync handlers are wrapped as deferred
  rtps::Writer *reply_writer{nullptr};
  rtps::Reader *request_reader{nullptr};
};

// Deferred-reply state: the reply writer + the identity to echo, so a response
// can be sent once, later, from any thread.
struct RtpsParticipant::ServiceResponder::State {
  rtps::Writer *reply_writer{nullptr};
  rtps::rpc::SampleIdentity related{};
  std::atomic<bool> replied{false};
};

void RtpsParticipant::ServiceResponder::reply(std::span<const uint8_t> response) const {
  if (!state_ || state_->reply_writer == nullptr) {
    return;
  }
  bool expected = false;
  if (!state_->replied.compare_exchange_strong(expected, true)) {
    return; // reply exactly once
  }
  state_->reply_writer->newChangeWithRelatedSampleIdentity(
      rtps::ChangeKind_t::ALIVE, response.data(), static_cast<rtps::DataSize_t>(response.size()),
      state_->related);
}

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

  // Build a responder correlated to this request: echo {client reply-reader GUID
  // (from the request's related sample identity), request writerSeqNumber}. A
  // sync handler replies immediately; a deferred one may hold the responder.
  auto state = std::make_shared<ServiceResponder::State>();
  state->reply_writer = ctx->reply_writer;
  state->related.writer_guid = change.hasRelatedSampleIdentity
                                   ? change.relatedSampleIdentity.writer_guid
                                   : change.writerGuid;
  state->related.sequence_number = change.sn;
  ctx->handler(request, ServiceResponder(state));
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
  // A synchronous handler is a deferred handler that replies immediately.
  return add_service_server_deferred(
      config, [h = std::move(handler)](std::span<const uint8_t> request, ServiceResponder resp) {
        resp.reply(h(request));
      });
}

bool RtpsParticipant::add_service_server_deferred(const ServiceConfig &config,
                                                  service_deferred_handler_t handler) {
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

// ===========================================================================
// Actions (AMI) - 3 services (send_goal/cancel_goal/get_result) + 2 topics
// (feedback/status), composed over the service + pub/sub facade above.
// ===========================================================================

namespace {
namespace ract = rtps::rpc;

// Generate a unique 16-byte goal id: random_device bytes mixed with a process
// counter so uniqueness holds even if random_device is weak (e.g. on an MCU).
ract::GoalUuid generate_goal_id() {
  static std::atomic<uint32_t> counter{0};
  ract::GoalUuid id{};
  std::random_device rd;
  for (auto &b : id) {
    b = static_cast<uint8_t>(rd() & 0xFF);
  }
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
  std::vector<uint8_t> goal;                   // CDR goal payload
  RtpsParticipant *self{nullptr};              // for feedback/status publish
  std::shared_ptr<ActionServerContext> server; // owning server (topics + goals)
  std::mutex mutex;
  ract::GoalStatus status{ract::GoalStatus::ACCEPTED};
  bool done{false};
  std::vector<uint8_t> result;       // set on terminate
  ServiceResponder result_responder; // pending get_result (if any)
  std::atomic<bool> cancel_requested{false};
  std::thread exec_thread;
};

struct RtpsParticipant::ActionServerContext {
  RtpsParticipant *self{nullptr};
  std::string feedback_topic;
  std::string status_topic;
  action_execute_callback_t execute{nullptr};
  std::mutex goals_mutex;
  std::map<ract::GoalUuid, std::shared_ptr<ActionGoalHandle::State>> goals;
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
  state_->self->publish(state_->server->feedback_topic, {msg.data(), msg.size()});
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
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->done) {
      return;
    }
    state_->done = true;
    state_->status = status;
    state_->result.assign(result.begin(), result.end());
    responder = state_->result_responder; // fulfill any pending get_result
  }
  publish_goal_status(state_->self, state_->server->status_topic, state_->goal_id, status);
  if (responder.valid()) {
    responder.reply(
        ract::wrap_get_result_response(status, {state_->result.data(), state_->result.size()}));
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

bool RtpsParticipant::add_action_server(const ActionConfig &config, action_goal_callback_t on_goal,
                                        action_execute_callback_t execute,
                                        action_cancel_callback_t on_cancel) {
  if (!started_) {
    logger_.error("Cannot add action server '{}': not started", config.action);
    return false;
  }
  auto ctx = std::make_shared<ActionServerContext>();
  ctx->self = this;
  ctx->feedback_topic = rtps::rpc::action_feedback_topic(config.action);
  ctx->status_topic = rtps::rpc::action_status_topic(config.action);
  ctx->execute = std::move(execute);

  // Feedback + status publishers (plain reliable topics).
  if (!add_writer({ctx->feedback_topic, rtps::rpc::action_feedback_type(config.type_name),
                   Reliability::RELIABLE}) ||
      !add_writer({ctx->status_topic, rtps::rpc::action_status_type(), Reliability::RELIABLE})) {
    logger_.error("Action server '{}': feedback/status writer creation failed", config.action);
    return false;
  }

  auto weak = std::weak_ptr<ActionServerContext>(ctx);

  // send_goal service: accept/reject, then spawn the execute thread.
  const ServiceConfig send_goal_cfg{rtps::rpc::action_send_goal_service(config.action),
                                    rtps::rpc::action_send_goal_type(config.type_name)};
  bool ok = add_service_server(
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
        gstate->server = server;
        {
          std::lock_guard<std::mutex> lock(server->goals_mutex);
          server->goals[id] = gstate;
        }
        publish_goal_status(this, server->status_topic, id, ract::GoalStatus::EXECUTING);
        if (server->execute) {
          gstate->exec_thread =
              std::thread([server, gstate]() { server->execute(ActionGoalHandle(gstate)); });
          gstate->exec_thread.detach();
        }
        return ract::make_send_goal_response(true);
      });

  // get_result service (DEFERRED): reply now if done, else hold the responder.
  const ServiceConfig get_result_cfg{rtps::rpc::action_get_result_service(config.action),
                                     rtps::rpc::action_get_result_type(config.type_name)};
  ok = ok && add_service_server_deferred(
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
                   std::lock_guard<std::mutex> lock(gstate->mutex);
                   if (gstate->done) {
                     responder.reply(ract::wrap_get_result_response(
                         gstate->status, {gstate->result.data(), gstate->result.size()}));
                   } else {
                     gstate->result_responder = responder; // fulfilled on terminate()
                   }
                 });

  // cancel_goal service: mark the goal canceling; the execute callback observes
  // is_canceling(). Minimal CancelGoal_Response (return_code=0, empty list).
  const ServiceConfig cancel_cfg{rtps::rpc::action_cancel_goal_service(config.action),
                                 rtps::rpc::action_cancel_goal_type()};
  ok = ok &&
       add_service_server(
           cancel_cfg, [weak, on_cancel](std::span<const uint8_t> req) -> std::vector<uint8_t> {
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
    logger_.error("Action server '{}': service endpoint creation failed", config.action);
    return false;
  }
  action_servers_.push_back(std::move(ctx));
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
  auto impl = std::make_unique<ActionClient::Impl>();
  impl->self = this;
  impl->action = config.action;
  impl->send_goal_client = add_service_client({rtps::rpc::action_send_goal_service(config.action),
                                               rtps::rpc::action_send_goal_type(config.type_name)});
  impl->get_result_client =
      add_service_client({rtps::rpc::action_get_result_service(config.action),
                          rtps::rpc::action_get_result_type(config.type_name)});
  impl->cancel_client = add_service_client(
      {rtps::rpc::action_cancel_goal_service(config.action), rtps::rpc::action_cancel_goal_type()});
  if (!impl->send_goal_client || !impl->get_result_client || !impl->cancel_client) {
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
                   }})) {
    logger_.error("Action client '{}': feedback reader creation failed", config.action);
    return nullptr;
  }

  auto client = std::shared_ptr<ActionClient>(new ActionClient(std::move(impl)));
  action_clients_.push_back(client);
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

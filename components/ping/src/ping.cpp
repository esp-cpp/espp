#include "ping.hpp"

#include <cstring>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <lwip/netdb.h>

namespace espp {

Ping::Ping(const Config &config)
    : BaseComponent("Ping", config.log_level) {
  config_ = config;
}

Ping::~Ping() { destroy_session(); }

bool Ping::run(std::error_code &ec) { return run_sync(ec); }

bool Ping::run_async(std::error_code &ec) {
  std::unique_lock<std::mutex> slk(state_mutex_);
  if (session_active_) {
    ec = std::make_error_code(std::errc::device_or_resource_busy);
    return false;
  }
  if (!create_session(ec))
    return false;
  {
    std::lock_guard<std::mutex> clk(config_mutex_);
    if (config_.callbacks.on_session_start)
      config_.callbacks.on_session_start();
  }
  session_active_ = true;
  session_completed_ = false;
  esp_err_t err = esp_ping_start(handle_);
  if (err != ESP_OK) {
    logger_.error("esp_ping_start failed: {}", (int)err);
    ec = std::make_error_code(std::errc::io_error);
    destroy_session();
    session_active_ = false;
    state_cv_.notify_all();
    return false;
  }
  ec.clear();
  return true;
}

bool Ping::run_sync(std::error_code &ec) {
  if (!run_async(ec))
    return false;
  std::unique_lock<std::mutex> slk(state_mutex_);
  state_cv_.wait(slk, [&] { return session_completed_ || !session_active_; });
  ec.clear();
  return true;
}

void Ping::stop() {
  std::lock_guard<std::mutex> slk(state_mutex_);
  if (handle_) {
    destroy_session();
    session_active_ = false;
    session_completed_ = true;
    state_cv_.notify_all();
  }
}

bool Ping::is_running() const {
  std::lock_guard<std::mutex> slk(state_mutex_);
  return session_active_;
}

void Ping::set_target_host(std::string_view host) {
  std::lock_guard<std::mutex> clk(config_mutex_);
  config_.session.target_host.assign(host.data(), host.size());
}

std::string Ping::get_target_host() const {
  std::lock_guard<std::mutex> clk(config_mutex_);
  return config_.session.target_host;
}

void Ping::set_count(uint32_t count) {
  std::lock_guard<std::mutex> clk(config_mutex_);
  config_.session.count = count;
}
uint32_t Ping::get_count() const {
  std::lock_guard<std::mutex> clk(config_mutex_);
  return config_.session.count;
}

void Ping::set_interval_ms(uint32_t interval_ms) {
  std::lock_guard<std::mutex> clk(config_mutex_);
  config_.session.interval_ms = interval_ms;
}
uint32_t Ping::get_interval_ms() const {
  std::lock_guard<std::mutex> clk(config_mutex_);
  return config_.session.interval_ms;
}

void Ping::set_timeout_ms(uint32_t timeout_ms) {
  std::lock_guard<std::mutex> clk(config_mutex_);
  config_.session.timeout_ms = timeout_ms;
}
uint32_t Ping::get_timeout_ms() const {
  std::lock_guard<std::mutex> clk(config_mutex_);
  return config_.session.timeout_ms;
}

void Ping::set_data_size(uint32_t data_size) {
  std::lock_guard<std::mutex> clk(config_mutex_);
  config_.session.data_size = data_size;
}
uint32_t Ping::get_data_size() const {
  std::lock_guard<std::mutex> clk(config_mutex_);
  return config_.session.data_size;
}

void Ping::set_ttl(uint8_t ttl) {
  std::lock_guard<std::mutex> clk(config_mutex_);
  config_.session.ttl = ttl;
}
uint8_t Ping::get_ttl() const {
  std::lock_guard<std::mutex> clk(config_mutex_);
  return config_.session.ttl;
}

void Ping::set_tos(uint8_t tos) {
  std::lock_guard<std::mutex> clk(config_mutex_);
  config_.session.tos = tos;
}
uint8_t Ping::get_tos() const {
  std::lock_guard<std::mutex> clk(config_mutex_);
  return config_.session.tos;
}

void Ping::set_task_stack_size(size_t bytes) {
  std::lock_guard<std::mutex> clk(config_mutex_);
  config_.session.task_stack_size = bytes;
}
size_t Ping::get_task_stack_size() const {
  std::lock_guard<std::mutex> clk(config_mutex_);
  return config_.session.task_stack_size;
}

Ping::SessionConfig Ping::get_session_config() const {
  std::lock_guard<std::mutex> clk(config_mutex_);
  return config_.session;
}

std::unique_ptr<cli::Menu> Ping::Menu::get(std::string_view name, std::string_view description) {
  auto menu = std::make_unique<cli::Menu>(std::string(name), std::string(description));
  menu->Insert(
      "ping", {"host"},
      [this](std::ostream &out, const std::string &host) {
        std::error_code ec;
        ping_.get().set_target_host(host);
        bool ok = ping_.get().run_sync(ec);
        if (!ok || ec) {
          out << "Ping failed\n";
        } else {
          out << "Ping complete\n";
        }
      },
      "Run ping to host (default count/interval/timeout)");

  // setters/getters
  menu->Insert(
      "host", [this](std::ostream &out) { out << ping_.get().get_target_host() << "\n"; },
      "Get target host");
  menu->Insert(
      "host", {"hostname"},
      [this](std::ostream &out, const std::string &hostname) {
        ping_.get().set_target_host(hostname);
        out << "OK\n";
      },
      "Set target host");

  menu->Insert(
      "count", [this](std::ostream &out) { out << ping_.get().get_count() << "\n"; },
      "Get ping count");
  menu->Insert(
      "count", {"n"},
      [this](std::ostream &out, int n) {
        ping_.get().set_count((uint32_t)n);
        out << "OK\n";
      },
      "Set ping count");

  menu->Insert(
      "interval", [this](std::ostream &out) { out << ping_.get().get_interval_ms() << "\n"; },
      "Get interval (ms)");
  menu->Insert(
      "interval", {"ms"},
      [this](std::ostream &out, int ms) {
        ping_.get().set_interval_ms((uint32_t)ms);
        out << "OK\n";
      },
      "Set interval (ms)");

  menu->Insert(
      "timeout", [this](std::ostream &out) { out << ping_.get().get_timeout_ms() << "\n"; },
      "Get timeout (ms)");
  menu->Insert(
      "timeout", {"ms"},
      [this](std::ostream &out, int ms) {
        ping_.get().set_timeout_ms((uint32_t)ms);
        out << "OK\n";
      },
      "Set timeout (ms)");

  menu->Insert(
      "size", [this](std::ostream &out) { out << ping_.get().get_data_size() << "\n"; },
      "Get payload size");
  menu->Insert(
      "size", {"bytes"},
      [this](std::ostream &out, int bytes) {
        ping_.get().set_data_size((uint32_t)bytes);
        out << "OK\n";
      },
      "Set payload size (bytes)");

  menu->Insert(
      "ttl", [this](std::ostream &out) { out << (int)ping_.get().get_ttl() << "\n"; }, "Get TTL");
  menu->Insert(
      "ttl", {"value"},
      [this](std::ostream &out, int v) {
        ping_.get().set_ttl((uint8_t)v);
        out << "OK\n";
      },
      "Set TTL");

  menu->Insert(
      "tos", [this](std::ostream &out) { out << (int)ping_.get().get_tos() << "\n"; }, "Get TOS");
  menu->Insert(
      "tos", {"value"},
      [this](std::ostream &out, int v) {
        ping_.get().set_tos((uint8_t)v);
        out << "OK\n";
      },
      "Set TOS");

  menu->Insert(
      "stack", [this](std::ostream &out) { out << ping_.get().get_task_stack_size() << "\n"; },
      "Get ping task stack size");
  menu->Insert(
      "stack", {"bytes"},
      [this](std::ostream &out, int bytes) {
        ping_.get().set_task_stack_size((size_t)bytes);
        out << "OK\n";
      },
      "Set ping task stack size (bytes)");

  // Show full config
  menu->Insert(
      "config",
      [this](std::ostream &out) {
        auto s = ping_.get().get_session_config();
        out << fmt::format("{}\n", s);
      },
      "Show current session configuration");

  // Set full config in one command
  menu->Insert(
      "config", {"host", "count", "interval_ms", "timeout_ms", "size", "ttl", "tos", "stack"},
      [this](std::ostream &out, const std::string &host, uint32_t count, uint32_t interval_ms,
             uint32_t timeout_ms, uint32_t size, uint8_t ttl, uint8_t tos, size_t stack) {
        ping_.get().set_target_host(host);
        ping_.get().set_count(count);
        ping_.get().set_interval_ms(interval_ms);
        ping_.get().set_timeout_ms(timeout_ms);
        ping_.get().set_data_size(size);
        ping_.get().set_ttl(ttl);
        ping_.get().set_tos(tos);
        ping_.get().set_task_stack_size(stack);
        out << "OK\n";
      },
      "Set session config: host count interval_ms timeout_ms size ttl tos stack");
  return menu;
}

void Ping::on_ping_success(void *h, void *args) {
  auto *self = static_cast<Ping *>(args);
  if (!self)
    return;
  self->handle_success(h);
}

void Ping::on_ping_timeout(void *h, void *args) {
  auto *self = static_cast<Ping *>(args);
  if (!self)
    return;
  self->handle_timeout(h);
}

void Ping::on_ping_end(void *h, void *args) {
  auto *self = static_cast<Ping *>(args);
  if (!self)
    return;
  self->handle_end(h);
}

void Ping::handle_success(void *h) {
  uint32_t seqno, ttl, elapsed_time, data_size;
  esp_ping_get_profile(h, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
  esp_ping_get_profile(h, ESP_PING_PROF_TTL, &ttl, sizeof(ttl));
  esp_ping_get_profile(h, ESP_PING_PROF_TIMEGAP, &elapsed_time, sizeof(elapsed_time));
  esp_ping_get_profile(h, ESP_PING_PROF_SIZE, &data_size, sizeof(data_size));
  {
    std::lock_guard<std::mutex> clk(config_mutex_);
    if (config_.callbacks.on_reply)
      config_.callbacks.on_reply(seqno, ttl, elapsed_time, data_size);
  }
}

void Ping::handle_timeout(void *h) {
  uint32_t seqno = 0;
  esp_ping_get_profile(h, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
  logger_.debug("Request timed out for seq={}", seqno);
  std::lock_guard<std::mutex> clk(config_mutex_);
  if (config_.callbacks.on_timeout)
    config_.callbacks.on_timeout();
}

void Ping::handle_end(void *h) {
  logger_.debug("Ping session ended");
  // Summarize
  uint32_t transmitted = 0, received = 0;
  uint32_t avg_time = 0;
  esp_ping_get_profile(h, ESP_PING_PROF_REQUEST, &transmitted, sizeof(transmitted));
  esp_ping_get_profile(h, ESP_PING_PROF_REPLY, &received, sizeof(received));
  esp_ping_get_profile(h, ESP_PING_PROF_TIMEGAP, &avg_time, sizeof(avg_time));
  uint32_t loss_pct =
      transmitted == 0 ? 0 : (uint32_t)((100.0f * (transmitted - received)) / transmitted);
  uint32_t min_time = 0;
  uint32_t max_time = 0;
  {
    std::lock_guard<std::mutex> clk(config_mutex_);
    if (config_.callbacks.on_end)
      config_.callbacks.on_end(transmitted, received, loss_pct, avg_time, min_time, max_time);
  }
  {
    std::lock_guard<std::mutex> slk(state_mutex_);
    session_completed_ = true;
    session_active_ = false;
  }
  state_cv_.notify_all();
  destroy_session();
}

bool Ping::create_session(std::error_code &ec) {
  destroy_session();
  ip_addr_t target = resolve_target(ec);
  if (ec)
    return false;

  esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();
  {
    std::lock_guard<std::mutex> clk(config_mutex_);
    auto &s = config_.session;
    if (s.count > 0)
      ping_config.count = s.count;
    if (s.interval_ms > 0)
      ping_config.interval_ms = s.interval_ms;
    if (s.timeout_ms > 0)
      ping_config.timeout_ms = s.timeout_ms;
    if (s.data_size > 0)
      ping_config.data_size = s.data_size;
    if (s.ttl > 0)
      ping_config.ttl = s.ttl;
    if (s.tos > 0)
      ping_config.tos = s.tos;
    if (s.task_stack_size > 0)
      ping_config.task_stack_size = s.task_stack_size;
  }
  ping_config.target_addr = target;

  esp_ping_callbacks_t cbs = {};
  cbs.on_ping_success = &Ping::on_ping_success;
  cbs.on_ping_timeout = &Ping::on_ping_timeout;
  cbs.on_ping_end = &Ping::on_ping_end;
  cbs.cb_args = this;

  esp_err_t err = esp_ping_new_session(&ping_config, &cbs, &handle_);
  if (err != ESP_OK) {
    logger_.error("esp_ping_new_session failed: {}", (int)err);
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  ec.clear();
  return true;
}

void Ping::destroy_session() {
  if (handle_) {
    esp_ping_delete_session(handle_);
    handle_ = nullptr;
  }
}

ip_addr_t Ping::resolve_target(std::error_code &ec) const {
  std::string input_host;
  {
    std::lock_guard<std::mutex> clk(config_mutex_);
    input_host = config_.session.target_host;
  }
  ip_addr_t addr = IPADDR4_INIT(0);
  struct addrinfo *res = NULL;

  // Try dotted-quad first
  ip4_addr_t ip4;
  if (ip4addr_aton(input_host.c_str(), &ip4)) {
    IP_ADDR4(&addr, ip4.addr & 0xFF, (ip4.addr >> 8) & 0xFF, (ip4.addr >> 16) & 0xFF,
             (ip4.addr >> 24) & 0xFF);
    ec.clear();
    return addr;
  }
  // DNS resolve using getaddrinfo
  struct addrinfo hints = {};
  hints.ai_family = AF_INET;
  int r = getaddrinfo(input_host.c_str(), nullptr, &hints, &res);
  if (r == 0 && res && res->ai_addrlen >= sizeof(struct sockaddr_in)) {
    auto *sa = reinterpret_cast<struct sockaddr_in *>(res->ai_addr);
    uint32_t a = ntohl(sa->sin_addr.s_addr);
    IP_ADDR4(&addr, (a >> 24) & 0xFF, (a >> 16) & 0xFF, (a >> 8) & 0xFF, a & 0xFF);
    freeaddrinfo(res);
    ec.clear();
    return addr;
  }
  if (res)
    freeaddrinfo(res);
  ec = std::make_error_code(std::errc::host_unreachable);
  return addr;
}

} // namespace espp

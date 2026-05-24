#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "base_component.hpp"

namespace espp {

enum class AndroidTvRemoteErrc {
  success = 0,
  invalid_configuration,
  invalid_argument,
  not_ready,
  nvs_error,
  discovery_failed,
  pairing_failed,
  tls_error,
  connect_failed,
  handshake_failed,
  transport_error,
  timeout,
  protocol_error,
  unsupported,
  not_connected,
  canceled,
};

std::error_code make_error_code(AndroidTvRemoteErrc e);

/**
 * @brief Practical Android TV Remote v2 client for Google TV / Chromecast.
 *
 * This first release focuses on device discovery, pairing, connection
 * management, remote key injection, IME text input, and media helpers.
 *
 * \section android_tv_remote_ex1 Android TV Remote Example
 * \snippet android_tv_remote_example.cpp android tv remote example
 */
class AndroidTvRemote : public BaseComponent {
public:
  struct DeviceInfo {
    std::string name;
    std::string host;
    std::string hostname;
    uint16_t port{6466};
    std::map<std::string, std::string> txt;
  };

  enum class Key : uint32_t {
    Home = 3,
    Back = 4,
    DpadUp = 19,
    DpadDown = 20,
    DpadLeft = 21,
    DpadRight = 22,
    Enter = 23,
    VolumeUp = 24,
    VolumeDown = 25,
    Search = 84,
    PlayPause = 85,
    Next = 87,
    Previous = 88,
    Rewind = 89,
    FastForward = 90,
    Mute = 164,
  };

  enum class Action : uint32_t {
    StartLong = 1,
    EndLong = 2,
    Short = 3,
  };

  struct DiscoveryConfig {
    bool initialize_mdns{true};
    uint32_t timeout_ms{3000};
    size_t max_results{8};
  };

  struct PairingConfig {
    std::string client_name{"ESPP Android TV Remote"};
    uint16_t port{6467};
    std::chrono::milliseconds timeout{8000};
  };

  struct TransportConfig {
    uint16_t port{6466};
    bool enable_ime{true};
    bool skip_common_name_check{true};
    std::chrono::milliseconds connect_timeout{5000};
    std::chrono::milliseconds handshake_timeout{8000};
    std::chrono::milliseconds read_poll_timeout{250};
  };

  struct PersistenceConfig {
    std::string nvs_namespace{"atvremote"};
    std::string certificate_key{"cert_pem"};
    std::string private_key_key{"key_pem"};
  };

  struct Config {
    DiscoveryConfig discovery;
    PairingConfig pairing;
    TransportConfig transport;
    PersistenceConfig persistence;
    Logger::Verbosity log_level{Logger::Verbosity::WARN};
  };

  explicit AndroidTvRemote(const Config &config = {});
  ~AndroidTvRemote();

  bool discover(std::vector<DeviceInfo> &devices, std::error_code &ec);
  bool pair(std::string_view host, const std::function<std::optional<std::string>()> &code_provider,
            std::error_code &ec);
  bool connect(std::string_view host, std::error_code &ec);
  void disconnect();

  bool is_connected() const;
  std::optional<DeviceInfo> get_last_device_info() const;

  bool send_key(Key key, Action action, std::error_code &ec);
  bool send_text(std::string_view text, std::error_code &ec);

  bool home(std::error_code &ec) { return send_key(Key::Home, Action::Short, ec); }
  bool back(std::error_code &ec) { return send_key(Key::Back, Action::Short, ec); }
  bool search(std::error_code &ec) { return send_key(Key::Search, Action::Short, ec); }
  bool volume_up(std::error_code &ec) { return send_key(Key::VolumeUp, Action::Short, ec); }
  bool volume_down(std::error_code &ec) { return send_key(Key::VolumeDown, Action::Short, ec); }
  bool mute(std::error_code &ec) { return send_key(Key::Mute, Action::Short, ec); }
  bool media_play_pause(std::error_code &ec) { return send_key(Key::PlayPause, Action::Short, ec); }
  bool media_next(std::error_code &ec) { return send_key(Key::Next, Action::Short, ec); }
  bool media_previous(std::error_code &ec) { return send_key(Key::Previous, Action::Short, ec); }
  bool media_fast_forward(std::error_code &ec) {
    return send_key(Key::FastForward, Action::Short, ec);
  }
  bool media_rewind(std::error_code &ec) { return send_key(Key::Rewind, Action::Short, ec); }

protected:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace espp

namespace std {
template <> struct is_error_code_enum<espp::AndroidTvRemoteErrc> : true_type {};
} // namespace std

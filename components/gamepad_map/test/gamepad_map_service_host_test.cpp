// Host-buildable unit tests for GamepadMapService: requests go in through
// handle_frame(), replies are captured from `send` and parsed back; the store is
// a GamepadProfiles file in a temp directory. Covers the rumble options on the
// wire (INFO / MAP / SET_MAP, with and without the rumble block), the change
// callbacks, and a controller connecting while a change is being saved. Build &
// run from the repository root (third-party headers are -isystem so their own
// warnings do not fail the build):
//
//   g++ -std=c++23 -Wall -Wextra -Werror -Wno-comment \
//       -I components/gamepad_map/include -I components/persistent_data/include \
//       -I components/serialization/include -I components/base_component/include \
//       -I components/logger/include -I components/format/include \
//       -I components/task/include -I components/dispatcher/include \
//       -I components/stream_frame/include \
//       -isystem components/serialization/detail/alpaca/include \
//       -isystem components/format/detail/fmt/include \
//       components/gamepad_map/test/gamepad_map_service_host_test.cpp \
//       components/logger/src/logger.cpp components/task/src/task.cpp -lpthread \
//       -o gamepad_map_service_host_test
//   ./gamepad_map_service_host_test
//
// No ESP-IDF headers required.

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "gamepad_map_service.hpp"

namespace proto = espp::detail::gamepad_map_protocol;
using espp::GamepadMap;
using espp::GamepadMapService;
using espp::GamepadProfiles;
using espp::RumbleOptions;
using B = espp::GamepadState::Button;
using Type = proto::Type;
using Bytes = std::vector<uint8_t>;

static int failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                  \
      failures++;                                                                                  \
    }                                                                                              \
  } while (0)

// change() is protected: reach it to run a save that a controller connects during
class TestService : public GamepadMapService {
public:
  using GamepadMapService::GamepadMapService;
  template <typename Save> bool run_change(Save save) { return change(save); }
};

// Captures what the service sends and what the application is told.
struct Harness {
  espp::stream_frame::StreamParser parser;
  std::vector<espp::stream_frame::Frame> frames;
  int map_calls{0};
  int rumble_calls{0};
  GamepadMap last_map{};
  RumbleOptions last_rumble{};

  GamepadMapService::Config config(GamepadProfiles *profiles) {
    return {.send =
                [this](std::span<const uint8_t> f) {
                  auto parsed = parser.feed(f);
                  frames.insert(frames.end(), parsed.begin(), parsed.end());
                },
            .profiles = profiles,
            .on_active_map_changed =
                [this](const GamepadMap &m) {
                  ++map_calls;
                  last_map = m;
                },
            .on_active_rumble_changed =
                [this](const RumbleOptions &r) {
                  ++rumble_calls;
                  last_rumble = r;
                }};
  }
  void reset_calls() { map_calls = rumble_calls = 0; }

  // Send one request; return the (single) reply.
  std::optional<espp::stream_frame::Frame> request(GamepadMapService &service, Type type,
                                                   const Bytes &payload = {}) {
    frames.clear();
    service.handle_frame(static_cast<uint8_t>(type), payload);
    // (the service's `send` fills `frames` from inside handle_frame())
    // cppcheck-suppress knownConditionTrueFalse
    if (frames.size() != 1)
      return std::nullopt;
    return frames.front();
  }
};

static bool is_ok(const std::optional<espp::stream_frame::Frame> &f) {
  return f && f->type == static_cast<uint8_t>(Type::Ok);
}

// ERROR: [request type u8][code u32][message]
static bool is_error(const std::optional<espp::stream_frame::Frame> &f, uint32_t code,
                     const std::string &message_prefix) {
  if (!f || f->type != static_cast<uint8_t>(Type::Error) || f->payload.size() < 5)
    return false;
  const auto &p = f->payload;
  const uint32_t got = static_cast<uint32_t>(p[1] | (p[2] << 8) | (p[3] << 16) | (p[4] << 24));
  const std::string message(p.begin() + 5, p.end());
  return got == code && message.rfind(message_prefix, 0) == 0;
}

static std::optional<proto::MapReply> get_map(Harness &h, GamepadMapService &s,
                                              const proto::Target &t) {
  const auto f = h.request(s, Type::GetMap, proto::encode_get_map(t));
  if (!f || f->type != static_cast<uint8_t>(Type::Map))
    return std::nullopt;
  return proto::decode_map_reply(f->payload);
}

int main() {
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "gamepad_map_service_host_test";
  fs::remove_all(dir);
  fs::create_directories(dir);
  std::error_code ec;

  const proto::Target def{};
  const proto::Target x{.profile = true, .vid = 0x1949, .pid = 0x0402, .name = "BSP-D8"};
  const proto::Target y{.profile = true, .vid = 0x045e, .pid = 0x0b13, .name = "Xbox"};
  const RumbleOptions off100{.flags = RumbleOptions::RUMBLE_OFF, .strength_percent = 100};
  const RumbleOptions swap150{.flags = RumbleOptions::SWAP_MOTORS, .strength_percent = 150};
  const RumbleOptions p150{.flags = 0, .strength_percent = 150};
  GamepadMap swapped;
  swapped.swap(B::South, B::East);

  GamepadProfiles profiles({.file_path = dir / "profiles.bin"});
  Harness h;
  TestService service(h.config(&profiles));

  // 1. INFO advertises rumble options (flags bit1)
  {
    const auto f = h.request(service, Type::GetInfo);
    CHECK(f && f->type == static_cast<uint8_t>(Type::Info));
    if (f) {
      const auto info = proto::decode_info(f->payload);
      CHECK(info && info->rumble_options);
    }
  }

  // 2. MAP always carries the rumble block
  {
    auto m = get_map(h, service, def);
    CHECK(m && m->exists && m->rumble == RumbleOptions{}); // default target: neutral
    CHECK(profiles.set_default_rumble(p150, ec));
    m = get_map(h, service, x);
    CHECK(m && !m->exists && m->rumble == p150); // missing profile: the default options
    CHECK(
        profiles.set_profile({.vid = x.vid, .pid = x.pid, .name = x.name, .rumble = swap150}, ec));
    m = get_map(h, service, x);
    CHECK(m && m->exists && m->rumble == swap150); // the profile's own
  }

  // 3. SET_MAP with a block saves map + rumble together
  {
    CHECK(is_ok(h.request(service, Type::SetMap, proto::encode_set_map(x, swapped, off100))));
    const auto m = get_map(h, service, x);
    CHECK(m && m->map == swapped && m->rumble == off100);
  }

  // 4. SET_MAP without a block (an older host) keeps the stored options
  {
    CHECK(is_ok(h.request(service, Type::SetMap, proto::encode_set_map(x, GamepadMap{}))));
    CHECK(profiles.map_for(x.vid, x.pid).is_identity() &&
          profiles.rumble_for(x.vid, x.pid) == off100);
    // a new profile starts from the default options (150 %)
    CHECK(is_ok(h.request(service, Type::SetMap, proto::encode_set_map(y, swapped))));
    CHECK(profiles.rumble_for(y.vid, y.pid) == p150);
    // the default target keeps the default options
    CHECK(is_ok(h.request(service, Type::SetMap, proto::encode_set_map(def, swapped))));
    CHECK(profiles.default_map() == swapped && profiles.default_rumble() == p150);
  }

  // 5. invalid options are refused, nothing saved; a short block is malformed
  {
    const auto before = profiles.rumble_for(x.vid, x.pid);
    CHECK(is_error(h.request(service, Type::SetMap,
                             proto::encode_set_map(x, GamepadMap{}, RumbleOptions{0, 201})),
                   22, "invalid rumble options"));
    CHECK(is_error(h.request(service, Type::SetMap,
                             proto::encode_set_map(x, GamepadMap{}, RumbleOptions{0x04, 100})),
                   22, "invalid rumble options"));
    Bytes one = proto::encode_set_map(x, GamepadMap{});
    one.push_back(0x01);
    CHECK(is_error(h.request(service, Type::SetMap, one), 22, "malformed SET_MAP"));
    CHECK(profiles.rumble_for(x.vid, x.pid) == before);
  }

  // 6. callbacks fire only for the connected controller's own changes
  {
    h.reset_calls();
    // nothing connected: nothing fires
    CHECK(is_ok(h.request(service, Type::SetMap, proto::encode_set_map(x, swapped, swap150))));
    CHECK(h.map_calls == 0 && h.rumble_calls == 0);

    service.set_controller(x.vid, x.pid, x.name);
    CHECK(service.active_rumble() == swap150);
    h.reset_calls();
    // X's rumble only
    CHECK(is_ok(h.request(service, Type::SetMap, proto::encode_set_map(x, swapped, off100))));
    CHECK(h.rumble_calls == 1 && h.last_rumble == off100 && h.map_calls == 0);
    h.reset_calls();
    // X's map only (the v1 form keeps the rumble)
    CHECK(is_ok(h.request(service, Type::SetMap, proto::encode_set_map(x, GamepadMap{}))));
    CHECK(h.map_calls == 1 && h.last_map.is_identity() && h.rumble_calls == 0);
    h.reset_calls();
    // another profile: nothing
    CHECK(is_ok(h.request(service, Type::SetMap, proto::encode_set_map(y, swapped, swap150))));
    CHECK(h.map_calls == 0 && h.rumble_calls == 0);
    // erasing X's profile: X falls back to the default options (150 %)
    CHECK(is_ok(
        h.request(service, Type::EraseProfile, proto::encode_erase_profile(x.vid, x.pid, x.name))));
    CHECK(h.rumble_calls == 1 && h.last_rumble == p150);
    h.reset_calls();
    // reset: neutral
    CHECK(is_ok(h.request(service, Type::ResetAll)));
    CHECK(h.rumble_calls == 1 && h.last_rumble == RumbleOptions{});
    service.clear_controller();
  }

  // 7. a controller that connects while a change is saved is told about it
  {
    h.reset_calls();
    GamepadMap m2;
    m2.left_deadzone_percent = 9;
    CHECK(service.run_change([&] {
      service.set_controller(x.vid, x.pid, x.name); // looks up the map before the save lands
      return profiles.set_profile(
          {.vid = x.vid, .pid = x.pid, .name = x.name, .map = m2, .rumble = swap150}, ec);
    }));
    CHECK(h.map_calls == 1 && h.last_map == m2);
    CHECK(h.rumble_calls == 1 && h.last_rumble == swap150);
    service.clear_controller();
  }

  fs::remove_all(dir);
  if (failures == 0) {
    std::printf("gamepad_map_service_host_test: all checks passed\n");
    return 0;
  }
  std::printf("gamepad_map_service_host_test: %d failure(s)\n", failures);
  return 1;
}

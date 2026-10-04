#pragma once

// CANopen / DS402: an NMT master + SDO client (espp::CanopenClient) and a CiA
// 402 drive panel (espp::Ds402Drive) on the bus Kconfig selects -- the
// in-firmware simulated DS402 node (the can_bridge example's SimulatedCanBus)
// or the TWAI peripheral. SDO calls block (the bus delivers the responses from
// its own receive task), so every bus transaction runs on the app's drive
// task: the widget handlers only queue commands, and the drive task polls the
// statusword / position / velocity and updates the widgets from there (any
// task may call the mutators).

#include <array>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "sdkconfig.h"

#include "canopen_client.hpp"
#include "desktop.hpp"
#include "ds402.hpp"
#include "task.hpp"
#if CONFIG_DESKTOP_EXAMPLE_CANOPEN_BUS_SIMULATED
#include "simulated_can_bus.hpp"
#else
#include "twai.hpp"
#endif

namespace desktop_example {

#if CONFIG_DESKTOP_EXAMPLE_CANOPEN_BUS_SIMULATED
using CanBus = can_bridge::SimulatedCanBus;
#else
using CanBus = espp::Twai;
#endif

/// One CANopen window's bus, client, drive and drive task. Rebuilt in place
/// (on the drive task) when the node id changes; shut down from the window's
/// on_close, which runs on the desktop task with no lock held.
struct CanopenSession {
  using D = espp::Desktop;
  using CanFrame = espp::CanopenClient::CanFrame;
  using NmtState = espp::CanopenClient::NmtState;

  explicit CanopenSession(D &desktop)
      : desktop(desktop) {}
  ~CanopenSession() { shutdown(); }

  D &desktop;
  D::Widget nmt_label, bus_label, state_label, status_label, mode_label, pos_label, vel_label,
      sdo_result;
  uint8_t node_id{1};
  // destroyed in reverse order: task (stopped first), then the bus (its
  // receive task feeds the client), then the drive (references the client)
  std::unique_ptr<espp::CanopenClient> client;
  std::unique_ptr<espp::Ds402Drive> drive;
  std::unique_ptr<CanBus> bus;
  std::mutex mutex;
  std::condition_variable cv;
  std::deque<std::function<void()>> queue; // bounded by kMaxQueued (see run())
  bool stopping{false};
  bool running{false}; // the drive task consumes `queue` (guarded by `mutex`)
  std::unique_ptr<espp::Task> task;

  /// One unsigned number taking the whole field (blanks around it allowed;
  /// `base` 0 = "0x.." hex or decimal); nullopt when the field is empty,
  /// malformed (trailing junk, a sign), overflows or exceeds `max`, so a bad
  /// entry is never narrowed into a different object or value.
  static std::optional<uint32_t> parse_field(const std::string &text, int base, uint32_t max) {
    const char *p = text.c_str();
    while (*p == ' ')
      ++p;
    if (!*p || *p == '-' || *p == '+')
      return std::nullopt;
    char *end = nullptr;
    errno = 0;
    const unsigned long v = std::strtoul(p, &end, base);
    if (end == p || errno == ERANGE)
      return std::nullopt;
    while (*end == ' ')
      ++end;
    if (*end || v > max)
      return std::nullopt;
    return static_cast<uint32_t>(v);
  }

  static const char *nmt_name(NmtState s) {
    switch (s) {
    case NmtState::BootUp:
      return "boot-up";
    case NmtState::Stopped:
      return "stopped";
    case NmtState::Operational:
      return "operational";
    case NmtState::PreOperational:
      return "pre-operational";
    default:
      return "unknown";
    }
  }

  /// (Re)create the bus, client and drive for `id`. Called at launch (before
  /// the drive task runs) and from the drive task afterwards.
  bool open(uint8_t id, std::error_code &ec) {
    bus.reset(); // first: nothing feeds the client while it is replaced
    drive.reset();
    client.reset();
    node_id = id;
    client = std::make_unique<espp::CanopenClient>(espp::CanopenClient::Config{
        .node_id = id,
        .send =
            [this](const CanFrame &f) {
              if (!bus)
                return false;
              CanBus::Message m{
                  .id = f.id, .extended = f.extended, .rtr = f.rtr, .dlc = f.dlc, .data = f.data};
              std::error_code tx_ec;
              return bus->transmit(m, tx_ec);
            },
        .on_heartbeat =
            [this](uint8_t node, NmtState s) { // bus task
              if (node == node_id)
                nmt_label.set_text("NMT state: {}", nmt_name(s));
            },
    });
    drive = std::make_unique<espp::Ds402Drive>(*client);
    CanBus::Config cfg;
#if CONFIG_DESKTOP_EXAMPLE_CANOPEN_BUS_SIMULATED
    cfg.node_id = id;
#else
    cfg.tx_gpio = CONFIG_DESKTOP_EXAMPLE_CAN_TX_GPIO;
    cfg.rx_gpio = CONFIG_DESKTOP_EXAMPLE_CAN_RX_GPIO;
    cfg.baudrate = CONFIG_DESKTOP_EXAMPLE_CAN_BAUDRATE;
#endif
    cfg.on_receive = [this](const CanBus::Message &m) { // bus receive task, never SDO here
      if (client)
        client->process_frame(
            {.id = m.id, .extended = m.extended, .rtr = m.rtr, .dlc = m.dlc, .data = m.data});
    };
    bus = std::make_unique<CanBus>(cfg);
    if (!bus->initialize(ec)) {
      bus.reset();
      return false;
    }
    nmt_label.set_text("NMT state: (nothing heard from node {} yet)", id);
    return true;
  }

  /// Queue a bus transaction for the drive task; refused (with a toast) when
  /// the task is not running or too many commands are already waiting.
  static constexpr size_t kMaxQueued = 64;
  bool run(std::function<void()> fn) {
    const char *refused = nullptr;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!running)
        refused = "the drive task is not running";
      else if (queue.size() >= kMaxQueued)
        refused = "too many commands pending; wait for the bus";
      else
        queue.push_back(std::move(fn));
    }
    if (refused) {
      desktop.notify({.title = "CANopen", .text = refused, .level = D::NotifyLevel::Error});
      return false;
    }
    cv.notify_all();
    return true;
  }

  /// `what` failed with `ec`: a toast (level error). `sdo` says the failing
  /// call was an SDO transaction, the only case in which the client's
  /// last_abort_code() describes THIS failure (NMT sends and bus init never
  /// touch it, so appending it there would show a stale, unrelated abort).
  void fail(std::string_view what, const std::error_code &ec, bool sdo = false) {
    std::string text = fmt::format("{}: {}", what, ec.message());
    if (sdo && client && client->last_abort_code())
      text += fmt::format(" (SDO abort 0x{:08X}: {})", client->last_abort_code(),
                          espp::CanopenClient::abort_code_to_string(client->last_abort_code()));
    desktop.notify({.title = "CANopen", .text = text, .level = D::NotifyLevel::Error});
  }

  /// Start the drive task; false (with a toast) when it could not be started,
  /// in which case run() refuses every command.
  bool start() {
    task = std::make_unique<espp::Task>(
        espp::Task::Config{.callback = [this]() { return step(); },
                           .task_config = {.name = "canopen_ui", .stack_size_bytes = 6 * 1024}});
    const bool started = task->start();
    {
      std::lock_guard<std::mutex> lock(mutex);
      running = started;
    }
    if (!started) {
      task.reset();
      desktop.notify({.title = "CANopen",
                      .text = "could not start the drive task; the window is inert",
                      .level = D::NotifyLevel::Error});
    }
    return started;
  }

  void shutdown() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      stopping = true;
      running = false;
      queue.clear(); // queued commands hold a reference to this session
    }
    cv.notify_all();
    task.reset(); // joins: the step returns within one poll period
    bus.reset();
    drive.reset();
    client.reset();
  }

  /// One drive-task iteration: a queued command, or (every 250 ms) a poll.
  bool step() {
    using namespace std::chrono_literals;
    std::function<void()> fn;
    {
      std::unique_lock<std::mutex> lock(mutex);
      cv.wait_for(lock, 250ms, [this] { return stopping || !queue.empty(); });
      if (stopping)
        return true;
      if (!queue.empty()) {
        fn = std::move(queue.front());
        queue.pop_front();
      }
    }
    if (fn)
      fn();
    else
      poll();
    return false;
  }

  void poll() {
    if (!bus || !drive)
      return;
    std::error_code ec;
    const uint16_t sw = drive->get_statusword(ec);
    if (ec) {
      state_label.set_text("State: no response from node {} ({})", node_id, ec.message());
      return;
    }
    state_label.set_text("State: {}",
                         espp::Ds402Drive::to_string(espp::Ds402Drive::state_from_statusword(sw)));
    status_label.set_text("Statusword: 0x{:04X}", sw);
    const int8_t mode = drive->get_mode_display(ec);
    if (!ec)
      mode_label.set_text("Mode (0x6061): {}", mode);
    const int32_t pos = drive->get_position_actual(ec);
    if (!ec)
      pos_label.set_text("Position: {}", pos);
    const int32_t vel = drive->get_velocity_actual(ec);
    if (!ec)
      vel_label.set_text("Velocity: {}", vel);
  }
};

} // namespace desktop_example

inline void register_canopen_app(espp::Desktop &desktop) {
  desktop.register_app({
      .name = "CANopen",
      .icon = "\xF0\x9F\x94\x8C", // electric plug
      .description = "CiA 301 NMT / SDO master and a CiA 402 drive panel",
      .launch =
          [](espp::Desktop &d, espp::Desktop::AppId app) {
            using D = espp::Desktop;
            using Mode = espp::Ds402Drive::OperatingMode;
            auto st = std::make_shared<desktop_example::CanopenSession>(d);
            // on_close owns the blocking teardown (it runs on the desktop task
            // with no lock held, after the widgets -- and their handler copies
            // of `st` -- are gone)
            auto win = d.create_window(
                {.title = "CANopen / DS402", .app = app, .w = 560, .h = 0, .on_close = [st]() {
                   st->shutdown();
                 }});
            using S = desktop_example::CanopenSession;

            // ---- bus / node ----
            auto top = win.row();
            win.label("Node id", top.id());
            auto node_box = win.textbox(fmt::format("{}", CONFIG_DESKTOP_EXAMPLE_CANOPEN_NODE_ID),
                                        nullptr, top.id(), "1-127");
            node_box.set_size(60, 0);
            win.button(
                "Apply",
                [=]() mutable {
                  const auto parsed = S::parse_field(node_box.text(), 10, 127);
                  if (!parsed || *parsed < 1) {
                    st->desktop.notify({.title = "CANopen",
                                        .text = "node id must be a decimal number 1..127",
                                        .level = D::NotifyLevel::Error});
                    return;
                  }
                  const uint8_t id = static_cast<uint8_t>(*parsed);
                  st->run([=]() {
                    std::error_code ec;
                    if (!st->open(id, ec))
                      st->fail("bus init", ec);
                    else
                      st->desktop.notify({.title = "CANopen",
                                          .text = fmt::format("talking to node {}", id),
                                          .level = D::NotifyLevel::Ok});
                  });
                },
                top.id());
#if CONFIG_DESKTOP_EXAMPLE_CANOPEN_BUS_SIMULATED
            st->bus_label = win.label("simulated DS402 node", top.id());
#else
            st->bus_label = win.label(fmt::format("TWAI tx {} rx {} @ {} bit/s",
                                                  CONFIG_DESKTOP_EXAMPLE_CAN_TX_GPIO,
                                                  CONFIG_DESKTOP_EXAMPLE_CAN_RX_GPIO,
                                                  CONFIG_DESKTOP_EXAMPLE_CAN_BAUDRATE),
                                      top.id());
#endif

            // ---- NMT ----
            auto nmt = win.group("NMT");
            auto nmt_row = win.row(nmt.id());
            using NmtFn = bool (espp::CanopenClient::*)(std::error_code &);
            auto nmt_button = [=](std::string_view text, NmtFn fn) mutable {
              win.button(
                  text,
                  [=]() {
                    st->run([=]() {
                      std::error_code ec;
                      if (!st->client || !((*st->client).*fn)(ec))
                        st->fail(fmt::format("NMT {}", text), ec);
                    });
                  },
                  nmt_row.id());
            };
            nmt_button("Start", &espp::CanopenClient::nmt_start);
            nmt_button("Stop", &espp::CanopenClient::nmt_stop);
            nmt_button("Pre-operational", &espp::CanopenClient::nmt_pre_operational);
            nmt_button("Reset node", &espp::CanopenClient::nmt_reset_node);
            st->nmt_label = win.label("NMT state: -", nmt.id(), D::kLabelMonospace);

            // ---- drive ----
            auto drv = win.group("Drive (CiA 402)");
            st->state_label = win.label("State: -", drv.id(), D::kLabelBold);
            auto info = win.row(drv.id());
            st->status_label = win.label("Statusword: -", info.id(), D::kLabelMonospace);
            st->mode_label = win.label("Mode (0x6061): -", info.id(), D::kLabelMonospace);
            auto mode_row = win.row(drv.id());
            win.label("Mode", mode_row.id());
            const std::vector<Mode> modes = {Mode::ProfilePosition, Mode::ProfileVelocity,
                                             Mode::ProfileTorque, Mode::Homing};
            win.select(
                // no selection until the user picks one: 0x6060 is only
                // written on a change (the label shows the drive's 0x6061)
                {"Profile position", "Profile velocity", "Profile torque", "Homing"},
                D::kNoSelection,
                [=](int32_t i) {
                  if (i < 0 || static_cast<size_t>(i) >= modes.size())
                    return;
                  const Mode m = modes[static_cast<size_t>(i)];
                  st->run([=]() {
                    std::error_code ec;
                    if (!st->drive || !st->drive->set_mode(m, ec))
                      st->fail("set mode", ec, /*sdo=*/true);
                  });
                },
                mode_row.id());
            auto ctl = win.row(drv.id());
            using DriveFn = bool (espp::Ds402Drive::*)(std::error_code &);
            auto drive_button = [=](std::string_view text, DriveFn fn, uint16_t flags = 0) mutable {
              win.button(
                  text,
                  [=]() {
                    st->run([=]() {
                      std::error_code ec;
                      if (!st->drive || !((*st->drive).*fn)(ec))
                        st->fail(text, ec, /*sdo=*/true); // controlword / statusword SDOs
                      else
                        st->desktop.notify({.title = "CANopen",
                                            .text = fmt::format("{}: ok", text),
                                            .level = D::NotifyLevel::Ok,
                                            .timeout = std::chrono::milliseconds(1500)});
                    });
                  },
                  ctl.id(), flags);
            };
            drive_button("Enable", &espp::Ds402Drive::enable_operation, D::kButtonPrimary);
            drive_button("Disable", &espp::Ds402Drive::disable);
            drive_button("Quick stop", &espp::Ds402Drive::quick_stop, D::kButtonDanger);
            drive_button("Fault reset", &espp::Ds402Drive::fault_reset);
#if CONFIG_DESKTOP_EXAMPLE_CANOPEN_BUS_SIMULATED
            win.button(
                "Inject fault",
                [=]() {
                  st->run([=]() { // manufacturer object 0x2000 of the simulated node
                    std::error_code ec;
                    if (!st->client || !st->client->write_u8(0x2000, 0, 1, ec))
                      st->fail("inject fault", ec, /*sdo=*/true);
                  });
                },
                ctl.id());
#endif

            // ---- motion ----
            auto motion = win.group("Motion");
            auto vel_row = win.row(motion.id());
            win.label("Target velocity", vel_row.id());
            auto vel_value = win.label("0", vel_row.id(), D::kLabelMonospace);
            auto slider = win.slider(
                0, -1000, 1000, [=](int32_t v) mutable { vel_value.set_text("{}", v); },
                vel_row.id(), 10);
            win.button(
                "Apply",
                [=]() {
                  const int32_t v = slider.value();
                  st->run([=]() {
                    std::error_code ec;
                    if (!st->drive || !st->drive->set_target_velocity(v, ec))
                      st->fail("target velocity", ec, /*sdo=*/true);
                  });
                },
                vel_row.id(), D::kButtonPrimary);
            auto actual = win.row(motion.id());
            st->pos_label = win.label("Position: -", actual.id(), D::kLabelMonospace);
            st->vel_label = win.label("Velocity: -", actual.id(), D::kLabelMonospace);

            // ---- SDO ----
            auto sdo = win.group("SDO");
            auto sdo_row = win.row(sdo.id());
            auto idx_box = win.textbox("6041", nullptr, sdo_row.id(), "index (hex)");
            idx_box.set_size(70, 0);
            auto sub_box = win.textbox("0", nullptr, sdo_row.id(), "sub (hex)");
            sub_box.set_size(50, 0);
            auto width = win.select({"u8", "u16", "u32"}, 1, nullptr, sdo_row.id());
            auto val_box = win.textbox("0", nullptr, sdo_row.id(), "value (0x.. or decimal)");
            val_box.set_size(110, 0);
            st->sdo_result = win.label("", sdo.id(), D::kLabelMonospace | D::kLabelWrap);
            // index / sub-index from the text boxes, or nullopt (with the
            // reason in the result label) when either field is not a whole
            // hex number within its width
            auto object = [=]() mutable -> std::optional<std::pair<uint16_t, uint8_t>> {
              const auto index = S::parse_field(idx_box.text(), 16, 0xFFFF);
              const auto sub = S::parse_field(sub_box.text(), 16, 0xFF);
              if (!index || !sub) {
                st->sdo_result.set_text(!index ? "index must be a hex value 0000..FFFF"
                                               : "sub-index must be a hex value 00..FF");
                return std::nullopt;
              }
              return std::pair<uint16_t, uint8_t>{static_cast<uint16_t>(*index),
                                                  static_cast<uint8_t>(*sub)};
            };
            win.button(
                "Read",
                [=]() mutable {
                  const auto obj = object();
                  if (!obj)
                    return;
                  const auto [index, sub] = *obj;
                  st->run([=]() {
                    std::error_code ec;
                    std::array<uint8_t, 4> buf{};
                    const size_t n = st->client ? st->client->sdo_upload(index, sub, buf, ec) : 0;
                    if (!n) {
                      st->sdo_result.set_text("read 0x{:04X}:{:02X} failed: {}", index, sub,
                                              ec.message());
                      st->fail("SDO read", ec, /*sdo=*/true);
                      return;
                    }
                    const uint32_t v = espp::detail::canopen::get_le(buf.data(), n);
                    st->sdo_result.set_text("0x{:04X}:{:02X} = 0x{:0{}X} ({}) [{} bytes]", index,
                                            sub, v, n * 2, v, n);
                  });
                },
                sdo_row.id());
            win.button(
                "Write",
                [=]() mutable {
                  const auto obj = object();
                  if (!obj)
                    return;
                  const auto [index, sub] = *obj;
                  const int32_t w = width.selected();
                  const uint32_t max = w == 0 ? 0xFF : w == 1 ? 0xFFFF : 0xFFFFFFFFu;
                  const auto parsed = S::parse_field(val_box.text(), 0, max);
                  if (!parsed) {
                    st->sdo_result.set_text("value must be a decimal or 0x.. number within {}",
                                            w == 0   ? "u8 (0..255)"
                                            : w == 1 ? "u16 (0..65535)"
                                                     : "u32 (0..4294967295)");
                    return;
                  }
                  const uint32_t value = *parsed;
                  st->run([=]() {
                    std::error_code ec;
                    bool ok = false;
                    if (st->client) {
                      if (w == 0)
                        ok = st->client->write_u8(index, sub, static_cast<uint8_t>(value), ec);
                      else if (w == 1)
                        ok = st->client->write_u16(index, sub, static_cast<uint16_t>(value), ec);
                      else
                        ok = st->client->write_u32(index, sub, value, ec);
                    }
                    if (ok)
                      st->sdo_result.set_text("0x{:04X}:{:02X} <- 0x{:X} ok", index, sub, value);
                    else {
                      st->sdo_result.set_text("write 0x{:04X}:{:02X} failed: {}", index, sub,
                                              ec.message());
                      st->fail("SDO write", ec, /*sdo=*/true);
                    }
                  });
                },
                sdo_row.id());

            // bring the bus up (quick), then start polling
            std::error_code ec;
            if (!st->open(CONFIG_DESKTOP_EXAMPLE_CANOPEN_NODE_ID, ec)) {
              st->state_label.set_text("State: bus init failed ({}); fix the wiring / Kconfig "
                                       "and press Apply",
                                       ec.message());
              st->fail("bus init", ec);
            }
            if (!st->start())
              st->state_label.set_text("State: the drive task could not be started");
          },
  });
}

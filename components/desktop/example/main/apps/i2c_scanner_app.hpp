#pragma once

// I2C scanner: probe every 7-bit address on the Kconfig bus (espp::I2c) from
// a short task and list what answers; read / write device registers from the
// window. A bus that fails to initialize shows a hint instead of the tools.

#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "sdkconfig.h"

#include "desktop.hpp"
#include "i2c.hpp"
#include "task.hpp"

namespace desktop_example {

struct I2cSession {
  std::unique_ptr<espp::I2c> bus;
  std::mutex mutex;
  std::vector<uint8_t> found;       // addresses in table order (scan task writes, handlers read)
  std::unique_ptr<espp::Task> scan; // the one-shot worker (scan / read / write); joined before
                                    // the bus goes away

  /// Parse a space / comma separated list of hex bytes ("01 ff, 0x10") into
  /// a byte list; nullopt when any token is malformed or above 0xFF (the
  /// whole field is rejected, nothing is truncated). An empty field is an
  /// empty list.
  static std::optional<std::vector<uint8_t>> parse_bytes(const std::string &text) {
    std::vector<uint8_t> out;
    const char *p = text.c_str();
    while (*p) {
      while (*p == ' ' || *p == ',')
        ++p;
      if (!*p)
        break;
      char *end = nullptr;
      const unsigned long v = std::strtoul(p, &end, 16);
      if (end == p || v > 0xFF || (*end && *end != ' ' && *end != ','))
        return std::nullopt;
      out.push_back(static_cast<uint8_t>(v));
      p = end;
    }
    return out;
  }
  /// One unsigned number in `base` taking the whole field (blanks around it
  /// allowed); nullopt when the field is empty, malformed (trailing junk, a
  /// sign) or outside `min`..`max`, so a bad entry never becomes a real
  /// target (an empty address masked to 0x00 would be the general-call
  /// broadcast) or a wrong transfer length.
  static std::optional<unsigned long> parse_field(const std::string &text, int base,
                                                  unsigned long min, unsigned long max) {
    const char *p = text.c_str();
    while (*p == ' ')
      ++p;
    if (!*p || *p == '-' || *p == '+')
      return std::nullopt;
    char *end = nullptr;
    const unsigned long v = std::strtoul(p, &end, base);
    if (end == p)
      return std::nullopt;
    while (*end == ' ')
      ++end;
    if (*end || v < min || v > max)
      return std::nullopt;
    return v;
  }
  /// A 7-bit device address in the addressable range 0x01..0x7F (hex).
  static std::optional<uint8_t> parse_addr(const std::string &text) {
    const auto v = parse_field(text, 16, 0x01, 0x7F);
    if (!v)
      return std::nullopt;
    return static_cast<uint8_t>(*v);
  }
  /// A full 8-bit register value (0x00..0xFF, hex).
  static std::optional<uint8_t> parse_byte(const std::string &text) {
    const auto v = parse_field(text, 16, 0x00, 0xFF);
    if (!v)
      return std::nullopt;
    return static_cast<uint8_t>(*v);
  }
  static std::string hex_dump(const std::vector<uint8_t> &bytes) {
    std::string s;
    for (const auto b : bytes)
      s += fmt::format("{:02X} ", b);
    return s;
  }
  static const char *note(uint8_t addr) {
    if (addr < 0x08 || addr > 0x77)
      return "reserved range";
    return "";
  }
};

} // namespace desktop_example

inline void register_i2c_scanner_app(espp::Desktop &desktop) {
  desktop.register_app({
      .name = "I2C",
      .icon = "\xF0\x9F\x94\x8D", // magnifying glass
      .description = fmt::format(
          "Scan and poke the I2C bus (port {}, SDA {}, SCL {})", CONFIG_DESKTOP_EXAMPLE_I2C_PORT,
          CONFIG_DESKTOP_EXAMPLE_I2C_SDA_GPIO, CONFIG_DESKTOP_EXAMPLE_I2C_SCL_GPIO),
      .launch =
          [](espp::Desktop &d, espp::Desktop::AppId app) {
            using D = espp::Desktop;
            using S = desktop_example::I2cSession;
            auto st = std::make_shared<S>();
            auto win = d.create_window(
                {.title = "I2C scanner", .app = app, .w = 520, .h = 420, .on_close = [st]() {
                   st->scan.reset(); // joins a scan in flight
                   st->bus.reset();
                 }});
            win.label(fmt::format("Port {}, SDA GPIO {}, SCL GPIO {}, {} kHz",
                                  CONFIG_DESKTOP_EXAMPLE_I2C_PORT,
                                  CONFIG_DESKTOP_EXAMPLE_I2C_SDA_GPIO,
                                  CONFIG_DESKTOP_EXAMPLE_I2C_SCL_GPIO,
                                  CONFIG_DESKTOP_EXAMPLE_I2C_FREQ_HZ / 1000),
                      0, D::kLabelMonospace);
            st->bus = std::make_unique<espp::I2c>(espp::I2c::Config{
                .port = static_cast<i2c_port_t>(CONFIG_DESKTOP_EXAMPLE_I2C_PORT),
                .sda_io_num = static_cast<gpio_num_t>(CONFIG_DESKTOP_EXAMPLE_I2C_SDA_GPIO),
                .scl_io_num = static_cast<gpio_num_t>(CONFIG_DESKTOP_EXAMPLE_I2C_SCL_GPIO),
                .sda_pullup_en = GPIO_PULLUP_ENABLE,
                .scl_pullup_en = GPIO_PULLUP_ENABLE,
                .clk_speed = CONFIG_DESKTOP_EXAMPLE_I2C_FREQ_HZ,
                .auto_init = false,
            });
            std::error_code ec;
            st->bus->init(ec);
            if (ec) {
              win.label(fmt::format("The I2C bus failed to initialize: {}. Check the GPIOs in "
                                    "menuconfig (Desktop Example Configuration).",
                                    ec.message()),
                        0, D::kLabelWrap);
              st->bus.reset();
              return;
            }

            // ---- widgets ----
            auto bar = win.row();
            auto status = win.label("Press Scan to probe addresses 0x01..0x7F.", bar.id());
            auto table = win.table({"Address", "Decimal", "Note"}, {}, nullptr);
            auto scan_btn = win.button("Scan", nullptr, bar.id(), D::kButtonPrimary);
            auto tools = win.group("Register access");
            auto fields = win.row(tools.id());
            auto addr_box = win.textbox("", nullptr, fields.id(), "addr (hex)");
            addr_box.set_size(80, 0);
            auto reg_box = win.textbox("0", nullptr, fields.id(), "register (hex)");
            reg_box.set_size(90, 0);
            auto len_box = win.textbox("1", nullptr, fields.id(), "length");
            len_box.set_size(60, 0);
            auto val_box = win.textbox("", nullptr, fields.id(), "bytes to write (hex, <= 64)");
            auto actions = win.row(tools.id());
            auto result = win.label("", tools.id(), D::kLabelMonospace | D::kLabelWrap);
            auto read_btn = win.button("Read", nullptr, actions.id(), D::kButtonPrimary);
            auto write_btn = win.button("Write", nullptr, actions.id());
            table.on_event([=](const D::WidgetEvent &e) mutable {
              if (e.kind != D::WidgetEventKind::Select || e.value < 0)
                return;
              std::lock_guard<std::mutex> lock(st->mutex);
              if (static_cast<size_t>(e.value) < st->found.size())
                addr_box.set_text("{:02X}", st->found[static_cast<size_t>(e.value)]);
            });

            // ---- the worker ----
            // Every bus transaction (a scan, a register read or write) runs on
            // one one-shot worker task, never on the desktop task (a scan is up
            // to ~1.3 s, a 64-byte write at 100 kHz ~6 ms); the three buttons
            // are disabled until it is done and the result is posted back
            // through the widget mutators. The bus outlives the task: on_close
            // joins it first. The bodies capture raw pointers (not `st`) so the
            // task stored in the session does not keep the session alive.
            auto set_busy = [=](bool busy) mutable {
              scan_btn.set_enabled(!busy);
              read_btn.set_enabled(!busy);
              write_btn.set_enabled(!busy);
            };
            auto run_worker = [=, &d](std::function<void()> body) mutable {
              if (st->scan && st->scan->is_running())
                return;
              st->scan.reset(); // the previous (finished) worker
              set_busy(true);
              st->scan = std::make_unique<espp::Task>(espp::Task::Config{
                  .callback =
                      [=]() mutable {
                        body();
                        set_busy(false);
                        return true; // one shot
                      },
                  .task_config = {.name = "i2c_worker", .stack_size_bytes = 4 * 1024}});
              if (!st->scan->start()) {
                // no task will ever re-enable the controls: restore the window
                st->scan.reset();
                set_busy(false);
                table.set_enabled(true);
                status.set_text("Could not start the I2C worker task.");
                d.notify({.title = "I2C",
                          .text = "could not start the I2C worker task (out of memory?)",
                          .level = D::NotifyLevel::Error});
              }
            };

            // ---- scan ----
            scan_btn.on_event([=](const D::WidgetEvent &e) mutable {
              if (e.kind != D::WidgetEventKind::Click)
                return;
              if (st->scan && st->scan->is_running())
                return;
              // the old rows and their address mapping go BEFORE the worker
              // starts (it publishes the new mapping before the new rows) and
              // the table stays disabled for the whole scan, so a selection
              // can never map an old row to a new address
              {
                std::lock_guard<std::mutex> lock(st->mutex);
                st->found.clear();
              }
              table.set_items({});
              table.set_selected(D::kNoSelection);
              table.set_enabled(false);
              status.set_text("Scanning\xE2\x80\xA6");
              auto *bus = st->bus.get();
              auto *session = st.get();
              run_worker([=]() mutable {
                std::vector<std::string> rows;
                std::vector<uint8_t> found;
                for (uint8_t addr = 0x01; addr <= 0x7F; ++addr) {
                  if (!bus->probe_device(addr))
                    continue;
                  found.push_back(addr);
                  rows.push_back(fmt::format("0x{:02X}\t{}\t{}", addr, addr, S::note(addr)));
                }
                {
                  std::lock_guard<std::mutex> lock(session->mutex);
                  session->found = found;
                }
                table.set_items(rows);
                table.set_enabled(true);
                status.set_text("{} device{} found", rows.size(), rows.size() == 1 ? "" : "s");
              });
            });

            // ---- register access (parsed on the desktop task, done on the worker) ----
            read_btn.on_event([=](const D::WidgetEvent &e) mutable {
              if (e.kind != D::WidgetEventKind::Click)
                return;
              const auto addr = S::parse_addr(addr_box.text());
              const auto reg = S::parse_byte(reg_box.text());
              if (!addr || !reg) {
                result.set_text(!addr ? "address must be a hex value in 0x01..0x7F"
                                      : "register must be a hex value in 0x00..0xFF");
                return;
              }
              const auto len = S::parse_field(len_box.text(), 10, 1, 64); // decimal
              if (!len) {
                result.set_text("length must be a decimal number 1..64");
                return;
              }
              auto *bus = st->bus.get();
              run_worker([=, a = *addr, r = *reg, n = static_cast<size_t>(*len)]() mutable {
                std::vector<uint8_t> data(n);
                if (bus->read_at_register(a, r, data.data(), data.size()))
                  result.set_text("0x{:02X} reg 0x{:02X}: {}", a, r, S::hex_dump(data));
                else
                  result.set_text("0x{:02X} reg 0x{:02X}: read failed (no ACK?)", a, r);
              });
            });
            write_btn.on_event([=](const D::WidgetEvent &e) mutable {
              if (e.kind != D::WidgetEventKind::Click)
                return;
              const auto addr = S::parse_addr(addr_box.text());
              const auto reg = S::parse_byte(reg_box.text());
              if (!addr || !reg) {
                result.set_text(!addr ? "address must be a hex value in 0x01..0x7F"
                                      : "register must be a hex value in 0x00..0xFF");
                return;
              }
              const auto bytes = S::parse_bytes(val_box.text());
              if (!bytes) {
                result.set_text("bytes must be hex values 00..FF, space / comma separated");
                return;
              }
              if (bytes->size() > 64) { // the same bound as a read
                result.set_text("at most 64 data bytes per write ({} given)", bytes->size());
                return;
              }
              std::vector<uint8_t> data{*reg};
              data.insert(data.end(), bytes->begin(), bytes->end());
              auto *bus = st->bus.get();
              run_worker([=, a = *addr]() mutable {
                if (bus->write(a, data.data(), data.size()))
                  result.set_text("0x{:02X} <- {}ok", a, S::hex_dump(data));
                else
                  result.set_text("0x{:02X} <- {}failed (no ACK?)", a, S::hex_dump(data));
              });
            });
          },
  });
}

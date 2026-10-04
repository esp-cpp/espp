#pragma once

// Settings: a small form kept in NVS (namespace "desktop"): device nickname,
// theme, accent color and the log capture tee. Theme / accent apply to the
// browser immediately through Desktop::set_theme / set_accent.

#include <algorithm>
#include <cstdlib>
#include <memory>
#include <string>

#include "sdkconfig.h"

#include "console_capture.hpp"
#include "desktop.hpp"
#include "nvs_handle_espp.hpp"

namespace desktop_example {

/// Apply the saved settings at boot (before the browser connects).
inline void apply_saved_settings(espp::Desktop &desktop) {
  std::error_code ec;
  espp::NvsHandle nvs("desktop", ec);
  if (ec)
    return;
  std::string nickname, theme;
  nvs.get("nickname", nickname, std::string(""), ec);
  nvs.get("theme", theme, std::string("auto"), ec);
  if (!nickname.empty())
    desktop.set_device_name(nickname);
  if (theme == "auto" || theme == "light" || theme == "dark")
    desktop.set_theme(theme);
  int32_t accent = -1;
  nvs.get("accent", accent, int32_t{-1}, ec);
  if (accent >= 0)
    desktop.set_accent(static_cast<uint32_t>(accent));
}

} // namespace desktop_example

inline void register_settings_app(espp::Desktop &desktop, std::string_view default_name) {
  desktop.register_app({
      .name = "Settings",
      .icon = "svg:settings",
      .description = "Nickname, theme, accent, log capture",
      .launch =
          [default_name = std::string(default_name)](espp::Desktop &d, espp::Desktop::AppId app) {
            using D = espp::Desktop;
            std::error_code ec;
            auto nvs = std::make_shared<espp::NvsHandle>("desktop", ec);
            std::string nickname, theme;
            int32_t accent = 0x3b82f6;
            nvs->get("nickname", nickname, std::string(""), ec);
            nvs->get("theme", theme, std::string("auto"), ec);
            nvs->get("accent", accent, int32_t{0x3b82f6}, ec);
            const std::vector<std::string> themes = {"auto", "light", "dark"};
            int32_t theme_index = 0;
            for (size_t i = 0; i < themes.size(); ++i)
              if (themes[i] == theme)
                theme_index = static_cast<int32_t>(i);

            auto win = d.create_window({.title = "Settings", .app = app, .w = 380, .h = 0});
            auto form = win.group("Appearance");
            auto field = [&](std::string_view key) {
              auto row = win.row(form.id());
              win.label(key, row.id()).set_size(90, 0);
              return row;
            };
            auto name_box = win.textbox(nickname, nullptr, field("Nickname").id(), default_name);
            // a Select index comes from the host: bounds-check it
            auto theme_at = [themes](int32_t i) {
              return (i >= 0 && static_cast<size_t>(i) < themes.size()) ? themes[i] : themes[0];
            };
            auto theme_sel = win.select(
                themes, theme_index, [=, &d](int32_t i) { d.set_theme(theme_at(i)); },
                field("Theme").id());
            auto accent_box =
                win.textbox(fmt::format("{:06x}", accent), nullptr, field("Accent").id(), "rrggbb");
            win.label("Accent takes effect on Save.", form.id());
            auto logs = win.group("Logging");
            win.checkbox(
                "Tee captured logs to the UART console", espp::ConsoleCapture::tee_to_console(),
                [](bool on) { espp::ConsoleCapture::set_tee_to_console(on); }, logs.id());
            auto bar = win.row();
            win.spacer(bar.id());
            win.button(
                "Save",
                [=, &d]() mutable {
                  std::error_code ec2;
                  const std::string name = name_box.text();
                  nvs->set("nickname", name, ec2);
                  nvs->set("theme", theme_at(theme_sel.selected()), ec2);
                  const int32_t rgb = static_cast<int32_t>(
                      std::strtoul(accent_box.text().c_str(), nullptr, 16) & 0xFFFFFF);
                  nvs->set("accent", rgb, ec2);
                  d.set_device_name(name.empty() ? default_name : name);
                  d.set_accent(static_cast<uint32_t>(rgb));
                  d.notify({.title = "Settings",
                            .text = ec2 ? "could not write NVS" : "saved",
                            .level = ec2 ? D::NotifyLevel::Error : D::NotifyLevel::Ok});
                },
                bar.id(), D::kButtonPrimary);
          },
  });
}

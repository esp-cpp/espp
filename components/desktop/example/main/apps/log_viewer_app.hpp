#pragma once

// Log Viewer: streams the captured console (espp::ConsoleCapture) into a
// read-only console TextArea, in payload-sized chunks every 200 ms.

#include <chrono>
#include <memory>

#include "sdkconfig.h"

#include "console_capture.hpp"
#include "desktop.hpp"

inline void register_log_viewer_app(espp::Desktop &desktop) {
  desktop.register_app({
      .name = "Log Viewer",
      .icon = "\xF0\x9F\x93\x9C", // scroll
      .description = "The device's console log, live",
      .launch =
          [](espp::Desktop &d, espp::Desktop::AppId app) {
            using namespace std::chrono_literals;
            using D = espp::Desktop;
            auto win = d.create_window({.title = "Log Viewer", .app = app, .w = 640, .h = 400});
            if (!espp::ConsoleCapture::installed()) {
              win.label("Log capture is not installed (CONFIG_DESKTOP_EXAMPLE_LOG_CAPTURE).", 0,
                        D::kLabelWrap);
              return;
            }
            auto bar = win.row();
            auto paused = std::make_shared<bool>(false);
            win.checkbox(
                "Pause", false, [=](bool on) { *paused = on; }, bar.id());
            auto status = win.label("", bar.id(), D::kLabelMonospace);
            auto view = win.textarea("", 0,
                                     D::kTextAreaReadOnly | D::kTextAreaMonospace |
                                         D::kTextAreaAutoScroll | D::kTextAreaAnsi,
                                     1, 1000);
            win.button(
                "Clear",
                [=]() mutable {
                  espp::ConsoleCapture::clear();
                  view.set_text("");
                },
                bar.id());
            // start from the oldest byte the ring still holds
            auto cursor = std::make_shared<uint64_t>(0);
            const size_t chunk = d.max_payload() - 64; // leaves room for the frame's fields
            auto pump = [=, &d]() mutable {
              if (*paused)
                return;
              std::string text;
              size_t dropped = 0;
              // at most a few chunks per tick so one busy logger cannot
              // starve the rest of the desktop
              for (int i = 0; i < 4; ++i) {
                text.clear();
                if (!espp::ConsoleCapture::read_since(cursor.get(), text, chunk, &dropped))
                  break;
                if (dropped)
                  view.append_text(fmt::format("\n[{} bytes of log dropped]\n", dropped));
                view.append_text(text);
              }
              status.set_text("{} bytes captured", espp::ConsoleCapture::total_bytes());
              (void)d;
            };
            pump();
            win.add_timer(200ms, pump);
          },
  });
}

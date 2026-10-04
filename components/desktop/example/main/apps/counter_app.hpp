#pragma once

// Counter: the smallest complete desktop app, and the API reference for the
// others -- one window, a label, two buttons, state kept in NVS.

#include <memory>

#include "desktop.hpp"
#include "nvs_handle_espp.hpp"

//! [counter_app]
inline void register_counter_app(espp::Desktop &desktop) {
  desktop.register_app({
      .name = "Counter",
      .icon = "\xF0\x9F\xA7\xAE", // abacus
      .description = "Counts clicks (kept in NVS)",
      // runs on the desktop task when the user launches the app
      .launch =
          [](espp::Desktop &d, espp::Desktop::AppId app) {
            std::error_code ec;
            auto nvs = std::make_shared<espp::NvsHandle>("desktop", ec);
            auto count = std::make_shared<int32_t>(0);
            nvs->get("count", *count, int32_t{0}, ec);

            auto win = d.create_window({.title = "Counter", .app = app, .w = 240, .h = 150});
            auto label = win.label(fmt::format("Count: {}", *count), 0, espp::Desktop::kLabelBold);
            auto row = win.row(); // a horizontal container for the buttons
            auto bump = [=](int32_t delta) mutable {
              *count += delta;
              nvs->set("count", *count, ec); // any task may mutate; this one is the desktop task
              label.set_text("Count: {}", *count);
            };
            win.button(
                "-1", [=]() mutable { bump(-1); }, row.id());
            win.button(
                "+1", [=]() mutable { bump(+1); }, row.id(), espp::Desktop::kButtonPrimary);
            win.button(
                "Reset",
                [=, &d]() mutable {
                  // a message box owned by the window; the answer arrives later
                  d.message_box({.owner = win.id(),
                                 .title = "Reset counter?",
                                 .text = "The count is kept in NVS.",
                                 .buttons = {"Reset", "Cancel"},
                                 .icon = espp::Desktop::DialogIcon::Question,
                                 .on_result = [=](int button) mutable {
                                   if (button == 0)
                                     bump(-*count);
                                 }});
                },
                row.id(), espp::Desktop::kButtonDanger);
          },
  });
}
//! [counter_app]

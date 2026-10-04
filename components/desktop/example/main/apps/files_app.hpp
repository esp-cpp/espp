#pragma once

// Files: browse the LittleFS partition (espp::FileSystem), create / rename /
// delete through dialogs, and open a file in the Editor (a second window of
// the same app: a TextArea whose edits come back as Text events, saved with
// std::ofstream).

#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "desktop.hpp"
#include "file_system.hpp"

namespace desktop_example {

inline void open_editor(espp::Desktop &d, espp::Desktop::AppId app,
                        const std::filesystem::path &path) {
  using D = espp::Desktop;
  std::string contents;
  {
    std::ifstream f(path, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    contents = ss.str();
  }
  auto win =
      d.create_window({.title = fmt::format("Editor \xE2\x80\x94 {}", path.filename().string()),
                       .app = app,
                       .w = 560,
                       .h = 400});
  auto bar = win.row();
  auto status = win.label(fmt::format("{} bytes", contents.size()), bar.id(), D::kLabelMonospace);
  auto text = win.textarea(contents, 0, D::kTextAreaMonospace, 1, 0);
  // the browser sends the edited text (chunked) when the field loses focus or
  // on Ctrl+S; the model keeps the latest, so Save just writes text.text()
  text.on_event([=](const D::WidgetEvent &e) mutable {
    if (e.kind == D::WidgetEventKind::Text)
      status.set_text("{} bytes (unsaved)", e.text.size());
  });
  win.button(
      "Save",
      [=, &d]() mutable {
        const std::string body = text.text();
        std::ofstream f(path, std::ios::binary | std::ios::trunc);
        f << body;
        const bool ok = f.good();
        status.set_text("{} bytes{}", body.size(), ok ? "" : " (write FAILED)");
        d.notify({.title = path.filename().string(),
                  .text = ok ? fmt::format("saved {} bytes", body.size()) : "write failed",
                  .level = ok ? D::NotifyLevel::Ok : D::NotifyLevel::Error});
      },
      bar.id(), D::kButtonPrimary);
  win.button(
      "Reload",
      [=]() mutable {
        std::ifstream f(path, std::ios::binary);
        std::stringstream ss;
        ss << f.rdbuf();
        text.set_text(ss.str());
        status.set_text("{} bytes", ss.str().size());
      },
      bar.id());
}

} // namespace desktop_example

inline void register_files_app(espp::Desktop &desktop) {
  desktop.register_app({
      .name = "Files",
      .icon = "\xF0\x9F\x93\x81", // folder
      .description = "Browse and edit the LittleFS partition",
      .launch =
          [](espp::Desktop &d, espp::Desktop::AppId app) {
            using D = espp::Desktop;
            namespace fs = std::filesystem;
            auto &fsys = espp::FileSystem::get();
            auto win = d.create_window({.title = "Files", .app = app, .w = 480, .h = 360});
            auto bar = win.row();
            auto path_label = win.label("", bar.id(), D::kLabelMonospace);
            auto list = win.list({}, nullptr);
            auto info = win.label("", 0, D::kLabelMonospace);
            auto actions = win.row();
            struct State {
              fs::path cwd;
              std::vector<fs::path> entries; // in list order
              int32_t selected{-1};
            };
            auto st = std::make_shared<State>();
            st->cwd = espp::FileSystem::get_root_path();
            auto refresh = [=, &fsys]() mutable {
              st->entries.clear();
              std::vector<std::string> rows;
              std::error_code ec;
              for (const auto &e : fs::directory_iterator(st->cwd, ec)) {
                const bool dir = e.is_directory(ec);
                st->entries.push_back(e.path());
                rows.push_back(fmt::format("{} {}{}", dir ? "\xF0\x9F\x93\x81" : "\xF0\x9F\x93\x84",
                                           e.path().filename().string(), dir ? "/" : ""));
              }
              list.set_items(rows);
              list.set_selected(-1);
              st->selected = -1;
              path_label.set_text(st->cwd.string());
              info.set_text("{} entries, {} / {} KiB used", rows.size(),
                            fsys.get_used_space() / 1024, fsys.get_total_space() / 1024);
            };
            auto selected_path = [=]() -> std::optional<fs::path> {
              if (st->selected < 0 || static_cast<size_t>(st->selected) >= st->entries.size())
                return std::nullopt;
              return st->entries[static_cast<size_t>(st->selected)];
            };
            list.on_event([=, &d](const D::WidgetEvent &e) mutable {
              if (e.kind == D::WidgetEventKind::Select) {
                st->selected = e.value;
              } else if (e.kind == D::WidgetEventKind::Activate) {
                st->selected = e.value;
                const auto p = selected_path();
                if (!p)
                  return;
                std::error_code ec;
                if (fs::is_directory(*p, ec)) {
                  st->cwd = *p;
                  refresh();
                } else {
                  desktop_example::open_editor(d, app, *p);
                }
              }
            });
            win.button(
                "Up",
                [=]() mutable {
                  if (st->cwd != espp::FileSystem::get_root_path()) {
                    st->cwd = st->cwd.parent_path();
                    refresh();
                  }
                },
                bar.id());
            win.button(
                "Open",
                [=, &d]() mutable {
                  const auto p = selected_path();
                  if (!p)
                    return;
                  std::error_code ec;
                  if (fs::is_directory(*p, ec)) {
                    st->cwd = *p;
                    refresh();
                  } else {
                    desktop_example::open_editor(d, app, *p);
                  }
                },
                actions.id(), D::kButtonPrimary);
            win.button(
                "New file",
                [=, &d]() mutable {
                  d.input_box({.owner = win.id(),
                               .title = "New file",
                               .text = "File name:",
                               .default_text = "notes.txt",
                               .on_result = [=](std::optional<std::string> name) mutable {
                                 if (!name || name->empty())
                                   return;
                                 std::ofstream f(st->cwd / *name, std::ios::binary | std::ios::app);
                                 refresh();
                               }});
                },
                actions.id());
            win.button(
                "New folder",
                [=, &d]() mutable {
                  d.input_box({.owner = win.id(),
                               .title = "New folder",
                               .text = "Folder name:",
                               .on_result = [=](std::optional<std::string> name) mutable {
                                 if (!name || name->empty())
                                   return;
                                 std::error_code ec;
                                 fs::create_directory(st->cwd / *name, ec);
                                 refresh();
                               }});
                },
                actions.id());
            win.button(
                "Rename",
                [=, &d]() mutable {
                  const auto p = selected_path();
                  if (!p)
                    return;
                  d.input_box({.owner = win.id(),
                               .title = "Rename",
                               .text = "New name:",
                               .default_text = p->filename().string(),
                               .on_result = [=](std::optional<std::string> name) mutable {
                                 if (!name || name->empty())
                                   return;
                                 std::error_code ec;
                                 fs::rename(*p, p->parent_path() / *name, ec);
                                 refresh();
                               }});
                },
                actions.id());
            win.button(
                "Delete",
                [=, &d, &fsys]() mutable {
                  const auto p = selected_path();
                  if (!p)
                    return;
                  d.message_box({.owner = win.id(),
                                 .title = "Delete?",
                                 .text = fmt::format("Delete {}? This cannot be undone.",
                                                     p->filename().string()),
                                 .buttons = {"Delete", "Cancel"},
                                 .icon = D::DialogIcon::Warning,
                                 .on_result = [=, &fsys](int button) mutable {
                                   if (button != 0)
                                     return;
                                   std::error_code ec;
                                   fsys.remove(*p, ec);
                                   refresh();
                                 }});
                },
                actions.id(), D::kButtonDanger);
            refresh();
          },
      .single_instance = false, // the Editor windows belong to this app too
  });
}

#pragma once

// Files: browse the LittleFS partition (espp::FileSystem), create / rename /
// delete through dialogs, and open a file in the Editor (a second window of
// the same app: a TextArea whose edits come back as Text events, saved with
// std::ofstream).

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "desktop.hpp"
#include "file_system.hpp"

namespace desktop_example {

/// Names typed into a dialog come from the host: accept only a single path
/// component (no separators, not "." / "..", printable, short enough for
/// LittleFS) so it cannot escape the directory it is joined with.
inline bool valid_leaf_name(const std::string &name) {
  if (name.empty() || name.size() > 64 || name == "." || name == "..")
    return false;
  for (const unsigned char c : name)
    if (c == '/' || c == '\\' || c < 0x20 || c == 0x7F)
      return false;
  return true;
}

/// Read at most `limit` bytes of a file; `size` gets the whole file's size.
inline std::string read_head(const std::filesystem::path &path, size_t limit, size_t &size) {
  std::error_code ec;
  const auto n = std::filesystem::file_size(path, ec);
  size = ec ? 0 : static_cast<size_t>(n);
  std::string contents(std::min(size, limit), '\0');
  std::ifstream f(path, std::ios::binary);
  f.read(contents.data(), static_cast<std::streamsize>(contents.size()));
  contents.resize(static_cast<size_t>(std::max<std::streamsize>(f.gcount(), 0)));
  return contents;
}

inline void open_editor(espp::Desktop &d, espp::Desktop::AppId app,
                        const std::filesystem::path &path) {
  using D = espp::Desktop;
  // The desktop retains at most max_text_bytes of a TextArea (and the browser
  // mirrors that bound through the Text replacement it receives), so a larger
  // file could only be edited as a truncated tail -- and Save would then
  // overwrite the file with that tail. Open such a file read-only instead.
  const size_t limit = d.max_text_bytes();
  size_t file_size = 0;
  const std::string contents = read_head(path, limit, file_size);
  const bool too_big = file_size > limit;
  auto win =
      d.create_window({.title = fmt::format("Editor \xE2\x80\x94 {}", path.filename().string()),
                       .app = app,
                       .w = 560,
                       .h = 400});
  auto bar = win.row();
  auto status = win.label(fmt::format("{} bytes", file_size), bar.id(), D::kLabelMonospace);
  if (too_big) {
    win.label(fmt::format("Read-only: {} bytes is more than the editor holds ({} bytes); "
                          "showing the first {}.",
                          file_size, limit, contents.size()),
              0, D::kLabelWrap);
    win.textarea(contents, 0, D::kTextAreaMonospace | D::kTextAreaReadOnly, 1, 0);
    return;
  }
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
        size_t size = 0;
        const std::string body = read_head(path, limit, size);
        if (size > limit) {
          // it grew past the bound since it was opened: never offer a
          // truncated tail for saving
          text.set_flags(D::kTextAreaMonospace | D::kTextAreaReadOnly);
          status.set_text("{} bytes (now read-only: larger than the editor holds)", size);
          return;
        }
        text.set_text(body);
        status.set_text("{} bytes", size);
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
                               .on_result = [=, &d](std::optional<std::string> name) mutable {
                                 if (!name)
                                   return;
                                 if (!desktop_example::valid_leaf_name(*name)) {
                                   d.notify({.title = "New file",
                                             .text = "invalid name: one path component, no "
                                                     "slashes",
                                             .level = D::NotifyLevel::Error});
                                   return;
                                 }
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
                               .on_result = [=, &d](std::optional<std::string> name) mutable {
                                 if (!name)
                                   return;
                                 if (!desktop_example::valid_leaf_name(*name)) {
                                   d.notify({.title = "New folder",
                                             .text = "invalid name: one path component, no "
                                                     "slashes",
                                             .level = D::NotifyLevel::Error});
                                   return;
                                 }
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
                               .on_result = [=, &d](std::optional<std::string> name) mutable {
                                 if (!name)
                                   return;
                                 if (!desktop_example::valid_leaf_name(*name)) {
                                   d.notify({.title = "Rename",
                                             .text = "invalid name: one path component, no "
                                                     "slashes",
                                             .level = D::NotifyLevel::Error});
                                   return;
                                 }
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

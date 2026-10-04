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
  return std::none_of(name.begin(), name.end(), [](const unsigned char c) {
    return c == '/' || c == '\\' || c < 0x20 || c == 0x7F;
  });
}

/// Write (or create) a file and report whether EVERYTHING reached it: the
/// open must succeed and the stream must still be good after close() -- a
/// buffered write only hits the medium on the final flush, so good() before
/// close() would call a failed write "saved".
inline bool write_file(const std::filesystem::path &path, std::string_view body,
                       std::ios::openmode mode) {
  std::ofstream f(path, std::ios::binary | mode);
  if (!f.is_open())
    return false;
  f.write(body.data(), static_cast<std::streamsize>(body.size()));
  f.close();
  return !f.fail() && !f.bad();
}

/// The outcome of reading (the head of) a file: on failure `error` says why
/// and the editor must not offer the (partial / empty) contents for saving.
struct ReadResult {
  std::string contents{}; ///< at most `limit` bytes
  size_t size{0};         ///< the whole file's size
  std::string error{};    ///< empty = ok
  bool ok() const { return error.empty(); }
};

/// Read at most `limit` bytes of a file, checking every step (size, open,
/// the number of bytes actually read).
inline ReadResult read_head(const std::filesystem::path &path, size_t limit) {
  ReadResult r;
  std::error_code ec;
  const auto n = std::filesystem::file_size(path, ec);
  if (ec) {
    r.error = fmt::format("size: {}", ec.message());
    return r;
  }
  r.size = static_cast<size_t>(n);
  std::ifstream f(path, std::ios::binary);
  if (!f.is_open()) {
    r.error = "open failed";
    return r;
  }
  const size_t want = std::min(r.size, limit);
  r.contents.assign(want, '\0');
  f.read(r.contents.data(), static_cast<std::streamsize>(want));
  const size_t got = static_cast<size_t>(std::max<std::streamsize>(f.gcount(), 0));
  if (f.bad() || got != want) {
    r.error = fmt::format("read {} of {} bytes", got, want);
    r.contents.resize(got);
  }
  return r;
}

inline void open_editor(espp::Desktop &d, espp::Desktop::AppId app,
                        const std::filesystem::path &path) {
  using D = espp::Desktop;
  // The desktop retains at most max_text_bytes of a TextArea (and the browser
  // mirrors that bound through the Text replacement it receives), so a larger
  // file could only be edited as a truncated tail -- and Save would then
  // overwrite the file with that tail. Open such a file read-only instead,
  // and likewise one that could not be read completely.
  const size_t limit = d.max_text_bytes();
  const ReadResult r = read_head(path, limit);
  const bool too_big = r.size > limit;
  auto win =
      d.create_window({.title = fmt::format("Editor \xE2\x80\x94 {}", path.filename().string()),
                       .app = app,
                       .w = 560,
                       .h = 400});
  auto bar = win.row();
  auto status = win.label(fmt::format("{} bytes", r.size), bar.id(), D::kLabelMonospace);
  if (!r.ok() || too_big) {
    win.label(!r.ok() ? fmt::format("Read-only: read failed ({}); showing the {} bytes that "
                                    "were read.",
                                    r.error, r.contents.size())
                      : fmt::format("Read-only: {} bytes is more than the editor holds ({} "
                                    "bytes); showing the first {}.",
                                    r.size, limit, r.contents.size()),
              0, D::kLabelWrap);
    win.textarea(r.contents, 0, D::kTextAreaMonospace | D::kTextAreaReadOnly, 1, 0);
    return;
  }
  auto text = win.textarea(r.contents, 0, D::kTextAreaMonospace, 1, 0);
  // once a reload fails (or finds the file grown past the bound) the window
  // becomes read-only for good: Save / Reload must never write a partial view
  auto read_only = std::make_shared<bool>(false);
  auto make_read_only = [=](std::string why) mutable {
    *read_only = true;
    text.set_flags(D::kTextAreaMonospace | D::kTextAreaReadOnly);
    status.set_text("read-only: {}", why);
  };
  // the browser sends the edited text (chunked) when the field loses focus or
  // on Ctrl+S; the model keeps the latest, so Save just writes text.text()
  text.on_event([=](const D::WidgetEvent &e) mutable {
    if (e.kind == D::WidgetEventKind::Text && !*read_only)
      status.set_text("{} bytes (unsaved)", e.text.size());
  });
  win.button(
      "Save",
      [=, &d]() mutable {
        if (*read_only) {
          d.notify({.title = path.filename().string(),
                    .text = "read-only: not saved",
                    .level = D::NotifyLevel::Warn});
          return;
        }
        const std::string body = text.text();
        const bool ok = write_file(path, body, std::ios::trunc);
        status.set_text("{} bytes{}", body.size(), ok ? "" : " (write FAILED)");
        d.notify({.title = path.filename().string(),
                  .text = ok ? fmt::format("saved {} bytes", body.size()) : "write failed",
                  .level = ok ? D::NotifyLevel::Ok : D::NotifyLevel::Error});
      },
      bar.id(), D::kButtonPrimary);
  win.button(
      "Reload",
      [=]() mutable {
        if (*read_only)
          return;
        const ReadResult again = read_head(path, limit);
        if (!again.ok()) {
          make_read_only(fmt::format("read failed ({})", again.error));
          return;
        }
        if (again.size > limit) {
          // it grew past the bound since it was opened: never offer a
          // truncated tail for saving
          make_read_only(fmt::format("{} bytes is larger than the editor holds", again.size));
          return;
        }
        text.set_text(again.contents);
        status.set_text("{} bytes", again.size);
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
                  d.input_box(
                      {.owner = win.id(),
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
                         if (!desktop_example::write_file(st->cwd / *name, "", std::ios::app))
                           d.notify({.title = "New file",
                                     .text = fmt::format("could not create {}", *name),
                                     .level = D::NotifyLevel::Error});
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

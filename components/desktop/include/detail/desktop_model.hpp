#pragma once

// The retained desktop model behind espp::Desktop: apps, windows, widget
// trees, dialogs, plus the dirty tracker that coalesces changes between
// flushes into the fewest frames, and the chunked-Text reassembler. It is
// deliberately host-buildable (desktop_protocol.hpp + the standard library)
// so the coalescing rules are unit-tested on the host
// (components/desktop/test/desktop_host_test.cpp). It does no locking and no
// I/O: espp::Desktop wraps it in a mutex and a task.
//
// Coalescing rules (DirtyTracker), per window and in flush order
// WINDOW_CLOSE -> WINDOW_OPEN -> WIDGET_ADD -> WIDGET_SET -> WIDGET_REMOVE:
//   - a pending open discards every other change of that window (the full
//     tree is encoded at flush time); closing a never-flushed window sends
//     nothing;
//   - a close discards every other pending change of that window;
//   - a widget pending in WIDGET_ADD absorbs its sets (its full state is
//     encoded at flush time); removing it cancels the add;
//   - sets: last value wins per tag, TextAppend pieces concatenate, a Text
//     cancels earlier appends, Items ranges accumulate in order and an
//     ItemCount (or a full set_items, which starts with one) discards earlier
//     ranges; a remove cancels the widget's pending sets, and a set after a
//     remove is ignored;
//   - dialogs: a close before the flush cancels the open; notifications queue;
//   - DESKTOP is resent (last) when apps or the desktop records changed.

#include <algorithm>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "detail/desktop_protocol.hpp"

namespace espp::detail::desktop_model {

namespace dp = espp::detail::desktop_protocol;

/// Reassembles chunked WIDGET_EVENT Text values (offset / total / bytes) per
/// (sink, window, widget) -- two attached hosts never interleave into one
/// buffer -- bounded in size.
class TextAssembler {
public:
  enum class Result {
    Partial,  ///< more chunks expected
    Complete, ///< `out` holds the whole text
    Rejected, ///< out of order, over the bound or inconsistent: buffer dropped
  };

  explicit TextAssembler(size_t max_bytes)
      : max_bytes_(max_bytes) {}

  Result feed(uint32_t sink, uint16_t window, uint16_t widget, const dp::WidgetEvent &chunk,
              std::string &out) {
    const uint64_t key = key_of(sink, window, widget);
    if (chunk.text_total > max_bytes_ || chunk.text_offset > chunk.text_total ||
        chunk.text.size() > chunk.text_total - chunk.text_offset) {
      bufs_.erase(key);
      return Result::Rejected;
    }
    auto it = bufs_.find(key);
    if (chunk.text_offset == 0) {
      if (it != bufs_.end())
        bufs_.erase(it);
      if (chunk.text.size() == chunk.text_total) {
        out = chunk.text;
        return Result::Complete;
      }
      bufs_[key] = Buf{.total = chunk.text_total, .data = chunk.text};
      return Result::Partial;
    }
    if (it == bufs_.end() || it->second.total != chunk.text_total ||
        it->second.data.size() != chunk.text_offset) {
      if (it != bufs_.end())
        bufs_.erase(it);
      return Result::Rejected;
    }
    it->second.data += chunk.text;
    if (it->second.data.size() < it->second.total)
      return Result::Partial;
    out = std::move(it->second.data);
    bufs_.erase(it);
    return Result::Complete;
  }

  /// Drop the buffers of every widget of a window (it closed), on every sink.
  void forget_window(uint16_t window) {
    std::erase_if(bufs_, [window](const auto &kv) {
      return static_cast<uint16_t>((kv.first >> 16) & 0xFFFF) == window;
    });
  }

  /// Drop every partial assembly a sink had (it detached).
  void forget_sink(uint32_t sink) {
    const uint64_t lo = static_cast<uint64_t>(sink) << 32;
    bufs_.erase(bufs_.lower_bound(lo), bufs_.lower_bound(lo + (1ull << 32)));
  }

  void clear() { bufs_.clear(); }

  size_t pending() const { return bufs_.size(); }

private:
  static uint64_t key_of(uint32_t sink, uint16_t window, uint16_t widget) {
    return (static_cast<uint64_t>(sink) << 32) | (static_cast<uint64_t>(window) << 16) | widget;
  }
  struct Buf {
    uint32_t total{0};
    std::string data{};
  };
  size_t max_bytes_;
  std::map<uint64_t, Buf> bufs_{};
};

/// Pending changes of one window.
struct WindowDirt {
  bool open{false};  ///< send WINDOW_OPEN with the full tree
  bool close{false}; ///< send WINDOW_CLOSE
  dp::WindowCloseReason close_reason{dp::WindowCloseReason::App};
  std::vector<uint16_t> added{};          ///< WIDGET_ADD, in order
  std::vector<dp::WidgetSetEntry> sets{}; ///< WIDGET_SET, coalesced
  std::vector<uint16_t> removed{};        ///< WIDGET_REMOVE
};

/// Coalesces changes between flushes (see the rules at the top).
class DirtyTracker {
public:
  bool any() const {
    return !windows.empty() || !dialogs_opened.empty() || !dialogs_closed.empty() ||
           !notifications.empty() || desktop_changed;
  }

  /// The pending changes of a window, or nullptr when it has none.
  WindowDirt *window(uint16_t id) {
    for (auto &[wid, d] : windows)
      if (wid == id)
        return &d;
    return nullptr;
  }

  void open(uint16_t win) {
    WindowDirt &d = get(win);
    d = WindowDirt{};
    d.open = true;
  }

  void close(uint16_t win, dp::WindowCloseReason reason) {
    const WindowDirt *d = window(win);
    if (d && d->open) {
      // never reached the host: it vanishes silently
      erase(win);
      return;
    }
    WindowDirt &nd = get(win);
    nd = WindowDirt{};
    nd.close = true;
    nd.close_reason = reason;
  }

  void add(uint16_t win, uint16_t widget) {
    WindowDirt &d = get(win);
    if (d.open || d.close)
      return;
    d.added.push_back(widget);
  }

  /// Record a property change; returns the size of the coalesced text value
  /// for Text / TextAppend (so a caller can bound a growing append), else 0.
  size_t set(uint16_t win, uint16_t widget, dp::Prop prop) {
    WindowDirt &d = get(win);
    if (d.open || d.close)
      return 0;
    if (std::find(d.added.begin(), d.added.end(), widget) != d.added.end())
      return 0;
    if (std::find(d.removed.begin(), d.removed.end(), widget) != d.removed.end())
      return 0;
    dp::WidgetSetEntry *e = nullptr;
    for (auto &entry : d.sets)
      if (entry.widget == widget)
        e = &entry;
    if (!e) {
      d.sets.push_back({.widget = widget});
      e = &d.sets.back();
    }
    auto &props = e->props;
    auto erase_tags = [&](auto pred) { std::erase_if(props, pred); };
    if (prop.is(dp::PropTag::Text)) {
      erase_tags([](const dp::Prop &p) {
        return p.is(dp::PropTag::Text) || p.is(dp::PropTag::TextAppend);
      });
      props.push_back(std::move(prop));
      return props.back().value.size();
    }
    if (prop.is(dp::PropTag::TextAppend)) {
      auto it = std::find_if(props.begin(), props.end(),
                             [](const dp::Prop &p) { return p.is(dp::PropTag::TextAppend); });
      if (it != props.end()) {
        it->value.insert(it->value.end(), prop.value.begin(), prop.value.end());
        return it->value.size();
      }
      props.push_back(std::move(prop));
      return props.back().value.size();
    }
    if (prop.is(dp::PropTag::Items)) {
      props.push_back(std::move(prop));
      return 0;
    }
    if (prop.is(dp::PropTag::ItemCount)) {
      erase_tags([](const dp::Prop &p) {
        return p.is(dp::PropTag::Items) || p.is(dp::PropTag::ItemCount);
      });
      props.push_back(std::move(prop));
      return 0;
    }
    auto same = std::find_if(props.begin(), props.end(),
                             [&prop](const dp::Prop &p) { return p.tag == prop.tag; });
    if (same != props.end()) {
      same->value = std::move(prop.value);
      return 0;
    }
    props.push_back(std::move(prop));
    return 0;
  }

  /// @param on_wire false for a descendant removed through its ancestor (its
  ///        pending changes are cancelled but it is not listed: the host
  ///        removes children with the parent).
  void remove(uint16_t win, uint16_t widget, bool on_wire) {
    WindowDirt &d = get(win);
    if (d.open || d.close)
      return;
    std::erase_if(d.sets, [widget](const dp::WidgetSetEntry &e) { return e.widget == widget; });
    const auto it = std::find(d.added.begin(), d.added.end(), widget);
    if (it != d.added.end()) {
      d.added.erase(it); // never reached the host
      return;
    }
    if (on_wire)
      d.removed.push_back(widget);
  }

  void dialog_open(uint16_t id) { dialogs_opened.push_back(id); }

  void dialog_close(uint16_t id) {
    const auto it = std::find(dialogs_opened.begin(), dialogs_opened.end(), id);
    if (it != dialogs_opened.end()) {
      dialogs_opened.erase(it); // never reached the host
      return;
    }
    dialogs_closed.push_back(id);
  }

  void notify(dp::Notify n) { notifications.push_back(std::move(n)); }

  void clear() {
    windows.clear();
    dialogs_opened.clear();
    dialogs_closed.clear();
    notifications.clear();
    desktop_changed = false;
  }

  std::vector<std::pair<uint16_t, WindowDirt>> windows{}; ///< in first-dirtied order
  std::vector<uint16_t> dialogs_opened{};
  std::vector<uint16_t> dialogs_closed{};
  std::vector<dp::Notify> notifications{};
  bool desktop_changed{false};

private:
  WindowDirt &get(uint16_t win) {
    if (auto *d = window(win))
      return *d;
    windows.push_back({win, WindowDirt{}});
    return windows.back().second;
  }
  void erase(uint16_t win) {
    std::erase_if(windows, [win](const auto &p) { return p.first == win; });
  }
};

/// A widget's retained state (the full state is what WINDOW_OPEN / WIDGET_ADD
/// encode; the host mirrors it).
struct WidgetState {
  uint16_t id{0};
  uint16_t parent{0};
  dp::WidgetType type{dp::WidgetType::Label};
  uint8_t weight{0};
  uint8_t layout{0};
  std::string text{};
  int32_t value{0};
  int32_t min{0};
  int32_t max{100};
  int32_t step{1};
  bool enabled{true};
  bool visible{true};
  uint32_t color{0xFFFFFFFF};
  uint32_t background{0xFFFFFFFF};
  std::vector<std::string> items{};
  std::vector<std::string> columns{};
  std::string placeholder{};
  std::string tooltip{};
  uint16_t flags{0};
  uint16_t max_lines{500};
  uint16_t width{0};
  uint16_t height{0};
  uint16_t insert_before{0}; ///< until the WIDGET_ADD is flushed
  std::function<void(const dp::WidgetEvent &)> on_event{nullptr};

  bool has_value() const {
    using T = dp::WidgetType;
    return type == T::Checkbox || type == T::Slider || type == T::Progress || type == T::Select ||
           type == T::List || type == T::Table;
  }
  bool has_range() const {
    return type == dp::WidgetType::Slider || type == dp::WidgetType::Progress;
  }

  /// The widget as a wire record carrying its full (non-default) state.
  dp::WidgetRec to_rec(bool with_insert_before) const {
    using dp::Prop;
    using dp::PropTag;
    dp::WidgetRec r{.id = id, .parent = parent, .type = type, .weight = weight, .layout = layout};
    if (!text.empty())
      r.props.push_back(Prop::text(PropTag::Text, text));
    if (has_value())
      r.props.push_back(Prop::i32(PropTag::Value, value));
    if (has_range()) {
      r.props.push_back(Prop::i32(PropTag::Min, min));
      r.props.push_back(Prop::i32(PropTag::Max, max));
      r.props.push_back(Prop::i32(PropTag::Step, step));
    }
    if (!enabled)
      r.props.push_back(Prop::u8(PropTag::Enabled, 0));
    if (!visible)
      r.props.push_back(Prop::u8(PropTag::Visible, 0));
    if (color != 0xFFFFFFFF)
      r.props.push_back(Prop::u32(PropTag::Color, color));
    if (background != 0xFFFFFFFF)
      r.props.push_back(Prop::u32(PropTag::Background, background));
    if (!columns.empty())
      r.props.push_back(Prop::columns(columns));
    if (!items.empty()) {
      r.props.push_back(Prop::u16(PropTag::ItemCount, static_cast<uint16_t>(items.size())));
      r.props.push_back(Prop::items(0, items));
    }
    if (!placeholder.empty())
      r.props.push_back(Prop::text(PropTag::Placeholder, placeholder));
    if (!tooltip.empty())
      r.props.push_back(Prop::text(PropTag::Tooltip, tooltip));
    if (flags)
      r.props.push_back(Prop::u16(PropTag::Flags, flags));
    if (max_lines != 500)
      r.props.push_back(Prop::u16(PropTag::MaxLines, max_lines));
    if (width)
      r.props.push_back(Prop::u16(PropTag::Width, width));
    if (height)
      r.props.push_back(Prop::u16(PropTag::Height, height));
    if (with_insert_before && insert_before)
      r.props.push_back(Prop::u16(PropTag::InsertBefore, insert_before));
    return r;
  }
};

/// What an application gives to add a widget (every field optional but type).
struct WidgetConfig {
  dp::WidgetType type{dp::WidgetType::Label};
  uint16_t parent{0}; ///< container widget id (0 = the window's root column)
  std::string text{};
  int32_t value{0};
  int32_t min{0};
  int32_t max{100};
  int32_t step{1};
  std::vector<std::string> items{};
  std::vector<std::string> columns{};
  std::string placeholder{};
  std::string tooltip{};
  uint16_t flags{0};
  uint8_t weight{0};
  uint8_t layout{0};
  uint16_t width{0};
  uint16_t height{0};
  uint16_t max_lines{500};
  bool enabled{true};
  bool visible{true};
  uint16_t insert_before{0}; ///< sibling to insert in front of (0 = append)
  std::function<void(const dp::WidgetEvent &)> on_event{nullptr};
};

/// What an application gives to open a window.
struct WindowConfig {
  std::string title{};
  uint8_t app{0}; ///< the app it belongs to (listed by DESKTOP; 0 = none)
  int16_t x{-1};  ///< -1 = the browser decides (remembered per app + title, else cascade)
  int16_t y{-1};
  uint16_t w{0}; ///< 0 = size to content
  uint16_t h{0};
  uint16_t flags{dp::kWinDefaultFlags};
  std::function<void()> on_close{nullptr}; ///< after the window is gone (any reason)
  std::function<void(const dp::WindowEvent &)> on_event{nullptr};
};

/// A window's retained state.
struct WindowState {
  uint16_t id{0};
  uint8_t app{0};
  std::string title{};
  uint16_t flags{dp::kWinDefaultFlags};
  dp::Geometry geometry{};
  bool minimized{false};
  bool maximized{false};
  bool focused{false};
  std::vector<WidgetState> widgets{}; ///< parent before child, siblings in display order
  uint16_t next_widget{1};
  std::function<void()> on_close{nullptr};
  std::function<void(const dp::WindowEvent &)> on_event{nullptr};

  WidgetState *widget(uint16_t widget_id) {
    const auto it = std::find_if(widgets.begin(), widgets.end(),
                                 [widget_id](const WidgetState &w) { return w.id == widget_id; });
    return it != widgets.end() ? &*it : nullptr;
  }
  const WidgetState *widget(uint16_t widget_id) const {
    const auto it = std::find_if(widgets.begin(), widgets.end(),
                                 [widget_id](const WidgetState &w) { return w.id == widget_id; });
    return it != widgets.end() ? &*it : nullptr;
  }

  dp::WindowOpen to_open(bool snapshot) const {
    dp::WindowOpen o{.id = id,
                     .app = app,
                     .flags = static_cast<uint16_t>(snapshot ? (flags | dp::kWinSnapshot) : flags),
                     .geometry = geometry,
                     .title = title};
    for (const auto &w : widgets)
      o.widgets.push_back(w.to_rec(false));
    o.total = static_cast<uint16_t>(o.widgets.size());
    return o;
  }
};

/// An open dialog.
struct DialogState {
  dp::Dialog dialog{};
  std::function<void(const dp::DialogResult &)> on_result{nullptr};
  uint16_t owner() const { return dialog.owner; }
};

/// The retained model + dirty tracker: every mutator updates the state and
/// records the change; flush() turns the pending changes into messages.
class Model {
public:
  // ---- desktop-level settings (DESKTOP records) ----
  std::string device_name{};
  std::string firmware{};
  std::string theme{"auto"};
  uint32_t accent{0x3b82f6};
  uint16_t flush_period_ms{50};
  size_t max_payload{4081};
  size_t max_text_bytes{16 * 1024}; ///< bound on a TextArea's retained text / one pending append

  DirtyTracker dirty;

  // ---- apps ----

  /// Register (or replace, by id) an app. False (nothing changed) when the
  /// records plus the full app list -- descriptions emptied, no windows, the
  /// floor the encoder can never trim below -- would no longer fit
  /// max_payload: apps are never trimmed on the wire, so they must always fit.
  bool register_app(dp::AppRec app) {
    dp::DesktopInfo floor = desktop_info(false);
    floor.windows.clear();
    std::erase_if(floor.apps, [&app](const dp::AppRec &a) { return a.id == app.id; });
    floor.apps.push_back(app);
    for (auto &a : floor.apps)
      a.description.clear();
    if (dp::encode_desktop(floor).size() > max_payload)
      return false;
    auto it = std::find_if(apps_.begin(), apps_.end(),
                           [&app](const dp::AppRec &a) { return a.id == app.id; });
    if (it != apps_.end()) {
      *it = std::move(app);
      dirty.desktop_changed = true;
      return true;
    }
    apps_.push_back(std::move(app));
    std::sort(apps_.begin(), apps_.end(),
              [](const dp::AppRec &a, const dp::AppRec &b) { return a.id < b.id; });
    dirty.desktop_changed = true;
    return true;
  }

  bool unregister_app(uint8_t id) {
    const size_t before = apps_.size();
    std::erase_if(apps_, [id](const dp::AppRec &a) { return a.id == id; });
    if (apps_.size() == before)
      return false;
    dirty.desktop_changed = true;
    return true;
  }

  const dp::AppRec *app(uint8_t id) const {
    const auto it =
        std::find_if(apps_.begin(), apps_.end(), [id](const dp::AppRec &a) { return a.id == id; });
    return it != apps_.end() ? &*it : nullptr;
  }
  const std::vector<dp::AppRec> &apps() const { return apps_; }

  /// A free app id (0 = none left): monotonic, wrapping, never one that an
  /// app or a still-open window references.
  uint8_t allocate_app_id() {
    return next_id(next_app_,
                   [this](uint8_t id) { return app(id) != nullptr || !windows_of(id).empty(); });
  }

  void mark_desktop_changed() { dirty.desktop_changed = true; }

  dp::DesktopInfo desktop_info(bool has_snapshot) const {
    using dp::Prop;
    using T = dp::DesktopTag;
    dp::DesktopInfo d;
    d.flags = has_snapshot ? dp::kDesktopHasSnapshot : 0;
    d.records = {
        Prop::text(static_cast<dp::PropTag>(T::DeviceName), device_name),
        Prop::text(static_cast<dp::PropTag>(T::Firmware), firmware),
        Prop::text(static_cast<dp::PropTag>(T::Theme), theme),
        Prop::u32(static_cast<dp::PropTag>(T::Accent), accent),
        Prop::u16(static_cast<dp::PropTag>(T::MaxPayload), static_cast<uint16_t>(max_payload)),
        Prop::u16(static_cast<dp::PropTag>(T::FlushPeriodMs), flush_period_ms)};
    d.apps = apps_;
    for (const auto &w : windows_)
      d.windows.push_back({.id = w.id, .app = w.app});
    return d;
  }

  // ---- windows ----

  /// Returns 0 (nothing created) when the title is not representable (a
  /// str8 on the wire: at most kMaxStr8Bytes).
  uint16_t create_window(WindowConfig cfg) {
    if (cfg.title.size() > dp::kMaxStr8Bytes)
      return 0;
    WindowState w;
    w.id = next_id(next_window_, [this](uint16_t id) { return window(id) != nullptr; });
    w.app = cfg.app;
    w.title = std::move(cfg.title);
    w.flags = cfg.flags;
    w.geometry = {.x = cfg.x, .y = cfg.y, .w = cfg.w, .h = cfg.h};
    w.on_close = std::move(cfg.on_close);
    w.on_event = std::move(cfg.on_event);
    windows_.push_back(std::move(w));
    dirty.open(windows_.back().id);
    return windows_.back().id;
  }

  WindowState *window(uint16_t id) {
    const auto it = std::find_if(windows_.begin(), windows_.end(),
                                 [id](const WindowState &w) { return w.id == id; });
    return it != windows_.end() ? &*it : nullptr;
  }
  const WindowState *window(uint16_t id) const {
    const auto it = std::find_if(windows_.begin(), windows_.end(),
                                 [id](const WindowState &w) { return w.id == id; });
    return it != windows_.end() ? &*it : nullptr;
  }
  const std::vector<WindowState> &windows() const { return windows_; }

  /// Ids of the open windows of an app (0 = every window).
  std::vector<uint16_t> windows_of(uint8_t app_id) const {
    std::vector<uint16_t> ids{};
    for (const auto &w : windows_)
      if (app_id == 0 || w.app == app_id)
        ids.push_back(w.id);
    return ids;
  }

  /// Remove a window (and its dialogs) from the model; the WINDOW_CLOSE is
  /// sent at the next flush. Returns the window's on_close callback (for the
  /// caller to run) or nullptr when the id is unknown.
  std::optional<std::function<void()>> close_window(uint16_t id, dp::WindowCloseReason reason) {
    WindowState *w = window(id);
    if (!w)
      return std::nullopt;
    std::function<void()> on_close = std::move(w->on_close);
    std::erase_if(windows_, [id](const WindowState &x) { return x.id == id; });
    dirty.close(id, reason);
    for (const auto &d : dialogs_)
      if (d.owner() == id)
        dirty.dialog_close(d.dialog.id);
    std::erase_if(dialogs_, [id](const DialogState &d) { return d.owner() == id; });
    return on_close;
  }

  // ---- widgets ----

  WidgetState *widget(uint16_t win, uint16_t id) {
    WindowState *w = window(win);
    return w ? w->widget(id) : nullptr;
  }
  const WidgetState *widget(uint16_t win, uint16_t id) const {
    const WindowState *w = window(win);
    return w ? w->widget(id) : nullptr;
  }

  /// Add a widget; returns its id, or 0 when the window / parent / sibling is
  /// unknown, the parent is not a container, or a value is not representable
  /// (see representable()).
  uint16_t add_widget(uint16_t win, WidgetConfig cfg) {
    WindowState *w = window(win);
    if (!w)
      return 0;
    if (cfg.placeholder.size() > dp::kMaxShortTextBytes ||
        cfg.tooltip.size() > dp::kMaxShortTextBytes || !columns_representable(cfg.columns) ||
        !items_representable(cfg.items, 0))
      return 0;
    if (cfg.parent) {
      const WidgetState *p = w->widget(cfg.parent);
      if (!p || !is_container(p->type))
        return 0;
    }
    WidgetState s;
    s.id = next_id(w->next_widget, [w](uint16_t id) { return w->widget(id) != nullptr; });
    s.parent = cfg.parent;
    s.type = cfg.type;
    s.weight = cfg.weight;
    s.layout = cfg.layout;
    s.text = std::move(cfg.text);
    s.value = cfg.value;
    s.min = cfg.min;
    s.max = cfg.max;
    s.step = cfg.step;
    s.enabled = cfg.enabled;
    s.visible = cfg.visible;
    s.items = std::move(cfg.items);
    s.columns = std::move(cfg.columns);
    s.placeholder = std::move(cfg.placeholder);
    s.tooltip = std::move(cfg.tooltip);
    s.flags = cfg.flags;
    s.max_lines = cfg.max_lines;
    s.width = cfg.width;
    s.height = cfg.height;
    s.on_event = std::move(cfg.on_event);
    if (s.type == dp::WidgetType::TextArea)
      bound_text(s);
    auto pos = w->widgets.end();
    if (cfg.insert_before) {
      pos = std::find_if(w->widgets.begin(), w->widgets.end(), [&](const WidgetState &x) {
        return x.id == cfg.insert_before && x.parent == cfg.parent;
      });
      if (pos == w->widgets.end())
        return 0;
      s.insert_before = cfg.insert_before;
    }
    const uint16_t id = s.id;
    w->widgets.insert(pos, std::move(s));
    dirty.add(win, id);
    return id;
  }

  /// Remove a widget and its descendants (one id on the wire).
  bool remove_widget(uint16_t win, uint16_t id) {
    WindowState *w = window(win);
    if (!w || !w->widget(id))
      return false;
    std::vector<uint16_t> gone{id};
    for (const auto &x : w->widgets)
      if (std::find(gone.begin(), gone.end(), x.parent) != gone.end() && x.id != id &&
          x.parent != 0)
        gone.push_back(x.id);
    std::erase_if(w->widgets, [&](const WidgetState &x) {
      return std::find(gone.begin(), gone.end(), x.id) != gone.end();
    });
    dirty.remove(win, id, true);
    for (size_t i = 1; i < gone.size(); ++i)
      dirty.remove(win, gone[i], false);
    return true;
  }

  /// Remove every widget of a window.
  void clear_window(uint16_t win) {
    const WindowState *w = window(win);
    if (!w)
      return;
    std::vector<uint16_t> roots{};
    for (const auto &x : w->widgets)
      if (x.parent == 0)
        roots.push_back(x.id);
    for (const uint16_t id : roots)
      remove_widget(win, id);
  }

  /// Apply a property to the model (widget 0 = the window) and record it.
  /// Returns false when the target is unknown or the prop malformed. A Text
  /// the byte bound (max_text_bytes) trims is recorded as the trimmed text,
  /// so the host (which applies only the line bound) keeps the same value.
  bool set_prop(uint16_t win, uint16_t widget_id, dp::Prop prop) {
    WindowState *w = window(win);
    if (!w || !prop.valid() || !representable(prop))
      return false;
    using dp::PropTag;
    if (widget_id == 0) {
      switch (prop.type()) {
      case PropTag::Title:
        w->title = std::string(prop.as_text());
        break;
      case PropTag::WindowFlags:
        w->flags = *prop.as_u16();
        break;
      case PropTag::Geometry: {
        const auto g = *prop.as_geometry();
        if (g.x != -1)
          w->geometry.x = g.x;
        if (g.y != -1)
          w->geometry.y = g.y;
        if (g.w)
          w->geometry.w = g.w;
        if (g.h)
          w->geometry.h = g.h;
        break;
      }
      case PropTag::Focus:
        break;
      default:
        return false;
      }
      dirty.set(win, 0, std::move(prop));
      return true;
    }
    WidgetState *s = w->widget(widget_id);
    if (!s)
      return false;
    switch (prop.type()) {
    case PropTag::Text:
      s->text = std::string(prop.as_text());
      if (s->type == dp::WidgetType::TextArea && bound_text(*s))
        prop = dp::Prop::text(PropTag::Text, s->text);
      break;
    case PropTag::TextAppend:
      return append_text(win, widget_id, prop.as_text());
    case PropTag::Value:
      s->value = *prop.as_i32();
      break;
    case PropTag::Min:
      s->min = *prop.as_i32();
      break;
    case PropTag::Max:
      s->max = *prop.as_i32();
      break;
    case PropTag::Step:
      s->step = *prop.as_i32();
      break;
    case PropTag::Enabled:
      s->enabled = *prop.as_u8() != 0;
      break;
    case PropTag::Visible:
      s->visible = *prop.as_u8() != 0;
      break;
    case PropTag::Color:
      s->color = *prop.as_u32();
      break;
    case PropTag::Background:
      s->background = *prop.as_u32();
      break;
    case PropTag::ItemCount:
      s->items.resize(*prop.as_u16());
      break;
    case PropTag::Items: {
      const auto v = *prop.as_items();
      if (s->items.size() < v.start + v.items.size())
        s->items.resize(v.start + v.items.size());
      std::copy(v.items.begin(), v.items.end(), s->items.begin() + v.start);
      break;
    }
    case PropTag::Columns:
      s->columns = *prop.as_columns();
      break;
    case PropTag::Placeholder:
      s->placeholder = std::string(prop.as_text());
      break;
    case PropTag::Tooltip:
      s->tooltip = std::string(prop.as_text());
      break;
    case PropTag::Flags:
      s->flags = *prop.as_u16();
      break;
    case PropTag::MaxLines:
      s->max_lines = *prop.as_u16();
      if (s->type == dp::WidgetType::TextArea && bound_text(*s))
        dirty.set(win, widget_id, dp::Prop::text(PropTag::Text, s->text));
      break;
    case PropTag::Focus:
      break;
    case PropTag::Width:
      s->width = *prop.as_u16();
      break;
    case PropTag::Height:
      s->height = *prop.as_u16();
      break;
    case PropTag::InsertBefore:
    case PropTag::Title:
    case PropTag::WindowFlags:
    case PropTag::Geometry:
      return false;
    }
    dirty.set(win, widget_id, std::move(prop));
    return true;
  }

  /// Append to a widget's text (TextArea: bounded by max_lines, which the host
  /// applies identically, and max_text_bytes, which only the device applies:
  /// when the latter trims, the host gets a full Text replacement instead of
  /// the append so the two never diverge).
  bool append_text(uint16_t win, uint16_t widget_id, std::string_view text) {
    WidgetState *s = this->widget(win, widget_id);
    if (!s)
      return false;
    s->text += text;
    if (s->type == dp::WidgetType::TextArea && bound_text(*s)) {
      dirty.set(win, widget_id, dp::Prop::text(dp::PropTag::Text, s->text));
      return true;
    }
    const size_t pending = dirty.set(win, widget_id, dp::Prop::text(dp::PropTag::TextAppend, text));
    if (pending > max_text_bytes) {
      // the host would receive more than it keeps: replace with the bounded text
      dirty.set(win, widget_id, dp::Prop::text(dp::PropTag::Text, s->text));
    }
    return true;
  }

  // ---- dialogs / notifications ----

  /// Returns 0 (nothing opened) when the dialog is not representable (title
  /// and each button at most kMaxStr8Bytes, at most 255 buttons, text and
  /// default at most kMaxStr16Bytes) or would not fit one frame
  /// (max_payload): a dialog is never truncated.
  uint16_t open_dialog(dp::Dialog dialog,
                       std::function<void(const dp::DialogResult &)> on_result = nullptr) {
    if (dialog.buttons.empty())
      dialog.buttons.push_back("OK");
    if (dialog.title.size() > dp::kMaxStr8Bytes || dialog.text.size() > dp::kMaxStr16Bytes ||
        dialog.default_text.size() > dp::kMaxStr16Bytes || dialog.buttons.size() > 255 ||
        std::any_of(dialog.buttons.begin(), dialog.buttons.end(),
                    [](const std::string &b) { return b.size() > dp::kMaxStr8Bytes; }))
      return 0;
    if (dp::encode_dialog(dialog).size() > max_payload)
      return 0;
    dialog.id = next_id(next_dialog_, [this](uint16_t id) { return this->dialog(id) != nullptr; });
    dialogs_.push_back({.dialog = std::move(dialog), .on_result = std::move(on_result)});
    dirty.dialog_open(dialogs_.back().dialog.id);
    return dialogs_.back().dialog.id;
  }

  DialogState *dialog(uint16_t id) {
    const auto it = std::find_if(dialogs_.begin(), dialogs_.end(),
                                 [id](const DialogState &d) { return d.dialog.id == id; });
    return it != dialogs_.end() ? &*it : nullptr;
  }
  const std::vector<DialogState> &dialogs() const { return dialogs_; }

  /// Remove a dialog; returns its on_result callback (to run with the result).
  /// @param on_wire false when the host already closed it (DIALOG_RESULT):
  ///        no DIALOG_CLOSE is sent.
  std::optional<std::function<void(const dp::DialogResult &)>> close_dialog(uint16_t id,
                                                                            bool on_wire = true) {
    DialogState *d = dialog(id);
    if (!d)
      return std::nullopt;
    auto fn = std::move(d->on_result);
    std::erase_if(dialogs_, [id](const DialogState &x) { return x.dialog.id == id; });
    if (on_wire)
      dirty.dialog_close(id);
    else
      std::erase(dirty.dialogs_opened, id); // never shown: nothing to close either
    return fn;
  }

  /// False (nothing queued) when the notification is not representable
  /// (title at most kMaxStr8Bytes, text at most kMaxStr16Bytes) or would not
  /// fit one frame.
  bool notify(dp::Notify n) {
    if (n.title.size() > dp::kMaxStr8Bytes || n.text.size() > dp::kMaxStr16Bytes)
      return false;
    if (dp::encode_notify(n).size() > max_payload)
      return false;
    dirty.notify(std::move(n));
    return true;
  }

  // ---- host events (update the model without echoing) ----

  bool apply_window_event(const dp::WindowEvent &e) {
    WindowState *w = window(e.window);
    if (!w)
      return false;
    using K = dp::WindowEventKind;
    switch (e.kind) {
    case K::Focus:
      for (auto &x : windows_)
        x.focused = false;
      w->focused = true;
      break;
    case K::Blur:
      w->focused = false;
      break;
    case K::Minimize:
      w->minimized = true;
      break;
    case K::Restore:
      w->minimized = false;
      w->maximized = false;
      break;
    case K::Maximize:
      w->maximized = true;
      w->minimized = false;
      break;
    case K::Moved:
    case K::Resized:
      w->geometry = {.x = e.x, .y = e.y, .w = e.w, .h = e.h};
      break;
    }
    return true;
  }

  /// @note For WidgetEventKind::Text pass the reassembled text in `e.text`.
  bool apply_widget_event(const dp::WidgetEvent &e) {
    WidgetState *s = widget(e.window, e.widget);
    if (!s)
      return false;
    using K = dp::WidgetEventKind;
    switch (e.kind) {
    case K::Change:
    case K::Select:
      s->value = e.value;
      break;
    case K::Submit:
    case K::Text:
      s->text = e.text;
      break;
    default:
      break;
    }
    return true;
  }

  // ---- output ----

  /// Everything pending, as messages in flush order; clears the tracker.
  std::vector<dp::Message> flush(size_t *dropped = nullptr) {
    using dp::Type;
    std::vector<dp::Message> out{};
    for (auto &[win_id, d] : dirty.windows) {
      if (d.close)
        out.push_back(
            {Type::WindowClose, dp::encode_window_close({.id = win_id, .reason = d.close_reason})});
      WindowState *w = window(win_id);
      if (!w)
        continue;
      if (d.open) {
        std::for_each(w->widgets.begin(), w->widgets.end(),
                      [](WidgetState &x) { x.insert_before = 0; });
        append_messages(out, dp::encode_window_open(w->to_open(false), max_payload, dropped));
        continue;
      }
      if (!d.added.empty()) {
        dp::WidgetAdd add{.window = win_id};
        std::for_each(d.added.begin(), d.added.end(), [&](uint16_t id) {
          if (WidgetState *s = w->widget(id)) {
            add.widgets.push_back(s->to_rec(true));
            s->insert_before = 0;
          }
        });
        append_messages(out, dp::encode_widget_add(add, max_payload, dropped));
      }
      if (!d.sets.empty())
        append_payloads(out, Type::WidgetSet,
                        dp::encode_widget_set({.window = win_id, .entries = std::move(d.sets)},
                                              max_payload, dropped));
      if (!d.removed.empty())
        append_payloads(
            out, Type::WidgetRemove,
            dp::encode_widget_remove({.window = win_id, .widgets = d.removed}, max_payload));
    }
    std::for_each(dirty.dialogs_opened.begin(), dirty.dialogs_opened.end(), [&](uint16_t id) {
      if (const DialogState *d = dialog(id))
        out.push_back({Type::Dialog, dp::encode_dialog(d->dialog)});
    });
    std::transform(dirty.dialogs_closed.begin(), dirty.dialogs_closed.end(),
                   std::back_inserter(out), [](uint16_t id) {
                     return dp::Message{Type::DialogClose, dp::encode_dialog_close({.id = id})};
                   });
    std::transform(dirty.notifications.begin(), dirty.notifications.end(), std::back_inserter(out),
                   [](const dp::Notify &n) {
                     return dp::Message{Type::Notify, dp::encode_notify(n)};
                   });
    if (dirty.desktop_changed)
      out.push_back({Type::Desktop, dp::encode_desktop(desktop_info(false), max_payload)});
    dirty.clear();
    return out;
  }

  /// The full state for a (re)connecting host: WINDOW_OPEN(Snapshot) per open
  /// window and DIALOG per open dialog (the DESKTOP reply precedes them;
  /// see desktop_info(true)).
  std::vector<dp::Message> snapshot(size_t *dropped = nullptr) const {
    std::vector<dp::Message> out{};
    std::for_each(windows_.begin(), windows_.end(), [&](const WindowState &w) {
      append_messages(out, dp::encode_window_open(w.to_open(true), max_payload, dropped));
    });
    std::transform(dialogs_.begin(), dialogs_.end(), std::back_inserter(out),
                   [](const DialogState &d) {
                     return dp::Message{dp::Type::Dialog, dp::encode_dialog(d.dialog)};
                   });
    return out;
  }

  /// Move encoded messages onto the end of `out`.
  static void append_messages(std::vector<dp::Message> &out, std::vector<dp::Message> msgs) {
    std::move(msgs.begin(), msgs.end(), std::back_inserter(out));
  }

  /// Append payloads as messages of one type.
  static void append_payloads(std::vector<dp::Message> &out, dp::Type type,
                              std::vector<std::vector<uint8_t>> payloads) {
    std::transform(payloads.begin(), payloads.end(), std::back_inserter(out),
                   [type](std::vector<uint8_t> &p) {
                     return dp::Message{type, std::move(p)};
                   });
  }

  /// Room in a frame for one property record beside the WIDGET_SET /
  /// WIDGET_ADD headers, a widget base and the record's own header.
  size_t rec_room() const { return max_payload > 24 ? max_payload - 24 : 0; }

  /// Columns: at most 255 names of at most kMaxStr8Bytes, whose record fits a
  /// frame (checked on the strings, BEFORE Prop::columns could cut them).
  bool columns_representable(const std::vector<std::string> &cols) const {
    if (cols.size() > 255)
      return false;
    size_t encoded = 1;
    for (const auto &c : cols) {
      if (c.size() > dp::kMaxStr8Bytes)
        return false;
      encoded += 1 + c.size();
    }
    return encoded <= rec_room();
  }

  /// Items: every entry small enough for a frame of its own (ranges split, an
  /// entry does not) and at most kMaxStr16Bytes; the range `start .. start +
  /// size` must fit the u16 index space (the wire carries u16 start / count /
  /// ItemCount).
  bool items_representable(std::span<const std::string> items, size_t start = 0) const {
    if (items.size() > 0xFFFF || start > 0xFFFF || start + items.size() > 0xFFFF)
      return false;
    return std::all_of(items.begin(), items.end(), [this](const std::string &i) {
      return i.size() <= dp::kMaxStr16Bytes && i.size() + 2 + 4 <= rec_room();
    });
  }

  /// Replace the column names (validated on the strings; false = refused).
  bool set_columns(uint16_t win, uint16_t widget_id, const std::vector<std::string> &cols) {
    return columns_representable(cols) && set_prop(win, widget_id, dp::Prop::columns(cols));
  }

  /// Replace a range of items (validated on the strings and the range; false =
  /// refused, nothing changed).
  bool set_items(uint16_t win, uint16_t widget_id, uint16_t start,
                 std::span<const std::string> items) {
    return items_representable(items, start) && widget(win, widget_id) != nullptr &&
           set_prop(win, widget_id, dp::Prop::items(start, items));
  }

  /// Replace every item (ItemCount + the full range). Validated BEFORE anything
  /// is mutated: false = refused, the model is unchanged.
  bool replace_items(uint16_t win, uint16_t widget_id, std::span<const std::string> items) {
    if (!items_representable(items, 0) || widget(win, widget_id) == nullptr)
      return false;
    set_prop(win, widget_id,
             dp::Prop::u16(dp::PropTag::ItemCount, static_cast<uint16_t>(items.size())));
    return items.empty() || set_prop(win, widget_id, dp::Prop::items(0, items));
  }

  /// Whether a property's value can be carried on the wire within this
  /// model's payload cap: Title / Placeholder / Tooltip at most
  /// kMaxShortTextBytes (they are not splittable), Columns / Items as above
  /// (on the already-built record). Text / TextAppend split: any length.
  bool representable(const dp::Prop &prop) const {
    using dp::PropTag;
    switch (prop.type()) {
    case PropTag::Title:
    case PropTag::Placeholder:
    case PropTag::Tooltip:
      return prop.value.size() <= dp::kMaxShortTextBytes;
    case PropTag::Columns: {
      const auto cols = prop.as_columns();
      return cols && columns_representable(*cols);
    }
    case PropTag::Items: {
      const auto items = prop.as_items();
      return items && items_representable(items->items);
    }
    default:
      return true;
    }
  }

  static bool is_container(dp::WidgetType t) {
    return t == dp::WidgetType::Column || t == dp::WidgetType::Row || t == dp::WidgetType::Group;
  }

private:
  /// Monotonic u16 ids, 0 reserved; skips ids still in use after a wrap.
  template <typename Id, typename InUse> static Id next_id(Id &counter, InUse in_use) {
    constexpr Id last = std::numeric_limits<Id>::max();
    for (size_t i = 0; i < static_cast<size_t>(last); ++i) {
      const Id id = counter;
      counter = static_cast<Id>(counter == last ? 1 : counter + 1);
      if (id != 0 && !in_use(id))
        return id;
    }
    return 0;
  }

  /// Keep a TextArea's text to its last max_lines lines (the same rule as the
  /// browser's ring: the text after the last newline counts as a line, so
  /// "a\nb\n" is three lines) and to max_text_bytes bytes. Returns true when
  /// the BYTE bound cut something (the host does not apply it).
  bool bound_text(WidgetState &s) const {
    if (s.max_lines) {
      const size_t lines = static_cast<size_t>(std::count(s.text.begin(), s.text.end(), '\n')) + 1;
      if (lines > s.max_lines) {
        size_t drop = lines - s.max_lines;
        size_t pos = 0;
        while (drop > 0) {
          pos = s.text.find('\n', pos) + 1;
          --drop;
        }
        s.text.erase(0, pos);
      }
    }
    if (s.text.size() <= max_text_bytes)
      return false;
    size_t cut = s.text.size() - max_text_bytes;
    // keep whole lines / UTF-8 sequences
    const size_t nl = s.text.find('\n', cut);
    if (nl != std::string::npos && nl + 1 < s.text.size())
      cut = nl + 1;
    while (cut < s.text.size() && (static_cast<uint8_t>(s.text[cut]) & 0xC0) == 0x80)
      ++cut;
    s.text.erase(0, cut);
    return true;
  }

  std::vector<dp::AppRec> apps_{};
  std::vector<WindowState> windows_{};
  std::vector<DialogState> dialogs_{};
  uint16_t next_window_{1};
  uint16_t next_dialog_{1};
  uint8_t next_app_{1};
};

} // namespace espp::detail::desktop_model

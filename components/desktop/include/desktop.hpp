#pragma once

// espp::Desktop -- a browser-rendered windowed desktop for microcontrollers.
// The firmware describes APPS, WINDOWS and WIDGETS (a retained tree); a web
// app (components/desktop/web/desktop.html) draws them, lets the user move /
// resize / close windows and operate the widgets, and sends the events back.
// The transport is any framed byte stream: espp::DesktopService adapts one
// Desktop to a dispatcher module (WebUSB / Web Serial in the examples).
//
// Threading (the rules every app can rely on):
//   - every application callback (App::launch, widget / window / dialog
//     handlers, timers, post()) runs on the desktop task, with NO lock held
//     (the function is copied out under the lock, then called), so a handler
//     may freely call any Desktop / Window / Widget method, including closing
//     its own window;
//   - any task may call the mutators (set_text, add, close_window, notify,
//     ...): they take the model mutex, record the change and wake the desktop
//     task; they never send;
//   - all device->host frames are produced by the desktop task, once per
//     iteration: drain host commands -> run due timers / posted functions ->
//     flush. A flush coalesces everything changed since the last one (last
//     value wins, text appends concatenate, a removed widget cancels its
//     pending changes ...; see detail/desktop_model.hpp) into the fewest
//     frames, encoded under the lock and sent outside it to every active sink
//     under that sink's own send mutex. The only frame sent from elsewhere is
//     DesktopService's ERROR for a malformed request.
//
// Ids are monotonic u16 (0 reserved); handles (Window / Widget) are plain
// values that become invalid (valid() == false, methods no-ops) once the
// target is gone.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <variant>
#include <vector>

#include "base_component.hpp"
#include "format.hpp"
#include "task.hpp"

#include "detail/desktop_model.hpp"
#include "detail/desktop_protocol.hpp"

namespace espp {
namespace detail {
namespace dp = desktop_protocol; // short alias for the wire types used below
} // namespace detail

/**
 * @brief The retained desktop: apps, windows, widgets, dialogs and
 *        notifications, flushed to the browser by its own task. One per
 *        device; attach one espp::DesktopService per transport.
 *
 * \section desktop_ex1 Counter app (the API in 30 lines)
 * \snippet counter_app.hpp counter_app
 * \section desktop_ex2 Wiring
 * \snippet desktop_example.cpp desktop_example
 */
class Desktop : public BaseComponent {
public:
  using AppId = uint8_t;
  using WindowId = uint16_t;
  using WidgetId = uint16_t;
  using DialogId = uint16_t;
  using TimerId = uint32_t;
  using SinkId = uint32_t;

  using WidgetType = detail::dp::WidgetType;
  using WidgetEventKind = detail::dp::WidgetEventKind;
  using WindowEventKind = detail::dp::WindowEventKind;
  using WindowCloseReason = detail::dp::WindowCloseReason;
  using DialogKind = detail::dp::DialogKind;
  using DialogIcon = detail::dp::DialogIcon;
  using NotifyLevel = detail::dp::NotifyLevel;
  using Geometry = detail::dp::Geometry;
  /// A widget event from the host: `kind` says which of value / text / key
  /// apply (Text events arrive reassembled, `text` holds the whole text).
  using WidgetEvent = detail::dp::WidgetEvent;
  /// A window event from the host (Moved / Resized carry the new geometry).
  using WindowEvent = detail::dp::WindowEvent;
  using WidgetConfig = detail::desktop_model::WidgetConfig;
  using WindowConfig = detail::desktop_model::WindowConfig;
  using widget_event_fn = std::function<void(const WidgetEvent &)>;
  using window_event_fn = std::function<void(const WindowEvent &)>;
  /// Transmits one encoded frame to a host (one per transport; see add_sink).
  /// Returns false when the frame could NOT be queued (the transport is
  /// gone, or its FIFO did not drain in time): the desktop then flags the
  /// sink as needing a resync (sink_needs_resync) and logs, since the host's
  /// mirror of the desktop is now incomplete until its next GET_DESKTOP. A
  /// transport must never queue a partial frame (write a frame all-or-nothing).
  using send_fn = std::function<bool(std::span<const uint8_t> frame)>;

  // Window flags (WindowConfig::flags / Window::set_flags).
  static constexpr uint16_t kWinMovable = detail::dp::kWinMovable;
  static constexpr uint16_t kWinResizable = detail::dp::kWinResizable;
  static constexpr uint16_t kWinClosable = detail::dp::kWinClosable;
  static constexpr uint16_t kWinModal = detail::dp::kWinModal;
  static constexpr uint16_t kWinMinimizable = detail::dp::kWinMinimizable;
  static constexpr uint16_t kWinMaximizable = detail::dp::kWinMaximizable;
  static constexpr uint16_t kWinCentered = detail::dp::kWinCentered;
  static constexpr uint16_t kWinPinned = detail::dp::kWinPinned;
  static constexpr uint16_t kWinWantsGeometry = detail::dp::kWinWantsGeometry;
  static constexpr uint16_t kWinDefaultFlags = detail::dp::kWinDefaultFlags;
  // Widget layout bits (WidgetConfig::layout).
  static constexpr uint8_t kLayoutStretch = detail::dp::kLayoutStretch;
  static constexpr uint8_t kLayoutScroll = detail::dp::kLayoutScroll;
  static constexpr uint8_t kLayoutAlignEnd = detail::dp::kLayoutAlignEnd;
  static constexpr uint8_t kLayoutAlignCenter = detail::dp::kLayoutAlignCenter;
  // Widget flags (WidgetConfig::flags / Widget::set_flags), per type.
  static constexpr uint16_t kTextAreaReadOnly = detail::dp::kTextAreaReadOnly;
  static constexpr uint16_t kTextAreaMonospace = detail::dp::kTextAreaMonospace;
  static constexpr uint16_t kTextAreaWantKeys = detail::dp::kTextAreaWantKeys;
  static constexpr uint16_t kTextAreaAutoScroll = detail::dp::kTextAreaAutoScroll;
  static constexpr uint16_t kTextAreaAnsi = detail::dp::kTextAreaAnsi;
  static constexpr uint16_t kTextBoxPassword = detail::dp::kTextBoxPassword;
  static constexpr uint16_t kTextBoxReadOnly = detail::dp::kTextBoxReadOnly;
  static constexpr uint16_t kLabelBold = detail::dp::kLabelBold;
  static constexpr uint16_t kLabelMonospace = detail::dp::kLabelMonospace;
  static constexpr uint16_t kLabelWrap = detail::dp::kLabelWrap;
  static constexpr uint16_t kButtonPrimary = detail::dp::kButtonPrimary;
  static constexpr uint16_t kButtonDanger = detail::dp::kButtonDanger;
  /// A list / table / select selection meaning "nothing".
  static constexpr int32_t kNoSelection = -1;
  /// Smallest Config::max_frame_bytes: the largest frame header + CRC + a
  /// 64-byte payload (enough for every fixed-layout message and a widget base).
  static constexpr size_t kMinFrameBytes =
      espp::stream_frame::kMaxHeaderSize + espp::stream_frame::kCrcSize + 64;
  // Registry limits (DESKTOP is one frame; see detail/desktop_protocol.hpp).
  static constexpr size_t kMaxApps = detail::dp::kMaxApps;
  static constexpr size_t kMaxAppNameBytes = detail::dp::kMaxAppNameBytes;
  static constexpr size_t kMaxAppIconBytes = detail::dp::kMaxAppIconBytes;
  static constexpr size_t kMaxAppDescriptionBytes = detail::dp::kMaxAppDescriptionBytes;
  static constexpr size_t kMaxDeviceNameBytes = detail::dp::kMaxDeviceNameBytes;
  static constexpr size_t kMaxFirmwareBytes = detail::dp::kMaxFirmwareBytes;

  /// Configuration for the Desktop.
  struct Config {
    /// Shown in the browser's tray / title (at most kMaxDeviceNameBytes, else truncated).
    std::string device_name{"espp"};
    /// e.g. project name + version (DESKTOP record; at most kMaxFirmwareBytes).
    std::string firmware{};
    std::string theme{"auto"}; ///< "auto" | "light" | "dark" (the browser's initial theme).
    uint32_t accent{0x3b82f6}; ///< Accent color, 0xRRGGBB.
    /// How often pending changes are coalesced and sent (the latency of a
    /// set_text, and the period that bounds the frame rate of a busy app).
    std::chrono::milliseconds flush_period{50};
    /// Largest encoded frame (header + payload + CRC) a sink can carry in one
    /// write; every widget payload is split to fit (4096 = the TinyUSB FIFOs of
    /// the espp examples; the stream_frame maximum is kMaxFrameSize). At least
    /// kMinFrameBytes (a 64-byte payload); a smaller value is clamped with a
    /// warning. Dialogs, notifications and the DESKTOP record set are single
    /// frames, so a small cap limits them (see the k* limits in the protocol).
    size_t max_frame_bytes{4096};
    /// Bound on host commands queued for the desktop task (at least 1). When
    /// it is full a request (GET_DESKTOP / LAUNCH_APP / CLOSE_WINDOW) is
    /// refused with ERROR(EAGAIN) and an event is dropped (logged, rate-limited);
    /// queued commands are never evicted.
    size_t max_queued_commands{64};
    /// Bound on a TextArea's retained text and on a text the host sends for
    /// one widget (Text events are reassembled up to this size).
    size_t max_text_bytes{16 * 1024};
    /// The desktop task: every app callback runs on it, so size the stack for
    /// the apps (file I/O and fmt formatting comfortably fit 8 KiB).
    Task::BaseConfig task_config{.name = "desktop", .stack_size_bytes = 8 * 1024};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Logger verbosity.
  };

  /// An application the desktop lists (register_app) and launches on request.
  struct App {
    std::string name{};        ///< Shown under the icon / in the start menu.
    std::string icon{};        ///< An emoji / short text, or "svg:<name>" from the built-in set.
    std::string description{}; ///< Tooltip.
    /// Called on the desktop task when the user launches the app (or
    /// launch() is called): create the window(s) here.
    std::function<void(Desktop &, AppId)> launch{nullptr};
    bool single_instance{true}; ///< Launching again focuses the open window.
    bool hidden{false};         ///< Not shown on the desktop (launch() only).
  };

  class Window;
  class Widget;

  /// A message box (message_box()). `on_result(button)` gets the index of the
  /// button pressed, or -1 when dismissed.
  struct MessageBoxConfig {
    WindowId owner{0}; ///< Modal to this window (0 = to the whole desktop).
    std::string title{};
    std::string text{};
    std::vector<std::string> buttons{"OK"}; ///< Button 0 is the default.
    DialogIcon icon{DialogIcon::None};
    std::function<void(int button)> on_result{nullptr};
  };

  /// An input box (input_box()). `on_result(text)` gets the text when the
  /// default button (0) was pressed, nullopt when cancelled / dismissed.
  struct InputBoxConfig {
    WindowId owner{0};
    std::string title{};
    std::string text{};         ///< The prompt.
    std::string default_text{}; ///< Initial field contents.
    std::vector<std::string> buttons{"OK", "Cancel"};
    DialogIcon icon{DialogIcon::Question};
    std::function<void(std::optional<std::string> text)> on_result{nullptr};
  };

  /// A toast notification (notify()).
  struct NotifyConfig {
    std::string title{};
    std::string text{};
    NotifyLevel level{NotifyLevel::Info};
    std::chrono::milliseconds timeout{4000}; ///< 0 = sticky until dismissed.
  };

  /// A GET_DESKTOP request (Command).
  struct GetDesktop {};

  /// A decoded host request, submitted by a DesktopService (or a test) and
  /// handled on the desktop task. Replies (if any) go to `sink`.
  struct Command {
    SinkId sink{0};
    std::optional<uint16_t> correlation{};
    std::variant<GetDesktop, detail::dp::LaunchApp, detail::dp::CloseWindow,
                 detail::dp::WindowEvent, detail::dp::WidgetEvent, detail::dp::DialogResult>
        request{GetDesktop{}};
  };

  /// A handle to a widget: a value, safe to copy and keep; every method is a
  /// no-op once the widget (or its window) is gone.
  class Widget {
  public:
    Widget() = default;
    Widget(Desktop *desktop, WindowId window, WidgetId id)
        : desktop_(desktop)
        , window_(window)
        , id_(id) {}
    bool valid() const { return desktop_ && desktop_->widget_exists(window_, id_); }
    explicit operator bool() const { return valid(); }
    WidgetId id() const { return id_; }
    WindowId window() const { return window_; }

    void set_text(std::string_view text) {
      set(detail::dp::Prop::text(detail::dp::PropTag::Text, text));
    }
    template <typename... Args>
    requires(sizeof...(Args) > 0) void set_text(fmt::format_string<Args...> f, Args &&...args) {
      set_text(std::string_view(fmt::format(f, std::forward<Args>(args)...)));
    }
    void append_text(std::string_view text) {
      if (desktop_)
        desktop_->append_text(window_, id_, text);
    }
    void set_value(int32_t value) { set(detail::dp::Prop::i32(detail::dp::PropTag::Value, value)); }
    void set_checked(bool checked) { set_value(checked ? 1 : 0); }
    void set_range(int32_t min, int32_t max, int32_t step = 1) {
      set(detail::dp::Prop::i32(detail::dp::PropTag::Min, min));
      set(detail::dp::Prop::i32(detail::dp::PropTag::Max, max));
      set(detail::dp::Prop::i32(detail::dp::PropTag::Step, step));
    }
    void set_enabled(bool enabled) {
      set(detail::dp::Prop::u8(detail::dp::PropTag::Enabled, enabled));
    }
    void set_visible(bool visible) {
      set(detail::dp::Prop::u8(detail::dp::PropTag::Visible, visible));
    }
    /// 0xRRGGBB; 0xFFFFFFFF = the theme's default.
    void set_color(uint32_t rgb) { set(detail::dp::Prop::u32(detail::dp::PropTag::Color, rgb)); }
    void set_background(uint32_t rgb) {
      set(detail::dp::Prop::u32(detail::dp::PropTag::Background, rgb));
    }
    /// Replace every item (table rows: cells '\t'-separated).
    void set_items(const std::vector<std::string> &items) {
      set(detail::dp::Prop::u16(detail::dp::PropTag::ItemCount,
                                static_cast<uint16_t>(items.size())));
      if (!items.empty())
        set(detail::dp::Prop::items(0, items));
    }
    /// Replace one item (the list grows to fit).
    void set_item(uint16_t index, std::string_view item) {
      const std::vector<std::string> one{std::string(item)};
      set(detail::dp::Prop::items(index, one));
    }
    /// Replace a range of items starting at `start`.
    void set_items(uint16_t start, const std::vector<std::string> &items) {
      if (!items.empty())
        set(detail::dp::Prop::items(start, items));
    }
    void set_item_count(uint16_t count) {
      set(detail::dp::Prop::u16(detail::dp::PropTag::ItemCount, count));
    }
    /// Select an item (kNoSelection = none).
    void set_selected(int32_t index) { set_value(index); }
    void set_columns(const std::vector<std::string> &columns) {
      set(detail::dp::Prop::columns(columns));
    }
    void set_flags(uint16_t flags) {
      set(detail::dp::Prop::u16(detail::dp::PropTag::Flags, flags));
    }
    void set_placeholder(std::string_view text) {
      set(detail::dp::Prop::text(detail::dp::PropTag::Placeholder, text));
    }
    void set_tooltip(std::string_view text) {
      set(detail::dp::Prop::text(detail::dp::PropTag::Tooltip, text));
    }
    void set_max_lines(uint16_t lines) {
      set(detail::dp::Prop::u16(detail::dp::PropTag::MaxLines, lines));
    }
    void set_size(uint16_t width, uint16_t height) {
      set(detail::dp::Prop::u16(detail::dp::PropTag::Width, width));
      set(detail::dp::Prop::u16(detail::dp::PropTag::Height, height));
    }
    /// Give the widget keyboard focus.
    void focus() { set(detail::dp::Prop::u8(detail::dp::PropTag::Focus, 1)); }
    /// Remove the widget (and its children).
    void remove() {
      if (desktop_)
        desktop_->remove_widget(window_, id_);
    }
    /// Replace the event handler.
    void on_event(widget_event_fn fn) {
      if (desktop_)
        desktop_->set_widget_handler(window_, id_, std::move(fn));
    }

    // Getters read the model (host events update it before the handler runs).
    std::string text() const {
      std::string s{};
      if (desktop_)
        desktop_->with_widget(window_, id_, [&](const auto &w) { s = w.text; });
      return s;
    }
    int32_t value() const {
      int32_t v = 0;
      if (desktop_)
        desktop_->with_widget(window_, id_, [&](const auto &w) { v = w.value; });
      return v;
    }
    bool checked() const { return value() != 0; }
    int32_t selected() const { return value(); }
    std::vector<std::string> items() const {
      std::vector<std::string> v{};
      if (desktop_)
        desktop_->with_widget(window_, id_, [&](const auto &w) { v = w.items; });
      return v;
    }
    bool enabled() const {
      bool v = false;
      if (desktop_)
        desktop_->with_widget(window_, id_, [&](const auto &w) { v = w.enabled; });
      return v;
    }

  private:
    void set(detail::dp::Prop prop) {
      if (desktop_)
        desktop_->set_prop(window_, id_, std::move(prop));
    }
    Desktop *desktop_{nullptr};
    WindowId window_{0};
    WidgetId id_{0};
  };

  /// A handle to a window: a value, safe to copy and keep; every method is a
  /// no-op once the window is closed.
  class Window {
  public:
    Window() = default;
    Window(Desktop *desktop, WindowId id)
        : desktop_(desktop)
        , id_(id) {}
    bool valid() const { return desktop_ && desktop_->window_exists(id_); }
    explicit operator bool() const { return valid(); }
    WindowId id() const { return id_; }

    /// Add any widget (the general form; the helpers below cover the
    /// common ones). Returns an invalid handle on failure (unknown parent,
    /// parent not a container, window gone).
    Widget add(WidgetConfig cfg) {
      const WidgetId wid = desktop_ ? desktop_->add_widget(id_, std::move(cfg)) : 0;
      return wid ? Widget(desktop_, id_, wid) : Widget();
    }
    Widget widget(WidgetId widget_id) const { return Widget(desktop_, id_, widget_id); }

    // ---- containers (parent 0 = the window's root column) ----
    Widget column(WidgetId parent = 0, uint8_t weight = 0, uint8_t layout = 0) {
      return add(
          {.type = WidgetType::Column, .parent = parent, .weight = weight, .layout = layout});
    }
    Widget row(WidgetId parent = 0, uint8_t weight = 0, uint8_t layout = 0) {
      return add({.type = WidgetType::Row, .parent = parent, .weight = weight, .layout = layout});
    }
    Widget group(std::string_view title, WidgetId parent = 0, uint8_t weight = 0,
                 uint8_t layout = 0) {
      return add({.type = WidgetType::Group,
                  .parent = parent,
                  .text = std::string(title),
                  .weight = weight,
                  .layout = layout});
    }
    // ---- leaves ----
    Widget label(std::string_view text, WidgetId parent = 0, uint16_t flags = 0) {
      return add(
          {.type = WidgetType::Label, .parent = parent, .text = std::string(text), .flags = flags});
    }
    Widget button(std::string_view text, std::function<void()> on_click, WidgetId parent = 0,
                  uint16_t flags = 0) {
      return add({.type = WidgetType::Button,
                  .parent = parent,
                  .text = std::string(text),
                  .flags = flags,
                  .on_event = [fn = std::move(on_click)](const WidgetEvent &e) {
                    if (fn && e.kind == WidgetEventKind::Click)
                      fn();
                  }});
    }
    Widget checkbox(std::string_view text, bool checked, std::function<void(bool)> on_change,
                    WidgetId parent = 0) {
      return add({.type = WidgetType::Checkbox,
                  .parent = parent,
                  .text = std::string(text),
                  .value = checked ? 1 : 0,
                  .on_event = [fn = std::move(on_change)](const WidgetEvent &e) {
                    if (fn && e.kind == WidgetEventKind::Change)
                      fn(e.value != 0);
                  }});
    }
    /// A single-line field; `on_submit` gets the text on Enter.
    Widget textbox(std::string_view text, std::function<void(const std::string &)> on_submit,
                   WidgetId parent = 0, std::string_view placeholder = "", uint16_t flags = 0) {
      return add({.type = WidgetType::TextBox,
                  .parent = parent,
                  .text = std::string(text),
                  .placeholder = std::string(placeholder),
                  .flags = flags,
                  .on_event = [fn = std::move(on_submit)](const WidgetEvent &e) {
                    if (fn && e.kind == WidgetEventKind::Submit)
                      fn(e.text);
                  }});
    }
    Widget textarea(std::string_view text, WidgetId parent = 0, uint16_t flags = 0,
                    uint8_t weight = 1, uint16_t max_lines = 500) {
      return add({.type = WidgetType::TextArea,
                  .parent = parent,
                  .text = std::string(text),
                  .flags = flags,
                  .weight = weight,
                  .layout = kLayoutStretch,
                  .max_lines = max_lines});
    }
    /// `on_select(index)` on selection change; Activate (double-click /
    /// Enter) arrives through Widget::on_event.
    Widget list(const std::vector<std::string> &items, std::function<void(int32_t)> on_select,
                WidgetId parent = 0, uint8_t weight = 1) {
      return add({.type = WidgetType::List,
                  .parent = parent,
                  .value = kNoSelection,
                  .items = items,
                  .weight = weight,
                  .layout = kLayoutStretch,
                  .on_event = [fn = std::move(on_select)](const WidgetEvent &e) {
                    if (fn && e.kind == WidgetEventKind::Select)
                      fn(e.value);
                  }});
    }
    Widget table(const std::vector<std::string> &columns, const std::vector<std::string> &rows,
                 std::function<void(int32_t)> on_select, WidgetId parent = 0, uint8_t weight = 1) {
      return add({.type = WidgetType::Table,
                  .parent = parent,
                  .value = kNoSelection,
                  .items = rows,
                  .columns = columns,
                  .weight = weight,
                  .layout = kLayoutStretch,
                  .on_event = [fn = std::move(on_select)](const WidgetEvent &e) {
                    if (fn && e.kind == WidgetEventKind::Select)
                      fn(e.value);
                  }});
    }
    Widget select(const std::vector<std::string> &items, int32_t selected,
                  std::function<void(int32_t)> on_change, WidgetId parent = 0) {
      return add({.type = WidgetType::Select,
                  .parent = parent,
                  .value = selected,
                  .items = items,
                  .on_event = [fn = std::move(on_change)](const WidgetEvent &e) {
                    if (fn && e.kind == WidgetEventKind::Change)
                      fn(e.value);
                  }});
    }
    Widget progress(int32_t value, int32_t min = 0, int32_t max = 100, WidgetId parent = 0) {
      return add({.type = WidgetType::Progress,
                  .parent = parent,
                  .value = value,
                  .min = min,
                  .max = max,
                  .layout = kLayoutStretch});
    }
    Widget slider(int32_t value, int32_t min, int32_t max, std::function<void(int32_t)> on_change,
                  WidgetId parent = 0, int32_t step = 1) {
      return add({.type = WidgetType::Slider,
                  .parent = parent,
                  .value = value,
                  .min = min,
                  .max = max,
                  .step = step,
                  .layout = kLayoutStretch,
                  .on_event = [fn = std::move(on_change)](const WidgetEvent &e) {
                    if (fn && e.kind == WidgetEventKind::Change)
                      fn(e.value);
                  }});
    }
    Widget separator(WidgetId parent = 0) {
      return add({.type = WidgetType::Separator, .parent = parent});
    }
    Widget spacer(WidgetId parent = 0, uint8_t weight = 1) {
      return add({.type = WidgetType::Spacer, .parent = parent, .weight = weight});
    }

    // ---- the window itself ----
    void set_title(std::string_view title) {
      if (desktop_)
        desktop_->set_prop(id_, 0, detail::dp::Prop::text(detail::dp::PropTag::Title, title));
    }
    template <typename... Args>
    requires(sizeof...(Args) > 0) void set_title(fmt::format_string<Args...> f, Args &&...args) {
      set_title(std::string_view(fmt::format(f, std::forward<Args>(args)...)));
    }
    void set_flags(uint16_t flags) {
      if (desktop_)
        desktop_->set_prop(id_, 0, detail::dp::Prop::u16(detail::dp::PropTag::WindowFlags, flags));
    }
    /// Ask the browser to move / resize (-1 / 0 = keep); the host answers
    /// with a Moved / Resized event carrying the clamped result.
    void request_geometry(int16_t x, int16_t y, uint16_t w, uint16_t h) {
      if (desktop_)
        desktop_->set_prop(id_, 0, detail::dp::Prop::geometry({.x = x, .y = y, .w = w, .h = h}));
    }
    /// Raise and focus the window.
    void focus() {
      if (desktop_)
        desktop_->set_prop(id_, 0, detail::dp::Prop::u8(detail::dp::PropTag::Focus, 1));
    }
    /// Remove every widget.
    void clear() {
      if (desktop_)
        desktop_->clear_window(id_);
    }
    void close() {
      if (desktop_)
        desktop_->close_window(id_);
    }
    /// A periodic callback on the desktop task, cancelled when the window closes.
    TimerId add_timer(std::chrono::milliseconds period, std::function<void()> fn) {
      return desktop_ ? desktop_->add_timer(period, std::move(fn), id_) : 0;
    }
    std::string title() const {
      std::string s{};
      if (desktop_)
        desktop_->with_window(id_, [&](const auto &w) { s = w.title; });
      return s;
    }
    Geometry geometry() const {
      Geometry g{};
      if (desktop_)
        desktop_->with_window(id_, [&](const auto &w) { g = w.geometry; });
      return g;
    }
    Desktop *desktop() const { return desktop_; }

  private:
    Desktop *desktop_{nullptr};
    WindowId id_{0};
  };

  /// @brief Construct the desktop and start its task.
  explicit Desktop(const Config &config)
      : BaseComponent("Desktop", config.log_level)
      , config_(config)
      , text_(config.max_text_bytes) {
    if (config_.max_frame_bytes < kMinFrameBytes) {
      logger_.warn("max_frame_bytes {} is below the minimum {}; using the minimum",
                   config_.max_frame_bytes, kMinFrameBytes);
      config_.max_frame_bytes = kMinFrameBytes;
    }
    if (config_.max_queued_commands == 0) {
      logger_.warn("max_queued_commands must be at least 1; using 1");
      config_.max_queued_commands = 1;
    }
    model_.device_name = truncated("device_name", config.device_name, kMaxDeviceNameBytes);
    model_.firmware = truncated("firmware", config.firmware, kMaxFirmwareBytes);
    model_.theme = config.theme;
    model_.accent = config.accent;
    model_.flush_period_ms =
        static_cast<uint16_t>(std::min<int64_t>(config.flush_period.count(), 65535));
    model_.max_payload = max_payload();
    model_.max_text_bytes = config.max_text_bytes;
    last_flush_ = std::chrono::steady_clock::now();
    task_ = std::make_unique<Task>(
        Task::Config{.callback = [this](std::mutex &m, std::condition_variable &cv,
                                        bool &notified) { return step(m, cv, notified); },
                     .task_config = config.task_config});
    task_->start();
  }

  ~Desktop() {
    if (task_)
      task_->stop();
    task_.reset();
  }

  // ---- apps ----

  /// @brief Register an app; returns its id, or 0 (logged) when kMaxApps are
  ///        registered already or the DESKTOP record set would no longer fit
  ///        one frame. Name / icon / description longer than kMaxAppNameBytes
  ///        / kMaxAppIconBytes / kMaxAppDescriptionBytes are truncated (logged).
  AppId register_app(App app) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (model_.apps().size() >= kMaxApps) {
      logger_.error("cannot register '{}': {} apps already (kMaxApps)", app.name, kMaxApps);
      return 0;
    }
    const AppId id = model_.allocate_app_id();
    if (!id) {
      logger_.error("no free app id for '{}'", app.name);
      return 0;
    }
    detail::dp::AppRec rec{
        .id = id,
        .flags = static_cast<uint8_t>((app.single_instance ? detail::dp::kAppSingleInstance : 0) |
                                      (app.hidden ? detail::dp::kAppHidden : 0)),
        .name = truncated("app name", app.name, kMaxAppNameBytes),
        .icon = truncated("app icon", app.icon, kMaxAppIconBytes),
        .description = truncated("app description", app.description, kMaxAppDescriptionBytes)};
    // the record set must stay one frame (with the windows open right now)
    detail::dp::DesktopInfo probe = model_.desktop_info(false);
    probe.apps.push_back(rec);
    if (detail::dp::encode_desktop(probe).size() > max_payload()) {
      logger_.error("cannot register '{}': the DESKTOP record set would exceed {} bytes", app.name,
                    max_payload());
      return 0;
    }
    model_.register_app(std::move(rec));
    launchers_[id] = std::move(app.launch);
    wake();
    return id;
  }

  bool unregister_app(AppId id) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    launchers_.erase(id);
    const bool ok = model_.unregister_app(id);
    if (ok)
      wake();
    return ok;
  }

  /// @brief Launch an app from the firmware (its launch callback runs on the
  ///        desktop task; a single-instance app that is open is focused).
  void launch(AppId id) { submit({.sink = 0, .request = detail::dp::LaunchApp{.app = id}}); }

  // ---- windows ----

  /// @brief Open a window (sent at the next flush).
  Window create_window(WindowConfig cfg) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const WindowId id = model_.create_window(std::move(cfg));
    wake();
    return id ? Window(this, id) : Window();
  }

  /// @brief Close a window; its on_close runs on the desktop task.
  bool close_window(WindowId id, WindowCloseReason reason = WindowCloseReason::App) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto on_close = model_.close_window(id, reason);
    if (!on_close)
      return false;
    forget_window(id);
    if (*on_close)
      posted_.push_back(std::move(*on_close));
    wake();
    return true;
  }

  /// @brief The open windows (of one app, or every app when id == 0).
  std::vector<Window> windows(AppId app = 0) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const auto ids = model_.windows_of(app);
    std::vector<Window> out{};
    out.reserve(ids.size());
    std::transform(ids.begin(), ids.end(), std::back_inserter(out),
                   [this](WindowId id) { return Window(this, id); });
    return out;
  }

  Window window(WindowId id) { return Window(this, id); }

  // ---- dialogs / notifications ----

  /// @brief Open a message box; returns its id, or 0 (logged) when it would
  ///        not fit one frame (max_payload(): title / text / buttons too long).
  DialogId message_box(MessageBoxConfig cfg) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto fn = std::move(cfg.on_result);
    const std::string title = cfg.title;
    const DialogId id = model_.open_dialog(
        {.owner = cfg.owner,
         .kind = DialogKind::Message,
         .icon = static_cast<uint8_t>(cfg.icon),
         .title = std::move(cfg.title),
         .text = std::move(cfg.text),
         .buttons = std::move(cfg.buttons)},
        [fn = std::move(fn)](const detail::dp::DialogResult &r) {
          if (fn)
            fn(r.button == detail::dp::kDialogDismissed ? -1 : static_cast<int>(r.button));
        });
    if (!id) {
      logger_.error("message box '{}' does not fit one frame ({} bytes); not shown", title,
                    max_payload());
      return 0;
    }
    wake();
    return id;
  }

  /// @brief Open an input box; returns its id, or 0 (logged) when it would
  ///        not fit one frame (max_payload()).
  DialogId input_box(InputBoxConfig cfg) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto fn = std::move(cfg.on_result);
    const std::string title = cfg.title;
    const DialogId id = model_.open_dialog({.owner = cfg.owner,
                                            .kind = DialogKind::Input,
                                            .icon = static_cast<uint8_t>(cfg.icon),
                                            .title = std::move(cfg.title),
                                            .text = std::move(cfg.text),
                                            .default_text = std::move(cfg.default_text),
                                            .buttons = std::move(cfg.buttons)},
                                           [fn = std::move(fn)](const detail::dp::DialogResult &r) {
                                             if (!fn)
                                               return;
                                             if (r.button == 0)
                                               fn(r.text);
                                             else
                                               fn(std::nullopt);
                                           });
    if (!id) {
      logger_.error("input box '{}' does not fit one frame ({} bytes); not shown", title,
                    max_payload());
      return 0;
    }
    wake();
    return id;
  }

  /// @brief Close a dialog from the firmware (its callback does not run).
  bool close_dialog(DialogId id) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const bool ok = model_.close_dialog(id, true).has_value();
    if (ok)
      wake();
    return ok;
  }

  /// @brief Show a toast; false (logged) when it would not fit one frame.
  bool notify(NotifyConfig cfg) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const std::string title = cfg.title;
    const bool ok = model_.notify(
        {.level = cfg.level,
         .timeout_ms = static_cast<uint16_t>(std::min<int64_t>(cfg.timeout.count(), 65535)),
         .title = std::move(cfg.title),
         .text = std::move(cfg.text)});
    if (!ok) {
      logger_.error("notification '{}' does not fit one frame ({} bytes); not shown", title,
                    max_payload());
      return false;
    }
    wake();
    return true;
  }

  // ---- scheduling ----

  /// @brief Run a function on the desktop task (soon).
  void post(std::function<void()> fn) {
    if (!fn)
      return;
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    posted_.push_back(std::move(fn));
    wake();
  }

  /// @brief A periodic callback on the desktop task; `owner` (a window id)
  ///        cancels it when that window closes. Returns the timer id.
  TimerId add_timer(std::chrono::milliseconds period, std::function<void()> fn,
                    WindowId owner = 0) {
    if (!fn || period.count() <= 0)
      return 0;
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const TimerId id = next_timer_++;
    timers_.push_back({.id = id,
                       .due = std::chrono::steady_clock::now() + period,
                       .period = period,
                       .fn = std::move(fn),
                       .owner = owner});
    wake();
    return id;
  }

  bool cancel_timer(TimerId id) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const size_t n = timers_.size();
    std::erase_if(timers_, [id](const TimerEntry &t) { return t.id == id; });
    return timers_.size() != n;
  }

  /// @brief Whether the caller is the desktop task (where app callbacks run).
  bool on_desktop_task() const { return std::this_thread::get_id() == task_thread_.load(); }

  // ---- desktop settings ----

  void set_theme(std::string_view theme) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    model_.theme = std::string(theme);
    model_.mark_desktop_changed();
    wake();
  }
  void set_accent(uint32_t rgb) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    model_.accent = rgb;
    model_.mark_desktop_changed();
    wake();
  }
  /// @brief Rename the device (at most kMaxDeviceNameBytes, else truncated).
  void set_device_name(std::string_view name) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    model_.device_name = truncated("device_name", name, kMaxDeviceNameBytes);
    model_.mark_desktop_changed();
    wake();
  }
  std::string theme() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return model_.theme;
  }

  // ---- sinks / transport (used by DesktopService) ----

  /// @brief Register a transport. Events are sent to it once a GET_DESKTOP
  ///        arrived through it (set_sink_active). Returns its id.
  /// @param module The dispatcher module id stamped on the frames sent to it.
  SinkId add_sink(send_fn send, uint8_t module = detail::dp::kModule) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto s = std::make_shared<Sink>();
    s->id = next_sink_++;
    s->module = module;
    s->send = std::move(send);
    sinks_.push_back(s);
    return s->id;
  }

  void remove_sink(SinkId id) {
    std::shared_ptr<Sink> gone;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      const auto it = std::find_if(sinks_.begin(), sinks_.end(),
                                   [id](const std::shared_ptr<Sink> &s) { return s->id == id; });
      if (it != sinks_.end()) {
        gone = *it;
        sinks_.erase(it);
      }
    }
    if (gone) {
      // wait for a send in flight on it, then drop the callback
      std::lock_guard<std::mutex> send_lock(gone->mutex);
      gone->send = nullptr;
    }
  }

  /// @brief Start / stop broadcasting events to a sink (a disconnected
  ///        transport should be deactivated; the next GET_DESKTOP reactivates it).
  // cppcheck-suppress functionConst // mutates the sink (through its shared_ptr): not a const
  // operation
  void set_sink_active(SinkId id, bool active) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (auto s = find_sink(id))
      s->active = active;
  }

  bool sink_active(SinkId id) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto s = find_sink(id);
    return s && s->active;
  }

  /// @brief Queue a decoded host request for the desktop task. When the queue
  ///        is full (Config::max_queued_commands) a request is refused with
  ///        ERROR(EAGAIN) on its sink and an event is dropped (logged); what
  ///        is already queued is never evicted.
  void submit(Command cmd) {
    std::shared_ptr<Sink> reject_sink;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (commands_.size() < config_.max_queued_commands) {
        commands_.push_back(std::move(cmd));
        wake();
        return;
      }
      if (is_event(cmd)) {
        logger_.warn_rate_limited("command queue full ({}); dropping an event", commands_.size());
        return;
      }
      reject_sink = find_sink(cmd.sink);
    }
    logger_.warn_rate_limited("command queue full ({}); refusing a request (type 0x{:02x})",
                              config_.max_queued_commands, static_cast<uint8_t>(request_type(cmd)));
    if (!reject_sink)
      return;
    std::vector<OutMsg> out;
    out.push_back({.type = detail::dp::Type::Error,
                   .payload = detail::dp::encode_error(
                       static_cast<uint8_t>(request_type(cmd)),
                       static_cast<uint32_t>(
                           std::make_error_code(std::errc::resource_unavailable_try_again).value()),
                       "desktop command queue full; try again"),
                   .correlation = cmd.correlation});
    send(reject_sink, out);
  }

  /// @brief Whether a frame to this sink was dropped since its last
  ///        GET_DESKTOP (the host's mirror is incomplete until it resyncs).
  bool sink_needs_resync(SinkId id) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto s = find_sink(id);
    return s && s->needs_resync;
  }

  /// @brief The bound on a TextArea's retained text (Config::max_text_bytes).
  size_t max_text_bytes() const { return config_.max_text_bytes; }

  /// @brief The largest payload sent / accepted (Config::max_frame_bytes less
  ///        the frame overhead, at most the codec's limit).
  size_t max_payload() const { return detail::dp::max_payload_for(config_.max_frame_bytes); }

  // ---- model access (any task; used by the handles) ----

  bool window_exists(WindowId id) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return model_.window(id) != nullptr;
  }
  bool widget_exists(WindowId win, WidgetId id) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return model_.widget(win, id) != nullptr;
  }
  template <typename Fn> bool with_window(WindowId id, Fn fn) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const auto *w = model_.window(id);
    if (!w)
      return false;
    fn(*w);
    return true;
  }
  template <typename Fn> bool with_widget(WindowId win, WidgetId id, Fn fn) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const auto *w = model_.widget(win, id);
    if (!w)
      return false;
    fn(*w);
    return true;
  }

  WidgetId add_widget(WindowId win, WidgetConfig cfg) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const WidgetId id = model_.add_widget(win, std::move(cfg));
    if (!id)
      logger_.warn("add_widget: unknown window {} / parent (or parent not a container)", win);
    else
      wake();
    return id;
  }
  bool remove_widget(WindowId win, WidgetId id) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const bool ok = model_.remove_widget(win, id);
    if (ok)
      wake();
    return ok;
  }
  void clear_window(WindowId win) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    model_.clear_window(win);
    wake();
  }
  bool set_prop(WindowId win, WidgetId widget, detail::dp::Prop prop) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const bool ok = model_.set_prop(win, widget, std::move(prop));
    if (ok)
      wake();
    return ok;
  }
  bool append_text(WindowId win, WidgetId widget, std::string_view text) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const bool ok = model_.append_text(win, widget, text);
    if (ok)
      wake();
    return ok;
  }
  bool set_widget_handler(WindowId win, WidgetId id, widget_event_fn fn) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto *w = model_.widget(win, id);
    if (!w)
      return false;
    w->on_event = std::move(fn);
    return true;
  }

protected:
  using Model = detail::desktop_model::Model;

  struct Sink {
    SinkId id{0};
    uint8_t module{detail::dp::kModule};
    send_fn send{nullptr};
    std::mutex mutex; ///< held across `send` so this sink's frames never interleave
    bool active{false};
    std::atomic<bool> needs_resync{false}; ///< a frame was dropped since the last GET_DESKTOP
  };

  /// The wire type of a command's request (for an ERROR reply).
  static detail::dp::Type request_type(const Command &cmd) {
    using T = detail::dp::Type;
    switch (cmd.request.index()) {
    case 0:
      return T::GetDesktop;
    case 1:
      return T::LaunchApp;
    case 2:
      return T::CloseWindow;
    case 3:
      return T::WindowEvent;
    case 4:
      return T::WidgetEvent;
    default:
      return T::DialogResult;
    }
  }
  /// Events are not acknowledged (and may be dropped); requests are answered.
  static bool is_event(const Command &cmd) { return cmd.request.index() >= 3; }

  /// `s` cut to `limit` bytes on a UTF-8 boundary (logged when it was longer).
  std::string truncated(std::string_view what, std::string_view s, size_t limit) {
    if (s.size() <= limit)
      return std::string(s);
    size_t n = limit;
    while (n > 0 && (static_cast<uint8_t>(s[n]) & 0xC0) == 0x80)
      --n;
    logger_.warn("{} longer than {} bytes; truncated", what, limit);
    return std::string(s.substr(0, n));
  }

  struct TimerEntry {
    TimerId id{0};
    std::chrono::steady_clock::time_point due{};
    std::chrono::milliseconds period{0};
    std::function<void()> fn{};
    WindowId owner{0};
  };

  /// A message to send (its frame is built per sink, with the sink's module).
  struct OutMsg {
    detail::dp::Type type;
    std::vector<uint8_t> payload{};
    std::optional<uint16_t> correlation{}; ///< replies echo the request's
  };

  /// A model message as an outgoing (event) message; the payload is moved out.
  static OutMsg to_out_msg(detail::dp::Message &m) {
    return {.type = m.type, .payload = std::move(m.payload)};
  }

  std::shared_ptr<Sink> find_sink(SinkId id) const {
    const auto it = std::find_if(sinks_.begin(), sinks_.end(),
                                 [id](const std::shared_ptr<Sink> &s) { return s->id == id; });
    return it != sinks_.end() ? *it : nullptr;
  }

  /// Wake the desktop task (with mutex_ held or not).
  void wake() {
    std::mutex *m = wake_mutex_.load();
    if (!m)
      return; // the task has not run yet: its first iteration drains everything
    {
      std::lock_guard<std::mutex> lock(*m);
      *wake_flag_ = true;
    }
    wake_cv_.load()->notify_all();
  }

  /// Drop everything that referenced a closed window (timers, text buffers).
  void forget_window(WindowId id) {
    std::erase_if(timers_, [id](const TimerEntry &t) { return t.owner == id; });
    text_.forget_window(id);
  }

  /// One iteration of the desktop task.
  bool step(std::mutex &m, std::condition_variable &cv, bool &notified) {
    if (!wake_mutex_.load()) {
      task_thread_.store(std::this_thread::get_id());
      wake_flag_ = &notified;
      wake_cv_.store(&cv);
      wake_mutex_.store(&m); // published last: wake() needs the other two
    }
    run_pending();
    flush_if_due();
    // wait for the next due time (timer, flush) or a wake-up
    std::chrono::steady_clock::time_point next = std::chrono::steady_clock::time_point::max();
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!commands_.empty() || !posted_.empty())
        next = std::chrono::steady_clock::now();
      if (!timers_.empty())
        next = std::min(next, std::min_element(timers_.begin(), timers_.end(),
                                               [](const TimerEntry &a, const TimerEntry &b) {
                                                 return a.due < b.due;
                                               })
                                  ->due);
      if (model_.dirty.any())
        next = std::min(next, last_flush_ + config_.flush_period);
    }
    std::unique_lock<std::mutex> lock(m);
    if (next == std::chrono::steady_clock::time_point::max())
      cv.wait(lock, [&notified] { return notified; });
    else
      cv.wait_until(lock, next, [&notified] { return notified; });
    notified = false; // consumed, under the mutex, per the Task contract
    return false;     // keep running until stopped
  }

  /// Drain host commands, posted functions and due timers; each unit of work
  /// is pulled under the lock and run outside it.
  void run_pending() {
    for (int guard = 0; guard < 1000; ++guard) {
      std::optional<Command> cmd{};
      std::function<void()> fn{};
      {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (!commands_.empty()) {
          cmd = std::move(commands_.front());
          commands_.pop_front();
        } else if (!posted_.empty()) {
          fn = std::move(posted_.front());
          posted_.pop_front();
        } else {
          const auto now = std::chrono::steady_clock::now();
          auto it = std::min_element(
              timers_.begin(), timers_.end(),
              [](const TimerEntry &a, const TimerEntry &b) { return a.due < b.due; });
          if (it == timers_.end() || it->due > now)
            return;
          fn = it->fn; // copied: the timer may cancel itself
          it->due = now + it->period;
        }
      }
      if (cmd)
        handle_command(std::move(*cmd));
      else if (fn)
        fn();
    }
  }

  void handle_command(Command cmd) {
    std::visit([&](auto &req) { handle(cmd, req); }, cmd.request);
  }

  void handle(const Command &cmd, const GetDesktop &) {
    // send what is pending to the sinks that were active, then the snapshot
    // to the requesting one (which becomes active)
    flush_now();
    std::shared_ptr<Sink> sink;
    std::vector<OutMsg> out{};
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      sink = find_sink(cmd.sink);
      if (!sink)
        return;
      sink->active = true;
      sink->needs_resync.store(false); // this reply is the resync
      const bool has_snapshot = !model_.windows().empty() || !model_.dialogs().empty();
      bool trimmed = false;
      out.push_back({.type = detail::dp::Type::Desktop,
                     .payload = detail::dp::encode_desktop(model_.desktop_info(has_snapshot),
                                                           max_payload(), &trimmed),
                     .correlation = cmd.correlation});
      if (trimmed)
        logger_.warn("DESKTOP record set trimmed to fit {} bytes", max_payload());
      size_t dropped = 0;
      auto msgs = model_.snapshot(&dropped);
      std::transform(msgs.begin(), msgs.end(), std::back_inserter(out), to_out_msg);
      if (dropped)
        logger_.warn("snapshot: {} oversized properties dropped", dropped);
    }
    send(sink, out);
    logger_.debug("GET_DESKTOP from sink {}: {} frames", cmd.sink, out.size());
  }

  void handle(const Command &cmd, const detail::dp::LaunchApp &req) {
    // decide under the lock, send after it (send never runs with mutex_ held)
    std::function<void(Desktop &, AppId)> launch_fn{};
    bool found = false, focused = false;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      const auto *app = model_.app(req.app);
      found = app != nullptr;
      if (found && (app->flags & detail::dp::kAppSingleInstance)) {
        const auto open = model_.windows_of(req.app);
        if (!open.empty()) {
          model_.set_prop(open.front(), 0, detail::dp::Prop::u8(detail::dp::PropTag::Focus, 1));
          wake();
          focused = true;
        }
      }
      if (found && !focused) {
        const auto it = launchers_.find(req.app);
        if (it != launchers_.end())
          launch_fn = it->second;
      }
    }
    if (!found) {
      reply_error(cmd, detail::dp::Type::LaunchApp, std::errc::no_such_file_or_directory,
                  "no such app");
      return;
    }
    reply_ok(cmd, detail::dp::Type::LaunchApp);
    if (focused)
      return;
    if (launch_fn)
      launch_fn(*this, req.app);
    else
      logger_.warn("app {} has no launch function", req.app);
  }

  void handle(const Command &cmd, const detail::dp::CloseWindow &req) {
    std::function<void()> on_close{};
    bool found = false;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      auto fn = model_.close_window(req.window, WindowCloseReason::Host);
      found = fn.has_value();
      if (found) {
        forget_window(req.window);
        on_close = std::move(*fn);
      }
    }
    if (!found) {
      reply_error(cmd, detail::dp::Type::CloseWindow, std::errc::no_such_file_or_directory,
                  "no such window");
      return;
    }
    reply_ok(cmd, detail::dp::Type::CloseWindow);
    if (on_close)
      on_close();
  }

  void handle(const Command &, const detail::dp::WindowEvent &e) {
    window_event_fn fn;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!model_.apply_window_event(e))
        return;
      fn = model_.window(e.window)->on_event;
    }
    if (fn)
      fn(e);
  }

  void handle(const Command &, const detail::dp::WidgetEvent &e) {
    widget_event_fn fn;
    WidgetEvent ev = e;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      auto *w = model_.widget(e.window, e.widget);
      if (!w)
        return;
      if (e.kind == WidgetEventKind::Text) {
        std::string full{};
        switch (text_.feed(e.window, e.widget, e, full)) {
        case detail::desktop_model::TextAssembler::Result::Partial:
          return;
        case detail::desktop_model::TextAssembler::Result::Rejected:
          logger_.warn_rate_limited("text chunk rejected (window {} widget {} offset {} total {})",
                                    e.window, e.widget, e.text_offset, e.text_total);
          return;
        case detail::desktop_model::TextAssembler::Result::Complete:
          ev.text = std::move(full);
          ev.text_offset = 0;
          break;
        }
      }
      model_.apply_widget_event(ev);
      fn = w->on_event;
    }
    if (fn)
      fn(ev);
  }

  void handle(const Command &, const detail::dp::DialogResult &r) {
    std::function<void(const detail::dp::DialogResult &)> fn{};
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      auto f = model_.close_dialog(r.dialog, false); // the host closed its own UI
      if (!f)
        return;
      fn = std::move(*f);
    }
    if (fn)
      fn(r);
  }

  void reply_ok(const Command &cmd, detail::dp::Type request) {
    reply(cmd, {.type = detail::dp::Type::Ok,
                .payload = detail::dp::encode_ok(static_cast<uint8_t>(request)),
                .correlation = cmd.correlation});
  }

  void reply_error(const Command &cmd, detail::dp::Type request, std::errc errc,
                   std::string_view message) {
    logger_.warn("{} (type 0x{:02x})", message, static_cast<uint8_t>(request));
    reply(cmd, {.type = detail::dp::Type::Error,
                .payload = detail::dp::encode_error(
                    static_cast<uint8_t>(request),
                    static_cast<uint32_t>(std::make_error_code(errc).value()), message),
                .correlation = cmd.correlation});
  }

  /// Send one reply to the command's sink (sink 0 = a firmware-internal
  /// command: nothing to answer).
  void reply(const Command &cmd, OutMsg msg) {
    if (!cmd.sink)
      return;
    std::shared_ptr<Sink> sink;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      sink = find_sink(cmd.sink);
    }
    if (!sink)
      return;
    std::vector<OutMsg> out{};
    out.push_back(std::move(msg));
    send(sink, out);
  }

  /// Flush when the period elapsed (or nothing was sent recently).
  void flush_if_due() {
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!model_.dirty.any())
        return;
      if (std::chrono::steady_clock::now() < last_flush_ + config_.flush_period)
        return;
    }
    flush_now();
  }

  /// Encode everything pending (under the lock) and send it to every active
  /// sink (outside it). With no active sink the changes are simply dropped:
  /// the next GET_DESKTOP replays the full state anyway.
  void flush_now() {
    std::vector<OutMsg> out{};
    std::vector<std::shared_ptr<Sink>> targets{};
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!model_.dirty.any())
        return;
      size_t dropped = 0;
      auto msgs = model_.flush(&dropped);
      std::transform(msgs.begin(), msgs.end(), std::back_inserter(out), to_out_msg);
      if (dropped)
        logger_.warn("flush: {} oversized properties dropped (max payload {})", dropped,
                     max_payload());
      last_flush_ = std::chrono::steady_clock::now();
      std::copy_if(sinks_.begin(), sinks_.end(), std::back_inserter(targets),
                   [](const std::shared_ptr<Sink> &s) { return s->active; });
    }
    if (out.empty())
      return;
    for (const auto &s : targets)
      send(s, out);
  }

  /// Build the frames for one sink (with its module id) and transmit them,
  /// serialized on the sink's mutex (held across the callback). Never called
  /// with mutex_ held.
  void send(const std::shared_ptr<Sink> &sink, const std::vector<OutMsg> &out) {
    if (!sink)
      return;
    std::lock_guard<std::mutex> lock(sink->mutex);
    if (!sink->send)
      return;
    for (const auto &m : out) {
      const auto frame = detail::dp::build_frame(m.type, m.payload, sink->module, m.correlation);
      if (frame.empty()) {
        logger_.warn_rate_limited("dropping an oversized frame ({} bytes of payload)",
                                  m.payload.size());
        continue;
      }
      if (!sink->send(frame) && !sink->needs_resync.exchange(true))
        logger_.warn("sink {} could not take a {}-byte frame; the host must resync "
                     "(GET_DESKTOP)",
                     sink->id, frame.size());
    }
  }

private:
  Config config_;
  mutable std::recursive_mutex mutex_; ///< the model, queues, timers, sinks
  Model model_;
  detail::desktop_model::TextAssembler text_;
  std::map<AppId, std::function<void(Desktop &, AppId)>> launchers_{};
  std::deque<Command> commands_{};
  std::deque<std::function<void()>> posted_{};
  std::vector<TimerEntry> timers_{};
  std::vector<std::shared_ptr<Sink>> sinks_{};
  SinkId next_sink_{1};
  TimerId next_timer_{1};
  std::chrono::steady_clock::time_point last_flush_{};
  // the desktop task's wait primitives (captured on its first iteration)
  std::atomic<std::mutex *> wake_mutex_{nullptr};
  std::atomic<std::condition_variable *> wake_cv_{nullptr};
  bool *wake_flag_{nullptr};
  std::atomic<std::thread::id> task_thread_{};
  std::unique_ptr<Task> task_; ///< last: destroyed (stopped) first
};

} // namespace espp

#pragma once

// Wire protocol of espp::DesktopService / espp::Desktop: a browser-rendered
// windowed desktop (apps, windows, widgets, dialogs, notifications) over the
// espp stream_frame codec, routed by an espp::Dispatcher on module 9 by
// default (`espp.desktop` v1 through discovery).
//
// This header is deliberately host-buildable (stream_frame.hpp + the standard
// library only) so the codec is unit-tested on the host
// (components/desktop/test/desktop_host_test.cpp against the shared fixture
// test/desktop_vectors.txt, which the web app's node test reads too) and so
// host tools can reuse it. All multi-byte fields are little-endian.
//
// Primitive encodings:
//   str8  = [len u8][utf8]          (encoders truncate at 255 bytes)
//   str16 = [len u16][utf8]         (encoders truncate at 65535 bytes)
//   rec   = [tag u8][len u16][value] (a decoder skips tags it does not know;
//                                     a value running past its container is
//                                     malformed)
//
// Requests (host -> device, reply flag clear):
//   0x01 GET_DESKTOP   (no payload) -> DESKTOP reply, then one WINDOW_OPEN
//                      (flag Snapshot) per open window, then one DIALOG per
//                      open dialog. Marks the requesting sink active.
//   0x02 LAUNCH_APP    [app u8] -> OK / ERROR(ENOENT). A single-instance app
//                      that is already open answers OK and focuses its window.
//   0x03 CLOSE_WINDOW  [win u16] -> OK / ERROR(ENOENT), then WINDOW_CLOSE(reason 1)
//   0x04 WINDOW_EVENT  [win u16][ev u8][x i16][y i16][w u16][h u16] (no ack)
//   0x05 WIDGET_EVENT  [win u16][widget u16][ev u8][value...] (no ack; see WidgetEventKind)
//   0x06 DIALOG_RESULT [dialog u16][button u8 (0xFF dismissed)][text utf8 rest] (no ack)
// Replies / events (device -> host, high bit set = frame reply flag):
//   0x81 DESKTOP       [proto u8 = 1][flags u8][rec count u8]{rec}[app count u8]{app}
//                      [win count u8]{[win u16][app u8]}
//                      app = [id u8][flags u8][name str8][icon str8][desc str8]
//   0x82 WINDOW_OPEN   [win u16][app u8][flags u16][x i16][y i16][w u16][h u16]
//                      [title str8][widget total u16][count u16]{widget rec}
//                      (the rest of the tree follows in WIDGET_ADD frames until
//                      `total` widgets were received)
//   0x83 WINDOW_CLOSE  [win u16][reason u8]
//   0x84 WIDGET_SET    [win u16][entries u8]{[widget u16][props u8]{rec}}
//                      (widget 0 = the window itself: Title / WindowFlags /
//                      Geometry / Focus)
//   0x85 WIDGET_ADD    [win u16][count u16]{widget rec} (appended to the parent;
//                      an InsertBefore prop positions it)
//   0x86 WIDGET_REMOVE [win u16][count u16]{widget u16} (children go too)
//   0x87 DIALOG        [dialog u16][owner win u16 (0 = desktop)][kind u8][icon u8]
//                      [title str8][text str16][default str16][buttons u8]{str8}
//   0x88 NOTIFY        [level u8][timeout ms u16 (0 sticky)][title str8][text str16]
//   0x89 DIALOG_CLOSE  [dialog u16]
//   0x8E OK            [request u8]
//   0x8F ERROR         [request u8][errno u32][utf8 message]
//   widget rec = [id u16][parent u16 (0 = window root)][type u8][weight u8]
//                [layout u8][props u8]{rec}
// Replies (DESKTOP, OK, ERROR) echo the request frame's correlation id; events
// carry none. Every payload is at most the negotiated max payload (DESKTOP
// record MaxPayload, <= 4081): the widget encoders below SPLIT across frames
// (a long Text becomes Text + TextAppend pieces, an Items range becomes
// several ranges, a tree continues in WIDGET_ADD) and never truncate. DIALOG
// and NOTIFY are single frames: the Desktop API refuses one that would not
// fit. DESKTOP is a single frame too, with one invariant: the records and the
// FULL app list always fit the selected cap (the registry is bounded --
// kMaxApps, kMaxApp*Bytes, kMaxDeviceNameBytes -- and Desktop::register_app
// refuses an app that would break the fit even with every description
// empty); the only elastic parts are the window list (trimmed first, and
// then flags bit1 WindowListComplete is clear so the host does not reconcile
// against it) and the app descriptions. Apps are NEVER trimmed: a host
// replaces its app registry on every DESKTOP frame. Unsplittable widget
// properties (Title / Placeholder / Tooltip str8-sized, Columns, a single
// Items entry) are bounded at the Desktop API so the model never holds a
// value the wire cannot carry.

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <iterator>
#include <numeric>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "stream_frame.hpp"

namespace espp::detail::desktop_protocol {

/// Default dispatcher module id (a routing key only; see DesktopService::Config::module).
inline constexpr uint8_t kModule = 9;
/// Stable protocol identifier + version advertised through discovery.
inline constexpr const char *kProtocol = "espp.desktop";
inline constexpr uint16_t kProtocolVersion = 1;
/// The `proto` byte at the head of every DESKTOP payload.
inline constexpr uint8_t kDesktopProto = 1;

/// Registry limits that keep DESKTOP a single frame (enforced by
/// Desktop::register_app / Desktop::Config, documented in the README): 24
/// apps of maximal size (2 + 3 + 32 + 16 + 64 = 117 bytes each = 2808), the
/// records (<= 64 + 64 + 8 + 4 + 2 + 2 + 6 * 3 = 162) and the counts leave a
/// 4081-byte payload room for ~360 open windows (3 bytes each).
inline constexpr size_t kMaxApps = 24;
inline constexpr size_t kMaxAppNameBytes = 32;
inline constexpr size_t kMaxAppIconBytes = 16;
inline constexpr size_t kMaxAppDescriptionBytes = 64;
inline constexpr size_t kMaxDeviceNameBytes = 64;
inline constexpr size_t kMaxFirmwareBytes = 64;
/// Theme values ("auto" | "light" | "dark"); the longest is 5 bytes.
inline constexpr size_t kMaxThemeBytes = 5;
/// str8 / str16 capacities: a value longer than these is not representable
/// (the Desktop API refuses it rather than cut it).
inline constexpr size_t kMaxStr8Bytes = 255;
inline constexpr size_t kMaxStr16Bytes = 65535;
/// Unsplittable text properties (window Title, Placeholder, Tooltip) are
/// bounded like a str8 so they always fit a frame next to their siblings.
inline constexpr size_t kMaxShortTextBytes = 255;
/// The DESKTOP record set at its largest (every record at its limit, no apps,
/// no windows): 3-byte head + 6 recs with 3-byte headers + the two counts.
inline constexpr size_t kDesktopRecordsMaxBytes = 3 + (3 + kMaxDeviceNameBytes) +
                                                  (3 + kMaxFirmwareBytes) + (3 + kMaxThemeBytes) +
                                                  (3 + 4) + (3 + 2) + (3 + 2) + 1 + 1;
/// The largest payloads the API lets through that cannot be split, each of
/// which must fit the smallest cap on its own:
///  - a WINDOW_OPEN head with a kMaxStr8Bytes title and no widgets
///    (13 fixed bytes + str8 + total + count),
///  - a WIDGET_ADD frame holding one widget record whose single property is a
///    kMaxShortTextBytes Placeholder / Tooltip (frame head + base + rec),
///  - a WIDGET_SET frame holding one such property (frame head + entry head + rec),
///  - the maximal DESKTOP record set.
/// (Columns / Items entries are bounded against the selected cap by the model;
/// DIALOG / NOTIFY are checked against the selected cap at the API.)
inline constexpr size_t kWindowOpenHeadMaxBytes = 13 + 1 + kMaxStr8Bytes + 2 + 2;
inline constexpr size_t kWidgetAddMaxUnsplittableBytes = 4 + 8 + 3 + kMaxShortTextBytes;
inline constexpr size_t kWidgetSetMaxUnsplittableBytes = 3 + 3 + 3 + kMaxShortTextBytes;
/// Smallest payload cap a Desktop accepts (Desktop::kMinFrameBytes derives from
/// it): the largest of the unsplittable payloads above.
inline constexpr size_t kMinPayloadBytes =
    std::max({kWindowOpenHeadMaxBytes, kWidgetAddMaxUnsplittableBytes,
              kWidgetSetMaxUnsplittableBytes, kDesktopRecordsMaxBytes});
static_assert(kDesktopRecordsMaxBytes <= kMinPayloadBytes,
              "the DESKTOP record set must always fit the smallest payload cap");
/// The smallest app record (empty name / icon / description): id, flags and
/// three str8 lengths. At least one such app always registers at the minimum
/// cap; a maximal registry (kMaxApps apps at every limit) plus 255 windows
/// fits the default 4096-byte frame.
inline constexpr size_t kAppRecMinBytes = 2 + 3;
inline constexpr size_t kAppRecMaxBytes =
    2 + 3 + kMaxAppNameBytes + kMaxAppIconBytes + kMaxAppDescriptionBytes;
static_assert(kDesktopRecordsMaxBytes + kAppRecMinBytes <= kMinPayloadBytes,
              "at least one app must register at the smallest payload cap");
static_assert(kDesktopRecordsMaxBytes + kMaxApps * kAppRecMaxBytes + 255 * 3 <=
                  4096 - espp::stream_frame::kMaxHeaderSize - espp::stream_frame::kCrcSize,
              "a maximal registry with 255 open windows must fit the default frame");
static_assert(kWindowOpenHeadMaxBytes <= kMinPayloadBytes &&
                  kWidgetAddMaxUnsplittableBytes <= kMinPayloadBytes &&
                  kWidgetSetMaxUnsplittableBytes <= kMinPayloadBytes,
              "every unsplittable payload the API accepts must fit the smallest cap");

/// Frame `type` values within the desktop module.
enum class Type : uint8_t {
  // host -> device
  GetDesktop = 0x01,
  LaunchApp = 0x02,
  CloseWindow = 0x03,
  WindowEvent = 0x04,
  WidgetEvent = 0x05,
  DialogResult = 0x06,
  // device -> host (high bit set)
  Desktop = 0x81,
  WindowOpen = 0x82,
  WindowClose = 0x83,
  WidgetSet = 0x84,
  WidgetAdd = 0x85,
  WidgetRemove = 0x86,
  Dialog = 0x87,
  Notify = 0x88,
  DialogClose = 0x89,
  Ok = 0x8E,
  Error = 0x8F,
};

/// DESKTOP flags byte.
inline constexpr uint8_t kDesktopHasSnapshot = 0x01; ///< WINDOW_OPEN(Snapshot) frames follow
/// The window list is complete (set by the encoder unless it had to trim it):
/// a host may close windows missing from it only when this bit is set.
inline constexpr uint8_t kDesktopWindowListComplete = 0x02;

/// Tags of the DESKTOP records.
enum class DesktopTag : uint8_t {
  DeviceName = 1,    ///< str
  Firmware = 2,      ///< str
  Theme = 3,         ///< str: "auto" | "light" | "dark"
  Accent = 4,        ///< u32 RGB (0xRRGGBB)
  MaxPayload = 5,    ///< u16: largest payload the device sends / accepts
  FlushPeriodMs = 6, ///< u16: how often the device coalesces + flushes changes
};

/// App record flags.
inline constexpr uint8_t kAppSingleInstance = 0x01; ///< LAUNCH_APP focuses an open window instead
inline constexpr uint8_t kAppHidden = 0x02;         ///< not shown on the desktop / start menu

/// Widget types.
enum class WidgetType : uint8_t {
  Column = 1,
  Row = 2,
  Group = 3,
  Label = 4,
  Button = 5,
  Checkbox = 6,
  TextBox = 7,
  TextArea = 8,
  List = 9,
  Table = 10,
  Select = 11,
  Progress = 12,
  Slider = 13,
  Separator = 14,
  Spacer = 15,
};

/// Widget `layout` bits (the box model: weight = flex-grow along the parent's
/// axis, these bits set the cross-axis behaviour).
inline constexpr uint8_t kLayoutStretch = 0x01;     ///< stretch across the cross axis
inline constexpr uint8_t kLayoutScroll = 0x02;      ///< scroll instead of grow
inline constexpr uint8_t kLayoutAlignEnd = 0x04;    ///< align to the end of the cross axis
inline constexpr uint8_t kLayoutAlignCenter = 0x08; ///< center on the cross axis

/// Widget property tags (rec tags in widget recs and WIDGET_SET entries).
enum class PropTag : uint8_t {
  Text = 1,       ///< str (replace)
  TextAppend = 2, ///< str (append)
  Value = 3, ///< i32: checkbox 0/1, slider, progress, select index, list/table selection (-1 none)
  Min = 4,   ///< i32
  Max = 5,   ///< i32
  Step = 6,  ///< i32
  Enabled = 7,       ///< u8
  Visible = 8,       ///< u8
  Color = 9,         ///< u32 RGB (0xFFFFFFFF = theme default)
  Background = 10,   ///< u32 RGB (0xFFFFFFFF = theme default)
  ItemCount = 11,    ///< u16 (truncates / extends the item list)
  Items = 12,        ///< [start u16][count u16]{str16} (table rows: cells '\t'-separated)
  Columns = 13,      ///< [n u8]{str8}
  Placeholder = 14,  ///< str
  Tooltip = 15,      ///< str
  Flags = 16,        ///< u16 (per widget type; see k* below)
  MaxLines = 17,     ///< u16 (TextArea ring; default 500)
  Focus = 18,        ///< u8 (1 = take focus)
  Width = 19,        ///< u16 preferred px
  Height = 20,       ///< u16 preferred px
  InsertBefore = 21, ///< u16 sibling id (WIDGET_ADD only)
  Title = 22,        ///< str (window only, widget 0)
  WindowFlags = 23,  ///< u16 (window only, widget 0)
  Geometry = 24,     ///< [x i16][y i16][w u16][h u16] (window only; -1 / 0 = keep)
};

/// Widget Flags (PropTag::Flags) per type.
inline constexpr uint16_t kTextAreaReadOnly = 0x01;
inline constexpr uint16_t kTextAreaMonospace = 0x02;
inline constexpr uint16_t kTextAreaWantKeys = 0x04;
inline constexpr uint16_t kTextAreaAutoScroll = 0x08;
inline constexpr uint16_t kTextAreaAnsi = 0x10;
inline constexpr uint16_t kTextBoxPassword = 0x01;
inline constexpr uint16_t kTextBoxReadOnly = 0x02;
inline constexpr uint16_t kLabelBold = 0x01;
inline constexpr uint16_t kLabelMonospace = 0x02;
inline constexpr uint16_t kLabelWrap = 0x04;
inline constexpr uint16_t kButtonPrimary = 0x01;
inline constexpr uint16_t kButtonDanger = 0x02;

/// Window flags (WINDOW_OPEN flags / PropTag::WindowFlags).
inline constexpr uint16_t kWinMovable = 1u << 0;
inline constexpr uint16_t kWinResizable = 1u << 1;
inline constexpr uint16_t kWinClosable = 1u << 2;
inline constexpr uint16_t kWinModal = 1u << 3;
inline constexpr uint16_t kWinMinimizable = 1u << 4;
inline constexpr uint16_t kWinMaximizable = 1u << 5;
inline constexpr uint16_t kWinSnapshot = 1u << 6; ///< replay: restore stored state, do not raise
inline constexpr uint16_t kWinCentered = 1u << 7;
inline constexpr uint16_t kWinPinned = 1u << 8;        ///< firmware geometry beats the stored one
inline constexpr uint16_t kWinWantsGeometry = 1u << 9; ///< stream Moved / Resized during gestures
/// The flags a plain application window gets by default.
inline constexpr uint16_t kWinDefaultFlags =
    kWinMovable | kWinResizable | kWinClosable | kWinMinimizable | kWinMaximizable;

/// WINDOW_EVENT kinds.
enum class WindowEventKind : uint8_t {
  Focus = 1,
  Blur = 2,
  Minimize = 3,
  Restore = 4,
  Maximize = 5,
  Moved = 6,
  Resized = 7,
};

/// WIDGET_EVENT kinds and their value layouts.
enum class WidgetEventKind : uint8_t {
  Click = 1,    ///< (none)
  Change = 2,   ///< [i32] Checkbox / Slider / Select
  Submit = 3,   ///< [utf8] TextBox Enter
  Text = 4,     ///< [offset u32][total u32][bytes] chunked to MaxPayload
  Select = 5,   ///< [i32] List / Table selection
  Activate = 6, ///< [i32] double-click / Enter on an item
  Key = 7,      ///< [key u16][mods u8][codepoint u32] (TextArea WantKeys)
  Scroll = 8,   ///< [i32] first visible line
};

/// Key codes carried by WidgetEventKind::Key (printable characters use key 0 +
/// the codepoint).
inline constexpr uint16_t kKeyEnter = 13;
inline constexpr uint16_t kKeyBackspace = 8;
inline constexpr uint16_t kKeyTab = 9;
inline constexpr uint16_t kKeyEscape = 27;
inline constexpr uint16_t kKeyLeft = 0x100;
inline constexpr uint16_t kKeyRight = 0x101;
inline constexpr uint16_t kKeyUp = 0x102;
inline constexpr uint16_t kKeyDown = 0x103;
inline constexpr uint16_t kKeyHome = 0x104;
inline constexpr uint16_t kKeyEnd = 0x105;
inline constexpr uint16_t kKeyPageUp = 0x106;
inline constexpr uint16_t kKeyPageDown = 0x107;
inline constexpr uint16_t kKeyDelete = 0x108;
inline constexpr uint16_t kKeyInsert = 0x109;
inline constexpr uint16_t kKeyF1 = 0x110; ///< F1..F12 = 0x110..0x11B
inline constexpr uint8_t kModShift = 0x01;
inline constexpr uint8_t kModCtrl = 0x02;
inline constexpr uint8_t kModAlt = 0x04;
inline constexpr uint8_t kModMeta = 0x08;

/// WINDOW_CLOSE reasons.
enum class WindowCloseReason : uint8_t {
  App = 0,      ///< the application closed it
  Host = 1,     ///< in answer to CLOSE_WINDOW
  Shutdown = 2, ///< the desktop is going away
};

/// DIALOG kinds.
enum class DialogKind : uint8_t {
  Message = 0,
  Input = 1,
};

/// DIALOG icons.
enum class DialogIcon : uint8_t {
  None = 0,
  Info = 1,
  Question = 2,
  Warning = 3,
  Error = 4,
};

/// DIALOG_RESULT button value when the dialog was dismissed (Escape / close).
inline constexpr uint8_t kDialogDismissed = 0xFF;

/// NOTIFY levels.
enum class NotifyLevel : uint8_t {
  Info = 0,
  Ok = 1,
  Warn = 2,
  Error = 3,
};

/// Whether a type value is a device->host reply / event.
inline constexpr bool is_reply(Type type) { return (static_cast<uint8_t>(type) & 0x80) != 0; }

/// Build an encoded frame for a desktop message (device->host types map to the
/// frame reply flag).
/// @param correlation The stream_frame correlation id to carry: a reply echoes
///        the request's, so a host can pair a reply with its request.
inline std::vector<uint8_t> build_frame(Type type, std::span<const uint8_t> payload = {},
                                        uint8_t module = kModule,
                                        std::optional<uint16_t> correlation = std::nullopt) {
  return espp::stream_frame::build_frame(is_reply(type), module, static_cast<uint8_t>(type),
                                         payload, correlation);
}

/// The largest payload the codec can carry in a frame of `max_frame_bytes`:
/// less the largest header (with a correlation id) and the CRC, never above
/// the stream_frame payload limit. 4096 -> 4081.
inline constexpr size_t max_payload_for(size_t max_frame_bytes) {
  constexpr size_t overhead = espp::stream_frame::kMaxHeaderSize + espp::stream_frame::kCrcSize;
  const size_t cap = max_frame_bytes > overhead ? max_frame_bytes - overhead : 0;
  return cap < espp::stream_frame::kMaxPayloadSize ? cap : espp::stream_frame::kMaxPayloadSize;
}

// ---- primitive writers / reader ----------------------------------------------

inline void put_u8(std::vector<uint8_t> &out, uint8_t v) { out.push_back(v); }
using espp::stream_frame::put_u16;
using espp::stream_frame::put_u32;
inline void put_i16(std::vector<uint8_t> &out, int16_t v) {
  put_u16(out, static_cast<uint16_t>(v));
}
inline void put_i32(std::vector<uint8_t> &out, int32_t v) {
  put_u32(out, static_cast<uint32_t>(v));
}
inline void put_bytes(std::vector<uint8_t> &out, std::string_view s) {
  out.insert(out.end(), s.begin(), s.end());
}
inline void put_bytes(std::vector<uint8_t> &out, std::span<const uint8_t> b) {
  out.insert(out.end(), b.begin(), b.end());
}
/// str8: truncated at 255 bytes.
inline void put_str8(std::vector<uint8_t> &out, std::string_view s) {
  const size_t n = s.size() > 255 ? 255 : s.size();
  put_u8(out, static_cast<uint8_t>(n));
  put_bytes(out, s.substr(0, n));
}
/// str16: truncated at 65535 bytes.
inline void put_str16(std::vector<uint8_t> &out, std::string_view s) {
  const size_t n = s.size() > 65535 ? 65535 : s.size();
  put_u16(out, static_cast<uint16_t>(n));
  put_bytes(out, s.substr(0, n));
}
/// rec: [tag u8][len u16][value] (a value longer than 65535 bytes is a caller
/// bug; the encoders never produce one).
inline void put_rec(std::vector<uint8_t> &out, uint8_t tag, std::span<const uint8_t> value) {
  put_u8(out, tag);
  put_u16(out, static_cast<uint16_t>(value.size()));
  put_bytes(out, value);
}

/// Bounds-checked little-endian reader: every accessor returns a zero value
/// once a read ran past the end, and ok() reports it, so a decoder can read a
/// whole layout and check once.
class Reader {
public:
  explicit Reader(std::span<const uint8_t> p)
      : p_(p) {}
  bool ok() const { return ok_; }
  size_t pos() const { return pos_; }
  size_t remaining() const { return p_.size() - pos_; }
  bool at_end() const { return pos_ == p_.size(); }
  uint8_t u8() {
    if (!need(1))
      return 0;
    return p_[pos_++];
  }
  uint16_t u16() {
    if (!need(2))
      return 0;
    const uint16_t v = espp::stream_frame::get_u16(p_.subspan(pos_));
    pos_ += 2;
    return v;
  }
  uint32_t u32() {
    if (!need(4))
      return 0;
    const uint32_t v = espp::stream_frame::get_u32(p_.subspan(pos_));
    pos_ += 4;
    return v;
  }
  int16_t i16() { return static_cast<int16_t>(u16()); }
  int32_t i32() { return static_cast<int32_t>(u32()); }
  std::span<const uint8_t> bytes(size_t n) {
    if (!need(n))
      return {};
    const auto s = p_.subspan(pos_, n);
    pos_ += n;
    return s;
  }
  std::string str(size_t n) {
    const auto b = bytes(n);
    return std::string(reinterpret_cast<const char *>(b.data()), b.size());
  }
  std::string str8() { return str(u8()); }
  std::string str16() { return str(u16()); }
  /// Everything left (as a string).
  std::string rest() { return str(remaining()); }
  /// Mark the read malformed (a decoder found an inconsistency).
  void fail() { ok_ = false; }

private:
  bool need(size_t n) {
    if (!ok_ || n > p_.size() - pos_) {
      ok_ = false;
      return false;
    }
    return true;
  }
  std::span<const uint8_t> p_;
  size_t pos_{0};
  bool ok_{true};
};

// ---- properties ------------------------------------------------------------------

/// How a property's value is laid out (by tag).
enum class PropKind : uint8_t { Text, I32, U8, U16, U32, Items, Columns, Geometry, Unknown };

inline constexpr PropKind prop_kind(uint8_t tag) {
  switch (static_cast<PropTag>(tag)) {
  case PropTag::Text:
  case PropTag::TextAppend:
  case PropTag::Placeholder:
  case PropTag::Tooltip:
  case PropTag::Title:
    return PropKind::Text;
  case PropTag::Value:
  case PropTag::Min:
  case PropTag::Max:
  case PropTag::Step:
    return PropKind::I32;
  case PropTag::Enabled:
  case PropTag::Visible:
  case PropTag::Focus:
    return PropKind::U8;
  case PropTag::ItemCount:
  case PropTag::Flags:
  case PropTag::MaxLines:
  case PropTag::Width:
  case PropTag::Height:
  case PropTag::InsertBefore:
  case PropTag::WindowFlags:
    return PropKind::U16;
  case PropTag::Color:
  case PropTag::Background:
    return PropKind::U32;
  case PropTag::Items:
    return PropKind::Items;
  case PropTag::Columns:
    return PropKind::Columns;
  case PropTag::Geometry:
    return PropKind::Geometry;
  }
  return PropKind::Unknown;
}

/// A decoded Items value: a range of the widget's item list.
struct ItemsValue {
  uint16_t start{0};
  std::vector<std::string> items{};
  bool operator==(const ItemsValue &) const = default;
};

/// A window geometry (PropTag::Geometry / WINDOW_OPEN). x / y -1 and w / h 0
/// mean "keep / let the browser decide".
struct Geometry {
  int16_t x{-1};
  int16_t y{-1};
  uint16_t w{0};
  uint16_t h{0};
  bool operator==(const Geometry &) const = default;
};

/// One property record: a tag and its raw little-endian value. The typed
/// constructors / accessors implement the per-tag layouts; unknown tags are
/// carried raw so a host can round-trip them.
struct Prop {
  uint8_t tag{0};
  std::vector<uint8_t> value{};

  bool operator==(const Prop &) const = default;

  static Prop text(PropTag tag, std::string_view s) {
    Prop p{.tag = static_cast<uint8_t>(tag)};
    put_bytes(p.value, s);
    return p;
  }
  static Prop i32(PropTag tag, int32_t v) {
    Prop p{.tag = static_cast<uint8_t>(tag)};
    put_i32(p.value, v);
    return p;
  }
  static Prop u8(PropTag tag, uint8_t v) {
    Prop p{.tag = static_cast<uint8_t>(tag)};
    put_u8(p.value, v);
    return p;
  }
  static Prop u16(PropTag tag, uint16_t v) {
    Prop p{.tag = static_cast<uint8_t>(tag)};
    put_u16(p.value, v);
    return p;
  }
  static Prop u32(PropTag tag, uint32_t v) {
    Prop p{.tag = static_cast<uint8_t>(tag)};
    put_u32(p.value, v);
    return p;
  }
  static Prop items(uint16_t start, std::span<const std::string> list) {
    Prop p{.tag = static_cast<uint8_t>(PropTag::Items)};
    put_u16(p.value, start);
    put_u16(p.value, static_cast<uint16_t>(list.size()));
    for (const auto &s : list)
      put_str16(p.value, s);
    return p;
  }
  static Prop columns(std::span<const std::string> cols) {
    Prop p{.tag = static_cast<uint8_t>(PropTag::Columns)};
    const size_t n = cols.size() > 255 ? 255 : cols.size();
    put_u8(p.value, static_cast<uint8_t>(n));
    for (size_t i = 0; i < n; ++i)
      put_str8(p.value, cols[i]);
    return p;
  }
  static Prop geometry(const Geometry &g) {
    Prop p{.tag = static_cast<uint8_t>(PropTag::Geometry)};
    put_i16(p.value, g.x);
    put_i16(p.value, g.y);
    put_u16(p.value, g.w);
    put_u16(p.value, g.h);
    return p;
  }

  PropTag type() const { return static_cast<PropTag>(tag); }
  PropKind kind() const { return prop_kind(tag); }
  bool is(PropTag t) const { return tag == static_cast<uint8_t>(t); }

  std::string_view as_text() const {
    return std::string_view(reinterpret_cast<const char *>(value.data()), value.size());
  }
  std::optional<int32_t> as_i32() const {
    if (value.size() != 4)
      return std::nullopt;
    return static_cast<int32_t>(espp::stream_frame::get_u32(value));
  }
  std::optional<uint8_t> as_u8() const {
    if (value.size() != 1)
      return std::nullopt;
    return value[0];
  }
  std::optional<uint16_t> as_u16() const {
    if (value.size() != 2)
      return std::nullopt;
    return espp::stream_frame::get_u16(value);
  }
  std::optional<uint32_t> as_u32() const {
    if (value.size() != 4)
      return std::nullopt;
    return espp::stream_frame::get_u32(value);
  }
  std::optional<ItemsValue> as_items() const {
    Reader r(value);
    ItemsValue v;
    v.start = r.u16();
    const uint16_t n = r.u16();
    for (uint16_t i = 0; i < n && r.ok(); ++i)
      v.items.push_back(r.str16());
    if (!r.ok() || !r.at_end())
      return std::nullopt;
    return v;
  }
  std::optional<std::vector<std::string>> as_columns() const {
    Reader r(value);
    std::vector<std::string> cols{};
    const uint8_t n = r.u8();
    for (uint8_t i = 0; i < n && r.ok(); ++i)
      cols.push_back(r.str8());
    if (!r.ok() || !r.at_end())
      return std::nullopt;
    return cols;
  }
  std::optional<Geometry> as_geometry() const {
    if (value.size() != 8)
      return std::nullopt;
    Reader r(value);
    Geometry g{};
    g.x = r.i16();
    g.y = r.i16();
    g.w = r.u16();
    g.h = r.u16();
    return g;
  }
  /// Whether the typed value is well-formed for the tag (unknown tags are).
  bool valid() const {
    switch (kind()) {
    case PropKind::Text:
    case PropKind::Unknown:
      return true;
    case PropKind::I32:
      return value.size() == 4;
    case PropKind::U8:
      return value.size() == 1;
    case PropKind::U16:
      return value.size() == 2;
    case PropKind::U32:
      return value.size() == 4;
    case PropKind::Items:
      return as_items().has_value();
    case PropKind::Columns:
      return as_columns().has_value();
    case PropKind::Geometry:
      return value.size() == 8;
    }
    return false;
  }
  /// Encoded size as a rec.
  size_t encoded_size() const { return 3 + value.size(); }
  void encode(std::vector<uint8_t> &out) const { put_rec(out, tag, value); }
};

/// Read `count` recs into props (raw; every tag kept). False on an overrun.
inline bool read_props(Reader &r, size_t count, std::vector<Prop> &props) {
  for (size_t i = 0; i < count; ++i) {
    Prop p;
    p.tag = r.u8();
    const uint16_t len = r.u16();
    const auto v = r.bytes(len);
    if (!r.ok())
      return false;
    p.value.assign(v.begin(), v.end());
    props.push_back(std::move(p));
  }
  return true;
}

// ---- message structs ---------------------------------------------------------------

/// One app in the DESKTOP app list.
struct AppRec {
  uint8_t id{0};
  uint8_t flags{0}; ///< kAppSingleInstance | kAppHidden
  std::string name{};
  std::string icon{}; ///< emoji / short text, or "svg:<name>" from the built-in set
  std::string description{};
  bool operator==(const AppRec &) const = default;
};

/// An open window listed by DESKTOP.
struct WindowRef {
  uint16_t id{0};
  uint8_t app{0};
  bool operator==(const WindowRef &) const = default;
};

/// DESKTOP payload.
struct DesktopInfo {
  uint8_t proto{kDesktopProto};
  uint8_t flags{0};            ///< kDesktopHasSnapshot
  std::vector<Prop> records{}; ///< DesktopTag recs (unknown tags kept raw)
  std::vector<AppRec> apps{};
  std::vector<WindowRef> windows{};
  bool operator==(const DesktopInfo &) const = default;
};

/// A widget record (WINDOW_OPEN / WIDGET_ADD).
struct WidgetRec {
  uint16_t id{0};
  uint16_t parent{0}; ///< 0 = the window's root column
  WidgetType type{WidgetType::Label};
  uint8_t weight{0}; ///< flex-grow along the parent's axis (0 = natural size)
  uint8_t layout{0}; ///< kLayout* bits
  std::vector<Prop> props{};
  bool operator==(const WidgetRec &) const = default;
  /// Encoded size of the base (without props) and of the whole record.
  static constexpr size_t kBaseSize = 8;
  size_t encoded_size() const {
    return std::accumulate(props.begin(), props.end(), kBaseSize,
                           [](size_t n, const Prop &p) { return n + p.encoded_size(); });
  }
};

/// WINDOW_OPEN payload.
struct WindowOpen {
  uint16_t id{0};
  uint8_t app{0};
  uint16_t flags{kWinDefaultFlags};
  Geometry geometry{};
  std::string title{};
  uint16_t total{0}; ///< widgets in the whole tree (the rest arrive in WIDGET_ADD)
  std::vector<WidgetRec> widgets{};
  bool operator==(const WindowOpen &) const = default;
};

/// WINDOW_CLOSE payload.
struct WindowClose {
  uint16_t id{0};
  WindowCloseReason reason{WindowCloseReason::App};
  bool operator==(const WindowClose &) const = default;
};

/// One WIDGET_SET entry.
struct WidgetSetEntry {
  uint16_t widget{0}; ///< 0 = the window
  std::vector<Prop> props{};
  bool operator==(const WidgetSetEntry &) const = default;
};

/// WIDGET_SET payload.
struct WidgetSet {
  uint16_t window{0};
  std::vector<WidgetSetEntry> entries{};
  bool operator==(const WidgetSet &) const = default;
};

/// WIDGET_ADD payload.
struct WidgetAdd {
  uint16_t window{0};
  std::vector<WidgetRec> widgets{};
  bool operator==(const WidgetAdd &) const = default;
};

/// WIDGET_REMOVE payload.
struct WidgetRemove {
  uint16_t window{0};
  std::vector<uint16_t> widgets{};
  bool operator==(const WidgetRemove &) const = default;
};

/// DIALOG payload.
struct Dialog {
  uint16_t id{0};
  uint16_t owner{0}; ///< owning window (0 = modal to the desktop)
  DialogKind kind{DialogKind::Message};
  uint8_t icon{0}; ///< DialogIcon
  std::string title{};
  std::string text{};
  std::string default_text{};         ///< Input kind: initial field contents
  std::vector<std::string> buttons{}; ///< button 0 is the default
  bool operator==(const Dialog &) const = default;
};

/// NOTIFY payload.
struct Notify {
  NotifyLevel level{NotifyLevel::Info};
  uint16_t timeout_ms{0}; ///< 0 = sticky
  std::string title{};
  std::string text{};
  bool operator==(const Notify &) const = default;
};

/// DIALOG_CLOSE payload.
struct DialogClose {
  uint16_t id{0};
  bool operator==(const DialogClose &) const = default;
};

/// Decoded OK payload.
struct Ok {
  uint8_t request_type{0};
  bool operator==(const Ok &) const = default;
};

/// Decoded ERROR payload.
struct Error {
  uint8_t request_type{0};
  uint32_t code{0};
  std::string message{};
  bool operator==(const Error &) const = default;
};

/// LAUNCH_APP payload.
struct LaunchApp {
  uint8_t app{0};
  bool operator==(const LaunchApp &) const = default;
};

/// CLOSE_WINDOW payload.
struct CloseWindow {
  uint16_t window{0};
  bool operator==(const CloseWindow &) const = default;
};

/// WINDOW_EVENT payload.
struct WindowEvent {
  uint16_t window{0};
  WindowEventKind kind{WindowEventKind::Focus};
  int16_t x{0};
  int16_t y{0};
  uint16_t w{0};
  uint16_t h{0};
  bool operator==(const WindowEvent &) const = default;
};

/// WIDGET_EVENT payload (the fields a kind does not carry stay zero / empty).
struct WidgetEvent {
  uint16_t window{0};
  uint16_t widget{0};
  WidgetEventKind kind{WidgetEventKind::Click};
  int32_t value{0};        ///< Change / Select / Activate / Scroll
  std::string text{};      ///< Submit (whole), Text (this chunk)
  uint32_t text_offset{0}; ///< Text: byte offset of this chunk
  uint32_t text_total{0};  ///< Text: total byte length
  uint16_t key{0};         ///< Key
  uint8_t mods{0};         ///< Key: kMod* bits
  uint32_t codepoint{0};   ///< Key: printable character (key 0)
  bool operator==(const WidgetEvent &) const = default;
};

/// DIALOG_RESULT payload.
struct DialogResult {
  uint16_t dialog{0};
  uint8_t button{kDialogDismissed};
  std::string text{}; ///< Input kind: the field contents
  bool operator==(const DialogResult &) const = default;
};

/// A message ready for build_frame (the splitting encoders return several).
struct Message {
  Type type{Type::Ok};
  std::vector<uint8_t> payload{};
  bool operator==(const Message &) const = default;
};

// ---- device -> host encoders -----------------------------------------------------------

/// DESKTOP, a single frame. Invariant (kept by Desktop::register_app): the
/// records plus the FULL app list, descriptions emptied, always fit
/// `max_payload`. Should the whole thing not fit (many windows, long
/// descriptions), it is trimmed in this order rather than overflow: the
/// window list from the end (then flags bit1 WindowListComplete is clear:
/// the host must not reconcile its windows against it), then the app
/// descriptions. Apps are never dropped (a host replaces its registry on
/// every DESKTOP frame); if the invariant were broken the payload would
/// simply exceed the cap (the frame builder then refuses it).
/// @param trimmed Set when something was left out.
inline std::vector<uint8_t> encode_desktop(const DesktopInfo &d,
                                           size_t max_payload = espp::stream_frame::kMaxPayloadSize,
                                           bool *trimmed = nullptr) {
  auto encode = [&](size_t napps, bool with_desc, size_t nwin) {
    std::vector<uint8_t> p{};
    put_u8(p, d.proto);
    const bool complete = nwin == d.windows.size();
    put_u8(p, static_cast<uint8_t>((d.flags & ~kDesktopWindowListComplete) |
                                   (complete ? kDesktopWindowListComplete : 0)));
    put_u8(p, static_cast<uint8_t>(std::min<size_t>(d.records.size(), 255)));
    for (size_t i = 0; i < d.records.size() && i < 255; ++i)
      d.records[i].encode(p);
    put_u8(p, static_cast<uint8_t>(napps));
    for (size_t i = 0; i < napps; ++i) {
      const auto &a = d.apps[i];
      put_u8(p, a.id);
      put_u8(p, a.flags);
      put_str8(p, a.name);
      put_str8(p, a.icon);
      put_str8(p, with_desc ? std::string_view(a.description) : std::string_view());
    }
    put_u8(p, static_cast<uint8_t>(nwin));
    for (size_t i = 0; i < nwin; ++i) {
      put_u16(p, d.windows[i].id);
      put_u8(p, d.windows[i].app);
    }
    return p;
  };
  size_t napps = std::min<size_t>(d.apps.size(), 255);
  size_t nwin = std::min<size_t>(d.windows.size(), 255);
  bool with_desc = true;
  std::vector<uint8_t> p = encode(napps, with_desc, nwin);
  bool cut = napps < d.apps.size() || nwin < d.windows.size();
  while (p.size() > max_payload) {
    if (nwin > 0)
      --nwin;
    else if (with_desc)
      with_desc = false;
    else
      break; // records + apps exceed the cap: never trim apps (see the invariant)
    cut = true;
    p = encode(napps, with_desc, nwin);
  }
  if (trimmed)
    *trimmed = cut;
  return p;
}

inline std::vector<uint8_t> encode_window_close(const WindowClose &c) {
  std::vector<uint8_t> p{};
  put_u16(p, c.id);
  put_u8(p, static_cast<uint8_t>(c.reason));
  return p;
}

/// DIALOG: a single frame. The caller (Desktop::message_box / input_box)
/// refuses a dialog whose encoding exceeds the payload cap.
inline std::vector<uint8_t> encode_dialog(const Dialog &d) {
  std::vector<uint8_t> p{};
  put_u16(p, d.id);
  put_u16(p, d.owner);
  put_u8(p, static_cast<uint8_t>(d.kind));
  put_u8(p, d.icon);
  put_str8(p, d.title);
  put_str16(p, d.text);
  put_str16(p, d.default_text);
  const size_t n = d.buttons.size() > 255 ? 255 : d.buttons.size();
  put_u8(p, static_cast<uint8_t>(n));
  for (size_t i = 0; i < n; ++i)
    put_str8(p, d.buttons[i]);
  return p;
}

/// NOTIFY: a single frame. The caller (Desktop::notify) refuses one whose
/// encoding exceeds the payload cap.
inline std::vector<uint8_t> encode_notify(const Notify &n) {
  std::vector<uint8_t> p{};
  put_u8(p, static_cast<uint8_t>(n.level));
  put_u16(p, n.timeout_ms);
  put_str8(p, n.title);
  put_str16(p, n.text);
  return p;
}

inline std::vector<uint8_t> encode_dialog_close(const DialogClose &c) {
  std::vector<uint8_t> p{};
  put_u16(p, c.id);
  return p;
}

/// Encode an OK payload.
inline std::vector<uint8_t> encode_ok(uint8_t request_type) { return {request_type}; }

/// Encode an ERROR payload.
inline std::vector<uint8_t> encode_error(uint8_t request_type, uint32_t code,
                                         std::string_view message) {
  std::vector<uint8_t> p{};
  p.push_back(request_type);
  put_u32(p, code);
  p.insert(p.end(), message.begin(), message.end());
  return p;
}

/// Split a Text / TextAppend / Items prop so that its first piece encodes in at
/// most `room` bytes (as a rec, i.e. 3 + value). Returns false when no piece
/// fits (room too small for even one byte / one item); on success `first` is
/// the piece (a Text first piece keeps the Text tag, the remainder becomes
/// TextAppend) and `rest` the remainder, or nullopt when nothing is left.
inline bool split_prop(const Prop &p, size_t room, Prop &first, std::optional<Prop> &rest) {
  if (room < 4)
    return false;
  const size_t avail = room - 3;
  if (p.is(PropTag::Text) || p.is(PropTag::TextAppend)) {
    if (p.value.size() <= avail) {
      first = p;
      rest.reset();
      return true;
    }
    // never cut a UTF-8 sequence in half: back off to a sequence start
    size_t n = avail;
    while (n > 0 && (p.value[n] & 0xC0) == 0x80)
      --n;
    if (n == 0)
      return false;
    first = Prop{.tag = p.tag, .value = std::vector<uint8_t>(p.value.begin(), p.value.begin() + n)};
    rest = Prop{.tag = static_cast<uint8_t>(PropTag::TextAppend),
                .value = std::vector<uint8_t>(p.value.begin() + n, p.value.end())};
    return true;
  }
  if (p.is(PropTag::Items)) {
    if (p.value.size() <= avail) {
      first = p;
      rest.reset();
      return true;
    }
    const auto items = p.as_items();
    if (!items)
      return false;
    size_t used = 4, n = 0;
    while (n < items->items.size() && used + 2 + items->items[n].size() <= avail) {
      used += 2 + items->items[n].size();
      ++n;
    }
    if (n == 0)
      return false;
    first = Prop::items(items->start, std::span<const std::string>(items->items.data(), n));
    rest =
        Prop::items(static_cast<uint16_t>(items->start + n),
                    std::span<const std::string>(items->items.data() + n, items->items.size() - n));
    return true;
  }
  return false;
}

/// Packs WIDGET_SET entries into payloads of at most `max_payload` bytes:
/// entries and props are appended in order, a Text / TextAppend / Items prop
/// that does not fit is split across frames (Text + TextAppend pieces, Items
/// ranges), an entry continues in the next frame under the same widget id.
/// A prop that cannot be split and does not fit an empty frame is dropped
/// (counted by dropped()).
class WidgetSetWriter {
public:
  WidgetSetWriter(uint16_t window, size_t max_payload)
      : window_(window)
      , cap_(max_payload) {
    begin_frame();
  }

  void add(uint16_t widget, Prop prop) {
    while (true) {
      if (!in_entry_ || widget_ != widget || entry_props_ == 255)
        begin_entry(widget);
      const size_t room = cap_ > frame_.size() ? cap_ - frame_.size() : 0;
      if (prop.encoded_size() <= room) {
        prop.encode(frame_);
        ++entry_props_;
        return;
      }
      Prop first;
      std::optional<Prop> rest{};
      if (split_prop(prop, room, first, rest)) {
        first.encode(frame_);
        ++entry_props_;
        if (!rest)
          return;
        prop = std::move(*rest);
        continue;
      }
      if (frame_has_content()) {
        end_frame();
        begin_frame();
        continue;
      }
      // a fresh frame with a fresh entry and it still does not fit: give up on it
      ++dropped_;
      return;
    }
  }

  void add(uint16_t widget, std::span<const Prop> props) {
    for (const auto &p : props)
      add(widget, p);
  }

  /// Every payload produced (none when nothing was added).
  std::vector<std::vector<uint8_t>> finish() {
    if (frame_has_content())
      end_frame();
    frame_.clear();
    return std::move(frames_);
  }

  size_t dropped() const { return dropped_; }

private:
  static constexpr size_t kHeader = 3;      // [win u16][entries u8]
  static constexpr size_t kEntryHeader = 3; // [widget u16][props u8]

  bool frame_has_content() const { return entries_ > 0 || entry_props_ > 0; }

  void begin_frame() {
    frame_.clear();
    put_u16(frame_, window_);
    put_u8(frame_, 0);
    entries_ = 0;
    in_entry_ = false;
    entry_props_ = 0;
  }
  void end_entry() {
    if (!in_entry_)
      return;
    if (entry_props_ == 0) {
      frame_.resize(entry_start_); // nothing was written for it: drop the header
    } else {
      frame_[entry_start_ + 2] = entry_props_;
      ++entries_;
    }
    in_entry_ = false;
    entry_props_ = 0;
  }
  void end_frame() {
    end_entry();
    frame_[2] = entries_;
    frames_.push_back(frame_);
    entries_ = 0;
  }
  void begin_entry(uint16_t widget) {
    end_entry();
    // room for the entry header + the smallest useful rec (3 + 1)
    const size_t room = cap_ > frame_.size() ? cap_ - frame_.size() : 0;
    if (entries_ == 255 || room < kEntryHeader + 4) {
      if (frame_has_content())
        end_frame();
      begin_frame();
    }
    entry_start_ = frame_.size();
    put_u16(frame_, widget);
    put_u8(frame_, 0);
    widget_ = widget;
    in_entry_ = true;
    entry_props_ = 0;
  }

  uint16_t window_;
  size_t cap_;
  std::vector<uint8_t> frame_{};
  std::vector<std::vector<uint8_t>> frames_{};
  size_t entries_{0};
  size_t entry_start_{0};
  bool in_entry_{false};
  uint8_t entry_props_{0};
  uint16_t widget_{0};
  size_t dropped_{0};
};

/// Packs a widget tree (WINDOW_OPEN + WIDGET_ADD continuation frames, or plain
/// WIDGET_ADD frames) into payloads of at most `max_payload` bytes. A widget
/// record whose props do not fit keeps the props that do (a Text / Items prop
/// is split at the boundary) and the remainder is returned as WIDGET_SET
/// entries (`leftovers`) to be sent after the last add frame.
class WidgetTreeWriter {
public:
  /// @param open When set, the first frame is a WINDOW_OPEN with this header
  ///        (its `widgets` are ignored; add() supplies them, `total` is the
  ///        number of widgets that will be added).
  WidgetTreeWriter(uint16_t window, size_t max_payload, std::optional<WindowOpen> open,
                   uint16_t total)
      : window_(window)
      , cap_(max_payload)
      , open_(std::move(open))
      , total_(total) {
    begin_frame(open_.has_value());
  }

  void add(const WidgetRec &w) {
    // every widget record needs its base; make sure at least the base fits
    // (and, if there are props, the smallest useful rec with it)
    const size_t need = WidgetRec::kBaseSize + (w.props.empty() ? 0 : 4);
    // roll over when the base does not fit: also behind a WINDOW_OPEN head
    // that already filled the frame (a long title) -- that frame then
    // carries zero widgets and the tree continues in WIDGET_ADD
    if ((count_ > 0 || is_open_) && (cap_ > frame_.size() ? cap_ - frame_.size() : 0) < need) {
      end_frame();
      begin_frame(false);
    }
    // an empty WIDGET_ADD frame always holds a widget base plus one small rec
    // (max_payload >= kMinPayloadBytes); props that do not fit go to leftovers
    const size_t base_at = frame_.size();
    put_u16(frame_, w.id);
    put_u16(frame_, w.parent);
    put_u8(frame_, static_cast<uint8_t>(w.type));
    put_u8(frame_, w.weight);
    put_u8(frame_, w.layout);
    put_u8(frame_, 0); // props count, patched below
    uint8_t nprops = 0;
    bool full = false;
    for (const auto &p : w.props) {
      const size_t room = cap_ > frame_.size() ? cap_ - frame_.size() : 0;
      if (!full && nprops < 255 && p.encoded_size() <= room) {
        p.encode(frame_);
        ++nprops;
        continue;
      }
      Prop first;
      std::optional<Prop> rest{};
      if (!full && nprops < 255 && split_prop(p, room, first, rest)) {
        first.encode(frame_);
        ++nprops;
        if (rest)
          leftovers.push_back({w.id, std::move(*rest)});
        full = true; // the frame is spent; everything else trails
        continue;
      }
      leftovers.push_back({w.id, p});
      full = true;
    }
    frame_[base_at + 7] = nprops;
    ++count_;
    ++added_;
  }

  /// Every message produced, in order (the WINDOW_OPEN first when requested,
  /// even with no widgets).
  std::vector<Message> finish() {
    if (count_ > 0 || (frames_.empty() && open_))
      end_frame();
    frame_.clear();
    return std::move(frames_);
  }

  /// Props that did not fit their widget record: send as WIDGET_SET after the
  /// last message (in this order).
  std::vector<std::pair<uint16_t, Prop>> leftovers{};

private:
  void begin_frame(bool as_open) {
    frame_.clear();
    is_open_ = as_open;
    if (as_open) {
      const auto &o = *open_;
      put_u16(frame_, o.id);
      put_u8(frame_, o.app);
      put_u16(frame_, o.flags);
      put_i16(frame_, o.geometry.x);
      put_i16(frame_, o.geometry.y);
      put_u16(frame_, o.geometry.w);
      put_u16(frame_, o.geometry.h);
      put_str8(frame_, o.title);
      put_u16(frame_, total_);
    } else {
      put_u16(frame_, window_);
    }
    count_at_ = frame_.size();
    put_u16(frame_, 0); // count, patched at end_frame
    count_ = 0;
  }
  void end_frame() {
    frame_[count_at_] = static_cast<uint8_t>(count_ & 0xFF);
    frame_[count_at_ + 1] = static_cast<uint8_t>(count_ >> 8);
    frames_.push_back({is_open_ ? Type::WindowOpen : Type::WidgetAdd, frame_});
    count_ = 0;
  }

  uint16_t window_;
  size_t cap_;
  std::optional<WindowOpen> open_{};
  uint16_t total_;
  std::vector<uint8_t> frame_{};
  std::vector<Message> frames_{};
  size_t count_at_{0};
  size_t count_{0};
  size_t added_{0};
  bool is_open_{false};
};

/// The props that did not fit their widget records, as WIDGET_SET messages
/// appended after the add frames.
inline void append_leftovers(std::vector<Message> &out, uint16_t window,
                             const std::vector<std::pair<uint16_t, Prop>> &leftovers,
                             size_t max_payload, size_t *dropped) {
  if (leftovers.empty())
    return;
  WidgetSetWriter s(window, max_payload);
  for (const auto &[id, p] : leftovers)
    s.add(id, p);
  auto payloads = s.finish();
  std::transform(payloads.begin(), payloads.end(), std::back_inserter(out),
                 [](std::vector<uint8_t> &p) {
                   return Message{Type::WidgetSet, std::move(p)};
                 });
  if (dropped)
    *dropped += s.dropped();
}

/// WINDOW_OPEN (+ WIDGET_ADD continuations + WIDGET_SET leftovers) for a whole
/// window, split at `max_payload`. `open.widgets` is the full tree in
/// parent-before-child order; `open.total` is set from it.
inline std::vector<Message> encode_window_open(const WindowOpen &open, size_t max_payload,
                                               size_t *dropped = nullptr) {
  WindowOpen hdr = open;
  hdr.total = static_cast<uint16_t>(open.widgets.size());
  hdr.widgets.clear();
  WidgetTreeWriter w(open.id, max_payload, hdr, hdr.total);
  for (const auto &rec : open.widgets)
    w.add(rec);
  auto out = w.finish();
  append_leftovers(out, open.id, w.leftovers, max_payload, dropped);
  return out;
}

/// WIDGET_ADD (+ WIDGET_SET leftovers), split at `max_payload`.
inline std::vector<Message> encode_widget_add(const WidgetAdd &add, size_t max_payload,
                                              size_t *dropped = nullptr) {
  if (add.widgets.empty())
    return {};
  WidgetTreeWriter w(add.window, max_payload, std::nullopt, 0);
  for (const auto &rec : add.widgets)
    w.add(rec);
  auto out = w.finish();
  append_leftovers(out, add.window, w.leftovers, max_payload, dropped);
  return out;
}

/// WIDGET_SET, split at `max_payload` (see WidgetSetWriter).
inline std::vector<std::vector<uint8_t>> encode_widget_set(const WidgetSet &set, size_t max_payload,
                                                           size_t *dropped = nullptr) {
  WidgetSetWriter w(set.window, max_payload);
  for (const auto &e : set.entries)
    w.add(e.widget, e.props);
  auto out = w.finish();
  if (dropped)
    *dropped += w.dropped();
  return out;
}

/// WIDGET_REMOVE, split at `max_payload`.
inline std::vector<std::vector<uint8_t>> encode_widget_remove(const WidgetRemove &rm,
                                                              size_t max_payload) {
  std::vector<std::vector<uint8_t>> out{};
  const size_t per_frame = std::max<size_t>(1, (max_payload > 4 ? max_payload - 4 : 0) / 2);
  for (size_t i = 0; i < rm.widgets.size(); i += per_frame) {
    const size_t n = std::min(per_frame, rm.widgets.size() - i);
    std::vector<uint8_t> p{};
    put_u16(p, rm.window);
    put_u16(p, static_cast<uint16_t>(n));
    for (size_t k = 0; k < n; ++k)
      put_u16(p, rm.widgets[i + k]);
    out.push_back(std::move(p));
  }
  return out;
}

// ---- host -> device encoders (host tools / tests) ------------------------------------------

inline std::vector<uint8_t> encode_launch_app(const LaunchApp &l) { return {l.app}; }

inline std::vector<uint8_t> encode_close_window(const CloseWindow &c) {
  std::vector<uint8_t> p{};
  put_u16(p, c.window);
  return p;
}

inline std::vector<uint8_t> encode_window_event(const WindowEvent &e) {
  std::vector<uint8_t> p{};
  put_u16(p, e.window);
  put_u8(p, static_cast<uint8_t>(e.kind));
  put_i16(p, e.x);
  put_i16(p, e.y);
  put_u16(p, e.w);
  put_u16(p, e.h);
  return p;
}

inline std::vector<uint8_t> encode_widget_event(const WidgetEvent &e) {
  std::vector<uint8_t> p{};
  put_u16(p, e.window);
  put_u16(p, e.widget);
  put_u8(p, static_cast<uint8_t>(e.kind));
  switch (e.kind) {
  case WidgetEventKind::Click:
    break;
  case WidgetEventKind::Change:
  case WidgetEventKind::Select:
  case WidgetEventKind::Activate:
  case WidgetEventKind::Scroll:
    put_i32(p, e.value);
    break;
  case WidgetEventKind::Submit:
    put_bytes(p, e.text);
    break;
  case WidgetEventKind::Text:
    put_u32(p, e.text_offset);
    put_u32(p, e.text_total);
    put_bytes(p, e.text);
    break;
  case WidgetEventKind::Key:
    put_u16(p, e.key);
    put_u8(p, e.mods);
    put_u32(p, e.codepoint);
    break;
  }
  return p;
}

inline std::vector<uint8_t> encode_dialog_result(const DialogResult &r) {
  std::vector<uint8_t> p{};
  put_u16(p, r.dialog);
  put_u8(p, r.button);
  put_bytes(p, r.text);
  return p;
}

// ---- decoders (both directions) ------------------------------------------------------------
// Every decoder returns nullopt on a payload that is truncated, has trailing
// bytes, or declares a count / length running past the end. Unknown rec tags
// are kept raw (Prop::kind() == Unknown); a known tag with a malformed value
// is also kept raw, for the consumer to ignore (Prop::valid()).

inline std::optional<DesktopInfo> decode_desktop(std::span<const uint8_t> p) {
  Reader r(p);
  DesktopInfo d;
  d.proto = r.u8();
  d.flags = r.u8();
  const uint8_t nrec = r.u8();
  if (!read_props(r, nrec, d.records))
    return std::nullopt;
  const uint8_t napps = r.u8();
  for (uint8_t i = 0; i < napps && r.ok(); ++i) {
    AppRec a;
    a.id = r.u8();
    a.flags = r.u8();
    a.name = r.str8();
    a.icon = r.str8();
    a.description = r.str8();
    d.apps.push_back(std::move(a));
  }
  const uint8_t nwin = r.u8();
  for (uint8_t i = 0; i < nwin && r.ok(); ++i) {
    WindowRef w;
    w.id = r.u16();
    w.app = r.u8();
    d.windows.push_back(w);
  }
  if (!r.ok() || !r.at_end())
    return std::nullopt;
  return d;
}

inline bool read_widget_rec(Reader &r, WidgetRec &w) {
  w.id = r.u16();
  w.parent = r.u16();
  w.type = static_cast<WidgetType>(r.u8());
  w.weight = r.u8();
  w.layout = r.u8();
  const uint8_t nprops = r.u8();
  return r.ok() && read_props(r, nprops, w.props);
}

inline std::optional<WindowOpen> decode_window_open(std::span<const uint8_t> p) {
  Reader r(p);
  WindowOpen o;
  o.id = r.u16();
  o.app = r.u8();
  o.flags = r.u16();
  o.geometry.x = r.i16();
  o.geometry.y = r.i16();
  o.geometry.w = r.u16();
  o.geometry.h = r.u16();
  o.title = r.str8();
  o.total = r.u16();
  const uint16_t n = r.u16();
  for (uint16_t i = 0; i < n && r.ok(); ++i) {
    WidgetRec w;
    if (!read_widget_rec(r, w))
      return std::nullopt;
    o.widgets.push_back(std::move(w));
  }
  if (!r.ok() || !r.at_end())
    return std::nullopt;
  return o;
}

inline std::optional<WindowClose> decode_window_close(std::span<const uint8_t> p) {
  if (p.size() != 3)
    return std::nullopt;
  Reader r(p);
  WindowClose c;
  c.id = r.u16();
  c.reason = static_cast<WindowCloseReason>(r.u8());
  return c;
}

inline std::optional<WidgetSet> decode_widget_set(std::span<const uint8_t> p) {
  Reader r(p);
  WidgetSet s;
  s.window = r.u16();
  const uint8_t n = r.u8();
  for (uint8_t i = 0; i < n && r.ok(); ++i) {
    WidgetSetEntry e;
    e.widget = r.u16();
    const uint8_t nprops = r.u8();
    if (!r.ok() || !read_props(r, nprops, e.props))
      return std::nullopt;
    s.entries.push_back(std::move(e));
  }
  if (!r.ok() || !r.at_end())
    return std::nullopt;
  return s;
}

inline std::optional<WidgetAdd> decode_widget_add(std::span<const uint8_t> p) {
  Reader r(p);
  WidgetAdd a;
  a.window = r.u16();
  const uint16_t n = r.u16();
  for (uint16_t i = 0; i < n && r.ok(); ++i) {
    WidgetRec w;
    if (!read_widget_rec(r, w))
      return std::nullopt;
    a.widgets.push_back(std::move(w));
  }
  if (!r.ok() || !r.at_end())
    return std::nullopt;
  return a;
}

inline std::optional<WidgetRemove> decode_widget_remove(std::span<const uint8_t> p) {
  Reader r(p);
  WidgetRemove rm;
  rm.window = r.u16();
  const uint16_t n = r.u16();
  for (uint16_t i = 0; i < n && r.ok(); ++i)
    rm.widgets.push_back(r.u16());
  if (!r.ok() || !r.at_end())
    return std::nullopt;
  return rm;
}

inline std::optional<Dialog> decode_dialog(std::span<const uint8_t> p) {
  Reader r(p);
  Dialog d;
  d.id = r.u16();
  d.owner = r.u16();
  d.kind = static_cast<DialogKind>(r.u8());
  d.icon = r.u8();
  d.title = r.str8();
  d.text = r.str16();
  d.default_text = r.str16();
  const uint8_t n = r.u8();
  for (uint8_t i = 0; i < n && r.ok(); ++i)
    d.buttons.push_back(r.str8());
  if (!r.ok() || !r.at_end())
    return std::nullopt;
  return d;
}

inline std::optional<Notify> decode_notify(std::span<const uint8_t> p) {
  Reader r(p);
  Notify n;
  n.level = static_cast<NotifyLevel>(r.u8());
  n.timeout_ms = r.u16();
  n.title = r.str8();
  n.text = r.str16();
  if (!r.ok() || !r.at_end())
    return std::nullopt;
  return n;
}

inline std::optional<DialogClose> decode_dialog_close(std::span<const uint8_t> p) {
  if (p.size() != 2)
    return std::nullopt;
  return DialogClose{.id = espp::stream_frame::get_u16(p)};
}

inline std::optional<Ok> decode_ok(std::span<const uint8_t> p) {
  if (p.size() != 1)
    return std::nullopt;
  return Ok{.request_type = p[0]};
}

inline std::optional<Error> decode_error(std::span<const uint8_t> p) {
  if (p.size() < 5)
    return std::nullopt;
  Error e;
  e.request_type = p[0];
  e.code = espp::stream_frame::get_u32(p.subspan(1));
  e.message.assign(reinterpret_cast<const char *>(p.data() + 5), p.size() - 5);
  return e;
}

inline std::optional<LaunchApp> decode_launch_app(std::span<const uint8_t> p) {
  if (p.size() != 1)
    return std::nullopt;
  return LaunchApp{.app = p[0]};
}

inline std::optional<CloseWindow> decode_close_window(std::span<const uint8_t> p) {
  if (p.size() != 2)
    return std::nullopt;
  return CloseWindow{.window = espp::stream_frame::get_u16(p)};
}

/// nullopt also for an unknown event kind (so a handler never sees one).
inline std::optional<WindowEvent> decode_window_event(std::span<const uint8_t> p) {
  if (p.size() != 11)
    return std::nullopt;
  Reader r(p);
  WindowEvent e;
  e.window = r.u16();
  const uint8_t kind = r.u8();
  if (kind < static_cast<uint8_t>(WindowEventKind::Focus) ||
      kind > static_cast<uint8_t>(WindowEventKind::Resized))
    return std::nullopt;
  e.kind = static_cast<WindowEventKind>(kind);
  e.x = r.i16();
  e.y = r.i16();
  e.w = r.u16();
  e.h = r.u16();
  return e;
}

/// nullopt also for an unknown event kind (its value layout is unknown).
inline std::optional<WidgetEvent> decode_widget_event(std::span<const uint8_t> p) {
  Reader r(p);
  WidgetEvent e;
  e.window = r.u16();
  e.widget = r.u16();
  e.kind = static_cast<WidgetEventKind>(r.u8());
  if (!r.ok())
    return std::nullopt;
  switch (e.kind) {
  case WidgetEventKind::Click:
    break;
  case WidgetEventKind::Change:
  case WidgetEventKind::Select:
  case WidgetEventKind::Activate:
  case WidgetEventKind::Scroll:
    e.value = r.i32();
    break;
  case WidgetEventKind::Submit:
    e.text = r.rest();
    break;
  case WidgetEventKind::Text:
    e.text_offset = r.u32();
    e.text_total = r.u32();
    e.text = r.rest();
    break;
  case WidgetEventKind::Key:
    e.key = r.u16();
    e.mods = r.u8();
    e.codepoint = r.u32();
    break;
  default:
    return std::nullopt;
  }
  if (!r.ok() || !r.at_end())
    return std::nullopt;
  return e;
}

inline std::optional<DialogResult> decode_dialog_result(std::span<const uint8_t> p) {
  if (p.size() < 3)
    return std::nullopt;
  Reader r(p);
  DialogResult d;
  d.dialog = r.u16();
  d.button = r.u8();
  d.text = r.rest();
  return d;
}

} // namespace espp::detail::desktop_protocol

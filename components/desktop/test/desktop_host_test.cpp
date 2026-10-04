// Host-buildable unit tests for the espp desktop wire codec
// (include/detail/desktop_protocol.hpp) and the ESP-free desktop model
// (include/detail/desktop_model.hpp). Build & run from this directory with:
//   c++ -std=c++20 -I../include -I../../stream_frame/include desktop_host_test.cpp -o test &&
//   ./test
//
// The golden vectors live in desktop_vectors.txt (shared with the web app's
// node test, components/desktop/web/test/desktop_codec_test.js): this program
// checks that its encoders reproduce every vector's hex column and that its
// decoders turn the hex back into the same value, and that the file's JSON
// column matches what this program derives from the decoded value.
//   ./test --gen [path]   rewrites the fixture from the catalogue below
//   ./test [path]         verifies against the fixture (default ./desktop_vectors.txt)

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <functional>
#include <map>
#include <numeric>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <vector>

#include "detail/desktop_model.hpp"
#include "detail/desktop_protocol.hpp"

namespace dp = espp::detail::desktop_protocol;
namespace dm = espp::detail::desktop_model;
namespace sf = espp::stream_frame;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

// ---- helpers ------------------------------------------------------------------------------

static std::string hex(std::span<const uint8_t> b) {
  static const char *d = "0123456789abcdef";
  std::string s;
  for (const uint8_t x : b) {
    s.push_back(d[x >> 4]);
    s.push_back(d[x & 15]);
  }
  return s;
}

static std::vector<uint8_t> unhex(const std::string &s) {
  std::vector<uint8_t> out;
  auto nib = [](char c) -> int {
    if (c >= '0' && c <= '9')
      return c - '0';
    if (c >= 'a' && c <= 'f')
      return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
      return c - 'A' + 10;
    return -1;
  };
  for (size_t i = 0; i + 1 < s.size(); i += 2)
    out.push_back(static_cast<uint8_t>(nib(s[i]) * 16 + nib(s[i + 1])));
  return out;
}

// A minimal JSON writer: objects keep insertion order, strings are escaped the
// way JSON.stringify does for the characters that matter (UTF-8 passes through).
struct Json {
  std::string s;
  static std::string str(std::string_view v) {
    std::string o = "\"";
    for (const unsigned char c : v) {
      switch (c) {
      case '"':
        o += "\\\"";
        break;
      case '\\':
        o += "\\\\";
        break;
      case '\n':
        o += "\\n";
        break;
      case '\r':
        o += "\\r";
        break;
      case '\t':
        o += "\\t";
        break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          o += buf;
        } else {
          o.push_back(static_cast<char>(c));
        }
      }
    }
    return o + "\"";
  }
  template <typename T> static std::string num(T v) { return std::to_string(v); }
  static std::string strs(const std::vector<std::string> &v) {
    std::string o = "[";
    for (size_t i = 0; i < v.size(); ++i)
      o += (i ? "," : "") + str(v[i]);
    return o + "]";
  }
  static std::string prop(const dp::Prop &p) {
    std::string o = "{\"tag\":" + num(+p.tag) + ",";
    switch (p.kind()) {
    case dp::PropKind::Text:
      return o + "\"value\":" + str(p.as_text()) + "}";
    case dp::PropKind::I32:
      if (auto v = p.as_i32())
        return o + "\"value\":" + num(*v) + "}";
      break;
    case dp::PropKind::U8:
      if (auto v = p.as_u8())
        return o + "\"value\":" + num(+*v) + "}";
      break;
    case dp::PropKind::U16:
      if (auto v = p.as_u16())
        return o + "\"value\":" + num(*v) + "}";
      break;
    case dp::PropKind::U32:
      if (auto v = p.as_u32())
        return o + "\"value\":" + num(*v) + "}";
      break;
    case dp::PropKind::Items:
      if (auto v = p.as_items())
        return o + "\"value\":{\"start\":" + num(v->start) + ",\"items\":" + strs(v->items) + "}}";
      break;
    case dp::PropKind::Columns:
      if (auto v = p.as_columns())
        return o + "\"value\":" + strs(*v) + "}";
      break;
    case dp::PropKind::Geometry:
      if (auto v = p.as_geometry())
        return o + "\"value\":{\"x\":" + num(v->x) + ",\"y\":" + num(v->y) + ",\"w\":" + num(v->w) +
               ",\"h\":" + num(v->h) + "}}";
      break;
    case dp::PropKind::Unknown:
      break;
    }
    return o + "\"raw\":" + str(hex(p.value)) + "}";
  }
  static std::string props(const std::vector<dp::Prop> &v) {
    std::string o = "[";
    for (size_t i = 0; i < v.size(); ++i)
      o += (i ? "," : "") + prop(v[i]);
    return o + "]";
  }
  static std::string widget(const dp::WidgetRec &w) {
    return "{\"id\":" + num(w.id) + ",\"parent\":" + num(w.parent) +
           ",\"type\":" + num(+static_cast<uint8_t>(w.type)) + ",\"weight\":" + num(+w.weight) +
           ",\"layout\":" + num(+w.layout) + ",\"props\":" + props(w.props) + "}";
  }
  static std::string widgets(const std::vector<dp::WidgetRec> &v) {
    std::string o = "[";
    for (size_t i = 0; i < v.size(); ++i)
      o += (i ? "," : "") + widget(v[i]);
    return o + "]";
  }
};

// ---- the vector catalogue ---------------------------------------------------------------------
// One entry per fixture line: the value, its encoder and a decode-and-compare.

struct Vector {
  std::string name;
  bool d2h{false};
  dp::Type type{dp::Type::Ok};
  std::vector<uint8_t> payload;
  std::string json;
  std::function<bool(std::span<const uint8_t>)> roundtrip; // decode == value
};

template <typename T, typename Dec>
static std::function<bool(std::span<const uint8_t>)> rt(const T &value, Dec dec) {
  return [value, dec](std::span<const uint8_t> p) {
    const auto d = dec(p);
    return d && *d == value;
  };
}

static std::vector<Vector> catalogue() {
  using namespace dp;
  std::vector<Vector> v;

  // DESKTOP: 3 apps, every record tag + an unknown one, one open window
  {
    DesktopInfo d;
    d.flags = kDesktopHasSnapshot | kDesktopWindowListComplete; // the encoder sets bit1
    d.records = {Prop::text(static_cast<PropTag>(DesktopTag::DeviceName), "espp Desktop"),
                 Prop::text(static_cast<PropTag>(DesktopTag::Firmware), "desktop_example 1.0"),
                 Prop::text(static_cast<PropTag>(DesktopTag::Theme), "auto"),
                 Prop::u32(static_cast<PropTag>(DesktopTag::Accent), 0x3b82f6),
                 Prop::u16(static_cast<PropTag>(DesktopTag::MaxPayload), 4081),
                 Prop::u16(static_cast<PropTag>(DesktopTag::FlushPeriodMs), 50),
                 Prop{.tag = 200, .value = {0xAA, 0xBB}}};
    d.apps = {{.id = 1,
               .flags = kAppSingleInstance,
               .name = "Counter",
               .icon = "\xF0\x9F\xA7\xAE",
               .description = "Counts clicks"},
              {.id = 2,
               .flags = kAppSingleInstance | kAppHidden,
               .name = "About",
               .icon = "svg:info",
               .description = "About this device"},
              {.id = 3,
               .flags = 0,
               .name = "Editor",
               .icon = "\xF0\x9F\x93\x9D",
               .description = "Edit a file"}};
    d.windows = {{.id = 1, .app = 1}};
    auto apps_json = [](const DesktopInfo &info) {
      std::string json = "[";
      for (size_t i = 0; i < info.apps.size(); ++i) {
        const auto &a = info.apps[i];
        json += std::string(i ? "," : "") + "{\"id\":" + Json::num(+a.id) +
                ",\"flags\":" + Json::num(+a.flags) + ",\"name\":" + Json::str(a.name) +
                ",\"icon\":" + Json::str(a.icon) + ",\"description\":" + Json::str(a.description) +
                "}";
      }
      return json + "]";
    };
    std::string json = "{\"proto\":1,\"flags\":3,\"records\":" + Json::props(d.records) +
                       ",\"apps\":" + apps_json(d) + ",\"windows\":[{\"id\":1,\"app\":1}]}";
    v.push_back({"desktop", true, Type::Desktop, encode_desktop(d), json, rt(d, decode_desktop)});
    // the same desktop encoded under a cap that forces the encoder to trim:
    // descriptions go first, then apps, then the window list -- and once the
    // window list is cut, flags bit1 (WindowListComplete) is clear so the host
    // does not close windows missing from it
    DesktopInfo three = d;
    three.windows = {{.id = 1, .app = 1}, {.id = 2, .app = 3}, {.id = 3, .app = 1}};
    // records (3 + 15 + 22 + 7 + 7 + 5 + 5 + 5 = 69) + 1 app count + 1 win count
    // = 71; a cap of 71 + 3 * 2 = 77 leaves room for exactly two windows and
    // no apps
    bool was_trimmed = false;
    const auto payload = encode_desktop(three, 77, &was_trimmed);
    CHECK(was_trimmed && payload.size() == 77);
    DesktopInfo expect = three;
    expect.flags = kDesktopHasSnapshot; // bit1 clear
    expect.apps.clear();
    expect.windows = {{.id = 1, .app = 1}, {.id = 2, .app = 3}};
    v.push_back({"desktop_window_list_trimmed", true, Type::Desktop, payload,
                 "{\"proto\":1,\"flags\":1,\"records\":" + Json::props(d.records) +
                     ",\"apps\":[],\"windows\":[{\"id\":1,\"app\":1},{\"id\":2,\"app\":3}]}",
                 rt(expect, decode_desktop)});
  }
  // WINDOW_OPEN: all flags, -1 geometry, 4 widgets incl. a Row child, a long
  // Text (> 255 bytes) and an unknown prop tag
  {
    WindowOpen o;
    o.id = 1;
    o.app = 1;
    o.flags = 0x3FF;
    o.geometry = {.x = -1, .y = -1, .w = 0, .h = 0};
    o.title = "Counter";
    std::string long_text;
    for (int i = 0; i < 30; ++i)
      long_text += "0123456789"; // 300 bytes
    o.widgets = {
        {.id = 1, .parent = 0, .type = WidgetType::Column, .weight = 1, .layout = kLayoutStretch},
        {.id = 2, .parent = 1, .type = WidgetType::Row, .weight = 0, .layout = kLayoutAlignCenter},
        {.id = 3,
         .parent = 2,
         .type = WidgetType::Label,
         .props = {Prop::text(PropTag::Text, "Count: 0"), Prop::u16(PropTag::Flags, kLabelBold)}},
        {.id = 4,
         .parent = 1,
         .type = WidgetType::TextArea,
         .weight = 1,
         .layout = kLayoutStretch | kLayoutScroll,
         .props = {Prop::text(PropTag::Text, long_text),
                   Prop::u16(PropTag::Flags, kTextAreaReadOnly | kTextAreaMonospace),
                   Prop::u16(PropTag::MaxLines, 200), Prop{.tag = 201, .value = {'x', 'y'}}}}};
    o.total = 4;
    const auto msgs = encode_window_open(o, 4081);
    CHECK(msgs.size() == 1 && msgs[0].type == Type::WindowOpen);
    std::string json = "{\"window\":1,\"app\":1,\"flags\":1023,\"x\":-1,\"y\":-1,\"w\":0,\"h\":0,"
                       "\"title\":\"Counter\",\"total\":4,\"widgets\":" +
                       Json::widgets(o.widgets) + "}";
    v.push_back(
        {"window_open", true, Type::WindowOpen, msgs[0].payload, json, rt(o, decode_window_open)});
  }
  {
    WindowClose c{.id = 7, .reason = WindowCloseReason::Host};
    v.push_back({"window_close", true, Type::WindowClose, encode_window_close(c),
                 "{\"window\":7,\"reason\":1}", rt(c, decode_window_close)});
  }
  // WIDGET_SET: window entry (0) with title + geometry, two widgets with an
  // Items range and a TextAppend
  {
    WidgetSet s;
    s.window = 1;
    const std::vector<std::string> rows = {"b\tB", "c\tC"};
    s.entries = {
        {.widget = 0,
         .props = {Prop::text(PropTag::Title, "Counter (3)"),
                   Prop::geometry({.x = 10, .y = 20, .w = 300, .h = 200})}},
        {.widget = 3,
         .props = {Prop::text(PropTag::Text, "Count: 3"), Prop::u32(PropTag::Color, 0xff0000)}},
        {.widget = 5, .props = {Prop::u16(PropTag::ItemCount, 3), Prop::items(1, rows)}},
        {.widget = 4, .props = {Prop::text(PropTag::TextAppend, "line\n")}}};
    const auto payloads = encode_widget_set(s, 4081);
    CHECK(payloads.size() == 1);
    std::string json = "{\"window\":1,\"entries\":[";
    for (size_t i = 0; i < s.entries.size(); ++i)
      json += std::string(i ? "," : "") + "{\"widget\":" + Json::num(s.entries[i].widget) +
              ",\"props\":" + Json::props(s.entries[i].props) + "}";
    json += "]}";
    v.push_back({"widget_set", true, Type::WidgetSet, payloads[0], json, rt(s, decode_widget_set)});
  }
  {
    WidgetAdd a;
    a.window = 1;
    a.widgets = {
        {.id = 6,
         .parent = 1,
         .type = WidgetType::Button,
         .props = {Prop::text(PropTag::Text, "Reset"), Prop::u16(PropTag::Flags, kButtonDanger),
                   Prop::u16(PropTag::InsertBefore, 4)}}};
    const auto msgs = encode_widget_add(a, 4081);
    CHECK(msgs.size() == 1 && msgs[0].type == Type::WidgetAdd);
    v.push_back({"widget_add", true, Type::WidgetAdd, msgs[0].payload,
                 "{\"window\":1,\"widgets\":" + Json::widgets(a.widgets) + "}",
                 rt(a, decode_widget_add)});
  }
  {
    WidgetRemove r{.window = 1, .widgets = {3, 6}};
    const auto payloads = encode_widget_remove(r, 4081);
    CHECK(payloads.size() == 1);
    v.push_back({"widget_remove", true, Type::WidgetRemove, payloads[0],
                 "{\"window\":1,\"widgets\":[3,6]}", rt(r, decode_widget_remove)});
  }
  {
    Dialog d{.id = 2,
             .owner = 1,
             .kind = DialogKind::Input,
             .icon = static_cast<uint8_t>(DialogIcon::Question),
             .title = "Rename",
             .text = "New name:",
             .default_text = "notes.txt",
             .buttons = {"OK", "Cancel"}};
    v.push_back(
        {"dialog", true, Type::Dialog, encode_dialog(d),
         "{\"dialog\":2,\"owner\":1,\"kind\":1,\"icon\":2,\"title\":\"Rename\",\"text\":\"New "
         "name:\",\"default\":\"notes.txt\",\"buttons\":[\"OK\",\"Cancel\"]}",
         rt(d, decode_dialog)});
  }
  {
    Notify n{.level = NotifyLevel::Warn,
             .timeout_ms = 3000,
             .title = "Saved",
             .text = "notes.txt written"};
    v.push_back(
        {"notify", true, Type::Notify, encode_notify(n),
         "{\"level\":2,\"timeoutMs\":3000,\"title\":\"Saved\",\"text\":\"notes.txt written\"}",
         rt(n, decode_notify)});
  }
  {
    DialogClose c{.id = 2};
    v.push_back({"dialog_close", true, Type::DialogClose, encode_dialog_close(c), "{\"dialog\":2}",
                 rt(c, decode_dialog_close)});
  }
  {
    Ok ok{.request_type = 0x02};
    v.push_back(
        {"ok", true, Type::Ok, encode_ok(ok.request_type), "{\"request\":2}", rt(ok, decode_ok)});
  }
  {
    Error e{.request_type = 0x02, .code = 2, .message = "no such app"};
    v.push_back({"error", true, Type::Error, encode_error(e.request_type, e.code, e.message),
                 "{\"request\":2,\"errno\":2,\"message\":\"no such app\"}", rt(e, decode_error)});
  }
  // ---- host -> device
  v.push_back({"get_desktop", false, Type::GetDesktop, {}, "{}", [](std::span<const uint8_t> p) {
                 return p.empty();
               }});
  {
    LaunchApp l{.app = 3};
    v.push_back({"launch_app", false, Type::LaunchApp, encode_launch_app(l), "{\"app\":3}",
                 rt(l, decode_launch_app)});
  }
  {
    CloseWindow c{.window = 7};
    v.push_back({"close_window", false, Type::CloseWindow, encode_close_window(c), "{\"window\":7}",
                 rt(c, decode_close_window)});
  }
  {
    WindowEvent e{
        .window = 1, .kind = WindowEventKind::Moved, .x = -20, .y = 30, .w = 400, .h = 300};
    v.push_back({"window_event", false, Type::WindowEvent, encode_window_event(e),
                 "{\"window\":1,\"event\":6,\"x\":-20,\"y\":30,\"w\":400,\"h\":300}",
                 rt(e, decode_window_event)});
  }
  auto we = [&](const std::string &name, const WidgetEvent &e, const std::string &extra) {
    v.push_back({name, false, Type::WidgetEvent, encode_widget_event(e),
                 "{\"window\":" + Json::num(e.window) + ",\"widget\":" + Json::num(e.widget) +
                     ",\"event\":" + Json::num(+static_cast<uint8_t>(e.kind)) + extra + "}",
                 rt(e, decode_widget_event)});
  };
  we("widget_event_click", {.window = 1, .widget = 6, .kind = WidgetEventKind::Click}, "");
  we("widget_event_change",
     {.window = 1, .widget = 8, .kind = WidgetEventKind::Change, .value = -1}, ",\"value\":-1");
  we("widget_event_submit",
     {.window = 1, .widget = 9, .kind = WidgetEventKind::Submit, .text = "hello"},
     ",\"text\":\"hello\"");
  we("widget_event_text_0",
     {.window = 1,
      .widget = 4,
      .kind = WidgetEventKind::Text,
      .text = "hello ",
      .text_offset = 0,
      .text_total = 10},
     ",\"offset\":0,\"total\":10,\"text\":\"hello \"");
  we("widget_event_text_6",
     {.window = 1,
      .widget = 4,
      .kind = WidgetEventKind::Text,
      .text = "wrld",
      .text_offset = 6,
      .text_total = 10},
     ",\"offset\":6,\"total\":10,\"text\":\"wrld\"");
  we("widget_event_select", {.window = 1, .widget = 5, .kind = WidgetEventKind::Select, .value = 2},
     ",\"value\":2");
  we("widget_event_activate",
     {.window = 1, .widget = 5, .kind = WidgetEventKind::Activate, .value = 2}, ",\"value\":2");
  we("widget_event_key",
     {.window = 1,
      .widget = 4,
      .kind = WidgetEventKind::Key,
      .key = kKeyLeft,
      .mods = kModShift | kModCtrl,
      .codepoint = 0},
     ",\"key\":256,\"mods\":3,\"codepoint\":0");
  we("widget_event_key_char",
     {.window = 1,
      .widget = 4,
      .kind = WidgetEventKind::Key,
      .key = 0,
      .mods = 0,
      .codepoint = 0x1F600},
     ",\"key\":0,\"mods\":0,\"codepoint\":128512");
  we("widget_event_scroll",
     {.window = 1, .widget = 4, .kind = WidgetEventKind::Scroll, .value = 120}, ",\"value\":120");
  {
    DialogResult r{.dialog = 2, .button = 0, .text = "renamed.txt"};
    v.push_back({"dialog_result_text", false, Type::DialogResult, encode_dialog_result(r),
                 "{\"dialog\":2,\"button\":0,\"text\":\"renamed.txt\"}",
                 rt(r, decode_dialog_result)});
    DialogResult d{.dialog = 2, .button = kDialogDismissed, .text = ""};
    v.push_back({"dialog_result_dismissed", false, Type::DialogResult, encode_dialog_result(d),
                 "{\"dialog\":2,\"button\":255,\"text\":\"\"}", rt(d, decode_dialog_result)});
  }
  return v;
}

static const char *kFixtureHeader =
    "# espp desktop protocol golden vectors (espp.desktop v1, dispatcher module 9).\n"
    "# Generated by components/desktop/test/desktop_host_test.cpp --gen; edit the\n"
    "# catalogue there, not this file. Shared by the C++ host test (encoders must\n"
    "# reproduce the hex, decoders must reproduce the value) and the web app's node\n"
    "# test (decode(hex) must equal the JSON for d2h, encode(JSON) the hex for h2d).\n"
    "#\n"
    "# One vector per line: name<TAB>d2h|h2d<TAB>type hex<TAB>payload hex<TAB>decoded JSON\n"
    "# (payload hex has no spaces; an empty payload is an empty field).\n"
    "#\n"
    "# JSON shapes (numbers are plain, strings are UTF-8):\n"
    "#   prop        {tag, value} where value is by tag: Text-like -> string; Value/Min/Max/Step\n"
    "#               -> int; Enabled/Visible/Focus/ItemCount/Flags/MaxLines/Width/Height/\n"
    "#               InsertBefore/WindowFlags/Color/Background -> uint; Items ->\n"
    "#               {start, items[]}; Columns -> [strings]; Geometry -> {x,y,w,h};\n"
    "#               an unknown tag (or a malformed known one) -> {tag, raw: hex}\n"
    "#   widget      {id, parent, type, weight, layout, props[]}\n"
    "#   DESKTOP     {proto, flags, records[prop], apps[{id, flags, name, icon, description}],\n"
    "#               windows[{id, app}]}\n"
    "#   WINDOW_OPEN {window, app, flags, x, y, w, h, title, total, widgets[]}\n"
    "#   WINDOW_CLOSE {window, reason}   WIDGET_SET {window, entries[{widget, props[]}]}\n"
    "#   WIDGET_ADD  {window, widgets[]}  WIDGET_REMOVE {window, widgets[ids]}\n"
    "#   DIALOG      {dialog, owner, kind, icon, title, text, default, buttons[]}\n"
    "#   NOTIFY      {level, timeoutMs, title, text}   DIALOG_CLOSE {dialog}\n"
    "#   OK          {request}   ERROR {request, errno, message}\n"
    "#   GET_DESKTOP {}   LAUNCH_APP {app}   CLOSE_WINDOW {window}\n"
    "#   WINDOW_EVENT {window, event, x, y, w, h}\n"
    "#   WIDGET_EVENT {window, widget, event} + by event: Change/Select/Activate/Scroll {value},\n"
    "#               Submit {text}, Text {offset, total, text}, Key {key, mods, codepoint}\n"
    "#   DIALOG_RESULT {dialog, button, text}\n";

static void generate(const std::string &path) {
  std::ofstream f(path, std::ios::binary);
  f << kFixtureHeader;
  for (const auto &v : catalogue()) {
    char type[8];
    std::snprintf(type, sizeof(type), "%02x", static_cast<uint8_t>(v.type));
    f << v.name << '\t' << (v.d2h ? "d2h" : "h2d") << '\t' << type << '\t' << hex(v.payload) << '\t'
      << v.json << '\n';
  }
  std::printf("wrote %s\n", path.c_str());
}

struct Line {
  std::string name, dir, type, payload, json;
};

static std::vector<Line> read_fixture(const std::string &path) {
  std::vector<Line> lines;
  std::ifstream f(path, std::ios::binary);
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#')
      continue;
    std::vector<std::string> cols;
    std::string cur;
    for (const char c : line) {
      if (c == '\t') {
        cols.push_back(cur);
        cur.clear();
      } else {
        cur.push_back(c);
      }
    }
    cols.push_back(cur);
    if (cols.size() != 5) {
      std::printf("  FAIL: malformed fixture line: %s\n", line.c_str());
      ++g_failures;
      continue;
    }
    lines.push_back({cols[0], cols[1], cols[2], cols[3], cols[4]});
  }
  return lines;
}

static void test_vectors(const std::string &path) {
  std::printf("test_vectors (%s)\n", path.c_str());
  const auto lines = read_fixture(path);
  CHECK(!lines.empty());
  std::map<std::string, Line> by_name;
  for (const auto &l : lines)
    by_name[l.name] = l;
  const auto cat = catalogue();
  CHECK(cat.size() == lines.size());
  for (const auto &v : cat) {
    const auto it = by_name.find(v.name);
    if (it == by_name.end()) {
      std::printf("  FAIL: vector '%s' missing from the fixture (regenerate with --gen)\n",
                  v.name.c_str());
      ++g_failures;
      continue;
    }
    const Line &l = it->second;
    char type[8];
    std::snprintf(type, sizeof(type), "%02x", static_cast<uint8_t>(v.type));
    if (l.dir != (v.d2h ? "d2h" : "h2d") || l.type != type) {
      std::printf("  FAIL: vector '%s' direction / type differ\n", v.name.c_str());
      ++g_failures;
    }
    // encoder reproduces the hex column
    if (hex(v.payload) != l.payload) {
      std::printf("  FAIL: vector '%s' encoder output differs from the fixture\n", v.name.c_str());
      ++g_failures;
    }
    // decoder reproduces the value from the hex column
    const auto bytes = unhex(l.payload);
    if (!v.roundtrip(bytes)) {
      std::printf("  FAIL: vector '%s' did not decode to the catalogue value\n", v.name.c_str());
      ++g_failures;
    }
    if (v.json != l.json) {
      std::printf("  FAIL: vector '%s' JSON differs from the fixture\n", v.name.c_str());
      ++g_failures;
    }
    // every strict prefix is rejected or decodes to something else (no
    // decoder silently accepts a truncated payload as the full value)
    for (size_t n = 0; n < bytes.size(); ++n) {
      if (v.roundtrip(std::span<const uint8_t>(bytes.data(), n))) {
        std::printf("  FAIL: vector '%s' prefix of %zu bytes decodes as the full value\n",
                    v.name.c_str(), n);
        ++g_failures;
        break;
      }
    }
    // the payload builds a frame (fits the codec) that parses back
    const auto frame = dp::build_frame(
        v.type, bytes, dp::kModule, v.d2h ? std::optional<uint16_t>{} : std::optional<uint16_t>{7});
    const auto frames = sf::StreamParser{}.feed(frame);
    CHECK(frames.size() == 1 && frames[0].payload == bytes && frames[0].is_reply() == v.d2h &&
          frames[0].type == static_cast<uint8_t>(v.type));
  }
}

// ---- codec behaviour beyond the goldens --------------------------------------------------------

static void test_truncation_and_unknown() {
  std::printf("test_truncation_and_unknown\n");
  using namespace dp;
  // an unknown prop tag is kept raw and does not disturb the next one
  WidgetSet s{.window = 1,
              .entries = {{.widget = 2,
                           .props = {Prop{.tag = 250, .value = {1, 2, 3}},
                                     Prop::i32(PropTag::Value, -5)}}}};
  const auto enc = encode_widget_set(s, 4081);
  CHECK(enc.size() == 1);
  const auto dec = decode_widget_set(enc[0]);
  CHECK(dec && dec->entries.size() == 1 && dec->entries[0].props.size() == 2);
  CHECK(dec && dec->entries[0].props[0].kind() == PropKind::Unknown &&
        dec->entries[0].props[1].as_i32() == -5);
  // a known tag with the wrong length is kept raw and flagged invalid
  Prop bad{.tag = static_cast<uint8_t>(PropTag::Value), .value = {1, 2}};
  CHECK(!bad.valid() && !bad.as_i32());
  CHECK(Prop::i32(PropTag::Value, 1).valid() && Prop::geometry({}).valid());
  CHECK(Prop::items(0, std::vector<std::string>{"a"}).valid());
  // a rec whose len runs past the payload is malformed
  std::vector<uint8_t> p = {1, 0, 1, 2, 0, 1, 0x05, 0, 'a'};
  CHECK(!decode_widget_set(p));
  // trailing bytes are malformed too
  p = enc[0];
  p.push_back(0);
  CHECK(!decode_widget_set(p));
  // WIDGET_EVENT: wrong value sizes / unknown kinds are rejected
  p = {1, 0, 2, 0, 2, 1, 0}; // Change with 2 value bytes
  CHECK(!decode_widget_event(p));
  p = {1, 0, 2, 0, 1, 9}; // Click with trailing byte
  CHECK(!decode_widget_event(p));
  p = {1, 0, 2, 0, 99}; // unknown kind
  CHECK(!decode_widget_event(p));
  p = {1, 0, 2, 0, 3}; // Submit with empty text is fine
  CHECK(decode_widget_event(p) && decode_widget_event(p)->text.empty());
  // WINDOW_EVENT is exactly 11 bytes
  p = encode_window_event({.window = 1, .kind = WindowEventKind::Focus});
  CHECK(p.size() == 11 && decode_window_event(p));
  p.push_back(0);
  CHECK(!decode_window_event(p));
  // DIALOG_RESULT needs at least 3 bytes
  p = {1, 0};
  CHECK(!decode_dialog_result(p));
  // str8 truncates at 255, str16 at 65535
  std::vector<uint8_t> o;
  put_str8(o, std::string(300, 'x'));
  CHECK(o.size() == 256 && o[0] == 255);
  o.clear();
  put_str16(o, std::string(70000, 'x'));
  CHECK(o.size() == 65537 && o[0] == 0xFF && o[1] == 0xFF);
  // Reader
  const uint8_t two[] = {0x34, 0x12};
  Reader r(two);
  CHECK(r.u16() == 0x1234 && r.ok() && r.at_end());
  CHECK(r.u8() == 0 && !r.ok());
  // max_payload_for
  CHECK(max_payload_for(4096) == 4081 && max_payload_for(10) == 0 &&
        max_payload_for(100000) == sf::kMaxPayloadSize);
}

static void test_widget_set_splitting() {
  std::printf("test_widget_set_splitting\n");
  using namespace dp;
  const size_t cap = 64;
  // a Text longer than the cap splits into Text + TextAppend pieces, in order
  std::string text;
  for (int i = 0; i < 20; ++i)
    text += "abcdefghij"; // 200 bytes
  WidgetSet s{.window = 3, .entries = {{.widget = 4, .props = {Prop::text(PropTag::Text, text)}}}};
  size_t dropped = 0;
  const auto frames = encode_widget_set(s, cap, &dropped);
  CHECK(dropped == 0 && frames.size() >= 4);
  std::string joined;
  for (size_t i = 0; i < frames.size(); ++i) {
    CHECK(frames[i].size() <= cap);
    const auto d = decode_widget_set(frames[i]);
    CHECK(d && d->window == 3 && d->entries.size() == 1 && d->entries[0].widget == 4 &&
          d->entries[0].props.size() == 1);
    if (!d)
      continue;
    const auto &p = d->entries[0].props[0];
    CHECK(p.is(i == 0 ? PropTag::Text : PropTag::TextAppend));
    joined += p.as_text();
  }
  CHECK(joined == text);
  // a multi-byte UTF-8 sequence is never cut: pieces end on sequence boundaries
  std::string emoji;
  for (int i = 0; i < 30; ++i)
    emoji += "\xF0\x9F\x98\x80"; // 120 bytes of 4-byte sequences
  s.entries[0].props = {Prop::text(PropTag::Text, emoji)};
  joined.clear();
  for (const auto &f : encode_widget_set(s, cap)) {
    const auto d = decode_widget_set(f);
    CHECK(d);
    if (!d)
      continue;
    const auto t = d->entries[0].props[0].as_text();
    CHECK(t.size() % 4 == 0);
    joined += t;
  }
  CHECK(joined == emoji);
  // Items split into ranges with advancing start
  std::vector<std::string> items;
  for (int i = 0; i < 40; ++i)
    items.push_back("item" + std::to_string(i));
  s.entries[0].props = {Prop::u16(PropTag::ItemCount, 40), Prop::items(0, items)};
  const auto iframes = encode_widget_set(s, cap, &dropped);
  CHECK(dropped == 0 && iframes.size() > 1);
  std::vector<std::string> got;
  size_t next_start = 0;
  bool count_first = false;
  for (size_t i = 0; i < iframes.size(); ++i) {
    CHECK(iframes[i].size() <= cap);
    const auto d = decode_widget_set(iframes[i]);
    CHECK(d);
    if (!d)
      continue;
    for (const auto &e : d->entries)
      for (const auto &p : e.props) {
        if (p.is(PropTag::ItemCount)) {
          count_first = got.empty();
        } else {
          const auto iv = p.as_items();
          CHECK(iv && iv->start == next_start);
          if (iv) {
            got.insert(got.end(), iv->items.begin(), iv->items.end());
            next_start += iv->items.size();
          }
        }
      }
  }
  CHECK(count_first && got == items);
  // several widgets, more entries than fit: entries continue in later frames
  WidgetSet many{.window = 1};
  for (uint16_t w = 1; w <= 30; ++w)
    many.entries.push_back({.widget = w, .props = {Prop::i32(PropTag::Value, w)}});
  const auto mframes = encode_widget_set(many, cap, &dropped);
  CHECK(dropped == 0 && mframes.size() > 1);
  size_t seen = 0;
  for (const auto &f : mframes) {
    CHECK(f.size() <= cap);
    const auto d = decode_widget_set(f);
    CHECK(d);
    if (d)
      for (const auto &e : d->entries) {
        ++seen;
        CHECK(e.widget == seen && e.props.size() == 1 && e.props[0].as_i32() == (int32_t)seen);
      }
  }
  CHECK(seen == 30);
  // an unsplittable prop larger than an empty frame is dropped, the rest survives
  std::vector<std::string> cols;
  for (int i = 0; i < 40; ++i)
    cols.push_back("col" + std::to_string(i));
  WidgetSet big{
      .window = 1,
      .entries = {{.widget = 2, .props = {Prop::columns(cols), Prop::i32(PropTag::Value, 1)}}}};
  dropped = 0;
  const auto bframes = encode_widget_set(big, cap, &dropped);
  CHECK(dropped == 1 && bframes.size() == 1);
  const auto bd = decode_widget_set(bframes[0]);
  CHECK(bd && bd->entries.size() == 1 && bd->entries[0].props.size() == 1 &&
        bd->entries[0].props[0].is(PropTag::Value));
  // nothing added -> no frames
  CHECK(encode_widget_set({.window = 1}, cap).empty());
  // an entry's props never exceed 255: the 256th goes into a new entry
  WidgetSet lots{.window = 1, .entries = {{.widget = 9}}};
  for (int i = 0; i < 300; ++i)
    lots.entries[0].props.push_back(Prop::u8(PropTag::Enabled, 1));
  size_t props_seen = 0;
  for (const auto &f : encode_widget_set(lots, 4081)) {
    const auto d = decode_widget_set(f);
    CHECK(d);
    if (d)
      props_seen =
          std::accumulate(d->entries.begin(), d->entries.end(), props_seen,
                          [](size_t n, const dp::WidgetSetEntry &e) { return n + e.props.size(); });
  }
  CHECK(props_seen == 300);
}

static void test_window_open_splitting() {
  std::printf("test_window_open_splitting\n");
  using namespace dp;
  const size_t cap = 96;
  WindowOpen o;
  o.id = 5;
  o.app = 2;
  o.title = "Split";
  o.geometry = {.x = 10, .y = 10, .w = 400, .h = 300};
  std::string text;
  for (int i = 0; i < 25; ++i)
    text += "0123456789";
  for (uint16_t i = 1; i <= 12; ++i) {
    WidgetRec w{.id = i,
                .parent = static_cast<uint16_t>(i == 1 ? 0 : 1),
                .type = i == 1 ? WidgetType::Column : WidgetType::Label};
    if (i == 6)
      w.props = {Prop::text(PropTag::Text, text), Prop::u16(PropTag::Flags, kLabelWrap)};
    else if (i > 1)
      w.props = {Prop::text(PropTag::Text, "label " + std::to_string(i))};
    o.widgets.push_back(w);
  }
  size_t dropped = 0;
  const auto msgs = encode_window_open(o, cap, &dropped);
  CHECK(dropped == 0 && msgs.size() > 2);
  CHECK(!msgs.empty() && msgs[0].type == Type::WindowOpen);
  size_t received = 0;
  bool adds_before_sets = true, seen_set = false;
  std::string text6;
  uint16_t flags6 = 0;
  for (size_t i = 0; i < msgs.size(); ++i) {
    const auto &m = msgs[i];
    CHECK(m.payload.size() <= cap);
    if (m.type == Type::WindowOpen) {
      const auto d = decode_window_open(m.payload);
      CHECK(d && d->id == 5 && d->app == 2 && d->title == "Split" && d->total == 12 &&
            d->geometry == o.geometry && d->flags == kWinDefaultFlags);
      if (d)
        for (const auto &w : d->widgets) {
          ++received;
          CHECK(w.id == received);
          for (const auto &p : w.props)
            if (w.id == 6 && p.is(PropTag::Text))
              text6 += p.as_text();
        }
    } else if (m.type == Type::WidgetAdd) {
      if (seen_set)
        adds_before_sets = false;
      const auto d = decode_widget_add(m.payload);
      CHECK(d && d->window == 5);
      if (d)
        for (const auto &w : d->widgets) {
          ++received;
          CHECK(w.id == received);
          for (const auto &p : w.props)
            if (w.id == 6 && p.is(PropTag::Text))
              text6 += p.as_text();
        }
    } else {
      CHECK(m.type == Type::WidgetSet);
      seen_set = true;
      const auto d = decode_widget_set(m.payload);
      CHECK(d && d->window == 5);
      if (d)
        for (const auto &e : d->entries) {
          CHECK(e.widget == 6);
          for (const auto &p : e.props) {
            if (p.is(PropTag::TextAppend))
              text6 += p.as_text();
            if (p.is(PropTag::Flags))
              flags6 = *p.as_u16();
          }
        }
    }
  }
  CHECK(received == 12 && adds_before_sets && text6 == text && flags6 == kLabelWrap);
  // an empty window still produces exactly one WINDOW_OPEN
  WindowOpen empty{.id = 9, .app = 1, .title = "Empty"};
  const auto em = encode_window_open(empty, cap);
  CHECK(em.size() == 1 && em[0].type == Type::WindowOpen);
  const auto ed = decode_window_open(em[0].payload);
  CHECK(ed && ed->total == 0 && ed->widgets.empty());
  // WIDGET_ADD of many widgets splits into several add frames
  WidgetAdd add{.window = 9};
  for (uint16_t i = 1; i <= 20; ++i)
    add.widgets.push_back({.id = i, .type = WidgetType::Separator});
  const auto am = encode_widget_add(add, cap);
  CHECK(am.size() > 1);
  size_t n = 0;
  for (const auto &m : am) {
    CHECK(m.type == Type::WidgetAdd && m.payload.size() <= cap);
    const auto d = decode_widget_add(m.payload);
    CHECK(d);
    if (d)
      n += d->widgets.size();
  }
  CHECK(n == 20);
  CHECK(encode_widget_add({.window = 9}, cap).empty());
  // WIDGET_REMOVE splits too
  WidgetRemove rm{.window = 9};
  for (uint16_t i = 1; i <= 100; ++i)
    rm.widgets.push_back(i);
  const auto rf = encode_widget_remove(rm, 24);
  CHECK(rf.size() == 10);
  for (const auto &f : rf)
    CHECK(f.size() <= 24 && decode_widget_remove(f));
}

static void test_dialog_notify_limits() {
  std::printf("test_dialog_notify_limits\n");
  using namespace dp;
  // dialogs and notifications are single frames: the encoders never cut
  // anything, the model refuses one that would not fit its max_payload
  Dialog d{.id = 1,
           .title = "T",
           .text = std::string(100, 't'),
           .default_text = std::string(50, 'd'),
           .buttons = {"OK"}};
  CHECK(decode_dialog(encode_dialog(d)) == d);
  Notify n{
      .level = NotifyLevel::Error, .timeout_ms = 0, .title = "Oops", .text = std::string(500, 'n')};
  CHECK(decode_notify(encode_notify(n)) == n);
  dm::Model m;
  m.max_payload = 120;
  CHECK(m.open_dialog({.title = "fits", .text = "short"}) != 0);
  CHECK(m.open_dialog(d) == 0); // 166 bytes > 120: refused, nothing queued
  CHECK(m.dirty.dialogs_opened.size() == 1);
  CHECK(m.notify({.title = "n", .text = "ok"}) && !m.notify(n));
  CHECK(m.dirty.notifications.size() == 1);
  // DESKTOP trims (windows, then descriptions, then apps) rather than overflow
  DesktopInfo big;
  for (uint8_t i = 1; i <= 3; ++i)
    big.apps.push_back({.id = i, .name = "app", .icon = "i", .description = std::string(40, 'd')});
  for (uint16_t w = 1; w <= 20; ++w)
    big.windows.push_back({.id = w, .app = 1});
  bool trimmed = false;
  CHECK(encode_desktop(big, 4081, &trimmed).size() == 3 + 1 + 3 * (2 + 4 + 2 + 41) + 1 + 60 &&
        !trimmed);
  // descriptions go first (the window list stays complete: bit1 set) ...
  auto t = decode_desktop(encode_desktop(big, 160, &trimmed));
  CHECK(trimmed && t && t->apps.size() == 3 && t->windows.size() == 20 &&
        t->apps[0].description.empty() && (t->flags & kDesktopWindowListComplete));
  // ... then apps, and the window list only last (bit1 cleared)
  t = decode_desktop(encode_desktop(big, 60, &trimmed));
  CHECK(trimmed && t && t->apps.empty() && t->windows.size() == 18 &&
        !(t->flags & kDesktopWindowListComplete));
  t = decode_desktop(encode_desktop(big, 20, &trimmed));
  CHECK(trimmed && t && t->apps.empty() && t->windows.size() == 5);
  // a full window list always carries bit1, whatever the caller put in flags
  t = decode_desktop(encode_desktop(big, 4081));
  CHECK(t && (t->flags & kDesktopWindowListComplete) && t->windows.size() == 20);
  // the registry limits keep a maximal registry within the default cap
  DesktopInfo maxed;
  maxed.records = {
      Prop::text(static_cast<PropTag>(DesktopTag::DeviceName),
                 std::string(kMaxDeviceNameBytes, 'n')),
      Prop::text(static_cast<PropTag>(DesktopTag::Firmware), std::string(kMaxFirmwareBytes, 'f')),
      Prop::text(static_cast<PropTag>(DesktopTag::Theme), "light"),
      Prop::u32(static_cast<PropTag>(DesktopTag::Accent), 0),
      Prop::u16(static_cast<PropTag>(DesktopTag::MaxPayload), 4081),
      Prop::u16(static_cast<PropTag>(DesktopTag::FlushPeriodMs), 50)};
  for (size_t i = 1; i <= kMaxApps; ++i)
    maxed.apps.push_back({.id = static_cast<uint8_t>(i),
                          .name = std::string(kMaxAppNameBytes, 'n'),
                          .icon = std::string(kMaxAppIconBytes, 'i'),
                          .description = std::string(kMaxAppDescriptionBytes, 'd')});
  for (uint16_t w = 1; w <= 255; ++w)
    maxed.windows.push_back({.id = w, .app = 1});
  CHECK(encode_desktop(maxed, 4081, &trimmed).size() <= 4081 && !trimmed);
  // the maximal mandatory record set alone always fits the smallest cap
  DesktopInfo records_only;
  records_only.records = maxed.records;
  CHECK(records_only.records[2].value.size() == kMaxThemeBytes);
  const auto minimal = encode_desktop(records_only, kMinPayloadBytes, &trimmed);
  CHECK(!trimmed && minimal.size() == kDesktopRecordsMaxBytes &&
        minimal.size() <= kMinPayloadBytes);
  // representability is checked before anything is cut into a str8 / str16
  dm::Model lim;
  CHECK(lim.open_dialog({.title = std::string(256, 't')}) == 0);
  CHECK(lim.open_dialog({.title = "ok", .buttons = {"fine", std::string(256, 'b')}}) == 0);
  CHECK(lim.open_dialog({.title = "ok", .buttons = std::vector<std::string>(256, "b")}) == 0);
  CHECK(lim.open_dialog({.title = "ok", .text = std::string(65536, 'x')}) == 0);
  CHECK(lim.open_dialog({.title = std::string(255, 't'), .buttons = {std::string(255, 'b')}}) != 0);
  CHECK(!lim.notify({.title = std::string(256, 'n')}));
  CHECK(!lim.notify({.title = "n", .text = std::string(65536, 'x')}));
  CHECK(lim.notify({.title = std::string(255, 'n')}));
  // unsplittable widget values are refused at the model, never cut
  const uint16_t lw = lim.create_window({.title = "w"});
  CHECK(lim.create_window({.title = std::string(256, 'w')}) == 0 &&
        lim.create_window({.title = std::string(255, 'w')}) != 0);
  const uint16_t lbl = lim.add_widget(lw, {.type = WidgetType::Label, .text = "x"});
  CHECK(lbl != 0);
  CHECK(!lim.set_prop(lw, 0, Prop::text(PropTag::Title, std::string(256, 't'))));
  CHECK(lim.set_prop(lw, 0, Prop::text(PropTag::Title, std::string(255, 't'))));
  CHECK(!lim.set_prop(lw, lbl, Prop::text(PropTag::Placeholder, std::string(256, 'p'))));
  CHECK(!lim.set_prop(lw, lbl, Prop::text(PropTag::Tooltip, std::string(256, 'p'))));
  CHECK(lim.set_prop(lw, lbl, Prop::text(PropTag::Text, std::string(5000, 'x')))); // splits
  // (Prop::columns would cut a 256th name / a 256-byte name to str8 limits,
  // so the string-level set_columns is the gate; the Prop-level check still
  // bounds the record)
  CHECK(!lim.set_columns(lw, lbl, std::vector<std::string>(256, "c")));
  CHECK(!lim.set_columns(lw, lbl, {std::string(256, 'c')}));
  CHECK(lim.set_columns(lw, lbl, {std::string(255, 'c')}));
  CHECK(lim.set_prop(lw, lbl, Prop::columns(std::vector<std::string>{std::string(255, 'c')})));
  CHECK(!lim.set_prop(lw, lbl, Prop::items(0, std::vector<std::string>{std::string(5000, 'i')})));
  CHECK(lim.set_prop(lw, lbl, Prop::items(0, std::vector<std::string>{std::string(4000, 'i')})));
  CHECK(lim.add_widget(lw, {.type = WidgetType::TextBox, .placeholder = std::string(256, 'p')}) ==
        0);
  CHECK(lim.add_widget(lw, {.type = WidgetType::Table, .columns = {std::string(256, 'c')}}) == 0);
  CHECK(lim.add_widget(lw, {.type = WidgetType::List, .items = {std::string(5000, 'i')}}) == 0);
  // a small cap bounds Columns / Items entries accordingly
  lim.max_payload = kMinPayloadBytes;
  CHECK(!lim.set_prop(lw, lbl, Prop::columns(std::vector<std::string>(3, std::string(100, 'c')))));
  CHECK(lim.set_prop(lw, lbl, Prop::columns(std::vector<std::string>(2, std::string(100, 'c')))));
  CHECK(!lim.set_prop(lw, lbl, Prop::items(0, std::vector<std::string>{std::string(240, 'i')})));
  CHECK(lim.set_prop(
      lw, lbl,
      Prop::items(0, std::vector<std::string>{std::string(200, 'i'), std::string(200, 'j')})));
  // app ids are monotonic and never reused while a window still references one
  CHECK(lim.allocate_app_id() == 1);
  lim.register_app({.id = 1, .name = "a"});
  const uint16_t aw = lim.create_window({.title = "of app 1", .app = 1});
  CHECK(aw != 0 && lim.unregister_app(1));
  CHECK(lim.allocate_app_id() == 2); // 1 is still referenced by the window
  lim.close_window(aw, WindowCloseReason::App);
  CHECK(lim.allocate_app_id() == 3); // monotonic: 1 comes back only after a wrap
}

static void test_frames_and_correlation() {
  std::printf("test_frames_and_correlation\n");
  using namespace dp;
  // replies carry the reply flag, requests do not; module is stamped
  const auto req = build_frame(Type::LaunchApp, encode_launch_app({.app = 1}), 9);
  CHECK(req[2] == 0x10 && req[3] == 9 && req[4] == 0x02);
  const auto rep = build_frame(Type::Desktop, {}, 11);
  CHECK(rep[2] == 0x11 && rep[3] == 11 && rep[4] == 0x81);
  // correlation: replies (DESKTOP, OK, ERROR) echo it, events carry none
  const auto creq = build_frame(Type::GetDesktop, {}, 9, 0x1234);
  const auto cf = sf::StreamParser{}.feed(creq);
  CHECK(cf.size() == 1 && cf[0].correlation == std::optional<uint16_t>(0x1234));
  for (const auto t : {Type::Desktop, Type::Ok, Type::Error}) {
    const auto crep = build_frame(t, encode_ok(1), 9, cf[0].correlation);
    const auto cr = sf::StreamParser{}.feed(crep);
    CHECK(cr.size() == 1 && cr[0].is_reply() &&
          cr[0].correlation == std::optional<uint16_t>(0x1234));
  }
  const auto ev = sf::StreamParser{}.feed(build_frame(Type::Notify, encode_notify({}), 9));
  CHECK(ev.size() == 1 && !ev[0].has_correlation());
  CHECK(is_reply(Type::Ok) && !is_reply(Type::WidgetEvent));
}

// ---- model: text reassembly + dirty tracking --------------------------------------------------

static void test_text_assembler() {
  std::printf("test_text_assembler\n");
  dm::TextAssembler ta(32);
  std::string out;
  // two chunks in order complete the text
  CHECK(ta.feed(1, 4, {.text = "hello ", .text_offset = 0, .text_total = 10}, out) ==
        dm::TextAssembler::Result::Partial);
  CHECK(ta.feed(1, 4, {.text = "wrld", .text_offset = 6, .text_total = 10}, out) ==
        dm::TextAssembler::Result::Complete);
  CHECK(out == "hello wrld");
  // a single full chunk completes at once
  CHECK(ta.feed(1, 4, {.text = "x", .text_offset = 0, .text_total = 1}, out) ==
            dm::TextAssembler::Result::Complete &&
        out == "x");
  // an empty text completes at once
  CHECK(ta.feed(1, 4, {.text = "", .text_offset = 0, .text_total = 0}, out) ==
            dm::TextAssembler::Result::Complete &&
        out.empty());
  // an out-of-order chunk resets the buffer (rejected)
  CHECK(ta.feed(1, 4, {.text = "ab", .text_offset = 0, .text_total = 4}, out) ==
        dm::TextAssembler::Result::Partial);
  CHECK(ta.feed(1, 4, {.text = "cd", .text_offset = 3, .text_total = 4}, out) ==
        dm::TextAssembler::Result::Rejected);
  CHECK(ta.feed(1, 4, {.text = "ab", .text_offset = 0, .text_total = 4}, out) ==
        dm::TextAssembler::Result::Partial);
  CHECK(ta.feed(1, 4, {.text = "cd", .text_offset = 2, .text_total = 4}, out) ==
            dm::TextAssembler::Result::Complete &&
        out == "abcd");
  // over the byte bound: rejected
  CHECK(ta.feed(1, 4, {.text = "0123456789", .text_offset = 0, .text_total = 40}, out) ==
        dm::TextAssembler::Result::Rejected);
  // chunks for different widgets do not mix; a closed window drops its buffers
  CHECK(ta.feed(1, 4, {.text = "ab", .text_offset = 0, .text_total = 4}, out) ==
        dm::TextAssembler::Result::Partial);
  CHECK(ta.feed(1, 5, {.text = "zz", .text_offset = 0, .text_total = 2}, out) ==
            dm::TextAssembler::Result::Complete &&
        out == "zz");
  ta.forget_window(1);
  CHECK(ta.feed(1, 4, {.text = "cd", .text_offset = 2, .text_total = 4}, out) ==
        dm::TextAssembler::Result::Rejected);
}

static void test_dirty_tracker() {
  std::printf("test_dirty_tracker\n");
  using namespace dp;
  dm::DirtyTracker t;
  CHECK(!t.any());
  // last-wins for a tag; TextAppend concatenates; Text cancels earlier appends
  t.set(1, 3, Prop::i32(PropTag::Value, 1));
  t.set(1, 3, Prop::i32(PropTag::Value, 2));
  t.set(1, 3, Prop::text(PropTag::TextAppend, "a"));
  t.set(1, 3, Prop::text(PropTag::TextAppend, "b"));
  CHECK(t.any());
  {
    auto *w = t.window(1);
    CHECK(w && w->sets.size() == 1 && w->sets[0].widget == 3 && w->sets[0].props.size() == 2);
    CHECK(w && w->sets[0].props[0].as_i32() == 2 && w->sets[0].props[1].as_text() == "ab");
  }
  t.set(1, 3, Prop::text(PropTag::Text, "fresh"));
  t.set(1, 3, Prop::text(PropTag::TextAppend, "!"));
  {
    auto *w = t.window(1);
    CHECK(w && w->sets[0].props.size() == 3 && w->sets[0].props[1].is(PropTag::Text) &&
          w->sets[0].props[1].as_text() == "fresh" && w->sets[0].props[2].as_text() == "!");
  }
  // Items ranges accumulate in order; ItemCount discards earlier ranges
  t.set(1, 5, Prop::items(2, std::vector<std::string>{"c"}));
  t.set(1, 5, Prop::items(0, std::vector<std::string>{"a"}));
  {
    auto *w = t.window(1);
    CHECK(w && w->sets.size() == 2 && w->sets[1].props.size() == 2);
  }
  t.set(1, 5, Prop::u16(PropTag::ItemCount, 1));
  t.set(1, 5, Prop::items(0, std::vector<std::string>{"z"}));
  {
    auto *w = t.window(1);
    CHECK(w && w->sets[1].props.size() == 2 && w->sets[1].props[0].is(PropTag::ItemCount) &&
          w->sets[1].props[1].as_items()->items == std::vector<std::string>{"z"});
  }
  // a remove cancels pending sets and is recorded; a set after it is ignored
  t.remove(1, 3, true);
  t.set(1, 3, Prop::i32(PropTag::Value, 9));
  {
    auto *w = t.window(1);
    CHECK(w && w->sets.size() == 1 && w->sets[0].widget == 5 &&
          w->removed == std::vector<uint16_t>{3});
  }
  // a descendant removed "under" its parent is cancelled but not listed
  t.set(1, 7, Prop::u8(PropTag::Visible, 0));
  t.remove(1, 7, false);
  {
    auto *w = t.window(1);
    CHECK(w && w->sets.size() == 1 && w->removed.size() == 1);
  }
  // an added widget absorbs its sets (its full state is encoded at flush) and
  // a remove of a pending add cancels both
  t.add(1, 8);
  t.set(1, 8, Prop::i32(PropTag::Value, 1));
  {
    auto *w = t.window(1);
    CHECK(w && w->added == std::vector<uint16_t>{8} && w->sets.size() == 1);
  }
  t.remove(1, 8, true);
  {
    auto *w = t.window(1);
    CHECK(w && w->added.empty() && w->removed == std::vector<uint16_t>{3});
  }
  // a pending open discards everything else for that window; a close of a
  // never-flushed open drops the window silently
  t.open(2);
  t.add(2, 1);
  t.set(2, 1, Prop::i32(PropTag::Value, 1));
  t.set(2, 0, Prop::text(PropTag::Title, "x"));
  {
    auto *w = t.window(2);
    CHECK(w && w->open && w->added.empty() && w->sets.empty());
  }
  t.close(2, WindowCloseReason::App);
  CHECK(!t.window(2));
  // a close of a flushed window discards its other dirt and is recorded
  t.set(1, 5, Prop::i32(PropTag::Value, 3));
  t.close(1, WindowCloseReason::Host);
  {
    auto *w = t.window(1);
    CHECK(w && w->close && w->close_reason == WindowCloseReason::Host && w->sets.empty() &&
          w->removed.empty());
  }
  // dialogs: a close before the flush cancels the open; notifications queue
  t.dialog_open(1);
  t.dialog_open(2);
  t.dialog_close(1);
  CHECK(t.dialogs_opened == std::vector<uint16_t>{2} && t.dialogs_closed.empty());
  t.dialog_close(2);
  CHECK(t.dialogs_opened.empty() && t.dialogs_closed.empty());
  t.dialog_close(3); // an already-flushed dialog
  CHECK(t.dialogs_closed == std::vector<uint16_t>{3});
  t.notify({.level = NotifyLevel::Ok, .title = "a"});
  t.notify({.level = NotifyLevel::Ok, .title = "b"});
  CHECK(t.notifications.size() == 2);
  t.desktop_changed = true;
  t.clear();
  CHECK(!t.any() && !t.window(1) && t.notifications.empty() && !t.desktop_changed);
}

static void test_model_flush() {
  std::printf("test_model_flush\n");
  using namespace dp;
  dm::Model m;
  m.max_payload = 4081;
  // a window with a tree, flushed: one WINDOW_OPEN with the full tree
  const uint16_t win = m.create_window({.title = "W", .app = 1, .flags = kWinDefaultFlags});
  CHECK(win == 1);
  const uint16_t col = m.add_widget(win, {.type = WidgetType::Column, .weight = 1});
  const uint16_t lbl = m.add_widget(win, {.type = WidgetType::Label, .parent = col, .text = "hi"});
  const uint16_t sl = m.add_widget(
      win, {.type = WidgetType::Slider, .parent = col, .value = 5, .min = 0, .max = 10, .step = 1});
  CHECK(col == 1 && lbl == 2 && sl == 3);
  m.set_prop(win, lbl, Prop::text(PropTag::Text, "ignored: pending open"));
  auto out = m.flush();
  CHECK(out.size() == 1 && out[0].type == Type::WindowOpen);
  auto o = decode_window_open(out[0].payload);
  CHECK(o && o->id == 1 && o->app == 1 && o->title == "W" && o->total == 3 &&
        o->widgets.size() == 3);
  CHECK(o && o->widgets[1].props.size() == 1 &&
        o->widgets[1].props[0].as_text() == "ignored: pending open");
  // the slider carries Value/Min/Max/Step; the label just its text
  CHECK(o && o->widgets[2].props.size() == 4 && o->widgets[2].props[0].as_i32() == 5 &&
        o->widgets[2].props[2].as_i32() == 10);
  CHECK(m.flush().empty());
  // now sets coalesce into one WIDGET_SET; appends to a TextArea update the model
  m.set_prop(win, lbl, Prop::text(PropTag::Text, "a"));
  m.set_prop(win, lbl, Prop::text(PropTag::Text, "b"));
  m.set_prop(win, 0, Prop::text(PropTag::Title, "W2"));
  out = m.flush();
  CHECK(out.size() == 1 && out[0].type == Type::WidgetSet);
  auto s = decode_widget_set(out[0].payload);
  CHECK(s && s->entries.size() == 2 && s->entries[0].widget == lbl &&
        s->entries[0].props.size() == 1 && s->entries[0].props[0].as_text() == "b");
  CHECK(s && s->entries[1].widget == 0 && s->entries[1].props[0].as_text() == "W2");
  CHECK(m.widget(win, lbl)->text == "b" && m.window(win)->title == "W2");
  // add + remove ordering: close -> open -> add -> set -> remove
  const uint16_t btn = m.add_widget(win, {.type = WidgetType::Button, .parent = col, .text = "B"});
  m.set_prop(win, sl, Prop::i32(PropTag::Value, 7));
  m.remove_widget(win, lbl);
  const uint16_t win2 = m.create_window({.title = "Second", .app = 2});
  out = m.flush();
  CHECK(out.size() == 4);
  CHECK(out.size() == 4 && out[0].type == Type::WidgetAdd && out[1].type == Type::WidgetSet &&
        out[2].type == Type::WidgetRemove && out[3].type == Type::WindowOpen);
  {
    const auto a = decode_widget_add(out[0].payload);
    CHECK(a && a->widgets.size() == 1 && a->widgets[0].id == btn && a->widgets[0].parent == col);
    const auto r = decode_widget_remove(out[2].payload);
    CHECK(r && r->widgets == std::vector<uint16_t>{lbl});
    const auto w2 = decode_window_open(out[3].payload);
    CHECK(w2 && w2->id == win2 && w2->total == 0);
  }
  CHECK(!m.widget(win, lbl) && m.widget(win, sl)->value == 7);
  // removing a container removes its children from the model, one id on the wire
  const uint16_t row = m.add_widget(win, {.type = WidgetType::Row, .parent = col});
  const uint16_t c1 = m.add_widget(win, {.type = WidgetType::Label, .parent = row, .text = "c1"});
  m.flush();
  m.set_prop(win, c1, Prop::text(PropTag::Text, "x"));
  m.remove_widget(win, row);
  out = m.flush();
  CHECK(out.size() == 1 && out[0].type == Type::WidgetRemove);
  CHECK(!m.widget(win, row) && !m.widget(win, c1) && m.widget(win, btn));
  // closing a window: WINDOW_CLOSE, and the window is gone from the model
  m.set_prop(win2, 0, Prop::text(PropTag::Title, "dropped"));
  m.close_window(win2, WindowCloseReason::Host);
  out = m.flush();
  CHECK(out.size() == 1 && out[0].type == Type::WindowClose);
  CHECK(decode_window_close(out[0].payload)->reason == WindowCloseReason::Host && !m.window(win2));
  // a window created and closed between flushes sends nothing
  const uint16_t win3 = m.create_window({.title = "blink", .app = 1});
  m.close_window(win3, WindowCloseReason::App);
  CHECK(m.flush().empty() && !m.window(win3));
  // snapshot: the full tree of every open window with the Snapshot flag
  auto snap = m.snapshot();
  CHECK(snap.size() == 1 && snap[0].type == Type::WindowOpen);
  const auto so = decode_window_open(snap[0].payload);
  CHECK(so && (so->flags & kWinSnapshot) && so->total == 3 && so->widgets[0].id == col);
  // host events update the model: geometry, value, text
  m.apply_window_event(
      {.window = win, .kind = WindowEventKind::Moved, .x = 5, .y = 6, .w = 7, .h = 8});
  CHECK((m.window(win)->geometry == Geometry{.x = 5, .y = 6, .w = 7, .h = 8}));
  m.apply_widget_event({.window = win, .widget = sl, .kind = WidgetEventKind::Change, .value = 3});
  CHECK(m.widget(win, sl)->value == 3);
  CHECK(m.flush().empty()); // host-originated changes are not echoed back
  // TextArea append bounded by max_lines and max_text_bytes
  m.max_text_bytes = 64;
  const uint16_t ta =
      m.add_widget(win, {.type = WidgetType::TextArea, .parent = col, .max_lines = 3});
  m.flush();
  for (int i = 0; i < 5; ++i)
    m.append_text(win, ta, "line" + std::to_string(i) + "\n");
  // the line bound counts the (empty) text after the last newline as a line,
  // exactly like the browser's ring, so three lines = "line3\nline4\n" + ""
  CHECK(m.widget(win, ta)->text == "line3\nline4\n");
  out = m.flush();
  CHECK(out.size() == 1);
  s = decode_widget_set(out[0].payload);
  CHECK(s && s->entries.size() == 1 && s->entries[0].props.size() == 1 &&
        s->entries[0].props[0].is(PropTag::TextAppend) &&
        s->entries[0].props[0].as_text() == "line0\nline1\nline2\nline3\nline4\n");
  m.append_text(win, ta, std::string(100, 'x'));
  CHECK(m.widget(win, ta)->text.size() <= 64);
  // the pending append grew past max_text_bytes: it became a Text (replace)
  // of the bounded text, cancelling the append
  out = m.flush();
  s = out.size() == 1 ? decode_widget_set(out[0].payload) : std::nullopt;
  CHECK(s && s->entries.size() == 1 && s->entries[0].props.size() == 1 &&
        s->entries[0].props[0].is(PropTag::Text) &&
        s->entries[0].props[0].as_text() == m.widget(win, ta)->text);
  // desktop info + apps
  m.register_app({.id = 1, .name = "A", .icon = "a"});
  m.register_app({.id = 2, .flags = kAppHidden, .name = "B", .icon = "b"});
  CHECK(m.dirty.desktop_changed);
  out = m.flush();
  CHECK(out.size() == 1 && out[0].type == Type::Desktop);
  const auto d = decode_desktop(out[0].payload);
  CHECK(d && d->apps.size() == 2 && d->apps[1].flags == kAppHidden && d->windows.size() == 1 &&
        d->windows[0].id == win);
  CHECK(m.unregister_app(2));
  CHECK(!m.unregister_app(2)); // already gone
  // dialogs + notify
  const uint16_t dlg =
      m.open_dialog({.owner = win, .kind = DialogKind::Input, .title = "T", .buttons = {"OK"}});
  m.notify({.level = NotifyLevel::Info, .title = "n"});
  out = m.flush();
  CHECK(out.size() == 3 && out[0].type == Type::Dialog && out[1].type == Type::Notify &&
        out[2].type == Type::Desktop);
  CHECK(m.dialog(dlg) && m.dialog(dlg)->owner() == win);
  m.close_dialog(dlg);
  out = m.flush();
  CHECK(out.size() == 1 && out[0].type == Type::DialogClose && !m.dialog(dlg));
  // closing a window closes its dialogs too (DIALOG_CLOSE precedes? no: the
  // window close discards them, the host dismisses owned dialogs itself)
  const uint16_t dlg2 = m.open_dialog({.owner = win, .title = "T2"});
  m.flush();
  m.close_window(win, WindowCloseReason::App);
  out = m.flush();
  CHECK(!m.dialog(dlg2) && out.size() == 2 && out[0].type == Type::WindowClose &&
        out[1].type == Type::DialogClose);
}

int main(int argc, char **argv) {
  std::string path = "desktop_vectors.txt";
  bool gen = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--gen")
      gen = true;
    else
      path = a;
  }
  if (gen) {
    generate(path);
    return g_failures ? 1 : 0;
  }
  test_vectors(path);
  test_truncation_and_unknown();
  test_widget_set_splitting();
  test_window_open_splitting();
  test_dialog_notify_limits();
  test_frames_and_correlation();
  test_text_assembler();
  test_dirty_tracker();
  test_model_flush();
  if (g_failures) {
    std::printf("%d FAILURE(S)\n", g_failures);
    return 1;
  }
  std::printf("ALL TESTS PASSED\n");
  return 0;
}

# Desktop Component

[![Badge](https://components.espressif.com/components/espp/desktop/badge.svg)](https://components.espressif.com/components/espp/desktop)

A browser-rendered **windowed desktop for microcontrollers**: the firmware
describes apps, windows and widgets; a web app draws them and sends the
user's actions back, over WebUSB / Web Serial (or any framed byte stream).

- `espp::Desktop` — the retained model and the app API: `register_app`,
  `create_window`, `Window` / `Widget` value handles with TinyDesk-style
  helpers (`label`, `button`, `checkbox`, `textbox`, `textarea`, `list`,
  `table`, `select`, `progress`, `slider`, `separator`, `spacer`, `column`,
  `row`, `group`), `set_text` (with `fmt` formatting), `append_text`,
  `set_value`, `set_items`, ..., `message_box`, `input_box`, `notify`, window
  timers and `post()`. Changes are coalesced and flushed by the desktop task
  every `flush_period`; every app callback runs on that task.
- `espp::DesktopService` — one `Desktop` as a transport-agnostic
  [dispatcher](../dispatcher) module (`espp.desktop` v1, module 9 by default;
  one instance per transport).
- `espp::ConsoleCapture` — tees stdout / stderr into a byte ring (through a
  write-only VFS device) so an app can show the device log live.

The hosted [desktop](https://esp-cpp.github.io/espp/apps/desktop.html) web app
(`web/desktop.html`) renders it: desktop icons and a start menu, draggable /
resizable windows with a taskbar, modal dialogs, toasts and a frame log.

*Screenshots: to be added after hardware testing.*

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Desktop Component](#desktop-component)
  - [Writing an app](#writing-an-app)
  - [Threading rules](#threading-rules)
  - [Protocol (module 9, `espp.desktop` v1)](#protocol-module-9-esppdesktop-v1)
  - [Log capture](#log-capture)
  - [Example](#example)

<!-- markdown-toc end -->

## Writing an app

```cpp
desktop.register_app({
    .name = "Counter", .icon = "🧮", .description = "Counts clicks",
    .launch = [](espp::Desktop &d, espp::Desktop::AppId app) {
      auto count = std::make_shared<int32_t>(0);
      auto win = d.create_window({.title = "Counter", .app = app, .w = 240, .h = 150});
      auto label = win.label("Count: 0", 0, espp::Desktop::kLabelBold);
      auto row = win.row();
      win.button("+1", [=]() mutable { label.set_text("Count: {}", ++*count); }, row.id());
      win.button("Reset", [=]() mutable { *count = 0; label.set_text("Count: 0"); }, row.id());
    }});
```

Widgets live in a window's root column unless a `parent` container is given
(`win.row()`, `win.column()`, `win.group("Title")`); the browser lays them out
as flexbox: `weight` = flex-grow along the parent's axis, `layout` bits
stretch / scroll / align, `width` / `height` a preferred size. A `Widget`'s
getters (`text()`, `value()`, `selected()`, `items()`) read the model, which
host events update before the handler runs. Handles are plain values; once a
widget or window is gone they are `!valid()` and every method is a no-op.

Dialogs are asynchronous: `message_box({.owner = win.id(), .title = ..., .buttons = {"OK", "Cancel"}, .on_result = [](int button) {...}})`
and `input_box({..., .on_result = [](std::optional<std::string> text) {...}})`;
`notify({.title, .text, .level, .timeout})` shows a toast. `win.add_timer(1s, fn)`
runs `fn` on the desktop task until the window closes; `desktop.post(fn)` runs
once.

## Threading rules

- Every app callback (launch, widget / window / dialog handlers, timers,
  `post`) runs on the desktop task with no lock held, so it may call any
  Desktop / Window / Widget method, including closing its own window.
- Any task may call the mutators; they take the model mutex, record the
  change, wake the desktop task and never send.
- All device->host frames are produced by the desktop task, once per
  iteration (drain host commands -> due timers / posted functions -> flush),
  encoded under the lock and sent outside it to every active transport under
  that transport's send mutex. A flush coalesces: last value wins per
  property, `TextAppend` pieces concatenate, a `Text` cancels earlier appends,
  `Items` ranges accumulate (an `ItemCount` / full `set_items` discards earlier
  ranges), a removed widget cancels its pending changes, a window opened and
  closed between flushes sends nothing. A TextArea keeps its last `max_lines`
  lines (the browser applies the same rule) and at most `max_text_bytes`; when
  the byte bound trims, the host gets a full `Text` replacement so both sides
  hold the same text. Per-window order: WINDOW_CLOSE ->
  WINDOW_OPEN (full tree) -> WIDGET_ADD -> WIDGET_SET -> WIDGET_REMOVE; then
  DIALOG / DIALOG_CLOSE -> NOTIFY -> DESKTOP (when apps or settings changed).
- The only frame sent from another context is `DesktopService`'s ERROR for a
  malformed request.

## Protocol (module 9, `espp.desktop` v1)

Framed with `stream_frame` and routed by `espp::Dispatcher`; requests carry
the reply flag clear, replies / events set it (type high bit). All multi-byte
fields are little-endian; `str8` = `[len u8][utf8]`, `str16` = `[len u16][utf8]`,
`rec` = `[tag u8][len u16][value]` (unknown tags skipped). See
`include/detail/desktop_protocol.hpp` (host-buildable, with
`test/desktop_host_test.cpp` and the shared fixture `test/desktop_vectors.txt`).

| Type | Dir | Meaning |
|------|-----|---------|
| `0x01` GET_DESKTOP | H→D | none → DESKTOP reply, then WINDOW_OPEN (flag Snapshot) per open window, then DIALOG per open dialog; marks the transport active |
| `0x02` LAUNCH_APP | H→D | `[app u8]` → OK / ERROR(ENOENT); a single-instance app already open is focused |
| `0x03` CLOSE_WINDOW | H→D | `[win u16]` → OK / ERROR(ENOENT), then WINDOW_CLOSE(reason 1) |
| `0x04` WINDOW_EVENT | H→D | `[win u16][ev u8][x i16][y i16][w u16][h u16]` ev: 1 Focus 2 Blur 3 Minimize 4 Restore 5 Maximize 6 Moved 7 Resized (no ack) |
| `0x05` WIDGET_EVENT | H→D | `[win u16][widget u16][ev u8][value]` ev: 1 Click · 2 Change `[i32]` · 3 Submit `[utf8]` · 4 Text `[offset u32][total u32][bytes]` (chunked) · 5 Select `[i32]` · 6 Activate `[i32]` · 7 Key `[key u16][mods u8][codepoint u32]` · 8 Scroll `[i32]` (no ack) |
| `0x06` DIALOG_RESULT | H→D | `[dialog u16][button u8 (0xFF dismissed)][text utf8 rest]` (no ack) |
| `0x81` DESKTOP | D→H | `[proto u8=1][flags u8 bit0 HasSnapshot][rec count u8]{rec}[app count u8]{[id u8][flags u8 bit0 SingleInstance bit1 Hidden][name str8][icon str8][desc str8]}[win count u8]{[win u16][app u8]}`; recs: 1 DeviceName 2 Firmware 3 Theme 4 Accent u32 5 MaxPayload u16 6 FlushPeriodMs u16. Also sent when apps / settings change |
| `0x82` WINDOW_OPEN | D→H | `[win u16][app u8][flags u16][x i16][y i16][w u16][h u16][title str8][widget total u16][count u16]{widget rec}`; the rest of the tree follows in WIDGET_ADD |
| `0x83` WINDOW_CLOSE | D→H | `[win u16][reason u8: 0 app 1 host 2 shutdown]` |
| `0x84` WIDGET_SET | D→H | `[win u16][entries u8]{[widget u16][props u8]{rec}}`; widget 0 = the window (Title / WindowFlags / Geometry / Focus) |
| `0x85` WIDGET_ADD | D→H | `[win u16][count u16]{widget rec}` (appended; `InsertBefore` positions) |
| `0x86` WIDGET_REMOVE | D→H | `[win u16][count u16]{widget u16}` (children go too) |
| `0x87` DIALOG | D→H | `[dialog u16][owner win u16 (0 desktop)][kind u8 0 msg 1 input][icon u8][title str8][text str16][default str16][buttons u8]{str8}`; button 0 default |
| `0x88` NOTIFY | D→H | `[level u8 0 info 1 ok 2 warn 3 err][timeout ms u16 (0 sticky)][title str8][text str16]` |
| `0x89` DIALOG_CLOSE | D→H | `[dialog u16]` |
| `0x8E` OK | D→H | `[request u8]` |
| `0x8F` ERROR | D→H | `[request u8][errno u32][utf8 message]` |

Widget rec: `[id u16][parent u16 (0 = window root)][type u8][weight u8][layout u8][props u8]{rec}`.
Types: 1 Column 2 Row 3 Group 4 Label 5 Button 6 Checkbox 7 TextBox 8 TextArea
9 List 10 Table 11 Select 12 Progress 13 Slider 14 Separator 15 Spacer.
`layout`: bit0 Stretch bit1 Scroll bit2 AlignEnd bit3 AlignCenter.
Prop tags: 1 Text 2 TextAppend 3 Value i32 4 Min 5 Max 6 Step 7 Enabled u8
8 Visible u8 9 Color u32 10 Background u32 11 ItemCount u16 12 Items
`[start u16][count u16]{str16}` 13 Columns `[n u8]{str8}` 14 Placeholder
15 Tooltip 16 Flags u16 17 MaxLines u16 18 Focus u8 19 Width u16 20 Height u16
21 InsertBefore u16 22 Title 23 WindowFlags u16 24 Geometry `[x i16][y i16][w u16][h u16]`.
Window flags: bit0 Movable bit1 Resizable bit2 Closable bit3 Modal
bit4 Minimizable bit5 Maximizable bit6 Snapshot bit7 Centered bit8 Pinned
bit9 WantsGeometry. Widget flags — TextArea: bit0 ReadOnly bit1 Monospace
bit2 WantKeys bit3 AutoScroll bit4 Ansi; TextBox: bit0 Password bit1 ReadOnly;
Label: bit0 Bold bit1 Monospace bit2 Wrap; Button: bit0 Primary bit1 Danger.

Replies (DESKTOP / OK / ERROR) echo the request's correlation id; events carry
none. Every payload is at most the negotiated MaxPayload (`max_frame_bytes`
less the frame overhead, 4081 for 4096; `max_frame_bytes` is at least
`kMinFrameBytes`, a 64-byte payload): the widget encoders split (Text +
TextAppend pieces, Items ranges, WIDGET_ADD continuations) and never truncate.
DIALOG and NOTIFY are single frames: `message_box` / `input_box` return 0 and
`notify` returns false (logged) for one that would not fit. DESKTOP is a
single frame too: the registry is bounded -- at most `kMaxApps` (24) apps,
names ≤ 32, icons ≤ 16, descriptions ≤ 64 bytes (`register_app` truncates
longer ones and refuses an app that would overflow), device name / firmware
≤ 64 bytes -- which leaves the default payload room for hundreds of open
windows; should a smaller cap still overflow, the encoder trims the window
list, then the descriptions, then apps (logged) rather than send a bad frame.

Flow control: a `DesktopService::Config::send` returns whether the frame was
queued (all-or-nothing); when it was not, the desktop logs and flags that
transport (`needs_resync()`) until the host's next GET_DESKTOP. A full
command queue (`Config::max_queued_commands`) refuses a request with
ERROR(EAGAIN) and drops an event; nothing queued is evicted.

## Log capture

`espp::ConsoleCapture::install({.capacity_bytes = 16 * 1024})` first thing in
`app_main()` registers `/dev/logcap`, a write-only VFS device, and re-opens
stdout / stderr on it; its `write()` forwards to the original console
(`/dev/console`, falling back to the UART / USB-Serial-JTAG device) and
appends to the ring. `read_since(&cursor, out, max)` pages through it from any
task without blocking the writers and reports bytes the ring overwrote
before the reader got to them. Mutually exclusive with
`UsbDevice::route_console_to_cdc()`.

## Example

The [example](./example) serves the desktop with the Counter, About, System
Monitor, Task Manager, Log Viewer, Files + Editor and Settings apps on the
native USB port of an ESP32-S3 (WebUSB + Web Serial), next to the standard
System / Monitor / OTA / CoreDump services.

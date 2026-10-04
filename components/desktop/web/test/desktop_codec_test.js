#!/usr/bin/env node
// Unit tests for the desktop web app's pure logic (the block of desktop.html
// between the DESKTOP:BEGIN-PURE / DESKTOP:END-PURE markers): the wire codec
// against the shared golden vectors in components/desktop/test/
// desktop_vectors.txt (decode(hex) must equal the JSON for every d2h vector,
// encode(JSON) must reproduce the hex for every h2d vector, every truncated
// prefix must be rejected, unknown tags skipped), the key table, text
// chunking, geometry clamping / placement / the geometry store, the layout
// box model and the ANSI parser; plus the page's registry metadata and that
// the pure block touches no DOM. Dependency-free:
//
//   node components/desktop/web/test/desktop_codec_test.js
"use strict";
const fs = require("fs");
const path = require("path");
const assert = require("assert");

const html = fs.readFileSync(path.join(__dirname, "..", "desktop.html"), "utf8");
// (plain string search, not a regexp: the markers delimit the one pure block)
const begin = html.indexOf("// ==== DESKTOP:BEGIN-PURE ====");
const end = html.indexOf("// ==== DESKTOP:END-PURE ====");
assert(begin > 0 && end > begin, "pure-block markers not found in desktop.html");
const pure = html.slice(begin, end);
const P = new Function(pure + `
  return { pickAutoConnectCandidate, sortedOrder, compareCells, cellSortKey, DT, DT_NAME, PROP, WT, WIN, WEV, WGEV, KEY, MOD, LAYOUT, TA_FLAG, MIN_WINDOW, DESKTOP_MAX_PAYLOAD_CAP, COLOR_DEFAULT, DESKTOP_HAS_SNAPSHOT, DESKTOP_WINDOW_LIST_COMPLETE, MAX_TEXT_BYTES_DEFAULT, SUBMIT_EVENT_HEADER, DIALOG_RESULT_HEADER, utf8ByteLength, truncateUtf8,
           ByteReader, ByteWriter, hexOf, bytesOfHex, propKind, readRec, decodeProp, propBytes, desktopSettings,
           decodeMessage, encodeMessage, decodeWidgetSet, decodeDesktop, chunkText, keyCodeFor,
           clampGeometry, geometryEquals, resizeGeometry, placeWindow, createGeometryStore, geometryKey,
           layoutStyle, applyItemRange, resizeItems, ansiSplit };
`)();

// The whole inline script must parse (a syntax error anywhere breaks the page).
// (plain string search: the page's one inline script between its first
// "<script>" and its last "</script>")
const scriptOpen = html.indexOf("<script>");
const scriptClose = html.lastIndexOf("</script>");
assert(scriptOpen >= 0 && scriptClose > scriptOpen, "no inline <script> in the page");
new Function(html.slice(scriptOpen + "<script>".length, scriptClose));

let passed = 0;
function test(name, fn) {
  try { fn(); passed++; console.log("ok   " + name); }
  catch (e) { console.log("FAIL " + name + "\n     " + (e && e.stack ? e.stack : e)); process.exitCode = 1; }
}

// ---- the golden vectors -----------------------------------------------------
const vectorsPath = path.join(__dirname, "..", "..", "test", "desktop_vectors.txt");
const vectors = [];
for (const line of fs.readFileSync(vectorsPath, "utf8").split("\n")) {
  if (!line.trim() || line.startsWith("#")) continue;
  const f = line.split("\t");
  assert.strictEqual(f.length, 5, "malformed vector line: " + line);
  vectors.push({ name: f[0], dir: f[1], type: Number.parseInt(f[2], 16), hex: f[3], json: JSON.parse(f[4]) });
}
assert(vectors.length >= 20, "too few vectors: " + vectors.length);
const d2h = vectors.filter((v) => v.dir === "d2h"), h2d = vectors.filter((v) => v.dir === "h2d");
assert(d2h.length >= 11 && h2d.length >= 9, "vector split " + d2h.length + "/" + h2d.length);
// every message type of the protocol is covered, in both directions
const types = new Set(vectors.map((v) => v.type));
for (const t of Object.values(P.DT)) assert(types.has(t), "no vector for type 0x" + t.toString(16));

test("every d2h vector decodes to its JSON", () => {
  for (const v of d2h) {
    const got = P.decodeMessage(v.type, P.bytesOfHex(v.hex));
    assert.deepStrictEqual(got, v.json, v.name);
  }
});
test("every h2d vector encodes to its hex", () => {
  for (const v of h2d) {
    assert.strictEqual(P.hexOf(P.encodeMessage(v.type, v.json)), v.hex, v.name);
  }
});
test("every truncated prefix of a d2h payload is rejected", () => {
  for (const v of d2h) {
    const full = P.bytesOfHex(v.hex);
    // ERROR carries its message as "the rest": only a prefix shorter than the
    // fixed [request u8][errno u32] head is malformed by construction
    const upTo = v.type === P.DT.ERROR ? Math.min(5, full.length) : full.length;
    for (let n = 0; n < upTo; n++)
      assert.throws(() => P.decodeMessage(v.type, full.subarray(0, n)), v.name + " prefix " + n + " accepted");
    // and a trailing byte is rejected too (ERROR: the trailing bytes ARE the message)
    if (v.type !== P.DT.ERROR)
      assert.throws(() => P.decodeMessage(v.type, Uint8Array.from([...full, 0])), v.name + " trailing byte accepted");
  }
  assert.throws(() => P.decodeMessage(0x7F, new Uint8Array(0)), /unknown message type/);
});
test("unknown and malformed tags decode raw and are skippable", () => {
  // WIDGET_SET: unknown tag 200, Flags with a 3-byte value (malformed), then a good Text
  const hex = "0100" + "01" + "0300" + "03" + "c8" + "0200" + "aabb" + "10" + "0300" + "010203" + "01" + "0200" + "6869";
  const s = P.decodeWidgetSet(P.bytesOfHex(hex));
  assert.deepStrictEqual(s.entries[0].props, [{ tag: 200, raw: "aabb" }, { tag: 16, raw: "010203" }, { tag: 1, value: "hi" }]);
  // the vectors themselves carry unknown tags 200 / 201 and the raw form round-trips to bytes
  for (const p of [{ tag: 200, raw: "aabb" }, { tag: 1, value: "hé" }, { tag: 3, value: -1 }, { tag: 16, value: 0x1234 }, { tag: 9, value: 0xFF0000 },
                   { tag: 12, value: { start: 1, items: ["a", "b\tB"] } }, { tag: 13, value: ["x", "y"] }, { tag: 24, value: { x: -1, y: 2, w: 3, h: 4 } }]) {
    const b = P.propBytes(p);
    assert.deepStrictEqual(P.decodeProp(p.tag, b), p, JSON.stringify(p));
  }
  assert.strictEqual(P.propKind(200), "unknown");
  assert.strictEqual(P.propKind(P.PROP.GEOMETRY), "geometry");
});
test("DESKTOP records interpreted by DesktopTag (theme / accent / max payload)", () => {
  const d = P.decodeDesktop(P.bytesOfHex(d2h.find((v) => v.name === "desktop").hex));
  const s = P.desktopSettings(d.records);
  // record 7 (MaxTextBytes u32) may or may not be in the fixture: either way it must be read
  const tag7 = d.records.find((p) => p.tag === 7);
  const expectMaxText = tag7 && tag7.raw && tag7.raw.length === 8 ? new P.ByteReader(P.bytesOfHex(tag7.raw)).u32() : P.MAX_TEXT_BYTES_DEFAULT;
  assert.deepStrictEqual(s, { deviceName: "espp Desktop", firmware: "desktop_example 1.0", theme: "auto", accent: 0x3B82F6, maxPayload: 4081, flushPeriodMs: 50, maxTextBytes: expectMaxText });
  assert.strictEqual(P.MAX_TEXT_BYTES_DEFAULT, 16384);
  assert.strictEqual(P.desktopSettings([]).maxTextBytes, 16384, "absent record 7 -> 16384");
  assert.strictEqual(P.desktopSettings([{ tag: 7, raw: "00100000" }]).maxTextBytes, 4096, "record 7 decodes raw (tag 7 is a u8 widget prop) and reads as u32 LE");
  assert.strictEqual(P.desktopSettings([{ tag: 7, raw: "00000000" }]).maxTextBytes, 16384, "a zero MaxTextBytes keeps the default");
  assert.strictEqual(P.desktopSettings([{ tag: 7, value: 1 }]).maxTextBytes, 16384, "a 1-byte record 7 is ignored");
  // the single-frame bounds and the UTF-8 helpers behind them
  assert.strictEqual(P.SUBMIT_EVENT_HEADER, 5); assert.strictEqual(P.DIALOG_RESULT_HEADER, 3);
  assert.strictEqual(P.utf8ByteLength("héllo"), 6); assert.strictEqual(P.utf8ByteLength(""), 0); assert.strictEqual(P.utf8ByteLength(null), 0);
  assert.strictEqual(P.truncateUtf8("héllo", 2), "h", "never cuts inside a sequence");
  assert.strictEqual(P.truncateUtf8("héllo", 3), "hé"); assert.strictEqual(P.truncateUtf8("héllo", 100), "héllo"); assert.strictEqual(P.truncateUtf8("😀😀", 5), "😀");
  assert.strictEqual(P.truncateUtf8("abc", 0), "");
  assert.strictEqual(P.encodeMessage(P.DT.WIDGET_EVENT, { window: 1, widget: 8, event: P.WGEV.SUBMIT, text: "y".repeat(4081 - 5) }).length, 4081);
  assert.strictEqual(P.encodeMessage(P.DT.DIALOG_RESULT, { dialog: 2, button: 0, text: "y".repeat(4081 - 3) }).length, 4081);
  // a 5-byte theme ("light") decodes raw through the generic path and still resolves
  const light = P.desktopSettings([{ tag: 3, raw: P.hexOf(Buffer.from("light")) }, { tag: 5, raw: "ffff" }, { tag: 5, raw: "0100" }]);
  assert.strictEqual(light.theme, "light");
  assert.strictEqual(light.maxPayload, 64, "max payload clamped to [64, 4081]");
  assert.strictEqual(P.desktopSettings([{ tag: 3, value: 0 }]).theme, "auto", "an unknown theme keeps auto");
  assert.strictEqual(P.desktopSettings([{ tag: 5, raw: "ffff" }]).maxPayload, 4081);
});

// ---- widget events ------------------------------------------------------------
test("Text chunks fit MaxPayload and never cut a UTF-8 sequence", () => {
  const text = Buffer.from("héllo wörld ".repeat(600)); // 7800 bytes
  const chunks = P.chunkText(text, 100);
  assert(chunks.length > 1);
  let off = 0;
  for (const c of chunks) {
    assert.strictEqual(c.offset, off); assert.strictEqual(c.total, text.length);
    assert(c.bytes.length <= 100 - 13, "chunk too large: " + c.bytes.length);
    assert((c.bytes[0] & 0xC0) !== 0x80, "chunk starts inside a UTF-8 sequence");
    assert.strictEqual(P.encodeMessage(P.DT.WIDGET_EVENT, { window: 1, widget: 4, event: P.WGEV.TEXT, offset: c.offset, total: c.total, text: c.bytes }).length, 13 + c.bytes.length);
    off += c.bytes.length;
  }
  assert.strictEqual(off, text.length);
  assert.deepStrictEqual(P.chunkText(new Uint8Array(0), 4081).map((c) => [c.offset, c.total, c.bytes.length]), [[0, 0, 0]]);
  // the two Text vectors are exactly what a 19-byte payload budget produces for "hello wrld"
  const two = P.chunkText(Buffer.from("hello wrld"), 13 + 6);
  assert.deepStrictEqual(two.map((c) => [c.offset, Buffer.from(c.bytes).toString()]), [[0, "hello "], [6, "wrld"]]);
});
test("key table (keyCodeFor)", () => {
  const k = (key, mods = {}) => P.keyCodeFor({ key, ...mods });
  assert.deepStrictEqual(k("Enter"), { key: 13, mods: 0, codepoint: 0 });
  assert.deepStrictEqual(k("Backspace"), { key: 8, mods: 0, codepoint: 0 });
  assert.deepStrictEqual(k("Tab"), { key: 9, mods: 0, codepoint: 0 });
  assert.deepStrictEqual(k("Escape"), { key: 27, mods: 0, codepoint: 0 });
  assert.deepStrictEqual(["ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown"].map((n) => k(n).key), [0x100, 0x101, 0x102, 0x103]);
  assert.deepStrictEqual(["Home", "End", "PageUp", "PageDown", "Delete", "Insert"].map((n) => k(n).key), [0x104, 0x105, 0x106, 0x107, 0x108, 0x109]);
  assert.strictEqual(k("F1").key, 0x110); assert.strictEqual(k("F12").key, 0x11B); assert.strictEqual(k("F13"), null);
  assert.deepStrictEqual(k("ArrowLeft", { ctrlKey: true, shiftKey: true }), { key: 0x100, mods: 3, codepoint: 0 });
  assert.deepStrictEqual(k("a", { altKey: true, metaKey: true }), { key: 0, mods: 12, codepoint: 97 });
  assert.deepStrictEqual(k("😀"), { key: 0, mods: 0, codepoint: 128512 });
  assert.strictEqual(k("Shift"), null); assert.strictEqual(k("Dead"), null); assert.strictEqual(k(""), null);
  // the Key vectors
  const v = (n) => h2d.find((x) => x.name === n);
  assert.strictEqual(P.hexOf(P.encodeMessage(P.DT.WIDGET_EVENT, { window: 1, widget: 4, event: 7, ...k("ArrowLeft", { ctrlKey: true, shiftKey: true }) })), v("widget_event_key").hex);
  assert.strictEqual(P.hexOf(P.encodeMessage(P.DT.WIDGET_EVENT, { window: 1, widget: 4, event: 7, ...k("😀") })), v("widget_event_key_char").hex);
  assert.throws(() => P.encodeMessage(P.DT.WIDGET_EVENT, { window: 1, widget: 1, event: 99 }), /unknown widget event/);
});
test("WINDOW_EVENT clamps to the wire ranges", () => {
  const b = P.encodeMessage(P.DT.WINDOW_EVENT, { window: 1, event: 6, x: -40000, y: 40000, w: -5, h: 70000 });
  assert.strictEqual(P.hexOf(b), "010006" + "0080" + "ff7f" + "0000" + "ffff");
});

// ---- geometry ---------------------------------------------------------------------
test("clampGeometry keeps a window inside the area at the minimum size", () => {
  const area = { w: 800, h: 500 };
  assert.deepStrictEqual(P.clampGeometry({ x: -10, y: -10, w: 10, h: 10 }, area), { x: 0, y: 0, w: 160, h: 100 });
  assert.deepStrictEqual(P.clampGeometry({ x: 790, y: 490, w: 300, h: 200 }, area), { x: 500, y: 300, w: 300, h: 200 });
  assert.deepStrictEqual(P.clampGeometry({ x: 10, y: 10, w: 2000, h: 2000 }, area), { x: 0, y: 0, w: 800, h: 500 });
  assert.deepStrictEqual(P.clampGeometry({ x: 10.4, y: 20.6, w: 300.2, h: 199.5 }, area), { x: 10, y: 21, w: 300, h: 200 });
  // a tiny area never pushes the size below the minimum
  assert.deepStrictEqual(P.clampGeometry({ x: 5, y: 5, w: 300, h: 300 }, { w: 100, h: 50 }), { x: 0, y: 0, w: 160, h: 100 });
  assert(P.geometryEquals({ x: 1, y: 2, w: 3, h: 4 }, { x: 1, y: 2, w: 3, h: 4 }) && !P.geometryEquals({ x: 1, y: 2, w: 3, h: 4 }, { x: 1, y: 2, w: 3, h: 5 }));
});
test("resizeGeometry per handle, with the opposite edge fixed", () => {
  const s = { x: 100, y: 100, w: 300, h: 200 };
  assert.deepStrictEqual(P.resizeGeometry(s, "se", 40, 20), { x: 100, y: 100, w: 340, h: 220 });
  assert.deepStrictEqual(P.resizeGeometry(s, "nw", 40, 20), { x: 140, y: 120, w: 260, h: 180 });
  assert.deepStrictEqual(P.resizeGeometry(s, "e", -500, 0), { x: 100, y: 100, w: 160, h: 200 });
  assert.deepStrictEqual(P.resizeGeometry(s, "w", 500, 0), { x: 240, y: 100, w: 160, h: 200 });
  assert.deepStrictEqual(P.resizeGeometry(s, "n", 500, 500), { x: 100, y: 200, w: 300, h: 100 });
  assert.deepStrictEqual(P.resizeGeometry(s, "s", 0, 5), { x: 100, y: 100, w: 300, h: 205 });
});
test("placeWindow: stored > firmware > content / cascade, Pinned swaps, Centered", () => {
  const area = { w: 1000, h: 600 };
  const stored = { x: 50, y: 60, w: 400, h: 300, max: false };
  const req = { x: 10, y: 20, w: 300, h: 200 };
  let p = P.placeWindow({ requested: req, flags: 0, stored, area });
  assert.deepStrictEqual([p.x, p.y, p.w, p.h, p.posSource, p.sizeSource], [50, 60, 400, 300, "stored", "stored"]);
  p = P.placeWindow({ requested: req, flags: P.WIN.PINNED, stored, area });
  assert.deepStrictEqual([p.x, p.y, p.w, p.h, p.posSource, p.sizeSource], [10, 20, 300, 200, "firmware", "firmware"]);
  p = P.placeWindow({ requested: req, flags: 0, stored: null, area });
  assert.deepStrictEqual([p.x, p.y, p.w, p.h, p.posSource, p.sizeSource], [10, 20, 300, 200, "firmware", "firmware"]);
  // x,y = -1 and w,h = 0: the browser decides; without content the size is pending
  assert.deepStrictEqual(P.placeWindow({ requested: { x: -1, y: -1, w: 0, h: 0 }, flags: 0, stored: null, area }), { pending: true });
  p = P.placeWindow({ requested: { x: -1, y: -1, w: 0, h: 0 }, flags: 0, stored: null, area, content: { w: 250, h: 150 }, cascadeIndex: 2 });
  assert.deepStrictEqual([p.x, p.y, p.w, p.h, p.posSource, p.sizeSource], [120 + 56, 24 + 56, 250, 150, "cascade", "content"]);
  // per-dimension size: width from the firmware, height from the content
  p = P.placeWindow({ requested: { x: -1, y: -1, w: 500, h: 0 }, flags: 0, stored: null, area, content: { w: 250, h: 150 } });
  assert.deepStrictEqual([p.w, p.h, p.sizeSource], [500, 150, "firmware+content"]);
  // Centered without a position
  p = P.placeWindow({ requested: { x: -1, y: -1, w: 400, h: 200 }, flags: P.WIN.CENTERED, stored: null, area });
  assert.deepStrictEqual([p.x, p.y, p.posSource], [300, 200, "centered"]);
  // a half-given position counts as none
  p = P.placeWindow({ requested: { x: 10, y: -1, w: 400, h: 200 }, flags: 0, stored: null, area, cascadeIndex: 0 });
  assert.strictEqual(p.posSource, "cascade");
  // stored off-screen geometry (a smaller screen now) is clamped back in
  p = P.placeWindow({ requested: req, flags: 0, stored: { x: 900, y: 550, w: 400, h: 300 }, area });
  assert.deepStrictEqual([p.x, p.y, p.w, p.h], [600, 300, 400, 300]);
  // the stored maximised state is restored unless Pinned
  assert.strictEqual(P.placeWindow({ requested: req, flags: 0, stored: { ...stored, max: true }, area }).maximized, true);
  assert.strictEqual(P.placeWindow({ requested: req, flags: P.WIN.PINNED, stored: { ...stored, max: true }, area }).maximized, false);
  // the cascade wraps
  assert.deepStrictEqual(P.placeWindow({ requested: {}, flags: 0, area, content: { w: 200, h: 100 }, cascadeIndex: 10 }).x, 120);
});
test("geometry store: keyed by app + title, injectable storage, LRU 200", () => {
  const mem = {};
  const storage = { getItem: (k) => (k in mem ? mem[k] : null), setItem: (k, v) => { mem[k] = v; } };
  const st = P.createGeometryStore(storage, { limit: 3 });
  assert.strictEqual(st.get("Counter", "Counter"), null);
  st.set("Counter", "Counter", { x: 1, y: 2, w: 300, h: 200 });
  assert.deepStrictEqual(st.get("Counter", "Counter"), { x: 1, y: 2, w: 300, h: 200, max: false });
  assert.strictEqual(st.get("Counter", "Counter (3)"), null, "the title is part of the key");
  assert(Object.keys(mem)[0] === "espp.desktop.geometry");
  st.set("A", "1", { x: 0, y: 0, w: 200, h: 100, max: true });
  st.set("B", "1", { x: 0, y: 0, w: 200, h: 100 });
  st.set("Counter", "Counter", { x: 5, y: 5, w: 300, h: 200 }); // touched: now the newest
  st.set("C", "1", { x: 0, y: 0, w: 200, h: 100 });              // evicts A (the oldest)
  assert.strictEqual(st.count(), 3);
  assert.strictEqual(st.get("A", "1"), null);
  assert.deepStrictEqual(st.get("Counter", "Counter"), { x: 5, y: 5, w: 300, h: 200, max: false });
  assert.strictEqual(P.createGeometryStore(null).get("x", "y"), null, "no storage: nothing stored, nothing thrown");
  P.createGeometryStore(null).set("x", "y", { x: 0, y: 0, w: 1, h: 1 });
  mem["espp.desktop.geometry"] = "{not json";
  assert.strictEqual(st.get("Counter", "Counter"), null, "corrupt storage reads as empty");
  const dflt = P.createGeometryStore(storage);
  for (let i = 0; i < 205; i++) dflt.set("app", "t" + i, { x: i, y: 0, w: 200, h: 100 });
  assert.strictEqual(dflt.count(), 200);
  assert.strictEqual(dflt.get("app", "t4"), null); assert.strictEqual(dflt.get("app", "t204").x, 204);
  assert.strictEqual(P.geometryKey("a", "b"), JSON.stringify(["a", "b"]));
  // app + title are encoded unambiguously: "a b" / "c" and "a" / "b c" are different windows
  assert.notStrictEqual(P.geometryKey("a b", "c"), P.geometryKey("a", "b c"));
  const col = P.createGeometryStore(storage, { key: "collision" });
  col.set("a b", "c", { x: 1, y: 1, w: 200, h: 100 });
  assert.strictEqual(col.get("a", "b c"), null, "a different app / title split must not read the other entry");
  assert.deepStrictEqual(col.get("a b", "c"), { x: 1, y: 1, w: 200, h: 100, max: false });
});

// ---- layout / items / ansi ------------------------------------------------------------
test("layoutStyle: weight = flex-grow along the axis, layout bits = cross axis", () => {
  let s = P.layoutStyle({ weight: 0, layout: 0 }, "column");
  assert.deepStrictEqual(s, { flex: "0 0 auto", alignSelf: "", overflow: "", minWidth: "", minHeight: "", width: "", height: "" });
  s = P.layoutStyle({ weight: 2, layout: P.LAYOUT.STRETCH | P.LAYOUT.SCROLL }, "column");
  assert.deepStrictEqual([s.flex, s.alignSelf, s.overflow, s.minHeight, s.minWidth], ["2 1 0px", "stretch", "auto", "0", "0"]);
  s = P.layoutStyle({ weight: 1, layout: P.LAYOUT.ALIGN_END, width: 120, height: 40 }, "row");
  assert.deepStrictEqual([s.flex, s.alignSelf, s.width, s.height, s.minWidth], ["1 1 120px", "flex-end", "", "40px", "0"]);
  s = P.layoutStyle({ weight: 0, layout: P.LAYOUT.ALIGN_CENTER, width: 120, height: 40 }, "column");
  assert.deepStrictEqual([s.flex, s.alignSelf, s.width, s.height], ["0 0 auto", "center", "120px", "40px"]);
});
test("item ranges: replace / extend / truncate", () => {
  const items = [];
  P.applyItemRange(items, 2, ["c", "d"]);
  assert.deepStrictEqual(items, ["", "", "c", "d"]);
  P.applyItemRange(items, 0, ["a"]);
  assert.deepStrictEqual(items, ["a", "", "c", "d"]);
  assert.deepStrictEqual(P.resizeItems(items, 2), ["a", ""]);
  assert.deepStrictEqual(P.resizeItems(items, 3), ["a", "", ""]);
});
test("ansiSplit: SGR colours / attributes, state carried, other escapes dropped", () => {
  const E = "\x1b";
  let r = P.ansiSplit("plain " + E + "[31mred" + E + "[1;44m bold-on-blue" + E + "[0m end");
  assert.deepStrictEqual(r.segments.map((s) => [s.text, s.fg, s.bg, s.bold]),
    [["plain ", -1, -1, false], ["red", 1, -1, false], [" bold-on-blue", 1, 4, true], [" end", -1, -1, false]]);
  // the state carries into the next line; 90-97 are bright, 38;5;n within 16
  r = P.ansiSplit("a" + E + "[92m", null);
  assert.strictEqual(r.state.fg, 10);
  r = P.ansiSplit("b" + E + "[38;5;3mc" + E + "[38;5;200md", r.state);
  assert.deepStrictEqual(r.segments.map((s) => [s.text, s.fg]), [["b", 10], ["c", 3], ["d", -1]]);
  // cursor moves / OSC titles vanish; an incomplete escape at the end is held back
  r = P.ansiSplit(E + "[2J" + E + "]0;title\x07x" + E + "[3");
  assert.deepStrictEqual(r.segments.map((s) => s.text), ["x"]);
  r = P.ansiSplit("y" + E);
  assert.deepStrictEqual(r.segments.map((s) => s.text), ["y"]);
  assert.deepStrictEqual(P.ansiSplit("", null).segments, []);
  r = P.ansiSplit(E + "[4;2mu" + E + "[24;22mv");
  assert.deepStrictEqual(r.segments.map((s) => [s.text, s.underline, s.dim]), [["u", true, true], ["v", false, false]]);
});

// ---- the page itself ------------------------------------------------------------------
test("registry metadata and wiring constants", () => {
  const meta = (name) => { const m = new RegExp('<meta\\s+name="' + name + '"\\s+content="([^"]*)"').exec(html); return m ? m[1] : null; };
  assert.strictEqual(meta("espp-category"), "device management");
  assert.strictEqual(meta("espp-protocols"), "espp.desktop:1");
  assert.strictEqual(meta("espp-transports"), "webusb webserial");
  assert(/<title>espp Desktop \(WebUSB \/ Web Serial\)<\/title>/.test(html));
  assert(html.includes('const DEFAULT_MODULE_DESKTOP = 9;'));
  assert(html.includes('const DESKTOP_PROTOCOL = "espp.desktop", DESKTOP_PROTOCOL_VERSION = 1;'));
  assert(html.includes('let moduleDesktop = DEFAULT_MODULE_DESKTOP;'));
  assert.strictEqual(P.DESKTOP_MAX_PAYLOAD_CAP, 4081);
  assert.strictEqual(P.DESKTOP_HAS_SNAPSHOT, 0x01); assert.strictEqual(P.DESKTOP_WINDOW_LIST_COMPLETE, 0x02);
  // the DESKTOP flags byte decodes as-is: the golden vector has a complete list (bit1 set),
  // the trimmed vector does not
  assert.strictEqual(P.decodeMessage(P.DT.DESKTOP, P.bytesOfHex(d2h.find((v) => v.name === "desktop").hex)).flags & P.DESKTOP_WINDOW_LIST_COMPLETE, 2);
  assert.strictEqual(P.decodeMessage(P.DT.DESKTOP, P.bytesOfHex(d2h.find((v) => v.name === "desktop_window_list_trimmed").hex)).flags & P.DESKTOP_WINDOW_LIST_COMPLETE, 0);
  // table sorting: numeric-aware, numbers before text, stable, dir 0 = firmware order
  {
    const rows = ["b\t10\t0", "a\t9\t1", "c\t100\t0", "d\tany\tx", "e\t9\t2"];
    assert.deepStrictEqual(P.sortedOrder(rows, 1, 0), [0, 1, 2, 3, 4]);
    assert.deepStrictEqual(P.sortedOrder(rows, 1, 1), [1, 4, 0, 2, 3], "ascending: 9, 9 (stable), 10, 100, text last");
    assert.deepStrictEqual(P.sortedOrder(rows, 1, -1), [3, 2, 0, 1, 4], "descending: text first, then 100, 10, 9, 9 (stable)");
    assert.deepStrictEqual(P.sortedOrder(rows, 0, 1), [1, 0, 2, 3, 4], "text column sorts by locale");
    assert.deepStrictEqual(P.sortedOrder(rows, 7, 1), [0, 1, 2, 3, 4], "a missing column compares equal: firmware order");
    assert.ok(P.compareCells("file2", "file10") < 0 && P.compareCells("B", "a") > 0, "locale compare is numeric-aware and case-insensitive");
    assert.strictEqual(P.cellSortKey(" -3.5 %"), -3.5); assert.strictEqual(P.cellSortKey("any"), null);
  }
  // automatic connect candidate: the one granted espp USB device, else the one espp serial port, never a guess
  {
    const espp = { vendorId: 0x1209, productId: 0x0d38 }, other = { vendorId: 0x2341, productId: 1 };
    const port = (v) => ({ getInfo: () => ({ usbVendorId: v }) });
    assert.deepStrictEqual(P.pickAutoConnectCandidate([espp, other], [], 0x1209), { kind: "usb", device: espp });
    assert.deepStrictEqual(P.pickAutoConnectCandidate([espp, espp], [port(0x1209)], 0x1209), { kind: "usb", several: 2 });
    assert.strictEqual(P.pickAutoConnectCandidate([other], [port(0x1209)], 0x1209).kind, "serial");
    assert.deepStrictEqual(P.pickAutoConnectCandidate([], [port(0x1209), port(0x1209)], 0x1209), { kind: "serial", several: 2 });
    assert.strictEqual(P.pickAutoConnectCandidate([other], [port(0x2341), { getInfo: () => { throw new Error("x"); } }], 0x1209), null);
    assert.strictEqual(P.pickAutoConnectCandidate(null, undefined, 0x1209), null);
  }
  assert.strictEqual(P.MIN_WINDOW.w, 160);
});
test("the pure block touches no DOM, window, storage or timers", () => {
  // strip comments and string literals, then look for free references
  const code = pure.replace(/\/\/[^\n]*/g, "").replace(/"(?:[^"\\\n]|\\.)*"|'(?:[^'\\\n]|\\.)*'|`(?:[^`\\]|\\.)*`/g, '""');
  const bad = [];
  for (const m of code.matchAll(/(?<![.\w$])(document|window|localStorage|sessionStorage|navigator|location|setTimeout|setInterval|requestAnimationFrame)\b(?!\s*:)/g)) bad.push(m[1] + "@" + m.index);
  assert.deepStrictEqual(bad, [], "free references in the pure block: " + bad.join(", "));
});

console.log(passed + " test(s) passed" + (process.exitCode ? ", some FAILED" : ""));

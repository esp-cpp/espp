#!/usr/bin/env node
// Host test for the discovery parser + module id resolution helpers every
// espp web console embeds (parseDiscovery / resolveModuleId /
// moduleOverrideFromQuery / describeModuleChoice), plus the Device Hub's
// stricter parser.
//
// The helper block is pasted verbatim into each console (there is no shared
// JS file: every app is a single self-contained HTML page), so this test first
// checks that all consoles carry the IDENTICAL block, then extracts it from one
// of them and exercises it.
//
//   node components/dispatcher/web/test/resolve_module_id_test.js
"use strict";
const fs = require("fs");
const path = require("path");
const assert = require("assert");

const root = path.resolve(__dirname, "..", "..", "..", "..");
const consoles = [
  "components/ota/web/ota_console.html",
  "components/coredump/web/coredump_console.html",
  "components/canopen/web/can_bridge_console.html",
  "components/canopen/web/ds402_panel.html",
  "components/mcp266/web/mcp266_console.html",
  "components/bldc_haptics/web/haptics_console.html",
  "components/telemetry/web/telemetry.html",
  "components/system/web/system_console.html",
];
const hub = "components/dispatcher/web/dispatcher_hub.html";

// The block runs from `function parseDiscovery(payload) {` through the end of
// `describeModuleChoice` (the last helper), each at 4-space indentation.
function extractBlock(src, rel) {
  const start = src.indexOf("    function parseDiscovery(payload) {");
  assert.ok(start >= 0, rel + ": no parseDiscovery");
  const marker = "    function describeModuleChoice(";
  const mid = src.indexOf(marker, start);
  assert.ok(mid >= 0, rel + ": no describeModuleChoice");
  const end = src.indexOf("\n    }\n", mid);
  assert.ok(end >= 0, rel + ": unterminated describeModuleChoice");
  return src.slice(start, end + "\n    }\n".length);
}

let block = null;
for (const rel of consoles) {
  const src = fs.readFileSync(path.join(root, rel), "utf8");
  const b = extractBlock(src, rel);
  if (block === null) block = b;
  else assert.strictEqual(b, block, rel + ": discovery helper block differs from " + consoles[0]);
}
console.log("PASS every console carries the identical discovery helper block (" + consoles.length + " files)");

// Evaluate the block; it only needs TextDecoder + URLSearchParams (globals in node >= 11).
const helpers = new Function(block + "\n return { parseDiscovery, resolveModuleId, moduleOverrideFromQuery, describeModuleChoice, DISCOVERY_VERSION_KNOWN };")();
const { parseDiscovery, resolveModuleId, moduleOverrideFromQuery, describeModuleChoice } = helpers;

// The hub has its own (stricter) parser: trailing bytes are an error for a
// payload version it fully understands.
const hubSrc = fs.readFileSync(path.join(root, hub), "utf8");
const hubStart = hubSrc.indexOf("    const DISCOVERY_VERSION_KNOWN = 2;");
const hubParseStart = hubSrc.indexOf("    function parseDiscovery(payload) {", hubStart);
const hubEnd = hubSrc.indexOf("\n    }\n", hubParseStart) + "\n    }\n".length;
assert.ok(hubStart >= 0 && hubParseStart > hubStart && hubEnd > hubParseStart, "hub parser not found");
const hubParse = new Function(hubSrc.slice(hubStart, hubEnd) + "\n return parseDiscovery;")();

// ---- TLV builders (mirror espp::Dispatcher::describe) ----------------------
const enc = new TextEncoder();
const str = (s) => { const b = enc.encode(s); return [b.length, ...b]; };
const u16 = (v) => [v & 0xff, (v >> 8) & 0xff];
// Records carry no length: a version > 2 must use the exact v2 record layout
// and may only append bytes AFTER the record list (`trailing`).
function payload(version, mods, trailing = []) {
  const out = [version, 0, ...str("espp Hub"), ...str("1.2.3"), mods.length];
  for (const m of mods) {
    out.push(m.id, ...str(m.name), ...str(m.app), ...str(m.desc || ""));
    if (version >= 2) out.push(...str(m.protocol || ""), ...u16(m.protocolVersion || 0));
  }
  out.push(...trailing);
  return new Uint8Array(out);
}
const OTA = { id: 0, name: "OTA", app: "ota_console.html", desc: "fw", protocol: "espp.ota", protocolVersion: 1 };
const CD = { id: 4, name: "Core Dump", app: "coredump_console.html", desc: "crash", protocol: "espp.coredump", protocolVersion: 1 };

// ---- parseDiscovery: v1 and v2 ---------------------------------------------
{
  const v1 = parseDiscovery(payload(1, [OTA, CD]));
  assert.strictEqual(v1.version, 1);
  assert.strictEqual(v1.device, "espp Hub");
  assert.strictEqual(v1.fw, "1.2.3");
  assert.deepStrictEqual(v1.modules.map((m) => [m.id, m.name, m.app, m.desc, m.protocol, m.protocolVersion]),
    [[0, "OTA", "ota_console.html", "fw", "", 0], [4, "Core Dump", "coredump_console.html", "crash", "", 0]]);
  const v2 = parseDiscovery(payload(2, [OTA, { ...CD, protocolVersion: 0x0102 }]));
  assert.strictEqual(v2.version, 2);
  assert.deepStrictEqual(v2.modules.map((m) => [m.id, m.protocol, m.protocolVersion]),
    [[0, "espp.ota", 1], [4, "espp.coredump", 0x0102]]); // u16 little-endian
  assert.throws(() => parseDiscovery(payload(2, [OTA]).subarray(0, 12)), /truncated/);
  // an empty protocol encodes as len 0 and parses back as ""
  const none = parseDiscovery(payload(2, [{ id: 9, name: "X", app: "" }]));
  assert.strictEqual(none.modules[0].protocol, "");
  assert.strictEqual(none.modules[0].protocolVersion, 0);
  assert.strictEqual(v2.trailing, 0);
  // a newer version is parsed with the exact v2 record layout: SEVERAL records
  // decode intact, and only bytes after the whole list are tolerated (counted)
  const v3 = parseDiscovery(payload(3, [OTA, CD, { id: 9, name: "X", app: "", protocol: "x.y", protocolVersion: 3 }], [0xAA, 0xBB, 0xCC]));
  assert.strictEqual(v3.version, 3);
  assert.deepStrictEqual(v3.modules.map((m) => [m.id, m.protocol, m.protocolVersion]), [[0, "espp.ota", 1], [4, "espp.coredump", 1], [9, "x.y", 3]]);
  assert.strictEqual(v3.trailing, 3);
  assert.strictEqual(parseDiscovery(payload(3, [OTA, CD])).trailing, 0);
  console.log("PASS parseDiscovery decodes v1 and v2 payloads (v3+: v2 records, trailing bytes counted)");
}
{
  const p2 = payload(2, [OTA, CD]);
  assert.strictEqual(hubParse(p2).modules[1].protocol, "espp.coredump");
  assert.throws(() => hubParse(new Uint8Array([...p2, 0])), /trailing/);
  // a v3 payload: several v2-layout records, then trailing bytes, tolerated and counted
  const p3 = payload(3, [OTA, CD], [0xAA, 0xBB]);
  const h3 = hubParse(p3);
  assert.deepStrictEqual(h3.modules.map((m) => [m.id, m.protocol]), [[0, "espp.ota"], [4, "espp.coredump"]]);
  assert.strictEqual(h3.trailing, 2);
  assert.strictEqual(hubParse(payload(3, [OTA, CD])).trailing, 0);
  console.log("PASS hub parser: strict for known versions, trailing-only tolerance for newer ones");
}

// ---- resolveModuleId --------------------------------------------------------
const ident = (over = {}) => ({ protocol: "espp.ota", protocolVersion: 1, app: "ota_console.html", name: "OTA", fallback: 0, override: null, ...over });
const info = (version, mods) => ({ version, device: "d", fw: "1", modules: mods });
{
  // no discovery reply -> the default, silently
  let r = resolveModuleId(null, ident());
  assert.deepStrictEqual([r.id, r.source, r.protocolVersion, r.warnings, r.notes], [0, "default", null, [], []]);
  // protocol id first, even when app / name / id point elsewhere
  r = resolveModuleId(info(2, [{ id: 0, name: "OTA", app: "ota_console.html", protocol: "espp.other", protocolVersion: 1 },
                               { id: 7, name: "Updater", app: "x.html", protocol: "espp.ota", protocolVersion: 1 }]), ident());
  assert.deepStrictEqual([r.id, r.source, r.protocolVersion, r.warnings], [7, "protocol", 1, []]);
  // then app (basename, case-insensitive, query stripped) -- what v1 devices offer
  r = resolveModuleId(info(1, [{ id: 3, name: "Telemetry", app: "telemetry.html" }, { id: 9, name: "fw", app: "apps/OTA_Console.html?x" }]), ident());
  assert.deepStrictEqual([r.id, r.source], [9, "app"]);
  // then name
  r = resolveModuleId(info(1, [{ id: 11, name: " ota ", app: "" }]), ident());
  assert.deepStrictEqual([r.id, r.source], [11, "name"]);
  // nothing matches -> default + a warning listing what was advertised
  r = resolveModuleId(info(2, [{ id: 4, name: "Core Dump", app: "coredump_console.html", protocol: "espp.coredump", protocolVersion: 1 }]), ident());
  assert.deepStrictEqual([r.id, r.source], [0, "default"]);
  assert.strictEqual(r.warnings.length, 1);
  assert.ok(r.warnings[0].includes("Core Dump (#4)") && r.warnings[0].includes("#0"), r.warnings[0]);
  // several candidates -> the first, with a warning naming all of them
  r = resolveModuleId(info(2, [{ id: 5, name: "A", app: "", protocol: "espp.ota", protocolVersion: 1 }, { id: 6, name: "B", app: "", protocol: "espp.ota", protocolVersion: 1 }]), ident());
  assert.deepStrictEqual([r.id, r.source], [5, "protocol"]);
  assert.ok(r.warnings.length === 1 && r.warnings[0].includes("A (#5)") && r.warnings[0].includes("B (#6)"), r.warnings);
  // protocol version mismatch: a warning, not a refusal
  r = resolveModuleId(info(2, [{ id: 0, name: "OTA", app: "", protocol: "espp.ota", protocolVersion: 2 }]), ident());
  assert.deepStrictEqual([r.id, r.protocolVersion], [0, 2]);
  assert.ok(r.warnings.length === 1 && /v2/.test(r.warnings[0]) && /v1/.test(r.warnings[0]), r.warnings);
  // version 0 = unspecified: no mismatch warning
  r = resolveModuleId(info(2, [{ id: 0, name: "OTA", app: "", protocol: "espp.ota", protocolVersion: 0 }]), ident());
  assert.deepStrictEqual(r.warnings, []);
  // override wins: silent when it matches, warned when unadvertised / another protocol
  const two = info(2, [OTA, CD]);
  r = resolveModuleId(two, ident({ override: 0 }));
  assert.deepStrictEqual([r.id, r.source, r.protocolVersion, r.warnings], [0, "override", 1, []]);
  r = resolveModuleId(two, ident({ override: 9 }));
  assert.ok(r.id === 9 && r.source === "override" && r.warnings.length === 1 && r.warnings[0].includes("#9"), r);
  r = resolveModuleId(two, ident({ override: 4 }));
  assert.ok(r.id === 4 && r.warnings.length === 1 && r.warnings[0].includes("espp.coredump"), r);
  r = resolveModuleId(null, ident({ override: 4 }));
  assert.deepStrictEqual([r.id, r.source, r.warnings], [4, "override", []]);
  // an out-of-range override is ignored (falls through to discovery)
  r = resolveModuleId(two, ident({ override: 0xFF }));
  assert.deepStrictEqual([r.id, r.source], [0, "protocol"]);
  // a newer payload than understood is warned about (and still resolved); trailing
  // bytes after the list are an informational note, not a warning
  r = resolveModuleId(info(3, [OTA]), ident());
  assert.ok(r.id === 0 && r.source === "protocol" && r.warnings.length === 1 && /v3/.test(r.warnings[0]) && r.notes.length === 0, r);
  r = resolveModuleId({ ...info(3, [OTA]), trailing: 5 }, ident());
  assert.ok(r.warnings.length === 1 && r.notes.length === 1 && /5 trailing/.test(r.notes[0]), r);
  r = resolveModuleId(parseDiscovery(payload(3, [OTA, CD], [1, 2])), ident());
  assert.ok(r.id === 0 && r.source === "protocol" && r.notes.length === 1 && /2 trailing/.test(r.notes[0]), r);
  console.log("PASS resolveModuleId: override > protocol > app > name > default, with warnings");
}

// ---- moduleOverrideFromQuery + describeModuleChoice ------------------------
{
  assert.strictEqual(moduleOverrideFromQuery("?module=9"), 9);
  assert.strictEqual(moduleOverrideFromQuery("?a=1&module=0x1F"), 0x1F);
  assert.strictEqual(moduleOverrideFromQuery("?module=0"), 0);
  assert.strictEqual(moduleOverrideFromQuery("?module="), null);
  assert.strictEqual(moduleOverrideFromQuery("?module=255"), null);
  // 0xF0..0xFF are reserved for dispatcher / meta use: not selectable
  assert.strictEqual(moduleOverrideFromQuery("?module=0xEF"), 0xEF);
  assert.strictEqual(moduleOverrideFromQuery("?module=239"), 239);
  assert.strictEqual(moduleOverrideFromQuery("?module=0xF0"), null);
  assert.strictEqual(moduleOverrideFromQuery("?module=240"), null);
  assert.strictEqual(moduleOverrideFromQuery("?module=abc"), null);
  // the whole token must be a number: a numeric prefix is not accepted
  assert.strictEqual(moduleOverrideFromQuery("?module=9oops"), null);
  assert.strictEqual(moduleOverrideFromQuery("?module=0x1g"), null);
  assert.strictEqual(moduleOverrideFromQuery("?module=1.5"), null);
  assert.strictEqual(moduleOverrideFromQuery("?module=-1"), null);
  assert.strictEqual(moduleOverrideFromQuery("?module=%2B3"), null); // a literal "+3" (a bare + is a space in a query)
  assert.strictEqual(moduleOverrideFromQuery("?module=0x"), null);
  assert.strictEqual(moduleOverrideFromQuery("?module= 7 "), 7);
  assert.strictEqual(moduleOverrideFromQuery("?module=0XeF"), 0xEF);
  assert.strictEqual(moduleOverrideFromQuery(""), null);
  assert.strictEqual(moduleOverrideFromQuery(undefined), null);
  const two = info(2, [OTA, CD]);
  assert.strictEqual(describeModuleChoice("OTA", resolveModuleId(two, ident()), ident(), two), "OTA module: #0 (advertises espp.ota v1)");
  assert.strictEqual(describeModuleChoice("OTA", resolveModuleId(two, ident({ override: 4 })), ident({ override: 4 }), two), "OTA module: #4 (from ?module=)");
  assert.strictEqual(describeModuleChoice("OTA", resolveModuleId(null, ident()), ident(), null), "OTA module: #0 (default; the device did not answer discovery)");
  const v1 = info(1, [{ id: 9, name: "fw", app: "ota_console.html" }]);
  assert.strictEqual(describeModuleChoice("OTA", resolveModuleId(v1, ident()), ident(), v1), "OTA module: #9 (advertised as app ota_console.html)");
  console.log("PASS moduleOverrideFromQuery + describeModuleChoice");
}

// ---- lint: the resolved id is used everywhere, the DEFAULT_* constant nowhere else --
// Each console declares `let <resolved> = DEFAULT_<X>;` and uses <resolved> for
// every frame it builds and every reply it matches. The DEFAULT constant may
// appear only in that declaration and as the resolver's `fallback:`.
{
  const discoveryConsts = new Set(["MODULE_DISCOVERY", "SF_MODULE_DISCOVERY"]);
  const topLevelArgs = (text) => { // split a call's argument text on top-level commas
    const out = []; let depth = 0, cur = "";
    for (const ch of text) {
      if ("([{".includes(ch)) depth++;
      if (")]}".includes(ch)) depth--;
      if (ch === "," && depth === 0) { out.push(cur.trim()); cur = ""; } else cur += ch;
    }
    if (cur.trim()) out.push(cur.trim());
    return out;
  };
  const callArgs = (src, name) => { // argument text of every `name(` call (not its definition)
    const found = []; const re = new RegExp("(?<!function )\\b" + name + "\\(", "g"); let m;
    while ((m = re.exec(src))) {
      let depth = 1, j = m.index + m[0].length;
      for (; j < src.length && depth > 0; j++) { if (src[j] === "(") depth++; else if (src[j] === ")") depth--; }
      found.push({ at: m.index, args: topLevelArgs(src.slice(m.index + m[0].length, j - 1)) });
    }
    return found;
  };
  for (const rel of consoles) {
    const src = fs.readFileSync(path.join(root, rel), "utf8");
    const resolved = [...src.matchAll(/^\s*let (\w+) = (DEFAULT_\w+);/gm)].map((m) => m[1]);
    assert.ok(resolved.length >= 1, rel + ": no `let <resolved> = DEFAULT_*` declaration");
    const allowed = new Set([...resolved, ...discoveryConsts]);
    // every `.module ===` / `!==` comparison must be against a resolved id or the discovery module
    for (const m of src.matchAll(/\.module\s*(===|!==)\s*([A-Za-z_$][\w$]*)/g))
      assert.ok(allowed.has(m[2]), rel + ": frame.module compared against `" + m[2] + "` (allowed: " + [...allowed].join(", ") + ")");
    // every frame built must be stamped with a resolved id (or the discovery module), never a DEFAULT constant
    for (const name of ["buildFrame", "sfBuild"]) {
      for (const call of callArgs(src, name)) {
        for (const a of call.args) {
          assert.ok(!/DEFAULT_/.test(a), rel + ": " + name + "(" + call.args.join(", ") + ") uses a DEFAULT_ constant");
          if (/^(module[A-Z]\w*|tmModule|\w*MODULE(_\w+)?)$/.test(a))
            assert.ok(allowed.has(a), rel + ": " + name + "(" + call.args.join(", ") + ") stamps `" + a + "`, not a resolved id");
        }
      }
    }
    // the DEFAULT constant appears only in the declaration and the resolver's fallback
    for (const m of src.matchAll(/DEFAULT_(?!VID|PID|ZOOM)\w+/g)) {
      const line = src.slice(src.lastIndexOf("\n", m.index) + 1, src.indexOf("\n", m.index));
      const ok = /^\s*const DEFAULT_/.test(line) || /^\s*let \w+ = DEFAULT_/.test(line) || /fallback: DEFAULT_/.test(line) || /^\s*\/\//.test(line);
      assert.ok(ok, rel + ": stray use of " + m[0] + ": " + line.trim());
    }
    // every send path is gated on the discovery probe having completed
    assert.ok(/let moduleReady = false;/.test(src) && /moduleReady = true;/.test(src), rel + ": no moduleReady gate");
    // a probe that outlives its connection must not adopt into the next one:
    // between the probe's start and its adopt call, the connection it sent on
    // is compared against the current one (transport / usb / device, or the
    // haptics connection generation)
    const probe = /(?:async )?function (?:checkModulePresent|discoverModule)\([^)]*\) \{([\s\S]*?)adoptModules?\(/.exec(src);
    assert.ok(probe, rel + ": no discovery probe function found");
    assert.ok(/(t !== (?:transport|usb)|device === d|generation !== connectGeneration)/.test(probe[1]),
      rel + ": the discovery probe adopts its result without checking its connection is still current");
  }
  console.log("PASS lint: every console builds / matches frames with its resolved module id only");
}

// ---- the hub links every app with its module id + the device identity -------
{
  // the href is the plain ?module=N link (a middle-click / "open in new tab" bypasses the
  // click handler while the hub still owns the device); the auto-connect URL is built
  // only by the controlled click, which releases the device first
  assert.ok(hubSrc.includes('a.href = file + "?" + connectQuery(moduleId, null);'), "hub: the link href must not carry auto-connect parameters");
  assert.ok(hubSrc.includes('handOff(file + "?" + connectQuery(moduleId, transport.identity()), file, tab);'),
    "hub must link app?<connectQuery(module id, device identity)>");
  assert.ok(hubSrc.includes("const a = appLink(m.app, m.id);") && hubSrc.includes("const a = appLink(e.app.file, e.module.id);"),
    "every hub app link must go through appLink()");
  console.log("PASS hub links each console with ?module=<id>&autoconnect=... via appLink()");
}

// =============================================================================
// Connection helpers: auto-connect params, permitted-device matching, the
// reconnect supervisor, hand-off notices. One block, byte-identical in every
// console AND the hub, delimited by the begin / end marker comments.
// =============================================================================
// The markers are located without assuming their indentation or the line
// ending: the block runs from the start of the begin-marker line to the end of
// the end-marker line (LF or CRLF).
function extractConnectBlock(src, rel) {
  const beginAt = src.indexOf("// --- begin connection helpers");
  assert.ok(beginAt >= 0, rel + ": no connection helper block");
  const start = src.lastIndexOf("\n", beginAt) + 1;
  const endAt = src.indexOf("// --- end connection helpers ---", beginAt);
  assert.ok(endAt >= 0, rel + ": unterminated connection helper block");
  const nl = src.indexOf("\n", endAt);
  return src.slice(start, nl >= 0 ? nl + 1 : src.length);
}
let connectBlock = null;
for (const rel of [...consoles, hub]) {
  const b = extractConnectBlock(fs.readFileSync(path.join(root, rel), "utf8"), rel);
  if (connectBlock === null) connectBlock = b;
  else assert.strictEqual(b, connectBlock, rel + ": connection helper block differs from " + consoles[0]);
}
console.log("PASS every console and the hub carry the identical connection helper block (" + (consoles.length + 1) + " files)");

// the hub-only link helper lives in its own marked block right after the shared one
function extractHubLinkBlock(src) {
  const beginAt = src.indexOf("// --- begin hub link helpers");
  assert.ok(beginAt >= 0, "hub: no hub link helper block");
  const endAt = src.indexOf("// --- end hub link helpers ---", beginAt);
  assert.ok(endAt >= 0, "hub: unterminated hub link helper block");
  const nl = src.indexOf("\n", endAt);
  return src.slice(src.lastIndexOf("\n", beginAt) + 1, nl >= 0 ? nl + 1 : src.length);
}
const hubLinkBlock = extractHubLinkBlock(fs.readFileSync(path.join(root, hub), "utf8"));
assert.ok(!connectBlock.includes("function connectQuery"), "connectQuery is hub-only (unused in the consoles)");
const conn = new Function(connectBlock + hubLinkBlock + "\n return { parseConnectParams, usbIdentity, serialIdentity, describeIdentity, connectQuery, pickUsbDevice, pickSerialPort, findPermittedUsbDevice, findPermittedSerialPort, loadAutoReconnect, saveAutoReconnect, createReconnectSupervisor, watchDeviceArrivals, openDeviceChannel, postReleased, createOpenGate, identityMatches };")();

// ---- parseConnectParams / connectQuery ---------------------------------------
{
  const p = conn.parseConnectParams("?module=3&autoconnect=1&transport=USB&vid=0x1209&pid=3378&serial=AB%20C");
  assert.deepStrictEqual(p, { autoconnect: true, transport: "usb", vid: 0x1209, pid: 3378, serial: "AB C" });
  assert.deepStrictEqual(conn.parseConnectParams(""), { autoconnect: false, transport: null, vid: null, pid: null, serial: null });
  assert.deepStrictEqual(conn.parseConnectParams(undefined).autoconnect, false);
  // flag spellings; anything else is off
  for (const v of ["1", "true", "YES"]) assert.strictEqual(conn.parseConnectParams("?autoconnect=" + v).autoconnect, true, v);
  for (const v of ["0", "false", "", "on"]) assert.strictEqual(conn.parseConnectParams("?autoconnect=" + v).autoconnect, false, v);
  // transport: usb / serial only
  assert.strictEqual(conn.parseConnectParams("?transport=Serial").transport, "serial");
  assert.strictEqual(conn.parseConnectParams("?transport=ble").transport, null);
  // ids: whole-token hex or decimal, 16-bit; garbage -> null (never NaN)
  assert.strictEqual(conn.parseConnectParams("?vid=0x1209").vid, 0x1209);
  assert.strictEqual(conn.parseConnectParams("?vid=4617").vid, 4617);
  assert.strictEqual(conn.parseConnectParams("?vid= 0XFFFF ").vid, 0xFFFF);
  for (const bad of ["0x10000", "65536", "-1", "1.5", "0x", "12ab", "abc", ""]) assert.strictEqual(conn.parseConnectParams("?vid=" + bad).vid, null, bad);
  assert.strictEqual(conn.parseConnectParams("?serial=%20%20").serial, null);
  // connectQuery round-trips through parseConnectParams and keeps ?module=
  const id = { transport: "usb", vid: 0x1209, pid: 0x0d32, serial: "AB C" };
  const q = conn.connectQuery(9, id);
  assert.strictEqual(q, "module=9&autoconnect=1&transport=usb&vid=0x1209&pid=0x0d32&serial=AB+C");
  assert.strictEqual(moduleOverrideFromQuery("?" + q), 9);
  assert.deepStrictEqual(conn.parseConnectParams("?" + q), { autoconnect: true, ...id });
  // no identity -> module only (a plain link, no auto-connect); serial ports carry no serial
  assert.strictEqual(conn.connectQuery(4, null), "module=4");
  assert.strictEqual(conn.connectQuery(4, { transport: "serial", vid: 0x1209, pid: 0x0d36, serial: null }), "module=4&autoconnect=1&transport=serial&vid=0x1209&pid=0x0d36");
  assert.strictEqual(conn.connectQuery(4, { transport: "ble", vid: 1, pid: 2 }), "module=4");
  // identities
  assert.deepStrictEqual(conn.usbIdentity({ vendorId: 1, productId: 2, serialNumber: "" }), { transport: "usb", vid: 1, pid: 2, serial: null });
  assert.deepStrictEqual(conn.serialIdentity({ getInfo: () => ({ usbVendorId: 1, usbProductId: 2 }) }), { transport: "serial", vid: 1, pid: 2, serial: null });
  assert.deepStrictEqual(conn.serialIdentity({ getInfo: () => ({}) }), { transport: "serial", vid: null, pid: null, serial: null });
  assert.deepStrictEqual(conn.serialIdentity({}), { transport: "serial", vid: null, pid: null, serial: null });
  assert.strictEqual(conn.describeIdentity(id), "USB device 0x1209:0x0d32 sn AB C");
  assert.strictEqual(conn.describeIdentity({ transport: "serial", vid: null, pid: 3 }), "serial port ?:0x0003");
  assert.strictEqual(conn.describeIdentity(null), "device");
  console.log("PASS parseConnectParams / connectQuery / identities");
}

// ---- permitted-device matching ------------------------------------------------
{
  const dev = (vid, pid, sn) => ({ vendorId: vid, productId: pid, serialNumber: sn });
  const A = dev(0x1209, 1, "A"), B = dev(0x1209, 1, "B"), N = dev(0x1209, 1, undefined), O = dev(0x1209, 2, "O"), X = dev(0x2341, 1, "X");
  // vid + pid + serial: exact serial wins; a different serial is another device
  assert.strictEqual(conn.pickUsbDevice([A, B, O], { vid: 0x1209, pid: 1, serial: "B" }), B);
  assert.strictEqual(conn.pickUsbDevice([A, O], { vid: 0x1209, pid: 1, serial: "B" }), null);
  // wanted serial but the device reports none: only when it is the sole unnamed candidate
  assert.strictEqual(conn.pickUsbDevice([A, N], { vid: 0x1209, pid: 1, serial: "B" }), N);
  assert.strictEqual(conn.pickUsbDevice([N, dev(0x1209, 1, null)], { vid: 0x1209, pid: 1, serial: "B" }), null);
  // no serial wanted: first vid+pid match
  assert.strictEqual(conn.pickUsbDevice([O, B, A], { vid: 0x1209, pid: 1, serial: null }), B);
  assert.strictEqual(conn.pickUsbDevice([O, X], { vid: 0x1209, pid: 1 }), null);
  // pid alone / vid alone filter on what is given
  assert.strictEqual(conn.pickUsbDevice([X, O], { vid: null, pid: 2 }), O);
  assert.strictEqual(conn.pickUsbDevice([X, O], { vid: 0x2341, pid: null }), X);
  // nothing wanted: only a lone permitted device
  assert.strictEqual(conn.pickUsbDevice([A], null), A);
  assert.strictEqual(conn.pickUsbDevice([A, B], { vid: null, pid: null, serial: null }), null);
  assert.strictEqual(conn.pickUsbDevice([], { vid: 0x1209, pid: 1 }), null);
  assert.strictEqual(conn.pickUsbDevice(null, { vid: 0x1209, pid: 1 }), null);
  assert.strictEqual(conn.pickUsbDevice([null, A], { vid: 0x1209, pid: 1 }), A);
  // serial ports: vid + pid from getInfo(); serial numbers are not exposed, so ignored
  const port = (vid, pid) => ({ getInfo: () => (vid == null ? {} : { usbVendorId: vid, usbProductId: pid }) });
  const P1 = port(0x1209, 1), P2 = port(0x1209, 2), PN = port(null);
  assert.strictEqual(conn.pickSerialPort([PN, P2, P1], { vid: 0x1209, pid: 1, serial: "ignored" }), P1);
  assert.strictEqual(conn.pickSerialPort([PN, P2], { vid: 0x1209, pid: 1 }), null);
  assert.strictEqual(conn.pickSerialPort([PN], null), PN);
  assert.strictEqual(conn.pickSerialPort([PN, P1], { vid: null, pid: null }), null);
  assert.strictEqual(conn.pickSerialPort([{}], { vid: 1, pid: 1 }), null); // no getInfo at all
  console.log("PASS pickUsbDevice / pickSerialPort matching rules");
}

// ---- findPermitted* with a fake navigator (missing API, rejecting API) ---------
(async () => {
  const saved = global.navigator;
  const setNav = (v) => Object.defineProperty(global, "navigator", { value: v, configurable: true, writable: true });
  try {
    setNav({});
    assert.strictEqual(await conn.findPermittedUsbDevice({ vid: 1, pid: 1 }), null);
    assert.strictEqual(await conn.findPermittedSerialPort({ vid: 1, pid: 1 }), null);
    const D = { vendorId: 1, productId: 1, serialNumber: "s" };
    setNav({ usb: { getDevices: async () => [D] }, serial: { getPorts: async () => { throw new Error("denied"); } } });
    assert.strictEqual(await conn.findPermittedUsbDevice({ vid: 1, pid: 1, serial: "s" }), D);
    assert.strictEqual(await conn.findPermittedSerialPort({ vid: 1, pid: 1 }), null); // a rejecting API is "nothing found"
    // watchDeviceArrivals wires both platform "connect" events to the supervisor
    const listeners = {};
    setNav({ usb: { addEventListener: (n, f) => { listeners["usb:" + n] = f; } }, serial: { addEventListener: (n, f) => { listeners["serial:" + n] = f; } } });
    const appeared = [];
    conn.watchDeviceArrivals({ onDeviceAppeared: (identity) => appeared.push(identity) });
    listeners["usb:connect"]({ device: { vendorId: 1, productId: 2, serialNumber: "s" } });
    listeners["serial:connect"]({ port: { getInfo: () => ({ usbVendorId: 3, usbProductId: 4 }) } });
    listeners["serial:connect"]({ target: { getInfo: () => ({ usbVendorId: 5, usbProductId: 6 }) } }); // SerialPort as the event target
    listeners["usb:connect"]({}); listeners["serial:connect"]({ target: {} }); // no usable identity -> null, never a match
    assert.deepStrictEqual(appeared, [
      { transport: "usb", vid: 1, pid: 2, serial: "s" }, { transport: "serial", vid: 3, pid: 4, serial: null },
      { transport: "serial", vid: 5, pid: 6, serial: null }, null, { transport: "serial", vid: null, pid: null, serial: null }]);
    console.log("PASS findPermittedUsbDevice / findPermittedSerialPort / watchDeviceArrivals");
  } finally {
    if (saved === undefined) delete global.navigator; else setNav(saved);
  }
  await supervisorTests();
  await noticeTests();
  await navTests();
  console.log("ALL TESTS PASSED");
})().catch((e) => { console.error(e); process.exit(1); });

// ---- the reconnect supervisor -------------------------------------------------
function tick(ms) { return new Promise((r) => setTimeout(r, ms)); }
async function supervisorTests() {
  const id = { transport: "usb", vid: 1, pid: 2, serial: null };
  // reconnects after an unexpected loss: retries on the back-off until an attempt succeeds
  {
    const log = []; let n = 0;
    const s = conn.createReconnectSupervisor({ delaysMs: [1, 1, 1, 1], enabled: () => true, log: (c, t) => log.push(c + ":" + t), reconnect: async () => (++n >= 3) });
    s.onLinkLost(id);
    assert.ok(s.isActive());
    await tick(40);
    assert.strictEqual(n, 3); assert.ok(!s.isActive());
    assert.ok(log[0].startsWith("sys:Link lost — trying to reconnect to USB device 0x0001:0x0002"), log[0]);
    assert.strictEqual(log.length, 1); // success logs nothing more
  }
  // gives up after the plan is exhausted (and says so); attempt errors are logged, not fatal
  {
    const log = []; let n = 0;
    const s = conn.createReconnectSupervisor({ delaysMs: [1, 1, 1], enabled: () => true, log: (c, t) => log.push(c + ":" + t), reconnect: async () => { n++; throw new Error("busy"); } });
    s.onLinkLost(id);
    await tick(40);
    assert.strictEqual(n, 3); assert.ok(!s.isActive());
    assert.strictEqual(log.filter((l) => l.startsWith("warn:reconnect attempt")).length, 3);
    assert.ok(log[log.length - 1].includes("gave up reconnecting after 3 attempts"), log);
  }
  // the checkbox: off at the loss -> nothing; turned off mid-way -> stops
  {
    let n = 0, on = false;
    const s = conn.createReconnectSupervisor({ delaysMs: [1, 1, 1], enabled: () => on, reconnect: async () => { n++; return false; } });
    s.onLinkLost(id); assert.ok(!s.isActive());
    on = true; s.onLinkLost(id); assert.ok(s.isActive());
    on = false; await tick(20);
    assert.strictEqual(n, 0); assert.ok(!s.isActive());
  }
  // expectReboot(): the longer plan + a "rebooting" message; connected() clears it
  {
    const log = [];
    const s = conn.createReconnectSupervisor({ delaysMs: [1], rebootDelaysMs: [1, 1, 1, 1, 1], enabled: () => true, log: (c, t) => log.push(t), reconnect: async () => false });
    s.expectReboot(); s.onLinkLost(id);
    await tick(40);
    assert.ok(log[0].startsWith("Device is rebooting — reconnecting to"), log[0]);
    assert.ok(log[log.length - 1].includes("after 5 attempts"), log);
    s.expectReboot(); s.connected(); s.onLinkLost(id); await tick(20);
    assert.ok(log.some((t) => t.startsWith("Link lost")) && log[log.length - 1].includes("after 1 attempts"), log);
  }
  // suppress(): a bootloader reboot -> no attempt at all, explained once, then cleared
  {
    const log = []; let n = 0;
    const s = conn.createReconnectSupervisor({ delaysMs: [1], enabled: () => true, log: (c, t) => log.push(t), reconnect: async () => { n++; return true; } });
    s.suppress(); s.onLinkLost(id);
    await tick(10);
    assert.strictEqual(n, 0); assert.ok(!s.isActive());
    assert.ok(log.length === 1 && /bootloader/.test(log[0]), log);
    s.onLinkLost(id); await tick(10); // the next loss reconnects normally
    assert.strictEqual(n, 1);
  }
  // onDeviceAppeared(identity): a platform connect event for THE device being
  // recovered retries at once (before the long delay); unrelated arrivals
  // neither wake the supervisor nor consume a retry
  {
    let n = 0;
    const s = conn.createReconnectSupervisor({ delaysMs: [10000, 10000], enabled: () => true, reconnect: async () => (++n >= 1) });
    s.onLinkLost(id); s.onDeviceAppeared({ transport: "usb", vid: 1, pid: 2, serial: null });
    await tick(150);
    assert.strictEqual(n, 1); assert.ok(!s.isActive());
    s.onDeviceAppeared(id); await tick(150); assert.strictEqual(n, 1); // inert when idle
  }
  {
    let n = 0;
    const wantSn = { transport: "usb", vid: 1, pid: 2, serial: "SN1" };
    const s = conn.createReconnectSupervisor({ delaysMs: [10000, 10000, 10000], enabled: () => true, reconnect: async () => { n++; return false; } });
    s.onLinkLost(wantSn);
    for (const other of [
      { transport: "usb", vid: 1, pid: 3, serial: null },       // another pid
      { transport: "usb", vid: 9, pid: 2, serial: null },       // another vid
      { transport: "usb", vid: 1, pid: 2, serial: "SN2" },      // same ids, another serial number
      { transport: "usb", vid: null, pid: null, serial: null }, // no usable ids
      null, undefined,
    ]) s.onDeviceAppeared(other);
    await tick(200);
    assert.strictEqual(n, 0, "unrelated arrivals must not wake the supervisor"); assert.ok(s.isActive());
    // the device itself (same serial), or a serial-port arrival for it (no serial number) -> immediate retry
    s.onDeviceAppeared({ transport: "serial", vid: 1, pid: 2, serial: null }); await tick(150);
    assert.strictEqual(n, 1);
    s.onDeviceAppeared(wantSn); await tick(150);
    assert.strictEqual(n, 2); assert.ok(s.isActive());
    // and the arrival that matches during an in-flight attempt is still only remembered
    s.stop();
    assert.ok(conn.identityMatches({ vid: 1, pid: 2, serial: null }, { vid: 1, pid: 2, serial: "any" }));
    assert.ok(!conn.identityMatches({ vid: 1, pid: 2 }, { vid: 1, pid: null }));
    assert.ok(!conn.identityMatches(null, { vid: 1, pid: 2 }));
    assert.ok(conn.identityMatches({ vid: null, pid: null, serial: null }, { vid: 1, pid: 2 })); // nothing wanted beyond "some device"
  }
  // attempts are serialized: an arrival event during an in-flight attempt does
  // NOT start a second open; it is remembered and retried once, right after
  // the current attempt settles (before the plan's long delay)
  {
    let inFlight = 0, maxInFlight = 0, n = 0, release = null;
    const s = conn.createReconnectSupervisor({ delaysMs: [1, 10000, 10000], enabled: () => true,
      reconnect: () => new Promise((r) => { n++; inFlight++; maxInFlight = Math.max(maxInFlight, inFlight); release = (ok) => { inFlight--; r(ok); }; }) });
    s.onLinkLost(id);
    await tick(10);
    assert.strictEqual(n, 1);
    s.onDeviceAppeared(id); s.onDeviceAppeared(id); // arrivals while attempt 1 is still open
    await tick(150);
    assert.strictEqual(n, 1); // nothing overlapped
    release(false);          // attempt 1 fails -> one immediate retry instead of the 10 s wait
    await tick(150);
    assert.strictEqual(n, 2); assert.strictEqual(maxInFlight, 1);
    release(true); await tick(10);
    assert.ok(!s.isActive());
    // a success with a pending arrival does not retry
    s.onLinkLost(id); await tick(10); assert.strictEqual(n, 3);
    s.onDeviceAppeared(id); release(true); await tick(150);
    assert.strictEqual(n, 3); assert.ok(!s.isActive()); assert.strictEqual(maxInFlight, 1);
  }
  // a second loss while an older attempt is still in flight (the device dropped
  // again during the post-open probe): the new generation's timer fires into
  // the in-flight attempt; when that attempt settles the retry must still happen
  {
    let n = 0, release = null;
    const s = conn.createReconnectSupervisor({ delaysMs: [1, 1, 1, 1], enabled: () => true,
      reconnect: () => new Promise((r) => { n++; release = r; }) });
    s.onLinkLost(id); await tick(10);
    assert.strictEqual(n, 1);
    s.connected();      // the console's connect path opened the device...
    s.onLinkLost(id);   // ...and it dropped again before attempt 1 returned
    await tick(20);     // the new generation's timer fires into the in-flight attempt
    assert.strictEqual(n, 1); assert.ok(s.isActive());
    release(true);      // stale attempt settles (it even "succeeded")
    await tick(150);
    assert.strictEqual(n, 2, "the current generation must retry after the stale attempt settles");
    release(true); await tick(10);
    assert.ok(!s.isActive());
    // same with the stale attempt failing, and without connected() in between
    n = 0; s.onLinkLost(id); await tick(10); assert.strictEqual(n, 1);
    s.onLinkLost(id); await tick(20); assert.strictEqual(n, 1);
    release(false); await tick(150);
    assert.strictEqual(n, 2); release(true); await tick(10); assert.ok(!s.isActive());
  }
  // the open gate: one open at a time across manual / auto-connect / supervisor.
  // While a manual open holds the gate (a pending chooser, say) the supervisor
  // does not open a second connection and does not count it as a failed
  // attempt; it opens once the gate frees. A refused entry resolves false.
  {
    const gate = conn.createOpenGate();
    let releaseManual = null, manualRuns = 0;
    const manual = gate.run(() => new Promise((r) => { manualRuns++; releaseManual = r; }));
    assert.ok(gate.isBusy());
    assert.strictEqual(await gate.run(async () => "second"), false); // refused while busy
    let n = 0;
    const s = conn.createReconnectSupervisor({ gate, delaysMs: [1, 1], enabled: () => true, reconnect: async () => { n++; return true; } });
    s.onLinkLost(id);
    await tick(600);             // several poll intervals (250 ms) pass while the manual open is pending
    assert.strictEqual(n, 0); assert.ok(s.isActive()); assert.strictEqual(manualRuns, 1);
    releaseManual("manual-done"); // the chooser settled
    assert.strictEqual(await manual, "manual-done");
    await tick(400);
    assert.strictEqual(n, 1); assert.ok(!s.isActive()); assert.ok(!gate.isBusy());
    // a gate built with the page's connected() predicate refuses (opens nothing)
    // while the page already holds a device, and is not left busy by that
    {
      let isConnected = false, opened = 0;
      const g = conn.createOpenGate({ connected: () => isConnected });
      assert.strictEqual(await g.run(async () => { opened++; return "ok"; }), "ok");
      isConnected = true;
      assert.strictEqual(await g.run(async () => { opened++; return "ok"; }), false);
      assert.strictEqual(opened, 1); assert.ok(!g.isBusy()); assert.ok(g.isConnected());
      isConnected = false;
      assert.strictEqual(await g.run(async () => { opened++; return "again"; }), "again");
      assert.strictEqual(opened, 2);
    }
    // the gate frees after a throwing open as well
    await assert.rejects(gate.run(async () => { throw new Error("open failed"); }), /open failed/);
    assert.ok(!gate.isBusy());
    assert.strictEqual(await gate.run(async () => 7), 7);
  }
  // cancellation of an attempt already blocked inside opts.reconnect(): the
  // checkbox is unticked (stop()) while the open is pending; when it settles
  // the page's connected() call is ignored and opts.discard() closes the open
  {
    let release = null, discarded = 0, tokens = [];
    const s = conn.createReconnectSupervisor({ delaysMs: [1, 1, 1], enabled: () => true,
      reconnect: (identity, n, token) => new Promise((r) => { tokens.push(token); release = (ok) => { if (ok) s.connected(); r(ok); }; }),
      discard: async () => { discarded++; } });
    s.onLinkLost(id); await tick(10);
    assert.strictEqual(tokens.length, 1); assert.ok(!tokens[0].cancelled());
    s.stop("auto-reconnect is off");           // unticked while the open is pending
    assert.ok(tokens[0].cancelled());
    release(true);                              // the open finished anyway and the page called connected()
    await tick(20);
    assert.strictEqual(discarded, 1, "a cancelled attempt's open must be discarded");
    assert.ok(!s.isActive());
    // connected() was ignored: a real connected() clears suppress() (stop()
    // does not), so a surviving suppression is observable through the next
    // loss, which must refuse to reconnect
    const log = []; let n2 = 0;
    const s2 = conn.createReconnectSupervisor({ delaysMs: [1, 1], enabled: () => true, log: (c, m) => log.push(m),
      reconnect: (identity, n, token) => new Promise((r) => { n2++; release = (ok) => { if (ok) s2.connected(); r(ok); }; }), discard: async () => { discarded++; } });
    s2.onLinkLost(id); await tick(10);
    s2.suppress(); s2.stop(); release(true); await tick(20);
    assert.strictEqual(discarded, 2); assert.strictEqual(n2, 1);
    s2.onLinkLost(id); await tick(20);
    assert.ok(!s2.isActive() && /bootloader/.test(log[log.length - 1]) && n2 === 1, "connected() from a cancelled attempt must not reset the supervisor");
    // a newer loss cancels the older in-flight attempt too (its open is discarded),
    // and the current generation still retries
    discarded = 0; n2 = 0;
    const s3 = conn.createReconnectSupervisor({ delaysMs: [1, 1, 1], enabled: () => true,
      reconnect: (identity, n, token) => new Promise((r) => { n2++; release = (ok) => r(ok); }), discard: async () => { discarded++; } });
    s3.onLinkLost(id); await tick(10); assert.strictEqual(n2, 1);
    const first = release; s3.onLinkLost(id); await tick(20);
    first(true); await tick(150);
    assert.strictEqual(discarded, 1); assert.strictEqual(n2, 2);
    release(true); await tick(10); assert.ok(!s3.isActive());
    // stop -> the user connects manually while the cancelled attempt's lookup is
    // still pending -> that lookup fails (returned false, opened nothing): the
    // page's discard (= its disconnect) must NOT run, or it would drop the new
    // manual connection
    discarded = 0;
    const s5 = conn.createReconnectSupervisor({ delaysMs: [1, 1], enabled: () => true,
      reconnect: () => new Promise((r) => { release = r; }), discard: async () => { discarded++; } });
    s5.onLinkLost(id); await tick(10); s5.stop("auto-reconnect is off");
    s5.connected();                              // the manual connection
    release(false); await tick(20);              // the stale lookup settles without an open
    assert.strictEqual(discarded, 0, "a cancelled attempt that opened nothing must not discard the manual connection");
    // ...whereas a cancelled attempt that DID open something still discards it
    s5.onLinkLost(id); await tick(10); s5.stop(); release(true); await tick(20);
    assert.strictEqual(discarded, 1);
    // without a discard callback a cancelled attempt is simply dropped (no throw)
    const s4 = conn.createReconnectSupervisor({ delaysMs: [1], enabled: () => true, reconnect: () => new Promise((r) => { release = r; }) });
    s4.onLinkLost(id); await tick(10); s4.stop(); release(true); await tick(10); assert.ok(!s4.isActive());
  }
  // stop(): a manual disconnect cancels a pending attempt; an attempt that
  // completes after stop() / a newer loss is ignored (generation check)
  {
    let n = 0, resolveAttempt = null;
    const s = conn.createReconnectSupervisor({ delaysMs: [1, 1], enabled: () => true, reconnect: () => new Promise((r) => { n++; resolveAttempt = r; }) });
    s.onLinkLost(id); await tick(10);
    assert.strictEqual(n, 1); s.stop();
    assert.ok(!s.isActive()); resolveAttempt(false); await tick(10);
    assert.strictEqual(n, 1); // no reschedule after stop
    // no identity -> nothing to reconnect to
    s.onLinkLost(null); assert.ok(!s.isActive());
  }
  console.log("PASS reconnect supervisor: back-off, give-up, checkbox, expectReboot, suppress, device arrival, stop");
}

// ---- hand-off notices + the checkbox preference ---------------------------------
async function noticeTests() {
  const posted = [];
  conn.postReleased({ postMessage: (m) => posted.push(m) }, { transport: "usb", vid: 1, pid: 2, serial: "s" });
  assert.strictEqual(posted.length, 1);
  assert.deepStrictEqual(posted[0], { type: "released", identity: { transport: "usb", vid: 1, pid: 2, serial: "s" }, page: "" });
  conn.postReleased(null, { transport: "usb" }); conn.postReleased({ postMessage: () => {} }, null);
  conn.postReleased({ postMessage: () => { throw new Error("closed"); } }, { transport: "usb" }); // best effort
  assert.strictEqual(posted.length, 1);
  // no BroadcastChannel / no localStorage in this environment: safe defaults
  const BC = global.BroadcastChannel; delete global.BroadcastChannel;
  assert.strictEqual(conn.openDeviceChannel(), null);
  if (BC) global.BroadcastChannel = BC;
  assert.strictEqual(conn.loadAutoReconnect(), true); // default on (no storage at all here)
  conn.saveAutoReconnect(false); // must not throw without localStorage
  console.log("PASS released notice + auto-reconnect preference defaults");
}

// ---- lint: every console wires the helpers the same way ---------------------------
{
  const must = (src, rel, re, what) => assert.ok(re.test(src), rel + ": " + what);
  for (const rel of consoles) {
    const src = fs.readFileSync(path.join(root, rel), "utf8");
    must(src, rel, /<input type="checkbox" id="autoReconnect" checked>/, "no auto-reconnect checkbox (default on)");
    must(src, rel, /const connectParams = parseConnectParams\(location\.search\);/, "does not parse the connect params");
    must(src, rel, /const reconnect = createReconnectSupervisor\(\{/, "no reconnect supervisor");
    must(src, rel, /watchDeviceArrivals\(reconnect\);/, "does not watch device arrivals");
    must(src, rel, /loadAutoReconnect\(\)/, "checkbox not initialised from the stored preference");
    must(src, rel, /saveAutoReconnect\(/, "checkbox change not persisted");
    must(src, rel, /reconnect\.connected\(\);/, "a successful connect must reset the supervisor");
    must(src, rel, /reconnect\.onLinkLost\(connIdentity\);/, "an unexpected link loss must arm the supervisor");
    // a manual disconnect stops the supervisor, keeps the identity aside, CLOSES
    // the device, and only then posts the released notice (postMessage is
    // synchronous; posting first would let the hub reopen a device this page
    // still holds). pagehide is the documented exception.
    must(src, rel, /reconnect\.stop\(\); const released = connIdentity; connIdentity = null;/, "a manual disconnect must stop the supervisor and keep the identity for the notice");
    const holds = [...src.matchAll(/reconnect\.stop\(\); const released = connIdentity; connIdentity = null;/g)];
    for (const h of holds) {
      const post = src.indexOf("postReleased(deviceChannel, released);", h.index);
      assert.ok(post > 0, rel + ": a manual disconnect never posts the released notice");
      const between = src.slice(h.index, post);
      assert.ok(/await (?:t\.close\(\)|safeClose\(\)|port\.close\(\)|usbReadDone)/.test(between) && !/function /.test(between), rel + ": the released notice must be posted after the transport is closed");
    }
    assert.strictEqual((src.match(/postReleased\(deviceChannel, connIdentity\)/g) || []).length, 1, rel + ": only the pagehide path may post before closing");
    must(src, rel, /createReconnectSupervisor\(\{\s*gate: openGate,\s*discard: \(\) => /, "the supervisor has no discard path for a cancelled attempt's open");
    must(src, rel, /window\.addEventListener\("pagehide", \(\) => \{ if \([^)]*\) postReleased\(deviceChannel, connIdentity\); \}\);/, "no released notice on pagehide");
    must(src, rel, /if \(connectParams\.autoconnect\) autoConnect\(\);/, "no auto-connect kick-off");
    // the chooser-less open paths exist: a preset device / port skips requestDevice / requestPort
    must(src, rel, /preset \|\| await navigator\.(usb\.requestDevice|serial\.requestPort)|if \(preset\) return preset;/, "no chooser-less (preset) open path");
    if (src.includes("navigator.serial.requestPort(")) must(src, rel, /preset \|\| await navigator\.serial\.requestPort\(/, "serial open path ignores a preset port");
  }
  // every connect entry point goes through the page's open gate: each
  // `<name>Locked` function is called only from its `<name>` wrapper, which is
  // exactly `return openGate.run(() => <name>Locked(...))`, and the supervisor
  // shares the gate
  for (const rel of [...consoles, hub]) {
    const src = fs.readFileSync(path.join(root, rel), "utf8");
    must(src, rel, /const openGate = createOpenGate\(\{ connected: \(\) => !!/, "the open gate must be built with the page's connected() predicate");
    if (rel !== hub) {
      // the reconnect callback: a lookup outside the gate, then — before opening —
      // a re-check that the attempt was not cancelled and no manual connect completed
      const cb = /async function reconnectAttempt\(identity, attempt, token\) \{([\s\S]*?)\n    \}\n/.exec(src);
      assert.ok(cb, rel + ": reconnectAttempt must take (identity, attempt, token)");
      const lookups = cb[1].split("\n").filter((l) => /await findPermitted(?:UsbDevice|SerialPort)\(/.test(l)).length; // lookup LINES (a ternary holds two calls)
      const rechecks = (cb[1].match(/if \(token\.cancelled\(\) \|\| (?:device|transport|port \|\| usb)\) return false;/g) || []).length;
      assert.ok(lookups >= 1 && rechecks === lookups, rel + ": every permitted-device lookup in reconnectAttempt must be followed by `if (token.cancelled() || <connected>) return false;` (" + lookups + " lookups, " + rechecks + " re-checks)");
      for (const m of cb[1].matchAll(/await findPermitted(?:UsbDevice|SerialPort)\([^\n]*\n([^\n]*)/g))
        assert.ok(/if \(token\.cancelled\(\) \|\|/.test(m[1]), rel + ": the re-check must directly follow the lookup: " + m[1].trim());
    }
    const locked = [...src.matchAll(/async function (\w+)Locked\(([^)]*)\) \{/g)];
    assert.ok(locked.length >= 1, rel + ": no gated connect entry point");
    for (const [, name, params] of locked) {
      const wrapper = "    async function " + name + "(" + params + ") {\n      return openGate.run(() => " + name + "Locked(" + params + "));\n    }\n";
      assert.ok(src.includes(wrapper), rel + ": " + name + "() is not the gate wrapper for " + name + "Locked()");
      const calls = [...src.matchAll(new RegExp("(?<!function )\\b" + name + "Locked\\(", "g"))];
      assert.strictEqual(calls.length, 1, rel + ": " + name + "Locked() must be called only through openGate.run()");
    }
    if (rel !== hub) must(src, rel, /createReconnectSupervisor\(\{\s*gate: openGate,/, "the supervisor does not share the open gate");
    // a page with two transports gates both through the SAME gate
    if (src.includes("navigator.serial.requestPort(") && src.includes("navigator.usb.requestDevice("))
      assert.ok(locked.length >= 2 || /async function connectLocked\(kind, preset\)/.test(src), rel + ": both transports must be gated");
  }
  const sys = fs.readFileSync(path.join(root, "components/system/web/system_console.html"), "utf8");
  assert.ok(/if \(bootloader\) reconnect\.suppress\(\); else reconnect\.expectReboot\(\);/.test(sys), "system console: REBOOT arms, REBOOT_TO_BOOTLOADER suppresses");
  const ota = fs.readFileSync(path.join(root, "components/ota/web/ota_console.html"), "utf8");
  assert.ok((ota.match(/reconnect\.expectReboot\(\);/g) || []).length >= 2, "ota console: FINISH and rollback arm the supervisor");
  const cd = fs.readFileSync(path.join(root, "components/coredump/web/coredump_console.html"), "utf8");
  assert.ok(/reconnect\.expectReboot\(\);/.test(cd), "coredump console: a triggered crash arms the supervisor");
  // the hub: hands off (closes BEFORE opening the app), offers Reconnect, never auto-reconnects
  const handOff = /async function handOff\([^)]*\) \{([\s\S]*?)\n    \}\n/.exec(hubSrc);
  assert.ok(handOff, "hub: no handOff()");
  // (the early return for "not connected" opens the plain link first; the hand-off path closes, then opens)
  // the tab is reserved synchronously in the click (pop-up blockers need the
  // gesture) and the app is navigated into it only after the device is closed;
  // a blocked tab releases nothing
  assert.ok(!handOff[1].includes("window.open("), "hub: handOff must not open a tab itself (it is reserved in the click handler)");
  assert.ok(handOff[1].indexOf("await disconnect();") >= 0 && handOff[1].indexOf("await disconnect();") < handOff[1].lastIndexOf("tab.location.href = href;"), "hub: handOff must close the device before navigating the reserved tab");
  const click = /a\.addEventListener\("click", \(ev\) => \{([\s\S]*?)\n      \}\);/.exec(hubSrc);
  assert.ok(click && click[1].indexOf("const tab = reserveTab(file);") >= 0 && click[1].indexOf("if (!tab) return;") >= 0 && click[1].indexOf("reserveTab(") < click[1].indexOf("handOff("), "hub: the click handler must reserve the tab before handing off, and stop when blocked");
  assert.ok(/function reserveTab\(appFile\) \{[\s\S]*?window\.open\("", "_blank"\)/.test(hubSrc), "hub: reserveTab opens a blank tab");
  // a hand-off is atomic: `handingOff` holds from the click until the tab is
  // navigated (cleared in finally), and app clicks in between are ignored
  assert.ok(/let handingOff = false;/.test(hubSrc), "hub: no hand-off guard");
  assert.ok(handOff[1].indexOf("handingOff = true;") >= 0 && handOff[1].indexOf("handingOff = true;") < handOff[1].indexOf("await disconnect();") && /finally \{\s*handingOff = false;\s*\}/.test(handOff[1]), "hub: handOff must hold the guard from before disconnect() until it settles");
  assert.ok(click && click[1].indexOf('if (handingOff) { ev.preventDefault(); return; }') >= 0 && click[1].indexOf("if (handingOff)") < click[1].indexOf("reserveTab("), "hub: a click during a hand-off must be ignored before reserving a tab");
  // any successful connection (manual or Reconnect) ends a pending hand-off
  const hubConnect = /async function connectLocked\(kind, preset\) \{([\s\S]*?)\n    \}\n/.exec(hubSrc);
  assert.ok(hubConnect && /handedOff = null; hideHandoff\(\);/.test(hubConnect[1]), "hub: connect() must clear the hand-off state on success");
  assert.ok(!/createReconnectSupervisor\(\{/.test(hubSrc.replace(connectBlock, "")), "hub must not auto-reconnect");
  assert.ok(/els\.reconnectBtn\.addEventListener\("click", reconnectHandedOff\);/.test(hubSrc), "hub: no Reconnect button");
  assert.ok(/deviceChannel\.onmessage = /.test(hubSrc) && /msg\.type !== "released"/.test(hubSrc), "hub: does not listen for released notices");
  assert.ok(/const preset = kind === "usb" \? await findPermittedUsbDevice\(want\) : await findPermittedSerialPort\(want\);/.test(hubSrc), "hub: Reconnect must try the permitted device before the chooser");
  // the hub's unexpected-loss path must drop the stale transport, or connect()'s
  // one-at-a-time guard would refuse every later Connect / Reconnect
  const hubLoss = /function onLinkLost\(\) \{([\s\S]*?)\n    \}\n/.exec(hubSrc);
  assert.ok(hubLoss && /const t = transport; transport = null;/.test(hubLoss[1]), "hub: onLinkLost must clear the stale transport");
  assert.ok(/async function connectLocked\(kind, preset\) \{\n      if \(transport\) return;/.test(hubSrc), "hub: connect() refuses a second open while connected (and the gate serializes pending ones)");
  // every console's unexpected-loss path clears its transport / device too
  for (const rel of consoles) {
    const src = fs.readFileSync(path.join(root, rel), "utf8");
    const loss = /function (?:onLinkLost|onTransportGone|usbLinkLost|handleLinkLost)\([^)]*\) \{([\s\S]*?)\n    \}\n/.exec(src);
    if (loss) assert.ok(/(transport|port|usb) = null/.test(loss[1]), rel + ": the loss path keeps the stale transport");
    else assert.ok(/safeClose\(\)/.test(src), rel + ": no loss path found");
  }
  console.log("PASS lint: every console wires auto-connect / auto-reconnect / released notices; the hub hands off and never auto-reconnects");
}



// =============================================================================
// App navigation: every web app (the 8 dispatcher-module consoles, the hub and
// the 7 other apps) links back to the Device Hub (primary) and the apps page;
// a connected console hands its device back to the hub on a plain click.
// =============================================================================
const otherApps = [
  "components/basicmicro/web/mcp_console.html",
  "components/odrive_ascii/web/hid_visualizer.html",
  "components/odrive_ascii/web/odrive_console.html",
  "components/odrive_ascii/web/odrive_control_panel.html",
  "components/odrive_ascii/web/odrive_webusb_console.html",
  "components/twai/web/can_console.html",
  "components/usb_device/web/board_console.html",
];
function extractNavBlock(src, rel) {
  const start = src.indexOf("    // --- begin app navigation");
  assert.ok(start >= 0, rel + ": no app navigation block");
  const endMarker = "    // --- end app navigation ---\n";
  const end = src.indexOf(endMarker, start);
  assert.ok(end >= 0, rel + ": unterminated app navigation block");
  return src.slice(start, end + endMarker.length);
}
function navTests() {
  let navBlock = null;
  const allApps = [...consoles, hub, ...otherApps];
  for (const rel of allApps) {
    const src = fs.readFileSync(path.join(root, rel), "utf8");
    const b = extractNavBlock(src, rel);
    if (navBlock === null) navBlock = b;
    else assert.strictEqual(b, navBlock, rel + ": app navigation block differs from " + consoles[0]);
    // the links: every app has both (hub primary, apps page), the hub only "All apps"
    const hubLink = /<a id="navHub" class="hub" href="dispatcher_hub.html"[^>]*>Device Hub<\/a>/.test(src);
    const appsLink = /<a id="navApps" href="index.html"[^>]*>All apps<\/a>/.test(src);
    assert.ok(appsLink, rel + ": no \"All apps\" link to index.html");
    if (rel === hub) assert.ok(!hubLink && !src.includes('id="navHub"'), "the hub must not link to itself");
    else assert.ok(hubLink, rel + ": no primary \"Device Hub\" link to dispatcher_hub.html");
    assert.ok(/<nav class="espp-nav" aria-label="espp web apps">/.test(src), rel + ": no nav strip");
    // installed from real code: consoles hand the device back, the rest navigate plainly
    if (consoles.includes(rel)) {
      const call = /installAppNav\(\{ connected: \(\) => (!!\w+|!!\(port \|\| usb\)), handBack: async \(\) => \{ const id = connIdentity; await (?:disconnect\(\)|\(usb \? usbDisconnect\(\) : disconnect\(\)\)); return id; \} \}\);/.exec(src);
      assert.ok(call, rel + ": installAppNav must get the page's connected() and a handBack that runs the manual disconnect and resolves the identity");
    } else {
      assert.ok(/^\s*installAppNav\(\);/m.test(src), rel + ": installAppNav() not called");
    }
  }
  console.log("PASS every web app carries the identical app navigation block and both links (" + allApps.length + " files)");
  // navHref: siblings when hosted, the hosted copies from file://
  const nav = new Function(navBlock + "\n return { navHref, hubConnectQuery, installAppNav, ESPP_APPS_BASE };")();
  const setLoc = (v) => Object.defineProperty(global, "location", { value: v, configurable: true, writable: true });
  const savedLoc = global.location;
  try {
    setLoc({ protocol: "https:" });
    assert.strictEqual(nav.navHref("dispatcher_hub.html"), "dispatcher_hub.html");
    setLoc({ protocol: "http:" });
    assert.strictEqual(nav.navHref("index.html"), "index.html");
    setLoc({ protocol: "file:" });
    assert.strictEqual(nav.navHref("dispatcher_hub.html"), "https://esp-cpp.github.io/espp/apps/dispatcher_hub.html");
    assert.strictEqual(nav.navHref("index.html"), nav.ESPP_APPS_BASE + "index.html");
  } finally { if (savedLoc === undefined) delete global.location; else setLoc(savedLoc); }
  // the hand-back query: the device identity, no module; the hub parses it back
  const q = nav.hubConnectQuery({ transport: "usb", vid: 0x1209, pid: 0x0d32, serial: "AB C" });
  assert.strictEqual(q, "autoconnect=1&transport=usb&vid=0x1209&pid=0x0d32&serial=AB+C");
  assert.deepStrictEqual(conn.parseConnectParams("?" + q), { autoconnect: true, transport: "usb", vid: 0x1209, pid: 0x0d32, serial: "AB C" });
  assert.strictEqual(moduleOverrideFromQuery("?" + q), null);
  assert.strictEqual(nav.hubConnectQuery({ transport: "serial", vid: 1, pid: 2, serial: null }), "autoconnect=1&transport=serial&vid=0x0001&pid=0x0002");
  assert.strictEqual(nav.hubConnectQuery(null), "");
  assert.strictEqual(nav.hubConnectQuery({ transport: "ble", vid: 1, pid: 2 }), "");
  // installAppNav with a fake DOM: a plain click while connected closes FIRST
  // (handBack), then navigates this tab with the query; modified / middle
  // clicks, or an unconnected page, leave the plain link alone
  const savedDoc = global.document;
  const setDoc = (v) => Object.defineProperty(global, "document", { value: v, configurable: true, writable: true });
  try {
    const assigned = [];
    setLoc({ protocol: "https:", assign: (u) => assigned.push(u) });
    const mkLink = () => { const handlers = {}; return { href: "", addEventListener: (n, f) => { handlers[n] = f; }, handlers }; };
    const hub = mkLink(), apps = mkLink();
    setDoc({ getElementById: (id) => (id === "navHub" ? hub : id === "navApps" ? apps : null) });
    const order = []; let connected = true;
    nav.installAppNav({ connected: () => connected, handBack: async () => { order.push("closed"); return { transport: "usb", vid: 1, pid: 2, serial: "S" }; } });
    assert.strictEqual(hub.href, "dispatcher_hub.html"); assert.strictEqual(apps.href, "index.html");
    const ev = (over) => ({ button: 0, defaultPrevented: false, metaKey: false, ctrlKey: false, shiftKey: false, altKey: false, preventDefault() { this.defaultPrevented = true; order.push("prevented"); }, ...over });
    return (async () => {
      let e = ev({}); hub.handlers.click(e); await tick(10);
      assert.deepStrictEqual(order, ["prevented", "closed"]);
      assert.deepStrictEqual(assigned, ["dispatcher_hub.html?autoconnect=1&transport=usb&vid=0x0001&pid=0x0002&serial=S"]);
      for (const mod of [{ button: 1 }, { metaKey: true }, { ctrlKey: true }, { shiftKey: true }, { altKey: true }]) { e = ev(mod); hub.handlers.click(e); assert.ok(!e.defaultPrevented, JSON.stringify(mod)); }
      connected = false; e = ev({}); hub.handlers.click(e); await tick(10);
      assert.ok(!e.defaultPrevented); assert.strictEqual(assigned.length, 1);
      // a failing handBack still navigates (plain hub URL)
      connected = true;
      const hub2 = mkLink(); setDoc({ getElementById: (id) => (id === "navHub" ? hub2 : null) });
      nav.installAppNav({ connected: () => true, handBack: async () => { throw new Error("close failed"); } });
      e = ev({}); hub2.handlers.click(e); await tick(10);
      assert.strictEqual(assigned[1], "dispatcher_hub.html");
      // no links on the page: nothing to do
      setDoc({ getElementById: () => null }); nav.installAppNav();
      console.log("PASS app navigation: hosted vs file:// hrefs, hand-back query, click hands the device back before navigating");
    })().finally(() => { if (savedDoc === undefined) delete global.document; else setDoc(savedDoc); if (savedLoc === undefined) delete global.location; else setLoc(savedLoc); });
  } catch (e) { if (savedDoc === undefined) delete global.document; else setDoc(savedDoc); throw e; }
}
// the hub honours the hand-back query on load (through connect(kind, preset) and the gate)
{
  assert.ok(/const connectParams = parseConnectParams\(location\.search\);/.test(hubSrc), "hub: does not parse the connect params");
  const load = /async function autoConnectOnLoad\(\) \{([\s\S]*?)\n    \}\n/.exec(hubSrc);
  assert.ok(load && /await findPermittedUsbDevice\(want\)/.test(load[1]) && /await findPermittedSerialPort\(want\)/.test(load[1]) && /await connect\(kind, preset\);/.test(load[1]), "hub: autoConnectOnLoad must open the permitted device through connect(kind, preset)");
  assert.ok(/if \(connectParams\.autoconnect\) autoConnectOnLoad\(\);/.test(hubSrc), "hub: no auto-connect kick-off");
  assert.ok(!/createReconnectSupervisor\(\{/.test(hubSrc.replace(extractConnectBlock(hubSrc, hub), "")), "hub must still never auto-reconnect");
  console.log("PASS hub honours the hand-back auto-connect query on load");
}

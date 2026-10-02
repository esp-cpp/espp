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
  assert.ok(hubSrc.includes('a.href = file + "?" + connectQuery(moduleId, transport ? transport.identity() : null);'),
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

const conn = new Function(connectBlock + "\n return { parseConnectParams, usbIdentity, serialIdentity, describeIdentity, connectQuery, pickUsbDevice, pickSerialPort, findPermittedUsbDevice, findPermittedSerialPort, loadAutoReconnect, saveAutoReconnect, createReconnectSupervisor, watchDeviceArrivals, openDeviceChannel, postReleased };")();

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
    let appeared = 0;
    conn.watchDeviceArrivals({ onDeviceAppeared: () => appeared++ });
    listeners["usb:connect"](); listeners["serial:connect"]();
    assert.strictEqual(appeared, 2);
    console.log("PASS findPermittedUsbDevice / findPermittedSerialPort / watchDeviceArrivals");
  } finally {
    if (saved === undefined) delete global.navigator; else setNav(saved);
  }
  await supervisorTests();
  await noticeTests();
  console.log("ALL TESTS PASSED");
})().catch((e) => { console.error(e); process.exit(1); });

// ---- the reconnect supervisor -------------------------------------------------
const tick = (ms) => new Promise((r) => setTimeout(r, ms));
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
  // onDeviceAppeared(): a platform connect event retries at once (before the long delay)
  {
    let n = 0;
    const s = conn.createReconnectSupervisor({ delaysMs: [10000, 10000], enabled: () => true, reconnect: async () => (++n >= 1) });
    s.onLinkLost(id); s.onDeviceAppeared();
    await tick(150);
    assert.strictEqual(n, 1); assert.ok(!s.isActive());
    s.onDeviceAppeared(); await tick(150); assert.strictEqual(n, 1); // inert when idle
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
    s.onDeviceAppeared(); s.onDeviceAppeared(); // arrivals while attempt 1 is still open
    await tick(150);
    assert.strictEqual(n, 1); // nothing overlapped
    release(false);          // attempt 1 fails -> one immediate retry instead of the 10 s wait
    await tick(150);
    assert.strictEqual(n, 2); assert.strictEqual(maxInFlight, 1);
    release(true); await tick(10);
    assert.ok(!s.isActive());
    // a success with a pending arrival does not retry
    s.onLinkLost(id); await tick(10); assert.strictEqual(n, 3);
    s.onDeviceAppeared(); release(true); await tick(150);
    assert.strictEqual(n, 3); assert.ok(!s.isActive()); assert.strictEqual(maxInFlight, 1);
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
    must(src, rel, /reconnect\.stop\(\);\s*postReleased\(deviceChannel, connIdentity\);\s*connIdentity = null;/, "a manual disconnect must stop the supervisor and post the released notice");
    must(src, rel, /window\.addEventListener\("pagehide", \(\) => \{ if \([^)]*\) postReleased\(deviceChannel, connIdentity\); \}\);/, "no released notice on pagehide");
    must(src, rel, /if \(connectParams\.autoconnect\) autoConnect\(\);/, "no auto-connect kick-off");
    // the chooser-less open paths exist: a preset device / port skips requestDevice / requestPort
    must(src, rel, /preset \|\| await navigator\.(usb\.requestDevice|serial\.requestPort)|if \(preset\) return preset;/, "no chooser-less (preset) open path");
    if (src.includes("navigator.serial.requestPort(")) must(src, rel, /preset \|\| await navigator\.serial\.requestPort\(/, "serial open path ignores a preset port");
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
  assert.ok(handOff[1].indexOf("await disconnect();") >= 0 && handOff[1].indexOf("await disconnect();") < handOff[1].lastIndexOf("window.open("), "hub: handOff must close the device before opening the app");
  assert.ok(!/createReconnectSupervisor\(\{/.test(hubSrc.replace(connectBlock, "")), "hub must not auto-reconnect");
  assert.ok(/els\.reconnectBtn\.addEventListener\("click", reconnectHandedOff\);/.test(hubSrc), "hub: no Reconnect button");
  assert.ok(/deviceChannel\.onmessage = /.test(hubSrc) && /msg\.type !== "released"/.test(hubSrc), "hub: does not listen for released notices");
  assert.ok(/const preset = kind === "usb" \? await findPermittedUsbDevice\(want\) : await findPermittedSerialPort\(want\);/.test(hubSrc), "hub: Reconnect must try the permitted device before the chooser");
  // the hub's unexpected-loss path must drop the stale transport, or connect()'s
  // one-at-a-time guard would refuse every later Connect / Reconnect
  const hubLoss = /function onLinkLost\(\) \{([\s\S]*?)\n    \}\n/.exec(hubSrc);
  assert.ok(hubLoss && /const t = transport; transport = null;/.test(hubLoss[1]), "hub: onLinkLost must clear the stale transport");
  assert.ok(/async function connect\(kind, preset\) \{\n      if \(transport\) return;/.test(hubSrc), "hub: connect() guards against a second concurrent open");
  // every console's unexpected-loss path clears its transport / device too
  for (const rel of consoles) {
    const src = fs.readFileSync(path.join(root, rel), "utf8");
    const loss = /function (?:onLinkLost|onTransportGone|usbLinkLost|handleLinkLost)\([^)]*\) \{([\s\S]*?)\n    \}\n/.exec(src);
    if (loss) assert.ok(/(transport|port|usb) = null/.test(loss[1]), rel + ": the loss path keeps the stale transport");
    else assert.ok(/safeClose\(\)/.test(src), rel + ": no loss path found");
  }
  console.log("PASS lint: every console wires auto-connect / auto-reconnect / released notices; the hub hands off and never auto-reconnects");
}



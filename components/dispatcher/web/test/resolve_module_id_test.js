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
function payload(version, mods, extraPerRecord = []) {
  const out = [version, 0, ...str("espp Hub"), ...str("1.2.3"), mods.length];
  for (const m of mods) {
    out.push(m.id, ...str(m.name), ...str(m.app), ...str(m.desc || ""));
    if (version >= 2) out.push(...str(m.protocol || ""), ...u16(m.protocolVersion || 0));
    out.push(...extraPerRecord);
  }
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
  // a newer version is parsed as v2 (any extra per-record fields are left unread)
  const v3 = parseDiscovery(payload(3, [OTA]));
  assert.strictEqual(v3.version, 3);
  assert.strictEqual(v3.modules[0].protocol, "espp.ota");
  console.log("PASS parseDiscovery decodes v1 and v2 payloads");
}
{
  const p2 = payload(2, [OTA, CD]);
  assert.strictEqual(hubParse(p2).modules[1].protocol, "espp.coredump");
  assert.throws(() => hubParse(new Uint8Array([...p2, 0])), /trailing/);
  // a v3 payload with extra per-record bytes is tolerated by the hub (parsed as v2)
  const p3 = payload(3, [OTA], [0xAA]);
  assert.strictEqual(hubParse(p3).modules[0].protocol, "espp.ota");
  console.log("PASS hub parser: strict for known versions, tolerant for newer ones");
}

// ---- resolveModuleId --------------------------------------------------------
const ident = (over = {}) => ({ protocol: "espp.ota", protocolVersion: 1, app: "ota_console.html", name: "OTA", fallback: 0, override: null, ...over });
const info = (version, mods) => ({ version, device: "d", fw: "1", modules: mods });
{
  // no discovery reply -> the default, silently
  let r = resolveModuleId(null, ident());
  assert.deepStrictEqual([r.id, r.source, r.protocolVersion, r.warnings], [0, "default", null, []]);
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
  // a newer payload than understood is warned about (and still resolved)
  r = resolveModuleId(info(3, [OTA]), ident());
  assert.ok(r.id === 0 && r.source === "protocol" && r.warnings.length === 1 && /v3/.test(r.warnings[0]), r);
  console.log("PASS resolveModuleId: override > protocol > app > name > default, with warnings");
}

// ---- moduleOverrideFromQuery + describeModuleChoice ------------------------
{
  assert.strictEqual(moduleOverrideFromQuery("?module=9"), 9);
  assert.strictEqual(moduleOverrideFromQuery("?a=1&module=0x1F"), 0x1F);
  assert.strictEqual(moduleOverrideFromQuery("?module=0"), 0);
  assert.strictEqual(moduleOverrideFromQuery("?module="), null);
  assert.strictEqual(moduleOverrideFromQuery("?module=255"), null);
  assert.strictEqual(moduleOverrideFromQuery("?module=abc"), null);
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

// ---- the hub links every app with its module id ------------------------------
{
  assert.ok(hubSrc.includes('a.href = m.app + "?module=" + m.id;'), "hub must link app?module=<id>");
  console.log("PASS hub links each console with ?module=<id>");
}

console.log("ALL TESTS PASSED");

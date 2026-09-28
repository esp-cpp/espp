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
  assert.strictEqual(moduleOverrideFromQuery("?module=abc"), null);
  // the whole token must be a number: a numeric prefix is not accepted
  assert.strictEqual(moduleOverrideFromQuery("?module=9oops"), null);
  assert.strictEqual(moduleOverrideFromQuery("?module=0x1g"), null);
  assert.strictEqual(moduleOverrideFromQuery("?module=1.5"), null);
  assert.strictEqual(moduleOverrideFromQuery("?module=-1"), null);
  assert.strictEqual(moduleOverrideFromQuery("?module=%2B3"), null); // a literal "+3" (a bare + is a space in a query)
  assert.strictEqual(moduleOverrideFromQuery("?module=0x"), null);
  assert.strictEqual(moduleOverrideFromQuery("?module= 7 "), 7);
  assert.strictEqual(moduleOverrideFromQuery("?module=0XfE"), 0xFE);
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

// ---- the hub links every app with its module id ------------------------------
{
  assert.ok(hubSrc.includes('a.href = m.app + "?module=" + m.id;'), "hub must link app?module=<id>");
  console.log("PASS hub links each console with ?module=<id>");
}

console.log("ALL TESTS PASSED");

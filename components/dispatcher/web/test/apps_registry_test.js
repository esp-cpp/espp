#!/usr/bin/env node
// Host test for the hosted-app registry: every app page carries the metadata
// the docs build needs (espp-category / espp-protocols / espp-transports), the
// protocol ids + versions it declares agree with the constants in its own
// script, doc/generate_apps_index.py accepts the real pages (and rejects bad
// ones), and the Device Hub's registry matching (appsForDevice /
// appsForModule / appNotes) links the right apps for a discovered device.
//
// Dependency-free (node + python3 on PATH); CI does not run it, so run it by
// hand after touching an app's <head>, the generator, or the hub:
//
//   node components/dispatcher/web/test/apps_registry_test.js
"use strict";
const fs = require("fs");
const os = require("os");
const path = require("path");
const assert = require("assert");
const { spawnSync } = require("child_process");

const root = path.resolve(__dirname, "..", "..", "..", "..");
const CATEGORIES = ["device management", "motor control", "bus tools", "input devices", "utilities"];
const TRANSPORTS = ["webusb", "webserial", "webhid"];

function meta(src, name) {
  // each attribute value ends at the quote it opened with (an apostrophe inside a double-quoted value is content)
  const m = new RegExp('<meta\\s+name=(["\'])' + name + '\\1\\s+content=(["\'])([\\s\\S]*?)\\2', "i").exec(src);
  return m ? m[3].trim() : null;
}
function parseProtocols(spec) {
  return spec.split(/\s+/).filter(Boolean).map((t) => {
    const m = /^([A-Za-z0-9_-]+(?:\.[A-Za-z0-9_-]+)+):([0-9]+)(\?)?$/.exec(t);
    assert.ok(m, "malformed protocol entry " + t);
    return { id: m[1], version: Number(m[2]), optional: m[3] === "?" };
  });
}

// ---- 1. every hosted page (what the docs workflow globs) carries valid metadata
const pages = [];
for (const comp of fs.readdirSync(path.join(root, "components"))) {
  const web = path.join(root, "components", comp, "web");
  if (!fs.existsSync(web)) continue;
  for (const f of fs.readdirSync(web)) if (f.endsWith(".html")) pages.push(path.join(web, f));
}
pages.sort();
assert.ok(pages.length >= 16, "expected the hosted app pages, found " + pages.length);
const declared = {};
for (const p of pages) {
  const rel = path.relative(root, p);
  const src = fs.readFileSync(p, "utf8");
  const cat = meta(src, "espp-category");
  assert.ok(cat !== null, rel + ": missing <meta name=\"espp-category\">");
  assert.ok(CATEGORIES.includes(cat), rel + ": unknown category " + cat);
  const transports = (meta(src, "espp-transports") || "").split(/\s+/).filter(Boolean);
  assert.ok(transports.length > 0, rel + ": no espp-transports");
  for (const t of transports) assert.ok(TRANSPORTS.includes(t), rel + ": unknown transport " + t);
  const protocols = parseProtocols(meta(src, "espp-protocols") || "");
  // a page that declares an espp protocol must mention that id in its script
  // (its own PROTOCOL constant) — and vice versa for the known id constants
  for (const pr of protocols) assert.ok(src.includes('"' + pr.id + '"'), rel + ": declares " + pr.id + " but its script never mentions it");
  const idsInScript = new Set();
  // constants are declared as `<PREFIX_>PROTOCOL = "id", <PREFIX_>PROTOCOL_VERSION = n`
  // with the same (possibly empty) prefix, e.g. OTA_PROTOCOL / OTA_PROTOCOL_VERSION
  // or the bare PROTOCOL / PROTOCOL_VERSION pair
  for (const m of src.matchAll(/\b[A-Z0-9_]*PROTOCOL\s*=\s*"(espp\.[a-z0-9.-]+)"/g)) idsInScript.add(m[1]);
  for (const id of idsInScript) assert.ok(protocols.some((p) => p.id === id), rel + ": script defines protocol " + id + " not declared in espp-protocols");
  // and the version it implements: every declared protocol must have its pair
  for (const pr of protocols) {
    const escaped = pr.id.replace(/[.*+?^${}()|[\]\\]/g, "\\$&"); // every regexp metacharacter, backslash included
    const vm = new RegExp('\\b([A-Z0-9_]*)PROTOCOL\\s*=\\s*"' + escaped + '"[\\s\\S]{0,200}?\\b\\1PROTOCOL_VERSION\\s*=\\s*(\\d+)').exec(src);
    assert.ok(vm, rel + ": no <PREFIX_>PROTOCOL / <PREFIX_>PROTOCOL_VERSION constant pair found for " + pr.id);
    assert.strictEqual(Number(vm[2]), pr.version, rel + ": " + pr.id + " version differs between the meta tag and the script");
  }
  declared[path.basename(p)] = { category: cat, protocols, transports };
}
console.log("PASS metadata on " + pages.length + " app pages");
assert.strictEqual(declared["can_bridge_console.html"].protocols[0].id, "espp.can-bridge");
assert.strictEqual(declared["ds402_panel.html"].protocols[0].id, "espp.can-bridge");
assert.deepStrictEqual(declared["system_console.html"].protocols.map((p) => [p.id, p.optional]),
  [["espp.system", false], ["espp.monitor", true]]);
assert.deepStrictEqual(declared["coredump_console.html"].protocols.map((p) => [p.id, p.optional]),
  [["espp.coredump", false], ["espp.coredump-crash-trigger", true]]);

// ---- 2. the generator accepts the real pages and emits the three outputs
const tmp = fs.mkdtempSync(path.join(os.tmpdir(), "espp-apps-"));
const apps = path.join(tmp, "apps");
fs.mkdirSync(apps);
for (const p of pages) fs.copyFileSync(p, path.join(apps, path.basename(p))); // copyFileSync follows symlinks
const gen = path.join(root, "doc", "generate_apps_index.py");
function runGen(dir) { return spawnSync("python3", [gen, dir], { encoding: "utf8" }); }
let r = runGen(apps);
assert.strictEqual(r.status, 0, "generator failed:\n" + r.stderr);
const indexHtml = fs.readFileSync(path.join(apps, "index.html"), "utf8");
const registryJs = fs.readFileSync(path.join(apps, "registry.js"), "utf8");
const registryJson = JSON.parse(fs.readFileSync(path.join(apps, "registry.json"), "utf8"));
// registry.js evaluates to the same object as registry.json
const win = {};
new Function("window", registryJs)(win);
assert.deepStrictEqual(win.ESPP_APPS, registryJson);
assert.strictEqual(registryJson.apps.length, pages.length);
for (const a of registryJson.apps) {
  const d = declared[a.file];
  assert.ok(d, "registry lists unknown page " + a.file);
  assert.strictEqual(a.category, d.category);
  assert.deepStrictEqual(a.protocols, d.protocols);
  assert.deepStrictEqual(a.transports, d.transports);
  assert.ok(a.title && a.description, a.file + ": title/description missing from the registry");
  // the description is taken whole: an apostrophe inside the double-quoted value does not end it
  assert.strictEqual(a.description, meta(fs.readFileSync(path.join(apps, a.file), "utf8"), "description"), a.file + ": description differs from the page's meta tag");
  // listed exactly once on the landing page, inside the right section
  // anchors carrying both class="card" and this file's href, in any attribute order (the title's Device Hub button is not a card)
  const cards = (indexHtml.match(/<a [^>]*>/g) || []).filter((tag) => /\sclass="card"/.test(tag) && tag.includes('href="' + a.file + '"')).length;
  assert.strictEqual(cards, 1, a.file + " card count " + cards);
  const sec = new RegExp('<section class="group" data-category="' + a.category + '">[\\s\\S]*?</section>').exec(indexHtml);
  assert.ok(sec && sec[0].includes('href="' + a.file + '"'), a.file + " not in its category section");
}
{
  const ota = registryJson.apps.find((a) => a.file === "ota_console.html");
  assert.ok(ota.description.includes("component's") && ota.description.endsWith("rollback-aware finish."), "ota_console.html description truncated at the apostrophe: " + ota.description);
}
// sections in the fixed category order, with counts
const order = [...indexHtml.matchAll(/<section class="group" data-category="([^"]+)">/g)].map((m) => m[1]);
assert.deepStrictEqual(order, CATEGORIES.filter((c) => registryJson.apps.some((a) => a.category === c)));
assert.ok(indexHtml.includes('<meta name="espp-category"') === false, "index.html must not advertise itself as an app");
assert.ok(/<a class="hub-link" href="dispatcher_hub.html"[^>]*>Device Hub/.test(indexHtml), "index.html must link to the Device Hub near its title");
// the landing page's inline script is valid JS (no external resources)
// (plain string search, not a regexp: this locates the one inline script, it does not filter HTML)
const scriptOpen = indexHtml.indexOf("<script>");
const scriptClose = indexHtml.lastIndexOf("</script>");
assert.ok(scriptOpen >= 0 && scriptClose > scriptOpen, "index.html has no inline script");
assert.strictEqual(indexHtml.toLowerCase().split("<script").length - 1, 1, "index.html must have exactly one script");
new Function(indexHtml.slice(scriptOpen + "<script>".length, scriptClose));
assert.ok(!/<(script|link)[^>]+(src|href)=["']https?:/.test(indexHtml), "index.html references an external resource");
console.log("PASS generator: index.html + registry.js + registry.json for " + registryJson.apps.length + " apps");

// ---- 3. the generator rejects bad metadata with a clear message
function expectFailure(name, mutate, needle) {
  const dir = path.join(tmp, name);
  fs.mkdirSync(dir);
  fs.writeFileSync(path.join(dir, "ok.html"), '<title>ok</title><meta name="description" content="d"><meta name="espp-category" content="utilities"><meta name="espp-transports" content="webusb">');
  fs.writeFileSync(path.join(dir, "bad.html"), mutate);
  const res = runGen(dir);
  assert.notStrictEqual(res.status, 0, name + ": generator should fail");
  assert.ok(res.stderr.includes("bad.html") && res.stderr.includes(needle), name + ": unexpected message:\n" + res.stderr);
  assert.ok(!fs.existsSync(path.join(dir, "index.html")), name + ": wrote index.html despite the error");
}
expectFailure("missing-category", '<title>x</title><meta name="espp-transports" content="webusb">', "missing");
expectFailure("unknown-category", '<title>x</title><meta name="espp-category" content="gadgets">', "unknown espp-category");
expectFailure("bad-protocol", '<title>x</title><meta name="espp-category" content="utilities"><meta name="espp-protocols" content="espp.ota">', "malformed espp-protocols");
expectFailure("bad-transport", '<title>x</title><meta name="espp-category" content="utilities"><meta name="espp-transports" content="bluetooth">', "unknown espp-transports");
console.log("PASS generator rejects a missing/unknown category, a malformed protocol entry, an unknown transport");

// ---- 4. the hub's registry matching
const hubSrc = fs.readFileSync(path.join(root, "components/dispatcher/web/dispatcher_hub.html"), "utf8");
assert.ok(hubSrc.includes('<script src="registry.js"></script>'), "hub does not load registry.js");
const safe = /    function safeAppName\(app\) \{[\s\S]*?\n    \}\n/.exec(hubSrc);
assert.ok(safe, "hub: no safeAppName");
const begin = hubSrc.indexOf("    // --- begin app matching");
const end = hubSrc.indexOf("    // --- end app matching ---");
assert.ok(begin >= 0 && end > begin, "hub: app matching block markers missing");
const hub = new Function(safe[0] + hubSrc.slice(begin, end) + "\n return { registryApps, moduleForProtocol, appNotes, appAnchor, appsForDevice, appsForModule };")();
const reg = registryJson;
const mod = (id, protocol, protocolVersion, app) => ({ id, name: protocol || "m" + id, app: app || "", desc: "", protocol: protocol || "", protocolVersion: protocolVersion || 0 });
const files = (entries) => entries.map((e) => e.app.file);

// CAN bridge device: the advertised app is the bridge console, ds402 is "also works with"
let modules = [mod(5, "espp.can-bridge", 1, "can_bridge_console.html")];
let also = hub.appsForModule(reg, modules[0], modules);
assert.deepStrictEqual(files(also), ["ds402_panel.html"]);
assert.deepStrictEqual(also[0].notes, []);
assert.strictEqual(also[0].module.id, 5);
let dev = hub.appsForDevice(reg, modules);
assert.deepStrictEqual(files(dev).sort(), ["can_bridge_console.html", "ds402_panel.html"]);
assert.ok(dev.every((e) => e.module.id === 5));

// a full USB example: ota + coredump + crash trigger + system + monitor
modules = [mod(0, "espp.ota", 1, "ota_console.html"), mod(4, "espp.coredump", 1, "coredump_console.html"),
  mod(1, "espp.coredump-crash-trigger", 1), mod(7, "espp.system", 1, "system_console.html"), mod(8, "espp.monitor", 1)];
dev = hub.appsForDevice(reg, modules);
assert.deepStrictEqual(files(dev).sort(), ["coredump_console.html", "ota_console.html", "system_console.html"]);
for (const e of dev) assert.deepStrictEqual(e.notes, [], e.app.file + " unexpected notes");
assert.strictEqual(dev.find((e) => e.app.file === "system_console.html").module.id, 7, "linked with the required protocol's module");
// the same protocol on two modules: each pane links its own module (?module=N
// disambiguates), while the global list keeps the first advertised one
modules = [mod(5, "espp.can-bridge", 1, "can_bridge_console.html"), mod(6, "espp.can-bridge", 1, "can_bridge_console.html")];
assert.strictEqual(hub.appsForModule(reg, modules[0], modules)[0].module.id, 5);
assert.strictEqual(hub.appsForModule(reg, modules[1], modules)[0].module.id, 6);
assert.ok(hub.appsForDevice(reg, modules).every((e) => e.module.id === 5));
modules = [mod(0, "espp.ota", 1, "ota_console.html"), mod(4, "espp.coredump", 1, "coredump_console.html"),
  mod(1, "espp.coredump-crash-trigger", 1), mod(7, "espp.system", 1, "system_console.html"), mod(8, "espp.monitor", 1)];
// the crash trigger module advertises no app; its only matching app is the coredump
// console, and the link is anchored to the coredump module (the console applies
// ?module to espp.coredump only), not to the crash trigger
also = hub.appsForModule(reg, modules[2], modules);
assert.deepStrictEqual(files(also), ["coredump_console.html"]);
assert.strictEqual(also[0].module.id, 4, "anchored to the module speaking the app's required protocol");
assert.strictEqual(hub.appAnchor(reg.apps.find((a) => a.file === "coredump_console.html"), modules).id, 4);
// without a coredump module the crash trigger alone still lists the console, anchored to itself, with a warning
also = hub.appsForModule(reg, modules[2], [modules[2]]);
assert.deepStrictEqual(also.map((e) => [e.app.file, e.module.id, e.notes]),
  [["coredump_console.html", 1, [{ text: "needs espp.coredump, not advertised", warn: true }]]]);
// no module advertises "espp.monitor" here: system console still listed (monitor optional), with a note
modules = [mod(7, "espp.system", 1, "system_console.html")];
dev = hub.appsForDevice(reg, modules);
assert.deepStrictEqual(files(dev), ["system_console.html"]);
assert.deepStrictEqual(dev[0].notes, [{ text: "optional espp.monitor not advertised", warn: false }]);
// but a monitor-only device does not get the system console (espp.system required)
modules = [mod(8, "espp.monitor", 1)];
assert.deepStrictEqual(hub.appsForDevice(reg, modules), []);
assert.deepStrictEqual(hub.appsForModule(reg, modules[0], modules).map((e) => [e.app.file, e.notes]),
  [["system_console.html", [{ text: "needs espp.system, not advertised", warn: true }]]]);

// version mismatch: still linked, with a warning note
modules = [mod(5, "espp.can-bridge", 2, "can_bridge_console.html")];
dev = hub.appsForDevice(reg, modules);
assert.deepStrictEqual(files(dev).sort(), ["can_bridge_console.html", "ds402_panel.html"]);
for (const e of dev) assert.deepStrictEqual(e.notes, [{ text: "app speaks espp.can-bridge v1, device advertises v2", warn: true }]);
// an advertised version 0 means "unspecified": no mismatch note
modules = [mod(5, "espp.can-bridge", 0, "can_bridge_console.html")];
for (const e of hub.appsForDevice(reg, modules)) assert.deepStrictEqual(e.notes, [], e.app.file + ": v0 must not warn");

// unknown protocol / v1 payload (no protocol ids): nothing matches, the advertised app alone is linked by the hub
modules = [mod(9, "acme.widget", 1, "widget.html")];
assert.deepStrictEqual(hub.appsForDevice(reg, modules), []);
assert.deepStrictEqual(hub.appsForModule(reg, modules[0], modules), []);
modules = [mod(0, "", 0, "ota_console.html")];
assert.deepStrictEqual(hub.appsForDevice(reg, modules), []);
assert.deepStrictEqual(hub.appsForModule(reg, modules[0], modules), []);
// the primary app is never repeated in "also works with"
modules = [mod(5, "espp.can-bridge", 1, "ds402_panel.html")];
assert.deepStrictEqual(files(hub.appsForModule(reg, modules[0], modules)), ["can_bridge_console.html"]);

// no registry loaded (file://, or a copy of the hub on its own)
modules = [mod(5, "espp.can-bridge", 1, "can_bridge_console.html")];
assert.deepStrictEqual(hub.appsForDevice(undefined, modules), []);
assert.deepStrictEqual(hub.appsForModule(undefined, modules[0], modules), []);
assert.deepStrictEqual(hub.appsForDevice({ apps: "nope" }, modules), []);
// registry entries with an unsafe file name are never linked
const evil = { apps: [{ file: "javascript:alert(1)", title: "x", protocols: [{ id: "espp.can-bridge", version: 1, optional: false }] },
  { file: "../x.html", title: "x", protocols: [{ id: "espp.can-bridge", version: 1, optional: false }] }] };
assert.deepStrictEqual(hub.appsForDevice(evil, modules), []);
console.log("PASS hub registry matching (also-works-with, apps-for-device, optional/missing/version notes, fallbacks)");

fs.rmSync(tmp, { recursive: true, force: true });
console.log("ALL PASS");

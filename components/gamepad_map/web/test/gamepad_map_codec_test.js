#!/usr/bin/env node
// Unit tests for the gamepad map console's pure logic: the block of
// gamepad_map_console.html between `// --- begin gamepad map codec` and
// `// --- end gamepad map codec ---` (the espp.gamepad_map v1 wire codec,
// local map validation, rule descriptions, presets and the learn detector).
// Device -> host decoders follow the protocol's compatibility rule: a
// truncated message fails, bytes after the known fields are ignored.
// The byte vectors are derived by hand from
// components/gamepad_map/include/detail/gamepad_map_protocol.hpp (layouts) and
// gamepad_map.hpp (GamepadMap's builder helpers). Dependency-free:
//
//   node components/gamepad_map/web/test/gamepad_map_codec_test.js
"use strict";
const fs = require("fs");
const path = require("path");
const assert = require("assert");

const html = fs.readFileSync(path.join(__dirname, "..", "gamepad_map_console.html"), "utf8");
// (plain string search: the markers delimit the one codec block)
const begin = html.indexOf("    // --- begin gamepad map codec");
const end = html.indexOf("    // --- end gamepad map codec ---");
assert.ok(begin > 0 && end > begin, "codec block markers not found in gamepad_map_console.html");
const block = html.slice(begin, end);
assert.ok(!/\b(document|window|els|navigator|location)\b/.test(block.replace(/\/\/.*$/gm, "")), "the codec block must not touch the DOM / page state");
const G = new Function(block + `
  return { T_GET_INFO, T_GET_MAP, T_SET_MAP, T_ERASE_PROFILE, T_LIST_PROFILES, T_SET_STREAM, T_RESET_ALL,
           T_INFO, T_MAP, T_OK, T_ERROR, T_PROFILES, T_INPUT, BUTTON_NAMES, AXIS_NAMES, BUTTON_COUNT, AXIS_COUNT, B, A, K,
           RULE_NONE, KEEP_SOURCE, AXIS_MAX, STATE_SIZE, RULE_SIZE, PRESETS,
           buttonLabel, axisLabel, percentToParam, paramToPercent, GmReader, GmWriter, readMap, readState, encodeState,
           encodeGetMap, encodeSetMap, encodeEraseProfile, encodeSetStream,
           decodeInfo, decodeMapReply, decodeProfiles, decodeInput, decodeOk, decodeError,
           describeTarget, sameTarget, validateTarget, gmLimits, validateRule, validateMap,
           ruleButton, ruleAxisToButton, ruleButtonToAxis, ruleAxis, changeRuleKind, describeRule,
           learnBaseline, learnStep, applyLearnHit };
`)();

// The whole inline script must parse (a syntax error anywhere breaks the page).
const scriptOpen = html.indexOf("<script>");
const scriptClose = html.lastIndexOf("</script>");
assert.ok(scriptOpen >= 0 && scriptClose > scriptOpen, "no inline <script> in the page");
new Function(html.slice(scriptOpen + "<script>".length, scriptClose));

let passed = 0;
function test(name, fn) { fn(); passed++; console.log("PASS " + name); }
const bytes = (...xs) => new Uint8Array(xs.flat());
const str = (s) => { const b = new TextEncoder().encode(s); return [b.length, ...b]; };
const eq = (a, b, msg) => assert.deepStrictEqual(Array.from(a), Array.from(b), msg);
// every strict prefix of a valid payload is rejected; trailing bytes after the
// known fields are ignored (the compatibility rule for device -> host messages)
function rejectsTruncation(decode, p, name) {
  for (let n = 0; n < p.length; n++) assert.strictEqual(decode(p.subarray(0, n)), null, name + ": prefix of " + n + " bytes accepted");
  const whole = decode(p);
  assert.notStrictEqual(whole, null, name + ": the payload itself is rejected");
  assert.deepStrictEqual(decode(bytes([...p, 0])), whole, name + ": a trailing byte changed the result");
  assert.deepStrictEqual(decode(bytes([...p, 0xAA, 0xBB, 0xCC])), whole, name + ": trailing bytes changed the result");
}

test("protocol constants mirror gamepad_map_protocol.hpp / gamepad_state.hpp", () => {
  assert.deepStrictEqual([G.T_GET_INFO, G.T_GET_MAP, G.T_SET_MAP, G.T_ERASE_PROFILE, G.T_LIST_PROFILES, G.T_SET_STREAM, G.T_RESET_ALL], [1, 2, 3, 4, 5, 6, 7]);
  assert.deepStrictEqual([G.T_INFO, G.T_MAP, G.T_OK, G.T_ERROR, G.T_PROFILES, G.T_INPUT], [0x81, 0x82, 0x83, 0x84, 0x85, 0x86]);
  assert.strictEqual(G.BUTTON_COUNT, 22); assert.strictEqual(G.AXIS_COUNT, 6);
  assert.strictEqual(G.B.SOUTH, 0); assert.strictEqual(G.B.CAPTURE, 13); assert.strictEqual(G.B.UP, 14); assert.strictEqual(G.B.RIGHT, 17); assert.strictEqual(G.B.PADDLE4, 21);
  assert.strictEqual(G.A.LEFT_X, 0); assert.strictEqual(G.A.RIGHT_Y, 3); assert.strictEqual(G.A.RIGHT_TRIGGER, 5);
  assert.deepStrictEqual([G.RULE_NONE, G.KEEP_SOURCE, G.AXIS_MAX, G.STATE_SIZE, G.RULE_SIZE], [0xFF, 0x01, 32767, 16, 6]);
  assert.strictEqual(G.buttonLabel(G.B.SOUTH), "South (Xbox A / Nintendo B / PS Cross)");
  assert.strictEqual(G.buttonLabel(G.B.EAST), "East (Xbox B / Nintendo A / PS Circle)");
  assert.strictEqual(G.buttonLabel(G.B.LEFT), "D-pad Left");
  assert.strictEqual(G.axisLabel(G.A.LEFT_Y), "Left stick Y (- up / + down)");
});

// A map: dead zones 10 / 20 %, East presses South; Left stick X <= -50 % presses
// d-pad Left (keep source). 3 header bytes + 6 per rule.
const MAP = { leftDeadzone: 10, rightDeadzone: 20, rules: [
  { kind: 0, source: 1, target: 0, flags: 0, param: 0 },
  { kind: 1, source: 0, target: 16, flags: 1, param: -16384 }] };
const MAP_BYTES = [10, 20, 2, /* rule 0 */ 0, 1, 0, 0, 0x00, 0x00, /* rule 1 */ 1, 0, 16, 1, 0x00, 0xC0];
const XBOX = { profile: true, vid: 0x045e, pid: 0x0b13, name: "Xbox" };
const XBOX_TARGET = [1, 0x5e, 0x04, 0x13, 0x0b, ...str("Xbox")];

test("GET_MAP / SET_MAP / MAP: target + map layout, round trip", () => {
  // MAP = [target][flags u8 (bit0 = exists)][map]
  eq(G.encodeGetMap({ profile: false }), [0]);
  eq(G.encodeGetMap(XBOX), XBOX_TARGET);
  eq(G.encodeSetMap({ profile: false }, MAP), [0, ...MAP_BYTES]);
  eq(G.encodeSetMap(XBOX, MAP), [...XBOX_TARGET, ...MAP_BYTES]);
  assert.deepStrictEqual(G.decodeMapReply(bytes([0, 1, ...MAP_BYTES])), { target: { profile: false }, exists: true, map: MAP });
  assert.deepStrictEqual(G.decodeMapReply(bytes([...XBOX_TARGET, 1, ...MAP_BYTES])), { target: XBOX, exists: true, map: MAP });
  // a missing profile: the target as requested, exists 0, the default map
  assert.deepStrictEqual(G.decodeMapReply(bytes([...XBOX_TARGET, 0, 0, 0, 0])), { target: XBOX, exists: false, map: { leftDeadzone: 0, rightDeadzone: 0, rules: [] } });
  // other flag bits carry nothing in v1
  assert.strictEqual(G.decodeMapReply(bytes([0, 0xFE, 0, 0, 0])).exists, false);
  rejectsTruncation(G.decodeMapReply, bytes([...XBOX_TARGET, 1, ...MAP_BYTES]), "MAP");
  rejectsTruncation(G.decodeMapReply, bytes([0, 0, 0, 0, 0]), "MAP (default, empty)");
  assert.strictEqual(G.decodeMapReply(bytes([2, 1, ...MAP_BYTES])), null, "target kind 2 is not a target");
  // param is a signed i16 (both extremes)
  const ext = { leftDeadzone: 0, rightDeadzone: 0, rules: [{ kind: 2, source: 14, target: 1, flags: 1, param: -32767 }, { kind: 3, source: 2, target: 3, flags: 0, param: 400 }] };
  const eb = G.encodeSetMap({ profile: false }, ext);
  eq(eb, [0, 0, 0, 2, 2, 14, 1, 1, 0x01, 0x80, 3, 2, 3, 0, 0x90, 0x01]);
  assert.deepStrictEqual(G.decodeMapReply(bytes([0, 1, ...eb.subarray(1)])).map, ext);
  // a name-only profile (VID and PID 0) and a UTF-8 name
  const pad = { profile: true, vid: 0, pid: 0, name: "Pad é" };
  eq(G.encodeGetMap(pad), [1, 0, 0, 0, 0, 6, 0x50, 0x61, 0x64, 0x20, 0xC3, 0xA9]);
  assert.deepStrictEqual(G.decodeMapReply(bytes([...G.encodeGetMap(pad), 1, 0, 0, 0])).target, pad);
});

test("ERASE_PROFILE / SET_STREAM / GET_INFO etc. request payloads", () => {
  eq(G.encodeEraseProfile(XBOX), [0x5e, 0x04, 0x13, 0x0b, ...str("Xbox")]);
  eq(G.encodeSetStream(true, 50), [1, 50, 0]);
  eq(G.encodeSetStream(false, 0x1234), [0, 0x34, 0x12]);
  // a name longer than 255 bytes is truncated like put_str()
  const long = G.encodeEraseProfile({ vid: 1, pid: 2, name: "x".repeat(300) });
  assert.strictEqual(long.length, 4 + 1 + 255); assert.strictEqual(long[4], 255);
});

test("INFO with and without a controller (reply or unsolicited: same layout)", () => {
  const none = bytes([22, 6, 64, 16, 3, 0]);
  assert.deepStrictEqual(G.decodeInfo(none), { buttonCount: 22, axisCount: 6, maxRules: 64, maxProfiles: 16, profileCount: 3, controller: null });
  rejectsTruncation(G.decodeInfo, none, "INFO (no controller)");
  const withCtl = bytes([22, 6, 64, 16, 3, 1, 0x5e, 0x04, 0x13, 0x0b, ...str("Xbox"), 1]);
  assert.deepStrictEqual(G.decodeInfo(withCtl), { buttonCount: 22, axisCount: 6, maxRules: 64, maxProfiles: 16, profileCount: 3,
    controller: { vid: 0x045e, pid: 0x0b13, name: "Xbox", hasProfile: true } });
  rejectsTruncation(G.decodeInfo, withCtl, "INFO (controller)");
  assert.strictEqual(G.decodeInfo(bytes([22, 6, 64, 16, 0, 1, 0, 0, 0, 0, 0, 0])).controller.hasProfile, false);
  // other flag bits carry nothing in v1
  assert.strictEqual(G.decodeInfo(bytes([22, 6, 64, 16, 3, 0x80])).controller, null);
});

test("PROFILES", () => {
  const p = bytes([2, 0x5e, 0x04, 0x13, 0x0b, ...str("Xbox"), 4, 0, 0, 0, 0, ...str("Pad"), 0]);
  assert.deepStrictEqual(G.decodeProfiles(p), [{ vid: 0x045e, pid: 0x0b13, name: "Xbox", ruleCount: 4 }, { vid: 0, pid: 0, name: "Pad", ruleCount: 0 }]);
  assert.deepStrictEqual(G.decodeProfiles(bytes([0])), []);
  rejectsTruncation(G.decodeProfiles, p, "PROFILES");
});

test("INPUT: flags + raw state + mapped state (16 bytes each)", () => {
  // raw: South + East pressed, left stick full left / full down, right Y 0x1234, right trigger 50 %
  const raw = [0x03, 0, 0, 0, 0x01, 0x80, 0xFF, 0x7F, 0, 0, 0x34, 0x12, 0, 0, 0x00, 0x40];
  // mapped: d-pad Left (bit 16) + Paddle4 (bit 21) + bit 31 (u32, not negative), axes 0
  const mapped = [0, 0, 0x21, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
  const p = bytes([3, ...raw, ...mapped]);
  assert.strictEqual(p.length, 33);
  const d = G.decodeInput(p);
  assert.deepStrictEqual(d, { connected: true, changed: true,
    raw: { buttons: 3, axes: [-32767, 32767, 0, 0x1234, 0, 16384] },
    mapped: { buttons: ((1 << 16) | (1 << 21) | 0x80000000) >>> 0, axes: [0, 0, 0, 0, 0, 0] } });
  assert.deepStrictEqual([G.decodeInput(bytes([1, ...raw, ...mapped])).connected, G.decodeInput(bytes([1, ...raw, ...mapped])).changed], [true, false]);
  assert.deepStrictEqual([G.decodeInput(bytes([2, ...raw, ...mapped])).connected, G.decodeInput(bytes([2, ...raw, ...mapped])).changed], [false, true]);
  rejectsTruncation(G.decodeInput, p, "INPUT");
  eq(G.encodeState(d.raw), raw); // the state layout round-trips
});

test("OK / ERROR", () => {
  assert.deepStrictEqual(G.decodeOk(bytes([3])), { requestType: 3 });
  assert.deepStrictEqual(G.decodeOk(bytes([3, 9, 9])), { requestType: 3 }, "OK ignores trailing bytes");
  assert.strictEqual(G.decodeOk(bytes([])), null);
  const e = G.decodeError(bytes([3, 22, 0, 0, 0, ...new TextEncoder().encode("invalid map: rule 0: source out of range")]));
  assert.deepStrictEqual(e, { requestType: 3, code: 22, message: "invalid map: rule 0: source out of range" });
  assert.deepStrictEqual(G.decodeError(bytes([4, 0x02, 0x01, 0, 0])), { requestType: 4, code: 0x102, message: "" });
  assert.strictEqual(G.decodeError(bytes([3, 22, 0, 0])), null);
  for (let n = 0; n < 5; n++) assert.strictEqual(G.decodeError(bytes([3, 22, 0, 0, 0]).subarray(0, n)), null, "ERROR prefix " + n);
  // (the message runs to the end of the payload: there is no "after" for ERROR)
});

test("percent <-> param (thresholds / button -> axis values)", () => {
  assert.strictEqual(G.percentToParam(50), 16384);   // GamepadMap::left_stick_to_dpad default
  assert.strictEqual(G.percentToParam(-50), -16384);
  assert.strictEqual(G.percentToParam(40), 13107);   // GamepadMap::triggers_to_buttons default
  assert.strictEqual(G.percentToParam(100), 32767);
  assert.strictEqual(G.percentToParam(-100), -32767);
  assert.strictEqual(G.percentToParam(250), 32767);
  assert.strictEqual(G.percentToParam(0), 0);
  assert.strictEqual(G.percentToParam("x"), 0);
  assert.strictEqual(G.paramToPercent(16384), 50);
  assert.strictEqual(G.paramToPercent(13107), 40);
  assert.strictEqual(G.paramToPercent(-32767), -100);
  for (let pct = -100; pct <= 100; pct++) assert.strictEqual(G.paramToPercent(G.percentToParam(pct)), pct);
});

test("local validation mirrors GamepadMap::valid() / GamepadMapRule::valid()", () => {
  const lim = G.gmLimits({ maxRules: 4, buttonCount: 22, axisCount: 6 });
  assert.deepStrictEqual(G.validateMap(MAP, lim), []);
  assert.deepStrictEqual(G.validateMap(MAP), []); // default limits = the C++ constants
  assert.deepStrictEqual(G.gmLimits(null), { maxRules: 64, buttonCount: 22, axisCount: 6 });
  const r = (o) => ({ kind: 0, source: 0, target: 0, flags: 0, param: 0, ...o });
  const why = (o) => G.validateRule(r(o), lim);
  assert.strictEqual(why({}), null);
  assert.strictEqual(why({ kind: 4 }), "unknown rule kind");
  assert.strictEqual(why({ flags: 2 }), "unknown flags");
  assert.strictEqual(why({ source: 22 }), "source out of range");
  assert.strictEqual(why({ kind: 1, source: 6, param: 1 }), "source out of range");
  assert.strictEqual(why({ kind: 1, source: 5, param: 1 }), null);
  // a trigger never goes below 0: a negative axis -> button threshold on one is refused
  assert.strictEqual(why({ kind: 1, source: 4, param: -100 }), "a trigger never goes below 0: its threshold must be positive");
  assert.strictEqual(why({ kind: 1, source: 5, param: -1 }), "a trigger never goes below 0: its threshold must be positive");
  assert.strictEqual(why({ kind: 1, source: 3, param: -16384 }), null, "a stick may go below 0");
  // keep source + no target does nothing: refused
  assert.strictEqual(why({ target: 0xFF, flags: 1 }), "a rule that keeps its source needs a target");
  assert.strictEqual(why({ kind: 3, target: 0xFF, flags: 1, param: 100 }), "a rule that keeps its source needs a target");
  assert.strictEqual(why({ target: 22 }), "target out of range");
  assert.strictEqual(why({ target: 0xFF }), null); // NONE
  assert.strictEqual(why({ kind: 2, target: 6, param: 1 }), "target out of range");
  assert.strictEqual(why({ kind: 1, param: 0 }), "axis -> button needs a non-zero threshold");
  assert.strictEqual(why({ kind: 2, param: 0 }), null); // button -> axis 0 is pointless but valid
  assert.strictEqual(why({ kind: 3, param: 401 }), "axis -> axis scale out of range");
  assert.strictEqual(why({ kind: 3, param: -400 }), null);
  // map level: dead zones, rule count, every bad rule listed with its index
  const m = { leftDeadzone: 91, rightDeadzone: 0, rules: [r({}), r({ kind: 1 }), r({}), r({}), r({ source: 30 })] };
  assert.deepStrictEqual(G.validateMap(m, lim), [
    { rule: -1, why: "dead zone out of range" },
    { rule: -1, why: "too many rules (5; the device takes at most 4)" },
    { rule: 1, why: "axis -> button needs a non-zero threshold" },
    { rule: 4, why: "source out of range" }]);
  assert.deepStrictEqual(G.validateMap({ leftDeadzone: 0, rightDeadzone: 90, rules: [] }, lim), []);
  // targets
  assert.strictEqual(G.validateTarget({ profile: false }), null);
  assert.strictEqual(G.validateTarget(XBOX), null);
  assert.ok(/needs a name/.test(G.validateTarget({ profile: true, vid: 0, pid: 0, name: "" })));
  assert.strictEqual(G.validateTarget({ profile: true, vid: 0, pid: 0, name: "Pad" }), null);
  assert.ok(/255 bytes/.test(G.validateTarget({ profile: true, vid: 1, pid: 1, name: "x".repeat(256) })));
});

test("targets: description + GamepadProfile::matches() semantics", () => {
  assert.strictEqual(G.describeTarget({ profile: false }), "default map");
  assert.strictEqual(G.describeTarget(XBOX), "profile 045e:0b13 “Xbox”");
  assert.strictEqual(G.describeTarget({ profile: true, vid: 0, pid: 0, name: "Pad" }), "profile “Pad” (matched by name)");
  assert.ok(G.sameTarget(XBOX, { ...XBOX, name: "Xbox Wireless Controller" }), "VID:PID match ignores the name");
  assert.ok(!G.sameTarget(XBOX, { ...XBOX, pid: 0x0b12 }));
  assert.ok(G.sameTarget({ profile: true, vid: 0, pid: 0, name: "Pad" }, { profile: true, vid: 0, pid: 0, name: "Pad" }));
  assert.ok(!G.sameTarget({ profile: true, vid: 0, pid: 0, name: "" }, { profile: true, vid: 0, pid: 0, name: "" }));
  assert.ok(G.sameTarget({ profile: false }, { profile: false }));
  assert.ok(!G.sameTarget({ profile: false }, XBOX));
});

test("rule descriptions", () => {
  const { B, A } = G;
  assert.strictEqual(G.describeRule(G.ruleButton(B.EAST, B.SOUTH)), "East presses South");
  assert.strictEqual(G.describeRule(G.ruleButton(B.EAST, B.PADDLE1, true)), "East presses Paddle 1 (East also stays)");
  assert.strictEqual(G.describeRule(G.ruleAxisToButton(A.LEFT_X, -16384, B.LEFT, true)), "Left stick X below -50% presses D-pad Left (Left stick X also stays)");
  assert.strictEqual(G.describeRule(G.ruleAxisToButton(A.LEFT_TRIGGER, 13107, B.L2)), "Left trigger above +40% presses L2");
  assert.strictEqual(G.describeRule(G.ruleButtonToAxis(B.UP, A.LEFT_Y, -32767)), "D-pad Up adds -100% to Left stick Y");
  assert.strictEqual(G.describeRule(G.ruleAxis(A.LEFT_Y, A.LEFT_Y, -100)), "Left stick Y is inverted");
  assert.strictEqual(G.describeRule(G.ruleAxis(A.LEFT_X, A.RIGHT_X)), "Left stick X drives Right stick X");
  assert.strictEqual(G.describeRule(G.ruleAxis(A.LEFT_X, A.RIGHT_Y, 50)), "Left stick X × 50% drives Right stick Y");
  assert.strictEqual(G.describeRule({ kind: 0, source: B.HOME, target: 0xFF, flags: 0, param: 0 }), "Home is removed (never pressed)");
  assert.strictEqual(G.describeRule({ kind: 3, source: A.RIGHT_TRIGGER, target: 0xFF, flags: 0, param: 100 }), "Right trigger is removed (reads released)");
  assert.ok(/invalid/.test(G.describeRule({ kind: 1, source: 0, target: 0, flags: 0, param: 0 })));
  assert.ok(/never goes below 0/.test(G.describeRule({ kind: 1, source: 4, target: 6, flags: 0, param: -13107 })));
  assert.ok(/invalid/.test(G.describeRule({ kind: 0, source: 0, target: 0xFF, flags: 1, param: 0 })));
});

test("changing a rule's kind", () => {
  const { K, B, A } = G;
  // button -> button to axis -> button: the source becomes an axis, the target stays a button
  let r = G.changeRuleKind(G.ruleButton(B.EAST, B.NORTH, true), K.AXIS_TO_BUTTON);
  assert.deepStrictEqual(r, { kind: 1, source: 0, target: B.NORTH, flags: 1, param: 16384 });
  r = G.changeRuleKind(r, K.AXIS_TO_AXIS);
  assert.deepStrictEqual(r, { kind: 3, source: 0, target: 0, flags: 1, param: 100 });
  r = G.changeRuleKind({ kind: 0, source: 3, target: 0xFF, flags: 0, param: 0 }, K.BUTTON_TO_AXIS);
  assert.deepStrictEqual(r, { kind: 2, source: 3, target: 0xFF, flags: 0, param: 32767 }, "a none target stays none");
  void A;
});

test("presets encode exactly like the GamepadMap builder helpers", () => {
  const P = Object.fromEntries(G.PRESETS.map((p) => [p.id, p.rules()]));
  const enc = (rules) => Array.from(G.encodeSetMap({ profile: false }, { leftDeadzone: 0, rightDeadzone: 0, rules })).slice(4);
  // swap(South, East): button(South, East) + button(East, South)
  assert.deepStrictEqual(enc(P.swapSouthEast), [0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0]);
  assert.deepStrictEqual(enc(P.swapWestNorth), [0, 2, 3, 0, 0, 0, 0, 3, 2, 0, 0, 0]);
  // invert(LeftY) = axis(LeftY, LeftY, -100): -100 = 0xFF9C
  assert.deepStrictEqual(enc(P.invertLeftY), [3, 1, 1, 0, 0x9C, 0xFF]);
  assert.deepStrictEqual(enc(P.invertRightY), [3, 3, 3, 0, 0x9C, 0xFF]);
  // swap_sticks(): swap(LeftX, RightX), swap(LeftY, RightY), scale 100 = 0x0064
  assert.deepStrictEqual(enc(P.swapSticks), [3, 0, 2, 0, 100, 0, 3, 2, 0, 0, 100, 0, 3, 1, 3, 0, 100, 0, 3, 3, 1, 0, 100, 0]);
  // left_stick_to_dpad(16384, keep): LX -16384 Left, LX +16384 Right, LY -16384 Up, LY +16384 Down
  assert.deepStrictEqual(enc(P.leftStickToDpad), [1, 0, 16, 1, 0x00, 0xC0, 1, 0, 17, 1, 0x00, 0x40, 1, 1, 14, 1, 0x00, 0xC0, 1, 1, 15, 1, 0x00, 0x40]);
  // dpad_to_left_stick(keep): +-32767 = 0x7FFF / 0x8001
  assert.deepStrictEqual(enc(P.dpadToLeftStick), [2, 16, 0, 1, 0x01, 0x80, 2, 17, 0, 1, 0xFF, 0x7F, 2, 14, 1, 1, 0x01, 0x80, 2, 15, 1, 1, 0xFF, 0x7F]);
  // triggers_to_buttons(13107, keep): 13107 = 0x3333
  assert.deepStrictEqual(enc(P.triggersToButtons), [1, 4, 6, 1, 0x33, 0x33, 1, 5, 7, 1, 0x33, 0x33]);
  // buttons_to_triggers(keep)
  assert.deepStrictEqual(enc(P.buttonsToTriggers), [2, 6, 4, 1, 0xFF, 0x7F, 2, 7, 5, 1, 0xFF, 0x7F]);
  for (const p of G.PRESETS) assert.deepStrictEqual(G.validateMap({ leftDeadzone: 0, rightDeadzone: 0, rules: p.rules() }), [], p.id + " is valid");
});

test("learn: the next control that changes on the raw input", () => {
  const { B, A, K } = G;
  const st = (buttons, axes = [0, 0, 0, 0, 0, 0]) => ({ buttons, axes });
  // East held when the learn starts: ignored until released and pressed again; North is new
  let base = G.learnBaseline(st(1 << B.EAST));
  let s = G.learnStep(base, st((1 << B.EAST) | (1 << B.NORTH)));
  assert.deepStrictEqual(s.hit, { type: "button", index: B.NORTH });
  s = G.learnStep(base, st(1 << B.EAST)); assert.strictEqual(s.hit, null);
  s = G.learnStep(s.base, st(0)); assert.strictEqual(s.hit, null); // released
  s = G.learnStep(s.base, st(1 << B.EAST)); assert.deepStrictEqual(s.hit, { type: "button", index: B.EAST });
  // bits past the button count are not buttons
  assert.strictEqual(G.learnStep(G.learnBaseline(st(0)), st(1 << 25)).hit, null);
  // axes: past 50 %, with the direction; small moves do not count
  base = G.learnBaseline(st(0));
  assert.strictEqual(G.learnStep(base, st(0, [16000, 0, 0, 0, 0, 0])).hit, null);
  assert.deepStrictEqual(G.learnStep(base, st(0, [-20000, 0, 0, 0, 0, 0])).hit, { type: "axis", index: A.LEFT_X, sign: -1 });
  assert.deepStrictEqual(G.learnStep(base, st(0, [0, 0, 0, 20000, 0, 0])).hit, { type: "axis", index: A.RIGHT_Y, sign: 1 });
  assert.deepStrictEqual(G.learnStep(base, st(0, [0, 0, 0, 0, 0, 30000])).hit, { type: "axis", index: A.RIGHT_TRIGGER, sign: 1 });
  // a stick already deflected at the start counts only after it came back (or flipped)
  base = G.learnBaseline(st(0, [30000, 0, 0, 0, 0, 0]));
  assert.strictEqual(G.learnStep(base, st(0, [31000, 0, 0, 0, 0, 0])).hit, null);
  assert.deepStrictEqual(G.learnStep(base, st(0, [-31000, 0, 0, 0, 0, 0])).hit, { type: "axis", index: A.LEFT_X, sign: -1 });
  s = G.learnStep(base, st(0)); assert.strictEqual(s.hit, null);
  assert.deepStrictEqual(G.learnStep(s.base, st(0, [30000, 0, 0, 0, 0, 0])).hit, { type: "axis", index: A.LEFT_X, sign: 1 });
  // applying a hit: the kind follows the learned control's type, the target keeps its type
  let r = G.applyLearnHit(G.ruleButton(B.SOUTH, B.LEFT), { type: "axis", index: A.LEFT_X, sign: -1 });
  assert.deepStrictEqual(r, { kind: K.AXIS_TO_BUTTON, source: A.LEFT_X, target: B.LEFT, flags: 0, param: -16384 });
  r = G.applyLearnHit(G.ruleAxisToButton(A.LEFT_X, -13107, B.LEFT), { type: "axis", index: A.LEFT_Y, sign: 1 });
  assert.deepStrictEqual(r, { kind: K.AXIS_TO_BUTTON, source: A.LEFT_Y, target: B.LEFT, flags: 0, param: 13107 }, "keeps the threshold's size, takes the direction");
  r = G.applyLearnHit(G.ruleAxisToButton(A.LEFT_X, -16384, B.LEFT), { type: "button", index: B.WEST });
  assert.deepStrictEqual(r, { kind: K.BUTTON_TO_BUTTON, source: B.WEST, target: B.LEFT, flags: 0, param: 0 });
  r = G.applyLearnHit(G.ruleButtonToAxis(B.UP, A.LEFT_Y, -32767, true), { type: "axis", index: A.RIGHT_Y, sign: -1 });
  assert.deepStrictEqual(r, { kind: K.AXIS_TO_AXIS, source: A.RIGHT_Y, target: A.LEFT_Y, flags: 1, param: 100 });
  r = G.applyLearnHit(G.ruleAxis(A.LEFT_X, A.RIGHT_X, -100), { type: "button", index: B.L2 });
  assert.deepStrictEqual(r, { kind: K.BUTTON_TO_AXIS, source: B.L2, target: A.RIGHT_X, flags: 0, param: 32767 });
});

console.log("ALL PASS (" + passed + " groups)");

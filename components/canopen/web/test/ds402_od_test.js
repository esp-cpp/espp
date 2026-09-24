#!/usr/bin/env node
// Unit tests for the DS402 panel's object-dictionary logic (the pure part of
// ds402_panel.html between the OD:BEGIN-PURE / OD:END-PURE markers): the EDS
// parser, the built-in CiA 301/402 table, value decoding, abort classification
// and the scan walk against a mocked SDO client. Dependency-free:
//
//   node components/canopen/web/test/ds402_od_test.js
"use strict";
const fs = require("fs");
const path = require("path");
const assert = require("assert");

const html = fs.readFileSync(path.join(__dirname, "..", "ds402_panel.html"), "utf8");
const begin = html.indexOf("// ==== OD:BEGIN-PURE ====");
const end = html.indexOf("// ==== OD:END-PURE ====");
assert(begin > 0 && end > begin, "pure-block markers not found in ds402_panel.html");
const src = html.slice(begin, end);
const od = new Function(src + `
  return { OD_DATATYPE, parseEds, builtinObjects, odDecodeValue, odClassifyError, odScan, odCounts, odToCsv, odKindToSelect, odTypeName };
`)();

// Also make sure the whole inline script parses (a syntax error anywhere in
// the page would break every panel, not just this one).
const scriptBody = /<script>([\s\S]*)<\/script>/.exec(html)[1];
new Function(scriptBody);   // throws SyntaxError on a bad script

let passed = 0;
function test(name, fn) {
  return Promise.resolve().then(fn).then(() => { passed++; console.log("ok   " + name); },
    (e) => { console.log("FAIL " + name + "\n     " + (e && e.stack ? e.stack : e)); process.exitCode = 1; });
}

// The SDO client's error shapes (see sdoAbortError / sdoSendAndWait in the page).
const abort = (code, name) => new Error("SDO abort 0x" + code.toString(16).padStart(8, "0") + " (" + name + ")");
const timeout = () => new Error("SDO timeout (no response from node 2)");

const EDS_SAMPLE = `
; a small EDS in the CiA 306 style
[FileInfo]
FileName=sample.eds
EDSVersion=4.0

[DeviceInfo]
VendorName=ACME Drives
ProductName=Servo 9000

[MandatoryObjects]
SupportedObjects=3
1=0x1000
2=0x1001
3=0x1018

[OptionalObjects]
SupportedObjects=3
1=0x1003
2=0x1600
3=0x6040

[ManufacturerObjects]
SupportedObjects=1
1=0x2001

[1000]
ParameterName=Device type
ObjectType=0x7
DataType=0x0007
AccessType=ro
DefaultValue=0x00020192
PDOMapping=0

[1001]
ParameterName=Error register
ObjectType=0x7
DataType=0x0005
AccessType=ro

[1018]
ParameterName=Identity object
ObjectType=0x9
SubNumber=3

[1018sub0]
ParameterName=Number of entries
ObjectType=0x7
DataType=0x0005
AccessType=ro
DefaultValue=2

[1018sub1]
ParameterName=Vendor-ID
ObjectType=0x7
DataType=0x0007
AccessType=ro
DefaultValue=0x0000abcd

[1018sub2]
ParameterName=Product code
ObjectType=0x7
DataType=0x0007
AccessType=ro

[1003]
ParameterName=Pre-defined error field
ObjectType=0x8
DataType=0x0007
AccessType=ro
CompactSubObj=4

[1003Name]
NrOfEntries=1
1=Newest error

[1600]
ParameterName=RPDO 1 mapping ; trailing comment
ObjectType=0x8
DataType=0x0007
AccessType=rw
CompactSubObj=8

[6040]
ParameterName=Controlword
ObjectType=0x7
DataType=0x0006
AccessType=rww
PDOMapping=1

[2001]
ParameterName=Secret knob
ObjectType=0x7
DataType=0x0006
AccessType=wo
`;

(async () => {
  await test("parseEds: object list, names, types, access", () => {
    const eds = od.parseEds(EDS_SAMPLE);
    assert.strictEqual(eds.edsVersion, "4.0");
    assert.strictEqual(eds.deviceInfo.productname, "Servo 9000");
    assert.deepStrictEqual(eds.objects.map((o) => o.index), [0x1000, 0x1001, 0x1003, 0x1018, 0x1600, 0x2001, 0x6040]);
    const dt = eds.objects.find((o) => o.index === 0x1000);
    assert.strictEqual(dt.name, "Device type");
    assert.strictEqual(dt.objectType, 7);
    assert.strictEqual(dt.dataType, 0x7);
    assert.strictEqual(dt.access, "ro");
    assert.strictEqual(dt.defaultValue, "0x00020192");
    const cw = eds.objects.find((o) => o.index === 0x6040);
    assert.strictEqual(cw.access, "rww");
    assert.strictEqual(cw.pdoMapping, true);
    assert.strictEqual(eds.warnings.length, 0);
  });
  await test("parseEds: RECORD with explicit sub sections", () => {
    const id = od.parseEds(EDS_SAMPLE).objects.find((o) => o.index === 0x1018);
    assert.strictEqual(id.objectType, 9);
    assert.strictEqual(id.subNumber, 3);
    assert.deepStrictEqual(id.subs.map((s) => s.sub), [0, 1, 2]);
    assert.strictEqual(id.subs[1].name, "Vendor-ID");
    assert.strictEqual(id.subs[1].dataType, 0x7);
    assert.strictEqual(id.subs[0].defaultValue, "2");
  });
  await test("parseEds: ARRAY with CompactSubObj and a Name section", () => {
    const ef = od.parseEds(EDS_SAMPLE).objects.find((o) => o.index === 0x1003);
    assert.strictEqual(ef.objectType, 8);
    assert.strictEqual(ef.compact, 4);
    assert.deepStrictEqual(ef.subs.map((s) => s.sub), [0, 1, 2, 3, 4]);
    assert.strictEqual(ef.subs[1].name, "Newest error");          // from [1003Name]
    assert.strictEqual(ef.subs[2].name, "Pre-defined error field 2");
    assert.strictEqual(ef.subs[2].dataType, 0x7);                  // inherited
    assert.strictEqual(ef.subs[2].access, "ro");
    const map = od.parseEds(EDS_SAMPLE).objects.find((o) => o.index === 0x1600);
    assert.strictEqual(map.name, "RPDO 1 mapping");               // trailing comment stripped
    assert.strictEqual(map.subs.length, 9);
  });
  await test("parseEds: not an EDS", () => {
    assert.strictEqual(od.parseEds("hello world").objects.length, 0);
  });

  await test("builtinObjects: table shape and axis offset", () => {
    const t = od.builtinObjects(1);
    const idx = t.map((o) => o.index);
    for (const must of [0x1000, 0x1018, 0x1021, 0x1400, 0x1A03, 0x6040, 0x6041, 0x607D, 0x60FF, 0x6502]) assert(idx.includes(must), "missing " + must.toString(16));
    const identity = t.find((o) => o.index === 0x1018);
    assert.strictEqual(identity.objectType, 9);
    assert.strictEqual(identity.subs.length, 5);
    assert.strictEqual(identity.subs[4].name, "Serial number");
    const mapping = t.find((o) => o.index === 0x1600);
    assert.strictEqual(mapping.objectType, 8);
    assert.strictEqual(mapping.dynamicCount, true);
    assert.strictEqual(mapping.subs.length, 9);
    const t2 = od.builtinObjects(2);
    assert(t2.some((o) => o.index === 0x6840 && o.name === "Controlword"), "axis 2 controlword at 0x6840");
    assert(t2.some((o) => o.index === 0x1000), "communication profile is never offset");
    assert(!t2.some((o) => o.index === 0x6040), "axis 1 objects not in the axis 2 table");
    // every entry has a name and an object type
    for (const o of t) { assert(o.name && (o.objectType === 7 || o.objectType === 8 || o.objectType === 9), "bad row " + o.index.toString(16)); }
  });

  await test("odDecodeValue: typed and inferred", () => {
    const b = (...a) => Uint8Array.from(a);
    assert.strictEqual(od.odDecodeValue(b(0xff), 0x2).text, "-1");                 // INTEGER8
    assert.strictEqual(od.odDecodeValue(b(0xff), 0x5).text, "255");                // UNSIGNED8
    assert.strictEqual(od.odDecodeValue(b(0x37, 0x02), 0x6).text, "567");          // UNSIGNED16 LE
    assert.strictEqual(od.odDecodeValue(b(0xff, 0xff, 0xff, 0xff), 0x4).text, "-1"); // INTEGER32
    assert.strictEqual(od.odDecodeValue(b(0x92, 0x01, 0x02, 0x00), 0x7).text, "131474");
    assert.strictEqual(od.odDecodeValue(b(0x00, 0x00, 0x80, 0x3f), 0x8).text, "1"); // REAL32 1.0
    assert.strictEqual(od.odDecodeValue(b(0x53, 0x65, 0x72, 0x76, 0x6f, 0x00), 0x9).text, JSON.stringify("Servo"));
    assert.strictEqual(od.odDecodeValue(b(1, 2, 3), 0xF).text, "3 bytes");        // DOMAIN
    assert.strictEqual(od.odDecodeValue(b(1, 2, 3), 0xF).hex, "01 02 03");
    assert.strictEqual(od.odDecodeValue(b(0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff), 0x15).text, "-1"); // INTEGER64
    assert.strictEqual(od.odDecodeValue(b(0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff), 0x1B).text, "18446744073709551615");
    // unknown type: inferred from the length
    const u = od.odDecodeValue(b(0xfe, 0xff), null);
    assert.strictEqual(u.text, "65534 / -2");
    assert.strictEqual(u.kind, "u16?");
    assert.strictEqual(od.odDecodeValue(b(0x68, 0x69, 0x21, 0x20, 0x33), null).kind, "string?");
    assert.strictEqual(od.odDecodeValue(b(0x00, 0x01, 0x02, 0x03, 0x04, 0x05), null).kind, "hex?");
    // declared wider than the object: falls back to inference instead of throwing
    assert.strictEqual(od.odDecodeValue(b(0x05), 0x7).kind, "u8?");
    assert.strictEqual(od.odKindToSelect(0x3), "i16");
    assert.strictEqual(od.odKindToSelect(0x9), "string");
    assert.strictEqual(od.odKindToSelect(0xF), "hex");
    assert.strictEqual(od.odTypeName(0x10), "INTEGER24");
    assert.strictEqual(od.odTypeName(0x1234), "type 0x1234");
  });

  await test("odClassifyError: abort codes, timeouts, cancellation", () => {
    assert.strictEqual(od.odClassifyError(abort(0x06020000, "object does not exist in the dictionary")).status, "absent");
    assert.strictEqual(od.odClassifyError(abort(0x06090011, "subindex does not exist")).status, "no-sub");
    assert.strictEqual(od.odClassifyError(abort(0x06010001, "attempt to read a write-only object")).status, "write-only");
    const other = od.odClassifyError(abort(0x08000020, "data cannot be transferred/stored"));
    assert.strictEqual(other.status, "abort");
    assert.strictEqual(other.text, "abort 0x08000020 data cannot be transferred/stored");
    assert.strictEqual(od.odClassifyError(timeout()).status, "timeout");
    assert.strictEqual(od.odClassifyError(new Error("SDO aborted")).status, "cancelled");
    assert.strictEqual(od.odClassifyError(new Error("disconnected")).status, "cancelled");
    assert.strictEqual(od.odClassifyError(new Error("something else")).status, "error");
  });

  // A mocked node: 0x1000 present, 0x1001 write-only (odd but tests the path),
  // 0x1018 a record with 2 entries, 0x1003 an array whose count says 2,
  // 0x1600 absent, 0x6040 times out, 0x2001 write-only by access type.
  function mockDevice(log) {
    const values = new Map([
      ["1000:0", Uint8Array.of(0x92, 0x01, 0x02, 0x00)],
      ["1018:0", Uint8Array.of(2)],
      ["1018:1", Uint8Array.of(0xcd, 0xab, 0x00, 0x00)],
      ["1018:2", Uint8Array.of(0x01, 0x00, 0x00, 0x00)],
      ["1003:0", Uint8Array.of(2)],
      ["1003:1", Uint8Array.of(0x11, 0x00, 0x00, 0x00)],
      ["1003:2", Uint8Array.of(0x22, 0x00, 0x00, 0x00)],
    ]);
    return async (index, sub) => {
      log.push(index.toString(16) + ":" + sub);
      if (index === 0x1001) throw abort(0x06010001, "attempt to read a write-only object");
      if (index === 0x1600) throw abort(0x06020000, "object does not exist in the dictionary");
      if (index === 0x6040) throw timeout();
      const key = index.toString(16) + ":" + sub;
      if (values.has(key)) return values.get(key);
      if (index === 0x1018 || index === 0x1003) throw abort(0x06090011, "subindex does not exist");
      throw abort(0x06020000, "object does not exist in the dictionary");
    };
  }

  await test("odScan: walk, classification, counts and CSV", async () => {
    const eds = od.parseEds(EDS_SAMPLE);
    const log = [];
    const progress = [];
    const rows = await od.odScan(eds.objects, mockDevice(log), { progress: (d, t) => progress.push([d, t]) });
    const byKey = new Map(rows.map((r) => [r.index.toString(16) + ":" + r.sub, r]));
    assert.strictEqual(byKey.get("1000:0").status, "ok");
    assert.strictEqual(byKey.get("1000:0").value.text, "131474");
    assert.strictEqual(byKey.get("1001:0").status, "write-only");
    assert.strictEqual(byKey.get("1018:0").status, "ok");
    assert.strictEqual(byKey.get("1018:0").isGroup, true);
    assert.strictEqual(byKey.get("1018:1").value.text, "43981");
    assert.strictEqual(byKey.get("1018:2").status, "ok");
    assert.strictEqual(byKey.get("1018:3"), undefined);              // count says 2, the EDS listed up to 2 anyway
    // 0x1003: count 2 -> subs 3 and 4 are reported absent WITHOUT being read
    assert.strictEqual(byKey.get("1003:2").status, "ok");
    assert.strictEqual(byKey.get("1003:3").status, "absent");
    assert(!log.includes("1003:3"), "sub above the entry count must not be read");
    assert.strictEqual(byKey.get("1600:0").status, "absent");
    assert(!log.includes("1600:1"), "subs of an absent object must not be read");
    assert.strictEqual(byKey.get("6040:0").status, "timeout");
    assert.strictEqual(byKey.get("6040:0").error, "no response");
    assert.strictEqual(byKey.get("2001:0").status, "write-only");   // by access type, no read issued
    assert(!log.includes("2001:0"));
    const c = od.odCounts(rows);
    // present: 1000:0, 1001:0 (write-only counts as present), 1018:0..2, 1003:0..2, 2001:0
    assert.deepStrictEqual(c, { present: 9, absent: 3, errors: 1 });
    assert(progress.length > 0 && progress[progress.length - 1][0] === progress[progress.length - 1][1], "progress ends at total");
    const csv = od.odToCsv(rows);
    assert(csv.startsWith("index,sub,name,type,access,status,value,raw\n"));
    assert(csv.includes("0x1000,0,Device type,UNSIGNED32,ro,ok,131474,92 01 02 00"));
    // an ARRAY's header row is its sub 0 (the UNSIGNED8 entry count)
    assert(csv.includes("0x1600,0,RPDO 1 mapping,UNSIGNED8,ro,absent,absent,"), csv);
    assert(csv.includes('0x6040,0,Controlword,UNSIGNED16,rww,timeout,no response,'));
  });

  await test("odScan: built-in table, dynamic array count and cancellation", async () => {
    const t = od.builtinObjects(1).filter((o) => o.index === 0x1003 || o.index === 0x1018 || o.index === 0x6041);
    const log = [];
    const rows = await od.odScan(t, mockDevice(log), {});
    const byKey = new Map(rows.map((r) => [r.index.toString(16) + ":" + r.sub, r]));
    // 0x1003 is a dynamic ARRAY (up to 254): the count said 2, so 2 subs read, none above
    assert.strictEqual(byKey.get("1003:1").status, "ok");
    assert.strictEqual(byKey.get("1003:2").status, "ok");
    assert(!log.includes("1003:3"));
    // 0x1018 count 2: subs 3 and 4 listed by the table are absent without a read
    assert.strictEqual(byKey.get("1018:3").status, "absent");
    assert.strictEqual(byKey.get("1018:4").status, "absent");
    assert(!log.includes("1018:3"));
    assert.strictEqual(byKey.get("6041:0").status, "absent");
    // cancellation stops the walk
    const log2 = [];
    let n = 0;
    const rows2 = await od.odScan(od.builtinObjects(1), mockDevice(log2), { cancelled: () => ++n > 3 });
    assert(rows2.length <= 4 && log2.length <= 4, "cancelled scan must stop early");
    // a disconnect surfaces as the SDO client's rejection: the walk returns what it has
    const rows3 = await od.odScan(od.builtinObjects(1), async () => { throw new Error("SDO aborted"); }, {});
    assert.strictEqual(rows3.length, 1);
    assert.strictEqual(rows3[0].status, "cancelled");
  });

  await test("odScan: record whose sub 0 cannot be read falls back to the listed subs", async () => {
    const t = od.builtinObjects(1).filter((o) => o.index === 0x607D);
    const rows = await od.odScan(t, async (index, sub) => {
      if (sub === 0) throw abort(0x06090011, "subindex does not exist");
      return Uint8Array.of(sub, 0, 0, 0);
    }, {});
    assert.strictEqual(rows[0].status, "no-sub");
    assert.strictEqual(rows.length, 3);
    assert.strictEqual(rows[1].value.text, "1");
    assert.strictEqual(rows[2].value.text, "2");
  });

  console.log(passed + " test(s) passed" + (process.exitCode ? ", some FAILED" : ""));
})();

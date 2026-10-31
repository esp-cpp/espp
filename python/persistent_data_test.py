"""espp.persistent_data test: the Python mirror of components/persistent_data.

Checks the dataclass-driven layout fingerprint and alpaca payload codec byte for
byte against vectors produced by the C++ codec (python/persistent_data_vectors.txt,
generated from include/detail/persistent_data_codec.hpp), the newest-first
version list with its migration chain, every error path, and the file-backed
PersistentData object (atomic save, load_or_create, update, erase).

Runs against an installed espp wheel (``import espp``) or, without one, straight
from the source tree. Exit code 0 on full pass, 1 on any failure.
"""

from __future__ import annotations

import enum
import os
import sys
import tempfile
from dataclasses import dataclass, field
from typing import Annotated, ClassVar, Optional

try:
    from espp import persistent_data as pd
except ImportError:  # no wheel installed: load the pure module from the source tree
    import importlib.util

    _src = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lib", "python_bindings",
                        "espp", "persistent_data.py")
    _spec = importlib.util.spec_from_file_location("persistent_data", _src)
    pd = importlib.util.module_from_spec(_spec)
    sys.modules["persistent_data"] = pd  # dataclasses defined in the module look it up here
    _spec.loader.exec_module(pd)

u8, u16, u32, u64, i8, i16, i32, i64, f32, f64 = (pd.u8, pd.u16, pd.u32, pd.u64, pd.i8, pd.i16,
                                                  pd.i32, pd.i64, pd.f32, pd.f64)
Errc = pd.Errc
PersistentDataError = pd.PersistentDataError

failures = 0
checks = 0


def check(desc: str, condition: bool) -> None:
    global failures, checks
    checks += 1
    if condition:
        print(f"  PASS: {desc}")
    else:
        print(f"  FAIL: {desc}")
        failures += 1


def raises(errc: Errc, fn) -> bool:
    try:
        fn()
    except PersistentDataError as exc:
        return exc.errc == errc
    return False


# ---------------------------------------------------------------------------
# The C++ types from persistent_data_vectors.txt, as dataclasses
# ---------------------------------------------------------------------------


class Mode(enum.IntEnum):
    off = 0
    on = 1


@dataclass
class Limits:
    min: f32 = 0.0
    max: f32 = 1.0


@dataclass
class V1:
    VERSION: ClassVar[int] = 1
    counter: u32 = 0
    name: str = ""


@dataclass
class V2:
    VERSION: ClassVar[int] = 2
    counter: u32 = 0
    name: str = ""
    gains: list[f32] = field(default_factory=list)

    @classmethod
    def migrate(cls, v1: V1) -> "V2":
        return cls(counter=v1.counter, name=v1.name, gains=[1.0])


@dataclass
class V3:
    VERSION: ClassVar[int] = 3
    name: str = ""
    gains: list[f32] = field(default_factory=list)
    limits: Limits = field(default_factory=Limits)
    enabled: bool = True
    mode: Annotated[Mode, "B"] = Mode.off
    tag: pd.array(u8, 3) = field(default_factory=lambda: [0, 0, 0])
    offset: i16 = 0
    scale: f64 = 1.0
    big: u64 = 0
    small: i8 = 0

    @classmethod
    def migrate(cls, v2: V2) -> "V3":
        return cls(name=v2.name, gains=v2.gains, limits=Limits(-1.0, 1.0), enabled=v2.counter > 0)


# the same layout as V3 but claiming version 3 under another name: the fingerprint must agree
@dataclass
class SameAsV3:
    VERSION: ClassVar[int] = 3
    n: str = ""
    g: list[f32] = field(default_factory=list)
    lim: Limits = field(default_factory=Limits)
    e: bool = True
    m: Annotated[Mode, "B"] = Mode.off
    t: pd.array(u8, 3) = field(default_factory=lambda: [0, 0, 0])
    o: i16 = 0
    s: f64 = 1.0
    b: u64 = 0
    sm: i8 = 0


# a different layout that reuses version 3
@dataclass
class OtherV3:
    VERSION: ClassVar[int] = 3
    a: u64 = 0
    b: u64 = 0


# a version this "firmware" does not know
@dataclass
class V4:
    VERSION: ClassVar[int] = 4
    name: str = "future"
    flags: u8 = 0


VALUES = {
    "v1": V1(7, "one"),
    "v2": V2(0, "two", [2.0, 0.5]),
    "v3": V3("abc", [0.25], Limits(-3.0, 3.0), False, Mode.on, [1, 2, 3], -5, 2.5, 10000000000, -1),
    "v3_empty": V3(),
}
CLASSES = {"v1": V1, "v2": V2, "v3": V3, "v3_empty": V3}


def load_vectors():
    here = os.path.dirname(os.path.abspath(__file__))
    out = {}
    with open(os.path.join(here, "persistent_data_vectors.txt")) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            name, ver, fp, desc, file = line.split()
            out[name] = (int(ver), int(fp, 16), bytes.fromhex(desc), bytes.fromhex(file))
    return out


# ---------------------------------------------------------------------------
print("--- layout fingerprint vs the C++ codec ---")
vectors = load_vectors()
check("vectors loaded", set(vectors) == set(VALUES))
for name, (ver, fp, desc, file) in vectors.items():
    cls = CLASSES[name]
    check(f"{name}: describe() bytes match C++", pd.describe(cls) == desc)
    check(f"{name}: fingerprint matches C++", pd.fingerprint(cls) == fp)
check("same layout under other field names -> same fingerprint", pd.fingerprint(SameAsV3) == pd.fingerprint(V3))
check("different layout -> different fingerprint", pd.fingerprint(OtherV3) != pd.fingerprint(V3))
check("V1 / V2 differ", pd.fingerprint(V1) != pd.fingerprint(V2))

# the grammar, spelled out for one struct: struct_(25) nfields=2 u32(4) string(13)
check("describe grammar for V1", pd.describe(V1) == bytes([25, 2, 0, 4, 13]))

# ---------------------------------------------------------------------------
print("--- payload / file encoding vs the C++ codec ---")
for name, (ver, fp, desc, file) in vectors.items():
    value = VALUES[name]
    encoded = pd.encode(value)
    check(f"{name}: encode() is byte-identical to the C++ file", encoded == file)
    version, layout, payload = pd.parse(file)
    check(f"{name}: header version / layout", version == ver and layout == fp)
    decoded = pd.decode_payload(CLASSES[name], payload)
    check(f"{name}: decode_payload() of the C++ payload equals the value", decoded == value)
check("decoded enum is the Enum member", type(pd.decode_payload(V3, pd.parse(vectors["v3"][3])[2]).mode) is Mode)

# ---------------------------------------------------------------------------
print("--- version list + migration chain ---")
full = pd.VersionList([V3, V2, V1])
check("current / current_version", full.current is V3 and full.current_version == 3)
check("supports", full.supports(1) and full.supports(3) and not full.supports(4))

value, info = pd.decode(full, vectors["v1"][3])
check("v1 file migrates v1 -> v2 -> v3", isinstance(value, V3) and value.name == "one"
      and value.gains == [1.0] and value.limits == Limits(-1.0, 1.0) and value.enabled is True)
check("LoadInfo reports version 1, migrated", info.file_version == 1 and info.migrated)
value, info = pd.decode(full, vectors["v2"][3])
check("v2 file migrates v2 -> v3", value.gains == [2.0, 0.5] and value.enabled is False and info.migrated)
value, info = pd.decode(full, vectors["v3"][3])
check("v3 file is not migrated", value == VALUES["v3"] and not info.migrated)

check("dropped v1 support -> unknown_version",
      raises(Errc.unknown_version, lambda: pd.decode([V3, V2], vectors["v1"][3])))
check("newer firmware's file -> unknown_version",
      raises(Errc.unknown_version, lambda: pd.decode(full, pd.encode(V4()))))
check("same version, different layout -> layout_mismatch",
      raises(Errc.layout_mismatch, lambda: pd.decode(full, pd.encode(OtherV3(1, 2)))))
check("same layout, other class name -> decodes",
      pd.decode(full, pd.encode(SameAsV3(n="x")))[0].name == "x")


def bad_list(classes):
    try:
        pd.VersionList(classes)
    except TypeError:
        return True
    return False


check("oldest-first list is rejected", bad_list([V1, V2, V3]))
check("missing migrate is rejected", bad_list([OtherV3, V2]))


@dataclass
class NoVersion:
    x: u8 = 0


check("missing VERSION is rejected", bad_list([NoVersion]))


@dataclass
class BareInt:
    VERSION: ClassVar[int] = 1
    x: int = 0


check("bare int field is rejected", bad_list([BareInt]))


@dataclass
class VersionAsField:
    VERSION: int = 1  # not a ClassVar: a mistake worth a clear message
    x: u8 = 0


check("VERSION as a dataclass field is rejected", bad_list([VersionAsField]))


@dataclass
class Validated:
    VERSION: ClassVar[int] = 1
    x: u8 = 0

    def __post_init__(self):
        if self.x > 9:
            raise ValueError("x must be <= 9")


check("constructor validation failure -> deserialize_failed",
      raises(Errc.deserialize_failed, lambda: pd.decode_payload(Validated, b"\x10")))

# ---------------------------------------------------------------------------
print("--- corrupt / truncated input ---")
good = vectors["v3"][3]
check("empty -> bad_header", raises(Errc.bad_header, lambda: pd.parse(b"")))
check("short header -> bad_header", raises(Errc.bad_header, lambda: pd.parse(good[:10])))
check("wrong magic -> bad_header", raises(Errc.bad_header, lambda: pd.parse(b"X" + good[1:])))
check("truncated payload -> bad_header", raises(Errc.bad_header, lambda: pd.parse(good[:-1])))
check("trailing junk -> bad_header", raises(Errc.bad_header, lambda: pd.parse(good + b"\0")))
flipped = bytearray(good)
flipped[pd.HEADER_SIZE + 2] ^= 1
check("flipped payload bit -> bad_checksum", raises(Errc.bad_checksum, lambda: pd.parse(bytes(flipped))))
flipped = bytearray(good)
flipped[4] ^= 1
check("flipped version byte -> bad_checksum", raises(Errc.bad_checksum, lambda: pd.parse(bytes(flipped))))
# a payload that passes the header but is short for the type (strict)
check("short payload -> deserialize_failed",
      raises(Errc.deserialize_failed, lambda: pd.decode_payload(V1, b"\x07\x00\x00\x00")))
check("payload with trailing bytes -> deserialize_failed",
      raises(Errc.deserialize_failed, lambda: pd.decode_payload(V1, pd.encode_payload(V1(7, "one")) + b"\0")))
check("absurd element count -> deserialize_failed",
      raises(Errc.deserialize_failed, lambda: pd.decode_payload(V1, b"\x07\x00\x00\x00\xff\xff\xff\xff")))
check("array of the wrong length -> serialize_failed",
      raises(Errc.serialize_failed, lambda: pd.encode(V3(tag=[1, 2]))))
check("out-of-range integer -> serialize_failed",
      raises(Errc.serialize_failed, lambda: pd.encode(V3(small=200))))

# ---------------------------------------------------------------------------
print("--- the other field types round-trip ---")


@dataclass
class Kitchen:
    VERSION: ClassVar[int] = 1
    c: pd.char = "x"
    m: dict[str, i32] = field(default_factory=dict)
    um: pd.unordered(dict[u8, str]) = field(default_factory=dict)
    s: set[u16] = field(default_factory=set)
    o: Optional[f64] = None
    o2: Optional[str] = None
    t: tuple[i8, str, f32] = (0, "", 0.0)
    nested: list[Limits] = field(default_factory=list)


k = Kitchen(c="z", m={"b": 2, "a": 1}, um={1: "one"}, s={3, 1, 2}, o=2.5, o2=None, t=(-1, "t", 0.5),
            nested=[Limits(1, 2), Limits(3, 4)])
rt = pd.decode_payload(Kitchen, pd.encode_payload(k))
check("kitchen-sink round trip", rt == k)
check("std::map keys are written sorted",
      pd.encode_payload(Kitchen(m={"b": 2, "a": 1})) == pd.encode_payload(Kitchen(m={"a": 1, "b": 2})))
check("kitchen describe grammar", pd.describe(Kitchen) == bytes(
    [25, 8, 0,  # struct, 8 fields
     1,  # char
     16, 13, 8,  # map<string, int32>
     17, 2, 13,  # unordered_map<uint8, string>
     18, 3,  # set<uint16>
     20, 11,  # optional<double>
     20, 13,  # optional<string>
     22, 3, 0, 6, 13, 10,  # tuple<int8, string, float>
     15, 25, 2, 0, 10, 10]))  # vector<Limits{float, float}>

# ---------------------------------------------------------------------------
print("--- PersistentData file operations ---")
with tempfile.TemporaryDirectory() as tmp:
    path = os.path.join(tmp, "settings.bin")
    store = pd.PersistentData([V3, V2, V1], path)
    check("no file: exists() false, not loaded", not store.exists() and not store.is_loaded)
    check("load of a missing file -> file_not_found", raises(Errc.file_not_found, store.load))
    check("erase of a missing file succeeds", (store.erase(), True)[1])

    created = store.load_or_create(V3(name="fresh"))
    check("load_or_create creates the file", store.exists() and created.name == "fresh" and store.is_loaded)
    check("no temporary left behind", not os.path.exists(path + ".tmp"))
    again = pd.PersistentData([V3, V2, V1], path, auto_load=True)
    check("auto_load reads it back", again.get() == V3(name="fresh") and again.loaded_version == 3
          and not again.was_migrated)

    check("update() mutating in place saves",
          store.update(lambda s: s.gains.append(0.75)) and pd.PersistentData([V3], path, auto_load=True).get().gains == [0.75])
    check("update() returning a replacement saves it",
          store.update(lambda s: V3(name="replaced")) and pd.PersistentData([V3], path, auto_load=True).get().name == "replaced")
    check("update() returning False does not save",
          not store.update(lambda s: False))
    check("save(data) replaces memory and file",
          (store.save(V3(name="saved")), pd.PersistentData([V3], path, auto_load=True).get().name)[1] == "saved")

    # an old-version file on disk is migrated and rewritten
    with open(path, "wb") as f:
        f.write(vectors["v1"][3])
    loaded = store.load()
    check("v1 file on disk loads migrated", loaded.name == "one" and store.loaded_version == 1 and store.was_migrated)
    fresh = pd.PersistentData([V3, V2, V1], path, auto_load=True)
    check("file was rewritten as the current version", fresh.loaded_version == 3 and not fresh.was_migrated)

    store2 = pd.PersistentData([V3, V2, V1], path, save_after_migration=False)
    with open(path, "wb") as f:
        f.write(vectors["v2"][3])
    store2.load()
    check("save_after_migration=False keeps the old file",
          pd.PersistentData([V3, V2, V1], path, auto_load=True).loaded_version == 2)

    # a newer-firmware file is never replaced by load_or_create
    with open(path, "wb") as f:
        f.write(pd.encode(V4()))
    check("load_or_create does not replace a newer file",
          raises(Errc.unknown_version, lambda: store.load_or_create(V3())))
    check("memory untouched after a failed load", store.get().name == "one")

    # corruption on disk
    with open(path, "r+b") as f:
        f.seek(pd.HEADER_SIZE)
        f.write(b"\xff")
    check("corrupt file -> bad_checksum", raises(Errc.bad_checksum, store.load))
    with open(path, "wb") as f:
        f.write(vectors["v3"][3][:pd.HEADER_SIZE + 3])
    check("truncated file -> bad_header", raises(Errc.bad_header, store.load))

    # size cap
    small = pd.PersistentData([V3], path, max_file_size=8)
    check("file over max_file_size -> file_too_large", raises(Errc.file_too_large, small.load))

    # type checks
    try:
        store.save(V1())
        check("save() of the wrong type is rejected", False)
    except TypeError:
        check("save() of the wrong type is rejected", True)

    # erase clears
    store.erase()
    check("erase removes the file", not store.exists() and not store.is_loaded)

    # save into a directory that does not exist -> io_error
    bad = pd.PersistentData([V3], os.path.join(tmp, "missing", "x.bin"))
    check("save into a missing directory -> io_error", raises(Errc.io_error, lambda: bad.save(V3())))

print(f"\n{checks - failures}/{checks} checks passed")
sys.exit(1 if failures else 0)

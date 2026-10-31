"""Python counterpart of the C++ ``espp::PersistentData`` component.

Reads and writes the same files the firmware does -- a 20-byte header (magic,
schema version, layout fingerprint, payload length, CRC32) in front of an
alpaca payload (``fixed_length_encoding | strict``) -- so a PC tool can inspect,
edit or generate a device's settings file, and a host test can exercise a
migration chain without hardware. Pure Python (``struct`` + ``zlib``), no
extension module needed: the payload encoding is fixed-width little-endian
integers, 32-bit length prefixes and nothing else.

Schema versions are plain :mod:`dataclasses`, newest first, exactly like the C++
type list. Because Python's ``int`` and ``float`` carry no width, numeric fields
use the width aliases below (``u8`` ... ``f64``); a bare ``int``/``float`` field
is an error rather than a silent guess. Every version declares a ``VERSION``
class variable, and every version except the oldest provides ``migrate(older)``
(a classmethod or staticmethod) from the version after it in the list::

    from dataclasses import dataclass, field
    from typing import ClassVar
    from espp.persistent_data import PersistentData, u8, u32, f32, array

    @dataclass
    class SettingsV1:
        VERSION: ClassVar[int] = 1
        brightness: u8 = 50
        device_name: str = "espp"

    @dataclass
    class Settings:
        VERSION: ClassVar[int] = 2
        device_name: str = "espp"
        brightness: f32 = 0.5
        favorites: list[str] = field(default_factory=list)
        gains: array(f32, 4) = field(default_factory=lambda: [1.0] * 4)

        @classmethod
        def migrate(cls, v1: SettingsV1) -> "Settings":
            return cls(device_name=v1.device_name, brightness=v1.brightness / 100)

    settings = PersistentData([Settings, SettingsV1], "/mnt/device/settings.bin")
    data = settings.load_or_create(Settings())   # migrates a v1 file, rewrites it
    settings.update(lambda s: s.favorites.append("red"))

Type mapping (Python annotation -> alpaca field type, as the firmware sees it):

======================================  ============================================
``bool``                                ``bool``
``u8 u16 u32 u64 i8 i16 i32 i64``        the fixed-width integer
``f32`` / ``f64``                        ``float`` / ``double``
``char``                                 ``char`` (one-character ``str``)
``str``                                  ``std::string`` (UTF-8)
``list[T]``                              ``std::vector<T>``
``array(T, N)``                          ``std::array<T, N>``
``dict[K, V]`` / ``unordered(dict[K, V])``  ``std::map`` / ``std::unordered_map``
``set[T]`` / ``unordered(set[T])``       ``std::set`` / ``std::unordered_set``
``Optional[T]``                          ``std::optional<T>``
``tuple[A, B, ...]``                     ``std::tuple<A, B, ...>``
``enum.Enum`` (``Annotated[E, "B"]`` ...)  ``enum class E : uint8_t`` (default ``int``)
nested ``@dataclass``                    nested aggregate
======================================  ============================================

The layout fingerprint in the header is computed from these annotations by the
grammar documented in ``components/persistent_data/README.md``, so a file written
here verifies on the device and a device file verifies here. A mismatch between
the Python dataclass and the C++ struct surfaces as ``layout_mismatch``, never
as silently wrong values.
"""

from __future__ import annotations

import dataclasses
import enum
import os
import struct
import threading
import types
import typing
import zlib
from typing import Annotated, Any, Callable, ClassVar, Optional, Sequence

__all__ = [
    "Errc",
    "PersistentDataError",
    "PersistentData",
    "LoadInfo",
    "MAGIC",
    "HEADER_SIZE",
    "bool_",
    "char",
    "u8",
    "u16",
    "u32",
    "u64",
    "i8",
    "i16",
    "i32",
    "i64",
    "f32",
    "f64",
    "array",
    "unordered",
    "describe",
    "fingerprint",
    "encode_payload",
    "decode_payload",
    "encode",
    "decode",
    "VersionList",
]

# ---------------------------------------------------------------------------
# File format constants (must match include/detail/persistent_data_codec.hpp)
# ---------------------------------------------------------------------------

MAGIC = 0x44505345  # "ESPD" little-endian
HEADER_SIZE = 20
_HEADER = struct.Struct("<IIIII")  # magic, version, layout, length, crc32
_U16 = struct.Struct("<H")
_U32 = struct.Struct("<I")


class Errc(enum.IntEnum):
    """Mirror of ``espp::PersistentDataErrc``."""

    file_not_found = 1
    io_error = 2
    file_too_large = 3
    bad_header = 4
    bad_checksum = 5
    unknown_version = 6
    deserialize_failed = 7
    serialize_failed = 8
    layout_mismatch = 9


_ERRC_MESSAGES = {
    Errc.file_not_found: "File not found",
    Errc.io_error: "Filesystem I/O error",
    Errc.file_too_large: "File larger than the configured maximum",
    Errc.bad_header: "Bad file header (wrong magic, wrong length or truncated)",
    Errc.bad_checksum: "Checksum mismatch (corrupt file)",
    Errc.unknown_version: "Unsupported schema version",
    Errc.deserialize_failed: "Payload does not decode as the type registered for its version",
    Errc.serialize_failed: "Could not serialize the data",
    Errc.layout_mismatch: "Payload layout does not match the type registered for its version",
}


class PersistentDataError(Exception):
    """Raised by every operation that fails; ``errc`` says why (compare with :class:`Errc`)."""

    def __init__(self, errc: Errc, detail: str = ""):
        self.errc = Errc(errc)
        self.detail = detail
        msg = _ERRC_MESSAGES[self.errc]
        super().__init__(f"{msg}: {detail}" if detail else msg)


# ---------------------------------------------------------------------------
# Field-type codes (alpaca/detail/field_type.h) and the width aliases
# ---------------------------------------------------------------------------


class _FT(enum.IntEnum):
    bool_ = 0
    char_ = 1
    uint8 = 2
    uint16 = 3
    uint32 = 4
    uint64 = 5
    int8 = 6
    int16 = 7
    int32 = 8
    int64 = 9
    float32 = 10
    float64 = 11
    enum_class = 12
    string = 13
    array = 14
    vector = 15
    map = 16
    unordered_map = 17
    set = 18
    unordered_set = 19
    optional = 20
    pair = 21
    tuple = 22
    variant = 23
    unique_ptr = 24
    struct_ = 25


# width code (struct-module letter) -> (field type, struct format)
_SCALARS = {
    "?": (_FT.bool_, "<?"),
    "c": (_FT.char_, "<c"),
    "b": (_FT.int8, "<b"),
    "B": (_FT.uint8, "<B"),
    "h": (_FT.int16, "<h"),
    "H": (_FT.uint16, "<H"),
    "i": (_FT.int32, "<i"),
    "I": (_FT.uint32, "<I"),
    "q": (_FT.int64, "<q"),
    "Q": (_FT.uint64, "<Q"),
    "f": (_FT.float32, "<f"),
    "d": (_FT.float64, "<d"),
}

bool_ = Annotated[bool, "?"]
char = Annotated[str, "c"]
u8 = Annotated[int, "B"]
u16 = Annotated[int, "H"]
u32 = Annotated[int, "I"]
u64 = Annotated[int, "Q"]
i8 = Annotated[int, "b"]
i16 = Annotated[int, "h"]
i32 = Annotated[int, "i"]
i64 = Annotated[int, "q"]
f32 = Annotated[float, "f"]
f64 = Annotated[float, "d"]


class _ArrayLen:
    __slots__ = ("n",)

    def __init__(self, n: int):
        self.n = int(n)

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"_ArrayLen({self.n})"


class _Unordered:
    __slots__ = ()

    def __repr__(self) -> str:  # pragma: no cover
        return "_Unordered()"


def array(elem: Any, n: int):
    """``std::array<elem, n>``: a list of exactly ``n`` elements (no length prefix on the wire)."""
    return Annotated[list[elem], _ArrayLen(n)]


def unordered(tp: Any):
    """Mark a ``dict``/``set`` annotation as ``std::unordered_map`` / ``std::unordered_set``."""
    return Annotated[tp, _Unordered()]


# ---------------------------------------------------------------------------
# Annotation -> Spec tree
# ---------------------------------------------------------------------------


@dataclasses.dataclass(frozen=True)
class _Spec:
    kind: _FT
    fmt: str = ""  # scalars: struct format
    elems: tuple = ()  # vector/array/set/optional: (elem,); map: (key, value); tuple: elems
    n: int = 0  # array length
    cls: Any = None  # struct / enum class
    fields: tuple = ()  # struct: ((name, spec), ...)


_NONE_TYPE = type(None)


def _split_annotated(tp: Any):
    """Return (base type, [hints]) for Annotated[...] (hints possibly nested), else (tp, [])."""
    hints: list = []
    while typing.get_origin(tp) is Annotated:
        args = typing.get_args(tp)
        tp = args[0]
        hints = list(args[1:]) + hints
    return tp, hints


def _spec_of(tp: Any, where: str) -> _Spec:
    base, hints = _split_annotated(tp)
    width = next((h for h in hints if isinstance(h, str)), None)
    array_len = next((h for h in hints if isinstance(h, _ArrayLen)), None)
    is_unordered = any(isinstance(h, _Unordered) for h in hints)

    # enums before int (IntEnum is an int subclass); the underlying type is the
    # width hint, default int32 like an un-typed C++ `enum class`
    if isinstance(base, type) and issubclass(base, enum.Enum):
        code = width or "i"
        if code not in _SCALARS or code in ("?", "c", "f", "d"):
            raise TypeError(f"{where}: enum underlying type must be an integer width, got {code!r}")
        ft, fmt = _SCALARS[code]
        return _Spec(_FT.enum_class, elems=(_Spec(ft, fmt),), cls=base)

    if base is bool:
        return _Spec(*_SCALARS["?"])
    if base is int:
        if width is None:
            raise TypeError(f"{where}: bare 'int' has no width; use u8/u16/u32/u64/i8/i16/i32/i64")
        if width not in _SCALARS or width in ("?", "c", "f", "d"):
            raise TypeError(f"{where}: {width!r} is not an integer width")
        return _Spec(*_SCALARS[width])
    if base is float:
        if width is None:
            raise TypeError(f"{where}: bare 'float' has no width; use f32 or f64")
        if width not in ("f", "d"):
            raise TypeError(f"{where}: {width!r} is not a float width")
        return _Spec(*_SCALARS[width])
    if base is str:
        if width == "c":
            return _Spec(*_SCALARS["c"])
        return _Spec(_FT.string)
    if base is bytes:
        raise TypeError(f"{where}: 'bytes' is ambiguous; use list[u8] (std::vector<uint8_t>) or str")

    origin = typing.get_origin(base)
    args = typing.get_args(base)
    if origin is list:
        if len(args) != 1:
            raise TypeError(f"{where}: list needs one element type")
        elem = _spec_of(args[0], where + "[]")
        if array_len is not None:
            return _Spec(_FT.array, elems=(elem,), n=array_len.n)
        return _Spec(_FT.vector, elems=(elem,))
    if origin is dict:
        if len(args) != 2:
            raise TypeError(f"{where}: dict needs key and value types")
        k = _spec_of(args[0], where + ".key")
        v = _spec_of(args[1], where + ".value")
        return _Spec(_FT.unordered_map if is_unordered else _FT.map, elems=(k, v))
    if origin in (set, frozenset):
        if len(args) != 1:
            raise TypeError(f"{where}: set needs one element type")
        return _Spec(_FT.unordered_set if is_unordered else _FT.set,
                     elems=(_spec_of(args[0], where + "{}"),))
    if origin is tuple:
        if not args or (len(args) == 2 and args[1] is Ellipsis):
            raise TypeError(f"{where}: tuple needs a fixed list of element types")
        return _Spec(_FT.tuple, elems=tuple(_spec_of(a, f"{where}[{i}]") for i, a in enumerate(args)))
    if origin is typing.Union or origin is types.UnionType:
        non_none = [a for a in args if a is not _NONE_TYPE]
        if len(non_none) == 1 and len(args) == 2:
            return _Spec(_FT.optional, elems=(_spec_of(non_none[0], where + "?"),))
        raise TypeError(f"{where}: only Optional[T] unions are supported (std::variant is not)")
    if dataclasses.is_dataclass(base) and isinstance(base, type):
        return _struct_spec(base)
    raise TypeError(f"{where}: unsupported field type {tp!r}")


_struct_cache: dict = {}


def _struct_spec(cls: type) -> _Spec:
    spec = _struct_cache.get(cls)
    if spec is not None:
        return spec
    if not dataclasses.is_dataclass(cls):
        raise TypeError(f"{cls.__qualname__} is not a dataclass")
    hints = typing.get_type_hints(cls, include_extras=True)
    fields = []
    for f in dataclasses.fields(cls):
        fields.append((f.name, _spec_of(hints[f.name], f"{cls.__qualname__}.{f.name}")))
    spec = _Spec(_FT.struct_, cls=cls, fields=tuple(fields))
    _struct_cache[cls] = spec
    return spec


# ---------------------------------------------------------------------------
# Layout description + fingerprint (README "Layout fingerprint" grammar)
# ---------------------------------------------------------------------------


def _describe(spec: _Spec, out: bytearray) -> None:
    out.append(int(spec.kind))
    if spec.kind in (_FT.enum_class, _FT.vector, _FT.set, _FT.unordered_set, _FT.optional):
        _describe(spec.elems[0], out)
    elif spec.kind == _FT.array:
        out += _U32.pack(spec.n)
        _describe(spec.elems[0], out)
    elif spec.kind in (_FT.map, _FT.unordered_map):
        _describe(spec.elems[0], out)
        _describe(spec.elems[1], out)
    elif spec.kind == _FT.tuple:
        out += _U16.pack(len(spec.elems))
        for e in spec.elems:
            _describe(e, out)
    elif spec.kind == _FT.struct_:
        out += _U16.pack(len(spec.fields))
        for _, f in spec.fields:
            _describe(f, out)


def describe(cls: type) -> bytes:
    """The layout description bytes of a dataclass (what the fingerprint hashes)."""
    out = bytearray()
    _describe(_struct_spec(cls), out)
    return bytes(out)


def fingerprint(cls: type) -> int:
    """CRC32 of :func:`describe`; the ``layout`` field of the file header."""
    return zlib.crc32(describe(cls)) & 0xFFFFFFFF


# ---------------------------------------------------------------------------
# Payload codec (alpaca, fixed_length_encoding | strict)
# ---------------------------------------------------------------------------


def _enc(spec: _Spec, value: Any, out: bytearray, where: str) -> None:
    k = spec.kind
    try:
        if spec.fmt:  # scalar
            if k == _FT.char_:
                b = value.encode("latin-1") if isinstance(value, str) else bytes([value])
                if len(b) != 1:
                    raise ValueError("char must be exactly one byte")
                out += b
            else:
                out += struct.pack(spec.fmt, value)
        elif k == _FT.enum_class:
            v = value.value if isinstance(value, enum.Enum) else value
            out += struct.pack(spec.elems[0].fmt, v)
        elif k == _FT.string:
            b = value.encode("utf-8") if isinstance(value, str) else bytes(value)
            out += _U32.pack(len(b)) + b
        elif k == _FT.vector:
            out += _U32.pack(len(value))
            for i, v in enumerate(value):
                _enc(spec.elems[0], v, out, f"{where}[{i}]")
        elif k == _FT.array:
            if len(value) != spec.n:
                raise ValueError(f"array needs exactly {spec.n} elements, got {len(value)}")
            for i, v in enumerate(value):
                _enc(spec.elems[0], v, out, f"{where}[{i}]")
        elif k in (_FT.map, _FT.unordered_map):
            out += _U32.pack(len(value))
            items = value.items()
            if k == _FT.map:
                items = sorted(items)  # std::map iterates in key order
            for kk, vv in items:
                _enc(spec.elems[0], kk, out, f"{where}.key")
                _enc(spec.elems[1], vv, out, f"{where}[{kk!r}]")
        elif k in (_FT.set, _FT.unordered_set):
            out += _U32.pack(len(value))
            elems = sorted(value) if k == _FT.set else list(value)
            for i, v in enumerate(elems):
                _enc(spec.elems[0], v, out, f"{where}{{{i}}}")
        elif k == _FT.optional:
            if value is None:
                out.append(0)
            else:
                out.append(1)
                _enc(spec.elems[0], value, out, where + "?")
        elif k == _FT.tuple:
            if len(value) != len(spec.elems):
                raise ValueError(f"tuple needs {len(spec.elems)} elements, got {len(value)}")
            for i, (e, v) in enumerate(zip(spec.elems, value)):
                _enc(e, v, out, f"{where}[{i}]")
        elif k == _FT.struct_:
            for name, f in spec.fields:
                _enc(f, getattr(value, name), out, f"{where}.{name}")
        else:  # pragma: no cover
            raise ValueError(f"unsupported kind {k}")
    except PersistentDataError:
        raise
    except (struct.error, ValueError, TypeError, AttributeError, UnicodeError) as exc:
        raise PersistentDataError(Errc.serialize_failed, f"{where}: {exc}") from None


class _Reader:
    __slots__ = ("data", "pos")

    def __init__(self, data: bytes):
        self.data = data
        self.pos = 0

    def take(self, n: int, where: str) -> bytes:
        if n < 0 or self.pos + n > len(self.data):
            raise PersistentDataError(Errc.deserialize_failed,
                                      f"{where}: input ended ({self.pos + n} > {len(self.data)} bytes)")
        b = self.data[self.pos:self.pos + n]
        self.pos += n
        return b

    def count(self, where: str) -> int:
        n = _U32.unpack(self.take(4, where))[0]
        if n > len(self.data) - self.pos:
            raise PersistentDataError(Errc.deserialize_failed,
                                      f"{where}: element count {n} exceeds the remaining bytes")
        return n


def _dec(spec: _Spec, r: _Reader, where: str) -> Any:
    k = spec.kind
    if spec.fmt:
        b = r.take(struct.calcsize(spec.fmt), where)
        v = struct.unpack(spec.fmt, b)[0]
        return v.decode("latin-1") if k == _FT.char_ else v
    if k == _FT.enum_class:
        raw = struct.unpack(spec.elems[0].fmt, r.take(struct.calcsize(spec.elems[0].fmt), where))[0]
        try:
            return spec.cls(raw)
        except ValueError:
            raise PersistentDataError(Errc.deserialize_failed,
                                      f"{where}: {raw} is not a {spec.cls.__qualname__}") from None
    if k == _FT.string:
        n = r.count(where)
        try:
            return r.take(n, where).decode("utf-8")
        except UnicodeDecodeError:
            raise PersistentDataError(Errc.deserialize_failed, f"{where}: invalid UTF-8") from None
    if k == _FT.vector:
        n = r.count(where)
        return [_dec(spec.elems[0], r, f"{where}[{i}]") for i in range(n)]
    if k == _FT.array:
        return [_dec(spec.elems[0], r, f"{where}[{i}]") for i in range(spec.n)]
    if k in (_FT.map, _FT.unordered_map):
        n = r.count(where)
        out = {}
        for i in range(n):
            kk = _dec(spec.elems[0], r, f"{where}.key[{i}]")
            out[kk] = _dec(spec.elems[1], r, f"{where}[{kk!r}]")
        return out
    if k in (_FT.set, _FT.unordered_set):
        n = r.count(where)
        return {_dec(spec.elems[0], r, f"{where}{{{i}}}") for i in range(n)}
    if k == _FT.optional:
        has = r.take(1, where)[0]
        return _dec(spec.elems[0], r, where + "?") if has else None
    if k == _FT.tuple:
        return tuple(_dec(e, r, f"{where}[{i}]") for i, e in enumerate(spec.elems))
    if k == _FT.struct_:
        kwargs = {name: _dec(f, r, f"{where}.{name}") for name, f in spec.fields}
        return spec.cls(**kwargs)
    raise PersistentDataError(Errc.deserialize_failed, f"{where}: unsupported kind {k}")  # pragma: no cover


def encode_payload(value: Any) -> bytes:
    """alpaca-encode a dataclass instance (no header)."""
    out = bytearray()
    _enc(_struct_spec(type(value)), value, out, type(value).__qualname__)
    return bytes(out)


def decode_payload(cls: type, payload: bytes) -> Any:
    """Decode an alpaca payload (no header) as ``cls``. Strict: the whole payload must be consumed."""
    r = _Reader(bytes(payload))
    value = _dec(_struct_spec(cls), r, cls.__qualname__)
    if r.pos != len(r.data):
        raise PersistentDataError(Errc.deserialize_failed,
                                  f"{cls.__qualname__}: {len(r.data) - r.pos} trailing byte(s)")
    return value


# ---------------------------------------------------------------------------
# File format: header + payload
# ---------------------------------------------------------------------------


def _version_of(cls: type) -> int:
    v = getattr(cls, "VERSION", None)
    if not isinstance(v, int) or isinstance(v, bool):
        raise TypeError(f"{cls.__qualname__} needs an integer VERSION class variable")
    return v


def encode(value: Any, version: Optional[int] = None) -> bytes:
    """Encode a dataclass instance into the file format (header + payload).

    ``version`` defaults to the class's ``VERSION``.
    """
    cls = type(value)
    version = _version_of(cls) if version is None else int(version)
    payload = encode_payload(value)
    head = struct.pack("<IIII", MAGIC, version, fingerprint(cls), len(payload))
    crc = zlib.crc32(payload, zlib.crc32(head)) & 0xFFFFFFFF
    return head + _U32.pack(crc) + payload


def parse(data: bytes):
    """Validate the header + checksum; return ``(version, layout, payload)``."""
    data = bytes(data)
    if len(data) < HEADER_SIZE:
        raise PersistentDataError(Errc.bad_header, f"{len(data)} bytes is shorter than the header")
    magic, version, layout, length, crc = _HEADER.unpack_from(data)
    if magic != MAGIC:
        raise PersistentDataError(Errc.bad_header, f"bad magic 0x{magic:08x}")
    if length != len(data) - HEADER_SIZE:
        raise PersistentDataError(Errc.bad_header,
                                  f"length field {length} != {len(data) - HEADER_SIZE} payload bytes")
    computed = zlib.crc32(data[HEADER_SIZE:], zlib.crc32(data[:16])) & 0xFFFFFFFF
    if computed != crc:
        raise PersistentDataError(Errc.bad_checksum, f"0x{computed:08x} != 0x{crc:08x}")
    return version, layout, data[HEADER_SIZE:]


@dataclasses.dataclass
class LoadInfo:
    """What a successful load found."""

    file_version: int = 0
    migrated: bool = False


class VersionList:
    """An ordered list of schema versions, newest first (mirror of ``persistent_data::VersionList``)."""

    def __init__(self, versions: Sequence[type]):
        versions = list(versions)
        if not versions:
            raise TypeError("at least one version class is required")
        self.classes = versions
        self.versions = [_version_of(c) for c in versions]
        for i in range(1, len(self.versions)):
            if self.versions[i] >= self.versions[i - 1]:
                raise TypeError("VERSION values must strictly decrease along the list (newest first)")
        for i in range(len(versions) - 1):
            if not callable(getattr(versions[i], "migrate", None)):
                raise TypeError(f"{versions[i].__qualname__} needs a migrate() from "
                                f"{versions[i + 1].__qualname__}")
        for c in versions:
            _struct_spec(c)  # validate annotations up front
        self.fingerprints = [fingerprint(c) for c in versions]

    @property
    def current(self) -> type:
        return self.classes[0]

    @property
    def current_version(self) -> int:
        return self.versions[0]

    def supports(self, version: int) -> bool:
        return version in self.versions

    def decode(self, version: int, layout: int, payload: bytes) -> Any:
        """Decode a payload of schema ``version`` and migrate it to the current class."""
        if version not in self.versions:
            raise PersistentDataError(Errc.unknown_version, f"version {version}")
        idx = self.versions.index(version)
        if layout != self.fingerprints[idx]:
            raise PersistentDataError(
                Errc.layout_mismatch,
                f"file 0x{layout:08x} != {self.classes[idx].__qualname__} 0x{self.fingerprints[idx]:08x}")
        value = decode_payload(self.classes[idx], payload)
        while idx > 0:
            idx -= 1
            value = self.classes[idx].migrate(value)
        return value


def decode(versions: Sequence[type], data: bytes):
    """Decode bytes produced by :func:`encode` (any listed version) -> ``(value, LoadInfo)``."""
    vl = versions if isinstance(versions, VersionList) else VersionList(versions)
    version, layout, payload = parse(data)
    value = vl.decode(version, layout, payload)
    return value, LoadInfo(file_version=version, migrated=version != vl.current_version)


# ---------------------------------------------------------------------------
# PersistentData: the file-backed object
# ---------------------------------------------------------------------------


class PersistentData:
    """File-backed storage of one dataclass with schema versioning and forward migration.

    ``versions`` is the newest-first version list (the first class is the current
    one). All methods are thread-safe; ``get()`` returns the in-memory object
    itself (dataclasses are mutable), so use :meth:`update` for a locked
    read-modify-write.
    """

    def __init__(self, versions: Sequence[type], path: os.PathLike | str, *,
                 max_file_size: int = 64 * 1024, save_after_migration: bool = True,
                 auto_load: bool = False):
        self.versions = versions if isinstance(versions, VersionList) else VersionList(versions)
        self.path = os.fspath(path)
        self.max_file_size = int(max_file_size)
        self.save_after_migration = bool(save_after_migration)
        self._lock = threading.RLock()
        self._data: Any = None
        self._loaded = False
        self._info = LoadInfo()
        if auto_load:
            self.load()

    # --- state -------------------------------------------------------------

    @property
    def current(self) -> type:
        """The current (newest) version class."""
        return self.versions.current

    @property
    def current_version(self) -> int:
        return self.versions.current_version

    @property
    def is_loaded(self) -> bool:
        """False until a load or save succeeds."""
        with self._lock:
            return self._loaded

    @property
    def loaded_version(self) -> int:
        """Schema version found in the file by the last successful load (0 if none)."""
        with self._lock:
            return self._info.file_version

    @property
    def was_migrated(self) -> bool:
        with self._lock:
            return self._info.migrated

    def exists(self) -> bool:
        return os.path.isfile(self.path)

    def get(self) -> Any:
        """The data in memory (``None`` before any load / set / save)."""
        with self._lock:
            return self._data

    def set(self, data: Any) -> None:
        """Replace the data in memory (does not write the file)."""
        self._check_type(data)
        with self._lock:
            self._data = data

    # --- file operations ---------------------------------------------------

    def load(self) -> Any:
        """Load the file, migrating an older version if needed; returns the data.

        On failure the data in memory is unchanged and :class:`PersistentDataError`
        says why (``file_not_found``, ``io_error``, ``file_too_large``,
        ``bad_header``, ``bad_checksum``, ``unknown_version``, ``layout_mismatch``,
        ``deserialize_failed``).
        """
        with self._lock:
            raw = self._read_file()
            version, layout, payload = parse(raw)
            value = self.versions.decode(version, layout, payload)
            self._data = value
            self._loaded = True
            self._info = LoadInfo(file_version=version, migrated=version != self.current_version)
            if self._info.migrated and self.save_after_migration:
                try:
                    self._write_file(encode(value))
                except PersistentDataError:
                    pass  # the data is loaded fine; only the rewrite failed
            return value

    def load_or_create(self, defaults: Any) -> Any:
        """Load the file, or save ``defaults`` and return it if there is no file.

        Any error other than ``file_not_found`` (a corrupt file, a newer
        firmware's file) is raised, never papered over.
        """
        self._check_type(defaults)
        with self._lock:
            try:
                return self.load()
            except PersistentDataError as exc:
                if exc.errc != Errc.file_not_found:
                    raise
            self._data = defaults
            self._write_file(encode(defaults))
            self._loaded = True
            return defaults

    def save(self, data: Any = None) -> None:
        """Save ``data`` (or the data in memory) to the file, atomically."""
        with self._lock:
            if data is not None:
                self._check_type(data)
                self._data = data
            if self._data is None:
                raise PersistentDataError(Errc.serialize_failed, "nothing to save: no data set")
            self._write_file(encode(self._data))
            self._loaded = True

    def update(self, fn: Callable[[Any], Any]) -> bool:
        """Modify the data in memory under the lock and save it.

        ``fn`` receives the data object; it may mutate it in place and return
        ``None``, return a replacement object, or return ``False`` to skip the
        save (the in-memory change is kept). Returns True if the file was written.
        """
        with self._lock:
            if self._data is None:
                raise PersistentDataError(Errc.serialize_failed, "nothing to update: no data loaded")
            result = fn(self._data)
            if result is False:
                return False
            if result is not None and result is not True:
                self._check_type(result)
                self._data = result
            self._write_file(encode(self._data))
            self._loaded = True
            return True

    def erase(self) -> None:
        """Remove the file (and a leftover temporary); succeeds when there is none."""
        with self._lock:
            for p in (self.path + ".tmp", self.path):
                try:
                    os.remove(p)
                except FileNotFoundError:
                    pass
                except OSError as exc:
                    raise PersistentDataError(Errc.io_error, f"{p}: {exc}") from None
            self._loaded = False
            self._info = LoadInfo()

    # --- bytes without the filesystem -------------------------------------

    def encode(self, data: Any) -> bytes:
        """Encode ``data`` as the current version (header + payload)."""
        self._check_type(data)
        return encode(data)

    def decode(self, data: bytes):
        """Decode bytes of any listed version -> ``(value, LoadInfo)``, migrations applied."""
        return decode(self.versions, data)

    # --- internals ---------------------------------------------------------

    def _check_type(self, data: Any) -> None:
        if not isinstance(data, self.current):
            raise TypeError(f"expected {self.current.__qualname__}, got {type(data).__qualname__}")

    def _read_file(self) -> bytes:
        try:
            size = os.path.getsize(self.path)
        except FileNotFoundError:
            raise PersistentDataError(Errc.file_not_found, self.path) from None
        except OSError as exc:
            raise PersistentDataError(Errc.io_error, f"{self.path}: {exc}") from None
        if size > self.max_file_size:
            raise PersistentDataError(Errc.file_too_large, f"{size} > {self.max_file_size} bytes")
        try:
            with open(self.path, "rb") as f:
                return f.read()
        except OSError as exc:
            raise PersistentDataError(Errc.io_error, f"{self.path}: {exc}") from None

    def _write_file(self, data: bytes) -> None:
        tmp = self.path + ".tmp"
        try:
            with open(tmp, "wb") as f:
                f.write(data)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.path)
        except OSError as exc:
            try:
                os.remove(tmp)
            except OSError:
                pass
            raise PersistentDataError(Errc.io_error, f"{self.path}: {exc}") from None

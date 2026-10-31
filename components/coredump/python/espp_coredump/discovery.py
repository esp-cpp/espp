"""Dispatcher capability discovery: the ListModules TLV parser and the
module-id resolution rule every espp host tool shares.

The dispatcher ``module`` byte is only a routing key: a device may serve any
protocol on any id. Which protocol a module speaks is what the device
advertises in its discovery reply -- the stable protocol id first (e.g.
``"espp.ota"``, ``"espp.coredump"``), then the hosted web app filename, then
the module name. :func:`resolve_module_id` implements exactly the rule the web
consoles' ``resolveModuleId`` uses (``override`` > protocol > app > name >
default), so a host tool and the browser console land on the same module.

Component-local copy: ``components/ota/python/espp_ota/discovery.py`` and
``components/coredump/python/espp_coredump/discovery.py`` are identical.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

from . import frame as _f

#: Reserved dispatcher discovery module and its ListModules request type.
DISCOVERY_MODULE = 0xFF
DISCOVERY_LIST_MODULES = 0x00

#: Latest discovery payload version this parser understands. Version 1 records
#: end after the description; version 2 records add ``[protocol str]
#: [protocol_version u16 LE]``.
#:
#: Forward compatibility: records carry no length, so a newer version can NOT
#: extend the per-record layout (an older parser would read the extra bytes as
#: the next record's id / name). The record layout is frozen at the version-2
#: shape; a newer version may only append data after the whole record list.
#: A payload with a version above this one is therefore parsed as version 2
#: and whatever follows its last record is ignored.
DISCOVERY_VERSION_KNOWN = 2


@dataclass
class ModuleInfo:
    id: int
    name: str
    app: str
    description: str
    protocol: str = ""        # stable protocol id, e.g. "espp.ota" (v2 payloads; "" if unset)
    protocol_version: int = 0  # 0 = unspecified

    def label(self) -> str:
        return f"{self.name or self.protocol or '?'} (#{self.id})"


@dataclass
class DiscoveryInfo:
    version: int
    device_name: str
    firmware: str
    modules: List[ModuleInfo] = field(default_factory=list)

    def has_module(self, module_id: int) -> bool:
        return any(m.id == module_id for m in self.modules)

    def find(self, module_id: int) -> Optional[ModuleInfo]:
        return next((m for m in self.modules if m.id == module_id), None)

    def speaking(self, protocol: str) -> List[ModuleInfo]:
        """Every advertised module whose protocol id equals ``protocol``."""
        return [m for m in self.modules if protocol and m.protocol == protocol]


def make_discovery_request() -> bytes:
    """A dispatcher discovery (ListModules) request on module 0xFF."""
    return _f.build_frame(DISCOVERY_MODULE, DISCOVERY_LIST_MODULES, b"", reply=False)


def parse_discovery(fr: _f.Frame) -> Optional[DiscoveryInfo]:
    """Decode a dispatcher ListModules reply::

        [version u8][reserved u8][device_name str][device_fw str][module_count u8]
        then per module: [id u8][name str][app str][desc str]
                         + (version >= 2) [protocol str][protocol_version u16 LE]

    where ``str`` = ``[len u8][bytes]``. A version above
    :data:`DISCOVERY_VERSION_KNOWN` is parsed with the version-2 record layout
    and any bytes after the last record are ignored (the layout contract, see
    there). Returns None on a malformed payload (a record truncated mid-way is
    dropped, the ones before it are kept: the device itself trims records that
    would not fit the frame)."""
    p = fr.payload
    if len(p) < 3:
        return None
    pos = 0
    version = p[pos]
    pos += 2  # version + reserved

    def read_str() -> Optional[str]:
        nonlocal pos
        if pos >= len(p):
            return None
        n = p[pos]
        pos += 1
        if pos + n > len(p):
            return None
        s = p[pos:pos + n].decode("utf-8", errors="replace")
        pos += n
        return s

    def read_u16() -> Optional[int]:
        nonlocal pos
        if pos + 2 > len(p):
            return None
        v = p[pos] | (p[pos + 1] << 8)
        pos += 2
        return v

    device_name = read_str()
    firmware = read_str()
    if device_name is None or firmware is None or pos >= len(p):
        return None
    count = p[pos]
    pos += 1
    info = DiscoveryInfo(version, device_name, firmware)
    for _ in range(count):
        if pos >= len(p):
            break
        mid = p[pos]
        pos += 1
        name = read_str()
        app = read_str()
        desc = read_str()
        if name is None or app is None or desc is None:
            break
        protocol, protocol_version = "", 0
        if version >= 2:
            proto = read_str()
            pver = read_u16()
            if proto is None or pver is None:
                break
            protocol, protocol_version = proto, pver
        info.modules.append(ModuleInfo(mid, name, app, desc, protocol, protocol_version))
    return info


# ---- module id resolution ----------------------------------------------------
@dataclass
class Resolution:
    id: int
    source: str  # "override" | "protocol" | "app" | "name" | "default"
    protocol_version: Optional[int] = None  # advertised by the chosen module, if any
    warnings: List[str] = field(default_factory=list)


def _norm(s: Optional[str]) -> str:
    return (s or "").strip().lower()


def _base(s: Optional[str]) -> str:
    n = _norm(s).split("?")[0].split("#")[0]
    return n[n.rfind("/") + 1:]


def resolve_module_id(info: Optional[DiscoveryInfo], *, protocol: str, app: str, name: str,
                      fallback: int, protocol_version: Optional[int] = None,
                      override: Optional[int] = None) -> Resolution:
    """Pick the module id a tool should talk to.

    ``info`` is the device's decoded discovery reply, or None when it gave
    none. ``protocol`` / ``app`` / ``name`` identify the protocol the tool
    implements (``protocol_version`` the version it supports); ``fallback`` is
    the protocol's published default id; ``override`` (a ``--module`` value)
    wins over discovery. Order: override > protocol id (exact) > app filename
    > module name > fallback. Warnings describe anything surprising (an
    override the device does not advertise, several candidates, a protocol
    version mismatch, a newer discovery payload than understood)."""
    warnings: List[str] = []
    modules = info.modules if info is not None else None
    if info is not None and info.version > DISCOVERY_VERSION_KNOWN:
        warnings.append(f"discovery payload v{info.version} is newer than this tool understands "
                        f"(v{DISCOVERY_VERSION_KNOWN}); parsed as v{DISCOVERY_VERSION_KNOWN}")

    def done(m: Optional[ModuleInfo], source: str, id_: Optional[int] = None) -> Resolution:
        pver = m.protocol_version if m is not None and m.protocol_version else None
        if m is not None and protocol and m.protocol and m.protocol != protocol:
            warnings.append(f"module #{m.id} advertises protocol {m.protocol}, not {protocol}")
        elif pver and protocol_version is not None and pver != protocol_version:
            warnings.append(f"device speaks {m.protocol or protocol} v{pver}, this tool supports "
                            f"v{protocol_version}")
        return Resolution(id_ if id_ is not None else (m.id if m else fallback), source, pver,
                          warnings)

    if override is not None:
        if not (0 <= override <= 0xEF):
            raise ValueError("module id must be 0..239 (0xF0..0xFF are reserved)")
        m = info.find(override) if info is not None else None
        if modules is not None and m is None:
            warnings.append(f"module #{override} was requested but the device does not advertise it")
        return done(m, "override", override)
    if modules is not None:
        by_proto = info.speaking(protocol)
        if len(by_proto) > 1:
            warnings.append(f"several modules speak {protocol} ("
                            + ", ".join(m.label() for m in by_proto)
                            + "); using the first -- pass --module N to pick another")
        if by_proto:
            return done(by_proto[0], "protocol")
        want_app, want_name = _base(app), _norm(name)
        by_app = next((m for m in modules if want_app and _base(m.app) == want_app), None)
        if by_app is not None:
            return done(by_app, "app")
        by_name = next((m for m in modules if want_name and _norm(m.name) == want_name), None)
        if by_name is not None:
            return done(by_name, "name")
        warnings.append(f"the device does not advertise {protocol or name or app} (advertised: "
                        + (", ".join(m.label() for m in modules) or "none")
                        + f"); using the default module #{fallback}")
    return Resolution(fallback, "default", None, warnings)


def describe_resolution(label: str, r: Resolution, *, protocol: str, app: str, name: str,
                        answered: bool) -> str:
    """One line saying how a module id was chosen (``answered``: the device
    replied to discovery at all)."""
    if r.source == "override":
        how = "from --module"
    elif r.source == "protocol":
        how = f"advertises {protocol}" + (f" v{r.protocol_version}" if r.protocol_version else "")
    elif r.source == "app":
        how = f"advertised as app {app}"
    elif r.source == "name":
        how = f'advertised as "{name}"'
    else:
        how = "default" if answered else "default; the device did not answer discovery"
    return f"{label} module: #{r.id} ({how})"

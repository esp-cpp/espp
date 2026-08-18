"""Python bindings for the cross-platform components of espp.

The actual implementation lives in the compiled ``espp._espp`` extension
module (pybind11); this package re-exports everything from it and ships the
type stubs (``__init__.pyi``) that describe the full API.
"""

from ._espp import *  # type: ignore # noqa: F403
from ._espp import __version__  # noqa: F401

# Pure-Python typed pub/sub layer over the bound RtpsParticipant (accessible as
# ``espp.rtps.Publisher`` / ``espp.rtps.Subscriber``).
from . import rtps  # noqa: F401,E402

# The pure-python ODrive legacy-native protocol CLIENT (espp_odrive, sourced
# from components/odrive_native/python) ships alongside this package in the
# wheel / installed prefix; expose it as ``espp.odrive`` for convenience. The
# bound C++ SERVER side is ``espp.OdriveNative`` (from _espp above). Optional:
# a source-tree espp package without the sibling won't have the alias.
try:
    import espp_odrive as odrive  # noqa: F401,E402
except ImportError:  # pragma: no cover - sibling package not on the path
    pass

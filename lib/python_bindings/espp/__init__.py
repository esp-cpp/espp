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

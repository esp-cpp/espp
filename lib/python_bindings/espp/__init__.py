"""Python bindings for the cross-platform components of espp.

The actual implementation lives in the compiled ``espp._espp`` extension
module (pybind11); this package re-exports everything from it and ships the
type stubs (``__init__.pyi``) that describe the full API.
"""

from espp._espp import *  # type: ignore # noqa: F403
from espp._espp import __version__  # noqa: F401

"""idf.py extension loader for the espp `coredump` component.

idf.py picks this file up when the component is part of the build (from a
trusted source: ESP-IDF, the project's components, EXTRA_COMPONENT_DIRS, or an
`espressif/` registry component -- otherwise it prints a warning and asks for
IDF_EXTENSION_ALLOW_UNTRUSTED=1) and gets an `idf.py coredump-usb` action with
options (--gdb, --summary, --out, --vid/--pid/--serial/--interface). The
implementation lives with the host tool in python/espp_coredump/idf_ext.py;
this file only puts that package on the path.
"""

import os
import sys

_PKG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "python")


def action_extensions(base_actions, project_path):
    if _PKG_DIR not in sys.path:
        sys.path.insert(0, _PKG_DIR)
    from espp_coredump.idf_ext import action_extensions as _action_extensions

    return _action_extensions(base_actions, project_path)

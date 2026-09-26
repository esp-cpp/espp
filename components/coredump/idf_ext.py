"""idf.py extension loader for the espp `coredump` component.

idf.py picks this file up when the component is part of the build (from a
trusted source: ESP-IDF, the project's components, EXTRA_COMPONENT_DIRS, or an
`espressif/` registry component -- otherwise it prints a warning and asks for
IDF_EXTENSION_ALLOW_UNTRUSTED=1) and gets an `idf.py coredump-usb` action with
options (--gdb, --summary, --erase, --out, --vid/--pid/--serial/--interface).
The implementation lives with the host tool in python/espp_coredump/idf_ext.py;
this file only loads that package.

The package is loaded from its file path (importlib), not by putting the
directory on sys.path: nothing else in idf.py's process sees a changed import
order, and the in-tree copy is the one used (an `espp_coredump` already
imported -- e.g. an installed wheel loaded earlier -- is left alone).
"""

import importlib
import importlib.util
import os
import sys

_PACKAGE = "espp_coredump"
_PKG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "python", _PACKAGE)


def _load_package():
    if _PACKAGE in sys.modules:
        return sys.modules[_PACKAGE]
    spec = importlib.util.spec_from_file_location(
        _PACKAGE, os.path.join(_PKG_DIR, "__init__.py"), submodule_search_locations=[_PKG_DIR])
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load {_PACKAGE} from {_PKG_DIR}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[_PACKAGE] = module
    try:
        spec.loader.exec_module(module)
    except BaseException:
        sys.modules.pop(_PACKAGE, None)
        raise
    return module


def action_extensions(base_actions, project_path):
    _load_package()
    return importlib.import_module(f"{_PACKAGE}.idf_ext").action_extensions(base_actions,
                                                                             project_path)

"""Serialisers that turn :class:`~masscompose.types.MassProperties` into the
formats consumers actually read.

URDF is the only one in v0.1. MJCF and USD are explicitly out of scope; adding
a second format here should mean a second small module beside ``urdf.py``, not
a format-agnostic abstraction over one implementation.
"""

__all__: list[str] = []

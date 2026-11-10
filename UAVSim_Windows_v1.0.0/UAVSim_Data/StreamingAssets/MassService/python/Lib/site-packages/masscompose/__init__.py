"""Compose mesh inertia and point masses into a combined COM/inertia tensor.

``__all__`` is the documented public API: the six names re-exported here are
what this package promises to keep working. The internals live in
``masscompose.types``, ``.mesh``, ``.compose``, ``.validate``, and ``.export``,
and are importable by their full path for anyone who knowingly wants one.

``mesh_units`` accepts the plain strings ``"mm"``, ``"cm"``, ``"m"``, so the
:class:`~masscompose.types.MeshUnits` enum is an implementation detail rather
than part of the surface.
"""

from masscompose.compose import compose
from masscompose.export.urdf import to_urdf_inertial
from masscompose.mesh import load_mesh
from masscompose.types import MassProperties, MeshBody, PointMass

__all__ = [
    "MassProperties",
    "MeshBody",
    "PointMass",
    "compose",
    "load_mesh",
    "to_urdf_inertial",
]

"""Load a mesh file and turn it into SI mass properties.

The one thing this module refuses to guess is units. A mesh file records
coordinates, not what they mean, so ``mesh_units`` is keyword-only with no
default: the caller states it or gets a ``TypeError``. The predecessor to this
code assumed millimetres unconditionally, which silently produced masses off by
10^9 whenever the file was in metres.
"""

from __future__ import annotations

import os
import warnings

import numpy as np
import trimesh

from masscompose.types import MeshBody, MeshUnits

__all__ = ["load_mesh"]


def _require_positive_finite(value: float, name: str) -> float:
    """Reject NaN, inf, and non-positive values for a scalar input."""
    number = float(value)
    if not np.isfinite(number):
        raise ValueError(f"{name} must be finite, got {value!r}")
    if number <= 0.0:
        raise ValueError(f"{name} must be positive, got {number}")
    return number


def load_mesh(
    path: str | os.PathLike[str],
    *,
    mesh_units: MeshUnits | str,
    density: float | None = None,
    mass: float | None = None,
    infill: float = 1.0,
    allow_open: bool = False,
) -> MeshBody:
    """Compute the mass properties of a single mesh file.

    Args:
        path: Mesh file to load (anything trimesh reads; STL is the usual case).
        mesh_units: Units the file's coordinates are in — ``"mm"``, ``"cm"``,
            or ``"m"``. Keyword-only and mandatory; there is no default.
        density: Material density in kg/m^3. Mutually exclusive with ``mass``.
        mass: Known total mass in kg, from which density is back-derived.
            Mutually exclusive with ``density``.
        infill: Fraction of the enclosed volume actually filled with material,
            in (0, 1]. Scales the effective density, so mass and inertia scale
            with it linearly. Only meaningful on the ``density`` path — a given
            ``mass`` is already the real mass.
        allow_open: Downgrade the watertight requirement to a warning. The
            volume of an open surface is whatever trimesh's divergence integral
            happens to return, so the result is an estimate, not a measurement.

    Returns:
        A :class:`MeshBody` in SI units (kg, m, kg·m²), with ``inertia`` taken
        about the mesh's own centre of mass rather than the frame origin.

    Raises:
        ValueError: if neither or both of ``density``/``mass`` are given, if
            ``infill`` is outside (0, 1], if ``infill`` is combined with
            ``mass``, if the mesh is not watertight (and ``allow_open`` is
            false), if its volume is non-positive, or if the effective density
            works out non-positive.
        TypeError: if ``mesh_units`` is omitted.
    """
    units = MeshUnits.coerce(mesh_units)

    if (density is None) == (mass is None):
        raise ValueError(
            "exactly one of density or mass must be given, not "
            + ("both" if density is not None else "neither")
        )

    infill = float(infill)
    if not np.isfinite(infill) or not 0.0 < infill <= 1.0:
        raise ValueError(f"infill must be in (0, 1], got {infill}")
    if mass is not None and infill != 1.0:
        raise ValueError(
            "infill cannot be combined with mass: mass is already the real "
            "mass of the part, so scaling it by infill would double-count the "
            "voids. Pass density with infill instead."
        )

    if density is not None:
        _require_positive_finite(density, "density")
    else:
        _require_positive_finite(mass, "mass")

    mesh = trimesh.load(path, force="mesh")

    # Scale first, then measure: everything downstream is in metres, and the
    # volume used to back-derive density has to be the scaled one.
    mesh.apply_scale(units.scale)

    if not mesh.is_watertight:
        message = (
            f"mesh '{os.fspath(path)}' is not watertight; volume, centre of "
            "mass, and inertia all assume a closed surface. Repair the mesh, "
            "or pass allow_open=True to accept an estimate."
        )
        if allow_open:
            warnings.warn(message, stacklevel=2)
        else:
            raise ValueError(message)

    volume = float(mesh.volume)
    if volume <= 0.0:
        raise ValueError(
            f"mesh '{os.fspath(path)}' has non-positive volume ({volume} m^3), "
            "which usually means inverted normals."
        )

    if density is not None:
        effective_density = float(density) * infill
    else:
        # mass / (raw_volume * scale^3), with the scaling already applied.
        effective_density = float(mass) / volume

    if effective_density <= 0.0:
        raise ValueError(
            f"effective density must be positive, got {effective_density} "
            "kg/m^3"
        )

    mesh.density = effective_density

    return MeshBody(
        mass=float(mesh.mass),
        com=np.asarray(mesh.center_mass, dtype=np.float64),
        inertia=np.asarray(mesh.moment_inertia, dtype=np.float64),
    )

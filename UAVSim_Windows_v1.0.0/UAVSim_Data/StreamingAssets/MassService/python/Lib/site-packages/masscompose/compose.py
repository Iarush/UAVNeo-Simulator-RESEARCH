"""Combine a mesh body with point masses into one set of mass properties.

1. The system centre of mass is the mass-weighted average of the stl body each point mass\  centre of mass.
2. Every body's tensor is shifted from its own centre of mass by the parallel axis theorem, and the shifted tensors add.

A point mass has no extent to simplify calculations, so its own tensor is exactly zero and its whole
contribution is the shift term. That zero is written out rather than special-
cased, so the mesh and the point masses travel the same code path.

The eigen-decomposition, quaternion conversion, and arm-length averaging that
the predecessor bolted onto the end of this are consumer concerns, and are not
here.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from masscompose.types import MassProperties, MeshBody, PointMass
from masscompose.validate import validate_mass_properties

__all__ = ["compose"]


def _parallel_axis_shift(
    inertia_own: np.ndarray, mass: float, displacement: np.ndarray
) -> np.ndarray:
    """Shift a tensor from a body's own COM to a new point.

        I_new = I_own + m (|d|^2 I_3 - d (x) d)

    Args:
        inertia_own: 3x3 tensor about the body's own centre of mass.
        mass: the body's mass, kg.
        displacement: 3-vector from the body's centre of mass to the new
            reference point, in metres. The sign of ``d`` does not matter —
            both terms are quadratic in it — but the direction is documented
            so the caller does not have to rediscover that.

    Returns:
        The 3x3 tensor about the new reference point.
    """
    d = np.asarray(displacement, dtype=np.float64)
    return inertia_own + mass * (float(d @ d) * np.eye(3) - np.outer(d, d))


def compose(
    body: MeshBody, point_masses: Sequence[PointMass]
) -> MassProperties:
    """Combine a mesh body and its point masses about the system centre of mass.

    Args:
        body: The mesh's own mass properties, with ``inertia`` about ``com`` —
            i.e. exactly what :func:`masscompose.mesh.load_mesh` returns.
        point_masses: Point masses in the *same* frame as ``body.com``. May be
            empty, in which case the result is the mesh's own properties.

    Returns:
        A :class:`MassProperties` whose ``inertia`` is about its own ``com``,
        in SI units (kg, m, kg·m²).

    Raises:
        ValueError: if the total mass is not positive, since the centre of mass
            of a massless system is not defined, or if the composed result
            fails :func:`masscompose.validate.validate_mass_properties`. The
            latter is a post-condition: reaching it means either the mesh body
            arrived with an impossible tensor of its own, or this function has
            a bug. Either way the caller gets an error instead of a plausible
            number.
    """
    masses = [float(body.mass)] + [float(p.mass) for p in point_masses]
    positions = [body.com] + [p.position for p in point_masses]
    inertias = [body.inertia] + [np.zeros((3, 3))] * len(point_masses)

    total_mass = float(np.sum(masses))
    if not np.isfinite(total_mass) or total_mass <= 0.0:
        raise ValueError(
            f"total mass must be positive, got {total_mass} kg; the centre of "
            "mass of a massless system is undefined"
        )

    # Step 1 — system centre of mass: r_sys = sum(m_i r_i) / sum(m_i).
    weighted = np.zeros(3)
    for mass, position in zip(masses, positions):
        weighted += mass * np.asarray(position, dtype=np.float64)
    system_com = weighted / total_mass

    # Step 2 — parallel axis: shift every body's own tensor to the system COM.
    inertia = np.zeros((3, 3))
    for inertia_own, mass, position in zip(inertias, masses, positions):
        displacement = system_com - np.asarray(position, dtype=np.float64)
        inertia += _parallel_axis_shift(inertia_own, mass, displacement)

    # The tensor is symmetric analytically; summing many shifts leaves float
    # noise of order 1e-18 off the diagonal. Fold it away so downstream eigen
    # solvers get the symmetry they assume.
    inertia = (inertia + inertia.T) / 2.0

    result = MassProperties(mass=total_mass, com=system_com, inertia=inertia)

    # Post-condition. The checks are microseconds against the mesh load that
    # precedes them, and the failure they catch — a tensor that is symmetric,
    # positive semi-definite, and still physically impossible — is exactly the
    # kind that survives review and shows up as a simulation that tumbles.
    try:
        validate_mass_properties(result)
    except ValueError as error:
        raise ValueError(
            f"compose() produced mass properties that are not physical: "
            f"{error}"
        ) from error

    return result

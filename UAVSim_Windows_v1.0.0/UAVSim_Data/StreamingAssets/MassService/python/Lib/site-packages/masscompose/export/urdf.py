"""Render mass properties as a URDF ``<inertial>`` block.

On the sign convention, since this is the one thing about the format that is
routinely got wrong: URDF's six ``i**`` attributes are *entries of the matrix*,
not products of inertia with their own sign rule. The spec says the element
holds "the 3x3 rotational inertia matrix ... only 6 above-diagonal elements of
this matrix are specified here", and the two reference consumers read them that
way — ``urdfdom`` stores the attributes verbatim, and the KDL parser hands them
straight to ``RotationalInertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz)``, which lays them
into the matrix unchanged.

So ``ixy`` is ``inertia[0, 1]``, which is the standard physics product of
inertia ``-integral(x y dm)`` and is *negative* for mass in the first octant.
There is no flip here, and the widespread claim that URDF wants positive
products would silently mirror every asymmetric body about its own axes.

Everything written is SI — metres, kilograms, kg·m² — which is what the rest of
the package produces and what URDF requires. There is no unit argument, because
there is no other correct answer.
"""

from __future__ import annotations

import numpy as np

from masscompose.types import MassProperties

__all__ = ["to_urdf_inertial"]

#: Significant figures written per value. Twelve round-trips to a relative
#: error near 1e-12 — three orders below the 1e-9 tolerance the validation
#: module works to — while keeping the block readable, which ``repr`` (17
#: figures on most values) does not.
_PRECISION = 12


def _fmt(value: float) -> str:
    """Format one number for an XML attribute.

    ``+ 0.0`` folds a negative zero back to positive: arithmetic that lands on
    ``-0.0`` is numerically identical to ``0.0``, and writing ``"-0"`` into a
    URDF only invites someone to go looking for the sign.
    """
    return f"{float(value) + 0.0:.{_PRECISION}g}"


def to_urdf_inertial(mp: MassProperties) -> str:
    """Render mass properties as a URDF ``<inertial>`` element.

    Args:
        mp: Mass properties in SI, with ``inertia`` about ``com`` — i.e. what
            :func:`masscompose.compose.compose` returns. The ``<origin>`` is
            written from ``com``, which is what makes the tensor's reference
            point and the frame URDF assumes agree.

    Returns:
        The ``<inertial>`` block as a string, two-space indented and with no
        trailing newline, ready to paste inside a ``<link>``.

    Raises:
        ValueError: if any value is NaN or infinite. Those would format as
            ``"nan"`` and ``"inf"``, which most URDF parsers accept without
            complaint and turn into a simulation that misbehaves much later.

    Example:
        >>> import numpy as np
        >>> from masscompose import MassProperties, to_urdf_inertial
        >>> mp = MassProperties(
        ...     mass=2.0, com=np.zeros(3), inertia=np.diag([0.1, 0.2, 0.3])
        ... )
        >>> print(to_urdf_inertial(mp))
        <inertial>
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <mass value="2"/>
          <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.3"/>
        </inertial>

    The ``rpy`` is always zero: the tensor is already expressed in the link's
    own axes, so the inertial frame differs from the link frame by a
    translation only.
    """
    mass = float(mp.mass)
    com = np.asarray(mp.com, dtype=np.float64)
    inertia = np.asarray(mp.inertia, dtype=np.float64)

    if not np.isfinite(mass):
        raise ValueError(f"mass must be finite, got {mass!r}")
    for value, name in ((com, "com"), (inertia, "inertia")):
        if not np.all(np.isfinite(value)):
            raise ValueError(
                f"{name} must be finite, got {np.array2string(value)}"
            )

    xyz = " ".join(_fmt(component) for component in com)
    return (
        "<inertial>\n"
        f'  <origin xyz="{xyz}" rpy="0 0 0"/>\n'
        f'  <mass value="{_fmt(mass)}"/>\n'
        f'  <inertia ixx="{_fmt(inertia[0, 0])}" ixy="{_fmt(inertia[0, 1])}" '
        f'ixz="{_fmt(inertia[0, 2])}" iyy="{_fmt(inertia[1, 1])}" '
        f'iyz="{_fmt(inertia[1, 2])}" izz="{_fmt(inertia[2, 2])}"/>\n'
        "</inertial>"
    )

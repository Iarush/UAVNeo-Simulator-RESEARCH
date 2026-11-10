"""Post-condition checks on mass properties.

Three things have to be true of any inertia tensor that came from a real solid:

* it is symmetric — ``I_xy`` and ``I_yx`` are the same integral;
* it is positive semi-definite — no axis has negative rotational inertia;
* it obeys the triangle inequality, ``I_xx + I_yy >= I_zz`` and permutations,
  because ``I_xx + I_yy - I_zz = 2 integral z^2 dm`` cannot come out negative.

The first two are cheap to satisfy by accident; a garbage tensor built by, say,
summing displacements from the origin instead of from the system centre of mass
is still symmetric and still PSD. The triangle inequality is the one that
catches it, which is why it is checked here rather than left to the caller.

The triangle check runs on the *principal* moments (the eigenvalues) rather than
on the diagonal of whichever frame the tensor happens to be written in. The
principal form is the stronger of the two: it implies the diagonal form in every
frame, while the diagonal form holding in one frame proves nothing — rotate
``diag([1, 1, 5])`` by 45 degrees about x and its diagonal reads ``[1, 3, 3]``,
which passes a naive ``ixx + iyy >= izz`` test while the body remains
impossible.
"""

from __future__ import annotations

import numpy as np

from masscompose.types import MassProperties, MeshBody

__all__ = ["validate_tensor", "validate_mass_properties"]

#: Slack allowed on every comparison, relative to the largest entry of the
#: tensor. Summing parallel-axis shifts leaves noise of order machine epsilon
#: times the tensor scale (~1e-16); 1e-9 sits far above that and far below any
#: violation that means something physically.
DEFAULT_RTOL = 1e-9


def _finite(value, shape: tuple[int, ...], name: str) -> np.ndarray:
    """Coerce to a float64 array of exactly ``shape``, with no NaN or inf."""
    array = np.asarray(value, dtype=np.float64)
    if array.shape != shape:
        raise ValueError(f"{name} must have shape {shape}, got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(
            f"{name} must be finite, got {np.array2string(array)}"
        )
    return array


def validate_tensor(
    inertia, *, name: str = "inertia", rtol: float = DEFAULT_RTOL
) -> np.ndarray:
    """Check that a 3x3 array could be the inertia tensor of a real solid.

    Args:
        inertia: Anything array-like of shape (3, 3), in kg·m².
        name: How to refer to the tensor in error messages, so a caller
            validating several bodies can say which one failed.
        rtol: Tolerance for every comparison, relative to the largest entry of
            the tensor. An exactly-zero tensor is checked exactly.

    Returns:
        The tensor as a float64 array, for convenience.

    Raises:
        ValueError: with a diagnosis naming the specific property that failed —
            shape, finiteness, symmetry, positive semi-definiteness, or the
            triangle inequality — and the numbers that failed it.
    """
    tensor = _finite(inertia, (3, 3), name)
    scale = float(np.abs(tensor).max())
    tol = rtol * scale

    asymmetry = float(np.abs(tensor - tensor.T).max())
    if asymmetry > tol:
        i, j = np.unravel_index(
            int(np.argmax(np.abs(tensor - tensor.T))), (3, 3)
        )
        raise ValueError(
            f"{name} is not symmetric: [{i},{j}] = {tensor[i, j]!r} but "
            f"[{j},{i}] = {tensor[j, i]!r} (differ by {asymmetry}). A tensor "
            "about a single point has only six independent components."
        )

    # Symmetrised before the decomposition so that eigvalsh — which reads only
    # the lower triangle — cannot silently disagree with the check above.
    principal = np.linalg.eigvalsh((tensor + tensor.T) / 2.0)
    smallest, middle, largest = (float(value) for value in principal)

    if smallest < -tol:
        raise ValueError(
            f"{name} is not positive semi-definite: principal moments are "
            f"{[smallest, middle, largest]}. A negative principal moment means "
            "some axis has negative rotational inertia, which no distribution "
            "of non-negative mass produces."
        )

    if smallest + middle < largest - tol:
        raise ValueError(
            f"{name} violates the triangle inequality: principal moments "
            f"{[smallest, middle, largest]} have {smallest} + {middle} < "
            f"{largest}. Since I1 + I2 - I3 = 2 integral d^2 dm over the axis "
            "of I3, no real solid can do this — the tensor was built wrong, "
            "most often by shifting inertia to the wrong reference point."
        )

    return tensor


def validate_mass_properties(
    mp: MassProperties | MeshBody, *, rtol: float = DEFAULT_RTOL
) -> None:
    """Check a whole set of mass properties: mass, centre of mass, and tensor.

    Args:
        mp: A :class:`~masscompose.types.MassProperties` or
            :class:`~masscompose.types.MeshBody`. The two carry the same three
            fields and the same invariants.
        rtol: Passed through to :func:`validate_tensor`.

    Raises:
        ValueError: if the mass is not positive and finite, if the centre of
            mass has a NaN or an inf in it, or if the tensor fails any check in
            :func:`validate_tensor`.
    """
    mass = float(mp.mass)
    if not np.isfinite(mass):
        raise ValueError(f"mass must be finite, got {mass!r}")
    if mass <= 0.0:
        raise ValueError(
            f"mass must be positive, got {mass} kg; a massless body has no "
            "centre of mass and no inertia to speak of"
        )

    _finite(mp.com, (3,), "com")
    validate_tensor(mp.inertia, rtol=rtol)

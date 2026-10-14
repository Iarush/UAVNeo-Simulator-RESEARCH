"""
mass_composer.py

Turns a frame's mass properties and a list of point masses into the
principal-axis + rotation form Unity's Rigidbody expects.

The composition math itself is no longer here. Total mass, system center of
mass, and the parallel-axis shift live in the `masscompose` package, which
this module imports; what remains is the part that is specific to this
transport and this consumer:

  * eigen-decomposition of the tensor into Rigidbody.inertiaTensor
  * the rotation-matrix -> quaternion conversion for
    Rigidbody.inertiaTensorRotation
  * the motor arm length that replaces Flight.cs's hardcoded constant

COORDINATE SYSTEM CONTRACT:
All positions (the frame's center of mass, and every node's position) MUST be
expressed in the same local coordinate system: the STL mesh's own local
coordinates, in meters, as returned by masscompose.load_mesh(). This is
typically the same local coordinate system as the drone GameObject in Unity,
assuming the STL is loaded with no additional offset/scale applied.
"""

from typing import List, Sequence

import numpy as np
from masscompose import MeshBody, PointMass, compose


def to_unity_principal_axes(inertia_tensor) -> dict:
    """
    Converts a full symmetric 3x3 inertia tensor into the principal-axis
    representation Unity's Rigidbody expects: a diagonal inertia vector
    (Rigidbody.inertiaTensor) plus a rotation (Rigidbody.inertiaTensorRotation)
    describing the axes in which the tensor IS diagonal.

    Args:
        inertia_tensor: 3x3 symmetric matrix, kg*m^2. Anything array-like --
            a MassProperties.inertia straight off compose(), or a nested list
            off the wire.

    Returns:
        dict with keys:
            "principal_moments": list[float] of length 3 (the diagonal
                values in the rotated frame) -- maps to Rigidbody.inertiaTensor.
            "rotation_matrix": list[list[float]], 3x3, a proper rotation
                (determinant +1) whose columns are the principal axes,
                expressed in the original local coordinate system. Convert
                this to a quaternion in Unity/C# for Rigidbody.inertiaTensorRotation.

    Raises:
        ValueError: if any eigenvalue is negative beyond a small numerical
                    tolerance, which would indicate a bug upstream (moment
                    of inertia can never be physically negative).
    """
    I = np.array(inertia_tensor, dtype=np.float64)

    # compose() already symmetrizes its result and checks positive
    # semi-definiteness, so on the normal path the next two steps are no-ops.
    # They stay because this function is also called directly with hand-built
    # tensors, and eigh's symmetric-matrix assumption should not be taken on
    # trust.
    I = (I + I.T) / 2.0

    eigenvalues, eigenvectors = np.linalg.eigh(I)  # eigh: for symmetric matrices, returns ascending eigenvalues

    tolerance = -1e-6 * max(1.0, np.abs(eigenvalues).max())
    if np.any(eigenvalues < tolerance):
        raise ValueError(
            f"Computed a negative principal moment of inertia ({eigenvalues}), "
            "which is not physically possible. This indicates a bug in the "
            "upstream mass/geometry calculation, not a valid drone configuration."
        )
    eigenvalues = np.clip(eigenvalues, 0.0, None)  # clamp tiny negative noise to exactly 0

    # eigh does not guarantee a proper rotation (determinant +1); it can
    # return a reflection (determinant -1). Fix by flipping the sign of one
    # eigenvector column, which does not change the physics (moment of
    # inertia along a flipped axis is identical) but fixes the handedness
    # needed for a valid rotation/quaternion.
    if np.linalg.det(eigenvectors) < 0:
        eigenvectors[:, 0] *= -1

    return {
        "principal_moments": eigenvalues.tolist(),
        "rotation_matrix": eigenvectors.tolist(),
    }


def _rotation_matrix_to_quaternion(R: np.ndarray) -> tuple:
    """
    Converts a 3x3 proper rotation matrix (determinant +1) to a quaternion
    (x, y, z, w), matching Unity's Quaternion convention.

    Standard robust conversion (avoids division-by-near-zero issues that a
    naive formula has near certain rotation angles).
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    return float(x), float(y), float(z), float(w)


def compute_arm_length(system_com: List[float], nodes: Sequence[PointMass]) -> float:
    """
    Computes the average distance from the system center of mass to every
    node whose name starts with "motor" (case-insensitive). Used to
    replace Flight.cs's hardcoded armLength constant with a value derived
    from the user's actual motor placement.

    If no nodes are named as motors, falls back to averaging over ALL
    nodes -- better than crashing, though the caller should ideally ensure
    motor nodes are named correctly (e.g. "motor_fr", "motor_fl", etc.)
    upstream in the UI/C# node-placement code.
    """
    com = np.array(system_com, dtype=np.float64)
    motor_nodes = [n for n in nodes if (n.name or "").lower().startswith("motor")]
    relevant = motor_nodes if motor_nodes else nodes

    distances = [np.linalg.norm(np.asarray(n.position) - com) for n in relevant]
    return float(np.mean(distances))


def compute_unity_payload(frame: MeshBody, nodes: Sequence[PointMass]) -> dict:
    """
    Full pipeline: composes the system, eigen-decomposes the tensor, and
    computes armLength -- returning exactly the fields needed for the
    45-byte UDP response payload (see udp_listener.py's docstring).

    Args:
        frame: the mesh's own mass properties, as returned by
               masscompose.load_mesh().
        nodes: point masses in the SAME local coordinate system as the
               frame's center of mass.

    Returns dict with keys: "com" (list[3]), "principal_moments" (list[3]),
    "quaternion" (tuple x,y,z,w), "arm_length" (float).

    Raises:
        ValueError: if nodes is empty. masscompose.compose() will happily
                    compose a frame on its own, but armLength is a mean over
                    the node positions, and the mean of nothing is NaN -- a
                    payload Unity accepts and then behaves strangely on. This
                    is a requirement of this transport, not of the math.
    """
    if not nodes:
        raise ValueError(
            "nodes list is empty -- at least one point mass (e.g. battery) is required"
        )

    system = compose(frame, nodes)
    principal = to_unity_principal_axes(system.inertia)
    quaternion = _rotation_matrix_to_quaternion(np.array(principal["rotation_matrix"]))
    arm_length = compute_arm_length(system.com, nodes)

    return {
        "com": system.com.tolist(),
        "principal_moments": principal["principal_moments"],
        "quaternion": quaternion,
        "arm_length": arm_length,
    }

r"""Generate a 7-inch-class true-X quadcopter frame STL plus its mass-node list.

CAD frame (what CustomFrameLoader / the mass service expect):
  millimetres in the STL, Z-up, +Y forward, +X right, origin at the frame centre.
Mass nodes are written in the same frame but in METRES (CustomFrameLoader contract).

Run with the MassCompose venv (needs trimesh + manifold3d for the boolean union):
  ..\..\MassCompose\.venv\Scripts\python.exe make_drone_7inch.py
"""

import json
import math
from pathlib import Path

import numpy as np
import trimesh

OUT_DIR = Path(__file__).resolve().parent
STL_PATH = OUT_DIR / "drone_7inch.stl"
NODES_PATH = OUT_DIR / "drone_7inch_nodes.json"

# ---- Geometry (mm) -----------------------------------------------------------
WHEELBASE = 315.0                      # motor-to-motor diagonal, typical 7" long-range
MOTOR_R = WHEELBASE / 2.0              # 157.5 mm from centre to each motor axis
MOTOR_XY = MOTOR_R / math.sqrt(2.0)    # 111.37 mm on each axis (true X)
PROP_DIAMETER = 7 * 25.4               # 177.8 mm - clearance check only

ARM_T = 6.0      # arm thickness (z 0..6)
ARM_W = 14.0     # arm width
PAD_D = 32.0     # motor mount pad diameter (fits 16x16 / 19x19 2806.5 bolt pattern)
BOT_T = 2.5      # bottom plate (z -2.5..0)
TOP_T = 2.0      # top plate
STACK_H = 26.0   # standoff height between arms and top plate
PLATE_X, PLATE_Y = 50.0, 120.0
STANDOFF_D = 6.0

DENSITY_KG_M3 = 1600.0   # carbon-fibre/epoxy laminate


def box(extents, center):
    m = trimesh.creation.box(extents=extents)
    m.apply_translation(center)
    return m


def cylinder(radius, height, center):
    m = trimesh.creation.cylinder(radius=radius, height=height, sections=64)
    m.apply_translation(center)
    return m


def build_frame() -> trimesh.Trimesh:
    parts = []
    corners = [(-1, 1), (1, 1), (-1, -1), (1, -1)]  # FL, FR, BL, BR (x sign, y sign)

    for sx, sy in corners:
        # Arm: from 20 mm out to the motor axis, rotated onto the diagonal.
        r0, r1 = 20.0, MOTOR_R
        arm = box([r1 - r0, ARM_W, ARM_T], [(r0 + r1) / 2.0, 0.0, ARM_T / 2.0])
        angle = math.atan2(sy, sx)
        arm.apply_transform(trimesh.transformations.rotation_matrix(angle, [0, 0, 1]))
        parts.append(arm)
        # Motor mount pad at the arm tip.
        parts.append(cylinder(PAD_D / 2.0, ARM_T, [sx * MOTOR_XY, sy * MOTOR_XY, ARM_T / 2.0]))

    # Central body: bottom plate, top plate, four standoffs.
    parts.append(box([PLATE_X, PLATE_Y, BOT_T], [0, 0, -BOT_T / 2.0]))
    top_z = ARM_T + STACK_H
    parts.append(box([PLATE_X, PLATE_Y, TOP_T], [0, 0, top_z + TOP_T / 2.0]))
    for sx in (-1, 1):
        for sy in (-1, 1):
            parts.append(cylinder(STANDOFF_D / 2.0, STACK_H, [sx * 20.0, sy * 50.0, ARM_T + STACK_H / 2.0]))

    frame = trimesh.boolean.union(parts, engine="manifold")
    frame.merge_vertices()
    assert frame.is_watertight and frame.is_winding_consistent and frame.volume > 0
    return frame


def build_nodes():
    """(name, type, x, y, z [m], mass [kg]). Types: 0 motor, 1 battery, 2 FC, 3 other."""
    mm = 0.001
    arm_top = ARM_T
    nodes = []
    corners = {"FL": (-1, 1), "FR": (1, 1), "BL": (-1, -1), "BR": (1, -1)}
    for tag, (sx, sy) in corners.items():
        x, y = sx * MOTOR_XY, sy * MOTOR_XY
        nodes.append((f"motor_{tag}", 0, x * mm, y * mm, (arm_top + 13.0) * mm, 0.048))   # 2806.5 1300KV
    for tag, (sx, sy) in corners.items():
        x, y = sx * MOTOR_XY, sy * MOTOR_XY
        nodes.append((f"prop_{tag}", 3, x * mm, y * mm, (arm_top + 30.0) * mm, 0.0065))   # 7" tri-blade
    for tag, (sx, sy) in corners.items():
        d = 70.0 / math.sqrt(2.0)  # 70 mm out along the arm, on top of it
        nodes.append((f"esc_{tag}", 3, sx * d * mm, sy * d * mm, (arm_top + 3.0) * mm, 0.008))  # 45A single ESC
    nodes.append(("flight_controller", 2, 0.0, 0.0, (arm_top + 10.0) * mm, 0.008))           # 30x30 F7
    nodes.append(("battery", 1, 0.0, -5.0 * mm, (arm_top + STACK_H + TOP_T + 20.0) * mm, 0.330))  # 6S 2200 LiPo, top-mounted
    nodes.append(("receiver", 3, 0.0, -45.0 * mm, (arm_top + 14.0) * mm, 0.003))             # ELRS / nano RX
    nodes.append(("camera", 3, 0.0, 58.0 * mm, (arm_top + 14.0) * mm, 0.006))                # micro FPV cam
    return nodes


def main():
    frame = build_frame()
    frame.export(STL_PATH)

    clearance = 2 * MOTOR_XY - PROP_DIAMETER
    nodes = build_nodes()
    vol_m3 = frame.volume * 1e-9
    frame_mass = vol_m3 * DENSITY_KG_M3
    data = {
        "stl_path": str(STL_PATH),
        "density_kg_m3": DENSITY_KG_M3,
        "frame_volume_m3": vol_m3,
        "frame_mass_kg": frame_mass,
        "prop_gap_between_adjacent_props_mm": clearance,
        "nodes": [dict(name=n, type=t, x=x, y=y, z=z, mass_kg=m) for n, t, x, y, z, m in nodes],
    }
    NODES_PATH.write_text(json.dumps(data, indent=2))

    print(f"STL: {STL_PATH}  faces={len(frame.faces)} watertight={frame.is_watertight}")
    print(f"bounds (mm): {np.round(frame.bounds, 2).tolist()}")
    print(f"volume = {frame.volume:.1f} mm^3 -> frame mass {frame_mass * 1000:.1f} g at {DENSITY_KG_M3} kg/m^3")
    print(f"adjacent prop tip gap = {clearance:.1f} mm")
    total = frame_mass + sum(n[5] for n in nodes)
    print(f"nodes = {len(nodes)}, node mass = {sum(n[5] for n in nodes) * 1000:.1f} g, AUW = {total * 1000:.1f} g")


if __name__ == "__main__":
    main()

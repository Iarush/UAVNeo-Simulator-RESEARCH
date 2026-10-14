# Python mass-properties backend — development history

Reconstructed from Claude Code session transcripts. The backend has no transcripts
of its own; everything here was recovered from sessions whose working directory was
the Unity project or the `drone-student` repo.

## Sources

| Session | Date | What it contributes |
|---|---|---|
| `4a91e903` | 2026-08-01 | Full `udp_listener.py` source; the `ConnectionResetError` fix and its testing |
| `63f30480` | 2026-07-27 | Full `mesh_properties.py` source; the mm→m unit fix |
| `a30b06be` | 2026-07-27/28 | Unity-side consumption of the response; units evidence; live log values |
| `44bace66` | 2026-07-28→08-01 | Confirms no Python exists in the Unity repo |

**Sessions checked and found to contain nothing on this subject:** `dc6b7298`
(summer-course lab exercises), `b5d78f3e`, `8a09ccb1`, `ae941104`, `dd82fd97`,
`ee7a5279`.

### Service location

```
C:\Users\Owner\Documents\Projects\uav-neo-installer\drone-student\library\simulation\mass-properties\
```

Established from file paths in tool calls in `63f30480` and `4a91e903`. Confirmed
in `4a91e903`: *"The file is `udp_listener.py` in the `mass-properties` directory of
the **other** working directory — the Unity repo has no Python."*

### Recovery status of each file

| File | Status |
|---|---|
| `mesh_properties.py` | **Full source recovered** (`63f30480`, 101 lines) |
| `udp_listener.py` | **Full source recovered** (`4a91e903`, 197 lines, pre-fix state) |
| `mass_composer.py` | **Full source recovered** (read directly from disk, 281 lines) |

> **Note on provenance.** `mesh_properties.py` and `udp_listener.py` were recovered
> from session transcripts. `mass_composer.py` appears in **no** transcript — it was
> read directly from disk on 2026-08-22 and documented in Section 3 from the source
> itself. Its development history therefore remains unrecorded: this document describes
> what the module *does*, not how it came to be that way. Sections 1, 2 and 4–6 are
> still transcript-derived and cite session IDs.

---

## 1. Architecture decisions and reasoning

### Why the math is split across three files

Stated directly in the `mesh_properties.py` module docstring (`63f30480`):

> ```
> This module is intentionally standalone: it has no knowledge of motors,
> batteries, UDP protocols, or Unity. It answers exactly one question:
> "given this STL frame and this density, what are its mass properties?"
>
> Combining this frame data with motor/battery point masses (parallel axis
> theorem, system COM, eigen-decomposition for Unity) happens in a separate
> module, mass_composer.py.
> ```

So the split is: **geometry** (`mesh_properties`) → **composition with point masses**
(`mass_composer`) → **transport** (`udp_listener`). The transport layer's own
docstring reinforces the same separation, describing itself as running "the geometry
+ composition math (`mesh_properties.py` + `mass_composer.py`)".

`udp_listener.py` also separates its pure logic from its I/O, stated in the
`handle_request` docstring (`4a91e903`):

> ```
> Pure function, no socket I/O -- kept separate from run_server() so it can
> be unit tested directly.
> ```

> **Flag.** No transcript records a *decision moment* for the three-way split — no
> discussion of alternatives, no rejected designs. These docstrings are the entire
> surviving rationale.

### Why a binary wire protocol instead of JSON

> **Flag — not answered anywhere in the transcripts.** No session contains any
> discussion of JSON versus binary, or any comparison of serialisation formats. I
> searched all 15 transcripts for this and found nothing. Do not infer a rationale
> from the sections below.

The closest adjacent rationale is about *what the response carries*, not how it is
encoded — from the `udp_listener.py` docstring (`4a91e903`):

> ```
> This keeps ALL eigen-decomposition and rotation-matrix math on the
> Python side (already written and tested in mass_composer.py), so the
> C# side only needs to unpack floats and assign them -- no linear
> algebra required in Unity.
> ```

That explains why the response ships *principal moments plus a quaternion* rather
than a raw 3×3 tensor. It does not explain the choice of a binary framing.

### The port choice (5070)

From `udp_listener.py` (`4a91e903`):

```python
# Separate port from the existing drone_core sync channel (5065), to avoid
# any conflict with the existing student-facing protocol. Unity's C# side
# must send its request to this exact port.
LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5070
```

The recorded reason is **avoiding a clash with drone_core's channel on 5065**.

> **Flag.** Why `5070` specifically rather than `5066` or any other free port is not
> recorded. Only the "must differ from 5065" constraint is.

Note `LISTEN_IP` is `0.0.0.0` (all interfaces) while the Unity client targets
`127.0.0.1` (per `CustomFrameLoader.cs`).

The same "separate channel" reasoning drives the error header value:

> ```
> Error response, header byte value 255 (reserved for this service --
> distinct from drone_core's own Header.error=0, since this is a separate
> protocol channel)
> ```

### Where the service lives relative to drone_core and Unity

From the `udp_listener.py` docstring (`4a91e903`):

> ```
> This is intentionally a SEPARATE process/port from the existing drone_core
> sync channel (port 5065). It does not import or depend on drone_core_sim.py
> or physics_sim.py at all -- it only needs to agree with Unity on the same
> Header byte values and wire format.
> ```

So: it sits **inside the `drone-student` repo tree** (`library/simulation/mass-properties/`),
alongside drone_core, but is deliberately decoupled from it at the code level — no
imports, separate process, separate port. Its only coupling to Unity is the wire
format. Unity holds no Python; confirmed independently in `44bace66`:

> *"`mass_composer.py` and `udp_listener.py` do not exist anywhere in the
> repository — I searched all `.py` files tracked and untracked, and
> `git ls-files "*.py"` returns nothing."*

### Default density values and where they came from

Two density constants appear.

**1. Server-side fallback** — `udp_listener.py` (`4a91e903`):

```python
# Fallback used only if a request somehow sends density <= 0 (shouldn't
# happen if the C# side validates its UI input, but the server should not
# trust the network blindly). PETG, a common 3D-printed frame material.
FALLBACK_DENSITY_KG_PER_M3 = 1250.0
```

**2. Documentation guidance** — `mesh_properties.py` `load_frame` docstring (`63f30480`):

> ```
> density: Material density in kg/m^3 (e.g. ~1250 for PETG, ~1600 for
>          carbon fiber composite, ~2700 for aluminum).
> ```

`1250` is also the value used in every recorded test run, and matches the
`TestDensity` in the Unity `DebugFrameTestSetup`.

> **Flag.** No source, datasheet, or derivation is given for `1250`, `1600`, or
> `2700` in any transcript. They appear already-written in the docstring the first
> time the file is read. The only stated justification for 1250 is the inline
> comment "PETG, a common 3D-printed frame material."

---

## 2. The wire protocol

Reproduced from the `udp_listener.py` module docstring, cross-checked against its
`_parse_request` / `_build_result_payload` / `_build_error_payload` implementations
(`4a91e903`). All multi-byte values little-endian.

### Request — Unity → Python, header 40

| Offset | Type | Field |
|---|---|---|
| 0 | `uint8` | header = **40** |
| 1 | `float32` | density, kg/m³ (must be > 0) |
| 5 | `uint32` | `pathByteCount` |
| 9 | `byte[]` | STL path, UTF-8 |
| 9+n | `uint32` | `nodeCount` |
| … | node[] | 17 bytes each |

Per node: `uint8` nodeType, then `float32` x, y, z, massKg.
Node type values: `0=motor, 1=battery, 2=flight_controller, 3=companion_computer`.

The parser reads the type byte and the 16-byte remainder separately:

```python
[node_type] = struct.unpack_from("<B", data, offset); offset += 1
chunk = data[offset:offset + 16]
x, y, z, mass = struct.unpack("<ffff", chunk)
nodes.append(MassNode(x, y, z, mass, label=f"{label}_{i}"))
```

Unrecognised type bytes are not rejected — they become `f"unknown_type_{node_type}"`.
Because `arm_length` is defined over labels starting with `"motor"`, an unknown type
simply never counts as a motor.

### Response — success, exactly 45 bytes, header 41

| Offset | Type | Field |
|---|---|---|
| 0 | `uint8` | header = **41** |
| 1, 5, 9 | `float32` | com x, y, z |
| 13, 17, 21 | `float32` | principal Ixx, Iyy, Izz |
| 25, 29, 33, 37 | `float32` | quaternion x, y, z, w |
| 41 | `float32` | armLength |

Packed in one call: `struct.pack("<Bfffffffffff", ...)`. The docstring states the
size derivation as `1 + 3*4 + 3*4 + 4*4 + 1*4 = 45 bytes`, and the mapping:

> ```
> principalIxx/Iyy/Izz map directly to Rigidbody.inertiaTensor.
> The quaternion maps directly to Rigidbody.inertiaTensorRotation.
> ```

`armLength` is documented in the same block as *"avg distance from COM to motor
nodes (label starts with `"motor"`)"* — see Section 3 for empirical confirmation.

### Response — error, header 255

| Offset | Type | Field |
|---|---|---|
| 0 | `uint8` | header = **255** |
| 1 | `uint32` | `messageByteCount` |
| 5 | `byte[]` | message, UTF-8 |

```python
def _build_error_payload(message: str) -> bytes:
    message_bytes = message.encode("utf-8")
    return struct.pack("<BI", HEADER_ERROR, len(message_bytes)) + message_bytes
```

Reached on: empty packet, wrong header byte, or **any** exception raised during
parse/compute — the handler wraps the whole body in a deliberate catch-all so that
bad input reaches the client as a readable string instead of killing the process.

### Revisions

**One substantive revision is recorded, and it changed the response's units, not its
byte layout.**

- **2026-07-27 (`63f30480`) — response units became metres.** `load_frame` originally
  assumed the STL was already in metres. Against the real test STL this produced a
  bounding box of `[152.5, 142.2, 50.8]` "metres" and a mass of **186 million kg**.
  The fix added an unconditional `mesh.apply_scale(0.001)`, after which the same file
  gave `[0.1525, 0.1422, 0.0508]` m and **0.186 kg**. Everything the service returns
  has been SI-metre-denominated since.

- **2026-07-27 (`a30b06be`) — a client-side misreading of those units, corrected.**
  Not a protocol change, but it is the event that pinned the units down. Unity briefly
  applied its own mm→m scale to `com`, which drove a real offset to floating-point
  zero (`com=(0.0000, 0.0000, 0.0000)` in the log). The scale was removed once live
  telemetry showed `moments` (0.0047–0.0100) and `armLength` (0.2128) arriving in
  already-sane SI ranges with no scaling applied. The next run logged
  `com=(0.0000, -0.0236, 0.0002)`.

> **Flag.** No revision to the *byte layout* is recorded anywhere. Headers 40/41/255,
> the 17-byte node, and the 45-byte response are identical in every transcript that
> touches them, from 2026-07-27 through 2026-08-01. This is consistent with the layout
> having been fixed before the earliest surviving session — but the transcripts cannot
> prove there were no earlier revisions, because they do not reach back that far.

---

## 3. What each module does

### `mesh_properties.py` — full source recovered (`63f30480`)

**Inputs:** `load_frame(stl_path: str, density: float) -> dict`

**Outputs:** a dict with

| Key | Type | Notes |
|---|---|---|
| `mass` | `float` | kg |
| `centroid` | `list[float]` len 3 | metres, in the mesh's original local frame |
| `inertia_tensor` | `list[list[float]]` 3×3 | kg·m², **about the centroid, not the origin** |

**Key assumptions**, all explicit in code or docstring:

- **Uniform density.** Stated in the module docstring.
- **Watertight mesh.** Enforced, not assumed: `if not mesh.is_watertight: raise ValueError(...)`,
  with the message noting that "Volume and mass-property calculations require a
  watertight mesh."
- **Positive density.** `if density <= 0: raise ValueError`.
- **Positive volume.** `if volume <= 0` raises, with the message "This usually means
  the mesh normals are inverted."
- **STL is in millimetres.** Applied unconditionally: `mesh.apply_scale(0.001)`.

**The actual math** is delegated to `trimesh`, not hand-rolled:

```python
volume = mesh.volume        # trimesh: signed volume via tetrahedron integration
mass = volume * density
centroid = np.array(mesh.center_mass, dtype=np.float64)
mesh.density = density
inertia_tensor = np.array(mesh.moment_inertia, dtype=np.float64)
```

An in-code comment explains why `mesh.density` is set rather than rescaling
afterward: trimesh "assumes density=1 unless mesh.density is set, so we set it
explicitly here to get a correctly-scaled tensor directly, rather than manually
rescaling a unit-density tensor afterward." Another comment identifies
`mesh.center_mass` as "the `r_f` in the project's math."

Also has a CLI: `python mesh_properties.py <stl_path> <density_kg_per_m3>`, printing
the result dict as indented JSON.

### `mass_composer.py` — full source recovered (read directly, 281 lines)

**Inputs:** `compute_unity_payload(frame: dict, nodes: List[MassNode]) -> dict`, where
`frame` is the dict from `mesh_properties.load_frame` and `nodes` are point masses built
by `udp_listener` from the wire payload.

```python
@dataclass
class MassNode:
    x: float; y: float; z: float
    mass_kg: float
    label: str = ""   # "optional, for UI/diagnostics only — not used in physics math"
```

`MassNode.position` is a `@property` returning `np.array([x, y, z])`.

**Outputs:** exactly the four fields the 45-byte response needs —

| Key | Shape | Feeds |
|---|---|---|
| `com` | list[3] | response bytes 1–12 |
| `principal_moments` | list[3] | bytes 13–24 → `Rigidbody.inertiaTensor` |
| `quaternion` | tuple (x, y, z, w) | bytes 25–40 → `Rigidbody.inertiaTensorRotation` |
| `arm_length` | float | bytes 41–44 |

**Key assumptions**

- **One shared coordinate frame**, stated as a contract in the module docstring:

  > All positions (the frame's centroid, and every node's position) MUST be expressed in
  > the same local coordinate system: the STL mesh's own local coordinates, in meters …

  This is asserted, never verified — nothing checks that node coordinates arrived in the
  STL frame rather than Unity's.
- **Nodes are dimensionless point masses.** Stated in-code: a point mass "has zero 'self'
  inertia (all its mass is at a single point, no size), so its own inertia tensor is the
  zero matrix." A 50 g motor is modelled as a point, not a cylinder.
- **Validated, raising `ValueError`:** `frame_mass > 0`; `nodes` non-empty ("at least one
  point mass (e.g. battery) is required"); every `n.mass_kg > 0`; no principal moment
  negative beyond tolerance.
- **Not validated:** finiteness of any coordinate, node count, or whether motor nodes are
  labelled at all (see `arm_length` below).

---

#### The math

**1. System mass and centre of mass**

A mass-weighted average, with the frame treated as one more point mass sitting at its own
centroid:

```
m_sys = m_f + Σ mᵢ
r_sys = (m_f · r_f + Σ mᵢ · rᵢ) / m_sys
```

```python
weighted_sum = frame_mass * frame_centroid + (node_masses[:, None] * node_positions).sum(axis=0)
system_com = weighted_sum / total_mass
```

The non-obvious part is `node_masses[:, None]`: it reshapes the `(N,)` mass vector to
`(N, 1)` so NumPy broadcasts it against the `(N, 3)` position array **row-wise**, scaling
each node's whole position vector by that node's own mass. Without the inserted axis the
shapes would not broadcast. `.sum(axis=0)` then collapses the N rows into a single
3-vector.

**2. Parallel axis theorem**

```
I_new = I_own + m · (|d|² · E₃ − d ⊗ d)
```

```python
d_squared = np.dot(d, d)
shift = mass * (d_squared * np.eye(3) - np.outer(d, d))
return inertia_own + shift
```

`np.outer(d, d)` is the outer product `d dᵀ`. The bracketed term is the standard
displacement tensor: it adds `m|d|²` to the moment about every axis, then subtracts back
the component **along** `d` — because mass displaced along an axis contributes nothing to
rotation about that same axis.

Worth noting, since the docstring is specific about direction ("from the object's own
centroid TO the new reference point"): **the formula is invariant under `d → −d`**,
because `|−d|² = |d|²` and `(−d)(−d)ᵀ = d dᵀ`. Getting the sign backwards would be
harmless. The code is consistent regardless, using `system_com − source_point` at both
call sites.

**3. Assembling the system tensor**

The frame's tensor arrives from `load_frame` already about the frame's *own* centroid, so
it must be shifted to the *system* COM:

```python
frame_displacement = system_com - frame_centroid
inertia_total = _parallel_axis_shift(frame_inertia_own, frame_mass, frame_displacement)
```

Each node then contributes through the same helper, passing an explicit zero matrix as its
self-inertia — so its entire contribution is the shift term:

```python
for n, m in zip(nodes, node_masses):
    node_displacement = system_com - n.position
    inertia_total += _parallel_axis_shift(np.zeros((3, 3)), m, node_displacement)
```

Passing `np.zeros((3,3))` rather than special-casing point masses keeps one code path and
puts the physical assumption in view at the call site.

The result is a full symmetric 3×3 about the system COM, in the STL's local frame —
generally **not** diagonal. Off-diagonal products of inertia appear whenever the mass
distribution is not symmetric about the coordinate planes.

**4. Eigen-decomposition → principal moments + rotation**

Unity cannot accept a full tensor: `Rigidbody.inertiaTensor` is a diagonal 3-vector and
`Rigidbody.inertiaTensorRotation` is the rotation into the frame where the tensor *is*
diagonal. That is exactly an eigenvalue problem.

```python
I = (I + I.T) / 2.0
eigenvalues, eigenvectors = np.linalg.eigh(I)
```

Three non-obvious steps:

- **Defensive symmetrization.** The tensor is symmetric in exact arithmetic, but it was
  built by repeatedly `+=`-ing shift matrices, and float accumulation "can leave tiny
  asymmetries (e.g. 1e-14 differences between `I[0][1]` and `I[1][0]`)". `eigh` *assumes*
  symmetry and reads only one triangle, so an unsymmetrized input would silently use
  whichever half it happens to read. Averaging costs nothing and removes the ambiguity.
- **`eigh`, not `eig`.** For a real symmetric matrix, `eigh` guarantees real eigenvalues
  and an orthonormal eigenvector basis; the general `eig` can return complex values and a
  non-orthogonal basis. An inertia tensor is symmetric positive semi-definite, so `eigh`
  is the correct routine. Its eigenvalues come back **ascending**, which means
  `principal_moments` are sorted smallest-first and have **no correspondence to the
  original x/y/z axes**. The rotation matrix is what carries that information.
- **Negative-eigenvalue guard, scaled to the tensor's magnitude:**

  ```python
  tolerance = -1e-6 * max(1.0, np.abs(eigenvalues).max())
  if np.any(eigenvalues < tolerance):
      raise ValueError(…"not physically possible"…)
  eigenvalues = np.clip(eigenvalues, 0.0, None)
  ```

  A moment of inertia can never be negative, so a genuinely negative eigenvalue means an
  upstream bug — the error message says exactly that. The tolerance is *relative*, so it
  stays meaningful for both gram-scale and kilogram-scale drones. Values inside the
  tolerance band are clamped to exactly zero rather than passed on as `-1e-19`.

**5. Forcing a proper rotation**

```python
if np.linalg.det(eigenvectors) < 0:
    eigenvectors[:, 0] *= -1
```

`eigh` returns an orthonormal basis but promises nothing about **handedness** — the
eigenvector matrix may have determinant −1, i.e. be a reflection. A quaternion can only
represent a proper rotation, so a reflection would yield a meaningless one. Negating a
single column flips the determinant's sign, and it is physically free: the moment of
inertia is the quadratic form `vᵀ I v`, so an axis and its negation give an identical
moment. The in-code comment makes the same argument.

The columns of `eigenvectors` are the principal axes expressed in the local frame, so the
matrix maps principal-frame coordinates → local coordinates — which is the convention
`Rigidbody.inertiaTensorRotation` expects.

**6. Rotation matrix → quaternion**

`_rotation_matrix_to_quaternion(R)` is the standard four-branch trace method (Shepperd's).
It computes an intermediate `s` from the trace, or from whichever of `R[0,0]`, `R[1,1]`,
`R[2,2]` is largest:

```python
trace = R[0, 0] + R[1, 1] + R[2, 2]
if trace > 0:
    s = 0.5 / np.sqrt(trace + 1.0)
    w = 0.25 / s
    x = (R[2, 1] - R[1, 2]) * s
    …
```

The branching exists because the naive single-formula conversion divides by a quantity
that approaches zero near certain rotation angles (notably 180°), which would blow up
numerically. Choosing the branch with the largest denominator keeps the division
well-conditioned for every input. The docstring names this: it "avoids
division-by-near-zero issues that a naive formula has near certain rotation angles."

Returned as `(x, y, z, w)` — Unity's component order, matching the wire layout at response
offsets 25/29/33/37.

**7. `arm_length`**

```python
motor_nodes = [n for n in nodes if n.label.lower().startswith("motor")]
relevant = motor_nodes if motor_nodes else nodes
distances = [np.linalg.norm(n.position - com) for n in relevant]
return float(np.mean(distances))
```

Mean Euclidean distance from the **system COM** — not the origin, not the frame centroid —
to each motor node. Label matching is a case-insensitive prefix test, which is what makes
`udp_listener`'s `f"{label}_{i}"` naming work: type byte 0 becomes `motor_0`, `motor_1`, …

This confirms the earlier empirical derivation exactly. Against the live values from
`a30b06be` — four motors at `(±0.15, 0, ±0.15)` m, system COM `(0.0000, 0.0002, 0.0236)` —
per-motor distances are `0.19616, 0.19616, 0.22943, 0.22943`, mean **0.212792**, matching
the logged `armLength=0.2128` to all four printed digits. Distance-from-origin would have
given `0.2121`.

> **Gotcha — the silent fallback.** With no motor-labelled nodes, `relevant` becomes
> **all** nodes, so `arm_length` averages in the battery and electronics too. The
> docstring calls it "better than crashing," but it yields a plausible wrong number rather
> than an error. In the shipped pipeline this is unreachable for valid requests —
> `udp_listener` always labels type byte 0 as `motor` — but an unrecognised type byte
> becomes `unknown_type_{n}_{i}`, which never matches. A request whose motors all carried
> an unknown type byte would take this path.

---

#### `compute_final_payload` — dead code describing a different protocol

The module's last function is not imported by `udp_listener` and is unreachable in the
shipped pipeline. It returns the **raw** tensor's six independent components instead of
the eigen-decomposed form:

```python
return {
    "com": system["system_com"],
    "Ixx": I[0][0], "Iyy": I[1][1], "Izz": I[2][2],
    "Ixy": I[0][1], "Ixz": I[0][2], "Iyz": I[1][2],
}
```

Its docstring describes an alternative wire design — a message it calls
`physics_mass_properties_result`, in which "Unity receives the raw 6 values and does the
eigen-decomposition itself in C#" — and closes: "This wrapper supports the former; call
`to_unity_principal_axes` separately if you choose the latter design."

**The shipped protocol is the latter.** `compute_unity_payload` does the decomposition in
Python and sends 3 moments + 4 quaternion components, exactly as `udp_listener`'s docstring
argues it should. So this function is a vestige of a rejected design, retained alongside
the one that won.

> **Flag.** Whether it is kept deliberately as a documented alternative or simply never
> deleted is not recorded anywhere. It is unreferenced, and the message name it cites does
> not exist in the shipped protocol.

#### Stale docstring

`to_unity_principal_axes` returns a `rotation_matrix` and instructs the caller to "Convert
this to a quaternion in Unity/C# for `Rigidbody.inertiaTensorRotation`." That stopped being
true once `compute_unity_payload` began calling `_rotation_matrix_to_quaternion` itself.
The conversion happens in Python; C# only unpacks four floats. Same stale-docstring pattern
as `mesh_properties.py`.

#### No self-test

Unlike `mesh_properties.py`, this module has **no `if __name__ == "__main__"` block** and
no test file accompanies it — confirming from the source what Section 4 could previously
only infer from the transcripts.

### `udp_listener.py` — full source recovered (`4a91e903`)

**Inputs:** UDP datagrams on `0.0.0.0:5070`, `recvfrom(65535)`.
**Outputs:** a 45-byte success payload or a variable-length error payload, back to the
sender's address.

Structure:

- `_parse_request(data)` → `(density, stl_path, nodes)`. Explicitly length-checks both
  the path (`if len(stl_path_bytes) != path_byte_count`) and each node chunk
  (`if len(chunk) != 16`), raising `ValueError` with the expected-vs-got counts.
- `_build_result_payload(payload)` / `_build_error_payload(message)` — pure packers.
- `handle_request(data)` — pure function, no socket I/O, so it can be unit-tested
  directly. Validates length ≥ 1 and the header byte, then parses, computes, packs.
- `run_server()` — bind and loop.

**Density fallback lives here, not in `mesh_properties`.** If a request carries
`density <= 0`, the listener warns and substitutes `1250.0` rather than letting
`load_frame` raise:

```python
if density <= 0:
    print(f">> WARNING: received non-positive density ({density}); "
          f"falling back to {FALLBACK_DENSITY_KG_PER_M3} kg/m^3")
    density = FALLBACK_DENSITY_KG_PER_M3
```

**Assumption worth noting:** the pipeline is `load_frame(stl_path, density)` →
`compute_unity_payload(frame, nodes)`. The STL path arrives as a UTF-8 string from
the network and is opened directly, with no validation beyond the length check.

**Logging** is `print(">> ...")` throughout; the module has no `logging` setup.

---

## 4. Verification and testing

> **Flag — this is the weakest section, and the gap is the finding.** Searching all
> 15 transcripts for `pytest`, `test_mass`, analytic-inertia checks, two-point-mass
> symmetry, and quaternion round-trips returns **no hits** outside of boilerplate.
> No formal test suite for the Python math appears anywhere in the record. None of
> the three specific validations named in the brief — analytically-known cube
> inertia, two-point-mass symmetry, quaternion round-trip — is recorded as having
> been performed.

What *was* verified:

### Unit-scale validation of `mesh_properties.py` (`63f30480`, 2026-07-27)

The only check against an expected value. Run against
`C:/Users/Owner/Downloads/Part Studio 1 - Part 1.stl` at density 1250 kg/m³:

| | Before `apply_scale(0.001)` | After |
|---|---|---|
| Bounding box extents | `[152.51875305, 142.18869019, 50.79999924]` | `[0.15251875, 0.14218869, 0.0508]` m |
| Mass | ≈186,000,000 kg | **0.186 kg** |

The expected range (`~[0.15, 0.14, 0.05]`) was specified in advance by the user, and
the result matched. This validates the *unit conversion*, and gives a plausibility
check on mass. It does not validate the inertia tensor.

### Socket-resilience testing of `udp_listener.py` (`4a91e903`, 2026-08-01)

Thorough, but it tests the server loop — **not the math**. A harness reproduced the
Unity lifecycle (clients that send and vanish before the reply lands):

| Scenario | Result |
|---|---|
| Control — original loop, unmodified | **Died on the 2nd `recvfrom`**, WinError 10054 |
| Fixed, source suppression active | 33 clients / 3 batches, **zero** resets raised, alive |
| Fixed, suppression stubbed out (simulates Linux) | **23** `ConnectionResetError`s caught and ignored, 0 tracebacks, alive |
| Full cycle, real header-40 request | 3× request → valid 45-byte response → disconnect, alive |

The two layers were verified independently — row 2 proves the ioctl works, row 3
proves the handler alone suffices. The session also notes a process caveat: an early
"DEAD" reading turned out to be an orphaned test server holding the port, and the
figures above are from clean re-runs on fresh ports.

### End-to-end round trip, explicitly *not* a math check (`4a91e903`)

Because the original reference STL was gone from Downloads, a watertight 100 mm cube
was generated and sent through the real service. Three complete cycles returned valid
45-byte responses with `com=(0.0321, 0.0321, 0.0449)` and `arm=0.2193`. The session's
own conclusion is unambiguous:

> *"plausible, but I did not check the math against hand arithmetic, so this is not
> the mass sanity check listed as open in CLAUDE.md."*

Note that a known-geometry cube at known density was in fact generated here — the
input for an analytic inertia check existed, and the check was not run.

### Diagnostic bisection of the inertia rotation (`a30b06be`, 2026-07-27)

Not a Python-side test, but the only empirical probe of the quaternion's effect. The
inertia rotation was temporarily forced to `Quaternion.identity` in Unity to isolate
it. User result: *"it definitely improved."*

> **Flag.** "Improved" is the entire recorded outcome — no numbers, and the
> investigation moved on to other causes without resolving whether the composition
> order was wrong. This is evidence that something about the rotation handling was
> off; it is not a diagnosis, and it does not establish whether the fault was in
> `mass_composer.py` or in Unity's consumption of its output.

---

## 5. Bugs and gotchas on the Python side

### Fixed

**1. STL units — `mesh_properties.py` assumed metres** (`63f30480`, 2026-07-27).
CAD tools export STLs in millimetres. The unconverted mesh produced a 152 m drone
massing 186 million kg. Fixed with `mesh.apply_scale(0.001)` immediately after load.

**2. `ConnectionResetError` (WinError 10054) killed the server** (`4a91e903`, 2026-08-01).
On Windows, a client that closes its socket after receiving its reply provokes an
ICMP port-unreachable, which Winsock surfaces on the server's *next* `recvfrom`. It
refers to an already-gone client and means nothing for UDP correctness, but unhandled
it terminated `run_server()`. Reproduced as a control case before fixing: the
unmodified loop died on the second `recvfrom`.

Fixed in two independent layers: `try/except ConnectionResetError → log + continue`
around **both** `recvfrom` and `sendto` (the client can also vanish before the reply
goes out), plus source-level suppression via `SIO_UDP_CONNRESET`.

### Gotchas discovered while fixing #2

**3. `hasattr(socket, "SIO_UDP_CONNRESET")` is always `False`** — on every platform,
Windows included. CPython's `socket` module defines only `SIO_RCVALL`,
`SIO_KEEPALIVE_VALS`, and `SIO_LOOPBACK_FAST_PATH`. A guard written that way is a
permanent no-op that looks correct and does nothing. This was the form originally
specified in the task; it was caught because the fix was tested rather than assumed.

**4. Supplying the constant literal to `sock.ioctl` is worse than doing nothing.**
CPython's `socket.ioctl` accepts only that same three-command whitelist and raises
`ValueError: invalid ioctl command 2550136844` for anything else. `ValueError` is not
an `OSError`, so an `except OSError` guard does not catch it — the first attempted fix
crashed the server *at startup*. The session states this plainly: *"That's a real
failure I introduced and caught in testing, not a hypothetical."*

**5. The suppression is only reachable through `ctypes` → `WSAIoctl`.** The final
implementation calls Winsock directly with the literal control code, derived in a
comment as `SIO_UDP_CONNRESET = _WSAIOW(IOC_VENDOR, 12) = 0x80000000 | 0x18000000 | 12 = 0x9800000C`,
and sizes the `SOCKET` handle per-architecture:

```python
socket_t = (ctypes.c_uint64 if ctypes.sizeof(ctypes.c_void_p) == 8
            else ctypes.c_uint32)
```

It is wrapped in a bare `except Exception` so that no ctypes/Winsock failure can
prevent startup — it degrades to a warning and falls back on the exception handler.
Guarded by `sys.platform != "win32"` rather than `hasattr`.

### Standing gotchas, not bugs

**6. `mesh_properties.py`'s docstring is stale.** Flagged in `63f30480` immediately
after the units fix: the docstring still says units are "assumed to be meters" and
that mm conversion "should happen before calling", which stopped being true when
`apply_scale(0.001)` went in. The session asked whether to update it or whether the
scale factor was meant to be temporary. **No answer is recorded in any transcript,
and no follow-up edit appears.** Assume it is still stale.

**7. The mm→m conversion is unconditional.** Any STL genuinely exported in metres
will be silently scaled down by 1000×.

**8. Unknown node-type bytes are accepted silently**, becoming
`unknown_type_{n}`. They contribute mass and position but never count as motors for
`arm_length`.

**9. The catch-all in `handle_request` is deliberate** — the comment states that
malformed requests, missing files and non-watertight meshes "should all reach the
client as a readable error message rather than silently crashing this long-running
server process." It prints a traceback server-side and returns `str(e)` to the client.

---

## 6. Open questions / unfinished items

**1. `mass_composer.py`'s development history is unrecorded.** The module is now read
and documented (Section 3), but it appears in no transcript, so there is no record of
why it was written the way it was, what alternatives were weighed, or whether it was
ever reviewed. `44bace66` records it as never code-reviewed, and nothing since
contradicts that. Two concrete loose ends the source itself raises: the unreferenced
`compute_final_payload`, and the stale `to_unity_principal_axes` docstring.

**2. The inertia principal-axis quaternion is unresolved.** Forcing it to identity
"definitely improved" flight behaviour (`a30b06be`), which points at either the
composition order in Unity or the convention on the Python side. The investigation
moved on without concluding. The composition currently in Unity is
`rotation = ZUpToYUpRotation * rawRotation`, described in-session as a guess at both
axis handling and order, with no way to verify against the service.

**3. No analytic validation of the math has ever been run.** No cube-inertia check
against `m(a²+b²)/12`, no two-point-mass symmetry test, no quaternion round-trip.
A watertight cube of known size and density was generated in `4a91e903` and put
through the service, and the resulting numbers were explicitly *not* checked against
hand arithmetic.

**4. The response carries no mass.** Raised in `a30b06be`: the 45-byte response is
com + moments + quaternion + armLength only, so Unity's `Rigidbody.mass` stays at
the prefab default (logged as `2.0000`) no matter what the custom frame actually
weighs. `mesh_properties.load_frame` *does* compute and return `mass`, so the value
exists server-side and is discarded at the wire boundary. Whether that is intentional
is not recorded, and no decision appears in any transcript.

**5. The stale `mesh_properties.py` docstring** (Gotcha 6) — question asked, never
answered.

**6. Whether the `>> (debug)` print idiom should become real logging.** Raised in
`4a91e903`: *"The module has no `logging` setup — it's all `print(">> ...")` — so I
matched that idiom... Say the word if you'd prefer a real logger."* No response
recorded.

**7. `LISTEN_IP = "0.0.0.0"` binds all interfaces** while the client connects to
`127.0.0.1`. Not raised as a question in any transcript; noted here because the
asymmetry is visible in the source and its intent is unrecorded.

**8. Physics-model mismatch that no backend fix can address.** From `a30b06be`:
Unity's motor mixing uses a single scalar `armEffective` plus hardcoded sign arrays
and never reads individual motor positions, so it assumes a symmetric X-configuration
quad regardless of the real geometry. However precise the service's COM and inertia
outputs are, an asymmetric frame cannot be represented downstream. This constrains
what the backend can usefully deliver.

---

*Compiled 2026-08-22 from read-only analysis of Claude Code session transcripts.
Sessions were parsed programmatically; user messages and assistant text were
extracted and tool-result noise dropped, with Python source recovered separately
from file-write and file-read tool payloads.*

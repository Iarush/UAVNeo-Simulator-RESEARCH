# UAVNeo Simulator — Custom Drone Frame Import

## Project

Lets a user import a custom `.stl` drone frame instead of flying the hardcoded default drone. Unity sends the STL path, a material density, and a list of mass nodes (motors/battery/FC/companion computer) to a local Python service over UDP; the service returns centre of mass, principal moments of inertia, an inertia principal-axis rotation, and an arm length. Unity applies those to the drone's `Rigidbody`, swaps in the STL mesh, and repositions the propellers.

## Final goal

A student can **import their own drone frame (`.stl`) into UAVSim and fly it**. It should fly the way that physical frame would, given its geometry, its material, and where the motors, battery and electronics sit. In short: **CAD file in → physically plausible custom quadcopter flying in the sim.**

### Target user journey

1. On the main menu, the student opens the import panel (the custom drone page). A **Preset Drone config** dropdown at the top lists saved presets.
2. They browse to an `.stl` and enter a material density (kg/m³), or pick a preset from the dropdown, which fills in the path, density and node rows. Any filled value can still be edited.
3. On the same panel, below the STL and density, they enter type, position and mass for each node (4 motors plus any battery, flight controller and companion computer) in a scrolling table. It starts with 6 rows; **+ Add Row** / **- Remove Row** change that between 4 and 12. Positions are in the CAD file's Z-up frame, in metres.
4. **Build Drone** validates the STL, density and rows together and stores the statics; **Back** closes the panel to the menu.
4a. **Save as preset:** the student can save the current STL path, density and shown node rows (however many there are) under a name. The preset is written to disk and appears in the dropdown next time, including in later sessions.
5. They press **Begin**. The flight scene loads, the default drone is replaced with their mesh, the `Rigidbody` gets the computed COM / principal inertia / arm length, and the propellers move to the motor positions.
6. The drone hovers stably at sensible throttle and responds to roll/pitch/yaw the way its mass distribution predicts.
7. On any failure (service down, bad STL, invalid layout) it logs a clear reason and falls back to the default drone. It never crashes or flies a broken model.

### Definition of done

- [ ] Custom-drone path reaches flight from the main menu with no Editor tricks. (No longer blocked: Back is wired to `ImportPageUI.Close`. On 2026-10-13, an Editor run calling Build Drone, then `Close()`, then `BeginSimulation()` flew the 100 mm cube with the expected values. Still to do: click through it by hand.)
- [ ] `DebugFrameTestSetup` / `TestSetup` removed; the normal gameplay route no longer touches custom-frame statics.
- [x] Mass sanity check passes: a known-volume cube at known density matches hand arithmetic for mass, COM and inertia. (Passed 2026-10-13 end to end through Unity: `tools/test-data/cube100.stl` at 1000 kg/m³, four 0.05 kg motors at CAD (±0.15, ±0.15, 0), battery 0.5 kg. With the battery at the origin: 1.7 kg, COM 0, moments 0.006167 / 0.006167 / 0.010667. With the battery at (0.1, 0, 0): COM x 0.02941, moments 0.006167 / 0.009696 / 0.014196. Both match hand arithmetic to the printed digits.)
- [ ] A symmetric X-quad with correct inputs hovers without drift or climb; `Flight.cs` audited against the unbalanced/climbing symptom. (Hover part passed 2026-10-13 via Unity MCP, for the cube quad and the default drone. Over 20 s: altitude error under 2.3 mm, pitch, roll and yaw change 0.000°, drift 0.0000 m, all four motors equal at √(m·g/4/kf). Caveat: an off-centre-COM control run hovered just as perfectly, so this test cannot detect imbalance; see DEVLOG 2026-10-13. Climb part fixed 2026-10-14: gains scale with inertia, the light 1.2 kg and 0.28 kg frames that climbed away now hover (altitude error ≤ 2.5 mm, tilt 0.000°), and the default drone is bit-identical; see DEVLOG 2026-10-14. Still to do: a hand click-through.)
- [ ] `mass_composer.py` / `udp_listener.py` are in version control and reviewed, or their location is documented and pinned. (In version control: the copies Unity runs are at `UAVSim/Assets/MassService~/mass-properties/` since `282d53c`, and `masscompose`, `numpy` and `trimesh` are pinned in `tools/mass-service-requirements.txt`. Still open: a code review.)
- [ ] Lab code and `send_pcmd` meaning unchanged; flight timing is realistic for all drones (see `.plans/real-flight-physics-spec.md`). (Lab regression is run once after all phases, owner decision 2026-10-20. Not run yet.)
- [ ] Saving a preset writes its STL path, density and shown node rows (4–12) to disk; after restarting Unity the preset shows in the **Preset Drone config** dropdown. (Built. Verified 2026-10-13 via Unity MCP: a fresh Play session lists the saved presets, and after an Editor restart the file written by the previous Editor session listed and loaded both presets with every value. The file had been deleted and restored from an unchanged copy in between, so it was not on disk during the restart itself. Still to do: a hand click-through.)
- [ ] Selecting a preset fills the path, density and node rows exactly as they were saved, and Begin then flies it through the same path as a hand-entered import. (Verified 2026-10-13 via Unity MCP: a 7-node preset restored every value and the row count, then Build → Back → Begin flew it at 1.850 kg with COM matching hand arithmetic. Still to do: a hand click-through.)
- [ ] A preset whose STL file is missing or moved fails with a clear message in the panel and does not crash; the student can still re-browse and resave. (Verified 2026-10-13 via Unity MCP. Still to do: a hand click-through.)

## Architecture

### Python side — lives OUTSIDE this repo, in the MassCompose repo

**The backend files to use are in `C:\Users\Owner\Documents\MassCompose\src\mass-properties`.** No `.py` files exist in this repository (searched tracked + untracked, excluding `UAVSim/Library/`). The only in-repo evidence of the service is the client: `UAVSim/Assets/Scripts/Drone/CustomFrameLoader.cs:140` (`127.0.0.1:5070`).

| File | Role |
|---|---|
| `src/mass-properties/udp_listener.py` | Entry point. UDP server on `0.0.0.0:5070`; parses the request, calls `masscompose.load_mesh(path, mesh_units="mm", density=…)`, builds the 41/255 response. |
| `src/mass-properties/mass_composer.py` | Unity adapter only: eigen-decomposition → `inertiaTensor`, rotation matrix → quaternion, motor arm length. Calls `masscompose.compose`. |

- **`masscompose` is a PyPI package, a dependency like `numpy` and `trimesh`. Do not copy `src/masscompose/`.** The two files import it (`from masscompose import PointMass, load_mesh, compose, MeshBody`). Install with `pip install masscompose`; it pulls in `numpy>=1.24` and `trimesh>=4.0`. `pyproject.toml` in MassCompose excludes `src/mass-properties` from the package ("legacy backend"), so these two files are run as scripts, not installed.
- **Published version checked:** PyPI `masscompose` 0.1.0 (the only release). Its wheel was diffed against `MassCompose/src/masscompose/` and all seven `.py` files are identical, so the published package matches what the service files call.
- **Python version problem (unresolved):** the PyPI metadata says `requires_python >=3.13`. The student installer builds a Python **3.9** venv (`drone-venv`, numpy 1.26.2 pinned), so `pip install masscompose` there will be refused as things stand. The local `pyproject.toml` declares no `requires-python`, so the published metadata came from a different build config. Whether the code actually runs on 3.9 is untested. Fixing this means either lowering `requires-python` and republishing, or running the service under a 3.13 interpreter.
- **Run command** (import-checked, not run end to end): `C:\Users\Owner\Documents\MassCompose\.venv\Scripts\python.exe C:\Users\Owner\Documents\MassCompose\src\mass-properties\udp_listener.py`. That venv is Python 3.13.14 with `masscompose` (editable install of local source), `numpy` 2.5.2, `trimesh` 5.0.0. It is the only environment known to run the service.
- **MassCompose is its own git repo** (HEAD `3e52487`). At last check both service files, plus `masscompose/__init__.py` and `types.py`, had UNCOMMITTED changes.
- **Wire protocol is unchanged** versus the older copy: same headers 40/41/255, port 5070, and struct formats (full diff of `udp_listener.py` read). New in this version: `PointMass` rejects negative or non-finite node masses, so those now come back as a 255 error instead of a NaN response.
- **Superseded copy — do not use:** `uav-neo-installer/drone-student/library/simulation/mass-properties/` (`mesh_properties.py`, `mass_composer.py`, `udp_listener.py`). It still has the old `mesh_properties.load_frame` and an inlined composer. `docs/dev-history-python.md` describes that older copy.
- **Liveness probe:** any packet whose first byte is not 40 gets a 255 error reply, so a one-byte datagram tells the client whether the service is up without touching the locked protocol (verified by calling `handle_request` directly).
- **Auto-launch is BUILT and Editor-tested (built player still pending):** Unity starts the service from a bundled Python 3.13 when the import panel opens. Unity runs its **own copy** of the two scripts, in `UAVSim/Assets/MassService~/mass-properties/`, not the MassCompose originals. See the section "Auto-launch the Python backend with a bundled Python".
- **MassCompose is out of scope.** Do not edit or commit anything in the MassCompose repo, including the `compose.py` SyntaxWarning. Its local source is not what was published to PyPI. Python-side changes go in the in-repo copy.

### C# side (all paths under `UAVSim/Assets/Scripts/`)

- `Drone/CustomFrameLoader.cs` — the whole import pipeline. Builds/sends the UDP request, parses the response, loads the mesh, replaces the drone visual, applies mass properties, repositions propellers. Runs from `Awake()`, `[DefaultExecutionOrder(-100)]`.
- `Drone/StlMeshLoader.cs` — static ASCII+binary STL parser → `Mesh`; owns the mm→m scale and the Z-up↔Y-up conversion helpers.
- `Drone/Flight.cs` — flight model. `ApplyCustomMassProperties()` receives the computed values; `Start()` writes them onto the `Rigidbody`, sizes the motors and builds the allocation from the motor positions about the COM. Control is PX4 v1.17.0's attitude and rate loops plus allocation, sub-stepped at 250 Hz (see Flight model). The inertia gain scaling of 2026-10-14 is gone. Audited 2026-10-13, see `docs/flight-audit-2026-10-13.md`.
- `Drone/FlightConfig.cs` — loads `StreamingAssets/Flight/uav-neo.params` and `flight-sim.json` with built-in fallbacks; holds the PX4 stock gains.
- `Drone/PropellerSpin.cs` — propeller visuals; `SetPropellerPositions(Vector3[])` accepts FL/FR/BL/BR.
- `Drone/DebugFrameTestSetup.cs` — temporary test harness, see Known cruft.
- `Drone/MotorQuadrants.cs` — shared FL/FR/BL/BR contract. `ClassifyUnity` holds the one copy of the corner rule (Unity-local Y-up); `Classify` converts from CAD Z-up and calls it; `TryValidateMotorLayout` is the single validator used by both the UI and `Awake()`.
- `UI/ImportPageUI.cs` — the single hand-built custom drone panel (`CustomDroneImportPanel`): browse for STL, density entry, and the **Build Drone** button (`OnBuildDroneClicked`), which checks the STL exists and the density, calls `NodePlacementUI.TryStoreNodes`, and only then writes `SelectedSTLPath` / `SelectedDensity`. Also hosts the preset row: the **Preset Drone config** dropdown (`OnPresetSelected`) and **Save Preset** (`OnSavePresetClicked`).
- `Drone/MassServiceLauncher.cs` — static class, creates no GameObjects. `EnsureRunning()` probes port 5070 and, if nothing answers, starts the bundled `python.exe` and polls until it is ready. It kills only the process it started, on `Application.quitting`. `Current` exposes the state for the UI.
- `Editor/MassServiceBuildStep.cs` — `IPostprocessBuildWithReport`; copies `Assets/MassService~` into a Windows player's `StreamingAssets/MassService`.
- `Drone/DronePresetStore.cs` — static; reads and writes the preset file (see Drone presets). Pure data, no UI; the import panel calls it. Also holds the `DronePreset` / `DronePresetNode` data classes.
- `UI/NodePlacementUI.cs` — on the same GameObject as `ImportPageUI`. Binds the node rows (12 hand-built under `All Nodes`, `Node00`–`Node11`), the Add/Remove Row buttons and the paste box, all inside the `NodeScroll` scroll view. Parses the shown rows (hidden rows are ignored), validates, and writes `SelectedNodes` in `TryStoreNodes()`. Does not launch the sim. (Merged from a separate `NodePlacementPanel` on 2026-10-13.)
- `UI/MainMenu.cs` — menu; `BeginSimulation()` loads the flight scene. Finds and shows/hides the import panel.
- `UI/CustomDroneBuilderUI.cs` — ORPHANED, do not revive. See Hard constraint.
- `UI/DronePreviewOrbit.cs` — orbit/click-raycast helper for a mesh preview. Currently unused; out of MVP scope.
- `Plugins/SimpleFileBrowser/` — third-party file picker (`autoReferenced: true`, so usable from `Assembly-CSharp`).

## Flight model (real-flight-physics, phases 0–1 done 2026-10-20)

Spec: `.plans/real-flight-physics-spec.md`; results in DEVLOG 2026-10-20.

- **Config:** `FlightConfig` reads `UAVSim/Assets/StreamingAssets/Flight/uav-neo.params` (the real drone's PX4 v1.17.0 params, made by `tools/extract-px4-params.py` from QGC `.tlog` files) and `flight-sim.json` (sim-only values, each with its source). Rate-loop gains are PX4 stock, never the file's. Exceptions decided by the owner: `MC_ROLL_P` / `MC_PITCH_P` 6.5, and yaw 1.0 = 1 rad/s.
- **Motors:** sized so hover is at `MPC_THR_HOVER`; first-order lag (12.5 / 25 ms); one force per motor at its position; yaw reaction from spin direction. Motor index order is FR 0, FL 1, BL 2, BR 3 (`MotorQuadrants.MotorIndex`).
- **Motor positions:** custom frames use the propeller transforms (placed by `CustomFrameLoader`). The stock drone uses an X layout from `armLength` (0.16 m), because its propeller visuals sit at about ±0.72 m and do not match its physics.
- **Still old (phases 2–5):** position hold, banking, altitude PD, kinematic launch, drags and brake. Lab regression is deferred to the end.
- **Tests:** `UAVSim/Assets/Tests/PlayMode/FlightConfigTests.cs`, `FlightMotorTests.cs` (the `Baseline` category is the slow 20 s, 5-frame hover trace).

## Wire protocol (LOCKED — do not change either side)

UDP to `127.0.0.1:5070`, receive timeout 5000 ms (`CustomFrameLoader.cs:44`, `:140`, `:143`).
All multi-byte values little-endian (`BinaryWriter` default). Built at `CustomFrameLoader.cs:220-241`.

### Request

| Offset | Type | Value |
|---|---|---|
| 0 | `byte` | header = **40** (`:41`) |
| 1 | `float32` | density (kg/m³) |
| 5 | `uint32` | path byte length |
| 9 | `byte[]` | STL path, UTF-8 |
| 9+len | `uint32` | node count |
| … | node[] | 17 bytes each |

Per node (17 bytes): `byte` type, `float32` x, `float32` y, `float32` z, `float32` massKg (`:231-238`). Buffer sizing at `:223` hardcodes `count * 17`.

### Response — success, exactly 45 bytes (`:43`, `:178-182`)

| Offset | Type | Field |
|---|---|---|
| 0 | `byte` | header = **41** (`:42`) |
| 1, 5, 9 | `float32` | centre of mass x, y, z |
| 13, 17, 21 | `float32` | principal moments Ixx, Iyy, Izz |
| 25, 29, 33, 37 | `float32` | inertia rotation quaternion x, y, z, w |
| 41 | `float32` | arm length |

Rejected unless: all values finite, quaternion `sqrMagnitude > 0`, all three moments `> 0`, `armLength > 0` (`:197-202`). On any failure it logs and falls back to the default drone.

### Response — error

`byte` 255 (`byte.MaxValue`) at offset 0, `uint32` message length at 1, UTF-8 message from 5 (`:154-171`). Requires `length >= 5` and `messageLength <= length - 5`.

### Enum is part of the protocol

`CustomFrameLoader.NodeType` (`:16-22`) is declared `: byte` with pinned explicit values — `Motor = 0, Battery = 1, FlightController = 2, OffboardComputer = 3`. It is written to the socket as its raw byte at `:233`. **Reordering or inserting a value silently changes the wire protocol.**

## Coordinate conventions

- **`SelectedNodes[].position` must be in the STL's Z-up frame, in METERS.** Not Unity Y-up, not millimetres. `CustomFrameLoader.cs:90-93` applies `ConvertZUpToYUp` to them on the way back in, and the comment at `:92` states they are already metre-scale so no mm scaling is applied.
- **The loaded mesh ends up in Unity Y-up, in metres.** Both conversions happen inside `StlMeshLoader.AddTriangle`, the single funnel for the ASCII and binary parse paths.
- Conversion sites:
  - Z-up → Y-up rotation, applied to normal + all 3 vertices: `StlMeshLoader.cs:137-140`
  - mm → m scale, applied exactly once per triangle: `StlMeshLoader.cs:146-148` (constant at `:14`)
  - Service `com` converted Z-up → Y-up, already metres: `CustomFrameLoader.cs:188`
  - Service rotation composed: `rotation = ZUpToYUpRotation * rawRotation`, `CustomFrameLoader.cs:194`
  - Inverse (Unity Y-up → STL Z-up), for UI-picked points heading to the service: `StlMeshLoader.cs:34-37`
- **The loaded mesh is MIRRORED relative to the CAD source.** `ZUpToYUpRotation` (`StlMeshLoader.cs:22`) is `Quaternion.AngleAxis(90f, Vector3.right)` — a proper rotation, determinant +1. CAD exporters are right-handed Z-up; Unity is left-handed Y-up. A determinant +1 transform cannot flip handedness, so the geometry is reinterpreted mirrored. Deliberate and documented at `StlMeshLoader.cs:16-21`. See Open questions.
- **Quadrants use CAD X and Y only; Z is ignored.** `MotorQuadrants.Classify` and `CustomFrameLoader.cs:104-106` convert first, then test the Unity `.z`/`.x`, which resolve to CAD `y`/`x`. So: `+Y = forward`, `+X = right`, `+Z = up and irrelevant to corner assignment`.
- Boundaries are **asymmetric and origin-based**: `isFront` is `y >= 0` (so `y == 0` is FRONT), `isLeft` is `x < 0` (so `x == 0` is RIGHT). A motor at the origin classifies FRONT-RIGHT.
- The split planes are `x = 0` / `y = 0` in the **CAD file's own origin** — NOT the computed COM and NOT the mesh bounds centre. A model exported with its origin at a corner puts all four motors in one quadrant. `RepositionPropellers` never receives `com`.
- A plus-configuration quad (motors on the axes) **cannot be classified** by this logic.

## Integration contract

**There is NO entry-point method on `CustomFrameLoader`.** Do not look for one; do not add one without deciding to change this contract.

The contract is: **populate the statics, then load the flight scene.** The UDP request fires from `Awake()` (`CustomFrameLoader.cs:46`) when the drone prefab is instantiated, ordered by `[DefaultExecutionOrder(-100)]` (`:13`).

Full public surface of `CustomFrameLoader` — everything else is `private`:

| Member | Line |
|---|---|
| `enum NodeType : byte` | `:16` |
| `struct NodeData { NodeType type; Vector3 position; float massKg; }` | `:24-30` |
| `static string SelectedSTLPath` | `:33` |
| `static float SelectedDensity` | `:36` |
| `static readonly List<NodeData> SelectedNodes` | `:39` |

`Awake()` sequence: `TryGetMassProperties` (UDP) → `StlMeshLoader.TryLoadStl` → `ReplaceDefaultVisual` → `Flight.ApplyCustomMassProperties` → `RepositionPropellers`.

Downstream public methods: `Flight.ApplyCustomMassProperties(Vector3 com, Vector3 principalMoments, Quaternion inertiaRotation, float newArmLength)` (`Flight.cs:460`); `PropellerSpin.SetPropellerPositions(Vector3[])` (`PropellerSpin.cs:97`).

**The custom drone panel does not start the simulation.** Build Drone (`ImportPageUI.OnBuildDroneClicked`) stores `SelectedNodes` via `NodePlacementUI.TryStoreNodes`, then path + density, and stops. The only route to flight is the `Begin` button in `Main.unity` (GameObject `936562977`), wired directly to `MainMenu.BeginSimulation` (`MainMenu.cs:69`), which calls `SceneManager.LoadScene` at `:106`.

`Awake()` validates the motor layout via `MotorQuadrants.TryValidateMotorLayout` before any socket traffic (`CustomFrameLoader.cs:57-67`) and falls back to the default drone on failure. The pipeline is therefore **quadcopter-only**: exactly 4 motors, one per quadrant. This makes `RepositionPropellers`' own duplicate/missing checks (`:114`, `:121`) unreachable in practice.

**Pattern to copy:** `NodePlacementUI.TryStoreNodes` — clear `SelectedNodes`, add each node with its position **verbatim** (already CAD Z-up metres, no conversion), then stop. Do NOT copy `CustomDroneBuilderUI.OnConfirmClicked`: it converts via `ConvertYUpToZUp` (correct only for points picked off a Y-up preview mesh, which no longer exists) and ends in `BeginSimulation()`.

**Modification policy for `CustomFrameLoader.cs`:** treat as change-by-approval only. It is the whole import pipeline and the only implementation of the wire protocol, so edits carry protocol risk. Ask before editing; do not refactor it opportunistically. The wire protocol itself is LOCKED regardless (see above).

## Auto-launch the Python backend with a bundled Python — BUILT, Editor-tested; built player pending

Decided and built 2026-10-13 on branch `feature/bundled-mass-service`. Goal: the student never starts the mass service by hand. Unity starts it when the import panel opens, from a Python 3.13 interpreter shipped inside the build, and stops it on quit.

**Build it:** `powershell -ExecutionPolicy Bypass -File tools\build-mass-service.ps1 [-Clean]`. Pins are in `tools/mass-service-requirements.txt`. The script downloads the embeddable zip, runs pip `--target` (platform pinned to win_amd64/cp313, so any host Python with pip works; `-BuildPython` defaults to the MassCompose venv interpreter, which is only used to run pip and is not modified), writes the `._pth` file, runs `compileall`, then runs the cube smoke test under the embedded interpreter.

**Verified 2026-10-13 (in the project):**
- The build script runs. The smoke test matches hand arithmetic. The bundle is 75.8 MB unpacked.
- Real UDP socket under the embedded interpreter: ready in about 0.8 s, bound to `127.0.0.1` only. The 1-byte probe got a 30-byte 255 reply.
- `--parent-pid`: killing the parent process stopped the service within 5 s.
- All three new C# files compile in the Editor (Unity 6000.4.5f1).
- **Editor play-test (via Unity MCP) passed:**
  - Opening the import panel through `MainMenu.ShowCustomDroneBuilder()` spawned the service with `--parent-pid` set to the Editor's PID. It was ready after 1321 ms, and the panel showed `Mass service: ready`. The status box is 56 px tall and one line is about 14.5 px, so two lines fit.
  - The 100 mm cube at 1000 kg/m³ plus the five nodes, then Begin, gave `[CustomFrameLoader] Mass properties: com=(0,0,0) moments=(0.006167,0.006167,0.010667) armLength=0.2121`. `Flight` applied mass 1.7 kg, and the propellers moved to (±0.15, 0, ±0.15).
  - Leaving Play mode killed the service: no bundled `python.exe` was left and port 5070 was free. No errors came from the launcher.

**Not yet verified:** a built player, a machine with no Python, and Windows Firewall/SmartScreen behaviour.

**Why this route.** Student Python is a WSL2 3.9 venv; the sim is a Windows `.exe`; PyPI `masscompose` needs Python >= 3.13 (see Architecture). A bundled interpreter needs nothing from the student's machine.

**Rejected:** launching through `wsl.exe` (venv discovery, orphaned Python inside WSL when Unity dies); requiring a student-installed Windows Python (extra manual step); a PyInstaller exe (one build per OS, antivirus false positives); porting the math to C# (kept as the long-term alternative; its size estimate was made against the older installer copy and must be redone against `masscompose`).

**Scope limit:** Windows only. python.org publishes the embeddable package for Windows only, so Mac/Linux builds would need a different answer. Not covered here.

### Bundle layout

```
UAVSim/Assets/MassService~/            tilde = ignored by the Unity importer (no .meta spam, no plugin import of .pyd/.dll)
  python/                              embeddable CPython 3.13.14 + Lib/site-packages (numpy, trimesh, masscompose)
  mass-properties/                     udp_listener.py, mass_composer.py  (copied from MassCompose)
```

- At runtime in a build: `<Name>_Data/StreamingAssets/MassService/`, filled by an `IPostprocessBuildWithReport` step that copies the folder. In the Editor: `Application.dataPath + "/MassService~"`. One resolver function picks the right one.
- `python/` is a build output and is git-ignored. The `mass-properties/` copy is committed (decided: it is not pulled from MassCompose).
- The `~` folder has been exercised in a running Editor (2026-10-13; see Verified above). The post-build copy compiles but has not yet been exercised in a build.

### Build recipe (steps 1–5 verified on 2026-10-13)

1. Download `https://www.python.org/ftp/python/3.13.14/python-3.13.14-embed-amd64.zip` (11 MB) and unzip into `python/`.
2. With any Python 3.13 on win_amd64: `python -m pip install --target python/Lib/site-packages --only-binary=:all: masscompose==0.1.0`. This resolved to numpy 2.5.4 and trimesh 5.1.1. Pin all three in a requirements file for reproducibility.
3. Replace `python/python313._pth` (stock content is only `python313.zip` and `.`) with these four lines:
   ```
   python313.zip
   .
   Lib\site-packages
   ..\mass-properties
   ```
   **Required.** With a `._pth` file present the script's own folder is NOT on `sys.path`, so without the last line even `import udp_listener` fails with `ModuleNotFoundError` (reproduced). Do not rely on `PYTHONPATH` here (untested).
4. Copy `udp_listener.py` and `mass_composer.py` into `mass-properties/`.
5. Launch command: `python\python.exe mass-properties\udp_listener.py`. It works from any working directory (tested from `/`).
6. Untested nice-to-have: pre-compile with `python -m compileall` so the first run doesn't compile everything.

**Verification run (scratch folder, not the project).** 100 mm cube at 1000 kg/m³, battery 0.5 kg at the origin, four 0.05 kg motors at (±0.15, ±0.15, 0), through `udp_listener.handle_request` under the embedded interpreter: 45-byte response, header 41, COM (0, 0, 0), principal moments 0.0061667 / 0.0061667 / 0.0106667 (equal to hand arithmetic), armLength 0.21213. This also doubles as a partial pass of the "known cube vs hand arithmetic" definition-of-done item, for the Python side only.

**Not covered by that run:** the real UDP socket under the embedded interpreter (since covered, see Verified above), any Windows Firewall prompt, antivirus or SmartScreen behaviour on the bundled `python.exe`, a clean machine without Python or the VC++ runtime (the zip ships `vcruntime140*.dll`; older Windows 10 without the Universal CRT is untested), and a cold file cache.

**Measured.** Bundle 65 MB unpacked (numpy plus `numpy.libs` is about 43 MB of that), 26.3 MB deflated. Interpreter start, imports and one request took about 0.9–1.3 s on a warm cache (3 runs). Hence launch on panel open, not in `Awake()`: `CustomFrameLoader` times out after 5 s.

### Unity side (built)

`Drone/MassServiceLauncher.cs` (static class, creates no UI). `CustomFrameLoader` and the wire protocol are untouched.

- `EnsureRunning()` is idempotent and called from `ImportPageUI.OnEnable()`. `OnEnable` fires every time the panel is reopened, hence the idempotency. The service only starts when the custom path is used, so default-drone students never pay for it. The probe, spawn and wait run on a background thread; `ImportPageUI.Update()` polls `MassServiceLauncher.Current.version` and redraws.
- **Status display:** `ImportPageUI.statusText` holds two lines: the page's own message (set via `SetStatus`) and `Mass service: starting... / ready / <failure reason>`. Every page message goes through `SetStatus` so neither line overwrites the other. Since the merge the box is 32 px tall, and both lines showed in a 2026-10-13 Play-mode screenshot. Node validation messages go to a separate `Node Info Text`, at the bottom left beside Build Drone.
- **Paths:** `ResolveServiceRoot()` returns `Application.dataPath/MassService~` in the Editor and `Application.streamingAssetsPath/MassService` in a player.
- **Launch command:** `python\python.exe -u "<...>\udp_listener.py" --parent-pid <Unity pid>`. `-u` keeps the log unbuffered; `--parent-pid` is the crash protection (decided: parent-PID watch, not an idle timeout or Job Object).
- **Probe before spawn.** Send a 1-byte datagram whose value is not 40 to `127.0.0.1:5070`. The service answers any non-40 header with a 255 error packet (verified), so any reply means "already running". No reply within about 300 ms, or a connection-reset exception from the closed port, means "down".
- **Spawn:** `Process.Start` with `UseShellExecute = false`, `CreateNoWindow = true`, working directory `mass-properties/`, arguments quoted (paths may contain spaces). Send output to `Application.persistentDataPath/mass-service.log`.
- **Port already in use** (the student ran it by hand): the child exits on bind failure, the probe still succeeds, so nothing special is needed.
- **Readiness:** poll the probe for up to about 10 s. Report "Starting…", "Ready" or "Failed: <reason>" through the existing `ImportPageUI.statusText` only (SetText on an existing object, per the no-procedural-UI rule).
- **Shutdown:** on `Application.quitting` (which also fires when leaving Play mode in the Editor) it kills the process it started, never one it did not start. If Unity crashes, the service's `--parent-pid` watch ends it.
- **Failure:** logs `[MassServiceLauncher] Mass service failed to start: <reason>` and keeps the existing fallback to the default drone. The student can still fly. Pressing Begin while the service is still starting has the same result: `CustomFrameLoader` times out after 5 s and falls back.
- **Windows only:** off Windows the probe still detects a hand-started service; otherwise it reports Failed.

### Python-side changes (made in the in-repo copy, not in MassCompose)

`UAVSim/Assets/MassService~/mass-properties/udp_listener.py` differs from the MassCompose original in two ways, both documented in its docstring:
- It binds `127.0.0.1` instead of `0.0.0.0`.
- It accepts `--parent-pid`. A daemon thread waits on that process (Windows: `OpenProcess(SYNCHRONIZE)` + `WaitForSingleObject`, which pins the process so a reused PID cannot be confused with the original; never `os.kill(pid, 0)`, which on Windows sends CTRL_C_EVENT) and then calls `os._exit(0)`.

`mass_composer.py` is an unmodified copy. The wire protocol is unchanged.

### Order of work

1. ~~In-repo copy of the scripts: bind change, parent exit~~ (done)
2. ~~`tools/build-mass-service.ps1`, pinned requirements, `.gitignore`~~ (done)
3. ~~`MassServiceLauncher`, path resolver, `ImportPageUI` hook and status line~~ (done)
4. ~~`IPostprocessBuildWithReport` copy step~~ (done)
5. ~~Test in the Editor~~ (passed 2026-10-13, see Verified above)
6. **Test a built player** on a machine with no Python installed (and, if possible, one not previously used for development).
7. ~~Update `README.md` and the Visual Check step 2.1~~ (done)

## Current MVP scope

Node entry: **typed numeric rows, hand-built in the Editor**, on the same panel as the STL and density. The rows, the Add/Remove Row buttons and the paste box sit in a `ScrollRect` (`NodeScroll`, built in the Editor), so the panel fits a 16:9 screen (800 × 450 canvas). Column headers stay pinned above it. The script binds to existing GameObjects and reads typed values.

**Add / remove rows (built 2026-10-13):** all 12 rows exist in the scene (`Node00`–`Node11`), and 6 are shown at start. **+ Add Row** shows the first hidden row, blank. **- Remove Row** clears and hides the last shown row. Clearing means a re-added row never brings back old values. Rows can go from 4 up to the 12 that exist; the minimum of 4 is the quad-only motor count (`NodePlacementUI.MinShownRows`). **To raise the cap, duplicate rows in the Editor**, keeping the stripes alternating; never `Instantiate` rows from code. The paste box resizes the shown rows to the pasted line count (minimum 4) and rejects more than 12. `Content` uses a `VerticalLayoutGroup` + `ContentSizeFitter`, so the scroll range follows the shown rows with no layout code. Each part has a `LayoutElement` with a fixed height, except `All Nodes`, whose own layout group gives its height.

Verified in Play mode on 2026-10-13: the 4 and 12 limits hold; a re-added row is blank; pasting 9, 3 or 13 lines gave 9 rows, 4 rows, and an error. A 7-node build (4 motors, battery, FC, offboard computer) with the 100 mm cube flew at 1.750 kg, and the COM matched hand arithmetic: CAD (0, −0.00171, −0.00229) m.

### Drone presets (built 2026-10-13)

A preset is a saved copy of one import: the STL path, the density, and the filled node rows, 4 to 12 of them (type, x, y, z, massKg, all in the CAD Z-up frame, metres). It is a convenience for re-running the same import. It is not a new kind of drone. Blank shown rows are not saved, so a preset restores as many rows as it has nodes (minimum 4), like the paste box.

- **Panel row (built in the Editor):** at the top of `CustomDroneImportPanel`, above the STL path: `PresetLabel` ("Preset Drone config"), `PresetDropdown`, `PresetNameInput` (max 40 characters) and `SavePresetButton`. Everything below moved down 20 px; `NodeScroll` lost 20 px of height.
- **Storage:** `DronePresetStore`, one JSON file at `Application.persistentDataPath/drone_presets.json` (on Windows: `%USERPROFILE%\AppData\LocalLow\MIT Beaver Works\UAVSim\`). It holds `{version, presets[]}`, sorted by name. Every save rewrites the whole file through `drone_presets.json.tmp` + `File.Replace`, so a crash mid-write leaves the old file. Floats are written by `JsonUtility` at double precision (0.15 shows as 0.15000000596…); they read back exactly.
- **Corrupt file:** loading logs the reason and gives an empty list. The next save moves the bad file aside to `drone_presets.json.unreadable-<timestamp>` instead of overwriting it. Entries that fail the structural checks (name, path, density, 4+ nodes, known types, finite values, mass > 0) are skipped with a warning.
- **Save:** `ImportPageUI.OnSavePresetClicked` runs the same checks as Build Drone (name, STL exists, density, `NodePlacementUI.TryGetValidNodes`, which includes `MotorQuadrants.TryValidateMotorLayout`), so an invalid layout cannot be saved. Saving under an existing name (case-insensitive, trimmed) overwrites that preset and takes the newly typed spelling.
- **Select:** `ImportPageUI.OnPresetSelected` (wired as the dropdown's On Value Changed in the Editor) fills the rows first (`NodePlacementUI.TryShowNodes`; if the preset does not fit, nothing changes), then the path, density and name field. It writes only to the panel's fields. It does not start the sim and does not touch the statics; Build Drone does that. A missing STL is reported in the status line on selection; Build Drone then refuses it. Option 0 is a prompt ("Choose a preset..." / "(no saved presets)").
- **Re-read on every open:** `ImportPageUI.OnEnable` reloads the file and rebuilds the options with `ClearOptions` / `AddOptions`. Do not instantiate option items.
- **Known limitation:** choosing the preset the dropdown already shows does nothing (TMP_Dropdown fires only on a change). To reload it after editing, pick another option first.
- **Back keeps the panel's path:** `OnEnable` now restores the path from `SelectedSTLPath` only when the panel has none, so a preset-filled but unbuilt path survives Back and reopen.
- **Wire protocol and `CustomFrameLoader`:** unchanged. A preset only feeds the same statics that hand entry feeds, so the request format, the locked enum and the Awake() pipeline stay as they are.

**Not building in this pass:** rename, delete, export or import of presets, sharing presets between machines, and presets for non-quadcopter layouts.

**EXPLICITLY NOT BUILDING — do not add, do not suggest:**
- mesh preview, preview camera, `RenderTexture`, `MeshCollider`
- click-to-place, raycasting
- add/remove rows by creating or destroying row objects from code, marker objects. (Add/remove by showing and hiding the 12 hand-built rows IS built; see above.)
- non-quadcopter layouts (hex, octo, plus-configuration)
- fixing the CAD↔Unity mirror for chiral frames (deferred; see Open questions)

## Hard constraint: no procedural UI

`CustomDroneBuilderUI.cs` built its entire UI at runtime and failed with `MissingReferenceException`s, duplicate objects, and empty panels. It is orphaned — not attached to any GameObject, not referenced by any script — and **must not be revived.**

- All UI GameObjects are hand-built in the Unity Editor.
- Scripts only ever **read, write, or `SetActive`** existing objects.
- **Never `Instantiate` or `Destroy` a UI GameObject.**
- Expose `public` TMP/UI fields for Inspector drag-assignment; wire buttons via `On Click ()`.

## Retracted theories — do NOT re-chase these

- **Animator / animation clip causing the visual spin** — ruled out, not the cause. (UNVERIFIED in this session; carried from prior diagnosis.)
- **Missing winding-order swap** — deliberately absent and CORRECT. The Z-up→Y-up conversion is a proper rotation, so winding is preserved; adding a 2nd/3rd vertex swap would invert faces. Rationale in-code at `StlMeshLoader.cs:19-21`.
- **`AddForceAtPosition` using local instead of world space** — ruled out, not the cause. (UNVERIFIED in this session.)
- **A `CustomFrameLoader` entry-point method** — no such method exists. Verified: every method on the class is `private`.

## Open questions (unresolved — do not assume answered)

- `Flight.cs` was audited on 2026-10-13 (`docs/flight-audit-2026-10-13.md`). **The climbing symptom is fixed (2026-10-14):** fixed attitude gains made frames with roll/pitch inertia below about 0.00165 kg·m² or yaw inertia below about 0.00555 kg·m² unstable, and the motor-saturation rule turned that into a runaway climb. **Superseded 2026-10-20 by real-flight-physics phase 1** (see Flight model): the inertia gain scaling was replaced by PX4 loops and allocation. Yaw now gives way first through `MC_AIRMODE` 0 desaturation, and the light 1.2 kg and 0.28 kg frames hover. Also fixed in phase 1: the mirrored sign tables (removed), the `CrashSystem` `MotorHealth` index (now by corner), and COM offset and motor positions now affect flight (lever-rule test). Correction: a NaN `armLength` would make every motor 0 rad/s (`Mathf.Max(NaN, 0f)` returns 0), so the drone would fall, not climb. That hole is closed at `CustomFrameLoader.cs` (`!IsFinite(armLength)`).
- `mass_composer.py` and `udp_listener.py` have **never been code-reviewed**. The copies Unity runs are in this repo at `UAVSim/Assets/MassService~/mass-properties/` (since `282d53c`), with dependencies pinned in `tools/mass-service-requirements.txt`; the originals live in the MassCompose repo (see Architecture).
- The mirror (see Coordinate conventions): harmless on a symmetric X-quad, wrong on chiral geometry. **Deferred, not fixed.**
- The visual spin of the mock drone remains unexplained.
- Preset STL paths are absolute. If the student moves or renames the `.stl`, the preset loads with a missing file. Decide whether to store a copy of the STL next to the preset file instead. Not decided.

## Known cruft

**`DebugFrameTestSetup`** (`UAVSim/Assets/Scripts/Drone/DebugFrameTestSetup.cs`) — temporary test harness; its own header says it is safe to delete once the real import UI exists (which it now does).

- **Still present but INACTIVE in `UAVSim/Assets/Scenes/Main.unity`** on a GameObject named `TestSetup` (`m_IsActive: 0` in the committed scene, checked 2026-10-13), a child of `MainMenu`. While inactive, none of the injection below runs, and the default route launches with no fallback error. The notes below describe what happens if it is re-enabled.
- `[DefaultExecutionOrder(-200)]` — runs **before** `CustomFrameLoader` (`-100`), so its values land first.
- The scene carries a hardcoded personal path: `Main.unity:18750` → `TestSTLPath: C:\Users\Owner\Downloads\Part Studio 1 - Part 1.stl`. The C# default (`:9`) is `""`; the serialized Inspector value overrides it.
- **Node injection is guarded** by `if (SelectedNodes.Count == 0)` (`:20`) — it only backfills an empty list.
- **The `SelectedSTLPath` assignment is UNCONDITIONAL** (`:14-17`, guarded only against an empty `TestSTLPath`). This is the one that bites: it applies on the normal gameplay route too, not just the import route.
- A second copy exists at `UAVSim/Assets/_Recovery/0 (1).unity:17696` (untracked, not in build).
- **Its four motor values are authored in the wrong frame.** All have `y = 0` and vary in `x`/`z`, i.e. written as if position were Unity-style Y-up. In the CAD Z-up frame they carry zero fore/aft separation and ±0.15 m of *vertical* spread: `ConvertZUpToYUp((0.15, 0, 0.15))` = `(0.15, -0.15, 0)`. They describe a physically nonsensical quad.
- Consequence: every motor sits on the `y == 0` boundary, so the layout guard now rejects it and **every launch falls back to the default drone**, including the normal gameplay route (the `SelectedSTLPath` assignment is unconditional). Expect the boundary error on every Play.
- These values fed the COM and inertia tensor Python has been computing. UNVERIFIED whether this explains the unbalanced/climbing symptom.

The five injected nodes (`:22-26`), positions in the Z-up service frame, metres — **literal values for parity testing**:

| Type | position (x, y, z) | massKg |
|---|---|---|
| `Battery` | `(0, 0, 0.05)` | `0.5` |
| `Motor` | `(0.15, 0, 0.15)` | `0.05` |
| `Motor` | `(-0.15, 0, 0.15)` | `0.05` |
| `Motor` | `(0.15, 0, -0.15)` | `0.05` |
| `Motor` | `(-0.15, 0, -0.15)` | `0.05` |

`TestDensity` = `1250` (C# default `:10`, and the serialized scene value at `Main.unity:18751`).

<!-- The two sections below mirror README.md. Keep both in sync when either changes. -->

## Manual Flight Controls

In default flight mode you can fly the drone from the keyboard or a controller:

| Action | Keyboard | Controller |
|---|---|---|
| Launch / land | `L` | — |
| Forward / back / strafe | `W` `A` `S` `D` | Left joystick |
| Ascend / descend | `Right Shift` / `Left Shift` | Right / left trigger |
| Yaw | `←` / `→` | Right joystick X |
| Max speed down / up | `Z` / `/` | LB / RB |
| Cycle camera (chase, top-down, front) | `Space` | — |
| Zoom camera in / out | Mouse wheel, or `=` / `-` (keypad `+` / `-`) | — |

### Zoom bar

A **zoom bar** (slider) appears on screen in the flight scene. Drag it to zoom in or out: moving it up (right) zooms in, down (left) zooms out. When a custom STL frame is loaded, the bar starts at a zoom fitted to the frame's size, so large or small imported drones start fully in view.

The bar is shown while the drone is on the ground and hides once the drone launches, so it can't capture the arrow keys used for yaw. The mouse wheel and `=` / `-` keys keep zooming in flight. Zoom only moves the follow cameras, so the drone's flight and propeller animation are unchanged.

## Visual Check: Drone and Propellers

Run this check after changing the drone prefab, the flight code, or the custom-frame import, and whenever you import a new model. It confirms that the drone looks right and its propellers spin properly.

**1. Default drone**

1. Open `Main.unity`, press Play, and press **Begin** without importing a frame.
2. Use the zoom bar to zoom in. You should see the stock drone body with **four propellers, one at each corner**.
3. Before launch, the propellers are **still**. This is expected because the motors are off.
4. Press `L` to launch. All four propellers should **start spinning right away and keep spinning** for the whole flight, including while hovering, turning, and landing.
5. Spin directions should alternate diagonally: **front-left and back-right clockwise, front-right and back-left counter-clockwise** (seen from above).
6. Fly around (`W` `A` `S` `D`, `Shift`, arrow keys). No propeller should freeze, drift away from its arm, or flicker.

**2. Imported model**

1. Open the import panel and wait for **Mass service: ready** under the status text. The panel starts the bundled service itself (run `tools/build-mass-service.ps1` once first so the bundle exists). If it shows **Mass service: failed**, the reason is shown there and in the Console, and the service log is `mass-service.log` in `Application.persistentDataPath`.
2. On the same panel, load an `.stl`, enter the density, and fill the node rows in the scrolling table below them. Or pick a saved setup from **Preset Drone config** at the top, which fills all three; to save the current setup, type a name next to it and press **Save Preset**. Press **Build Drone**, then **Back**, then **Begin** on the main menu.
3. Check that **your frame is shown and the stock body is hidden**. If you see both overlapping, or only the stock drone, check the Console (see below).
4. Use the zoom bar to fit the whole frame on screen. Nothing should be clipped or too small to inspect.
5. Check that **each propeller sits on a motor position** of your frame, not where the stock drone's propellers were.
6. Repeat steps 3–6 from the default-drone check. Propellers should be still before launch, spin as soon as you press `L`, and keep the same diagonal directions.

**3. Console messages**

With the Console open during Play:

- **Expected:** `[PropellerSpin] Found 4 propellers` and one `[PropellerSpin] Propeller_N ...` line per propeller showing its corner (Front Left, Front Right, Back Left, Back Right) and spin direction. With an imported model, `[CustomFrameLoader] Mass properties: ...` and `[CustomFrameLoader] Motor positions ...` should also appear.
- **Problem:** any `[CustomFrameLoader] ... Using the default drone.` error means the import failed and the stock drone was used. The message gives the reason (for example, a missing file, an invalid motor layout, or the service not responding).
- **Problem:** `... is inside default body root ...` means a propeller is under an object that is hidden for custom frames, so that propeller will disappear with an imported model.
- **Problem:** `defaultBodyObjects is empty` means the stock body will draw on top of your frame.

> **Tip**: Fast-spinning propellers can look slow, stopped, or even backwards on screen because of the frame rate (the "wagon-wheel" effect). If a propeller looks wrong, check its direction in the `[PropellerSpin]` Console line before treating it as a bug.

## Status

**Done**
- STL parsing, mm→m, Z-up→Y-up — `StlMeshLoader.cs`
- UDP request/response, validation, mesh swap, propeller repositioning — `CustomFrameLoader.cs`
- Layout guard + `armLength` finiteness check — `CustomFrameLoader.cs:57-67`, `:201`
- Shared quadrant contract — `MotorQuadrants.cs`
- Single custom drone panel (STL, density, scrolling node table, one Build Drone button; Back wired to `Close`) — `ImportPageUI.cs`, `NodePlacementUI.cs`
- Drone presets (save, select, persist to `drone_presets.json`) — `DronePresetStore.cs`, `ImportPageUI.cs`
- One motor-corner rule (`MotorQuadrants.ClassifyUnity`), used by the panel, the loader and `PropellerSpin` — `MotorQuadrants.cs`, `CustomFrameLoader.cs`, `PropellerSpin.cs`
- `Flight.cs` audit (read-only), symmetric X-quad hover test, cube mass sanity check — `docs/flight-audit-2026-10-13.md`, DEVLOG 2026-10-13/14
- Climbing fix: attitude gains scaled by each frame's inertia (stock drone bit-identical) — `Flight.cs`, DEVLOG 2026-10-14

**Next**

> **Start here:** the audit, the hover test and the motor-corner refactor from `docs/handoff-flight-audit-and-motor-corners.md` are all done (2026-10-13/14). Fix 1 from `docs/handoff-flight-fixes.md` (inertia-scaled gains) was applied and verified on 2026-10-14. The rest of that file (Fix 2, section 7) is still drafts that each need user approval. The test cube is at `tools/test-data/cube100.stl`. Leave `TestSetup` in place for now (user decision, 2026-10-13).

1. ~~Wire a Back button to `ImportPageUI.Close`~~ (already wired in `Main.unity`; confirmed 2026-10-13)
2. Remove the `TestSetup` GameObject from `Main.unity`
3. ~~Audit `Flight.cs` against the unbalanced/climbing symptom~~ (audit done 2026-10-13, read-only, no code changed: `docs/flight-audit-2026-10-13.md`. Fixes it proposes are waiting on user approval. The symmetric X-quad hover test passed 2026-10-13, but an off-centre-COM control hovered just as perfectly; see DEVLOG. Climbing cause found 2026-10-14: frames with low inertia go unstable at the fixed gains, and the saturation rule then makes them climb away; see audit finding 1. **Fixed 2026-10-14** by scaling the attitude gains by inertia (`Flight.cs`, user-approved, Fix 1 only): the 1.2 kg and 0.28 kg frames now hover, and the default drone's kick traces are identical to before. See audit finding 1, "Fix applied". Still drafts needing approval: Fix 2 and section 7 of `docs/handoff-flight-fixes.md`. Still to do: a hand flight.)
4. ~~Drone presets: `DronePresetStore.cs`, Save-as-preset button, and the **Preset Drone config** dropdown on the import panel~~ (built 2026-10-13; see Drone presets. Still to do: a hand click-through)
5. Optional: main-menu "custom drone armed" indicator + Clear
6. ~~**Single source for the motor-corner rule.**~~ (done 2026-10-14). The rule "front = at or ahead of the centre line, left = left of it (on the line counts as right)" now lives only in `MotorQuadrants.ClassifyUnity(Vector3)` (Unity-local Y-up, no conversion). `MotorQuadrants.Classify` (CAD Z-up, panel validation) converts and calls it, and so do `CustomFrameLoader.RepositionPropellers` (user-approved edit) and both `PropellerSpin` sites (`FindPropellers`: spin direction and motor index; `SetPropellerPositions`: propeller-to-motor placement). `PropellerSpin` maps corner → `Flight.MotorSpeeds` index (FR=0, FL=1, BL=2, BR=3) explicitly; never cast a `Quadrant` to a motor index. Verified via Unity MCP: logs for the default drone and the cube quad match a pre-edit baseline. See DEVLOG 2026-10-14. Related: `CrashSystem.DetachArmGroup` now zeroes `MotorHealth` by corner (fixed 2026-10-20).
7. Auto-launch the mass service from a bundled Python 3.13: built and Editor-tested; built-player test pending (see the section above)

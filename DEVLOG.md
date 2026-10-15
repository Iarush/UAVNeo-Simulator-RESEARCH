# UAVNeo Simulator Dev Log

Running record of changes made to this project. Newest entries go at the top.
Each entry lists what changed and where, and why when it isn't obvious.
Entries marked *(from git/CHANGELOG)* come from the repo's history, not from a
chat transcript. Entries marked *(from CLAUDE.md)* come from the status notes
in CLAUDE.md and have not been checked file by file.

---

## Custom drone frame import (committed 2026-10-13 in `edc07f5`)

*(from CLAUDE.md)* This section was written while this work was still uncommitted.
It was committed on branch `feature/bundled-mass-service` as `edc07f5`. The files
implement custom `.stl` drone import:

- `UAVSim/Assets/Scripts/Drone/CustomFrameLoader.cs`: UDP mass-properties
  pipeline to `127.0.0.1:5070`, mesh swap, propeller repositioning.
- `UAVSim/Assets/Scripts/Drone/StlMeshLoader.cs`: ASCII and binary STL parsing,
  mm→m scale, Z-up→Y-up conversion.
- `UAVSim/Assets/Scripts/Drone/MotorQuadrants.cs`: shared FL/FR/BL/BR layout
  validator used by the UI and `Awake()`.
- `UAVSim/Assets/Scripts/UI/ImportPageUI.cs` and `NodePlacementUI.cs`: import
  panel and six-row node placement panel. (Merged into one panel with 4–12 rows
  in `47d6d13`; see 2026-10-13.)
- `UAVSim/Assets/Scripts/Drone/CustomFrameLoader.cs` also includes the
  `armLength` finiteness check.
- New scene `UAVSim/Assets/Scenes/Drone Builder.unity`, plus `Main.unity`,
  `DronePlayer.prefab` and `Hud.prefab` edits.
- Untracked helpers: `DroneCameraZoom.cs`, `DronePreviewOrbit.cs`,
  `DebugFrameTestSetup.cs` (temporary, slated for removal), `MotorQuadrants.cs`,
  `PropellerSpin.cs` (modified).
- `CustomDroneBuilderUI.cs` exists in the tree but is orphaned and must not be
  revived (see CLAUDE.md).

**Known open items (from CLAUDE.md):**
- ~~The custom-drone path cannot reach flight: nothing wires a button to
  `ImportPageUI.Close`.~~ Wrong: Back was already wired to `Close` in `Main.unity`
  (checked 2026-10-13).
- `TestSetup` (`DebugFrameTestSetup`) is still active in `Main.unity` with a
  hardcoded personal STL path. Remove it.
- `Flight.cs` has not been audited for the unbalanced/climbing symptom.
- ~~Drone presets (`DronePresetStore.cs`, save/select UI) are planned, not
  built.~~ Built 2026-10-13; see that entry.
- The Python mass service (`mass_composer.py`, `udp_listener.py`) is not in this
  repo. The files to use are in the MassCompose repo, at
  `C:\Users\Owner\Documents\MassCompose\src\mass-properties` (see CLAUDE.md).

---

## 2026-10-14

- **Climbing bug fixed: attitude gains scale with each frame's inertia
  (`Flight.cs`, user-approved, Fix 1 of `docs/handoff-flight-fixes.md` only).**
  Each axis's commanded torque in `FixedUpdate()` is multiplied by (body-axis
  inertia ÷ stock 0.003 / 0.006 / 0.003). New: constants `GainRefIxx/Iyy/Izz`,
  fields `pitchGainScale` / `yawGainScale` / `rollGainScale` (set in `Start()`
  and logged as `[Flight] Attitude gain scale (pitch, yaw, roll) = …`), helpers
  `BodyAxisInertia` and `GainScale`. For the stock drone every scale is exactly
  1, so its maths is bit-for-bit unchanged. Verified in Unity via Unity MCP:
  - Default drone: scales (1, 1, 1); yaw-kick trace 0.50 −0.42 0.36 −0.31 0.26
    and roll-kick trace 2.29 0.54 −1.28 −0.63 0.64 identical to the
    pre-fix baseline; motors 732.97 × 4.
  - Cube quad 1.7 kg: (2.056, 1.778, 2.056), motors 675.76 × 4, kicks settle.
  - 1.2 kg yaw-light and 0.28 kg frames, which climbed away before: (0.722,
    0.444, 0.722) and (0.226, 0.198, 0.226). Both hover with altitude error
    ≤ 2.5 mm, tilt 0.000°, four equal motors, no crash messages, kicks settle.
  - Stick test on all four frames (full forward, strafe, yaw, climb): similar
    response everywhere (20.3–20.6° tilt, yaw at the 3 rad/s clamp, 3.75–4.24 m
    climb in 1 s), all level afterwards.
  - Residual: the angular drag torque is not scaled, so yaw now goes unstable
    only below Iyy ≈ 0.0006 kg·m² (was 0.00555) and roll/pitch below ≈ 0.00011
    (was 0.00165). Details in `docs/flight-audit-2026-10-13.md`, finding 1,
    "Fix applied".
  - Not applied, each still waiting on approval: Fix 2 (yaw lowest priority in
    the mixer), sign tables, `CrashSystem` motor index, too-heavy warning, dead
    code. Not yet flown by hand.
  - Test artefact, not a regression: reloading `DroneWorld` in place from a
    RunCommand logs a `SocketException` from `PythonInterface..ctor`
    (`PythonInterface.cs:72`, port still bound by the previous scene's instance).
- **CLAUDE.md stale entries fixed** (the list in the handoff, §1). The mass
  sanity check is ticked: the cube runs matched hand arithmetic for mass, COM
  and inertia, including the off-centre-battery case. The Python service files
  are noted as in version control and pinned, with only a review still open.
  The Editor run of the `~` folder is marked as done. "Start here" now points
  to `docs/handoff-flight-fixes.md`. Status → Done gains the motor-corner rule
  and the audit, hover test and mass check.
- **Climbing symptom reproduced and explained (audit finding 1, confirmed in
  Unity). No code changed.** Frames whose yaw inertia is below about
  0.00555 kg·m², or whose roll/pitch inertia is below about 0.00165 kg·m², go
  unstable at the 0.02 s physics step with `Flight.cs`'s fixed gains. Every step
  the motors then saturate, and the "raise all motors" rule (`Flight.cs:757-762`)
  turns that into a runaway climb. Unity traces via temporary runtime probes:
  - 1.2 kg frame (cube100 at 1000 kg/m³, four 0.05 kg motors at CAD ±0.05 m;
    yaw inertia 0.00267): stayed level, yaw rate grew ×−3.2 per step (predicted
    ×−3.16), diagonal motor pairs alternated 0/1257 rad/s, climbing at 33 m/s
    after 3 s.
  - 0.28 kg frame (cube at 100 kg/m³, four 0.02 kg motors at ±0.08 m, 0.1 kg
    battery): diverged within 0.3 s, climbed to about 111 m, hard crash, and
    repeated on every respawn.
  - Default drone kicks: yaw rate rang at exactly −0.85 per step as predicted
    (stock yaw inertia is only 8% above the limit); roll kick settled in 0.12 s.
  - `tools/flight-audit/flight_loop_replica.py` gained a coupled 3-axis +
    altitude model that reproduces all four cases. Audit doc updated
    (`docs/flight-audit-2026-10-13.md`, finding 1, "Climb symptom", task B
    summary). Proposed fixes (inertia-scaled gains; no unrequested thrust from
    saturation handling) wait on user approval.
- **One shared motor-corner rule (task C), done, no behaviour change.** The rule (front = Unity `z >= 0`, left = `x < 0`, a point on a line
  counts as front / right) now lives only in the new
  `MotorQuadrants.ClassifyUnity(Vector3)`, which takes a Unity-local Y-up position.
  - `MotorQuadrants.Classify` (CAD Z-up) converts with `ConvertZUpToYUp`, then
    calls `ClassifyUnity`.
  - `PropellerSpin.FindPropellers` and `SetPropellerPositions` switch on
    `ClassifyUnity`. The corner → `Flight.MotorSpeeds` index map (FR=0, FL=1,
    BL=2, BR=3) is written out explicitly per case, never cast from `Quadrant`
    (FL=0, FR=1, ...). Spin directions and the `→ Front Left ...` log text are
    unchanged.
  - `CustomFrameLoader.RepositionPropellers` switches on `ClassifyUnity` in
    place of its inline `isFront`/`isLeft` chain (user-approved; the file is
    change-by-approval). Same "already assigned" error path and log text; no
    protocol or `Awake()` change.
  - Verified via Unity MCP. Compiles clean. `ClassifyUnity`: (0,0,0) →
    FrontRight, (-0.1,0,0) → FrontLeft, (0,0,-0.1) → BackRight, (-0.1,0,-0.1) →
    BackLeft. `Classify` gives the same corners as before for the four cube-quad
    motors and for a plus-layout motor at CAD (0, 0.15, 0) (FrontRight). A layout
    with two front-right motors still says "Two in front-right motors, none in
    front-left." For both the default drone and the cube quad (B.1), the
    `[PropellerSpin]` lines (corner, target, meshInverted, sign) and
    `[CustomFrameLoader] Motor positions` match a pre-edit baseline line for line.
    The props land on (±0.15, 0, ±0.15). The cube quad was re-run after the loader
    edit, with the same result.

## 2026-10-13

- **Symmetric X-quad hover test (task B): PASSED, no code changed.** Run via
  Unity MCP. A sampler armed the drone, waited for `[Flight] Launch complete`,
  then logged state every 0.5 s for 20 s with no input.
  - Cube quad (`tools/test-data/cube100.stl`, 1000 kg/m³, four 0.05 kg motors
    at CAD (±0.15, ±0.15, 0), battery 0.5 kg at the origin): mass 1.7 kg, COM 0,
    inertia (0.006167, 0.006167, 0.010667), armEffective 0.15. Altitude error
    under 2.3 mm after 2 s and 0.0000 m from 10 to 20 s. Pitch, roll and yaw
    change 0.000°, horizontal drift 0.0000 m. All four motors at 675.76 rad/s,
    which equals √(m·g/4/kf).
  - Default drone: 2.0 kg, same result, motors at 732.97 rad/s.
  - **Control: the pass says little about balance.** With the battery moved
    to CAD (0.1, 0, 0) (COM (0.0294, 0, 0)) the drone still hovered dead level
    with all motors equal (0.000 % spread). A real quad would need about a 20 %
    motor-speed spread to hold that. This confirms audit lead 3: thrust is
    applied at the COM and mixing assumes an ideal symmetric X, so an
    off-centre COM has no effect in flight.
  - Hovers about 0.54 m below the logged launch altitude (7.36 vs 7.9). Launch
    ends within 0.5 m of the target and resets `TargetAltitude` to the current
    height. This is expected from the code, not drift.
  - `TestSetup` is `m_IsActive: 0` in the committed `Main.unity`, so the
    default route gets no fallback error (CLAUDE.md said it was active).
  - Side issue, not reproduced: in one Play session the launcher logged
    "no reply ... after 10 s" while the service answered the real request 5 s
    later. Other sessions were driving the same Editor at that moment.
- **`Flight.cs` audit (task A), read-only, no code changed.** Findings in
  `docs/flight-audit-2026-10-13.md`; offline replica of the control loops in
  `tools/flight-audit/flight_loop_replica.py`. Main results: the fixed gains go
  unstable at the 0.02 s step for roll/pitch inertia below about 0.00165 kg·m² and
  yaw inertia below about 0.00555 kg·m² (25 Hz jitter on small frames; the stock
  drone is just inside). Per-motor roll/yaw sign tables are mirrored left to right
  (flight unaffected, telemetry and crash damage affected). `CrashSystem` zeroes
  the wrong `MotorHealth` index. COM offset and arm length have no effect on
  flight. Frames over 5.88 kg cannot hover. Altitude hold has no steady-state climb.
  Correction: a NaN `armLength` would make the drone fall, not climb. Checked in
  Unity: rotation sign conventions, and the cube quad's inertia on the right axes
  (Y-Y = 0.010667). The proposed fixes wait on user approval.
- **Decided: one shared motor-corner rule** (CLAUDE.md Status → Next, item 6;
  was "Optional"). The front/back, left/right rule is copied in
  `MotorQuadrants.cs`, `CustomFrameLoader.cs` and twice in `PropellerSpin.cs`.
  The copies agree today; the change is to stop them drifting apart under
  future edits. Not started; `CustomFrameLoader.cs` edits need approval.
- **Handoff written:** `docs/handoff-flight-audit-and-motor-corners.md`. It has
  step-by-step instructions for the `Flight.cs` audit (leads with line
  numbers), the symmetric X-quad hover test (setup, sampling, pass criteria),
  and the motor-corner refactor (plan, the FL/FR motor-order trap, before/after
  checks). Test STL added at `tools/test-data/cube100.stl` (100 mm ASCII cube,
  centred). It also lists stale CLAUDE.md entries. `TestSetup` stays for now
  (user decision).
- **Drone presets** (`453d3fc`). Students can save the STL path, density
  and node rows under a name and pick them again later, including in later
  sessions.
  - New `UAVSim/Assets/Scripts/Drone/DronePresetStore.cs`: JSON file at
    `Application.persistentDataPath/drone_presets.json`, written through a temp
    file + `File.Replace`; corrupt file → empty list, moved aside on next save;
    same-name save (case-insensitive) overwrites.
  - `ImportPageUI.cs`: `presetDropdown`, `presetNameInput`,
    `OnPresetSelected(int)`, `OnSavePresetClicked()`, re-read on every open.
    `OnEnable` now restores the path from the statics only when the panel has
    none, so a preset-filled path survives Back.
  - `NodePlacementUI.cs`: `TryGetValidNodes` (shared by Build and Save),
    `TryShowNodes` (fills rows from a preset); paste and presets share one
    `FillRows` helper.
  - `Main.unity`: preset row (label, dropdown, name field, Save Preset) built
    via Unity MCP from copies of existing controls; everything below moved down
    20 px. Object-ID diff against HEAD: 84 added, 0 removed.
  - Verified in Play mode (Unity MCP): save writes the file; a scrambled panel
    is restored exactly (7 rows, types, values, density, path) from the
    dropdown; Build → Back → Begin flew it at 1.850 kg with COM matching hand
    arithmetic; overwrite, empty-name and bad-layout saves behave; a preset whose
    STL was deleted shows a clear message, Build refuses it, and re-browse +
    resave fixes it; a corrupt file loads as empty and is moved aside; a fresh
    Play session lists the saved presets. After an Editor restart: the preset
    row, its wiring and layout were intact, and the file written by the
    previous Editor session listed and loaded both presets exactly (the file
    was restored from an unchanged copy, as it had been deleted after testing).
    Not done: a hand click-through.
  - Known limitation: choosing the preset already shown in the dropdown does
    nothing; pick another option first to reload it.
  - Test presets: `drone_presets.json` on this machine holds two test presets,
    "Moved Frame" (1000 kg/m³) and "test quad 7" (1100 kg/m³). Both use the
    same 100 mm test cube from a temporary folder, so they look identical and
    will report the STL missing once that folder is cleared. Delete them, or the
    whole file, when no longer needed.
  - Docs: CLAUDE.md (definition of done, Architecture, Drone presets section,
    Status, Visual Check step 2.2) and README Visual Check step 2.2 updated.
- **Merged the import and node placement panels into one** (`47d6d13`).
  `CustomDroneImportPanel` now holds the STL path, density and node table;
  `NodePlacementPanel` was deleted. The node rows, row buttons and paste box sit
  in a `NodeScroll` scroll view built in the Editor. Column headers stay pinned
  above it, so the panel fits a 16:9 screen (800 × 450 canvas). One **Build
  Drone** button (`ImportPageUI.OnBuildDroneClicked`) replaces Confirm, Add Nodes
  and the node page's Build/Back. It checks that the STL file exists, then the
  density, then the rows (`NodePlacementUI.TryStoreNodes`), and stores all three
  statics only when everything is valid. Fixed on the way: Add Nodes used to open
  the node page without the density check. The unused `ArmLength` field was
  dropped.
  - `UAVSim/Assets/Scripts/UI/ImportPageUI.cs`: `nodePlacementPanel` →
    `nodePlacement`; `OnConfirmButtonClicked` → `OnBuildDroneClicked`.
  - `UAVSim/Assets/Scripts/UI/NodePlacementUI.cs`: now on the same GameObject as
    `ImportPageUI`; `OnBuildDroneClicked` → `bool TryStoreNodes()`; `importPanel`
    and `OnBackButtonClicked` removed.
  - `UAVSim/Assets/Scenes/Main.unity`: the layout, scroll view and wiring were
    done through Unity MCP in the Editor.
- **Add / remove node rows** (`47d6d13`). Students can enter more or fewer than
  six nodes. There are 12 hand-built rows (`Node00`–`Node11`) and 6 are shown at
  start. **+ Add Row** shows the next hidden row, blank. **- Remove Row** clears
  and hides the last shown row. The range is 4 (quad-only motor count) to 12. No
  UI is created at runtime, in line with the no-procedural-UI rule. Hidden rows
  are ignored when collecting nodes. The paste box now resizes the shown rows to
  the pasted line count. `Content` uses a `VerticalLayoutGroup` +
  `ContentSizeFitter`, so the scroll range follows the shown rows. Why: six rows
  could not hold 4 motors + battery + FC + offboard computer.
- **Verified in Play mode (Unity MCP):**
  - The 100 mm cube with 5 nodes went through Build, Back and Begin and flew at
    1.7 kg, moments 0.006167 / 0.006167 / 0.010667.
  - A 7-node build flew at 1.750 kg. Its COM matched hand arithmetic: CAD
    (0, −0.00171, −0.00229) m.
  - The 4 and 12 row limits hold, and a re-added row is blank.
  - Pasting 9, 3 and 13 lines gave 9 rows, 4 rows and an error.
  - Not done yet: a hand click-through, a 16:9 Game view, and a built player.
- **Docs:** CLAUDE.md now describes the merged panel and the row feature. The
  "nothing closes the import panel" blocker is removed. The "do not build" rule
  now forbids only creating or destroying rows from code. README Visual Check
  step 2.2 is updated.
- **Fixed: `SocketException` on leaving Play mode** (`d6e99a1`).
  `HandleExit` closed `udpClientAsync` while the async thread was blocked in an
  unguarded `Receive`, so the close surfaced as an unhandled exception. In
  `UAVSim/Assets/Scripts/NonMonoBehavior/PythonInterface.cs`:
  - The async client is now created in the constructor, not on its thread, so
    `HandleExit` can never find it null.
  - `HandleExit` sets a volatile `isClosing` flag before closing; the thread
    catches the resulting `SocketException` / `ObjectDisposedException` and
    returns quietly.
  - A `ConnectionReset` on `Receive` (Windows reports an earlier send to a
    closed Python port that way) is now skipped instead of killing the async
    thread.
  - Verified in Play mode (Unity MCP): two Main → Begin → DroneWorld → exit
    cycles, no exception; ports 5064/5065 bound in Play and free after. The
    async thread still answered after a depth-image request whose sender had
    already closed its socket. Not tested: a real Jupyter/`drone_core` session.
- **Fixed: `graphify update .` failing** with "uv trampoline failed to
  canonicalize script path". The `graphify.exe` launcher in `~/.local/bin`
  pointed at a uv tool environment that no longer existed. Reinstalled with
  `uv tool install --force graphifyy` (v0.9.90); `graphify update .` and
  `graphify query` run again. Not a repo change.
- **Auto-launch of the mass service with a bundled Python 3.13 was built**
  (`282d53c`, *from git*): `MassServiceLauncher.cs`, `MassServiceBuildStep.cs`,
  `tools/build-mass-service.ps1` and the in-repo service copy under
  `UAVSim/Assets/MassService~/`. It passed in the Editor; a built player is not
  yet tested. This supersedes the "Decided, not built" entry below.
- **Committed the custom drone frame import** (`edc07f5`, *from git*); see the
  section at the top.

Earlier the same day:

- **Documented where the Python backend lives.** CLAUDE.md's Architecture section
  said the Python was missing; it now points to
  `C:\Users\Owner\Documents\MassCompose\src\mass-properties` (`udp_listener.py`,
  `mass_composer.py`). Those files import `masscompose`, a PyPI package
  (`pip install masscompose`), like numpy and trimesh; it is not code to copy. The
  older copy in `uav-neo-installer/.../mass-properties/` is superseded. Checked:
  files import cleanly under MassCompose's `.venv`; wire protocol/port are
  unchanged; the PyPI 0.1.0 wheel is identical to the local `src/masscompose/`.
  Found: PyPI metadata says `requires_python >=3.13`, but the student installer
  uses Python 3.9, so installing it there will be refused. Not checked: an
  end-to-end request from Unity. Both service files have uncommitted changes in
  the MassCompose repo.
- **Decided, not built: auto-launch the service with a bundled Python 3.13.**
  Unity would start it when the import panel opens
  (`ImportPageUI.OnEnable()` / `MainMenu.ShowCustomDroneBuilder()`). Full plan,
  build recipe, and open risks are in CLAUDE.md ("Plan: auto-launch the Python
  backend with a bundled Python"). No project files were changed for it.
  Tried in a scratch folder only: embeddable Python 3.13.14 + `pip install
  --target` of masscompose/numpy/trimesh, then the service code run under it. A
  100 mm cube + battery + 4 motors matched hand arithmetic (moments 0.0061667 /
  0.0061667 / 0.0106667, arm 0.21213). The `._pth` edit is required (without it
  the script folder is not importable). Bundle 65 MB unpacked, 26.3 MB deflated;
  ~1 s start on a warm cache. Not tested: real UDP bind, firewall prompt,
  antivirus, a machine without Python.
- **Found:** `SyntaxWarning: invalid escape sequence` in `masscompose/compose.py`
  line 3 (PyPI 0.1.0 and local source). Harmless now; should be fixed and
  republished.
- **Not chosen:** porting the math to C#. Kept as a long-term alternative; the
  size estimate was made against the older copy and should be redone against
  `masscompose`.
- **Created `DEVLOG.md`.** Starting point for logging changes to this project.

## 2026-06-27 — v1.0.0 release

*(from git/CHANGELOG)* Commits `9585ad3`, `d0e32e9`, `8913bf6`, merged via PR #9.

- Lab curriculum A–I + Grand Prix (scenes, runtime generators for Labs B–F).
- Autograder system: per-lab challenge generators, tier scenes, HUD timer, summary screen.
- Drone Arena environment template; standardized lighting.
- New `uav-neo-2` airframe in a consolidated drone player prefab.
- Main menu redesign (dark-industrial theme); HUD overhead map; debug overlay.
- Removed procedural scene/prefab builders and their editor menus, outdated labs,
  duplicate drone prefab, and vestigial racecar UI.
- Fixes: spawn height for the new airframe, autograder summary/HUD, overhead-map
  markers, Grand Prix race mode, menu Exit/Cancel buttons.

## 2026-06-14 — Autograder fixes

*(from git)* Merged PR #4 (`1e6ccb9`) and PR #5 (`0f8f8dc`).

- `0134352` Module 2 autograder scoring, summary, and gate visuals.
- `8081514` CHANGELOG note on deleting the `gael/uav-unity-scenes` branch.

## 2026-05-03 — v0.0.3

*(from git/CHANGELOG)* Commits `8f472b3`, `d5c27a4`.

- Module 6 content and autograder wiring (`GRGAutograderSetup`).
- Wire protocol opcodes renamed `racecar_*` → `drone_*` (byte-compatible).
- C# renames: `RacecarModule`→`DroneModule`, `Racecar`→`Drone`, `GetCar`/`ResetCar`→`GetDrone`/`ResetDrone`.
- Folder moves: `Scripts/Racecar/` → `Scripts/Drone/`; prefab folder reorganization.
- HUD refactor: telemetry and indicator panels baked into `Hud.prefab`.
- Unity 6 API cleanup (`FindObjectsOfType` → `FindObjectsByType`).
- Removed dead `Lidar.cs` and `Drive.cs`, LIDAR opcodes, and about 1,700 racecar-only asset files.

## 2026-03-29 — v0.0.2

*(from git/CHANGELOG)* Merged PR #1 (`3dec7ae`, `ffeadd8`).

- AirSim Labs integration: Modules 1–5 scenes (`HelloDrone`, `DroneControl`,
  `ObjectDetection`, `ArUcoMaze`, `SearchAndRescue`, `MazeNavigation`).
- `Flight.cs` and `LevelCollection.cs` updated for lab behavior.
- `.gitignore` expanded for Unity build directories.

## 2026-03-13 — v0.0.1 beta

*(from git/CHANGELOG)* Commits `b9cd340`, `6f70598`, `c930eb1`.

- Initial UAVSim import, forked from RACECAR Neo Simulator.
- Drone prefab and scripts, AR markers, tape patterns, UI art, sign models.
- Windows build artifacts removed from `main` in `c930eb1`.

## 2026-02-23 — Initial commit

*(from git)* `bb54c93`.

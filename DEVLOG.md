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
- Drone presets (`DronePresetStore.cs`, save/select UI) are planned, not built.
- The Python mass service (`mass_composer.py`, `udp_listener.py`) is not in this
  repo. The files to use are in the MassCompose repo, at
  `C:\Users\Owner\Documents\MassCompose\src\mass-properties` (see CLAUDE.md).

---

## 2026-10-13

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
- **Fixed: `SocketException` on leaving Play mode** (uncommitted).
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

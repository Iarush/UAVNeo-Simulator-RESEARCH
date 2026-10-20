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

## 2026-10-20 — Real flight physics, phase 2 (PX4 velocity/position loops, velocity overrides removed)

Spec: `.plans/real-flight-physics-spec.md` §4.3, §4.4 (`Pcmd` path), §4.6.

**Changed (`Flight.cs`):**
- With no horizontal input the drone holds position the PX4 way. The velocity setpoint ramps from the
  current velocity to zero at `MPC_ACC_HOR` (3 m/s²). Once it is zero and the speed is below 0.1 m/s, the
  position locks. The PX4 velocity PID (`MPC_XY_VEL_P/I/D_ACC`) plus the ramp's acceleration fed forward
  (as PX4 does with its trajectory setpoint) gives an acceleration, which sets the thrust axis (tilt limit
  `MPC_TILTMAX_AIR`) at the held heading.
- Altitude hold uses the PX4 vertical loop (`MPC_Z_P`, `MPC_Z_VEL_P/I/D_ACC`, climb and descent limited by
  `MPC_Z_VEL_MAX_UP/DN`). Collective thrust is hover-normalised, divided by the setpoint tilt and kept
  within `MPC_THR_MIN/MAX`. A non-zero throttle input still gives a thrust offset, as before.
- Tilt input keeps its `send_pcmd` meaning: pitch/roll = value × `pcmdMaxTiltDeg` (35°) × `MaxSpeed`.
  Yaw 1.0 = 1 rad/s. The heading is held when yaw is released. Banking is removed.
- Removed: velocity rotation during yaw, cross-brake and reverse-brake drag, position-hold brake drag, the
  0.08 m/s velocity snap, and the angular-velocity write clamp (`rBody.maxAngularVelocity` stays as the
  engine cap). Drag is now plain quadratic horizontal and linear vertical; frame-scaled drag comes in phase 5.
- The kinematic launch moved into `KinematicLaunchStep()`, unchanged. It, `Arm()` and `FinishDisarm()`
  still write velocities until phase 4. `Relaunch()` (respawn) is the permanent exception.
- Pure vertical `send_pcmd` now holds position instead of forcing level with no hold (spec open item 5).

**Tests (Unity CLI, batch mode, Editor closed; owner decision 2026-10-20 to use the CLI instead of
Unity MCP):** FlightConfigTests + FlightMotorTests without Baseline and the phase 3 tests:
**21 passed, 0 failed, 0 ignored.** New tests are in `Tests/PlayMode/FlightControlTests.cs`, a partial
class of `FlightMotorTests`. Command:
`Unity.exe -batchmode -projectPath UAVSim -runTests -testPlatform PlayMode -testFilter "^UAVSim\.Tests\.PlayMode\.(FlightConfigTests|FlightMotorTests\.(?!PosCtl|Pcmd_Unchanged|Baseline))" -testResults <xml> -logFile <log>`.
- `NoVelocityWritesInFlight`: writes only in Arm, Relaunch, FinishDisarm and KinematicLaunchStep. The
  runtime probe's largest velocity change was 0.882 × the force-implied limit.
- `PcmdTilt_StepResponse`: pitch 0.5 → 8.74° (target 8.75°). 63% at 145 ms (expected 154 ms ±25%),
  overshoot −0.1%, max pitch rate 46.7°/s (limit 220).
- `PcmdYaw_RateAndNoBank`: 57.19°/s (expected 57.30), max roll 0.000°.
- `TurnKeepsMomentum`: peak angle between velocity and heading 58.9°, 28.8° 2 s after the turn.
- `Hold_StopsByTilting`: released at 1.00 m/s. Peak deceleration 3.02 m/s² (`MPC_ACC_HOR` 3.0; limit
  5.5), stopping distance 0.384 m (minimum 0.101), up to 17.6° nose-up, moved 2.4 cm between 5 s and 8 s.
  Before the acceleration feed-forward the same test braked at only 1.42 m/s², took 0.827 m and still
  moved 8.1 cm while settling.
- `LightFrames_DoNotClimbAway`: yaw-light 1.2 kg and small 0.28 kg both hover at u 0.5000, altitude error
  0.10 cm, tilt 0.000°.
- `Saturation_YawGivesWayFirst`: roll alone 2.2198 N·m; roll + yaw gives roll 2.2198 N·m and yaw 0.0000
  N·m (yaw alone 0.9810).
- Phase 1 tests still pass: default/cube/7-inch hover at u 0.5000; lever rule 59.80% vs 59.80%; sign
  checks; crash by corner.

**Baseline hover traces** (Editor run, same phase 2 controller before the feed-forward, which does not act
in a steady hover): 20 s hover, max altitude error / drift / tilt: default drone 9.11 mm / 0.01 mm / 0.000°,
cube 8.83 / 0.00 / 0.000°, 7-inch 8.78 / 0.12 / 0.000°, yaw-light 1.2 kg 8.87 / 0.00 / 0.000°, small 0.28 kg
8.87 / 0.00 / 0.000°. The largest error is at t = 0, right after the kinematic launch hands over, and it decays
below 0.05 mm by 10 s.

**Not run:** the lab regression report (deferred to the end, owner decision 2026-10-20) and a hand flight.

**Also in the working tree, not part of phase 2:** the phase 3 keyboard path (`Flight.ControlMode`, set in
`LevelManager.Update` when the simulation mode changes) and `Tests/PlayMode/FlightPosCtlTests.cs`. Its last
Editor run, before the feed-forward, failed 5 tests on a 7–8% speed and climb overshoot; not re-run since.

---

## 2026-10-20 — Real flight physics, phases 0 and 1 (branch `feature/real-flight-physics`)

Spec: `.plans/real-flight-physics-spec.md`. All tests run through Unity MCP in the
Editor (Unity 6000.4.5f1).

**Phase 0: foundations (no flight change)**
- `tools/extract-px4-params.py` (stdlib only; run with the bundled
  `UAVSim/Assets/MassService~/python/python.exe -I`): reads QGC `.tlog` files,
  CRC-checks MAVLink v1/v2 `PARAM_VALUE` and `AUTOPILOT_VERSION`, refuses logs from
  a different vehicle uid, writes QGC-format params.
- `UAVSim/Assets/StreamingAssets/Flight/uav-neo.params`: 1133 params from the
  2026-07-29 17-08-00 log, PX4 v1.17.0 official. Merging all 5 logs gives identical values.
- `UAVSim/Assets/StreamingAssets/Flight/flight-sim.json`: sim-only values, each
  with its source (motor time constants, launch height, `send_pcmd` scaling, drag,
  sub-steps).
- `Drone/FlightConfig.cs`: loads both files with built-in fallbacks (one
  `Debug.LogError` per bad file). `Px4StockGains` holds the PX4 v1.17.0 stock rate
  gains, confirmed from the v1.17.0 source (`mc_rate_control_params.c`,
  `mc_att_control_params.c`); the real drone's tuned rate gains are not used.
- Baseline hover traces on the old physics (20 s each; `Baseline_HoverTrace`):
  default 2.0 kg 7.23 mm max altitude error, cube 1.7 kg 7.06 mm, 7-inch 0.741 kg
  7.05 mm, yaw-light 1.2 kg 7.22 mm, small 0.28 kg 7.14 mm; drift and tilt 0 on all.
- `FlightConfigTests`: 4/4 pass.

**Phase 1: motors, plus the PX4 loops (pulled forward from phase 2, owner decision)**
- `Flight.cs` (owner-approved):
  - Motors sized per frame so hover is at `MPC_THR_HOVER` (Tmax = m·g / (4·0.5)).
    First-order lag, τ_up 12.5 ms and τ_down 25 ms. One `AddForceAtPosition` per
    motor at its real position. Yaw reaction from each motor's spin direction
    (`CA_ROTOR0_KM` 0.05).
  - The allocation is the inverse of the effectiveness matrix built from the motor
    positions about the real COM. The sign tables, the old PD gains and the
    inertia gain scaling (2026-10-14) are removed.
  - PX4 v1.17.0 attitude loop (reduced attitude, `MC_YAW_WEIGHT`), rate loop
    (P/I/D/FF/K with PX4's integrator rules), normalised mix and `MC_AIRMODE` 0
    sequential desaturation. The rate loop, allocation and motor lag are
    sub-stepped 5× per 0.02 s physics step (250 Hz).
  - Why the PX4 loops were pulled forward: with motor lag τ, the old PD loop is
    stable only if Kd > τ·Kp, and the old 10 / 0.06 gains fail that test. The first
    phase 1 run (old PD plus lag) rocked 6–8° and the cube and 7-inch lost metres
    of altitude.
  - Motor positions come from the propellers only for custom frames. The stock
    prefab's propeller visuals sit at about ±0.72 m, but its physics model is a
    0.16 m arm with 0.003 kg·m² inertia; using the visuals flipped it. The stock
    drone therefore uses an X layout from `armLength`.
- `CrashSystem.cs`: a detached arm now zeroes `MotorHealth` of that arm's corner
  (audit finding 5). `MotorQuadrants.MotorIndex(Quadrant)` maps FR 0, FL 1, BL 2, BR 3.
- Owner decisions 2026-10-20:
  - `MC_ROLL_P` / `MC_PITCH_P` are the real drone's 6.5, not stock 4.0.
  - `send_pcmd` and keyboard yaw 1.0 = 1 rad/s (57.3 °/s; was 200 °/s;
    `pcmdMaxYawRateDegS`). This answers task 0.6.
  - Lab regression is run once after all phases, not per phase. It has not been
    run yet. **Lab-visible changes in this phase:** max yaw rate 1 rad/s; tilt
    response with motor lag and the 6.5 attitude gain.
- `FlightMotorTests` (new), final run with 6.5 and 1 rad/s, 13/13 pass with `FlightConfigTests`:
  - Motor lag: 63% at 12.51 ms up and 25.00 ms down.
  - Hover (default, cube, 7-inch): mean u 0.5000, max altitude error 0.11–0.13 cm,
    drift 0, tilt 0.000°.
  - Off-centre COM (battery at CAD x 0.1): right pair 59.80% measured vs 59.80%
    lever rule, tilt 0.000°. This is the imbalance test the 2026-10-13 hover test
    could not do.
  - Pitch forward and roll right: 17.4°, 1.12 m moved in 1 s, correct motors lead.
  - Yaw right: 56.6 °/s after 0.8 s (limit 57.3), FR+BL lead.
  - Crash: FL thrust 4.906 → 2.204 N after one step (lag) → 0 N.
  - Baseline hover on the new physics (run with gain 4.0 and 200 °/s, before the
    owner decisions): all 5 frames ≤ 7.48 mm, drift and tilt 0.
- Test changes from the spec:
  - Yaw: the commands are averaged over the first 5 steps. The stock drone reaches
    its rate setpoint within a step or two, so a single sample lands on the
    oscillation.
  - Crash: the "dips toward FL" check is dropped. Under the PX4 loops the drone
    stays nearly level and the other three motors saturate, so the response is
    reported, not asserted.
- Not done: the profiler measurement of the sub-step cost (agreed: drop to 2
  sub-steps only if it shows up). A hand flight.
- Temporary: `UAVSim/Assets/Editor/TmpTestResultWriter.cs` writes test results
  to `UAVSim/Temp/tmp-test-results.txt` for the MCP runs. Delete it before committing.

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

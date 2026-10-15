# Handoff: Flight.cs audit, symmetric X-quad hover test, motor-corner rule

Written 2026-10-13 at the end of a long session, so a fresh session can pick this up
cold. Read CLAUDE.md first (it is loaded automatically); this file adds the
step-by-step plan and the traps found while preparing it.

Do the tasks in this order: **A (audit) → B (hover test) → C (motor-corner rule)**.
A and B are investigation; report findings to the user before changing flight
behaviour. C is a refactor that must not change behaviour.

---

## 0. Ground rules (from CLAUDE.md and the user)

- **Do not remove `TestSetup` / `DebugFrameTestSetup` for now.** The user said to
  leave it. Work around it (see B.1).
- **Ask before editing `CustomFrameLoader.cs`** (task C needs this). The wire
  protocol is LOCKED.
- **Never Instantiate/Destroy UI GameObjects at runtime.** UI is hand-built in the
  Editor.
- **Default drone and all lab modules must behave exactly as before.** `Flight.cs`
  drives every drone in every lab, so any change to it needs the user's OK first.
  Audit and report; don't "fix" on your own.
- **Commit only when asked.** Commits end with the attribution lines in the system
  reminder.
- **Keep the docs current.** After each piece of work, update `DEVLOG.md` (new
  entries at the top of the 2026-10-13 section or a new dated section) and the
  CLAUDE.md Status / Definition of done. Run `graphify update .` from `UAVSim/`
  after code changes (it takes several minutes; run it in the background).
- MassCompose repo is out of scope.

## 1. Current state

- Repo: `C:\Users\Owner\Documents\Projects\UAVNeo-Simulator-RESEARCH\UAVNeo-Simulator-RESEARCH`,
  branch `feature/bundled-mass-service`, not pushed.
- Recent commits: `f3b9eea` DEVLOG, `453d3fc` drone presets, `d6e99a1`
  PythonInterface socket fix, `47d6d13` merged panel + add/remove rows,
  `282d53c` auto-launch mass service, `edc07f5` custom frame import.
- Uncommitted at handoff: CLAUDE.md Status item 6 (motor-corner rule now a
  required to-do), a DEVLOG line about that decision, this file, and
  `tools/test-data/cube100.stl`.
- Unity 6000.4.5f1. Project: `UAVSim/`. Main scene: `Assets/Scenes/Main.unity`.
  Flight scene: `Assets/Scenes/Sandbox_Worlds/DroneWorld.unity`.
- Test STL: **`tools/test-data/cube100.stl`**, a 100 mm ASCII cube centred on the
  origin (±50 mm). At 1000 kg/m³ it weighs 1.0 kg; its moments about its own
  centre are 1/6 · m · a² = 0.0016667 kg·m² on every axis.
- `drone_presets.json` (in `%USERPROFILE%\AppData\LocalLow\MIT Beaver Works\UAVSim\`)
  holds two test presets that point at a cube in an old temp folder. Ignore them,
  or delete them if they get in the way.

### Known stale entries in CLAUDE.md (fix when convenient)

- Definition of done, "Mass sanity check": this is effectively passed. The cube at
  1000 kg/m³ plus battery 0.5 kg at origin plus 4 × 0.05 kg motors at (±0.15, ±0.15, 0)
  flew at 1.7 kg with moments 0.006167 / 0.006167 / 0.010667 (= hand
  arithmetic). Open questions still says "No end-to-end mass sanity check has
  been run".
- Definition of done, Python files: they ARE in version control at
  `UAVSim/Assets/MassService~/mass-properties/` (since `282d53c`), and pinned in
  `tools/mass-service-requirements.txt`. Only "reviewed" is open. Open
  questions still says they "are not in this repo".
- Auto-launch section: "The `~` folder and the post-build copy compile but have
  not yet been exercised in a running Editor": the Editor part has been
  exercised. Only the built player is untested.
- Line numbers drift. `CustomFrameLoader` layout guard is now at `:79`, not
  `:57-67`. `Flight.ApplyCustomMassProperties` is at `:461` and takes **5**
  arguments (`..., float newArmLength, float totalMassKg`). Always re-check line
  numbers before quoting them.

## 2. Unity MCP tips (learned the hard way)

- Tools: `mcp__unity-mcp__Unity_RunCommand` (C# with `internal class CommandScript : IRunCommand`),
  `Unity_GetConsoleLogs`. Load them with ToolSearch first.
- Enter or leave Play mode: `EditorApplication.isPlaying = true/false;`. Then wait
  by polling the Unity window title from PowerShell until it starts with `UAVSim`
  (it shows "Reloading Domain", "Running managed callbacks", etc. while busy). The
  MCP connection drops during domain reloads.
- **Scripts that touch files with `System.IO.File.*` (Copy, Delete, ...) are refused**
  ("User interactions are not supported"). Do file work in PowerShell instead.
  Project code that writes files (e.g. `DronePresetStore.Save`) is fine.
- `result.Log("{0:F3}", x)` does not apply format specifiers. Build the string
  yourself: `result.Log("m=" + x.ToString("F4"))`.
- Screenshots: `ScreenCapture.CaptureScreenshot(@"<scratchpad>\x.png")` in Play
  mode, then Read the PNG. `Unity_Camera_Capture` does not include overlay UI.
- Private methods can be invoked with `gameObject.SendMessage("OnStlPicked", new[] { path })`.
- Open the import panel: `Object.FindAnyObjectByType<MainMenu>().ShowCustomDroneBuilder();`.
  This also starts the bundled mass service (ready after about 1.3 s).
  Then fill the panel: `page.densityInput.text`, `page.nodePlacement.bulkPasteInput.text`
  + `OnApplyPasteClicked()`, `page.OnBuildDroneClicked()`, `page.Close()`, then
  `MainMenu.BeginSimulation()`.
- A Unity console error logged on purpose makes RunCommand report
  "UNEXPECTED_ERROR ... executed partially". Read the logs; it may be fine.

---

## A. Audit `Flight.cs` (read-only)

File: `UAVSim/Assets/Scripts/Drone/Flight.cs` (927 lines). Never audited. The
reported symptom: a custom drone flies unbalanced / keeps climbing.

Read it end to end. Then check each lead below and write down evidence for each,
not guesses. Leads, in rough priority order:

1. **Inertia axes.** Comments: `Ixx` = pitch (Unity X), `Iyy` = yaw (Unity Y),
   `Izz` = roll (Unity Z) (`:86-96`). `ApplyCustomMassProperties` (`:461`) copies
   the service's principal moments straight into Ixx/Iyy/Izz, and the rotation
   into `inertiaTensorRotation` (`:491-492`). For the cube quad, Unity showed
   `inertiaTensor = (0.006167, 0.006167, 0.010667)`. The largest moment belongs to
   the vertical (yaw) axis, which is Unity **Y**, so `inertiaTensorRotation` must
   rotate local Z onto Y. Verify: compute R·diag(I)·Rᵀ from `rb.inertiaTensorRotation`
   and check the Y-Y term is 0.010667. If the yaw axis gets a roll moment, yaw
   and roll response are wrong.
2. **Gains are fixed, not scaled to inertia.** `Kp_rp = 10`, `Kd_rp = 0.06`
   (`:123-127`), tuned for `Ixx = Izz = 0.003`. The damping ratio is
   ζ = Kd / (2·√(Kp·I)). That gives about 0.17 for the stock 0.003 and about 0.12
   for the cube quad's 0.006167: underdamped, and worse as frames get heavier.
   Check whether attitude oscillates after a disturbance on custom frames.
3. **Torques ignore real motor positions.** Mixing (`:736-808`) uses sign tables
   (`:426-436`) and one `armEffective = armLength/√2` (`:50`) for all four motors.
   This is an ideal symmetric X-quad. Thrust is applied at the centre of mass
   (`AddRelativeForce`, `:815`), so an off-centre COM creates **no** tipping
   moment. Expected consequence: custom frames always fly "balanced" whatever the
   layout. Confirm, and tell the user. This is a realism limit, not necessarily a bug.
4. **Motor saturation.** Max thrust per motor = kf · ωmax² = 9.13e-6 · 1256.6² ≈
   14.4 N, or 57.7 N for four. Altitude hold can ask for up to (1 + 1.8) × hover
   thrust (`maxVerticalThrustFraction`, `:152`), so frames heavier than about
   57.7 / (2.8 · 9.81) ≈ 2.1 kg saturate on climbs, and above about 5.9 kg they
   cannot hover at all. Check how saturation (`:757-768`) behaves for a heavy frame.
5. **Launch is kinematic.** `IsLaunching` (`:580-623`) moves the body with
   `rBody.Move` and skips physics until within 0.5 m of the target. Only then does
   altitude hold start (`TargetAltitude` = current Y). Any "climb" must be judged
   after the "[Flight] Launch complete" log. Also note: the launch-complete check
   at `:692` looks unreachable during launch, since `:580` returns first. Confirm.
6. **Altitude hold** (`:681-706`): PD on altitude with `Kp_alt 0.5`, `Kd_alt 0.8`,
   plus tilt compensation (`:711-718`). Check for steady-state error or drift
   while hovering.
7. **Position-hold braking hides drift.** With no stick input the code applies
   heavy linear drag (`:837-854`) and **snaps horizontal velocity to zero below
   0.08 m/s** (`:843-847`). Slow sideways drift is masked by design, so the hover
   test must not rely only on position.
8. **Sign conventions.** Check roll/pitch/yaw signs with small step inputs: a small
   `StrafeSpeed` right should roll right and move right; `ForwardSpeed` forward
   should pitch nose-down and move forward; `YawSpeed` right should turn right.
9. Smaller items: Euler-angle reads (`:546-548`, fine near level),
   angular-velocity clamp at 3 rad/s (`:559-568`), propeller gyro term (`:821`),
   momentum rotation during yaw (`:905-916`).

Deliverable for A: a short findings list for the user (what is a bug, what is a
deliberate simplification, what is unknown), with line numbers and evidence.

## B. Symmetric X-quad hover test

Goal (Definition of done): "A symmetric X-quad with correct inputs hovers without
drift or climb".

### B.1 Setup

`TestSetup` is still active in `Main.unity`. On every load it sets
`SelectedSTLPath` to a file in the user's Downloads and backfills invalid nodes.
Going through the panel's **Build Drone** overwrites path, density and nodes, so
the custom test is unaffected. For the **default-drone** run, expect a
`[CustomFrameLoader] ... Using the default drone.` error first. That is caused by
TestSetup and is expected.

Run two configurations, the default drone and the symmetric custom quad:

```
STL:     tools/test-data/cube100.stl (use the absolute path)
Density: 1000
Nodes (paste into the panel; Type, X, Y, Z, Mass; CAD Z-up, metres):
Motor, 0.15, 0.15, 0, 0.05
Motor, -0.15, 0.15, 0, 0.05
Motor, 0.15, -0.15, 0, 0.05
Motor, -0.15, -0.15, 0, 0.05
Battery, 0, 0, 0, 0.5
```

Expected from the service: mass 1.7 kg, COM (0, 0, 0), moments
0.006167 / 0.006167 / 0.010667, armLength 0.2121, so armEffective = 0.15.

### B.2 Run

1. Play, open the panel, fill as above, Build Drone, Close, `BeginSimulation()`.
   Wait for DroneWorld.
2. Check the console for `[CustomFrameLoader] Mass properties:` and the `[Flight] Applied
   to rigidbody:` line (mass, COM, inertia, rotation, armLength, armEffective).
3. Arm: `Object.FindAnyObjectByType<Flight>().Arm();` (the keyboard equivalent is `L`).
   Wait for `[Flight] Launch complete - entering altitude hold`.
4. Sample for 20 s with no input. One way: a RunCommand that registers an
   `EditorApplication.update` callback, logs a line every 0.5 s with
   `Time.time`, position, `TargetAltitude`, Euler pitch/roll/yaw, linear and
   angular velocity, and all four `MotorSpeeds`, then unregisters itself after
   20 s. Read the lines back with `Unity_GetConsoleLogs`.
5. Repeat for the default drone (Begin without building; mass from the prefab).

### B.3 Pass criteria (suggested; agree with the user)

- Altitude stays within ±0.1 m of `TargetAltitude`, with no steady climb or sink trend.
- |pitch|, |roll| < 1°; yaw changes < 1° over 20 s.
- Horizontal position change < 0.1 m. Remember lead 7: braking hides slow drift.
- **The four `MotorSpeeds` are equal in steady hover** (within 1 %). For a
  symmetric quad this is the most sensitive signal: any persistent spread means
  the controller is fighting a torque. Expected value for 1.7 kg:
  ω = √(m·g / 4 / kf) = √(1.7·9.81/4/9.13e-6) ≈ 676 rad/s.

If it fails, say which lead from A explains it. Then propose a fix to the user
before touching `Flight.cs`.

## C. One shared motor-corner rule (refactor, no behaviour change)

CLAUDE.md Status → Next, item 6. The corner rule is: front = at or ahead of the
centre line (`z >= 0` in Unity), left = `x < 0`. A motor exactly on a line counts
as front / right. Today it is written out four times:

| Where | Frame | Uses it for |
|---|---|---|
| `Drone/MotorQuadrants.cs:41-51` `Classify(stlPosition)` | converts CAD Z-up → Unity, then tests | Panel validation and the loader's layout guard |
| `Drone/CustomFrameLoader.cs:140-148` `RepositionPropellers` | already converted to Unity | Which motor position goes to which corner |
| `Drone/PropellerSpin.cs:59-71, :86-89` `FindPropellers` | Unity drone-local | Spin direction, `motorIndices`, log label |
| `Drone/PropellerSpin.cs:126-136` `SetPropellerPositions` | Unity drone-local | Which propeller moves to which target |

### Plan

1. **Ask the user for approval to edit `CustomFrameLoader.cs`** (only
   `RepositionPropellers`; no protocol or Awake changes).
2. In `MotorQuadrants`, add a non-converting overload, e.g.
   `public static Quadrant ClassifyUnity(Vector3 unityPosition)`, holding the one
   copy of the rule (`z >= 0f` front, `x < 0f` left). Make
   `Classify(stlPosition)` call `ClassifyUnity(StlMeshLoader.ConvertZUpToYUp(stlPosition))`.
   Keep `Classify`'s existing doc comment about converting first.
3. `CustomFrameLoader.RepositionPropellers`: replace the two bools and the if-chain
   with a `switch (MotorQuadrants.ClassifyUnity(position))`. Keep the
   "already assigned" error path and all log text identical.
4. `PropellerSpin.FindPropellers` and `SetPropellerPositions`: use
   `MotorQuadrants.ClassifyUnity(localPos)` and switch on the result.

### Traps

- **Two different motor orders exist.** `MotorQuadrants.Quadrant` is FL=0, FR=1,
  BL=2, BR=3. `Flight.MotorSpeeds` / `PropellerSpin.motorIndices` are **FR=0, FL=1,
  BL=2, BR=3** (`Flight.cs:259`, `PropellerSpin.cs:84-87`), which match the sign
  tables in `Flight.cs:426-436`. **Never cast a `Quadrant` to a motor index.** Map
  it explicitly (FL→1, FR→0, BL→2, BR→3).
- Spin directions: FL CW, FR CCW, BL CCW, BR CW (`PropellerSpin.cs:64-71`),
  which match `Flight.motorYawSign`. Keep that mapping and the `meshInverted` flip.
- `SetPropellerPositions` takes `positions` in **FL, FR, BL, BR** order. Keep it.
- Boundary behaviour must not change: `z == 0` → front, `x == 0` → right.
- The log line `[PropellerSpin] Propeller_N ... → Front Left ...` is used by the
  README Visual Check. Keep its wording, or build the label from
  `MotorQuadrants.DisplayName` only if the text stays the same.

### Verify (no behaviour change)

1. Before editing, record the `[PropellerSpin]` lines (corner, target, sign) and
   `[CustomFrameLoader] Motor positions ...` for (a) the default drone and (b) the
   cube quad from B.1. After the change, they must be identical.
2. In Edit mode via RunCommand, check `ClassifyUnity` on boundary points:
   (0,0,0) → FrontRight; (-0.1,0,0) → FrontLeft; (0,0,-0.1) → BackRight;
   (-0.1,0,-0.1) → BackLeft. Check `Classify` (CAD) gives the same answers as
   before for the B.1 motors and for a plus-layout motor at (0, 0.15, 0).
3. Panel: a bad layout still shows the same validation message (e.g. two motors
   front-right).
4. Compile clean; no new console errors; flight scene still flies.
5. Update CLAUDE.md (Status item 6 done; the "Optional" note in the Architecture or
   Integration contract text if any), DEVLOG.md, and run `graphify update .`.

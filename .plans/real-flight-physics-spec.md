# Real Flight Physics: Spec

Status: **approved scope, not started.** Written 2026-10-19 after an interview with the project owner.
Every `Flight.cs` change in this spec still needs the owner's approval before it is applied (CLAUDE.md rule).

## 1. Goal

Make the simulated drone respond to commands the way a real PX4 quadcopter does, for **every** drone (the
default drone and every imported `.stl` frame). Student code tuned in the sim should then behave the same way on
the real UAV Neo drone.

Today the sim feels instantaneous:
- each motor reaches its new speed in one physics step;
- the four motor forces are lumped into one force at the centre of mass;
- several pieces of code set the drone's velocity directly, so it changes direction and stops almost at once.

After this work:
- motors spin up and down over a short delay;
- each motor pushes at its own position, so the centre of mass and motor layout matter;
- nothing sets the drone's velocity directly in flight; the drone turns and stops only by tilting, as a real
  drone does;
- the inner flight controller behaves like PX4 v1.17.0 with stock gains, using the real drone's speed and
  acceleration limits.

### Guiding principles (decided)

1. **Do not overfit to the program drone.** The program's drone can change, and students import their own
   frames. Use the real drone's **behaviour settings** (speed, tilt and acceleration limits), never its
   **hardware data** (mass, motors, propellers, tuned gains). Hardware values come from each imported frame.
2. **Do not change the labs or the meaning of `send_pcmd`.** Instructors designed the labs around today's
   command meaning. Only the physics underneath changes, so timing becomes real (tilting and stopping take
   longer). Labs are a separate, later step.
3. **Every phase has built-in tests that an agent runs through Unity MCP** and must pass before the owner
   approves the phase.

## 2. Decisions from the interview

| Topic | Decision |
|---|---|
| Scope | All drones, including the default drone and all lab modules. This replaces the CLAUDE.md done-item "Default drone … behave exactly as before" for flight feel. |
| Motor thrust for any frame | **Auto-size:** each motor's maximum thrust = frame weight ÷ (4 × `MPC_THR_HOVER`). With `MPC_THR_HOVER = 0.5`, total maximum thrust = 2 × weight, and the drone hovers at 50% throttle. No new UI. |
| Inner-loop gains | **PX4 stock defaults** (v1.17.0), not the program drone's tuned values. The student PID labs tune the student's own controller in Python, not these. |
| Frame that is unstable with stock gains | Fly it anyway (realistic). Warn on the **import panel status at Build Drone** and as an **in-flight HUD message**. |
| Physics rate | Unity stays at 50 Hz (`Fixed Timestep 0.02`). The motor model and the rate loop are **sub-stepped inside `Flight`**. |
| Where PX4 values live | A **bundled QGC `.params` file** plus a **`tools/` extractor** that makes it from a QGroundControl `.tlog`. Updating the drone means re-exporting and committing; no code change. |
| `send_pcmd` meaning | **Unchanged**: pitch/roll 1.0 = 35° tilt × `MaxSpeed`; throttle = thrust offset; all-zero = hold. |
| `send_pcmd` yaw | **1 rad/s (57.3°/s)** for 1.0 (owner decision 2026-10-20; was 200°/s). |
| Yaw banking (yaw input also rolls the drone) | **Removed.** Yaw only yaws, as in PX4. |
| Keyboard / controller flight | **PX4 POSCTL** behaviour on its own input path: sticks command velocity, releasing them brakes by tilting and holds position. Limits are the real ones × the `MaxSpeed` keys. |
| Braking | **Only by tilting**, as in PX4 position hold: the drone stops and holds, but not instantly. All brake forces and velocity "snaps" are removed. |
| Takeoff | **Physics takeoff** at the real climb rate to a **lower target height** (1.0 m, matching `neo_lab.DEFAULT_LAUNCH_HEIGHT`). Labs fly at 0.5–1.0 m, so the old 7.5 m launch is not needed. |
| Drag | **Scaled from the frame's size** (projected area from the mesh), times generic coefficients. |
| Custom frame ground contact | **Convex `MeshCollider`** from the imported mesh (changes a CLAUDE.md rule; see §9). |
| Sensor noise / estimator lag | **Out of scope**; listed as a follow-up. |
| Audit bugs (mirrored sign tables, wrong `MotorHealth` index) | **Fixed in phase 1.** |
| Delivery | **Phases**, each with agent-run tests and an owner approval. |
| Test runner | **Unity MCP only.** |
| Lab regression report | **Included.** The agent runs it if a Windows Python environment for the labs exists; otherwise the owner runs the listed commands. |

## 3. Background

### 3.1 What `Flight.cs` does today (read 2026-10-19)

| Location | Behaviour | Problem |
|---|---|---|
| `Flight.cs:792-797` | Motor speed = `sqrt(thrust / kf)`, set in the same step. | No spin-up delay. |
| `Flight.cs:819-840` | Sums the four thrusts into one `AddRelativeForce` at the centre of mass; torque from ±1 sign tables × one shared `armEffective`. | Motor positions and centre-of-mass offset have no effect. Sign tables are mirrored left/right (audit finding). |
| `Flight.cs:927-938` | Rotates the velocity vector by the yaw angle each step. | Direction of travel follows the nose instantly ("on rails"). |
| `Flight.cs:897-904` | Cross-brake drag, 6.0 N·s/m on sideways velocity. | Sharp direction changes. |
| `Flight.cs:859-875` | Position-hold brake drag, 4.5 N·s/m. | Stops in about 0.4 s (2 kg drone). |
| `Flight.cs:865-868` | Sets horizontal velocity to 0 below 0.08 m/s. | Hard stop. |
| `Flight.cs:581-590` | Clamps angular velocity by writing it. | Velocity write in flight. |
| `Flight.cs:602-644` | Kinematic launch with `rBody.Move` to 7.5 m. | No physics on takeoff. |
| `Flight.cs:691-693` | Yaw input adds a roll ("banking"). | Not PX4 behaviour. |
| `Flight.cs:510-514` | Attitude gains scaled by inertia (fix of 2026-10-14). | Replaced by PX4 control allocation. |
| `Flight.cs:805`, `CrashSystem.cs:516-522` | `MotorHealth` indexed by `Propeller_N` order, not motor order. | A crash can disable the wrong motor. |

`Rigidbody` rotation does go through PhysX with the imported inertia, so rotational inertia is already real; the
"instant" feel comes from the items above.

### 3.2 How commands arrive

- The keyboard/controller (`Drone.cs:145-158`) and Python `send_pcmd` (`PythonInterface.cs:405-408`) write the
  **same four fields**: `ForwardSpeed`, `StrafeSpeed`, `YawSpeed`, `VerticalSpeed`. `MaxSpeed` defaults to 0.5
  (`Flight.cs:35`). Keyboard backward input is scaled by 0.6 (`Drone.cs:148-149`).

### 3.3 The real drone (from QGroundControl telemetry logs)

Source: `C:\Users\Owner\OneDrive\Documents\QGroundControl\Telemetry\*.tlog` (5 logs, 27–29 July 2026). They
contain all 1,124 PX4 parameters (`PARAM_VALUE`), the firmware version (`AUTOPILOT_VERSION`) and flight data.

- **Firmware:** PX4 **v1.17.0**, official release. Airframe `SYS_AUTOSTART 4001` (Generic Quadcopter).
- **Estimation:** external vision (`EKF2_EV_CTRL 15`), optical flow, rangefinder height (`EKF2_HGT_REF 1`). Indoor drone.
- **Rotor geometry in PX4** is the generic placeholder (`CA_ROTOR*_PX/PY = ±1`, `CT 6.5`, `KM ±0.05`).
- **Rate gains are tuned** away from defaults (for example `MC_ROLLRATE_I 0.213`, `D 0.0065`, `K 0.924`). Not used (principle 1).

Parameters this spec uses (the real drone's behaviour settings):

| Parameter | Value | Used for |
|---|---|---|
| `MPC_THR_HOVER` | 0.5 | Motor auto-sizing |
| `MPC_VEL_MANUAL` | 1.0 m/s | Keyboard POSCTL top speed (× `MaxSpeed`) |
| `MPC_XY_VEL_MAX` | 1.0 m/s | Horizontal velocity cap |
| `MPC_Z_VEL_MAX_UP` / `_DN` | 0.6 / 0.6 m/s | Climb/descent caps, takeoff and landing speed |
| `MPC_TKO_SPEED` | 1.5 m/s | Takeoff speed (capped by `MPC_Z_VEL_MAX_UP`, so 0.6) |
| `MPC_LAND_SPEED` | 0.7 m/s | Landing speed (capped by `MPC_Z_VEL_MAX_DN`, so 0.6) |
| `MPC_ACC_HOR` / `_MAX` | 3.0 / 5.0 m/s² | POSCTL acceleration and braking |
| `MPC_ACC_UP_MAX` / `_DOWN_MAX` | 4.0 / 3.0 m/s² | Vertical acceleration limits |
| `MPC_JERK_MAX` | 8.0 m/s³ | POSCTL jerk limit |
| `MPC_TILTMAX_AIR` | 45° | Tilt limit of the velocity controller |
| `MPC_MAN_Y_MAX` | 150°/s | Keyboard POSCTL yaw rate |
| `MPC_POS_MODE` | 4 (acceleration-based) | Keyboard POSCTL stick behaviour |
| `MPC_XY_P`, `MPC_XY_VEL_P/I/D_ACC` | 0.95, 1.8 / 0.4 / 0.2 | Velocity/position loop (equal to PX4 defaults) |
| `MPC_Z_P`, `MPC_Z_VEL_P/I/D_ACC` | 1.0, 4.0 / 2.0 / 0.0 | Vertical loop (defaults) |
| `MPC_THR_MIN` / `_MAX` | 0.12 / 1.0 | Collective thrust limits |
| `MC_ROLL_P`, `MC_PITCH_P`, `MC_YAW_P`, `MC_YAW_WEIGHT` | 6.5, 6.5, 2.8, 0.4 | Attitude loop (defaults) |
| `MC_ROLLRATE_MAX`, `MC_PITCHRATE_MAX`, `MC_YAWRATE_MAX` | 220, 220, 250 °/s | Rate setpoint limits |
| `MC_AIRMODE` | 0 | Allocation desaturation mode |
| `CA_ROTOR*_KM` | 0.05 | Yaw moment per unit thrust (generic) |
| `THR_MDL_FAC` | 0 | Linear throttle-to-thrust |

**Rate gains used instead (PX4 v1.17.0 stock defaults, verify in phase 0):** `MC_ROLLRATE_P/I/D/K = 0.15 / 0.2 /
0.003 / 1.0`, same for pitch; `MC_YAWRATE_P/I/D/K = 0.2 / 0.1 / 0.0 / 1.0`; all `*_FF = 0`.

### 3.4 Command meaning on the real drone (for reference; not changed here)

On the real drone, `send_pcmd` is a **velocity** setpoint in PX4 OFFBOARD mode: 1.0 = 0.5 m/s and yaw 1.0 =
0.5 rad/s (`uav_neo_ros2_driver/config/mux.yaml`, `library/real/flight_real.py:26-27`). The sim keeps its tilt
meaning (decision). `labs/neo_lab.send_velocity` already bridges the two for horizontal motion; yaw is passed
through raw, so yaw is about 7× faster in the sim. The owner recalls the real yaw being raised to about 3 rad/s
after July; no file or log confirms it. Task 0.6 checks it on the drone.

### 3.5 Real flight data available

Armed time per mode: STABILIZED 244 s, POSCTL 164 s, ALTCTL 90 s, OFFBOARD 42 s. OFFBOARD flight is nearly all
hover (top horizontal speed 0.24 m/s), so it is good for hover checks only (decision: hover only, plus PX4-maths
checks).

## 4. Design

### 4.1 Configuration

- `UAVSim/Assets/StreamingAssets/Flight/uav-neo.params`: QGC parameter format, tab-separated
  (`1<TAB>1<TAB>NAME<TAB>VALUE<TAB>TYPE`), extracted from the newest `.tlog`. Committed.
- `UAVSim/Assets/StreamingAssets/Flight/flight-sim.json`: sim-only values that PX4 does not have. Each has a
  comment field stating its source or "estimate":
  - `motorTimeConstantUp` 0.0125 s, `motorTimeConstantDown` 0.025 s (estimate: PX4 Gazebo x500 model);
  - `launchHeight` 1.0 m (from `neo_lab.DEFAULT_LAUNCH_HEIGHT`);
  - `pcmdMaxTiltDeg` 35, `pcmdMaxYawRateDegS` 200 (today's `send_pcmd` meaning);
  - `dragCd` 1.0, `dragFillFactor` 0.5, `angularDragCoeff` (estimates);
  - `subSteps` 5 (250 Hz inner loop);
  - `stabilityBand` (filled in phase 5).
- `Drone/FlightConfig.cs` (new, static): loads both files once, falls back to built-in copies of the same values
  with an error log if a file is missing or unreadable, and exposes typed values. **The PX4 rate gains come from a
  built-in table of v1.17.0 defaults, never from the file** (principle 1).
- `tools/extract-px4-params.py` (new): reads one or more `.tlog` files and writes the `.params` file. Format
  notes: each tlog record is an 8-byte big-endian microsecond timestamp followed by one MAVLink v1 (`0xFE`) or v2
  (`0xFD`) packet. `PARAM_VALUE` is message 22, payload `float value, uint16 count, uint16 index, char[16] id, uint8
  type`. MAVLink v2 trims trailing zero bytes, so pad to 25 bytes. PX4 sends `INT32` (type 6) parameters as raw
  bits in the float field. `AUTOPILOT_VERSION` is message 148, with `flight_sw_version` (uint32) at payload offset
  16. The tool also prints the PX4 version and refuses to mix logs from different vehicles.

### 4.2 Plant model (what the drone physically is)

All values per frame, computed in `Flight.Start()` after mass properties are applied.

1. **Motor positions.** Read from the four propeller transforms that `PropellerSpin` already finds and places
   (default drone and custom frames alike), converted to Rigidbody-local space and keyed by corner with
   `MotorQuadrants.ClassifyUnity`. No change to `CustomFrameLoader`. Thrust axis = body up.
2. **Motor sizing.** `Tmax = m·g / (4 · MPC_THR_HOVER)` per motor. For visuals and telemetry, keep `MotorSpeeds` in
   rad/s with a per-frame `kf = Tmax / motorSpeedMax²`, so the propeller spin range is unchanged.
3. **Motor lag.** Per motor, per sub-step: `T += (u·Tmax − T)·(1 − exp(−dt/τ))`, where `τ = τ_up` if rising,
   else `τ_down`. `u` is PX4's normalised motor command (0–1, linear because `THR_MDL_FAC = 0`).
4. **Forces.** Each motor: `AddForceAtPosition(T_i · up, motorWorldPos_i)`. Yaw reaction:
   `AddRelativeTorque(±KM · T_i about up)`, with the spin direction from the corner (FL and BR clockwise, FR and
   BL counter-clockwise, as in the CLAUDE.md visual check). Keep the propeller gyroscopic term.
5. **Crash damage.** Multiplies each motor's `u` by `MotorHealth` for **that corner**. `CrashSystem.DetachArmGroup`
   is changed to index by motor corner (fixes audit finding 4).
6. **Drag.** Per body axis: `F = −½ · ρ · Cd · A_axis · |v_axis| · v_axis`, with ρ = 1.225 kg/m³ and
   `A_axis = projected bounds area × dragFillFactor`, computed from the frame's mesh (custom) or renderer
   bounds (default). Angular drag: `−angularDragCoeff · ω`. Set `rBody.linearDamping = rBody.angularDamping = 0`.
7. **Ground contact.** Custom frames get a convex `MeshCollider` on the imported mesh (Unity limits a convex hull
   to 255 faces and simplifies automatically). The default drone keeps its existing colliders. Exact place to add
   it (in `CustomFrameLoader.ReplaceDefaultVisual` or from `Flight`) is decided in phase 4; a `CustomFrameLoader`
   edit needs owner approval.

### 4.3 Controller (PX4 v1.17.0 behaviour)

Port the structure and behaviour, not the code. Cite the PX4 source file in each class header. All inner loops
run `subSteps` times per `FixedUpdate`; inside a step, the controller sees an angular rate propagated from the
torque it applied in the previous sub-step (`ω += I⁻¹·(τ − ω×Iω)·dt_sub`). PhysX receives the sub-step-averaged
force and torque once per `FixedUpdate`.

| Layer | PX4 reference | Behaviour |
|---|---|---|
| Rate loop | `src/lib/rate_control/rate_control.cpp` | `τ_norm = K·(P·e + I·∫e − D·ω̇) + FF·ω_sp`, D on measurement, integrator limit (`MC_RR_INT_LIM` default). Output normalised torque. |
| Attitude loop | `src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp` | Quaternion error, reduced attitude with `MC_YAW_WEIGHT`, `rate_sp = P·e`, limited by `MC_*RATE_MAX`. |
| Velocity/position loop | `src/modules/mc_pos_control/PositionControl/PositionControl.cpp` | Velocity PID to acceleration, hover-thrust normalisation, tilt limit `MPC_TILTMAX_AIR`, thrust limits; acceleration → attitude and collective thrust (`ControlMath::thrustToAttitude`). |
| POSCTL sticks | `FlightTaskManualAcceleration` (`MPC_POS_MODE 4`) | Stick → acceleration with velocity limit, jerk limit and braking at `MPC_ACC_HOR`; position lock once stopped. |
| Allocation | `ActuatorEffectivenessRotors`, `ControlAllocationSequentialDesaturation` | Effectiveness matrix from the **frame's real motor positions** (not the generic ±1), `KM`, normalised as PX4 does; `MC_AIRMODE 0` desaturation (yaw gives way first). |

Each layer is its own small class, so a Play-mode test (§6) can drive it on its own with fixed inputs.

### 4.4 Input paths

`Flight` gets an explicit `ControlMode`: `Pcmd` (Python) or `PosCtl` (keyboard/controller). Phase 3 finds how
`Drone.cs` and `PythonInterface` decide who owns the inputs today and sets the mode in that one place.

**`Pcmd` (meaning unchanged, physics new):**
- pitch/roll with either |value| ≥ 0.05 → attitude setpoint, tilt = value × 35° × `MaxSpeed` (same signs as today)
  → attitude and rate loops.
- pitch and roll both < 0.05 → hold: velocity setpoint 0, braking through the velocity loop (acceleration limited by
  `MPC_ACC_HOR`), then position lock. This also applies to pure vertical flight (today that forced level attitude
  with no hold; the hold is the realistic equivalent and is noted in the DEVLOG).
- yaw → yaw-rate setpoint = value × 1 rad/s (`pcmdMaxYawRateDegS`). **No banking.**
- throttle → collective thrust offset as today (value × `MaxSpeed` × `maxVerticalThrustFraction` × hover thrust);
  0 → altitude hold through the PX4 vertical loop, limited by `MPC_Z_VEL_MAX_*`.

**`PosCtl` (keyboard/controller):**
- sticks → horizontal velocity up to `MPC_VEL_MANUAL × MaxSpeed` (0.5 m/s by default, 1.0 m/s at full), backward
  ×0.6 as today; triggers → climb rate up to `MPC_Z_VEL_MAX_UP/DN × MaxSpeed`; yaw → up to `MPC_MAN_Y_MAX`
  (150°/s, not scaled, like today's yaw).
- release → brake by tilting and hold position.

### 4.5 Takeoff and landing

- **Arm / takeoff:** motors spin up from 0 through the lag model; the vertical loop climbs at
  `min(MPC_TKO_SPEED, MPC_Z_VEL_MAX_UP)` = 0.6 m/s to ground + `launchHeight` (1.0 m), about 2 s. Keep the
  `[Flight] Launch complete` log and the rule that horizontal input ends the launch early. Remove `rBody.Move`,
  the rotation write and the velocity writes from `Arm()`. `Relaunch()` after a crash respawn may still zero
  velocity, because a respawn is a teleport.
- **Land:** descend at `min(MPC_LAND_SPEED, MPC_Z_VEL_MAX_DN)` = 0.6 m/s. Disarm when ground contact and low
  vertical speed persist for 0.5 s (a simple PX4-style land detector). Motors spin down through the lag model;
  `FinishDisarm()` no longer writes velocities.

### 4.6 Code removed from `Flight.cs`

Yaw velocity rotation (section 10), cross-brake, position-hold brake drag, dead-zone snap, angular-velocity write
clamp (replaced by rate limits; `rBody.maxAngularVelocity` stays as an engine safety cap), kinematic launch,
banking, ±1 sign tables, inertia gain scaling and its fields. After phase 2, `Flight.cs` writes
`rBody.linearVelocity` / `angularVelocity` only in `Relaunch()` (respawn).

### 4.7 Stability warning

- **In flight (exact):** after mass properties are applied, compute the normalised-torque authority per axis
  (maximum torque from allocation ÷ inertia). Outside `stabilityBand` → `LevelManager.ShowMessage` on the existing
  HUD line, plus a Console warning. Also warn if sustained attitude oscillation is detected (sign changes of the
  rate error above a threshold for 2 s).
- **At Build Drone (estimate):** `ImportPageUI` cannot see the inertia yet (the mass service is called in the
  flight scene). Use an estimate from the node table and the mesh bounds, and word it as "may be unstable". Open
  item 1 offers a more exact option.
- `stabilityBand` is measured in phase 5 by sweeping test frames with the stock gains.

## 5. Phases

Each phase: implement → run its tests through Unity MCP → run the hover regression and the lab regression report
→ DEVLOG entry with results → owner approval → commit. Work on a new branch from the current branch,
`feature/real-flight-physics`.

### Phase 0: Foundations (no flight change)

- 0.1 `tools/extract-px4-params.py`; generate `uav-neo.params` from the newest `.tlog`; commit both.
- 0.2 `flight-sim.json` with the values in §4.1.
- 0.3 `Drone/FlightConfig.cs` loader with fallbacks.
- 0.4 Confirm the PX4 v1.17.0 stock defaults for every `MC_*` gain used, from the v1.17.0 parameter metadata
  or source, and record where they came from in `FlightConfig.cs`.
- 0.5 **Baseline:** run the lab regression report (§7) and record hover traces for the default drone, the 100 mm
  cube (`tools/test-data/cube100.stl`), the 7-inch fixture, and the light 1.2 kg and 0.28 kg frames from DEVLOG
  2026-10-14.
- 0.6 Ask the owner to check `max_yaw_rate` in `config/mux.yaml` on the drone's Raspberry Pi; record the answer.
  Answered 2026-10-20: yaw 1.0 = 1 rad/s in the sim.
- Tests: `FlightConfigTests` (file loads; every required key present; missing file falls back and logs one
  error; rate gains equal the built-in defaults even if the file differs).

### Phase 1: Motors (lag, per-motor forces, audit bugs)

- Motor positions from `PropellerSpin`, motor sizing, lag model, `AddForceAtPosition` per motor, yaw reaction from
  corner, `MotorHealth` by corner (`CrashSystem` change), remove sign tables.
- The existing attitude controller stays for this phase (it is replaced in phase 2), driving the new allocation.
- Tests:
  - `MotorLag_StepReaches63PercentInTau`: command step 0 → 1 on one motor in a frozen test harness; 63% at τ_up ±10%, spin-down at τ_down ±10%.
  - `Hover_ThrottleIsThrHover`: default drone, cube, 7-inch: mean `u` = 0.50 ± 0.02; altitude error < 2 cm over 20 s; drift < 5 cm.
  - `OffCentreCom_MotorsShareByLeverRule`: cube with the battery at CAD (0.1, 0, 0) hovers level within 1°, and the motor thrusts match the lever-rule split (the test that the 2026-10-13 hover test could not do).
  - `Yaw_RightCommandYawsRight` and `Roll/Pitch` sign checks per corner.
  - `Crash_DetachedArmLosesThatCornersMotor`: disable FL → that motor's thrust is 0 and the drone dips toward FL.

### Phase 2: PX4 controller and removal of velocity overrides

- Rate, attitude, velocity/position loops and allocation (§4.3) with sub-stepping; `Pcmd` path (§4.4);
  remove everything in §4.6 except takeoff/landing (phase 4).
- Tests:
  - `NoVelocityWritesInFlight`: reflection/IL or source scan: `Flight.cs` assigns `linearVelocity`/`angularVelocity` only in `Relaunch`; and a runtime probe that velocity never changes by more than the force-implied amount between steps.
  - `PcmdTilt_StepResponse`: pitch 0.5 → reaches 17.5° × `MaxSpeed`; time to 63% within ±25% of 1/`MC_PITCH_P` (≈0.15 s); overshoot < 10%; rate never above `MC_PITCHRATE_MAX`.
  - `PcmdYaw_RateAndNoBank`: yaw 1.0 → 57.3°/s ±5% steady; roll stays within 1° of level.
  - `TurnKeepsMomentum`: fly at 1 m/s, yaw 90°: the velocity direction lags the heading (angle between them > 20° at some point) and then converges as the drone tilts.
  - `Hold_StopsByTilting`: from 1 m/s, release: peak deceleration ≤ `MPC_ACC_HOR_MAX` + 10%; stopping distance ≥ v²/(2·`MPC_ACC_HOR_MAX`); drone tilts backwards while stopping; position held within 0.1 m after 5 s.
  - `LightFrames_DoNotClimbAway`: 1.2 kg and 0.28 kg frames hover (regression of the 2026-10-14 fix).
  - `Saturation_YawGivesWayFirst`: full roll + full yaw command: roll torque is preserved, yaw is reduced.

### Phase 3: Keyboard POSCTL

- `ControlMode` and the `PosCtl` path (§4.4); find and use the single place that decides input ownership.
- Tests:
  - `PosCtl_FullStickSpeed`: steady speed = `MPC_VEL_MANUAL × MaxSpeed` ±5% at `MaxSpeed` 0.5 and 1.0.
  - `PosCtl_ClimbRateLimit`: climb ≤ `MPC_Z_VEL_MAX_UP × MaxSpeed` +5%.
  - `PosCtl_ReleaseBrakesAndHolds`: as `Hold_StopsByTilting`.
  - `Pcmd_UnchangedByPosCtl`: Python `send_pcmd` still uses the tilt meaning while keyboard uses POSCTL.

### Phase 4: Physics takeoff and landing, ground contact

- §4.5 and the convex `MeshCollider` for custom frames.
- Tests:
  - `Takeoff_ClimbsUnderThrust`: from rest on the ground; motor thrust rises with lag; climb rate ≤ 0.6 m/s +5%; reaches ground + 1.0 m ±0.1 m; logs `Launch complete`; no `rBody.Move` call.
  - `Land_DescendsAndDisarms`: descent ≤ 0.6 m/s +5%; disarms within 1.5 s of touchdown; drone rests on its collider (custom frame does not sink or float, ±1 cm).
  - `CustomFrame_RestsOnConvexCollider`: 7-inch and cube rest level on the ground before arming.

### Phase 5: Drag and stability warnings

- §4.2 item 6 and §4.7; sweep test frames (scaled cubes and the 7-inch fixture over a range of inertias) to set
  `stabilityBand`.
- Tests:
  - `Drag_ScalesWithArea`: two frames, same mass, 2× area → terminal speed ratio ≈ 1/√2 (±10%) at a fixed tilt.
  - `StabilityWarning_ShownForOutOfBandFrame`: a deliberately out-of-band frame shows the HUD message and the Console warning; an in-band frame shows none.
  - `BuildDrone_WarnsForOutOfBandEstimate`: the panel status shows the warning for the same frame.

## 6. Test harness rules

- All tests live in `UAVSim/Assets/Tests/PlayMode/` (assembly `UAVSim.Tests.PlayMode`) and follow
  `Drone7InchTests.cs`: reach game types by reflection (an asmdef cannot reference `Assembly-CSharp`), load
  `Main`, fill the `CustomFrameLoader` statics, call `MainMenu.BeginSimulation`.
- Tests that need the mass service report **Ignored** when it is down, as `Drone7InchTests` does.
- New fixtures go in `Fixtures~/` with the script that generates them.
- The agent runs tests **through Unity MCP** (Editor open) and pastes the summary (pass/fail/ignored counts and
  failing names) into the DEVLOG entry.
- Every test prints its measured values, not only pass/fail, so that the DEVLOG can record them.

## 7. Lab regression report

**Purpose:** show which existing labs behave differently after each phase. Lab code is never edited.

1. Phase 0: run each lab's `main_solution.py` against the current sim; record whether it meets its goal (for
   example "holds within 0.25 m of 1.0 m", "finishes the square", "reaches the gate") and its key timings.
2. After each phase: run them again.
3. Write `docs/lab-regression-<date>.md`: one row per lab, columns before / after / note; highlight labs that
   passed before and fail now.

**Who runs it:** the agent, if a Windows Python environment with the lab dependencies (the `drone-student`
library, `numpy`, `opencv-python`) exists, starting the sim through Unity MCP. Otherwise the plan step stops and
gives the owner the exact commands.

## 8. Out of scope (follow-ups)

- Sensor realism: noise, bias and estimator lag on `uav.physics` readings (like RACECAR Realism mode).
- Changing the labs, `neo_lab.send_velocity`, or `send_pcmd`'s meaning (separate step).
- Matching `send_pcmd` yaw to the real drone.
- Rotor drag, ground effect, wind, battery sag.
- Non-quadcopter layouts.
- Raising Unity's fixed timestep.

## 9. Documentation updates (part of the phases)

- CLAUDE.md:
  - the done-item "Default (non-custom) drone and all existing lab modules behave exactly as before" → "Lab code
    and `send_pcmd` meaning unchanged; flight timing is realistic for all drones (see this spec)";
  - "EXPLICITLY NOT BUILDING: … `MeshCollider`" → allow a convex `MeshCollider` for physics contact (preview
    still not built);
  - `Flight.cs` description (gain scaling replaced by PX4 allocation) and the Open questions entry for the audit
    findings;
  - new section "Flight model" pointing to `FlightConfig`, the params file and the extractor.
- `docs/flight-audit-2026-10-13.md`: mark findings fixed by phase.
- README Manual Flight Controls: keyboard now flies like PX4 position mode (speeds, braking); takeoff height 1.0 m.
- DEVLOG entry per phase, with test output.

## 10. Open items

1. **Build-time warning accuracy.** The estimate in §4.7 ignores the frame mesh's own inertia. An exact option is
   a C# port of the mesh inertia integral (about 60 lines, duplicates the Python maths) or asking the mass
   service from the panel (a second caller of the locked protocol; protocol unchanged). Decide in phase 5.
2. **Real yaw rate** (task 0.6): 0.5 rad/s in every published config and log; the owner recalls about 3 rad/s.
3. **`MaxSpeed` and POSCTL:** the default `MaxSpeed` 0.5 gives a 0.5 m/s keyboard top speed. Confirm this feels
   right after phase 3.
4. **Motor time constants** are estimates (PX4 Gazebo x500). Revisit if real motor data or logs become available.
5. **Pure vertical `send_pcmd`** now holds position instead of forcing level attitude without a hold (§4.4). Watch
   for it in the lab report.

## 11. Risks

| Risk | Mitigation |
|---|---|
| Stock PX4 gains are unstable at the sim's 50 Hz physics for some frames | Sub-stepping (§4.3); `stabilityBand` sweep; warnings. If the default drone itself is unstable in phase 2, stop and report before tuning anything. |
| Labs fail because timing is now real | Lab regression report each phase; labs are not changed here; findings go to the instructors. |
| Sub-step propagation and PhysX disagree | Test `PcmdTilt_StepResponse` and hover tests catch drift; compare sub-step-predicted ω with PhysX ω each step and log the largest difference. |
| Convex hull of an odd frame is a poor ground contact | Visual check step; fallback to a bounds box if the hull has errors. |
| `Drone.cs` and Python both write the inputs every frame | Phase 3 finds the single owner before adding `ControlMode`. |

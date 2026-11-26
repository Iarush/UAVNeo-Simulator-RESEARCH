# POSCTL Yaw Reversal, Tilt Limit and Rotor Drag: Spec

Interview date: 2026-11-22. Owner of decisions: the project owner. Branch: `feature/real-flight-physics` (continue on it, as phase 6 of `.plans/real-flight-physics-spec.md`).

This spec is written for an orchestrator agent. **Work to the end without stopping for approval** (owner decision). Report results at the end. Report honestly any acceptance check that fails, with the measured numbers.

---

## 1. Problem (as reported)

The owner flies the **default drone** with the keyboard (PX4 POSCTL path, `Flight.ControlMode.PosCtl`) at the default MaxSpeed (0.5).

1. Hold forward, so the drone speeds up in +X.
2. Keep holding forward and yaw 180°.
3. Expected: the drone turns around quickly and goes in −X, as the real UAV Neo does.
4. Seen: momentum carries it in +X for a long time after the nose has turned around.

The owner also wants tilt limited to 15° (the real drone's `MPC_MAN_TILT_MAX`), because the drone leans too far.

Owner's reference: the owner flies real Neos at about **5 m/s** maximum, and at that speed they feel snappy. 15 m/s is "way too fast".

## 2. Diagnosis (from reading the code on 2026-11-22; phase 0 must confirm it)

- Keyboard top speed is `KeyboardMaxSpeed` (30 m/s) × MaxSpeed (0.5), which gives **15 m/s**. The real drone's `MPC_VEL_MANUAL` is 1 m/s.
- The keyboard path works like PX4 **`MPC_POS_MODE` 3** (smoothed velocity). The stick is a velocity target in the heading frame (`Flight.cs:767`). The setpoint ramps toward it at `MPC_ACC_HOR_MAX` = 5 m/s² with jerk `MPC_JERK_MAX` = 8 m/s³ (`Flight.cs:774-785`).
- The real drone runs **`MPC_POS_MODE` = 4** (acceleration based, `StickAccelerationXY`), which has a strong virtual drag on the setpoint (2·`MPC_ACC_HOR`/`MPC_VEL_MANUAL`) and a brake boost when the sticks are released. The sim does not implement mode 4.
- The only drag is weak quadratic body drag (`Flight.ConfigureDrag`, `dragCd` 1.0). There is **no rotor drag**, so the drone glides. A real quad at speed has strong drag, mostly from the rotors, which brakes it without any tilt.
- Tilt limit in the PX4 loops is `MPC_TILTMAX_AIR` = 45° (`Flight.cs:800`, `:806`, `:1413`). send_pcmd tilt is 35° × MaxSpeed × command (`Flight.cs:750-752`).
- **Physics floor.** Horizontal acceleration from thrust is at most g·tan(tilt): 2.63 m/s² at 15°, 9.81 m/s² at 45°. Reversing from +15 to −15 m/s therefore takes at least ~11 s at 15° with no drag. No controller tuning can beat this, and writing the velocity directly is ruled out (§3). The fix is a realistic speed, rotor drag and the real drone's stick mode, not an arcade override.

**Rejected (owner decision: realism must win):** rotating the drone's velocity with its heading ("yaw velocity rotation"), or any other direct write of `Rigidbody.velocity`. Phase 2 removed that behaviour on purpose. The rule "only `Relaunch()` writes velocities" stands.

## 3. Decisions

| # | Topic | Decision |
|---|---|---|
| D1 | Realism | Realism must win. No non-physical forces and no velocity writes. Every change must be sourced to PX4 v1.17.0 code, the real drone's params, or a published model. |
| D2 | Stick mode | Port PX4 v1.17.0 **`StickAccelerationXY`** (mode 4) exactly as the keyboard/controller POSCTL path. |
| D3 | Old mode | **Keep the current mode-3 path as a fallback.** Select it with `MPC_POS_MODE` from `uav-neo.params`: 4 → new path, 3 → current path. Any other value: log a warning once and use 4. The real drone's file says 4. |
| D4 | Keyboard speed | **5 m/s at the default MaxSpeed 0.5**, so `KeyboardMaxSpeed` becomes **10 m/s** (× MaxSpeed). This applies to modes 3 and 4. |
| D5 | MaxSpeed in mode 4 | Use `KeyboardMaxSpeed × MaxSpeed` **in place of `MPC_VEL_MANUAL`** in every mode-4 formula (velocity scale, drag coefficient, brake-boost `sqrt(MPC_VEL_MANUAL)` term). Keep `MPC_ACC_HOR` (3 m/s²) from the params file. Do not use PX4's dynamic velocity-constraint path for MaxSpeed, because it would also cut the acceleration at the default setting. Document this as a deliberate sim substitution, as `KeyboardMaxSpeed` already is. |
| D6 | Tilt limit | **15° everywhere**, read from `MPC_MAN_TILT_MAX` in `uav-neo.params`: the mode-4 `applyTiltLimit`, the mode-3 path, the PX4 position/velocity loop's `limitTilt` and integral clamp (`Flight.cs:800`, `:806`), takeoff, landing, hold, and send_pcmd. The owner chose this knowing that real PX4 POSCTL uses `MPC_TILTMAX_AIR` (45°). Document it as a deliberate deviation. |
| D7 | send_pcmd | Keep the formula (pitch/roll = command × `pcmdMaxTiltDeg` 35° × MaxSpeed) and **clamp the result at 15°**. Clamp the **combined** tilt (the magnitude of the pitch/roll vector, keeping its direction), so a diagonal command cannot reach 21°. Small commands are unchanged. Nothing else in send_pcmd changes. |
| D8 | Rotor drag | Add rotor drag to the plant: per rotor, opposing the airflow perpendicular to the rotor axis, scaled by that rotor's actual speed (Gazebo `multicopter_motor_model` form: `F = −|ω|·c·v⊥`). Use a **published coefficient** for a similar quad (e.g. PX4 Gazebo x500 `rotorDragCoefficient`, or PX4 SIH), with its source written in `flight-sim.json`. The sim's ω comes from its own `kf`, so convert the coefficient so the drag force per rotor at a given thrust and airspeed matches the source model, and show the conversion. Keep the existing body drag. Rotor drag applies to custom frames through the same per-motor code. |
| D9 | Climb rate | **Out of scope.** `KeyboardMaxClimbRate` (10 m/s × MaxSpeed) is unchanged. |
| D10 | Scope | Keyboard/controller POSCTL gets the mode-4 port and the new speed. send_pcmd gets only the 15° clamp (D7). The rotor drag and the tilt limit are physics and apply to every path. |
| D11 | Process | Measure first (phase 0), then build. No approval stops; the orchestrator oversees. Lab regression is still run once after all physics phases (existing owner decision), not in this work. |

## 4. PX4 v1.17.0 reference (fetched 2026-11-22, tag `v1.17.0`)

Source: `src/modules/flight_mode_manager/tasks/Utility/StickAccelerationXY.cpp` / `.hpp`, and `tasks/ManualAcceleration/FlightTaskManualAcceleration.cpp`. **Re-read the files at that tag before porting; do not port from this summary alone.**

`generateSetpoints(stick_xy, yaw, yaw_sp, pos, vel_sp_feedback, dt)`, in order:
1. `velocity_sc = min(MPC_VEL_MANUAL, slew-limited velocity constraint)`. `acceleration_sc = MPC_ACC_HOR × velocity_sc / MPC_VEL_MANUAL`, then **× 2** ("because of drag the average acceleration is half").
2. `limitStickUnitLengthXY`. The side/back scaling is skipped because `MPC_VEL_MAN_SIDE` and `MPC_VEL_MAN_BACK` are −1 in the params.
3. `rotateIntoHeadingFrameXY(stick, yaw, yaw_sp)`. **Read `lib/sticks/Sticks.hpp`** to see which of yaw and yaw_sp it uses and how it handles NaN. Map that onto the sim's `yawLock` / heading (`Flight.cs:661-666`). This is the step that decides how the stick direction follows the nose during a yaw.
4. `acc_sp = stick × acceleration_scale`.
5. `drag = calculateDrag(acceleration_scale / velocity_scale, dt, stick, vel_sp)`. The brake-boost `AlphaFilter` (time constant 0.8 s) moves toward `max(2, sqrt(MPC_VEL_MANUAL))` when the stick is released and toward 1 otherwise. Below 1 m/s, while braking, a sqrt boost is blended in (`math::sqrt_linear`, `brake_scale = interpolate(boost, 1, 2, 0, 1)`).
6. Sign-change guard: if the acceleration is zero or changed sign on an axis, and `|vel_sp|² < |drag·dt|²`, zero both drag and vel_sp.
7. `acc_sp -= drag`, then `applyJerkLimit` (per-axis slew at `MPC_JERK_MAX`, scaled by the direction of the increment), then `applyTiltLimit` (scale down to `tan(tilt_limit)·g`; in the sim, tilt_limit = `MPC_MAN_TILT_MAX`, D6).
8. `vel_sp += acc_sp·dt`, then `lockPosition`: lock when vel_sp is zero, unlock when moving and reset vel_sp from the velocity feedback.

The output (position, velocity and acceleration setpoints) goes into the existing PX4 position/velocity loop (`Flight.cs:794-807`) as the trajectory setpoint, with the acceleration as feed-forward. That is how the current mode-3 ramp already feeds it. `FlightTaskManualAcceleration::activate()` resets the velocity and acceleration from the last setpoint; do the same when keyboard control takes over (e.g. after takeoff completes).

Expected feel at the default (vel scale 5 m/s, `MPC_ACC_HOR` 3): stick acceleration 6 m/s², virtual drag 1.2 s⁻¹, both cut to 2.63 m/s² by the 15° limit. Steady speed is 5 m/s.

## 5. Phases

Run all tests with the Unity CLI, Editor closed (owner rule):
`"C:\Program Files\Unity\Hub\Editor\6000.4.5f1\Editor\Unity.exe" -batchmode -projectPath UAVSim -runTests -testPlatform PlayMode -testFilter <regex> -testResults <xml> -logFile <log>`

### Phase 0: Measure the current behaviour (no flight change)

Add a PlayMode test (new file `UAVSim/Assets/Tests/PlayMode/FlightYawReversalTests.cs`, category `YawReversal`) that flies the reported manoeuvre on the **default drone** in PosCtl and writes a CSV trace (time, position, velocity, velocity setpoint, acceleration setpoint, attitude tilt, heading, motor thrusts) to the test results folder.

The manoeuvre:
1. Launch and wait until hover is steady.
2. Set ForwardSpeed to 1 until the speed along the heading is steady (within 2 % for 1 s).
3. Keep ForwardSpeed at 1 and set YawSpeed to 1 until the heading has turned 180°. Then set YawSpeed to 0 and keep ForwardSpeed at 1.
4. Run until the velocity along the new heading has been steady for 1 s, or 30 s, whichever is first.

Metrics, measured from the start of the yaw:
- **t_half**: time until the velocity along the new heading is ≥ +0.5 × the cruise speed.
- **overshoot**: furthest travel along the original direction after the yaw starts.
- max tilt, altitude change, and the largest gap between the velocity setpoint and the actual velocity.
- **floor**: the minimum t_half the physics allows. Integrate a point mass whose horizontal acceleration is at most g·tan(tilt limit) plus the active drag, always pushing toward the new-heading velocity, with the same yaw rate. Show this calculation in the report.

Run it now at MaxSpeed 0.5 (15 m/s, today's code) to reproduce the owner's report. Write the numbers into the DEVLOG.

**Interpretation rule:** if t_half is much bigger than the floor (rule of thumb: > 1.5×), or the velocity lags the setpoint by more than about 1 s, then the controller is also at fault. Find the cause and fix it in a PX4-faithful way. Do not retune gains away from PX4 stock without writing down why. If t_half is close to the floor, the physics is the limit, and phases 1–3 are the fix.

### Phase 1: Tilt limit 15° (D6, D7)

- Add `MPC_MAN_TILT_MAX` to what `FlightConfig` reads. Use it for `tiltMax` in every PX4-loop path and for the send_pcmd clamp (combined tilt magnitude).
- Keep `MPC_TILTMAX_AIR` loaded but unused, with a comment explaining why.

### Phase 2: Rotor drag (D8)

- Find and cite the published coefficient. Convert it to this sim's motor model and record the value and source in `flight-sim.json` (e.g. key `rotorDragCoeff`).
- Apply it per motor in the sub-stepped motor loop, at each motor's position, using the airflow at the rotor (body velocity plus ω × r), with the component along the rotor axis removed.
- At zero airspeed the drag is zero, so hover traces must not change.

### Phase 3: Mode-4 port, speed change and mode switch (D2–D5)

- `KeyboardMaxSpeed` = 10.
- Port `StickAccelerationXY` as in §4, behind `MPC_POS_MODE` (read from params; 4 → new, 3 → the existing ramp code, kept as it is apart from the speed and tilt changes).
- Release behaviour comes from mode 4 (virtual drag + brake boost + position lock) and replaces the mode-3 brake ramp only when in mode 4.

### Phase 4: Verify and document

Run every suite. Rerun the phase-0 test after the change. Update the docs (§7).

## 6. Acceptance checks

All on the default drone unless stated.

1. **Yaw reversal (the main check):** MaxSpeed 0.5 (5 m/s), mode 4, the phase-0 manoeuvre. **t_half ≤ 4.0 s.** Also report t_half and the overshoot at MaxSpeed 1.0 (10 m/s); that run reports numbers only and has no pass/fail.
2. **Cruise speed:** steady forward speed at MaxSpeed 0.5 is 5.0 m/s ± 5 %.
3. **Tilt:** attitude tilt never exceeds 15.5° in any test: keyboard full stick, yaw reversal, release braking, takeoff, landing, and send_pcmd at command 1.0 on pitch and roll together at MaxSpeed 1.0.
4. **Release braking:** from 5 m/s, release all sticks. The drone stops and the position locks. Report the stopping distance and time. There must be no oscillation around the lock point (position error settles monotonically or within 2 overshoots of < 0.1 m).
5. **Altitude during the reversal:** altitude change ≤ 0.5 m.
6. **Hover unchanged:** the `Baseline` category (20 s hover trace) gives the same results as before the change.
7. **Custom frames:** existing custom-frame tests (`Drone7InchTests`, the light-frame hover cases) still pass, and the stability warning band (`FlightDragStabilityTests`) still holds. If rotor drag changes the expected values of the phase-5 drag tests, update them with the reason written down.
8. **Fallback:** with `MPC_POS_MODE` forced to 3 in the test, the existing `FlightControlTests` and `FlightPosCtlTests` pass. Update only the assertions that depend on `KeyboardMaxSpeed` (30 → 10) or on the 45° tilt; say which ones changed and why.
9. **Mode-4 unit tests:** new tests check the port against hand-computed values from the PX4 formulas: the steady speed equals the velocity scale, the drag coefficient is 2·ACC_HOR/vel_scale, the brake boost settles toward max(2, √vel_scale), and the jerk and tilt limits hold.
10. **send_pcmd:** commands whose tilt is below 15° give exactly the same attitude setpoint as before (bit-identical).

## 7. Documentation updates (part of the work)

- `CLAUDE.md`, section "Flight model": add a phase-6 bullet covering mode 4, `MPC_POS_MODE` selection, 10 m/s × MaxSpeed, 15° everywhere (a deviation from PX4 POSCTL), rotor drag and its source. Fix the phase-3 bullet's "30 m/s".
- `.plans/real-flight-physics-spec.md`: add a phase-6 section that points to this spec.
- `DEVLOG`: phase-0 numbers, the floor calculation, the final numbers for every acceptance check, and the rotor-drag conversion.
- `README.md` / the controls table: correct any keyboard speed figure.

## 8. Out of scope

- Climb rate (D9), the yaw rate (`MPC_MAN_Y_MAX` 150°/s unchanged), the send_pcmd meaning other than the 15° clamp, the lab regression (run at the end of all phases, separately).
- Any arcade turn assist or velocity rotation (rejected, §2).
- Fitting drag from real flight logs: the QGC `.tlog` files are almost all hover. The owner may fly the real Neo fast later to calibrate.
- Changes to `CustomFrameLoader.cs` or the wire protocol (not needed).

## 9. Risks and things to watch

- **Mode-4 brake boost with the bigger velocity scale.** PX4 tuned it for `MPC_VEL_MANUAL` 1 m/s. With 5–10 m/s, `sqrt(vel_scale)` gives a boost of 2.2–3.2. Check acceptance item 4 for overshoot.
- **The 15° limit with mode 4:** the stick asks for 6 m/s², but only 2.63 m/s² is allowed, so the tilt limit is active most of the time. The jerk limit runs before the tilt limit in PX4, so watch for setpoint windup and correct it only in a PX4-faithful way.
- **Rotor-drag unit conversion** is easy to get wrong by orders of magnitude. Sanity check: at 5 m/s, rotor plus body drag on the default drone should be a few tenths of g at most, well below the 2.63 m/s² the tilt gives. Report the value.
- **Heading frame during the yaw** (§4 step 3): if the sim uses the locked yaw while yawing, the stick direction lags the nose. Match PX4's `rotateIntoHeadingFrameXY`.
- **Old-mode tests** depend on the 30 m/s constant through reflection (`FlightPosCtlTests.cs:14`). Changing the constant changes what those tests expect.

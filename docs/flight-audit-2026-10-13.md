# Flight.cs audit (task A), 2026-10-13

Read-only audit of `UAVSim/Assets/Scripts/Drone/Flight.cs` (927 lines), done against
the plan in `docs/handoff-flight-audit-and-motor-corners.md`. **No code was changed.**
Every fix below needs the user's approval first, because `Flight.cs` flies every drone
in every lab.

## How it was checked

- `Flight.cs` read in full. Also read: `PropellerSpin.cs`, the crash-damage parts of
  `CrashSystem.cs`, the rotation code in `CustomFrameLoader.cs` and `mass_composer.py`,
  `ProjectSettings/TimeManager.asset` (fixed step **0.02 s**), and the git diff of
  `Flight.cs` from `8f472b3` to `edc07f5`.
- **Unity (via MCP, Play mode, 2026-10-13/14):** Unity's rotation signs; the cube quad's
  inertia as applied to the `Rigidbody`; kick tests on the default drone; and step-by-step
  traces of two light custom frames (finding 1). Probes were temporary components added
  at runtime. No project file was changed. The symmetric-quad hover test (task B) was run
  by another session ("X quadrotor test"). Its results are summarised under "Hover test
  (task B)" below.
- **Offline replica:** `tools/flight-audit/flight_loop_replica.py` copies the roll/pitch,
  yaw and altitude loops from `Flight.cs` and integrates them the way PhysX does
  (semi-implicit Euler, 0.02 s). Run it with
  `python tools/flight-audit/flight_loop_replica.py`. The tables below come from it unless
  marked "Unity". Its first part models one axis at a time. Its last part couples all three
  axes and altitude through the shared motor mixer. Unity matched it where both were
  measured (default-drone yaw ringing, the light-frame yaw blow-up).

Unity facts measured in the Editor (left-handed, Y up, Z forward):

| Check | Result | Meaning |
|---|---|---|
| `Euler(30,0,0) * forward` | `(0, -0.5, 0.866)` | +pitch = nose down |
| `Euler(0,0,30) * right` | `(0.866, 0.5, 0)` | +roll = right side up (rolls left) |
| `Euler(0,30,0) * forward` | `(0.5, 0, 0.866)` | +yaw = turn right (clockwise from above) |
| `Cross(right, up)` | `(0, 0, 1)` | lift on the right side gives +roll torque |

## Findings

Legend: **BUG** = wrong behaviour. **LIMIT** = a deliberate simplification that limits
realism. **OK** = checked and correct. **UNKNOWN** = not settled.

### 1. BUG: light frames are unstable at the 50 Hz physics rate and climb away (confirmed in Unity)

**This explains the "keeps climbing" symptom.**

The gains are fixed (`Kp_rp = 10`, `Kd_rp = 0.06`, `Kp_yaw = 0.55`; `:123-132`) and do
not use the frame's inertia. With a 0.02 s step, the explicit loop goes unstable when
inertia is small:

- **Roll/pitch:** unstable when `I < (Kp·dt² + 2·(Kd + 0.005)·dt) / 4 ≈ 0.00165 kg·m²`.
- **Yaw:** unstable when `Iyy < (Kp_yaw + 0.005)·dt / 2 ≈ 0.00555 kg·m²`. The comment at
  `:130` gives the yaw time constant as 0.011 s, which is shorter than the 0.02 s step.
  That is the cause.

On its own, an unstable axis ends up flipping sign every physics step (25 Hz), bounded
only by the 3 rad/s clamp (`:559-568`). **Coupled to the mixer it is much worse.** The
unstable axis demands more torque than the motors can give, so every step two motors
are pinned at 0 and two at maximum. The rule at `:757-762` then raises all four motors to
lift the lowest one back to zero. Total thrust ends up several times hover, and the drone
**climbs out of control**. Altitude hold cannot stop it: its −1.8 × hover limit is far
smaller than the boost.

**Unity traces** (temporary probe, fixed steps after `[Flight] Launch complete`, no input;
frames built from `tools/test-data/cube100.stl`):

| Frame | Mass | I body X / Y(yaw) / Z | What happened |
|---|---|---|---|
| cube100 at 1000 kg/m³ + 4 × 0.05 kg motors at (±0.05, ±0.05, 0) | 1.20 kg | 0.00217 / **0.00267** / 0.00217 | Roll and pitch stayed at 0.0° throughout. Yaw rate went 0.01 → −0.03 → 0.10 → −0.30 → 0.96 rad/s (×−3.2 per step; predicted ×−3.16), then pinned at ±3.00 alternating. From step 19 the motors alternated (1257, 0, 1257, 0) / (0, 1257, 0, 1257). **Level, climbing**: +3.1 m/s at 0.6 s, +32.8 m/s at 3 s, 45 m up, then it hit something and fell. |
| cube100 at 100 kg/m³ + 4 × 0.02 kg motors at (±0.08, ±0.08, 0) + 0.1 kg battery | 0.28 kg | **0.000679 / 0.001191** / 0.000679 | Diverged from step 11. Yaw pinned at ±2.98 alternating, motors switching between 0 and 1257, then tilted about 16° / 21°. Climbing at 95 m/s after 2.2 s, then "HARD crash ... broken apart" about 111 m up. Respawned and repeated on every launch. |

The offline model in the table below shows the single-axis case:

| I (kg·m²) | 0.01 rad/s kick, max angle over the last 1 s of 6 s |
|---|---|
| 0.0012 | 3.89° (sustained) |
| 0.0016 | 3.48° (sustained) |
| 0.0017 | 0.000° (decays) |
| 0.0020 | 0.000° (decays) |

| Iyy (kg·m²) | yaw rate 3 s after a 0.5 rad/s kick |
|---|---|
| 0.0050 | 3.0 rad/s (pinned at the clamp) |
| 0.0060 (stock) | 0.0000 (decays, but rings: each step flips sign and shrinks only 15%) |
| 0.0107 (cube quad) | 0.0000 |

**Unity, default drone, 0.5 rad/s yaw kick:** 0.50, −0.42, 0.36, −0.31, 0.26 … rad/s, a
factor of −0.85 per step, exactly as predicted. It settled in 0.5 s.

The stock drone (0.003 / 0.006 / 0.003) sits just inside both limits. Its yaw inertia
is only 8% above the yaw limit. A typical small student frame does not. Rough estimate for a
250 mm quad with 30 g motors at ±88 mm: Ixx ≈ 0.0012, Iyy ≈ 0.0024. Both are below the
limits, so it would behave like the 0.28 kg frame above. **Yaw is the tighter limit.** A
frame can have healthy roll/pitch inertia and still climb away, as the 1.2 kg frame did:
motors close to the centre give little yaw inertia.

Proposed fix (needs approval): scale the gains by the frame's inertia, e.g.
`Kp_rp' = Kp_rp · Ixx / 0.003`, `Kd_rp' = Kd_rp · Ixx / 0.003`,
`Kp_yaw' = Kp_yaw · Iyy / 0.006` (with Ixx/Iyy/Izz taken as the body-axis diagonal of
R·diag(I)·Rᵀ). Every frame would then respond like the stock drone, and the stock drone
would keep its current numbers, so labs are unchanged. Separately, the stock yaw gain
leaves almost no margin.

Second line of defence (also needs approval): stop the saturation handling from adding
thrust that altitude hold did not ask for. For example, scale the yaw command down first
when a motor would go negative; real flight controllers give yaw the lowest priority. A
bad frame would then spin or wobble in place instead of rocketing upward.

### 2. LIMIT: heavier frames are underdamped (lead 2)

Damping ratio ζ = (Kd + 0.005) / (2·√(Kp·I)) drops as inertia grows:

| I | ζ | 17.5° step overshoot | settle (5%) |
|---|---|---|---|
| 0.003 (stock) | 0.19 | 17.9% | 0.12 s |
| 0.006167 (cube quad) | 0.13 | 18.3% | 0.30 s |
| 0.012 | 0.09 | 27.2% | 0.64 s |
| 0.03 | 0.06 | 48.0% | 2.06 s |
| 0.1 | 0.03 | 90.4% | does not settle in 3 s |

The cube quad is fine. Larger frames will wobble after every stick input. The fix in
finding 1 removes this too.

**Unity, default drone, 2 rad/s roll kick:** peak 2.29°, then −1.28°, 0.64°, −0.39°,
0.26° … It settled within 0.5° after 0.12 s. The replica gave 1.30° and 0.06 s, so the
peak differs because the probe samples one step later. Both show a lightly damped
response that settles quickly.

### 3. LIMIT: the motor layout and COM have almost no effect on flight (lead 3)

- Thrust goes through `AddRelativeForce` (`:815`), which acts at the centre of mass, so
  **an off-centre COM creates no tipping moment.** Gravity and drag also act at the COM.
  A frame with its battery 5 cm forward hovers level with four equal motor speeds. A
  real one would need the rear motors to work harder.
- The torques are taken from the sign tables (`:426-436`) and applied directly
  (`AddRelativeTorque`, `:818`), not from the real motor positions.
- **`armLength` cancels out.** The mixer adds `tau / (4·armEffective)` per motor
  (`:737`) and the applied torque multiplies by `armEffective` again (`:803-804`). Arm
  length only matters once a motor saturates.
- What a custom frame does change: **mass** (hover thrust, how hard the braking and drag
  forces feel) and **inertia** (response to the fixed gains). The service's COM only
  moves the pivot for collisions.

**Confirmed in Unity by the task B session:** with the battery moved to CAD (0.1, 0, 0),
`rb.centerOfMass` = (0.0294, 0, 0), yet the frame hovered dead level with all four motors
at exactly 675.76 rad/s (0.000% spread). A real quad would need about a 20% speed spread
to hold that offset.

So the goal "flies the way its mass distribution predicts" holds for mass and inertia,
not for COM offset or motor placement. Making it hold needs thrust applied at each
motor's position (`AddForceAtPosition`) plus a mixer built from the real positions. That
is a design change for the user to decide on.

### 4. BUG: per-motor values are mirrored left to right (roll and yaw)

Measured above: +roll needs more lift on the **right**, and +yaw (clockwise from above)
comes from the reaction torque of **counter-clockwise** propellers. The tables say the
opposite:

- `motorRollSign = {-1, 1, 1, -1}` for FR, FL, BL, BR (`:431`): +roll speeds up the
  **left** motors.
- `motorYawSign = {-1, 1, -1, 1}` (`:426`) gives the CW motors (FL, BR) +yaw. The comment
  at `:420` says "CW motors produce positive yaw reaction torque". A CW propeller pushes
  the body counter-clockwise, which is −yaw in Unity.
- `motorPitchSign` (`:436`) is correct.

This is consistent with tables taken from a right-handed reference (the Wil Selby model
in the header) and used in Unity's left-handed frame. **Flight itself is unaffected**
because the commanded torque is applied directly. But anything that reads or changes
individual motors sees mirrored values:

- `MotorSpeeds` / `MotorRPMs` (telemetry, `DroneDebugLogger.cs:233` CSV). When rolling
  right, the right motors show as faster; a real quad speeds up the left ones.
- `PropellerSpin` visual speeds (hidden by the 5000°/s floor and the wagon-wheel effect).
- Crash damage (`CrashSystem.MotorHealth`, `:783`): losing a motor tips the drone toward
  the wrong side.

Fix (needs approval): flip `motorRollSign` and `motorYawSign`. Because the torque is
applied straight from the same tables, flight should stay identical, but confirm that in
Unity before relying on it.

### 5. BUG (in `CrashSystem.cs`): a broken arm kills the wrong motor

`CrashSystem` sorts propellers by name (`CrashSystem.cs:202`) and builds arm groups in
that order. `DetachArmGroup(index)` then sets `MotorHealth[index] = 0` (`:522`).
`Flight` reads `MotorHealth[i]` in `MotorSpeeds` order, which is FR, FL, BL, BR
(`Flight.cs:259`, `:783`). On the stock prefab the name order is FL, BR, FR, BL (from the
`[PropellerSpin]` logs: Propeller_0 = Front Left, _1 = Back Right, _2 = Front Right,
_3 = Back Left), so:

| Arm broken | Motor actually killed |
|---|---|
| Propeller_0 (FL) | FR |
| Propeller_1 (BR) | FL |
| Propeller_2 (FR) | BL |
| Propeller_3 (BL) | BR |

This is the same index trap `PropellerSpin` already handles with `motorIndices`
(`PropellerSpin.cs:84-87`). It affects the default drone and predates the custom-frame
work. It is out of `Flight.cs` but feeds it, so it is listed here. The motor-corner
refactor (task C) is a natural place to fix it.

### 6. LIMIT: thrust limits for heavy frames (lead 4)

Maximum thrust is 14.42 N per motor, 57.67 N in total (`kf · 1256.6²`).

- Above **5.88 kg** a frame cannot hover. No warning is logged. The kinematic launch
  (finding 7) still lifts it to 7.5 m, then it sinks once altitude hold takes over.
- Above **2.10 kg**, a full climb command (1 + 1.8 × hover) saturates. The handling at
  `:757-768` lowers all four motors equally, so attitude keeps priority over climb rate.
  That is the intended behaviour.
- When a large attitude command drives a motor below zero, `:757-762` raises all four
  motors. In the replica a 17.5° step briefly gives 22% (cube quad) to 38% (stock) more
  thrust than hover. For a stable frame that is only a short climb bump on hard stick
  inputs. When an axis is unstable (finding 1), the same rule fires every step and drives
  the runaway climb.

Suggestion: log a warning in `Start()` when `mass · g > 4 · F_max`. That should be safe
for labs because the default drone (2 kg) is far below the limit.

### 7. OK with caveats: kinematic launch (lead 5)

- `IsLaunching` (`:580-623`) moves the body with `rBody.Move` and returns before any
  thrust is computed. Climb behaviour can only be judged after
  `[Flight] Launch complete - entering altitude hold`.
- **Confirmed dead code:** `:691-696`. When `IsLaunching` is true at `:580`, either user
  input clears it (`:585`) or the branch returns (`:621`), so the check at `:692` never
  sees `IsLaunching == true`. It is harmless and could be deleted.
- At hand-off the drone can still be rising at up to 0.5 m/s, and `TargetAltitude` is
  reset to the current height (`:617`). The replica gives a 0.046 m overshoot that then
  settles. This is not a steady climb.
- Because launch ends anywhere within 0.5 m of the target and then holds the current
  height, the drone settles short of the logged target. The task B session saw about
  0.54 m short every time (target 7.9 m, hover about 7.36 m). This is expected, not
  drift.
- UNKNOWN: `rBody.Move` on a non-kinematic body, followed by setting `linearVelocity`
  (`:600-603`), may move the drone twice per step (teleport plus velocity). That would
  only change launch speed. Not checked.

### 8. OK: altitude hold has no steady error when thrust is delivered (lead 6)

`thrust = m·g·(1 + 0.5·e − 0.8·v)·tilt` (`:681-718`). The hover feedforward uses
`rBody.mass`, so the gravity term cancels exactly for any custom mass. In normalised
terms this is z'' = g(0.5·e − 0.8·v): ζ ≈ 1.8, overdamped, with the same behaviour for
every mass. **There is no integral term**, so a steady error appears only when thrust is
lost:

- crash damage (`MotorHealth < 1`) makes the drone settle below its target;
- frames over 5.88 kg (finding 6) cannot hold altitude.

Altitude hold itself does not cause a steady climb. The climb comes from the motor
saturation handling when an attitude axis is unstable (finding 1).

### 9. OK by design: position hold hides slow drift (lead 7)

With no stick input, horizontal velocity is set to zero below 0.08 m/s (`:843-847`), on
top of 4.5 N/(m/s) linear braking (`:849-853`). The hover test must judge drift from
attitude and motor speeds, not position alone. The braking force is not scaled by mass:
a 0.5 kg frame stops 4× harder than the 2 kg stock drone, and a 5 kg frame 2.5× softer.

### 10. OK: stick signs (lead 8), checked against Unity's rotation signs

- `ForwardSpeed > 0` gives desiredPitch > 0 (`:661`), which is nose down. Thrust tilts
  to +Z, so the drone moves forward. ✓
- `StrafeSpeed > 0` gives desiredRoll < 0 (`:664`), which drops the right side. Thrust
  tilts to +X, so the drone moves right. ✓
- `YawSpeed > 0` gives +Y rate (`:667`), a clockwise turn from above, which is a right
  turn. Banking (`:671`) rolls right. ✓
- Position hold opposes the body-frame velocity with the same signs (`:642-643`). ✓

These are calculated from the code plus the measured Unity conventions. They were not
flown as step inputs in Unity.

### 11. OK: inertia axes for custom frames (lead 1)

Cube quad in Play mode: `inertiaTensor = (0.006167, 0.006167, 0.010667)`,
`inertiaTensorRotation = (0.70711, 0, 0, 0.70711)` (90° about X). R·diag(I)·Rᵀ evaluated
in the Editor gives the body diagonal **X 0.006167, Y 0.010667, Z 0.006167**, so the
largest moment is on the yaw axis (Unity Y). `mass_composer.py` (`:82`) forces the
eigenvector matrix to determinant +1, so the quaternion is a proper rotation. The
default drone keeps COM (0,0,0) and identity rotation, exactly as before `edc07f5`
(git diff checked). Note: the `Ixx` / `Iyy` / `Izz` fields (`:88-96`) only feed
`rBody.inertiaTensor`. No gain reads them.

### 12. Smaller items (lead 9)

- Euler reads (`:546-548`): fine near level. Unity's ZXY order couples roll and pitch at
  large tilts. LIMIT.
- The 3 rad/s clamp (`:559-568`) applies to the total rate vector. Full yaw stick asks
  for 200°/s = 3.49 rad/s (`:146`), which can never be reached, so the top yaw rate is
  about 172°/s. While yawing at the clamp, the roll/pitch rate shares the same budget.
  Minor BUG.
- Propeller gyro term (`:821`): it uses the mirrored yaw signs (finding 4), so its sign
  is also mirrored. Its size, about JTP·Ω·rate ≈ 3e-3 N·m, is negligible next to control
  torques of about 1 N·m.
- Momentum rotation during yaw (`:905-916`) rotates the velocity by the *commanded* yaw
  rate. Because of the clamp, the actual rate is lower, so velocity turns up to 16%
  faster than the body. An arcade-style simplification.

## Hover test (task B), run by the "X quadrotor test" session

20 s after `[Flight] Launch complete`, no input, in Unity:

- **Cube quad** (1.7 kg, COM 0) and **default drone** (2.0 kg): altitude error under
  2.3 mm after 2 s and 0.0000 from 10 to 20 s. Pitch, roll and yaw changed 0.000°.
  Horizontal movement 0.0000 m. All four motor speeds were identical, 675.76 and
  732.97 rad/s, both exactly √(m·g/4/kf).
- So a symmetric X-quad whose inertia is above the finding 1 limits hovers perfectly.
- `TestSetup` is `m_IsActive: 0` in the committed `Main.unity` (checked in the scene file
  at HEAD), so the default route no longer falls back. CLAUDE.md's "still ACTIVE" is out
  of date.

## Climb symptom: what the audit says

- **Cause found and reproduced in Unity:** any frame whose yaw inertia is below about
  0.00555 kg·m², or whose roll/pitch inertia is below about 0.00165 kg·m², goes unstable
  within about 0.4 s of launch. The saturation rule (`:757-762`) then turns that into a
  runaway climb, even with the frame perfectly level (finding 1). The stock drone and
  the 1.7 kg cube quad are above both limits and hover perfectly.
- **Correction to CLAUDE.md:** a NaN `armLength` would **not** cause a climb. NaN makes
  every `motorThrusts[i]` NaN. `Mathf.Max(NaN, 0f)` returns 0, because `NaN > 0` is
  false. So all four motors go to 0 rad/s and the drone **falls**.
- Not checked: the inertia of the STL the student originally flew
  (`Part Studio 1 - Part 1.stl`). Checking it against the two limits would confirm it was
  this mechanism.

## Proposed next steps (each needs user approval)

1. Scale the attitude gains by inertia (finding 1, which also fixes finding 2). The
   stock drone is unchanged by construction. This is the fix for the climb.
2. Stop saturation handling from adding unrequested thrust, e.g. yaw gets lowest
   priority (finding 1, second line of defence).
3. Flip `motorRollSign` / `motorYawSign` (finding 4) and fix the `CrashSystem` index
   map (finding 5). Then check in Unity that flight is identical and that telemetry and
   crash tipping are now correct.
4. Log a warning when a frame is too heavy to hover (finding 6).
5. Decide whether custom frames should feel COM offset and motor placement (finding 3).
   This is a design change, not a bug fix.
6. After any fix, re-run the two light frames from finding 1 plus the default drone and
   cube quad in Unity.

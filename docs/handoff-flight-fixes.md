# Handoff: fixing the custom-drone "keeps climbing" bug in Flight.cs

Written 2026-10-14 so a fresh session can pick this up cold. CLAUDE.md loads
automatically; read it first. This file has the cause, the drafted code changes, how to
verify them, and the traps found along the way.

**Update 2026-10-14: Fix 1 (section 3) is APPLIED and verified in Unity** (user-approved;
results in `docs/flight-audit-2026-10-13.md`, finding 1, "Fix applied"). The user chose
Fix 1 only. Fix 2 (section 4) and section 7 are still drafts that need separate approval.

**Original status: the changes below are DRAFTS. They are not applied to `Flight.cs`.** The user has
had the fix explained and asked for this draft. **Get the user's explicit approval before
editing `Flight.cs`.** It flies every drone in every lab, and CLAUDE.md requires the
default drone and all labs to behave exactly as before.

---

## 1. The problem, in one paragraph

`Flight.cs` keeps the drone level with fixed-strength corrective torques
(`Kp_rp = 10`, `Kd_rp = 0.06`, `Kp_yaw = 0.55`, `Flight.cs:123-132`), tuned for the stock
drone's inertia (0.003 / 0.006 / 0.003 kg·m²). Physics runs at a fixed 0.02 s step. A frame
that is easier to turn (lower inertia) over-corrects, and each correction overshoots more
than the last. This happens when **yaw inertia is below about 0.00555 kg·m²** or
**roll/pitch inertia is below about 0.00165 kg·m²**. The motors then saturate every step,
two at 0 and two at maximum. The mixer's saturation rule (`Flight.cs:757-762`) raises all
four motors to lift the lowest back to 0, which adds several times hover thrust, so the
drone climbs out of control. Altitude hold cannot cancel it.

Plain-language version (already given to the user): the autopilot's nudges are sized for
the stock drone. A light drone overshoots each nudge, and the safety rule that keeps the
motors above zero adds lift nobody asked for.

## 2. Evidence (all gathered 2026-10-13/14, no code changed)

Full audit: **`docs/flight-audit-2026-10-13.md`** (12 findings with line numbers; finding 1
is this bug). Offline models: `tools/flight-audit/flight_loop_replica.py` (current
behaviour) and `tools/flight-audit/fix_check.py` (current vs drafted fixes). Run either
with `python tools/flight-audit/<file>.py`.

Unity traces (Play mode via Unity MCP, temporary runtime probe, no input after
`[Flight] Launch complete`). All frames used `tools/test-data/cube100.stl`.

| Frame (CAD Z-up nodes, metres) | Mass | Body inertia X / Y(yaw) / Z | Result today |
|---|---|---|---|
| Default drone (no import) | 2.0 kg | 0.003 / 0.006 / 0.003 | Hovers. Yaw kick 0.5 rad/s rings 0.50, −0.42, 0.36, −0.31, 0.26 (×−0.85 per step, exactly as predicted; only 8% from unstable). Roll kick 2 rad/s: 2.29, 0.54, −1.28, −0.63, 0.64, 0.54, −0.27, −0.39 … °, settled in 0.12 s. Motors 733.0 rad/s each. |
| Cube quad: cube at 1000 kg/m³, 4 × Motor 0.05 kg at (±0.15, ±0.15, 0), Battery 0.5 kg at (0,0,0) | 1.7 kg | 0.006167 / 0.010667 / 0.006167 | Hovers perfectly (task B): altitude error 0.0000 m from 10 to 20 s, motors all 675.76 rad/s. |
| Yaw-light: cube at 1000 kg/m³, 4 × Motor 0.05 kg at (±0.05, ±0.05, 0) | 1.2 kg | 0.002167 / **0.002667** / 0.002167 | **Level runaway climb.** Yaw rate ×−3.2 per step (predicted ×−3.16), pinned at ±3.00 alternating, motors (1257,0,1257,0) ↔ (0,1257,0,1257), climbing at 32.8 m/s after 3 s. |
| Small: cube at 100 kg/m³, 4 × Motor 0.02 kg at (±0.08, ±0.08, 0), Battery 0.1 kg at (0,0,0) | 0.28 kg | **0.000679 / 0.001191** / 0.000679 | Runaway climb at 95 m/s after 2.2 s, hard crash about 111 m up, repeats on every respawn. |

## 3. Fix 1 (the cure): scale the attitude gains to each frame's inertia

Multiply each axis's commanded torque by (this frame's inertia about that axis) ÷ (stock
inertia). Every frame then responds like the stock drone. For the stock drone the
multipliers are exactly 1.0f, so its maths is bit-for-bit unchanged.
`Matrix4x4.Rotate(Quaternion.identity)` gives exact 1/0 entries, 0.003f/0.003f = 1f, and
x·1f is exact.

### 3.1 New fields (put them after the attitude gains, near `Flight.cs:132`)

```csharp
    // --------- Inertia-scaled gains ---------

    // The attitude gains above were tuned for the stock drone's inertia. A custom frame's
    // torques are scaled by (its body-axis inertia / these values) so it responds like the
    // stock drone. Unscaled, frames with yaw inertia below ~0.0056 kg*m^2 (roll/pitch below
    // ~0.0017) are unstable at the 0.02 s physics step and climb away (docs/flight-audit-2026-10-13.md).
    private const float GainRefIxx = 0.003f;
    private const float GainRefIyy = 0.006f;
    private const float GainRefIzz = 0.003f;

    // Exactly 1 for the stock drone.
    private float pitchGainScale = 1f;
    private float yawGainScale = 1f;
    private float rollGainScale = 1f;
```

### 3.2 In `Start()`, right after `this.rBody.inertiaTensorRotation = ...` (`Flight.cs:492`)

```csharp
        Vector3 bodyInertia = BodyAxisInertia(new Vector3(Ixx, Iyy, Izz), this.customInertiaRotation);
        this.pitchGainScale = GainScale(bodyInertia.x, GainRefIxx);
        this.yawGainScale = GainScale(bodyInertia.y, GainRefIyy);
        this.rollGainScale = GainScale(bodyInertia.z, GainRefIzz);
        Debug.Log($"[Flight] Attitude gain scale (pitch, yaw, roll) = ({this.pitchGainScale:F3}, {this.yawGainScale:F3}, {this.rollGainScale:F3})");
```

Use the fields (`Ixx/Iyy/Izz`, `customInertiaRotation`), not values read back from the
`Rigidbody`, so the stock case stays exact.

### 3.3 Two helpers (next to `NormalizeAngle`, end of class)

```csharp
    /// <summary>
    /// Moment of inertia about each body axis: the diagonal of R * diag(principal) * R^T.
    /// </summary>
    private static Vector3 BodyAxisInertia(Vector3 principal, Quaternion rotation)
    {
        Matrix4x4 R = Matrix4x4.Rotate(rotation);
        Vector3 result = Vector3.zero;
        for (int a = 0; a < 3; a++)
            for (int k = 0; k < 3; k++)
                result[a] += R[a, k] * R[a, k] * principal[k];
        return result;
    }

    /// <summary>inertia / reference, or 1 if that is not a usable positive number.</summary>
    private static float GainScale(float inertia, float reference)
    {
        float scale = inertia / reference;
        return float.IsFinite(scale) && scale > 0f ? scale : 1f;
    }
```

### 3.4 In `FixedUpdate()`, replace the three torque lines (`Flight.cs:728-730`)

```csharp
        float tauPitch = (Kp_rp * pitchError - Kd_rp * p) * this.pitchGainScale;
        float tauRoll = (Kp_rp * rollError - Kd_rp * r) * this.rollGainScale;
        float tauYaw = Kp_yaw * yawRateError * this.yawGainScale;
```

Also update the `Kp_yaw` comment at `:129-131` ("Time constant = Iyy/Kp_yaw = 0.011s"),
e.g. "for the stock inertia; scaled per frame, see yawGainScale".

### 3.5 Expected gain-scale log per test frame

| Frame | pitch, yaw, roll |
|---|---|
| Default drone | 1.000, 1.000, 1.000 |
| Cube quad | 2.056, 1.778, 2.056 |
| Yaw-light 1.2 kg | 0.722, 0.444, 0.722 |
| Small 0.28 kg | 0.226, 0.199, 0.226 |

## 4. Fix 2 (safety net, optional): yaw gets the lowest priority in the mixer

When the motors cannot meet the yaw demand, scale the yaw part back instead of letting
the saturation rule add lift. Real flight controllers do this. **This one changes stock
behaviour on full yaw stick**, because the stock drone's full-yaw demand already
saturates. Ask the user separately whether to apply it, and check the labs.

Replace the mixing loop and the `Fmax` line (`Flight.cs:741-755`) with:

```csharp
        float Fmax = kf * motorSpeedMax * motorSpeedMax;

        // Roll/pitch first. Yaw is added afterwards, scaled back if it would push any motor
        // outside [0, Fmax]: yaw has the lowest priority (as on real flight controllers), so
        // a yaw demand the motors cannot meet never adds unrequested thrust.
        float yawScale = 1f;
        for (int i = 0; i < 4; i++)
        {
            motorThrusts[i] = thrustPerMotor
                             + motorRollSign[i] * tauRoll * rollMix
                             + motorPitchSign[i] * tauPitch * pitchMix;

            float yawPart = motorYawSign[i] * tauYaw * yawMix;
            float total = motorThrusts[i] + yawPart;
            if (yawPart < 0f && total < 0f)
                yawScale = Mathf.Min(yawScale, Mathf.Max(0f, motorThrusts[i]) / -yawPart);
            else if (yawPart > 0f && total > Fmax)
                yawScale = Mathf.Min(yawScale, Mathf.Max(0f, Fmax - motorThrusts[i]) / yawPart);
        }

        float minFi = float.MaxValue;
        float maxFi = float.MinValue;

        for (int i = 0; i < 4; i++)
        {
            motorThrusts[i] += motorYawSign[i] * tauYaw * yawMix * yawScale;

            if (motorThrusts[i] < minFi) minFi = motorThrusts[i];
            if (motorThrusts[i] > maxFi) maxFi = motorThrusts[i];
        }
```

Keep the existing `if (minFi < 0f)` boost and `if (maxFi > Fmax)` reduction after it
unchanged. They now only act on roll/pitch demand, which keeps attitude priority over
altitude by design. When `yawScale == 1` the arithmetic matches today's: the sum is still
((T + roll) + pitch) + yaw.

## 5. Offline results for the drafts (`tools/flight-audit/fix_check.py`)

"Kick" = 0.5 rad/s roll kick while hovering. "Yaw stick" = full right yaw for 2 s
(MaxSpeed 0.5). max|dz| = largest altitude change.

| Frame | Current: kick / yaw stick | Fix 1: kick / yaw stick | Fix 1+2: kick / yaw stick |
|---|---|---|---|
| Stock | 0 m / 0.261 m | 0 m / 0.261 m (**identical**) | 0 m / 0.003 m |
| Cube quad | 0 m / 0.381 m | 0 m / 1.088 m | 0 m / 0.027 m |
| Yaw-light 1.2 kg | **345 m runaway** / 212 m | 0 m / 0.539 m | 0 m / 0.235 m |
| Small 0.28 kg | **1205 m runaway** / 1136 m | 0 m / 1.031 m | 0 m / 0.099 m |
| Big 3 kg | 0 m / 0.088 m | 0 m / 1.809 m | 0 m / 0.012 m |

Reading: Fix 1 alone ends both runaways and leaves the stock drone identical. Its side
effect is a bigger altitude bump (up to about 1.8 m) on full yaw stick for frames
*heavier to turn* than stock, because their scaled yaw demand saturates more. Fix 2
removes those bumps, at the price of changing the stock drone's full-yaw behaviour (its
0.26 m bump goes to 0.003 m). The model is single-rigid-body and leaves out drag and
banking-induced drift, so confirm everything in Unity.

## 6. Verification plan (Unity, after the user approves an edit)

1. Compile clean (Unity 6000.4.5f1). No new console errors.
2. **Default drone, bit-identical check:** log shows `(1.000, 1.000, 1.000)`. Re-run the kick
   probe: yaw trace 0.50, −0.42, 0.36, −0.31, 0.26 …; roll trace 2.29, 0.54, −1.28, −0.63,
   0.64 …; hover motors 733.0. They must match section 2 exactly.
3. **Cube quad:** hovers as in task B (motors equal at 675.76, altitude error 0.0000 after
   10 s). Kicks settle.
4. **Yaw-light 1.2 kg and small 0.28 kg:** no climb. Over 20 s after launch: altitude
   within ±0.1 m of target, |roll|, |pitch| < 1°, motors equal within 1%, no CrashSystem
   messages.
5. Fly each frame with stick inputs (W/A/S/D, yaw arrows) to check it still responds.
6. If Fix 2 is applied: full yaw stick on the stock drone. Expect only a tiny altitude
   change, with yaw still reaching the 3 rad/s clamp. Ask the user to check one lab module.
7. Update `docs/flight-audit-2026-10-13.md` (mark finding 1 fixed, with results), DEVLOG.md
   (new entry at the top of the newest date), CLAUDE.md (Status → Next item 3, Open
   questions bullet about `Flight.cs`, Definition of done item 4). Then run
   `graphify update .` from `UAVSim/` in the background (it takes minutes).

### How to load a test frame without clicking (integration contract: set statics, then load the scene)

Enter Play mode (`EditorApplication.isPlaying = true;`), wait until the Unity window title
starts with `UAVSim`, then in a RunCommand:

```csharp
MassServiceLauncher.EnsureRunning();   // bundled service, ready after ~1.3-2.3 s; run this in its own command first
CustomFrameLoader.SelectedSTLPath = @"C:\Users\Owner\Documents\Projects\UAVNeo-Simulator-RESEARCH\UAVNeo-Simulator-RESEARCH\tools\test-data\cube100.stl";
CustomFrameLoader.SelectedDensity = 1000f;
var n = CustomFrameLoader.SelectedNodes; n.Clear();
n.Add(new CustomFrameLoader.NodeData { type = CustomFrameLoader.NodeType.Motor, position = new Vector3(0.05f, 0.05f, 0f), massKg = 0.05f });
// ... other three motors (+-x, +-y), plus Battery if the frame has one
Object.FindAnyObjectByType<MainMenu>().BeginSimulation();          // from Main
// or, already in DroneWorld: SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
```

For the default drone set `SelectedSTLPath = ""` and clear `SelectedNodes`. Afterwards,
clear the statics and leave Play mode.

### Probe that worked (add as a component at runtime; it arms the drone itself)

Define it inside the RunCommand script, then
`Object.FindAnyObjectByType<Flight>().gameObject.AddComponent<TraceProbe>()`. Adding a
MonoBehaviour from the RunCommand assembly works. It records every physics step after
launch completes and logs one `[Trace]` line.

```csharp
internal class TraceProbe : MonoBehaviour
{
    Flight f; Rigidbody rb; CrashSystem cs; int state, steps; float y0;
    System.Text.StringBuilder sb = new System.Text.StringBuilder();
    static float N(float a) { return a > 180f ? a - 360f : a; }
    void Start() { f = GetComponent<Flight>(); rb = GetComponent<Rigidbody>(); cs = GetComponent<CrashSystem>(); }
    void FixedUpdate()
    {
        steps++;
        if (state == 0) { if (!f.IsArmed) f.Arm(); state = 1; steps = 0; return; }
        if (state == 1) { if (!f.IsLaunching) { state = 2; steps = 0; y0 = transform.position.y; } return; }
        if (state != 2) return;
        bool dead = cs != null && cs.IsDestroyed;
        if (steps <= 30 || steps % 10 == 0 || dead)
        {
            var e = transform.eulerAngles; var w = transform.InverseTransformDirection(rb.angularVelocity); var m = f.MotorSpeeds;
            sb.AppendLine("s" + steps + " dy=" + (transform.position.y - y0).ToString("F2") + " vy=" + rb.linearVelocity.y.ToString("F2")
                + " r/p=" + N(e.z).ToString("F1") + "/" + N(e.x).ToString("F1") + " w=" + w.ToString("F2")
                + " mot=" + m[0].ToString("F0") + "," + m[1].ToString("F0") + "," + m[2].ToString("F0") + "," + m[3].ToString("F0") + (dead ? " DESTROYED" : ""));
        }
        if (dead || steps >= 1000) { Debug.Log("[Trace]\n" + sb); enabled = false; }
    }
}
```

For kick tests, set `rb.angularVelocity = transform.TransformDirection(new Vector3(0, 0, 2f))`
(roll) or `(0, 0.5f, 0)` (yaw) about 8 s after launch completes, and record `N(eulerAngles.z)`
or the body yaw rate every step.

### Unity MCP tips

- Tools: `mcp__unity-mcp__Unity_RunCommand` (class must be `internal class CommandScript : IRunCommand`)
  and `mcp__unity-mcp__Unity_GetConsoleLogs`. Load them with ToolSearch.
- Wait for Play mode or reloads by polling the window title from PowerShell:
  `(Get-Process -Id <UnityPid>).MainWindowTitle` until it starts with `UAVSim`. The PID was
  30044 on 2026-10-14; look it up again.
- `result.Log` ignores format specifiers, so build strings with `.ToString("F3")`. File I/O
  (`System.IO.File.*`) is refused inside RunCommand.
- Long `[Trace]` logs are easiest found with `GetConsoleLogs` (`logTypes: "Info"`,
  `maxEntries` ~15). A crashing frame floods the console with CrashSystem lines.
- A frame that crashes respawns and relaunches forever. Leave Play mode to stop it.

### Sharing the Editor with other sessions (important)

Other Claude sessions use the **same** Unity Editor: "Motor corner instance problem"
(task C) and "X quadrotor test" (task B). Commands collided twice on 2026-10-13. Before
any Unity call, use `ListAgents` and message the busy sessions asking for a turn. Wait
for "go", and message them when you hand the Editor back (out of Play mode, statics
cleared, no `python.exe` from `MassService~` left running). Never recompile or leave Play
mode while another session is mid-run.

## 7. Other audit findings with drafted fixes (lower priority, each needs approval)

- **Mirrored per-motor sign tables** (audit finding 4). `motorRollSign` (`:431`) and
  `motorYawSign` (`:426`) are left-right mirrored for Unity's left-handed frame. Flight is
  unaffected (torque is applied straight from the tables), but `MotorSpeeds`/`MotorRPMs`,
  crash damage and the prop gyro sign are mirrored. Draft: `motorRollSign = { 1f, -1f, -1f, 1f }`,
  `motorYawSign = { 1f, -1f, 1f, -1f }`, and fix the comment at `:419-425`. Verify that
  flight traces are identical and that rolling right now speeds up the LEFT motors.
- **CrashSystem kills the wrong motor** (finding 5). `DetachArmGroup(index)` sets
  `MotorHealth[index]` with `index` in Propeller_N name order (stock: FL, BR, FR, BL), but
  `Flight` reads FR, FL, BL, BR. Draft: when the propellers are found, store a
  name-index → motor-index map using `MotorQuadrants.ClassifyUnity` on each propeller's
  drone-root-local position, mapped explicitly FR→0, FL→1, BL→2, BR→3 (never cast the
  enum). Use it at `CrashSystem.cs:522`. `ClassifyUnity` was added by the task C session.
  Coordinate with it.
- **Too-heavy frames** (finding 6). Above 5.88 kg a frame cannot hover. Draft for `Start()`:
  `if (rBody.mass * Physics.gravity.magnitude > 4f * kf * motorSpeedMax * motorSpeedMax) Debug.LogWarning("[Flight] ... cannot hover ...");`
- **Dead code** `Flight.cs:691-696` (unreachable launch-complete check). Safe to delete.
- **COM offset and motor positions have no effect on flight** (finding 3, confirmed in
  Unity). This is a design decision for the user, not a bug fix.

## 8. Repository state at handoff (2026-10-14)

- Branch `feature/bundled-mass-service`, not pushed. Nothing from the audit is committed.
  Commit only when the user asks; end messages with the attribution line in the system
  reminder.
- Created by the audit session (untracked): `docs/flight-audit-2026-10-13.md`,
  `docs/handoff-flight-fixes.md` (this file), `tools/flight-audit/flight_loop_replica.py`,
  `tools/flight-audit/fix_check.py`.
- Edited (uncommitted): `CLAUDE.md`, `DEVLOG.md`. Other sessions also edited these.
- Uncommitted code from the **task C session** (motor-corner refactor, not this work):
  `MotorQuadrants.cs`, `PropellerSpin.cs`, `CustomFrameLoader.cs`. Don't revert or
  overwrite them. That session reported the refactor **finished** on 2026-10-14,
  including `CustomFrameLoader.RepositionPropellers`, and said it no longer needs the
  Editor.
- `Flight.cs` is unchanged from HEAD.
- Earlier handoff for tasks A/B/C: `docs/handoff-flight-audit-and-motor-corners.md`.

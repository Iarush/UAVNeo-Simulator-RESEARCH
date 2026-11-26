using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace UAVSim.Tests.PlayMode
{
    /// <summary>
    /// .plans/posctl-yaw-reversal-spec.md (phase 6 of the real-flight-physics work): the keyboard
    /// yaw reversal the owner reported, the 15 deg tilt limit, rotor drag and the PX4 MPC_POS_MODE 4
    /// (StickAccelerationXY) port. Uses the flight harness in FlightMotorTests.cs.
    ///
    /// Traces (CSV) go to the folder in the UAVSIM_TRACE_DIR environment variable, or to
    /// Application.temporaryCachePath when it is not set.
    /// </summary>
    public partial class FlightMotorTests
    {
        private static string TraceDir
        {
            get
            {
                string dir = Environment.GetEnvironmentVariable("UAVSIM_TRACE_DIR");
                if (string.IsNullOrEmpty(dir)) dir = Application.temporaryCachePath;
                Directory.CreateDirectory(dir);
                return dir;
            }
        }

        /// <summary>The horizontal velocity setpoint (world x, z) the PX4 velocity loop is tracking.</summary>
        private Vector2 VelocitySetpointXY()
        {
            if (flight.GetType().GetProperty("HorizontalVelocitySetpoint") != null)
                return (Vector2)Get(flight, "HorizontalVelocitySetpoint");
            return (Vector2)Get(flight, "brakeVelocitySetpoint"); // before phase 6
        }

        /// <summary>The horizontal acceleration setpoint (feed-forward, world x, z).</summary>
        private Vector2 AccelerationSetpointXY()
        {
            if (flight.GetType().GetProperty("HorizontalAccelerationSetpoint") != null)
                return (Vector2)Get(flight, "HorizontalAccelerationSetpoint");
            return (Vector2)Get(flight, "setpointAcceleration"); // before phase 6
        }

        /// <summary>Forces MPC_POS_MODE for this flight (3 = smoothed velocity, 4 = acceleration based).
        /// No-op on code from before phase 6, which only had the mode-3 path.</summary>
        private void ForcePosMode(int mode)
        {
            if (flight.GetType().GetField("posMode", AnyMember) != null)
                Set(flight, "posMode", mode);
        }

        private float TiltLimitDeg() => (float)Get(flight, "tiltMax") * Mathf.Rad2Deg;

        private float RotorDragSim() =>
            flight.GetType().GetField("rotorDragSim", AnyMember) != null ? (float)Get(flight, "rotorDragSim") : 0f;

        private static float Tilt(Transform t) => Vector3.Angle(t.up, Vector3.up);

        private struct ReversalResult
        {
            public float cruise, tHalf, overshoot, maxTilt, maxTiltAll, altChange, maxGap, lag, floorHeading, floorDirect, yawTime;
        }

        /// <summary>
        /// Spec section 5, phase 0: the reported manoeuvre on the default drone in PosCtl. Forward until
        /// the speed is steady, then yaw 180 deg holding forward, then hold forward until the speed along
        /// the new heading is steady for 1 s (or 30 s). Writes a CSV trace and reports the metrics.
        /// </summary>
        [UnityTest, Category("YawReversal")]
        public IEnumerator YawReversal_Trace([Values(0.5f, 1.0f)] float maxSpeed)
        {
            ReversalResult r = default;
            yield return FlyReversal(maxSpeed, -1, x => r = x);

            if (Mathf.Approximately(maxSpeed, 0.5f) && TiltLimitDeg() < 20f)
            {
                // Acceptance checks 1, 3 (reversal part) and 5; only meaningful once phase 6 is in.
                // Check 2 (cruise speed) is CruiseSpeed_AndReversalFromSettledCruise: here the
                // "2 % for 1 s" rule may start the yaw before mode 4 has reached its steady speed.
                Assert.LessOrEqual(r.tHalf, 4.0f, "t_half");
                Assert.LessOrEqual(r.maxTiltAll, 15.5f, "tilt");
                Assert.LessOrEqual(r.altChange, 0.5f, "altitude change during the reversal");
            }
            Assert.IsEmpty(errors, string.Join("\n", errors));
        }

        /// <summary>
        /// Acceptance check 2 (cruise speed 5.0 m/s +-5 % at MaxSpeed 0.5) and a stricter reading of
        /// check 1: mode 4 approaches its steady speed exponentially (virtual drag 1.2 /s), so the
        /// phase-0 "2 % for 1 s" rule can start the yaw before the speed has settled. Here the drone
        /// cruises for at least 15 s first; the cruise is the mean of the last 1 s.
        /// </summary>
        [UnityTest, Category("YawReversal")]
        public IEnumerator CruiseSpeed_AndReversalFromSettledCruise()
        {
            ReversalResult r = default;
            yield return FlyReversal(0.5f, -1, x => r = x, 15f);
            Assert.AreEqual(5.0f, r.cruise, 0.05f * 5.0f, "cruise speed");
            Assert.LessOrEqual(r.tHalf, 4.0f, "t_half from a settled cruise");
            Assert.LessOrEqual(r.maxTiltAll, 15.5f, "tilt");
            Assert.LessOrEqual(r.altChange, 0.5f, "altitude change during the reversal");
            Assert.IsEmpty(errors, string.Join("\n", errors));
        }

        private IEnumerator FlyReversal(float maxSpeed, int posMode, Action<ReversalResult> done, float minCruiseSeconds = 0f)
        {
            yield return Fly(DefaultDrone());
            ClearSky();
            SetMode("PosCtl");
            if (posMode > 0) ForcePosMode(posMode);
            Set(flight, "MaxSpeed", maxSpeed);
            yield return FastSteps(100); // hover steady

            float dt = Time.fixedDeltaTime;
            Vector3 h0 = Flat(drone.transform.forward);
            var csv = new StringBuilder("phase,t,x,y,z,vx,vy,vz,vsp_x,vsp_z,asp_x,asp_z,tilt_deg,heading_deg,T_FR,T_FL,T_BL,T_BR\n");
            float t = 0f;
            void Row(string phase)
            {
                Vector3 p = drone.transform.position, v = rb.linearVelocity;
                Vector2 vsp = VelocitySetpointXY(), asp = AccelerationSetpointXY();
                float[] th = (float[])Get(flight, "MotorThrusts");
                csv.Append(string.Format(CultureInfo.InvariantCulture,
                    "{0},{1:F3},{2:F4},{3:F4},{4:F4},{5:F4},{6:F4},{7:F4},{8:F4},{9:F4},{10:F4},{11:F4},{12:F3},{13:F2},{14:F4},{15:F4},{16:F4},{17:F4}\n",
                    phase, t, p.x, p.y, p.z, v.x, v.y, v.z, vsp.x, vsp.y, asp.x, asp.y, Tilt(drone.transform), drone.transform.eulerAngles.y,
                    th[0], th[1], th[2], th[3]));
            }

            // 1. Cruise: forward until the speed along the heading is within 2 % for 1 s (max 30 s).
            var window = new Queue<float>();
            float maxTiltAll = 0f;
            bool steady = false;
            for (int f = 0; f < Mathf.Max(1500, Mathf.CeilToInt(minCruiseSeconds / dt) + 50) && !steady; f++)
            {
                Set(flight, "ForwardSpeed", 1f);
                yield return Steps(1);
                t += dt;
                Row("cruise");
                maxTiltAll = Mathf.Max(maxTiltAll, Tilt(drone.transform));
                window.Enqueue(Vector3.Dot(rb.linearVelocity, h0));
                if (window.Count > 50) window.Dequeue();
                if (window.Count == 50)
                {
                    float lo = float.MaxValue, hi = float.MinValue, sum = 0f;
                    foreach (float s in window) { lo = Mathf.Min(lo, s); hi = Mathf.Max(hi, s); sum += s; }
                    steady = sum / 50f > 0.5f && hi - lo <= 0.02f * (sum / 50f) && t >= minCruiseSeconds;
                }
            }
            float cruise = 0f;
            foreach (float s in window) cruise += s;
            cruise /= Mathf.Max(1, window.Count);

            // 2. Yaw 180 deg holding forward, then 3. hold forward until steady along the new heading.
            Vector3 p0 = drone.transform.position, hNew = -h0;
            float heading = drone.transform.eulerAngles.y, turned = 0f, yawTime = float.NaN;
            float tHalf = float.NaN, overshoot = 0f, maxTilt = 0f, altChange = 0f, maxGap = 0f;
            float spZero = float.NaN, vZero = float.NaN;
            window.Clear();
            float t0 = t;
            bool yawing = true, done2 = false;
            for (int f = 0; f < 1500 && !done2; f++)
            {
                Set(flight, "ForwardSpeed", 1f);
                Set(flight, "YawSpeed", yawing ? 1f : 0f);
                yield return Steps(1);
                t += dt;
                Row(yawing ? "yaw" : "after");
                float h = drone.transform.eulerAngles.y;
                turned += Mathf.DeltaAngle(heading, h);
                heading = h;
                if (yawing && turned >= 180f) { yawing = false; yawTime = t - t0; }

                Vector3 v = rb.linearVelocity;
                Vector2 vsp = VelocitySetpointXY();
                float along = Vector3.Dot(v, hNew);
                float tilt = Tilt(drone.transform);
                maxTilt = Mathf.Max(maxTilt, tilt);
                maxTiltAll = Mathf.Max(maxTiltAll, tilt);
                overshoot = Mathf.Max(overshoot, Vector3.Dot(drone.transform.position - p0, h0));
                altChange = Mathf.Max(altChange, Mathf.Abs(drone.transform.position.y - p0.y));
                maxGap = Mathf.Max(maxGap, (vsp - new Vector2(v.x, v.z)).magnitude);
                if (float.IsNaN(tHalf) && along >= 0.5f * cruise) tHalf = t - t0;
                if (float.IsNaN(spZero) && Vector2.Dot(vsp, new Vector2(h0.x, h0.z)) <= 0f) spZero = t - t0;
                if (float.IsNaN(vZero) && Vector3.Dot(v, h0) <= 0f) vZero = t - t0;

                if (!yawing)
                {
                    window.Enqueue(along);
                    if (window.Count > 50) window.Dequeue();
                    if (window.Count == 50 && along > 0f)
                    {
                        float lo = float.MaxValue, hi = float.MinValue, sum = 0f;
                        foreach (float s in window) { lo = Mathf.Min(lo, s); hi = Mathf.Max(hi, s); sum += s; }
                        done2 = hi - lo <= 0.02f * (sum / 50f);
                    }
                }
            }
            Set(flight, "ForwardSpeed", 0f);
            Set(flight, "YawSpeed", 0f);

            float tiltLimit = TiltLimitDeg();
            float floorHeading = PhysicsFloor(cruise, tiltLimit, true, out string floorNote);
            float floorDirect = PhysicsFloor(cruise, tiltLimit, false, out _);
            string mode = flight.GetType().GetField("posMode", AnyMember) != null ? ((int)Get(flight, "posMode")).ToString() : "3 (pre-phase-6 code)";
            string file = Path.Combine(TraceDir, $"yaw_reversal_maxspeed{maxSpeed:F1}_mode{mode.Substring(0, 1)}{(minCruiseSeconds > 0f ? $"_cruise{minCruiseSeconds:F0}s" : "")}.csv");
            File.WriteAllText(file, csv.ToString());

            var result = new ReversalResult
            {
                cruise = cruise, tHalf = tHalf, overshoot = overshoot, maxTilt = maxTilt, maxTiltAll = maxTiltAll,
                altChange = altChange, maxGap = maxGap, lag = vZero - spZero, floorHeading = floorHeading,
                floorDirect = floorDirect, yawTime = yawTime,
            };
            Report($"YAW REVERSAL MaxSpeed {maxSpeed:F1}{(minCruiseSeconds > 0f ? $" (cruise held at least {minCruiseSeconds:F0} s)" : "")}, MPC_POS_MODE {mode}, tilt limit {tiltLimit:F1} deg, rotor drag c {RotorDragSim():E3}: " +
                   $"cruise {cruise:F3} m/s (steady {steady}); 180 deg yaw took {yawTime:F2} s; t_half {tHalf:F2} s; overshoot {overshoot:F2} m; " +
                   $"max tilt {maxTilt:F2} deg (whole test {maxTiltAll:F2}); altitude change {altChange:F3} m; " +
                   $"largest |vel_sp - vel| {maxGap:F2} m/s; setpoint crossed zero along the old heading at {spZero:F2} s, velocity at {vZero:F2} s (lag {vZero - spZero:F2} s); " +
                   $"physics floor t_half {floorHeading:F2} s (stick follows the heading), {floorDirect:F2} s (push straight at the new velocity from t=0); " +
                   $"t_half / floor {tHalf / floorHeading:F2}; {floorNote}; trace {file}");
            done(result);
        }

        /// <summary>
        /// Spec phase 0 "floor": the shortest t_half the physics allows. A point mass with this drone's
        /// mass, body drag (per heading axis) and rotor drag, whose thrust gives at most
        /// g*tan(tilt limit) horizontally, always pushing toward the target velocity. With
        /// followHeading the target is cruise x the heading, which turns at MPC_MAN_Y_MAX to 180 deg;
        /// otherwise the target is the final velocity from t = 0. Integrated at 1 ms.
        /// </summary>
        private float PhysicsFloor(float cruise, float tiltLimitDeg, bool followHeading, out string note)
        {
            float g = Physics.gravity.magnitude, m = rb.mass, h = 0.001f;
            float aMax = g * Mathf.Tan(tiltLimitDeg * Mathf.Deg2Rad);
            float yawRate = Cfg("Px4", "MPC_MAN_Y_MAX");
            Vector3 area = (Vector3)Get(flight, "DragArea");
            float cd = Cfg("Sim", "dragCd");
            float kBody = 0.5f * 1.225f * cd;
            float tmax = (float)Get(flight, "MotorThrustMax"), kf = (float)Get(flight, "kf");
            float c = RotorDragSim();
            Vector2 v = new Vector2(cruise, 0f); // x = original heading
            float psi = 0f, t = 0f;
            while (t < 30f)
            {
                psi = Mathf.Min(180f, psi + yawRate * h);
                float a = (followHeading ? psi : 180f) * Mathf.Deg2Rad;
                Vector2 target = cruise * new Vector2(Mathf.Cos(a), Mathf.Sin(a));
                // Body axes follow the real heading.
                float b = psi * Mathf.Deg2Rad;
                Vector2 fwd = new Vector2(Mathf.Cos(b), Mathf.Sin(b)), side = new Vector2(-fwd.y, fwd.x);
                float vf = Vector2.Dot(v, fwd), vs = Vector2.Dot(v, side);
                Vector2 bodyDrag = -kBody * (area.z * Mathf.Abs(vf) * vf * fwd + area.x * Mathf.Abs(vs) * vs * side);
                // Rotor drag at the thrust that holds the drone up at the tilt limit (Gazebo form).
                float thrust = m * g / (4f * Mathf.Cos(tiltLimitDeg * Mathf.Deg2Rad));
                float omega = Mathf.Sqrt(Mathf.Min(thrust, tmax) / kf);
                Vector2 rotorDrag = -4f * c * omega * v;
                Vector2 err = target - v;
                Vector2 push = err.magnitude > aMax * h ? err.normalized * aMax : err / h;
                v += (push + (bodyDrag + rotorDrag) / m) * h;
                t += h;
                if (-v.x >= 0.5f * cruise) break;
            }
            note = $"floor inputs: a_max = g*tan({tiltLimitDeg:F1}) = {aMax:F2} m/s^2, mass {m:F3} kg, front/side area {area.z:F4}/{area.x:F4} m^2, Cd {cd:F2}, " +
                   $"rotor c {c:E3} N/(rad/s)/(m/s), yaw rate {yawRate:F0} deg/s";
            return t;
        }

        // ---------------------------------------------------------------------------------------
        // Acceptance check 4: release braking from 5 m/s (mode 4).
        // ---------------------------------------------------------------------------------------

        [UnityTest, Category("YawReversal")]
        public IEnumerator ReleaseBraking_StopsAndLocks()
        {
            yield return Fly(DefaultDrone());
            ClearSky();
            SetMode("PosCtl");
            ForcePosMode(4);
            Set(flight, "MaxSpeed", 0.5f);
            yield return FastSteps(100);
            float dt = Time.fixedDeltaTime;

            // Forward until steady (2 % for 1 s, max 30 s).
            Vector3 h0 = Flat(drone.transform.forward);
            var window = new Queue<float>();
            float maxTilt = 0f;
            for (int f = 0; f < 1500; f++)
            {
                Set(flight, "ForwardSpeed", 1f);
                yield return Steps(1);
                maxTilt = Mathf.Max(maxTilt, Tilt(drone.transform));
                window.Enqueue(Vector3.Dot(rb.linearVelocity, h0));
                if (window.Count > 50) window.Dequeue();
                if (window.Count == 50)
                {
                    float lo = float.MaxValue, hi = float.MinValue, sum = 0f;
                    foreach (float s in window) { lo = Mathf.Min(lo, s); hi = Mathf.Max(hi, s); sum += s; }
                    if (sum / 50f > 0.5f && hi - lo <= 0.02f * (sum / 50f)) break;
                }
            }
            float v0 = Vector3.Dot(rb.linearVelocity, h0);
            Vector3 start = drone.transform.position;
            var csv = new StringBuilder("t,along_m,v_along,vsp_along,locked,tilt_deg\n");
            Set(flight, "ForwardSpeed", 0f);

            float stopTime = float.NaN, stopDistance = float.NaN, previous = v0, peakDecel = 0f;
            int lockEvents = 0;
            bool wasLocked = false;
            var along = new List<float>();
            var lockedAt = new List<bool>();
            const int steps = 600; // 12 s
            for (int f = 1; f <= steps; f++)
            {
                yield return Steps(1);
                float t = f * dt;
                float v = Vector3.Dot(rb.linearVelocity, h0);
                float x = Vector3.Dot(drone.transform.position - start, h0);
                bool locked = StickLocked();
                if (locked && !wasLocked) lockEvents++;
                wasLocked = locked;
                along.Add(x);
                lockedAt.Add(locked);
                maxTilt = Mathf.Max(maxTilt, Tilt(drone.transform));
                peakDecel = Mathf.Max(peakDecel, (previous - v) / dt);
                previous = v;
                if (float.IsNaN(stopTime) && v <= 0f) { stopTime = t; stopDistance = x; }
                csv.Append(string.Format(CultureInfo.InvariantCulture, "{0:F3},{1:F4},{2:F4},{3:F4},{4},{5:F3}\n",
                    t, x, v, Vector2.Dot(VelocitySetpointXY(), new Vector2(h0.x, h0.z)), locked ? 1 : 0, Tilt(drone.transform)));
            }
            File.WriteAllText(Path.Combine(TraceDir, "release_braking_maxspeed0.5.csv"), csv.ToString());

            // The lock point: the position setpoint at the end (the last lock).
            Vector2 lockPoint = StickPositionSetpoint();
            float lockAlong = Vector2.Dot(lockPoint - new Vector2(start.x, start.z), new Vector2(h0.x, h0.z));
            int firstLock = lockedAt.IndexOf(true);
            // Error about the lock point from the first lock on: count overshoots (sign changes
            // beyond 2 mm) and the largest error after the first overshoot.
            int overshoots = 0, sign = 0;
            float maxAfterOvershoot = 0f, maxError = 0f;
            for (int i = Mathf.Max(0, firstLock); i < along.Count && firstLock >= 0; i++)
            {
                float e = along[i] - lockAlong;
                maxError = Mathf.Max(maxError, Mathf.Abs(e));
                int s = e > 0.002f ? 1 : e < -0.002f ? -1 : 0;
                if (s != 0 && sign != 0 && s != sign) overshoots++;
                if (s != 0) sign = s;
                if (overshoots > 0) maxAfterOvershoot = Mathf.Max(maxAfterOvershoot, Mathf.Abs(e));
            }
            float finalError = along[along.Count - 1] - lockAlong;
            bool settled = overshoots == 0 || (overshoots <= 2 && maxAfterOvershoot < 0.1f);
            Report($"RELEASE BRAKING from {v0:F3} m/s (MPC_POS_MODE {Get(flight, "posMode")}): stopped after {stopTime:F2} s in {stopDistance:F3} m; " +
                   $"peak deceleration {peakDecel:F2} m/s^2; max tilt {maxTilt:F2} deg; first lock at {(firstLock + 1) * dt:F2} s, lock events {lockEvents}, " +
                   $"locked at end {StickLocked()}; lock point {lockAlong:F3} m along; error about it after the first lock: max {maxError * 100:F1} cm, " +
                   $"overshoots {overshoots}, largest after an overshoot {maxAfterOvershoot * 100:F1} cm, final {finalError * 100:F2} cm");
            Assert.IsTrue(StickLocked(), "position not locked at the end");
            Assert.IsTrue(settled, "oscillation around the lock point");
            Assert.Less(Mathf.Abs(finalError), 0.1f, "not at the lock point");
            Assert.LessOrEqual(maxTilt, 15.5f, "tilt");
            Assert.IsEmpty(errors, string.Join("\n", errors));
        }

        private object StickState() => Get(flight, "stickAcceleration");
        private bool StickLocked() { object s = StickState(); return s != null && (bool)Get(s, "PositionLocked"); }
        private Vector2 StickPositionSetpoint() { object s = StickState(); return s == null ? new Vector2(float.NaN, float.NaN) : (Vector2)Get(s, "PositionSetpoint"); }

        // ---------------------------------------------------------------------------------------
        // Acceptance check 3: the tilt never exceeds 15.5 deg on any path.
        // ---------------------------------------------------------------------------------------

        [UnityTest, Category("YawReversal")]
        public IEnumerator TiltLimit_AllPaths()
        {
            yield return Fly(DefaultDrone(), arm: false);
            var max = new Dictionary<string, float>();
            void Sample(string what) => max[what] = Mathf.Max(max.TryGetValue(what, out float m) ? m : 0f, Tilt(drone.transform));

            Call(flight, "Arm");
            for (int f = 0; f < 600 && (bool)Get(flight, "IsLaunching"); f++) { yield return Steps(1); Sample("takeoff"); }
            yield return FastSteps(50);
            Call(flight, "BeginLanding");
            for (int f = 0; f < 1000 && (bool)Get(flight, "IsArmed"); f++) { yield return Steps(1); Sample("landing"); }
            Assert.IsFalse((bool)Get(flight, "IsArmed"), "never landed");

            Call(flight, "Arm");
            for (int f = 0; f < 300 && (bool)Get(flight, "IsLaunching"); f++) yield return FastSteps(10);
            Time.timeScale = 1f;
            ClearSky();
            SetMode("PosCtl");
            ForcePosMode(4);
            Set(flight, "MaxSpeed", 1f);
            for (int f = 0; f < 400; f++) { Set(flight, "ForwardSpeed", 1f); yield return Steps(1); Sample("keyboard full stick (MaxSpeed 1.0)"); }
            for (int f = 0; f < 150; f++) { Set(flight, "ForwardSpeed", 1f); Set(flight, "StrafeSpeed", 1f); yield return Steps(1); Sample("keyboard full stick diagonal"); }
            Set(flight, "StrafeSpeed", 0f);
            for (int f = 0; f < 400; f++) { Set(flight, "ForwardSpeed", 0f); yield return Steps(1); Sample("keyboard release braking"); }

            SetMode("Pcmd");
            for (int f = 0; f < 150; f++) { Set(flight, "ForwardSpeed", 1f); Set(flight, "StrafeSpeed", 1f); yield return Steps(1); Sample("send_pcmd pitch 1.0 + roll 1.0 (MaxSpeed 1.0)"); }
            Set(flight, "ForwardSpeed", 0f); Set(flight, "StrafeSpeed", 0f);
            for (int f = 0; f < 300; f++) { yield return Steps(1); Sample("send_pcmd release (hold)"); }

            var parts = new List<string>();
            foreach (var kv in max) parts.Add($"{kv.Key} {kv.Value:F2} deg");
            Report("TILT max per path: " + string.Join("; ", parts));
            foreach (var kv in max)
                Assert.LessOrEqual(kv.Value, 15.5f, $"tilt on {kv.Key}");
            Assert.IsEmpty(errors, string.Join("\n", errors));
        }

        // ---------------------------------------------------------------------------------------
        // Acceptance check 10: send_pcmd below the limit is bit-identical; above it the combined tilt is 15 deg.
        // ---------------------------------------------------------------------------------------

        [UnityTest, Category("YawReversal")]
        public IEnumerator Pcmd_TiltBelowLimitUnchanged()
        {
            yield return null;
            var m = GameType("Flight").GetMethod("PcmdTiltSetpoint", System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Static);
            Assert.IsNotNull(m, "Flight.PcmdTiltSetpoint not found");
            float maxTilt = Cfg("Sim", "pcmdMaxTiltDeg"), limit = Cfg("Px4", "MPC_MAN_TILT_MAX");
            int same = 0, clamped = 0, bad = 0;
            string firstBad = null;
            for (int k = 1; k <= 10; k++)
            {
                float scale = k / 10f;
                for (int i = -20; i <= 20; i++)
                    for (int j = -20; j <= 20; j++)
                    {
                        float fwd = i / 20f, strafe = j / 20f;
                        // The formula before phase 6 (Flight.cs FixedUpdate section 5).
                        float oldPitch = fwd * maxTilt * scale, oldRoll = -strafe * maxTilt * scale;
                        var r = (Vector2)m.Invoke(null, new object[] { fwd, strafe, maxTilt, scale, limit });
                        float mag = Mathf.Sqrt(oldPitch * oldPitch + oldRoll * oldRoll);
                        bool ok;
                        if (mag <= limit) { ok = r.x == oldPitch && r.y == oldRoll; if (ok) same++; }
                        else
                        {
                            // Same direction: normalised cross product ~0 and positive dot product
                            // (Vector2.Angle's acos is too coarse near 0 for this check).
                            Vector2 o = new Vector2(oldPitch, oldRoll);
                            float cross = (r.x * o.y - r.y * o.x) / (r.magnitude * o.magnitude);
                            ok = Mathf.Abs(r.magnitude - limit) < 1e-4f && Mathf.Abs(cross) < 1e-5f && Vector2.Dot(r, o) > 0f;
                            if (ok) clamped++;
                        }
                        if (!ok) { bad++; firstBad = firstBad ?? $"fwd {fwd} strafe {strafe} scale {scale}: got {r}, old ({oldPitch}, {oldRoll})"; }
                    }
            }
            var diag = (Vector2)m.Invoke(null, new object[] { 1f, 1f, maxTilt, 1f, limit });
            Report($"send_pcmd tilt: {same} commands below {limit} deg bit-identical, {clamped} clamped to {limit} deg, {bad} wrong; " +
                   $"pitch 1 + roll 1 at MaxSpeed 1: ({diag.x:F3}, {diag.y:F3}) deg, combined {diag.magnitude:F3}");
            Assert.AreEqual(0, bad, firstBad);
        }

        // ---------------------------------------------------------------------------------------
        // Acceptance check 9: the StickAccelerationXY port against hand-computed PX4 values.
        // ---------------------------------------------------------------------------------------

        private const float UnitDt = 0.02f;

        private static object NewStickAccel(float velocityScale, float tiltDeg)
        {
            Type t = GameType("StickAccelerationXY");
            object s = Activator.CreateInstance(t);
            t.GetField("VelocityScale").SetValue(s, velocityScale);
            t.GetField("AccHor").SetValue(s, 3f);
            t.GetField("JerkMax").SetValue(s, 8f);
            t.GetField("TiltLimit").SetValue(s, tiltDeg * Mathf.Deg2Rad);
            t.GetField("Gravity").SetValue(s, 9.81f);
            return s;
        }

        private static Vector2 SA(object s, string p) => (Vector2)s.GetType().GetProperty(p).GetValue(s);
        private static float SAf(object s, string p) => (float)s.GetType().GetProperty(p).GetValue(s);

        /// <summary>Runs n steps with a fixed stick; the position integrates the velocity setpoint.</summary>
        private static void RunStick(object s, Vector2 stick, int n, float yaw = 0f, float yawSp = 0f, Action<int> each = null)
        {
            var gen = s.GetType().GetMethod("GenerateSetpoints");
            Vector2 pos = Vector2.zero;
            for (int i = 0; i < n; i++)
            {
                Vector2 fb = SA(s, "VelocitySetpoint");
                gen.Invoke(s, new object[] { stick, yaw, yawSp, pos, fb, UnitDt });
                pos += SA(s, "VelocitySetpoint") * UnitDt;
                each?.Invoke(i);
            }
        }

        [UnityTest, Category("YawReversal")]
        public IEnumerator StickAccel_HandComputedValues()
        {
            yield return null;
            float g = 9.81f, accTilt = g * Mathf.Tan(15f * Mathf.Deg2Rad);

            // First step from rest, stick forward, yaw 0: target 2*ACC_HOR = 6, drag 0; the jerk
            // limit allows 8 * 0.02 = 0.16 m/s^2; velocity 0.16 * 0.02 = 0.0032 m/s; heading 0 = +z.
            object s = NewStickAccel(5f, 15f);
            RunStick(s, new Vector2(1f, 0f), 1);
            Vector2 a1 = SA(s, "AccelerationSetpoint"), v1 = SA(s, "VelocitySetpoint");
            Report($"step 1: acc {a1.x:F6},{a1.y:F6} (expected 0, 0.16); vel {v1.x:F6},{v1.y:F6} (expected 0, 0.0032); drag coefficient {SAf(s, "DragCoefficient"):F6} (expected 1.2)");
            Assert.AreEqual(0f, a1.x, 1e-7f); Assert.AreEqual(0.16f, a1.y, 1e-6f);
            Assert.AreEqual(0.0032f, v1.y, 1e-7f);
            Assert.AreEqual(2f * 3f / 5f, SAf(s, "DragCoefficient"), 1e-6f, "drag coefficient 2*ACC_HOR/vel_scale");

            // Steady speed = the velocity scale; at steady state the drag equals the stick acceleration (1.2 * 5 = 6).
            float maxAcc = 0f, maxJerkStep = 0f;
            Vector2 prev = a1;
            RunStick(s, new Vector2(1f, 0f), 1500, 0f, 0f, i =>
            {
                Vector2 a = SA(s, "AccelerationSetpoint");
                maxAcc = Mathf.Max(maxAcc, a.magnitude);
                prev = a;
            });
            Vector2 vs = SA(s, "VelocitySetpoint"), drag = SA(s, "LastDrag");
            Report($"after 30 s full stick: vel {vs.x:F5},{vs.y:F5} (expected 0, 5); drag {drag.y:F4} (expected 6.0); brake boost {SAf(s, "BrakeBoost"):F6} (expected 1); max |acc| {maxAcc:F4} (tilt cap {accTilt:F4})");
            Assert.AreEqual(5f, vs.y, 1e-3f, "steady speed = velocity scale");
            Assert.AreEqual(0f, vs.x, 1e-5f);
            Assert.AreEqual(6f, drag.y, 1e-2f, "drag at steady speed");
            Assert.LessOrEqual(maxAcc, accTilt + 1e-4f, "tilt limit");

            // Release: the brake boost moves toward max(2, sqrt(5)) = 2.2361 with alpha = dt/(0.8+dt).
            RunStick(s, Vector2.zero, 1);
            float b1 = SAf(s, "BrakeBoost"), b1Expected = 1f + UnitDt / (0.8f + UnitDt) * (Mathf.Sqrt(5f) - 1f);
            RunStick(s, Vector2.zero, 499);
            float b10 = SAf(s, "BrakeBoost");
            bool locked = (bool)s.GetType().GetProperty("PositionLocked").GetValue(s);
            Vector2 vEnd = SA(s, "VelocitySetpoint");
            Report($"release: boost after 1 step {b1:F6} (expected {b1Expected:F6}), after 10 s {b10:F5} (expected {Mathf.Sqrt(5f):F5}); vel {vEnd}, position locked {locked}");
            Assert.AreEqual(b1Expected, b1, 1e-5f);
            Assert.AreEqual(Mathf.Sqrt(5f), b10, 1e-3f);
            Assert.AreEqual(Vector2.zero, vEnd, "velocity setpoint not zero after release");
            Assert.IsTrue(locked, "position not locked after release");

            // Velocity scale 1 (PX4's MPC_VEL_MANUAL): the boost goes to max(2, 1) = 2.
            object s1 = NewStickAccel(1f, 15f);
            RunStick(s1, Vector2.zero, 500);
            Report($"velocity scale 1: boost after 10 s released {SAf(s1, "BrakeBoost"):F5} (expected 2)");
            Assert.AreEqual(2f, SAf(s1, "BrakeBoost"), 1e-3f);

            // Velocity scale 10: steady speed 10.
            object s10 = NewStickAccel(10f, 15f);
            RunStick(s10, new Vector2(1f, 0f), 2000);
            Report($"velocity scale 10: steady {SA(s10, "VelocitySetpoint").y:F4} (expected 10)");
            Assert.AreEqual(10f, SA(s10, "VelocitySetpoint").y, 1e-2f);
        }

        [UnityTest, Category("YawReversal")]
        public IEnumerator StickAccel_JerkTiltAndHeading()
        {
            yield return null;
            // Jerk: with the tilt limit out of the way (89 deg) no axis changes by more than
            // MPC_JERK_MAX * dt per step, through a sequence of stick moves.
            object s = NewStickAccel(5f, 89f);
            Vector2[] sticks = { new Vector2(1f, 0f), new Vector2(-1f, 0f), new Vector2(0.7f, 0.7f), Vector2.zero, new Vector2(0f, -1f) };
            float maxStep = 0f;
            Vector2 prev = Vector2.zero;
            foreach (Vector2 st in sticks)
                RunStick(s, st, 150, 0f, 0f, i =>
                {
                    Vector2 a = SA(s, "AccelerationSetpoint");
                    maxStep = Mathf.Max(maxStep, Mathf.Max(Mathf.Abs(a.x - prev.x), Mathf.Abs(a.y - prev.y)));
                    prev = a;
                });
            Report($"jerk: largest per-axis change per step {maxStep:F5} m/s^2 (limit {8f * UnitDt:F5})");
            Assert.LessOrEqual(maxStep, 8f * UnitDt + 1e-5f, "jerk limit");

            // Tilt: 15 deg caps |acc| at g*tan(15) through the same sequence.
            object t = NewStickAccel(5f, 15f);
            float maxAcc = 0f, cap = 9.81f * Mathf.Tan(15f * Mathf.Deg2Rad);
            foreach (Vector2 st in sticks)
                RunStick(t, st, 150, 0f, 0f, i => maxAcc = Mathf.Max(maxAcc, SA(t, "AccelerationSetpoint").magnitude));
            Report($"tilt: largest |acc| {maxAcc:F5} m/s^2 (cap {cap:F5})");
            Assert.LessOrEqual(maxAcc, cap + 1e-4f, "tilt limit");
            Assert.Greater(maxAcc, cap - 1e-3f, "the cap is reached with full stick");

            // Heading frame (Sticks::rotateIntoHeadingFrameXY): the yaw setpoint if finite, else the yaw.
            object h1 = NewStickAccel(5f, 15f);
            RunStick(h1, new Vector2(1f, 0f), 1, 90f, float.NaN);
            object h2 = NewStickAccel(5f, 15f);
            RunStick(h2, new Vector2(1f, 0f), 1, 90f, 0f);
            Vector2 a1 = SA(h1, "AccelerationSetpoint"), a2 = SA(h2, "AccelerationSetpoint");
            Report($"heading: yaw 90 while yawing -> acc {a1} (expected +x); yaw 90 with yaw setpoint 0 -> acc {a2} (expected +z)");
            Assert.AreEqual(0.16f, a1.x, 1e-5f); Assert.AreEqual(0f, a1.y, 1e-5f);
            Assert.AreEqual(0f, a2.x, 1e-5f); Assert.AreEqual(0.16f, a2.y, 1e-5f);
        }
    }
}

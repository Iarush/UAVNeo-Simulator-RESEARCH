using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace UAVSim.Tests.PlayMode
{
    /// <summary>
    /// Phase 2 of .plans/real-flight-physics-spec.md: PX4 velocity/position loops, the send_pcmd
    /// path and the removal of the velocity overrides. Uses the flight harness in FlightMotorTests.cs.
    /// </summary>
    public partial class FlightMotorTests
    {
        /// <summary>Methods that may still write the Rigidbody velocity: Relaunch (respawn) only.</summary>
        private static readonly HashSet<string> VelocityWriters = new HashSet<string> { "Relaunch" };

        [UnityTest]
        public IEnumerator NoVelocityWritesInFlight()
        {
            // 1. Source scan: every assignment to linearVelocity / angularVelocity and its method.
            string[] lines = File.ReadAllLines(Path.Combine(Application.dataPath, "Scripts", "Drone", "Flight.cs"));
            var method = new Regex(@"^\s*(?:public|private|protected|internal)[^=;(]*\s(\w+)\s*\(");
            var write = new Regex(@"\.(linearVelocity|angularVelocity)\s*(=(?!=)|\+=|-=|\*=)");
            string current = "?";
            var found = new List<string>();
            var bad = new List<string>();
            for (int i = 0; i < lines.Length; i++)
            {
                string code = lines[i].Split(new[] { "//" }, System.StringSplitOptions.None)[0];
                Match m = method.Match(code);
                if (m.Success) current = m.Groups[1].Value;
                if (!write.IsMatch(code)) continue;
                found.Add($"{current}:{i + 1}");
                if (!VelocityWriters.Contains(current))
                    bad.Add($"Flight.cs:{i + 1} in {current}: {lines[i].Trim()}");
            }
            Report($"velocity writes: {string.Join(", ", found)} (allowed: Relaunch)");
            Assert.IsEmpty(bad, "velocity written in flight:\n" + string.Join("\n", bad));

            // 2. Runtime probe: forward, yaw, then release. The velocity change per step must not
            //    exceed what gravity plus the motor thrust can give (10% margin for the lag average).
            yield return Fly(DefaultDrone());
            yield return FastSteps(100);
            Vector3 previous = rb.linearVelocity;
            float dt = Time.fixedDeltaTime, worst = 0f;
            int worstStep = -1;
            for (int f = 0; f < 250; f++)
            {
                Set(flight, "ForwardSpeed", f < 50 ? 1f : 0f);
                Set(flight, "YawSpeed", f >= 50 && f < 100 ? 1f : 0f);
                yield return Steps(1);
                Vector3 v = rb.linearVelocity;
                float[] t = (float[])Get(flight, "MotorThrusts");
                float limit = (t[0] + t[1] + t[2] + t[3]) / rb.mass * dt * 1.1f + 0.01f;
                float excess = (v - previous - Physics.gravity * dt).magnitude / limit;
                if (excess > worst) { worst = excess; worstStep = f; }
                previous = v;
            }
            Report($"runtime probe: largest velocity change / force-implied limit = {worst:F3} (step {worstStep})");
            Assert.LessOrEqual(worst, 1f, "velocity changed by more than the forces allow");
            Assert.IsEmpty(errors, string.Join("\n", errors));
        }

        [UnityTest]
        public IEnumerator PcmdTilt_StepResponse()
        {
            yield return Fly(DefaultDrone());
            yield return FastSteps(100);
            float target = 0.5f * Cfg("Sim", "pcmdMaxTiltDeg") * (float)Get(flight, "MaxSpeed");
            float rateMax = Cfg("Px4", "MC_PITCHRATE_MAX");
            float dt = Time.fixedDeltaTime, t63 = float.NaN, peak = 0f, maxRate = 0f, previous = Normalize(drone.transform.eulerAngles.x);
            for (int f = 1; f <= 75; f++)
            {
                Set(flight, "ForwardSpeed", 0.5f);
                yield return Steps(1);
                float pitch = Normalize(drone.transform.eulerAngles.x);
                if (float.IsNaN(t63) && pitch >= 0.632f * target)
                    t63 = (f - 1 + (0.632f * target - previous) / (pitch - previous)) * dt;
                peak = Mathf.Max(peak, pitch);
                maxRate = Mathf.Max(maxRate, Mathf.Abs(drone.transform.InverseTransformDirection(rb.angularVelocity).x) * Mathf.Rad2Deg);
                previous = pitch;
            }
            Set(flight, "ForwardSpeed", 0f);
            float pitchP = (float)GameType("FlightConfig").GetNestedType("Px4StockGains").GetField("PitchP").GetRawConstantValue();
            float expected = 1f / pitchP;
            float overshoot = peak / target - 1f;
            Report($"pitch 0.5: target {target:F2} deg, final {previous:F2} deg, 63% at {t63 * 1000:F0} ms (expected {expected * 1000:F0} ms +-25%), " +
                   $"overshoot {overshoot:P1}, max pitch rate {maxRate:F1} deg/s (limit {rateMax})");
            Assert.AreEqual(target, previous, 0.05f * target, "steady tilt");
            Assert.AreEqual(expected, t63, 0.25f * expected, "time to 63%");
            Assert.Less(overshoot, 0.10f, "overshoot");
            Assert.LessOrEqual(maxRate, rateMax * 1.01f, "pitch rate above MC_PITCHRATE_MAX");
            Assert.IsEmpty(errors, string.Join("\n", errors));
        }

        [UnityTest]
        public IEnumerator PcmdYaw_RateAndNoBank()
        {
            yield return Fly(DefaultDrone());
            yield return FastSteps(100);
            float maxRoll = 0f, sum = 0f;
            int n = 0;
            for (int f = 0; f < 150; f++)
            {
                Set(flight, "YawSpeed", 1f);
                yield return Steps(1);
                maxRoll = Mathf.Max(maxRoll, Mathf.Abs(Normalize(drone.transform.eulerAngles.z)));
                if (f >= 100) { sum += rb.angularVelocity.y * Mathf.Rad2Deg; n++; }
            }
            Set(flight, "YawSpeed", 0f);
            float rate = sum / n, expected = Cfg("Sim", "pcmdMaxYawRateDegS");
            Report($"yaw 1.0: steady rate {rate:F2} deg/s (expected {expected:F2}), max roll {maxRoll:F3} deg");
            Assert.AreEqual(expected, rate, 0.05f * expected, "yaw rate");
            Assert.Less(maxRoll, 1f, "yaw input rolled the drone (banking)");
            Assert.IsEmpty(errors, string.Join("\n", errors));
        }

        [UnityTest]
        public IEnumerator TurnKeepsMomentum()
        {
            yield return Fly(DefaultDrone());
            yield return FastSteps(100);
            int f = 0;
            for (; f < 250 && Flat3(rb.linearVelocity).magnitude < 1f; f++)
            {
                Set(flight, "ForwardSpeed", 0.5f);
                yield return Steps(1);
            }
            Report($"reached {Flat3(rb.linearVelocity).magnitude:F2} m/s after {f * Time.fixedDeltaTime:F2} s");
            Assert.GreaterOrEqual(Flat3(rb.linearVelocity).magnitude, 1f, "never reached 1 m/s");

            float peak = 0f, end = 0f;
            int turnSteps = Mathf.RoundToInt(Mathf.PI / 2f / (Cfg("Sim", "pcmdMaxYawRateDegS") * Mathf.Deg2Rad) / Time.fixedDeltaTime);
            for (int s = 0; s < turnSteps + 100; s++)
            {
                Set(flight, "ForwardSpeed", 0.5f);
                Set(flight, "YawSpeed", s < turnSteps ? 1f : 0f);
                yield return Steps(1);
                end = Vector3.Angle(Flat3(rb.linearVelocity), Flat(drone.transform.forward));
                peak = Mathf.Max(peak, end);
            }
            Set(flight, "ForwardSpeed", 0f);
            Report($"90 deg turn: peak velocity-heading angle {peak:F1} deg, {100 * Time.fixedDeltaTime:F0} s after the turn {end:F1} deg, speed {Flat3(rb.linearVelocity).magnitude:F2} m/s");
            Assert.Greater(peak, 20f, "velocity followed the nose instantly");
            Assert.Less(end, peak / 2f, "velocity did not converge toward the heading");
            Assert.IsEmpty(errors, string.Join("\n", errors));
        }

        [UnityTest]
        public IEnumerator Hold_StopsByTilting()
        {
            yield return Fly(DefaultDrone());
            yield return FastSteps(100);
            for (int f = 0; f < 250 && Flat3(rb.linearVelocity).magnitude < 1f; f++)
            {
                Set(flight, "ForwardSpeed", 0.5f);
                yield return Steps(1);
            }
            Vector3 dir = Flat3(rb.linearVelocity).normalized;
            float v0 = Flat3(rb.linearVelocity).magnitude, dt = Time.fixedDeltaTime;
            Vector3 start = drone.transform.position;
            float previous = v0, peakDecel = 0f, minPitch = 0f, stopDistance = float.NaN;
            Vector3 at5 = Vector3.zero;
            Set(flight, "ForwardSpeed", 0f);
            for (int f = 1; f <= 400; f++)
            {
                yield return Steps(1);
                float along = Vector3.Dot(rb.linearVelocity, dir);
                peakDecel = Mathf.Max(peakDecel, (previous - along) / dt);
                minPitch = Mathf.Min(minPitch, Normalize(drone.transform.eulerAngles.x));
                if (float.IsNaN(stopDistance) && along <= 0f)
                    stopDistance = Vector3.Dot(drone.transform.position - start, dir);
                if (f == 250) at5 = drone.transform.position;
                previous = along;
            }
            float accMax = Cfg("Px4", "MPC_ACC_HOR_MAX");
            float held = Flat3(drone.transform.position - at5).magnitude;
            Report($"release at {v0:F2} m/s: peak deceleration {peakDecel:F2} m/s^2 (limit {accMax * 1.1f:F2}), stopping distance {stopDistance:F3} m " +
                   $"(minimum {v0 * v0 / (2f * accMax):F3}), max nose-up pitch {-minPitch:F2} deg, moved {held * 100:F1} cm from 5 s to 8 s");
            Assert.LessOrEqual(peakDecel, accMax * 1.1f, "braked harder than MPC_ACC_HOR_MAX");
            Assert.GreaterOrEqual(stopDistance, v0 * v0 / (2f * accMax), "stopped too quickly");
            Assert.Less(minPitch, -1f, "drone did not tilt backwards to brake");
            Assert.Less(held, 0.1f, "position not held");
            Assert.IsEmpty(errors, string.Join("\n", errors));
        }

        [UnityTest]
        public IEnumerator LightFrames_DoNotClimbAway_YawLight() { yield return HoverCheck(YawLight()); }

        [UnityTest]
        public IEnumerator LightFrames_DoNotClimbAway_Small() { yield return HoverCheck(Small()); }

        [UnityTest]
        public IEnumerator Saturation_YawGivesWayFirst()
        {
            yield return Fly(DefaultDrone());
            float hover = rb.mass * Physics.gravity.magnitude, tmax = (float)Get(flight, "MotorThrustMax");
            var eff = (float[,])Get(flight, "effectiveness");

            Vector3 Torque(Vector3 normalised)
            {
                float[] u = new float[4];
                Call(flight, "Allocate", normalised, hover, u);
                Vector3 tau = Vector3.zero;
                for (int i = 0; i < 4; i++)
                    tau += new Vector3(eff[1, i], eff[2, i], eff[3, i]) * u[i] * tmax;
                return tau;
            }

            // Normalised torque axes: x pitch, y yaw, z roll.
            Vector3 rollOnly = Torque(new Vector3(0f, 0f, 1f));
            Vector3 yawOnly = Torque(new Vector3(0f, 1f, 0f));
            Vector3 both = Torque(new Vector3(0f, 1f, 1f));
            Report($"roll alone {rollOnly.z:F4} N*m; yaw alone {yawOnly.y:F4} N*m; roll+yaw: roll {both.z:F4} N*m, yaw {both.y:F4} N*m");
            Assert.AreEqual(rollOnly.z, both.z, 0.02f * Mathf.Abs(rollOnly.z), "roll torque not preserved");
            Assert.Less(Mathf.Abs(both.y), 0.9f * Mathf.Abs(yawOnly.y), "yaw did not give way");
        }

        private static Vector3 Flat3(Vector3 v) { v.y = 0f; return v; }
    }
}

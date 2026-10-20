using System.Collections;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace UAVSim.Tests.PlayMode
{
    /// <summary>
    /// Phase 3 of .plans/real-flight-physics-spec.md: the keyboard/controller path (PX4 position
    /// mode). Uses the flight harness in FlightMotorTests.cs.
    /// </summary>
    public partial class FlightMotorTests
    {
        private static float KeyboardMaxSpeed() => (float)GameType("Flight").GetField("KeyboardMaxSpeed").GetRawConstantValue();

        [UnityTest]
        public IEnumerator PosCtl_FullStickSpeed([Values(0.5f, 1.0f)] float maxSpeed)
        {
            yield return Fly(DefaultDrone());
            SetMode("PosCtl");
            Set(flight, "MaxSpeed", maxSpeed);
            yield return FastSteps(50);
            float sum = 0f;
            int n = 0;
            for (int f = 0; f < 700; f++)
            {
                Set(flight, "ForwardSpeed", 1f);
                yield return Steps(1);
                if (f >= 600) { sum += Flat3(rb.linearVelocity).magnitude; n++; }
            }
            Set(flight, "ForwardSpeed", 0f);
            float speed = sum / n, expected = KeyboardMaxSpeed() * maxSpeed;
            Report($"PosCtl full stick at MaxSpeed {maxSpeed}: steady speed {speed:F3} m/s (expected {expected:F3})");
            Assert.AreEqual(expected, speed, 0.05f * expected, "steady speed");
            Assert.IsEmpty(errors, string.Join("\n", errors));
        }

        [UnityTest]
        public IEnumerator PosCtl_ClimbRateLimit([Values(0.5f, 1.0f)] float maxSpeed)
        {
            yield return Fly(DefaultDrone());
            SetMode("PosCtl");
            Set(flight, "MaxSpeed", maxSpeed);
            float peak = 0f;
            for (int f = 0; f < 150; f++)
            {
                Set(flight, "VerticalSpeed", 1f);
                yield return Steps(1);
                peak = Mathf.Max(peak, rb.linearVelocity.y);
            }
            Set(flight, "VerticalSpeed", 0f);
            float limit = (float)GameType("Flight").GetField("KeyboardMaxClimbRate").GetRawConstantValue() * maxSpeed;
            Report($"PosCtl full climb at MaxSpeed {maxSpeed}: peak climb {peak:F3} m/s (limit {limit:F3} +5%)");
            Assert.LessOrEqual(peak, limit * 1.05f, "climb rate above the limit");
            Assert.Greater(peak, limit * 0.9f, "never reached the climb rate");
            Assert.IsEmpty(errors, string.Join("\n", errors));
        }

        [UnityTest]
        public IEnumerator PosCtl_ReleaseBrakesAndHolds()
        {
            yield return Fly(DefaultDrone());
            SetMode("PosCtl");
            Set(flight, "MaxSpeed", 1f);
            for (int f = 0; f < 200; f++)
            {
                Set(flight, "ForwardSpeed", 1f);
                yield return Steps(1);
            }
            Vector3 dir = Flat3(rb.linearVelocity).normalized;
            float v0 = Flat3(rb.linearVelocity).magnitude, dt = Time.fixedDeltaTime;
            Vector3 start = drone.transform.position, at5 = Vector3.zero;
            float previous = v0, peakDecel = 0f, minPitch = 0f, stopDistance = float.NaN;
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
            Report($"PosCtl release at {v0:F2} m/s: peak deceleration {peakDecel:F2} m/s^2, stopping distance {stopDistance:F3} m " +
                   $"(minimum {v0 * v0 / (2f * accMax):F3}), max nose-up {-minPitch:F2} deg, moved {held * 100:F1} cm from 5 s to 8 s");
            Assert.LessOrEqual(peakDecel, accMax * 1.1f, "braked harder than MPC_ACC_HOR_MAX");
            Assert.GreaterOrEqual(stopDistance, v0 * v0 / (2f * accMax), "stopped too quickly");
            Assert.Less(minPitch, -1f, "drone did not tilt backwards to brake");
            Assert.Less(held, 0.1f, "position not held");
            Assert.IsEmpty(errors, string.Join("\n", errors));
        }

        [UnityTest]
        public IEnumerator Pcmd_UnchangedByPosCtl()
        {
            yield return Fly(DefaultDrone());
            float maxSpeed = (float)Get(flight, "MaxSpeed");

            // send_pcmd: forward 0.5 is a tilt.
            for (int f = 0; f < 50; f++) { Set(flight, "ForwardSpeed", 0.5f); yield return Steps(1); }
            float pcmdPitch = Normalize(drone.transform.eulerAngles.x);
            float tiltTarget = 0.5f * Cfg("Sim", "pcmdMaxTiltDeg") * maxSpeed;
            Set(flight, "ForwardSpeed", 0f);
            yield return FastSteps(300);

            // Keyboard: forward 0.5 is a speed; the drone ends level at that speed.
            SetMode("PosCtl");
            float sum = 0f; int n = 0;
            for (int f = 0; f < 700; f++)
            {
                Set(flight, "ForwardSpeed", 0.5f);
                yield return Steps(1);
                if (f >= 600) { sum += Flat3(rb.linearVelocity).magnitude; n++; }
            }
            Set(flight, "ForwardSpeed", 0f);
            float speed = sum / n, speedTarget = 0.5f * KeyboardMaxSpeed() * maxSpeed;
            Report($"send_pcmd forward 0.5: pitch {pcmdPitch:F2} deg (target {tiltTarget:F2}); keyboard forward 0.5: speed {speed:F3} m/s (target {speedTarget:F3})");
            Assert.AreEqual(tiltTarget, pcmdPitch, 0.05f * tiltTarget, "send_pcmd tilt meaning changed");
            Assert.AreEqual(speedTarget, speed, 0.05f * speedTarget, "keyboard speed");
            Assert.IsEmpty(errors, string.Join("\n", errors));
        }
    }
}

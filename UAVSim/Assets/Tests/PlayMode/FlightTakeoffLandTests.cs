using System.Collections;
using System.IO;
using System.Text.RegularExpressions;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace UAVSim.Tests.PlayMode
{
    /// <summary>
    /// Phase 4 of .plans/real-flight-physics-spec.md: physics takeoff and landing, and the convex
    /// MeshCollider ground contact for custom frames. Uses the flight harness in FlightMotorTests.cs.
    /// </summary>
    public partial class FlightMotorTests
    {
        [UnityTest]
        public IEnumerator Takeoff_ClimbsUnderThrust()
        {
            // Source scan: no kinematic moves left in Flight.cs.
            string source = File.ReadAllText(Path.Combine(Application.dataPath, "Scripts", "Drone", "Flight.cs"));
            var move = new Regex(@"\.(Move|MovePosition|MoveRotation)\s*\(");
            Assert.IsFalse(move.IsMatch(source), "Flight.cs still moves the Rigidbody kinematically");

            yield return Fly(DefaultDrone(), arm: false);
            yield return Settle();
            float groundY = drone.transform.position.y;

            bool launchLogged = false;
            Application.LogCallback onLog = (m, s, t) => { if (m.Contains("Launch complete")) launchLogged = true; };
            Application.logMessageReceived += onLog;
            try
            {
                Call(flight, "Arm");
                float tmax = (float)Get(flight, "MotorThrustMax");
                float firstThrust = 0f, firstCommanded = 0f, maxClimb = 0f;
                var trace = new System.Text.StringBuilder();
                int steps = 0;
                for (; steps < 1000 && (bool)Get(flight, "IsLaunching"); steps++)
                {
                    yield return Steps(1);
                    if (steps % 5 == 0 && steps <= 100)
                        trace.Append($" {steps * Time.fixedDeltaTime:F1}s vz {rb.linearVelocity.y:F3} h {drone.transform.position.y - groundY:F3};");
                    if (steps == 0)
                    {
                        float[] t = (float[])Get(flight, "MotorThrusts"), u = (float[])Get(flight, "MotorCommands");
                        for (int i = 0; i < 4; i++) { firstThrust += t[i]; firstCommanded += u[i] * tmax; }
                    }
                    maxClimb = Mathf.Max(maxClimb, rb.linearVelocity.y);
                }
                Assert.IsFalse((bool)Get(flight, "IsLaunching"), "takeoff never finished");
                float doneHeight = drone.transform.position.y - groundY;
                yield return Steps(50);
                float height = drone.transform.position.y - groundY;
                float limit = Mathf.Min(Cfg("Px4", "MPC_TKO_SPEED"), Cfg("Px4", "MPC_Z_VEL_MAX_UP"));
                float launch = Cfg("Sim", "launchHeight");
                Report($"takeoff: first step thrust {firstThrust:F3} N of {firstCommanded:F3} N commanded (lag); max climb {maxClimb:F3} m/s (limit {limit:F2}); " +
                       $"done after {steps * Time.fixedDeltaTime:F2} s at {doneHeight:F3} m, {height:F3} m 1 s later (target {launch:F2}); 'Launch complete' logged {launchLogged}");
                Report($"takeoff trace:{trace}");
                Assert.Greater(firstThrust, 0f, "motors did not start");
                Assert.Less(firstThrust, 0.99f * firstCommanded, "thrust reached its command in one step: no spin-up lag");
                Assert.LessOrEqual(maxClimb, limit * 1.05f, "climb rate above the takeoff speed");
                Assert.AreEqual(launch, height, 0.1f, "takeoff height");
                Assert.IsTrue(launchLogged, "no 'Launch complete' log");
                Assert.IsEmpty(errors, string.Join("\n", errors));
            }
            finally { Application.logMessageReceived -= onLog; }
        }

        [UnityTest]
        public IEnumerator Land_DescendsAndDisarms_DefaultDrone() { yield return LandCheck(DefaultDrone()); }

        [UnityTest]
        public IEnumerator Land_DescendsAndDisarms_Cube() { yield return LandCheck(Cube(0f)); }

        [UnityTest]
        public IEnumerator CustomFrame_RestsOnConvexCollider_Cube() { yield return RestCheck(Cube(0f)); }

        [UnityTest]
        public IEnumerator CustomFrame_RestsOnConvexCollider_SevenInch() { yield return RestCheck(SevenInch()); }

        private IEnumerator LandCheck(Frame frame)
        {
            yield return Fly(frame);
            yield return FastSteps(100);

            Call(flight, "BeginLanding");
            float maxDescent = 0f, touchdown = float.NaN, disarmed = float.NaN;
            for (int f = 0; f < 1000; f++)
            {
                yield return Steps(1);
                float time = (f + 1) * Time.fixedDeltaTime;
                maxDescent = Mathf.Max(maxDescent, -rb.linearVelocity.y);
                if (float.IsNaN(touchdown) && (bool)Get(flight, "groundContact")) touchdown = time;
                if (!(bool)Get(flight, "IsArmed")) { disarmed = time; break; }
            }
            yield return FastSteps(100);
            float gap = GroundGap();
            float limit = Mathf.Min(Cfg("Px4", "MPC_LAND_SPEED"), Cfg("Px4", "MPC_Z_VEL_MAX_DN"));
            float[] t = (float[])Get(flight, "MotorThrusts");
            Report($"{frame.name} landing: max descent {maxDescent:F3} m/s (limit {limit:F2}); touchdown {touchdown:F2} s, disarmed {disarmed:F2} s " +
                   $"({disarmed - touchdown:F2} s after); resting gap {gap * 1000:F1} mm, tilt {Vector3.Angle(drone.transform.up, Vector3.up):F2} deg; thrusts {Fmt(t)} N");
            Assert.IsFalse(float.IsNaN(touchdown), "never touched the ground");
            Assert.IsFalse(float.IsNaN(disarmed), "never disarmed");
            Assert.LessOrEqual(maxDescent, limit * 1.05f, "descent faster than the landing speed");
            Assert.LessOrEqual(disarmed - touchdown, 1.5f, "disarm took too long after touchdown");
            // The stock drone's 1 m prefab box overlaps the launch pad and floor-grid decorations
            // (not changed by this phase), so its gap is reported; custom frames must rest on their hull.
            if (!frame.IsDefault)
                Assert.AreEqual(0f, gap, 0.01f, "custom frame sinks into or floats above the ground");
            Assert.AreEqual(0f, t[0] + t[1] + t[2] + t[3], 1e-3f, "motors still pushing after disarm");
            Assert.IsEmpty(errors, string.Join("\n", errors));
        }

        private IEnumerator RestCheck(Frame frame)
        {
            yield return Fly(frame, arm: false);
            yield return Settle();

            MeshCollider hull = drone.GetComponentInChildren<MeshCollider>();
            Assert.IsNotNull(hull, "custom frame has no MeshCollider");
            Assert.IsTrue(hull.convex && hull.enabled, "MeshCollider must be convex and enabled");
            int otherSolid = 0;
            foreach (Collider c in drone.GetComponentsInChildren<Collider>(true))
                if (c != hull && c.enabled && !c.isTrigger) otherSolid++;
            float gap = GroundGap();
            float tilt = Vector3.Angle(drone.transform.up, Vector3.up);
            Report($"{frame.name} at rest: hull bounds {hull.bounds.size:F4} m; other solid colliders {otherSolid}; gap {gap * 1000:F1} mm; tilt {tilt:F2} deg; " +
                   $"vertical speed {rb.linearVelocity.y:F4} m/s");
            Assert.AreEqual(0, otherSolid, "the prefab's colliders are still active on a custom frame");
            Assert.AreEqual(0f, gap, 0.01f, "custom frame sinks or floats");
            Assert.Less(tilt, 1f, "custom frame does not rest level");
            Assert.Less(Mathf.Abs(rb.linearVelocity.y), 0.01f, "custom frame still moving");
            Assert.IsEmpty(errors, string.Join("\n", errors));
        }

        /// <summary>Waits (up to 10 s of physics) until the disarmed drone has come to rest.</summary>
        private IEnumerator Settle()
        {
            for (int i = 0; i < 50; i++)
            {
                yield return FastSteps(10);
                if (rb.linearVelocity.magnitude < 0.005f && rb.angularVelocity.magnitude < 0.01f) break;
            }
            yield return FastSteps(20);
        }

        /// <summary>
        /// Gap between the drone's solid colliders and the surface it rests on (m): negative = sunk
        /// in (largest PhysX penetration depth), positive = floating (sweep distance down).
        /// Works on uneven ground and with a slight tilt, unlike comparing bounds with a raycast.
        /// </summary>
        private float GroundGap()
        {
            float sink = 0f;
            var names = new System.Collections.Generic.List<string>();
            foreach (Collider c in drone.GetComponentsInChildren<Collider>())
            {
                if (!c.enabled || c.isTrigger) continue;
                names.Add($"{c.GetType().Name} on {c.name}");
                Bounds b = c.bounds;
                foreach (Collider o in Physics.OverlapBox(b.center, b.extents + Vector3.one * 0.05f, Quaternion.identity, ~0, QueryTriggerInteraction.Ignore))
                {
                    if (o.attachedRigidbody == rb) continue;
                    if (Physics.ComputePenetration(c, c.transform.position, c.transform.rotation, o, o.transform.position, o.transform.rotation, out Vector3 dir, out float depth))
                    {
                        sink = Mathf.Max(sink, depth);
                        names.Add($"overlaps {o.name} by {depth * 1000:F1} mm along {dir:F2}");
                    }
                }
            }
            Report("colliders: " + string.Join("; ", names));
            if (sink > 0f) return -sink;
            return rb.SweepTest(Vector3.down, out RaycastHit hit, 1f, QueryTriggerInteraction.Ignore) ? hit.distance : 1f;
        }
    }
}

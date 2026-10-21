using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

namespace UAVSim.Tests.PlayMode
{
    /// <summary>
    /// Phase 1 of .plans/real-flight-physics-spec.md: motor lag, per-motor forces at the real
    /// motor positions, allocation about the centre of mass, sign checks and crash damage by corner.
    ///
    /// Every flight test loads Main, fills the CustomFrameLoader statics (empty = default drone),
    /// calls MainMenu.BeginSimulation, arms, waits for the launch to finish and then runs with one
    /// physics step per frame (Time.captureDeltaTime = fixedDeltaTime). Custom-frame tests need the
    /// mass service; they try MassServiceLauncher.EnsureRunning and report Ignored if it never answers.
    /// Game code is in Assembly-CSharp, so it is reached by reflection.
    /// Motor order everywhere: FR 0, FL 1, BL 2, BR 3.
    /// </summary>
    [Timeout(900000)]
    public partial class FlightMotorTests
    {
        private const int ServicePort = 5070;
        private static readonly string[] Corner = { "FR", "FL", "BL", "BR" };

        private struct Node
        {
            public int type; public Vector3 cad; public float kg;
            public Node(int type, float x, float y, float z, float kg) { this.type = type; cad = new Vector3(x, y, z); this.kg = kg; }
        }

        private class Frame
        {
            public string name, stl; public float density; public List<Node> nodes;
            public bool IsDefault => stl == null;
        }

        private static string RepoRoot => Path.GetFullPath(Path.Combine(Application.dataPath, "..", ".."));
        private static string CubeStl => Path.Combine(RepoRoot, "tools", "test-data", "cube100.stl");
        private static string FixtureDir => Path.Combine(Application.dataPath, "Tests", "PlayMode", "Fixtures~");

        private static Frame DefaultDrone() => new Frame { name = "default drone" };

        private static Frame Cube(float batteryX)
        {
            return new Frame
            {
                name = batteryX == 0f ? "cube quad" : $"cube quad, battery at x={batteryX}",
                stl = CubeStl,
                density = 1000f,
                nodes = new List<Node>
                {
                    new Node(1, batteryX, 0f, 0f, 0.5f),
                    new Node(0, 0.15f, 0.15f, 0f, 0.05f),
                    new Node(0, -0.15f, 0.15f, 0f, 0.05f),
                    new Node(0, 0.15f, -0.15f, 0f, 0.05f),
                    new Node(0, -0.15f, -0.15f, 0f, 0.05f),
                },
            };
        }

        // DEVLOG 2026-10-14 / docs/handoff-flight-fixes.md section 2: frames that climbed away before the gain fix.
        private static Frame YawLight() => new Frame
        {
            name = "yaw-light 1.2 kg", stl = CubeStl, density = 1000f,
            nodes = new List<Node>
            {
                new Node(0, 0.05f, 0.05f, 0f, 0.05f), new Node(0, -0.05f, 0.05f, 0f, 0.05f),
                new Node(0, 0.05f, -0.05f, 0f, 0.05f), new Node(0, -0.05f, -0.05f, 0f, 0.05f),
            },
        };

        private static Frame Small() => new Frame
        {
            name = "small 0.28 kg", stl = CubeStl, density = 100f,
            nodes = new List<Node>
            {
                new Node(1, 0f, 0f, 0f, 0.1f),
                new Node(0, 0.08f, 0.08f, 0f, 0.02f), new Node(0, -0.08f, 0.08f, 0f, 0.02f),
                new Node(0, 0.08f, -0.08f, 0f, 0.02f), new Node(0, -0.08f, -0.08f, 0f, 0.02f),
            },
        };

        [Serializable] private class NodeJson { public int type; public double x, y, z, mass_kg; }
        [Serializable] private class FixtureJson { public double density_kg_m3; public NodeJson[] nodes; }

        private static Frame SevenInch()
        {
            var f = JsonUtility.FromJson<FixtureJson>(File.ReadAllText(Path.Combine(FixtureDir, "drone_7inch_nodes.json")));
            var frame = new Frame { name = "7-inch", stl = Path.Combine(FixtureDir, "drone_7inch.stl"), density = (float)f.density_kg_m3, nodes = new List<Node>() };
            foreach (NodeJson n in f.nodes)
                frame.nodes.Add(new Node(n.type, (float)n.x, (float)n.y, (float)n.z, (float)n.mass_kg));
            return frame;
        }

        private Component drone, flight, crash;
        private Rigidbody rb;
        private readonly List<string> errors = new List<string>();
        private string savedPath; private float savedDensity; private readonly List<object> savedNodes = new List<object>();

        #region Tests

        [UnityTest]
        public IEnumerator MotorLag_StepReaches63PercentInTau()
        {
            yield return null;
            Type flightType = GameType("Flight");
            MethodInfo step = flightType.GetMethod("MotorLagStep", BindingFlags.Public | BindingFlags.Static);
            Assert.IsNotNull(step, "Flight.MotorLagStep not found");
            float tauUp = Cfg("Sim", "motorTimeConstantUp"), tauDown = Cfg("Sim", "motorTimeConstantDown");

            float Cross(float from, float to, float level)
            {
                const float dt = 1e-5f;
                float t = from, time = 0f;
                while (time < 1f)
                {
                    float next = (float)step.Invoke(null, new object[] { t, to, dt, tauUp, tauDown });
                    time += dt;
                    if ((to > from && next >= level) || (to < from && next <= level))
                        return time;
                    t = next;
                }
                return float.NaN;
            }

            float up = Cross(0f, 1f, 1f - Mathf.Exp(-1f));
            float down = Cross(1f, 0f, Mathf.Exp(-1f));
            Report($"spin-up 63% at {up * 1000:F2} ms (tau_up {tauUp * 1000:F2} ms); spin-down 63% at {down * 1000:F2} ms (tau_down {tauDown * 1000:F2} ms)");
            Assert.AreEqual(tauUp, up, tauUp * 0.1f, "spin-up time constant");
            Assert.AreEqual(tauDown, down, tauDown * 0.1f, "spin-down time constant");

            // Same answer in the sub-step size Flight uses.
            float sub = Time.fixedDeltaTime / Cfg("Sim", "subSteps");
            float th = 0f;
            int steps = Mathf.RoundToInt(tauUp / sub);
            for (int i = 0; i < steps; i++)
                th = (float)step.Invoke(null, new object[] { th, 1f, sub, tauUp, tauDown });
            Report($"after {steps} sub-steps of {sub * 1000:F1} ms: {th:P1}");
            Assert.AreEqual(1f - Mathf.Exp(-steps * sub / tauUp), th, 1e-4f);
        }

        [UnityTest]
        public IEnumerator Hover_ThrottleIsThrHover_DefaultDrone() { yield return HoverCheck(DefaultDrone()); }

        [UnityTest]
        public IEnumerator Hover_ThrottleIsThrHover_Cube() { yield return HoverCheck(Cube(0f)); }

        [UnityTest]
        public IEnumerator Hover_ThrottleIsThrHover_SevenInch() { yield return HoverCheck(SevenInch()); }

        [UnityTest]
        public IEnumerator OffCentreCom_MotorsShareByLeverRule()
        {
            yield return Fly(Cube(0.1f));
            yield return FastSteps(150);

            Vector3[] pos = (Vector3[])Get(flight, "MotorPositions");
            Vector3 com = rb.centerOfMass;
            float weight = rb.mass * Physics.gravity.magnitude;

            float[] sum = new float[4];
            float maxTilt = 0f;
            const int n = 250;
            for (int f = 0; f < n; f++)
            {
                yield return Steps(1);
                float[] t = (float[])Get(flight, "MotorThrusts");
                for (int i = 0; i < 4; i++) sum[i] += t[i];
                maxTilt = Mathf.Max(maxTilt, Vector3.Angle(drone.transform.up, Vector3.up));
            }

            // Lever rule: right-pair share = (COM-to-left distance) / (left-to-right distance).
            float xR = (pos[0].x + pos[3].x) / 2f - com.x, xL = com.x - (pos[1].x + pos[2].x) / 2f;
            float rightShare = xL / (xL + xR);
            float rightMeasured = (sum[0] + sum[3]) / (sum[0] + sum[1] + sum[2] + sum[3]);
            Report($"COM x {com.x:F5} m; thrust FR {sum[0] / n:F4} FL {sum[1] / n:F4} BL {sum[2] / n:F4} BR {sum[3] / n:F4} N (weight {weight:F4} N)");
            Report($"right-pair share expected {rightShare:P2} measured {rightMeasured:P2}; max tilt {maxTilt:F3} deg");

            Assert.Greater(Mathf.Abs(com.x), 0.02f, "fixture assumption: the battery moved the COM sideways");
            Assert.AreEqual(rightShare, rightMeasured, 0.01f, "motor thrusts do not follow the lever rule");
            Assert.Less(maxTilt, 1f, "off-centre COM frame did not hover level");
            Assert.IsEmpty(errors, string.Join("\n", errors));
        }

        [UnityTest]
        public IEnumerator Yaw_RightCommandYawsRight()
        {
            yield return Fly(DefaultDrone());
            // The stock drone's yaw inertia is small, so it reaches the rate setpoint within a
            // step or two and the commands then oscillate around it; sum the first 5 steps.
            float[] first = new float[4];
            for (int f = 0; f < 40; f++)
            {
                Set(flight, "YawSpeed", 1f);
                yield return Steps(1);
                if (f < 5)
                {
                    float[] u = (float[])Get(flight, "MotorCommands");
                    for (int i = 0; i < 4; i++) first[i] += u[i] / 5f;
                }
            }
            float yawRate = drone.transform.InverseTransformDirection(rb.angularVelocity).y;
            Report($"yaw 1.0: rate {yawRate * Mathf.Rad2Deg:F1} deg/s after 0.8 s; mean commands over steps 1-5: {Fmt(first)}");
            Assert.Greater(yawRate, 0.5f, "Yaw right must turn clockwise seen from above (+Y in Unity)");
            // A right yaw needs the counter-clockwise props (FR, BL) to push harder.
            Assert.Greater(first[0] + first[2], first[1] + first[3], "FR+BL should lead a right yaw");
            Set(flight, "YawSpeed", 0f);
        }

        [UnityTest]
        public IEnumerator Pitch_ForwardCommandTiltsAndMovesForward()
        {
            yield return Fly(DefaultDrone());
            float[] first = null;
            Vector3 start = drone.transform.position;
            Vector3 fwd = Flat(drone.transform.forward);
            for (int f = 0; f < 50; f++)
            {
                Set(flight, "ForwardSpeed", 1f);
                yield return Steps(1);
                if (f == 1) first = (float[])((float[])Get(flight, "MotorCommands")).Clone();
            }
            float pitch = Normalize(drone.transform.eulerAngles.x);
            float moved = Vector3.Dot(drone.transform.position - start, fwd);
            Report($"forward 1.0: pitch {pitch:F1} deg (nose down +), moved {moved:F3} m forward in 1 s; commands at step 2: {Fmt(first)}");
            Assert.Greater(first[2] + first[3], first[0] + first[1], "back motors (BL+BR) should lead a nose-down pitch");
            Assert.Greater(pitch, 2f, "nose should go down");
            Assert.Greater(moved, 0.05f, "drone should move forward");
            Set(flight, "ForwardSpeed", 0f);
        }

        [UnityTest]
        public IEnumerator Roll_RightCommandTiltsAndMovesRight()
        {
            yield return Fly(DefaultDrone());
            float[] first = null;
            Vector3 start = drone.transform.position;
            Vector3 right = Flat(drone.transform.right);
            for (int f = 0; f < 50; f++)
            {
                Set(flight, "StrafeSpeed", 1f);
                yield return Steps(1);
                if (f == 1) first = (float[])((float[])Get(flight, "MotorCommands")).Clone();
            }
            float roll = Normalize(drone.transform.eulerAngles.z);
            float moved = Vector3.Dot(drone.transform.position - start, right);
            Report($"strafe 1.0: roll {roll:F1} deg (right is -), moved {moved:F3} m right in 1 s; commands at step 2: {Fmt(first)}");
            Assert.Greater(first[1] + first[2], first[0] + first[3], "left motors (FL+BL) should lead a right roll");
            Assert.Less(roll, -2f, "drone should bank right");
            Assert.Greater(moved, 0.05f, "drone should move right");
            Set(flight, "StrafeSpeed", 0f);
        }

        [UnityTest]
        public IEnumerator Crash_DetachedArmLosesThatCornersMotor()
        {
            yield return Fly(DefaultDrone());
            yield return FastSteps(50);

            // Find CrashSystem's own index for the front-left arm (its Propeller_N order).
            var arms = (Transform[])Get(crash, "propellerTransforms");
            int flArm = -1;
            for (int i = 0; i < arms.Length; i++)
            {
                Vector3 lp = drone.transform.InverseTransformPoint(arms[i].position);
                if (lp.z >= 0 && lp.x < 0) flArm = i;
            }
            Assert.GreaterOrEqual(flArm, 0, "no front-left propeller");
            Vector3 flLocal = ((Vector3[])Get(flight, "MotorPositions"))[1];
            Vector3 brLocal = ((Vector3[])Get(flight, "MotorPositions"))[3];
            float before = ((float[])Get(flight, "MotorThrusts"))[1];

            Call(crash, "DetachArmGroup", flArm, arms[flArm].position);
            float[] health = (float[])Get(crash, "MotorHealth");
            Report($"detached CrashSystem arm {flArm} (front-left); MotorHealth FR {health[0]} FL {health[1]} BL {health[2]} BR {health[3]}");
            Assert.AreEqual(0f, health[1], "front-left motor (index 1) should be disabled");
            Assert.AreEqual(3f, health[0] + health[2] + health[3], "only the front-left motor should be disabled");

            yield return Steps(1);
            float afterOne = ((float[])Get(flight, "MotorThrusts"))[1];
            yield return Steps(14);
            float[] t = (float[])Get(flight, "MotorThrusts");
            float flY = drone.transform.TransformPoint(flLocal).y, brY = drone.transform.TransformPoint(brLocal).y;
            Report($"FL thrust {before:F3} N -> {afterOne:F3} N after 1 step (lag) -> {t[1]:F4} N after 0.3 s; " +
                   $"thrusts FR {t[0]:F3} BL {t[2]:F3} BR {t[3]:F3} N; FL corner {flY - brY:F3} m relative to BR");
            Assert.Greater(afterOne, 0.2f * before, "thrust dropped in one step: no spin-down lag");
            Assert.Less(t[1], 0.01f * before + 1e-4f, "front-left motor still produces thrust");
            // How the controller then copes (here the other three saturate) is reported, not asserted.
        }

        #endregion

        #region Hover

        /// <summary>
        /// Spec task 0.5 baseline: hover trace for each frame. Reads only positions, attitude and
        /// MotorSpeeds, so it runs unchanged on the old and the new Flight.cs. Reports; asserts only
        /// that the drone is still in the air.
        /// </summary>
        [UnityTest, Category("Baseline")]
        public IEnumerator Baseline_HoverTrace([Values("default", "cube", "7inch", "yawlight", "small")] string which)
        {
            Frame frame = which == "default" ? DefaultDrone() : which == "cube" ? Cube(0f) : which == "7inch" ? SevenInch()
                        : which == "yawlight" ? YawLight() : Small();
            yield return Fly(frame);
            float target = (float)Get(flight, "TargetAltitude");
            Vector3 start = drone.transform.position;
            float maxAlt = 0f, maxDrift = 0f, maxTilt = 0f;
            var trace = new System.Text.StringBuilder();
            for (int f = 0; f < 1000; f++)
            {
                yield return Steps(1);
                Vector3 p = drone.transform.position;
                maxAlt = Mathf.Max(maxAlt, Mathf.Abs(p.y - target));
                maxDrift = Mathf.Max(maxDrift, new Vector2(p.x - start.x, p.z - start.z).magnitude);
                maxTilt = Mathf.Max(maxTilt, Vector3.Angle(drone.transform.up, Vector3.up));
                if (f % 250 == 0)
                    trace.Append($" t={f * Time.fixedDeltaTime:F0}s alt err {(p.y - target) * 1000:F2} mm motors {Fmt((float[])Get(flight, "MotorSpeeds"))};");
            }
            Report($"BASELINE {frame.name}: mass {rb.mass:F4} kg; 20 s hover: max altitude error {maxAlt * 1000:F2} mm, drift {maxDrift * 1000:F2} mm, max tilt {maxTilt:F3} deg;{trace}");
            Assert.Less(maxAlt, 1f, "drone left its hover altitude");
        }

        private IEnumerator HoverCheck(Frame frame)
        {
            yield return Fly(frame);
            yield return FastSteps(150); // settle 3 s

            float target = (float)Get(flight, "TargetAltitude");
            Vector3 start = drone.transform.position;
            double sumU = 0; int samples = 0;
            float maxAlt = 0f, maxDrift = 0f, maxTilt = 0f;
            for (int f = 0; f < 1000; f++) // 20 s
            {
                yield return Steps(1);
                float[] u = (float[])Get(flight, "MotorCommands");
                sumU += (u[0] + u[1] + u[2] + u[3]) / 4.0;
                samples++;
                Vector3 p = drone.transform.position;
                maxAlt = Mathf.Max(maxAlt, Mathf.Abs(p.y - target));
                maxDrift = Mathf.Max(maxDrift, new Vector2(p.x - start.x, p.z - start.z).magnitude);
                maxTilt = Mathf.Max(maxTilt, Vector3.Angle(drone.transform.up, Vector3.up));
            }
            float meanU = (float)(sumU / samples);
            float[] th = (float[])Get(flight, "MotorThrusts");
            Report($"{frame.name}: mass {rb.mass:F4} kg, Tmax {(float)Get(flight, "MotorThrustMax"):F3} N; mean u {meanU:F4}; " +
                   $"max altitude error {maxAlt * 100:F2} cm; drift {maxDrift * 100:F2} cm; max tilt {maxTilt:F3} deg; thrusts {Fmt(th)} N");
            Assert.AreEqual(Cfg("Px4", "MPC_THR_HOVER"), meanU, 0.02f, "hover throttle");
            Assert.Less(maxAlt, 0.02f, "altitude error");
            Assert.Less(maxDrift, 0.05f, "horizontal drift");
            Assert.IsEmpty(errors, string.Join("\n", errors));
        }

        #endregion

        #region Flight harness

        [TearDown]
        public void Restore()
        {
            Time.captureDeltaTime = 0f;
            Time.timeScale = 1f;
            Application.logMessageReceived -= OnLog;
            if (savedPath == null) return;
            Type loader = GameType("CustomFrameLoader");
            SetStatic(loader, "SelectedSTLPath", savedPath);
            SetStatic(loader, "SelectedDensity", savedDensity);
            IList nodes = (IList)GetStatic(loader, "SelectedNodes");
            nodes.Clear();
            foreach (object n in savedNodes) nodes.Add(n);
            savedPath = null;
            savedNodes.Clear();
        }

        private void OnLog(string message, string stack, LogType type)
        {
            if ((type == LogType.Error || type == LogType.Exception || type == LogType.Assert) && !IsKnownNoise(message))
                errors.Add($"{type}: {message}");
        }

        /// <summary>Loads the frame, arms, waits for the launch to finish. One physics step per frame afterwards.
        /// With arm false it stops after loading, with the drone disarmed.</summary>
        private IEnumerator Fly(Frame frame, bool arm = true)
        {
            if (!frame.IsDefault)
            {
                Assert.IsTrue(File.Exists(frame.stl), $"Fixture missing: {frame.stl}");
                yield return EnsureService();
            }

            errors.Clear();
            // Errors are collected (minus known noise) and asserted by each test instead.
            LogAssert.ignoreFailingMessages = true;
            Application.logMessageReceived += OnLog;
            SceneManager.LoadScene(0);
            yield return null;
            yield return null;

            Type loader = GameType("CustomFrameLoader");
            savedPath = (string)GetStatic(loader, "SelectedSTLPath") ?? "";
            savedDensity = (float)GetStatic(loader, "SelectedDensity");
            IList nodes = (IList)GetStatic(loader, "SelectedNodes");
            savedNodes.Clear();
            foreach (object n in nodes) savedNodes.Add(n);

            nodes.Clear();
            SetStatic(loader, "SelectedSTLPath", frame.IsDefault ? "" : frame.stl);
            SetStatic(loader, "SelectedDensity", frame.density);
            if (!frame.IsDefault)
            {
                Type dataType = loader.GetNestedType("NodeData"), typeType = loader.GetNestedType("NodeType");
                foreach (Node n in frame.nodes)
                {
                    object d = Activator.CreateInstance(dataType);
                    dataType.GetField("type").SetValue(d, Enum.ToObject(typeType, (byte)n.type));
                    dataType.GetField("position").SetValue(d, n.cad);
                    dataType.GetField("massKg").SetValue(d, n.kg);
                    nodes.Add(d);
                }
            }

            UnityEngine.Object menu = UnityEngine.Object.FindAnyObjectByType(GameType("MainMenu"));
            Assert.IsNotNull(menu, "MainMenu not found");
            Call(menu, "BeginSimulation");

            float deadline = Time.realtimeSinceStartup + 60f;
            drone = null;
            while (drone == null && Time.realtimeSinceStartup < deadline)
            {
                yield return null;
                drone = (Component)UnityEngine.Object.FindAnyObjectByType(GameType("Drone"));
            }
            Assert.IsNotNull(drone, "No Drone after BeginSimulation");
            for (int i = 0; i < 5; i++) yield return null;

            flight = drone.GetComponent(GameType("Flight"));
            crash = drone.GetComponent(GameType("CrashSystem"));
            rb = drone.GetComponent<Rigidbody>();
            Assert.IsNotNull(flight);
            if (!frame.IsDefault)
                Assert.IsFalse(errors.Exists(e => e.Contains("Using the default drone")), $"{frame.name} import fell back:\n" + string.Join("\n", errors));
            if (!arm) yield break;

            Call(flight, "Arm");
            yield return Steps(1);
            for (int f = 0; f < 300 && (bool)Get(flight, "IsLaunching"); f++)
                yield return FastSteps(10);
            Time.timeScale = 1f;
            Assert.IsFalse((bool)Get(flight, "IsLaunching"), "Launch never finished");
            // Flight-model tests drive the send_pcmd path unless they choose PosCtl.
            SetMode("Pcmd");
        }

        private void SetMode(string mode) =>
            Set(flight, "Mode", Enum.Parse(GameType("Flight").GetNestedType("ControlMode"), mode));

        /// <summary>n physics steps at one step per frame (for sampling every step).</summary>
        private static IEnumerator Steps(int n)
        {
            Time.timeScale = 1f;
            Time.captureDeltaTime = Time.fixedDeltaTime;
            for (int i = 0; i < n; i++) yield return null;
        }

        /// <summary>About n physics steps at 10 steps per frame (deterministic, for waiting only).</summary>
        private static IEnumerator FastSteps(int n)
        {
            Time.captureDeltaTime = Time.fixedDeltaTime;
            Time.timeScale = 10f;
            for (int i = 0; i < n; i += 10) yield return null;
            Time.timeScale = 1f;
        }

        // Known noise, not from the flight code: reloading Main re-binds PythonInterface's port, and the
        // Unity AI Assistant package logs its own connection errors.
        private static bool IsKnownNoise(string message) =>
            message.Contains("Only one usage of each socket address") || message.Contains("connection.state_change");

        private static IEnumerator EnsureService()
        {
            if (ServiceResponds()) yield break;
            Type launcher = Type.GetType("MassServiceLauncher, Assembly-CSharp");
            launcher?.GetMethod("EnsureRunning", BindingFlags.Public | BindingFlags.Static)?.Invoke(null, null);
            float deadline = Time.realtimeSinceStartup + 20f;
            while (Time.realtimeSinceStartup < deadline)
            {
                yield return null;
                if (ServiceResponds()) yield break;
            }
            Assert.Ignore($"Mass service not answering on 127.0.0.1:{ServicePort}.");
        }

        private static bool ServiceResponds()
        {
            try
            {
                using (var udp = new UdpClient())
                {
                    udp.Client.ReceiveTimeout = 300;
                    udp.Connect(IPAddress.Loopback, ServicePort);
                    udp.Send(new byte[] { 0 }, 1);
                    IPEndPoint from = null;
                    return udp.Receive(ref from).Length > 0;
                }
            }
            catch (SocketException) { return false; }
        }

        #endregion

        #region Helpers

        private static float Cfg(string kind, string key) =>
            (float)GameType("FlightConfig").GetMethod(kind, BindingFlags.Public | BindingFlags.Static).Invoke(null, new object[] { key });

        private static Vector3 Flat(Vector3 v) { v.y = 0f; return v.normalized; }
        private static float Normalize(float a) => a > 180f ? a - 360f : a;
        private static string Fmt(float[] a) => a == null ? "null" : $"FR {a[0]:F3} FL {a[1]:F3} BL {a[2]:F3} BR {a[3]:F3}";

        private const BindingFlags AnyMember = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static | BindingFlags.FlattenHierarchy;

        private static Type GameType(string name)
        {
            Type t = Type.GetType(name + ", Assembly-CSharp");
            Assert.IsNotNull(t, $"Game type {name} not found in Assembly-CSharp");
            return t;
        }

        private static object Get(object target, string member) => GetMember(target.GetType(), target, member);
        private static object GetStatic(Type type, string member) => GetMember(type, null, member);
        private static void Set(object target, string member, object value) => SetMember(target.GetType(), target, member, value);
        private static void SetStatic(Type type, string member, object value) => SetMember(type, null, member, value);

        private static object GetMember(Type type, object target, string member)
        {
            PropertyInfo p = type.GetProperty(member, AnyMember);
            if (p != null) return p.GetValue(target);
            FieldInfo f = type.GetField(member, AnyMember);
            Assert.IsNotNull(f, $"{type.Name}.{member} not found");
            return f.GetValue(target);
        }

        private static void SetMember(Type type, object target, string member, object value)
        {
            PropertyInfo p = type.GetProperty(member, AnyMember);
            if (p != null) { p.SetValue(target, value); return; }
            FieldInfo f = type.GetField(member, AnyMember);
            Assert.IsNotNull(f, $"{type.Name}.{member} not found");
            f.SetValue(target, value);
        }

        private static object Call(object target, string method, params object[] args)
        {
            var types = new Type[args.Length];
            for (int i = 0; i < args.Length; i++) types[i] = args[i].GetType();
            MethodInfo m = target.GetType().GetMethod(method, AnyMember, null, types, null);
            Assert.IsNotNull(m, $"{target.GetType().Name}.{method} not found");
            return m.Invoke(target, args);
        }

        private static void Report(string line)
        {
            TestContext.WriteLine(line);
            Debug.Log("[FlightMotorTests] " + line);
        }

        #endregion
    }
}

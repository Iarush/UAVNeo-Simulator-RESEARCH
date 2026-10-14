using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using NUnit.Framework;
using Unity.Collections;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using UnityEngine.UI;

namespace UAVSim.Tests.PlayMode
{
    /// <summary>
    /// End-to-end PlayMode check of the custom-frame import with a 7-inch quad fixture
    /// (Fixtures~/drone_7inch.stl + drone_7inch_nodes.json, made by Fixtures~/make_drone_7inch.py).
    ///
    /// Goes through the real route: fill the CustomFrameLoader statics, load Main, call
    /// MainMenu.BeginSimulation, then check load errors, mass properties (against this file's own
    /// STL integration), the zoom slider, propeller placement/size and propeller spin.
    ///
    /// Needs the mass-properties service (udp_listener.py) on 127.0.0.1:5070; without it every
    /// test is reported Ignored. Game code lives in Assembly-CSharp, which an asmdef cannot
    /// reference, so game types are reached by name through the small reflection helpers below.
    /// </summary>
    public class Drone7InchTests : IPrebuildSetup
    {
        private const int ServicePort = 5070;
        private const double PropDiameterMm = 7 * 25.4;

        // CAD Z-up -> Unity Y-up as documented in CLAUDE.md (StlMeshLoader.ZUpToYUpRotation,
        // AngleAxis(90, X)): (x, y, z) -> (x, -z, y).
        private static readonly double[,] CadToUnity = { { 1, 0, 0 }, { 0, 0, -1 }, { 0, 1, 0 } };

        [Serializable]
        private class NodeJson
        {
            public string name;
            public int type;
            public double x, y, z;
            public double mass_kg;
        }

        [Serializable]
        private class FixtureJson
        {
            public double density_kg_m3;
            public NodeJson[] nodes;
        }

        private static string FixtureDir => Path.Combine(Application.dataPath, "Tests", "PlayMode", "Fixtures~");
        private static string StlPath => Path.Combine(FixtureDir, "drone_7inch.stl");

        private FixtureJson fixture;
        private bool loaded;
        private readonly List<string> logErrors = new List<string>();
        private readonly List<string> logLines = new List<string>();

        private string savedStlPath;
        private float savedDensity;
        private readonly List<object> savedNodes = new List<object>();

        private Transform root;
        private Component drone, flight, frameLoader, zoom;
        private Transform[] props;
        private Dictionary<Transform, int> propQuadrant; // Flight.MotorSpeeds index: FR 0, FL 1, BL 2, BR 3
        private Dictionary<Transform, Vector3> propLocalPos;
        private float visualSpeedScale;

        #region Setup

        [OneTimeSetUp]
        public void CheckServiceAndFixtures()
        {
            Assert.IsTrue(File.Exists(StlPath), $"Fixture missing: {StlPath}. Run Fixtures~/make_drone_7inch.py.");
            fixture = JsonUtility.FromJson<FixtureJson>(File.ReadAllText(Path.Combine(FixtureDir, "drone_7inch_nodes.json")));
            Assert.IsNotNull(fixture?.nodes);

            if (!ServiceResponds())
                Assert.Ignore($"Mass-properties service not answering on 127.0.0.1:{ServicePort}. " +
                              "Start udp_listener.py (MassCompose/src/mass-properties) and run again.");
        }

        [UnitySetUp]
        public IEnumerator LoadSimulationOnce()
        {
            if (loaded)
                yield break;
            loaded = true;

            Application.logMessageReceived += OnLog;

            SceneManager.LoadScene(0); // Main.unity
            yield return null;
            yield return null;

            // Fill the statics the import UI would, then use the real Begin button handler.
            Type loaderType = GameType("CustomFrameLoader");
            savedStlPath = (string)GetStatic(loaderType, "SelectedSTLPath");
            savedDensity = (float)GetStatic(loaderType, "SelectedDensity");
            IList nodes = (IList)GetStatic(loaderType, "SelectedNodes");
            foreach (object n in nodes)
                savedNodes.Add(n);

            SetStatic(loaderType, "SelectedSTLPath", StlPath);
            SetStatic(loaderType, "SelectedDensity", (float)fixture.density_kg_m3);
            nodes.Clear();
            Type nodeDataType = loaderType.GetNestedType("NodeData");
            Type nodeTypeType = loaderType.GetNestedType("NodeType");
            foreach (NodeJson n in fixture.nodes)
            {
                object data = Activator.CreateInstance(nodeDataType);
                nodeDataType.GetField("type").SetValue(data, Enum.ToObject(nodeTypeType, (byte)n.type));
                nodeDataType.GetField("position").SetValue(data, new Vector3((float)n.x, (float)n.y, (float)n.z));
                nodeDataType.GetField("massKg").SetValue(data, (float)n.mass_kg);
                nodes.Add(data);
            }

            UnityEngine.Object menu = UnityEngine.Object.FindAnyObjectByType(GameType("MainMenu"));
            Assert.IsNotNull(menu, "MainMenu not found in Main.unity");
            Call(menu, "BeginSimulation");

            float deadline = Time.realtimeSinceStartup + 60f;
            while (drone == null && Time.realtimeSinceStartup < deadline)
            {
                yield return null;
                drone = (Component)UnityEngine.Object.FindAnyObjectByType(GameType("Drone"));
            }
            Assert.IsNotNull(drone, "No Drone appeared after BeginSimulation");

            // Let Start() run on everything (Flight writes the Rigidbody, DroneCameraZoom auto-fits).
            for (int i = 0; i < 5; i++)
                yield return null;

            root = drone.transform;
            flight = drone.GetComponent(GameType("Flight"));
            frameLoader = drone.GetComponent(GameType("CustomFrameLoader"));
            zoom = drone.GetComponent(GameType("DroneCameraZoom"));
            Component spin = drone.GetComponent(GameType("PropellerSpin"));
            Assert.IsNotNull(flight);
            Assert.IsNotNull(frameLoader);
            Assert.IsNotNull(zoom);
            Assert.IsNotNull(spin);
            visualSpeedScale = (float)Get(spin, "visualSpeedScale");

            var found = new List<Transform>();
            foreach (Transform t in root.GetComponentsInChildren<Transform>(true))
                if (t.name.StartsWith("Propeller_"))
                    found.Add(t);
            props = found.ToArray();
            propQuadrant = new Dictionary<Transform, int>();
            propLocalPos = new Dictionary<Transform, Vector3>();
            foreach (Transform p in props)
            {
                Vector3 lp = root.InverseTransformPoint(p.position);
                bool front = lp.z >= 0, left = lp.x < 0;
                propQuadrant[p] = front ? (left ? 1 : 0) : (left ? 2 : 3);
                propLocalPos[p] = lp;
            }
        }

        [OneTimeTearDown]
        public void Restore()
        {
            Time.captureDeltaTime = 0f;
            Application.logMessageReceived -= OnLog;
            if (savedStlPath == null)
                return;
            Type loaderType = GameType("CustomFrameLoader");
            SetStatic(loaderType, "SelectedSTLPath", savedStlPath);
            SetStatic(loaderType, "SelectedDensity", savedDensity);
            IList nodes = (IList)GetStatic(loaderType, "SelectedNodes");
            nodes.Clear();
            foreach (object n in savedNodes)
                nodes.Add(n);
        }

        private void OnLog(string message, string stackTrace, LogType type)
        {
            logLines.Add(message);
            if (type == LogType.Error || type == LogType.Exception || type == LogType.Assert)
                logErrors.Add($"{type}: {message}");
        }

        #endregion

        #region Tests

        [UnityTest, Order(1)]
        public IEnumerator Load_NoErrors_CustomFrameShown()
        {
            yield return null;
            Assert.IsEmpty(logErrors, "Errors while loading:\n" + string.Join("\n", logErrors));
            Assert.IsTrue(logLines.Exists(l => l.StartsWith("[CustomFrameLoader] Total mass")),
                "CustomFrameLoader never reached the mass step (import fell back to the default drone?)");

            object bounds = Get(frameLoader, "CustomFrameBounds");
            Assert.IsNotNull(bounds, "CustomFrameBounds is null: the STL mesh did not land on the visual");

            var visual = (MeshFilter)Get(frameLoader, "customFrameVisual");
            Assert.IsNotNull(visual, "customFrameVisual is not assigned on the prefab");
            Assert.IsTrue(visual.gameObject.activeInHierarchy, "CustomFrameVisual is inactive");
            Assert.IsNotNull(visual.sharedMesh);
            Assert.Greater(visual.sharedMesh.vertexCount, 0);

            var stock = (GameObject[])Get(frameLoader, "defaultBodyObjects");
            Assert.IsNotNull(stock);
            Assert.IsNotEmpty(stock, "defaultBodyObjects is empty, so the stock body would overlap the frame");
            foreach (GameObject g in stock)
                Assert.IsFalse(g.activeInHierarchy, $"Stock body part '{g.name}' is still visible");

            Assert.AreEqual(4, props.Length, "Expected 4 Propeller_N objects");
            foreach (Transform p in props)
                Assert.IsTrue(p.gameObject.activeInHierarchy, $"{p.name} is hidden");
        }

        [UnityTest, Order(2)]
        public IEnumerator MassProperties_MatchIndependentCalculation()
        {
            yield return null;

            // Independent: own STL read + signed-tetrahedron integration + parallel axis (CAD frame, metres).
            List<double[]> tris = ReadStlMillimetres(StlPath);
            SolidProperties(tris, fixture.density_kg_m3, out double meshMass, out double[] meshCom, out double[,] meshI);
            double nodeMass = 0;
            foreach (NodeJson n in fixture.nodes)
                nodeMass += n.mass_kg;
            double total = meshMass + nodeMass;

            double[] com = new double[3];
            for (int k = 0; k < 3; k++)
                com[k] = meshCom[k] * meshMass;
            foreach (NodeJson n in fixture.nodes)
            {
                com[0] += n.mass_kg * n.x;
                com[1] += n.mass_kg * n.y;
                com[2] += n.mass_kg * n.z;
            }
            for (int k = 0; k < 3; k++)
                com[k] /= total;

            double[,] inertia = (double[,])meshI.Clone();
            AddPointInertia(inertia, meshMass, meshCom, com);
            double armSum = 0;
            int motors = 0;
            foreach (NodeJson n in fixture.nodes)
            {
                double[] p = { n.x, n.y, n.z };
                AddPointInertia(inertia, n.mass_kg, p, com);
                if (n.type == 0)
                {
                    armSum += Math.Sqrt(Sq(p[0] - com[0]) + Sq(p[1] - com[1]) + Sq(p[2] - com[2]));
                    motors++;
                }
            }
            double arm = armSum / motors;

            // Expected values in Unity's drone-root frame.
            double[] comU = MatVec(CadToUnity, com);
            double[,] inertiaU = MatMul(MatMul(CadToUnity, inertia), Transpose(CadToUnity));

            // Actual values from the Rigidbody.
            Rigidbody rb = root.GetComponent<Rigidbody>();
            Assert.IsNotNull(rb, "No Rigidbody on the drone root");
            Matrix4x4 q = Matrix4x4.Rotate(rb.inertiaTensorRotation);
            Vector3 d = rb.inertiaTensor;
            double[,] actualI = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    actualI[i, j] = q[i, 0] * d.x * q[j, 0] + q[i, 1] * d.y * q[j, 1] + q[i, 2] * d.z * q[j, 2];
            Vector3 c = rb.centerOfMass;
            double actualArm = (float)Get(flight, "armLength");

            double massErr = Math.Abs(rb.mass - total) / total;
            double comErr = Math.Max(Math.Abs(c.x - comU[0]), Math.Max(Math.Abs(c.y - comU[1]), Math.Abs(c.z - comU[2])));
            double tensorErr = MaxAbsDiff(actualI, inertiaU) / MaxAbs(inertiaU);
            double armErr = Math.Abs(actualArm - arm);

            Report($"mesh {meshMass * 1000:F2} g + nodes {nodeMass * 1000:F2} g = {total:F5} kg; rb.mass {rb.mass:F5} (rel err {massErr:E2})");
            Report($"COM expected ({comU[0]:F6}, {comU[1]:F6}, {comU[2]:F6}) actual ({c.x:F6}, {c.y:F6}, {c.z:F6}) (max abs err {comErr:E2} m)");
            Report($"inertia tensor rel err {tensorErr:E2}; armLength expected {arm:F5} actual {actualArm:F5}");

            Assert.Less(massErr, 1e-4, "rb.mass != mesh mass + node masses");
            Assert.Less(comErr, 1e-5, "centre of mass mismatch");
            Assert.Less(tensorErr, 1e-4, "inertia tensor mismatch");
            Assert.Less(armErr, 1e-4, "armLength mismatch");
        }

        [UnityTest, Order(3)]
        public IEnumerator ZoomSlider_UpZoomsIn_Smooth_Clamped_DroneVisible()
        {
            Assert.IsFalse((bool)Get(flight, "IsArmed"), "Zoom slider is only shown before launch");
            var panel = (GameObject)Get(zoom, "zoomPanel");
            Assert.IsNotNull(panel, "zoomPanel not assigned");
            Assert.IsTrue(panel.activeInHierarchy, "Zoom panel hidden before launch");
            var slider = (Slider)Get(zoom, "zoomSlider");
            Assert.IsNotNull(slider, "No Slider under zoomPanel");
            Assert.AreEqual(0f, slider.minValue);
            Assert.AreEqual(1f, slider.maxValue);

            var cam = (Camera)Get(drone, "PrimaryCamera");
            Assert.IsNotNull(cam);
            var bounds = (Bounds)Get(frameLoader, "CustomFrameBounds");

            // Fixed 1/20 s frames: the chase camera lerps 40% of the way per frame, so 40 frames settle it.
            Time.captureDeltaTime = 1f / 20f;
            const int steps = 10;
            const int settleFrames = 40;
            var multipliers = new List<float>();
            var distances = new List<float>();

            for (int s = 0; s <= steps; s++)
            {
                float value = s / (float)steps;
                slider.value = value; // fires OnValueChanged -> DroneCameraZoom.OnZoomChanged, as a drag would
                multipliers.Add((float)Get(drone, "ZoomMultiplier"));

                float prev = CameraDistance(cam);
                float target = float.NaN;
                for (int f = 0; f < settleFrames; f++)
                {
                    yield return null;
                    float now = CameraDistance(cam);
                    if (s > 0)
                    {
                        // Camera must move steadily inward (slider went up), never back out.
                        Assert.LessOrEqual(now, prev + 1e-4f, $"Camera moved away while zooming in (step {s}, frame {f})");
                    }
                    prev = now;
                    target = now;
                }
                distances.Add(target);
                AssertDroneVisible(cam, bounds, $"slider {value:F1}");
            }

            for (int s = 1; s <= steps; s++)
            {
                Assert.Less(multipliers[s], multipliers[s - 1], $"Slider up must zoom in (step {s})");
                Assert.Less(distances[s], distances[s - 1], $"Camera distance must shrink as the slider goes up (step {s})");
            }
            // Linear mapping: every equal slider step changes the zoom by the same amount (no jumps).
            float firstStep = multipliers[0] - multipliers[1];
            for (int s = 1; s <= steps; s++)
                Assert.AreEqual(firstStep, multipliers[s - 1] - multipliers[s], firstStep * 1e-3f, $"Uneven zoom step {s}");

            // Limits: pushing past either end (as the wheel/keys do) holds the end value exactly.
            float atMax = multipliers[steps], atMin = multipliers[0];
            Call(zoom, "OnZoomChanged", 1.5f);
            Assert.AreEqual(atMax, (float)Get(drone, "ZoomMultiplier"), 1e-6f, "Zoom overshoots its inner limit");
            Call(zoom, "SetZoom", 2f);
            Assert.AreEqual(1f, slider.value, "Slider not clamped at 1");
            Call(zoom, "OnZoomChanged", -0.5f);
            Assert.AreEqual(atMin, (float)Get(drone, "ZoomMultiplier"), 1e-6f, "Zoom overshoots its outer limit");
            Call(zoom, "SetZoom", -1f);
            Assert.AreEqual(0f, slider.value, "Slider not clamped at 0");

            Report($"zoom multiplier {atMin:F3} (slider 0) -> {atMax:F3} (slider 1); camera distance {distances[0]:F3} m -> {distances[steps]:F3} m");

            // Back to the default framing for the remaining tests.
            Call(zoom, "SetZoom", (float)GetStatic(zoom.GetType(), "InitialSliderValue"));
            for (int f = 0; f < settleFrames; f++)
                yield return null;
            Time.captureDeltaTime = 0f;
        }

        [UnityTest, Order(4)]
        public IEnumerator Propellers_OnMotorMounts_SevenInch()
        {
            yield return null;
            var seen = new HashSet<int>();
            foreach (Transform p in props)
            {
                int quad = propQuadrant[p];
                Assert.IsTrue(seen.Add(quad), $"Two propellers in quadrant {QuadName(quad)}");

                NodeJson motor = MotorFor(quad);
                double[] u = MatVec(CadToUnity, new[] { motor.x, motor.y, motor.z });
                Vector3 expected = new Vector3((float)u[0], (float)u[1], (float)u[2]);
                float offsetMm = (propLocalPos[p] - expected).magnitude * 1000f;

                double diameterMm = PropDiameter(p) * 1000.0;
                Report($"{p.name} {QuadName(quad)}: {offsetMm:F3} mm from {motor.name}, diameter {diameterMm:F1} mm");
                Assert.Less(offsetMm, 0.5f, $"{p.name} is not on {motor.name}");
                Assert.AreEqual(PropDiameterMm, diameterMm, 5.0, $"{p.name} is not 7-inch");
            }
        }

        [UnityTest, Order(5)]
        public IEnumerator Propellers_SpinDirection_FrameRateIndependent()
        {
            // Motors off: propellers stay still.
            Time.captureDeltaTime = 1f / 60f;
            var prev = SnapshotLocalRotations();
            for (int f = 0; f < 10; f++)
            {
                yield return null;
                foreach (Transform p in props)
                    Assert.Less(Quaternion.Angle(prev[p], LocalRotation(p)), 0.01f, $"{p.name} spins while disarmed");
            }

            Call(flight, "Arm");
            yield return null;

            var stats = new SpinStats();
            Time.captureDeltaTime = 1f / 240f;
            yield return MeasureSpin(120, stats, checkSign: true);
            float residualFast = stats.MaxResidual;
            Time.captureDeltaTime = 1f / 30f;
            yield return MeasureSpin(60, stats, checkSign: false);
            Time.captureDeltaTime = 0f;

            Report($"spin: {stats.Frames} frames, dt {stats.MinDt:F4}-{stats.MaxDt:F4} s, max residual {residualFast:F3} deg at 1/240 s, " +
                   $"{stats.MaxResidual:F3} deg overall, wrong-direction frames {stats.WrongSign}, rate {stats.MinRate:F0}-{stats.MaxRate:F0} deg/s");
            Assert.AreEqual(0, stats.WrongSign, "A propeller turned the wrong way");
            Assert.Less(stats.MaxResidual, 0.5f, "Propeller rotation does not equal rate x deltaTime");

            foreach (Transform p in props)
                Assert.Less((root.InverseTransformPoint(p.position) - propLocalPos[p]).magnitude, 1e-5f, $"{p.name} drifted off its mount");
        }

        [UnityTest, Order(6)]
        public IEnumerator Propellers_FollowTheirOwnMotor_DuringYaw()
        {
            if (!(bool)Get(flight, "IsArmed"))
            {
                Call(flight, "Arm");
                yield return null;
            }

            Time.captureDeltaTime = 1f / 50f;
            for (int f = 0; f < 1500 && (bool)Get(flight, "IsLaunching"); f++)
                yield return null;
            Assert.IsFalse((bool)Get(flight, "IsLaunching"), "Launch never finished");

            // Yaw makes the diagonal motor pairs run at different speeds, so a prop driven by the
            // wrong motor shows up. Drone.DefaultFlightUpdate rewrites YawSpeed from input every
            // Update, so set it after Update; the next frame's FixedUpdate uses it.
            Time.captureDeltaTime = 1f / 240f;
            for (int f = 0; f < 60; f++)
            {
                Set(flight, "YawSpeed", 1f);
                yield return null;
            }

            var stats = new SpinStats();
            yield return MeasureSpin(60, stats, checkSign: true, yaw: 1f);
            Time.captureDeltaTime = 0f;
            Set(flight, "YawSpeed", 0f);

            var speeds = (float[])Get(flight, "MotorSpeeds");
            // Yaw torque comes from the diagonal pairs (FR+BL vs FL+BR) running at different speeds.
            float spread = (speeds[0] + speeds[2]) / (speeds[1] + speeds[3]);
            Report($"yaw: motors FR {speeds[0]:F1} FL {speeds[1]:F1} BL {speeds[2]:F1} BR {speeds[3]:F1} rad/s; " +
                   $"residual {stats.MaxResidual:F3} deg; with FL/FR swapped it would be {stats.MaxSwappedResidual:F3} deg");
            Assert.Greater(Mathf.Abs(spread - 1f), 0.3f, "Yaw did not split the motor speeds; test cannot tell motors apart");
            Assert.AreEqual(0, stats.WrongSign);
            Assert.Less(stats.MaxResidual, 0.5f, "A propeller is not following its own motor");
            Assert.Greater(stats.MaxSwappedResidual, 5f, "Test is not sensitive to the motor mapping");
        }

        #endregion

        #region Spin measurement

        private class SpinStats
        {
            public int Frames, WrongSign;
            public float MaxResidual, MaxSwappedResidual;
            public float MinDt = float.MaxValue, MaxDt;
            public float MinRate = float.MaxValue, MaxRate;
        }

        private IEnumerator MeasureSpin(int frames, SpinStats stats, bool checkSign, float yaw = float.NaN)
        {
            // One frame so the new captureDeltaTime is in effect before measuring.
            if (!float.IsNaN(yaw))
                Set(flight, "YawSpeed", yaw);
            yield return null;
            var prev = SnapshotLocalRotations();

            for (int f = 0; f < frames; f++)
            {
                if (!float.IsNaN(yaw))
                    Set(flight, "YawSpeed", yaw);
                yield return null;

                // Runs after this frame's Update, so MotorSpeeds/deltaTime are the ones PropellerSpin used.
                float dt = Time.deltaTime;
                var speeds = (float[])Get(flight, "MotorSpeeds");
                bool armed = (bool)Get(flight, "IsArmed");
                stats.Frames++;
                stats.MinDt = Mathf.Min(stats.MinDt, dt);
                stats.MaxDt = Mathf.Max(stats.MaxDt, dt);

                foreach (Transform p in props)
                {
                    int quad = propQuadrant[p];
                    Quaternion now = LocalRotation(p);
                    Quaternion delta = now * Quaternion.Inverse(prev[p]);
                    prev[p] = now;

                    Vector3 axis = Quaternion.Inverse(root.rotation) * p.up;
                    if (Vector3.Dot(axis, Vector3.up) < 0)
                        axis = -axis;

                    float rate = ExpectedRate(speeds[quad], armed);
                    // Seen from above, FL and BR turn clockwise, which is a positive angle about +Y in Unity.
                    float sign = quad == 1 || quad == 3 ? 1f : -1f;
                    float expectedDeg = rate * sign * dt;
                    stats.MinRate = Mathf.Min(stats.MinRate, rate);
                    stats.MaxRate = Mathf.Max(stats.MaxRate, rate);

                    float residual = Quaternion.Angle(delta, Quaternion.AngleAxis(expectedDeg, axis));
                    stats.MaxResidual = Mathf.Max(stats.MaxResidual, residual);

                    int swappedQuad = quad == 0 ? 1 : quad == 1 ? 0 : quad;
                    if (swappedQuad != quad)
                    {
                        float swappedDeg = ExpectedRate(speeds[swappedQuad], armed) * sign * dt;
                        stats.MaxSwappedResidual = Mathf.Max(stats.MaxSwappedResidual,
                            Quaternion.Angle(delta, Quaternion.AngleAxis(swappedDeg, axis)));
                    }

                    if (checkSign && Mathf.Sign(SignedAngle(delta, axis)) != sign)
                        stats.WrongSign++;
                }
            }
        }

        private float ExpectedRate(float motorRadPerSec, bool armed)
        {
            // Mirrors PropellerSpin.Update: motor speed x visualSpeedScale, floored at 5000 deg/s while armed.
            float rate = motorRadPerSec * visualSpeedScale;
            return armed ? Mathf.Max(rate, 5000f) : rate;
        }

        private static float SignedAngle(Quaternion delta, Vector3 axis)
        {
            delta.ToAngleAxis(out float angle, out Vector3 a);
            if (angle > 180f)
                angle -= 360f;
            return angle * Mathf.Sign(Vector3.Dot(a, axis));
        }

        private Quaternion LocalRotation(Transform p) => Quaternion.Inverse(root.rotation) * p.rotation;

        private Dictionary<Transform, Quaternion> SnapshotLocalRotations()
        {
            var d = new Dictionary<Transform, Quaternion>();
            foreach (Transform p in props)
                d[p] = LocalRotation(p);
            return d;
        }

        #endregion

        #region Geometry helpers

        private float CameraDistance(Camera cam) => (cam.transform.position - root.position).magnitude;

        private void AssertDroneVisible(Camera cam, Bounds localBounds, string label)
        {
            Vector3 c = localBounds.center, e = localBounds.extents;
            for (int i = 0; i < 8; i++)
            {
                Vector3 corner = c + new Vector3((i & 1) == 0 ? -e.x : e.x, (i & 2) == 0 ? -e.y : e.y, (i & 4) == 0 ? -e.z : e.z);
                Vector3 v = cam.WorldToViewportPoint(root.TransformPoint(corner));
                Assert.IsTrue(v.z > cam.nearClipPlane && v.x >= 0 && v.x <= 1 && v.y >= 0 && v.y <= 1,
                    $"Frame corner off screen at {label}: viewport {v}");
            }

            Vector3 centre = root.TransformPoint(c);
            if (Physics.Linecast(cam.transform.position, centre, out RaycastHit hit, ~0, QueryTriggerInteraction.Ignore))
                Assert.IsTrue(hit.transform.IsChildOf(root), $"Drone hidden behind '{hit.transform.name}' at {label}");
        }

        /// <summary>
        /// Edit-mode step (runs before play mode starts). The stock prop mesh is not Read/Write
        /// enabled, so its vertices can only be read outside play mode. Record each prop's blade-tip
        /// radius in its own local units; in play mode it is multiplied by the prop's runtime scale.
        /// </summary>
        public void Setup()
        {
#if UNITY_EDITOR
            var prefab = UnityEditor.AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/DronePlayer.prefab");
            var lines = new List<string>();
            foreach (Transform t in prefab.GetComponentsInChildren<Transform>(true))
                if (t.name.StartsWith("Propeller_"))
                    lines.Add(t.name + " " + PropTipRadiusLocal(t).ToString("R", CultureInfo.InvariantCulture));
            File.WriteAllLines(PropRadiusFile, lines);
#endif
        }

        private static string PropRadiusFile => Path.GetFullPath(Path.Combine(Application.dataPath, "..", "Temp", "Drone7InchTests_propRadius.txt"));

        private static double PropTipRadiusLocal(Transform prop)
        {
            // Largest distance from the spin axis (prop local Y) over every mesh vertex under the prop.
            double r = 0;
            foreach (MeshFilter mf in prop.GetComponentsInChildren<MeshFilter>(true))
            {
                if (mf.sharedMesh == null)
                    continue;
                using (Mesh.MeshDataArray data = Mesh.AcquireReadOnlyMeshData(mf.sharedMesh))
                using (var vertices = new NativeArray<Vector3>(data[0].vertexCount, Allocator.Temp))
                {
                    data[0].GetVertices(vertices);
                    foreach (Vector3 v in vertices)
                    {
                        Vector3 local = prop.InverseTransformPoint(mf.transform.TransformPoint(v));
                        r = Math.Max(r, Math.Sqrt(local.x * local.x + local.z * local.z));
                    }
                }
            }
            return r;
        }

        private static double PropDiameter(Transform prop)
        {
            Assert.IsTrue(File.Exists(PropRadiusFile), "Prop radii were not recorded; run from the Test Runner so Setup() runs first");
            double localRadius = double.NaN;
            foreach (string line in File.ReadAllLines(PropRadiusFile))
            {
                string[] parts = line.Split(' ');
                if (parts[0] == prop.name)
                    localRadius = double.Parse(parts[1], CultureInfo.InvariantCulture);
            }
            Assert.IsFalse(double.IsNaN(localRadius), $"No recorded radius for {prop.name}");

            Vector3 s = prop.lossyScale;
            Assert.AreEqual(Mathf.Abs(s.x), Mathf.Abs(s.z), Mathf.Abs(s.x) * 1e-3f, $"{prop.name} is not scaled uniformly in its rotor plane");
            return 2 * localRadius * Mathf.Abs(s.x);
        }

        private NodeJson MotorFor(int quad)
        {
            // CAD frame: +Y front, -X left (same split as MotorQuadrants).
            foreach (NodeJson n in fixture.nodes)
            {
                if (n.type != 0)
                    continue;
                bool front = n.y >= 0, left = n.x < 0;
                if ((front ? (left ? 1 : 0) : (left ? 2 : 3)) == quad)
                    return n;
            }
            Assert.Fail($"No motor node in quadrant {QuadName(quad)}");
            return null;
        }

        private static string QuadName(int q) => q == 0 ? "front-right" : q == 1 ? "front-left" : q == 2 ? "back-left" : "back-right";

        #endregion

        #region Independent mass properties

        /// <summary>Reads a binary or ASCII STL; each entry is 9 doubles (3 vertices), in the file's millimetres.</summary>
        private static List<double[]> ReadStlMillimetres(string path)
        {
            byte[] data = File.ReadAllBytes(path);
            var tris = new List<double[]>();
            if (data.Length >= 84)
            {
                uint count = BitConverter.ToUInt32(data, 80);
                if (84 + 50L * count == data.Length)
                {
                    for (int t = 0; t < count; t++)
                    {
                        int o = 84 + 50 * t + 12;
                        var tri = new double[9];
                        for (int k = 0; k < 9; k++)
                            tri[k] = BitConverter.ToSingle(data, o + 4 * k);
                        tris.Add(tri);
                    }
                    return tris;
                }
            }

            var cur = new List<double>();
            foreach (string line in File.ReadAllLines(path))
            {
                string[] parts = line.Trim().Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length == 4 && parts[0] == "vertex")
                {
                    for (int k = 1; k < 4; k++)
                        cur.Add(double.Parse(parts[k], CultureInfo.InvariantCulture));
                    if (cur.Count == 9)
                    {
                        tris.Add(cur.ToArray());
                        cur.Clear();
                    }
                }
            }
            return tris;
        }

        /// <summary>Mass, COM and inertia about the COM of a closed mesh, by signed tetrahedra from the origin (metres, kg).</summary>
        private static void SolidProperties(List<double[]> trisMm, double density, out double mass, out double[] com, out double[,] inertia)
        {
            double vol6 = 0;
            double[] first = new double[3];
            double[,] second = new double[3, 3]; // integral of x_i x_j dV, times 120
            foreach (double[] t in trisMm)
            {
                double[] a = { t[0] * 1e-3, t[1] * 1e-3, t[2] * 1e-3 };
                double[] b = { t[3] * 1e-3, t[4] * 1e-3, t[5] * 1e-3 };
                double[] c = { t[6] * 1e-3, t[7] * 1e-3, t[8] * 1e-3 };
                double det = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
                vol6 += det;
                for (int i = 0; i < 3; i++)
                {
                    double si = a[i] + b[i] + c[i];
                    first[i] += si * det;
                    for (int j = 0; j < 3; j++)
                        second[i, j] += det * (a[i] * a[j] + b[i] * b[j] + c[i] * c[j] + si * (a[j] + b[j] + c[j]));
                }
            }

            double volume = vol6 / 6.0;
            mass = density * volume;
            com = new double[3];
            for (int i = 0; i < 3; i++)
                com[i] = first[i] / (24.0 * volume);

            double trace = 0;
            for (int i = 0; i < 3; i++)
                trace += second[i, i];
            inertia = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                {
                    double origin = density * ((i == j ? trace : 0) - second[i, j]) / 120.0;
                    // Shift from the origin to the COM.
                    double shift = mass * ((i == j ? Dot(com, com) : 0) - com[i] * com[j]);
                    inertia[i, j] = origin - shift;
                }
        }

        private static void AddPointInertia(double[,] inertia, double m, double[] p, double[] about)
        {
            double[] d = { p[0] - about[0], p[1] - about[1], p[2] - about[2] };
            double dd = Dot(d, d);
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    inertia[i, j] += m * ((i == j ? dd : 0) - d[i] * d[j]);
        }

        private static double Sq(double x) => x * x;
        private static double Dot(double[] a, double[] b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

        private static double[] MatVec(double[,] m, double[] v)
        {
            var r = new double[3];
            for (int i = 0; i < 3; i++)
                r[i] = m[i, 0] * v[0] + m[i, 1] * v[1] + m[i, 2] * v[2];
            return r;
        }

        private static double[,] MatMul(double[,] a, double[,] b)
        {
            var r = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    for (int k = 0; k < 3; k++)
                        r[i, j] += a[i, k] * b[k, j];
            return r;
        }

        private static double[,] Transpose(double[,] a)
        {
            var r = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    r[i, j] = a[j, i];
            return r;
        }

        private static double MaxAbs(double[,] a)
        {
            double m = 0;
            foreach (double x in a)
                m = Math.Max(m, Math.Abs(x));
            return m;
        }

        private static double MaxAbsDiff(double[,] a, double[,] b)
        {
            double m = 0;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    m = Math.Max(m, Math.Abs(a[i, j] - b[i, j]));
            return m;
        }

        #endregion

        #region Service, reflection, reporting

        private static bool ServiceResponds()
        {
            // Header 0 is not a valid request; the service answers it with a 255 error packet.
            try
            {
                using (var udp = new UdpClient())
                {
                    udp.Client.ReceiveTimeout = 1500;
                    udp.Connect(IPAddress.Loopback, ServicePort);
                    udp.Send(new byte[] { 0 }, 1);
                    IPEndPoint from = null;
                    byte[] reply = udp.Receive(ref from);
                    return reply.Length > 0 && reply[0] == 255;
                }
            }
            catch (SocketException)
            {
                return false;
            }
        }

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
            if (p != null)
                return p.GetValue(target);
            FieldInfo f = type.GetField(member, AnyMember);
            Assert.IsNotNull(f, $"{type.Name}.{member} not found");
            return f.GetValue(target);
        }

        private static void SetMember(Type type, object target, string member, object value)
        {
            PropertyInfo p = type.GetProperty(member, AnyMember);
            if (p != null)
            {
                p.SetValue(target, value);
                return;
            }
            FieldInfo f = type.GetField(member, AnyMember);
            Assert.IsNotNull(f, $"{type.Name}.{member} not found");
            f.SetValue(target, value);
        }

        private static object Call(object target, string method, params object[] args)
        {
            var types = new Type[args.Length];
            for (int i = 0; i < args.Length; i++)
                types[i] = args[i].GetType();
            MethodInfo m = target.GetType().GetMethod(method, AnyMember, null, types, null);
            Assert.IsNotNull(m, $"{target.GetType().Name}.{method} not found");
            return m.Invoke(target, args);
        }

        private static void Report(string line)
        {
            TestContext.WriteLine(line);
            Debug.Log("[Drone7InchTests] " + line);
        }

        #endregion
    }
}

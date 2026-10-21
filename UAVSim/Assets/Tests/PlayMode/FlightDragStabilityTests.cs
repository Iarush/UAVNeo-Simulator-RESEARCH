using System;
using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace UAVSim.Tests.PlayMode
{
    /// <summary>
    /// Phase 5 of .plans/real-flight-physics-spec.md: frame-scaled drag (section 4.2 item 6) and the
    /// stability warnings (section 4.7). Uses the flight harness in FlightMotorTests.cs.
    ///
    /// Frames of other inertias are made from the stock drone by changing its inertia and running
    /// Flight.Start again before arming, so the sweep does not need the mass service.
    /// </summary>
    public partial class FlightMotorTests
    {
        [UnityTest]
        public IEnumerator Drag_ScalesWithArea()
        {
            yield return Fly(DefaultDrone());
            // 50x the stock area keeps the terminal speed low, so it is reached in a few seconds
            // before the drone meets any scenery; the 2x ratio is what is tested.
            Vector3 stockArea = (Vector3)Get(flight, "DragArea"), area = 50f * stockArea;
            // Fly 40 m up, clear of the scenery ahead of the spawn (a teleport, test only).
            rb.position += 40f * Vector3.up;
            Set(flight, "TargetAltitude", (float)Get(flight, "TargetAltitude") + 40f);
            yield return Steps(5);
            Vector3 start = rb.position;
            float v1 = 0f, v2 = 0f;
            yield return TerminalSpeed(area, v => v1 = v);
            yield return TerminalSpeed(2f * area, v => v2 = v);
            float ratio = v2 / v1;
            Report($"drag: stock area side/top/front {stockArea:F5} m^2, test area {area:F5}; travelled {Flat3(drone.transform.position - start).magnitude:F1} m; terminal speed at pitch 0.5 {v1:F3} m/s, with 2x area {v2:F3} m/s, ratio {ratio:F3} (expected {1f / Mathf.Sqrt(2f):F3} +-10%)");
            Assert.AreEqual(1f / Mathf.Sqrt(2f), ratio, 0.1f / Mathf.Sqrt(2f), "terminal speed ratio");
            Assert.IsEmpty(errors, string.Join("\n", errors));
        }

        /// <summary>Holds pitch 0.5 for 12 s; the speed is the mean of the last 2 s. Reports a trace.</summary>
        private IEnumerator TerminalSpeed(Vector3 area, Action<float> result)
        {
            Set(flight, "dragArea", area);
            var trace = new System.Text.StringBuilder();
            float sum = 0f; int n = 0;
            // Drone.Update rewrites the inputs every frame, so set them before every physics step.
            for (int f = 1; f <= 600; f++)
            {
                Set(flight, "ForwardSpeed", 0.5f);
                yield return Steps(1);
                float speed = Flat3(rb.linearVelocity).magnitude;
                if (f % 50 == 0)
                    trace.Append($" {f * Time.fixedDeltaTime:F0}s {speed:F2} m/s pitch {Normalize(drone.transform.eulerAngles.x):F1};");
                if (f > 500) { sum += speed; n++; }
            }
            Set(flight, "ForwardSpeed", 0f);
            Report($"drag trace, area {area:F4}:{trace}");
            result(sum / n);
        }

        [UnityTest]
        public IEnumerator StabilityWarning_ShownForOutOfBandFrame()
        {
            // In band: the stock drone.
            var warnings = new List<string>();
            Application.LogCallback onLog = (m, s, t) => { if (t == LogType.Warning && m.Contains("Stability warning")) warnings.Add(m); };
            Application.logMessageReceived += onLog;
            try
            {
                yield return Fly(DefaultDrone(), arm: false);
                Vector3 stock = (Vector3)Get(flight, "ControlAuthority");
                string stockWarning = (string)Get(flight, "StabilityWarning");
                int stockLogs = warnings.Count;

                // Out of band: the stock drone with 100x its inertia (sluggish on every axis).
                Vector3 inertia = new Vector3((float)Get(flight, "Ixx"), (float)Get(flight, "Iyy"), (float)Get(flight, "Izz"));
                Reconfigure(inertia * 100f);
                Vector3 heavy = (Vector3)Get(flight, "ControlAuthority");
                string heavyWarning = (string)Get(flight, "StabilityWarning");
                string hud = HudMessage();
                Report($"stock drone authority (pitch, yaw, roll) {stock:F0}, warning '{stockWarning}', logs {stockLogs}; " +
                       $"100x inertia authority {heavy:F0}, warning '{heavyWarning}', HUD '{hud}', logs {warnings.Count - stockLogs}");
                Assert.IsNull(stockWarning, "stock drone warned");
                Assert.AreEqual(0, stockLogs, "stock drone logged a stability warning");
                Assert.IsNotNull(heavyWarning, "out-of-band frame did not warn");
                Assert.AreEqual(1, warnings.Count - stockLogs, "out-of-band frame: no Console warning");
                Assert.AreEqual(heavyWarning, hud, "out-of-band frame: HUD message not shown");
                Assert.IsEmpty(errors, string.Join("\n", errors));
            }
            finally { Application.logMessageReceived -= onLog; }
        }

        [UnityTest]
        public IEnumerator StabilityWarning_CustomFrames()
        {
            // Real imports through the mass service: the cube quad must not warn, the
            // short-arm frame with heavy outboard nodes must.
            yield return Fly(Cube(0f), arm: false);
            string cube = (string)Get(flight, "StabilityWarning");
            Vector3 cubeAuthority = (Vector3)Get(flight, "ControlAuthority");
            yield return Fly(ShortArm(), arm: false);
            string shortArm = (string)Get(flight, "StabilityWarning");
            Vector3 shortArmAuthority = (Vector3)Get(flight, "ControlAuthority");
            Report($"cube quad authority {cubeAuthority:F0} warning '{cube}'; short-arm frame authority {shortArmAuthority:F0} warning '{shortArm}'");
            Assert.IsNull(cube, "cube quad warned");
            Assert.IsNotNull(shortArm, "short-arm frame did not warn");
            Assert.IsEmpty(errors, string.Join("\n", errors));
        }

        [UnityTest]
        public IEnumerator BuildDrone_WarnsForOutOfBandEstimate()
        {
            yield return Fly(DefaultDrone(), arm: false);
            // Back to the menu: the panel lives in Main.
            UnityEngine.SceneManagement.SceneManager.LoadScene(0);
            yield return null;
            yield return null;

            Type panelType = GameType("ImportPageUI");
            var panel = (Component)UnityEngine.Object.FindAnyObjectByType(panelType, FindObjectsInactive.Include);
            Assert.IsNotNull(panel, "ImportPageUI not found");
            // The node rows bind when the panel opens.
            for (Transform t = panel.transform; t != null; t = t.parent) t.gameObject.SetActive(true);
            yield return null;

            string Build(Frame frame)
            {
                Set(panel, "selectedPath", frame.stl);
                object density = Get(panel, "densityInput");
                Set(density, "text", frame.density.ToString(System.Globalization.CultureInfo.InvariantCulture));
                Type nodeType = GameType("DronePresetNode");
                IList list = (IList)Activator.CreateInstance(typeof(List<>).MakeGenericType(nodeType));
                foreach (Node n in frame.nodes)
                {
                    object d = Activator.CreateInstance(nodeType);
                    nodeType.GetField("type").SetValue(d, n.type);
                    nodeType.GetField("x").SetValue(d, n.cad.x);
                    nodeType.GetField("y").SetValue(d, n.cad.y);
                    nodeType.GetField("z").SetValue(d, n.cad.z);
                    nodeType.GetField("massKg").SetValue(d, n.kg);
                    list.Add(d);
                }
                object nodes = Get(panel, "nodePlacement");
                object[] args = { list, null };
                Assert.IsTrue((bool)nodes.GetType().GetMethod("TryShowNodes").Invoke(nodes, args), $"TryShowNodes: {args[1]}");
                Call(panel, "OnBuildDroneClicked");
                return (string)Get(panel, "pageMessage");
            }

            string cube = Build(Cube(0f));
            string shortArm = Build(ShortArm());
            Report($"Build Drone status: cube quad '{cube}'; short-arm frame '{shortArm}'");
            Assert.IsFalse(cube.Contains("unstable"), "cube quad estimate warned");
            Assert.IsTrue(shortArm.Contains("May be unstable"), "short-arm frame estimate did not warn");
            Assert.IsEmpty(errors, string.Join("\n", errors));
        }

        /// <summary>
        /// Sweep for stabilityBand: the stock drone with its roll/pitch or yaw inertia scaled, kicked by
        /// a pitch-and-yaw step, reports authority and whether it settles. Slow; run on demand.
        /// </summary>
        [UnityTest, Category("Sweep")]
        public IEnumerator StabilityBand_Sweep()
        {
            float[] scales = { 0.01f, 0.02f, 0.05f, 0.1f, 0.2f, 0.5f, 1f, 2f, 5f, 10f, 20f, 50f, 100f, 200f };
            foreach (string axes in new[] { "roll/pitch", "yaw" })
                foreach (float k in scales)
                {
                    yield return Fly(DefaultDrone(), arm: false);
                    Vector3 i0 = new Vector3((float)Get(flight, "Ixx"), (float)Get(flight, "Iyy"), (float)Get(flight, "Izz"));
                    Vector3 i = axes == "yaw" ? new Vector3(i0.x, i0.y * k, i0.z) : new Vector3(i0.x * k, i0.y, i0.z * k);
                    Reconfigure(i);
                    Vector3 authority = (Vector3)Get(flight, "ControlAuthority");

                    Call(flight, "Arm");
                    yield return Steps(1);
                    for (int f = 0; f < 300 && (bool)Get(flight, "IsLaunching"); f++)
                        yield return FastSteps(10);
                    bool launched = !(bool)Get(flight, "IsLaunching");
                    SetMode("Pcmd");
                    yield return FastSteps(50);

                    for (int f = 0; f < 25; f++) { Set(flight, "ForwardSpeed", 0.5f); Set(flight, "YawSpeed", 0.5f); yield return Steps(1); }
                    Set(flight, "ForwardSpeed", 0f); Set(flight, "YawSpeed", 0f);
                    float maxTilt = 0f, endRate = 0f, endTilt = 0f;
                    for (int f = 0; f < 200; f++)
                    {
                        yield return Steps(1);
                        float tilt = Vector3.Angle(drone.transform.up, Vector3.up);
                        maxTilt = Mathf.Max(maxTilt, tilt);
                        if (f >= 150) { endRate = Mathf.Max(endRate, rb.angularVelocity.magnitude); endTilt = Mathf.Max(endTilt, tilt); }
                    }
                    bool stable = launched && maxTilt < 60f && endTilt < 3f && endRate < 0.2f;
                    Report($"sweep {axes} x{k}: authority (pitch, yaw, roll) {authority:F0}; launched {launched}; max tilt {maxTilt:F1} deg; " +
                           $"last 1 s tilt {endTilt:F2} deg, rate {endRate:F3} rad/s; oscillation warning '{Get(flight, "StabilityWarning")}'; {(stable ? "STABLE" : "UNSTABLE")}");
                }
        }

        /// <summary>
        /// Out of band (too little roll authority): the cube with its motors on 0.03 m arms and two
        /// 1 kg nodes 0.4 m out along CAD x, which give a large roll inertia.
        /// </summary>
        private static Frame ShortArm() => new Frame
        {
            name = "short-arm frame", stl = CubeStl, density = 1000f,
            nodes = new List<Node>
            {
                new Node(1, 0.4f, 0f, 0f, 1f), new Node(2, -0.4f, 0f, 0f, 1f),
                new Node(0, 0.03f, 0.03f, 0f, 0.05f), new Node(0, -0.03f, 0.03f, 0f, 0.05f),
                new Node(0, 0.03f, -0.03f, 0f, 0.05f), new Node(0, -0.03f, -0.03f, 0f, 0.05f),
            },
        };

        /// <summary>Gives the loaded (disarmed) drone a new inertia and runs Flight.Start again.</summary>
        private void Reconfigure(Vector3 inertia)
        {
            Set(flight, "Ixx", inertia.x);
            Set(flight, "Iyy", inertia.y);
            Set(flight, "Izz", inertia.z);
            Call(flight, "Start");
        }

        private static string HudMessage()
        {
            UnityEngine.Object hud = UnityEngine.Object.FindAnyObjectByType(GameType("Hud"));
            return hud == null ? null : ((UnityEngine.UI.Text)Get(hud, "messageText"))?.text;
        }
    }
}

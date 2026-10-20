using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Text.RegularExpressions;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace UAVSim.Tests.PlayMode
{
    /// <summary>
    /// Phase 0 of .plans/real-flight-physics-spec.md: FlightConfig loads both flight files, every
    /// required key is present, a missing file falls back with one error, and the PX4 controller
    /// gains are the built-in stock defaults whatever the params file says.
    /// FlightConfig lives in Assembly-CSharp, so it is reached by reflection.
    /// </summary>
    public class FlightConfigTests
    {
        private static Type Config => Type.GetType("FlightConfig, Assembly-CSharp");
        private const BindingFlags Any = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static;

        private static void Reload(string dir) => Config.GetMethod("Reload", Any).Invoke(null, new object[] { dir });
        private static float Px4(string name) => (float)Config.GetMethod("Px4", Any).Invoke(null, new object[] { name });
        private static float Sim(string key) => (float)Config.GetMethod("Sim", Any).Invoke(null, new object[] { key });
        private static bool Prop(string name) => (bool)Config.GetProperty(name, Any).GetValue(null);
        private static IReadOnlyDictionary<string, float> Table(string name) => (IReadOnlyDictionary<string, float>)Config.GetField(name, Any).GetValue(null);
        private static string Dir => (string)Config.GetProperty("DefaultDirectory", Any).GetValue(null);

        private string tempDir;

        [SetUp]
        public void Setup() => Assert.IsNotNull(Config, "FlightConfig not found in Assembly-CSharp");

        [TearDown]
        public void Cleanup()
        {
            if (tempDir != null && Directory.Exists(tempDir))
                Directory.Delete(tempDir, true);
            tempDir = null;
            Reload(null);
        }

        [UnityTest]
        public IEnumerator Files_Load_AllKeysPresent()
        {
            yield return null;
            Reload(null);
            Assert.IsFalse(Prop("Px4FromFallback"), "uav-neo.params did not load");
            Assert.IsFalse(Prop("SimFromFallback"), "flight-sim.json did not load");

            foreach (var kv in Table("BuiltInPx4"))
            {
                float v = Px4(kv.Key);
                Report($"{kv.Key} = {v} (built-in {kv.Value})");
                Assert.AreEqual(kv.Value, v, Math.Abs(kv.Value) * 1e-4f + 1e-5f, $"{kv.Key} in the file differs from the built-in copy");
            }
            foreach (var kv in Table("BuiltInSim"))
            {
                float v = Sim(kv.Key);
                Report($"{kv.Key} = {v}");
                Assert.AreEqual(kv.Value, v, 1e-6f, $"{kv.Key} in the file differs from the built-in copy");
            }
        }

        [UnityTest]
        public IEnumerator MissingParamsFile_FallsBack_LogsOneError()
        {
            yield return null;
            tempDir = Path.Combine(Path.GetTempPath(), "FlightConfigTests_" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(tempDir);
            File.Copy(Path.Combine(Dir, "flight-sim.json"), Path.Combine(tempDir, "flight-sim.json"));

            LogAssert.Expect(LogType.Error, new Regex(@"^\[FlightConfig\] Cannot read .*uav-neo\.params"));
            Reload(tempDir);
            LogAssert.NoUnexpectedReceived();

            Assert.IsTrue(Prop("Px4FromFallback"));
            Assert.IsFalse(Prop("SimFromFallback"));
            Assert.AreEqual(0.5f, Px4("MPC_THR_HOVER"));
            Report($"fallback MPC_THR_HOVER = {Px4("MPC_THR_HOVER")}");
        }

        [UnityTest]
        public IEnumerator MissingSimFile_FallsBack_LogsOneError()
        {
            yield return null;
            tempDir = Path.Combine(Path.GetTempPath(), "FlightConfigTests_" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(tempDir);
            File.Copy(Path.Combine(Dir, "uav-neo.params"), Path.Combine(tempDir, "uav-neo.params"));

            LogAssert.Expect(LogType.Error, new Regex(@"^\[FlightConfig\] Cannot read .*flight-sim\.json"));
            Reload(tempDir);
            LogAssert.NoUnexpectedReceived();

            Assert.IsFalse(Prop("Px4FromFallback"));
            Assert.IsTrue(Prop("SimFromFallback"));
            Assert.AreEqual(0.0125f, Sim("motorTimeConstantUp"));
        }

        [UnityTest]
        public IEnumerator RateGains_AreStockDefaults_EvenWhenFileDiffers()
        {
            yield return null;
            Reload(null);
            // The real drone's tuned values are in the file; the sim must not pick them up.
            string text = File.ReadAllText(Path.Combine(Dir, "uav-neo.params"));
            StringAssert.Contains("MC_ROLLRATE_I\t0.213", text, "fixture assumption: the file holds the drone's tuned MC_ROLLRATE_I");
            Assert.Throws<TargetInvocationException>(() => Px4("MC_ROLLRATE_I"), "Rate gains must not be readable from the params file");

            Type g = Config.GetNestedType("Px4StockGains");
            var expected = new Dictionary<string, float>
            {
                { "RollRateP", 0.15f }, { "RollRateI", 0.2f }, { "RollRateD", 0.003f }, { "RollRateK", 1f }, { "RollRateFF", 0f },
                { "PitchRateP", 0.15f }, { "PitchRateI", 0.2f }, { "PitchRateD", 0.003f }, { "PitchRateK", 1f }, { "PitchRateFF", 0f },
                { "YawRateP", 0.2f }, { "YawRateI", 0.1f }, { "YawRateD", 0f }, { "YawRateK", 1f }, { "YawRateFF", 0f },
                { "RollP", 6.5f }, { "PitchP", 6.5f }, { "YawP", 2.8f },
            };
            foreach (var kv in expected)
            {
                float v = (float)g.GetField(kv.Key).GetValue(null);
                Report($"stock {kv.Key} = {v}");
                Assert.AreEqual(kv.Value, v, $"{kv.Key} is not the PX4 v1.17.0 default");
            }
        }

        private static void Report(string line)
        {
            TestContext.WriteLine(line);
            Debug.Log("[FlightConfigTests] " + line);
        }
    }
}

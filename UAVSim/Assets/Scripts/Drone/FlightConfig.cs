using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

/// <summary>
/// Flight-model configuration (spec: .plans/real-flight-physics-spec.md, section 4.1).
///
/// Two files in StreamingAssets/Flight, loaded once on first use:
///   - uav-neo.params: the real drone's PX4 parameters in QGC format, made from a QGroundControl
///     .tlog by tools/extract-px4-params.py. Only the BEHAVIOUR settings listed in
///     <see cref="BuiltInPx4"/> are read from it (speed, tilt, acceleration limits, ...).
///   - flight-sim.json: sim-only values PX4 does not have (motor lag, launch height, ...).
///
/// A missing or unreadable file logs one error and falls back to the built-in copies below,
/// which hold the same values the committed files had when this was written.
///
/// PX4 controller GAINS never come from the file: the real drone's tuned gains are hardware
/// data (spec principle 1). <see cref="Px4StockGains"/> holds the PX4 v1.17.0 stock defaults.
/// </summary>
public static class FlightConfig
{
    public const string ParamsFileName = "uav-neo.params";
    public const string SimFileName = "flight-sim.json";

    /// <summary>
    /// PX4 v1.17.0 stock defaults for the controller gains.
    /// Source (checked 2026-10-19): PX4-Autopilot tag v1.17.0,
    ///   src/modules/mc_rate_control/mc_rate_control_params.c (MC_*RATE_P/I/D/FF/K, MC_*_INT_LIM)
    ///   src/modules/mc_att_control/mc_att_control_params.c (MC_ROLL_P, MC_PITCH_P, MC_YAW_P).
    /// Exception (owner decision 2026-10-20): MC_ROLL_P / MC_PITCH_P use the real drone's 6.5,
    /// not stock 4.0. The real drone's tuned rate I/D/K are not used.
    /// </summary>
    public static class Px4StockGains
    {
        public const float RollRateP = 0.15f, RollRateI = 0.2f, RollRateD = 0.003f, RollRateFF = 0f, RollRateK = 1f, RollRateIntLim = 0.30f;
        public const float PitchRateP = 0.15f, PitchRateI = 0.2f, PitchRateD = 0.003f, PitchRateFF = 0f, PitchRateK = 1f, PitchRateIntLim = 0.30f;
        public const float YawRateP = 0.2f, YawRateI = 0.1f, YawRateD = 0f, YawRateFF = 0f, YawRateK = 1f, YawRateIntLim = 0.30f;
        public const float RollP = 6.5f, PitchP = 6.5f, YawP = 2.8f;
    }

    /// <summary>
    /// The PX4 behaviour settings the sim reads, with the real drone's values (uav-neo.params,
    /// PX4 v1.17.0, log 2026-07-29 17-08-00) as the fallback. Every key here is required in the file.
    /// </summary>
    public static readonly IReadOnlyDictionary<string, float> BuiltInPx4 = new Dictionary<string, float>
    {
        { "MPC_THR_HOVER", 0.5f },
        { "MPC_VEL_MANUAL", 1.0f },
        { "MPC_XY_VEL_MAX", 1.0f },
        { "MPC_Z_VEL_MAX_UP", 0.6f },
        { "MPC_Z_VEL_MAX_DN", 0.6f },
        { "MPC_TKO_SPEED", 1.5f },
        { "MPC_TKO_RAMP_T", 3.0f },
        { "MPC_LAND_SPEED", 0.7f },
        { "MPC_ACC_HOR", 3.0f },
        { "MPC_ACC_HOR_MAX", 5.0f },
        { "MPC_ACC_UP_MAX", 4.0f },
        { "MPC_ACC_DOWN_MAX", 3.0f },
        { "MPC_JERK_MAX", 8.0f },
        { "MPC_TILTMAX_AIR", 45f },
        { "MPC_MAN_Y_MAX", 150f },
        { "MPC_POS_MODE", 4f },
        { "MPC_XY_P", 0.95f },
        { "MPC_XY_VEL_P_ACC", 1.8f },
        { "MPC_XY_VEL_I_ACC", 0.4f },
        { "MPC_XY_VEL_D_ACC", 0.2f },
        { "MPC_Z_P", 1.0f },
        { "MPC_Z_VEL_P_ACC", 4.0f },
        { "MPC_Z_VEL_I_ACC", 2.0f },
        { "MPC_Z_VEL_D_ACC", 0.0f },
        { "MPC_THR_MIN", 0.12f },
        { "MPC_THR_MAX", 1.0f },
        { "MC_YAW_WEIGHT", 0.4f },
        { "MC_ROLLRATE_MAX", 220f },
        { "MC_PITCHRATE_MAX", 220f },
        { "MC_YAWRATE_MAX", 250f },
        { "MC_AIRMODE", 0f },
        { "CA_ROTOR0_KM", 0.05f },
        { "THR_MDL_FAC", 0f },
    };

    /// <summary>Sim-only values (flight-sim.json), with the committed file's values as the fallback.</summary>
    public static readonly IReadOnlyDictionary<string, float> BuiltInSim = new Dictionary<string, float>
    {
        { "motorTimeConstantUp", 0.0125f },
        { "motorTimeConstantDown", 0.025f },
        { "launchHeight", 1.0f },
        { "pcmdMaxTiltDeg", 35f },
        { "pcmdMaxYawRateDegS", 57.29578f },        { "dragCd", 1.0f },
        { "dragFillFactor", 0.5f },
        { "angularDragCoeff", 0.005f },
        { "subSteps", 5f },
    };

    [Serializable]
    private class SimEntry
    {
        public string key;
        public float value;
        public string source;
    }

    [Serializable]
    private class SimFile
    {
        public int version;
        public SimEntry[] values;
    }

    private static Dictionary<string, float> px4;
    private static Dictionary<string, float> sim;

    /// <summary>True when the last load used a built-in copy for that file.</summary>
    public static bool Px4FromFallback { get; private set; }
    public static bool SimFromFallback { get; private set; }

    public static string DefaultDirectory => Path.Combine(Application.streamingAssetsPath, "Flight");

    /// <summary>A PX4 behaviour setting by name, e.g. "MPC_THR_HOVER".</summary>
    public static float Px4(string name)
    {
        EnsureLoaded();
        if (px4.TryGetValue(name, out float v)) return v;
        throw new KeyNotFoundException($"[FlightConfig] {name} is not a PX4 setting the sim reads (see FlightConfig.BuiltInPx4).");
    }

    /// <summary>A sim-only value by key, e.g. "motorTimeConstantUp".</summary>
    public static float Sim(string key)
    {
        EnsureLoaded();
        if (sim.TryGetValue(key, out float v)) return v;
        throw new KeyNotFoundException($"[FlightConfig] {key} is not a sim setting (see FlightConfig.BuiltInSim).");
    }

    private static void EnsureLoaded()
    {
        if (px4 == null || sim == null)
            Reload(null);
    }

    /// <summary>(Re)loads both files from <paramref name="directory"/>, or StreamingAssets/Flight when null.</summary>
    public static void Reload(string directory)
    {
        directory = directory ?? DefaultDirectory;

        px4 = new Dictionary<string, float>(BuiltInPx4);
        Px4FromFallback = !TryLoadParams(Path.Combine(directory, ParamsFileName), px4, out string px4Error);
        if (Px4FromFallback)
        {
            px4 = new Dictionary<string, float>(BuiltInPx4);
            Debug.LogError($"[FlightConfig] {px4Error} Using the built-in PX4 values.");
        }

        sim = new Dictionary<string, float>(BuiltInSim);
        SimFromFallback = !TryLoadSim(Path.Combine(directory, SimFileName), sim, out string simError);
        if (SimFromFallback)
        {
            sim = new Dictionary<string, float>(BuiltInSim);
            Debug.LogError($"[FlightConfig] {simError} Using the built-in sim values.");
        }
    }

    private static bool TryLoadParams(string path, Dictionary<string, float> into, out string error)
    {
        string[] lines;
        try { lines = File.ReadAllLines(path); }
        catch (Exception e) { error = $"Cannot read {path}: {e.Message}."; return false; }

        var found = new HashSet<string>();
        foreach (string raw in lines)
        {
            string line = raw.Trim();
            if (line.Length == 0 || line[0] == '#') continue;
            // QGC format: vehicle-id, component-id, name, value, type (tab-separated).
            string[] parts = line.Split('\t');
            if (parts.Length < 4 || !into.ContainsKey(parts[2])) continue;
            if (!float.TryParse(parts[3], NumberStyles.Float, CultureInfo.InvariantCulture, out float v) || !float.IsFinite(v))
            {
                error = $"{path}: bad value '{parts[3]}' for {parts[2]}.";
                return false;
            }
            into[parts[2]] = v;
            found.Add(parts[2]);
        }

        var missing = new List<string>();
        foreach (string key in BuiltInPx4.Keys)
            if (!found.Contains(key)) missing.Add(key);
        if (missing.Count > 0)
        {
            error = $"{path} is missing {string.Join(", ", missing)}.";
            return false;
        }
        error = null;
        return true;
    }

    private static bool TryLoadSim(string path, Dictionary<string, float> into, out string error)
    {
        SimFile file;
        try { file = JsonUtility.FromJson<SimFile>(File.ReadAllText(path)); }
        catch (Exception e) { error = $"Cannot read {path}: {e.Message}."; return false; }
        if (file?.values == null) { error = $"{path} has no \"values\" list."; return false; }

        var found = new HashSet<string>();
        foreach (SimEntry e in file.values)
        {
            if (e?.key == null || !into.ContainsKey(e.key)) continue;
            if (!float.IsFinite(e.value)) { error = $"{path}: {e.key} is not a finite number."; return false; }
            into[e.key] = e.value;
            found.Add(e.key);
        }

        var missing = new List<string>();
        foreach (string key in BuiltInSim.Keys)
            if (!found.Contains(key)) missing.Add(key);
        if (missing.Count > 0)
        {
            error = $"{path} is missing {string.Join(", ", missing)}.";
            return false;
        }
        error = null;
        return true;
    }
}

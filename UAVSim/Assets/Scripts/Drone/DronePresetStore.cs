using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

/// <summary>
/// One saved import: the STL path, density and node rows a student entered on the custom drone
/// panel. Node positions are in the STL/CAD Z-up frame in metres, exactly as typed, the same frame
/// as <see cref="CustomFrameLoader.NodeData.position"/>.
/// </summary>
[Serializable]
public class DronePreset
{
    public string name;
    public string stlPath;
    public float density;
    public List<DronePresetNode> nodes = new List<DronePresetNode>();
}

[Serializable]
public class DronePresetNode
{
    /// <summary>The <see cref="CustomFrameLoader.NodeType"/> value.</summary>
    public int type;
    public float x;
    public float y;
    public float z;
    public float massKg;
}

/// <summary>
/// Reads and writes drone presets as one JSON file in Application.persistentDataPath. Pure data,
/// no UI: <see cref="ImportPageUI"/> calls it.
///
/// Every save rewrites the whole file through a temporary file, so a crash mid-write leaves the
/// previous file intact. A file that cannot be read gives an empty list and a logged reason, and is
/// moved aside on the next save rather than overwritten.
/// </summary>
public static class DronePresetStore
{
    private const string FileName = "drone_presets.json";
    private const int FileVersion = 1;

    /// <summary>Longest preset name accepted, so names fit the dropdown.</summary>
    public const int MaxNameLength = 40;

    [Serializable]
    private class PresetFile
    {
        public int version = FileVersion;
        public List<DronePreset> presets = new List<DronePreset>();
    }

    public static string FilePath => Path.Combine(Application.persistentDataPath, FileName);

    /// <summary>
    /// Returns every readable preset, sorted by name. Never throws: a missing file gives an empty
    /// list, and an unreadable file or entry is logged and skipped.
    /// </summary>
    public static List<DronePreset> LoadAll()
    {
        TryReadFile(out List<DronePreset> presets);
        return presets;
    }

    /// <summary>
    /// Saves the preset, replacing any existing preset with the same name (case-insensitive).
    /// </summary>
    /// <param name="replaced">True when a preset with this name already existed.</param>
    /// <param name="error">Why the save failed; empty on success.</param>
    public static bool Save(DronePreset preset, out bool replaced, out string error)
    {
        replaced = false;

        if (!IsValid(preset, out error))
            return false;

        preset.name = preset.name.Trim();

        if (!TryReadFile(out List<DronePreset> presets))
        {
            // Keep the unreadable file for inspection instead of silently overwriting it.
            string asidePath = FilePath + ".unreadable-" + DateTime.Now.ToString("yyyyMMdd-HHmmss");
            try
            {
                File.Move(FilePath, asidePath);
                Debug.LogWarning($"[DronePresetStore] Moved the unreadable preset file to {asidePath}.");
            }
            catch (Exception e)
            {
                error = $"The preset file could not be read or moved aside: {e.Message}";
                Debug.LogError($"[DronePresetStore] {error}");
                return false;
            }
        }

        int existing = presets.FindIndex(p => SameName(p.name, preset.name));
        replaced = existing >= 0;
        if (replaced)
            presets[existing] = preset;
        else
            presets.Add(preset);
        Sort(presets);

        string json = JsonUtility.ToJson(new PresetFile { presets = presets }, true);
        string tempPath = FilePath + ".tmp";
        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(FilePath));
            File.WriteAllText(tempPath, json);
            if (File.Exists(FilePath))
                File.Replace(tempPath, FilePath, null);
            else
                File.Move(tempPath, FilePath);
        }
        catch (Exception e)
        {
            error = $"Could not write {FilePath}: {e.Message}";
            Debug.LogError($"[DronePresetStore] {error}");
            try { File.Delete(tempPath); } catch (Exception) { /* best effort */ }
            return false;
        }

        error = string.Empty;
        return true;
    }

    public static bool SameName(string a, string b)
    {
        return string.Equals(a?.Trim(), b?.Trim(), StringComparison.OrdinalIgnoreCase);
    }

    /// <summary>
    /// False only when the file exists but cannot be read or parsed. Presets is always non-null.
    /// </summary>
    private static bool TryReadFile(out List<DronePreset> presets)
    {
        presets = new List<DronePreset>();

        string path = FilePath;
        if (!File.Exists(path))
            return true;

        PresetFile file;
        try
        {
            file = JsonUtility.FromJson<PresetFile>(File.ReadAllText(path));
        }
        catch (Exception e)
        {
            Debug.LogError($"[DronePresetStore] Could not read {path}; no presets loaded. {e.Message}");
            return false;
        }

        if (file == null || file.presets == null)
        {
            Debug.LogError($"[DronePresetStore] {path} holds no preset list; no presets loaded.");
            return false;
        }

        foreach (DronePreset preset in file.presets)
        {
            if (!IsValid(preset, out string reason))
            {
                Debug.LogWarning($"[DronePresetStore] Skipped preset '{preset?.name}' in {path}: {reason}");
                continue;
            }
            if (presets.Exists(p => SameName(p.name, preset.name)))
            {
                Debug.LogWarning($"[DronePresetStore] Skipped duplicate preset name '{preset.name}' in {path}.");
                continue;
            }
            presets.Add(preset);
        }

        Sort(presets);
        return true;
    }

    /// <summary>
    /// Structural checks only. The motor layout is checked by the panel before saving, with the
    /// same validator the loader uses.
    /// </summary>
    private static bool IsValid(DronePreset preset, out string reason)
    {
        if (preset == null)
        {
            reason = "missing preset.";
            return false;
        }
        if (string.IsNullOrWhiteSpace(preset.name))
        {
            reason = "the name is empty.";
            return false;
        }
        if (preset.name.Trim().Length > MaxNameLength)
        {
            reason = $"the name is longer than {MaxNameLength} characters.";
            return false;
        }
        if (string.IsNullOrWhiteSpace(preset.stlPath))
        {
            reason = "no STL path.";
            return false;
        }
        if (!(preset.density > 0f) || float.IsInfinity(preset.density))
        {
            reason = "density must be a positive number.";
            return false;
        }
        if (preset.nodes == null || preset.nodes.Count < MotorQuadrants.RequiredMotorCount)
        {
            reason = $"needs at least {MotorQuadrants.RequiredMotorCount} nodes.";
            return false;
        }
        foreach (DronePresetNode node in preset.nodes)
        {
            if (node == null || node.type < 0 || node.type > byte.MaxValue ||
                !Enum.IsDefined(typeof(CustomFrameLoader.NodeType), (byte)node.type))
            {
                reason = "a node has an unknown type.";
                return false;
            }
            if (!IsFinite(node.x) || !IsFinite(node.y) || !IsFinite(node.z) || !(node.massKg > 0f) || float.IsInfinity(node.massKg))
            {
                reason = "a node has an invalid position or mass.";
                return false;
            }
        }

        reason = string.Empty;
        return true;
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    private static void Sort(List<DronePreset> presets)
    {
        presets.Sort((a, b) => string.Compare(a.name, b.name, StringComparison.OrdinalIgnoreCase));
    }
}

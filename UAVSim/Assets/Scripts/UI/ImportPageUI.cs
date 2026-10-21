using System.Collections.Generic;
using System.Globalization;
using System.IO;
using SimpleFileBrowser;
using TMPro;
using UnityEngine;

/// <summary>
/// Binds the hand-built custom drone panel: pick an STL, enter a material density, and fill the
/// node table below them (bound by <see cref="NodePlacementUI"/> on the same panel). Build Drone
/// stores all three into the CustomFrameLoader statics; it does not start the simulation.
/// </summary>
public class ImportPageUI : MonoBehaviour
{
    private const string NoPathLabel = "File Path: (none selected)";

    public TMP_Text selectedPathText;
    public TMP_InputField densityInput;
    public TMP_Text statusText;

    /// <summary>Node table on this panel. Build Drone stores its rows with the path and density.</summary>
    public NodePlacementUI nodePlacement;

    /// <summary>
    /// Saved presets (see <see cref="DronePresetStore"/>). Option 0 is a prompt; option i is
    /// preset i - 1. Its On Value Changed is wired to <see cref="OnPresetSelected"/> in the Editor.
    /// </summary>
    [Header("Presets")]
    public TMP_Dropdown presetDropdown;
    /// <summary>Name to save under. Filled with the preset's name when one is selected.</summary>
    public TMP_InputField presetNameInput;

    // The picked path is tracked here rather than read back out of selectedPathText, since that
    // label carries placeholder text ("File Path: ...") that would otherwise read as a valid
    // selection during confirm-time validation.
    private string selectedPath;

    // statusText carries two things: the page's own message, and a second line with the mass
    // service state. Both are kept here and recombined by SetStatus/RefreshStatus so neither
    // overwrites the other.
    private string pageMessage = string.Empty;
    private int shownServiceVersion = -1;

    // Presets as last read from disk, in dropdown order (option i + 1).
    private List<DronePreset> presets = new List<DronePreset>();
    private string selectedPresetName;

    private void OnEnable()
    {
        // Started here rather than in CustomFrameLoader.Awake(): the bundled interpreter takes
        // about a second to come up, and Awake() gives the service only 5 s to answer. Only the
        // custom-frame path pays for it; default-drone students never open this panel.
        MassServiceLauncher.EnsureRunning();

        // Restored from the statics only when this panel has no path of its own yet (first open,
        // or after the scene reloads on return from flight), so Back and reopen keeps whatever the
        // panel showed - including a path filled from a preset but not yet built.
        if (selectedPath == null && !string.IsNullOrEmpty(CustomFrameLoader.SelectedSTLPath))
            selectedPath = CustomFrameLoader.SelectedSTLPath;

        if (selectedPathText != null)
            selectedPathText.text = selectedPath == null ? NoPathLabel : $"File Path: {selectedPath}";

        SetStatus(selectedPath == null ? string.Empty : DescribeSelection(selectedPath));

        // Re-read on every open, so the list always matches the file.
        RefreshPresetDropdown(selectedPresetName);
    }

    private void Update()
    {
        if (MassServiceLauncher.Current.version != shownServiceVersion)
            RefreshStatus();
    }

    /// <summary>Hides this panel, returning the user to the main menu.</summary>
    public void Close()
    {
        gameObject.SetActive(false);
    }

    public void OnBrowseButtonClicked()
    {
        FileBrowser.SetFilters(true, new FileBrowser.Filter("STL Files", ".stl"));
        FileBrowser.SetDefaultFilter(".stl");
        FileBrowser.ShowLoadDialog(
            OnStlPicked,
            OnStlPickCancelled,
            FileBrowser.PickMode.Files,
            false, null, null,
            "Select Drone STL", "Select");
    }

    private void OnStlPicked(string[] paths)
    {
        if (paths == null || paths.Length == 0)
        {
            OnStlPickCancelled();
            return;
        }

        string path = paths[0];
        selectedPath = path;
        selectedPathText.text = $"File Path: {path}";
        CustomFrameLoader.SelectedSTLPath = path;

        // Node positions belong to the STL they were placed against, and SelectedNodes is a
        // static that outlives the scene. Clearing here makes it impossible to fly a new frame
        // carrying a previous frame's nodes - only NodePlacementUI repopulates the list.
        CustomFrameLoader.SelectedNodes.Clear();

        SetStatus(DescribeSelection(path));
    }

    private void OnStlPickCancelled()
    {
        SetStatus("No file selected.");
    }

    /// <summary>
    /// Validates the STL, density and node table together and stores all three into the
    /// CustomFrameLoader statics. Does not start the simulation - the student presses Back, then
    /// Begin on the main menu.
    /// </summary>
    public void OnBuildDroneClicked()
    {
        if (string.IsNullOrEmpty(selectedPath))
        {
            SetStatus("Select an STL file before building.");
            return;
        }

        if (!File.Exists(selectedPath))
        {
            SetStatus($"{Path.GetFileName(selectedPath)} was not found - browse to it again.");
            return;
        }

        if (!TryReadDensity(out float density))
        {
            SetStatus("Density must be a positive number.");
            return;
        }

        if (nodePlacement == null)
        {
            Debug.LogError("[ImportPageUI] nodePlacement is not assigned - cannot read the node table.", this);
            SetStatus("Could not read the node table - see console.");
            return;
        }

        // Nodes go last: TryStoreNodes writes SelectedNodes only when every row is valid, so any
        // failure leaves all three statics as they were. Its own message shows under the table.
        if (!nodePlacement.TryStoreNodes())
        {
            SetStatus(DescribeSelection(selectedPath));
            return;
        }

        CustomFrameLoader.SelectedSTLPath = selectedPath;
        CustomFrameLoader.SelectedDensity = density;

        // Spec 4.7: an estimate here (the mass service is only asked in the flight scene).
        string stability = FrameStabilityEstimate.Warn(selectedPath, density, CustomFrameLoader.SelectedNodes);
        SetStatus($"Stored {Path.GetFileName(selectedPath)} at {density.ToString(CultureInfo.InvariantCulture)} kg/m^3." +
                  (stability == null ? "" : " " + stability));
    }

    // ================================================================
    // Presets
    // ================================================================

    /// <summary>
    /// Fills the path, density and node rows from the chosen preset. Writes only to this panel's
    /// fields: the CustomFrameLoader statics change only when the student presses Build Drone.
    /// </summary>
    public void OnPresetSelected(int index)
    {
        if (index <= 0 || index > presets.Count)
            return;

        DronePreset preset = presets[index - 1];

        if (nodePlacement == null)
        {
            Debug.LogError("[ImportPageUI] nodePlacement is not assigned - cannot fill the node table.", this);
            SetStatus("Could not fill the node table - see console.");
            return;
        }

        // Rows first: if the preset does not fit the table, nothing on the panel changes.
        if (!nodePlacement.TryShowNodes(preset.nodes, out string nodeMessage))
        {
            SetStatus($"Preset '{preset.name}' could not be loaded: {nodeMessage}");
            return;
        }

        selectedPresetName = preset.name;
        selectedPath = preset.stlPath;
        if (selectedPathText != null)
            selectedPathText.text = $"File Path: {selectedPath}";
        if (densityInput != null)
            densityInput.text = preset.density.ToString("R", CultureInfo.InvariantCulture);
        if (presetNameInput != null)
            presetNameInput.text = preset.name;

        // The STL path is absolute, so a moved or renamed file is expected: say so here rather
        // than only when Build Drone is pressed.
        SetStatus(File.Exists(selectedPath)
            ? $"Loaded preset '{preset.name}' - press Build Drone to use it."
            : $"Preset '{preset.name}': {Path.GetFileName(selectedPath)} was not found. Browse to it, then save the preset again.");
    }

    /// <summary>
    /// Saves the STL path, density and shown node rows under the typed name, replacing a preset
    /// of the same name. Uses the same checks as Build Drone, so a preset always builds.
    /// </summary>
    public void OnSavePresetClicked()
    {
        string presetName = presetNameInput != null ? presetNameInput.text.Trim() : string.Empty;
        if (presetName.Length == 0)
        {
            SetStatus("Type a name for the preset, then press Save Preset.");
            return;
        }

        if (presetName.Length > DronePresetStore.MaxNameLength)
        {
            SetStatus($"Preset names can be at most {DronePresetStore.MaxNameLength} characters.");
            return;
        }

        if (string.IsNullOrEmpty(selectedPath))
        {
            SetStatus("Select an STL file before saving a preset.");
            return;
        }

        if (!File.Exists(selectedPath))
        {
            SetStatus($"{Path.GetFileName(selectedPath)} was not found - browse to it again before saving.");
            return;
        }

        if (!TryReadDensity(out float density))
        {
            SetStatus("Density must be a positive number.");
            return;
        }

        if (nodePlacement == null)
        {
            Debug.LogError("[ImportPageUI] nodePlacement is not assigned - cannot read the node table.", this);
            SetStatus("Could not read the node table - see console.");
            return;
        }

        // The reason is already shown under the table by the live validation.
        if (!nodePlacement.TryGetValidNodes(out List<CustomFrameLoader.NodeData> nodes, out _))
        {
            SetStatus("Fix the node table before saving - see the message below it.");
            return;
        }

        var preset = new DronePreset
        {
            name = presetName,
            stlPath = selectedPath,
            density = density
        };
        foreach (CustomFrameLoader.NodeData node in nodes)
        {
            preset.nodes.Add(new DronePresetNode
            {
                type = (int)node.type,
                x = node.position.x,
                y = node.position.y,
                z = node.position.z,
                massKg = node.massKg
            });
        }

        if (!DronePresetStore.Save(preset, out bool replaced, out string error))
        {
            SetStatus($"Could not save the preset: {error}");
            return;
        }

        selectedPresetName = preset.name;
        RefreshPresetDropdown(selectedPresetName);
        SetStatus(replaced
            ? $"Updated preset '{preset.name}' ({nodes.Count} nodes)."
            : $"Saved preset '{preset.name}' ({nodes.Count} nodes).");
    }

    /// <summary>
    /// Re-reads the preset file and rebuilds the dropdown's option list (ClearOptions/AddOptions on
    /// the hand-built dropdown), selecting the named preset if it still exists. Does not fire
    /// OnPresetSelected.
    /// </summary>
    private void RefreshPresetDropdown(string selectName)
    {
        presets = DronePresetStore.LoadAll();

        if (presetDropdown == null)
            return;

        var options = new List<string>
        {
            presets.Count == 0 ? "(no saved presets)" : "Choose a preset..."
        };
        foreach (DronePreset preset in presets)
            options.Add(preset.name);

        presetDropdown.ClearOptions();
        presetDropdown.AddOptions(options);

        int index = presets.FindIndex(p => DronePresetStore.SameName(p.name, selectName)) + 1;
        if (index == 0)
            selectedPresetName = null;
        presetDropdown.SetValueWithoutNotify(index);
        presetDropdown.RefreshShownValue();
    }

    private void SetStatus(string message)
    {
        pageMessage = message;
        RefreshStatus();
    }

    private void RefreshStatus()
    {
        MassServiceLauncher.Status service = MassServiceLauncher.Current;
        shownServiceVersion = service.version;

        if (statusText == null)
            return;

        string serviceLine = service.state == MassServiceLauncher.State.NotStarted
            ? string.Empty
            : $"Mass service: {service.message}";

        if (string.IsNullOrEmpty(pageMessage))
            statusText.text = serviceLine;
        else if (string.IsNullOrEmpty(serviceLine))
            statusText.text = pageMessage;
        else
            statusText.text = $"{pageMessage}\n{serviceLine}";
    }

    private string DescribeSelection(string path)
    {
        string fileName = Path.GetFileName(path);
        return TryReadDensity(out float density)
            ? $"{fileName} - density {density.ToString(CultureInfo.InvariantCulture)} kg/m^3 will be used."
            : $"{fileName} - enter a positive density.";
    }

    private bool TryReadDensity(out float density)
    {
        density = 0f;
        return densityInput != null &&
               float.TryParse(densityInput.text, NumberStyles.Float, CultureInfo.InvariantCulture, out density) &&
               density > 0f;
    }
}

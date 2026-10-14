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

    // The picked path is tracked here rather than read back out of selectedPathText, since that
    // label carries placeholder text ("File Path: ...") that would otherwise read as a valid
    // selection during confirm-time validation.
    private string selectedPath;

    // statusText carries two things: the page's own message, and a second line with the mass
    // service state. Both are kept here and recombined by SetStatus/RefreshStatus so neither
    // overwrites the other.
    private string pageMessage = string.Empty;
    private int shownServiceVersion = -1;

    private void OnEnable()
    {
        // Started here rather than in CustomFrameLoader.Awake(): the bundled interpreter takes
        // about a second to come up, and Awake() gives the service only 5 s to answer. Only the
        // custom-frame path pays for it; default-drone students never open this panel.
        MassServiceLauncher.EnsureRunning();

        // State is restored from the statics rather than blanked, so closing the panel with Back
        // and reopening it does not lose the selected STL.
        selectedPath = string.IsNullOrEmpty(CustomFrameLoader.SelectedSTLPath)
            ? null
            : CustomFrameLoader.SelectedSTLPath;

        if (selectedPathText != null)
            selectedPathText.text = selectedPath == null ? NoPathLabel : $"File Path: {selectedPath}";

        SetStatus(selectedPath == null ? string.Empty : DescribeSelection(selectedPath));
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

        SetStatus($"Stored {Path.GetFileName(selectedPath)} at {density.ToString(CultureInfo.InvariantCulture)} kg/m^3.");
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

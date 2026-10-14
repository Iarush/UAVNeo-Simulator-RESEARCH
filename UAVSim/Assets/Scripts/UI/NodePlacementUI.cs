using System;
using System.Collections.Generic;
using System.Globalization;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Binds the hand-built node table on the custom drone panel: rows of typed values, inside a
/// scroll view, that populate <see cref="CustomFrameLoader.SelectedNodes"/>.
/// <see cref="ImportPageUI.OnBuildDroneClicked"/> calls <see cref="TryStoreNodes"/>.
///
/// Every row that can ever be shown is built in the Editor under <see cref="nodeListContainer"/>.
/// Add Row / Remove Row only SetActive them, always keeping the shown rows as one block at the top,
/// so the hand-built child count is the row cap. Hidden rows are cleared and ignored.
///
/// This script never creates, instantiates or destroys a GameObject. It caches references to
/// existing children, reads and writes their values, and calls SetActive.
///
/// Typed positions are in the STL/CAD Z-up frame in metres (+X right, +Y forward, +Z up), which
/// is exactly the frame CustomFrameLoader.SelectedNodes expects - no conversion is applied.
/// </summary>
public class NodePlacementUI : MonoBehaviour
{
    private const string ConfirmationMessage =
        "Drone configured - press Back, then Begin to fly.";

    /// <summary>
    /// Remove Row stops here: the pipeline is quadcopter-only, so a valid layout always needs at
    /// least the four motor rows.
    /// </summary>
    private const int MinShownRows = 4;

    public Transform nodeListContainer;
    public TMP_Text infoText;
    public Button buildDroneButton;
    public Button addRowButton;
    public Button removeRowButton;

    /// <summary>
    /// Optional multi-line field for pasting a whole table of nodes. Rows are written straight into
    /// the hand-built rows above (dropdown + four input fields), so the paste path and the typed
    /// path share one validator and one store: <see cref="TryCollectNodes"/>.
    /// </summary>
    public TMP_InputField bulkPasteInput;

    private class Row
    {
        public GameObject Root;
        public TMP_Dropdown Type;
        public TMP_InputField X;
        public TMP_InputField Y;
        public TMP_InputField Z;
        public TMP_InputField Mass;

        public bool IsBound => Type != null && X != null && Y != null && Z != null && Mass != null;
    }

    private readonly List<Row> rows = new List<Row>();

    // True while the post-Build confirmation is displayed. Any field edit clears it, so the
    // confirmation can never describe values that differ from what was actually stored.
    private bool showingConfirmation;
    private bool started;

    private void Awake()
    {
        if (buildDroneButton != null)
            buildDroneButton.interactable = false;

        BindRows();
    }

    private void Start()
    {
        AssertDropdownOptions();
        RegisterFieldListeners();
        started = true;
        RefreshRowButtons();
        RefreshValidation();
    }

    private void OnEnable()
    {
        // Deliberately does NOT clear any input: typed values must survive closing the panel with
        // Back and reopening it.
        if (started && !showingConfirmation)
            RefreshValidation();
    }

    // ================================================================
    // Binding
    // ================================================================

    private void BindRows()
    {
        rows.Clear();

        if (nodeListContainer == null)
        {
            Debug.LogError("[NodePlacementUI] nodeListContainer is not assigned.", this);
            return;
        }

        foreach (Transform child in nodeListContainer)
        {
            rows.Add(new Row
            {
                Root = child.gameObject,
                Type = FindControl<TMP_Dropdown>(child, "TypeDropdown"),
                X = FindControl<TMP_InputField>(child, "XPosInput"),
                Y = FindControl<TMP_InputField>(child, "YPosInput"),
                Z = FindControl<TMP_InputField>(child, "ZPosInput"),
                Mass = FindControl<TMP_InputField>(child, "MassInput")
            });
        }
    }

    /// <summary>
    /// Errors are logged with the row as the context object, so clicking the console entry pings
    /// the offending row in the hierarchy.
    /// </summary>
    private static T FindControl<T>(Transform row, string childName) where T : Component
    {
        Transform found = FindDescendant(row, childName);
        if (found == null)
        {
            Debug.LogError($"[NodePlacementUI] Row '{row.name}' has no child named '{childName}'.", row.gameObject);
            return null;
        }

        T component = found.GetComponent<T>();
        if (component == null)
            Debug.LogError($"[NodePlacementUI] '{childName}' on row '{row.name}' has no {typeof(T).Name}.", found.gameObject);

        return component;
    }

    private static Transform FindDescendant(Transform parent, string childName)
    {
        Transform direct = parent.Find(childName);
        if (direct != null)
            return direct;

        foreach (Transform child in parent)
        {
            Transform found = FindDescendant(child, childName);
            if (found != null)
                return found;
        }
        return null;
    }

    /// <summary>
    /// The dropdown's selected index is cast straight to NodeType and written to the socket as a
    /// raw byte (CustomFrameLoader.cs:233), so options that don't match the enum order silently
    /// send the wrong node type with no runtime error.
    /// </summary>
    private void AssertDropdownOptions()
    {
        // Display labels indexed by NodeType value: 0 = Motor, 1 = Battery,
        // 2 = FlightController, 3 = OffboardComputer. Labels match the enum identifiers apart
        // from spacing - what must hold is the ORDER, since the selected index is cast straight
        // to NodeType and written to the socket as a raw byte.
        string[] expected = { "Motor", "Battery", "Flight Controller", "Offboard Computer" };

        foreach (Row row in rows)
        {
            if (row.Type == null)
                continue;

            List<TMP_Dropdown.OptionData> options = row.Type.options;
            if (options.Count != expected.Length)
            {
                Debug.LogError(
                    $"[NodePlacementUI] Row '{row.Root.name}' TypeDropdown has {options.Count} options, expected " +
                    $"{expected.Length} in order: {string.Join(", ", expected)}. The selected index is written to " +
                    "the wire protocol as a raw byte, so a mismatch silently corrupts node types.", row.Root);
                continue;
            }

            for (int i = 0; i < expected.Length; i++)
            {
                if (!string.Equals(options[i].text, expected[i], StringComparison.Ordinal))
                {
                    Debug.LogError(
                        $"[NodePlacementUI] Row '{row.Root.name}' TypeDropdown option {i} is '{options[i].text}', " +
                        $"expected '{expected[i]}'. The selected index is written to the wire protocol as a raw " +
                        "byte, so a mismatch silently corrupts node types.", row.Root);
                }
            }
        }
    }

    private void RegisterFieldListeners()
    {
        foreach (Row row in rows)
        {
            if (!row.IsBound)
                continue;

            row.Type.onValueChanged.AddListener(_ => HandleFieldChanged());
            row.X.onValueChanged.AddListener(_ => HandleFieldChanged());
            row.Y.onValueChanged.AddListener(_ => HandleFieldChanged());
            row.Z.onValueChanged.AddListener(_ => HandleFieldChanged());
            row.Mass.onValueChanged.AddListener(_ => HandleFieldChanged());
        }
    }

    private void HandleFieldChanged()
    {
        // Edited values invalidate any stored-configuration confirmation immediately.
        showingConfirmation = false;
        RefreshValidation();
    }

    // ================================================================
    // Validation
    // ================================================================

    private void RefreshValidation()
    {
        bool valid = TryCollectNodes(out List<CustomFrameLoader.NodeData> nodes, out string message);
        if (valid)
            valid = MotorQuadrants.TryValidateMotorLayout(nodes, out message);

        if (buildDroneButton != null)
            buildDroneButton.interactable = valid;

        if (showingConfirmation)
            return;

        if (infoText != null)
            infoText.text = valid ? string.Empty : message;
    }

    /// <summary>
    /// Parses every shown row; hidden rows are ignored. A row with all four numeric fields blank is
    /// skipped silently; a row with any field filled must have all four valid, with mass strictly
    /// greater than zero.
    /// </summary>
    private bool TryCollectNodes(out List<CustomFrameLoader.NodeData> nodes, out string message)
    {
        nodes = new List<CustomFrameLoader.NodeData>();
        message = string.Empty;

        foreach (Row row in rows)
        {
            if (row.Root != null && !row.Root.activeSelf)
                continue;

            string rowName = row.Root != null ? row.Root.name : "(unnamed row)";

            if (!row.IsBound)
            {
                message = $"{rowName} is missing one or more controls - see the console.";
                return false;
            }

            bool anyFilled =
                !string.IsNullOrWhiteSpace(row.X.text) ||
                !string.IsNullOrWhiteSpace(row.Y.text) ||
                !string.IsNullOrWhiteSpace(row.Z.text) ||
                !string.IsNullOrWhiteSpace(row.Mass.text);

            if (!anyFilled)
                continue;

            // Sequential rather than a ?? chain: short-circuiting would leave the later out
            // variables unassigned on the error paths.
            message = ReadNumber(row.X.text, "X", rowName, out float x);
            if (message != null) return false;

            message = ReadNumber(row.Y.text, "Y", rowName, out float y);
            if (message != null) return false;

            message = ReadNumber(row.Z.text, "Z", rowName, out float z);
            if (message != null) return false;

            message = ReadNumber(row.Mass.text, "mass", rowName, out float mass);
            if (message != null) return false;

            if (mass <= 0f)
            {
                message = $"{rowName}: mass must be greater than 0.";
                return false;
            }

            nodes.Add(new CustomFrameLoader.NodeData
            {
                type = (CustomFrameLoader.NodeType)row.Type.value,
                // Already in the STL Z-up CAD frame, in metres - stored verbatim.
                position = new Vector3(x, y, z),
                massKg = mass
            });
        }

        message = string.Empty;

        for (int i = 0; i < nodes.Count; i++)
        {
            for (int j = i + 1; j < nodes.Count; j++)
            {
                if (nodes[i].position == nodes[j].position)
                {
                    Vector3 p = nodes[i].position;
                    message = $"Two nodes share the same position ({p.x:F3}, {p.y:F3}, {p.z:F3}).";
                    return false;
                }
            }
        }

        return true;
    }

    /// <summary>Returns null when the field is valid, or the error message to display.</summary>
    private static string ReadNumber(string text, string label, string rowName, out float value)
    {
        value = 0f;

        if (string.IsNullOrWhiteSpace(text))
            return $"{rowName}: {label} is empty - fill every field in a row, or clear the whole row.";

        if (!float.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value))
            return $"{rowName}: {label} is not a number.";

        return null;
    }

    // ================================================================
    // Buttons
    // ================================================================

    /// <summary>
    /// Stores the node rows into <see cref="CustomFrameLoader.SelectedNodes"/>, or reports why they
    /// are invalid and leaves the list untouched. Called by Build Drone via ImportPageUI, which
    /// stores the path and density only when this returns true. Deliberately does NOT start the
    /// simulation - the contract is populate the statics, then load the flight scene, which
    /// happens when the user presses Begin from the main menu.
    /// </summary>
    public bool TryStoreNodes()
    {
        bool valid = TryCollectNodes(out List<CustomFrameLoader.NodeData> nodes, out string message);
        if (valid)
            valid = MotorQuadrants.TryValidateMotorLayout(nodes, out message);

        if (!valid)
        {
            showingConfirmation = false;
            if (infoText != null)
                infoText.text = message;
            if (buildDroneButton != null)
                buildDroneButton.interactable = false;
            return false;
        }

        CustomFrameLoader.SelectedNodes.Clear();
        foreach (CustomFrameLoader.NodeData node in nodes)
            CustomFrameLoader.SelectedNodes.Add(node);

        showingConfirmation = true;
        if (infoText != null)
            infoText.text = ConfirmationMessage;
        return true;
    }

    /// <summary>
    /// Parses the pasted table in <see cref="bulkPasteInput"/> into the existing rows. Column order
    /// matches the table on screen: Type, X, Y, Z, Mass - one node per line, comma- or
    /// tab-separated. Type accepts the dropdown label (spacing/case ignored), "FC"/"Offboard", or
    /// the raw NodeType index. The shown rows are resized to the pasted count (never below
    /// <see cref="MinShownRows"/>, which leaves any extra shown rows blank, i.e. "no node").
    /// Nothing is stored here: the rows fire their onValueChanged listeners exactly as if the user
    /// had typed, and Build Drone remains the only writer of SelectedNodes.
    /// </summary>
    public void OnApplyPasteClicked()
    {
        showingConfirmation = false;

        if (bulkPasteInput == null)
        {
            Debug.LogError("[NodePlacementUI] bulkPasteInput is not assigned.", this);
            return;
        }

        string[] lines = bulkPasteInput.text
            .Split(new[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);

        var parsed = new List<(int type, string x, string y, string z, string mass)>();
        foreach (string rawLine in lines)
        {
            string line = rawLine.Trim();
            if (line.Length == 0)
                continue;

            string[] cols = line.Split(new[] { '\t', ',' });
            for (int i = 0; i < cols.Length; i++)
                cols[i] = cols[i].Trim();

            if (cols.Length != 5)
            {
                Report($"Paste line {parsed.Count + 1}: expected 5 columns (Type, X, Y, Z, Mass), found {cols.Length}.");
                return;
            }

            // A header row ("Type, X, Y, Z, Mass") is tolerated only as the first line.
            if (parsed.Count == 0 && !TryParseNodeType(cols[0], out _) && !IsNumber(cols[1]))
                continue;

            if (!TryParseNodeType(cols[0], out int type))
            {
                Report($"Paste line {parsed.Count + 1}: unknown type '{cols[0]}'. Use Motor, Battery, Flight Controller or Offboard Computer.");
                return;
            }

            for (int i = 1; i < 5; i++)
            {
                if (!IsNumber(cols[i]))
                {
                    Report($"Paste line {parsed.Count + 1}: '{cols[i]}' is not a number.");
                    return;
                }
            }

            parsed.Add((type, cols[1], cols[2], cols[3], cols[4]));
        }

        if (parsed.Count == 0)
        {
            Report("Paste one node per line: Type, X, Y, Z, Mass (comma or tab separated).");
            return;
        }

        if (parsed.Count > rows.Count)
        {
            Report($"Pasted {parsed.Count} nodes but the table holds at most {rows.Count} rows.");
            return;
        }

        int shown = Mathf.Max(parsed.Count, MinShownRows);
        for (int i = 0; i < rows.Count; i++)
        {
            Row row = rows[i];
            if (!row.IsBound)
                continue;

            if (i < parsed.Count)
            {
                row.Type.value = parsed[i].type;
                row.X.text = parsed[i].x;
                row.Y.text = parsed[i].y;
                row.Z.text = parsed[i].z;
                row.Mass.text = parsed[i].mass;
            }
            else
            {
                ClearRow(row);
            }

            row.Root.SetActive(i < shown);
        }

        RefreshRowButtons();

        // The field writes above already re-validated through the onValueChanged listeners, but
        // before the SetActive calls; this pass sees the final shown rows, and then decides whether
        // to replace an empty (valid) message with a summary.
        RefreshValidation();
        if (buildDroneButton != null && buildDroneButton.interactable && infoText != null)
            infoText.text = $"Filled {parsed.Count} row(s) from the pasted table - check the values, then press Build Drone.";
    }

    // ================================================================
    // Add / remove rows
    // ================================================================

    /// <summary>Shows the first hidden row, blank.</summary>
    public void OnAddRowClicked()
    {
        Row row = rows.Find(r => r.Root != null && !r.Root.activeSelf);
        if (row == null)
            return;

        if (row.IsBound)
            ClearRow(row);
        row.Root.SetActive(true);

        RefreshRowButtons();
        HandleFieldChanged();
    }

    /// <summary>Clears and hides the last shown row, never going below <see cref="MinShownRows"/>.</summary>
    public void OnRemoveRowClicked()
    {
        if (CountShownRows() <= MinShownRows)
            return;

        Row row = rows.FindLast(r => r.Root != null && r.Root.activeSelf);
        // Cleared as well as hidden, so a later Add Row never brings back stale values.
        if (row.IsBound)
            ClearRow(row);
        row.Root.SetActive(false);

        RefreshRowButtons();
        HandleFieldChanged();
    }

    private int CountShownRows()
    {
        int shown = 0;
        foreach (Row row in rows)
        {
            if (row.Root != null && row.Root.activeSelf)
                shown++;
        }
        return shown;
    }

    private void RefreshRowButtons()
    {
        int shown = CountShownRows();
        if (addRowButton != null)
            addRowButton.interactable = shown < rows.Count;
        if (removeRowButton != null)
            removeRowButton.interactable = shown > MinShownRows;
    }

    private static void ClearRow(Row row)
    {
        row.Type.value = (int)CustomFrameLoader.NodeType.Motor;
        row.X.text = string.Empty;
        row.Y.text = string.Empty;
        row.Z.text = string.Empty;
        row.Mass.text = string.Empty;
    }

    private void Report(string message)
    {
        if (infoText != null)
            infoText.text = message;
    }

    private static bool IsNumber(string text)
    {
        return float.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out _);
    }

    /// <summary>
    /// Maps a pasted type cell to the dropdown index, which is the NodeType value written to the
    /// wire (see AssertDropdownOptions). Accepts labels, common short forms, or the index itself.
    /// </summary>
    private static bool TryParseNodeType(string text, out int type)
    {
        string key = text.Replace(" ", string.Empty).Replace("_", string.Empty).Replace("-", string.Empty).ToLowerInvariant();
        switch (key)
        {
            case "0": case "motor": type = (int)CustomFrameLoader.NodeType.Motor; return true;
            case "1": case "battery": type = (int)CustomFrameLoader.NodeType.Battery; return true;
            case "2": case "flightcontroller": case "fc": type = (int)CustomFrameLoader.NodeType.FlightController; return true;
            case "3": case "offboardcomputer": case "offboard": case "companioncomputer": type = (int)CustomFrameLoader.NodeType.OffboardComputer; return true;
        }
        type = -1;
        return false;
    }
}

using System.Collections.Generic;
using System.Globalization;
using SimpleFileBrowser;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Custom Drone Builder wizard: pick an STL, place mass nodes on it (motor/battery/flight
/// controller/companion computer), then hand off to CustomFrameLoader and start flying it.
/// The whole UI is built procedurally in Awake() rather than as a hand-authored prefab, since
/// the mesh preview it shows is inherently data-driven (depends on whatever STL gets picked).
/// </summary>
public class CustomDroneBuilderUI : MonoBehaviour
{
    private const float DefaultDensity = 1250f;
    // Positioned far from the origin so the preview camera/mesh/collider can never overlap
    // anything else in whatever scene this UI happens to live in.
    private static readonly Vector3 PreviewWorldOrigin = new Vector3(0f, 20000f, 0f);
    private const int PreviewTextureSize = 512;

    private struct PlacedNode
    {
        public CustomFrameLoader.NodeType Type;
        public float MassKg;
        public Vector3 LocalPosition; // Same local space as the preview mesh's own vertices (Y-up, meters).
        public GameObject Marker;
        public GameObject ListRow;
    }

    private MainMenu mainMenu;

    // Pages
    private GameObject importPage;
    private GameObject nodePlacementPage;
    private GameObject nodePopup;

    // Import page widgets
    private Text importStatusText;
    private InputField densityInput;
    private Button nextButton;

    // Node placement page widgets
    private RawImage previewImage;
    private Camera previewCamera;
    private RenderTexture previewTexture;
    private Transform previewRoot;
    private GameObject previewMeshObject;
    private MeshFilter previewMeshFilter;
    private MeshCollider previewMeshCollider;
    private Transform nodeListContent;
    private Text validationWarningText;

    // Node popup widgets
    private Dropdown nodeTypeDropdown;
    private InputField nodeMassInput;
    private Vector3 pendingNodeLocalPosition;

    private string selectedStlPath;
    private readonly List<PlacedNode> nodes = new List<PlacedNode>();

    private bool uiBuilt;

    private void Awake()
    {
        mainMenu = GetComponentInParent<MainMenu>();
        EnsureUIBuilt();
    }

    private void OnEnable()
    {
        // Defensive: OnEnable should always run after Awake has built the UI, but if a stale
        // domain-reload/editor-state ever puts them out of order, build first rather than
        // NullReferenceException on unbuilt widgets (see ResetWizard/RefreshValidationWarning).
        EnsureUIBuilt();
        ResetWizard();
    }

    private void EnsureUIBuilt()
    {
        if (uiBuilt)
            return;
        // Set the flag AFTER BuildUI() succeeds, not before - if it throws partway through,
        // this must stay false so the next OnEnable() retries instead of permanently skipping
        // the rest of construction with the flag wrongly marked "done".
        BuildUI();
        uiBuilt = true;
    }

    private void OnDisable()
    {
        if (previewCamera != null)
            previewCamera.enabled = false;
    }

    public void Close()
    {
        gameObject.SetActive(false);
    }

    // ================================================================
    // Wizard state
    // ================================================================

    private void ResetWizard()
    {
        // Switch to the Import page FIRST - if anything below throws, the user still ends up
        // looking at a usable page instead of whatever page was active before this call.
        ShowImportPage();

        selectedStlPath = null;
        ClearNodes();
        if (previewMeshFilter != null)
            previewMeshFilter.sharedMesh = null;
        if (previewMeshCollider != null)
            previewMeshCollider.sharedMesh = null;

        importStatusText.text = "No file selected.";
        densityInput.text = DefaultDensity.ToString(CultureInfo.InvariantCulture);
        nextButton.interactable = false;
    }

    private void ClearNodes()
    {
        foreach (PlacedNode node in nodes)
        {
            if (node.Marker != null) Destroy(node.Marker);
            if (node.ListRow != null) Destroy(node.ListRow);
        }
        nodes.Clear();
        RefreshValidationWarning();
    }

    private void ShowImportPage()
    {
        importPage.SetActive(true);
        nodePlacementPage.SetActive(false);
        nodePopup.SetActive(false);
        if (previewCamera != null) previewCamera.enabled = false;
    }

    private void ShowNodePlacementPage()
    {
        importPage.SetActive(false);
        nodePlacementPage.SetActive(true);
        nodePopup.SetActive(false);
        if (previewCamera != null) previewCamera.enabled = true;
    }

    // ================================================================
    // Import page behavior
    // ================================================================

    private void BrowseForStl()
    {
        FileBrowser.SetFilters(true, new FileBrowser.Filter("STL Files", ".stl"));
        FileBrowser.SetDefaultFilter(".stl");
        FileBrowser.ShowLoadDialog(
            OnStlPicked,
            () => { },
            FileBrowser.PickMode.Files,
            false, null, null,
            "Select Drone STL", "Select");
    }

    private void OnStlPicked(string[] paths)
    {
        if (paths == null || paths.Length == 0)
            return;

        string path = paths[0];
        if (!StlMeshLoader.TryLoadStl(path, out Mesh mesh))
        {
            importStatusText.text = $"Could not load '{System.IO.Path.GetFileName(path)}' - see console for details.";
            nextButton.interactable = false;
            return;
        }

        // A new mesh invalidates any previously-placed node positions.
        ClearNodes();

        selectedStlPath = path;
        importStatusText.text = $"Selected: {System.IO.Path.GetFileName(path)}";

        if (previewMeshFilter.sharedMesh != null)
            Destroy(previewMeshFilter.sharedMesh);
        previewMeshFilter.sharedMesh = mesh;
        previewMeshCollider.sharedMesh = mesh;

        FramePreviewCamera(mesh.bounds);
        nextButton.interactable = true;
    }

    private void FramePreviewCamera(Bounds bounds)
    {
        float radius = Mathf.Max(bounds.extents.magnitude, 0.01f);
        float distance = radius / Mathf.Sin(previewCamera.fieldOfView * 0.5f * Mathf.Deg2Rad) * 1.4f;

        previewMeshObject.transform.localPosition = -bounds.center;
        previewCamera.transform.localPosition = new Vector3(0f, 0f, -distance);
        previewCamera.nearClipPlane = Mathf.Max(distance - radius * 2f, 0.01f);
        previewCamera.farClipPlane = distance + radius * 2f;
    }

    // ================================================================
    // Node placement page behavior
    // ================================================================

    private void HandlePreviewSurfaceClicked(Vector3 worldPoint)
    {
        pendingNodeLocalPosition = previewRoot.InverseTransformPoint(worldPoint);
        nodeTypeDropdown.value = 0;
        nodeMassInput.text = "0.05";
        nodePopup.SetActive(true);
    }

    private void ConfirmNodePopup()
    {
        if (!float.TryParse(nodeMassInput.text, NumberStyles.Float, CultureInfo.InvariantCulture, out float massKg) || massKg <= 0f)
        {
            Debug.LogWarning("[CustomDroneBuilderUI] Node mass must be a positive number.");
            return;
        }

        CustomFrameLoader.NodeType type = (CustomFrameLoader.NodeType)nodeTypeDropdown.value;
        AddNode(type, massKg, pendingNodeLocalPosition);
        nodePopup.SetActive(false);
    }

    private void CancelNodePopup()
    {
        nodePopup.SetActive(false);
    }

    private void AddNode(CustomFrameLoader.NodeType type, float massKg, Vector3 localPosition)
    {
        GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        Destroy(marker.GetComponent<Collider>());
        marker.transform.SetParent(previewRoot, false);
        marker.transform.localPosition = localPosition;
        marker.transform.localScale = Vector3.one * (previewMeshFilter.sharedMesh.bounds.extents.magnitude * 0.05f + 0.01f);
        marker.GetComponent<MeshRenderer>().sharedMaterial = new Material(Shader.Find("Unlit/Color")) { color = ColorForNodeType(type) };

        PlacedNode node = new PlacedNode
        {
            Type = type,
            MassKg = massKg,
            LocalPosition = localPosition,
            Marker = marker
        };
        node.ListRow = BuildNodeListRow(nodes.Count, type, massKg);
        nodes.Add(node);
        RefreshValidationWarning();
    }

    private void DeleteNode(int index)
    {
        if (index < 0 || index >= nodes.Count)
            return;

        PlacedNode node = nodes[index];
        if (node.Marker != null) Destroy(node.Marker);
        if (node.ListRow != null) Destroy(node.ListRow);
        nodes.RemoveAt(index);

        // Row click handlers were bound to indices, which just shifted - rebuild the list.
        RebuildNodeListRows();
        RefreshValidationWarning();
    }

    private void RebuildNodeListRows()
    {
        for (int i = 0; i < nodes.Count; i++)
        {
            if (nodes[i].ListRow != null) Destroy(nodes[i].ListRow);
            PlacedNode node = nodes[i];
            node.ListRow = BuildNodeListRow(i, node.Type, node.MassKg);
            nodes[i] = node;
        }
    }

    private void RefreshValidationWarning()
    {
        if (validationWarningText == null)
        {
            Debug.LogWarning("[CustomDroneBuilderUI] RefreshValidationWarning called before validationWarningText was built - skipping.");
            return;
        }

        bool hasBattery = false;
        foreach (PlacedNode node in nodes)
        {
            if (node.Type == CustomFrameLoader.NodeType.Battery) { hasBattery = true; break; }
        }
        validationWarningText.gameObject.SetActive(!hasBattery && nodes.Count > 0);
        validationWarningText.text = "Warning: no Battery node placed.";
    }

    private static Color ColorForNodeType(CustomFrameLoader.NodeType type)
    {
        switch (type)
        {
            case CustomFrameLoader.NodeType.Motor: return Color.red;
            case CustomFrameLoader.NodeType.Battery: return Color.green;
            case CustomFrameLoader.NodeType.FlightController: return Color.cyan;
            default: return Color.yellow;
        }
    }

    // ================================================================
    // Confirm
    // ================================================================

    private void OnConfirmClicked()
    {
        if (nodes.Count == 0)
        {
            Debug.LogWarning("[CustomDroneBuilderUI] Place at least one node before confirming.");
            validationWarningText.gameObject.SetActive(true);
            validationWarningText.text = "Place at least one node before confirming.";
            return;
        }

        CustomFrameLoader.SelectedNodes.Clear();
        foreach (PlacedNode node in nodes)
        {
            CustomFrameLoader.SelectedNodes.Add(new CustomFrameLoader.NodeData
            {
                type = node.Type,
                position = StlMeshLoader.ConvertYUpToZUp(node.LocalPosition),
                massKg = node.MassKg
            });
        }

        CustomFrameLoader.SelectedSTLPath = selectedStlPath;
        CustomFrameLoader.SelectedDensity =
            float.TryParse(densityInput.text, NumberStyles.Float, CultureInfo.InvariantCulture, out float density) && density > 0f
                ? density
                : DefaultDensity;

        if (mainMenu == null)
        {
            Debug.LogError("[CustomDroneBuilderUI] No MainMenu found in parent hierarchy - cannot start the simulation.");
            return;
        }

        mainMenu.BeginSimulation();
    }

    // ================================================================
    // UI construction
    // ================================================================

    private void BuildUI()
    {
        Debug.Log($"[CustomDroneBuilderUI] BuildUI() START - instance={GetInstanceID()} frame={Time.frameCount} existingChildCount={transform.childCount}");

        RectTransform root = (RectTransform)transform;
        root.anchorMin = Vector2.zero;
        root.anchorMax = Vector2.one;
        root.offsetMin = Vector2.zero;
        root.offsetMax = Vector2.zero;

        Image background = gameObject.AddComponent<Image>();
        background.color = new Color(0f, 0f, 0f, 0.85f);

        BuildImportPage();
        BuildNodePlacementPage();
        BuildNodePopup();
        BuildPreviewScene();

        Debug.Log($"[CustomDroneBuilderUI] BuildUI() END - instance={GetInstanceID()} frame={Time.frameCount} finalChildCount={transform.childCount}");
    }

    private void BuildImportPage()
    {
        importPage = CreateStretchPanel(transform, "ImportPage");
        VerticalLayoutGroup layout = importPage.AddComponent<VerticalLayoutGroup>();
        layout.childAlignment = TextAnchor.UpperCenter;
        layout.spacing = 12f;
        layout.padding = new RectOffset(40, 40, 40, 40);
        layout.childForceExpandHeight = false;

        CreateText(importPage.transform, "Title", "Custom Drone Builder - Import STL", 22, TextAnchor.MiddleCenter);
        CreateButton(importPage.transform, "BrowseButton", "Browse for STL...", BrowseForStl);
        importStatusText = CreateText(importPage.transform, "StatusText", "No file selected.", 16, TextAnchor.MiddleCenter);

        GameObject densityRow = CreateHorizontalRow(importPage.transform, "DensityRow");
        CreateText(densityRow.transform, "DensityLabel", "Material Density (kg/m^3):", 16, TextAnchor.MiddleLeft);
        densityInput = CreateInputField(densityRow.transform, "DensityInput", DefaultDensity.ToString(CultureInfo.InvariantCulture), InputField.ContentType.DecimalNumber);

        GameObject buttonRow = CreateHorizontalRow(importPage.transform, "ButtonRow");
        CreateButton(buttonRow.transform, "CancelButton", "Cancel", Close);
        nextButton = CreateButton(buttonRow.transform, "NextButton", "Next >", ShowNodePlacementPage);
        nextButton.interactable = false;
    }

    private void BuildNodePlacementPage()
    {
        nodePlacementPage = CreateStretchPanel(transform, "NodePlacementPage");
        HorizontalLayoutGroup layout = nodePlacementPage.AddComponent<HorizontalLayoutGroup>();
        layout.spacing = 16f;
        layout.padding = new RectOffset(24, 24, 24, 24);
        layout.childForceExpandWidth = false;

        // --- Left: preview ---
        GameObject previewColumn = CreateVerticalColumn(nodePlacementPage.transform, "PreviewColumn");
        CreateText(previewColumn.transform, "Instructions", "Drag to orbit. Click the mesh surface to place a node.", 14, TextAnchor.MiddleCenter);

        GameObject previewContainer = new GameObject("PreviewImage", typeof(RectTransform));
        previewContainer.transform.SetParent(previewColumn.transform, false);
        LayoutElement previewLayoutElement = previewContainer.AddComponent<LayoutElement>();
        previewLayoutElement.preferredWidth = PreviewTextureSize;
        previewLayoutElement.preferredHeight = PreviewTextureSize;
        previewImage = previewContainer.AddComponent<RawImage>();
        DronePreviewOrbit orbit = previewContainer.AddComponent<DronePreviewOrbit>();
        orbit.OnSurfaceClicked = HandlePreviewSurfaceClicked;

        // Stash for BuildPreviewScene to wire up once the camera/mesh exist.
        pendingOrbitComponent = orbit;

        CreateButton(previewColumn.transform, "BackButton", "< Back", ShowImportPage);

        // --- Right: node list + confirm ---
        GameObject listColumn = CreateVerticalColumn(nodePlacementPage.transform, "NodeListColumn");
        CreateText(listColumn.transform, "ListTitle", "Placed Nodes", 18, TextAnchor.MiddleCenter);

        GameObject scrollGO = new GameObject("NodeListScroll", typeof(RectTransform));
        scrollGO.transform.SetParent(listColumn.transform, false);
        LayoutElement scrollLayoutElement = scrollGO.AddComponent<LayoutElement>();
        scrollLayoutElement.flexibleHeight = 1f;
        scrollLayoutElement.minHeight = 200f;
        Image scrollBg = scrollGO.AddComponent<Image>();
        scrollBg.color = new Color(1f, 1f, 1f, 0.05f);
        ScrollRect scrollRect = scrollGO.AddComponent<ScrollRect>();
        scrollRect.horizontal = false;

        GameObject viewportGO = new GameObject("Viewport", typeof(RectTransform));
        viewportGO.transform.SetParent(scrollGO.transform, false);
        RectTransform viewportRT = (RectTransform)viewportGO.transform;
        viewportRT.anchorMin = Vector2.zero;
        viewportRT.anchorMax = Vector2.one;
        viewportRT.offsetMin = Vector2.zero;
        viewportRT.offsetMax = Vector2.zero;
        viewportGO.AddComponent<Image>().color = new Color(0f, 0f, 0f, 0.01f);
        viewportGO.AddComponent<Mask>().showMaskGraphic = false;

        GameObject contentGO = new GameObject("Content", typeof(RectTransform));
        contentGO.transform.SetParent(viewportGO.transform, false);
        RectTransform contentRT = (RectTransform)contentGO.transform;
        contentRT.anchorMin = new Vector2(0f, 1f);
        contentRT.anchorMax = new Vector2(1f, 1f);
        contentRT.pivot = new Vector2(0.5f, 1f);
        contentRT.offsetMin = Vector2.zero;
        contentRT.offsetMax = Vector2.zero;
        VerticalLayoutGroup contentLayout = contentGO.AddComponent<VerticalLayoutGroup>();
        contentLayout.childForceExpandHeight = false;
        contentLayout.spacing = 4f;
        ContentSizeFitter contentFitter = contentGO.AddComponent<ContentSizeFitter>();
        contentFitter.verticalFit = ContentSizeFitter.FitMode.PreferredSize;

        scrollRect.viewport = viewportRT;
        scrollRect.content = contentRT;
        nodeListContent = contentGO.transform;

        validationWarningText = CreateText(listColumn.transform, "ValidationWarning", "", 14, TextAnchor.MiddleCenter);
        validationWarningText.color = new Color(1f, 0.7f, 0.2f);
        validationWarningText.gameObject.SetActive(false);

        CreateButton(listColumn.transform, "ConfirmButton", "Confirm & Fly", OnConfirmClicked);

        // Hidden by default at construction time, not just later via ShowImportPage() -
        // otherwise this full-screen page stays active simultaneously with importPage and,
        // being the later sibling, renders on top and completely covers it.
        nodePlacementPage.SetActive(false);
    }

    private DronePreviewOrbit pendingOrbitComponent;

    private GameObject BuildNodeListRow(int index, CustomFrameLoader.NodeType type, float massKg)
    {
        GameObject row = CreateHorizontalRow(nodeListContent, $"Node_{index}");
        CreateText(row.transform, "Label", $"{type}  -  {massKg:F3} kg", 14, TextAnchor.MiddleLeft);
        CreateButton(row.transform, "DeleteButton", "Delete", () => DeleteNode(index));
        return row;
    }

    private void BuildNodePopup()
    {
        nodePopup = CreateStretchPanel(transform, "NodePopup");
        Image dim = nodePopup.GetComponent<Image>();
        dim.color = new Color(0f, 0f, 0f, 0.6f);

        GameObject box = new GameObject("PopupBox", typeof(RectTransform));
        box.transform.SetParent(nodePopup.transform, false);
        RectTransform boxRT = (RectTransform)box.transform;
        boxRT.anchorMin = new Vector2(0.5f, 0.5f);
        boxRT.anchorMax = new Vector2(0.5f, 0.5f);
        boxRT.sizeDelta = new Vector2(360f, 220f);
        box.AddComponent<Image>().color = new Color(0.15f, 0.15f, 0.15f, 0.97f);
        VerticalLayoutGroup boxLayout = box.AddComponent<VerticalLayoutGroup>();
        boxLayout.padding = new RectOffset(20, 20, 20, 20);
        boxLayout.spacing = 10f;
        boxLayout.childForceExpandHeight = false;

        CreateText(box.transform, "Title", "Add Node", 18, TextAnchor.MiddleCenter);

        GameObject typeRow = CreateHorizontalRow(box.transform, "TypeRow");
        CreateText(typeRow.transform, "TypeLabel", "Type:", 14, TextAnchor.MiddleLeft);
        List<string> options = new List<string>(System.Enum.GetNames(typeof(CustomFrameLoader.NodeType)));
        nodeTypeDropdown = CreateDropdown(typeRow.transform, "TypeDropdown", options);

        GameObject massRow = CreateHorizontalRow(box.transform, "MassRow");
        CreateText(massRow.transform, "MassLabel", "Mass (kg):", 14, TextAnchor.MiddleLeft);
        nodeMassInput = CreateInputField(massRow.transform, "MassInput", "0.05", InputField.ContentType.DecimalNumber);

        GameObject popupButtonRow = CreateHorizontalRow(box.transform, "PopupButtonRow");
        CreateButton(popupButtonRow.transform, "CancelButton", "Cancel", CancelNodePopup);
        CreateButton(popupButtonRow.transform, "ConfirmButton", "Add", ConfirmNodePopup);

        nodePopup.SetActive(false);
    }

    private void BuildPreviewScene()
    {
        previewRoot = new GameObject("DronePreviewRoot").transform;
        previewRoot.position = PreviewWorldOrigin;

        Light light = new GameObject("PreviewLight").AddComponent<Light>();
        light.type = LightType.Directional;
        light.transform.SetParent(previewRoot, false);
        light.transform.rotation = Quaternion.Euler(45f, -30f, 0f);

        previewMeshObject = new GameObject("PreviewMesh");
        previewMeshObject.transform.SetParent(previewRoot, false);
        previewMeshFilter = previewMeshObject.AddComponent<MeshFilter>();
        MeshRenderer meshRenderer = previewMeshObject.AddComponent<MeshRenderer>();
        meshRenderer.sharedMaterial = new Material(Shader.Find("Standard")) { color = new Color(0.8f, 0.8f, 0.8f) };
        previewMeshCollider = previewMeshObject.AddComponent<MeshCollider>();

        GameObject cameraGO = new GameObject("PreviewCamera");
        cameraGO.transform.SetParent(previewRoot, false);
        previewCamera = cameraGO.AddComponent<Camera>();
        previewCamera.clearFlags = CameraClearFlags.SolidColor;
        previewCamera.backgroundColor = new Color(0.12f, 0.12f, 0.14f);
        previewCamera.fieldOfView = 45f;
        previewCamera.enabled = false;

        previewTexture = new RenderTexture(PreviewTextureSize, PreviewTextureSize, 16);
        previewCamera.targetTexture = previewTexture;
        previewImage.texture = previewTexture;

        pendingOrbitComponent.Target = previewRoot;
        pendingOrbitComponent.PreviewCamera = previewCamera;
        pendingOrbitComponent.RaycastTarget = previewMeshCollider;
    }

    // ---- Small UGUI construction helpers ----

    private static GameObject CreateStretchPanel(Transform parent, string name)
    {
        GameObject go = new GameObject(name, typeof(RectTransform));
        go.transform.SetParent(parent, false);
        RectTransform rt = (RectTransform)go.transform;
        rt.anchorMin = Vector2.zero;
        rt.anchorMax = Vector2.one;
        rt.offsetMin = Vector2.zero;
        rt.offsetMax = Vector2.zero;
        go.AddComponent<Image>().color = Color.clear;
        return go;
    }

    private static GameObject CreateVerticalColumn(Transform parent, string name)
    {
        GameObject go = new GameObject(name, typeof(RectTransform));
        go.transform.SetParent(parent, false);
        LayoutElement layoutElement = go.AddComponent<LayoutElement>();
        layoutElement.flexibleWidth = 1f;
        VerticalLayoutGroup layout = go.AddComponent<VerticalLayoutGroup>();
        layout.spacing = 8f;
        layout.childForceExpandHeight = false;
        layout.childAlignment = TextAnchor.UpperCenter;
        return go;
    }

    private static GameObject CreateHorizontalRow(Transform parent, string name)
    {
        GameObject go = new GameObject(name, typeof(RectTransform));
        go.transform.SetParent(parent, false);
        LayoutElement layoutElement = go.AddComponent<LayoutElement>();
        layoutElement.preferredHeight = 32f;
        HorizontalLayoutGroup layout = go.AddComponent<HorizontalLayoutGroup>();
        layout.spacing = 8f;
        layout.childForceExpandWidth = false;
        layout.childAlignment = TextAnchor.MiddleLeft;
        return go;
    }

    private static Text CreateText(Transform parent, string name, string content, int fontSize, TextAnchor alignment)
    {
        GameObject go = new GameObject(name, typeof(RectTransform));
        go.transform.SetParent(parent, false);
        Text text = go.AddComponent<Text>();
        text.font = Resources.GetBuiltinResource<Font>("Arial.ttf") ?? Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        text.fontSize = fontSize;
        text.alignment = alignment;
        text.color = Color.white;
        text.text = content;
        go.AddComponent<LayoutElement>().preferredHeight = fontSize + 8;
        return text;
    }

    private static Button CreateButton(Transform parent, string name, string label, UnityEngine.Events.UnityAction onClick)
    {
        GameObject go = new GameObject(name, typeof(RectTransform));
        go.transform.SetParent(parent, false);
        LayoutElement layoutElement = go.AddComponent<LayoutElement>();
        layoutElement.preferredWidth = 140f;
        layoutElement.preferredHeight = 32f;
        Image image = go.AddComponent<Image>();
        image.color = new Color(0.25f, 0.35f, 0.55f);
        Button button = go.AddComponent<Button>();
        button.targetGraphic = image;
        button.onClick.AddListener(onClick);

        Text text = CreateText(go.transform, "Label", label, 14, TextAnchor.MiddleCenter);
        RectTransform textRT = (RectTransform)text.transform;
        textRT.anchorMin = Vector2.zero;
        textRT.anchorMax = Vector2.one;
        textRT.offsetMin = Vector2.zero;
        textRT.offsetMax = Vector2.zero;
        Destroy(text.GetComponent<LayoutElement>());

        return button;
    }

    private static InputField CreateInputField(Transform parent, string name, string defaultText, InputField.ContentType contentType)
    {
        GameObject go = new GameObject(name, typeof(RectTransform));
        go.transform.SetParent(parent, false);
        LayoutElement layoutElement = go.AddComponent<LayoutElement>();
        layoutElement.preferredWidth = 120f;
        layoutElement.preferredHeight = 28f;
        go.AddComponent<Image>().color = new Color(1f, 1f, 1f, 0.9f);
        InputField field = go.AddComponent<InputField>();
        field.contentType = contentType;

        GameObject textGO = new GameObject("Text", typeof(RectTransform));
        textGO.transform.SetParent(go.transform, false);
        Text text = textGO.AddComponent<Text>();
        text.font = Resources.GetBuiltinResource<Font>("Arial.ttf") ?? Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        text.fontSize = 14;
        text.color = Color.black;
        text.alignment = TextAnchor.MiddleLeft;
        text.supportRichText = false;
        RectTransform textRT = (RectTransform)textGO.transform;
        textRT.anchorMin = Vector2.zero;
        textRT.anchorMax = Vector2.one;
        textRT.offsetMin = new Vector2(6f, 2f);
        textRT.offsetMax = new Vector2(-6f, -2f);

        field.textComponent = text;
        field.text = defaultText;
        return field;
    }

    private static Dropdown CreateDropdown(Transform parent, string name, List<string> options)
    {
        GameObject go = new GameObject(name, typeof(RectTransform));
        go.transform.SetParent(parent, false);
        LayoutElement layoutElement = go.AddComponent<LayoutElement>();
        layoutElement.preferredWidth = 160f;
        layoutElement.preferredHeight = 28f;
        go.AddComponent<Image>().color = new Color(1f, 1f, 1f, 0.9f);
        Dropdown dropdown = go.AddComponent<Dropdown>();

        GameObject labelGO = new GameObject("Label", typeof(RectTransform));
        labelGO.transform.SetParent(go.transform, false);
        Text label = labelGO.AddComponent<Text>();
        label.font = Resources.GetBuiltinResource<Font>("Arial.ttf") ?? Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        label.fontSize = 14;
        label.color = Color.black;
        label.alignment = TextAnchor.MiddleLeft;
        RectTransform labelRT = (RectTransform)labelGO.transform;
        labelRT.anchorMin = Vector2.zero;
        labelRT.anchorMax = Vector2.one;
        labelRT.offsetMin = new Vector2(6f, 2f);
        labelRT.offsetMax = new Vector2(-6f, -2f);
        dropdown.captionText = label;

        // Minimal template so the dropdown can open a plain option list.
        GameObject template = new GameObject("Template", typeof(RectTransform));
        template.transform.SetParent(go.transform, false);
        RectTransform templateRT = (RectTransform)template.transform;
        templateRT.anchorMin = new Vector2(0f, 0f);
        templateRT.anchorMax = new Vector2(1f, 0f);
        templateRT.pivot = new Vector2(0.5f, 1f);
        templateRT.anchoredPosition = new Vector2(0f, 2f);
        templateRT.sizeDelta = new Vector2(0f, 28f * Mathf.Max(options.Count, 1));
        template.AddComponent<Image>().color = new Color(1f, 1f, 1f, 0.97f);
        ScrollRect templateScroll = template.AddComponent<ScrollRect>();

        GameObject viewport = new GameObject("Viewport", typeof(RectTransform));
        viewport.transform.SetParent(template.transform, false);
        RectTransform viewportRT = (RectTransform)viewport.transform;
        viewportRT.anchorMin = Vector2.zero;
        viewportRT.anchorMax = Vector2.one;
        viewportRT.offsetMin = Vector2.zero;
        viewportRT.offsetMax = Vector2.zero;
        viewport.AddComponent<Image>().color = new Color(0f, 0f, 0f, 0.01f);
        viewport.AddComponent<Mask>().showMaskGraphic = false;

        GameObject content = new GameObject("Content", typeof(RectTransform));
        content.transform.SetParent(viewport.transform, false);
        RectTransform contentRT = (RectTransform)content.transform;
        contentRT.anchorMin = new Vector2(0f, 1f);
        contentRT.anchorMax = new Vector2(1f, 1f);
        contentRT.pivot = new Vector2(0.5f, 1f);
        contentRT.sizeDelta = new Vector2(0f, 28f * Mathf.Max(options.Count, 1));

        GameObject item = new GameObject("Item", typeof(RectTransform));
        item.transform.SetParent(content.transform, false);
        RectTransform itemRT = (RectTransform)item.transform;
        itemRT.anchorMin = new Vector2(0f, 1f);
        itemRT.anchorMax = new Vector2(1f, 1f);
        itemRT.sizeDelta = new Vector2(0f, 28f);
        Toggle itemToggle = item.AddComponent<Toggle>();
        Image itemBg = item.AddComponent<Image>();
        itemBg.color = new Color(0.9f, 0.9f, 0.9f);
        itemToggle.targetGraphic = itemBg;

        GameObject itemLabelGO = new GameObject("Item Label", typeof(RectTransform));
        itemLabelGO.transform.SetParent(item.transform, false);
        Text itemLabel = itemLabelGO.AddComponent<Text>();
        itemLabel.font = label.font;
        itemLabel.fontSize = 14;
        itemLabel.color = Color.black;
        itemLabel.alignment = TextAnchor.MiddleLeft;
        RectTransform itemLabelRT = (RectTransform)itemLabelGO.transform;
        itemLabelRT.anchorMin = Vector2.zero;
        itemLabelRT.anchorMax = Vector2.one;
        itemLabelRT.offsetMin = new Vector2(6f, 0f);
        itemLabelRT.offsetMax = Vector2.zero;

        templateScroll.content = contentRT;
        templateScroll.viewport = viewportRT;
        templateScroll.horizontal = false;

        dropdown.template = templateRT;
        dropdown.itemText = itemLabel;
        template.SetActive(false);

        dropdown.options.Clear();
        foreach (string option in options)
            dropdown.options.Add(new Dropdown.OptionData(option));
        dropdown.value = 0;
        dropdown.RefreshShownValue();

        return dropdown;
    }
}

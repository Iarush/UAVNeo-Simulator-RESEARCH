using TMPro;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

/// <summary>
/// Owns the third-person chase-camera zoom level for a <see cref="Drone"/>: maps a UGUI
/// Slider's [0,1] value to <see cref="Drone.ZoomMultiplier"/>, computes a sane initial zoom
/// from a loaded custom STL frame's mesh bounds (so a large imported drone isn't clipped or
/// overfills the screen on load), and shows/hides an assigned zoom panel based on
/// <see cref="Flight.IsArmed"/> (visible only while idle/pre-launch). The mouse wheel and
/// -/= keys zoom at any time, in flight included, and stay in sync with the slider.
/// </summary>
[RequireComponent(typeof(Drone))]
public class DroneCameraZoom : MonoBehaviour
{
    /// <summary>Closest the slider can bring the camera relative to the auto-fit distance.</summary>
    private const float MinZoomFactor = 0.5f;

    /// <summary>Farthest the slider can push the camera relative to the auto-fit distance.</summary>
    private const float MaxZoomFactor = 2.5f;

    /// <summary>Sanity floor for the auto-fit multiplier itself (relative to stock framing).</summary>
    private const float MinBaseMultiplier = 0.4f;

    /// <summary>Sanity ceiling for the auto-fit multiplier itself (relative to stock framing).</summary>
    private const float MaxBaseMultiplier = 8f;

    /// <summary>Extra headroom around the frame's bounds so it doesn't touch the screen edges.</summary>
    private const float FramingPadding = 1.6f;

    /// <summary>
    /// Slider value that reproduces the auto-fit distance exactly (factor 1): solves
    /// MaxZoomFactor + (MinZoomFactor - MaxZoomFactor) * value == 1.
    /// </summary>
    private const float InitialSliderValue = (MaxZoomFactor - 1f) / (MaxZoomFactor - MinZoomFactor);

    /// <summary>Slider units moved per mouse-wheel notch. Positive scroll (wheel up) zooms in.</summary>
    private const float ScrollStep = 0.05f;

    /// <summary>Slider units per second while a zoom key (-/=) is held.</summary>
    private const float KeyZoomSpeed = 0.5f;

    [Tooltip("Panel (slider + label) shown only while the drone is idle/pre-launch. Assign in the Inspector.")]
    public GameObject zoomPanel;

    [Tooltip("The zoom slider. Optional: if unassigned, the first Slider under zoomPanel is used.")]
    public Slider zoomSlider;

    private Drone drone;
    private Flight flight;
    private CustomFrameLoader frameLoader;
    private float baseZoomMultiplier = 1f;

    /// <summary>Current zoom position in [0,1]; the single source of truth shared by slider, wheel and keys.</summary>
    private float zoomValue = InitialSliderValue;

    private void Awake()
    {
        this.drone = this.GetComponent<Drone>();
        this.flight = this.GetComponent<Flight>();
        this.frameLoader = this.GetComponent<CustomFrameLoader>();

        if (this.zoomSlider == null && this.zoomPanel != null)
            this.zoomSlider = this.zoomPanel.GetComponentInChildren<Slider>(true);
    }

    private void Start()
    {
        // CustomFrameLoader runs its Awake (DefaultExecutionOrder -100) before this Start, so
        // CustomFrameBounds is already populated here if a custom STL frame was loaded.
        Bounds? customBounds = this.frameLoader != null ? this.frameLoader.CustomFrameBounds : null;
        this.baseZoomMultiplier = customBounds.HasValue ? this.ComputeAutoFitMultiplier(customBounds.Value) : 1f;
        this.SetZoom(InitialSliderValue);
    }

    private void Update()
    {
        // Mouse wheel and -/= keys zoom at any time, including in flight. Zooming only moves the
        // chase cameras (Drone.LateUpdate); it never touches Flight or PropellerSpin.
        float delta = Input.mouseScrollDelta.y * ScrollStep;
        if (!this.IsTypingInUI())
        {
            if (Input.GetKey(KeyCode.Equals) || Input.GetKey(KeyCode.KeypadPlus))
                delta += KeyZoomSpeed * Time.deltaTime;
            if (Input.GetKey(KeyCode.Minus) || Input.GetKey(KeyCode.KeypadMinus))
                delta -= KeyZoomSpeed * Time.deltaTime;
        }
        if (delta != 0f)
            this.SetZoom(this.zoomValue + delta);

        if (this.zoomPanel == null || this.flight == null)
            return;

        // The slider stays hidden in flight: a focused Slider would capture the arrow keys used for yaw.
        bool shouldShow = !this.flight.IsArmed;
        if (this.zoomPanel.activeSelf != shouldShow)
            this.zoomPanel.SetActive(shouldShow);
    }

    /// <summary>
    /// Wire this to a UGUI Slider's OnValueChanged event (range 0-1). 1 is the closest allowed
    /// zoom (slider up = zoom in), 0 the farthest; the slider's default value (see
    /// InitialSliderValue) lands exactly on the auto-fit distance computed for the current drone.
    /// </summary>
    public void OnZoomChanged(float value)
    {
        this.zoomValue = Mathf.Clamp01(value);
        float factor = Mathf.Lerp(MaxZoomFactor, MinZoomFactor, this.zoomValue);
        this.drone.ZoomMultiplier = this.baseZoomMultiplier * factor;
    }

    /// <summary>
    /// Applies a zoom value and keeps the slider in sync without re-firing its OnValueChanged.
    /// </summary>
    private void SetZoom(float value)
    {
        this.OnZoomChanged(value);
        if (this.zoomSlider != null)
            this.zoomSlider.SetValueWithoutNotify(this.zoomValue);
    }

    private bool IsTypingInUI()
    {
        GameObject selected = EventSystem.current != null ? EventSystem.current.currentSelectedGameObject : null;
        return selected != null && (selected.GetComponent<TMP_InputField>() != null || selected.GetComponent<InputField>() != null);
    }

    /// <summary>
    /// Computes the chase-camera zoom multiplier (relative to Drone.DefaultCameraDistance)
    /// needed to fit the given drone-relative mesh bounds within the primary camera's FOV.
    /// </summary>
    private float ComputeAutoFitMultiplier(Bounds meshBounds)
    {
        Camera primaryCamera = this.drone.PrimaryCamera;
        float fov = primaryCamera != null ? primaryCamera.fieldOfView : 60f;

        float boundingRadius = meshBounds.extents.magnitude;
        float requiredDistance = boundingRadius * FramingPadding / Mathf.Tan(fov * 0.5f * Mathf.Deg2Rad);

        return Mathf.Clamp(requiredDistance / Drone.DefaultCameraDistance, MinBaseMultiplier, MaxBaseMultiplier);
    }
}

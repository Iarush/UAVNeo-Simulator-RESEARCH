using System;
using UnityEngine;
using UnityEngine.EventSystems;

/// <summary>
/// Lives on the RawImage showing the drone mesh preview. Mouse-drag orbits
/// <see cref="Target"/>; a plain click (no drag) raycasts against
/// <see cref="RaycastTarget"/> and reports the world-space hit point.
/// </summary>
public class DronePreviewOrbit : MonoBehaviour, IBeginDragHandler, IDragHandler, IEndDragHandler, IPointerClickHandler
{
    private const float DragDegreesPerPixel = 0.35f;
    private const float MaxPitch = 80f;
    private const float MaxRaycastDistance = 1000f;

    /// <summary>Rotated in response to drag input.</summary>
    public Transform Target;

    /// <summary>Camera the preview is rendered through; used to build the click ray.</summary>
    public Camera PreviewCamera;

    /// <summary>Only hits against this exact collider are reported (nothing else lives out at the preview's isolated location, but this makes the intent explicit).</summary>
    public Collider RaycastTarget;

    /// <summary>Invoked with the world-space point where the user clicked on RaycastTarget.</summary>
    public Action<Vector3> OnSurfaceClicked;

    private float yaw;
    private float pitch;

    public void OnBeginDrag(PointerEventData eventData) { }

    public void OnDrag(PointerEventData eventData)
    {
        if (Target == null)
            return;

        yaw += eventData.delta.x * DragDegreesPerPixel;
        pitch = Mathf.Clamp(pitch - eventData.delta.y * DragDegreesPerPixel, -MaxPitch, MaxPitch);
        Target.localRotation = Quaternion.Euler(pitch, yaw, 0f);
    }

    public void OnEndDrag(PointerEventData eventData) { }

    public void OnPointerClick(PointerEventData eventData)
    {
        if (PreviewCamera == null || RaycastTarget == null)
            return;

        RectTransform rectTransform = (RectTransform)transform;
        if (!RectTransformUtility.ScreenPointToLocalPointInRectangle(rectTransform, eventData.position, eventData.pressEventCamera, out Vector2 localPoint))
            return;

        Rect rect = rectTransform.rect;
        float u = (localPoint.x - rect.x) / rect.width;
        float v = (localPoint.y - rect.y) / rect.height;
        if (u < 0f || u > 1f || v < 0f || v > 1f)
            return;

        Ray ray = PreviewCamera.ViewportPointToRay(new Vector3(u, v, 0f));
        if (Physics.Raycast(ray, out RaycastHit hit, MaxRaycastDistance) && hit.collider == RaycastTarget)
            OnSurfaceClicked?.Invoke(hit.point);
    }
}

using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Spins the propeller GameObjects based on actual motor speeds from the Flight physics.
///
/// Instead of relying on index ordering (which depends on prefab builder sorting),
/// each propeller's spin direction is determined at runtime by its ACTUAL POSITION
/// relative to the drone center:
///
///   FL (CW ↻)    FR (CCW ↺)
///   BL (CCW ↺)   BR (CW ↻)
///
/// The sign mapping also accounts for whether the propeller mesh's local-Y axis
/// is aligned or inverted relative to the drone's up vector.
/// </summary>
public class PropellerSpin : MonoBehaviour
{
    [SerializeField]
    private float fallbackSpinSpeed = 3000f;

    [SerializeField]
    private float visualSpeedScale = 20.0f;

    private Transform[] propellers;
    private float[] directions;
    private int[] motorIndices;
    private Flight flight;

    private void Start()
    {
        flight = GetComponent<Flight>();
        FindPropellers();
    }

    private void FindPropellers()
    {
        List<Transform> found = new List<Transform>();
        Transform[] allChildren = GetComponentsInChildren<Transform>(true);
        foreach (Transform t in allChildren)
        {
            if (t.name.StartsWith("Propeller_"))
                found.Add(t);
        }

        // Sort by name for a stable order; motorIndices maps each one to Flight.MotorSpeeds
        found.Sort((a, b) => a.name.CompareTo(b.name));

        propellers = found.ToArray();
        directions = new float[propellers.Length];
        motorIndices = new int[propellers.Length];

        // Assign spin direction based on each propeller's actual world position.
        // This removes all dependency on the index-to-position mapping.
        for (int i = 0; i < propellers.Length; i++)
        {
            // Get propeller position in the drone's local coordinate system
            // (drone forward = +Z, drone right = +X, drone up = +Y)
            Vector3 localPos = transform.InverseTransformPoint(propellers[i].position);

            bool isFront = localPos.z >= 0;
            bool isLeft = localPos.x < 0;

            // Desired spin directions:
            //   FL = CW,  FR = CCW
            //   BL = CCW, BR = CW
            bool wantCW;
            if (isFront && isLeft) wantCW = true;       // FL → CW
            else if (isFront && !isLeft) wantCW = false; // FR → CCW
            else if (!isFront && isLeft) wantCW = false;  // BL → CCW
            else wantCW = true;                            // BR → CW

            // Check if this propeller's local Y aligns with or opposes the drone's up.
            // FBX imports can flip the local axis on some children. If inverted,
            // we need to reverse the rotation sign so the visual still looks correct.
            float yDot = Vector3.Dot(propellers[i].up, transform.up);
            bool meshInverted = yDot < 0;

            float sign = wantCW ? 1f : -1f;
            if (meshInverted) sign *= -1f;

            directions[i] = sign;

            // Flight.MotorSpeeds order is FR, FL, BL, BR (Flight.cs MotorSpeeds doc), which is
            // not the Propeller_N name order, so map by quadrant.
            if (isFront) motorIndices[i] = isLeft ? 1 : 0;
            else motorIndices[i] = isLeft ? 2 : 3;

            string quadrant = (isFront ? "Front" : "Back") + (isLeft ? " Left" : " Right");
            string target = wantCW ? "CW" : "CCW";
            Debug.Log($"[PropellerSpin] Propeller_{i} at local({localPos.x:F3}, {localPos.y:F3}, {localPos.z:F3}) → {quadrant}, target={target}, meshInverted={meshInverted}, sign={sign}");
        }

        Debug.Log($"[PropellerSpin] Found {propellers.Length} propellers, Flight={flight != null}");
    }

    /// <summary>
    /// Overrides each propeller's local position after a custom frame is loaded.
    /// <paramref name="positions"/> must contain exactly 4 entries in FL, FR, BL, BR order.
    /// Positions are matched to propellers by each propeller's CURRENT quadrant (not by
    /// array index), so the propellers[]/directions[] index order — which must stay aligned
    /// with Flight.MotorSpeeds — is preserved.
    /// </summary>
    public void SetPropellerPositions(Vector3[] positions)
    {
        if (positions == null || positions.Length != 4)
        {
            Debug.LogError($"[PropellerSpin] SetPropellerPositions requires exactly 4 positions (FL, FR, BL, BR), got {positions?.Length ?? 0}. Propeller positions were not changed.");
            return;
        }

        // This can be called before Start() has run (e.g. from another script's Awake()),
        // so make sure the propellers are found before repositioning them.
        if (propellers == null)
            FindPropellers();

        if (propellers.Length != 4)
        {
            Debug.LogError($"[PropellerSpin] Expected 4 propellers to reposition, found {propellers.Length}. Propeller positions were not changed.");
            return;
        }

        Vector3 fl = positions[0], fr = positions[1], bl = positions[2], br = positions[3];
        for (int i = 0; i < propellers.Length; i++)
        {
            Vector3 localPos = transform.InverseTransformPoint(propellers[i].position);
            bool isFront = localPos.z >= 0;
            bool isLeft = localPos.x < 0;

            // positions are drone-root local; propellers live under the rotated, 4.5x-scaled
            // DroneModel, so convert through world space rather than assigning localPosition.
            Vector3 target;
            if (isFront && isLeft) target = fl;
            else if (isFront && !isLeft) target = fr;
            else if (!isFront && isLeft) target = bl;
            else target = br;
            propellers[i].position = transform.TransformPoint(target);

            // Keep the stock prop-diameter-to-arm ratio so props match the new frame's size
            // (the stock props are ~1.15 m across, sized for the 4.5x-scaled stock model).
            float oldArm = new Vector2(localPos.x, localPos.z).magnitude;
            float newArm = new Vector2(target.x, target.z).magnitude;
            if (oldArm > 0f)
                propellers[i].localScale *= newArm / oldArm;
        }

        // Positions changed, so quadrants/spin directions need to be recomputed.
        FindPropellers();
    }

    private void Update()
    {
        if (propellers == null) return;

        for (int i = 0; i < propellers.Length; i++)
        {
            if (propellers[i] == null) continue;

            float degPerSec;
            if (flight != null && motorIndices[i] < flight.MotorSpeeds.Length)
            {
                degPerSec = flight.MotorSpeeds[motorIndices[i]] * visualSpeedScale;
                // Ensure propellers always visually spin when the drone is armed.
                // Motor mixing can drive individual motors to near-zero at certain
                // attitudes, but real props never fully stop in flight.
                if (flight.IsArmed)
                    degPerSec = Mathf.Max(degPerSec, 5000f);
            }
            else
            {
                degPerSec = fallbackSpinSpeed;
            }

            propellers[i].Rotate(0f, degPerSec * directions[i] * Time.deltaTime, 0f, Space.Self);
        }
    }
}

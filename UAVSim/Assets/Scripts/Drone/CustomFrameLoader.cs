using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

/// <summary>
/// Requests calculated mass properties for a user-selected STL and replaces the default
/// drone visual when the request and mesh load both succeed.
/// </summary>
[DefaultExecutionOrder(-100)]
public class CustomFrameLoader : MonoBehaviour
{
    public enum NodeType : byte
    {
        Motor = 0,
        Battery = 1,
        FlightController = 2,
        OffboardComputer = 3
    }

    [Serializable]
    public struct NodeData
    {
        public NodeType type;
        public Vector3 position;
        public float massKg;
    }

    /// <summary>Set by the custom-frame UI before the drone prefab is instantiated.</summary>
    public static string SelectedSTLPath = "";

    /// <summary>Material density sent to the mass-properties service.</summary>
    public static float SelectedDensity = 1000f;

    /// <summary>Mass nodes sent to the mass-properties service.</summary>
    public static readonly List<NodeData> SelectedNodes = new List<NodeData>();

    /// <summary>
    /// Local-space bounds of the most recently loaded custom STL mesh (identity-transform child
    /// of the drone root, so this is also drone-relative). Null until a custom frame has actually
    /// replaced the default visual. Read by DroneCameraZoom to auto-fit the chase camera.
    /// </summary>
    public Bounds? CustomFrameBounds { get; private set; }

    [Header("Custom frame visual (assign in the prefab)")]
    [SerializeField]
    [Tooltip("Dedicated child that exists only to host the imported STL mesh. Needs a MeshFilter " +
             "and a MeshRenderer with a material. Keep it a direct child of the drone root with an " +
             "identity transform - the STL arrives in metres, Unity Y-up, drone-root relative.")]
    private MeshFilter customFrameVisual;

    [SerializeField]
    [Tooltip("Default body roots deactivated when a custom frame loads. Must NOT contain " +
             "Propeller_0-3, which stay active so the propellers remain visible and keep spinning.")]
    private GameObject[] defaultBodyObjects;

    private const byte RequestHeader = 40;
    private const byte ResponseHeader = 41;
    private const int ResponseLength = 45;
    private const int TimeoutMilliseconds = 5000;

    private void Awake()
    {
        if (string.IsNullOrWhiteSpace(SelectedSTLPath))
            return;

        if (!File.Exists(SelectedSTLPath))
        {
            Debug.LogError($"[CustomFrameLoader] Selected STL file does not exist: {SelectedSTLPath}. Using the default drone.");
            return;
        }

        // Checked before any socket traffic, and shared with the node placement UI so the two can
        // never disagree. This also guarantees RepositionPropellers below cannot hit its own
        // failure paths, which fail soft after the mesh and mass properties are already applied.
        if (!MotorQuadrants.TryValidateMotorLayout(SelectedNodes, out string layoutError))
        {
            Debug.LogError($"[CustomFrameLoader] {layoutError} The import pipeline currently " +
                           "supports quadcopters only - hexacopters and octocopters are not " +
                           "supported. Using the default drone.");
            return;
        }

        if (!TryGetMassProperties(out Vector3 com, out Vector3 moments, out Quaternion rotation, out float armLength))
            return;

        if (!StlMeshLoader.TryLoadStl(SelectedSTLPath, out Mesh mesh))
            return;

        Flight flight = GetComponent<Flight>();
        if (flight == null)
        {
            Debug.LogError("[CustomFrameLoader] Flight component is missing. Using the default drone.");
            Destroy(mesh);
            return;
        }

        // The service response carries no mass (locked protocol), so total mass is mesh volume
        // x density + node masses, from the same mesh and nodes the service was given.
        float totalMass = SignedVolume(mesh) * SelectedDensity;
        foreach (NodeData node in SelectedNodes)
            totalMass += node.massKg;
        if (!IsFinite(totalMass) || totalMass <= 0f)
        {
            Debug.LogError($"[CustomFrameLoader] Computed total mass {totalMass} kg is invalid. Using the default drone.");
            Destroy(mesh);
            return;
        }
        Debug.Log($"[CustomFrameLoader] Total mass: {totalMass:F5} kg");

        // Do all fallible work first so a failure never partially changes the drone.
        ReplaceDefaultVisual(mesh);
        // ReplaceDefaultVisual destroys mesh on failure (e.g. customFrameVisual unassigned), so
        // only record bounds once we can see it actually landed on the visual.
        if (customFrameVisual != null && customFrameVisual.sharedMesh == mesh)
            this.CustomFrameBounds = mesh.bounds;
        flight.ApplyCustomMassProperties(com, moments, rotation, armLength, totalMass);
        RepositionPropellers();
    }

    /// <summary>
    /// Moves the propeller visuals to match the motor nodes supplied for this custom frame,
    /// matching each Motor node to a corner with MotorQuadrants.ClassifyUnity (the same rule
    /// PropellerSpin and the panel validation use: +Z front, -X left).
    /// </summary>
    private void RepositionPropellers()
    {
        Vector3? fl = null, fr = null, bl = null, br = null;
        foreach (NodeData node in SelectedNodes)
        {
            if (node.type != NodeType.Motor)
                continue;

            // Node positions are defined in the same Z-up frame as the STL/mass-properties
            // service, so they need the same axis conversion as the mesh/com (they're already
            // meter-scale, not millimeters, so no MillimetersToMeters scale here).
            Vector3 position = StlMeshLoader.ConvertZUpToYUp(node.position);

            bool alreadyAssigned;
            switch (MotorQuadrants.ClassifyUnity(position))
            {
                case MotorQuadrants.Quadrant.FrontLeft: alreadyAssigned = fl.HasValue; fl = position; break;
                case MotorQuadrants.Quadrant.FrontRight: alreadyAssigned = fr.HasValue; fr = position; break;
                case MotorQuadrants.Quadrant.BackLeft: alreadyAssigned = bl.HasValue; bl = position; break;
                default: alreadyAssigned = br.HasValue; br = position; break;
            }

            if (alreadyAssigned)
            {
                Debug.LogError("[CustomFrameLoader] More than one motor node maps to the same propeller corner. Propeller positions were not overridden.");
                return;
            }
        }

        if (!fl.HasValue || !fr.HasValue || !bl.HasValue || !br.HasValue)
        {
            Debug.LogError($"[CustomFrameLoader] Expected exactly one motor node in each of the FL/FR/BL/BR quadrants (found FL={fl.HasValue}, FR={fr.HasValue}, BL={bl.HasValue}, BR={br.HasValue}). Propeller positions were not overridden.");
            return;
        }

        // Flight's motor mixing assumes a symmetric ideal X-quad (single armLength for all
        // 4 motors) - log actual positions so we can check how far this frame is from that.
        Debug.Log($"[CustomFrameLoader] Motor positions (Unity-local, post-conversion): FL={fl.Value:F4} FR={fr.Value:F4} BL={bl.Value:F4} BR={br.Value:F4}");

        PropellerSpin propellerSpin = GetComponent<PropellerSpin>();
        if (propellerSpin == null)
        {
            Debug.LogError("[CustomFrameLoader] PropellerSpin component is missing. Propeller positions were not overridden.");
            return;
        }

        propellerSpin.SetPropellerPositions(new[] { fl.Value, fr.Value, bl.Value, br.Value });
    }

    private bool TryGetMassProperties(out Vector3 com, out Vector3 moments, out Quaternion rotation, out float armLength)
    {
        com = Vector3.zero;
        moments = Vector3.zero;
        rotation = Quaternion.identity;
        armLength = 0f;

        try
        {
            byte[] request = BuildRequest();
            IPEndPoint server = new IPEndPoint(IPAddress.Loopback, 5070);
            using (UdpClient client = new UdpClient())
            {
                client.Client.ReceiveTimeout = TimeoutMilliseconds;
                client.Send(request, request.Length, server);
                IPEndPoint responseEndpoint = server;
                byte[] response = client.Receive(ref responseEndpoint);

                if (response == null || response.Length == 0)
                {
                    Debug.LogError($"[CustomFrameLoader] Invalid mass-properties response length (expected {ResponseLength}, received {response?.Length ?? 0}). Using the default drone.");
                    return false;
                }

                if (response[0] == byte.MaxValue)
                {
                    if (response.Length < 5)
                    {
                        Debug.LogError("[CustomFrameLoader] Malformed error response from the mass-properties service. Using the default drone.");
                        return false;
                    }

                    uint messageLength = BitConverter.ToUInt32(response, 1);
                    if (messageLength > response.Length - 5)
                    {
                        Debug.LogError("[CustomFrameLoader] Malformed error response from the mass-properties service. Using the default drone.");
                        return false;
                    }

                    Debug.LogError(Encoding.UTF8.GetString(response, 5, (int)messageLength));
                    return false;
                }

                if (response[0] != ResponseHeader)
                {
                    Debug.LogError($"[CustomFrameLoader] Unexpected mass-properties response header {response[0]} (expected {ResponseHeader}). Using the default drone.");
                    return false;
                }
                if (response.Length != ResponseLength)
                {
                    Debug.LogError($"[CustomFrameLoader] Invalid mass-properties response length (expected {ResponseLength}, received {response.Length}). Using the default drone.");
                    return false;
                }

                // Same Z-up frame as the STL mesh, so it needs the same axis conversion.
                // Unlike the raw mesh vertices, the service already returns this in meters
                // (confirmed from logged output: moments/armLength are already sane SI-unit
                // magnitudes with no scaling applied), so no MillimetersToMeters here.
                com = StlMeshLoader.ConvertZUpToYUp(new Vector3(ReadFloat(response, 1), ReadFloat(response, 5), ReadFloat(response, 9)));
                moments = new Vector3(ReadFloat(response, 13), ReadFloat(response, 17), ReadFloat(response, 21));
                // Same Z-up frame as com/mesh, so it needs the same axis conversion. This is a
                // proper rotation (not a reflection), so it composes safely with the service's
                // principal-axis rotation via ordinary quaternion multiplication.
                Quaternion rawRotation = new Quaternion(ReadFloat(response, 25), ReadFloat(response, 29), ReadFloat(response, 33), ReadFloat(response, 37));
                rotation = StlMeshLoader.ZUpToYUpRotation * rawRotation;
                armLength = ReadFloat(response, 41);
                Debug.Log($"[CustomFrameLoader] Mass properties: com={com:F4} moments=({moments.x:F6},{moments.y:F6},{moments.z:F6}) rotation={rotation} armLength={armLength:F4}");
                // armLength needs its own IsFinite check: com/moments/rotation get one via
                // IsValid, but every comparison with NaN is false, so a NaN armLength would slip
                // straight through "armLength <= 0f" and reach Flight.armEffective.
                if (!IsValid(com) || !IsValid(moments) || !IsValid(rotation) ||
                    moments.x <= 0f || moments.y <= 0f || moments.z <= 0f ||
                    !IsFinite(armLength) || armLength <= 0f)
                {
                    Debug.LogError("[CustomFrameLoader] Mass-properties response contains invalid values. Using the default drone.");
                    return false;
                }
                return true;
            }
        }
        catch (SocketException e)
        {
            string reason = e.SocketErrorCode == SocketError.TimedOut
                ? "No response received from the mass-properties service within 5 seconds."
                : $"Socket error while contacting the mass-properties service: {e}";
            Debug.LogError($"[CustomFrameLoader] {reason} Using the default drone.");
        }
        catch (Exception e)
        {
            Debug.LogError($"[CustomFrameLoader] Could not request mass properties: {e}. Using the default drone.");
        }
        return false;
    }

    private byte[] BuildRequest()
    {
        byte[] pathBytes = Encoding.UTF8.GetBytes(SelectedSTLPath);
        using (MemoryStream stream = new MemoryStream(1 + 4 + 4 + pathBytes.Length + 4 + SelectedNodes.Count * 17))
        using (BinaryWriter writer = new BinaryWriter(stream, Encoding.UTF8))
        {
            writer.Write(RequestHeader);
            writer.Write(SelectedDensity);
            writer.Write((uint)pathBytes.Length);
            writer.Write(pathBytes);
            writer.Write((uint)SelectedNodes.Count);
            foreach (NodeData node in SelectedNodes)
            {
                writer.Write((byte)node.type);
                writer.Write(node.position.x);
                writer.Write(node.position.y);
                writer.Write(node.position.z);
                writer.Write(node.massKg);
            }
            return stream.ToArray();
        }
    }

    private static float ReadFloat(byte[] data, int offset)
    {
        return BitConverter.ToSingle(data, offset);
    }

    private static bool IsValid(Vector3 value)
    {
        return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
    }

    private static bool IsValid(Quaternion value)
    {
        float sqrMagnitude = value.x * value.x + value.y * value.y + value.z * value.z + value.w * value.w;
        return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z) && IsFinite(value.w) && sqrMagnitude > 0f;
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    /// <summary>Enclosed volume (m^3) of a closed mesh, summed over origin-based tetrahedra.</summary>
    private static float SignedVolume(Mesh mesh)
    {
        Vector3[] v = mesh.vertices;
        int[] t = mesh.triangles;
        double sum = 0.0;
        for (int i = 0; i < t.Length; i += 3)
            sum += Vector3.Dot(v[t[i]], Vector3.Cross(v[t[i + 1]], v[t[i + 2]]));
        return (float)(sum / 6.0);
    }

    private void ReplaceDefaultVisual(Mesh mesh)
    {
        // The custom mesh goes onto a dedicated child assigned in the prefab, never onto a node
        // of the imported FBX. Selecting a target by name previously tested only the candidate's
        // OWN GameObject name, so a mesh node parented under Propeller_* was not excluded and the
        // frame could land on a transform PropellerSpin rotates.
        if (customFrameVisual == null)
        {
            Debug.LogError("[CustomFrameLoader] customFrameVisual is not assigned on this prefab. Using the default drone.", this);
            Destroy(mesh);
            return;
        }

        MeshRenderer renderer = customFrameVisual.GetComponent<MeshRenderer>();
        if (renderer == null)
        {
            Debug.LogError("[CustomFrameLoader] customFrameVisual has no MeshRenderer on the same GameObject. Using the default drone.", this);
            Destroy(mesh);
            return;
        }

        customFrameVisual.sharedMesh = mesh;
        customFrameVisual.gameObject.SetActive(true);
        renderer.enabled = true;

        if (defaultBodyObjects == null || defaultBodyObjects.Length == 0)
        {
            Debug.LogWarning("[CustomFrameLoader] defaultBodyObjects is empty, so the default body will render on top of the custom frame.", this);
            return;
        }

        // Only the default body is hidden. Propeller_0-3 must stay active so they remain visible
        // and keep spinning, so they must not live under any of these roots.
        foreach (GameObject bodyObject in defaultBodyObjects)
        {
            if (bodyObject == null)
                continue;

            foreach (Transform descendant in bodyObject.GetComponentsInChildren<Transform>(true))
            {
                if (descendant.name.StartsWith("Propeller_"))
                {
                    Debug.LogWarning($"[CustomFrameLoader] '{descendant.name}' is inside default body root " +
                                     $"'{bodyObject.name}', so hiding that root also hides the propeller. " +
                                     "Reassign defaultBodyObjects to roots that exclude the propellers.", descendant);
                }
            }

            bodyObject.SetActive(false);
        }
    }
}

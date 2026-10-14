using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// Shared STL-to-Mesh parsing, used by both CustomFrameLoader (loading a custom drone into the
/// flight scene) and CustomDroneBuilderUI (previewing a custom drone while placing mass nodes).
/// </summary>
public static class StlMeshLoader
{
    public const float MillimetersToMeters = 0.001f;

    // The source CAD tool exports Z-up; Unity is Y-up. This is a proper +90 deg rotation
    // about local X ((x,y,z) -> (x,-z,y)), verified to flip Z-up "top" to Y-up "top" -
    // NOT a Y/Z swap, which would be a mirror (reflection) and put the mesh upside-down.
    // Being a proper rotation (not a reflection) also means triangle winding stays correct
    // on its own, and this same rotation can be safely composed with other rotations
    // (e.g. an inertia-tensor principal-axis rotation) via ordinary quaternion multiplication.
    public static readonly Quaternion ZUpToYUpRotation = Quaternion.AngleAxis(90f, Vector3.right);

    public static Vector3 ConvertZUpToYUp(Vector3 v)
    {
        return ZUpToYUpRotation * v;
    }

    /// <summary>
    /// Inverse of <see cref="ConvertZUpToYUp"/> - used by the drone builder UI to convert a
    /// point picked on the (already Y-up, meter-scale) preview mesh back into the STL's raw
    /// Z-up frame, matching what CustomFrameLoader expects for node positions.
    /// </summary>
    public static Vector3 ConvertYUpToZUp(Vector3 v)
    {
        return Quaternion.Inverse(ZUpToYUpRotation) * v;
    }

    public static bool TryLoadStl(string path, out Mesh mesh)
    {
        mesh = null;
        try
        {
            byte[] bytes = File.ReadAllBytes(path);
            if (bytes.Length < 6)
                throw new InvalidDataException("The file is too small to be an STL.");

            List<Vector3> vertices = new List<Vector3>();
            List<Vector3> normals = new List<Vector3>();
            List<int> triangles = new List<int>();
            if (LooksLikeBinaryStl(bytes))
                ReadBinaryStl(bytes, vertices, normals, triangles);
            else
                ReadAsciiStl(Encoding.UTF8.GetString(bytes), vertices, normals, triangles);

            if (vertices.Count == 0 || triangles.Count == 0)
                throw new InvalidDataException("The STL contains no triangles.");

            mesh = new Mesh { name = Path.GetFileNameWithoutExtension(path) };
            if (vertices.Count > UInt16.MaxValue)
                mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
            mesh.SetVertices(vertices);
            mesh.SetTriangles(triangles, 0);
            mesh.SetNormals(normals);
            mesh.RecalculateBounds();
            return true;
        }
        catch (Exception e)
        {
            Debug.LogError($"[StlMeshLoader] Could not load STL '{path}': {e.Message}.");
            if (mesh != null) UnityEngine.Object.Destroy(mesh);
            mesh = null;
            return false;
        }
    }

    private static bool LooksLikeBinaryStl(byte[] bytes)
    {
        if (bytes.Length < 84) return false;
        uint triangleCount = BitConverter.ToUInt32(bytes, 80);
        return 84L + triangleCount * 50L == bytes.Length;
    }

    private static void ReadBinaryStl(byte[] data, List<Vector3> vertices, List<Vector3> normals, List<int> triangles)
    {
        int count = BitConverter.ToInt32(data, 80);
        for (int i = 0, offset = 84; i < count; i++, offset += 50)
        {
            Vector3 normal = new Vector3(ReadFloat(data, offset), ReadFloat(data, offset + 4), ReadFloat(data, offset + 8));
            AddTriangle(vertices, normals, triangles, normal,
                ReadVector(data, offset + 12), ReadVector(data, offset + 24), ReadVector(data, offset + 36));
        }
    }

    private static Vector3 ReadVector(byte[] data, int offset)
    {
        return new Vector3(ReadFloat(data, offset), ReadFloat(data, offset + 4), ReadFloat(data, offset + 8));
    }

    private static float ReadFloat(byte[] data, int offset)
    {
        return BitConverter.ToSingle(data, offset);
    }

    private static void ReadAsciiStl(string text, List<Vector3> vertices, List<Vector3> normals, List<int> triangles)
    {
        Vector3 currentNormal = Vector3.zero;
        List<Vector3> facet = new List<Vector3>(3);
        string[] lines = text.Split(new[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
        foreach (string rawLine in lines)
        {
            string[] parts = rawLine.Trim().Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length == 5 && string.Equals(parts[0], "facet", StringComparison.OrdinalIgnoreCase) && string.Equals(parts[1], "normal", StringComparison.OrdinalIgnoreCase))
                currentNormal = ParseVector(parts, 2);
            else if (parts.Length == 4 && string.Equals(parts[0], "vertex", StringComparison.OrdinalIgnoreCase))
            {
                facet.Add(ParseVector(parts, 1));
                if (facet.Count == 3)
                {
                    AddTriangle(vertices, normals, triangles, currentNormal, facet[0], facet[1], facet[2]);
                    facet.Clear();
                }
            }
        }
    }

    private static Vector3 ParseVector(string[] values, int start)
    {
        return new Vector3(
            float.Parse(values[start], CultureInfo.InvariantCulture),
            float.Parse(values[start + 1], CultureInfo.InvariantCulture),
            float.Parse(values[start + 2], CultureInfo.InvariantCulture));
    }

    private static void AddTriangle(List<Vector3> vertices, List<Vector3> normals, List<int> triangles, Vector3 normal, Vector3 a, Vector3 b, Vector3 c)
    {
        normal = ConvertZUpToYUp(normal);
        a = ConvertZUpToYUp(a);
        b = ConvertZUpToYUp(b);
        c = ConvertZUpToYUp(c);

        if (normal.sqrMagnitude == 0f)
            normal = Vector3.Cross(b - a, c - a).normalized;

        // STL files (e.g. OnShape/SolidWorks exports) are in millimeters; Unity uses meters.
        a *= MillimetersToMeters;
        b *= MillimetersToMeters;
        c *= MillimetersToMeters;

        int index = vertices.Count;
        vertices.Add(a); vertices.Add(b); vertices.Add(c);
        normals.Add(normal); normals.Add(normal); normals.Add(normal);
        triangles.Add(index); triangles.Add(index + 1); triangles.Add(index + 2);
    }
}

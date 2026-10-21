using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Build Drone stability estimate (spec .plans/real-flight-physics-spec.md, section 4.7 and open
/// item 1). The import panel cannot see the mass service's answer (that call happens in the flight
/// scene), so this estimates the frame's mass, centre of mass and inertia here: the STL as a solid
/// of uniform density (volume integral over its triangles), plus the node table as point masses.
/// The result feeds the same <see cref="Flight.TryControlAuthority"/> check the flight scene uses.
///
/// Estimate only: the inertia is taken on the frame's body axes (no principal-axis rotation), so
/// the panel words its message as "may be unstable". All maths is in Unity Y-up metres.
/// </summary>
public static class FrameStabilityEstimate
{
    /// <summary>
    /// Null when the frame looks stable (or cannot be estimated), else a short warning for the panel.
    /// </summary>
    public static string Warn(string stlPath, float density, IReadOnlyList<CustomFrameLoader.NodeData> nodes)
    {
        if (!TryEstimate(stlPath, density, nodes, out Vector3 authority, out string error))
        {
            Debug.LogWarning($"[FrameStabilityEstimate] No stability estimate: {error}");
            return null;
        }
        string problem = Flight.DescribeAuthorityProblem(authority);
        Debug.Log($"[FrameStabilityEstimate] Estimated control authority: roll {authority.z:F0}, pitch {authority.x:F0}, yaw {authority.y:F0}" +
                  (problem == null ? "" : $" -> {problem}"));
        return problem == null ? null : $"May be unstable: {problem}.";
    }

    public static bool TryEstimate(string stlPath, float density, IReadOnlyList<CustomFrameLoader.NodeData> nodes, out Vector3 authority, out string error)
    {
        authority = Vector3.zero;
        if (!StlMeshLoader.TryLoadStl(stlPath, out Mesh mesh))
        {
            error = "the STL could not be read";
            return false;
        }

        // Solid mesh: volume, first and second moments from signed tetrahedra (origin, a, b, c).
        double volume = 0;
        Vector3 first = Vector3.zero;
        Matrix4x4 second = Matrix4x4.zero; // integral of x x^T dV (3x3 block used)
        Vector3[] v = mesh.vertices;
        int[] tri = mesh.triangles;
        for (int t = 0; t + 2 < tri.Length; t += 3)
        {
            Vector3 a = v[tri[t]], b = v[tri[t + 1]], c = v[tri[t + 2]];
            float vol = Vector3.Dot(a, Vector3.Cross(b, c)) / 6f;
            volume += vol;
            Vector3 s = a + b + c;
            first += vol * s / 4f;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    second[i, j] += vol / 20f * (a[i] * a[j] + b[i] * b[j] + c[i] * c[j] + s[i] * s[j]);
        }
        Object.Destroy(mesh);

        // Winding direction does not matter for a closed mesh: flip everything if the volume is negative.
        float sign = volume < 0 ? -1f : 1f;
        float meshMass = density * (float)(sign * volume);
        Vector3 moment = density * sign * first;              // mass * position
        Matrix4x4 secondMass = Scale(second, density * sign);  // integral of rho x x^T dV

        float mass = meshMass;
        var motors = new Vector3[4];
        var seen = new bool[4];
        foreach (CustomFrameLoader.NodeData n in nodes)
        {
            Vector3 p = StlMeshLoader.ConvertZUpToYUp(n.position);
            mass += n.massKg;
            moment += n.massKg * p;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    secondMass[i, j] += n.massKg * p[i] * p[j];
            if (n.type == CustomFrameLoader.NodeType.Motor)
            {
                int index = MotorQuadrants.MotorIndex(MotorQuadrants.ClassifyUnity(p));
                motors[index] = p;
                seen[index] = true;
            }
        }
        if (!(mass > 0f) || !seen[0] || !seen[1] || !seen[2] || !seen[3])
        {
            error = "no mass or not one motor per corner";
            return false;
        }

        // Inertia about the COM: I = trace(S) E - S, with S the second moment about the COM.
        Vector3 com = moment / mass;
        float Sxx = secondMass[0, 0] - mass * com.x * com.x;
        float Syy = secondMass[1, 1] - mass * com.y * com.y;
        float Szz = secondMass[2, 2] - mass * com.z * com.z;
        Vector3 inertia = new Vector3(Syy + Szz, Sxx + Szz, Sxx + Syy);

        if (!Flight.TryControlAuthority(motors, com, mass, inertia, out authority))
        {
            error = "the motor layout gives no control on some axis";
            return false;
        }
        error = null;
        return true;
    }

    private static Matrix4x4 Scale(Matrix4x4 m, float k)
    {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                m[i, j] *= k;
        return m;
    }
}

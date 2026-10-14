using System.Collections.Generic;
using System.Text;
using UnityEngine;

/// <summary>
/// Single source of truth for the motor-to-propeller-corner contract enforced by
/// <c>CustomFrameLoader.RepositionPropellers</c> (CustomFrameLoader.cs:82-128).
///
/// The node-placement UI validates against this so it can never disagree with the loader:
/// anything <see cref="TryValidateMotorLayout"/> accepts is exactly what RepositionPropellers
/// will accept, and every quadrant name the UI shows is the corner the loader will pick.
///
/// Positions are in the STL/CAD Z-up frame (metres), the same frame as
/// <c>CustomFrameLoader.NodeData.position</c>. In that frame +X is right, +Y is forward and
/// +Z is up; Z is ignored entirely for quadrant assignment.
/// </summary>
public static class MotorQuadrants
{
    public enum Quadrant
    {
        FrontLeft = 0,
        FrontRight = 1,
        BackLeft = 2,
        BackRight = 3
    }

    /// <summary>Number of motor nodes RepositionPropellers requires - one per corner.</summary>
    public const int RequiredMotorCount = 4;

    private const int QuadrantCount = 4;

    /// <summary>
    /// Classifies a CAD-frame node position into the corner RepositionPropellers would pick.
    ///
    /// This deliberately performs the SAME two steps as CustomFrameLoader.cs:93-95 - convert to
    /// Unity Y-up first, then test the converted .z and .x - rather than short-circuiting to the
    /// equivalent CAD-frame test (y >= 0, x &lt; 0). Going through the same conversion means that
    /// if ConvertZUpToYUp is ever changed, this follows it automatically instead of silently
    /// drifting out of agreement with the loader.
    /// </summary>
    public static Quadrant Classify(Vector3 stlPosition)
    {
        Vector3 position = StlMeshLoader.ConvertZUpToYUp(stlPosition);
        bool isFront = position.z >= 0f;
        bool isLeft = position.x < 0f;

        if (isFront && isLeft) return Quadrant.FrontLeft;
        if (isFront && !isLeft) return Quadrant.FrontRight;
        if (!isFront && isLeft) return Quadrant.BackLeft;
        return Quadrant.BackRight;
    }

    /// <summary>
    /// True when a position sits exactly on one of the two dividing planes.
    ///
    /// Tested against the raw CAD values rather than the converted ones: this reports back what
    /// the user actually typed, and the conversion is a quaternion multiply that can turn a typed
    /// 0 into a small non-zero float.
    /// </summary>
    public static bool IsOnBoundary(Vector3 stlPosition)
    {
        return stlPosition.x == 0f || stlPosition.y == 0f;
    }

    /// <summary>Human-readable corner name, e.g. "front-left".</summary>
    public static string DisplayName(Quadrant quadrant)
    {
        switch (quadrant)
        {
            case Quadrant.FrontLeft: return "front-left";
            case Quadrant.FrontRight: return "front-right";
            case Quadrant.BackLeft: return "back-left";
            default: return "back-right";
        }
    }

    /// <summary>
    /// Checks a full node list against the RepositionPropellers contract: exactly four Motor
    /// nodes, one in each quadrant. Non-motor nodes are ignored, matching the loader's own
    /// filter at CustomFrameLoader.cs:87-88.
    /// </summary>
    /// <param name="message">Empty when valid; otherwise a specific description of the failure.</param>
    /// <returns>True when the layout will satisfy RepositionPropellers.</returns>
    public static bool TryValidateMotorLayout(IEnumerable<CustomFrameLoader.NodeData> nodes, out string message)
    {
        List<Vector3> motors = new List<Vector3>();
        if (nodes != null)
        {
            foreach (CustomFrameLoader.NodeData node in nodes)
            {
                if (node.type == CustomFrameLoader.NodeType.Motor)
                    motors.Add(node.position);
            }
        }

        if (motors.Count != RequiredMotorCount)
        {
            message = $"Need exactly {RequiredMotorCount} motor nodes, one per quadrant - found {motors.Count}.";
            return false;
        }

        int[] counts = new int[QuadrantCount];
        foreach (Vector3 motor in motors)
            counts[(int)Classify(motor)]++;

        // Special case 1: everything in one corner. The generic "three motors in front-right,
        // none in ..." message would send the user hunting for a typo when the real problem is
        // that the whole coordinate set is offset.
        for (int i = 0; i < QuadrantCount; i++)
        {
            if (counts[i] == RequiredMotorCount)
            {
                message = "All motors are in the same quadrant. The CAD origin is probably not " +
                          "at the frame centre - coordinates must be relative to the frame's " +
                          "centre, not a corner.";
                return false;
            }
        }

        // Special case 2: a motor sitting exactly on a dividing plane. Reported before the
        // generic message because the asymmetric boundaries make the generic wording actively
        // misleading - a plus-config quad reads as "two motors in front-right" when in fact
        // every motor was placed symmetrically.
        foreach (Vector3 motor in motors)
        {
            if (!IsOnBoundary(motor))
                continue;

            message = $"Motor at x={motor.x:F3}, y={motor.y:F3} sits exactly on a quadrant " +
                      "boundary. The boundaries are asymmetric (y=0 counts as front, x=0 counts " +
                      "as right), so a plus-configuration quad with motors on the axes cannot be " +
                      "classified correctly. Use an X configuration, or offset each motor off the axes.";
            return false;
        }

        // With exactly four motors across four quadrants, a duplicate always implies a gap and
        // vice versa, so both lists are populated together whenever the layout is wrong.
        List<string> duplicated = new List<string>();
        List<string> missing = new List<string>();
        for (int i = 0; i < QuadrantCount; i++)
        {
            Quadrant quadrant = (Quadrant)i;
            if (counts[i] > 1)
                duplicated.Add($"{CountWord(counts[i])} in {DisplayName(quadrant)}");
            else if (counts[i] == 0)
                missing.Add(DisplayName(quadrant));
        }

        if (duplicated.Count == 0 && missing.Count == 0)
        {
            message = string.Empty;
            return true;
        }

        StringBuilder builder = new StringBuilder();
        if (duplicated.Count > 0)
            builder.Append(Capitalize(Join(duplicated, "and"))).Append(" motors");
        if (duplicated.Count > 0 && missing.Count > 0)
            builder.Append(", ");
        if (missing.Count > 0)
            builder.Append(duplicated.Count > 0 ? "none in " : "No motor in ").Append(Join(missing, "or"));
        builder.Append('.');

        message = builder.ToString();
        return false;
    }

    private static string CountWord(int count)
    {
        switch (count)
        {
            case 2: return "two";
            case 3: return "three";
            case 4: return "four";
            default: return count.ToString();
        }
    }

    private static string Join(List<string> parts, string conjunction)
    {
        if (parts.Count == 1) return parts[0];
        if (parts.Count == 2) return $"{parts[0]} {conjunction} {parts[1]}";

        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < parts.Count; i++)
        {
            if (i > 0) builder.Append(i == parts.Count - 1 ? $", {conjunction} " : ", ");
            builder.Append(parts[i]);
        }
        return builder.ToString();
    }

    private static string Capitalize(string value)
    {
        return string.IsNullOrEmpty(value) ? value : char.ToUpperInvariant(value[0]) + value.Substring(1);
    }
}

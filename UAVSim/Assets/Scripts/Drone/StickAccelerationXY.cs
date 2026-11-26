using UnityEngine;

/// <summary>
/// PX4 v1.17.0 MPC_POS_MODE 4 ("acceleration based") stick shaping, ported from
/// src/modules/flight_mode_manager/tasks/Utility/StickAccelerationXY.cpp/.hpp, with the helpers it
/// uses: src/lib/sticks/Sticks.cpp (limitStickUnitLengthXY, rotateIntoHeadingFrameXY),
/// src/lib/slew_rate/SlewRate.hpp, src/lib/mathlib/math/filter/AlphaFilter.hpp and
/// src/lib/mathlib/math/Functions.hpp (sqrt_linear, interpolate). Source fetched at tag v1.17.0
/// on 2026-11-22. Spec: .plans/posctl-yaw-reversal-spec.md section 4.
///
/// Sim substitutions (owner decisions D5, D6 in that spec):
///   - <see cref="VelocityScale"/> (Flight.KeyboardMaxSpeed x MaxSpeed) stands in for MPC_VEL_MANUAL
///     in every formula: the velocity scale, the drag coefficient and the brake boost's
///     sqrt(MPC_VEL_MANUAL). PX4's dynamic velocity constraint (setVelocityConstraint, its slew at
///     MPC_ACC_HOR) is not used, so acceleration_sc = MPC_ACC_HOR always.
///   - <see cref="TiltLimit"/> is MPC_MAN_TILT_MAX, not takeoff_status.tilt_limit (MPC_TILTMAX_AIR).
///   - <see cref="Gravity"/> is the sim's gravity (PX4 uses CONSTANTS_ONE_G), so the tilt limit
///     matches the plant exactly.
///   - Collision prevention is not ported (not used on the real drone). MPC_VEL_MAN_SIDE and
///     MPC_VEL_MAN_BACK are -1 in uav-neo.params, so their scaling is skipped as in PX4.
///
/// Axes: Vector2 (x, y) = world (x, z) in Unity. Stick input is (forward, right). Headings are
/// Unity yaw in degrees (clockwise seen from above, 0 = +z).
/// </summary>
public class StickAccelerationXY
{
    /// <summary>FLT_EPSILON, as PX4 compares squared norms against it.</summary>
    private const float FltEpsilon = 1.1920929e-7f;

    /// <summary>Brake-boost filter time constant (s), hard-coded in PX4 calculateDrag.</summary>
    public const float BrakeBoostTimeConstant = 0.8f;

    /// <summary>Velocity scale (m/s): replaces MPC_VEL_MANUAL (D5). Must be &gt; 0.</summary>
    public float VelocityScale = 1f;

    /// <summary>MPC_ACC_HOR (m/s^2).</summary>
    public float AccHor = 3f;

    /// <summary>MPC_JERK_MAX (m/s^3).</summary>
    public float JerkMax = 8f;

    /// <summary>Tilt limit (rad) for applyTiltLimit: MPC_MAN_TILT_MAX in the sim (D6).</summary>
    public float TiltLimit = 15f * Mathf.Deg2Rad;

    /// <summary>Gravity (m/s^2) for applyTiltLimit.</summary>
    public float Gravity = 9.81f;

    /// <summary>Position setpoint (world x, z); NaN while the position is not locked.</summary>
    public Vector2 PositionSetpoint { get; private set; } = new Vector2(float.NaN, float.NaN);

    public Vector2 VelocitySetpoint { get; private set; }
    public Vector2 AccelerationSetpoint { get; private set; }

    /// <summary>Brake-boost filter state (1 with the stick moved, toward max(2, sqrt(velocity scale)) released).</summary>
    public float BrakeBoost { get; private set; } = 1f;

    /// <summary>The drag subtracted in the last call (m/s^2), and the coefficient before the boost (1/s).</summary>
    public Vector2 LastDrag { get; private set; }
    public float DragCoefficient { get; private set; }

    public bool PositionLocked => float.IsFinite(PositionSetpoint.x) && float.IsFinite(PositionSetpoint.y);

    private Vector2 accelerationSetpointPrev;
    private float slewX, slewY; // SlewRate<float> _acceleration_slew_rate_x/_y state

    public void ResetPosition() => PositionSetpoint = new Vector2(float.NaN, float.NaN);

    public void ResetPosition(Vector2 position) => PositionSetpoint = position;

    public void ResetVelocity(Vector2 velocity)
    {
        if (IsFinite(velocity)) VelocitySetpoint = velocity;
    }

    public void ResetAcceleration(Vector2 acceleration)
    {
        if (IsFinite(acceleration))
        {
            slewX = acceleration.x;
            slewY = acceleration.y;
        }
    }

    /// <summary>
    /// StickAccelerationXY::generateSetpoints. <paramref name="stickForwardRight"/> is (pitch, roll)
    /// stick, forward and right positive. <paramref name="yawSetpointDeg"/> is NaN while yawing.
    /// <paramref name="velocitySetpointFeedback"/> is the velocity setpoint the position loop used last step.
    /// </summary>
    public void GenerateSetpoints(Vector2 stickForwardRight, float yawDeg, float yawSetpointDeg, Vector2 position,
        Vector2 velocitySetpointFeedback, float dt)
    {
        // maximum commanded velocity (no dynamic constraint in the sim, D5)
        float velocitySc = VelocityScale;
        float accelerationSc = AccHor * (velocitySc / VelocityScale);
        Vector2 velocityScale = new Vector2(velocitySc, velocitySc);
        Vector2 accelerationScale = new Vector2(accelerationSc, accelerationSc);
        accelerationScale *= 2f; // because of drag the average acceleration is half

        // Map stick input to acceleration
        Vector2 stick = stickForwardRight;
        LimitStickUnitLengthXY(ref stick);
        stick = RotateIntoHeadingFrameXY(stick, yawDeg, yawSetpointDeg);
        Vector2 accelerationSetpoint = Vector2.Scale(stick, accelerationScale);

        // Add drag to limit speed and brake again
        DragCoefficient = accelerationScale.x / velocityScale.x;
        Vector2 drag = CalculateDrag(new Vector2(accelerationScale.x / velocityScale.x, accelerationScale.y / velocityScale.y), dt, stick, VelocitySetpoint);

        // Don't allow the drag to change the sign of the velocity, otherwise we might get into
        // oscillations around 0, due to discretization
        if ((accelerationSetpoint.sqrMagnitude < FltEpsilon
             || Sign(accelerationSetpointPrev.x) != Sign(accelerationSetpoint.x)
             || Sign(accelerationSetpointPrev.y) != Sign(accelerationSetpoint.y))
            && VelocitySetpoint.sqrMagnitude < drag.sqrMagnitude * dt * dt)
        {
            drag = Vector2.zero;
            VelocitySetpoint = Vector2.zero;
        }
        LastDrag = drag;

        accelerationSetpoint -= drag;

        accelerationSetpoint = ApplyJerkLimit(accelerationSetpoint, dt);
        accelerationSetpoint = ApplyTiltLimit(accelerationSetpoint);
        AccelerationSetpoint = accelerationSetpoint;

        // Generate velocity setpoint by forward integrating commanded acceleration
        VelocitySetpoint += accelerationSetpoint * dt;

        LockPosition(position, velocitySetpointFeedback);
        accelerationSetpointPrev = accelerationSetpoint;
    }

    /// <summary>StickAccelerationXY::applyJerkLimit: per-axis slew at MPC_JERK_MAX scaled by the
    /// direction of the increment from the last (final) acceleration setpoint.</summary>
    private Vector2 ApplyJerkLimit(Vector2 acceleration, float dt)
    {
        Vector2 dir = UnitOrZero(acceleration - accelerationSetpointPrev);
        float jerkMaxX = Mathf.Abs(dir.x) * JerkMax;
        float jerkMaxY = Mathf.Abs(dir.y) * JerkMax;
        slewX = SlewUpdate(slewX, acceleration.x, jerkMaxX, dt);
        slewY = SlewUpdate(slewY, acceleration.y, jerkMaxY, dt);
        return new Vector2(slewX, slewY);
    }

    /// <summary>SlewRate::update.</summary>
    private static float SlewUpdate(float value, float newValue, float slewRate, float dt)
    {
        float dvalueMax = slewRate * dt;
        return value + Mathf.Clamp(newValue - value, -dvalueMax, dvalueMax);
    }

    /// <summary>StickAccelerationXY::calculateDrag.</summary>
    private Vector2 CalculateDrag(Vector2 dragCoefficient, float dt, Vector2 stick, Vector2 velocitySetpoint)
    {
        // AlphaFilter::setParameters(dt, 0.8) then update
        float alpha = dt / (BrakeBoostTimeConstant + dt);
        float target = stick.sqrMagnitude < FltEpsilon ? Mathf.Max(2f, Mathf.Sqrt(VelocityScale)) : 1f;
        BrakeBoost += alpha * (target - BrakeBoost);

        dragCoefficient *= BrakeBoost;

        // increase drag with squareroot function when velocity is lower than 1m/s
        Vector2 velocityWithSqrtBoost = UnitOrZero(velocitySetpoint) * SqrtLinear(velocitySetpoint.magnitude);

        // only apply the drag increase below 1m/s when actually braking such that speeds below 1m/s
        // are exactly reached but do so by blending it with the filter to avoid any discontinuity
        float brakeScale = Interpolate(BrakeBoost, 1f, 2f, 0f, 1f);
        Vector2 mixedVelocity = brakeScale * velocityWithSqrtBoost + (1f - brakeScale) * velocitySetpoint;

        return Vector2.Scale(dragCoefficient, mixedVelocity);
    }

    /// <summary>StickAccelerationXY::applyTiltLimit.</summary>
    private Vector2 ApplyTiltLimit(Vector2 acceleration)
    {
        float acc = acceleration.magnitude;
        float accTiltMax = Mathf.Tan(TiltLimit) * Gravity;
        if (acc > accTiltMax)
            acceleration *= accTiltMax / acc;
        return acceleration;
    }

    /// <summary>StickAccelerationXY::lockPosition.</summary>
    private void LockPosition(Vector2 position, Vector2 velocitySetpointFeedback)
    {
        bool moving = VelocitySetpoint.sqrMagnitude > FltEpsilon;
        bool positionLocked = PositionLocked;

        // lock position
        if (!moving && !positionLocked)
            PositionSetpoint = position;

        // open position loop
        if (moving && positionLocked)
        {
            PositionSetpoint = new Vector2(float.NaN, float.NaN);
            // avoid velocity setpoint jump caused by ignoring remaining position error
            if (IsFinite(velocitySetpointFeedback))
                VelocitySetpoint = velocitySetpointFeedback;
        }
    }

    /// <summary>Sticks::limitStickUnitLengthXY.</summary>
    public static void LimitStickUnitLengthXY(ref Vector2 v)
    {
        float vl = v.magnitude;
        if (vl > 1f) v /= vl;
    }

    /// <summary>
    /// Sticks::rotateIntoHeadingFrameXY: rotates (forward, right) by the yaw setpoint when it is
    /// finite (heading locked), otherwise by the current yaw. Result in world (x, z).
    /// </summary>
    public static Vector2 RotateIntoHeadingFrameXY(Vector2 forwardRight, float yawDeg, float yawSetpointDeg)
    {
        float yawRotate = float.IsFinite(yawSetpointDeg) ? yawSetpointDeg : yawDeg;
        float s = Mathf.Sin(yawRotate * Mathf.Deg2Rad), c = Mathf.Cos(yawRotate * Mathf.Deg2Rad);
        // forward = (sin, cos), right = (cos, -sin) in world (x, z)
        return new Vector2(forwardRight.x * s + forwardRight.y * c, forwardRight.x * c - forwardRight.y * s);
    }

    /// <summary>math::sqrt_linear.</summary>
    public static float SqrtLinear(float value) => value < 0f ? 0f : value < 1f ? Mathf.Sqrt(value) : value;

    /// <summary>math::interpolate.</summary>
    public static float Interpolate(float value, float xLow, float xHigh, float yLow, float yHigh)
    {
        if (value <= xLow) return yLow;
        if (value > xHigh) return yHigh;
        float a = (yHigh - yLow) / (xHigh - xLow);
        float b = yLow - a * xLow;
        return a * value + b;
    }

    /// <summary>matrix::sign: -1, 0 or 1.</summary>
    private static int Sign(float v) => (0f < v ? 1 : 0) - (v < 0f ? 1 : 0);

    /// <summary>matrix::Vector::unit_or_zero (eps 1e-5).</summary>
    private static Vector2 UnitOrZero(Vector2 v)
    {
        float n = v.magnitude;
        return n > 1e-5f ? v / n : Vector2.zero;
    }

    private static bool IsFinite(Vector2 v) => float.IsFinite(v.x) && float.IsFinite(v.y);
}

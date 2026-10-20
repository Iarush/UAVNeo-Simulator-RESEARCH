using UnityEngine;

/// <summary>
/// Accurate quadrotor flight physics based on the full 6-DOF equations of motion.
///
/// Reference: Wil Selby, "System Modeling" (ArduCopter research)
/// https://wilselby.com/research/arducopter/modeling/
///
/// Implements:
///   - 4-motor model (spec .plans/real-flight-physics-spec.md, phase 1): motors auto-sized to
///     hover at MPC_THR_HOVER, first-order spin-up/down lag (sub-stepped), each motor's thrust
///     applied at its own position, yaw reaction KM * thrust per motor
///   - Allocation from the frame's real motor positions about the centre of mass
///   - Translational dynamics: gravity + thrust (rotated to global frame) + aerodynamic drag
///   - Rotational dynamics: motor torques + propeller gyroscopic effects
///     (rigid-body gyroscopic term wxIw is handled by Unity's Rigidbody inertia tensor)
///   - PD attitude stabilization controller (inner loop) with properly tuned gains
///   - Motor saturation handling (attitude priority over altitude)
///   - Angular velocity safety limiting
///   - Position hold mode: active velocity damping when no horizontal input
///   - Altitude hold mode: PD altitude controller when no vertical input
///   - Arm/disarm state: motors off until armed, auto-launch on first arm
///
/// The user-facing API remains unchanged: ForwardSpeed, StrafeSpeed, VerticalSpeed,
/// YawSpeed as [-1, 1] inputs, plus MaxSpeed as a scaling factor.
/// Internally, these inputs are translated to desired attitude angles and thrust,
/// which are then achieved through a PD controller that computes individual motor speeds.
/// </summary>
public class Flight : DroneModule
{
    #region Constants â€” Physical Parameters

    /// <summary>
    /// The default value of MaxSpeed (user-facing throttle scale, 0-1).
    /// 0.50 gives responsive flight out-of-the-box; user can still bump up to 1.0.
    /// </summary>
    public const float DefaultMaxSpeed = 0.50f;

    // --------- Drone Geometry ---------

    /// <summary>
    /// Distance from the center of mass to each motor (meters).
    /// Measured from propeller center positions in our OBJ model.
    /// </summary>
    private float armLength = 0.16f;

    // --------- Motor / Propeller Model ---------
    // Spec: .plans/real-flight-physics-spec.md, section 4.2. Every frame's motors are auto-sized
    // so it hovers at MPC_THR_HOVER: Tmax = m*g / (4 * MPC_THR_HOVER) per motor.

    /// <summary>Maximum thrust of one motor (N), set per frame in Start().</summary>
    private float motorThrustMax = 1f;

    /// <summary>
    /// Thrust coefficient kf (N/(rad/s)^2), per frame: Tmax / motorSpeedMax^2. Only used to show
    /// thrust as a motor speed (MotorSpeeds), so the propeller spin range is the same for every frame.
    /// </summary>
    private float kf = 9.13e-6f;

    /// <summary>
    /// Yaw reaction moment per unit thrust (m), PX4 CA_ROTOR*_KM.
    /// </summary>
    private float yawMomentPerThrust = 0.05f;

    /// <summary>Motor time constants (s) for spin-up and spin-down (flight-sim.json).</summary>
    private float motorTauUp = 0.0125f;
    private float motorTauDown = 0.025f;

    /// <summary>Motor sub-steps per FixedUpdate (flight-sim.json).</summary>
    private int subSteps = 5;

    /// <summary>
    /// Propeller moment of inertia about the spin axis (kg*m^2).
    /// Used for the propeller gyroscopic effect: JTP * Omega_r * (w x z_body).
    /// </summary>
    private const float JTP = 3.36e-5f;

    /// <summary>
    /// Maximum motor angular velocity (rad/s). Approx 12000 RPM.
    /// </summary>
    private const float motorSpeedMax = 1256.6f;

    /// <summary>
    /// Minimum motor angular velocity (rad/s). Motors idle at 0.
    /// </summary>
    private const float motorSpeedMin = 0f;

    // --------- Inertia Tensor ---------
    // Set on the Rigidbody in Start() so Unity computes wxIw automatically.

    /// <summary>Moment of inertia about the X axis (pitch). kg*m^2.
    /// Very low for racing-style snap response.</summary>
    private float Ixx = 0.003f;

    /// <summary>Moment of inertia about the Y axis (yaw). kg*m^2.
    /// Halved for quick yaw flicks.</summary>
    private float Iyy = 0.006f;

    /// <summary>Moment of inertia about the Z axis (roll). kg*m^2.
    /// Very low for racing-style snap response.</summary>
    private float Izz = 0.003f;

    // --------- Aerodynamic Drag ---------

    /// <summary>
    /// Quadratic drag coefficient in the horizontal plane (N/(m/s)Â²).
    /// Drag force is proportional to vÂ², giving minimal drag at low speed (explosive
    /// acceleration) and strong drag near terminal velocity (same max speed).
    /// Terminal velocity at full tilt (35Â°): v_max = sqrt(m*g*tan(35Â°) / dragXZ) â‰ˆ 35 m/s.
    /// </summary>
    private const float dragXZ = 0.011f;

    /// <summary>
    /// Linear drag coefficient along the vertical axis (N/(m/s)).
    /// Lower = faster ascent/descent response.
    /// </summary>
    private const float dragY = 0.12f;

    /// <summary>
    /// Rotational drag coefficient (N*m/(rad/s)), flight-sim.json angularDragCoeff.
    /// </summary>
    private float angularDragCoeff = 0.005f;

    // --------- PX4 attitude and rate loops (v1.17.0 stock gains, FlightConfig.Px4StockGains) ---------
    // Vectors are in Unity body axes: x = pitch, y = yaw, z = roll.

    private Vector3 attitudeP;      // MC_PITCH_P, MC_YAW_P, MC_ROLL_P
    private float yawWeight;        // MC_YAW_WEIGHT
    private Vector3 rateMax;        // MC_PITCHRATE_MAX, MC_YAWRATE_MAX, MC_ROLLRATE_MAX (rad/s)
    private Vector3 rateP, rateI, rateD, rateFF, rateIntLim; // K already multiplied into P, I, D
    private Vector3 rateIntegral;
    private Vector3 previousRate;
    private bool haveRate;
    private Vector3 saturationPositive, saturationNegative; // 1 = that axis saturated last allocation

    /// <summary>PX4-normalised mix (motor command per unit normalised torque) and its column scales.</summary>
    private float[,] mixNormalised = new float[4, 3]; // columns: pitch(x), yaw(y), roll(z)

    /// <summary>Body-frame inertia tensor and its inverse (for sub-step rate propagation).</summary>
    private Matrix4x4 inertiaBody = Matrix4x4.identity;
    private Matrix4x4 inertiaBodyInverse = Matrix4x4.identity;

    /// <summary>PX4 ControlAllocationSequentialDesaturation: extra yaw headroom.</summary>
    private const float MinimumYawMargin = 0.15f;

    // --------- User Input Limits ---------

    /// <summary>
    /// Maximum tilt angle the user can command (degrees).
    /// 35Â° at full scale â†’ ~35 m/s terminal velocity.
    /// </summary>
    private const float maxTiltAngle = 35f;

    /// <summary>
    /// Maximum yaw rate the user can command (degrees/second), from flight-sim.json
    /// (pcmdMaxYawRateDegS: 1 rad/s, the real drone's limit).
    /// </summary>
    private float maxYawRate = 57.29578f;

    /// <summary>
    /// Maximum additional vertical thrust as a fraction of hover thrust.
    /// 1.8 = very punchy vertical movement.
    /// </summary>
    private const float maxVerticalThrustFraction = 1.8f;

    /// <summary>
    /// Maximum angular velocity magnitude (rad/s) before safety clamping: 1.5x the largest PX4
    /// rate limit, so it never cuts into what the rate loop commands (set in ConfigureController).
    /// </summary>
    private float maxAngularVelocity = 3.0f;

    /// <summary>
    /// Maximum tilt compensation factor for altitude hold under tilt.
    /// </summary>
    private const float maxTiltCompensation = 1.15f;

    /// <summary>
    /// Maximum bank (roll) angle induced by yaw input for coordinated turns (degrees).
    /// Creates a natural banking feeling during yaw rotation, coupling yaw and roll
    /// like a real aircraft. Scales with MaxSpeed.
    /// </summary>
    private const float bankAngle = 15f;

    // --------- Position Hold ---------

    /// <summary>
    /// Gain for velocity-based position hold (normalized input per m/s of raw velocity).
    /// When no horizontal stick input, the drone tilts to oppose its velocity.
    /// At 2 m/s raw: input = 1.2 -> clamped to 1.0 -> max tilt -> aggressive braking.
    /// </summary>
    private const float posHoldVelGain = 0.60f;

    /// <summary>
    /// Heavy linear drag applied in the horizontal plane when no stick input.
    /// This simulates the drone's flight controller actively braking.
    /// </summary>
    private const float posHoldBrakeDrag = 4.5f;

    /// <summary>
    /// Extra drag applied when the stick direction opposes the current velocity.
    /// This lets the drone decelerate hard on direction changes without reducing top speed.
    /// </summary>
    private const float reverseBrakeDrag = 2.5f;

    /// <summary>
    /// Heavy linear drag applied to the velocity component PERPENDICULAR to stick input.
    /// When the pilot changes direction (e.g. forward â†’ right), this kills the unwanted
    /// cross-velocity almost instantly, giving razor-sharp directional control without
    /// affecting acceleration or top speed in the desired direction.
    /// </summary>
    private const float crossBrakeDrag = 6.0f;

    /// <summary>
    /// Horizontal speed (m/s) below which the drone snaps to a full stop.
    /// Prevents tiny residual drift after braking.
    /// </summary>
    private const float posHoldDeadZone = 0.08f;

    // --------- Altitude Hold ---------

    /// <summary>
    /// Proportional gain for altitude hold (thrust fraction per meter of altitude error).
    /// At 1m below target: 0.5 * hoverThrust extra -> steady climb.
    /// Clamped to maxVerticalThrustFraction.
    /// </summary>
    private const float Kp_alt = 0.5f;

    /// <summary>
    /// Derivative gain for altitude hold (thrust fraction per m/s of vertical velocity).
    /// Damps vertical oscillation around target altitude.
    /// </summary>
    private const float Kd_alt = 0.8f;

    // --------- Launch ---------

    /// <summary>
    /// Altitude above the starting position (meters) that the drone auto-ascends to on launch.
    /// </summary>
    private const float launchAltitude = 7.5f;

    #endregion

    #region Public Interface

    /// <summary>
    /// The input forward/backward speed, ranging from -1 (full backward) to 1 (full forward).
    /// </summary>
    public float ForwardSpeed { get; set; } = 0;

    /// <summary>
    /// The input strafe (left/right) speed, ranging from -1 (full left) to 1 (full right).
    /// </summary>
    public float StrafeSpeed { get; set; } = 0;

    /// <summary>
    /// The input vertical speed, ranging from -1 (full descent) to 1 (full ascent).
    /// </summary>
    public float VerticalSpeed { get; set; } = 0;

    /// <summary>
    /// The input yaw rotation, ranging from -1 (full left) to 1 (full right).
    /// </summary>
    public float YawSpeed { get; set; } = 0;

    /// <summary>
    /// The max speed multiplier set by the user, ranging from 0 to 1.
    /// </summary>
    public float MaxSpeed { get; set; } = Flight.DefaultMaxSpeed;

    /// <summary>
    /// Current motor angular velocities (rad/s) for all 4 motors.
    /// Index 0 = front-right, 1 = front-left, 2 = back-left, 3 = back-right.
    /// </summary>
    public float[] MotorSpeeds { get; private set; } = new float[4];

    /// <summary>
    /// Current thrust of each motor (N) after the spin-up/down lag, same order as MotorSpeeds.
    /// </summary>
    public float[] MotorThrusts { get; private set; } = new float[4];

    /// <summary>
    /// Normalised motor commands (0-1, PX4 actuator_motors), after crash damage, same order.
    /// </summary>
    public float[] MotorCommands { get; private set; } = new float[4];

    /// <summary>
    /// Motor positions in the drone's local frame (from the propellers), same order.
    /// </summary>
    public Vector3[] MotorPositions { get; private set; } = new Vector3[4];

    /// <summary>Maximum thrust of one motor (N): weight / (4 * MPC_THR_HOVER).</summary>
    public float MotorThrustMax => this.motorThrustMax;

    /// <summary>
    /// Current motor RPMs for visualization / telemetry.
    /// </summary>
    public float[] MotorRPMs
    {
        get
        {
            float[] rpms = new float[4];
            for (int i = 0; i < 4; i++)
            {
                rpms[i] = MotorSpeeds[i] * 60f / (2f * Mathf.PI);
            }
            return rpms;
        }
    }

    /// <summary>
    /// Whether the drone is armed (motors active). Starts false.
    /// </summary>
    public bool IsArmed { get; private set; } = false;

    /// <summary>
    /// Whether the drone is currently in the auto-launch ascent phase.
    /// </summary>
    public bool IsLaunching { get; private set; } = false;

    /// <summary>
    /// Whether the drone is in auto-land mode (descending gently to ground, then disarming).
    /// </summary>
    public bool IsLanding { get; private set; } = false;

    /// <summary>
    /// The current target altitude for altitude hold (world Y coordinate).
    /// </summary>
    public float TargetAltitude { get; private set; } = 0f;

    /// <summary>
    /// The Y position of the ground where the drone spawned. Used as landing target.
    /// </summary>
    private float groundY = 0f;

    /// <summary>
    /// Arms the drone and begins the auto-launch sequence.
    /// The drone will ascend to launchAltitude meters above its current position,
    /// then switch to normal pos-hold / alt-hold flight.
    /// </summary>
    public void Arm()
    {
        if (IsArmed) return;
        IsArmed = true;
        IsLaunching = true;
        IsLanding = false;
        groundY = this.transform.position.y;
        TargetAltitude = this.transform.position.y + launchAltitude;

        // Zero out all velocity and force level attitude so launch goes straight up
        this.rBody.linearVelocity = Vector3.zero;
        this.rBody.angularVelocity = Vector3.zero;
        this.transform.rotation = Quaternion.Euler(0f, this.transform.eulerAngles.y, 0f);

        Debug.Log($"[Flight] Armed! Launching to altitude {TargetAltitude:F1}m");
    }

    /// <summary>
    /// Re-triggers the auto-launch sequence (used after crash respawn/reset).
    /// Only works if the drone is already armed.
    /// </summary>
    public void Relaunch()
    {
        if (!IsArmed) return;
        IsLaunching = true;
        IsLanding = false;
        groundY = this.transform.position.y;
        TargetAltitude = this.transform.position.y + launchAltitude;

        // Clean start for relaunch
        this.rBody.linearVelocity = Vector3.zero;
        this.rBody.angularVelocity = Vector3.zero;

        Debug.Log($"[Flight] Relaunching to altitude {TargetAltitude:F1}m");
    }

    /// <summary>
    /// Begins a gentle auto-land sequence. The drone descends under altitude hold
    /// to the surface below, then fully disarms. Works on any surface (ground, rooftops, etc.).
    /// </summary>
    public void BeginLanding()
    {
        if (!IsArmed || IsLanding) return;
        IsLanding = true;
        IsLaunching = false;

        // Clear all user inputs
        this.ForwardSpeed = 0;
        this.StrafeSpeed = 0;
        this.VerticalSpeed = 0;
        this.YawSpeed = 0;

        // Raycast down to find the actual surface below (not just the spawn ground)
        float landingY = groundY;
        RaycastHit hit;
        if (Physics.Raycast(this.transform.position, Vector3.down, out hit, 500f))
        {
            landingY = hit.point.y;
        }

        TargetAltitude = landingY + 0.1f;

        Debug.Log($"[Flight] Landing initiated â€” target surface at Y={landingY:F1}");
    }

    /// <summary>
    /// Fully disarms the drone (called internally when auto-land completes).
    /// </summary>
    private void FinishDisarm()
    {
        IsArmed = false;
        IsLanding = false;
        IsLaunching = false;
        this.ForwardSpeed = 0;
        this.StrafeSpeed = 0;
        this.VerticalSpeed = 0;
        this.YawSpeed = 0;
        this.rBody.linearVelocity = Vector3.zero;
        this.rBody.angularVelocity = Vector3.zero;
        ZeroMotors();
        Debug.Log("[Flight] Landed and disarmed");
    }

    /// <summary>
    /// Stops all drone movement (sets all inputs to zero).
    /// Does NOT disarm the drone.
    /// </summary>
    public void Stop()
    {
        this.ForwardSpeed = 0;
        this.StrafeSpeed = 0;
        this.VerticalSpeed = 0;
        this.YawSpeed = 0;
    }

    #endregion

    #region Private State

    /// <summary>
    /// The rigidbody of the drone.
    /// </summary>
    private Rigidbody rBody;

    // These retain the existing Rigidbody defaults until a custom frame supplies values.
    private Vector3 customCenterOfMass = Vector3.zero;
    private Quaternion customInertiaRotation = Quaternion.identity;
    private float customMass = 0f; // 0 = keep the prefab's Rigidbody mass

    /// <summary>
    /// +1 for a propeller that turns clockwise seen from above (FL, BR), -1 for counter-clockwise
    /// (FR, BL), in MotorSpeeds order. Unity is left-handed, so clockwise from above is a positive
    /// rotation about +Y, and the reaction torque on the body is -spinCW * KM * T about +Y.
    /// </summary>
    private static readonly float[] spinCW = { -1f, 1f, -1f, 1f };

    /// <summary>
    /// Allocation: motor thrusts (N) = allocation * (total thrust, torque x, torque y, torque z),
    /// torques about the centre of mass in body axes. Inverse of the effectiveness matrix built
    /// from the frame's real motor positions in Start().
    /// </summary>
    private float[,] allocation = new float[4, 4];

    /// <summary>
    /// Effectiveness: (total thrust, torque x, torque y, torque z) per newton of each motor.
    /// </summary>
    private float[,] effectiveness = new float[4, 4];


    /// <summary>
    /// Reference to the CrashSystem for damage state queries.
    /// </summary>
    private CrashSystem crashSystem;

    #endregion

    protected override void Awake()
    {
        this.rBody = this.GetComponent<Rigidbody>();
        this.crashSystem = this.GetComponent<CrashSystem>();
        base.Awake();
    }

    /// <summary>
    /// Applies mass properties calculated for a custom drone frame before Start configures
    /// the Rigidbody.
    /// </summary>
    public void ApplyCustomMassProperties(Vector3 com, Vector3 principalMoments,
        Quaternion inertiaRotation, float newArmLength, float totalMassKg)
    {
        this.customMass = totalMassKg;
        this.Ixx = principalMoments.x;
        this.Iyy = principalMoments.y;
        this.Izz = principalMoments.z;
        this.armLength = newArmLength;
        this.customCenterOfMass = com;
        this.customInertiaRotation = inertiaRotation;
    }

    private void Start()
    {
        // === Configure Rigidbody for accurate physics ===
        this.rBody.useGravity = true;
        this.rBody.linearDamping = 0f;
        this.rBody.angularDamping = 0.02f;

        // Mass first: the inertia tensor below is absolute and must not be rescaled by it.
        if (this.customMass > 0f)
            this.rBody.mass = this.customMass;

        // Explicitly set center of mass (the transform origin by default).
        // Without this, Unity computes CoM from the collider shape, which can
        // be offset and causes the drone to behave as if its CoG shifts with tilt.
        this.rBody.centerOfMass = this.customCenterOfMass;

        // Set the inertia tensor so Unity correctly computes the rigid-body
        // gyroscopic term (w x Iw) in Euler's rotation equations.
        this.rBody.inertiaTensor = new Vector3(Ixx, Iyy, Izz);
        this.rBody.inertiaTensorRotation = this.customInertiaRotation;

        // Body-frame tensor R * diag(principal) * R^T, used to propagate the rate between sub-steps.
        Matrix4x4 R = Matrix4x4.Rotate(this.customInertiaRotation);
        this.inertiaBody = R * Matrix4x4.Scale(new Vector3(Ixx, Iyy, Izz)) * R.transpose;
        this.inertiaBodyInverse = R * Matrix4x4.Scale(new Vector3(1f / Ixx, 1f / Iyy, 1f / Izz)) * R.transpose;

        ConfigureController();

        this.rBody.constraints = RigidbodyConstraints.None;
        this.rBody.maxAngularVelocity = maxAngularVelocity * 2f;
        this.rBody.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        this.rBody.interpolation = RigidbodyInterpolation.Interpolate;

        Debug.Log($"[Flight] Applied to rigidbody: mass={this.rBody.mass:F4} centerOfMass={this.rBody.centerOfMass:F4} inertiaTensor={this.rBody.inertiaTensor:F6} inertiaTensorRotation={this.rBody.inertiaTensorRotation} armLength={this.armLength:F4}");

        ConfigureMotors();

        // Initialize target altitude and ground reference
        TargetAltitude = this.transform.position.y;
        groundY = this.transform.position.y;
    }

    private void FixedUpdate()
    {
        // ================================================================
        // 0. ARM CHECK â€” if not armed, zero all motors and do nothing
        // ================================================================
        if (!IsArmed)
        {
            ZeroMotors();
            return;
        }

        // ================================================================
        // 0a. LANDING CHECK â€” if near ground during auto-land, finish disarm
        // ================================================================
        if (IsLanding)
        {
            float altAboveTarget = this.transform.position.y - TargetAltitude;
            float vertSpeed = Mathf.Abs(this.rBody.linearVelocity.y);

            // Landed: close to target surface and barely moving vertically
            if (altAboveTarget < 0.3f && vertSpeed < 0.3f)
            {
                FinishDisarm();
                return;
            }
        }

        // ================================================================
        // 0b. CRASH CHECK â€” skip all motor logic if drone is destroyed
        // ================================================================
        if (crashSystem != null && crashSystem.IsDestroyed)
        {
            ZeroMotors();
            return;
        }

        // ================================================================
        // 1. READ CURRENT STATE
        // ================================================================

        Vector3 euler = this.transform.eulerAngles;
        float currentPitch = NormalizeAngle(euler.x);
        float currentRoll = NormalizeAngle(euler.z);

        Vector3 angVelWorld = this.rBody.angularVelocity;
        Vector3 angVelBody = this.transform.InverseTransformDirection(angVelWorld);
        float p = angVelBody.x; // pitch rate
        float q = angVelBody.y; // yaw rate
        float r = angVelBody.z; // roll rate

        // ================================================================
        // 2. SAFETY: ANGULAR VELOCITY LIMITING
        // ================================================================
        float angVelMag = angVelBody.magnitude;
        if (angVelMag > maxAngularVelocity)
        {
            this.rBody.angularVelocity = this.rBody.angularVelocity * (maxAngularVelocity / angVelMag);
            angVelWorld = this.rBody.angularVelocity;
            angVelBody = this.transform.InverseTransformDirection(angVelWorld);
            p = angVelBody.x;
            q = angVelBody.y;
            r = angVelBody.z;
        }

        // ================================================================
        // 2b. LAUNCH STABILIZATION: kinematic ascent to target altitude.
        //     Bypasses the physics engine entirely â€” directly sets position,
        //     rotation, and velocity each frame. This eliminates the
        //     limit-cycle oscillation caused by MoveRotation fighting motor
        //     torques on a non-kinematic Rigidbody.
        //
        //     If the user provides any horizontal input, launch ends early
        //     and normal motor-mixing takes over immediately.
        // ================================================================
        if (IsLaunching)
        {
            bool userInput = Mathf.Abs(this.ForwardSpeed) > 0.05f || Mathf.Abs(this.StrafeSpeed) > 0.05f;
            if (userInput)
            {
                IsLaunching = false;
                Debug.Log("[Flight] User took over during launch â€” exiting launch stabilization");
                // Fall through to normal motor mixing below
            }
            else
            {
                float altError = TargetAltitude - this.transform.position.y;
                float vertVel = this.rBody.linearVelocity.y;

                // Kinematic ascent: move drone directly upward, no physics forces
                float launchSpeed = Mathf.Clamp(altError * Kp_alt - vertVel * Kd_alt, -maxVerticalThrustFraction * 5f, maxVerticalThrustFraction * 5f);
                Vector3 newPos = this.transform.position + Vector3.up * launchSpeed * Time.fixedDeltaTime;

                // Lock attitude flat at current yaw â€” directly set transform, no physics
                float currentYaw = this.transform.eulerAngles.y;
                this.rBody.Move(newPos, Quaternion.Euler(0f, currentYaw, 0f));

                // Zero all physics velocities so no residual momentum fights us
                this.rBody.linearVelocity = new Vector3(0f, launchSpeed, 0f);
                this.rBody.angularVelocity = Vector3.zero;
                ResetRateController();

                // Telemetry only: hold each motor at its share of hover thrust (lever rule), so
                // the lag model starts from hover when the launch hands over to the motors.
                float launchHoverThrust = this.rBody.mass * Physics.gravity.magnitude;
                for (int i = 0; i < 4; i++)
                {
                    float t = Mathf.Clamp(this.allocation[i, 0] * launchHoverThrust, 0f, this.motorThrustMax);
                    this.MotorThrusts[i] = t;
                    this.MotorCommands[i] = t / this.motorThrustMax;
                    this.MotorSpeeds[i] = Mathf.Clamp(Mathf.Sqrt(t / kf), motorSpeedMin, motorSpeedMax);
                }

                // Check if launch is complete
                if (Mathf.Abs(altError) < 0.5f && Mathf.Abs(vertVel) < 0.5f)
                {
                    IsLaunching = false;
                    TargetAltitude = this.transform.position.y;
                    Debug.Log("[Flight] Launch complete - entering altitude hold");
                }

                return; // Skip motor mixing â€” kinematic launch handles everything
            }
        }

        // ================================================================
        // 3. POSITION HOLD: when no horizontal input, oppose velocity
        // ================================================================
        float effectiveForward = this.ForwardSpeed;
        float effectiveStrafe = this.StrafeSpeed;

        bool noHorizontalInput = Mathf.Abs(this.ForwardSpeed) < 0.05f && Mathf.Abs(this.StrafeSpeed) < 0.05f;
        bool noVerticalInput = Mathf.Abs(this.VerticalSpeed) < 0.05f;
        bool pureVerticalFlight = noHorizontalInput && !noVerticalInput;

        if (noHorizontalInput && !pureVerticalFlight)
        {
            // Convert world velocity to body-frame horizontal components
            Vector3 localVel = this.transform.InverseTransformDirection(this.rBody.linearVelocity);

            // Oppose forward velocity with pitch, lateral velocity with roll
            // localVel.z = forward speed in body frame, localVel.x = right speed
            effectiveForward = Mathf.Clamp(-localVel.z * posHoldVelGain, -1f, 1f);
            effectiveStrafe = Mathf.Clamp(-localVel.x * posHoldVelGain, -1f, 1f);
        }
        else if (pureVerticalFlight)
        {
            // Pure vertical flight (Shift held, no WASD): force level attitude.
            // Position hold tilt at high vertical speeds can cause oscillation/instability.
            // Drag alone handles any horizontal drift; PD controller keeps it level.
            effectiveForward = 0f;
            effectiveStrafe = 0f;
        }

        // ================================================================
        // 4. COMPUTE DESIRED STATE FROM INPUTS
        // ================================================================

        float scale = this.MaxSpeed;

        // Forward input -> desired pitch angle
        float desiredPitch = effectiveForward * maxTiltAngle * scale;

        // Strafe input -> desired roll angle
        float desiredRoll = -effectiveStrafe * maxTiltAngle * scale;

        // Yaw input -> desired yaw rate (always full authority, independent of speed scale)
        float desiredYawRate = this.YawSpeed * maxYawRate * Mathf.Deg2Rad;

        // Banking: yaw input induces a coordinated roll into the turn.
        // Positive YawSpeed (right) â†’ negative desiredRoll (bank right).
        desiredRoll -= this.YawSpeed * bankAngle * scale;

        // ================================================================
        // 5. ALTITUDE HOLD / VERTICAL THRUST
        // ================================================================

        float gravity = Physics.gravity.magnitude;
        float hoverThrust = this.rBody.mass * gravity;
        float thrustOffset;

        if (noVerticalInput)
        {
            // PD altitude hold: maintain TargetAltitude
            float altError = TargetAltitude - this.transform.position.y;
            float vertVel = this.rBody.linearVelocity.y;

            float altFraction = Kp_alt * altError - Kd_alt * vertVel;
            altFraction = Mathf.Clamp(altFraction, -maxVerticalThrustFraction, maxVerticalThrustFraction);
            thrustOffset = altFraction * hoverThrust;

            // If launching and close to target, end launch phase
            if (IsLaunching && Mathf.Abs(altError) < 0.5f && Mathf.Abs(vertVel) < 0.5f)
            {
                IsLaunching = false;
                Debug.Log("[Flight] Launch complete - entering altitude hold");
            }
        }
        else
        {
            // User commanding vertical motion: normal thrust
            thrustOffset = this.VerticalSpeed * scale * maxVerticalThrustFraction * hoverThrust;

            // Continuously update target altitude so that when the user releases
            // the stick, altitude hold kicks in at the current altitude.
            TargetAltitude = this.transform.position.y;
        }

        float desiredTotalThrust = hoverThrust + thrustOffset;

        // Gentle tilt compensation
        float cosRoll = Mathf.Cos(currentRoll * Mathf.Deg2Rad);
        float cosPitch = Mathf.Cos(currentPitch * Mathf.Deg2Rad);
        float tiltFactor = cosRoll * cosPitch;
        if (tiltFactor > 0.01f)
        {
            float compensation = Mathf.Min(1f / tiltFactor, maxTiltCompensation);
            desiredTotalThrust *= compensation;
        }

        // ================================================================
        // 6. PX4 ATTITUDE LOOP (once per FixedUpdate) -> RATE SETPOINT
        //    PX4 v1.17.0 src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp:
        //    reduced attitude (tilt first, yaw weighted by MC_YAW_WEIGHT), rate_sp = P * error
        //    plus the yaw-rate feed-forward, limited by MC_*RATE_MAX.
        // ================================================================

        Quaternion attitude = this.transform.rotation;
        Quaternion attitudeSetpoint = Quaternion.Euler(desiredPitch, euler.y, desiredRoll);
        Vector3 rateSetpoint = AttitudeControl(attitude, attitudeSetpoint, desiredYawRate);

        // ================================================================
        // 7. RATE LOOP + ALLOCATION + MOTOR LAG, SUB-STEPPED (subSteps per FixedUpdate)
        //    Rate loop: PX4 src/lib/rate_control/rate_control.cpp. Allocation: PX4
        //    ControlAllocationPseudoInverse (normalised mix) + ControlAllocationSequentialDesaturation
        //    with MC_AIRMODE 0. Between sub-steps the rate is propagated from the torque applied:
        //    w += I^-1 (tau - w x Iw) dt. PhysX gets the sub-step-averaged forces once per FixedUpdate.
        // ================================================================

        float Fmax = this.motorThrustMax;
        float dtSub = Time.fixedDeltaTime / this.subSteps;
        float globalScale = crashSystem != null ? crashSystem.MotorOutputScale : 1f;
        float[] thrustSum = new float[4];
        Vector3 rate = angVelBody;

        for (int s = 0; s < this.subSteps; s++)
        {
            Vector3 torqueNormalised = RateControl(rate, rateSetpoint, dtSub);
            Allocate(torqueNormalised, desiredTotalThrust, this.MotorCommands);

            // Crash damage scales the command of that corner's motor.
            if (crashSystem != null)
                for (int i = 0; i < 4; i++)
                    this.MotorCommands[i] *= crashSystem.MotorHealth[i] * globalScale;

            Vector3 torque = -this.angularDragCoeff * rate;
            for (int i = 0; i < 4; i++)
            {
                float t = MotorLagStep(this.MotorThrusts[i], this.MotorCommands[i] * Fmax, dtSub, this.motorTauUp, this.motorTauDown);
                this.MotorThrusts[i] = t;
                thrustSum[i] += t;
                Vector3 d = this.MotorPositions[i] - this.rBody.centerOfMass;
                torque += new Vector3(-d.z * t, -spinCW[i] * this.yawMomentPerThrust * t, d.x * t);
            }

            Vector3 Iw = this.inertiaBody.MultiplyVector(rate);
            rate += this.inertiaBodyInverse.MultiplyVector(torque - Vector3.Cross(rate, Iw)) * dtSub;
        }

        // ================================================================
        // 8. APPLY FORCES AND TORQUES (sub-step averages)
        // ================================================================

        float tauYawReaction = 0f;
        float Omega_r = 0f;
        for (int i = 0; i < 4; i++)
        {
            float avg = thrustSum[i] / this.subSteps;
            float wi = Mathf.Clamp(Mathf.Sqrt(this.MotorThrusts[i] / kf), motorSpeedMin, motorSpeedMax);
            this.MotorSpeeds[i] = wi;

            // Each motor pushes along body up at its own position.
            this.rBody.AddForceAtPosition(this.transform.up * avg, this.transform.TransformPoint(this.MotorPositions[i]));
            tauYawReaction += -spinCW[i] * this.yawMomentPerThrust * avg;
            Omega_r += spinCW[i] * wi;
        }

        // Motor drag-torque reaction about body up
        this.rBody.AddRelativeTorque(new Vector3(0f, tauYawReaction, 0f));

        // Propeller gyroscopic effect
        Vector3 propGyro = JTP * Omega_r * new Vector3(-r, 0f, p);
        this.rBody.AddRelativeTorque(propGyro);

        // â”€â”€ Aerodynamic drag (velocity-decomposed) â”€â”€
        // Horizontal velocity is split into a component ALIGNED with the stick
        // direction and a CROSS (perpendicular) component.
        //  â€¢ Aligned: quadratic drag â†’ near-zero at low speed (explosive accel),
        //    strong at terminal velocity (caps top speed).
        //  â€¢ Cross: heavy linear drag â†’ kills unwanted sideways momentum almost
        //    instantly â†’ razor-sharp direction changes.
        // Vertical drag remains simple linear.

        Vector3 vel = this.rBody.linearVelocity;
        float hSpeed = Mathf.Sqrt(vel.x * vel.x + vel.z * vel.z);
        Vector3 dragForce;

        if (noHorizontalInput)
        {
            // Position hold: strong LINEAR braking drag (constant deceleration feel)
            float brakeDrag = posHoldBrakeDrag;

            // Dead zone: snap to zero when nearly stopped
            if (hSpeed < posHoldDeadZone)
            {
                this.rBody.linearVelocity = new Vector3(0f, vel.y, 0f);
                vel = this.rBody.linearVelocity;
            }

            dragForce = new Vector3(
                -brakeDrag * vel.x,
                -dragY * vel.y,
                -brakeDrag * vel.z
            );
        }
        else
        {
            // Active flight: decompose horizontal velocity
            Vector3 velXZ = new Vector3(vel.x, 0f, vel.z);
            Vector3 stickWorld = this.transform.forward * this.ForwardSpeed
                               + this.transform.right * this.StrafeSpeed;
            stickWorld.y = 0f; // Project to horizontal plane â€” transform.forward tilts with the drone

            if (stickWorld.sqrMagnitude > 0.001f && velXZ.sqrMagnitude > 0.01f)
            {
                Vector3 stickDir = stickWorld.normalized;
                float speedXZ = velXZ.magnitude;
                float alignedMag = Vector3.Dot(velXZ, stickDir);
                Vector3 alignedVel = stickDir * alignedMag;
                Vector3 crossVel = velXZ - alignedVel;

                // Aligned component: quadratic drag (+ reverse brake if opposing)
                float alignedCoeff = dragXZ * Mathf.Abs(alignedMag);
                if (alignedMag < 0f) alignedCoeff += reverseBrakeDrag;

                // Cross component: scale braking by how perpendicular the velocity is
                // to the stick. When steering (W+D), velocity is mostly aligned â†’
                // minimal cross-brake â†’ speed preserved. When doing a hard 90Â° swap
                // (release W, press D only), velocity is fully perpendicular â†’ full brake.
                float alignedFraction = Mathf.Clamp01(alignedMag / speedXZ);
                float crossCoeff = crossBrakeDrag * (1f - alignedFraction);

                Vector3 hDrag = -alignedCoeff * alignedVel - crossCoeff * crossVel;
                dragForce = new Vector3(hDrag.x, -dragY * vel.y, hDrag.z);
            }
            else
            {
                // Negligible stick or velocity: fall back to simple quadratic
                float coeff = dragXZ * hSpeed;
                dragForce = new Vector3(-coeff * vel.x, -dragY * vel.y, -coeff * vel.z);
            }
        }

        this.rBody.AddForce(dragForce);

        // Aerodynamic angular drag
        Vector3 angDrag = -angularDragCoeff * angVelBody;
        this.rBody.AddRelativeTorque(angDrag);

        // ================================================================
        // 10. MOMENTUM CONSERVATION DURING YAW
        //     Rotate the horizontal velocity vector to follow the yaw rotation.
        //     This preserves speed when turning â€” the drone redirects its
        //     momentum instead of fighting drag to rebuild speed in a new direction.
        // ================================================================
        if (Mathf.Abs(this.YawSpeed) > 0.05f)
        {
            Vector3 currentVel = this.rBody.linearVelocity;
            float hSpeedSq = currentVel.x * currentVel.x + currentVel.z * currentVel.z;
            if (hSpeedSq > 0.1f)
            {
                float yawDeltaDeg = desiredYawRate * Mathf.Rad2Deg * Time.fixedDeltaTime;
                Vector3 hVel = new Vector3(currentVel.x, 0f, currentVel.z);
                Vector3 rotatedHVel = Quaternion.Euler(0f, yawDeltaDeg, 0f) * hVel;
                this.rBody.linearVelocity = new Vector3(rotatedHVel.x, currentVel.y, rotatedHVel.z);
            }
        }
    }

    /// <summary>
    /// Sizes the motors for this frame, finds their positions and builds the allocation.
    /// Runs in Start(), after the Rigidbody has its mass and centre of mass.
    /// </summary>
    private void ConfigureMotors()
    {
        float hover = FlightConfig.Px4("MPC_THR_HOVER");
        this.yawMomentPerThrust = FlightConfig.Px4("CA_ROTOR0_KM");
        this.motorTauUp = FlightConfig.Sim("motorTimeConstantUp");
        this.motorTauDown = FlightConfig.Sim("motorTimeConstantDown");
        this.subSteps = Mathf.Max(1, Mathf.RoundToInt(FlightConfig.Sim("subSteps")));

        this.motorThrustMax = this.rBody.mass * Physics.gravity.magnitude / (4f * hover);
        this.kf = this.motorThrustMax / (motorSpeedMax * motorSpeedMax);

        // The stock prefab's propeller visuals sit at about 0.72 m, far outside its physical
        // 0.16 m arm and 0.003 kg*m^2 inertia, so only a custom frame (propellers moved onto its
        // motors by CustomFrameLoader) takes positions from the propellers.
        string source;
        if (this.customMass > 0f)
            source = FindMotorPositions(this.MotorPositions) ? "propellers" : "fallback X layout from armLength";
        else
        {
            FallbackMotorPositions(this.MotorPositions, this.armLength);
            source = "stock X layout from armLength";
        }
        if (!BuildAllocation(this.MotorPositions, this.rBody.centerOfMass, this.yawMomentPerThrust, this.allocation))
        {
            Debug.LogError("[Flight] Motor layout gives no control authority on some axis; using the fallback X layout.");
            FallbackMotorPositions(this.MotorPositions, this.armLength);
            source = "fallback X layout from armLength";
            BuildAllocation(this.MotorPositions, this.rBody.centerOfMass, this.yawMomentPerThrust, this.allocation);
        }
        BuildNormalisedMix();

        Debug.Log($"[Flight] Motors: Tmax={this.motorThrustMax:F3} N each (hover at {hover:F2}), kf={this.kf:E3}, KM={this.yawMomentPerThrust:F3}, " +
                  $"tau up/down={this.motorTauUp:F4}/{this.motorTauDown:F4} s, {this.subSteps} sub-steps; positions from {source}: " +
                  $"FR {this.MotorPositions[0]:F4} FL {this.MotorPositions[1]:F4} BL {this.MotorPositions[2]:F4} BR {this.MotorPositions[3]:F4}");
    }

    /// <summary>
    /// Motor positions in the drone's local frame, taken from the four Propeller_N objects
    /// (which CustomFrameLoader has already moved onto a custom frame's motors) and keyed by
    /// corner. Returns false and fills the fallback X layout if there is not one per corner.
    /// </summary>
    private bool FindMotorPositions(Vector3[] positions)
    {
        bool[] seen = new bool[4];
        int count = 0;
        foreach (Transform t in this.GetComponentsInChildren<Transform>(true))
        {
            if (!t.name.StartsWith("Propeller_")) continue;
            Vector3 local = this.transform.InverseTransformPoint(t.position);
            int index = MotorQuadrants.MotorIndex(MotorQuadrants.ClassifyUnity(local));
            if (seen[index]) { count = -1; break; }
            seen[index] = true;
            positions[index] = local;
            count++;
        }
        if (count == 4) return true;

        Debug.LogWarning("[Flight] Did not find one propeller per corner; motor positions use an X layout from armLength.");
        FallbackMotorPositions(positions, this.armLength);
        return false;
    }

    private static void FallbackMotorPositions(Vector3[] positions, float arm)
    {
        float a = arm / Mathf.Sqrt(2f);
        positions[0] = new Vector3(a, 0f, a);    // FR
        positions[1] = new Vector3(-a, 0f, a);   // FL
        positions[2] = new Vector3(-a, 0f, -a);  // BL
        positions[3] = new Vector3(a, 0f, -a);   // BR
    }

    /// <summary>
    /// Builds the allocation (inverse effectiveness): thrust of motor i =
    /// A[i,0]*total + A[i,1]*torqueX + A[i,2]*torqueY + A[i,3]*torqueZ (body axes, about the COM).
    /// A motor at d (from the COM) with thrust T along body up gives torque d x (0, T, 0) =
    /// (-d.z*T, 0, d.x*T), plus the yaw reaction -spinCW*KM*T about up.
    /// </summary>
    private bool BuildAllocation(Vector3[] positions, Vector3 com, float km, float[,] result)
    {
        double[,] b = new double[4, 4];
        for (int i = 0; i < 4; i++)
        {
            Vector3 d = positions[i] - com;
            b[0, i] = 1.0;
            b[1, i] = -d.z;
            b[2, i] = -spinCW[i] * km;
            b[3, i] = d.x;
            for (int a = 0; a < 4; a++) this.effectiveness[a, i] = (float)b[a, i];
        }
        if (!Invert4(b, out double[,] inv)) return false;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
            {
                if (!double.IsFinite(inv[i, j])) return false;
                result[i, j] = (float)inv[i, j];
            }
        return true;
    }

    /// <summary>Gauss-Jordan inverse with partial pivoting; false if (near) singular.</summary>
    private static bool Invert4(double[,] m, out double[,] inv)
    {
        const int n = 4;
        double[,] a = (double[,])m.Clone();
        inv = new double[n, n];
        for (int i = 0; i < n; i++) inv[i, i] = 1.0;

        for (int c = 0; c < n; c++)
        {
            int pivot = c;
            for (int r = c + 1; r < n; r++)
                if (System.Math.Abs(a[r, c]) > System.Math.Abs(a[pivot, c])) pivot = r;
            if (System.Math.Abs(a[pivot, c]) < 1e-9) return false;
            for (int k = 0; k < n; k++)
            {
                (a[c, k], a[pivot, k]) = (a[pivot, k], a[c, k]);
                (inv[c, k], inv[pivot, k]) = (inv[pivot, k], inv[c, k]);
            }
            double scale = 1.0 / a[c, c];
            for (int k = 0; k < n; k++) { a[c, k] *= scale; inv[c, k] *= scale; }
            for (int r = 0; r < n; r++)
            {
                if (r == c) continue;
                double f = a[r, c];
                if (f == 0.0) continue;
                for (int k = 0; k < n; k++) { a[r, k] -= f * a[c, k]; inv[r, k] -= f * inv[c, k]; }
            }
        }
        return true;
    }

    /// <summary>
    /// One step of the first-order motor model: thrust moves toward target with time constant
    /// tauUp when rising and tauDown when falling (exact for a constant target over dt).
    /// </summary>
    public static float MotorLagStep(float thrust, float target, float dt, float tauUp, float tauDown)
    {
        float tau = target > thrust ? tauUp : tauDown;
        if (tau <= 0f) return target;
        return thrust + (target - thrust) * (1f - Mathf.Exp(-dt / tau));
    }

    private void ZeroMotors()
    {
        for (int i = 0; i < 4; i++)
        {
            this.MotorSpeeds[i] = 0f;
            this.MotorThrusts[i] = 0f;
            this.MotorCommands[i] = 0f;
        }
    }

    /// <summary>
    /// Normalizes a Unity Euler angle (0-360) to the range [-180, 180].
    /// </summary>
    private static float NormalizeAngle(float angle)
    {
        if (angle > 180f) angle -= 360f;
        return angle;
    }

    #region PX4 attitude and rate control

    /// <summary>Loads the PX4 stock gains and the real drone's rate limits.</summary>
    private void ConfigureController()
    {
        this.attitudeP = new Vector3(FlightConfig.Px4StockGains.PitchP, FlightConfig.Px4StockGains.YawP, FlightConfig.Px4StockGains.RollP);
        this.yawWeight = FlightConfig.Px4("MC_YAW_WEIGHT");
        this.maxYawRate = FlightConfig.Sim("pcmdMaxYawRateDegS");
        this.rateMax = new Vector3(FlightConfig.Px4("MC_PITCHRATE_MAX"), FlightConfig.Px4("MC_YAWRATE_MAX"), FlightConfig.Px4("MC_ROLLRATE_MAX")) * Mathf.Deg2Rad;

        // mc_rate_control: P, I and D are multiplied by K; FF is not.
        Vector3 k = new Vector3(FlightConfig.Px4StockGains.PitchRateK, FlightConfig.Px4StockGains.YawRateK, FlightConfig.Px4StockGains.RollRateK);
        this.rateP = Vector3.Scale(k, new Vector3(FlightConfig.Px4StockGains.PitchRateP, FlightConfig.Px4StockGains.YawRateP, FlightConfig.Px4StockGains.RollRateP));
        this.rateI = Vector3.Scale(k, new Vector3(FlightConfig.Px4StockGains.PitchRateI, FlightConfig.Px4StockGains.YawRateI, FlightConfig.Px4StockGains.RollRateI));
        this.rateD = Vector3.Scale(k, new Vector3(FlightConfig.Px4StockGains.PitchRateD, FlightConfig.Px4StockGains.YawRateD, FlightConfig.Px4StockGains.RollRateD));
        this.rateFF = new Vector3(FlightConfig.Px4StockGains.PitchRateFF, FlightConfig.Px4StockGains.YawRateFF, FlightConfig.Px4StockGains.RollRateFF);
        this.rateIntLim = new Vector3(FlightConfig.Px4StockGains.PitchRateIntLim, FlightConfig.Px4StockGains.YawRateIntLim, FlightConfig.Px4StockGains.RollRateIntLim);

        this.angularDragCoeff = FlightConfig.Sim("angularDragCoeff");
        this.maxAngularVelocity = 1.5f * Mathf.Max(this.rateMax.x, Mathf.Max(this.rateMax.y, this.rateMax.z));
        ResetRateController();

        Debug.Log($"[Flight] PX4 controller: attitude P (pitch, yaw, roll) {this.attitudeP}, yaw weight {this.yawWeight}, " +
                  $"rate max {this.rateMax * Mathf.Rad2Deg} deg/s, rate P {this.rateP:F3} I {this.rateI:F3} D {this.rateD:F4}");
    }

    private void ResetRateController()
    {
        this.rateIntegral = Vector3.zero;
        this.haveRate = false;
        this.saturationPositive = Vector3.zero;
        this.saturationNegative = Vector3.zero;
    }

    /// <summary>
    /// PX4 AttitudeControl::update (src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp,
    /// v1.17.0). Body-axis rate setpoint (rad/s) from the attitude and its setpoint; the yaw-rate
    /// setpoint (about world up) is fed forward. Unity quaternions, body up = +Y.
    /// </summary>
    private Vector3 AttitudeControl(Quaternion q, Quaternion qd, float yawRateSetpoint)
    {
        // Reduced attitude: the shortest rotation that tilts the thrust axis onto the setpoint's.
        Vector3 ez = q * Vector3.up, ezd = qd * Vector3.up;
        Quaternion qdRed = Quaternion.FromToRotation(ez, ezd) * q;

        // Mix in only yaw_weight of the remaining yaw rotation.
        Quaternion qMix = Canonical(Quaternion.Inverse(qdRed) * qd);
        float w = Mathf.Clamp(qMix.w, -1f, 1f), y = Mathf.Clamp(qMix.y, -1f, 1f);
        qd = qdRed * new Quaternion(0f, Mathf.Sin(this.yawWeight * Mathf.Asin(y)), 0f, Mathf.Cos(this.yawWeight * Mathf.Acos(w)));

        Quaternion qe = Canonical(Quaternion.Inverse(q) * qd);
        Vector3 eq = 2f * new Vector3(qe.x, qe.y, qe.z);
        Vector3 rateSp = Vector3.Scale(eq, this.attitudeP);

        rateSp += (Quaternion.Inverse(q) * Vector3.up) * yawRateSetpoint;

        for (int a = 0; a < 3; a++)
            rateSp[a] = Mathf.Clamp(rateSp[a], -this.rateMax[a], this.rateMax[a]);
        return rateSp;
    }

    private static Quaternion Canonical(Quaternion q) => q.w < 0f ? new Quaternion(-q.x, -q.y, -q.z, -q.w) : q;

    /// <summary>
    /// PX4 RateControl::update (src/lib/rate_control/rate_control.cpp, v1.17.0): normalised torque
    /// = P*e + I + FF*sp - D*angular_accel, integrator with saturation anti-windup, the 400 deg/s
    /// I-factor and MC_*_INT_LIM.
    /// </summary>
    private Vector3 RateControl(Vector3 rate, Vector3 rateSp, float dt)
    {
        Vector3 error = rateSp - rate;
        Vector3 accel = this.haveRate ? (rate - this.previousRate) / dt : Vector3.zero;
        this.previousRate = rate;
        this.haveRate = true;

        Vector3 torque = Vector3.Scale(this.rateP, error) + this.rateIntegral - Vector3.Scale(this.rateD, accel) + Vector3.Scale(this.rateFF, rateSp);

        for (int a = 0; a < 3; a++)
        {
            float e = error[a];
            if (this.saturationPositive[a] > 0f) e = Mathf.Min(e, 0f);
            if (this.saturationNegative[a] > 0f) e = Mathf.Max(e, 0f);
            float iFactor = e / (400f * Mathf.Deg2Rad);
            iFactor = Mathf.Max(0f, 1f - iFactor * iFactor);
            float ri = this.rateIntegral[a] + iFactor * this.rateI[a] * e * dt;
            if (float.IsFinite(ri))
                this.rateIntegral[a] = Mathf.Clamp(ri, -this.rateIntLim[a], this.rateIntLim[a]);
        }
        return torque;
    }

    /// <summary>
    /// PX4 ControlAllocationPseudoInverse::updateControlAllocationMatrixScale/normalize (v1.17.0):
    /// the mix (motor command u per unit torque) from this frame's allocation, with the roll and
    /// pitch columns scaled together by sqrt(|col|^2 / (n/2)) and yaw by its largest entry.
    /// </summary>
    private void BuildNormalisedMix()
    {
        // allocation columns: 0 thrust, 1 torque x (pitch), 2 torque y (yaw), 3 torque z (roll); in N.
        float[,] mu = new float[4, 3];
        for (int i = 0; i < 4; i++)
            for (int a = 0; a < 3; a++)
                mu[i, a] = this.allocation[i, a + 1] / this.motorThrustMax;

        float RollPitchScale(int a)
        {
            float sq = 0f; int n = 0;
            for (int i = 0; i < 4; i++) { sq += mu[i, a] * mu[i, a]; if (Mathf.Abs(mu[i, a]) > 1e-3f) n++; }
            return n > 0 ? Mathf.Sqrt(sq / (n / 2f)) : 1f;
        }
        float rp = Mathf.Max(RollPitchScale(0), RollPitchScale(2));
        float yaw = float.MinValue;
        for (int i = 0; i < 4; i++) yaw = Mathf.Max(yaw, mu[i, 1]);

        float[] scale = { rp, yaw, rp };
        for (int i = 0; i < 4; i++)
            for (int a = 0; a < 3; a++)
            {
                float v = scale[a] > float.Epsilon ? mu[i, a] / scale[a] : mu[i, a];
                this.mixNormalised[i, a] = Mathf.Abs(v) < 1e-3f ? 0f : v;
            }

        // Physical torque per unit normalised torque, for the stability warning (phase 5) and logs.
        Debug.Log($"[Flight] Allocation scale: roll/pitch {rp:F4}, yaw {yaw:F4} (u per N*m); " +
                  $"full normalised torque = {1f / rp:F4} N*m roll/pitch, {1f / yaw:F4} N*m yaw");
    }

    private readonly float[] allocIdeal = new float[4];
    private readonly float[] allocThrustZ = new float[4];
    private readonly float[] allocAxis = new float[4];

    /// <summary>
    /// PX4 ControlAllocationSequentialDesaturation with MC_AIRMODE 0 (mixAirmodeDisabled + mixYaw):
    /// roll/pitch/thrust first, thrust may only rise to unsaturate, then roll and pitch are
    /// desaturated, yaw is added last with 15% extra headroom and gives way first.
    /// Collective thrust uses this frame's lever-rule column in newtons. Sets the saturation flags
    /// the rate integrators use.
    /// </summary>
    private void Allocate(Vector3 torqueNormalised, float thrustN, float[] u)
    {
        float meanThrust = 0f;
        for (int i = 0; i < 4; i++) meanThrust += Mathf.Abs(this.allocation[i, 0]);
        meanThrust /= 4f;

        for (int i = 0; i < 4; i++)
        {
            float thrustPart = this.allocation[i, 0] * thrustN / this.motorThrustMax;
            u[i] = thrustPart + this.mixNormalised[i, 0] * torqueNormalised.x + this.mixNormalised[i, 2] * torqueNormalised.z;
            allocIdeal[i] = u[i] + this.mixNormalised[i, 1] * torqueNormalised.y;
            allocThrustZ[i] = this.allocation[i, 0] / meanThrust;
        }

        Desaturate(u, allocThrustZ, 1f, true);
        for (int i = 0; i < 4; i++) allocAxis[i] = this.mixNormalised[i, 2];
        Desaturate(u, allocAxis, 1f, false); // roll
        for (int i = 0; i < 4; i++) allocAxis[i] = this.mixNormalised[i, 0];
        Desaturate(u, allocAxis, 1f, false); // pitch

        // mixYaw
        for (int i = 0; i < 4; i++)
        {
            allocAxis[i] = this.mixNormalised[i, 1];
            u[i] += allocAxis[i] * torqueNormalised.y;
        }
        Desaturate(u, allocAxis, 1f + MinimumYawMargin, false);
        Desaturate(u, allocThrustZ, 1f, true);

        for (int i = 0; i < 4; i++) u[i] = Mathf.Clamp01(u[i]);

        // Saturation flags (PX4 ControlAllocator: unallocated torque per axis).
        for (int a = 0; a < 3; a++)
        {
            float unallocated = 0f;
            for (int i = 0; i < 4; i++)
                unallocated += this.effectiveness[a + 1, i] * (allocIdeal[i] - u[i]) * this.motorThrustMax;
            this.saturationPositive[a] = unallocated > 1e-5f ? 1f : 0f;
            this.saturationNegative[a] = unallocated < -1e-5f ? 1f : 0f;
        }
    }

    /// <summary>PX4 ControlAllocationSequentialDesaturation::desaturateActuators (actuator range 0..max).</summary>
    private static void Desaturate(float[] u, float[] direction, float max, bool increaseOnly)
    {
        float gain = DesaturationGain(direction, u, max);
        if (increaseOnly && gain < 0f) return;
        for (int i = 0; i < 4; i++) u[i] += gain * direction[i];
        gain = 0.5f * DesaturationGain(direction, u, max);
        for (int i = 0; i < 4; i++) u[i] += gain * direction[i];
    }

    private static float DesaturationGain(float[] direction, float[] u, float max)
    {
        float kMin = 0f, kMax = 0f;
        for (int i = 0; i < 4; i++)
        {
            if (Mathf.Abs(direction[i]) < 0.2f) continue;
            if (u[i] < 0f)
            {
                float k = -u[i] / direction[i];
                if (k < kMin) kMin = k;
                if (k > kMax) kMax = k;
            }
            if (u[i] > max)
            {
                float k = (max - u[i]) / direction[i];
                if (k < kMin) kMin = k;
                if (k > kMax) kMax = k;
            }
        }
        return kMin + kMax;
    }

    #endregion
}

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
///   - PX4 v1.17.0 attitude and rate loops, allocation with MC_AIRMODE 0 desaturation (phase 1)
///   - PX4 velocity/position loops (phase 2): with no horizontal input the drone brakes by
///     tilting (velocity setpoint ramped to zero at MPC_ACC_HOR) and then holds position;
///     altitude hold through the PX4 vertical loop. Nothing writes the velocity in flight.
///   - Arm/disarm (phase 4): physics takeoff at min(MPC_TKO_SPEED, MPC_Z_VEL_MAX_UP) to
///     launchHeight; landing at min(MPC_LAND_SPEED, MPC_Z_VEL_MAX_DN) with a land detector;
///     motors spin up and down through the lag model
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

    // --------- PX4 velocity and position loops (spec phase 2) ---------
    // PX4 v1.17.0 src/modules/mc_pos_control/PositionControl/PositionControl.cpp. The MPC_* values
    // come from the real drone's params file (they equal the PX4 defaults). World axes: y is up.

    private float xyP, xyVelP, xyVelI, xyVelD;   // MPC_XY_P, MPC_XY_VEL_P/I/D_ACC
    private float zP, zVelP, zVelI, zVelD;       // MPC_Z_P, MPC_Z_VEL_P/I/D_ACC
    private float xyVelMax, zVelMaxUp, zVelMaxDown, accHor, tiltMax, thrMin, thrMax;
    private float velManual, manualYawRate;      // MPC_VEL_MANUAL (m/s), MPC_MAN_Y_MAX (deg/s)
    private float accUpMax, accDownMax;          // MPC_ACC_UP_MAX, MPC_ACC_DOWN_MAX (m/s^2)
    private float climbSetpoint;                 // keyboard climb-rate setpoint, ramped (m/s)
    private Vector3 velIntegral;                 // m/s^2
    private Vector3 previousVelocity;
    private bool haveVelocity;

    /// <summary>Hold: velocity setpoint ramped to zero at MPC_ACC_HOR, then a position lock.</summary>
    private bool holding;
    private Vector2 brakeVelocitySetpoint;
    private Vector2 setpointAcceleration;        // the ramp's acceleration, jerk-limited (m/s^2)
    private float jerkMax;                       // MPC_JERK_MAX (m/s^3)
    private float accHorMax;                     // MPC_ACC_HOR_MAX (m/s^2), braking
    private bool positionLocked;
    private Vector2 lockedPosition;

    /// <summary>Heading (deg) held while there is no yaw input; NaN while yawing.</summary>
    private float yawLock = float.NaN;

    /// <summary>
    /// Keyboard speed (m/s) at 1.0 x MaxSpeed. Replaces the real drone's MPC_VEL_MANUAL (1 m/s),
    /// which felt too slow to fly by hand; 30 m/s matches the old sim's top speed (owner 2026-10-20).
    /// </summary>
    public const float KeyboardMaxSpeed = 30f;

    /// <summary>
    /// Keyboard climb/descent rate (m/s) at 1.0 x MaxSpeed. Replaces the real drone's
    /// MPC_Z_VEL_MAX_UP/DN (0.6 m/s), which felt too slow to fly by hand (owner 2026-10-20).
    /// </summary>
    public const float KeyboardMaxClimbRate = 10f;

    /// <summary>Stick values below this count as released (as before).</summary>
    private const float StickDeadband = 0.05f;

    /// <summary>The position lock engages once the brake setpoint is zero and the speed is below this (m/s).</summary>
    private const float PositionLockSpeed = 0.1f;

    // --------- User Input Limits ---------

    /// <summary>
    /// send_pcmd tilt (degrees) at 1.0 x MaxSpeed (flight-sim.json pcmdMaxTiltDeg).
    /// </summary>
    private float maxTiltAngle = 35f;

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

    // --------- Takeoff and landing (spec phase 4, section 4.5) ---------

    /// <summary>Height above the ground (m) the takeoff climbs to (flight-sim.json launchHeight).</summary>
    private float launchAltitude = 1.0f;

    /// <summary>Takeoff climb rate min(MPC_TKO_SPEED, MPC_Z_VEL_MAX_UP) and landing descent rate
    /// min(MPC_LAND_SPEED, MPC_Z_VEL_MAX_DN), m/s.</summary>
    private float takeoffSpeed = 0.6f, landSpeed = 0.6f;

    /// <summary>PX4 smooth takeoff (TakeoffHandling, MPC_TKO_RAMP_T): the climb-rate limit ramps
    /// from 0 to takeoffSpeed over this time (s) after arming.</summary>
    private float takeoffRampTime = 3f;
    private float launchTime;

    /// <summary>Acceleration of the takeoff/landing climb setpoint, jerk-limited at MPC_JERK_MAX (m/s^2).</summary>
    private float climbAcceleration;

    /// <summary>The takeoff ends once within this distance of its height (m) and this vertical speed (m/s).</summary>
    private const float LaunchDoneDistance = 0.05f, LaunchDoneSpeed = 0.1f;

    /// <summary>Land detector: ground contact (or thrust at MPC_THR_MIN) with |vertical speed| below
    /// LandedSpeed for LandedTime seconds disarms.</summary>
    private const float LandedSpeed = 0.2f, LandedTime = 0.5f;
    private float landedTimer;

    /// <summary>Set by OnCollisionStay when a collider below the drone touches it; read once per step.</summary>
    private bool groundContact;

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
    /// Who is flying (spec section 4.4). Pcmd: Python send_pcmd, tilt meaning. PosCtl:
    /// keyboard/controller, PX4 position mode (sticks command velocity, release brakes and holds).
    /// </summary>
    public enum ControlMode { Pcmd, PosCtl }

    /// <summary>
    /// The input path, set by LevelManager when the simulation mode changes.
    /// </summary>
    public ControlMode Mode { get; set; } = ControlMode.Pcmd;

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
    /// Arms the drone and begins the physics takeoff: the motors spin up through the lag model
    /// and the PX4 vertical loop climbs at the takeoff speed to launchAltitude above the
    /// current position, then holds altitude and position.
    /// </summary>
    public void Arm()
    {
        if (IsArmed) return;
        IsArmed = true;
        IsLaunching = true;
        IsLanding = false;
        groundY = this.transform.position.y;
        TargetAltitude = this.transform.position.y + launchAltitude;
        ResetPositionController();
        ResetRateController();

        Debug.Log($"[Flight] Armed! Launching to altitude {TargetAltitude:F2}m");
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

        // Clean start for relaunch (a respawn is a teleport)
        this.rBody.linearVelocity = Vector3.zero;
        this.rBody.angularVelocity = Vector3.zero;
        ResetPositionController();

        Debug.Log($"[Flight] Relaunching to altitude {TargetAltitude:F1}m");
    }

    /// <summary>
    /// Begins the auto-land: the drone descends at the landing speed, holding position, onto
    /// whatever surface is below (ground, rooftops, etc.), and disarms once the land detector
    /// sees it resting there.
    /// </summary>
    public void BeginLanding()
    {
        if (!IsArmed || IsLanding) return;
        IsLanding = true;
        IsLaunching = false;
        this.landedTimer = 0f;

        // Clear all user inputs
        this.ForwardSpeed = 0;
        this.StrafeSpeed = 0;
        this.VerticalSpeed = 0;
        this.YawSpeed = 0;

        Debug.Log($"[Flight] Landing initiated - descending at {this.landSpeed:F2} m/s");
    }

    /// <summary>
    /// Disarms the drone (called when the land detector triggers). The motors spin down
    /// through the lag model; nothing writes the velocity.
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
        TargetAltitude = this.transform.position.y;
        Debug.Log("[Flight] Landed and disarmed");
    }

    /// <summary>Ground contact for the land detector: a contact whose normal points up.</summary>
    private void OnCollisionStay(Collision collision)
    {
        for (int i = 0; i < collision.contactCount; i++)
            if (collision.GetContact(i).normal.y > 0.7f)
            {
                this.groundContact = true;
                return;
            }
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
        // Colliders first: the mass properties below are explicit and must be set after any
        // collider change, which would otherwise make Unity recompute them.
        if (this.customMass > 0f)
            AddCustomFrameCollider();

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
        // 0. ARM CHECK â€” if not armed, the motors spin down (lag) and nothing else runs
        // ================================================================
        bool touchingGround = this.groundContact;
        this.groundContact = false;
        if (!IsArmed)
        {
            SpinDownStep();
            return;
        }

        // ================================================================
        // 0a. LAND DETECTOR (PX4-style, simplified): resting on a surface, or the collective
        //     at MPC_THR_MIN, with low vertical speed for LandedTime -> disarm.
        // ================================================================
        if (IsLanding)
        {
            float collective = 0f;
            for (int i = 0; i < 4; i++) collective += this.MotorCommands[i];
            bool resting = touchingGround || collective <= 4f * this.thrMin + 1e-3f;
            if (resting && Mathf.Abs(this.rBody.linearVelocity.y) < LandedSpeed)
                this.landedTimer += Time.fixedDeltaTime;
            else
                this.landedTimer = 0f;
            if (this.landedTimer >= LandedTime)
            {
                FinishDisarm();
                SpinDownStep();
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

        float dt = Time.fixedDeltaTime;
        Vector3 angVelBody = this.transform.InverseTransformDirection(this.rBody.angularVelocity);
        float p = angVelBody.x; // pitch rate
        float r = angVelBody.z; // roll rate

        Vector3 velocity = this.rBody.linearVelocity;
        Vector3 acceleration = this.haveVelocity ? (velocity - this.previousVelocity) / dt : Vector3.zero;
        this.previousVelocity = velocity;
        this.haveVelocity = true;

        // ================================================================
        // 2. TAKEOFF: physics climb through the vertical loop (section 4 below). Horizontal input
        //    ends the takeoff early; it completes once at its height and nearly still.
        // ================================================================
        if (IsLaunching)
        {
            if (Mathf.Abs(this.ForwardSpeed) >= StickDeadband || Mathf.Abs(this.StrafeSpeed) >= StickDeadband)
            {
                IsLaunching = false;
                TargetAltitude = this.transform.position.y;
                Debug.Log("[Flight] User took over during launch - exiting launch stabilization");
            }
            else if (Mathf.Abs(TargetAltitude - this.transform.position.y) < LaunchDoneDistance && Mathf.Abs(velocity.y) < LaunchDoneSpeed)
            {
                IsLaunching = false;
                // Hand over to altitude hold at TargetAltitude (a leftover keyboard climb setpoint
                // would otherwise re-target the current height).
                this.climbSetpoint = 0f;
                this.climbAcceleration = 0f;
                Debug.Log($"[Flight] Launch complete - entering altitude hold at {this.transform.position.y - this.groundY:F2} m above ground");
            }
        }

        float scale = this.MaxSpeed;
        float gravity = Physics.gravity.magnitude;
        float mass = this.rBody.mass;
        float hoverThrust = mass * gravity;
        float heading = this.transform.eulerAngles.y;
        bool horizontalInput = Mathf.Abs(this.ForwardSpeed) >= StickDeadband || Mathf.Abs(this.StrafeSpeed) >= StickDeadband;
        // Takeoff and landing own the vertical axis; throttle input is ignored meanwhile.
        bool autoVertical = IsLaunching || IsLanding;
        bool verticalInput = !autoVertical && Mathf.Abs(this.VerticalSpeed) >= StickDeadband;
        bool posCtl = this.Mode == ControlMode.PosCtl;

        // ================================================================
        // 3. YAW: yaw 1.0 = pcmdMaxYawRateDegS (send_pcmd) or MPC_MAN_Y_MAX (keyboard) about
        //    world up, no banking. The heading is held when the yaw input is released.
        // ================================================================
        float desiredYawRate = this.YawSpeed * (posCtl ? this.manualYawRate : maxYawRate) * Mathf.Deg2Rad;
        if (this.YawSpeed != 0f)
            this.yawLock = float.NaN;
        else if (float.IsNaN(this.yawLock))
            this.yawLock = heading;
        float yawSetpoint = float.IsNaN(this.yawLock) ? heading : this.yawLock;

        // ================================================================
        // 4. VERTICAL: send_pcmd throttle = thrust offset (as before); keyboard triggers = climb
        //    rate up to MPC_Z_VEL_MAX_UP/DN x MaxSpeed; released = PX4 altitude hold
        //    (PositionControl _positionControl/_velocityControl, z axis), limited by MPC_Z_VEL_MAX_UP/DN.
        // ================================================================
        float accelerationUp = 0f;
        bool directThrust = verticalInput && !posCtl;
        // Keyboard climb release: the climb setpoint ramps back to zero before the altitude locks.
        bool climbStopping = posCtl && !verticalInput && !autoVertical && this.climbSetpoint != 0f;
        if (verticalInput || climbStopping)
            TargetAltitude = this.transform.position.y;
        if (directThrust)
        {
            this.velIntegral.y = 0f;
        }
        else
        {
            float vzSetpoint, accelerationFeedForward = 0f;
            if (verticalInput || climbStopping || autoVertical)
            {
                // Keyboard climb, takeoff and landing: the setpoint ramps at MPC_ACC_UP_MAX /
                // MPC_ACC_DOWN_MAX, and that acceleration is fed forward (PX4 trajectory setpoint),
                // so the loop does not overshoot. Takeoff climbs at takeoffSpeed toward TargetAltitude;
                // landing descends at landSpeed until the land detector disarms.
                float target = verticalInput ? this.VerticalSpeed * KeyboardMaxClimbRate * scale : 0f;
                if (IsLaunching)
                {
                    this.launchTime += dt;
                    float climbLimit = this.takeoffSpeed * Mathf.Clamp01(this.launchTime / Mathf.Max(this.takeoffRampTime, dt));
                    target = Mathf.Clamp(this.zP * (TargetAltitude - this.transform.position.y), -this.zVelMaxDown, climbLimit);
                }
                else if (IsLanding)
                    target = -this.landSpeed;
                float rampAcceleration = target > this.climbSetpoint ? this.accUpMax : this.accDownMax;
                float ramped;
                if (autoVertical)
                {
                    // Jerk-limited, as the horizontal ramp, so the climb rate does not overshoot
                    // when the setpoint stops changing.
                    float error = target - this.climbSetpoint;
                    float wanted = Mathf.Sign(error) * Mathf.Min(rampAcceleration, Mathf.Sqrt(2f * this.jerkMax * Mathf.Abs(error)));
                    this.climbAcceleration = Mathf.MoveTowards(this.climbAcceleration, wanted, this.jerkMax * dt);
                    float step = this.climbAcceleration * dt;
                    if (Mathf.Abs(step) >= Mathf.Abs(error) && step * error > 0f)
                    {
                        step = error;
                        this.climbAcceleration = 0f;
                    }
                    ramped = this.climbSetpoint + step;
                }
                else
                {
                    this.climbAcceleration = 0f;
                    ramped = Mathf.MoveTowards(this.climbSetpoint, target, rampAcceleration * dt);
                }
                accelerationFeedForward = (ramped - this.climbSetpoint) / dt;
                this.climbSetpoint = vzSetpoint = ramped;
            }
            else
            {
                vzSetpoint = Mathf.Clamp(this.zP * (TargetAltitude - this.transform.position.y), -this.zVelMaxDown, this.zVelMaxUp);
                this.climbSetpoint = posCtl ? 0f : velocity.y;
            }
            float vzError = vzSetpoint - velocity.y;
            accelerationUp = accelerationFeedForward + this.zVelP * vzError + this.velIntegral.y - this.zVelD * acceleration.y;
            this.velIntegral.y = Mathf.Clamp(this.velIntegral.y + this.zVelI * vzError * dt, -gravity, gravity);
        }

        // ================================================================
        // 5. HORIZONTAL -> ATTITUDE SETPOINT
        //    Tilt input (send_pcmd meaning): pitch/roll = value x pcmdMaxTiltDeg x MaxSpeed.
        //    Keyboard (PX4 POSCTL): sticks = velocity target up to MPC_VEL_MANUAL x MaxSpeed in the
        //    heading frame. In both modes the velocity setpoint ramps toward its target at
        //    MPC_ACC_HOR (released: zero, then the position locks); the PX4 velocity loop turns
        //    that into an acceleration, so the drone stops only by tilting.
        // ================================================================
        Quaternion attitudeSetpoint;
        if (horizontalInput && !posCtl)
        {
            this.holding = false;
            this.positionLocked = false;
            this.velIntegral.x = this.velIntegral.z = 0f;
            float desiredPitch = this.ForwardSpeed * maxTiltAngle * scale;
            float desiredRoll = -this.StrafeSpeed * maxTiltAngle * scale;
            attitudeSetpoint = Quaternion.Euler(desiredPitch, yawSetpoint, desiredRoll);
        }
        else
        {
            Vector2 horizontalVelocity = new Vector2(velocity.x, velocity.z);
            Vector2 position = new Vector2(this.transform.position.x, this.transform.position.z);
            if (!this.holding)
            {
                this.holding = true;
                this.brakeVelocitySetpoint = horizontalVelocity;
                this.setpointAcceleration = Vector2.ClampMagnitude(new Vector2(acceleration.x, acceleration.z), this.accHor);
            }
            Vector2 velocityTarget = Vector2.zero;
            if (horizontalInput)
            {
                Vector3 stick = Quaternion.Euler(0f, yawSetpoint, 0f) * new Vector3(this.StrafeSpeed, 0f, this.ForwardSpeed);
                velocityTarget = Vector2.ClampMagnitude(new Vector2(stick.x, stick.z), 1f) * this.velManual * scale;
                this.positionLocked = false;
            }
            // Jerk-limited ramp (PX4 trajectory, MPC_ACC_HOR / MPC_JERK_MAX): the acceleration builds
            // up and dies away at MPC_JERK_MAX, so the drone tilts in and out smoothly instead of
            // stepping. The wanted acceleration shrinks near the target so the setpoint lands on it.
            Vector2 setpointError = velocityTarget - this.brakeVelocitySetpoint;
            // MPC_ACC_HOR_MAX for speeding up, turning and braking (owner 2026-10-20: MPC_ACC_HOR felt sluggish).
            float rampAcceleration = this.accHorMax;
            Vector2 wantedAcceleration = setpointError.normalized * Mathf.Min(rampAcceleration,Mathf.Sqrt(2f * this.jerkMax * setpointError.magnitude));
            this.setpointAcceleration = Vector2.MoveTowards(this.setpointAcceleration, wantedAcceleration, this.jerkMax * dt);
            Vector2 step = this.setpointAcceleration * dt;
            if (step.magnitude >= setpointError.magnitude && Vector2.Dot(step, setpointError) > 0f)
            {
                step = setpointError;
                this.setpointAcceleration = Vector2.zero;
            }
            this.brakeVelocitySetpoint += step;
            // The ramp's acceleration is fed forward, as PX4 does with the trajectory setpoint.
            Vector2 accelerationFeedForward = this.positionLocked ? Vector2.zero : step / dt;
            if (!horizontalInput && !this.positionLocked && this.brakeVelocitySetpoint == Vector2.zero && horizontalVelocity.magnitude < PositionLockSpeed)
            {
                this.positionLocked = true;
                this.lockedPosition = position;
            }

            Vector2 velocitySetpoint = this.positionLocked
                ? Vector2.ClampMagnitude(this.xyP * (this.lockedPosition - position), this.xyVelMax)
                : this.brakeVelocitySetpoint;
            Vector2 velocityError = velocitySetpoint - horizontalVelocity;
            Vector2 integral = new Vector2(this.velIntegral.x, this.velIntegral.z);
            Vector2 accelerationXY = accelerationFeedForward + this.xyVelP * velocityError + integral - this.xyVelD * new Vector2(acceleration.x, acceleration.z);
            integral = Vector2.ClampMagnitude(integral + this.xyVelI * velocityError * dt, gravity * Mathf.Tan(this.tiltMax));
            this.velIntegral.x = integral.x;
            this.velIntegral.z = integral.y;

            // ControlMath::limitTilt + bodyzToAttitude: thrust axis from the horizontal acceleration.
            Vector3 bodyUp = new Vector3(accelerationXY.x, gravity, accelerationXY.y).normalized;
            bodyUp = Vector3.RotateTowards(Vector3.up, bodyUp, this.tiltMax, 0f);
            attitudeSetpoint = AttitudeFromThrustAxis(bodyUp, yawSetpoint);
        }

        // Collective thrust (N).
        float desiredTotalThrust;
        if (directThrust)
        {
            // As before: hover + offset, with a capped tilt compensation.
            desiredTotalThrust = hoverThrust + this.VerticalSpeed * scale * maxVerticalThrustFraction * hoverThrust;
            float tiltFactor = Vector3.Dot(this.transform.up, Vector3.up);
            if (tiltFactor > 0.01f)
                desiredTotalThrust *= Mathf.Min(1f / tiltFactor, maxTiltCompensation);
        }
        else
        {
            // PX4 _accelerationControl: hover-normalised, divided by the setpoint tilt, MPC_THR_MIN/MAX.
            float cosTilt = Mathf.Max(Vector3.Dot(attitudeSetpoint * Vector3.up, Vector3.up), 0.1f);
            desiredTotalThrust = mass * (gravity + accelerationUp) / cosTilt;
        }
        float allMotors = 4f * this.motorThrustMax;
        desiredTotalThrust = Mathf.Clamp(desiredTotalThrust, this.thrMin * allMotors, this.thrMax * allMotors);

        // ================================================================
        // 6. PX4 ATTITUDE LOOP (once per FixedUpdate) -> RATE SETPOINT
        //    PX4 v1.17.0 src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp:
        //    reduced attitude (tilt first, yaw weighted by MC_YAW_WEIGHT), rate_sp = P * error
        //    plus the yaw-rate feed-forward, limited by MC_*RATE_MAX.
        // ================================================================

        Quaternion attitude = this.transform.rotation;
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

        // Aerodynamic drag: quadratic horizontal, linear vertical (frame-scaled drag is phase 5).
        // No brake forces: the drone stops only by tilting.
        float hSpeed = Mathf.Sqrt(velocity.x * velocity.x + velocity.z * velocity.z);
        this.rBody.AddForce(new Vector3(-dragXZ * hSpeed * velocity.x, -dragY * velocity.y, -dragXZ * hSpeed * velocity.z));

        // Aerodynamic angular drag
        this.rBody.AddRelativeTorque(-angularDragCoeff * angVelBody);
    }

    /// <summary>
    /// Disarmed: motor commands are 0 and each motor's thrust decays through the lag model,
    /// still pushing at its own position until it has spun down.
    /// </summary>
    private void SpinDownStep()
    {
        float dtSub = Time.fixedDeltaTime / this.subSteps;
        for (int i = 0; i < 4; i++)
        {
            this.MotorCommands[i] = 0f;
            if (this.MotorThrusts[i] <= 0f) { this.MotorSpeeds[i] = 0f; continue; }

            float sum = 0f, t = this.MotorThrusts[i];
            for (int s = 0; s < this.subSteps; s++)
            {
                t = MotorLagStep(t, 0f, dtSub, this.motorTauUp, this.motorTauDown);
                sum += t;
            }
            if (t < 1e-4f * this.motorThrustMax) t = 0f;
            this.MotorThrusts[i] = t;
            this.MotorSpeeds[i] = Mathf.Clamp(Mathf.Sqrt(t / kf), motorSpeedMin, motorSpeedMax);
            this.rBody.AddForceAtPosition(this.transform.up * (sum / this.subSteps), this.transform.TransformPoint(this.MotorPositions[i]));
        }
    }

    /// <summary>
    /// Custom frames touch the ground through a convex MeshCollider on the imported mesh (Unity
    /// simplifies the hull to at most 255 faces); the prefab's box colliders, sized for the stock
    /// body, are disabled. Falls back to a box around the mesh bounds if the hull cannot be made.
    /// </summary>
    private void AddCustomFrameCollider()
    {
        CustomFrameLoader loader = this.GetComponent<CustomFrameLoader>();
        MeshFilter visual = null;
        foreach (MeshFilter mf in this.GetComponentsInChildren<MeshFilter>(false))
            if (mf.sharedMesh != null && loader != null && mf.sharedMesh.bounds == loader.CustomFrameBounds)
                visual = mf;
        if (visual == null)
        {
            Debug.LogWarning("[Flight] Custom frame mesh not found; keeping the prefab's colliders.");
            return;
        }

        foreach (Collider c in this.GetComponentsInChildren<Collider>(true))
            if (!c.isTrigger) c.enabled = false;

        MeshCollider hull = visual.gameObject.AddComponent<MeshCollider>();
        hull.convex = true;
        hull.sharedMesh = visual.sharedMesh;
        if (hull.sharedMesh == null || hull.bounds.size.sqrMagnitude <= 0f)
        {
            Destroy(hull);
            BoxCollider box = visual.gameObject.AddComponent<BoxCollider>();
            box.center = visual.sharedMesh.bounds.center;
            box.size = visual.sharedMesh.bounds.size;
            Debug.LogWarning("[Flight] Convex hull failed for the custom frame; using a box around its bounds.");
            return;
        }
        Debug.Log($"[Flight] Custom frame collider: convex MeshCollider on '{visual.name}', bounds {hull.bounds.size:F4} m");
    }

    /// <summary>
    /// PX4 ControlMath::bodyzToAttitude: the attitude whose up axis is <paramref name="bodyUp"/>
    /// and whose heading is <paramref name="yawDeg"/> (degrees about world up).
    /// </summary>
    private static Quaternion AttitudeFromThrustAxis(Vector3 bodyUp, float yawDeg)
    {
        Vector3 headingForward = Quaternion.Euler(0f, yawDeg, 0f) * Vector3.forward;
        Vector3 right = Vector3.Cross(bodyUp, headingForward);
        if (right.sqrMagnitude < 1e-8f) right = Quaternion.Euler(0f, yawDeg, 0f) * Vector3.right;
        Vector3 forward = Vector3.Cross(right.normalized, bodyUp);
        return Quaternion.LookRotation(forward, bodyUp);
    }

    /// <summary>Clears the hold, heading lock and velocity-loop state (arm, launch end, relaunch).</summary>
    private void ResetPositionController()
    {
        this.velIntegral = Vector3.zero;
        this.haveVelocity = false;
        this.holding = false;
        this.positionLocked = false;
        this.yawLock = float.NaN;
        this.climbSetpoint = 0f;
        this.climbAcceleration = 0f;
        this.landedTimer = 0f;
        this.launchTime = 0f;
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

        this.maxTiltAngle = FlightConfig.Sim("pcmdMaxTiltDeg");
        this.xyP = FlightConfig.Px4("MPC_XY_P");
        this.xyVelP = FlightConfig.Px4("MPC_XY_VEL_P_ACC");
        this.xyVelI = FlightConfig.Px4("MPC_XY_VEL_I_ACC");
        this.xyVelD = FlightConfig.Px4("MPC_XY_VEL_D_ACC");
        this.zP = FlightConfig.Px4("MPC_Z_P");
        this.zVelP = FlightConfig.Px4("MPC_Z_VEL_P_ACC");
        this.zVelI = FlightConfig.Px4("MPC_Z_VEL_I_ACC");
        this.zVelD = FlightConfig.Px4("MPC_Z_VEL_D_ACC");
        this.xyVelMax = FlightConfig.Px4("MPC_XY_VEL_MAX");
        this.zVelMaxUp = FlightConfig.Px4("MPC_Z_VEL_MAX_UP");
        this.zVelMaxDown = FlightConfig.Px4("MPC_Z_VEL_MAX_DN");
        this.accHor = FlightConfig.Px4("MPC_ACC_HOR");
        this.tiltMax = FlightConfig.Px4("MPC_TILTMAX_AIR") * Mathf.Deg2Rad;
        this.thrMin = FlightConfig.Px4("MPC_THR_MIN");
        this.thrMax = FlightConfig.Px4("MPC_THR_MAX");
        this.velManual = KeyboardMaxSpeed;
        this.jerkMax = FlightConfig.Px4("MPC_JERK_MAX");
        this.accHorMax = FlightConfig.Px4("MPC_ACC_HOR_MAX");
        this.manualYawRate = FlightConfig.Px4("MPC_MAN_Y_MAX");
        this.accUpMax = FlightConfig.Px4("MPC_ACC_UP_MAX");
        this.accDownMax = FlightConfig.Px4("MPC_ACC_DOWN_MAX");
        this.launchAltitude = FlightConfig.Sim("launchHeight");
        this.takeoffSpeed = Mathf.Min(FlightConfig.Px4("MPC_TKO_SPEED"), this.zVelMaxUp);
        this.landSpeed = Mathf.Min(FlightConfig.Px4("MPC_LAND_SPEED"), this.zVelMaxDown);
        this.takeoffRampTime = FlightConfig.Px4("MPC_TKO_RAMP_T");
        ResetPositionController();

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

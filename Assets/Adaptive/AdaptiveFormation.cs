using System.Runtime.InteropServices;
using UnityEngine;

[RequireComponent(typeof(FreeSpaceScanner))]
[RequireComponent(typeof(AdaptiveEllipseEstimator))]
[RequireComponent(typeof(JointFollowerCBFSolver))]
public class AdaptiveFormation : MonoBehaviour
{
    [Header("Rigidbodies")]
    public Rigidbody leader;
    public Rigidbody followerA;
    public Rigidbody followerB;

    [Header("Shape-Adaptive Safety Components")]
    public FreeSpaceScanner freeSpaceScanner;
    public AdaptiveEllipseEstimator ellipseEstimator;
    public JointFollowerCBFSolver cbfSolver;
    public LayerMask obstacleMask;

    [Header("Haptic Input")]
    public float hapticHz = 200f;
    public float inputScale = 3.0f;
    public float inputDeadzone = 0.003f;
    public float inputSmoothTau = 0.03f;
    public bool invertX = false;
    public bool invertY = false;
    public bool invertZ = false;

    [Header("Keyboard Fallback")]
    public bool useKeyboardFallback = true;
    public float keyboardInputScale = 1.0f;

    [Header("Leader Motion")]
    public float leaderSpeed = 6f;
    public float leaderMaxSpeed = 10f;
    public float leaderVelLerp = 0.4f;

    [Header("Formation Geometry")]
    public float formationSpacing = 3f;
    public bool followersInFrontOfLeader = true;
    public bool useMotionAlignedFormation = false;
    public float formationDirSmoothTau = 0.35f;
    public float minFormationDirUpdateSpeed = 0.8f;
    public bool preventFormationDirectionFlip = true;
    public float maxDirectionChangeAngle = 75f;

    [Header("Adaptive Impedance Formation")]
    public bool enableAdaptiveImpedance = true;
    public float kClear = 35f;
    public float kRisk = 16f;
    public float dClear = 14f;
    public float dRisk = 28f;
    public float impedanceForceMax = 160f;
    public float leaderVelocityFeedforward = 0.35f;
    public float followerNominalMaxSpeed = 5.0f;

    [Header("Optional Inter-Follower Shape Impedance")]
    public bool useInterFollowerImpedance = true;
    public float interFollowerKScale = 0.25f;
    public float interFollowerDScale = 0.50f;
    public float interFollowerForceMax = 50f;

    [Header("Follower Height Tracking")]
    public float heightKp = 30f;
    public float heightKd = 18f;
    public float heightForceMax = 60f;
    public float followerVerticalSpeedMax = 3.0f;

    public enum ComparisonCondition
    {
        M1_FixedCircle_FixedImpedance,
        M2_AdaptiveEllipse_FixedImpedance,
        M3_FixedCircle_AdaptiveImpedance,
        M4_AdaptiveEllipse_AdaptiveImpedance
    }

    [Header("Four-Way Ablation: One Shared CBF Solver")]
    public ComparisonCondition comparisonCondition = ComparisonCondition.M4_AdaptiveEllipse_AdaptiveImpedance;
    [Tooltip("Radius of the fixed circular CBF region in M1/M3, in metres.")]
    public float fixedCircleRadius = 0.50f;
    [Tooltip("Enable this for reproducible M1-M4 experiments; locks CBF and impedance flags to the selected condition.")]
    public bool enforceAblationConfiguration = true;

    [Header("Safety Modes")]
    public bool enableShapeAdaptiveCBF = true;

    [Tooltip("Whether the leader command is actively filtered by the same obstacle CBF solver.")]
    public bool leaderUsesCBF = true;

    [Header("Leader Active CBF Policy")]
    [Tooltip("For M1-M4: enable an active leader obstacle CBF in every condition. Keeps the ablation comparison fair.")]
    public bool leaderActiveCBFForAllConditions = true;

    public enum LeaderInfeasibleAction { BestEffort, StopPlanar }
    [Tooltip("If the approximate leader CBF reports an infeasible command, StopPlanar commands zero planar speed. This is not a formal safety guarantee.")]
    public LeaderInfeasibleAction leaderInfeasibleAction = LeaderInfeasibleAction.StopPlanar;

    public bool followersUseCBF = true;

    [Header("Follower Safety Interlock")]
    [Tooltip("When the joint follower solver cannot resolve its constraints even after speed-bounded re-solving, stop the leader's planar command too. This is a conservative fallback, NOT a safety proof.")]
    public bool pauseLeaderOnFollowerEmergency = false;
    [Header("Directional Haptic Virtual Fixture")]
    [Tooltip("Render force only in the direction opposite the OPERATOR'S requested movement into obstacles. Moving away/along a wall should be almost free.")]
    public bool useDirectionSelectiveVirtualFixture = true;
    [Tooltip("Minimum requested planar speed for direction gating. When neutral, fixture releases instead of pulling the operator.")]
    public float fixtureMinimumRequestedSpeed = 0.10f;
    [Range(0.0f, 1.0f)] public float fixtureCBFForceBlend = 0.30f;
    [Tooltip("If enabled, damping resists only handle motion TOWARD the currently threatening obstacle, not movement away or parallel.")]
    public bool useOneWayDirectionalDamping = true;
    [Tooltip("OFF by default: legacy group-wide freeze is a major cause of terminal deadlocks. Set ON ONLY for diagnostic comparisons.")]
    public bool enableLegacyWholeFormationEmergencyFreeze = false;
    [Tooltip("When followers cannot resolve a hard CBF, prevent the leader from driving directly into an immobilized follower. Does not globally freeze all three UAVs.")]
    public bool leaderContactGuardDuringFollowerFailure = true;
    public float leaderContactGuardExtraMargin = 0.12f;
    [Header("Recovery Runtime Diagnostics")]
    public bool dbgFollowerRecoveryAttempted;
    public bool dbgFollowerRecoverySucceeded;
    public float dbgFollowerRecoveryResidual;
    public float dbgFollowerHardPairMinimum;
    public float dbgFollowerSoftCorrectionMagnitude;
    [Tooltip("When a follower safety stop fires, also command zero planar velocity to the two followers.")]
    public bool pauseWholeFormationOnEmergency = false;
    [Header("Follower Solver Runtime Diagnostics")]
    public bool dbgFollowerAccelerationRelaxed;
    public bool dbgFollowerEmergencyStopped;
    public float dbgFollowerResidualWithAcceleration;

    [Header("Physics")]
    public bool disableGravity = true;
    public bool freezeAgentRotation = true;

    [Header("Leader CBF-Mediated Haptic Feedback")]
    public HapticForceSender hapticSender;
    public bool enableCBFMediatedHaptic = true;
    public float cbfCorrectionToForceGain = 0.35f;
    public float cbfCorrectionDeadzone = 0.02f;

    [Header("Leader-Only Proximity Haptic Warning")]
    [Tooltip("Advance warning based only on the leader-to-obstacle geometry. No follower or formation force is rendered.")]
    public bool enableLeaderProximityHaptic = true;

    [Tooltip("UAV X-Z box 0.22 x 0.22 m -> conservative circumscribed radius 0.15556 m. Must match actual world-space collider dimensions.")]
    public float leaderPhysicalPlanarRadius = 0.15556f;

    [Tooltip("Leader external surface clearance at which warning begins, at zero speed (m).")]
    public float leaderProximityStartClearance = 1.80f;

    [Tooltip("Extra early-warning range (m) per m/s of approach speed, independent of TTC.")]
    public float leaderProximitySpeedLookAhead = 0.75f;

    [Header("Leader Closing-Speed / TTC Warning")]
    [Tooltip("Start a TTC warning at this many seconds before predicted surface contact.")]
    public float leaderTTCWarningStart = 2.20f;
    [Tooltip("TTC at which time-to-collision warning reaches its maximum.")]
    public float leaderTTCWarningFull = 0.50f;
    [Tooltip("Estimated planar braking acceleration for warning purposes, m/s^2 (not a physical guarantee).")]
    public float leaderEstimatedBrakingAcceleration = 4.0f;
    [Tooltip("Additional stopping-distance warning margin, metres.")]
    public float leaderBrakingMargin = 0.35f;
    [Tooltip("Ignore very small closing speeds, m/s.")]
    public float leaderMinimumClosingSpeed = 0.15f;
    [Tooltip("Exponent applied to the forward alignment (2 means side-obstacles contribute much less than head-on obstacles).")]
    public float leaderForwardWeightExponent = 2.0f;
    [Tooltip("How strongly to retain non-frontal CBF directional feedback when TTC danger is absent (0 to 1).")]
    [Range(0f, 1f)] public float leaderCBFNonFrontalFeedbackScale = 0.25f;

    [Tooltip("Leader external surface clearance at which proximity force reaches its cap (m).")]
    public float leaderProximityFullForceClearance = 0.40f;

    [Tooltip("Maximum added proximity force (haptic force command units). Start small and calibrate device output.")]
    public float leaderProximityForceMax = 0.30f;

    [Tooltip("Obstacle search radius around leader center (m); should exceed the maximum warning clearance plus body radius.")]
    public float leaderProximityQueryRadius = 10.0f;

    [Range(-0.2f, 0.9f)]
    [Tooltip("Minimum alignment between leader direction of travel and direction toward obstacle: 0 = forward half-plane.")]
    public float leaderProximityForwardDot = 0.35f;

    [Tooltip("Include proximity risk in the damping term before the shadow CBF becomes active.")]
    public bool proximityAlsoModulatesDamping = true;

    [Header("Directional Fixture: Distance and Stability Guard (v7.1)")]
    [Tooltip("Absolute maximum external UAV-surface clearance for ANY proximity force (m). Obstacles farther away are ignored, even if TTC or estimated motion is abnormal.")]
    public float fixtureMaxCueClearance = 3.5f;
    [Tooltip("Maximum plausible TRANSLATIONAL obstacle speed. More extreme Rigidbody velocities are discarded for haptic prediction (m/s).")]
    public float fixtureMaxObstacleTranslationSpeed = 5.0f;
    [Tooltip("Hard force safety cap for the proximity component in directional fixture mode. Overrides a previously serialized larger Leader Proximity Force Max.")]
    public float fixtureProximityForceSafetyCap = 0.16f;
    [Tooltip("Hard safety cap for the CBF correction haptic component in directional fixture mode.")]
    public float fixtureCBFForceSafetyCap = 0.08f;
    [Tooltip("Hard safety cap for direction-selective damping in directional fixture mode.")]
    public float fixtureDampingForceSafetyCap = 0.06f;
    [Tooltip("Total directional fixture force cap, regardless of old serialized Haptic Force Max.")]
    public float fixtureTotalForceSafetyCap = 0.28f;
    [Tooltip("Minimum smoothing time constant for directional fixture output, seconds. Larger = less jitter but more lag.")]
    public float fixtureMinimumForceSmoothTau = 0.08f;
    [Header("Risk-Dependent Haptic Damping")]
    public bool enableRiskDamping = true;
    public float hapticBaseDamping = 0.0f;
    public float hapticRiskDamping = 0.18f;
    public float hapticRiskExponent = 1.5f;

    [Tooltip("Risk below this value produces no obstacle-dependent damping.")]
    [Range(0f, 0.95f)]
    public float hapticRiskDeadzone = 0.05f;

    [Tooltip("When true, risk damping is disabled unless the leader has at least one active obstacle CBF constraint.")]
    public bool dampingRequiresActiveConstraint = true;

    public float hapticDampingForceMax = 0.8f;
    public float hapticVelocitySmoothTau = 0.05f;
    public float hapticVelocityDeadzone = 0.01f;

    [Header("Final Haptic Force")]
    public float hapticForceMax = 2.2f;
    public float hapticForceSmoothTau = 0.04f;
    public float forceReleaseLerp = 0.25f;
    public bool hapticForcePlanarOnly = true;

    [Header("Haptic Force Axis Mapping")]
    public bool convertUnityForceToHapticAxes = false;
    public bool invertHapticForceDirection = false;

    [Header("Debug")]
    public bool drawDebugVectors = true;
    public bool drawGizmos = true;
    public bool drawFormationSlots = true;
    public bool drawActivationEllipses = true;

    [Header("Runtime Debug - Ellipses")]
    public float dbgLeaderMajor;
    public float dbgLeaderMinor;
    public float dbgLeaderRisk;
    public float dbgAMajor;
    public float dbgAMinor;
    public float dbgARisk;
    public float dbgBMajor;
    public float dbgBMinor;
    public float dbgBRisk;

    [Header("Runtime Debug - CBF")]
    public Vector3 dbgLeaderNominalVelocity;
    public Vector3 dbgLeaderSafeVelocity;
    public Vector3 dbgLeaderCorrection;
    public float dbgLeaderAggregateRisk;
    public int dbgLeaderConstraints;
    public bool dbgLeaderFeasible;

    public Vector3 dbgFollowerNominalA;
    public Vector3 dbgFollowerNominalB;
    public Vector3 dbgFollowerSafeA;
    public Vector3 dbgFollowerSafeB;
    public Vector3 dbgFollowerCorrectionA;
    public Vector3 dbgFollowerCorrectionB;
    public float dbgFollowerPairDistance;
    public int dbgFollowerTotalConstraints;
    public bool dbgFollowerFeasible;
    public float dbgFollowerMaxConstraintViolation;
    public float dbgLeaderMaxConstraintViolation;
    public float dbgFollowerMinBarrier;
    public float dbgLeaderMinBarrier;

    [Header("Runtime Debug - Impedance and Haptic")]
    public float dbgK_A;
    public float dbgK_B;
    public float dbgD_A;
    public float dbgD_B;
    public Vector3 dbgDirectionalHapticForce;
    public Vector3 dbgDampingHapticForce;
    public Vector3 dbgProximityHapticForce;
    public float dbgLeaderProximityClearance = -1f;
    public float dbgLeaderProximityRisk;
    public string dbgLeaderProximityObstacle = "";
    [Header("Debug - Directional Fixture Validation v7.1")]
    public int dbgFixtureDetectedColliders;
    public int dbgFixtureRejectedFar;
    public int dbgFixtureRejectedVelocity;
    public int dbgFixtureConsideredColliders;
    public Vector3 dbgFinalHapticForce;
    public Vector3 dbgDeviceForceSent;
    [Header("Runtime Debug - Leader TTC and Active CBF")]
    public Vector3 dbgLeaderAppliedVelocity;
    public bool dbgLeaderActiveCBFApplied;
    public float dbgLeaderClosingSpeed;
    public float dbgLeaderTTC = -1f;
    public float dbgLeaderBrakingDistance;

    [DllImport("HapticsDirect")]
    static extern int getPosition(string configName, double[] position3);

    [DllImport("HapticsDirect")]
    static extern int getButtons(string configName, int[] buttons4, int[] last_buttons4, ref int inkwell);

    const string deviceName = "Default Device";

    readonly double[] hapticPosition = new double[3];
    readonly int[] buttonState = new int[4];
    readonly int[] lastButtonState = new int[4];
    int inkwell;

    float nextHapticTime;
    Vector3 filteredInput;
    Vector3 lastMappedHapticInput;
    Vector3 filteredHapticVelocity;
    float lastHapticSampleTime;
    bool hasPreviousHapticSample;

    Vector3 formationForward = Vector3.forward;
    Vector3 leaderNominalVelocity;
    Vector3 lastUnityForce;

    // Reused buffer: no per-frame allocations. Entries are filtered by obstacleMask.
    readonly Collider[] leaderProximityColliders = new Collider[512];

    FreeSpaceScan leaderScan;
    FreeSpaceScan scanA;
    FreeSpaceScan scanB;

    AdaptiveEllipseState leaderEllipse;
    AdaptiveEllipseState ellipseA;
    AdaptiveEllipseState ellipseB;

    SingleAgentCBFResult leaderCBF;
    JointFollowerCBFResult followerCBF;

    bool enabledByExperiment = true;

    public void SetEnabled(bool on)
    {
        SetExperimentControlEnabled(on);
    }

    public void SetExperimentControlEnabled(bool on)
    {
        enabledByExperiment = on;

        if (!on)
        {
            if (leader != null) leader.velocity = Vector3.zero;
            if (followerA != null) followerA.velocity = Vector3.zero;
            if (followerB != null) followerB.velocity = Vector3.zero;

            leaderNominalVelocity = Vector3.zero;
            filteredHapticVelocity = Vector3.zero;
            lastUnityForce = Vector3.zero;
            dbgDirectionalHapticForce = Vector3.zero;
            dbgDampingHapticForce = Vector3.zero;
            dbgProximityHapticForce = Vector3.zero;
            dbgLeaderProximityClearance = -1f;
            dbgLeaderProximityRisk = 0f;
            dbgLeaderProximityObstacle = "";
            dbgFinalHapticForce = Vector3.zero;
            dbgDeviceForceSent = Vector3.zero;
            dbgLeaderAppliedVelocity = Vector3.zero;
            dbgLeaderActiveCBFApplied = false;
            dbgFollowerAccelerationRelaxed = false;
            dbgFollowerEmergencyStopped = false;
            dbgFollowerResidualWithAcceleration = 0f;
            dbgFollowerRecoveryAttempted = false;
            dbgFollowerRecoverySucceeded = false;
            dbgFollowerRecoveryResidual = 0f;
            dbgFollowerHardPairMinimum = 0f;
            dbgFollowerSoftCorrectionMagnitude = 0f;
            dbgLeaderClosingSpeed = 0f;
            dbgLeaderTTC = -1f;
            dbgLeaderBrakingDistance = 0f;

            if (cbfSolver != null)
                cbfSolver.ResetSolverMemory();
            if (ellipseEstimator != null)
                ellipseEstimator.ResetEstimatorMemory();

            hasPreviousHapticSample = false;
            filteredInput = Vector3.zero;
            lastMappedHapticInput = Vector3.zero;
            if (hapticSender != null)
                hapticSender.KickForce(Vector3.zero);
        }
    }

    void OnDisable()
    {
        // Prevent the last commanded force from remaining active when Unity
        // disables this component or leaves Play mode.
        lastUnityForce = Vector3.zero;
        if (hapticSender != null)
            hapticSender.KickForce(Vector3.zero);
    }

    void Awake()
    {
        if (freeSpaceScanner == null)
            freeSpaceScanner = GetComponent<FreeSpaceScanner>();
        if (ellipseEstimator == null)
            ellipseEstimator = GetComponent<AdaptiveEllipseEstimator>();
        if (cbfSolver == null)
            cbfSolver = GetComponent<JointFollowerCBFSolver>();

        if (leader == null || followerA == null || followerB == null)
        {
            Debug.LogError("AdaptiveFormation: leader, followerA, and followerB must be assigned.");
            enabled = false;
            return;
        }

        ApplyComparisonSettings();
        SyncSubsystemSettings();
    }

    void Start()
    {
        Vector3 initialForward = transform.forward;
        initialForward.y = 0f;
        if (initialForward.sqrMagnitude < 1e-6f)
            initialForward = Vector3.forward;
        formationForward = initialForward.normalized;

        ConfigureRigidbody(leader);
        ConfigureRigidbody(followerA);
        ConfigureRigidbody(followerB);
    }

    void OnValidate()
    {
        fixedCircleRadius = Mathf.Max(0.001f, fixedCircleRadius);
        leaderPhysicalPlanarRadius = Mathf.Max(0.001f, leaderPhysicalPlanarRadius);
        leaderProximityFullForceClearance = Mathf.Max(0f, leaderProximityFullForceClearance);
        leaderProximityStartClearance = Mathf.Max(leaderProximityFullForceClearance + 0.05f, leaderProximityStartClearance);
        leaderProximityQueryRadius = Mathf.Max(leaderPhysicalPlanarRadius + leaderProximityStartClearance, leaderProximityQueryRadius);
        leaderProximityForceMax = Mathf.Max(0f, leaderProximityForceMax);
        leaderProximitySpeedLookAhead = Mathf.Max(0f, leaderProximitySpeedLookAhead);
        leaderTTCWarningStart = Mathf.Max(0.1f, leaderTTCWarningStart);
        leaderTTCWarningFull = Mathf.Clamp(leaderTTCWarningFull, 0.02f, leaderTTCWarningStart - 0.02f);
        leaderEstimatedBrakingAcceleration = Mathf.Max(0.1f, leaderEstimatedBrakingAcceleration);
        leaderBrakingMargin = Mathf.Max(0f, leaderBrakingMargin);
        leaderMinimumClosingSpeed = Mathf.Max(0.01f, leaderMinimumClosingSpeed);
        leaderForwardWeightExponent = Mathf.Max(1.0f, leaderForwardWeightExponent);
        fixtureMinimumRequestedSpeed = Mathf.Max(0.001f, fixtureMinimumRequestedSpeed);
        fixtureMaxCueClearance = Mathf.Max(0.3f, fixtureMaxCueClearance);
        fixtureMaxObstacleTranslationSpeed = Mathf.Max(0.1f, fixtureMaxObstacleTranslationSpeed);
        fixtureProximityForceSafetyCap = Mathf.Max(0f, fixtureProximityForceSafetyCap);
        fixtureCBFForceSafetyCap = Mathf.Max(0f, fixtureCBFForceSafetyCap);
        fixtureDampingForceSafetyCap = Mathf.Max(0f, fixtureDampingForceSafetyCap);
        fixtureTotalForceSafetyCap = Mathf.Max(0f, fixtureTotalForceSafetyCap);
        fixtureMinimumForceSmoothTau = Mathf.Max(0.01f, fixtureMinimumForceSmoothTau);
        ApplyComparisonSettings();
        SyncSubsystemSettings();
    }

    public void ApplyComparisonSettings()
    {
        if (!enforceAblationConfiguration) return;
        enableShapeAdaptiveCBF = true;   // NEVER disable the common CBF filter for M1/M3
        followersUseCBF = true;
        leaderUsesCBF = leaderActiveCBFForAllConditions;  // same active-leader policy in every ablation condition
        enableAdaptiveImpedance = (comparisonCondition == ComparisonCondition.M3_FixedCircle_AdaptiveImpedance
                                 || comparisonCondition == ComparisonCondition.M4_AdaptiveEllipse_AdaptiveImpedance);
    }

    public bool UsesFixedCircle()
    {
        return comparisonCondition == ComparisonCondition.M1_FixedCircle_FixedImpedance
            || comparisonCondition == ComparisonCondition.M3_FixedCircle_AdaptiveImpedance;
    }

    AdaptiveEllipseState EffectiveSafetyGeometry(AdaptiveEllipseState state)
    {
        if (!state.Valid || !UsesFixedCircle()) return state;
        float radius = Mathf.Max(0.001f, fixedCircleRadius);
        state.MajorAxis = radius;
        state.MinorAxis = radius;
        state.Q = PlanarMatrix2x2.FromEllipse(Vector3.forward, radius, radius);
        state.QDot = PlanarMatrix2x2.Zero;
        state.Axis = Vector3.forward;
        state.LateralAxis = Vector3.right;
        // Preserve EnvironmentRisk, scanner density, and clearance measurements for impedance.
        return state;
    }

    public AdaptiveEllipseState GetEffectiveSafetyState(Rigidbody body)
    {
        if (body == leader) return leaderEllipse;
        if (body == followerA) return ellipseA;
        if (body == followerB) return ellipseB;
        return AdaptiveEllipseState.CreateDefault(body);
    }

    void SyncSubsystemSettings()
    {
        if (freeSpaceScanner != null)
            freeSpaceScanner.obstacleMask = obstacleMask;
        if (cbfSolver != null)
            cbfSolver.obstacleMask = obstacleMask;
    }

    void FixedUpdate()
    {
        if (!enabledByExperiment)
        {
            ReleaseHapticForce();
            return;
        }

        float dt = Time.fixedDeltaTime;
        ApplyComparisonSettings();

        DriveLeaderWithHapticInput();
        UpdateFormationDirection();
        UpdateScansAndEllipses(dt);

        EvaluateLeaderSafety(dt);

        Vector3 nominalA = ComputeFollowerNominalVelocity(
            followerA,
            true,
            ellipseA.EnvironmentRisk,
            out float kA,
            out float dA,
            dt
        );

        Vector3 nominalB = ComputeFollowerNominalVelocity(
            followerB,
            false,
            ellipseB.EnvironmentRisk,
            out float kB,
            out float dB,
            dt
        );

        if (useInterFollowerImpedance)
            ApplyInterFollowerNominalCorrection(ref nominalA, ref nominalB, dt);

        nominalA.y = ComputeFollowerVerticalVelocity(followerA, dt);
        nominalB.y = ComputeFollowerVerticalVelocity(followerB, dt);

        dbgK_A = kA;
        dbgK_B = kB;
        dbgD_A = dA;
        dbgD_B = dB;
        dbgFollowerNominalA = nominalA;
        dbgFollowerNominalB = nominalB;

        if (enableShapeAdaptiveCBF && followersUseCBF)
        {
            followerCBF = cbfSolver.SolveFollowers(
                followerA,
                followerB,
                leader,
                nominalA,
                nominalB,
                ellipseA,
                ellipseB,
                dt
            );

            followerA.velocity = followerCBF.SafeVelocityA;
            followerB.velocity = followerCBF.SafeVelocityB;
            if (followerCBF.EmergencyStopped && leaderContactGuardDuringFollowerFailure)
                GuardLeaderAgainstStoppedFollower();
            if (followerCBF.EmergencyStopped && enableLegacyWholeFormationEmergencyFreeze)
            {
                if (pauseWholeFormationOnEmergency)
                {
                    followerA.velocity = new Vector3(0f, followerA.velocity.y, 0f);
                    followerB.velocity = new Vector3(0f, followerB.velocity.y, 0f);
                }
                if (pauseLeaderOnFollowerEmergency)
                {
                    leader.velocity = new Vector3(0f, leader.velocity.y, 0f);
                    dbgLeaderAppliedVelocity = leader.velocity;
                }
            }
        }
        else
        {
            followerCBF = CreateInactiveJointResult(nominalA, nominalB);
            followerA.velocity = nominalA;
            followerB.velocity = nominalB;
        }

        UpdateRuntimeDebug();
        SendLeaderOnlyHapticFeedback(dt);
    }

    // Local collision interlock used ONLY when the joint solver has failed.
    // Unlike legacy emergency freeze, it does not halt safe lateral/away motion.
    void GuardLeaderAgainstStoppedFollower()
    {
        if (leader == null || followerA == null || followerB == null || cbfSolver == null) return;
        Vector3 vel = Planar(leader.velocity);
        if (vel.sqrMagnitude < 1e-8f) return;
        float margin = Mathf.Max(0f,leaderContactGuardExtraMargin);
        Rigidbody[] followers = { followerA, followerB };
        for (int i=0; i<followers.Length; i++)
        {
            Rigidbody f = followers[i];
            Vector3 toFollower = Planar(f.worldCenterOfMass-leader.worldCenterOfMass);
            float distance = toFollower.magnitude;
            if (distance < 1e-5f) continue;
            float hard = cbfSolver.HardPairDistance(f,leader,true);
            if (distance < hard+margin && Vector3.Dot(vel,toFollower/distance)>0f)
            {
                // Remove only the closing component, preserve tangential retreat.
                Vector3 normal = toFollower/distance;
                float closing = Vector3.Dot(vel,normal);
                vel -= normal*closing;
                leader.velocity = new Vector3(vel.x,leader.velocity.y,vel.z);
                dbgLeaderAppliedVelocity = leader.velocity;
            }
        }
    }

    void ConfigureRigidbody(Rigidbody rb)
    {
        if (rb == null)
            return;

        if (disableGravity)
            rb.useGravity = false;
        if (freezeAgentRotation)
            rb.freezeRotation = true;
    }

    void DriveLeaderWithHapticInput()
    {
        Vector3 inputDirection = ReadHapticAsUnityDirection();
        Vector3 desired = inputDirection * leaderSpeed;

        Vector3 planar = new Vector3(desired.x, 0f, desired.z);
        planar = Vector3.ClampMagnitude(planar, leaderMaxSpeed);

        float vertical = Mathf.Clamp(desired.y, -leaderMaxSpeed, leaderMaxSpeed);
        Vector3 target = new Vector3(planar.x, vertical, planar.z);

        leaderNominalVelocity = Vector3.Lerp(leader.velocity, target, leaderVelLerp);
        leader.velocity = leaderNominalVelocity;
    }

    void UpdateScansAndEllipses(float dt)
    {
        leaderScan = freeSpaceScanner.Scan(leader, dt);
        scanA = freeSpaceScanner.Scan(followerA, dt);
        scanB = freeSpaceScanner.Scan(followerB, dt);

        Vector3 leaderPreferred = GetPreferredDirection(leaderNominalVelocity);
        Vector3 followerPreferred = GetPreferredDirection(leader.velocity);

        leaderEllipse = ellipseEstimator.UpdateEllipse(
            leader,
            leaderNominalVelocity,
            leaderPreferred,
            leaderScan,
            dt
        );

        ellipseA = ellipseEstimator.UpdateEllipse(
            followerA,
            followerA.velocity,
            followerPreferred,
            scanA,
            dt
        );

        ellipseB = ellipseEstimator.UpdateEllipse(
            followerB,
            followerB.velocity,
            followerPreferred,
            scanB,
            dt
        );

        leaderEllipse = EffectiveSafetyGeometry(leaderEllipse);
        ellipseA = EffectiveSafetyGeometry(ellipseA);
        ellipseB = EffectiveSafetyGeometry(ellipseB);
    }

    void EvaluateLeaderSafety(float dt)
    {
        if (enableShapeAdaptiveCBF || enableCBFMediatedHaptic)
        {
            leaderCBF = cbfSolver.EvaluateSingleAgent(
                leader,
                leaderNominalVelocity,
                leaderEllipse,
                dt
            );
        }
        else
        {
            leaderCBF = CreateInactiveSingleResult(leaderNominalVelocity);
        }

        bool activeLeader = enableShapeAdaptiveCBF && leaderUsesCBF;
        Vector3 applied = activeLeader ? leaderCBF.SafeVelocity : leaderNominalVelocity;
        if (activeLeader && !leaderCBF.ApproximatelyFeasible &&
            leaderInfeasibleAction == LeaderInfeasibleAction.StopPlanar)
        {
            // A best-effort emergency stop is not a guarantee for moving obstacles.
            applied = new Vector3(0f, applied.y, 0f);
        }
        leader.velocity = applied;
        dbgLeaderAppliedVelocity = applied;
        dbgLeaderActiveCBFApplied = activeLeader;
    }

    Vector3 ComputeFollowerNominalVelocity(
        Rigidbody follower,
        bool isA,
        float environmentRisk,
        out float usedK,
        out float usedD,
        float dt
    )
    {
        Vector3 target = GetFollowerSlotWorldPosition(isA);
        Vector3 error = target - follower.position;
        error.y = 0f;

        Vector3 relativeVelocity = follower.velocity - leader.velocity;
        relativeVelocity.y = 0f;

        float risk = enableAdaptiveImpedance ? Mathf.Clamp01(environmentRisk) : 0f;
        float smoothRisk = risk * risk * (3f - 2f * risk);

        usedK = Mathf.Lerp(kClear, kRisk, smoothRisk);
        usedD = Mathf.Lerp(dClear, dRisk, smoothRisk);

        Vector3 force = usedK * error - usedD * relativeVelocity;
        Vector3 leaderPlanar = new Vector3(leader.velocity.x, 0f, leader.velocity.z);
        force += leaderPlanar * leaderVelocityFeedforward;
        force = Vector3.ClampMagnitude(force, impedanceForceMax);

        float mass = Mathf.Max(0.01f, follower.mass);
        Vector3 nominal = follower.velocity + force / mass * dt;
        nominal.y = follower.velocity.y;

        Vector3 planar = new Vector3(nominal.x, 0f, nominal.z);
        planar = Vector3.ClampMagnitude(planar, followerNominalMaxSpeed);
        nominal.x = planar.x;
        nominal.z = planar.z;

        if (drawDebugVectors)
        {
            Debug.DrawLine(follower.position, target, Color.yellow, 0.03f);
            Debug.DrawRay(follower.worldCenterOfMass, force.normalized * 0.7f, Color.magenta, 0.03f);
        }

        return nominal;
    }

    void ApplyInterFollowerNominalCorrection(
        ref Vector3 nominalA,
        ref Vector3 nominalB,
        float dt
    )
    {
        Vector3 delta = followerB.position - followerA.position;
        delta.y = 0f;

        float distance = delta.magnitude;
        if (distance < 1e-4f)
            return;

        Vector3 direction = delta / distance;
        float desiredDistance = Vector3.Distance(
            Planar(GetFollowerSlotWorldPosition(true)),
            Planar(GetFollowerSlotWorldPosition(false))
        );

        Vector3 relativeVelocity = followerB.velocity - followerA.velocity;
        relativeVelocity.y = 0f;

        float stretch = distance - desiredDistance;
        float relativeAlong = Vector3.Dot(relativeVelocity, direction);
        float pairRisk = Mathf.Max(ellipseA.EnvironmentRisk, ellipseB.EnvironmentRisk);

        float kPair = kClear * interFollowerKScale * Mathf.Lerp(1f, 0.35f, pairRisk);
        float dPair = dClear * interFollowerDScale;

        Vector3 forceOnA = (kPair * stretch + dPair * relativeAlong) * direction;
        forceOnA = Vector3.ClampMagnitude(forceOnA, interFollowerForceMax);

        nominalA += forceOnA / Mathf.Max(0.01f, followerA.mass) * dt;
        nominalB -= forceOnA / Mathf.Max(0.01f, followerB.mass) * dt;
    }

    float ComputeFollowerVerticalVelocity(Rigidbody follower, float dt)
    {
        float positionError = leader.position.y - follower.position.y;
        float velocityError = leader.velocity.y - follower.velocity.y;
        float force = heightKp * positionError + heightKd * velocityError;
        force = Mathf.Clamp(force, -heightForceMax, heightForceMax);

        float velocity = follower.velocity.y
            + force / Mathf.Max(0.01f, follower.mass) * dt;
        return Mathf.Clamp(velocity, -followerVerticalSpeedMax, followerVerticalSpeedMax);
    }

    Vector3 GetFollowerSlotWorldPosition(bool isA)
    {
        Vector3 forward = formationForward;
        forward.y = 0f;
        if (forward.sqrMagnitude < 1e-8f)
            forward = Vector3.forward;
        forward.Normalize();

        Vector3 right = Vector3.Cross(Vector3.up, forward).normalized;
        float half = formationSpacing * 0.5f;
        float depth = formationSpacing * Mathf.Sqrt(3f) * 0.5f;
        float sign = followersInFrontOfLeader ? 1f : -1f;

        Vector3 offset = isA
            ? forward * depth * sign - right * half
            : forward * depth * sign + right * half;

        Vector3 target = leader.position + offset;
        target.y = leader.position.y;
        return target;
    }

    void UpdateFormationDirection()
    {
        Vector3 desired = formationForward;

        if (useMotionAlignedFormation)
        {
            Vector3 velocity = leader.velocity;
            velocity.y = 0f;

            if (velocity.magnitude > minFormationDirUpdateSpeed)
            {
                Vector3 candidate = velocity.normalized;
                if (!preventFormationDirectionFlip
                    || Vector3.Angle(formationForward, candidate) <= maxDirectionChangeAngle)
                {
                    desired = candidate;
                }
            }
        }
        else
        {
            desired = transform.forward;
            desired.y = 0f;
        }

        if (desired.sqrMagnitude < 1e-8f)
            desired = Vector3.forward;
        desired.Normalize();

        float alpha = 1f - Mathf.Exp(
            -Time.fixedDeltaTime / Mathf.Max(1e-6f, formationDirSmoothTau)
        );

        formationForward = Vector3.Slerp(formationForward, desired, alpha);
        formationForward.y = 0f;
        if (formationForward.sqrMagnitude < 1e-8f)
            formationForward = desired;
        formationForward.Normalize();
    }

    Vector3 GetPreferredDirection(Vector3 velocity)
    {
        Vector3 preferred = velocity;
        preferred.y = 0f;
        if (preferred.sqrMagnitude < 0.15f * 0.15f)
            preferred = formationForward;
        if (preferred.sqrMagnitude < 1e-8f)
            preferred = Vector3.forward;
        return preferred.normalized;
    }

    /// <summary>
    /// Leader-only, direction-selective virtual fixture.
    /// F = CBF guidance gated by approaching risk + proximity repulsion
    ///     + optional one-way damping. A command parallel to / away from a
    /// nearby wall is NOT damped by that wall. Follower forces are excluded.
    /// Do NOT enable automatic force-axis matching: the tested legacy device
    /// mapping remains unchanged (convertUnityForceToHapticAxes = false).
    /// </summary>
    void SendLeaderOnlyHapticFeedback(float dt)
    {
        if (hapticSender == null)
        {
            dbgDirectionalHapticForce = Vector3.zero;
            dbgProximityHapticForce = Vector3.zero;
            dbgDampingHapticForce = Vector3.zero;
            dbgFinalHapticForce = Vector3.zero;
            dbgDeviceForceSent = Vector3.zero;
            return;
        }

        // Use the human's requested movement, NOT CBF-filtered leader velocity.
        // Otherwise an active CBF stopping the vehicle would silence the cue.
        // ReadHapticAsUnityDirection() clips input to a unit direction for flight,
        // but filteredInput itself is not clipped. Keep haptic prediction consistent
        // with the actual speed-limited leader command.
        Vector3 requested = Vector3.ClampMagnitude(Planar(filteredInput), 1f)
            * Mathf.Min(Mathf.Max(0f, leaderSpeed), Mathf.Max(0f, leaderMaxSpeed));
        bool activelyCommanding = requested.magnitude >= fixtureMinimumRequestedSpeed;
        Vector3 desiredUnit = activelyCommanding ? requested.normalized : Vector3.zero;

        Vector3 proximityForce = Vector3.zero;
        float proximityRisk = 0f;
        float clearance = -1f;
        string obstacleName = "";
        Vector3 primaryAway = Vector3.zero;
        if (enableLeaderProximityHaptic && activelyCommanding)
            ComputeLeaderProximityHaptic(requested,
                out proximityForce, out proximityRisk, out clearance,
                out obstacleName, out primaryAway);

        // The CBF correction describes an obstacle-induced alteration of the
        // operator command. In fixture mode, suppress it when the user is
        // moving AWAY from detected danger or along the surface.
        Vector3 directionalForce = Vector3.zero;
        if (enableCBFMediatedHaptic)
        {
            Vector3 correction = Planar(leaderCBF.Correction);
            float mag = correction.magnitude;
            if (mag > cbfCorrectionDeadzone)
            {
                float gate = 1f;
                if (useDirectionSelectiveVirtualFixture)
                {
                    // Only oppose user motion when correction actually opposes
                    // the request and the direction-sensitive obstacle cue is live.
                    float opposing = activelyCommanding
                        ? Mathf.Clamp01(-Vector3.Dot(correction.normalized, desiredUnit)) : 0f;
                    gate = opposing * Mathf.Clamp01(proximityRisk) *
                        Mathf.Clamp01(fixtureCBFForceBlend);
                }
                directionalForce = correction.normalized * (mag - cbfCorrectionDeadzone)
                    * cbfCorrectionToForceGain * gate;
                if (useDirectionSelectiveVirtualFixture)
                    directionalForce = Vector3.ClampMagnitude(
                        directionalForce, Mathf.Max(0f, fixtureCBFForceSafetyCap));
            }
        }

        Vector3 dampingForce = Vector3.zero;
        if (enableRiskDamping && activelyCommanding)
        {
            Vector3 handVelocity = Planar(filteredHapticVelocity);
            if (handVelocity.magnitude > hapticVelocityDeadzone)
            {
                float risk = proximityRisk;
                if (!useDirectionSelectiveVirtualFixture)
                    risk = Mathf.Max(risk, Mathf.Clamp01(leaderCBF.AggregateRisk));
                float deadzone = Mathf.Clamp(hapticRiskDeadzone,0f,0.95f);
                risk = Mathf.InverseLerp(deadzone, 1f, risk);
                float b = hapticBaseDamping + hapticRiskDamping *
                    Mathf.Pow(risk, Mathf.Max(0.01f,hapticRiskExponent));
                if (useOneWayDirectionalDamping && primaryAway.sqrMagnitude > 1e-8f)
                {
                    // Dampen only physical hand velocity TOWARDS this obstacle.
                    float towardSpeed = Mathf.Max(0f,
                        -Vector3.Dot(handVelocity, primaryAway.normalized));
                    dampingForce = primaryAway.normalized * b * towardSpeed;
                }
                else if (!useDirectionSelectiveVirtualFixture)
                    dampingForce = -b * handVelocity;
                float dampingCap = useDirectionSelectiveVirtualFixture
                    ? Mathf.Min(hapticDampingForceMax, fixtureDampingForceSafetyCap)
                    : hapticDampingForceMax;
                dampingForce = Vector3.ClampMagnitude(dampingForce, Mathf.Max(0f, dampingCap));
            }
        }

        Vector3 combined = directionalForce + proximityForce + dampingForce;
        float totalCap = useDirectionSelectiveVirtualFixture
            ? Mathf.Min(hapticForceMax, fixtureTotalForceSafetyCap)
            : hapticForceMax;
        combined = Vector3.ClampMagnitude(combined, Mathf.Max(0f, totalCap));
        float smoothTau = useDirectionSelectiveVirtualFixture
            ? Mathf.Max(hapticForceSmoothTau, fixtureMinimumForceSmoothTau)
            : hapticForceSmoothTau;
        float alpha = 1f - Mathf.Exp(-dt / Mathf.Max(1e-6f, smoothTau));
        lastUnityForce = Vector3.Lerp(lastUnityForce,combined,alpha);
        if (combined.sqrMagnitude < 1e-8f)
            lastUnityForce = Vector3.Lerp(lastUnityForce,Vector3.zero,forceReleaseLerp);

        dbgDirectionalHapticForce = directionalForce;
        dbgProximityHapticForce = proximityForce;
        dbgLeaderProximityRisk = proximityRisk;
        dbgLeaderProximityClearance = clearance;
        dbgLeaderProximityObstacle = obstacleName;
        dbgDampingHapticForce = dampingForce;
        dbgFinalHapticForce = lastUnityForce;
        dbgDeviceForceSent = ConvertUnityForceToHapticForce(lastUnityForce);

        if (drawDebugVectors && leader != null)
        {
            Vector3 p = leader.worldCenterOfMass;
            Debug.DrawRay(p, leaderCBF.NominalVelocity*0.25f, Color.blue, 0.03f);
            Debug.DrawRay(p, leaderCBF.SafeVelocity*0.25f, Color.green, 0.03f);
            Debug.DrawRay(p,proximityForce,Color.magenta,0.03f);
        }
        hapticSender.KickForce(dbgDeviceForceSent);
    }

    /// <summary>
    /// v7.1: direction-selective and DISTANCE-BOUNDED leader obstacle fixture.
    /// In contrast to v7, absurd collider point velocities can never extend the
    /// warning to tens/hundreds of metres. Multiple nearby obstacles are
    /// strength-weighted AVERAGED (not blindly summed).
    /// </summary>
    void ComputeLeaderProximityHaptic(Vector3 requested,
        out Vector3 resultForce, out float resultRisk,
        out float resultClearance, out string resultObstacle,
        out Vector3 strongestAway)
    {
        resultForce = Vector3.zero;
        resultRisk = 0f;
        resultClearance = -1f;
        resultObstacle = "";
        strongestAway = Vector3.zero;
        dbgLeaderClosingSpeed = 0f;
        dbgLeaderTTC = -1f;
        dbgLeaderBrakingDistance = 0f;
        dbgFixtureDetectedColliders = 0;
        dbgFixtureRejectedFar = 0;
        dbgFixtureRejectedVelocity = 0;
        dbgFixtureConsideredColliders = 0;

        if (leader == null || obstacleMask.value == 0 ||
            requested.magnitude < fixtureMinimumRequestedSpeed) return;

        Vector3 requestDirection = requested.normalized;
        float bodyRadius = Mathf.Max(0.001f, leaderPhysicalPlanarRadius);
        float full = Mathf.Max(0f, leaderProximityFullForceClearance);
        // All haptic cues end at this PHYSICAL distance, independently of speed.
        float maxCue = Mathf.Max(full + 0.1f, fixtureMaxCueClearance);
        float start = Mathf.Min(maxCue,
            Mathf.Max(full + 0.05f, leaderProximityStartClearance));
        float queryRadius = Mathf.Min(leaderProximityQueryRadius,
            bodyRadius + maxCue + 0.15f);
        Vector3 center = leader.worldCenterOfMass;
        int count = Physics.OverlapSphereNonAlloc(center, queryRadius,
            leaderProximityColliders, obstacleMask, QueryTriggerInteraction.Ignore);
        dbgFixtureDetectedColliders = count;
        if (count >= leaderProximityColliders.Length)
            Debug.LogWarning("[Directional Fixture] Overlap collider buffer is full; check obstacle layers.");

        Vector3 weightedAway = Vector3.zero;
        float sumWeights = 0f;
        float strongest = 0f;
        // V7 erroneously used 0 in directional mode, ignoring the Inspector setting.
        float forwardDot = Mathf.Clamp(leaderProximityForwardDot, 0f, 0.95f);
        for (int i = 0; i < count; ++i)
        {
            Collider obstacle = leaderProximityColliders[i];
            if (obstacle == null || obstacle.attachedRigidbody == leader ||
                obstacle.transform.IsChildOf(leader.transform)) continue;
            Vector3 contact = obstacle.ClosestPoint(center);
            Vector3 awayVector = Planar(center - contact);
            float dist = awayVector.magnitude;
            if (float.IsNaN(dist) || float.IsInfinity(dist)) continue;
            if (dist < 1e-5f && Mathf.Abs(center.y - contact.y) > 0.05f)
                continue;

            // Essential hard cutoff: ignored even if an object's reported speed
            // is nonsensical. OverlapSphere can return large/odd colliders.
            float gap = Mathf.Max(0f, dist - bodyRadius);
            if (gap > maxCue)
            {
                dbgFixtureRejectedFar++;
                continue;
            }
            Vector3 away = dist < 1e-5f ? -requestDirection : awayVector / dist;
            Vector3 toward = -away;
            float alignmentRaw = Vector3.Dot(requestDirection, toward);
            if (alignmentRaw <= forwardDot) continue;

            // Translational rigidbody velocity ONLY. GetPointVelocity(contact)
            // adds omega x r, which produced impossible 100-800 m/s values in
            // the 19:58 CSV and a false TTC alert at hundreds of metres.
            Vector3 obstacleVelocity = Vector3.zero;
            if (obstacle.attachedRigidbody != null)
            {
                Vector3 measured = Planar(obstacle.attachedRigidbody.velocity);
                if (float.IsNaN(measured.x) || float.IsInfinity(measured.x) ||
                    float.IsNaN(measured.z) || float.IsInfinity(measured.z) ||
                    measured.magnitude > fixtureMaxObstacleTranslationSpeed)
                {
                    dbgFixtureRejectedVelocity++;
                    // Treat invalid motion estimates as unknown/zero for
                    // haptic prediction. This is NOT collision safety logic.
                }
                else obstacleVelocity = measured;
            }

            float closing = Vector3.Dot(requested - obstacleVelocity, toward);
            if (closing <= Mathf.Max(0.01f, leaderMinimumClosingSpeed)) continue;
            closing = Mathf.Min(closing,
                Mathf.Max(0.01f, leaderMaxSpeed) + fixtureMaxObstacleTranslationSpeed);
            float ttc = gap / closing;
            float brake = closing * closing /
                (2f * Mathf.Max(0.1f, leaderEstimatedBrakingAcceleration));
            float early = Mathf.Min(maxCue,
                start + closing * Mathf.Max(0f, leaderProximitySpeedLookAhead));
            float timeRisk = Mathf.Clamp01((leaderTTCWarningStart - ttc) /
                Mathf.Max(0.05f, leaderTTCWarningStart - leaderTTCWarningFull));
            float distRisk = Mathf.Clamp01((early - gap) /
                Mathf.Max(0.05f, early - full));
            float brakeRisk = Mathf.Clamp01((brake + leaderBrakingMargin - gap) /
                Mathf.Max(0.05f, brake + leaderBrakingMargin - full));
            float danger = Mathf.Max(timeRisk, Mathf.Max(brakeRisk, 0.55f * distRisk));

            // Fade continuously to ZERO near maxCue even if TTC is short.
            // This prevents abrupt 'remote pulling' at the hard boundary.
            float rangeFraction = Mathf.Clamp01((maxCue - gap) /
                Mathf.Max(0.1f, maxCue - full));
            float rangeFade = rangeFraction * rangeFraction * (3f - 2f * rangeFraction);
            float alignment = Mathf.Clamp01((alignmentRaw - forwardDot) /
                Mathf.Max(0.05f, 1f - forwardDot));
            float strength = danger * rangeFade * Mathf.Pow(alignment,
                Mathf.Max(1f, leaderForwardWeightExponent));
            if (strength <= 1e-6f) continue;

            dbgFixtureConsideredColliders++;
            weightedAway += away * strength;
            sumWeights += strength;
            if (strength > strongest)
            {
                strongest = strength;
                strongestAway = away;
                resultClearance = gap;
                resultObstacle = obstacle.name;
                dbgLeaderClosingSpeed = closing;
                dbgLeaderTTC = ttc;
                dbgLeaderBrakingDistance = brake;
            }
        }

        // Averaging keeps multiple colliders of the SAME tree from saturating
        // the force while preserving constructive directional guidance.
        float peak = useDirectionSelectiveVirtualFixture
            ? Mathf.Min(leaderProximityForceMax, fixtureProximityForceSafetyCap)
            : leaderProximityForceMax;
        if (sumWeights > 1e-6f)
            resultForce = weightedAway / Mathf.Max(1f, sumWeights) * Mathf.Max(0f, peak);
        resultRisk = strongest;
    }

    void ReleaseHapticForce()
    {
        lastUnityForce = Vector3.Lerp(lastUnityForce, Vector3.zero, forceReleaseLerp);
        if (hapticSender != null)
            hapticSender.KickForce(ConvertUnityForceToHapticForce(lastUnityForce));
    }

    Vector3 ReadHapticAsUnityDirection()
    {
        if (Time.time >= nextHapticTime)
        {
            nextHapticTime = Time.time + 1f / Mathf.Max(1f, hapticHz);

            int readResult;
            try
            {
                readResult = getPosition(deviceName, hapticPosition);
            }
            catch
            {
                readResult = -1;
            }

            if (readResult == 0)
            {
                Vector3 mapped = new Vector3(
                    -(float)hapticPosition[0],
                    (float)hapticPosition[1],
                    -(float)hapticPosition[2]
                ) * inputScale;

                if (invertX) mapped.x = -mapped.x;
                if (invertY) mapped.y = -mapped.y;
                if (invertZ) mapped.z = -mapped.z;

                float sampleTime = Time.unscaledTime;
                float sampleDt = Time.fixedDeltaTime;

                if (hasPreviousHapticSample)
                {
                    sampleDt = sampleTime - lastHapticSampleTime;
                    if (sampleDt > 1e-4f && sampleDt < 0.25f)
                    {
                        Vector3 rawVelocity = (mapped - lastMappedHapticInput) / sampleDt;
                        float alphaVelocity = 1f - Mathf.Exp(
                            -sampleDt / Mathf.Max(1e-6f, hapticVelocitySmoothTau)
                        );
                        filteredHapticVelocity = Vector3.Lerp(
                            filteredHapticVelocity,
                            rawVelocity,
                            alphaVelocity
                        );
                    }
                    else
                    {
                        filteredHapticVelocity = Vector3.zero;
                    }
                }
                else
                {
                    filteredHapticVelocity = Vector3.zero;
                    hasPreviousHapticSample = true;
                }

                lastMappedHapticInput = mapped;
                lastHapticSampleTime = sampleTime;

                Vector3 input = mapped.magnitude < inputDeadzone
                    ? Vector3.zero
                    : mapped;

                float alphaInput = 1f - Mathf.Exp(
                    -sampleDt / Mathf.Max(1e-6f, inputSmoothTau)
                );
                filteredInput = Vector3.Lerp(filteredInput, input, alphaInput);
            }
            else
            {
                filteredInput = Vector3.zero;
                filteredHapticVelocity = Vector3.Lerp(
                    filteredHapticVelocity,
                    Vector3.zero,
                    0.25f
                );
                hasPreviousHapticSample = false;
            }

            try
            {
                getButtons(deviceName, buttonState, lastButtonState, ref inkwell);
            }
            catch
            {
            }
        }

        Vector3 direction = filteredInput;
        if (direction.sqrMagnitude < 1e-8f && useKeyboardFallback)
            direction = ReadKeyboardDirection();

        if (direction.magnitude > 1f)
            direction.Normalize();
        return direction;
    }

    Vector3 ReadKeyboardDirection()
    {
        float x = 0f;
        float y = 0f;
        float z = 0f;

        if (Input.GetKey(KeyCode.A)) x -= 1f;
        if (Input.GetKey(KeyCode.D)) x += 1f;
        if (Input.GetKey(KeyCode.Q)) y -= 1f;
        if (Input.GetKey(KeyCode.E)) y += 1f;
        if (Input.GetKey(KeyCode.S)) z -= 1f;
        if (Input.GetKey(KeyCode.W)) z += 1f;

        Vector3 input = new Vector3(x, y, z) * keyboardInputScale;
        if (input.magnitude > 1f)
            input.Normalize();
        return input;
    }

    Vector3 ConvertUnityForceToHapticForce(Vector3 unityForce)
    {
        Vector3 force = unityForce;

        // Input maps device (hx,hy,hz) -> Unity (-hx,hy,-hz), then applies
        // per-axis invertX/Y/Z. Force is a vector, so use the inverse map.
        // Leave convertUnityForceToHapticAxes OFF if your existing sender mapping is correct.
        // Preserve the working haptic output direction from the user's earlier project.
        // The v4 AutoMatch feature is deliberately removed; it inverted the force.
        if (convertUnityForceToHapticAxes)
        {
            force = new Vector3(
                (invertX ? 1f : -1f) * unityForce.x,
                (invertY ? -1f : 1f) * unityForce.y,
                (invertZ ? 1f : -1f) * unityForce.z
            );
        }

        if (invertHapticForceDirection)
            force = -force;
        return force;
    }

    void UpdateRuntimeDebug()
    {
        dbgLeaderMajor = leaderEllipse.MajorAxis;
        dbgLeaderMinor = leaderEllipse.MinorAxis;
        dbgLeaderRisk = leaderEllipse.EnvironmentRisk;
        dbgAMajor = ellipseA.MajorAxis;
        dbgAMinor = ellipseA.MinorAxis;
        dbgARisk = ellipseA.EnvironmentRisk;
        dbgBMajor = ellipseB.MajorAxis;
        dbgBMinor = ellipseB.MinorAxis;
        dbgBRisk = ellipseB.EnvironmentRisk;

        dbgLeaderNominalVelocity = leaderCBF.NominalVelocity;
        dbgLeaderSafeVelocity = leaderCBF.SafeVelocity;
        dbgLeaderCorrection = leaderCBF.Correction;
        dbgLeaderAggregateRisk = leaderCBF.AggregateRisk;
        dbgLeaderConstraints = leaderCBF.ActiveObstacleConstraints;
        dbgLeaderFeasible = leaderCBF.ApproximatelyFeasible;
        dbgLeaderMaxConstraintViolation = leaderCBF.MaxConstraintViolation;
        dbgLeaderMinBarrier = leaderCBF.MinBarrier;

        dbgFollowerSafeA = followerCBF.SafeVelocityA;
        dbgFollowerSafeB = followerCBF.SafeVelocityB;
        dbgFollowerCorrectionA = followerCBF.CorrectionA;
        dbgFollowerCorrectionB = followerCBF.CorrectionB;
        dbgFollowerPairDistance = followerCBF.MinimumPairDistance;
        dbgFollowerTotalConstraints = followerCBF.TotalConstraints;
        dbgFollowerFeasible = followerCBF.ApproximatelyFeasible;
        dbgFollowerMaxConstraintViolation = followerCBF.MaxConstraintViolation;
        dbgFollowerMinBarrier = followerCBF.MinBarrier;
        dbgFollowerAccelerationRelaxed = followerCBF.AccelerationRelaxed;
        dbgFollowerEmergencyStopped = followerCBF.EmergencyStopped;
        dbgFollowerResidualWithAcceleration = followerCBF.ResidualWithAcceleration;
        dbgFollowerRecoveryAttempted = followerCBF.RecoveryAttempted;
        dbgFollowerRecoverySucceeded = followerCBF.RecoverySucceeded;
        dbgFollowerRecoveryResidual = followerCBF.RecoveryResidual;
        dbgFollowerHardPairMinimum = followerCBF.HardPairMinimumDistance;
        dbgFollowerSoftCorrectionMagnitude = followerCBF.SoftSeparationCorrectionMagnitude;
    }

    SingleAgentCBFResult CreateInactiveSingleResult(Vector3 nominal)
    {
        SingleAgentCBFResult result = new SingleAgentCBFResult();
        result.NominalVelocity = nominal;
        result.SafeVelocity = nominal;
        result.Correction = Vector3.zero;
        result.MinBarrier = float.PositiveInfinity;
        result.ApproximatelyFeasible = true;
        return result;
    }

    JointFollowerCBFResult CreateInactiveJointResult(Vector3 nominalA, Vector3 nominalB)
    {
        JointFollowerCBFResult result = new JointFollowerCBFResult();
        result.NominalVelocityA = nominalA;
        result.NominalVelocityB = nominalB;
        result.SafeVelocityA = nominalA;
        result.SafeVelocityB = nominalB;
        result.CorrectionA = Vector3.zero;
        result.CorrectionB = Vector3.zero;
        result.MinBarrier = float.PositiveInfinity;
        result.MinimumPairDistance = Vector3.Distance(
            Planar(followerA.position),
            Planar(followerB.position)
        );
        result.ApproximatelyFeasible = true;
        return result;
    }

    Vector3 Planar(Vector3 value)
    {
        return new Vector3(value.x, 0f, value.z);
    }

    void OnDrawGizmosSelected()
    {
        if (!drawGizmos)
            return;

        DrawEllipse(leaderEllipse, Color.cyan, 1f);
        DrawEllipse(ellipseA, Color.green, 1f);
        DrawEllipse(ellipseB, Color.yellow, 1f);

        if (drawActivationEllipses && cbfSolver != null)
        {
            float scale = cbfSolver.obstacleActivationScale;
            DrawEllipse(leaderEllipse, Color.white, scale);
            DrawEllipse(ellipseA, Color.gray, scale);
            DrawEllipse(ellipseB, Color.gray, scale);
        }

        if (leader != null && followerA != null && followerB != null)
        {
            Gizmos.color = Color.white;
            Gizmos.DrawLine(leader.position, followerA.position);
            Gizmos.DrawLine(leader.position, followerB.position);
            Gizmos.DrawLine(followerA.position, followerB.position);
        }

        if (drawFormationSlots && leader != null)
        {
            Vector3 slotA = GetFollowerSlotWorldPosition(true);
            Vector3 slotB = GetFollowerSlotWorldPosition(false);
            Gizmos.color = Color.magenta;
            Gizmos.DrawSphere(slotA, 0.12f);
            Gizmos.DrawSphere(slotB, 0.12f);
            Gizmos.DrawLine(leader.position, slotA);
            Gizmos.DrawLine(leader.position, slotB);
        }
    }

    void DrawEllipse(AdaptiveEllipseState ellipse, Color color, float scale)
    {
        if (!ellipse.Valid)
            return;

        const int segments = 48;
        Gizmos.color = color;

        Vector3 center = ellipse.Center;
        Vector3 axis = ellipse.Axis.normalized;
        Vector3 lateral = ellipse.LateralAxis.normalized;
        float major = ellipse.MajorAxis * scale;
        float minor = ellipse.MinorAxis * scale;

        Vector3 previous = center + axis * major;
        for (int i = 1; i <= segments; i++)
        {
            float angle = 2f * Mathf.PI * i / segments;
            Vector3 point = center
                + axis * (Mathf.Cos(angle) * major)
                + lateral * (Mathf.Sin(angle) * minor);
            Gizmos.DrawLine(previous, point);
            previous = point;
        }
    }
}
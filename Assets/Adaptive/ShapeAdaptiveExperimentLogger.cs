using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// Experiment controller and CSV logger for the shape-adaptive three-UAV formation system.
///
/// Main functions:
/// 1) Start/finish/reset experiment segments.
/// 2) End the experiment automatically when the configured finish zone is reached.
/// 3) Log trajectory, velocity, haptic force, obstacle clearance, inter-agent distance,
///    formation error, adaptive ellipse parameters, CBF corrections and feasibility.
/// 4) Write a visualization-ready time-series CSV and a compact summary CSV.
///
/// Put an invisible BoxCollider finish zone slightly BEFORE the physical end wall.
/// The finish zone should normally be Is Trigger = true and must NOT be in obstacleMask.
/// </summary>
[DefaultExecutionOrder(1000)]
public class ShapeAdaptiveExperimentLogger : MonoBehaviour
{
    public enum ExperimentState
    {
        Idle,
        Running,
        Finished
    }

    public enum FinishMode
    {
        LeaderOnly,
        AllAgents,
        AnyAgent
    }

    public enum CollisionAction
    {
        LogOnly,
        ResetSegment,
        FinishRun
    }

    [Header("Experiment Metadata")]
    public string participantId = "P00";
    public string conditionName = "ShapeAdaptiveCBF";
    public int trialNumber = 1;

    [Header("Agents")]
    public Rigidbody leader;
    public Rigidbody followerA;
    public Rigidbody followerB;

    [Header("Control System")]
    public AdaptiveFormation formation;
    public HapticForceSender hapticSender;
    public bool enableControlOnlyWhileRunning = true;

    [Header("Start / Manual Finish")]
    public bool autoStart = false;
    public KeyCode startKey = KeyCode.Space;
    public KeyCode manualFinishKey = KeyCode.F;

    [Header("Automatic Finish Zone")]
    [Tooltip("Use a separate invisible BoxCollider placed slightly before the physical end wall.")]
    public Collider finishZone;
    public FinishMode finishMode = FinishMode.LeaderOnly;

    [Tooltip("Allows finishing when the agent is this close to the finish-zone volume, even if its center has not entered it.")]
    public float finishReachTolerance = 0.05f;

    [Tooltip("Require the finish condition to remain true for this duration to reject one-frame detections.")]
    public float finishHoldTime = 0.10f;

    [Header("Terminal Wall Safety Guard (Optional)")]
    [Tooltip("Assign ONLY the end wall Collider, never the generic Obstacle layer. A leader approaching it can safely end before followers enter the wall.")]
    public Collider terminalWall;
    [Tooltip("Finish successfully at a designated safe checkpoint BEFORE the wall, using the leader-to-end-wall planar distance (m). 4.5m assumes followers are ~2.6m in front and must be checked in your scene.")]
    public bool enableTerminalWallGuard = true;
    public float terminalGuardLeaderClearance = 4.5f;
    [Tooltip("Require leader to have moved some distance forward from start before guard triggers. Prevents immediate finish if scene starts near end wall.")]
    public float terminalGuardMinLeaderPath = 5.0f;

    [Header("Reset Transforms")]
    public Transform leaderStart;
    public Transform followerAStart;
    public Transform followerBStart;

    [Header("Collision Handling")]
    public bool autoAttachCollisionRelays = true;
    public LayerMask collisionMask = ~0;
    // A true physical contact terminates a failed run instead of silently resetting it.
    // Set LogOnly if you need to record collisions without terminating (can become stuck).
    public CollisionAction collisionAction = CollisionAction.FinishRun;
    public float collisionDebounceSeconds = 0.25f;

    [Header("Obstacle Clearance Logging")]
    public LayerMask obstacleMask;
    public float distanceQueryRadius = 6.0f;

    [Header("Independent Proximity Event Monitor")]
    public bool enableProximityEvents = true;
    [Tooltip("Conservative surface-clearance proxy, in metres, below which an encounter is recorded.")]
    public float proximityEnterClearance = 0.05f;
    [Tooltip("Rearm only after this larger clearance; avoids repeated counts each physics frame.")]
    public float proximityExitClearance = 0.10f;
    [Tooltip("Circumscribed planar radius of 0.22 x 0.22 m UAV box: sqrt(0.11^2 + 0.11^2).")]
    public float uavPlanarCircumscribedRadius = 0.15556f;

    [Header("Sampling")]
    public float sampleHz = 20.0f;
    public bool logHapticForce = true;

    [Header("Obstacle Geometry Export")]
    public bool exportObstacleGeometry = true;
    public bool logObstacleMotion = true;
    public ObstacleGeometryCsvExporter obstacleGeometryExporter;

    [Header("Output")]
    public bool saveToDesktop = true;
    public string outputFolderName = "ShapeAdaptiveExperiments";
    public bool autoQuitOnFinish = false;

    [Header("Runtime Status")]
    [SerializeField] ExperimentState state = ExperimentState.Idle;
    [SerializeField] int segmentId;
    [SerializeField] int collisionCount;
    [SerializeField] int proximityEventCount;
    [SerializeField] float minimumEstimatedSurfaceClearance;
    [SerializeField] float sessionElapsed;
    [SerializeField] float segmentElapsed;
    [SerializeField] float currentGroupMinObstacleDistance;
    [SerializeField] float bestGroupMinObstacleDistance;
    [SerializeField] float currentMinimumInterAgentDistance;
    [SerializeField] float bestMinimumInterAgentDistance;
    [SerializeField] string currentTimeSeriesPath;
    [SerializeField] string currentSummaryPath;
    [SerializeField] string currentObstacleGeometryPath;
    [SerializeField] string currentObstacleMotionPath;

    readonly Collider[] obstacleHits = new Collider[256];
    readonly Dictionary<string, float> lastCollisionTimeByPair = new Dictionary<string, float>();
    readonly HashSet<string> activeProximityPairs = new HashSet<string>();
    readonly HashSet<string> proximityPairsSeenThisStep = new HashSet<string>();
    readonly List<string> staleProximityPairs = new List<string>();

    StreamWriter timeSeriesWriter;
    float sessionStartTime = -1f;
    float segmentStartTime = -1f;
    float nextSampleTime;
    float finishConditionSince = -1f;

    bool pendingCollisionAction;
    string pendingCollisionReason = "";

    Vector3 previousLeaderPosition;
    Vector3 previousAPosition;
    Vector3 previousBPosition;
    bool hasPreviousPositions;

    float leaderPathTotal;
    float followerAPathTotal;
    float followerBPathTotal;
    float leaderPathSegment;
    float followerAPathSegment;
    float followerBPathSegment;

    float minLeaderObstacleTotal = float.PositiveInfinity;
    float minAObstacleTotal = float.PositiveInfinity;
    float minBObstacleTotal = float.PositiveInfinity;
    float minLeaderADistanceTotal = float.PositiveInfinity;
    float minLeaderBDistanceTotal = float.PositiveInfinity;
    float minABDistanceTotal = float.PositiveInfinity;
    float maxFormationRmseTotal;
    float maxHapticForceTotal;
    float maxLeaderCorrectionTotal;
    float maxFollowerCorrectionTotal;
    int infeasibleSampleCount;
    int sampleCount;

    public ExperimentState State
    {
        get { return state; }
    }

    void Awake()
    {
        ValidateReferences();

        if (autoAttachCollisionRelays)
        {
            AttachCollisionRelay(leader, "leader");
            AttachCollisionRelay(followerA, "followerA");
            AttachCollisionRelay(followerB, "followerB");
        }
    }

    void Start()
    {
        EnterIdle();

        if (autoStart)
            StartExperiment();
    }

    void Update()
    {
        if (state == ExperimentState.Idle)
        {
            if (Input.GetKeyDown(startKey))
                StartExperiment();
            return;
        }

        if (state == ExperimentState.Running && Input.GetKeyDown(manualFinishKey))
            FinishRun("manual_key");
    }

    void FixedUpdate()
    {
        if (state != ExperimentState.Running)
            return;

        sessionElapsed = Time.time - sessionStartTime;
        segmentElapsed = Time.time - segmentStartTime;

        UpdatePathLengths();
        UpdateAggregateMetrics();
        UpdateProximityEvents();
        SampleIfDue();
        CheckAutomaticFinish();

        if (pendingCollisionAction && state == ExperimentState.Running)
        {
            pendingCollisionAction = false;

            if (collisionAction == CollisionAction.ResetSegment)
                ResetToStartAndBeginNewSegment(pendingCollisionReason);
            else if (collisionAction == CollisionAction.FinishRun)
                FinishRun(pendingCollisionReason);
        }
    }

    void OnApplicationQuit()
    {
        CloseWriterSafely();

        if (obstacleGeometryExporter != null)
            obstacleGeometryExporter.CloseMotionLog();
    }

    void OnDestroy()
    {
        CloseWriterSafely();

        if (obstacleGeometryExporter != null)
            obstacleGeometryExporter.CloseMotionLog();
    }

    void ValidateReferences()
    {
        if (formation == null)
            formation = GetComponent<AdaptiveFormation>();

        if (obstacleGeometryExporter == null)
            obstacleGeometryExporter = GetComponent<ObstacleGeometryCsvExporter>();

        if (formation != null)
        {
            if (leader == null) leader = formation.leader;
            if (followerA == null) followerA = formation.followerA;
            if (followerB == null) followerB = formation.followerB;
            if (hapticSender == null) hapticSender = formation.hapticSender;
        }

        if (leader == null || followerA == null || followerB == null)
            Debug.LogError("ShapeAdaptiveExperimentLogger: assign leader, followerA and followerB.");
    }

    void AttachCollisionRelay(Rigidbody body, string agentName)
    {
        if (body == null)
            return;

        ExperimentCollisionRelay relay = body.GetComponent<ExperimentCollisionRelay>();
        if (relay == null)
            relay = body.gameObject.AddComponent<ExperimentCollisionRelay>();

        relay.logger = this;
        relay.agentName = agentName;
        relay.collisionMask = collisionMask;
    }

    void EnterIdle()
    {
        state = ExperimentState.Idle;
        sessionStartTime = -1f;
        segmentStartTime = -1f;
        segmentId = 0;
        collisionCount = 0;
        proximityEventCount = 0;
        sessionElapsed = 0f;
        segmentElapsed = 0f;
        finishConditionSince = -1f;
        pendingCollisionAction = false;
        currentTimeSeriesPath = "";
        currentSummaryPath = "";

        ResetMetrics();
        CloseWriterSafely();

        if (formation != null && enableControlOnlyWhileRunning)
            formation.SetEnabled(false);

        ZeroVelocities();
        Debug.Log("[Experiment] IDLE. Press SPACE to start.");
    }

    public void StartExperiment()
    {
        if (state == ExperimentState.Running)
            return;

        ValidateReferences();
        ResetMetrics();
        collisionCount = 0;
        proximityEventCount = 0;
        segmentId = 0;
        if (formation != null) formation.ApplyComparisonSettings();
        sessionStartTime = Time.time;
        nextSampleTime = Time.time;
        state = ExperimentState.Running;

        OpenOutputFiles();
        ExportObstacleFiles();
        BeginNewSegment("experiment_start");

        Debug.Log($"[Experiment] RUNNING: {participantId}, {conditionName}, trial {trialNumber}");
    }

    void BeginNewSegment(string reason)
    {
        segmentId++;
        segmentStartTime = Time.time;
        segmentElapsed = 0f;
        finishConditionSince = -1f;

        leaderPathSegment = 0f;
        followerAPathSegment = 0f;
        followerBPathSegment = 0f;
        hasPreviousPositions = false;

        if (formation != null && enableControlOnlyWhileRunning)
            formation.SetEnabled(true);

        WriteCurrentRow("EVENT", "SEGMENT_START", reason);
    }

    void ResetToStartAndBeginNewSegment(string reason)
    {
        WriteCurrentRow("EVENT", "SEGMENT_END", reason);

        if (formation != null && enableControlOnlyWhileRunning)
            formation.SetEnabled(false);

        Teleport(leader, leaderStart);
        Teleport(followerA, followerAStart);
        Teleport(followerB, followerBStart);
        ZeroVelocities();

        WriteCurrentRow("EVENT", "RESET_TO_START", reason);
        BeginNewSegment("after_collision_reset");
    }

    public void FinishRun()
    {
        FinishRun("external_call");
    }

    public void FinishRun(string reason)
    {
        if (state != ExperimentState.Running)
            return;

        sessionElapsed = Time.time - sessionStartTime;
        segmentElapsed = Time.time - segmentStartTime;

        UpdatePathLengths();
        UpdateAggregateMetrics();
        WriteCurrentRow("EVENT", "FINISH", reason);

        state = ExperimentState.Finished;

        if (formation != null && enableControlOnlyWhileRunning)
            formation.SetEnabled(false);

        ZeroVelocities();
        CloseWriterSafely();

        if (obstacleGeometryExporter != null)
            obstacleGeometryExporter.CloseMotionLog();

        WriteSummaryCsv(reason);

        Debug.Log(
            $"[Experiment] FINISHED | reason={reason} | collisions={collisionCount} " +
            $"| time={sessionElapsed:F2}s | groupMinObs={SafeFinite(bestGroupMinObstacleDistance):F3}m " +
            $"| minAgentDistance={SafeFinite(bestMinimumInterAgentDistance):F3}m " +
            $"| timeseries={currentTimeSeriesPath} | summary={currentSummaryPath} " +
            $"| obstacles={currentObstacleGeometryPath}"
        );

        if (autoQuitOnFinish)
        {
#if UNITY_EDITOR
            UnityEditor.EditorApplication.isPlaying = false;
#else
            Application.Quit();
#endif
        }
    }

    public void ReportCollision(
        string agentName,
        string otherObjectName,
        Vector3 contactPointWorld
    )
    {
        if (state != ExperimentState.Running)
            return;

        string pairKey = BuildCollisionPairKey(agentName, otherObjectName);
        float lastTime;
        if (lastCollisionTimeByPair.TryGetValue(pairKey, out lastTime))
        {
            if (Time.time - lastTime < collisionDebounceSeconds)
                return;
        }
        lastCollisionTimeByPair[pairKey] = Time.time;

        collisionCount++;
        string reason = $"{agentName}_with_{otherObjectName}";
        WriteCurrentRow(
            "EVENT",
            "COLLISION",
            reason + $"@({F(contactPointWorld.x)}|{F(contactPointWorld.y)}|{F(contactPointWorld.z)})"
        );

        if (collisionAction != CollisionAction.LogOnly)
        {
            pendingCollisionAction = true;
            pendingCollisionReason = "collision_" + reason;
        }
    }

    // Do not interpret this proxy as an exact collider-to-collider distance.
    // Center-to-obstacle-surface minus the UAV's circumscribed planar radius
    // is conservative for a planar 0.22 m x 0.22 m footprint near tall obstacles.
    float EstimateSurfaceClearance(float centerToObstacleSurface)
    {
        return IsFinite(centerToObstacleSurface)
            ? Mathf.Max(0f, centerToObstacleSurface - Mathf.Max(0f, uavPlanarCircumscribedRadius))
            : float.PositiveInfinity;
    }

    void UpdateProximityEvents()
    {
        if (!enableProximityEvents) return;
        proximityPairsSeenThisStep.Clear();
        CheckAgentProximity(leader, "leader");
        CheckAgentProximity(followerA, "followerA");
        CheckAgentProximity(followerB, "followerB");

        // Forget pairs that have left the query volume; re-arm for a later encounter.
        staleProximityPairs.Clear();
        foreach (string key in activeProximityPairs)
            if (!proximityPairsSeenThisStep.Contains(key)) staleProximityPairs.Add(key);
        foreach (string key in staleProximityPairs) activeProximityPairs.Remove(key);
    }

    void CheckAgentProximity(Rigidbody body, string agentName)
    {
        if (body == null) return;
        Vector3 p = body.worldCenterOfMass;
        int n = Physics.OverlapSphereNonAlloc(
            p, Mathf.Max(0.05f, distanceQueryRadius), obstacleHits,
            obstacleMask, QueryTriggerInteraction.Ignore);

        float exit = Mathf.Max(proximityExitClearance, proximityEnterClearance + 0.001f);
        for (int i = 0; i < n; i++)
        {
            Collider obstacle = obstacleHits[i];
            if (obstacle == null || obstacle == finishZone || IsAgentCollider(obstacle)) continue;
            string key = agentName + "|" + obstacle.GetInstanceID();
            proximityPairsSeenThisStep.Add(key);

            float centerDistance = Vector3.Distance(p, obstacle.ClosestPoint(p));
            float clearance = EstimateSurfaceClearance(centerDistance);
            if (clearance <= proximityEnterClearance && !activeProximityPairs.Contains(key))
            {
                activeProximityPairs.Add(key);
                proximityEventCount++;
                WriteCurrentRow("EVENT", "PROXIMITY_ENTER",
                    agentName + "_near_" + obstacle.name
                    + "_estimated_surface_clearance_m=" + F(clearance));
            }
            else if (clearance >= exit)
            {
                activeProximityPairs.Remove(key);
            }
        }
    }

    string BuildCollisionPairKey(string a, string b)
    {
        return string.CompareOrdinal(a, b) <= 0 ? a + "|" + b : b + "|" + a;
    }

    void CheckAutomaticFinish()
    {
        if (enableTerminalWallGuard && terminalWall != null && leader != null &&
            leaderPathTotal >= terminalGuardMinLeaderPath)
        {
            Vector3 p = leader.worldCenterOfMass;
            Vector3 closest = terminalWall.ClosestPoint(p);
            float dx = p.x - closest.x;
            float dz = p.z - closest.z;
            float planarDistance = Mathf.Sqrt(dx*dx + dz*dz);
            if (planarDistance <= Mathf.Max(0.1f, terminalGuardLeaderClearance))
            {
                // The guard is an explicitly configured SAFE finish checkpoint.
                // It is not a proof that each follower has arrived at finishZone.
                FinishRun("finish_guard_before_terminal_wall");
                return;
            }
        }
        if (finishZone == null) return;

        bool leaderReached = HasReachedFinish(leader);
        bool aReached = HasReachedFinish(followerA);
        bool bReached = HasReachedFinish(followerB);
        bool reached;
        switch (finishMode)
        {
            case FinishMode.AllAgents: reached = leaderReached && aReached && bReached; break;
            case FinishMode.AnyAgent: reached = leaderReached || aReached || bReached; break;
            default: reached = leaderReached; break;
        }
        if (!reached) { finishConditionSince = -1f; return; }
        if (finishConditionSince < 0f) finishConditionSince = Time.time;
        if (Time.time - finishConditionSince >= Mathf.Max(0f, finishHoldTime))
            FinishRun("finish_zone_" + finishMode);
    }

    bool HasReachedFinish(Rigidbody body)
    {
        if (body == null || finishZone == null)
            return false;

        Vector3 p = body.worldCenterOfMass;
        if (finishZone.bounds.Contains(p))
            return true;

        Vector3 closest = finishZone.ClosestPoint(p);
        float distance = Vector3.Distance(p, closest);
        return distance <= Mathf.Max(0f, finishReachTolerance);
    }

    void SampleIfDue()
    {
        float interval = 1f / Mathf.Max(0.1f, sampleHz);
        if (Time.time + 1e-6f < nextSampleTime)
            return;

        nextSampleTime = Time.time + interval;

        if (formation != null && (!formation.dbgLeaderFeasible || !formation.dbgFollowerFeasible))
            infeasibleSampleCount++;

        WriteCurrentRow("SAMPLE", "", "");

        if (
            exportObstacleGeometry &&
            logObstacleMotion &&
            obstacleGeometryExporter != null
        )
        {
            obstacleGeometryExporter.SampleMotion(
                Time.time - sessionStartTime
            );
        }

        sampleCount++;
    }

    void UpdatePathLengths()
    {
        if (leader == null || followerA == null || followerB == null)
            return;

        if (!hasPreviousPositions)
        {
            previousLeaderPosition = leader.position;
            previousAPosition = followerA.position;
            previousBPosition = followerB.position;
            hasPreviousPositions = true;
            return;
        }

        float dl = Vector3.Distance(leader.position, previousLeaderPosition);
        float da = Vector3.Distance(followerA.position, previousAPosition);
        float db = Vector3.Distance(followerB.position, previousBPosition);

        leaderPathTotal += dl;
        followerAPathTotal += da;
        followerBPathTotal += db;
        leaderPathSegment += dl;
        followerAPathSegment += da;
        followerBPathSegment += db;

        previousLeaderPosition = leader.position;
        previousAPosition = followerA.position;
        previousBPosition = followerB.position;
    }

    void UpdateAggregateMetrics()
    {
        Snapshot s = CaptureSnapshot();

        currentGroupMinObstacleDistance = MinFinite(s.leaderObstacleDistance, s.aObstacleDistance, s.bObstacleDistance);
        minimumEstimatedSurfaceClearance = EstimateSurfaceClearance(currentGroupMinObstacleDistance);
        currentMinimumInterAgentDistance = Mathf.Min(s.distanceLA, Mathf.Min(s.distanceLB, s.distanceAB));

        if (IsFinite(currentGroupMinObstacleDistance))
            bestGroupMinObstacleDistance = Mathf.Min(bestGroupMinObstacleDistance, currentGroupMinObstacleDistance);

        bestMinimumInterAgentDistance = Mathf.Min(bestMinimumInterAgentDistance, currentMinimumInterAgentDistance);

        if (IsFinite(s.leaderObstacleDistance)) minLeaderObstacleTotal = Mathf.Min(minLeaderObstacleTotal, s.leaderObstacleDistance);
        if (IsFinite(s.aObstacleDistance)) minAObstacleTotal = Mathf.Min(minAObstacleTotal, s.aObstacleDistance);
        if (IsFinite(s.bObstacleDistance)) minBObstacleTotal = Mathf.Min(minBObstacleTotal, s.bObstacleDistance);

        minLeaderADistanceTotal = Mathf.Min(minLeaderADistanceTotal, s.distanceLA);
        minLeaderBDistanceTotal = Mathf.Min(minLeaderBDistanceTotal, s.distanceLB);
        minABDistanceTotal = Mathf.Min(minABDistanceTotal, s.distanceAB);
        maxFormationRmseTotal = Mathf.Max(maxFormationRmseTotal, s.formationRmse);
        maxHapticForceTotal = Mathf.Max(maxHapticForceTotal, s.hapticForce.magnitude);
        maxLeaderCorrectionTotal = Mathf.Max(maxLeaderCorrectionTotal, s.leaderCorrection.magnitude);
        maxFollowerCorrectionTotal = Mathf.Max(
            maxFollowerCorrectionTotal,
            Mathf.Max(s.aCorrection.magnitude, s.bCorrection.magnitude)
        );
    }

    Snapshot CaptureSnapshot()
    {
        Snapshot s = new Snapshot();

        s.leaderPosition = GetPosition(leader);
        s.aPosition = GetPosition(followerA);
        s.bPosition = GetPosition(followerB);
        s.leaderVelocity = GetVelocity(leader);
        s.aVelocity = GetVelocity(followerA);
        s.bVelocity = GetVelocity(followerB);

        s.hapticForce = GetHapticForce();

        s.leaderObstacleDistance = MinDistanceToObstacles(leader);
        s.aObstacleDistance = MinDistanceToObstacles(followerA);
        s.bObstacleDistance = MinDistanceToObstacles(followerB);

        s.distanceLA = PlanarDistance(s.leaderPosition, s.aPosition);
        s.distanceLB = PlanarDistance(s.leaderPosition, s.bPosition);
        s.distanceAB = PlanarDistance(s.aPosition, s.bPosition);

        float targetSpacing = formation != null ? formation.formationSpacing : 0f;
        float eLA = s.distanceLA - targetSpacing;
        float eLB = s.distanceLB - targetSpacing;
        float eAB = s.distanceAB - targetSpacing;
        s.formationRmse = Mathf.Sqrt((eLA * eLA + eLB * eLB + eAB * eAB) / 3f);
        s.formationMaxError = Mathf.Max(Mathf.Abs(eLA), Mathf.Max(Mathf.Abs(eLB), Mathf.Abs(eAB)));

        if (formation != null)
        {
            s.leaderMajor = formation.dbgLeaderMajor;
            s.leaderMinor = formation.dbgLeaderMinor;
            s.leaderRisk = formation.dbgLeaderRisk;
            s.aMajor = formation.dbgAMajor;
            s.aMinor = formation.dbgAMinor;
            s.aRisk = formation.dbgARisk;
            s.bMajor = formation.dbgBMajor;
            s.bMinor = formation.dbgBMinor;
            s.bRisk = formation.dbgBRisk;

            if (formation.ellipseEstimator != null)
            {
                AdaptiveEllipseState leaderState = formation.GetEffectiveSafetyState(leader);
                AdaptiveEllipseState aState = formation.GetEffectiveSafetyState(followerA);
                AdaptiveEllipseState bState = formation.GetEffectiveSafetyState(followerB);

                s.leaderAxisDeg = AxisAngleDeg(leaderState.Axis);
                s.aAxisDeg = AxisAngleDeg(aState.Axis);
                s.bAxisDeg = AxisAngleDeg(bState.Axis);
                s.leaderDensity = leaderState.DensityScore;
                s.aDensity = aState.DensityScore;
                s.bDensity = bState.DensityScore;
                s.leaderMinScanDistance = leaderState.MinScanDistance;
                s.aMinScanDistance = aState.MinScanDistance;
                s.bMinScanDistance = bState.MinScanDistance;
                s.leaderGapClearance = leaderState.SelectedGapClearance;
                s.aGapClearance = aState.SelectedGapClearance;
                s.bGapClearance = bState.SelectedGapClearance;
            }

            s.leaderNominal = formation.dbgLeaderNominalVelocity;
            s.leaderSafe = formation.dbgLeaderSafeVelocity;
            s.leaderCorrection = formation.dbgLeaderCorrection;
            s.leaderCbfRisk = formation.dbgLeaderAggregateRisk;
            s.leaderConstraints = formation.dbgLeaderConstraints;
            s.leaderFeasible = formation.dbgLeaderFeasible;

            s.aNominal = formation.dbgFollowerNominalA;
            s.bNominal = formation.dbgFollowerNominalB;
            s.aSafe = formation.dbgFollowerSafeA;
            s.bSafe = formation.dbgFollowerSafeB;
            s.aCorrection = formation.dbgFollowerCorrectionA;
            s.bCorrection = formation.dbgFollowerCorrectionB;
            s.followerPairDistanceDebug = formation.dbgFollowerPairDistance;
            s.followerTotalConstraints = formation.dbgFollowerTotalConstraints;
            s.followerFeasible = formation.dbgFollowerFeasible;
            s.leaderMaxViolation = formation.dbgLeaderMaxConstraintViolation;
            s.followerMaxViolation = formation.dbgFollowerMaxConstraintViolation;
            s.leaderMinBarrier = formation.dbgLeaderMinBarrier;
            s.followerMinBarrier = formation.dbgFollowerMinBarrier;

            s.kA = formation.dbgK_A;
            s.dA = formation.dbgD_A;
            s.kB = formation.dbgK_B;
            s.dB = formation.dbgD_B;
            s.directionalHapticForce = formation.dbgDirectionalHapticForce;
            s.dampingHapticForce = formation.dbgDampingHapticForce;
            s.proximityHapticForce = formation.dbgProximityHapticForce;
            s.proximityHapticRisk = formation.dbgLeaderProximityRisk;
            s.proximityHapticClearance = formation.dbgLeaderProximityClearance;
            s.fixtureObstacleName = formation.dbgLeaderProximityObstacle;
            s.fixtureDetected = formation.dbgFixtureDetectedColliders;
            s.fixtureRejectedFar = formation.dbgFixtureRejectedFar;
            s.fixtureRejectedVelocity = formation.dbgFixtureRejectedVelocity;
            s.fixtureConsidered = formation.dbgFixtureConsideredColliders;
            s.leaderApplied = formation.dbgLeaderAppliedVelocity;
            s.leaderActiveCBF = formation.dbgLeaderActiveCBFApplied;
            s.leaderClosingSpeed = formation.dbgLeaderClosingSpeed;
            s.leaderTTC = formation.dbgLeaderTTC;
            s.leaderBrakingDistance = formation.dbgLeaderBrakingDistance;
            s.followerAccelerationRelaxed = formation.dbgFollowerAccelerationRelaxed;
            s.followerEmergencyStopped = formation.dbgFollowerEmergencyStopped;
            s.followerResidualWithAcceleration = formation.dbgFollowerResidualWithAcceleration;
            s.followerRecoveryAttempted = formation.dbgFollowerRecoveryAttempted;
            s.followerRecoverySucceeded = formation.dbgFollowerRecoverySucceeded;
            s.followerRecoveryResidual = formation.dbgFollowerRecoveryResidual;
            s.hardPairMinimum = formation.dbgFollowerHardPairMinimum;
            s.softSeparationCorrection = formation.dbgFollowerSoftCorrectionMagnitude;
        }
        else
        {
            s.leaderFeasible = true;
            s.followerFeasible = true;
        }

        return s;
    }

    float MinDistanceToObstacles(Rigidbody body)
    {
        if (body == null)
            return float.PositiveInfinity;

        Vector3 p = body.worldCenterOfMass;
        int hitCount = Physics.OverlapSphereNonAlloc(
            p,
            Mathf.Max(0.05f, distanceQueryRadius),
            obstacleHits,
            obstacleMask,
            QueryTriggerInteraction.Ignore
        );

        float best = float.PositiveInfinity;
        for (int i = 0; i < hitCount; i++)
        {
            Collider col = obstacleHits[i];
            if (col == null)
                continue;

            if (finishZone != null && col == finishZone)
                continue;

            if (IsAgentCollider(col))
                continue;

            Vector3 closest = col.ClosestPoint(p);
            float distance = Vector3.Distance(p, closest);
            best = Mathf.Min(best, distance);
        }

        return best;
    }

    bool IsAgentCollider(Collider col)
    {
        if (col == null)
            return false;

        return IsColliderFromBody(col, leader)
            || IsColliderFromBody(col, followerA)
            || IsColliderFromBody(col, followerB);
    }

    bool IsColliderFromBody(Collider col, Rigidbody body)
    {
        if (body == null)
            return false;

        return col.attachedRigidbody == body || col.transform.IsChildOf(body.transform);
    }

    void OpenOutputFiles()
    {
        CloseWriterSafely();

        string root = saveToDesktop
            ? Environment.GetFolderPath(Environment.SpecialFolder.Desktop)
            : Application.persistentDataPath;

        string folder = Path.Combine(root, SanitizeFileName(outputFolderName));
        Directory.CreateDirectory(folder);

        string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
        string baseName = string.Join(
            "_",
            SanitizeFileName(participantId),
            SanitizeFileName(conditionName),
            "T" + trialNumber.ToString("D2", CultureInfo.InvariantCulture),
            stamp
        );

        currentTimeSeriesPath = Path.Combine(folder, baseName + "_timeseries.csv");
        currentSummaryPath = Path.Combine(folder, baseName + "_summary.csv");
        currentObstacleGeometryPath = Path.Combine(folder, baseName + "_obstacles.csv");
        currentObstacleMotionPath = Path.Combine(folder, baseName + "_obstacle_motion.csv");

        timeSeriesWriter = new StreamWriter(currentTimeSeriesPath, false, new UTF8Encoding(false));
        timeSeriesWriter.WriteLine(BuildHeader());
        timeSeriesWriter.Flush();
    }

    void ExportObstacleFiles()
    {
        if (!exportObstacleGeometry)
            return;

        if (obstacleGeometryExporter == null)
        {
            Debug.LogWarning(
                "[Experiment] Obstacle geometry export is enabled, but " +
                "ObstacleGeometryCsvExporter is not assigned."
            );
            return;
        }

        obstacleGeometryExporter.logObstacleMotion = logObstacleMotion;

        obstacleGeometryExporter.ExportAtExperimentStart(
            currentObstacleGeometryPath,
            currentObstacleMotionPath,
            obstacleMask,
            finishZone,
            leader,
            followerA,
            followerB
        );
    }

    string BuildHeader()
    {
        return string.Join(",", new string[]
        {
            "row_type", "event", "reason",
            "participant_id", "condition", "trial", "session_t", "segment_t", "segment_id",

            "leader_x", "leader_y", "leader_z", "leader_vx", "leader_vy", "leader_vz",
            "A_x", "A_y", "A_z", "A_vx", "A_vy", "A_vz",
            "B_x", "B_y", "B_z", "B_vx", "B_vy", "B_vz",

            "haptic_fx", "haptic_fy", "haptic_fz", "haptic_fmag",
            "haptic_directional_mag", "haptic_damping_mag",

            "leader_obstacle_distance", "A_obstacle_distance", "B_obstacle_distance", "group_min_obstacle_distance",
            "distance_LA", "distance_LB", "distance_AB", "minimum_inter_agent_distance",
            "formation_target_spacing", "formation_rmse", "formation_max_abs_error",

            "leader_path_total", "A_path_total", "B_path_total",
            "leader_path_segment", "A_path_segment", "B_path_segment",

            "leader_ellipse_major", "leader_ellipse_minor", "leader_ellipse_ratio", "leader_ellipse_axis_deg",
            "leader_environment_risk", "leader_density", "leader_min_scan_distance", "leader_selected_gap_clearance",
            "A_ellipse_major", "A_ellipse_minor", "A_ellipse_ratio", "A_ellipse_axis_deg",
            "A_environment_risk", "A_density", "A_min_scan_distance", "A_selected_gap_clearance",
            "B_ellipse_major", "B_ellipse_minor", "B_ellipse_ratio", "B_ellipse_axis_deg",
            "B_environment_risk", "B_density", "B_min_scan_distance", "B_selected_gap_clearance",

            "leader_nominal_vx", "leader_nominal_vz", "leader_safe_vx", "leader_safe_vz",
            "leader_correction_x", "leader_correction_z", "leader_correction_mag",
            "leader_cbf_risk", "leader_active_constraints", "leader_cbf_feasible",

            "A_nominal_vx", "A_nominal_vz", "A_safe_vx", "A_safe_vz",
            "A_correction_x", "A_correction_z", "A_correction_mag",
            "B_nominal_vx", "B_nominal_vz", "B_safe_vx", "B_safe_vz",
            "B_correction_x", "B_correction_z", "B_correction_mag",

            "follower_pair_distance_debug", "follower_total_constraints", "follower_cbf_feasible",
            "K_A", "D_A", "K_B", "D_B", "collision_count",
            "proximity_event_count", "leader_est_surface_clearance_m", "A_est_surface_clearance_m",
            "B_est_surface_clearance_m", "group_est_surface_clearance_m",
            "leader_max_cbf_residual", "follower_max_cbf_residual",
            "leader_min_active_barrier", "follower_min_active_barrier", "ablation_condition",
            "leader_proximity_haptic_force_mag", "leader_proximity_haptic_risk", "leader_proximity_haptic_clearance_m",
            "leader_applied_vx", "leader_applied_vz", "leader_active_cbf_enabled",
            "leader_closing_speed_mps", "leader_ttc_s", "leader_braking_distance_m",
            "follower_acceleration_relaxed", "follower_emergency_stop", "follower_residual_with_acceleration",
            "follower_recovery_attempted", "follower_recovery_succeeded", "follower_recovery_residual",
            "hard_follower_pair_distance_m", "soft_separation_correction_mps",
            "fixture_selected_obstacle", "fixture_detected_colliders",
            "fixture_rejected_far", "fixture_rejected_velocity",
            "fixture_considered_colliders"
        });
    }

    void WriteCurrentRow(string rowType, string eventName, string reason)
    {
        if (timeSeriesWriter == null)
            return;

        Snapshot s = CaptureSnapshot();
        float groupMinObstacle = MinFinite(s.leaderObstacleDistance, s.aObstacleDistance, s.bObstacleDistance);
        float minAgentDistance = Mathf.Min(s.distanceLA, Mathf.Min(s.distanceLB, s.distanceAB));
        float targetSpacing = formation != null ? formation.formationSpacing : 0f;

        List<string> values = new List<string>(110)
        {
            EscapeCsv(rowType), EscapeCsv(eventName), EscapeCsv(reason),
            EscapeCsv(participantId), EscapeCsv(conditionName), trialNumber.ToString(CultureInfo.InvariantCulture),
            F(Time.time - sessionStartTime), F(Time.time - segmentStartTime), segmentId.ToString(CultureInfo.InvariantCulture),

            F(s.leaderPosition.x), F(s.leaderPosition.y), F(s.leaderPosition.z),
            F(s.leaderVelocity.x), F(s.leaderVelocity.y), F(s.leaderVelocity.z),
            F(s.aPosition.x), F(s.aPosition.y), F(s.aPosition.z),
            F(s.aVelocity.x), F(s.aVelocity.y), F(s.aVelocity.z),
            F(s.bPosition.x), F(s.bPosition.y), F(s.bPosition.z),
            F(s.bVelocity.x), F(s.bVelocity.y), F(s.bVelocity.z),

            F(s.hapticForce.x), F(s.hapticForce.y), F(s.hapticForce.z), F(s.hapticForce.magnitude),
            F(s.directionalHapticForce.magnitude), F(s.dampingHapticForce.magnitude),

            FF(s.leaderObstacleDistance), FF(s.aObstacleDistance), FF(s.bObstacleDistance), FF(groupMinObstacle),
            F(s.distanceLA), F(s.distanceLB), F(s.distanceAB), F(minAgentDistance),
            F(targetSpacing), F(s.formationRmse), F(s.formationMaxError),

            F(leaderPathTotal), F(followerAPathTotal), F(followerBPathTotal),
            F(leaderPathSegment), F(followerAPathSegment), F(followerBPathSegment),

            F(s.leaderMajor), F(s.leaderMinor), F(SafeRatio(s.leaderMajor, s.leaderMinor)), F(s.leaderAxisDeg),
            F(s.leaderRisk), F(s.leaderDensity), FF(s.leaderMinScanDistance), FF(s.leaderGapClearance),
            F(s.aMajor), F(s.aMinor), F(SafeRatio(s.aMajor, s.aMinor)), F(s.aAxisDeg),
            F(s.aRisk), F(s.aDensity), FF(s.aMinScanDistance), FF(s.aGapClearance),
            F(s.bMajor), F(s.bMinor), F(SafeRatio(s.bMajor, s.bMinor)), F(s.bAxisDeg),
            F(s.bRisk), F(s.bDensity), FF(s.bMinScanDistance), FF(s.bGapClearance),

            F(s.leaderNominal.x), F(s.leaderNominal.z), F(s.leaderSafe.x), F(s.leaderSafe.z),
            F(s.leaderCorrection.x), F(s.leaderCorrection.z), F(s.leaderCorrection.magnitude),
            F(s.leaderCbfRisk), s.leaderConstraints.ToString(CultureInfo.InvariantCulture), Bool01(s.leaderFeasible),

            F(s.aNominal.x), F(s.aNominal.z), F(s.aSafe.x), F(s.aSafe.z),
            F(s.aCorrection.x), F(s.aCorrection.z), F(s.aCorrection.magnitude),
            F(s.bNominal.x), F(s.bNominal.z), F(s.bSafe.x), F(s.bSafe.z),
            F(s.bCorrection.x), F(s.bCorrection.z), F(s.bCorrection.magnitude),

            F(s.followerPairDistanceDebug), s.followerTotalConstraints.ToString(CultureInfo.InvariantCulture), Bool01(s.followerFeasible),
            F(s.kA), F(s.dA), F(s.kB), F(s.dB), collisionCount.ToString(CultureInfo.InvariantCulture),
            proximityEventCount.ToString(CultureInfo.InvariantCulture),
            FF(EstimateSurfaceClearance(s.leaderObstacleDistance)),
            FF(EstimateSurfaceClearance(s.aObstacleDistance)),
            FF(EstimateSurfaceClearance(s.bObstacleDistance)),
            FF(EstimateSurfaceClearance(groupMinObstacle)),
            F(s.leaderMaxViolation), F(s.followerMaxViolation),
            FF(s.leaderMinBarrier), FF(s.followerMinBarrier),
            EscapeCsv(formation != null ? formation.comparisonCondition.ToString() : "none"),
            F(s.proximityHapticForce.magnitude), F(s.proximityHapticRisk),
            F(s.proximityHapticClearance),
            F(s.leaderApplied.x), F(s.leaderApplied.z), Bool01(s.leaderActiveCBF),
            F(s.leaderClosingSpeed), F(s.leaderTTC), F(s.leaderBrakingDistance),
            Bool01(s.followerAccelerationRelaxed), Bool01(s.followerEmergencyStopped), F(s.followerResidualWithAcceleration),
            Bool01(s.followerRecoveryAttempted), Bool01(s.followerRecoverySucceeded), F(s.followerRecoveryResidual),
            F(s.hardPairMinimum), F(s.softSeparationCorrection),
            EscapeCsv(s.fixtureObstacleName),
            s.fixtureDetected.ToString(CultureInfo.InvariantCulture),
            s.fixtureRejectedFar.ToString(CultureInfo.InvariantCulture),
            s.fixtureRejectedVelocity.ToString(CultureInfo.InvariantCulture),
            s.fixtureConsidered.ToString(CultureInfo.InvariantCulture)
        };

        timeSeriesWriter.WriteLine(string.Join(",", values));

        if (rowType != "SAMPLE" || sampleCount % Mathf.Max(1, Mathf.RoundToInt(sampleHz)) == 0)
            timeSeriesWriter.Flush();
    }

    void WriteSummaryCsv(string finishReason)
    {
        try
        {
            using (StreamWriter writer = new StreamWriter(currentSummaryPath, false, new UTF8Encoding(false)))
            {
                writer.WriteLine("metric,value");
                WriteSummary(writer, "participant_id", participantId);
                WriteSummary(writer, "condition", conditionName);
                WriteSummary(writer, "trial", trialNumber.ToString(CultureInfo.InvariantCulture));
                WriteSummary(writer, "finish_reason", finishReason);
                WriteSummary(writer, "completion_time_s", F(sessionElapsed));
                WriteSummary(writer, "segments", segmentId.ToString(CultureInfo.InvariantCulture));
                WriteSummary(writer, "collision_count", collisionCount.ToString(CultureInfo.InvariantCulture));
                WriteSummary(writer, "proximity_event_count", proximityEventCount.ToString(CultureInfo.InvariantCulture));
                WriteSummary(writer, "proximity_enter_threshold_m", F(proximityEnterClearance));
                WriteSummary(writer, "uav_planar_circumscribed_radius_m", F(uavPlanarCircumscribedRadius));
                bool reachedDesignatedFinish = finishReason.StartsWith("finish_zone_") ||
                    finishReason == "finish_guard_before_terminal_wall";
                WriteSummary(writer, "reached_designated_finish", Bool01(reachedDesignatedFinish));
                WriteSummary(writer, "collision_free_success", Bool01(reachedDesignatedFinish && collisionCount == 0));
                WriteSummary(writer, "trial_success", Bool01(reachedDesignatedFinish && collisionCount == 0));
                WriteSummary(writer, "terminal_guard_used", Bool01(finishReason == "finish_guard_before_terminal_wall"));
                WriteSummary(writer, "terminal_guard_clearance_m", F(terminalGuardLeaderClearance));
                WriteSummary(writer, "finish_mode", finishMode.ToString());
                if (formation != null)
                {
                    WriteSummary(writer, "ablation_condition", formation.comparisonCondition.ToString());
                    WriteSummary(writer, "fixed_circle_radius_m", F(formation.fixedCircleRadius));
                    WriteSummary(writer, "direction_selective_virtual_fixture", Bool01(formation.useDirectionSelectiveVirtualFixture));
                    WriteSummary(writer, "fixture_max_cue_clearance_m", F(formation.fixtureMaxCueClearance));
                    WriteSummary(writer, "fixture_max_obstacle_translation_mps", F(formation.fixtureMaxObstacleTranslationSpeed));
                    WriteSummary(writer, "fixture_proximity_force_cap", F(formation.fixtureProximityForceSafetyCap));
                    WriteSummary(writer, "fixture_cbf_force_cap", F(formation.fixtureCBFForceSafetyCap));
                    WriteSummary(writer, "fixture_damping_force_cap", F(formation.fixtureDampingForceSafetyCap));
                    WriteSummary(writer, "fixture_total_force_cap", F(formation.fixtureTotalForceSafetyCap));
                    WriteSummary(writer, "pause_leader_on_follower_emergency", Bool01(formation.pauseLeaderOnFollowerEmergency));
                    if (formation.ellipseEstimator != null)
                    {
                        WriteSummary(writer, "ellipse_physical_clearance_floor_enabled", Bool01(formation.ellipseEstimator.usePhysicalClearanceFloor));
                        WriteSummary(writer, "ellipse_physical_surface_margin_m", F(formation.ellipseEstimator.physicalSurfaceMargin));
                    }
                    WriteSummary(writer, "leader_uses_cbf", Bool01(formation.leaderUsesCBF));
                    WriteSummary(writer, "leader_active_cbf_for_all_conditions", Bool01(formation.leaderActiveCBFForAllConditions));
                    WriteSummary(writer, "leader_infeasibility_policy", formation.leaderInfeasibleAction.ToString());
                    WriteSummary(writer, "leader_ttc_warning_start_s", F(formation.leaderTTCWarningStart));
                    WriteSummary(writer, "leader_ttc_warning_full_s", F(formation.leaderTTCWarningFull));
                    WriteSummary(writer, "leader_assumed_braking_acceleration_mps2", F(formation.leaderEstimatedBrakingAcceleration));
                    WriteSummary(writer, "followers_use_cbf", Bool01(formation.followersUseCBF));
                    if (formation.cbfSolver != null)
                    {
                        WriteSummary(writer, "follower_solver_iterations", formation.cbfSolver.projectionIterations.ToString(CultureInfo.InvariantCulture));
                        WriteSummary(writer, "follower_solver_relax_acceleration", Bool01(formation.cbfSolver.relaxAccelerationForSafety));
                        WriteSummary(writer, "follower_solver_emergency_stop", Bool01(formation.cbfSolver.emergencyStopOnFailure));
                        WriteSummary(writer, "follower_solver_surface_jacobian", Bool01(formation.cbfSolver.useSurfaceJacobian));
                        WriteSummary(writer, "follower_solver_obstacle_activation_scale", F(formation.cbfSolver.obstacleActivationScale));
                        WriteSummary(writer, "follower_preferred_distance_m", F(formation.cbfSolver.minimumFollowerDistance));
                        WriteSummary(writer, "follower_hard_distance_floor_m", F(formation.cbfSolver.hardFollowerDistanceFloor));
                        WriteSummary(writer, "leader_follower_hard_distance_floor_m", F(formation.cbfSolver.hardLeaderFollowerDistanceFloor));
                        WriteSummary(writer, "interagent_clearance_margin_m", F(formation.cbfSolver.interAgentPhysicalClearance));
                        WriteSummary(writer, "follower_recovery_enabled", Bool01(formation.cbfSolver.enableConstraintRecovery));
                    }
                }
                WriteSummary(writer, "sample_count", sampleCount.ToString(CultureInfo.InvariantCulture));

                WriteSummary(writer, "leader_path_m", F(leaderPathTotal));
                WriteSummary(writer, "A_path_m", F(followerAPathTotal));
                WriteSummary(writer, "B_path_m", F(followerBPathTotal));

                WriteSummary(writer, "leader_min_obstacle_distance_m", FF(minLeaderObstacleTotal));
                WriteSummary(writer, "A_min_obstacle_distance_m", FF(minAObstacleTotal));
                WriteSummary(writer, "B_min_obstacle_distance_m", FF(minBObstacleTotal));
                WriteSummary(writer, "group_min_obstacle_distance_m", FF(bestGroupMinObstacleDistance));
                WriteSummary(writer, "group_min_est_surface_clearance_m", FF(EstimateSurfaceClearance(bestGroupMinObstacleDistance)));

                WriteSummary(writer, "minimum_LA_distance_m", FF(minLeaderADistanceTotal));
                WriteSummary(writer, "minimum_LB_distance_m", FF(minLeaderBDistanceTotal));
                WriteSummary(writer, "minimum_AB_distance_m", FF(minABDistanceTotal));
                WriteSummary(writer, "minimum_inter_agent_distance_m", FF(bestMinimumInterAgentDistance));

                WriteSummary(writer, "maximum_formation_rmse_m", F(maxFormationRmseTotal));
                WriteSummary(writer, "maximum_haptic_force", F(maxHapticForceTotal));
                WriteSummary(writer, "maximum_leader_cbf_correction", F(maxLeaderCorrectionTotal));
                WriteSummary(writer, "maximum_follower_cbf_correction", F(maxFollowerCorrectionTotal));
                WriteSummary(writer, "infeasible_sample_count", infeasibleSampleCount.ToString(CultureInfo.InvariantCulture));
            }
        }
        catch (Exception exception)
        {
            Debug.LogError("[Experiment] Failed to write summary CSV: " + exception.Message);
        }
    }

    void WriteSummary(StreamWriter writer, string metric, string value)
    {
        writer.WriteLine(EscapeCsv(metric) + "," + EscapeCsv(value));
    }

    void CloseWriterSafely()
    {
        if (timeSeriesWriter == null)
            return;

        try
        {
            timeSeriesWriter.Flush();
            timeSeriesWriter.Close();
        }
        catch (Exception exception)
        {
            Debug.LogWarning("[Experiment] Failed to close CSV writer cleanly: " + exception.Message);
        }
        finally
        {
            timeSeriesWriter = null;
        }
    }

    void ResetMetrics()
    {
        leaderPathTotal = 0f;
        followerAPathTotal = 0f;
        followerBPathTotal = 0f;
        leaderPathSegment = 0f;
        followerAPathSegment = 0f;
        followerBPathSegment = 0f;
        hasPreviousPositions = false;

        currentGroupMinObstacleDistance = float.PositiveInfinity;
        bestGroupMinObstacleDistance = float.PositiveInfinity;
        currentMinimumInterAgentDistance = float.PositiveInfinity;
        bestMinimumInterAgentDistance = float.PositiveInfinity;

        minLeaderObstacleTotal = float.PositiveInfinity;
        minAObstacleTotal = float.PositiveInfinity;
        minBObstacleTotal = float.PositiveInfinity;
        minLeaderADistanceTotal = float.PositiveInfinity;
        minLeaderBDistanceTotal = float.PositiveInfinity;
        minABDistanceTotal = float.PositiveInfinity;
        maxFormationRmseTotal = 0f;
        maxHapticForceTotal = 0f;
        maxLeaderCorrectionTotal = 0f;
        maxFollowerCorrectionTotal = 0f;
        infeasibleSampleCount = 0;
        sampleCount = 0;
        lastCollisionTimeByPair.Clear();
        activeProximityPairs.Clear();
        proximityPairsSeenThisStep.Clear();
        staleProximityPairs.Clear();
        proximityEventCount = 0;
        minimumEstimatedSurfaceClearance = float.PositiveInfinity;
    }

    Vector3 GetHapticForce()
    {
        if (!logHapticForce || hapticSender == null)
            return Vector3.zero;
        return hapticSender.ForceWorld;
    }

    Vector3 GetPosition(Rigidbody body)
    {
        return body != null ? body.position : Vector3.zero;
    }

    Vector3 GetVelocity(Rigidbody body)
    {
        return body != null ? body.velocity : Vector3.zero;
    }

    void Teleport(Rigidbody body, Transform target)
    {
        if (body == null || target == null)
            return;

        body.position = target.position;
        body.rotation = target.rotation;
    }

    void ZeroVelocities()
    {
        ZeroVelocity(leader);
        ZeroVelocity(followerA);
        ZeroVelocity(followerB);
    }

    void ZeroVelocity(Rigidbody body)
    {
        if (body == null)
            return;

        body.velocity = Vector3.zero;
        body.angularVelocity = Vector3.zero;
    }

    static float PlanarDistance(Vector3 a, Vector3 b)
    {
        Vector2 pa = new Vector2(a.x, a.z);
        Vector2 pb = new Vector2(b.x, b.z);
        return Vector2.Distance(pa, pb);
    }

    static float AxisAngleDeg(Vector3 axis)
    {
        axis.y = 0f;
        if (axis.sqrMagnitude < 1e-8f)
            return 0f;
        axis.Normalize();
        return Mathf.Atan2(axis.x, axis.z) * Mathf.Rad2Deg;
    }

    static float SafeRatio(float numerator, float denominator)
    {
        return Mathf.Abs(denominator) > 1e-6f ? numerator / denominator : 0f;
    }

    static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    static float SafeFinite(float value)
    {
        return IsFinite(value) ? value : -1f;
    }

    static float MinFinite(float a, float b, float c)
    {
        float result = float.PositiveInfinity;
        if (IsFinite(a)) result = Mathf.Min(result, a);
        if (IsFinite(b)) result = Mathf.Min(result, b);
        if (IsFinite(c)) result = Mathf.Min(result, c);
        return result;
    }

    static string F(float value)
    {
        return value.ToString("F6", CultureInfo.InvariantCulture);
    }

    static string FF(float value)
    {
        return SafeFinite(value).ToString("F6", CultureInfo.InvariantCulture);
    }

    static string Bool01(bool value)
    {
        return value ? "1" : "0";
    }

    static string EscapeCsv(string value)
    {
        if (value == null)
            return "";

        if (value.Contains(",") || value.Contains("\"") || value.Contains("\n") || value.Contains("\r"))
            return "\"" + value.Replace("\"", "\"\"") + "\"";
        return value;
    }

    static string SanitizeFileName(string value)
    {
        if (string.IsNullOrWhiteSpace(value))
            return "unnamed";

        StringBuilder result = new StringBuilder(value.Length);
        char[] invalid = Path.GetInvalidFileNameChars();

        foreach (char character in value.Trim())
        {
            bool isInvalid = Array.IndexOf(invalid, character) >= 0;
            result.Append(isInvalid || char.IsWhiteSpace(character) ? '_' : character);
        }

        return result.ToString();
    }

    struct Snapshot
    {
        public Vector3 leaderPosition;
        public Vector3 aPosition;
        public Vector3 bPosition;
        public Vector3 leaderVelocity;
        public Vector3 aVelocity;
        public Vector3 bVelocity;

        public Vector3 hapticForce;
        public Vector3 directionalHapticForce;
        public Vector3 dampingHapticForce;
        public Vector3 proximityHapticForce;
        public float proximityHapticRisk;
        public float proximityHapticClearance;
        public string fixtureObstacleName;
        public int fixtureDetected;
        public int fixtureRejectedFar;
        public int fixtureRejectedVelocity;
        public int fixtureConsidered;
        public Vector3 leaderApplied;
        public bool leaderActiveCBF;
        public float leaderClosingSpeed;
        public float leaderTTC;
        public float leaderBrakingDistance;
        public bool followerAccelerationRelaxed;
        public bool followerEmergencyStopped;
        public float followerResidualWithAcceleration;
        public bool followerRecoveryAttempted;
        public bool followerRecoverySucceeded;
        public float followerRecoveryResidual;
        public float hardPairMinimum;
        public float softSeparationCorrection;

        public float leaderMaxViolation;
        public float followerMaxViolation;
        public float leaderMinBarrier;
        public float followerMinBarrier;
        public float leaderObstacleDistance;
        public float aObstacleDistance;
        public float bObstacleDistance;
        public float distanceLA;
        public float distanceLB;
        public float distanceAB;
        public float formationRmse;
        public float formationMaxError;

        public float leaderMajor;
        public float leaderMinor;
        public float leaderRisk;
        public float aMajor;
        public float aMinor;
        public float aRisk;
        public float bMajor;
        public float bMinor;
        public float bRisk;

        public float leaderAxisDeg;
        public float aAxisDeg;
        public float bAxisDeg;
        public float leaderDensity;
        public float aDensity;
        public float bDensity;
        public float leaderMinScanDistance;
        public float aMinScanDistance;
        public float bMinScanDistance;
        public float leaderGapClearance;
        public float aGapClearance;
        public float bGapClearance;

        public Vector3 leaderNominal;
        public Vector3 leaderSafe;
        public Vector3 leaderCorrection;
        public float leaderCbfRisk;
        public int leaderConstraints;
        public bool leaderFeasible;

        public Vector3 aNominal;
        public Vector3 bNominal;
        public Vector3 aSafe;
        public Vector3 bSafe;
        public Vector3 aCorrection;
        public Vector3 bCorrection;
        public float followerPairDistanceDebug;
        public int followerTotalConstraints;
        public bool followerFeasible;

        public float kA;
        public float dA;
        public float kB;
        public float dB;
    }
}

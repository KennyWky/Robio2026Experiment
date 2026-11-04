using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Estimates one smooth, rotating, shape-adaptive safety ellipse for each rigidbody.
/// The ellipse is driven by local 360-degree free-space geometry, speed, and density.
/// </summary>
public class AdaptiveEllipseEstimator : MonoBehaviour
{
    [Header("Gap Direction Selection")]
    [Tooltip("Minimum clearance is evaluated over this angular window, not along a single ray.")]
    public float gapWindowHalfAngle = 15f;

    public float goalDirectionWeight = 1.2f;
    public float clearanceWeight = 2.0f;
    public float continuityWeight = 0.8f;
    public float reverseDirectionPenalty = 0.8f;

    [Header("Ellipse Semi-Axes")]
    [Tooltip("Base semi-axis along the selected free-space direction.")]
    public float baseMajorAxis = 0.75f;

    [Tooltip("Base semi-axis perpendicular to the selected free-space direction.")]
    public float baseMinorAxis = 0.65f;

    public float minMajorAxis = 0.55f;
    public float maxMajorAxis = 1.80f;
    public float minMinorAxis = 0.50f; // legacy lower bound when physical-adaptation is disabled

    [Header("Physical-Clearance Adaptive Minor Axis")]
    [Tooltip("ON: adaptive M2/M4 shrink the lateral semi-axis towards actual body radius + safety margin, bypassing legacy 0.50m Min Minor Axis. Fixed-circle M1/M3 remain unchanged.")]
    public bool usePhysicalClearanceFloor = true;
    public float physicalPlanarRadiusFallback = 0.15556f;
    [Tooltip("Added margin outside UAV planar bounding radius, in meters. 0.08 -> min axis about 0.236m for 0.22m square UAV.")]
    public float physicalSurfaceMargin = 0.08f;
    [Tooltip("Small absolute lower bound prevents near-zero ellipse sizes.")]
    public float absoluteMinorAxisFloor = 0.22f;
    [Tooltip("Allow lateral ellipse to contract faster than it expands in narrow passages.")]
    public float minorContractionRatePerSec = 4.0f;
    public float maxMinorAxis = 1.20f;

    [Header("Speed Adaptation")]
    [Tooltip("Linear look-ahead contribution along the major axis.")]
    public float speedLookAheadTime = 0.18f;

    [Tooltip("Used in v^2/(2a) braking-distance adaptation.")]
    public float assumedBrakingAcceleration = 4.0f;

    [Header("Density and Clearance Adaptation")]
    [Tooltip("Small conservative margin added in dense areas.")]
    public float densityMarginGain = 0.18f;

    [Tooltip("Fraction of measured side clearance allowed for the minor semi-axis.")]
    [Range(0.2f, 0.95f)]
    public float lateralClearanceUsage = 0.78f;

    public float lateralWindowHalfAngle = 12f;

    [Header("Risk")]
    public float nearObstacleRiskDistance = 1.5f;
    public float lateralRiskDistance = 1.2f;

    [Header("Temporal Smoothing")]
    public float directionSmoothTau = 0.18f;
    public float sizeSmoothTau = 0.15f;
    public float maxDirectionRateDegPerSec = 160f;
    public float maxAxisRatePerSec = 2.0f;

    readonly Dictionary<int, EllipseMemory> memoryByBody = new Dictionary<int, EllipseMemory>();

    public AdaptiveEllipseState UpdateEllipse(
        Rigidbody body,
        Vector3 motionVelocity,
        Vector3 preferredDirection,
        FreeSpaceScan scan,
        float dt
    )
    {
        AdaptiveEllipseState empty = AdaptiveEllipseState.CreateDefault(body);

        if (body == null || scan == null || !scan.Initialized)
            return empty;

        int key = body.GetInstanceID();
        EllipseMemory memory;

        if (!memoryByBody.TryGetValue(key, out memory))
        {
            memory = new EllipseMemory();
            memoryByBody[key] = memory;
        }

        Vector3 preferred = preferredDirection;
        preferred.y = 0f;

        if (preferred.sqrMagnitude < 1e-8f)
        {
            preferred = motionVelocity;
            preferred.y = 0f;
        }

        if (preferred.sqrMagnitude < 1e-8f)
            preferred = Vector3.forward;

        preferred.Normalize();

        Vector3 previousAxis = memory.Initialized ? memory.Axis : preferred;
        previousAxis.y = 0f;
        if (previousAxis.sqrMagnitude < 1e-8f)
            previousAxis = preferred;
        previousAxis.Normalize();

        Vector3 bestDirection = preferred;
        float bestClearance = scan.WindowMinimum(preferred, gapWindowHalfAngle);
        float bestScore = float.NegativeInfinity;

        for (int i = 0; i < scan.RayCount; i++)
        {
            Vector3 candidate = scan.Directions[i];
            candidate.y = 0f;
            if (candidate.sqrMagnitude < 1e-8f)
                continue;
            candidate.Normalize();

            float windowClearance = scan.WindowMinimum(candidate, gapWindowHalfAngle);
            float clearanceScore = Mathf.Clamp01(windowClearance / Mathf.Max(0.05f, scan.MaxDistance));

            float signedGoalAlignment = Vector3.Dot(candidate, preferred);
            float goalScore = 0.5f * (signedGoalAlignment + 1f);

            float axisContinuity = Mathf.Abs(Vector3.Dot(candidate, previousAxis));
            float reversePenalty = signedGoalAlignment < 0f
                ? reverseDirectionPenalty * (-signedGoalAlignment)
                : 0f;

            float score =
                clearanceWeight * clearanceScore
                + goalDirectionWeight * goalScore
                + continuityWeight * axisContinuity
                - reversePenalty;

            if (score > bestScore)
            {
                bestScore = score;
                bestDirection = candidate;
                bestClearance = windowClearance;
            }
        }

        // Ellipse orientation is sign-invariant. Avoid numerical 180-degree flips.
        if (Vector3.Dot(bestDirection, previousAxis) < 0f)
            bestDirection = -bestDirection;

        float safeDt = Mathf.Max(1e-5f, dt);
        float directionAlpha = directionSmoothTau <= 1e-6f
            ? 1f
            : 1f - Mathf.Exp(-safeDt / directionSmoothTau);

        Vector3 filteredTarget = Vector3.Slerp(previousAxis, bestDirection, directionAlpha);
        Vector3 axis = Vector3.RotateTowards(
            previousAxis,
            filteredTarget,
            Mathf.Deg2Rad * Mathf.Max(0f, maxDirectionRateDegPerSec) * safeDt,
            0f
        );

        axis.y = 0f;
        if (axis.sqrMagnitude < 1e-8f)
            axis = bestDirection;
        axis.Normalize();

        Vector3 lateral = Vector3.Cross(Vector3.up, axis).normalized;

        float leftClearance = scan.WindowMinimum(-lateral, lateralWindowHalfAngle);
        float rightClearance = scan.WindowMinimum(lateral, lateralWindowHalfAngle);
        float sideClearance = Mathf.Min(leftClearance, rightClearance);

        Vector3 planarVelocity = motionVelocity;
        planarVelocity.y = 0f;
        float speedAlongAxis = Mathf.Abs(Vector3.Dot(planarVelocity, axis));

        float brakingDistance = speedAlongAxis * speedAlongAxis
            / (2f * Mathf.Max(0.05f, assumedBrakingAcceleration));

        float densityMargin = densityMarginGain * scan.DensityScore;

        float targetMajor = baseMajorAxis
            + speedLookAheadTime * speedAlongAxis
            + brakingDistance
            + densityMargin;

        targetMajor = Mathf.Clamp(targetMajor, minMajorAxis, maxMajorAxis);

        float desiredMinor = baseMinorAxis + 0.5f * densityMargin;
        float clearanceLimitedMinor = sideClearance * lateralClearanceUsage;
        float targetMinor = Mathf.Min(desiredMinor, clearanceLimitedMinor);
        float physicalFloor = usePhysicalClearanceFloor
            ? Mathf.Max(absoluteMinorAxisFloor, Mathf.Max(physicalPlanarRadiusFallback, PhysicalPlanarRadius(body)) + Mathf.Max(0f,physicalSurfaceMargin))
            : minMinorAxis;
        // A physical shell remains even in corridors narrower than the shell.
        // If such a corridor is genuinely too narrow, the CBF must reject it.
        targetMinor = Mathf.Clamp(targetMinor, physicalFloor, Mathf.Max(physicalFloor,maxMinorAxis));

        // Keep the named major axis from becoming shorter than the minor axis.
        targetMajor = Mathf.Max(targetMajor, targetMinor);

        float previousMajor = memory.Initialized ? memory.MajorAxis : targetMajor;
        float previousMinor = memory.Initialized ? memory.MinorAxis : targetMinor;

        float sizeAlpha = sizeSmoothTau <= 1e-6f
            ? 1f
            : 1f - Mathf.Exp(-safeDt / sizeSmoothTau);

        float filteredMajor = Mathf.Lerp(previousMajor, targetMajor, sizeAlpha);
        float filteredMinor = Mathf.Lerp(previousMinor, targetMinor, sizeAlpha);

        float maxAxisStep = Mathf.Max(0f, maxAxisRatePerSec) * safeDt;
        float major = Mathf.MoveTowards(previousMajor, filteredMajor, maxAxisStep);
        float minorStep = targetMinor < previousMinor
            ? Mathf.Max(maxAxisStep, Mathf.Max(0f,minorContractionRatePerSec)*safeDt)
            : maxAxisStep;
        float minor = Mathf.MoveTowards(previousMinor, filteredMinor, minorStep);

        major = Mathf.Clamp(major, minMajorAxis, maxMajorAxis);
        minor = Mathf.Clamp(minor, physicalFloor, Mathf.Max(physicalFloor,maxMinorAxis));
        major = Mathf.Max(major, minor);

        PlanarMatrix2x2 q = PlanarMatrix2x2.FromEllipse(axis, major, minor);
        PlanarMatrix2x2 qDot = memory.Initialized
            ? (q - memory.Q) / safeDt
            : PlanarMatrix2x2.Zero;

        float nearRisk = 1f - Mathf.Clamp01(
            scan.MinDistance / Mathf.Max(0.05f, nearObstacleRiskDistance)
        );

        float lateralRisk = 1f - Mathf.Clamp01(
            sideClearance / Mathf.Max(0.05f, lateralRiskDistance)
        );

        float risk = Mathf.Clamp01(Mathf.Max(scan.DensityScore, Mathf.Max(nearRisk, lateralRisk)));
        risk = risk * risk * (3f - 2f * risk);

        AdaptiveEllipseState state = new AdaptiveEllipseState();
        state.Valid = true;
        state.Body = body;
        state.Center = body.worldCenterOfMass;
        state.Axis = axis;
        state.LateralAxis = lateral;
        state.GapDirection = bestDirection;
        state.MajorAxis = major;
        state.MinorAxis = minor;
        state.Q = q;
        state.QDot = qDot;
        state.DensityScore = scan.DensityScore;
        state.EnvironmentRisk = risk;
        state.MinScanDistance = scan.MinDistance;
        state.SelectedGapClearance = bestClearance;
        state.LeftClearance = leftClearance;
        state.RightClearance = rightClearance;

        memory.Initialized = true;
        memory.Axis = axis;
        memory.MajorAxis = major;
        memory.MinorAxis = minor;
        memory.Q = q;
        memory.LastState = state;

        return state;
    }

    static float PhysicalPlanarRadius(Rigidbody body)
    {
        // Planar AABB circle of each non-trigger collider, including its offset.
        // AABB changes with world rotation; frozen UAV rotation keeps it stable.
        const float fallback = 0.15556f;
        if (body == null) return fallback;
        float radius = fallback;
        Collider[] colliders = body.GetComponentsInChildren<Collider>();
        foreach (Collider c in colliders)
        {
            if (c == null || c.isTrigger || c.attachedRigidbody != body) continue;
            Vector3 offset = c.bounds.center - body.worldCenterOfMass;
            Vector3 e = c.bounds.extents;
            float r = new Vector2(offset.x,offset.z).magnitude +
                Mathf.Sqrt(e.x*e.x + e.z*e.z);
            radius = Mathf.Max(radius,r);
        }
        return radius;
    }

    // Call between trials to avoid previous-trial axis / QDot influencing a new run.
    public void ResetEstimatorMemory()
    {
        memoryByBody.Clear();
    }

    public AdaptiveEllipseState GetLastState(Rigidbody body)
    {
        if (body == null)
            return AdaptiveEllipseState.CreateDefault(null);

        EllipseMemory memory;
        if (memoryByBody.TryGetValue(body.GetInstanceID(), out memory) && memory.Initialized)
            return memory.LastState;

        return AdaptiveEllipseState.CreateDefault(body);
    }

    sealed class EllipseMemory
    {
        public bool Initialized;
        public Vector3 Axis;
        public float MajorAxis;
        public float MinorAxis;
        public PlanarMatrix2x2 Q;
        public AdaptiveEllipseState LastState;
    }
}

public struct AdaptiveEllipseState
{
    public bool Valid;
    public Rigidbody Body;
    public Vector3 Center;
    public Vector3 Axis;
    public Vector3 LateralAxis;
    public Vector3 GapDirection;
    public float MajorAxis;
    public float MinorAxis;
    public PlanarMatrix2x2 Q;
    public PlanarMatrix2x2 QDot;
    public float DensityScore;
    public float EnvironmentRisk;
    public float MinScanDistance;
    public float SelectedGapClearance;
    public float LeftClearance;
    public float RightClearance;

    public static AdaptiveEllipseState CreateDefault(Rigidbody body)
    {
        AdaptiveEllipseState state = new AdaptiveEllipseState();
        state.Valid = false;
        state.Body = body;
        state.Center = body != null ? body.worldCenterOfMass : Vector3.zero;
        state.Axis = Vector3.forward;
        state.LateralAxis = Vector3.right;
        state.GapDirection = Vector3.forward;
        state.MajorAxis = 0.7f;
        state.MinorAxis = 0.7f;
        state.Q = PlanarMatrix2x2.Identity;
        state.QDot = PlanarMatrix2x2.Zero;
        return state;
    }
}

/// <summary>
/// Compact symmetric 2x2 matrix for Unity's X-Z plane.
/// </summary>
public struct PlanarMatrix2x2
{
    public float m00;
    public float m01;
    public float m10;
    public float m11;

    public static PlanarMatrix2x2 Zero
    {
        get { return new PlanarMatrix2x2(); }
    }

    public static PlanarMatrix2x2 Identity
    {
        get
        {
            PlanarMatrix2x2 m = new PlanarMatrix2x2();
            m.m00 = 1f;
            m.m11 = 1f;
            return m;
        }
    }

    public Vector2 Multiply(Vector2 v)
    {
        return new Vector2(
            m00 * v.x + m01 * v.y,
            m10 * v.x + m11 * v.y
        );
    }

    public float Quadratic(Vector2 v)
    {
        Vector2 mv = Multiply(v);
        return Vector2.Dot(v, mv);
    }

    public static PlanarMatrix2x2 FromEllipse(Vector3 axis3, float major, float minor)
    {
        Vector2 u = new Vector2(axis3.x, axis3.z);
        if (u.sqrMagnitude < 1e-8f)
            u = Vector2.up;
        u.Normalize();

        Vector2 n = new Vector2(-u.y, u.x);
        float invA2 = 1f / Mathf.Max(1e-6f, major * major);
        float invB2 = 1f / Mathf.Max(1e-6f, minor * minor);

        PlanarMatrix2x2 q = new PlanarMatrix2x2();
        q.m00 = invA2 * u.x * u.x + invB2 * n.x * n.x;
        q.m01 = invA2 * u.x * u.y + invB2 * n.x * n.y;
        q.m10 = q.m01;
        q.m11 = invA2 * u.y * u.y + invB2 * n.y * n.y;
        return q;
    }

    public static PlanarMatrix2x2 operator -(PlanarMatrix2x2 a, PlanarMatrix2x2 b)
    {
        PlanarMatrix2x2 m = new PlanarMatrix2x2();
        m.m00 = a.m00 - b.m00;
        m.m01 = a.m01 - b.m01;
        m.m10 = a.m10 - b.m10;
        m.m11 = a.m11 - b.m11;
        return m;
    }

    public static PlanarMatrix2x2 operator /(PlanarMatrix2x2 a, float s)
    {
        float inv = 1f / Mathf.Max(1e-8f, Mathf.Abs(s));
        if (s < 0f) inv = -inv;

        PlanarMatrix2x2 m = new PlanarMatrix2x2();
        m.m00 = a.m00 * inv;
        m.m01 = a.m01 * inv;
        m.m10 = a.m10 * inv;
        m.m11 = a.m11 * inv;
        return m;
    }
}

using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Self-contained approximate joint CBF-QP solver for two followers.
/// It projects the nominal velocity onto the intersection of:
/// 1) each follower's shape-adaptive obstacle CBF half-spaces,
/// 2) follower-follower and leader-follower separation CBFs,
/// 3) speed and acceleration limits.
///
/// The solver uses repeated convex projections, so it requires no external QP package.
/// For a formal publication-grade QP implementation, this class can later be replaced
/// by OSQP/qpOASES while keeping the same public interface.
/// </summary>
public class JointFollowerCBFSolver : MonoBehaviour
{
    [Header("Obstacle CBF")]
    public LayerMask obstacleMask;
    public float obstacleQueryRadius = 4.0f;
    public float obstacleActivationScale = 2.4f;
    public float obstacleCBFAlpha = 3.0f;

    [Tooltip("Limits the e^T Qdot e contribution to avoid violent commands when the ellipse changes rapidly.")]
    public float maxShapeRateTerm = 4.0f;

    [Header("Inter-Agent CBF")]
    public float minimumFollowerDistance = 1.0f;
    public float followerPairActivationDistance = 2.0f;
    public float followerPairCBFAlpha = 4.0f;

    [Header("Two-Tier Separation: Hard Physical vs Soft Preferred")]
    [Tooltip("Preferred A-B spacing (m); no longer treated as an inviolable CBF boundary. Existing minimumFollowerDistance is used as preferred spacing.")]
    public bool useSoftSeparationGuidance = true;
    [Tooltip("Extra planar clearance between two UAV physical circumscribed disks (m).")]
    public float interAgentPhysicalClearance = 0.08f;
    public float physicalRadiusFallback = 0.15556f;
    [Tooltip("Additional absolute floor for the A-B HARD physical separation (m).")]
    public float hardFollowerDistanceFloor = 0.40f;
    [Tooltip("Additional absolute floor for leader-follower HARD physical separation (m).")]
    public float hardLeaderFollowerDistanceFloor = 0.40f;
    [Tooltip("Soft preferred spacing restoring velocity gain (1/s); not a hard CBF constraint.")]
    public float softPairSeparationGain = 1.0f;
    public float maxSoftSeparationCorrection = 1.0f;
    [Header("Recovery Re-solve")]
    [Tooltip("When normal speed-bounded solve fails, retry after building a direction away from currently violated hard constraints. Recovery is used only when the returned velocity passes ALL hard-constraint checks.")]
    public bool enableConstraintRecovery = true;
    [Range(1, 12)] public int recoveryEscapeProjectionPasses = 5;
    public float recoveryProjectionMargin = 0.02f;
    readonly Dictionary<int,float> physicalRadiusCache = new Dictionary<int,float>();

    public bool protectLeaderFollowerDistance = true;
    public float minimumLeaderFollowerDistance = 0.9f;
    public float leaderFollowerActivationDistance = 1.8f;
    public float leaderFollowerCBFAlpha = 4.0f;

    [Header("Projection Solver")]
    [Range(10, 300)]
    public int projectionIterations = 160;

    [Range(0.1f, 1.0f)]
    [Tooltip("Legacy serialized setting; Dykstra always uses exact half-space projections. This value is ignored.")]
    public float projectionRelaxation = 0.95f;

    [Tooltip("Adds temporal regularization. 0 tracks nominal velocity exactly; larger values prefer the previous safe velocity.")]
    public float temporalVelocityWeight = 0.25f;

    public float maxPlanarSpeed = 5.0f;
    public float maxPlanarAcceleration = 10.0f;
    public float feasibilityTolerance = 0.02f;
    [Tooltip("First solve with acceleration limits, then remove ONLY the acceleration limit if it conflicts with safety. Log every relaxation.")]
    public bool relaxAccelerationForSafety = true;
    [Tooltip("If even the speed-bounded CBF problem is unresolved/infeasible, return a zero-planar-velocity emergency command. A stop is NOT a safety certificate.")]
    public bool emergencyStopOnFailure = true;


    [Header("Closest-Point Derivative: Finite-Difference Surface Jacobian")]
    [Tooltip("LEGACY ignored: old static-obstacle closest-point time derivative was not control-affine. Kept to preserve Inspector serialization.")]
    public bool estimateClosestPointMotion = false;
    [Tooltip("LEGACY ignored: retained for old Inspector serialization.")]
    public float closestPointSmoothTau = 0.05f;
    [Tooltip("Max estimated translation speed for obstacles moved through Transform without Rigidbody (m/s).")]
    public float maxClosestPointSpeed = 8.0f;
    [Tooltip("Compute d(closestPoint)/d(UAV position) with two extra Collider.ClosestPoint calls per obstacle. Removes the misleading static-obstacle qdot approximation.")]
    public bool useSurfaceJacobian = true;
    [Range(0.005f, 0.15f)] public float surfaceJacobianStep = 0.03f;
    public float maxSurfaceJacobianColumnNorm = 2.0f;

    struct ObstacleMotionTrack { public Vector2 center; public Vector2 velocity; public float time; }
    readonly Dictionary<int, ObstacleMotionTrack> obstacleMotion = new Dictionary<int, ObstacleMotionTrack>();
    readonly Vector4State[] dykstraCorrections = new Vector4State[constraintsCapacity];
    const int constraintsCapacity = 260;

    [Header("Infeasibility Policy")]
    [Tooltip("BestEffort returns the final projected velocities (not guaranteed safe). StopFollowers commands zero velocity on residual violation (also not guaranteed safe with moving obstacles).")]
    public InfeasibilityPolicy infeasibilityPolicy = InfeasibilityPolicy.BestEffort;
    public enum InfeasibilityPolicy { BestEffort, StopFollowers }

    [Header("Debug")]
    public bool drawConstraintNormals = false;
    public float debugNormalScale = 0.35f;

    readonly Collider[] colliderHits = new Collider[192];
    readonly LinearConstraint4[] constraints = new LinearConstraint4[256];
    readonly Dictionary<int, Vector2> previousSingleSafe = new Dictionary<int, Vector2>();
    // q(p,t) depends on the moving query point p. For a static obstacle, qdot is
    // NOT an exogenous velocity. Use (I - D_p q)^T grad for the control coefficient.
    // Translation of the obstacle itself goes into the RHS separately.
    Vector2 EffectiveObstacleGradient(Collider col, Vector3 p, Vector2 grad)
    {
        if (!useSurfaceJacobian) return grad;
        float eps = Mathf.Max(0.005f, surfaceJacobianStep);
        Vector3 px = p + new Vector3(eps, 0f, 0f);
        Vector3 pz = p + new Vector3(0f, 0f, eps);
        Vector2 q0 = ToPlanar(col.ClosestPoint(p));
        Vector2 qx = ToPlanar(col.ClosestPoint(px));
        Vector2 qz = ToPlanar(col.ClosestPoint(pz));
        Vector2 jx = (qx - q0) / eps;
        Vector2 jz = (qz - q0) / eps;
        float clamp = Mathf.Max(0.1f, maxSurfaceJacobianColumnNorm);
        jx = Vector2.ClampMagnitude(jx, clamp);
        jz = Vector2.ClampMagnitude(jz, clamp);
        return new Vector2(
            grad.x - Vector2.Dot(jx, grad),
            grad.y - Vector2.Dot(jz, grad)
        );
    }

    Vector2 ObstacleSurfaceVelocity(Collider col, Vector3 closestWorld)
    {
        Rigidbody other = col.attachedRigidbody;
        if (other != null) return ToPlanar(other.GetPointVelocity(closestWorld));

        // For transform-driven obstacles WITHOUT a Rigidbody: translation estimate.
        // Rotations and deformations remain approximations, not guarantees.
        int id = col.GetInstanceID();
        float now = Time.fixedTime;
        Vector2 center = ToPlanar(col.bounds.center);
        ObstacleMotionTrack track;
        Vector2 estimated = Vector2.zero;
        if (obstacleMotion.TryGetValue(id, out track))
        {
            estimated = track.velocity;
            if (now > track.time + 1e-5f)
            {
                estimated = (center - track.center) / (now - track.time);
                estimated = Vector2.ClampMagnitude(estimated, Mathf.Max(0.1f, maxClosestPointSpeed));
            }
        }
        track.center = center;
        track.velocity = estimated;
        track.time = now;
        obstacleMotion[id] = track;
        return estimated;
    }

    bool hasPreviousJoint;
    Vector2 previousJointA;
    Vector2 previousJointB;

    public SingleAgentCBFResult EvaluateSingleAgent(
        Rigidbody body,
        Vector3 nominalVelocity,
        AdaptiveEllipseState ellipse,
        float dt
    )
    {
        SingleAgentCBFResult result = new SingleAgentCBFResult();
        result.NominalVelocity = nominalVelocity;
        result.SafeVelocity = nominalVelocity;
        result.Correction = Vector3.zero;
        result.MinBarrier = float.PositiveInfinity;
        result.ApproximatelyFeasible = true;

        if (body == null || !ellipse.Valid)
            return result;

        int count = 0;
        RiskAccumulator risk = new RiskAccumulator();
        risk.NoRiskProduct = 1f;
        risk.MinBarrier = float.PositiveInfinity;

        count = AddObstacleConstraints(
            body,
            ellipse,
            0,
            constraints,
            count,
            ref risk
        );

        int bodyId = body.GetInstanceID();
        Vector2 nominal = ToPlanar(nominalVelocity);

        // The leader's single-agent solver is used as a shadow CBF for haptic
        // guidance. In free space, its output must exactly equal the operator's
        // nominal command. Otherwise temporal regularization or acceleration
        // limiting is incorrectly perceived as an obstacle force.
        if (count == 0)
        {
            previousSingleSafe[bodyId] = nominal;
            result.SafeVelocity = nominalVelocity;
            result.Correction = Vector3.zero;
            result.ActiveObstacleConstraints = 0;
            result.MaxRisk = 0f;
            result.AggregateRisk = 0f;
            result.MinBarrier = float.PositiveInfinity;
            result.MaxConstraintViolation = 0f;
            result.ApproximatelyFeasible = true;
            return result;
        }

        // Start directly from the nominal command. Temporal regularization and
        // acceleration limits remain enabled for the joint follower controller,
        // but are intentionally excluded from the leader haptic preview so that
        // Correction represents only active CBF obstacle constraints.
        Vector4State z = new Vector4State(nominal.x, nominal.y, 0f, 0f);

        int iterations = Mathf.Max(2, projectionIterations);
        for (int iter = 0; iter < iterations; iter++)
        {
            for (int i = 0; i < count; i++)
                ProjectHalfSpace(ref z, constraints[i]);

            Vector2 va = new Vector2(z.x0, z.x1);
            va = Vector2.ClampMagnitude(
                va,
                Mathf.Max(0.01f, maxPlanarSpeed)
            );
            z.x0 = va.x;
            z.x1 = va.y;
        }

        float maxViolation = ComputeMaxViolation(z, count);
        Vector2 safe = new Vector2(z.x0, z.x1);
        previousSingleSafe[bodyId] = safe;

        result.SafeVelocity = FromPlanar(safe, nominalVelocity.y);
        result.Correction = result.SafeVelocity - nominalVelocity;
        result.ActiveObstacleConstraints = count;
        result.MaxRisk = risk.MaxRisk;
        result.AggregateRisk = 1f - Mathf.Clamp01(risk.NoRiskProduct);
        result.MinBarrier = risk.MinBarrier;
        result.MaxConstraintViolation = maxViolation;
        result.ApproximatelyFeasible = maxViolation <= feasibilityTolerance;
        return result;
    }

    public JointFollowerCBFResult SolveFollowers(
        Rigidbody followerA,
        Rigidbody followerB,
        Rigidbody leader,
        Vector3 nominalVelocityA,
        Vector3 nominalVelocityB,
        AdaptiveEllipseState ellipseA,
        AdaptiveEllipseState ellipseB,
        float dt
    )
    {
        JointFollowerCBFResult result = new JointFollowerCBFResult();
        result.NominalVelocityA = nominalVelocityA;
        result.NominalVelocityB = nominalVelocityB;
        result.SafeVelocityA = nominalVelocityA;
        result.SafeVelocityB = nominalVelocityB;
        result.MinBarrier = float.PositiveInfinity;
        result.MinimumPairDistance = followerA != null && followerB != null
            ? PlanarDistance(followerA.position, followerB.position)
            : float.PositiveInfinity;
        result.ApproximatelyFeasible = true;

        if (followerA == null || followerB == null || !ellipseA.Valid || !ellipseB.Valid)
            return result;

        int count = 0;
        RiskAccumulator risk = new RiskAccumulator();
        risk.NoRiskProduct = 1f;
        risk.MinBarrier = float.PositiveInfinity;

        int beforeA = count;
        count = AddObstacleConstraints(followerA, ellipseA, 0, constraints, count, ref risk);
        result.ObstacleConstraintsA = count - beforeA;

        int beforeB = count;
        count = AddObstacleConstraints(followerB, ellipseB, 2, constraints, count, ref risk);
        result.ObstacleConstraintsB = count - beforeB;

        bool pairAdded = AddFollowerPairConstraint(
            followerA,
            followerB,
            constraints,
            ref count,
            ref risk
        );
        result.FollowerPairConstraintActive = pairAdded;

        int leaderPairCount = 0;
        if (protectLeaderFollowerDistance && leader != null)
        {
            if (AddLeaderFollowerConstraint(followerA, leader, 0, constraints, ref count, ref risk))
                leaderPairCount++;
            if (AddLeaderFollowerConstraint(followerB, leader, 2, constraints, ref count, ref risk))
                leaderPairCount++;
        }
        result.LeaderFollowerConstraints = leaderPairCount;

        Vector2 nominalA = ToPlanar(nominalVelocityA);
        Vector2 nominalB = ToPlanar(nominalVelocityB);
        result.HardPairMinimumDistance = HardPairDistance(followerA, followerB, false);
        // Soft desired distance creates a nominal repulsion, never a hard CBF infeasibility.
        Vector2 softA = Vector2.zero;
        Vector2 softB = Vector2.zero;
        if (useSoftSeparationGuidance)
        {
            Vector2 pa = ToPlanar(followerA.worldCenterOfMass);
            Vector2 pb = ToPlanar(followerB.worldCenterOfMass);
            Vector2 sep = pa - pb;
            float dist = sep.magnitude;
            if (dist > 0.0001f && dist < minimumFollowerDistance)
            {
                Vector2 correction = sep / dist * Mathf.Min(maxSoftSeparationCorrection,
                    softPairSeparationGain * (minimumFollowerDistance - dist));
                softA += 0.5f * correction;
                softB -= 0.5f * correction;
            }
            if (leader != null && protectLeaderFollowerDistance)
            {
                Vector2 pl = ToPlanar(leader.worldCenterOfMass);
                SoftLeaderDistance(ref softA, pa, pl);
                SoftLeaderDistance(ref softB, pb, pl);
            }
            nominalA = Vector2.ClampMagnitude(nominalA + softA, maxPlanarSpeed);
            nominalB = Vector2.ClampMagnitude(nominalB + softB, maxPlanarSpeed);
        }
        result.SoftSeparationCorrectionMagnitude = Mathf.Max(softA.magnitude,softB.magnitude);

        Vector2 previousA = hasPreviousJoint ? previousJointA : nominalA;
        Vector2 previousB = hasPreviousJoint ? previousJointB : nominalB;

        Vector2 referenceA = hasPreviousJoint
            ? WeightedReference(nominalA, previousA, temporalVelocityWeight)
            : nominalA;
        Vector2 referenceB = hasPreviousJoint
            ? WeightedReference(nominalB, previousB, temporalVelocityWeight)
            : nominalB;

        Vector4State z = new Vector4State(
            referenceA.x,
            referenceA.y,
            referenceB.x,
            referenceB.y
        );

        // First attempt: respect CBF halfspaces, speed and acceleration balls.
        // Dykstra's method performs true projections onto convex sets, with dual
        // corrections; unlike the previous cyclic projection it does not blindly
        // reapply acceleration at the end without correcting the CBF violation.
        float constrainedResidual = DykstraSolve(
            ref z, count, previousA, previousB, hasPreviousJoint,
            dt, true);
        result.ResidualWithAcceleration = constrainedResidual;
        bool relaxed = false;
        bool emergencyStop = false;
        float maxViolation = constrainedResidual;

        if (maxViolation > feasibilityTolerance && relaxAccelerationForSafety)
        {
            // Safety-first stage: speed stays hard; temporal acceleration is
            // relaxed because Unity can directly command planar Rigidbody speed.
            // This is NOT applicable to physical drones without a matching model.
            relaxed = true;
            z = new Vector4State(referenceA.x, referenceA.y, referenceB.x, referenceB.y);
            maxViolation = DykstraSolve(
                ref z, count, previousA, previousB, hasPreviousJoint,
                dt, false);
        }

        bool approximatelyFeasible = maxViolation <= feasibilityTolerance;
        bool recoveryAttempted = false;
        bool recoverySucceeded = false;
        float recoveryResidual = maxViolation;
        if (!approximatelyFeasible && enableConstraintRecovery)
        {
            recoveryAttempted = true;
            Vector4State escapeStart = new Vector4State(0f, 0f, 0f, 0f);
            // A reproducible hard-CBF escape seed: iteratively project *towards*
            // satisfying the most violated hard halfspaces. Never issue this seed
            // directly; it must first pass the same speed-bounded CBF solve/check.
            for (int pass = 0; pass < Mathf.Clamp(recoveryEscapeProjectionPasses,1,12); pass++)
            {
                for (int k = 0; k < count; k++)
                {
                    LinearConstraint4 c = constraints[k];
                    float deficit = c.B - Vector4State.Dot(c.A, escapeStart);
                    if (deficit > 0f && c.A.SqrMagnitude > 1e-9f)
                        escapeStart += c.A * ((deficit + recoveryProjectionMargin) / c.A.SqrMagnitude);
                }
            }
            Vector2 av = Vector2.ClampMagnitude(new Vector2(escapeStart.x0,escapeStart.x1), maxPlanarSpeed);
            Vector2 bv = Vector2.ClampMagnitude(new Vector2(escapeStart.x2,escapeStart.x3), maxPlanarSpeed);
            Vector4State recovered = new Vector4State(av.x,av.y,bv.x,bv.y);
            recoveryResidual = DykstraSolve(ref recovered, count, previousA,previousB,
                hasPreviousJoint,dt,false);
            if (recoveryResidual <= feasibilityTolerance)
            {
                z = recovered;
                maxViolation = recoveryResidual;
                approximatelyFeasible = true;
                recoverySucceeded = true;
            }
        }
        result.RecoveryAttempted = recoveryAttempted;
        result.RecoverySucceeded = recoverySucceeded;
        result.RecoveryResidual = recoveryResidual;
        if (!approximatelyFeasible &&
            (emergencyStopOnFailure || infeasibilityPolicy == InfeasibilityPolicy.StopFollowers))
        {
            emergencyStop = true;
            z = new Vector4State(0f, 0f, 0f, 0f);
            // Record the residual of the ACTUALLY issued stop command.
            maxViolation = ComputeMaxViolation(z, count);
            // A valid stop command does NOT mean the failed optimization
            // suddenly converged. Keep solver-feasible false in the logs.
            approximatelyFeasible = false;
        }
        result.AccelerationRelaxed = relaxed;
        result.EmergencyStopped = emergencyStop;

        Vector2 safeA = new Vector2(z.x0, z.x1);
        Vector2 safeB = new Vector2(z.x2, z.x3);

        previousJointA = safeA;
        previousJointB = safeB;
        hasPreviousJoint = true;

        result.SafeVelocityA = FromPlanar(safeA, nominalVelocityA.y);
        result.SafeVelocityB = FromPlanar(safeB, nominalVelocityB.y);
        result.CorrectionA = result.SafeVelocityA - nominalVelocityA;
        result.CorrectionB = result.SafeVelocityB - nominalVelocityB;
        result.TotalConstraints = count;
        result.MaxRisk = risk.MaxRisk;
        result.AggregateRisk = 1f - Mathf.Clamp01(risk.NoRiskProduct);
        result.MinBarrier = risk.MinBarrier;
        result.MaxConstraintViolation = maxViolation;
        result.ApproximatelyFeasible = approximatelyFeasible;
        result.MinimumPairDistance = PlanarDistance(followerA.position, followerB.position);

        return result;
    }

    public void ResetSolverMemory()
    {
        previousSingleSafe.Clear();
        obstacleMotion.Clear();
        physicalRadiusCache.Clear();
        hasPreviousJoint = false;
        previousJointA = Vector2.zero;
        previousJointB = Vector2.zero;
    }

    int AddObstacleConstraints(
        Rigidbody body,
        AdaptiveEllipseState ellipse,
        int variableOffset,
        LinearConstraint4[] target,
        int count,
        ref RiskAccumulator risk
    )
    {
        Vector3 center3 = body.worldCenterOfMass;
        int hitCount = Physics.OverlapSphereNonAlloc(
            center3,
            obstacleQueryRadius,
            colliderHits,
            obstacleMask,
            QueryTriggerInteraction.Ignore
        );

        Vector2 p = ToPlanar(center3);

        for (int i = 0; i < hitCount && count < target.Length; i++)
        {
            Collider col = colliderHits[i];
            if (ShouldIgnoreCollider(col, body))
                continue;

            Vector3 closest3 = col.ClosestPoint(center3);
            Vector2 q = ToPlanar(closest3);
            Vector2 e = p - q;

            // This solver protects the horizontal X-Z plane. Ignore floor/ceiling-only
            // contacts whose closest point differs only in height.
            if (e.sqrMagnitude < 1e-8f && Mathf.Abs(center3.y - closest3.y) > 0.05f)
                continue;

            if (e.sqrMagnitude < 1e-8f)
            {
                Vector2 centerDiff = p - ToPlanar(col.bounds.center);
                if (centerDiff.sqrMagnitude < 1e-8f)
                    centerDiff = Vector2.right;
                e = centerDiff.normalized * 0.001f;
            }

            Vector2 obstaclePointVelocity = ObstacleSurfaceVelocity(col, closest3);

            float normalizedSquared = Mathf.Max(0f, ellipse.Q.Quadratic(e));
            float normalizedRadius = Mathf.Sqrt(normalizedSquared);

            if (normalizedRadius > obstacleActivationScale)
                continue;

            float h = normalizedSquared - 1f;
            Vector2 grad = 2f * ellipse.Q.Multiply(e);
            Vector2 effectiveGrad = EffectiveObstacleGradient(col, center3, grad);
            float gradNormSquared = effectiveGrad.sqrMagnitude;

            if (gradNormSquared < 1e-10f)
                continue;

            float shapeRateTerm = ellipse.QDot.Quadratic(e);
            shapeRateTerm = Mathf.Clamp(shapeRateTerm, -maxShapeRateTerm, maxShapeRateTerm);

            // d h/dt ~= effectiveGrad * (u - v_obstacle) + e^T Qdot e.
            // For a translating surface, (I-D_p q) applies to both u and v_obstacle;
            // rotation/deformation is only approximated by GetPointVelocity.
            // The surface Jacobian is a numerical local linearization, not an
            // unconditional continuous-time or sampled-data safety guarantee.
            float rhs = Vector2.Dot(effectiveGrad, obstaclePointVelocity)
                - shapeRateTerm
                - obstacleCBFAlpha * h;

            LinearConstraint4 constraint = new LinearConstraint4();
            if (variableOffset == 0)
                constraint.A = new Vector4State(effectiveGrad.x, effectiveGrad.y, 0f, 0f);
            else
                constraint.A = new Vector4State(0f, 0f, effectiveGrad.x, effectiveGrad.y);
            constraint.B = rhs;

            float riskValue = ComputeNormalizedBoundaryRisk(normalizedRadius, obstacleActivationScale);
            constraint.Risk = riskValue;
            target[count++] = constraint;

            risk.MaxRisk = Mathf.Max(risk.MaxRisk, riskValue);
            risk.NoRiskProduct *= 1f - riskValue;
            risk.MinBarrier = Mathf.Min(risk.MinBarrier, h);

            if (drawConstraintNormals)
            {
                Vector3 grad3 = new Vector3(grad.x, 0f, grad.y).normalized;
                Debug.DrawRay(center3, grad3 * debugNormalScale, Color.red, 0.03f);
            }
        }

        return count;
    }

    // Conservative horizontal radius: each Rigidbody's non-trigger child
    // Collider AABBs, including offsets from centre of mass. Cached per trial.
    // Must verify real world-scale Collider dimensions for publication.
    public float PhysicalPlanarRadius(Rigidbody body)
    {
        if (body == null) return Mathf.Max(0.001f, physicalRadiusFallback);
        int id = body.GetInstanceID();
        float cached;
        if (physicalRadiusCache.TryGetValue(id, out cached)) return cached;
        float radius = Mathf.Max(0.001f,physicalRadiusFallback);
        Collider[] attached = body.GetComponentsInChildren<Collider>();
        foreach (Collider col in attached)
        {
            if (col == null || col.isTrigger || col.attachedRigidbody != body) continue;
            Vector3 offset = col.bounds.center - body.worldCenterOfMass;
            Vector3 extent = col.bounds.extents;
            float r = new Vector2(offset.x,offset.z).magnitude +
                Mathf.Sqrt(extent.x*extent.x+extent.z*extent.z);
            radius = Mathf.Max(radius,r);
        }
        physicalRadiusCache[id] = radius;
        return radius;
    }

    public float HardPairDistance(Rigidbody a, Rigidbody b, bool leaderPair)
    {
        float geometry = PhysicalPlanarRadius(a)+PhysicalPlanarRadius(b)+
            Mathf.Max(0f,interAgentPhysicalClearance);
        float absoluteFloor = leaderPair ? hardLeaderFollowerDistanceFloor : hardFollowerDistanceFloor;
        return Mathf.Max(geometry,absoluteFloor);
    }

    void SoftLeaderDistance(ref Vector2 correction, Vector2 followerPos, Vector2 leaderPos)
    {
        Vector2 delta = followerPos-leaderPos;
        float dist = delta.magnitude;
        if (dist < 0.0001f || dist >= minimumLeaderFollowerDistance) return;
        float push = Mathf.Min(maxSoftSeparationCorrection,
            softPairSeparationGain*(minimumLeaderFollowerDistance-dist));
        correction += delta/dist * push;
        correction = Vector2.ClampMagnitude(correction, maxSoftSeparationCorrection);
    }

    bool AddFollowerPairConstraint(
        Rigidbody a,
        Rigidbody b,
        LinearConstraint4[] target,
        ref int count,
        ref RiskAccumulator risk
    )
    {
        if (count >= target.Length)
            return false;

        Vector2 pa = ToPlanar(a.worldCenterOfMass);
        Vector2 pb = ToPlanar(b.worldCenterOfMass);
        Vector2 e = pa - pb;
        float distance = e.magnitude;

        if (distance > followerPairActivationDistance)
            return false;

        if (e.sqrMagnitude < 1e-8f)
            e = Vector2.right * 0.001f;

        // Hard CBF uses PHYSICAL collision geometry + margin, NOT 1.0 m
        // preferred formation spacing. This avoids artificial deadlocks in corridors.
        float hardDistance = HardPairDistance(a, b, false);
        float h = e.sqrMagnitude - hardDistance * hardDistance;
        Vector2 grad = 2f * e;

        LinearConstraint4 constraint = new LinearConstraint4();
        constraint.A = new Vector4State(grad.x, grad.y, -grad.x, -grad.y);
        constraint.B = -followerPairCBFAlpha * h;
        constraint.Risk = ComputeDistanceRisk(
            distance,
            hardDistance,
            followerPairActivationDistance
        );
        target[count++] = constraint;

        risk.MaxRisk = Mathf.Max(risk.MaxRisk, constraint.Risk);
        risk.NoRiskProduct *= 1f - constraint.Risk;
        risk.MinBarrier = Mathf.Min(risk.MinBarrier, h);
        return true;
    }

    bool AddLeaderFollowerConstraint(
        Rigidbody follower,
        Rigidbody leader,
        int variableOffset,
        LinearConstraint4[] target,
        ref int count,
        ref RiskAccumulator risk
    )
    {
        if (count >= target.Length)
            return false;

        Vector2 pf = ToPlanar(follower.worldCenterOfMass);
        Vector2 pl = ToPlanar(leader.worldCenterOfMass);
        Vector2 e = pf - pl;
        float distance = e.magnitude;

        if (distance > leaderFollowerActivationDistance)
            return false;

        if (e.sqrMagnitude < 1e-8f)
            e = Vector2.right * 0.001f;

        float hardDistance = HardPairDistance(follower, leader, true);
        float h = e.sqrMagnitude - hardDistance * hardDistance;
        Vector2 grad = 2f * e;
        Vector2 leaderVelocity = ToPlanar(leader.velocity);

        LinearConstraint4 constraint = new LinearConstraint4();
        if (variableOffset == 0)
            constraint.A = new Vector4State(grad.x, grad.y, 0f, 0f);
        else
            constraint.A = new Vector4State(0f, 0f, grad.x, grad.y);

        constraint.B = Vector2.Dot(grad, leaderVelocity)
            - leaderFollowerCBFAlpha * h;
        constraint.Risk = ComputeDistanceRisk(
            distance,
            hardDistance,
            leaderFollowerActivationDistance
        );
        target[count++] = constraint;

        risk.MaxRisk = Mathf.Max(risk.MaxRisk, constraint.Risk);
        risk.NoRiskProduct *= 1f - constraint.Risk;
        risk.MinBarrier = Mathf.Min(risk.MinBarrier, h);
        return true;
    }

    void ProjectHalfSpace(ref Vector4State z, LinearConstraint4 constraint)
    {
        float value = Vector4State.Dot(constraint.A, z);
        if (value >= constraint.B)
            return;

        float denom = constraint.A.SqrMagnitude;
        if (denom < 1e-10f)
            return;

        float step = (constraint.B - value) / denom;
        // An exact half-space projection is required by Dykstra.
        // projectionRelaxation is retained only for saved Inspector compatibility.
        z += constraint.A * step;
    }

    // Projection on the intersection of halfspaces and Cartesian products of
    // planar speed / optional acceleration disks. One dual-correction slot per set.
    float DykstraSolve(ref Vector4State z, int count, Vector2 previousA,
        Vector2 previousB, bool hasPrevious, float dt, bool useAcceleration)
    {
        int sets = count + 2 + ((useAcceleration && hasPrevious && maxPlanarAcceleration > 0f) ? 2 : 0);
        for (int i = 0; i < sets; i++) dykstraCorrections[i] = new Vector4State();

        int limit = Mathf.Clamp(projectionIterations, 10, 300);
        for (int iter = 0; iter < limit; iter++)
        {
            for (int i = 0; i < sets; i++)
            {
                Vector4State y = z + dykstraCorrections[i];
                Vector4State projected = y;
                if (i < count)
                {
                    ProjectHalfSpace(ref projected, constraints[i]);
                }
                else
                {
                    int j = i - count;
                    float maxSpeed = Mathf.Max(0.01f, maxPlanarSpeed);
                    if (j == 0) ProjectPlanarBall(ref projected, 0, Vector2.zero, maxSpeed);
                    if (j == 1) ProjectPlanarBall(ref projected, 2, Vector2.zero, maxSpeed);
                    if (j == 2) ProjectPlanarBall(ref projected, 0, previousA,
                        Mathf.Max(0.0001f, maxPlanarAcceleration * dt));
                    if (j == 3) ProjectPlanarBall(ref projected, 2, previousB,
                        Mathf.Max(0.0001f, maxPlanarAcceleration * dt));
                }
                dykstraCorrections[i] = y - projected;
                z = projected;
            }
            float err = ComputeMaxViolation(z, count);
            float speedCap = Mathf.Max(0.01f,maxPlanarSpeed);
            err = Mathf.Max(err,(new Vector2(z.x0,z.x1)).magnitude-speedCap);
            err = Mathf.Max(err,(new Vector2(z.x2,z.x3)).magnitude-speedCap);
            if (useAcceleration && hasPrevious && maxPlanarAcceleration > 0f)
            {
                float aRadius = maxPlanarAcceleration * Mathf.Max(0f, dt);
                err = Mathf.Max(err, (new Vector2(z.x0,z.x1)-previousA).magnitude-aRadius);
                err = Mathf.Max(err, (new Vector2(z.x2,z.x3)-previousB).magnitude-aRadius);
            }
            if (err <= feasibilityTolerance) return Mathf.Max(0f, err);
        }
        float violation = ComputeMaxViolation(z, count);
        float speedBound = Mathf.Max(0.01f,maxPlanarSpeed);
        violation = Mathf.Max(violation,(new Vector2(z.x0,z.x1)).magnitude-speedBound);
        violation = Mathf.Max(violation,(new Vector2(z.x2,z.x3)).magnitude-speedBound);
        if (useAcceleration && hasPrevious && maxPlanarAcceleration > 0f)
        {
            float r = maxPlanarAcceleration * Mathf.Max(0f, dt);
            violation = Mathf.Max(violation, (new Vector2(z.x0,z.x1)-previousA).magnitude-r);
            violation = Mathf.Max(violation, (new Vector2(z.x2,z.x3)-previousB).magnitude-r);
        }
        return Mathf.Max(0f, violation);
    }

    void ProjectPlanarBall(ref Vector4State x, int offset, Vector2 center, float radius)
    {
        Vector2 v = offset == 0 ? new Vector2(x.x0,x.x1) : new Vector2(x.x2,x.x3);
        v = center + Vector2.ClampMagnitude(v-center, radius);
        if (offset == 0) { x.x0 = v.x; x.x1 = v.y; }
        else { x.x2 = v.x; x.x3 = v.y; }
    }

    Vector2 ProjectVelocityLimits(Vector2 velocity, Vector2 previous, bool hasPrevious, float dt)
    {
        velocity = Vector2.ClampMagnitude(velocity, Mathf.Max(0.01f, maxPlanarSpeed));

        if (hasPrevious && maxPlanarAcceleration > 0f && dt > 1e-6f)
        {
            float maxDelta = maxPlanarAcceleration * dt;
            Vector2 delta = velocity - previous;
            if (delta.magnitude > maxDelta)
                velocity = previous + delta.normalized * maxDelta;
        }

        return velocity;
    }

    float ComputeMaxViolation(Vector4State z, int count)
    {
        float maxViolation = 0f;
        for (int i = 0; i < count; i++)
        {
            float violation = constraints[i].B - Vector4State.Dot(constraints[i].A, z);
            maxViolation = Mathf.Max(maxViolation, violation);
        }
        Vector2 a = new Vector2(z.x0, z.x1);
        Vector2 b = new Vector2(z.x2, z.x3);
        maxViolation = Mathf.Max(maxViolation, a.magnitude - Mathf.Max(0.01f, maxPlanarSpeed));
        maxViolation = Mathf.Max(maxViolation, b.magnitude - Mathf.Max(0.01f, maxPlanarSpeed));
        return Mathf.Max(0f, maxViolation);
    }

    float ComputeNormalizedBoundaryRisk(float normalizedRadius, float activationScale)
    {
        if (normalizedRadius <= 1f)
            return 1f;

        float denom = Mathf.Max(1e-4f, activationScale - 1f);
        float risk = (activationScale - normalizedRadius) / denom;
        risk = Mathf.Clamp01(risk);
        return risk * risk * (3f - 2f * risk);
    }

    float ComputeDistanceRisk(float distance, float hardDistance, float activationDistance)
    {
        if (distance <= hardDistance)
            return 1f;

        float denom = Mathf.Max(1e-4f, activationDistance - hardDistance);
        float risk = (activationDistance - distance) / denom;
        risk = Mathf.Clamp01(risk);
        return risk * risk * (3f - 2f * risk);
    }

    bool ShouldIgnoreCollider(Collider col, Rigidbody self)
    {
        if (col == null)
            return true;

        if (col.attachedRigidbody == self)
            return true;

        if (self != null && col.transform.IsChildOf(self.transform))
            return true;

        return false;
    }

    Vector2 WeightedReference(Vector2 nominal, Vector2 previous, float weight)
    {
        float w = Mathf.Max(0f, weight);
        return (nominal + w * previous) / (1f + w);
    }

    static Vector2 ToPlanar(Vector3 v)
    {
        return new Vector2(v.x, v.z);
    }

    static Vector3 FromPlanar(Vector2 v, float y)
    {
        return new Vector3(v.x, y, v.y);
    }

    static float PlanarDistance(Vector3 a, Vector3 b)
    {
        Vector2 da = ToPlanar(a);
        Vector2 db = ToPlanar(b);
        return Vector2.Distance(da, db);
    }

    struct RiskAccumulator
    {
        public float MaxRisk;
        public float NoRiskProduct;
        public float MinBarrier;
    }

    struct LinearConstraint4
    {
        public Vector4State A;
        public float B;
        public float Risk;
    }

    struct Vector4State
    {
        public float x0;
        public float x1;
        public float x2;
        public float x3;

        public Vector4State(float v0, float v1, float v2, float v3)
        {
            x0 = v0;
            x1 = v1;
            x2 = v2;
            x3 = v3;
        }

        public float SqrMagnitude
        {
            get { return x0 * x0 + x1 * x1 + x2 * x2 + x3 * x3; }
        }

        public static float Dot(Vector4State a, Vector4State b)
        {
            return a.x0 * b.x0 + a.x1 * b.x1 + a.x2 * b.x2 + a.x3 * b.x3;
        }

        public static Vector4State operator +(Vector4State a, Vector4State b)
        {
            return new Vector4State(
                a.x0 + b.x0,
                a.x1 + b.x1,
                a.x2 + b.x2,
                a.x3 + b.x3
            );
        }

        public static Vector4State operator -(Vector4State a, Vector4State b)
        {
            return new Vector4State(a.x0-b.x0, a.x1-b.x1, a.x2-b.x2, a.x3-b.x3);
        }

        public static Vector4State operator *(Vector4State a, float s)
        {
            return new Vector4State(a.x0 * s, a.x1 * s, a.x2 * s, a.x3 * s);
        }
    }
}

public struct SingleAgentCBFResult
{
    public Vector3 NominalVelocity;
    public Vector3 SafeVelocity;
    public Vector3 Correction;
    public int ActiveObstacleConstraints;
    public float MaxRisk;
    public float AggregateRisk;
    public float MinBarrier;
    public float MaxConstraintViolation;
    public bool ApproximatelyFeasible;
}

public struct JointFollowerCBFResult
{
    public Vector3 NominalVelocityA;
    public Vector3 NominalVelocityB;
    public Vector3 SafeVelocityA;
    public Vector3 SafeVelocityB;
    public Vector3 CorrectionA;
    public Vector3 CorrectionB;
    public int ObstacleConstraintsA;
    public int ObstacleConstraintsB;
    public bool FollowerPairConstraintActive;
    public int LeaderFollowerConstraints;
    public int TotalConstraints;
    public float MaxRisk;
    public float AggregateRisk;
    public float MinBarrier;
    public float MaxConstraintViolation;
    public float MinimumPairDistance;
    public bool ApproximatelyFeasible;
    public bool AccelerationRelaxed;
    public bool EmergencyStopped;
    public float ResidualWithAcceleration;
    public float HardPairMinimumDistance;
    public float SoftSeparationCorrectionMagnitude;
    public bool RecoveryAttempted;
    public bool RecoverySucceeded;
    public float RecoveryResidual;
}
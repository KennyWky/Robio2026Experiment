using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Reusable horizontal 360-degree free-space scanner.
/// One component can scan multiple rigidbodies; each body receives an independent cached buffer.
/// </summary>
public class FreeSpaceScanner : MonoBehaviour
{
    [Header("Ray Scan")]
    [Range(12, 360)]
    public int rayCount = 72;

    public float maxDistance = 4.0f;
    public float rayOriginYOffset = 0.0f;
    public LayerMask obstacleMask;

    [Tooltip("0 uses ordinary raycasts. A small positive value uses sphere casts and is more conservative.")]
    public float probeRadius = 0.0f;

    [Header("Density Estimation")]
    [Tooltip("A ray shorter than this distance contributes strongly to the density score.")]
    public float denseDistance = 2.0f;

    [Tooltip("Low-pass time constant for each ray distance. Set 0 to disable.")]
    public float distanceSmoothTau = 0.06f;

    [Header("Debug")]
    public bool drawRays = false;
    public float debugRayDuration = 0.03f;

    readonly Dictionary<int, FreeSpaceScan> scanByBody = new Dictionary<int, FreeSpaceScan>();

    public FreeSpaceScan Scan(Rigidbody body, float dt)
    {
        if (body == null)
            return null;

        int count = Mathf.Max(12, rayCount);
        int key = body.GetInstanceID();

        FreeSpaceScan scan;
        if (!scanByBody.TryGetValue(key, out scan) || scan.RayCount != count)
        {
            scan = new FreeSpaceScan(count);
            scanByBody[key] = scan;
        }

        Vector3 origin = body.worldCenterOfMass + Vector3.up * rayOriginYOffset;
        scan.Origin = origin;
        scan.MaxDistance = Mathf.Max(0.05f, maxDistance);
        scan.Timestamp = Time.time;

        float alpha = 1f;
        if (distanceSmoothTau > 1e-6f && dt > 1e-6f)
            alpha = 1f - Mathf.Exp(-dt / distanceSmoothTau);

        float minDistance = scan.MaxDistance;
        float sumDistance = 0f;
        float densityAccumulator = 0f;

        for (int i = 0; i < count; i++)
        {
            float angle = 2f * Mathf.PI * i / count;
            Vector3 direction = new Vector3(Mathf.Sin(angle), 0f, Mathf.Cos(angle));

            float measuredDistance = scan.MaxDistance;
            RaycastHit hit;
            bool hasHit;

            if (probeRadius > 1e-5f)
            {
                hasHit = Physics.SphereCast(
                    origin,
                    probeRadius,
                    direction,
                    out hit,
                    scan.MaxDistance,
                    obstacleMask,
                    QueryTriggerInteraction.Ignore
                );
            }
            else
            {
                hasHit = Physics.Raycast(
                    origin,
                    direction,
                    out hit,
                    scan.MaxDistance,
                    obstacleMask,
                    QueryTriggerInteraction.Ignore
                );
            }

            if (hasHit)
                measuredDistance = Mathf.Clamp(hit.distance, 0f, scan.MaxDistance);

            scan.Directions[i] = direction;

            if (!scan.Initialized)
                scan.Distances[i] = measuredDistance;
            else
                scan.Distances[i] = Mathf.Lerp(scan.Distances[i], measuredDistance, alpha);

            float d = scan.Distances[i];
            minDistance = Mathf.Min(minDistance, d);
            sumDistance += d;

            float dense = 1f - Mathf.Clamp01(d / Mathf.Max(0.05f, denseDistance));
            densityAccumulator += dense * dense;

            if (drawRays)
            {
                Color c = hasHit ? Color.red : Color.gray;
                Debug.DrawRay(origin, direction * d, c, debugRayDuration);
            }
        }

        scan.Initialized = true;
        scan.MinDistance = minDistance;
        scan.MeanDistance = sumDistance / count;
        scan.DensityScore = Mathf.Clamp01(densityAccumulator / count);

        return scan;
    }
}

/// <summary>
/// Cached scan result. Directions use Unity's X-Z plane; index zero points along world +Z.
/// </summary>
public sealed class FreeSpaceScan
{
    public Vector3 Origin;
    public float MaxDistance;
    public float Timestamp;
    public float MinDistance;
    public float MeanDistance;
    public float DensityScore;
    public bool Initialized;

    public readonly float[] Distances;
    public readonly Vector3[] Directions;

    public int RayCount
    {
        get { return Distances.Length; }
    }

    public FreeSpaceScan(int rayCount)
    {
        int n = Mathf.Max(12, rayCount);
        Distances = new float[n];
        Directions = new Vector3[n];
    }

    public float SampleDistance(Vector3 worldDirection)
    {
        if (RayCount == 0)
            return MaxDistance;

        worldDirection.y = 0f;
        if (worldDirection.sqrMagnitude < 1e-8f)
            return MaxDistance;

        worldDirection.Normalize();

        float angle = Mathf.Atan2(worldDirection.x, worldDirection.z);
        if (angle < 0f)
            angle += 2f * Mathf.PI;

        float indexFloat = angle / (2f * Mathf.PI) * RayCount;
        int i0 = Mathf.FloorToInt(indexFloat) % RayCount;
        int i1 = (i0 + 1) % RayCount;
        float t = indexFloat - Mathf.Floor(indexFloat);

        return Mathf.Lerp(Distances[i0], Distances[i1], t);
    }

    public float WindowMinimum(Vector3 worldDirection, float halfAngleDegrees)
    {
        if (RayCount == 0)
            return MaxDistance;

        worldDirection.y = 0f;
        if (worldDirection.sqrMagnitude < 1e-8f)
            return MaxDistance;

        worldDirection.Normalize();

        float centerAngle = Mathf.Atan2(worldDirection.x, worldDirection.z);
        if (centerAngle < 0f)
            centerAngle += 2f * Mathf.PI;

        int centerIndex = Mathf.RoundToInt(centerAngle / (2f * Mathf.PI) * RayCount) % RayCount;
        int halfWindow = Mathf.Max(
            0,
            Mathf.CeilToInt(Mathf.Abs(halfAngleDegrees) / 360f * RayCount)
        );

        float minimum = MaxDistance;

        for (int offset = -halfWindow; offset <= halfWindow; offset++)
        {
            int index = centerIndex + offset;
            while (index < 0) index += RayCount;
            while (index >= RayCount) index -= RayCount;
            minimum = Mathf.Min(minimum, Distances[index]);
        }

        return minimum;
    }
}

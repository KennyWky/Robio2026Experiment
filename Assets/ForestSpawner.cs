using System.Collections.Generic;
using UnityEngine;


public class ForestSpawner : MonoBehaviour
{
    [Header("Tree Prefab")]
    public GameObject treePrefab;

    [Header("Spawn Region (in front of this Transform)")]
    public float width = 60f;         
    public float depth = 80f;         
    public float forwardOffset = 20f; 

    [Header("Count")]
    public int count = 100;

    [Header("Ground")]
    public LayerMask groundMask;      
    public float rayStartHeight = 200f;
    public float rayMaxDistance = 500f;

    [Header("Distribution")]
    public bool avoidOverlap = true;
    public float minSpacing = 4.0f;   
    public int maxTriesPerTree = 50;  

    [Header("Randomization")]
    public Vector2 uniformScaleRange = new Vector2(0.8f, 1.3f);
    public bool randomYaw = true;

    [Header("Runtime")]
    public bool spawnOnStart = true;
    public bool clearBeforeSpawn = true;

    readonly List<Vector3> placed = new List<Vector3>();

    void Start()
    {
        if (spawnOnStart)
            Spawn();
    }

    [ContextMenu("Spawn")]
    public void Spawn()
    {
        if (treePrefab == null) return;

        if (clearBeforeSpawn)
            ClearSpawned();

        placed.Clear();

        int spawned = 0;
        for (int i = 0; i < count; i++)
        {
            if (TrySpawnOne(out _))
                spawned++;
        }

        Debug.Log($"[ForestSpawner] Spawned {spawned}/{count} trees.");
    }

    bool TrySpawnOne(out GameObject obj)
    {
        obj = null;

        for (int t = 0; t < maxTriesPerTree; t++)
        {
            Vector3 p = SamplePointInRegion();

            if (RaycastToGround(p, out Vector3 hitPoint, out Quaternion groundRot) == false)
                continue;

            if (avoidOverlap && !IsFarEnough(hitPoint))
                continue;

            obj = Instantiate(treePrefab, hitPoint, Quaternion.identity, transform);

            if (randomYaw)
                obj.transform.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);

            float s = Random.Range(uniformScaleRange.x, uniformScaleRange.y);
            obj.transform.localScale = obj.transform.localScale * s;

            placed.Add(hitPoint);
            return true;
        }

        return false;
    }

    Vector3 SamplePointInRegion()
    {
        float x = Random.Range(-width * 0.5f, width * 0.5f);
        float z = Random.Range(0f, depth);

        Vector3 local = new Vector3(x, 0f, forwardOffset + z);
        return transform.TransformPoint(local);
    }

    bool RaycastToGround(Vector3 worldPoint, out Vector3 hitPoint, out Quaternion groundRot)
    {
        hitPoint = worldPoint;
        groundRot = Quaternion.identity;

        Vector3 origin = new Vector3(worldPoint.x, worldPoint.y + rayStartHeight, worldPoint.z);
        Ray r = new Ray(origin, Vector3.down);

        if (Physics.Raycast(r, out RaycastHit hit, rayMaxDistance, groundMask, QueryTriggerInteraction.Ignore))
        {
            hitPoint = hit.point;
            groundRot = Quaternion.FromToRotation(Vector3.up, hit.normal);
            return true;
        }

        return false;
    }

    bool IsFarEnough(Vector3 p)
    {
        float min2 = minSpacing * minSpacing;
        for (int i = 0; i < placed.Count; i++)
        {
            if ((placed[i] - p).sqrMagnitude < min2)
                return false;
        }
        return true;
    }

    [ContextMenu("ClearSpawned")]
    public void ClearSpawned()
    {
        var toDestroy = new List<GameObject>();
        for (int i = 0; i < transform.childCount; i++)
            toDestroy.Add(transform.GetChild(i).gameObject);

        for (int i = 0; i < toDestroy.Count; i++)
        {
            if (Application.isPlaying) Destroy(toDestroy[i]);
            else DestroyImmediate(toDestroy[i]);
        }
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.green;

        Vector3 cLocal = new Vector3(0f, 0f, forwardOffset + depth * 0.5f);
        Vector3 c = transform.TransformPoint(cLocal);

        Vector3 right = transform.right * (width * 0.5f);
        Vector3 forward = transform.forward * (depth * 0.5f);

        Vector3 p0 = c - right - forward;
        Vector3 p1 = c + right - forward;
        Vector3 p2 = c + right + forward;
        Vector3 p3 = c - right + forward;

        Gizmos.DrawLine(p0, p1);
        Gizmos.DrawLine(p1, p2);
        Gizmos.DrawLine(p2, p3);
        Gizmos.DrawLine(p3, p0);
    }
}

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// Exports obstacle collider geometry for X-Z/top-down visualization.
///
/// Files:
/// 1) *_obstacles.csv:
///    One row per obstacle collider, including its initial pose, dimensions,
///    collider type and an X-Z footprint polygon.
///
/// 2) *_obstacle_motion.csv:
///    Optional time-series pose/footprint data. This is useful when obstacles
///    move or rotate during the experiment.
///
/// The exporter only treats colliders in obstacleMask as CBF obstacles.
/// finishZone is exported separately with role="finish_zone" so that it can be
/// drawn without being included in obstacleMask.
/// </summary>
public class ObstacleGeometryCsvExporter : MonoBehaviour
{
    [Header("Obstacle Selection")]
    public LayerMask obstacleMask;
    public bool includeTriggerObstacles = false;

    [Tooltip("Active scene colliders are scanned when the experiment starts.")]
    public bool includeInactiveObjects = false;

    [Header("Geometry")]
    [Range(12, 96)]
    public int curvedFootprintSegments = 40;

    [Tooltip("Maximum number of MeshCollider vertices used to construct the X-Z convex hull.")]
    public int maxMeshVertices = 12000;

    [Header("Motion Logging")]
    public bool logObstacleMotion = true;

    [Tooltip("When true, all exported obstacles are sampled. This also captures obstacles moved without a Rigidbody.")]
    public bool logAllObstacleMotion = true;

    [Tooltip("When false, only colliders with an attached Rigidbody are written to the motion CSV.")]
    public bool includeKinematicRigidbodies = true;

    readonly List<ObstacleRecord> records = new List<ObstacleRecord>();
    StreamWriter motionWriter;

    Collider finishZoneToExport;
    Rigidbody leaderToIgnore;
    Rigidbody followerAToIgnore;
    Rigidbody followerBToIgnore;

    class ObstacleRecord
    {
        public string id;
        public string role;
        public Collider collider;
    }

    public void ExportAtExperimentStart(
        string geometryCsvPath,
        string motionCsvPath,
        LayerMask sourceObstacleMask,
        Collider finishZone,
        Rigidbody leader,
        Rigidbody followerA,
        Rigidbody followerB
    )
    {
        obstacleMask = sourceObstacleMask;
        finishZoneToExport = finishZone;
        leaderToIgnore = leader;
        followerAToIgnore = followerA;
        followerBToIgnore = followerB;

        BuildRecordList();
        WriteGeometryCsv(geometryCsvPath);

        CloseMotionLog();

        if (logObstacleMotion && !string.IsNullOrEmpty(motionCsvPath))
            OpenMotionLog(motionCsvPath);
    }

    public void SampleMotion(float sessionTime)
    {
        if (motionWriter == null)
            return;

        for (int i = 0; i < records.Count; i++)
        {
            ObstacleRecord record = records[i];

            if (record == null || record.collider == null)
                continue;

            if (!ShouldLogMotion(record))
                continue;

            Collider collider = record.collider;
            Vector3 center = GetWorldCenter(collider);
            Vector3 euler = collider.transform.rotation.eulerAngles;
            Bounds bounds = collider.bounds;
            string footprint = EncodeFootprint(GetFootprint(collider));

            motionWriter.WriteLine(string.Join(",", new string[]
            {
                F(sessionTime),
                Csv(record.id),
                Csv(record.role),
                Csv(collider.name),
                F(center.x),
                F(center.y),
                F(center.z),
                F(euler.x),
                F(euler.y),
                F(euler.z),
                F(bounds.min.x),
                F(bounds.min.y),
                F(bounds.min.z),
                F(bounds.max.x),
                F(bounds.max.y),
                F(bounds.max.z),
                Csv(footprint)
            }));
        }
    }

    public void CloseMotionLog()
    {
        if (motionWriter == null)
            return;

        try
        {
            motionWriter.Flush();
            motionWriter.Close();
        }
        catch (Exception exception)
        {
            Debug.LogWarning(
                "[ObstacleExporter] Failed to close motion CSV: " +
                exception.Message
            );
        }
        finally
        {
            motionWriter = null;
        }
    }

    void OnApplicationQuit()
    {
        CloseMotionLog();
    }

    void OnDestroy()
    {
        CloseMotionLog();
    }

    void BuildRecordList()
    {
        records.Clear();

        Collider[] colliders;

#if UNITY_2020_1_OR_NEWER
        if (includeInactiveObjects)
            colliders = Resources.FindObjectsOfTypeAll<Collider>();
        else
            colliders = FindObjectsOfType<Collider>();
#else
        colliders = FindObjectsOfType<Collider>();
#endif

        List<Collider> selected = new List<Collider>();

        for (int i = 0; i < colliders.Length; i++)
        {
            Collider collider = colliders[i];

            if (!IsValidSceneCollider(collider))
                continue;

            if (collider == finishZoneToExport)
                continue;

            if (!LayerIsIncluded(collider.gameObject.layer))
                continue;

            if (!includeTriggerObstacles && collider.isTrigger)
                continue;

            if (IsAgentCollider(collider))
                continue;

            selected.Add(collider);
        }

        selected.Sort(
            delegate(Collider a, Collider b)
            {
                string aPath = GetHierarchyPath(a.transform);
                string bPath = GetHierarchyPath(b.transform);
                int pathComparison = string.CompareOrdinal(aPath, bPath);

                if (pathComparison != 0)
                    return pathComparison;

                return string.CompareOrdinal(
                    a.GetType().Name,
                    b.GetType().Name
                );
            }
        );

        for (int i = 0; i < selected.Count; i++)
        {
            records.Add(new ObstacleRecord
            {
                id = "OBS_" + (i + 1).ToString("D4", CultureInfo.InvariantCulture),
                role = "obstacle",
                collider = selected[i]
            });
        }

        if (
            finishZoneToExport != null &&
            IsValidSceneCollider(finishZoneToExport)
        )
        {
            records.Add(new ObstacleRecord
            {
                id = "FINISH_0001",
                role = "finish_zone",
                collider = finishZoneToExport
            });
        }
    }

    bool IsValidSceneCollider(Collider collider)
    {
        if (collider == null || collider.gameObject == null)
            return false;

        if (!collider.gameObject.scene.IsValid())
            return false;

        if (!includeInactiveObjects && !collider.gameObject.activeInHierarchy)
            return false;

        return true;
    }

    bool LayerIsIncluded(int layer)
    {
        return (obstacleMask.value & (1 << layer)) != 0;
    }

    bool IsAgentCollider(Collider collider)
    {
        return ColliderBelongsTo(collider, leaderToIgnore) ||
            ColliderBelongsTo(collider, followerAToIgnore) ||
            ColliderBelongsTo(collider, followerBToIgnore);
    }

    bool ColliderBelongsTo(Collider collider, Rigidbody body)
    {
        if (collider == null || body == null)
            return false;

        return collider.attachedRigidbody == body ||
            collider.transform.IsChildOf(body.transform);
    }

    bool ShouldLogMotion(ObstacleRecord record)
    {
        if (record.role == "finish_zone")
            return false;

        if (logAllObstacleMotion)
            return true;

        Rigidbody body = record.collider.attachedRigidbody;

        if (body == null)
            return false;

        if (!includeKinematicRigidbodies && body.isKinematic)
            return false;

        return true;
    }

    void WriteGeometryCsv(string path)
    {
        try
        {
            string directory = Path.GetDirectoryName(path);

            if (!string.IsNullOrEmpty(directory))
                Directory.CreateDirectory(directory);

            using (
                StreamWriter writer = new StreamWriter(
                    path,
                    false,
                    new UTF8Encoding(false)
                )
            )
            {
                writer.WriteLine(string.Join(",", new string[]
                {
                    "obstacle_id",
                    "role",
                    "object_name",
                    "hierarchy_path",
                    "layer",
                    "tag",
                    "shape",
                    "collider_type",
                    "is_trigger",
                    "has_rigidbody",
                    "is_kinematic",
                    "center_x",
                    "center_y",
                    "center_z",
                    "rotation_x_deg",
                    "rotation_y_deg",
                    "rotation_z_deg",
                    "size_x",
                    "size_y",
                    "size_z",
                    "radius",
                    "height",
                    "capsule_direction",
                    "bounds_min_x",
                    "bounds_min_y",
                    "bounds_min_z",
                    "bounds_max_x",
                    "bounds_max_y",
                    "bounds_max_z",
                    "footprint_xz"
                }));

                for (int i = 0; i < records.Count; i++)
                {
                    ObstacleRecord record = records[i];

                    if (record == null || record.collider == null)
                        continue;

                    writer.WriteLine(BuildGeometryRow(record));
                }
            }

            Debug.Log(
                "[ObstacleExporter] Geometry CSV saved: " +
                path +
                " | records=" +
                records.Count
            );
        }
        catch (Exception exception)
        {
            Debug.LogError(
                "[ObstacleExporter] Failed to write geometry CSV: " +
                exception.Message
            );
        }
    }

    string BuildGeometryRow(ObstacleRecord record)
    {
        Collider collider = record.collider;
        Vector3 center = GetWorldCenter(collider);
        Vector3 euler = collider.transform.rotation.eulerAngles;
        Vector3 size = GetWorldSize(collider);
        float radius = GetWorldRadius(collider);
        float height = GetWorldHeight(collider);
        int capsuleDirection = GetCapsuleDirection(collider);
        Bounds bounds = collider.bounds;
        Rigidbody body = collider.attachedRigidbody;

        return string.Join(",", new string[]
        {
            Csv(record.id),
            Csv(record.role),
            Csv(collider.name),
            Csv(GetHierarchyPath(collider.transform)),
            Csv(LayerMask.LayerToName(collider.gameObject.layer)),
            Csv(SafeTag(collider.gameObject)),
            Csv(GetShapeName(collider)),
            Csv(collider.GetType().Name),
            Bool01(collider.isTrigger),
            Bool01(body != null),
            Bool01(body != null && body.isKinematic),
            F(center.x),
            F(center.y),
            F(center.z),
            F(euler.x),
            F(euler.y),
            F(euler.z),
            F(size.x),
            F(size.y),
            F(size.z),
            F(radius),
            F(height),
            capsuleDirection.ToString(CultureInfo.InvariantCulture),
            F(bounds.min.x),
            F(bounds.min.y),
            F(bounds.min.z),
            F(bounds.max.x),
            F(bounds.max.y),
            F(bounds.max.z),
            Csv(EncodeFootprint(GetFootprint(collider)))
        });
    }

    void OpenMotionLog(string path)
    {
        try
        {
            string directory = Path.GetDirectoryName(path);

            if (!string.IsNullOrEmpty(directory))
                Directory.CreateDirectory(directory);

            motionWriter = new StreamWriter(
                path,
                false,
                new UTF8Encoding(false)
            );

            motionWriter.WriteLine(string.Join(",", new string[]
            {
                "session_t",
                "obstacle_id",
                "role",
                "object_name",
                "center_x",
                "center_y",
                "center_z",
                "rotation_x_deg",
                "rotation_y_deg",
                "rotation_z_deg",
                "bounds_min_x",
                "bounds_min_y",
                "bounds_min_z",
                "bounds_max_x",
                "bounds_max_y",
                "bounds_max_z",
                "footprint_xz"
            }));

            motionWriter.Flush();

            Debug.Log(
                "[ObstacleExporter] Motion CSV opened: " +
                path
            );
        }
        catch (Exception exception)
        {
            Debug.LogError(
                "[ObstacleExporter] Failed to open motion CSV: " +
                exception.Message
            );

            motionWriter = null;
        }
    }

    Vector3 GetWorldCenter(Collider collider)
    {
        BoxCollider box = collider as BoxCollider;

        if (box != null)
            return box.transform.TransformPoint(box.center);

        SphereCollider sphere = collider as SphereCollider;

        if (sphere != null)
            return sphere.transform.TransformPoint(sphere.center);

        CapsuleCollider capsule = collider as CapsuleCollider;

        if (capsule != null)
            return capsule.transform.TransformPoint(capsule.center);

        MeshCollider mesh = collider as MeshCollider;

        if (mesh != null && mesh.sharedMesh != null)
            return mesh.transform.TransformPoint(mesh.sharedMesh.bounds.center);

        return collider.bounds.center;
    }

    Vector3 GetWorldSize(Collider collider)
    {
        BoxCollider box = collider as BoxCollider;

        if (box != null)
            return Vector3.Scale(box.size, Abs(box.transform.lossyScale));

        SphereCollider sphere = collider as SphereCollider;

        if (sphere != null)
        {
            Vector3 scale = Abs(sphere.transform.lossyScale);
            return new Vector3(
                2f * sphere.radius * scale.x,
                2f * sphere.radius * scale.y,
                2f * sphere.radius * scale.z
            );
        }

        CapsuleCollider capsule = collider as CapsuleCollider;

        if (capsule != null)
            return capsule.bounds.size;

        return collider.bounds.size;
    }

    float GetWorldRadius(Collider collider)
    {
        SphereCollider sphere = collider as SphereCollider;

        if (sphere != null)
        {
            Vector3 scale = Abs(sphere.transform.lossyScale);
            return sphere.radius * Mathf.Max(scale.x, Mathf.Max(scale.y, scale.z));
        }

        CapsuleCollider capsule = collider as CapsuleCollider;

        if (capsule != null)
        {
            Vector3 scale = Abs(capsule.transform.lossyScale);

            if (capsule.direction == 0)
                return capsule.radius * Mathf.Max(scale.y, scale.z);

            if (capsule.direction == 1)
                return capsule.radius * Mathf.Max(scale.x, scale.z);

            return capsule.radius * Mathf.Max(scale.x, scale.y);
        }

        return 0f;
    }

    float GetWorldHeight(Collider collider)
    {
        CapsuleCollider capsule = collider as CapsuleCollider;

        if (capsule == null)
            return 0f;

        Vector3 scale = Abs(capsule.transform.lossyScale);

        if (capsule.direction == 0)
            return capsule.height * scale.x;

        if (capsule.direction == 1)
            return capsule.height * scale.y;

        return capsule.height * scale.z;
    }

    int GetCapsuleDirection(Collider collider)
    {
        CapsuleCollider capsule = collider as CapsuleCollider;
        return capsule != null ? capsule.direction : -1;
    }

    string GetShapeName(Collider collider)
    {
        if (collider is BoxCollider)
            return "box";

        if (collider is SphereCollider)
            return "sphere";

        if (collider is CapsuleCollider)
            return "capsule_or_cylinder";

        if (collider is MeshCollider)
            return "mesh";

        return "bounds";
    }

    List<Vector2> GetFootprint(Collider collider)
    {
        BoxCollider box = collider as BoxCollider;

        if (box != null)
            return BoxFootprint(box);

        SphereCollider sphere = collider as SphereCollider;

        if (sphere != null)
            return EllipseFromBounds(sphere.bounds);

        CapsuleCollider capsule = collider as CapsuleCollider;

        if (capsule != null && capsule.direction == 1)
            return EllipseFromBounds(capsule.bounds);

        MeshCollider mesh = collider as MeshCollider;

        if (mesh != null)
        {
            List<Vector2> meshHull = MeshFootprint(mesh);

            if (meshHull.Count >= 3)
                return meshHull;
        }

        return BoundsFootprint(collider.bounds);
    }

    List<Vector2> BoxFootprint(BoxCollider box)
    {
        Vector3 half = box.size * 0.5f;
        List<Vector2> points = new List<Vector2>(8);

        for (int x = -1; x <= 1; x += 2)
        {
            for (int y = -1; y <= 1; y += 2)
            {
                for (int z = -1; z <= 1; z += 2)
                {
                    Vector3 local = box.center + new Vector3(
                        half.x * x,
                        half.y * y,
                        half.z * z
                    );

                    Vector3 world = box.transform.TransformPoint(local);
                    points.Add(new Vector2(world.x, world.z));
                }
            }
        }

        return ConvexHull(points);
    }

    List<Vector2> EllipseFromBounds(Bounds bounds)
    {
        int segments = Mathf.Clamp(curvedFootprintSegments, 12, 96);
        List<Vector2> points = new List<Vector2>(segments);

        for (int i = 0; i < segments; i++)
        {
            float angle = 2f * Mathf.PI * i / segments;

            points.Add(new Vector2(
                bounds.center.x + bounds.extents.x * Mathf.Cos(angle),
                bounds.center.z + bounds.extents.z * Mathf.Sin(angle)
            ));
        }

        return points;
    }

    List<Vector2> MeshFootprint(MeshCollider meshCollider)
    {
        Mesh mesh = meshCollider.sharedMesh;

        if (mesh == null)
            return new List<Vector2>();

        try
        {
            Vector3[] vertices = mesh.vertices;

            if (vertices == null || vertices.Length == 0)
                return new List<Vector2>();

            int maximum = Mathf.Max(16, maxMeshVertices);
            int stride = Mathf.Max(1, Mathf.CeilToInt((float)vertices.Length / maximum));
            List<Vector2> points = new List<Vector2>(
                Mathf.Min(vertices.Length, maximum)
            );

            for (int i = 0; i < vertices.Length; i += stride)
            {
                Vector3 world = meshCollider.transform.TransformPoint(vertices[i]);
                points.Add(new Vector2(world.x, world.z));
            }

            return ConvexHull(points);
        }
        catch (Exception)
        {
            return new List<Vector2>();
        }
    }

    List<Vector2> BoundsFootprint(Bounds bounds)
    {
        return new List<Vector2>
        {
            new Vector2(bounds.min.x, bounds.min.z),
            new Vector2(bounds.max.x, bounds.min.z),
            new Vector2(bounds.max.x, bounds.max.z),
            new Vector2(bounds.min.x, bounds.max.z)
        };
    }

    List<Vector2> ConvexHull(List<Vector2> input)
    {
        if (input == null || input.Count <= 1)
            return input ?? new List<Vector2>();

        List<Vector2> points = new List<Vector2>(input);

        points.Sort(
            delegate(Vector2 a, Vector2 b)
            {
                int xComparison = a.x.CompareTo(b.x);

                if (xComparison != 0)
                    return xComparison;

                return a.y.CompareTo(b.y);
            }
        );

        List<Vector2> unique = new List<Vector2>();

        for (int i = 0; i < points.Count; i++)
        {
            if (
                unique.Count == 0 ||
                (points[i] - unique[unique.Count - 1]).sqrMagnitude > 1e-10f
            )
            {
                unique.Add(points[i]);
            }
        }

        if (unique.Count <= 2)
            return unique;

        List<Vector2> lower = new List<Vector2>();

        for (int i = 0; i < unique.Count; i++)
        {
            while (
                lower.Count >= 2 &&
                Cross(
                    lower[lower.Count - 2],
                    lower[lower.Count - 1],
                    unique[i]
                ) <= 0f
            )
            {
                lower.RemoveAt(lower.Count - 1);
            }

            lower.Add(unique[i]);
        }

        List<Vector2> upper = new List<Vector2>();

        for (int i = unique.Count - 1; i >= 0; i--)
        {
            while (
                upper.Count >= 2 &&
                Cross(
                    upper[upper.Count - 2],
                    upper[upper.Count - 1],
                    unique[i]
                ) <= 0f
            )
            {
                upper.RemoveAt(upper.Count - 1);
            }

            upper.Add(unique[i]);
        }

        lower.RemoveAt(lower.Count - 1);
        upper.RemoveAt(upper.Count - 1);
        lower.AddRange(upper);

        return lower;
    }

    float Cross(Vector2 origin, Vector2 a, Vector2 b)
    {
        return (a.x - origin.x) * (b.y - origin.y) -
            (a.y - origin.y) * (b.x - origin.x);
    }

    string EncodeFootprint(List<Vector2> footprint)
    {
        if (footprint == null || footprint.Count == 0)
            return "";

        StringBuilder builder = new StringBuilder();

        for (int i = 0; i < footprint.Count; i++)
        {
            if (i > 0)
                builder.Append('|');

            builder.Append(F(footprint[i].x));
            builder.Append(':');
            builder.Append(F(footprint[i].y));
        }

        return builder.ToString();
    }

    string GetHierarchyPath(Transform target)
    {
        if (target == null)
            return "";

        string path = target.name;
        Transform parent = target.parent;

        while (parent != null)
        {
            path = parent.name + "/" + path;
            parent = parent.parent;
        }

        return path;
    }

    string SafeTag(GameObject obj)
    {
        try
        {
            return obj.tag;
        }
        catch
        {
            return "";
        }
    }

    Vector3 Abs(Vector3 value)
    {
        return new Vector3(
            Mathf.Abs(value.x),
            Mathf.Abs(value.y),
            Mathf.Abs(value.z)
        );
    }

    static bool BoolValue(bool value)
    {
        return value;
    }

    static string Bool01(bool value)
    {
        return value ? "1" : "0";
    }

    static string F(float value)
    {
        if (float.IsNaN(value) || float.IsInfinity(value))
            return "-1";

        return value.ToString("F6", CultureInfo.InvariantCulture);
    }

    static string Csv(string value)
    {
        if (value == null)
            return "";

        bool requiresQuotes =
            value.IndexOf(',') >= 0 ||
            value.IndexOf('"') >= 0 ||
            value.IndexOf('\n') >= 0 ||
            value.IndexOf('\r') >= 0;

        if (!requiresQuotes)
            return value;

        return "\"" + value.Replace("\"", "\"\"") + "\"";
    }
}

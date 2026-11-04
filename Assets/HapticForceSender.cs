using UnityEngine;
using System.Runtime.InteropServices;

public class HapticForceSender : MonoBehaviour
{
    public string deviceName = "Default Device";

    public float deviceMaxForce = 2f;
    [Range(0.01f, 0.5f)] public float decayFactor = 0.01f;
    public float watchdogTimeout = 0.15f;

    public enum AxisMap { XYZ, XZY, YXZ, YZX, ZXY, ZYX }
    public AxisMap axisMap = AxisMap.YZX;

    public bool invX = false;
    public bool invY = false;
    public bool invZ = false;

    [DllImport("HapticsDirect", CharSet = CharSet.Ansi)]
    private static extern void setConstantForceValues(string configName, double[] direction, double magnitude);

    Vector3 targetForce = Vector3.zero;
    Vector3 currentForce = Vector3.zero;
    float lastKickTime = -999f;

    // NEW: expose the actually applied force in world coordinates (after decay)
    public Vector3 ForceWorld => currentForce;

    public void KickForce(Vector3 forceWorld)
    {
        lastKickTime = Time.time;
        targetForce = forceWorld;
    }

    void Update()
    {
        if (Time.time - lastKickTime > watchdogTimeout)
            targetForce = Vector3.zero;

        currentForce = Vector3.Lerp(currentForce, targetForce, decayFactor);

        float mag = currentForce.magnitude;
        float normMag = Mathf.Clamp01(mag / Mathf.Max(1e-6f, deviceMaxForce));
        Vector3 dir = mag > 1e-6f ? (currentForce / mag) : Vector3.zero;

        if (invX) dir.x = -dir.x;
        if (invY) dir.y = -dir.y;
        if (invZ) dir.z = -dir.z;

        Vector3 mapped = Map(dir, axisMap);

        setConstantForceValues(deviceName, new double[] { mapped.x, mapped.y, mapped.z }, normMag);
    }

    void OnDisable()
    {
        setConstantForceValues(deviceName, new double[] { 0.0, 0.0, 0.0 }, 0.0);
    }

    static Vector3 Map(Vector3 v, AxisMap m)
    {
        switch (m)
        {
            case AxisMap.XYZ: return new Vector3(v.x, v.y, v.z);
            case AxisMap.XZY: return new Vector3(v.x, v.z, v.y);
            case AxisMap.YXZ: return new Vector3(v.y, v.x, v.z);
            case AxisMap.YZX: return new Vector3(v.y, v.z, v.x);
            case AxisMap.ZXY: return new Vector3(v.z, v.x, v.y);
            case AxisMap.ZYX: return new Vector3(v.z, v.y, v.x);
        }
        return v;
    }
}

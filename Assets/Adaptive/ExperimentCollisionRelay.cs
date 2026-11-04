using UnityEngine;

/// <summary>
/// Attach one relay to each UAV, or allow ShapeAdaptiveExperimentLogger to add it automatically.
/// It forwards collision events to the central experiment logger.
/// </summary>
public class ExperimentCollisionRelay : MonoBehaviour
{
    public ShapeAdaptiveExperimentLogger logger;
    public string agentName = "agent";
    public LayerMask collisionMask = ~0;

    void OnCollisionEnter(Collision collision)
    {
        if (logger == null || collision == null || collision.collider == null)
            return;

        int otherLayerBit = 1 << collision.collider.gameObject.layer;
        if ((collisionMask.value & otherLayerBit) == 0)
            return;

        Vector3 point = transform.position;
        if (collision.contactCount > 0)
            point = collision.GetContact(0).point;

        logger.ReportCollision(agentName, collision.collider.name, point);
    }
}

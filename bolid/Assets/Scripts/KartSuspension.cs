using UnityEngine;

public class KartSuspension : MonoBehaviour
{
    [Header("Config")]
    [SerializeField] private KartConfig config;

    [Header("Settings")]
    public LayerMask groundLayer = 1;

    [Header("Suspension Points")]
    [SerializeField] private Transform fl;
    [SerializeField] private Transform fr;
    [SerializeField] private Transform rl;
    [SerializeField] private Transform rr;

    [Header("Debug Data")]
    public float lastFLcompression;
    public float lastFRcompression;
    public float lastRLcompression;
    public float lastRRcompression;

    public bool isFLGrounded;
    public bool isFRGrounded;
    public bool isRLGrounded;
    public bool isRRGrounded;

    public RaycastHit flHit, frHit, rlHit, rrHit;

    private Rigidbody rb;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        if (config == null)
        {
            var controller = GetComponent<KartController>();
            if (controller != null) config = controller.Config;
        }
    }

    private void FixedUpdate()
    {
        if (config == null) return;

        SimulateWheel(fl, ref lastFLcompression, out isFLGrounded, out flHit);
        SimulateWheel(fr, ref lastFRcompression, out isFRGrounded, out frHit);
        SimulateWheel(rl, ref lastRLcompression, out isRLGrounded, out rlHit);
        SimulateWheel(rr, ref lastRRcompression, out isRRGrounded, out rrHit);

        ApplyAntiRollBars();
    }

    private void SimulateWheel(Transform pivot, ref float lastCompression, out bool isGrounded, out RaycastHit hitInfo)
    {
        Vector3 origin = pivot.position;
        Vector3 direction = -pivot.up;
        float maxDist = config.suspensionRestLength + config.suspensionTravel + config.wheelRadius;

        if (Physics.Raycast(origin, direction, out RaycastHit hit, maxDist, groundLayer))
        {
            isGrounded = true;
            hitInfo = hit;

            float currentLength = hit.distance - config.wheelRadius;
            currentLength = Mathf.Clamp(currentLength, 0, config.suspensionRestLength);
            float compression = config.suspensionRestLength - currentLength;

            float springForce = compression * config.springStiffness;

            float compressionVelocity = (compression - lastCompression) / Time.fixedDeltaTime;

            float damperForce = compressionVelocity * config.damperStiffness;
            float totalForce = springForce + damperForce;

            Vector3 forceVector = pivot.up * totalForce;
            rb.AddForceAtPosition(forceVector, pivot.position, ForceMode.Force);

            lastCompression = compression;
        }
        else
        {
            isGrounded = false;
            hitInfo = new RaycastHit();
            lastCompression = 0f;
        }
    }

    private void ApplyAntiRollBars()
    {
        float frontDiff = lastFLcompression - lastFRcompression;
        float frontForce = frontDiff * config.frontAntiRoll;

        if (isFLGrounded) rb.AddForceAtPosition(-fl.up * frontForce, fl.position, ForceMode.Force);
        if (isFRGrounded) rb.AddForceAtPosition(fr.up * frontForce, fr.position, ForceMode.Force);

        float rearDiff = lastRLcompression - lastRRcompression;
        float rearForce = rearDiff * config.rearAntiRoll;

        if (isRLGrounded) rb.AddForceAtPosition(-rl.up * rearForce, rl.position, ForceMode.Force);
        if (isRRGrounded) rb.AddForceAtPosition(rr.up * rearForce, rr.position, ForceMode.Force);
    }

    private void OnDrawGizmos()
    {
        if (config == null) return;

        DrawDebugRay(fl, isFLGrounded, flHit);
        DrawDebugRay(fr, isFRGrounded, frHit);
        DrawDebugRay(rl, isRLGrounded, rlHit);
        DrawDebugRay(rr, isRRGrounded, rrHit);
    }

    private void DrawDebugRay(Transform t, bool hit, RaycastHit hitInfo)
    {
        if (t == null) return;

        float dist = config.suspensionRestLength + config.suspensionTravel + config.wheelRadius;

        if (hit)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(t.position, hitInfo.point);
            Gizmos.DrawSphere(hitInfo.point, 0.05f); 
        }
        else
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(t.position, t.position - t.up * dist);
        }
    }
}
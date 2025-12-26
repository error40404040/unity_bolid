using UnityEngine;

public class KartEngine : MonoBehaviour
{
    [Header("Import parametrs")]
    [SerializeField] private bool _import = false;
    [SerializeField] private KartConfig _kartConfig;

    [Header("RPM settings")]
    [SerializeField] private float _idleRpm = 1000f;
    [SerializeField] private float _maxRpm = 8000f;

    [Header("Torque curve")]
    [SerializeField] private AnimationCurve _torqueCurve;

    public float CurrentRpm { get; private set; }
    public float CurrentTorque { get; private set; }

    private void Awake()
    {
        if (_import && _kartConfig != null) Initialize();
        CurrentRpm = _idleRpm;
    }

    private void Initialize()
    {
        _torqueCurve = _kartConfig.engineTorqueCurve;
        _maxRpm = _kartConfig.maxRpm;
    }

    public float Simulate(float throttleInput, float absForwardSpeed, float deltaTime)
    {
        float radius = _kartConfig != null ? _kartConfig.wheelRadius : 0.34f;
        float ratio = _kartConfig != null ? _kartConfig.gearRatio : 8f;

        float wheelRpm = (absForwardSpeed * 60f) / (2f * Mathf.PI * radius);
        float targetMechanicalRpm = wheelRpm * ratio;

        float slipRpm = 0f;
        if (targetMechanicalRpm < 3000f)
        {
            slipRpm = throttleInput * 2500f;
        }

        float desiredRpm = Mathf.Max(_idleRpm, targetMechanicalRpm + slipRpm);

        CurrentRpm = Mathf.Lerp(CurrentRpm, desiredRpm, deltaTime * 15f);

        CurrentRpm = Mathf.Clamp(CurrentRpm, _idleRpm, _maxRpm);

        if (targetMechanicalRpm > _maxRpm + 50f)
        {
            CurrentTorque = 0f;
            return 0f;
        }

        float torque = _torqueCurve.Evaluate(CurrentRpm);

        if (throttleInput > 0.05f)
        {
            CurrentTorque = torque * throttleInput;
        }
        else
        {
            CurrentTorque = 0f;
        }

        return CurrentTorque;
    }
}
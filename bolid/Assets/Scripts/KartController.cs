using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class KartController : MonoBehaviour
{
    [Header("Configuration")]
    [SerializeField] private bool _import = false;
    [SerializeField] private KartConfig _kartConfig;

    [Header("References")]
    [SerializeField] private KartEngine _engine;
    [SerializeField] private KartSuspension _suspension;

    [Header("Wheel Transforms")]
    [SerializeField] private Transform _frontLeftWheel;
    [SerializeField] private Transform _frontRightWheel;
    [SerializeField] private Transform _rearLeftWheel;
    [SerializeField] private Transform _rearRightWheel;

    [Header("Input")]
    [SerializeField] private InputActionAsset _playerInput;

    [Header("Physics")]
    [SerializeField] private Transform _centerOfMass;
    [SerializeField] private Transform _rearWingPosition;

    private Rigidbody _rigidbody;
    private InputAction _moveAction;

    private float _throttleInput;
    private float _steerInput;
    private bool _handbrakePressed;

    private float _currentDragForce;
    private float _currentDownforce;
    private float _currentGroundEffectForce;

    private Quaternion _flInitRot;
    private Quaternion _frInitRot;

    public KartConfig Config => _kartConfig;

    private void Awake()
    {
        _playerInput.Enable();
        _rigidbody = GetComponent<Rigidbody>();

        if (_engine == null) _engine = GetComponent<KartEngine>();
        if (_suspension == null) _suspension = GetComponent<KartSuspension>();

        var map = _playerInput.FindActionMap("Kart");
        _moveAction = map.FindAction("Move");

        if (_centerOfMass != null)
            _rigidbody.centerOfMass = _centerOfMass.localPosition;

        _flInitRot = _frontLeftWheel.localRotation;
        _frInitRot = _frontRightWheel.localRotation;

        if (_import && _kartConfig != null)
        {
            _rigidbody.mass = _kartConfig.mass;
        }
    }

    private void OnDisable()
    {
        _playerInput.Disable();
    }

    private void Update()
    {
        Vector2 move = _moveAction.ReadValue<Vector2>();
        _steerInput = move.x;
        _throttleInput = move.y;
        _handbrakePressed = Input.GetKey(KeyCode.Space);

        UpdateVisuals();
    }

    private void UpdateVisuals()
    {
        float steerAngle = _kartConfig.maxSteerAngle * _steerInput;
        Quaternion steerRot = Quaternion.Euler(0, steerAngle, 0);
        _frontLeftWheel.localRotation = _flInitRot * steerRot;
        _frontRightWheel.localRotation = _frInitRot * steerRot;

        ApplyWheelVisualPos(_frontLeftWheel, _suspension.isFLGrounded, _suspension.flHit);
        ApplyWheelVisualPos(_frontRightWheel, _suspension.isFRGrounded, _suspension.frHit);
        ApplyWheelVisualPos(_rearLeftWheel, _suspension.isRLGrounded, _suspension.rlHit);
        ApplyWheelVisualPos(_rearRightWheel, _suspension.isRRGrounded, _suspension.rrHit);
    }

    private void ApplyWheelVisualPos(Transform anchor, bool isGrounded, RaycastHit hit)
    {
        if (anchor.childCount == 0) return;
        Transform visualWheel = anchor.GetChild(0);

        Vector3 targetWorldPosition;

        if (isGrounded)
        {
            targetWorldPosition = hit.point + (transform.up * _kartConfig.wheelRadius);
        }
        else
        {
            targetWorldPosition = anchor.position - (anchor.up * _kartConfig.suspensionRestLength);
        }

        Vector3 targetLocalPos = anchor.InverseTransformPoint(targetWorldPosition);

        float targetY = targetLocalPos.y;

        Vector3 currentLocal = visualWheel.localPosition;
        float smoothY = Mathf.Lerp(currentLocal.y, targetY, Time.deltaTime * 40f);


        visualWheel.localPosition = new Vector3(0, smoothY, 0);

       
    }

    private void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        float forwardSpeed = Vector3.Dot(_rigidbody.linearVelocity, transform.forward);
        float absSpeed = Mathf.Abs(forwardSpeed);

        ApplyAerodynamics(absSpeed);

        float engineInput = Mathf.Abs(_throttleInput);
        if (forwardSpeed < -1f && _throttleInput > 0) engineInput = _throttleInput;

        float engineTorque = _engine.Simulate(engineInput, absSpeed, dt);
        float wheelTorque = (engineTorque * _kartConfig.gearRatio * 0.9f) / 2f;

        ProcessWheel(_frontLeftWheel, 0f, forwardSpeed, _suspension.isFLGrounded, _suspension.flHit, true, false);
        ProcessWheel(_frontRightWheel, 0f, forwardSpeed, _suspension.isFRGrounded, _suspension.frHit, true, false);
        ProcessWheel(_rearLeftWheel, wheelTorque, forwardSpeed, _suspension.isRLGrounded, _suspension.rlHit, false, true);
        ProcessWheel(_rearRightWheel, wheelTorque, forwardSpeed, _suspension.isRRGrounded, _suspension.rrHit, false, true);
    }

    private void ApplyAerodynamics(float speedMs)
    {
        float speedSq = speedMs * speedMs;
        if (speedSq < 0.1f)
        {
            _currentDragForce = 0;
            _currentDownforce = 0;
            _currentGroundEffectForce = 0;
            return;
        }

        float dragForce = 0.5f * _kartConfig.airDensity * _kartConfig.dragCoefficient * _kartConfig.frontalArea * speedSq;
        _rigidbody.AddForce(-_rigidbody.linearVelocity.normalized * dragForce, ForceMode.Force);
        _currentDragForce = dragForce;

        float liftForce = 0.5f * _kartConfig.airDensity * _kartConfig.wingLiftCoefficient * _kartConfig.wingArea * speedSq;
        Vector3 downforcePos = _rearWingPosition != null ? _rearWingPosition.position : (_rearLeftWheel.position + _rearRightWheel.position) * 0.5f;
        _rigidbody.AddForceAtPosition(-transform.up * liftForce, downforcePos, ForceMode.Force);
        _currentDownforce = liftForce;

        if (_suspension.isFLGrounded || _suspension.isFRGrounded || _suspension.isRLGrounded || _suspension.isRRGrounded)
        {
            float avgDist = 0f;
            int count = 0;
            if (_suspension.isFLGrounded) { avgDist += _suspension.flHit.distance; count++; }
            if (_suspension.isFRGrounded) { avgDist += _suspension.frHit.distance; count++; }
            if (_suspension.isRLGrounded) { avgDist += _suspension.rlHit.distance; count++; }
            if (_suspension.isRRGrounded) { avgDist += _suspension.rrHit.distance; count++; }

            if (count > 0) avgDist /= count;

            if (avgDist < _kartConfig.maxGroundEffectDist && avgDist > 0.05f)
            {
                float geForce = _kartConfig.groundEffectFactor / avgDist;
                _rigidbody.AddForce(-transform.up * geForce, ForceMode.Force);
                _currentGroundEffectForce = geForce;
            }
            else
            {
                _currentGroundEffectForce = 0f;
            }
        }
    }

    private void ProcessWheel(Transform wheelTr, float driveTorque, float forwardSpeed, bool isGrounded, RaycastHit hit, bool isSteer, bool isDrive)
    {
        if (!isGrounded) return;

        float normalForce = _kartConfig.mass * 9.81f / 4f;
        ApplyTireForces(wheelTr, hit, driveTorque, normalForce, forwardSpeed, isSteer, isDrive);
    }

    private void ApplyTireForces(Transform wheelTr, RaycastHit hit, float driveTorque, float normalForce, float forwardSpeed, bool isSteer, bool isDrive)
    {
        Vector3 pointVel = _rigidbody.GetPointVelocity(wheelTr.position);
        Vector3 velProjected = Vector3.ProjectOnPlane(pointVel, hit.normal);
        Vector3 wheelForward = wheelTr.forward;
        Vector3 wheelRight = wheelTr.right;

        float vLong = Vector3.Dot(velProjected, wheelForward);
        float vLat = Vector3.Dot(velProjected, wheelRight);
        float absSpeed = velProjected.magnitude;

        if (absSpeed < 0.1f && Mathf.Abs(_throttleInput) < 0.05f)
        {
            Vector3 stopForce = -velProjected * 500f * _rigidbody.mass * Time.fixedDeltaTime;
            _rigidbody.AddForceAtPosition(stopForce, hit.point, ForceMode.Force);
            return;
        }

        float speedFactor = Mathf.Clamp01(absSpeed / 1.0f);
        float currentLatStiffness = _kartConfig.lateralStiffness * speedFactor;

        if (_handbrakePressed && !isSteer) currentLatStiffness *= 0.1f;

        float fy = -vLat * currentLatStiffness;
        float fx = 0f;

        fx -= Mathf.Sign(vLong) * _kartConfig.rollingResistance;

        if (isDrive)
        {
            float safeRadius = Mathf.Max(_kartConfig.wheelRadius, 0.01f);
            float availableForce = driveTorque / safeRadius;

            bool isGoingForward = forwardSpeed > 0.5f;
            bool isGoingBackward = forwardSpeed < -0.5f;

            if (isGoingForward)
            {
                if (_throttleInput > 0) fx += availableForce;
                else if (_throttleInput < 0) fx -= 2500f;
            }
            else if (isGoingBackward)
            {
                if (_throttleInput < 0) fx -= availableForce;
                else if (_throttleInput > 0) fx += 2500f;
            }
            else
            {
                if (_throttleInput > 0) fx += availableForce;
                else if (_throttleInput < 0) fx -= availableForce;
            }
        }

        if (_handbrakePressed && !isSteer)
        {
            fx += (vLong > 0 ? -1f : 1f) * 2000f;
        }

        float maxFriction = normalForce * _kartConfig.frictionCoefficient;
        Vector2 combinedForce = new Vector2(fx, fy);

        if (combinedForce.magnitude > maxFriction)
        {
            combinedForce = combinedForce.normalized * maxFriction;
            fx = combinedForce.x;
            fy = combinedForce.y;
        }

        Vector3 finalForce = (wheelForward * fx) + (wheelRight * fy);
        _rigidbody.AddForceAtPosition(finalForce, hit.point, ForceMode.Force);
    }

    private void OnGUI()
    {
        GUIStyle style = new GUIStyle(GUI.skin.box);
        style.alignment = TextAnchor.UpperLeft;
        style.fontSize = 12;
        style.fontStyle = FontStyle.Bold;
        style.normal.textColor = Color.white;

        GUILayout.BeginArea(new Rect(10, 10, 300, 550), "TELEMETRY", style);
        GUILayout.Space(25);

        float speedMs = _rigidbody.linearVelocity.magnitude;
        GUILayout.Label($"Speed: {speedMs * 3.6f:F1} km/h  ({speedMs:F1} m/s)");

        GUILayout.Label($"RPM: {_engine.CurrentRpm:F0} / {_kartConfig.maxRpm:F0}");

        GUILayout.Label($"Drag: {_currentDragForce:F1} N");

        GUILayout.Label($"Downforce: {_currentDownforce:F1} N");

        GUILayout.Label($"Ground Effect: {_currentGroundEffectForce:F1} N");

        GUILayout.Space(10); 

        GUILayout.Label("--- Compression (m) ---");
        GUILayout.Label($"FL Compression: {_suspension.lastFLcompression:F3}");
        GUILayout.Label($"FR Compression: {_suspension.lastFRcompression:F3}");
        GUILayout.Label($"RL Compression: {_suspension.lastRLcompression:F3}");
        GUILayout.Label($"RR Compression: {_suspension.lastRRcompression:F3}");

        GUILayout.Space(10); 

        GUILayout.Label("--- Suspension Force (N) ---");
        float flForce = _suspension.lastFLcompression * _kartConfig.springStiffness;
        float frForce = _suspension.lastFRcompression * _kartConfig.springStiffness;
        float rlForce = _suspension.lastRLcompression * _kartConfig.springStiffness;
        float rrForce = _suspension.lastRRcompression * _kartConfig.springStiffness;
        GUILayout.Label($"FL Spring Force: {flForce:F0}");
        GUILayout.Label($"FR Spring Force: {frForce:F0}");
        GUILayout.Label($"RL Spring Force: {rlForce:F0}");
        GUILayout.Label($"RR Spring Force: {rrForce:F0}");

        GUILayout.Space(10); 

        GUILayout.Label("--- Distance to Ground (m) ---");
        GUILayout.Label($"FL Distance: {(_suspension.isFLGrounded ? _suspension.flHit.distance.ToString("F3") : "In Air")}");
        GUILayout.Label($"FR Distance: {(_suspension.isFRGrounded ? _suspension.frHit.distance.ToString("F3") : "In Air")}");
        GUILayout.Label($"RL Distance: {(_suspension.isRLGrounded ? _suspension.rlHit.distance.ToString("F3") : "In Air")}");
        GUILayout.Label($"RR Distance: {(_suspension.isRRGrounded ? _suspension.rrHit.distance.ToString("F3") : "In Air")}");

        GUILayout.Space(10); 

        GUILayout.Label("--- Center of Mass ---");
        GUILayout.Label($"CoM Height (Y): {_rigidbody.worldCenterOfMass.y:F2} m");

        GUILayout.EndArea();
    }
}
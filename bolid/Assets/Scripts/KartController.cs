using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class KartController : MonoBehaviour
{
    [Header("Configuration")]
    [SerializeField] private bool _import = false;
    [SerializeField] private KartConfig _kartConfig;

    [Header("Wheel References")]
    [SerializeField] private Transform _frontLeftWheel;
    [SerializeField] private Transform _frontRightWheel;
    [SerializeField] private Transform _rearLeftWheel;
    [SerializeField] private Transform _rearRightWheel;

    [Header("Components")]
    [SerializeField] private KartEngine _engine;
    [SerializeField] private InputActionAsset _playerInput;
    [SerializeField] private Transform _centerOfMass;

    [SerializeField] private Transform _rearWingPosition;

    private Rigidbody _rigidbody;
    private InputAction _moveAction;

    private float _throttleInput;
    private float _steerInput;
    private bool _handbrakePressed;

    private WheelData[] _wheels;
    private float _currentDragForce;
    private float _currentDownforce;      
    private float _currentGroundEffectForce; 
    private float _groundDistanceCenter;

    private float _suspensionRestLength = 0.5f;
    private float _springStiffness = 30000f;
    private float _damperStiffness = 4000f;
    private float _wheelRadius = 0.34f;

    private float _dragCoeff = 0.3f;
    private float _liftCoeff = 1.2f;       
    private float _airDensity = 1.225f;
    private float _frontalArea = 2.2f;
    private float _wingArea = 1.5f;        
    private float _groundEffectFactor = 50f; 
    private float _maxGroundEffectDist = 0.2f;

    private float _gearRatio = 8f;
    private float _drivetrainEfficiency = 0.9f;

    private float _maxSteeringAngle = 35f;
    private Quaternion _flInitRot, _frInitRot;

    private class WheelData
    {
        public Transform transform;
        public bool isSteer;
        public bool isDrive;

        public float previousCompression;
        public float currentCompression;
        public float currentSuspensionForce;
        public float distanceToGround;
        public float currentFx;
        public float currentFy;
        public bool isGrounded;

        public WheelData(Transform t, bool steer, bool drive)
        {
            transform = t;
            isSteer = steer;
            isDrive = drive;
        }
    }

    private void Awake()
    {
        _playerInput.Enable();
        _rigidbody = GetComponent<Rigidbody>();
        if (_engine == null) _engine = GetComponent<KartEngine>();

        var map = _playerInput.FindActionMap("Kart");
        _moveAction = map.FindAction("Move");

        if (_centerOfMass != null)
            _rigidbody.centerOfMass = _centerOfMass.localPosition;

        _flInitRot = _frontLeftWheel.localRotation;
        _frInitRot = _frontRightWheel.localRotation;

        InitializeWheels();

        if (_import) LoadConfig();
    }

    private void InitializeWheels()
    {
        _wheels = new WheelData[4];
        _wheels[0] = new WheelData(_frontLeftWheel, true, false);
        _wheels[1] = new WheelData(_frontRightWheel, true, false);
        _wheels[2] = new WheelData(_rearLeftWheel, false, true);
        _wheels[3] = new WheelData(_rearRightWheel, false, true);
    }

    private void LoadConfig()
    {
        if (_kartConfig == null) return;

        _rigidbody.mass = _kartConfig.mass;
        _wheelRadius = _kartConfig.wheelRadius;
        _suspensionRestLength = _kartConfig.suspensionRestLength;
        _springStiffness = _kartConfig.springStiffness;
        _damperStiffness = _kartConfig.damperStiffness;

        _dragCoeff = _kartConfig.dragCoefficient;
        _liftCoeff = _kartConfig.wingLiftCoefficient;
        _airDensity = _kartConfig.airDensity;
        _frontalArea = _kartConfig.frontalArea;
        _wingArea = _kartConfig.wingArea;
        _groundEffectFactor = _kartConfig.groundEffectFactor;
        _maxGroundEffectDist = _kartConfig.maxGroundEffectDist;

        _gearRatio = _kartConfig.gearRatio;
        _maxSteeringAngle = _kartConfig.maxSteerAngle;
    }

    private void OnDisable()
    {
        _playerInput.Disable();
    }

    private void Update()
    {
        ReadInput();
        UpdateVisuals();
    }

    private void ReadInput()
    {
        Vector2 move = _moveAction.ReadValue<Vector2>();
        _steerInput = move.x;
        _throttleInput = move.y;
        _handbrakePressed = Input.GetKey(KeyCode.Space);
    }

    private void UpdateVisuals()
    {
        float steerAngle = _maxSteeringAngle * _steerInput;
        Quaternion steerRot = Quaternion.Euler(0, steerAngle, 0);
        _frontLeftWheel.localRotation = _flInitRot * steerRot;
        _frontRightWheel.localRotation = _frInitRot * steerRot;
    }

    private void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        float forwardSpeed = Vector3.Dot(_rigidbody.linearVelocity, transform.forward);
        float absSpeed = Mathf.Abs(forwardSpeed);

        ApplyAerodynamics();

        float engineInput = Mathf.Abs(_throttleInput);

        if (forwardSpeed < -1f && _throttleInput > 0) engineInput = _throttleInput;

        float engineTorque = _engine.Simulate(engineInput, absSpeed, dt);

        float wheelTorque = (engineTorque * _gearRatio * _drivetrainEfficiency) / 2f;

        for (int i = 0; i < _wheels.Length; i++)
        {
            ProcessWheel(_wheels[i], wheelTorque, forwardSpeed, dt);
        }
    }

    private void ApplyAerodynamics()
    {
        float speedSq = _rigidbody.linearVelocity.sqrMagnitude;
        if (speedSq < 0.1f)
        {
            _currentDragForce = 0;
            _currentDownforce = 0;
            _currentGroundEffectForce = 0;
            return;
        }

        float dragForce = 0.5f * _airDensity * _dragCoeff * _frontalArea * speedSq;
        _rigidbody.AddForce(-_rigidbody.linearVelocity.normalized * dragForce, ForceMode.Force);
        _currentDragForce = dragForce;

        float liftForce = 0.5f * _airDensity * _liftCoeff * _wingArea * speedSq;

        Vector3 downforcePos = _rearWingPosition != null
            ? _rearWingPosition.position
            : (_rearLeftWheel.position + _rearRightWheel.position) * 0.5f;

        _rigidbody.AddForceAtPosition(-transform.up * liftForce, downforcePos, ForceMode.Force);
        _currentDownforce = liftForce;

        RaycastHit hit;
        if (Physics.Raycast(transform.position, -transform.up, out hit, 2.0f))
        {
            _groundDistanceCenter = hit.distance;
            if (_groundDistanceCenter < _maxGroundEffectDist && _groundDistanceCenter > 0.05f)
            {
                float geForce = _groundEffectFactor / _groundDistanceCenter;
                _rigidbody.AddForce(-transform.up * geForce, ForceMode.Force);
                _currentGroundEffectForce = geForce;
            }
            else
            {
                _currentGroundEffectForce = 0f;
            }
        }
    }

    private void ProcessWheel(WheelData wheel, float driveTorque, float forwardSpeed, float dt)
    {
        Vector3 rayOrigin = wheel.transform.position;
        Vector3 rayDir = -wheel.transform.up;

        RaycastHit hit;
        bool didHit = Physics.Raycast(rayOrigin, rayDir, out hit, _suspensionRestLength + _wheelRadius);

        wheel.isGrounded = didHit;

        if (didHit)
        {
            wheel.distanceToGround = hit.distance;

            float currentSuspensionLen = hit.distance - _wheelRadius;
            currentSuspensionLen = Mathf.Clamp(currentSuspensionLen, 0, _suspensionRestLength);

            wheel.currentCompression = _suspensionRestLength - currentSuspensionLen;

            float springForce = _springStiffness * wheel.currentCompression;
            float compressionVel = (wheel.currentCompression - wheel.previousCompression) / dt;
            float damperForce = _damperStiffness * compressionVel;

            float totalSuspensionForce = Mathf.Max(0, springForce + damperForce);
            wheel.currentSuspensionForce = totalSuspensionForce;
            wheel.previousCompression = wheel.currentCompression;

            Vector3 suspensionForceVector = wheel.transform.up * totalSuspensionForce;
            _rigidbody.AddForceAtPosition(suspensionForceVector, rayOrigin, ForceMode.Force);

            ApplyTireForces(wheel, hit, driveTorque, totalSuspensionForce, forwardSpeed);
        }
        else
        {
            wheel.currentCompression = 0;
            wheel.currentSuspensionForce = 0;
            wheel.previousCompression = 0;
            wheel.distanceToGround = float.MaxValue;
            wheel.currentFx = 0;
            wheel.currentFy = 0;
        }
    }

    private void ApplyTireForces(WheelData wheel, RaycastHit hit, float driveTorque, float normalForce, float forwardSpeed)
    {
        Vector3 pointVel = _rigidbody.GetPointVelocity(wheel.transform.position);
        Vector3 velProjected = Vector3.ProjectOnPlane(pointVel, hit.normal);

        Vector3 wheelForward = wheel.transform.forward;
        Vector3 wheelRight = wheel.transform.right;

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

        if (_handbrakePressed && !wheel.isSteer) currentLatStiffness *= 0.1f;
        float fy = -vLat * currentLatStiffness;

     
        float fx = 0f;
        fx -= Mathf.Sign(vLong) * _kartConfig.rollingResistance; 
        if (wheel.isDrive)
        {
            float safeRadius = Mathf.Max(_wheelRadius, 0.01f);
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

        // Ручник
        if (_handbrakePressed && !wheel.isSteer)
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

        wheel.currentFx = fx;
        wheel.currentFy = fy;
    }

    private void OnGUI()
    {
        DrawTelemetry();
    }

    private void DrawTelemetry()
    {
        GUIStyle style = new GUIStyle(GUI.skin.box);
        style.alignment = TextAnchor.UpperLeft;
        style.fontSize = 12;
        style.fontStyle = FontStyle.Bold;
        style.normal.textColor = Color.white;

        float width = 350;
        float height = 500;
        float padding = 10;

        GUILayout.BeginArea(new Rect(padding, padding, width, height), "ADVANCED TELEMETRY", style);
        GUILayout.Space(25);

        float speedMs = _rigidbody.linearVelocity.magnitude;
        float speedKmh = speedMs * 3.6f;
        DrawLabel($"Speed: {speedMs:F1} m/s | {speedKmh:F1} km/h");
        DrawLabel($"Engine RPM: {_engine.CurrentRpm:F0} / {_kartConfig.maxRpm:F0}");
        DrawLabel($"Drag Force: {_currentDragForce:F1} N");
        DrawLabel($"Downforce: {_currentDownforce:F1} N"); 
        DrawLabel($"Ground Effect: {_currentGroundEffectForce:F1} N"); 

        GUILayout.Space(10);
        GUILayout.Label("--- WHEEL DATA ---");

        for (int i = 0; i < _wheels.Length; i++)
        {
            WheelData w = _wheels[i];
            string name = i == 0 ? "FL" : i == 1 ? "FR" : i == 2 ? "RL" : "RR";

            GUILayout.BeginVertical(GUI.skin.box);
            GUILayout.Label($"{name} | Grounded: {w.isGrounded}");
            GUILayout.Label($"Susp Force: {w.currentSuspensionForce:F0} N");
            GUILayout.Label($"Compression: {w.currentCompression:F3} m");
            GUILayout.EndVertical();
        }

        GUILayout.Space(5);
        GUILayout.Label($"COM Height (Y): {_rigidbody.transform.position.y:F3} m");

        GUILayout.EndArea();
    }

    private void DrawLabel(string text)
    {
        GUILayout.Label(text);
    }

    private void OnDrawGizmos()
    {
        if (_wheels == null) return;

        foreach (var w in _wheels)
        {
            Gizmos.color = Color.yellow;
            Vector3 start = w.transform.position;
            Vector3 end = start - w.transform.up * (_suspensionRestLength + _wheelRadius);
            Gizmos.DrawLine(start, end);

            if (w.isGrounded)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawSphere(start - w.transform.up * w.distanceToGround, 0.05f);
            }
        }
    }
}
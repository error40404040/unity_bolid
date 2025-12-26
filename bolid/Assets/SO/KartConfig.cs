using UnityEngine;

[CreateAssetMenu(fileName = "KartConfig", menuName = "Scriptable Objects/KartConfig")]
public class KartConfig : ScriptableObject
{
    [Header("Base Settings")]
    public float mass = 1200f;

    [Header("Tires")]
    public float frictionCoefficient = 1.0f;
    public float lateralStiffness = 15f;
    public float rollingResistance = 5f;
    public float wheelRadius = 0.34f;

    [Header("Suspension")]
    public float suspensionRestLength = 0.5f;
    public float springStiffness = 30000f;
    public float damperStiffness = 4000f;

    [Header("Aerodynamics")]
    public float airDensity = 1.225f;
    public float frontalArea = 2.2f;
    public float dragCoefficient = 0.3f;       

    [Header("Wings & Downforce (Mandatory)")]
    public float wingArea = 1.5f;              
    public float wingLiftCoefficient = 1.2f;   

    [Header("Ground Effect (Mandatory)")]
    public float groundEffectFactor = 50f;    
    public float maxGroundEffectDist = 0.2f;   

    [Header("Engine & Transmission")]
    public AnimationCurve engineTorqueCurve;
    public float engineInertia = 0.2f;
    public float maxRpm = 8000f;
    public float gearRatio = 8f;
    public float maxSteerAngle = 35f;
}
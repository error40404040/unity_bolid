using UnityEngine;

[CreateAssetMenu(fileName = "KartConfig", menuName = "Scriptable Objects/KartConfig")]
public class KartConfig : ScriptableObject
{
    [Header("Base Settings")]
    public float mass = 1200f;

    [Header("Tires")]
    public float frictionCoefficient = 1.2f;
    public float lateralStiffness = 4000f;
    public float rollingResistance = 10f;
    public float wheelRadius = 0.34f; 

    [Header("Suspension")]
    public float suspensionRestLength = 0.5f;
    public float suspensionTravel = 0.2f;      
    public float springStiffness = 30000f;
    public float damperStiffness = 4000f;

    [Header("Anti-Roll Bar")]
    public float frontAntiRoll = 8000f;
    public float rearAntiRoll = 6000f;  

    [Header("Aerodynamics")]
    public float airDensity = 1.225f;
    public float frontalArea = 2.2f;
    public float dragCoefficient = 0.35f;
    public float wingArea = 1.8f;
    public float wingLiftCoefficient = 1.5f;
    public float groundEffectFactor = 80f;
    public float maxGroundEffectDist = 0.25f;

    [Header("Engine")]
    public AnimationCurve engineTorqueCurve;
    public float maxRpm = 8000f;
    public float gearRatio = 8f;
    public float maxSteerAngle = 40f;
}
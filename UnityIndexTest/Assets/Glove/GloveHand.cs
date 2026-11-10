using System;
using UnityEngine;

public class GloveHand : MonoBehaviour
{
    [Serializable]
    public class Finger
    {
        public string name;
        public Transform[] joints = new Transform[3];
        public Vector3 localBendAxis = Vector3.right;
        public float[] maxAngles = {65,85,55};
        [NonSerialized] public Quaternion[] rest;
    }
    public GloveReceiver receiver;
    [Tooltip("Order: thumb, index, middle, ring, little")]
    public Finger[] fingers = new Finger[5];

    public IndexContactSimulation contactSimulation;
    public readonly float[] VisualCurl = new float[5];

    void Start()
    {
        foreach (Finger f in fingers) {
            if (f == null || f.joints == null) continue;
            f.rest = new Quaternion[f.joints.Length];
            for (int j=0;j<f.joints.Length;j++)
                if (f.joints[j] != null) f.rest[j]=f.joints[j].localRotation;
        }
    }
    void LateUpdate()
    {
        if (receiver == null) return;
        for (int i=0;i<Mathf.Min(5,fingers.Length);i++) {
            Finger f=fingers[i];
            if (f == null || f.rest == null || f.maxAngles == null) continue;
            float measured = receiver.Curl[i];
            if (i == 1 && contactSimulation != null)
                measured = contactSimulation.Resolve(measured);
            VisualCurl[i] = measured;
            for (int j=0;j<f.joints.Length && j<f.maxAngles.Length;j++) {
                if (f.joints[j] == null) continue;
                f.joints[j].localRotation = f.rest[j] * Quaternion.AngleAxis(
                    VisualCurl[i]*f.maxAngles[j], f.localBendAxis.normalized);
            }
        }
    }
}
